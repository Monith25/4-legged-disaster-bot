#!/usr/bin/env python3
import socket, json, math, rospy, sys, time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf

# -------- CONFIG ----------
UDP_PORT = 5005
BIND_IP = "0.0.0.0"
N_CAL = 200
GRAV = 9.80665

# Laser scan parameters (tune to your scanner)
angle_min_deg = 45.0         # start angle of sweep (deg)
angle_max_deg = 135.0        # end angle of sweep (deg)
angle_res_deg = 1.0          # resolution for output scan (deg)
scan_frame = "laser_link"    # frame for published LaserScan
min_range_m = 0.02
max_range_m = 4.0
# ---------------------------

def extract_first_json(s):
    start = s.find('{')
    if start == -1:
        return None
    depth = 0
    for i in range(start, len(s)):
        c = s[i]
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return s[start:i+1]
    return None

def detect_and_scale_acc(a):
    if abs(a) > 100:
        return (a / 16384.0) * GRAV
    return float(a) * GRAV

def detect_and_scale_gyro(g):
    if abs(g) > 100:
        deg_s = g / 131.0
        return math.radians(deg_s)
    return math.radians(float(g))

def deg_to_index(angle_deg, amin, ares):
    return int(round((angle_deg - amin) / ares))

def clamp(v, a, b):
    return a if v < a else (b if v > b else v)

def main():
    rospy.init_node('udp_to_ros_telemetry', anonymous=True)
    imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=50)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=20)
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=5)

    # prepare UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((BIND_IP, UDP_PORT))
    sock.settimeout(1.0)
    rospy.loginfo("UDP->ROS listening on %s:%d", BIND_IP, UDP_PORT)

    # Calibration
    rospy.loginfo("Calibrating gyro bias (%d samples). Keep still.", N_CAL)
    gyro_bias = [0.0,0.0,0.0]; samples=0
    while samples < N_CAL and not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(8192)
            s = data.decode('utf-8', errors='ignore')
            jtxt = extract_first_json(s)
            if not jtxt: continue
            j0 = json.loads(jtxt)
            gx = float(j0.get('gx',0)); gy = float(j0.get('gy',0)); gz = float(j0.get('gz',0))
            if abs(gx)>100 or abs(gy)>100 or abs(gz)>100:
                gx = gx/131.0; gy = gy/131.0; gz = gz/131.0
            gyro_bias[0]+=gx; gyro_bias[1]+=gy; gyro_bias[2]+=gz
            samples+=1
        except socket.timeout:
            continue
        except Exception as e:
            rospy.logwarn("Calib exception %s", e)
    if samples==0:
        rospy.logwarn("No calibration samples; bias=0")
        gyro_bias = [0.0,0.0,0.0]
    else:
        gyro_bias = [x/samples for x in gyro_bias]
    rospy.loginfo("Gyro bias (deg/s): %s", gyro_bias)
    gyro_bias_rad = [math.radians(x) for x in gyro_bias]

    # prepare laser scan buffer
    amin = angle_min_deg
    amax = angle_max_deg
    ares = angle_res_deg
    n_bins = int(round((amax - amin) / ares)) + 1
    current_scan = [float('inf')] * n_bins
    last_angle_deg = None
    sweep_dir = 0
    last_publish_time = time.time()

    seq = 0
    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(8192)
            text = data.decode('utf-8', errors='ignore')
            jtxt = extract_first_json(text)
            if not jtxt:
                rospy.logwarn_throttle(10, "No JSON found")
                continue
            try:
                j = json.loads(jtxt)
            except Exception as e:
                rospy.logwarn("JSON parse failed: %s", e)
                continue

            ts = rospy.Time.now()
            # --- IMU ---
            imu = Imu()
            imu.header.stamp = ts
            imu.header.frame_id = "imu_link"
            imu.header.seq = seq

            # orientation (if present)
            if 'roll' in j and 'pitch' in j and 'yaw' in j:
                r = float(j['roll']); p = float(j['pitch']); y = float(j['yaw'])
                q = tf.transformations.quaternion_from_euler(math.radians(r), math.radians(p), math.radians(y))
                imu.orientation.x = q[0]; imu.orientation.y = q[1]; imu.orientation.z = q[2]; imu.orientation.w = q[3]
                imu.orientation_covariance = [0.01,0,0, 0,0.01,0, 0,0,0.01]
            else:
                imu.orientation_covariance = [-1.0] + [0.0]*8

            # gyro
            if 'gx' in j:
                gx_raw = float(j.get('gx',0))
                gy_raw = float(j.get('gy',0))
                gz_raw = float(j.get('gz',0))
                gx = detect_and_scale_gyro(gx_raw) - gyro_bias_rad[0]
                gy = detect_and_scale_gyro(gy_raw) - gyro_bias_rad[1]
                gz = detect_and_scale_gyro(gz_raw) - gyro_bias_rad[2]
                imu.angular_velocity.x = gx; imu.angular_velocity.y = gy; imu.angular_velocity.z = gz
                imu.angular_velocity_covariance = [0.02]*9

            # accel
            if 'ax' in j:
                ax = detect_and_scale_acc(float(j.get('ax',0)))
                ay = detect_and_scale_acc(float(j.get('ay',0)))
                az = detect_and_scale_acc(float(j.get('az',0)))
                imu.linear_acceleration.x = ax; imu.linear_acceleration.y = ay; imu.linear_acceleration.z = az
                imu.linear_acceleration_covariance = [0.1]*9

            imu_pub.publish(imu)

            # odom (minimal)
            odom = Odometry()
            odom.header.stamp = ts
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            if 'roll' in j and 'pitch' in j and 'yaw' in j:
                r = float(j['roll']); p = float(j['pitch']); y = float(j['yaw'])
                q = tf.transformations.quaternion_from_euler(math.radians(r), math.radians(p), math.radians(y))
                odom.pose.pose.orientation.x = q[0]; odom.pose.pose.orientation.y = q[1]; odom.pose.pose.orientation.z = q[2]; odom.pose.pose.orientation.w = q[3]
                odom.pose.covariance = [0.1]*36
            odom_pub.publish(odom)

            # --- LASERSCAN accumulation ---
            # **USE ONLY** keys "distance" and "angle"
            angle_deg = None
            if 'angle' in j:
                angle_deg = float(j['angle'])

            dist_val = None
            if 'distance' in j:
                dist_val = float(j['distance'])

            if angle_deg is not None and dist_val is not None:
                # convert units: if value > 10 treat as mm
                if dist_val > 10:
                    dist_m = dist_val / 1000.0
                else:
                    dist_m = dist_val
                # clamp range
                if dist_m < min_range_m or dist_m > max_range_m:
                    r_m = float('inf')
                else:
                    r_m = dist_m

                # check angle in our sweep window
                if angle_deg >= amin - (ares/2.0) and angle_deg <= amax + (ares/2.0):
                    idx = deg_to_index(angle_deg, amin, ares)
                    idx = clamp(idx, 0, n_bins-1)
                    current_scan[idx] = r_m

                # detect sweep direction / completion:
                if last_angle_deg is None:
                    last_angle_deg = angle_deg
                else:
                    dir_now = 1 if (angle_deg - last_angle_deg) > 0 else (-1 if (angle_deg - last_angle_deg) < 0 else sweep_dir)
                    if sweep_dir != 0 and dir_now != 0 and dir_now != sweep_dir:
                        # publish the accumulated scan
                        scan = LaserScan()
                        scan.header.stamp = ts
                        scan.header.frame_id = scan_frame
                        scan.angle_min = math.radians(amin)
                        scan.angle_max = math.radians(amax)
                        scan.angle_increment = math.radians(ares)
                        scan.time_increment = 0.0
                        scan.scan_time = 0.0
                        scan.range_min = min_range_m
                        scan.range_max = max_range_m
                        scan.ranges = list(current_scan)
                        scan_pub.publish(scan)
                        rospy.logdebug("Published LaserScan with %d bins", len(current_scan))
                        current_scan = [float('inf')] * n_bins
                        last_publish_time = time.time()
                    sweep_dir = dir_now
                    last_angle_deg = angle_deg

            seq += 1

        except socket.timeout:
            continue
        except Exception as e:
            rospy.logerr("Exception: %s", e)

if __name__ == "__main__":
    main()
