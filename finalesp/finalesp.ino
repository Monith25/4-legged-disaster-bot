#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include <WiFi.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
void startCameraServer();
void setupLedFlash();

const char* ssid = "wifi ssid";
const char* password = "password";

const char* PC_IP = "10.212.XXX.XXX";   // your PC IP
const uint16_t PC_PORT = 5005;

WiFiUDP Udp;
IPAddress pcIp;

String buffer = "";
const size_t MAX_BUF = 2048;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(300);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  delay(5000);

  Serial.println("\n=== ESP32 JSON RECEIVER STARTED ===");

  WiFi.begin(ssid, password);
  Serial.print("WiFi: ");
  int t=0;
  while (WiFi.status() != WL_CONNECTED && t < 40) {
    Serial.print(".");
    delay(200);
    t++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected! IP = ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi FAILED (continuing anyway)");
  }

  pcIp.fromString(PC_IP);
  Udp.begin(PC_PORT);

  Serial.println("Waiting for JSON packets from Nano...");
}

String extractJson(String &buf) {
  int s = buf.indexOf('{');
  if (s < 0) return "";

  int depth = 0;
  for (int i=s; i<buf.length(); i++) {
    char c = buf[i];
    if (c == '{') depth++;
    else if (c == '}') {
      depth--;
      if (depth == 0) {
        String json = buf.substring(s, i+1);
        buf = buf.substring(i+1);
        return json;
      }
    }
  }

  if (buf.length() > MAX_BUF)
    buf = buf.substring(buf.length()/2);

  return "";
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    buffer += c;
  }

  String json;
  while ((json = extractJson(buffer)).length() > 0) {

    Serial.print("JSON received: ");
    Serial.println(json);

    // forward UDP
    Udp.beginPacket(pcIp, PC_PORT);
    Serial.println(">>> TRYING TO SEND UDP <<<");

    Udp.write((const uint8_t*)json.c_str(), json.length());
    int result = Udp.endPacket();
    Serial.print("endPacket() = ");
    Serial.println(result);

  }

  delay(5);
}
