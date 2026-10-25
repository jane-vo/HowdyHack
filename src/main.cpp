#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "img_converters.h"

// ---- WiFi ----
const char* ssid     = "TAMU_IoT";   // or your phone hotspot name
const char* password = "";           // leave empty for TAMU_IoT

WebServer server(80);

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{font-family:Helvetica;text-align:center}img{width:100%;max-width:640px}</style>
</head><body>
<h2>ESP32-S3 Camera</h2>
<img id="cam" src="/jpg">
<script>
const img = document.getElementById('cam');
img.onload  = () => setTimeout(() => { img.src = '/jpg?t=' + Date.now(); }, 50);
img.onerror = () => setTimeout(() => { img.src = '/jpg?t=' + Date.now(); }, 1000);
</script>
</body></html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

void handleJpg() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "capture failed");
    return;
  }
  uint8_t *jpg = nullptr;
  size_t len = 0;
  bool ok = frame2jpg(fb, 80, &jpg, &len);
  esp_camera_fb_return(fb);
  if (!ok) {
    server.send(500, "text/plain", "jpeg conversion failed");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/jpeg", (const char*)jpg, len);
  free(jpg);
}

bool startCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = 8;
  config.pin_d1 = 9;
  config.pin_d2 = 10;
  config.pin_d3 = 11;
  config.pin_d4 = 12;
  config.pin_d5 = 13;
  config.pin_d6 = 14;
  config.pin_d7 = 15;
  config.pin_xclk  = 16;
  config.pin_pclk  = 17;
  config.pin_vsync = 6;
  config.pin_href  = 7;
  config.pin_sccb_sda = 4;
  config.pin_sccb_scl = 5;
  config.pin_pwdn  = -1;
  config.pin_reset = -1;
  config.xclk_freq_hz = 3000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.fb_count     = 1;
  config.frame_size  = FRAMESIZE_QQVGA;      // 160x120
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode   = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  Serial.printf("Camera init: 0x%x (%s)\n", err, esp_err_to_name(err));
  return err == ESP_OK;
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  if (!startCamera()) {
    Serial.println("Camera failed, stopping.");
    return;
  }

  Serial.print("Connecting to ");
  Serial.println(ssid);
  if (strlen(password) == 0) WiFi.begin(ssid);
  else WiFi.begin(ssid, password);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 60) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed. Check name, band (2.4 GHz), and registration.");
    return;
  }
  Serial.println("\nWiFi connected.");
  Serial.print("Open this in a browser: http://");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/jpg", handleJpg);
  server.begin();
}

void loop() {
  server.handleClient();
}
