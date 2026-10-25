#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "driver/ledc.h"

// ================= USER SETTINGS =================
const char* AP_SSID = "ESP32-Camera";
const char* AP_PASS = "12345678";          // at least 8 characters

// OV7670 -> ESP32-S3 pins
#define PIN_D0     8
#define PIN_D1     9
#define PIN_D2     10
#define PIN_D3     11
#define PIN_D4     12
#define PIN_D5     13
#define PIN_D6     14
#define PIN_D7     15
#define PIN_XCLK   20
#define PIN_PCLK   47
#define PIN_VSYNC  6
#define PIN_HREF   7
#define PIN_SDA    4
#define PIN_SCL    5
#define PIN_PWDN   -1
#define PIN_RESET  -1

// Servo and HC-SR04
#define PIN_SERVO  18
#define PIN_TRIG   1
#define PIN_ECHO   2      // through a 1k / 2k voltage divider

#define XCLK_HZ    8000000   // if the picture shows noisy bands, try 8000000

// Servo pulse range (500-2400 us suits most micro servos)
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400

// ---- Target range ----
#define MAX_CM         50.0f   // lock onto objects closer than this
#define MIN_CM         3.0f    // the HC-SR04 is unreliable closer than this

// ---- Scanning ----
#define SCAN_STEP      6       // degrees between samples
#define SETTLE_MS      120     // wait after each servo move before measuring
#define LOCK_SETTLE_MS 350     // wait after aiming before learning the target's appearance

// ---- Tracking ----
#define TW             28      // size of the target patch (pixels, on the 160x120 image)
#define TH             28
#define SR_X           44      // how far (pixels) the target may move between frames
#define SR_Y           16
#define MATCH_MAX      30      // max average brightness difference for a good match (lower = stricter)
#define MISS_MAX       10      // lose the lock after this many bad frames in a row
#define DIST_CHECK_MS  400     // how often the ultrasonic sensor re-checks the target
#define DIST_MISS_MAX  4       // lose the lock after this many failed distance checks in a row

// ---- Servo control (change live with the Serial Monitor) ----
#define INIT_FOV_DEG   60.0f   // camera's horizontal field of view
#define INIT_DIR       1       // 1 or -1: set so the camera turns TOWARD the target
#define KP             0.5f    // how hard to correct per frame (lower = smoother)
#define MAX_STEP_DEG   8.0f    // max servo movement per frame
#define DEADBAND_DEG   1.0f    // ignore smaller errors (stops twitching)
// =================================================

#define OUT_W 160
#define OUT_H 120
#define NPIX  (OUT_W * OUT_H)

// Overlay colors in the camera's byte order (correct with "swap bytes" unticked)
#define COL_GREEN  0xE007
#define COL_RED    0x00F8
#define COL_YELLOW 0xE0FF

enum State { ST_SCAN, ST_LOCK, ST_TRACK };

WebServer server(80);

static uint16_t *workBuf  = nullptr;   // 160x120 color frame the vision task draws on
static uint16_t *shareBuf = nullptr;   // latest finished frame
static uint16_t *sendBuf  = nullptr;   // copy used by the stream
static uint8_t  *gcur     = nullptr;   // 160x120 brightness of the current frame
static uint8_t   tmpl[TW * TH];        // brightness patch of the locked target
static SemaphoreHandle_t frameMutex;
static volatile uint32_t frameId = 0;

static volatile State st = ST_SCAN;
static volatile float fovDeg   = INIT_FOV_DEG;
static volatile int   servoDir = INIT_DIR;
static volatile bool  recenter = false, rescanReq = false;

static float servoAngle = 90;
static int   tx = OUT_W / 2, ty = OUT_H / 2;      // target position in the image
static volatile float lastDist = -1, lastScore = -1;

// scan state
static int   scanLo = 0, scanHi = 180, scanN = 0, scanK = 0;
static bool  scanUp = false, scanMoved = false;
static uint32_t scanMoveT = 0, lockT = 0;
static float scanD[40];


static bool camOk = false;
static char camMsg[48] = "not started";

// ---------------- Servo (LEDC timer 1 / channel 1; the camera uses timer 0 / channel 0) ----------------
void servoInit() {
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = LEDC_TIMER_14_BIT;
  t.timer_num       = LEDC_TIMER_1;
  t.freq_hz         = 50;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  ledc_channel_config_t c = {};
  c.gpio_num   = PIN_SERVO;
  c.speed_mode = LEDC_LOW_SPEED_MODE;
  c.channel    = LEDC_CHANNEL_1;
  c.timer_sel  = LEDC_TIMER_1;
  c.duty       = 0;
  c.hpoint     = 0;
  ledc_channel_config(&c);
}

void servoWrite(int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t us   = SERVO_MIN_US + (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
  uint32_t duty = us * 16384UL / 20000UL;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// ---------------- HC-SR04 ----------------
// Returns the distance in cm, or -1 if nothing is within MIN_CM..MAX_CM.
// Echo timing is done by polling so interrupts stay enabled (the camera needs them).
static uint32_t lastTrig = 0;
float readDistanceCm() {
  while (millis() - lastTrig < 60) delay(1);       // the sensor needs ~60 ms between measurements
  lastTrig = millis();

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  uint32_t t0 = micros();
  while (digitalRead(PIN_ECHO) == LOW)  if (micros() - t0 > 2000) return -1;      // echo never started
  uint32_t start = micros();
  while (digitalRead(PIN_ECHO) == HIGH) if (micros() - start > 3600) return -1;   // farther than ~60 cm
  float cm = (micros() - start) * 0.0343f / 2.0f;
  return (cm >= MIN_CM && cm <= MAX_CM) ? cm : -1;
}

// ---------------- Image helpers ----------------
// Brightness of one pixel (the camera's bytes arrive swapped when read as uint16_t)
static inline uint8_t grayOf(uint16_t p) {
  uint16_t v = (uint16_t)((p << 8) | (p >> 8));
  uint8_t r = ((v >> 11) & 31) << 3;
  uint8_t g = ((v >> 5)  & 63) << 2;
  uint8_t b = (v & 31) << 3;
  return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

// One pass over the 320x240 frame: workBuf = 160x120 color (for the stream),
// gcur = 160x120 brightness, each pixel averaged from a 2x2 block to cut noise
static void processFrame(const camera_fb_t *fb) {
  const uint16_t *src = (const uint16_t *)fb->buf;
  const int W = fb->width;
  for (int y = 0; y < OUT_H; y++) {
    const uint16_t *r0 = src + (2 * y) * W;
    const uint16_t *r1 = r0 + W;
    for (int x = 0; x < OUT_W; x++) {
      int i = 2 * x;
      workBuf[y * OUT_W + x] = r0[i];
      gcur[y * OUT_W + x] = (uint8_t)((grayOf(r0[i]) + grayOf(r0[i + 1]) +
                                       grayOf(r1[i]) + grayOf(r1[i + 1])) >> 2);
    }
  }
}

static inline void putPx(int x, int y, uint16_t col) {
  if (x >= 0 && x < OUT_W && y >= 0 && y < OUT_H) workBuf[y * OUT_W + x] = col;
}

static void drawBox(int x0, int y0, int x1, int y1, uint16_t col) {
  for (int x = x0; x <= x1; x++) { putPx(x, y0, col); putPx(x, y1, col); }
  for (int y = y0; y <= y1; y++) { putPx(x0, y, col); putPx(x1, y, col); }
}

// Sum of absolute brightness differences between the target patch and the image
// patch centered at (cx, cy). Stops early once the sum exceeds `limit`.
static uint32_t sadAt(int cx, int cy, int step, uint32_t limit) {
  int x0 = cx - TW / 2, y0 = cy - TH / 2;
  uint32_t sum = 0;
  for (int j = 0; j < TH; j += step) {
    const uint8_t *g = gcur + (y0 + j) * OUT_W + x0;
    const uint8_t *t = tmpl + j * TW;
    for (int i = 0; i < TW; i += step) {
      int d = (int)g[i] - (int)t[i];
      sum += (d < 0) ? -d : d;
    }
    if (sum > limit) return sum;
  }
  return sum;
}

// Find the target patch near (cx, cy). Returns the average brightness difference per pixel
// (lower = better match) and the best position in bx, by.
static int matchTemplate(int cx, int cy, int &bx, int &by) {
  const int hw = TW / 2, hh = TH / 2;
  int x0 = max(hw, cx - SR_X), x1 = min(OUT_W - hw, cx + SR_X);
  int y0 = max(hh, cy - SR_Y), y1 = min(OUT_H - hh, cy + SR_Y);

  uint32_t best = 0xFFFFFFFF;
  int px = constrain(cx, hw, OUT_W - hw), py = constrain(cy, hh, OUT_H - hh);
  for (int y = y0; y <= y1; y += 2) {                 // coarse search
    for (int x = x0; x <= x1; x += 2) {
      uint32_t s = sadAt(x, y, 2, best);
      if (s < best) { best = s; px = x; py = y; }
    }
  }
  best = 0xFFFFFFFF;
  int rx = px, ry = py;
  for (int dy = -1; dy <= 1; dy++) {                  // fine search around the best spot
    for (int dx = -1; dx <= 1; dx++) {
      int x = constrain(px + dx, hw, OUT_W - hw), y = constrain(py + dy, hh, OUT_H - hh);
      uint32_t s = sadAt(x, y, 1, best);
      if (s < best) { best = s; rx = x; ry = y; }
    }
  }
  bx = rx; by = ry;
  return (int)(best / (TW * TH));
}

// Let the patch slowly follow the target's changing appearance (lighting, angle)
static void adaptTemplate(int cx, int cy) {
  int x0 = cx - TW / 2, y0 = cy - TH / 2;
  for (int j = 0; j < TH; j++) {
    const uint8_t *g = gcur + (y0 + j) * OUT_W + x0;
    uint8_t *t = tmpl + j * TW;
    for (int i = 0; i < TW; i++) t[i] = (uint8_t)((int)t[i] + ((int)g[i] - (int)t[i]) / 8);
  }
}

// ---------------- Scan / lock logic ----------------
static void startScan(int lo, int hi) {
  scanLo = constrain(lo, 0, 180);
  scanHi = constrain(hi, 0, 180);
  scanN  = (scanHi - scanLo) / SCAN_STEP + 1;
  if (scanN > 40) scanN = 40;
  scanK = 0;
  scanMoved = false;
  scanUp = !scanUp;                                   // sweep back and forth
  for (int i = 0; i < scanN; i++) scanD[i] = -1;
  st = ST_SCAN;
}

static inline int scanAngleAt(int k) {
  return scanUp ? scanLo + k * SCAN_STEP : scanLo + (scanN - 1 - k) * SCAN_STEP;
}

static void finishScan() {
  int best = -1;
  float bestD = 9999;
  for (int i = 0; i < scanN; i++) {
    if (scanD[i] > 0 && scanD[i] < bestD) { bestD = scanD[i]; best = i; }
  }
  if (best < 0) {
    Serial.println("Scan: nothing within range");
    startScan(0, 180);
    return;
  }
  // Aim at the middle of the angles that saw the object at about the same distance
  int l = best, r = best;
  while (l > 0 && scanD[l - 1] > 0 && scanD[l - 1] <= bestD + 8) l--;
  while (r < scanN - 1 && scanD[r + 1] > 0 && scanD[r + 1] <= bestD + 8) r++;
  float ang = scanLo + 0.5f * (l + r) * SCAN_STEP;
  servoAngle = ang;
  servoWrite((int)ang);
  lockT = millis();
  st = ST_LOCK;
  Serial.printf("Scan: object at %.0f deg, %.1f cm - locking\n", ang, bestD);
}

// ---------------- Vision task ----------------
void visionTask(void *) {
  int miss = 0, distMiss = 0;
  uint32_t lastDistT = 0, lastPrint = 0;
  servoWrite(90);
  startScan(0, 180);

  for (;;) {
    if (recenter)  { recenter = false;  servoAngle = 90; servoWrite(90); startScan(0, 180); }
    if (rescanReq) { rescanReq = false; startScan(0, 180); }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
    if (fb->width != 320 || fb->height != 240) {
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    processFrame(fb);
    esp_camera_fb_return(fb);

    if (st == ST_SCAN) {
      if (!scanMoved) {
        int a = scanAngleAt(scanK);
        servoAngle = a;
        servoWrite(a);
        scanMoveT = millis();
        scanMoved = true;
      } else if (millis() - scanMoveT >= SETTLE_MS) {
        int a = scanAngleAt(scanK);
        scanD[(a - scanLo) / SCAN_STEP] = readDistanceCm();
        scanK++;
        scanMoved = false;
        if (scanK >= scanN) finishScan();
      }

    } else if (st == ST_LOCK) {
      if (millis() - lockT >= LOCK_SETTLE_MS) {
        float d = readDistanceCm();
        if (d < 0) {
          Serial.println("Lock: object gone, rescanning");
          startScan(0, 180);
        } else {
          // Learn the target's appearance from the middle of the image
          int x0 = OUT_W / 2 - TW / 2, y0 = OUT_H / 2 - TH / 2;
          for (int j = 0; j < TH; j++)
            for (int i = 0; i < TW; i++) tmpl[j * TW + i] = gcur[(y0 + j) * OUT_W + x0 + i];
          tx = OUT_W / 2;
          ty = OUT_H / 2;
          miss = 0;
          distMiss = 0;
          lastDist = d;
          lastDistT = millis();
          st = ST_TRACK;
          Serial.printf("Locked on target at %.1f cm\n", d);
        }
      }

    } else {   // ST_TRACK
      int bx, by;
      int score = matchTemplate(tx, ty, bx, by);
      lastScore = score;
      if (score <= MATCH_MAX) {
        miss = 0;
        tx = bx;
        ty = by;
        adaptTemplate(tx, ty);
      } else {
        miss++;
      }

      // Turn the servo to keep the target in the middle of the image
      float errDeg = servoDir * ((tx - OUT_W / 2.0f) / OUT_W) * fovDeg;
      if (miss == 0 && fabsf(errDeg) > DEADBAND_DEG) {
        float stepDeg = constrain(KP * errDeg, -MAX_STEP_DEG, MAX_STEP_DEG);
        servoAngle = constrain(servoAngle + stepDeg, 0.0f, 180.0f);
        servoWrite((int)servoAngle);
      }

      // Make sure something is still within range in front of the sensor
      if (millis() - lastDistT >= DIST_CHECK_MS) {
        lastDistT = millis();
        float d = readDistanceCm();
        if (d > 0) { distMiss = 0; lastDist = d; }
        else       { distMiss++; lastDist = -1; }
      }

      if (millis() - lastPrint > 500) {
        lastPrint = millis();
        Serial.printf("Tracking: x=%d match=%d servo=%d deg dist=%.1f cm\n",
                      tx, score, (int)servoAngle, (float)lastDist);
      }

      if (miss >= MISS_MAX || distMiss >= DIST_MISS_MAX) {
        Serial.printf("Target lost (%s), searching nearby\n",
                      miss >= MISS_MAX ? "image match" : "out of range");
        int a = (int)servoAngle;
        startScan(a - 30, a + 30);
      }
    }

    // Overlay: a bar at the top shows the state (red = scanning, yellow = locking, green = tracking)
    uint16_t barCol = (st == ST_SCAN) ? COL_RED : (st == ST_LOCK) ? COL_YELLOW : COL_GREEN;
    for (int y = 0; y < 3; y++) for (int x = 0; x < OUT_W; x++) putPx(x, y, barCol);
    if (st == ST_TRACK) {
      uint16_t boxCol = (miss == 0) ? COL_GREEN : COL_YELLOW;
      drawBox(tx - TW / 2, ty - TH / 2, tx + TW / 2 - 1, ty + TH / 2 - 1, boxCol);
      for (int k = -4; k <= 4; k++) { putPx(tx + k, ty, COL_RED); putPx(tx, ty + k, COL_RED); }
    }

    xSemaphoreTake(frameMutex, portMAX_DELAY);
    memcpy(shareBuf, workBuf, NPIX * 2);
    frameId++;
    xSemaphoreGive(frameMutex);
  }
}

// ---------------- Web page and stream ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:Helvetica;text-align:center;margin:12px}
  canvas{width:100%;max-width:640px;image-rendering:pixelated;background:#000}
  #fps{color:#666;font-size:14px}
</style>
</head><body>
<h2>OV7670 Target Lock</h2>
<canvas id="c" width="160" height="120"></canvas>
<p>Top bar: <b style="color:#d00">red</b> = scanning, <b style="color:#c90">yellow</b> = locking, <b style="color:#0a0">green</b> = tracking</p>
<p><label><input type="checkbox" id="swap"> swap bytes (tick if colors look wrong)</label></p>
<p id="fps">connecting...</p>
<script>
const cv = document.getElementById('c');
const ctx = cv.getContext('2d');
const swapBox = document.getElementById('swap');
const fpsEl = document.getElementById('fps');

function draw(img, frame, swap) {
  const d = img.data;
  for (let i = 0, j = 0; i < frame.length; i += 2, j += 4) {
    const v = swap ? ((frame[i + 1] << 8) | frame[i]) : ((frame[i] << 8) | frame[i + 1]);
    const r5 = (v >> 11) & 31, g6 = (v >> 5) & 63, b5 = v & 31;
    d[j]     = (r5 << 3) | (r5 >> 2);
    d[j + 1] = (g6 << 2) | (g6 >> 4);
    d[j + 2] = (b5 << 3) | (b5 >> 2);
    d[j + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

async function run() {
  try {
    const res = await fetch('/stream', {cache: 'no-store'});
    const reader = res.body.getReader();
    const hdr = new Uint8Array(4);
    let hfill = 0, W = 0, H = 0, frameLen = 0, frame = null, fill = 0, img = null;
    let frames = 0, t0 = performance.now();

    while (true) {
      const {value, done} = await reader.read();
      if (done) break;
      let p = 0;
      while (p < value.length) {
        if (!W) {
          const n = Math.min(4 - hfill, value.length - p);
          hdr.set(value.subarray(p, p + n), hfill);
          hfill += n; p += n;
          if (hfill === 4) {
            W = (hdr[0] << 8) | hdr[1];
            H = (hdr[2] << 8) | hdr[3];
            frameLen = W * H * 2;
            frame = new Uint8Array(frameLen);
            cv.width = W; cv.height = H;
            img = ctx.createImageData(W, H);
          }
        } else {
          const n = Math.min(frameLen - fill, value.length - p);
          frame.set(value.subarray(p, p + n), fill);
          fill += n; p += n;
          if (fill === frameLen) {
            draw(img, frame, swapBox.checked);
            fill = 0; frames++;
            const now = performance.now();
            if (now - t0 > 1000) {
              fpsEl.textContent = (frames * 1000 / (now - t0)).toFixed(1) + ' fps  (' + W + 'x' + H + ')';
              frames = 0; t0 = now;
            }
          }
        }
      }
    }
  } catch (e) {}
  fpsEl.textContent = 'stream lost, reconnecting...';
  setTimeout(run, 1000);
}
run();
</script>
</body></html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

static bool sendAll(WiFiClient &c, const uint8_t *p, size_t n) {
  uint32_t t = millis();
  while (n) {
    size_t w = c.write(p, n);
    if (w == 0) {
      if (!c.connected() || millis() - t > 2000) return false;
      delay(1);
      continue;
    }
    p += w; n -= w; t = millis();
  }
  return true;
}

void handleStream() {
  WiFiClient client = server.client();
  client.setNoDelay(true);
  client.print("HTTP/1.1 200 OK\r\n"
               "Content-Type: application/octet-stream\r\n"
               "Cache-Control: no-store\r\n"
               "Connection: close\r\n\r\n");

  uint8_t h[4] = { (uint8_t)(OUT_W >> 8), (uint8_t)OUT_W,
                   (uint8_t)(OUT_H >> 8), (uint8_t)OUT_H };
  if (!sendAll(client, h, 4)) { client.stop(); return; }

  uint32_t lastId = 0;
  while (client.connected()) {
    if (frameId == lastId) { delay(2); continue; }

    xSemaphoreTake(frameMutex, portMAX_DELAY);
    memcpy(sendBuf, shareBuf, NPIX * 2);
    lastId = frameId;
    xSemaphoreGive(frameMutex);

    if (!sendAll(client, (const uint8_t *)sendBuf, NPIX * 2)) break;
  }
  client.stop();
}

// ---------------- Camera setup ----------------
bool startCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = PIN_D0;
  config.pin_d1 = PIN_D1;
  config.pin_d2 = PIN_D2;
  config.pin_d3 = PIN_D3;
  config.pin_d4 = PIN_D4;
  config.pin_d5 = PIN_D5;
  config.pin_d6 = PIN_D6;
  config.pin_d7 = PIN_D7;
  config.pin_xclk  = PIN_XCLK;
  config.pin_pclk  = PIN_PCLK;
  config.pin_vsync = PIN_VSYNC;
  config.pin_href  = PIN_HREF;
  config.pin_sccb_sda = PIN_SDA;
  config.pin_sccb_scl = PIN_SCL;
  config.pin_pwdn  = PIN_PWDN;
  config.pin_reset = PIN_RESET;

  config.xclk_freq_hz = XCLK_HZ;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QVGA;      // 320x240
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    snprintf(camMsg, sizeof(camMsg), "init failed 0x%x", err);
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  snprintf(camMsg, sizeof(camMsg), "OK (sensor PID 0x%x)", s->id.PID);
  return true;
}

static void *allocBuf(size_t n, bool preferPsram) {
  void *p = nullptr;
  if (preferPsram) p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  if (!p) p = heap_caps_malloc(n, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (!p) p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  return p;
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);                        // never stall when no monitor is connected
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 8000) delay(10);   // wait up to 8 s for the monitor to reconnect
  delay(300);

  Serial.printf("PSRAM: %s | free internal heap: %u\n",
                psramFound() ? "found" : "NOT found",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  frameMutex = xSemaphoreCreateMutex();

  // 1. Camera first, so it gets its memory before anything else
  camOk = startCamera();
  Serial.printf("Camera: %s\n", camMsg);
  if (!camOk) {
    Serial.println("Camera failed. Check wiring, pull-ups, PWDN/RESET and power.");
    return;
  }

  // 2. Processing buffers
  workBuf  = (uint16_t *)allocBuf(NPIX * 2, false);
  shareBuf = (uint16_t *)allocBuf(NPIX * 2, true);
  sendBuf  = (uint16_t *)allocBuf(NPIX * 2, true);
  gcur     = (uint8_t  *)allocBuf(NPIX, false);
  if (!workBuf || !shareBuf || !sendBuf || !gcur) {
    snprintf(camMsg, sizeof(camMsg), "buffer allocation failed");
    Serial.println("Buffer allocation failed.");
    return;
  }
  memset(workBuf, 0, NPIX * 2);
  memset(shareBuf, 0, NPIX * 2);
  memset(gcur, 0, NPIX);

  // 3. Servo, vision task, WiFi, web server
  servoInit();
  xTaskCreatePinnedToCore(visionTask, "vision", 8192, nullptr, 2, nullptr, 1);

  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", handleRoot);
  server.on("/stream", handleStream);
  server.begin();
  Serial.print("Join WiFi '");
  Serial.print(AP_SSID);
  Serial.print("' and open http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("Serial commands: r = reverse turn direction, + / - = field of view +/-5 deg, x = rescan, c = center and rescan");
}

void loop() {
  server.handleClient();
  delay(1);

  // Live calibration from the Serial Monitor
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '+') { fovDeg = min(fovDeg + 5.0f, 140.0f); Serial.printf("FOV = %.0f deg\n", (float)fovDeg); }
    if (c == '-') { fovDeg = max(fovDeg - 5.0f, 10.0f);  Serial.printf("FOV = %.0f deg\n", (float)fovDeg); }
    if (c == 'r') { servoDir = -servoDir; Serial.printf("Direction = %d\n", (int)servoDir); }
    if (c == 'x') { rescanReq = true; Serial.println("Rescanning"); }
    if (c == 'c') { recenter = true; Serial.println("Centering"); }
  }

  // Status every 3 s (shows up even if the monitor connected late)
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 3000) {
    lastStatus = millis();
    const char *names[] = { "SCAN", "LOCK", "TRACK" };
    Serial.printf("[status] camera: %s | state: %s | frames: %u | http://%s\n",
                  camMsg, names[(int)st], (unsigned)frameId, WiFi.softAPIP().toString().c_str());
  }
}