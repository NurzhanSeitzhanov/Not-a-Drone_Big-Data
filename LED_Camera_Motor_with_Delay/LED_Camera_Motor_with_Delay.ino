#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "board_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"   // IDF LEDC for motor

// =========================== LED (GPIO9) ===========================
#define FIRE_LED_PIN 9
#define LED_MODE_OFF    0
#define LED_MODE_BLINK  1   // "Possible fire"
#define LED_MODE_ON     2   // "Fire likely"

#define BLINK_ON_MS   300
#define BLINK_OFF_MS  700

volatile uint8_t g_led_mode = LED_MODE_OFF;

static void taskLED(void*) {
  pinMode(FIRE_LED_PIN, OUTPUT);
  digitalWrite(FIRE_LED_PIN, LOW);

  // quick self-test
  for (int i = 0; i < 2; i++) {
    digitalWrite(FIRE_LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(120));
    digitalWrite(FIRE_LED_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(120));
  }

  for (;;) {
    uint8_t m = g_led_mode;

    if (m == LED_MODE_ON) {
      digitalWrite(FIRE_LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(80));   // short sleep; stays responsive to mode changes
    } else if (m == LED_MODE_BLINK) {
      digitalWrite(FIRE_LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(BLINK_ON_MS));
      if (g_led_mode != LED_MODE_BLINK) continue; // re-check mode
      digitalWrite(FIRE_LED_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(BLINK_OFF_MS));
    } else { // OFF
      digitalWrite(FIRE_LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(80));
    }
  }
}

// =========================== CAMERA (your original JPEG pipeline) ===========================
static bool init_camera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;           // camera uses LEDC timer0/ch0 (XCLK)
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 16500000;                 // your stable setting
  config.pixel_format = PIXFORMAT_JPEG;           // JPEG capture
  config.frame_size   = FRAMESIZE_QVGA;           // 320x240
  config.jpeg_quality = 15;
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_DRAM;

  if (esp_camera_init(&config) != ESP_OK) return false;

  camera_fb_t *t = esp_camera_fb_get();
  if (!t) return false;
  Serial.printf("Camera OK: %dx%d fmt=%d\n", t->width, t->height, (int)t->format);
  esp_camera_fb_return(t);
  return true;
}

static bool capture_once(camera_fb_t*& fb_out, int tries = 5) {
  for (int i = 0; i < tries; ++i) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) { fb_out = fb; return true; }
    Serial.printf("[CAM] Capture failed (try %d/%d)\n", i + 1, tries);
    delay(50);
  }
  return false;
}

// =========================== MOTOR (GPIO8, LEDC timer1/ch1) ===========================
#define MOTOR_PIN           8
#define MOTOR_SPEEDMODE     LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_TIMER    LEDC_TIMER_1
#define MOTOR_LEDC_CH       LEDC_CHANNEL_1
#define MOTOR_PWM_HZ        25000
#define MOTOR_PWM_BITS      LEDC_TIMER_8_BIT

// One-time startup delay BEFORE the first motor run (change as needed)
#define MOTOR_START_DELAY_MS  60000  // e.g. 60s

static const uint8_t  DUTY           = 150;
static const uint16_t STEP_MS        = 1000;
static const uint16_t PAUSE_MS       = 10000;
static const uint16_t FULL_PAUSE     = 60000;
static const uint16_t RAMP_MS        = 200;
static const int      STEPS_PER_TURN = 8;

static inline void motorWrite(uint8_t duty){
  ledc_set_duty(MOTOR_SPEEDMODE, MOTOR_LEDC_CH, duty);
  ledc_update_duty(MOTOR_SPEEDMODE, MOTOR_LEDC_CH);
}

static void motorRampTo(uint8_t target, uint16_t rampMs){
  static int cur = 0; const int N = 20;
  for (int i = 1; i <= N; i++) {
    int v = cur + (int)((target - cur) * (float)i / N);
    motorWrite((uint8_t)v);
    vTaskDelay(pdMS_TO_TICKS((rampMs / N) ? (rampMs / N) : 1));
  }
  motorWrite(target); cur = target;
}

static void taskMotor(void*) {
  ledc_timer_config_t tconf = {};
  tconf.speed_mode      = MOTOR_SPEEDMODE;
  tconf.duty_resolution = MOTOR_PWM_BITS;
  tconf.timer_num       = MOTOR_LEDC_TIMER;
  tconf.freq_hz         = MOTOR_PWM_HZ;
  tconf.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&tconf);

  ledc_channel_config_t cconf = {};
  cconf.gpio_num   = MOTOR_PIN;
  cconf.speed_mode = MOTOR_SPEEDMODE;
  cconf.channel    = MOTOR_LEDC_CH;
  cconf.intr_type  = LEDC_INTR_DISABLE;
  cconf.timer_sel  = MOTOR_LEDC_TIMER;
  cconf.duty       = 0;
  cconf.hpoint     = 0;
  ledc_channel_config(&cconf);

  motorWrite(0);

  // >>> One-time startup delay (no PAUSE_MS before first run)
  Serial.printf("[MOTOR] Startup delay: %d ms\n", MOTOR_START_DELAY_MS);
  vTaskDelay(pdMS_TO_TICKS(MOTOR_START_DELAY_MS));
  Serial.println("[MOTOR] Startup delay over — starting first run immediately.");

  for (;;) {
    for (int i = 0; i < STEPS_PER_TURN; i++) {
      Serial.printf("[MOTOR] Step %d/%d: run %u ms, then pause %u ms\n",
                    i+1, STEPS_PER_TURN, (unsigned)STEP_MS, (unsigned)PAUSE_MS);
      motorRampTo(DUTY, RAMP_MS);
      vTaskDelay(pdMS_TO_TICKS(STEP_MS));
      motorRampTo(0, RAMP_MS);
      vTaskDelay(pdMS_TO_TICKS(PAUSE_MS));
    }
    Serial.printf("[MOTOR] Full pause: %u ms\n", (unsigned)FULL_PAUSE);
    vTaskDelay(pdMS_TO_TICKS(FULL_PAUSE));
  }
}

// =========================== Fire detection (HSV, names, smoothing) ===========================
namespace fire {
  // Tunables
  constexpr uint8_t  S_MIN   = 100;  // saturation >= 100/255
  constexpr uint8_t  V_MIN   =  90;  // brightness >= 90/255
  constexpr int      H_RED_L = 345;  // red wrap (345..360)
  constexpr int      H_RED_H = 360;
  constexpr int      H_ROY_L =   0;  // red/orange/yellow band (0..60)
  constexpr int      H_ROY_H =  60;

  struct Stats {
    float hot_ratio;
    uint32_t avg_r, avg_g, avg_b;
    uint32_t avg_h, avg_s, avg_v;
    const char* hue_name;
    const char* confidence;
  };

  // Fast RGB888 -> HSV (H:0..360, S:0..255, V:0..255)
  inline void rgb_to_hsv_u8(uint8_t r, uint8_t g, uint8_t b,
                            uint16_t &H, uint8_t &S, uint8_t &V) {
    uint8_t maxc = r; if (g > maxc) maxc = g; if (b > maxc) maxc = b;
    uint8_t minc = r; if (g < minc) minc = g; if (b < minc) minc = b;
    V = maxc;
    uint8_t delta = (uint8_t)(maxc - minc);
    if (maxc == 0) { S = 0; H = 0; return; }
    S = (uint8_t)((uint16_t)255 * delta / maxc);
    if (delta == 0) { H = 0; return; }
    int32_t h;
    if (maxc == r)      h = 43 * (int32_t)(g - b) / (int32_t)delta;
    else if (maxc == g) h = 85 + 43 * (int32_t)(b - r) / (int32_t)delta;
    else                h = 171 + 43 * (int32_t)(r - g) / (int32_t)delta;
    if (h < 0) h += 256;
    H = (uint16_t)(h * 360u / 256u);
  }

  inline const char* hue_to_name(uint32_t Hdeg) {
    if (Hdeg >= 345 || Hdeg <  15) return "red";
    if (Hdeg <   35) return "orange";
    if (Hdeg <   65) return "yellow";
    if (Hdeg <  170) return "green";
    if (Hdeg <  255) return "cyan";
    if (Hdeg <  320) return "blue";
    return "magenta";
  }

  inline bool is_fire_hsv(uint16_t H, uint8_t S, uint8_t V) {
    bool red_wrap = (H >= H_RED_L && H <= H_RED_H);
    bool roy_band = (H >= H_ROY_L && H <= H_ROY_H);
    return (S >= S_MIN) && (V >= V_MIN) && (red_wrap || roy_band);
  }

  // Temporal smoothing (median-of-3)
  static float last_ratios[3] = {0,0,0};
  static uint8_t ratio_idx = 0;

  void analyze(const uint8_t* rgb, int w, int h, Stats& out) {
    const int total = w * h;
    uint32_t sumR=0, sumG=0, sumB=0, sumH=0, sumS=0, sumV=0;
    uint32_t hot=0;

    for (int i=0; i<total; ++i) {
      uint8_t r = rgb[i*3+0], g = rgb[i*3+1], b = rgb[i*3+2];
      sumR += r; sumG += g; sumB += b;

      uint16_t H; uint8_t S, V;
      rgb_to_hsv_u8(r,g,b, H,S,V);
      sumH += H; sumS += S; sumV += V;

      if (is_fire_hsv(H,S,V)) hot++;
    }

    float ratio = (float)hot / (float)total;

    last_ratios[ratio_idx] = ratio; ratio_idx = (ratio_idx+1)%3;
    float r0 = last_ratios[0], r1 = last_ratios[1], r2 = last_ratios[2];
    float smoothed = (r0 + r1 + r2 - min(r0, min(r1,r2)) - max(r0, max(r1,r2))); // median

    out.hot_ratio = smoothed;
    out.avg_r = sumR/total; out.avg_g = sumG/total; out.avg_b = sumB/total;
    out.avg_h = sumH/total; out.avg_s = sumS/total; out.avg_v = sumV/total;
    out.hue_name = hue_to_name(out.avg_h);

    if (smoothed > 0.01f && out.avg_s >= 120 && out.avg_v >= 110) out.confidence = "high";
    else if (smoothed > 0.004f)                                    out.confidence = "medium";
    else                                                           out.confidence = "low";
  }
} // namespace fire

// =========================== Arduino entry ===========================
unsigned long lastShot = 0;  // 2-second cadence

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nBoot...");

  // Start LED task (core 0), motor task (core 1)
  xTaskCreatePinnedToCore(taskLED,  "taskLED",  2048, NULL, 1, NULL, 0);

  if (!init_camera()) {
    Serial.println("Camera init failed; halting.");
    while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }
  Serial.println("Camera ready.");

  xTaskCreatePinnedToCore(taskMotor, "taskMotor", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // CAMERA: capture & analyze every 2 seconds
  if (millis() - lastShot < 2000) { delay(10); return; }
  lastShot = millis();

  camera_fb_t * fb = nullptr;
  if (!capture_once(fb, 5)) {
    Serial.println("[CAM] capture failed after retries");
    return;
  }

  const int w = fb->width, h = fb->height;
  const size_t out_len = (size_t)w * h * 3;
  uint8_t *rgb_buf = (uint8_t *)malloc(out_len);
  if (!rgb_buf) {
    Serial.printf("[CAM] malloc failed for %u bytes\n", (unsigned)out_len);
    esp_camera_fb_return(fb);
    return;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf);  // JPEG -> RGB888
  esp_camera_fb_return(fb);
  if (!converted) {
    Serial.println("fmt2rgb888() failed");
    free(rgb_buf);
    return;
  }

  fire::Stats fs{};
  fire::analyze(rgb_buf, w, h, fs);
  free(rgb_buf);

  // Human-friendly summary + decision
  Serial.printf("Hot: %.2f%% | AvgRGB: %lu/%lu/%lu | AvgHSV: H=%lu° S=%lu V=%lu (%s)\n",
                fs.hot_ratio * 100.0f,
                fs.avg_r, fs.avg_g, fs.avg_b,
                fs.avg_h, fs.avg_s, fs.avg_v, fs.hue_name);

  if (fs.hot_ratio > 0.005f) {
    Serial.printf("🔥 Fire likely (%s confidence)\n", fs.confidence);
    g_led_mode = LED_MODE_ON;       // steady ON
  } else if (fs.hot_ratio > 0.002f) {
    Serial.printf("⚠️  Possible fire (%s confidence)\n", fs.confidence);
    g_led_mode = LED_MODE_BLINK;    // slow blink
  } else {
    Serial.println("No fire");
    g_led_mode = LED_MODE_OFF;      // OFF
  }
}
