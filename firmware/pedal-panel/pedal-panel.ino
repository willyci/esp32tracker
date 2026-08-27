// ESP32-S3-Touch-AMOLED-2.06 → BLE BROADCAST touchscreen pedal panel.
//
// Replaces the three foot pedals (X-ray / DSA / capture) with one touchscreen: three
// full-width buttons on a 410x502 AMOLED, with LRA haptics and a speaker click. Also
// carries a SoftPot strip, so one board covers the floor controls and a touch input.
//
// WHY BROADCAST, NOT CONNECT: Vision Pro has a small BLE connection budget — with the two
// hand trackers connected it refuses more (CBError 11). So, exactly like the foot-pedal sketches,
// this panel never holds a connection: it ADVERTISES its state and consumers read it out
// of a continuous scan.
//
// CONSEQUENCE YOU CAN SEE ON SCREEN: a broadcaster cannot know whether anyone is
// listening. The status screen therefore reports that we are ADVERTISING, not that we are
// "connected", and the hop to the button screen is a 15 s timer from boot rather than a
// connection event. The ready chime likewise plays when advertising starts.
//
// WIRING — only ONE GPIO is claimed; everything else rides the board's own buses:
//   SoftPot  V+→3V3  GND→GND  wiper→IO19    (ADC2_CH8)
//     * 3V3, NOT VBUS: 5 V on the wiper would exceed the ADC input range.
//     * IO19 is USB D−, so NATIVE USB IS GIVEN UP. Set "USB CDC On Boot: Disabled" and
//       use the UART0 pins (RXD/TXD = 44/43) with a USB-serial adapter for the console.
//     * To reflash over USB, unplug the SoftPot from IO19 first — a resistive divider on
//       D− stops the port enumerating. Reconnect it afterwards.
//     * ADC2 is blocked while WiFi runs. WiFi is unused here and BLE does not take that
//       lock, so this works — but if the reading goes erratic ONLY while BLE transmits,
//       that is the cause; move the strip to an ADS1115 on the I2C bus (0x48).
//   DRV2605L SDA→IO15  SCL→IO14  VDD→3V3  GND→GND   (addr 0x5A, shares the board bus)
//   LRA      → DRV2605L OUT+/OUT-
//   The board's own 2 motor pads are LEFT UNUSED: that output is the GPIO18 ERM/DC driver,
//   which is the wrong waveform for an LRA (it would buzz, not click).
//
// TOOLCHAIN — THIS BOARD IS THE ODD ONE OUT IN THIS REPO:
//   * Arduino-ESP32 core **3.3.11** (Waveshare's stated version; the ES8311 audio API
//     ESP_I2S.h only exists on 3.x). Every OTHER board here needs core 2.0.17, because
//     NimBLE-Arduino 1.4.x crashes on 3.x — so you must switch the CORE in Boards Manager
//     depending on which board you are flashing.
//   * BLE here uses the **core-bundled BLE library**, not NimBLE-Arduino. NimBLE 1.4.x will
//     not work on core 3.x and NimBLE 2.x would then break the 2.0.17 boards, since the
//     library version is shared across the sketchbook. Using the bundled BLE means you swap
//     only the core and never the library.
//   * The advertisement is built by hand (flags + 128-bit service UUID + manufacturer data,
//     28 of the 31 available bytes) because the visionOS app scans with a SERVICE FILTER —
//     drop the UUID and the headset never sees this panel.
//
// Board: "ESP32S3 Dev Module", USB CDC On Boot: DISABLED, PSRAM enabled, 115200 baud.
// Libraries: lvgl 9 + Arduino_GFX + Arduino_DriveBus + XPowersLib (all bundled with the
// Waveshare repo), Adafruit DRV2605. BLE and ESP_I2S come with the core.

#include <Wire.h>
#include <Arduino.h>
#include "pin_config.h"
#include <lvgl.h>

#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "lv_conf.h"

#include "XPowersLib.h"
#include <BLEDevice.h>          // core-bundled BLE — deliberately NOT NimBLE, see below
#include <Adafruit_DRV2605.h>

#include "ESP_I2S.h"
#include "esp_check.h"
#include "es8311.h"
#include "driver/gpio.h"

// ---------------------------------------------------------------------------
// Pins and constants
// ---------------------------------------------------------------------------
static constexpr int PIN_SOFTPOT = 19;            // ADC2_CH8 — see WIRING note above
static constexpr int SOFTPOT_NOTOUCH_RAW = 80;    // raw ADC below this = no touch

static constexpr int PIN_I2S_BCLK = 41;           // from examples/arduino/08_ES8311
static constexpr int PIN_I2S_WS   = 45;
static constexpr int PIN_I2S_DOUT = 40;
static constexpr int PIN_I2S_DIN  = 42;
static constexpr int PIN_I2S_MCLK = 16;
static constexpr int PIN_AUDIO_PA = 46;           // speaker amplifier enable

static constexpr uint32_t SAMPLE_RATE   = 16000;
static constexpr int      VOICE_VOLUME  = 85;     // 0-100
static constexpr int      I2C_NUM       = 0;      // Wire's port; es8311 shares it

// ---- BLE identifiers — service UUID must match the app's scan filter ----
#define SERVICE_UUID  "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define DEVICE_NAME   "Pedal Panel"

// Manufacturer data, deliberately the SAME 5-byte shape the foot pedals use so consumers
// need only a small addition: [0xFF, 0xFF, captureCount, levelBits, flags].
//   [2] captureCount — increments once per CAPTURE tap (wraps 0-255; consumers watch for change)
//   [3] levelBits    — LIVE LEVELS: bit0 = X-ray held, bit1 = DSA held
//   [4] flags        — reserved, 0
static constexpr uint8_t MFG_ID_LO = 0xFF;
static constexpr uint8_t MFG_ID_HI = 0xFF;
static constexpr uint8_t LEVEL_XRAY = 0x01;
static constexpr uint8_t LEVEL_DSA  = 0x02;

static uint8_t captureCount = 0;
static uint8_t levelBits    = 0;
static uint8_t mfgFlags     = 0;

// Accumulated held-time, shown on the buttons. Local to the panel — a consumer can derive
// the same thing from the levels, so it is not worth spending broadcast bytes on.
static uint32_t xrayHeldMs = 0, dsaHeldMs = 0;
static uint32_t xraySinceMs = 0, dsaSinceMs = 0;   // millis() at the last press

static constexpr uint32_t AUTO_SWITCH_MS = 15000;  // status -> buttons, if not cancelled
static uint32_t bootMs = 0;
static bool autoSwitchDone = false;                // also set when the user hits BACK

// ---------------------------------------------------------------------------
// Display + touch (construction copied verbatim from examples/arduino/06_LVGL_Arduino_v9)
// ---------------------------------------------------------------------------
#define DIRECT_RENDER_MODE

uint32_t screenWidth, screenHeight, bufSize;
lv_display_t *disp;
lv_color_t *disp_draw_buf;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RESET, 0 /* rotation */,
                                      LCD_WIDTH, LCD_HEIGHT,
                                      22 /* col_offset1 */, 0, 0, 0);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);
std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, TP_INT,
                                                       Arduino_IIC_Touch_Interrupt));
void Arduino_IIC_Touch_Interrupt(void) { FT3168->IIC_Interrupt_Flag = true; }

XPowersPMU power;
Adafruit_DRV2605 drv;
static bool hapticsOK = false;

I2SClass i2s;
static bool audioOK = false;

BLEAdvertising *adv = nullptr;

// LVGL objects we update later
static lv_obj_t *scrStatus = nullptr, *scrButtons = nullptr;
static lv_obj_t *lblBleState = nullptr, *lblBatt = nullptr, *lblI2C = nullptr;
static lv_obj_t *lblSoftPot = nullptr, *lblCountdown = nullptr;
static lv_obj_t *lblXray = nullptr, *lblDsa = nullptr, *lblCapture = nullptr;

// ---------------------------------------------------------------------------
// BLE broadcast
// ---------------------------------------------------------------------------
// Rebuild the whole advertisement so the current state goes out on the air.
//
// The payload is assembled by hand rather than letting the library auto-generate it,
// because setAdvertisementData() REPLACES the auto payload — and the visionOS app scans
// with a service filter, so the 128-bit UUID has to be in there or the headset is blind
// to us. Budget: flags 3 + 128-bit UUID 18 + manufacturer data 7 = 28 of 31 bytes.
// The device NAME lives in the scan response (it would not fit alongside the UUID).
void publishAdvertising() {
  uint8_t mfg[5] = { MFG_ID_LO, MFG_ID_HI, captureCount, levelBits, mfgFlags };

  BLEAdvertisementData advData;
  advData.setFlags(0x06);                       // LE General Discoverable, BR/EDR not supported
  advData.setCompleteServices(BLEUUID(SERVICE_UUID));
  // String(ptr, len) — the length-taking ctor, since this payload contains NUL bytes.
  advData.setManufacturerData(String(reinterpret_cast<const char *>(mfg), sizeof(mfg)));

  BLEAdvertisementData scanData;
  scanData.setName(DEVICE_NAME);

  adv->stop();
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->start();
}

void setupBLE() {
  Serial.println("[BLE] init (broadcast-only)...");
  BLEDevice::init(DEVICE_NAME);

  adv = BLEDevice::getAdvertising();
  adv->setScanResponse(true);
  // Advertise fast and explicitly so on-air latency is pinned here rather than inherited.
  // (Measured caveat from the foot pedals: the HOST may still only deliver ~1-3 ads/s, so
  // never size consumer timeouts from this interval.)
  adv->setMinInterval(160);   // units of 0.625 ms -> 100 ms
  adv->setMaxInterval(240);   // -> 150 ms

  publishAdvertising();       // sets both payloads and starts advertising
  Serial.println("[BLE] advertising as " DEVICE_NAME " (no connection needed)");
}

// ---------------------------------------------------------------------------
// Audio — short tones generated in code (no embedded PCM)
// ---------------------------------------------------------------------------
esp_err_t es8311_codec_init(void) {
  es8311_handle_t es_handle = es8311_create(I2C_NUM, ES8311_ADDRRES_0);
  if (!es_handle) return ESP_FAIL;
  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = (int)(SAMPLE_RATE * 256),
    .sample_frequency = (int)SAMPLE_RATE
  };
  ESP_RETURN_ON_ERROR(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16),
                      "panel", "es8311 init failed");
  ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency,
                                                     es_clk.sample_frequency),
                      "panel", "es8311 sample rate failed");
  ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, VOICE_VOLUME, NULL),
                      "panel", "es8311 volume failed");
  es8311_microphone_config(es_handle, false);
  return ESP_OK;
}

// Blocking, so keep tones SHORT — a long one visibly stalls the LVGL redraw.
void playTone(uint16_t freqHz, uint16_t ms, float amplitude = 0.35f) {
  if (!audioOK) return;
  const uint32_t total = (uint32_t)SAMPLE_RATE * ms / 1000;
  static int16_t buf[512 * 2];                 // 512 stereo frames per chunk
  float phase = 0.0f;
  const float step = 2.0f * PI * freqHz / (float)SAMPLE_RATE;
  uint32_t done = 0;
  while (done < total) {
    uint32_t remain = total - done;
    uint32_t n = (remain < 512) ? remain : 512;   // NB: avoid min<>(), Arduino makes min a macro
    for (uint32_t i = 0; i < n; i++) {
      // Fade the envelope in and out so the click has no hard edge (which sounds like a pop).
      float env = 1.0f;
      uint32_t pos = done + i;
      const uint32_t fade = SAMPLE_RATE / 200;                    // ~5 ms
      if (pos < fade)               env = (float)pos / fade;
      else if (total - pos < fade)  env = (float)(total - pos) / fade;
      int16_t s = (int16_t)(sinf(phase) * 32767.0f * amplitude * env);
      buf[i * 2] = s; buf[i * 2 + 1] = s;                          // both channels
      phase += step;
      if (phase > 2.0f * PI) phase -= 2.0f * PI;
    }
    i2s.write((uint8_t *)buf, n * 2 * sizeof(int16_t));
    done += n;
  }
}

void soundClick()  { playTone(2000, 25); }
void soundReady()  { playTone(880, 70); playTone(1320, 90); }   // two-tone "BLE up" chime

// ---------------------------------------------------------------------------
// Haptics
// ---------------------------------------------------------------------------
void hapticClick(uint8_t effect = 1) {   // 1 = strong click 100%
  if (!hapticsOK) return;
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);                 // end of sequence
  drv.go();
}

// Both feedback channels together — every button press goes through here.
void pressFeedback(uint8_t effect = 1) {
  hapticClick(effect);
  soundClick();
}

// ---------------------------------------------------------------------------
// SoftPot — same behaviour as firmware/left-mini (touch-down capture + EMA smoothing)
// ---------------------------------------------------------------------------
static float softpotEMA = 0;
static bool  touching = false;
static uint8_t touchStart = 0, touchCurrent = 0;
static int lastSoftPotRaw = 0;

void readSoftPot() {
  int raw = analogRead(PIN_SOFTPOT);
  lastSoftPotRaw = raw;
  if (raw < SOFTPOT_NOTOUCH_RAW) {
    touching = false; touchStart = 0; touchCurrent = 0; softpotEMA = 0;
    return;
  }
  uint8_t pos = (uint8_t)constrain(map(raw, SOFTPOT_NOTOUCH_RAW, 4095, 1, 255), 1, 255);
  if (!touching) { touching = true; touchStart = pos; softpotEMA = pos; }
  else           { softpotEMA = 0.6f * softpotEMA + 0.4f * pos; }
  touchCurrent = (uint8_t)softpotEMA;
}

// ---------------------------------------------------------------------------
// I2C scan — bring-up aid; expect 0x38 touch, 0x34 AXP2101, 0x6A QMI8658,
// 0x51 RTC and 0x5A DRV2605L.
// ---------------------------------------------------------------------------
static char i2cSummary[64] = "scanning...";
void i2cScan() {
  Serial.println("[I2C] scanning bus...");
  uint8_t found = 0;
  size_t used = 0;
  i2cSummary[0] = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      const char *who = (addr == 0x38) ? " touch" : (addr == 0x34) ? " pmu"
                      : (addr == 0x5A) ? " drv"   : (addr == 0x51) ? " rtc" : "";
      Serial.printf("[I2C]   device at 0x%02X%s\n", addr, who);
      if (used < sizeof(i2cSummary) - 8)
        used += snprintf(i2cSummary + used, sizeof(i2cSummary) - used, "%02X ", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("[I2C]   NONE found");
    snprintf(i2cSummary, sizeof(i2cSummary), "none!");
  }
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
void showButtons() {
  autoSwitchDone = true;             // never auto-hop again this session
  lv_scr_load(scrButtons);
}

static void back_cb(lv_event_t *e) {
  LV_UNUSED(e);
  pressFeedback(24);                 // 24 = sharp tick, lighter than a pedal press
  autoSwitchDone = true;             // BACK must stick: don't bounce to buttons in 15 s
  lv_scr_load(scrStatus);
}

// X-ray and DSA are HOLD controls (dead-man switch, like the real pedals): the level is
// true for exactly as long as a finger is down. Capture is a tap.
static void xray_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED) {
    levelBits |= LEVEL_XRAY;
    xraySinceMs = millis();
    pressFeedback(1);
    publishAdvertising();
    Serial.println(">> X-RAY down");
  } else if (code == LV_EVENT_RELEASED) {
    levelBits &= ~LEVEL_XRAY;
    if (xraySinceMs) { xrayHeldMs += millis() - xraySinceMs; xraySinceMs = 0; }
    publishAdvertising();
    Serial.println(">> X-RAY up");
  }
}

static void dsa_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED) {
    levelBits |= LEVEL_DSA;
    dsaSinceMs = millis();
    pressFeedback(1);
    publishAdvertising();
    Serial.println(">> DSA run START");
  } else if (code == LV_EVENT_RELEASED) {
    levelBits &= ~LEVEL_DSA;
    if (dsaSinceMs) { dsaHeldMs += millis() - dsaSinceMs; dsaSinceMs = 0; }
    publishAdvertising();
    Serial.println(">> DSA run END");
  }
}

static void capture_cb(lv_event_t *e) {
  LV_UNUSED(e);
  captureCount++;
  pressFeedback(1);
  publishAdvertising();
  Serial.printf(">> CAPTURE #%u\n", captureCount);
}

static lv_obj_t *makePedalButton(lv_obj_t *parent, const char *text, lv_color_t colour,
                                 int yOffset, int height, lv_obj_t **labelOut,
                                 lv_event_cb_t cb, lv_event_code_t filter) {
  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_size(btn, LCD_WIDTH - 16, height);
  lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, yOffset);
  lv_obj_set_style_bg_color(btn, colour, LV_PART_MAIN);
  lv_obj_set_style_radius(btn, 14, LV_PART_MAIN);
  lv_obj_add_event_cb(btn, cb, filter, NULL);
  if (filter == LV_EVENT_PRESSED)          // hold buttons need both edges
    lv_obj_add_event_cb(btn, cb, LV_EVENT_RELEASED, NULL);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, text);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_center(lbl);
  *labelOut = lbl;
  return btn;
}

static lv_obj_t *makeStatusLine(lv_obj_t *parent, int y, const char *initial) {
  lv_obj_t *lbl = lv_label_create(parent);
  lv_label_set_text(lbl, initial);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xE8E8EA), LV_PART_MAIN);
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 14, y);
  return lbl;
}

void buildUI() {
  // ---- status screen ----
  scrStatus = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scrStatus, lv_color_hex(0x14161A), LV_PART_MAIN);

  lv_obj_t *title = lv_label_create(scrStatus);
  lv_label_set_text(title, "PEDAL PANEL");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_set_style_text_color(title, lv_color_hex(0x4ADE80), LV_PART_MAIN);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 24);

  lblBleState  = makeStatusLine(scrStatus, 90,  "BLE: starting...");
  lblBatt      = makeStatusLine(scrStatus, 130, "Batt: --");
  lblI2C       = makeStatusLine(scrStatus, 170, "I2C: --");
  lblSoftPot   = makeStatusLine(scrStatus, 210, "SoftPot: --");
  lblCountdown = makeStatusLine(scrStatus, 260, "");

  lv_obj_t *hint = lv_label_create(scrStatus);
  lv_label_set_text(hint, "Tap anywhere for the pedals");
  lv_obj_set_style_text_font(hint, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_style_text_color(hint, lv_color_hex(0x9AA0A8), LV_PART_MAIN);
  lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_add_flag(scrStatus, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(scrStatus, [](lv_event_t *e) { LV_UNUSED(e); showButtons(); },
                      LV_EVENT_CLICKED, NULL);

  // ---- button screen ----
  scrButtons = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scrButtons, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_pad_all(scrButtons, 0, LV_PART_MAIN);
  lv_obj_clear_flag(scrButtons, LV_OBJ_FLAG_SCROLLABLE);

  const int h = (LCD_HEIGHT - 4 * 8) / 3;        // three buttons + even gaps
  makePedalButton(scrButtons, "X-RAY 0s",   lv_color_hex(0x14532D), 8,
                  h, &lblXray, xray_cb, LV_EVENT_PRESSED);
  makePedalButton(scrButtons, "DSA 0s",     lv_color_hex(0x7C2D12), 8 + h + 8,
                  h, &lblDsa, dsa_cb, LV_EVENT_PRESSED);
  makePedalButton(scrButtons, "CAPTURE 0",  lv_color_hex(0x1E3A5F), 8 + 2 * (h + 8),
                  h, &lblCapture, capture_cb, LV_EVENT_CLICKED);

  // BACK sits ON TOP of the X-ray button; LVGL gives the click to the topmost object,
  // so it is created last and therefore wins.
  lv_obj_t *back = lv_button_create(scrButtons);
  lv_obj_set_size(back, 96, 52);
  lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
  lv_obj_set_style_bg_color(back, lv_color_hex(0x2A2E36), LV_PART_MAIN);
  lv_obj_set_style_radius(back, 10, LV_PART_MAIN);
  lv_obj_add_event_cb(back, back_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *backLbl = lv_label_create(back);
  lv_label_set_text(backLbl, LV_SYMBOL_LEFT " Back");
  lv_obj_set_style_text_font(backLbl, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_center(backLbl);

  lv_scr_load(scrStatus);
}

// Refresh both screens' text. Runs on an LVGL timer so it cannot fight the redraw.
static void ui_tick(lv_timer_t *t) {
  LV_UNUSED(t);
  char buf[48];

  // live held-time = accumulated + the run in progress
  uint32_t xs = xrayHeldMs + (xraySinceMs ? millis() - xraySinceMs : 0);
  uint32_t ds = dsaHeldMs  + (dsaSinceMs  ? millis() - dsaSinceMs  : 0);
  snprintf(buf, sizeof(buf), "X-RAY %us", (unsigned)(xs / 1000));  lv_label_set_text(lblXray, buf);
  snprintf(buf, sizeof(buf), "DSA %us",   (unsigned)(ds / 1000));  lv_label_set_text(lblDsa, buf);
  snprintf(buf, sizeof(buf), "CAPTURE %u", captureCount);          lv_label_set_text(lblCapture, buf);

  if (lv_scr_act() == scrStatus) {
    lv_label_set_text(lblBleState, "BLE: advertising as " DEVICE_NAME);
    if (power.isBatteryConnect())
      snprintf(buf, sizeof(buf), "Batt: %d%%  %dmV", power.getBatteryPercent(),
               power.getBattVoltage());
    else
      snprintf(buf, sizeof(buf), "Batt: not detected");
    lv_label_set_text(lblBatt, buf);

    snprintf(buf, sizeof(buf), "I2C: %s%s", i2cSummary, hapticsOK ? "(LRA ok)" : "(no LRA)");
    lv_label_set_text(lblI2C, buf);

    if (touchCurrent > 0)
      snprintf(buf, sizeof(buf), "SoftPot: %u  (raw %d)", touchCurrent, lastSoftPotRaw);
    else
      snprintf(buf, sizeof(buf), "SoftPot: -- (raw %d)", lastSoftPotRaw);
    lv_label_set_text(lblSoftPot, buf);

    if (!autoSwitchDone) {
      uint32_t left = (AUTO_SWITCH_MS - (millis() - bootMs)) / 1000;
      snprintf(buf, sizeof(buf), "Pedals in %us...", (unsigned)left + 1);
      lv_label_set_text(lblCountdown, buf);
    } else {
      lv_label_set_text(lblCountdown, "");
    }
  }
}

// ---------------------------------------------------------------------------
// LVGL plumbing (copied from examples/arduino/06_LVGL_Arduino_v9)
// ---------------------------------------------------------------------------
uint32_t millis_cb(void) { return millis(); }

void my_disp_flush(lv_display_t *d, const lv_area_t *area, uint8_t *px_map) {
#ifndef DIRECT_RENDER_MODE
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
#endif
  lv_disp_flush_ready(d);
}

void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
  int32_t touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  int32_t touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
  if (FT3168->IIC_Interrupt_Flag == true) {
    FT3168->IIC_Interrupt_Flag = false;
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void rounder_event_cb(lv_event_t *e) {
  lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
  area->x1 = (area->x1 >> 1) << 1;
  area->y1 = (area->y1 >> 1) << 1;
  area->x2 = ((area->x2 >> 1) << 1) + 1;
  area->y2 = ((area->y2 >> 1) << 1) + 1;
}

// ---------------------------------------------------------------------------
void setup() {
  // UART0 (RXD/TXD = 44/43), NOT USB CDC — IO19 is the SoftPot now, so native USB is gone.
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== Touchscreen Pedal Panel (X-ray / DSA / Capture) ===");

  if (!gfx->begin()) Serial.println("[GFX] begin() failed!");
  gfx->fillScreen(RGB565_BLACK);

  Wire.begin(IIC_SDA, IIC_SCL);

  while (FT3168->begin() == false) {
    Serial.println("[TOUCH] FT3168 init failed — retrying");
    delay(2000);
  }
  Serial.println("[TOUCH] FT3168 ready");
  FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                 FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);

  i2cScan();

  // Haptics. Optional at runtime: a missing DRV2605L must not stop the panel working.
  hapticsOK = drv.begin(&Wire);
  if (hapticsOK) {
    drv.selectLibrary(6);                 // 6 = LRA library
    drv.useLRA();
    drv.setMode(DRV2605_MODE_INTTRIG);
    Serial.println("[HAPTIC] DRV2605L ready (LRA mode)");
  } else {
    Serial.println("[HAPTIC] DRV2605L NOT found at 0x5A — running without haptics");
  }

  // Battery telemetry (display rails are already on; the PMU is only read here).
  power.enableBattDetection();
  power.enableBattVoltageMeasure();
  power.enableSystemVoltageMeasure();

  // SoftPot on IO19 (ADC2). Internal pulldown so an untouched, floating wiper reads ~0.
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOFTPOT, ADC_11db);
  analogRead(PIN_SOFTPOT);                        // let the core configure the pin first
  gpio_pulldown_en((gpio_num_t)PIN_SOFTPOT);

  // Audio
  pinMode(PIN_AUDIO_PA, OUTPUT);
  digitalWrite(PIN_AUDIO_PA, HIGH);               // enable the speaker amplifier
  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_DIN, PIN_I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                 I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("[AUDIO] I2S init failed — running silent");
  } else if (es8311_codec_init() != ESP_OK) {
    Serial.println("[AUDIO] ES8311 init failed — running silent");
  } else {
    audioOK = true;
    Serial.println("[AUDIO] ES8311 ready");
  }

  // ---- LVGL ----
  lv_init();
  lv_tick_set_cb(millis_cb);

  screenWidth = gfx->width();
  screenHeight = gfx->height();
#ifdef DIRECT_RENDER_MODE
  bufSize = screenWidth * screenHeight;
#else
  bufSize = screenWidth * 50;
#endif
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  if (!disp_draw_buf) {
    Serial.println("[LVGL] draw buffer allocation FAILED — is PSRAM enabled?");
    return;
  }

  disp = lv_display_create(screenWidth, screenHeight);
  lv_display_set_flush_cb(disp, my_disp_flush);
#ifdef DIRECT_RENDER_MODE
  lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_DIRECT);
#else
  lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);
  lv_display_add_event_cb(disp, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);

  buildUI();
  lv_timer_create(ui_tick, 250, NULL);

  setupBLE();
  soundReady();                                   // "BLE is up" chime — see the note on top

  bootMs = millis();
  Serial.println("=== setup complete ===");
}

void loop() {
  lv_task_handler();

#ifdef DIRECT_RENDER_MODE
  gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif

  const uint32_t now = millis();

  static uint32_t lastSoftPotMs = 0;
  if (now - lastSoftPotMs >= 20) {                // 50 Hz, same cadence as the trackers
    lastSoftPotMs = now;
    readSoftPot();
  }

  // Status -> buttons once the panel has been up for AUTO_SWITCH_MS. There is no
  // connection to wait for (see the header note), so this is purely a timer.
  if (!autoSwitchDone && (now - bootMs >= AUTO_SWITCH_MS)) showButtons();

  delay(5);
}
