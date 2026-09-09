// ===========================================================================
// PANEL SELF-TEST — instrumented hardware diagnostic (no LVGL, no BLE)
//
// WHY THIS EXISTS: 01_HelloWorld draws fine on this board, but the same display
// setup inside a larger sketch showed a blank screen. That rules out the panel,
// the board settings and the core — so something between gfx->begin() and the
// first draw is hanging or resetting the board.
//
// With IO19 taken by the SoftPot there is no USB serial, so the GLASS is the only
// output channel. Every init step therefore prints a breadcrumb to the screen
// BEFORE it runs the risky call. Whatever line is LAST on screen names the call
// that did not return.
//
// Each step is also individually switchable below: turn one off to prove it was
// the culprit, without editing any logic.
//
// Board: "ESP32S3 Dev Module", core 3.3.11, PSRAM enabled, 115200 baud.
// Serial works over USB only if the SoftPot is unplugged from IO19 AND
// "USB CDC On Boot" is Enabled; otherwise it is UART0 on RXD/TXD (44/43).
// ===========================================================================

#include <Arduino.h>
#include <Wire.h>
#include "pin_config.h"
#include "Arduino_GFX_Library.h"

// ---- step switches: set any to 0 to skip that peripheral entirely ----------
#define TEST_I2CSCAN 1
#define TEST_PMU     1
#define TEST_TOUCH   1
#define TEST_HAPTIC  1
#define TEST_AUDIO   1     // suspect #1: the ES8311 path uses the IDF I2C driver
#define TEST_SOFTPOT 1

#if TEST_TOUCH
#include "Arduino_DriveBus_Library.h"
#endif
#if TEST_PMU
#include "XPowersLib.h"
#endif
#if TEST_HAPTIC
#include <Adafruit_DRV2605.h>
#endif
#if TEST_AUDIO
#include "ESP_I2S.h"
#include "esp_check.h"
#include "es8311.h"
#endif
#include "driver/gpio.h"

static constexpr int PIN_SOFTPOT = 19;            // ADC2_CH8 (USB D-)
static constexpr int SOFTPOT_NOTOUCH_RAW = 80;

static constexpr int PIN_I2S_BCLK = 41;
static constexpr int PIN_I2S_WS   = 45;
static constexpr int PIN_I2S_DOUT = 40;
static constexpr int PIN_I2S_DIN  = 42;
static constexpr int PIN_I2S_MCLK = 16;
static constexpr int PIN_AUDIO_PA = 46;

static constexpr uint32_t SAMPLE_RATE  = 16000;
static constexpr int      VOICE_VOLUME = 85;
static constexpr int      I2C_NUM      = 0;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RESET, 0 /* rotation */,
                                      LCD_WIDTH, LCD_HEIGHT,
                                      22 /* col_offset1 */, 0, 0, 0);

#if TEST_TOUCH
std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
void Arduino_IIC_Touch_Interrupt(void);
std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, TP_INT,
                                                       Arduino_IIC_Touch_Interrupt));
void Arduino_IIC_Touch_Interrupt(void) { FT3168->IIC_Interrupt_Flag = true; }
#endif
#if TEST_PMU
XPowersPMU power;
#endif
#if TEST_HAPTIC
Adafruit_DRV2605 drv;
#endif
#if TEST_AUDIO
I2SClass i2s;
#endif

bool touchOK = false, hapticsOK = false, audioOK = false, pmuOK = false;
char i2cSummary[48] = "not scanned";
uint32_t tapCount = 0;
bool lastTouchState = false;
int lastPos = -1, lastRaw = -1;

// ---------------------------------------------------------------------------
// The debugger: a line on the glass, written BEFORE the risky call it names.
// ---------------------------------------------------------------------------
static int noteY = 0;

void bootNote(const char *msg, uint16_t colour = RGB565_WHITE) {
  Serial.printf("[BOOT] %s\n", msg);
  if (noteY > 470) {                 // wrap rather than run off the bottom
    gfx->fillRect(0, 70, LCD_WIDTH, LCD_HEIGHT - 70, RGB565_BLACK);
    noteY = 70;
  }
  gfx->setTextSize(2);
  gfx->setTextColor(colour, RGB565_BLACK);
  gfx->setCursor(14, noteY);
  gfx->print(msg);
  noteY += 24;
  delay(30);                         // give the QSPI push time to land
}

#if TEST_AUDIO
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
                      "selftest", "es8311 init failed");
  ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency,
                                                     es_clk.sample_frequency),
                      "selftest", "es8311 sample rate failed");
  ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, VOICE_VOLUME, NULL),
                      "selftest", "es8311 volume failed");
  es8311_microphone_config(es_handle, false);
  return ESP_OK;
}

void playBeep(uint16_t freqHz = 1600, uint16_t ms = 35) {
  if (!audioOK) return;
  const uint32_t total = (uint32_t)SAMPLE_RATE * ms / 1000;
  static int16_t buf[256 * 2];
  float phase = 0.0f;
  const float step = 2.0f * PI * freqHz / (float)SAMPLE_RATE;
  uint32_t done = 0;
  while (done < total) {
    uint32_t remain = total - done;
    uint32_t n = (remain < 256) ? remain : 256;
    for (uint32_t i = 0; i < n; i++) {
      float env = 1.0f;
      uint32_t pos = done + i;
      const uint32_t fade = SAMPLE_RATE / 200;      // ~5 ms
      if (pos < fade) env = (float)pos / fade;
      else if (total - pos < fade) env = (float)(total - pos) / fade;
      int16_t sv = (int16_t)(sinf(phase) * 32767.0f * 0.45f * env);
      buf[i * 2] = sv; buf[i * 2 + 1] = sv;
      phase += step;
      if (phase > 2.0f * PI) phase -= 2.0f * PI;
    }
    i2s.write((uint8_t *)buf, n * 2 * sizeof(int16_t));
    done += n;
  }
}
#else
void playBeep(uint16_t = 0, uint16_t = 0) {}
#endif

#if TEST_HAPTIC
void triggerHaptic(uint8_t effect = 1) {
  if (!hapticsOK) return;
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);
  drv.go();
}
#else
void triggerHaptic(uint8_t = 1) {}
#endif

#if TEST_I2CSCAN
void scanI2C() {
  size_t used = 0;
  i2cSummary[0] = 0;
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (used < sizeof(i2cSummary) - 8)
        used += snprintf(i2cSummary + used, sizeof(i2cSummary) - used, "%02X ", addr);
      found++;
    }
  }
  if (found == 0) snprintf(i2cSummary, sizeof(i2cSummary), "None found");
}
#endif

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);                        // let the rails settle
  Serial.println("\n=== Panel self-test ===");

  // Display FIRST and alone — 01_HelloWorld proves this much works.
  if (!gfx->begin()) Serial.println("[GFX] begin() failed!");
  gfx->fillScreen(RGB565_BLACK);

  gfx->setTextColor(RGB565_GREEN, RGB565_BLACK);
  gfx->setTextSize(3);
  gfx->setCursor(20, 18);
  gfx->print("PANEL SELFTEST");
  noteY = 70;

  bootNote("gfx ok", RGB565_GREEN);

#if TEST_I2CSCAN
  bootNote("Wire.begin...");
  Wire.begin(IIC_SDA, IIC_SCL);
  bootNote("i2c scan...");
  scanI2C();
  bootNote(i2cSummary, RGB565_CYAN);
#endif

#if TEST_PMU
  // NOTE: the working pedal-panel sketch never called power.begin() — it relied on
  // the rails already being up and only READ the PMU. This call also re-runs
  // Wire.begin() internally, so it is a prime suspect.
  bootNote("pmu begin...");
  pmuOK = power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);
  bootNote(pmuOK ? "pmu ok" : "pmu FAILED", pmuOK ? RGB565_GREEN : RGB565_RED);
  if (pmuOK) {
    power.enableBattDetection();
    power.enableBattVoltageMeasure();
    power.enableSystemVoltageMeasure();
    power.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);   // 4.2 V LiPo
  }
#endif

#if TEST_TOUCH
  bootNote("touch begin...");
  for (int attempt = 1; attempt <= 3 && !touchOK; attempt++) {
    touchOK = FT3168->begin();
    if (!touchOK) delay(200);
  }
  bootNote(touchOK ? "touch ok" : "touch FAILED", touchOK ? RGB565_GREEN : RGB565_RED);
  if (touchOK)
    FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                   FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);
#endif

#if TEST_HAPTIC
  bootNote("drv2605 begin...");
  hapticsOK = drv.begin(&Wire);
  bootNote(hapticsOK ? "haptic ok" : "haptic FAILED", hapticsOK ? RGB565_GREEN : RGB565_RED);
  if (hapticsOK) {
    drv.selectLibrary(6);            // 6 = LRA effect library
    drv.useLRA();
    drv.setMode(DRV2605_MODE_INTTRIG);
  }
#endif

#if TEST_AUDIO
  // Suspect #1. es8311_create() drives I2C port 0 through the IDF driver while Wire
  // already owns that port; i2s.begin() also claims DMA and clocks.
  bootNote("audio PA...");
  pinMode(PIN_AUDIO_PA, OUTPUT);
  digitalWrite(PIN_AUDIO_PA, HIGH);
  bootNote("i2s begin...");
  if (i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_DIN, PIN_I2S_MCLK),
      i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    bootNote("es8311 init...");
    audioOK = (es8311_codec_init() == ESP_OK);
  }
  bootNote(audioOK ? "audio ok" : "audio FAILED", audioOK ? RGB565_GREEN : RGB565_RED);
#endif

#if TEST_SOFTPOT
  bootNote("softpot...");
  gpio_reset_pin((gpio_num_t)PIN_SOFTPOT);     // take IO19 back from the USB PHY
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOFTPOT, ADC_11db);
  analogRead(PIN_SOFTPOT);                     // let the core configure the pin first
  gpio_pulldown_en((gpio_num_t)PIN_SOFTPOT);
  bootNote("softpot ok", RGB565_GREEN);
#endif

  bootNote("SETUP COMPLETE", RGB565_YELLOW);
  delay(1500);

  // ---- hand over to the live view ----
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextColor(RGB565_GREEN, RGB565_BLACK);
  gfx->setTextSize(3);
  gfx->setCursor(25, 20);
  gfx->print("HARDWARE TEST");

  gfx->setTextColor(RGB565_CYAN, RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(20, 75);  gfx->print("[I2C]");
  gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  gfx->setCursor(90, 75);  gfx->print(i2cSummary);

  gfx->setTextColor(RGB565_CYAN, RGB565_BLACK);
  gfx->setCursor(20, 110); gfx->print("[FEEDBACK]");
  gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  gfx->setCursor(20, 135);
  gfx->printf("Spk: %s   Haptic: %s", audioOK ? "OK" : "NO", hapticsOK ? "OK" : "NO");

  gfx->setTextColor(RGB565_CYAN, RGB565_BLACK);
  gfx->setCursor(20, 175); gfx->print("[BATTERY / POWER]");
  gfx->setCursor(20, 240); gfx->print("[TOUCH SCREEN]");
  gfx->setCursor(20, 335); gfx->print("[SOFTPOT STRIP]");

  gfx->drawRect(20, 420, 370, 26, RGB565_WHITE);

  gfx->setTextColor(RGB565_YELLOW, RGB565_BLACK);
  gfx->setCursor(20, 465);
  gfx->print("Tap screen: Beep + Vibrate");

  playBeep(2000, 50);
  triggerHaptic(1);
}

// ---------------------------------------------------------------------------
void loop() {
  int32_t touchX = -1, touchY = -1;
  bool isTouched = false;

#if TEST_TOUCH
  if (touchOK) {
    touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
    touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
    // The FT3168 interrupt flag is an EDGE latch, not a level. Holding it for a short
    // window turns it into something the UI can actually show; reading the flag alone
    // makes PRESSED flicker for a single pass and vanish.
    static uint32_t heldUntil = 0;
    if (FT3168->IIC_Interrupt_Flag) {
      FT3168->IIC_Interrupt_Flag = false;
      heldUntil = millis() + 150;
    }
    isTouched = (int32_t)(millis() - heldUntil) < 0;
  }
#endif

  if (isTouched && !lastTouchState) {
    tapCount++;
    playBeep(1800, 30);
    triggerHaptic(1);
  }
  lastTouchState = isTouched;

  int raw = 0, pos = 0;
#if TEST_SOFTPOT
  raw = analogRead(PIN_SOFTPOT);
  if (raw > SOFTPOT_NOTOUCH_RAW)
    pos = (int)constrain(map(raw, SOFTPOT_NOTOUCH_RAW, 4095, 1, 255), 1, 255);
#endif

  static uint32_t lastRefreshMs = 0;
  if (millis() - lastRefreshMs >= 40) {                 // ~25 Hz
    lastRefreshMs = millis();

    gfx->setTextSize(2);
    gfx->setCursor(20, 200);
#if TEST_PMU
    if (pmuOK && power.isBatteryConnect()) {
      gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      const char *chg = power.isCharging() ? "[CHARGING]"
                      : power.isVbusIn()   ? "[USB FULL]" : "[ON BATT]";
      gfx->printf("%dmV  %d%%  %-10s    ",
                  power.getBattVoltage(), power.getBatteryPercent(), chg);
    } else {
      gfx->setTextColor(RGB565_YELLOW, RGB565_BLACK);
      gfx->print("No battery detected        ");
    }
#else
    gfx->setTextColor(RGB565_DARKGREY, RGB565_BLACK);
    gfx->print("PMU test disabled          ");
#endif

    gfx->setCursor(20, 265);
    if (isTouched) {
      gfx->setTextColor(RGB565_GREEN, RGB565_BLACK);
      gfx->printf("Status: PRESSED   Taps: %-4u ", tapCount);
      gfx->setCursor(20, 290);
      gfx->printf("X: %-4d   Y: %-4d           ", (int)touchX, (int)touchY);
    } else {
      gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      gfx->printf("Status: %-9s Taps: %-4u ", touchOK ? "RELEASED" : "NO TOUCH", tapCount);
      gfx->setCursor(20, 290);
      if (tapCount > 0) gfx->printf("Last X: %-4d  Y: %-4d       ", (int)touchX, (int)touchY);
      else              gfx->print("Touch anywhere on glass... ");
    }

    if (raw != lastRaw || pos != lastPos) {
      lastRaw = raw;
      lastPos = pos;

      gfx->setCursor(20, 360);
      if (pos > 0) {
        gfx->setTextColor(RGB565_GREEN, RGB565_BLACK);
        gfx->printf("State: TOUCHED  Pos: %3d/255   ", pos);
      } else {
        gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
        gfx->print("State: IDLE (No Touch)       ");
      }

      gfx->setCursor(20, 385);
      gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
      gfx->printf("Raw ADC: %-4d (Pin IO19)     ", raw);

      int barWidth = (pos * 366) / 255;
      if (barWidth > 0)   gfx->fillRect(22, 422, barWidth, 22, RGB565_GREEN);
      if (barWidth < 366) gfx->fillRect(22 + barWidth, 422, 366 - barWidth, 22, RGB565_BLACK);
    }
  }

  delay(5);
}
