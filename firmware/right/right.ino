// ESP32-C3 + BNO085 + OLED + SoftPot touch strip → BLE tracker.
//
// Full working tracker: fused orientation (BNO085) over BLE, an on-board status OLED that
// wakes for SCREEN_ON_MS on power-up / BOOT press then auto-offs, and a SoftPot linear
// strip that reports the touch START point and CURRENT point (for slide / advance gestures).
// Verbose serial logging + an I2C scanner are kept for easy bring-up.
//
// Wiring:
//   BNO085  VIN→3V3  GND→GND  SDA→GPIO0  SCL→GPIO1   (HARDWARE I2C, addr 0x4B)
//   OLED    VCC→3V3  GND→GND  SDA→GPIO5  SCL→GPIO21  (SOFTWARE I2C — its OWN pins)
//   SoftPot V+→3V3   GND→GND  wiper→GPIO3  + 100k from GPIO3→GND (pulldown)
//   The OLED is on bit-banged software I2C on separate pins because U8g2/U8x8 hardware
//   I2C would not coexist with the BNO on the shared GPIO0/1 bus.
//
// Libraries: Adafruit BNO08x, NimBLE-Arduino 1.4.x, U8g2 (U8x8 mode).
// Board: "ESP32C3 Dev Module", core 2.0.17, USB CDC On Boot: Enabled, 115200 baud.

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <NimBLEDevice.h>
#include <U8x8lib.h>   // u8x8 = text-only mode (no frame buffer) — fast over software I2C

// ---- I2C pins (BNO085 hardware bus) ----
static constexpr int PIN_SDA = 0;
static constexpr int PIN_SCL = 1;
static constexpr uint8_t BNO_ADDR  = 0x4B;   // this board scans at 0x4B; some use 0x4A
static constexpr uint8_t OLED_ADDR = 0x3C;   // most 0.96" modules; a few are 0x3D

// ---- OLED on its OWN software-I2C pins (NOT the BNO's hardware bus) ----
static constexpr int PIN_OLED_SDA = 5;
static constexpr int PIN_OLED_SCL = 21;   // free because Serial runs over USB CDC, not UART0

// ---- BOOT button (GPIO9): press to wake the OLED ----
static constexpr int PIN_BOOT = 9;

// ---- SoftPot linear strip: wiper on GPIO3 (ADC1), 100k pulldown to GND ----
static constexpr int PIN_SOFTPOT = 3;
static constexpr int SOFTPOT_NOTOUCH_RAW = 80;   // raw ADC below this = no touch

// ---- BLE identifiers — must match the app exactly ----
#define SERVICE_UUID      "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define ORIENTATION_UUID  "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F"

// ---- Which hand is this board? ----  1 = left, 0 = right
#define IS_LEFT_HAND 0

#if IS_LEFT_HAND
  #define DEVICE_NAME "Left Hand Tracker"
  #define HAND_LABEL  "LEFT"
#else
  #define DEVICE_NAME "Right Hand Tracker"
  #define HAND_LABEL  "RIGHT"
#endif

// ---- 32-byte wire format ----
struct __attribute__((packed)) OrientationPacket {
  float   w, x, y, z;     // quaternion (real, i, j, k)
  float   ax, ay, az;     // accel, m/s^2
  uint8_t calib;          // 0–3
  uint8_t touchStart;     // SoftPot position where the touch began (0 = no touch)
  uint8_t touchCurrent;   // current SoftPot position while touched (0 = no touch)
  uint8_t touchActive;    // 1 = finger down, 0 = released
};
static_assert(sizeof(OrientationPacket) == 32, "packet must be 32 bytes");

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// OLED on its own software-I2C bus. (If text is garbled, your module is an SH1106 —
// change SSD1306 → SH1106 below.)
U8X8_SSD1306_128X64_NONAME_SW_I2C display(
    /*clock SCL=*/ PIN_OLED_SCL, /*data SDA=*/ PIN_OLED_SDA, /*reset=*/ U8X8_PIN_NONE);
bool oledOK = false;

NimBLECharacteristic* orientationChar = nullptr;
volatile bool deviceConnected = false;

static OrientationPacket pkt = { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 };

static constexpr uint32_t REPORT_INTERVAL_US  = 20000;
static constexpr uint32_t NOTIFY_INTERVAL_MS  = 20;
static constexpr uint32_t DISPLAY_INTERVAL_MS = 1000;  // OLED redraw 1 Hz — glanceable status;
                                                       // real-time data is on the dashboard/BLE.
static constexpr uint32_t STATUS_LOG_INTERVAL_MS = 1000;
static uint32_t lastNotifyMs  = 0;
static uint32_t lastDisplayMs = 0;
static uint32_t lastLogMs     = 0;
static uint32_t lastDrawUs    = 0;   // how long the last OLED redraw took (microseconds)

// Screen shows for SCREEN_ON_MS after power-up or a BOOT press, then auto-offs.
static constexpr uint32_t SCREEN_ON_MS = 10000;   // milliseconds (10 s)
static bool     screenEnabled = true;
static uint32_t screenOnMs    = 0;     // millis() when the screen was last turned on
static int      lastBtnState  = HIGH;
static uint32_t lastBtnMs     = 0;

// ---------------------------------------------------------------------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* /*server*/) override {
    deviceConnected = true;
    Serial.println(">> central CONNECTED");
  }
  void onDisconnect(NimBLEServer* /*server*/) override {
    deviceConnected = false;
    Serial.println(">> central DISCONNECTED — re-advertising");
    NimBLEDevice::startAdvertising();
  }
};

// Scan the I2C bus and log every device that answers (BNO085 at 0x4B). The OLED is on
// software I2C, so it will NOT appear here — that's expected.
void i2cScan() {
  Serial.println("[I2C] scanning bus...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C]   device at 0x%02X%s\n", addr,
                    (addr == BNO_ADDR) ? "  <- BNO085" : "");
      found++;
    }
  }
  if (found == 0)
    Serial.println("[I2C]   NONE found — check SDA/SCL/3V3/GND wiring & common ground");
  else
    Serial.printf("[I2C] scan done: %u device(s)\n", found);
}

void enableReports() {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US))
    Serial.println("[BNO] failed to enable rotation vector report");
  if (!bno08x.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US))
    Serial.println("[BNO] failed to enable accelerometer report");
}

void setupBLE() {
  Serial.println("[BLE] init...");
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(SERVICE_UUID);
  orientationChar = service->createCharacteristic(ORIENTATION_UUID, NIMBLE_PROPERTY::NOTIFY);
  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  NimBLEAdvertisementData scanData;
  scanData.setName(DEVICE_NAME);
  adv->setScanResponseData(scanData);
  adv->start();
  Serial.println("[BLE] advertising as " DEVICE_NAME);
}

void displayMessage(const char* l1, const char* l2) {
  if (!oledOK) return;
  display.clear();
  display.drawString(0, 0, HAND_LABEL);
  display.drawString(0, 2, l1);
  display.drawString(0, 4, l2);
}

void displayData() {
  if (!oledOK) return;
  char buf[20];
  // Overwrite each line in place (padded with trailing spaces so old digits are erased).
  snprintf(buf, sizeof(buf), "%-5s %-4s C:%u", HAND_LABEL, deviceConnected ? "CONN" : "ADV", pkt.calib);
  display.drawString(0, 0, buf);
  snprintf(buf, sizeof(buf), "w %+.3f ", pkt.w);  display.drawString(0, 2, buf);
  snprintf(buf, sizeof(buf), "x %+.3f ", pkt.x);  display.drawString(0, 3, buf);
  snprintf(buf, sizeof(buf), "y %+.3f ", pkt.y);  display.drawString(0, 4, buf);
  snprintf(buf, sizeof(buf), "z %+.3f ", pkt.z);  display.drawString(0, 5, buf);
  if (pkt.touchActive)
    snprintf(buf, sizeof(buf), "St%3u Cu%3u", pkt.touchStart, pkt.touchCurrent);
  else
    snprintf(buf, sizeof(buf), "touch: --   ");
  display.drawString(0, 7, buf);
}

// SoftPot: capture the START position on touch-down, track the CURRENT position while
// touched, and reset BOTH to 0 on release. (Pulldown holds an untouched strip near 0.)
static float softpotEMA = 0;
void readSoftPot() {
  int raw = analogRead(PIN_SOFTPOT);
  if (raw < SOFTPOT_NOTOUCH_RAW) {                 // released → reset both points
    pkt.touchActive = 0; pkt.touchStart = 0; pkt.touchCurrent = 0;
    softpotEMA = 0;
    return;
  }
  uint8_t pos = (uint8_t)constrain(map(raw, SOFTPOT_NOTOUCH_RAW, 4095, 1, 255), 1, 255);
  if (!pkt.touchActive) {                          // touch-down edge → capture the start point
    pkt.touchActive = 1;
    pkt.touchStart = pos;
    softpotEMA = pos;
  } else {
    softpotEMA = 0.6f * softpotEMA + 0.4f * pos;   // smooth the moving current point
  }
  pkt.touchCurrent = (uint8_t)softpotEMA;
}

// BOOT button (GPIO9, active LOW): a press WAKES the OLED for SCREEN_ON_MS, then it auto-offs.
// Most of the time the screen is OFF, so the loop runs like the no-screen firmware (cool + smooth).
void handleScreenButton(uint32_t now) {
  int btn = digitalRead(PIN_BOOT);
  if (btn != lastBtnState && (now - lastBtnMs) > 50) {   // simple debounce
    lastBtnMs = now;
    lastBtnState = btn;
    if (btn == LOW) {                                    // pressed → wake the screen
      screenEnabled = true;
      screenOnMs = now;                                  // (re)start the on-timer
      Serial.println(">> SCREEN ON");
      displayData();
      lastDisplayMs = now;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== ESP32 Touch Tracker (BNO085 + OLED + SoftPot) ===");
  Serial.println("Device: " DEVICE_NAME);

  pinMode(PIN_BOOT, INPUT_PULLUP);   // BOOT button wakes the OLED

  // SoftPot ADC on GPIO3: 12-bit, full ~0–3.3 V range.
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOFTPOT, ADC_11db);

  // --- I2C bus (DEFAULT 100 kHz — do NOT raise to 400 kHz; that breaks the BNO here) ---
  Serial.printf("[I2C] Wire.begin(SDA=%d, SCL=%d) @ 100kHz\n", PIN_SDA, PIN_SCL);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);         // don't let a stuck I2C transaction hang forever
  delay(300);                  // let the BNO finish its power-up boot
  i2cScan();
  delay(50);                   // let the bus settle after rapid-fire probing

  // --- OLED ---
  Serial.println("[OLED] begin (u8x8 text mode, software I2C)...");
  display.setBusClock(400000); // software I2C: bit-bang as fast as the CPU allows
  display.begin();
  display.setFont(u8x8_font_chroma48medium8_r);   // compact 8x8 font (16 cols x 8 rows)
  oledOK = true;               // SW I2C has no presence check
  Serial.println("[OLED] begin done");
  displayMessage("Starting...", "");

  // --- BNO085 (cold-boot NACK: retry begin_I2C a few times) ---
  bool bnoReady = false;
  for (int attempt = 1; attempt <= 5 && !bnoReady; attempt++) {
    Serial.printf("[BNO] begin_I2C(0x%02X) attempt %d/5...\n", BNO_ADDR, attempt);
    if (bno08x.begin_I2C(BNO_ADDR, &Wire)) { bnoReady = true; break; }
    Serial.println("[BNO]   failed — retrying");
    delay(200);
  }
  if (!bnoReady) {
    Serial.println("[BNO] NOT FOUND — check wiring/address (0x4A vs 0x4B)");
    displayMessage("BNO NOT FOUND", "check wiring");
    while (true) delay(1000);
  }
  Serial.println("[BNO] ready");
  enableReports();

  setupBLE();
  displayMessage("BLE advertising", DEVICE_NAME);
  screenOnMs = millis();   // start the on-screen timer from power-up
  Serial.println("=== setup complete ===");
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.println("[BNO] reset — re-enabling reports");
    enableReports();
  }

  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        pkt.w = sensorValue.un.rotationVector.real;
        pkt.x = sensorValue.un.rotationVector.i;
        pkt.y = sensorValue.un.rotationVector.j;
        pkt.z = sensorValue.un.rotationVector.k;
        pkt.calib = sensorValue.status & 0x03;
        break;
      case SH2_ACCELEROMETER:
        pkt.ax = sensorValue.un.accelerometer.x;
        pkt.ay = sensorValue.un.accelerometer.y;
        pkt.az = sensorValue.un.accelerometer.z;
        break;
      default: break;
    }
  }

  const uint32_t now = millis();

  handleScreenButton(now);   // BOOT press wakes the OLED

  // Auto-off the screen SCREEN_ON_MS after it was last turned on.
  if (screenEnabled && (now - screenOnMs >= SCREEN_ON_MS)) {
    screenEnabled = false;
    display.clear();
    Serial.println(">> SCREEN OFF (auto)");
  }

  if (now - lastNotifyMs >= NOTIFY_INTERVAL_MS) {
    lastNotifyMs = now;
    readSoftPot();   // sample the strip at 50 Hz — start/current/active live in pkt
    if (deviceConnected && orientationChar) {
      orientationChar->setValue(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
      orientationChar->notify();
    }
  }

  // Only redraw when the screen is enabled — disabled = no I2C blocking = full BLE speed.
  if (screenEnabled && (now - lastDisplayMs >= DISPLAY_INTERVAL_MS)) {
    lastDisplayMs = now;
    uint32_t t0 = micros();
    displayData();
    lastDrawUs = micros() - t0;
  }

  // Heartbeat log once a second — only when the screen is on (keeps "screen off" minimal).
  if (screenEnabled && now - lastLogMs >= STATUS_LOG_INTERVAL_MS) {
    lastLogMs = now;
    Serial.printf("[status] %s oled=%s touch(a=%u s=%u c=%u) q=%.2f,%.2f,%.2f,%.2f calib=%u\n",
                  deviceConnected ? "CONN" : "ADV", oledOK ? "ok" : "--",
                  pkt.touchActive, pkt.touchStart, pkt.touchCurrent,
                  pkt.w, pkt.x, pkt.y, pkt.z, pkt.calib);
  }

  // Yield ~5 ms so the CPU isn't pinned at 100% — cuts heat. Harmless to the 50 Hz BLE
  // (20 ms period) and to BNO polling (its FIFO holds samples between polls).
  delay(5);
}
