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
//   X-ray btn: momentary button across GPIO6 & GPIO7 (GPIO7 driven LOW = its ground). Each click
//              toggles X-ray; the dashboard/app owns the single shared state (either hand flips it).
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

// ---- X-ray button: momentary button across GPIO6 & GPIO7 ----
static constexpr int PIN_XRAY_BTN = 6;   // read with pull-up: LOW = pressed
static constexpr int PIN_XRAY_GND = 7;   // driven LOW so the button can bridge 6 → 7

// ---- BLE identifiers — must match the app exactly ----
#define SERVICE_UUID      "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define ORIENTATION_UUID  "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define XRAY_STATE_UUID   "4F7A0003-9B3E-4C2A-8D1F-0A1B2C3D4E5F"  // app WRITES shared X-ray here

// ---- Which hand is this board? ----  1 = left, 0 = right
#define IS_LEFT_HAND 1

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
  uint8_t xrayOn;         // X-ray toggle: flips 0/1 on each button click. ("Touched" is derivable
                          // from touchCurrent > 0, so this byte replaced the old touchActive.)
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
static constexpr uint32_t DISPLAY_INTERVAL_MS = 500;   // OLED refresh 2 Hz — cheap now: labels
                                                       // draw once, only changed rows rewrite.
static constexpr uint32_t STATUS_LOG_INTERVAL_MS = 1000;
static uint32_t lastNotifyMs  = 0;
static uint32_t lastDisplayMs = 0;
static uint32_t lastLogMs     = 0;
static uint32_t lastDrawUs    = 0;   // how long the last OLED redraw took (microseconds)

// Screen shows for SCREEN_ON_MS after power-up or a BOOT press, then auto-offs.
// (0 = stay on forever — updates are cheap single-row writes either way.)
static constexpr uint32_t SCREEN_ON_MS = 10000;   // milliseconds (10 s)
static bool     screenEnabled = true;
static uint32_t screenOnMs    = 0;     // millis() when the screen was last turned on
static int      lastBtnState  = HIGH;
static uint32_t lastBtnMs     = 0;

// X-ray toggle button — sustained-LOW debounce rejects brief touch-noise glitches on GPIO6.
static bool     xrayState    = false;   // the toggle we report in the packet
static bool     xrayPressed  = false;   // debounced (confirmed) button state
static int      xrayRaw      = HIGH;    // last raw pin reading
static uint32_t xrayRawMs    = 0;       // when the raw reading last changed

// The TRUE shared X-ray state, written back by the app (the app owns it; our button only
// sends flip events). The OLED shows THIS, so all three boards always agree.
static volatile bool xraySynced = false;

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

// The app writes the shared X-ray state (1 byte, 0/1) whenever it changes — on any
// board's button, the foot pedal, or the app's own UI — and once at connect.
class XrayStateCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* ch) override {
    NimBLEAttValue v = ch->getValue();
    if (v.size() >= 1) {
      xraySynced = (v[0] != 0);
      Serial.printf(">> X-RAY state from app -> %s\n", xraySynced ? "ON" : "OFF");
    }
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
  NimBLECharacteristic* xrayStateChar =
      service->createCharacteristic(XRAY_STATE_UUID,
                                    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
  xrayStateChar->setCallbacks(new XrayStateCallbacks());
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

// Data layout: label rows draw ONCE, then each refresh rewrites only the value rows
// that changed. One 16-char row costs a few ms on software I2C vs ~10x that for the
// old full-screen redraw, so the 50 Hz BLE loop never hiccups even with the screen on.
//   row 0: <HAND> <CONN/ADV> C:n    (rewritten only when it changes)
//   row 2: w   x   y   z            (labels, static)
//   row 3: +.12-.45+.99-.01         (quaternion — the ONE row drawn every tick)
//   row 5: XRAY  St   Cu            (labels, static)
//   row 6: ON    123  200           (rewritten only when it changes)
static bool labelsDrawn = false;
static char prevRow0[17] = "";
static char prevRow6[17] = "";

// Format v (-1..1) as exactly 4 chars: "+.12", "-.98", "+1.0" — four fit in one row.
static void fmt4(float v, char* out) {
  char sign = (v < 0) ? '-' : '+';
  float a = fabsf(v);
  if (a >= 0.995f) snprintf(out, 5, "%c1.0", sign);
  else             snprintf(out, 5, "%c.%02d", sign, (int)roundf(a * 100.0f));
}

void displayMessage(const char* l1, const char* l2) {
  if (!oledOK) return;
  display.clear();
  display.drawString(0, 0, HAND_LABEL);
  display.drawString(0, 2, l1);
  display.drawString(0, 4, l2);
  labelsDrawn = false;   // full-screen message wiped the data layout
}

void displayData() {
  if (!oledOK) return;
  if (!labelsDrawn) {                        // first draw after a clear: labels once
    display.clear();
    display.drawString(0, 2, "w   x   y   z");
    display.drawString(0, 5, "XRAY  St   Cu");
    prevRow0[0] = prevRow6[0] = '\0';
    labelsDrawn = true;
  }
  char row[17];

  snprintf(row, sizeof(row), "%-5s %-4s C:%u",
           HAND_LABEL, deviceConnected ? "CONN" : "ADV", pkt.calib);
  if (strcmp(row, prevRow0) != 0) { display.drawString(0, 0, row); strcpy(prevRow0, row); }

  char qw[5], qx[5], qy[5], qz[5];           // quaternion — changes every tick
  fmt4(pkt.w, qw); fmt4(pkt.x, qx); fmt4(pkt.y, qy); fmt4(pkt.z, qz);
  snprintf(row, sizeof(row), "%s%s%s%s", qw, qx, qy, qz);
  display.drawString(0, 3, row);

  if (pkt.touchCurrent > 0)
    snprintf(row, sizeof(row), "%-4s  %3u  %3u",
             xraySynced ? "ON" : "OFF", pkt.touchStart, pkt.touchCurrent);
  else
    snprintf(row, sizeof(row), "%-4s   --   --", xraySynced ? "ON" : "OFF");
  if (strcmp(row, prevRow6) != 0) { display.drawString(0, 6, row); strcpy(prevRow6, row); }
}

// SoftPot: capture the START position on touch-down, track the CURRENT position while
// touched, and reset BOTH to 0 on release. (Pulldown holds an untouched strip near 0.)
static float softpotEMA = 0;
static bool  touching = false;   // internal touch flag (packet no longer carries touchActive)
void readSoftPot() {
  int raw = analogRead(PIN_SOFTPOT);
  if (raw < SOFTPOT_NOTOUCH_RAW) {                 // released → reset both points
    touching = false; pkt.touchStart = 0; pkt.touchCurrent = 0;
    softpotEMA = 0;
    return;
  }
  uint8_t pos = (uint8_t)constrain(map(raw, SOFTPOT_NOTOUCH_RAW, 4095, 1, 255), 1, 255);
  if (!touching) {                                 // touch-down edge → capture the start point
    touching = true;
    pkt.touchStart = pos;
    softpotEMA = pos;
  } else {
    softpotEMA = 0.6f * softpotEMA + 0.4f * pos;   // smooth the moving current point
  }
  pkt.touchCurrent = (uint8_t)softpotEMA;
}

// BOOT button (GPIO9, active LOW): each press TOGGLES the OLED on/off. Updates are
// single-row now, so the screen can stay on; turn it off to save battery if you like.
void handleScreenButton(uint32_t now) {
  int btn = digitalRead(PIN_BOOT);
  if (btn != lastBtnState && (now - lastBtnMs) > 50) {   // simple debounce
    lastBtnMs = now;
    lastBtnState = btn;
    if (btn == LOW) {                                    // pressed → toggle the screen
      screenEnabled = !screenEnabled;
      Serial.println(screenEnabled ? ">> SCREEN ON" : ">> SCREEN OFF");
      if (screenEnabled) {
        screenOnMs = now;                                // (re)start the auto-off timer (if used)
        displayData();
        lastDisplayMs = now;
      } else {
        display.clear();
        labelsDrawn = false;
      }
    }
  }
}

// X-ray button (across GPIO6/GPIO7): each click TOGGLES X-ray. GPIO7 is driven LOW to act as the
// button's ground; GPIO6 is read with a pull-up (LOW = pressed). This board just flips its own
// xrayOn flag; the dashboard/app owns the single shared X-ray and flips it whenever EITHER hand's
// flag changes — so both hands' buttons do the same thing.
void handleXrayButton(uint32_t now) {
  int raw = digitalRead(PIN_XRAY_BTN);
  if (raw != xrayRaw) { xrayRaw = raw; xrayRawMs = now; }   // raw changed → restart the stability timer
  // Only accept the level after it's been STABLE for >=40 ms. A brief touch-noise spike never
  // lasts that long, so it's ignored; a real press (held far longer) is counted once.
  bool stable = (now - xrayRawMs) >= 40;
  if (stable && xrayRaw == LOW && !xrayPressed) {          // confirmed press → toggle
    xrayPressed = true;
    xrayState = !xrayState;
    pkt.xrayOn = xrayState ? 1 : 0;
    Serial.printf(">> X-RAY toggle -> %s\n", xrayState ? "ON" : "OFF");
  } else if (stable && xrayRaw == HIGH && xrayPressed) {   // confirmed release
    xrayPressed = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== ESP32 Touch Tracker (BNO085 + OLED + SoftPot) ===");
  Serial.println("Device: " DEVICE_NAME);

  pinMode(PIN_BOOT, INPUT_PULLUP);   // BOOT button wakes the OLED

  // X-ray button across GPIO6/GPIO7: drive GPIO7 LOW as its ground, read GPIO6 with a pull-up.
  pinMode(PIN_XRAY_GND, OUTPUT);
  digitalWrite(PIN_XRAY_GND, LOW);
  pinMode(PIN_XRAY_BTN, INPUT_PULLUP);

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
  handleXrayButton(now);     // GPIO6/7 button toggles X-ray

  // Auto-off the screen SCREEN_ON_MS after it was last turned on (0 = never).
  if (SCREEN_ON_MS > 0 && screenEnabled && (now - screenOnMs >= SCREEN_ON_MS)) {
    screenEnabled = false;
    display.clear();
    labelsDrawn = false;
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
    Serial.printf("[status] %s oled=%s draw=%luus xbit=%u xsync=%u touch(s=%u c=%u) q=%.2f,%.2f,%.2f,%.2f calib=%u\n",
                  deviceConnected ? "CONN" : "ADV", oledOK ? "ok" : "--",
                  (unsigned long)lastDrawUs, pkt.xrayOn, (unsigned)xraySynced,
                  pkt.touchStart, pkt.touchCurrent,
                  pkt.w, pkt.x, pkt.y, pkt.z, pkt.calib);
  }

  // Yield ~5 ms so the CPU isn't pinned at 100% — cuts heat. Harmless to the 50 Hz BLE
  // (20 ms period) and to BNO polling (its FIFO holds samples between polls).
  delay(5);
}
