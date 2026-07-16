// ESP32-C3 0.42" OLED board → BLE "Mini" hand tracker (no IMU, no external screen).
//
// A simplified glove unit: SoftPot touch strip (grab/twist) + TWO buttons (X-ray on/off,
// X-ray screen capture), with status on the board's own 72x40 OLED. No BNO085 — the
// quaternion in the packet stays identity; orientation comes from the headset's own
// hand tracking. Two boards from the SAME sketch — only IS_LEFT_HAND differs.
//
// Pins (this is the C3 board with the 0.42" OLED, ceramic antenna, Type-C):
//   SoftPot   V+→3V3  GND→GND  wiper→GPIO0 (ADC). Internal pulldown is enabled so no
//             external resistor is needed; if "no touch" readings jitter, add the same
//             100k wiper→GND the big trackers use.
//   X-ray button   across GPIO8 (sense) & GPIO7 (ground). GPIO8 is a STRAPPING pin, so it
//                  MUST be the INPUT_PULLUP sense pin — it then idles HIGH, which is what the
//                  chip needs at boot. GPIO7 is the driven-LOW ground (and isn't actually
//                  driven until firmware runs, so even holding the button at power-up is safe).
//                  Do NOT swap the two, or a floating GPIO8 can stop the board booting.
//   Capture button across GPIO3 & GPIO4 — GPIO4 driven LOW as the button's ground.
//   Onboard OLED   72x40 SSD1306, hardware I2C SDA=GPIO5 SCL=GPIO6 (fine here — the old
//                  U8g2-vs-BNO hardware-I2C conflict only existed because of the BNO).
//   DO NOT USE: GPIO2 (strapping — a floating wiper/ground here can stop the boot),
//               GPIO9 (onboard BOOT button), GPIO5/6 (the OLED). GPIO8 is a strapping pin
//               too, but is used SAFELY as the X-ray sense pin (see above). RX/TX are free.
//   Power: battery + → 5V pin, battery − → GND. NEVER battery on 5V while USB is plugged in.
//
// Packet: same 32 bytes as every other board. Byte 31 flips on each X-ray button press
// (= toggle shared X-ray). The CAPTURE button flips byte 28 — the calib slot, meaningless
// without an IMU — and the dashboard fires one screen capture per flip.
//
// Libraries: NimBLE-Arduino 1.4.x (not 2.x), U8g2 (needs a version with the 72X40_ER
// constructor, v2.34+ — update U8g2 if the constructor is not found).
// Board: "ESP32C3 Dev Module", core 2.0.17, USB CDC On Boot: Enabled, 115200 baud.

#include <NimBLEDevice.h>
#include <U8g2lib.h>
#include "driver/gpio.h"   // gpio_pulldown_en — keeps the SoftPot pin from floating

// ---- Which hand is this board? ----  1 = left, 0 = right
#define IS_LEFT_HAND 0

#if IS_LEFT_HAND
  #define DEVICE_NAME "Left Mini Tracker"
  #define HAND_LABEL  "LEFT"
#else
  #define DEVICE_NAME "Right Mini Tracker"
  #define HAND_LABEL  "RIGHT"
#endif

// ---- Pins ----
static constexpr int PIN_SOFTPOT  = 0;    // ADC1_CH0, internal pulldown enabled below
static constexpr int PIN_XRAY_BTN = 8;    // SENSE — INPUT_PULLUP idles HIGH (boot-safe: GPIO8 is strapping)
static constexpr int PIN_XRAY_GND = 7;    // driven LOW = the button's ground
static constexpr int PIN_CAP_BTN  = 3;    // capture button sense
static constexpr int PIN_CAP_GND  = 4;    // capture button ground
static constexpr int PIN_OLED_SDA = 5;    // onboard OLED (hardware I2C)
static constexpr int PIN_OLED_SCL = 6;

static constexpr int SOFTPOT_NOTOUCH_RAW = 80;   // raw ADC below this = no touch

// ---- BLE identifiers — must match the trackers/app exactly ----
#define SERVICE_UUID      "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define ORIENTATION_UUID  "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F"

// ---- Same 32-byte wire format as every other board (see ../../SPEC.md) ----
struct __attribute__((packed)) OrientationPacket {
  float   w, x, y, z;     // identity quaternion (no IMU on this board)
  float   ax, ay, az;     // 0
  uint8_t calib;          // REPURPOSED on Mini boards: capture bit, flips per capture press
  uint8_t touchStart;     // SoftPot position where the touch began (0 = no touch)
  uint8_t touchCurrent;   // current SoftPot position while touched (0 = no touch)
  uint8_t xrayOn;         // flips 0/1 on each X-ray button press
};
static_assert(sizeof(OrientationPacket) == 32, "packet must be 32 bytes");

static OrientationPacket pkt = { 1, 0, 0, 0,  0, 0, 0,  0, 0, 0, 0 };

// Onboard 72x40 OLED, hardware I2C on its dedicated pins.
U8G2_SSD1306_72X40_ER_F_HW_I2C display(U8G2_R0, /*reset=*/U8X8_PIN_NONE,
                                       /*clock=*/PIN_OLED_SCL, /*data=*/PIN_OLED_SDA);

NimBLECharacteristic* orientationChar = nullptr;
volatile bool deviceConnected = false;

static constexpr uint32_t NOTIFY_INTERVAL_MS  = 20;    // 50 Hz, same as the big trackers
static constexpr uint32_t DISPLAY_INTERVAL_MS = 500;   // OLED refresh 2 Hz
static uint32_t lastNotifyMs  = 0;
static uint32_t lastDisplayMs = 0;

// One debouncer per button — the same sustained-40ms scheme as the big trackers.
struct Debounce {
  bool     pressed = false;
  int      raw     = HIGH;
  uint32_t rawMs   = 0;
  // Returns true exactly once per confirmed press.
  bool clicked(int pin, uint32_t now) {
    int r = digitalRead(pin);
    if (r != raw) { raw = r; rawMs = now; }
    bool stable = (now - rawMs) >= 40;
    if (stable && raw == LOW && !pressed) { pressed = true; return true; }
    if (stable && raw == HIGH && pressed) { pressed = false; }
    return false;
  }
};
static Debounce xrayBtn, capBtn;
static bool     xrayState    = false;
static uint32_t captureCount = 0;   // local tally, shown on the OLED

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

// SoftPot: capture the START position on touch-down, track CURRENT while touched,
// reset both on release — identical behavior to the big trackers.
static float softpotEMA = 0;
static bool  touching   = false;
void readSoftPot() {
  int raw = analogRead(PIN_SOFTPOT);
  if (raw < SOFTPOT_NOTOUCH_RAW) {
    touching = false; pkt.touchStart = 0; pkt.touchCurrent = 0;
    softpotEMA = 0;
    return;
  }
  uint8_t pos = (uint8_t)constrain(map(raw, SOFTPOT_NOTOUCH_RAW, 4095, 1, 255), 1, 255);
  if (!touching) {
    touching = true;
    pkt.touchStart = pos;
    softpotEMA = pos;
  } else {
    softpotEMA = 0.6f * softpotEMA + 0.4f * pos;
  }
  pkt.touchCurrent = (uint8_t)softpotEMA;
}

void drawOLED() {
  char buf[16];
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);   // 72px wide = 12 chars, 40px = 4 lines
  snprintf(buf, sizeof(buf), "%s %s", HAND_LABEL, deviceConnected ? "CONN" : "ADV");
  display.drawStr(0, 9, buf);
  snprintf(buf, sizeof(buf), "XRAY %s", xrayState ? "ON" : "OFF");
  display.drawStr(0, 19, buf);
  snprintf(buf, sizeof(buf), "CAP  %lu", (unsigned long)captureCount);
  display.drawStr(0, 29, buf);
  if (pkt.touchCurrent > 0)
    snprintf(buf, sizeof(buf), "TCH  %u", pkt.touchCurrent);
  else
    snprintf(buf, sizeof(buf), "TCH  --");
  display.drawStr(0, 39, buf);
  display.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== ESP32-C3 Mini Tracker (SoftPot + 2 buttons, no IMU) ===");
  Serial.println("Device: " DEVICE_NAME);

  // Buttons: drive one pin of each pair LOW as its ground, read the other with the
  // internal pull-up. No external resistors.
  pinMode(PIN_XRAY_GND, OUTPUT); digitalWrite(PIN_XRAY_GND, LOW);
  pinMode(PIN_XRAY_BTN, INPUT_PULLUP);
  pinMode(PIN_CAP_GND,  OUTPUT); digitalWrite(PIN_CAP_GND, LOW);
  pinMode(PIN_CAP_BTN,  INPUT_PULLUP);

  // SoftPot ADC on GPIO0: 12-bit, full ~0–3.3 V range, with the INTERNAL (~45k)
  // pulldown enabled so an untouched (floating) wiper reads near 0 — no external
  // resistor. The pad pull stays active alongside the ADC.
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOFTPOT, ADC_11db);
  analogRead(PIN_SOFTPOT);                       // let the core configure the pin first
  gpio_pulldown_en((gpio_num_t)PIN_SOFTPOT);     // then latch the pulldown on

  Serial.println("[OLED] begin (72x40, hardware I2C SDA=5 SCL=6)...");
  display.begin();
  drawOLED();

  setupBLE();
  Serial.println("=== setup complete ===");
}

void loop() {
  const uint32_t now = millis();

  if (xrayBtn.clicked(PIN_XRAY_BTN, now)) {      // X-ray button → flip byte 31
    xrayState = !xrayState;
    pkt.xrayOn ^= 1;
    Serial.printf(">> X-RAY toggle -> %s\n", xrayState ? "ON" : "OFF");
  }
  if (capBtn.clicked(PIN_CAP_BTN, now)) {        // capture button → flip byte 28 (calib slot)
    pkt.calib ^= 1;
    captureCount++;
    Serial.printf(">> X-RAY CAPTURE #%lu\n", (unsigned long)captureCount);
  }

  if (now - lastNotifyMs >= NOTIFY_INTERVAL_MS) {
    lastNotifyMs = now;
    readSoftPot();                               // 50 Hz, same cadence as the big trackers
    if (deviceConnected && orientationChar) {
      orientationChar->setValue(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
      orientationChar->notify();
    }
  }

  if (now - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = now;
    drawOLED();
  }

  delay(5);   // same heat-friendly yield as the other boards
}
