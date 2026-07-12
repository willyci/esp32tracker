// ESP32-S3 SuperMini foot pedal → BLE. Companion to the hand trackers (same service/packet).
//
// Two pedals exist, built from the SAME sketch — only IS_LEFT_FOOT differs:
//   LEFT  foot ("Left Foot Pedal",  button across GPIO12 & GPIO11) → toggles X-ray on/off
//   RIGHT foot ("Right Foot Pedal", button across GPIO9  & GPIO8)  → triggers an X-ray screen capture
//
// Both pedals just FLIP packet byte 31 on each confirmed press (exactly like the hand
// trackers' X-ray button). The MEANING lives in the consumer, keyed by the advertised
// name: a flip from "Left Foot Pedal" toggles X-ray, a flip from "Right Foot Pedal"
// fires one screen capture. (Dashboard/app must add these two names to their scan list.)
//
// Wiring (NO external resistors needed):
//   Momentary pedal switch across the two GPIOs. One GPIO is driven LOW and acts as the
//   button's ground; the other is read with the chip's INTERNAL pull-up (LOW = pressed).
//   Power: battery + → 5V pin, battery − → GND. NEVER battery on 5V while USB is plugged in.
//
// Libraries: NimBLE-Arduino 1.4.x (not 2.x).
// Board: "ESP32S3 Dev Module" (that's the right choice for the S3 SuperMini), core 2.0.17,
// USB CDC On Boot: Enabled, 115200 baud. The SuperMini's onboard WS2812 sits on GPIO48 —
// the same pin this board definition maps RGB_BUILTIN to, so the status LED just works.

#include <NimBLEDevice.h>

// ---- Which foot is this board? ----  1 = left (X-ray toggle), 0 = right (capture)
#define IS_LEFT_FOOT 1

#if IS_LEFT_FOOT
  #define DEVICE_NAME "Left Foot Pedal"
  #define FOOT_LABEL  "LEFT FOOT (X-ray on/off)"
  static constexpr int PIN_BTN     = 12;   // read with internal pull-up: LOW = pressed
  static constexpr int PIN_BTN_GND = 11;   // driven LOW = the button's ground
#else
  #define DEVICE_NAME "Right Foot Pedal"
  #define FOOT_LABEL  "RIGHT FOOT (X-ray capture)"
  static constexpr int PIN_BTN     = 9;
  static constexpr int PIN_BTN_GND = 8;
#endif

// ---- BLE identifiers — must match the trackers/app exactly ----
#define SERVICE_UUID      "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define ORIENTATION_UUID  "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F"

// ---- Same 32-byte wire format as the hand trackers (see ../../SPEC.md) ----
// A pedal has no IMU/SoftPot: quaternion stays identity, accel/touch stay 0.
// Only byte 31 (xrayOn) changes — it flips 0/1 on each pedal press.
struct __attribute__((packed)) OrientationPacket {
  float   w, x, y, z;     // identity quaternion (w = 1)
  float   ax, ay, az;     // 0
  uint8_t calib;          // 0 (no IMU)
  uint8_t touchStart;     // 0
  uint8_t touchCurrent;   // 0
  uint8_t xrayOn;         // flips on each press — the pedal's only live field
};
static_assert(sizeof(OrientationPacket) == 32, "packet must be 32 bytes");

static OrientationPacket pkt = { 1, 0, 0, 0,  0, 0, 0,  0, 0, 0, 0 };

NimBLECharacteristic* orientationChar = nullptr;
volatile bool deviceConnected = false;

static constexpr uint32_t NOTIFY_INTERVAL_MS = 20;   // 50 Hz, same cadence as the trackers
static uint32_t lastNotifyMs = 0;

// Pedal debounce — same sustained-40ms scheme as the trackers' X-ray button.
static bool     pedalPressed = false;   // debounced (confirmed) state
static int      pedalRaw     = HIGH;    // last raw pin reading
static uint32_t pedalRawMs   = 0;       // when the raw reading last changed

// Onboard LED (if the board has one): ON while a central is connected, brief blink on press.
static uint32_t ledBlinkUntil = 0;
static void setLed(bool on) {
#if defined(RGB_BUILTIN)
  neopixelWrite(RGB_BUILTIN, 0, on ? 16 : 0, 0);   // dim green
#elif defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

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

// Each confirmed press FLIPS byte 31. Only a level that stays stable >=40 ms counts,
// so cable/contact noise is ignored and one stomp registers exactly once.
void handlePedal(uint32_t now) {
  int raw = digitalRead(PIN_BTN);
  if (raw != pedalRaw) { pedalRaw = raw; pedalRawMs = now; }   // restart the stability timer
  bool stable = (now - pedalRawMs) >= 40;
  if (stable && pedalRaw == LOW && !pedalPressed) {            // confirmed press → flip
    pedalPressed = true;
    pkt.xrayOn ^= 1;
    ledBlinkUntil = now + 120;
    Serial.printf(">> PEDAL press -> bit %u\n", pkt.xrayOn);
  } else if (stable && pedalRaw == HIGH && pedalPressed) {     // confirmed release
    pedalPressed = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== ESP32-S3 Foot Pedal ===");
  Serial.println("Device: " DEVICE_NAME "  —  " FOOT_LABEL);

  // Pedal switch across PIN_BTN and PIN_BTN_GND: drive one LOW as its ground,
  // read the other with the internal pull-up. No external resistors.
  pinMode(PIN_BTN_GND, OUTPUT);
  digitalWrite(PIN_BTN_GND, LOW);
  pinMode(PIN_BTN, INPUT_PULLUP);

#if !defined(RGB_BUILTIN) && defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  setupBLE();
  Serial.println("=== setup complete ===");
}

void loop() {
  const uint32_t now = millis();

  handlePedal(now);

  if (now - lastNotifyMs >= NOTIFY_INTERVAL_MS) {
    lastNotifyMs = now;
    if (deviceConnected && orientationChar) {
      orientationChar->setValue(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
      orientationChar->notify();
    }
  }

  // LED: on while connected, with a short off-blink acknowledging each press.
  setLed(deviceConnected && now >= ledBlinkUntil);

  delay(5);   // same heat-friendly yield as the trackers
}
