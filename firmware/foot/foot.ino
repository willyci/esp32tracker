// ESP32-C3 + foot pedal switch → BLE X-ray toggle pedal.
//
// A third device in the tracker family: no IMU, no OLED, no SoftPot — just a foot pedal
// that toggles the shared X-ray state in the dashboard / Vision Pro app. It speaks the
// exact same 32-byte packet as the hand trackers (identity quaternion, zero accel/touch)
// and FLIPS the xrayOn bit on each pedal press; the app owns the single shared X-ray and
// toggles it on any change — same protocol as the hand-mounted GPIO6/7 buttons, so the
// pedal and both hand buttons all do the same thing.
//
// Wiring:
//   Pedal switch across GPIO6 & GPIO7 (GPIO7 driven LOW = its ground) — identical to the
//   X-ray button wiring on the hand boards. Any normally-open momentary pedal works.
//   Battery on 3V3/5V as usual. Onboard LED (GPIO8, active LOW): blinks while
//   advertising, solid when a central is connected.
//
// Library: NimBLE-Arduino 1.4.x.
// Board: "ESP32C3 Dev Module", core 2.0.17, USB CDC On Boot: Enabled, 115200 baud.

#include <NimBLEDevice.h>

// ---- Pedal across GPIO6/GPIO7 (same convention as the hand boards' X-ray button) ----
static constexpr int PIN_PEDAL     = 6;   // read with pull-up: LOW = pressed
static constexpr int PIN_PEDAL_GND = 7;   // driven LOW so the pedal can bridge 6 → 7

// ---- Onboard LED (ESP32-C3 SuperMini: GPIO8, active LOW) ----
static constexpr int PIN_LED = 8;

// ---- BLE identifiers — must match the app exactly (same family as the hand trackers) ----
#define SERVICE_UUID      "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define ORIENTATION_UUID  "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define XRAY_STATE_UUID   "4F7A0003-9B3E-4C2A-8D1F-0A1B2C3D4E5F"  // app WRITES shared X-ray here
#define DEVICE_NAME       "Foot Pedal"

// ---- Same 32-byte wire format as the hand trackers ----
struct __attribute__((packed)) OrientationPacket {
  float   w, x, y, z;     // quaternion — identity, this device has no IMU
  float   ax, ay, az;     // accel — always 0
  uint8_t calib;          // always 0
  uint8_t touchStart;     // always 0
  uint8_t touchCurrent;   // always 0
  uint8_t xrayOn;         // flips 0/1 on each pedal press — the only live field
};
static_assert(sizeof(OrientationPacket) == 32, "packet must be 32 bytes");

static OrientationPacket pkt = { 1, 0, 0, 0,  0, 0, 0,  0, 0, 0, 0 };

NimBLECharacteristic* orientationChar = nullptr;
volatile bool deviceConnected = false;

static constexpr uint32_t NOTIFY_INTERVAL_MS     = 20;    // same 50 Hz cadence as the hands
static constexpr uint32_t STATUS_LOG_INTERVAL_MS = 1000;
static uint32_t lastNotifyMs = 0;
static uint32_t lastLogMs    = 0;

// Pedal — sustained-LOW debounce, same as the hand boards' X-ray button (pedals bounce a lot).
static bool     xrayBit      = false;   // the flip bit we report in the packet
static bool     pedalPressed = false;   // debounced (confirmed) pedal state
static int      pedalRaw     = HIGH;    // last raw pin reading
static uint32_t pedalRawMs   = 0;       // when the raw reading last changed

// The TRUE shared X-ray state, written back by the app (the app owns it; our pedal only
// sends flip events). The onboard LED mirrors THIS while connected.
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
// board's button, this pedal, or the app's own UI — and once at connect.
class XrayStateCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* ch) override {
    NimBLEAttValue v = ch->getValue();
    if (v.size() >= 1) {
      xraySynced = (v[0] != 0);
      Serial.printf(">> X-RAY state from app -> %s\n", xraySynced ? "ON" : "OFF");
    }
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

// Each confirmed press FLIPS the bit. Only accept a level that has been stable ≥40 ms:
// switch bounce never lasts that long, a real stomp (held far longer) is counted once.
void handlePedal(uint32_t now) {
  int raw = digitalRead(PIN_PEDAL);
  if (raw != pedalRaw) { pedalRaw = raw; pedalRawMs = now; }   // raw changed → restart timer
  bool stable = (now - pedalRawMs) >= 40;
  if (stable && pedalRaw == LOW && !pedalPressed) {            // confirmed press → flip
    pedalPressed = true;
    xrayBit = !xrayBit;
    pkt.xrayOn = xrayBit ? 1 : 0;
    Serial.printf(">> PEDAL press -> bit %u\n", pkt.xrayOn);
  } else if (stable && pedalRaw == HIGH && pedalPressed) {     // confirmed release
    pedalPressed = false;
  }
}

// LED: 1 Hz blink while advertising; once connected it mirrors the shared X-ray state
// (lit = X-ray ON), so the pedal shows the same truth as the hand OLEDs. Active LOW.
void updateLed(uint32_t now) {
  bool on = deviceConnected ? xraySynced : ((now / 500) % 2 == 0);
  digitalWrite(PIN_LED, on ? LOW : HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== ESP32 X-ray Foot Pedal ===");
  Serial.println("Device: " DEVICE_NAME);

  pinMode(PIN_PEDAL_GND, OUTPUT);
  digitalWrite(PIN_PEDAL_GND, LOW);
  pinMode(PIN_PEDAL, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

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

  if (now - lastLogMs >= STATUS_LOG_INTERVAL_MS) {
    lastLogMs = now;
    Serial.printf("[status] %s xbit=%u xsync=%u pedal=%s\n",
                  deviceConnected ? "CONN" : "ADV", pkt.xrayOn, (unsigned)xraySynced,
                  pedalPressed ? "DOWN" : "up");
  }

  delay(5);   // yield — keeps the CPU cool, harmless to the 50 Hz cadence
}
