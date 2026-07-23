// ESP32-S3 SuperMini foot pedal → BLE BROADCAST (connectionless).
//
// Two pedals, built from the SAME sketch — only IS_LEFT_FOOT differs:
//   LEFT  foot ("Left Foot Pedal",  button across GPIO12 & GPIO11) → toggles X-ray on/off
//   RIGHT foot ("Right Foot Pedal", button across GPIO9  & GPIO8)  → triggers an X-ray screen capture
//
// WHY BROADCAST, NOT CONNECT: Vision Pro has a small BLE connection budget. With the two
// hand trackers already connected, the headset refuses further connections
// (CBError 11 "connection limit reached"), so the pedals could never join. A pedal only
// needs to convey "I was pressed," so instead of holding a connection it just ADVERTISES
// a press counter in its manufacturer data. The app reads that counter straight from the
// scan — no connection, no slot used. Each confirmed press increments the counter; the
// app fires one event per change. Meaning is keyed by the advertised name.
//
// Wiring (NO external resistors needed):
//   Momentary pedal switch across the two GPIOs. One GPIO is driven LOW and acts as the
//   button's ground; the other is read with the chip's INTERNAL pull-up (LOW = pressed).
//   Power: battery + → 5V pin, battery − → GND. NEVER battery on 5V while USB is plugged in.
//
// Libraries: NimBLE-Arduino 1.4.x (not 2.x).
// Board: "ESP32S3 Dev Module", core 2.0.17, USB CDC On Boot: Enabled, 115200 baud.

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

// ---- BLE identifiers — service UUID must match the app's scan filter ----
#define SERVICE_UUID  "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"

// Manufacturer data layout the app reads (see BLEManager): [0xFF, 0xFF, pressCount].
// 0xFFFF is the "no specific company" test ID. pressCount wraps 0–255; the app only
// cares that it CHANGED, so wrapping is harmless.
static constexpr uint8_t MFG_ID_LO = 0xFF;
static constexpr uint8_t MFG_ID_HI = 0xFF;
static uint8_t pressCount = 0;

NimBLEAdvertising* adv = nullptr;

// Pedal debounce — sustained-40ms scheme (rejects contact noise; one press = one count).
static bool     pedalPressed = false;
static int      pedalRaw     = HIGH;
static uint32_t pedalRawMs   = 0;

// Rebuild the advertisement so the current pressCount goes out on the air. Keeps the
// service UUID (so the app's filtered scan sees us) and the name (in the scan response,
// so the app can tell the two pedals apart). No GATT server, no connections.
void publishAdvertising() {
  uint8_t mfg[3] = { MFG_ID_LO, MFG_ID_HI, pressCount };

  adv->stop();
  adv->setManufacturerData(std::string(reinterpret_cast<char*>(mfg), sizeof(mfg)));
  adv->start();
}

void setupBLE() {
  Serial.println("[BLE] init (broadcast-only)...");
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);   // so the app's service-filtered scan finds us
  adv->setScanResponse(true);
  NimBLEAdvertisementData scanData;
  scanData.setName(DEVICE_NAME);       // name in scan response identifies which pedal
  adv->setScanResponseData(scanData);

  publishAdvertising();                // sets manufacturer data + starts advertising
  Serial.println("[BLE] advertising as " DEVICE_NAME " (no connection needed)");
}

// Each confirmed press increments the broadcast counter. Only a level stable >=40 ms
// counts, so cable/contact noise is ignored and one stomp registers exactly once.
void handlePedal(uint32_t now) {
  int raw = digitalRead(PIN_BTN);
  if (raw != pedalRaw) { pedalRaw = raw; pedalRawMs = now; }
  bool stable = (now - pedalRawMs) >= 40;
  if (stable && pedalRaw == LOW && !pedalPressed) {            // confirmed press
    pedalPressed = true;
    pressCount++;
    publishAdvertising();
    Serial.printf(">> PEDAL press -> count %u (broadcast)\n", pressCount);
  } else if (stable && pedalRaw == HIGH && pedalPressed) {     // confirmed release
    pedalPressed = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== ESP32-S3 Foot Pedal (broadcast) ===");
  Serial.println("Device: " DEVICE_NAME "  —  " FOOT_LABEL);

  pinMode(PIN_BTN_GND, OUTPUT);
  digitalWrite(PIN_BTN_GND, LOW);
  pinMode(PIN_BTN, INPUT_PULLUP);

  setupBLE();
  Serial.println("=== setup complete ===");
}

void loop() {
  handlePedal(millis());
  delay(5);   // heat-friendly yield; advertising runs on its own in the background
}
