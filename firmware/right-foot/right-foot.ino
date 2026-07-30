// ESP32-S3 SuperMini foot pedal → BLE BROADCAST (connectionless).
//
// Two pedals, built from the SAME sketch — only IS_LEFT_FOOT differs:
//   LEFT  foot ("Left Foot Pedal",  button across GPIO12 & GPIO11) → X-ray while HELD DOWN
//   RIGHT foot ("Right Foot Pedal", button across GPIO9  & GPIO8)  → one X-ray screen capture
//
// The LEFT pedal is a DEAD-MAN SWITCH (like a real fluoro pedal), not a toggle: X-ray is on
// for exactly as long as the foot is down. Because a press COUNTER cannot express "still
// held", the broadcast carries a 4th byte = the live held level (see MFG data below).
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

// ---- Which foot is this board? ----  1 = left (X-ray while held), 0 = right (capture)
#define IS_LEFT_FOOT 0

#if IS_LEFT_FOOT
  #define DEVICE_NAME "Left Foot Pedal"
  #define FOOT_LABEL  "LEFT FOOT (X-ray while held)"
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

// Manufacturer data layout the app reads (see BLEManager): [0xFF, 0xFF, pressCount, held].
// 0xFFFF is the "no specific company" test ID.
//   [2] pressCount — increments once per confirmed press; wraps 0–255 (consumers only care
//                    that it CHANGED). This is what the RIGHT pedal's capture event uses.
//   [3] held       — LIVE LEVEL: 1 while the pedal is down, 0 when up. This is what the
//                    LEFT pedal's hold-to-activate X-ray uses. Byte [2] is kept for both
//                    pedals so a consumer reading only the counter still sees presses.
// Advertising repeats continuously, so the current level keeps going out on the air; a
// consumer that stops hearing a pedal must treat it as RELEASED (fail-safe) — never leave
// X-ray latched on a link that has gone quiet.
static constexpr uint8_t MFG_ID_LO = 0xFF;
static constexpr uint8_t MFG_ID_HI = 0xFF;
static uint8_t pressCount = 0;
static uint8_t heldLevel  = 0;   // 1 = foot down (LEFT pedal's dead-man level)

NimBLEAdvertising* adv = nullptr;

// Pedal debounce — sustained-40ms scheme (rejects contact noise; one press = one count).
static bool     pedalPressed = false;
static int      pedalRaw     = HIGH;
static uint32_t pedalRawMs   = 0;

// Rebuild the advertisement so the current pressCount goes out on the air. Keeps the
// service UUID (so the app's filtered scan sees us) and the name (in the scan response,
// so the app can tell the two pedals apart). No GATT server, no connections.
void publishAdvertising() {
  uint8_t mfg[4] = { MFG_ID_LO, MFG_ID_HI, pressCount, heldLevel };

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
  // Advertise FAST (units of 0.625 ms → 100/150 ms). A dead-man pedal's on/off latency is
  // one advertising interval, so the NimBLE default (~1.28 s) would feel badly sluggish.
  // 100 ms keeps hold/release crisp; the pedal is mains-idle most of the time anyway.
  adv->setMinInterval(160);
  adv->setMaxInterval(240);
  NimBLEAdvertisementData scanData;
  scanData.setName(DEVICE_NAME);       // name in scan response identifies which pedal
  adv->setScanResponseData(scanData);

  publishAdvertising();                // sets manufacturer data + starts advertising
  Serial.println("[BLE] advertising as " DEVICE_NAME " (no connection needed)");
}

// Debounce: only a level stable >=40 ms counts, so cable/contact noise is ignored and one
// stomp registers exactly once. Every confirmed press bumps pressCount; the held level
// tracks the foot and is re-broadcast on BOTH edges so the LEFT pedal can act as a
// dead-man switch (the RIGHT pedal's consumer just ignores the level).
void handlePedal(uint32_t now) {
  int raw = digitalRead(PIN_BTN);
  if (raw != pedalRaw) { pedalRaw = raw; pedalRawMs = now; }
  bool stable = (now - pedalRawMs) >= 40;
  if (stable && pedalRaw == LOW && !pedalPressed) {            // confirmed press
    pedalPressed = true;
    pressCount++;
    heldLevel = 1;
    publishAdvertising();
    Serial.printf(">> PEDAL down -> count %u held=1 (broadcast)\n", pressCount);
  } else if (stable && pedalRaw == HIGH && pedalPressed) {     // confirmed release
    pedalPressed = false;
    heldLevel = 0;
    publishAdvertising();                                      // release must go out too
    Serial.println(">> PEDAL up   -> held=0 (broadcast)");
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
