// ESP32-S3 SuperMini DSA foot pedal (pedal 3) → BLE BROADCAST (connectionless).
//
// DSA = digital subtraction angiography: the "run" the operator takes once the catheter is
// in position and contrast (X-ray dye) is being injected, so the vessels light up. This
// pedal reports a LIVE LEVEL — the run is active for exactly as long as the pedal is
// down/switched on — plus a run counter so consumers can number the runs.
//
// A DSA run IS an X-ray acquisition, so consumers treat "DSA active" as implying X-ray
// imaging on top of rendering contrast in the vessels.
//
// WHY BROADCAST, NOT CONNECT: Vision Pro has a small BLE connection budget — with the two
// hand trackers connected it refuses more (CBError 11 "connection limit reached"). Pedals
// therefore never hold a connection; they advertise their state and consumers read it out
// of a continuous scan. See firmware/left-foot for the same pattern.
//
// WIRING — 3-wire SPDT switch (COM / NO / NC), NO external resistors:
//   COM → GPIO12   driven LOW: the switch's common "ground"
//   NO  → GPIO10   INPUT_PULLUP, reads LOW when the pedal is PRESSED  (NO closes to COM)
//   NC  → GPIO8    INPUT_PULLUP, reads LOW when the pedal is AT REST  (NC closed to COM)
//   Power: battery + → 5V pin, battery − → GND. NEVER battery on 5V while USB is plugged in.
//
// Reading BOTH contacts makes the state self-checking, which a 2-wire button cannot do:
//   NO closed, NC open  → ACTIVE (running)
//   NO open,  NC closed → IDLE
//   BOTH open           → FAULT: cable unplugged / broken wire / switch failed open
//   BOTH closed         → FAULT: miswired or a short between NO and NC
// Either fault reports "not running" (fail safe — never start a run on a suspect switch)
// and raises a fault flag in the broadcast so the UI can say so out loud.
//
// This works with a momentary pedal OR a maintained (latching) toggle: the broadcast is a
// LEVEL that simply mirrors the switch position either way.
//
// Libraries: NimBLE-Arduino 1.4.x (not 2.x).
// Board: "ESP32S3 Dev Module", core 2.0.17, USB CDC On Boot: Enabled, 115200 baud.
// GPIO 8/10/12 are all plain GPIOs on the S3 (strapping pins there are 0/3/45/46).

#include <NimBLEDevice.h>

#define DEVICE_NAME "DSA Foot Pedal"

// ---- Pins (3-wire SPDT) ----
static constexpr int PIN_COM = 12;   // driven LOW = the switch's common ground
static constexpr int PIN_NO  = 10;   // LOW = pressed
static constexpr int PIN_NC  = 8;    // LOW = at rest

// ---- BLE identifiers — service UUID must match the app's scan filter ----
#define SERVICE_UUID  "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"

// Manufacturer data: [0xFF, 0xFF, runCount, active, flags].
// 0xFFFF is the "no specific company" test ID; consumers see the payload AFTER it, so
// byte layout matches the other pedals for the first two payload bytes:
//   [2] runCount — increments once per run STARTED; wraps 0-255 (consumers watch for change)
//   [3] active   — LIVE LEVEL: 1 while the run is on, 0 when it ends (same slot as the
//                  left pedal's `held`, so one consumer code path handles all pedals)
//   [4] flags    — bit0 = wiring fault (see above). Consumers that ignore it still work.
static constexpr uint8_t MFG_ID_LO = 0xFF;
static constexpr uint8_t MFG_ID_HI = 0xFF;
static constexpr uint8_t FLAG_FAULT = 0x01;

static uint8_t runCount   = 0;
static uint8_t activeLevel = 0;   // 1 = DSA run in progress
static uint8_t flags       = 0;

NimBLEAdvertising* adv = nullptr;

// ---- Debounced switch state ----
enum SwitchState : uint8_t { SW_IDLE, SW_ACTIVE, SW_FAULT };
static SwitchState swStable = SW_IDLE;   // last CONFIRMED state
static SwitchState swRaw     = SW_IDLE;  // last raw reading
static uint32_t    swRawMs    = 0;       // when the raw reading last changed

// Decode the two complementary contacts into one state (see the wiring notes above).
SwitchState readSwitch() {
  bool noClosed = (digitalRead(PIN_NO) == LOW);
  bool ncClosed = (digitalRead(PIN_NC) == LOW);
  if (noClosed && !ncClosed) return SW_ACTIVE;
  if (!noClosed && ncClosed) return SW_IDLE;
  return SW_FAULT;                       // both open (disconnected) or both closed (miswired)
}

// Rebuild the advertisement so the current state goes out on the air. Keeps the service
// UUID (for the app's filtered scan) and the name (in the scan response, to tell pedals
// apart). No GATT server, no connections.
void publishAdvertising() {
  uint8_t mfg[5] = { MFG_ID_LO, MFG_ID_HI, runCount, activeLevel, flags };

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
  // Advertise fast and EXPLICITLY (units of 0.625 ms → 100/150 ms). A level control's
  // on/off latency is one advertising interval, so pin it here rather than inheriting
  // whatever the stack/controller happens to default to.
  adv->setMinInterval(160);
  adv->setMaxInterval(240);
  NimBLEAdvertisementData scanData;
  scanData.setName(DEVICE_NAME);       // name in scan response identifies this pedal
  adv->setScanResponseData(scanData);

  publishAdvertising();                // sets manufacturer data + starts advertising
  Serial.println("[BLE] advertising as " DEVICE_NAME " (no connection needed)");
}

// Only a state that holds steady for >=40 ms counts, so contact bounce and the brief
// both-open instant as an SPDT switch transfers between NC and NO are ignored.
void handlePedal(uint32_t now) {
  SwitchState s = readSwitch();
  if (s != swRaw) { swRaw = s; swRawMs = now; }      // restart the stability timer
  if ((now - swRawMs) < 40 || swRaw == swStable) return;

  swStable = swRaw;
  switch (swStable) {
    case SW_ACTIVE:
      runCount++;
      activeLevel = 1;
      flags &= ~FLAG_FAULT;
      Serial.printf(">> DSA RUN START -> run %u (broadcast)\n", runCount);
      break;
    case SW_IDLE:
      activeLevel = 0;
      flags &= ~FLAG_FAULT;
      Serial.println(">> DSA run end   -> active=0");
      break;
    case SW_FAULT:
      activeLevel = 0;                              // fail safe: never run on a bad switch
      flags |= FLAG_FAULT;
      Serial.println("!! SWITCH FAULT (both contacts open/closed) — check COM/NO/NC wiring");
      break;
  }
  publishAdvertising();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== ESP32-S3 DSA Foot Pedal (broadcast) ===");
  Serial.println("Device: " DEVICE_NAME "  —  contrast run while held");

  // COM is driven LOW as the switch's ground; NO and NC are read with internal pull-ups.
  pinMode(PIN_COM, OUTPUT);
  digitalWrite(PIN_COM, LOW);
  pinMode(PIN_NO, INPUT_PULLUP);
  pinMode(PIN_NC, INPUT_PULLUP);

  // Adopt the switch's real position at boot (no phantom run if it powers up pressed,
  // and an immediate fault report if the cable isn't connected).
  delay(10);
  swStable = swRaw = readSwitch();
  swRawMs = millis();
  if (swStable == SW_FAULT) {
    flags |= FLAG_FAULT;
    Serial.println("!! SWITCH FAULT at boot — check COM/NO/NC wiring");
  } else {
    Serial.printf("Switch at boot: %s\n", swStable == SW_ACTIVE ? "PRESSED" : "idle");
    activeLevel = (swStable == SW_ACTIVE) ? 1 : 0;
  }

  setupBLE();
  Serial.println("=== setup complete ===");
}

void loop() {
  handlePedal(millis());
  delay(5);   // heat-friendly yield; advertising runs on its own in the background
}
