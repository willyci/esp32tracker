// ESP32-C3 + BNO085 → BLE orientation tracker
// Streams a fused quaternion + raw accel + calibration over a single BLE notify
// characteristic. Packet layout and UUIDs MUST match the visionOS app (see ../../SPEC.md
// and ../../ESP32Tracker/BLEManager.swift).
//
// Libraries (install via Arduino Library Manager or see platformio.ini):
//   - Adafruit BNO08x
//   - NimBLE-Arduino  (tested against 1.4.x — see notes near onDisconnect for 2.x)
//
// Board: "ESP32C3 Dev Module" (or your SuperMini variant).

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <NimBLEDevice.h>

// ---- I2C pins (ESP32-C3 SuperMini defaults; confirm against your board silk) ----
static constexpr int PIN_SDA = 0;
static constexpr int PIN_SCL = 1;
static constexpr uint8_t BNO_ADDR = 0x4B;   // this board scans at 0x4B; some use 0x4A

// ---- BLE identifiers — must match the app exactly ----
// Service + characteristic UUIDs are IDENTICAL on both boards (that's how the app
// finds any tracker). Only the advertised NAME differs, so the app can tell the
// left hand from the right.
#define SERVICE_UUID      "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define ORIENTATION_UUID  "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F"

// ---- Which hand is this board? Flip this ONE line before flashing each board. ----
//   1 = left hand   |   0 = right hand
#define IS_LEFT_HAND 1

#if IS_LEFT_HAND
  #define DEVICE_NAME "Left Hand Tracker"
#else
  #define DEVICE_NAME "Right Hand Tracker"
#endif

// ---- 32-byte wire format (little-endian; ESP32 + Apple silicon are both LE) ----
struct __attribute__((packed)) OrientationPacket {
  float   w, x, y, z;     // quaternion (real, i, j, k)
  float   ax, ay, az;     // accel, m/s^2
  uint8_t calib;          // 0–3
  uint8_t pad[3];
};
static_assert(sizeof(OrientationPacket) == 32, "packet must be 32 bytes");

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

NimBLECharacteristic* orientationChar = nullptr;
volatile bool deviceConnected = false;

// Latest values, refreshed as reports arrive; sent together on a fixed cadence.
static OrientationPacket pkt = { 0, 0, 0, 1, 0, 0, 0, 0, {0, 0, 0} };

// Report cadence: 50 Hz (20 ms). Send notifications at the same rate.
static constexpr uint32_t REPORT_INTERVAL_US = 20000;
static constexpr uint32_t NOTIFY_INTERVAL_MS = 20;
static uint32_t lastNotifyMs = 0;

// ---------------------------------------------------------------------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* /*server*/) override {
    deviceConnected = true;
  }
  void onDisconnect(NimBLEServer* /*server*/) override {
    deviceConnected = false;
    NimBLEDevice::startAdvertising();   // allow reconnection
  }
  // NimBLE 2.x note: signatures gain a `NimBLEConnInfo&` arg, e.g.
  //   void onConnect(NimBLEServer*, NimBLEConnInfo&) override;
};

void enableReports() {
  // Rotation Vector = magnetometer-fused absolute orientation (drift-corrected).
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("Failed to enable rotation vector report");
  }
  if (!bno08x.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US)) {
    Serial.println("Failed to enable accelerometer report");
  }
}

void setupBLE() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);   // max TX power for a reliable link

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(SERVICE_UUID);
  orientationChar = service->createCharacteristic(
      ORIENTATION_UUID, NIMBLE_PROPERTY::NOTIFY);
  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);          // 128-bit UUID fills most of the 31-byte adv packet
  adv->setScanResponse(true);

  // "Left/Right Hand Tracker" is too long to fit in the main advertising packet
  // alongside a 128-bit service UUID, so carry the name in the scan response.
  // iOS merges scan-response data, so CBAdvertisementDataLocalNameKey still sees the
  // full name — which is how the app tells the two hands apart.
  NimBLEAdvertisementData scanData;
  scanData.setName(DEVICE_NAME);
  adv->setScanResponseData(scanData);

  adv->start();
  Serial.println("BLE advertising as " DEVICE_NAME);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(PIN_SDA, PIN_SCL);

  // The BNO085 doesn't ACK reliably on its first I2C transaction after power-up;
  // give it time to boot, then retry begin_I2C() a few times before giving up.
  // Without this it reports "address not found" on a sensor that's wired fine.
  delay(300);

  bool bnoReady = false;
  for (int attempt = 1; attempt <= 5 && !bnoReady; attempt++) {
    if (bno08x.begin_I2C(BNO_ADDR, &Wire)) {
      bnoReady = true;
      break;
    }
    Serial.printf("BNO08x begin attempt %d/5 failed — retrying\n", attempt);
    delay(200);
  }
  if (!bnoReady) {
    Serial.println("BNO08x not found — check wiring/address (0x4A vs 0x4B)");
    while (true) delay(1000);
  }
  Serial.println("BNO08x ready");
  enableReports();

  setupBLE();
}

void loop() {
  // The BNO08x resets reports after its own internal reset; re-enable if asked.
  if (bno08x.wasReset()) {
    Serial.println("BNO08x reset — re-enabling reports");
    enableReports();
  }

  // Drain whatever reports are available, caching the latest of each kind.
  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        pkt.w     = sensorValue.un.rotationVector.real;
        pkt.x     = sensorValue.un.rotationVector.i;
        pkt.y     = sensorValue.un.rotationVector.j;
        pkt.z     = sensorValue.un.rotationVector.k;
        pkt.calib = sensorValue.status & 0x03;   // 0–3 accuracy
        break;
      case SH2_ACCELEROMETER:
        pkt.ax = sensorValue.un.accelerometer.x;
        pkt.ay = sensorValue.un.accelerometer.y;
        pkt.az = sensorValue.un.accelerometer.z;
        break;
      default:
        break;
    }
  }

  // Notify at a steady cadence (only when someone is listening).
  const uint32_t now = millis();
  if (deviceConnected && orientationChar && (now - lastNotifyMs >= NOTIFY_INTERVAL_MS)) {
    lastNotifyMs = now;
    orientationChar->setValue(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
    orientationChar->notify();
  }
}
