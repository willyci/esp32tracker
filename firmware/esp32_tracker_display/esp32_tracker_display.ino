// ESP32-C3 + BNO085 → BLE orientation tracker, WITH onboard OLED status display.
//
// Same tracker firmware as ../esp32_tracker/esp32_tracker.ino (identical BLE protocol,
// packet layout, and UUIDs — see ../../SPEC.md), plus a 128x64 OLED (GME12864 /
// SSD1306) that shows which hand this board is, the BLE state, and the live
// quaternion. Useful when running untethered on battery — no serial monitor needed.
//
// Wiring (in addition to the BNO085 on GPIO0/1):
//   OLED VCC → 3V3      OLED GND → GND
//   OLED SDA → GPIO0    OLED SCL → GPIO1   (SHARED hardware I2C bus with the BNO085 —
//                                           different addresses: OLED 0x3C, BNO 0x4B.
//                                           HW I2C @400kHz redraws in ~25 ms; software
//                                           I2C on separate pins was ~1 fps and stalled
//                                           the BLE stream.)
//
// Libraries (Arduino Library Manager):
//   - Adafruit BNO08x
//   - NimBLE-Arduino 1.4.x  (NOT 2.x — callback signatures differ)
//   - U8g2  (by oliver)
//
// Board: "ESP32C3 Dev Module", ESP32 core 2.0.17, USB CDC On Boot: Enabled.

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <NimBLEDevice.h>
#include <U8g2lib.h>

// ---- I2C pins (one shared hardware bus: BNO085 + OLED) ----
static constexpr int PIN_SDA = 0;
static constexpr int PIN_SCL = 1;
static constexpr uint8_t BNO_ADDR = 0x4B;   // this board scans at 0x4B; some use 0x4A

// ---- BLE identifiers — must match the app exactly ----
#define SERVICE_UUID      "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define ORIENTATION_UUID  "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F"

// ---- Which hand is this board? Flip this ONE line before flashing each board. ----
//   1 = left hand   |   0 = right hand
#define IS_LEFT_HAND 1

#if IS_LEFT_HAND
  #define DEVICE_NAME "Left Hand Tracker"
  #define HAND_LABEL  "LEFT"
#else
  #define DEVICE_NAME "Right Hand Tracker"
  #define HAND_LABEL  "RIGHT"
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

// Full-buffer SSD1306 on the shared hardware I2C bus. If your module is the 1.3"
// SH1106 variant (text shows but shifted/garbled), swap SSD1306 → SH1106 below.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(
    U8G2_R0, /*reset=*/ U8X8_PIN_NONE, /*clock=*/ PIN_SCL, /*data=*/ PIN_SDA);

NimBLECharacteristic* orientationChar = nullptr;
volatile bool deviceConnected = false;

// Latest values, refreshed as reports arrive; sent together on a fixed cadence.
static OrientationPacket pkt = { 0, 0, 0, 1, 0, 0, 0, 0, {0, 0, 0} };

// Report cadence: 50 Hz (20 ms). Notify at the same rate. The OLED redraws at 10 Hz —
// a full sendBuffer() at 400 kHz HW I2C blocks ~25 ms, so keep it infrequent enough
// that the BLE notify cadence stays solid.
static constexpr uint32_t REPORT_INTERVAL_US  = 20000;
static constexpr uint32_t NOTIFY_INTERVAL_MS  = 20;
static constexpr uint32_t DISPLAY_INTERVAL_MS = 100;
static uint32_t lastNotifyMs  = 0;
static uint32_t lastDisplayMs = 0;

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

  // The long device name travels in the scan response (see esp32_tracker.ino notes).
  NimBLEAdvertisementData scanData;
  scanData.setName(DEVICE_NAME);
  adv->setScanResponseData(scanData);

  adv->start();
  Serial.println("BLE advertising as " DEVICE_NAME);
}

// Show a startup/status message on the OLED (also used for error states).
void displayMessage(const char* line1, const char* line2) {
  display.clearBuffer();
  display.setFont(u8g2_font_9x15B_tf);
  display.drawStr(0, 14, HAND_LABEL);
  display.setFont(u8g2_font_7x13_tf);
  display.drawStr(0, 38, line1);
  display.drawStr(0, 54, line2);
  display.sendBuffer();
}

// Live data screen: hand + BLE state + calibration on top; quaternion (left column)
// and accelerometer (right column) below.
void displayData() {
  char buf[24];
  display.clearBuffer();

  display.setFont(u8g2_font_9x15B_tf);
  display.drawStr(0, 13, HAND_LABEL);

  display.setFont(u8g2_font_7x13_tf);
  display.drawStr(64, 13, deviceConnected ? "CONN" : "ADV");
  snprintf(buf, sizeof(buf), "C:%u", pkt.calib);
  display.drawStr(104, 13, buf);
  display.drawHLine(0, 16, 128);

  display.setFont(u8g2_font_6x10_tf);

  // Left column: quaternion.
  snprintf(buf, sizeof(buf), "w %+.3f", pkt.w);   display.drawStr(0, 27, buf);
  snprintf(buf, sizeof(buf), "x %+.3f", pkt.x);   display.drawStr(0, 39, buf);
  snprintf(buf, sizeof(buf), "y %+.3f", pkt.y);   display.drawStr(0, 51, buf);
  snprintf(buf, sizeof(buf), "z %+.3f", pkt.z);   display.drawStr(0, 63, buf);

  // Right column: accel, m/s^2 (range ±19.61 needs up to 6 value chars).
  snprintf(buf, sizeof(buf), "ax%+6.2f", pkt.ax); display.drawStr(68, 27, buf);
  snprintf(buf, sizeof(buf), "ay%+6.2f", pkt.ay); display.drawStr(68, 39, buf);
  snprintf(buf, sizeof(buf), "az%+6.2f", pkt.az); display.drawStr(68, 51, buf);

  display.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // One shared bus at 400 kHz (both the SSD1306 and BNO085 support fast mode).
  // setBusClock keeps U8g2 from dropping the bus back to its 100 kHz default
  // during display transfers, which would slow the sensor traffic too.
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  display.setBusClock(400000);
  display.begin();
  displayMessage("Starting...", "");

  // The BNO085 doesn't ACK reliably on its first I2C transaction after power-up;
  // give it time to boot, then retry begin_I2C() a few times before giving up.
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
    displayMessage("BNO085 NOT FOUND", "check wiring/addr");
    while (true) delay(1000);
  }
  Serial.println("BNO08x ready");
  enableReports();

  setupBLE();
  displayMessage("BLE advertising", DEVICE_NAME);
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

  const uint32_t now = millis();

  // Notify at a steady cadence (only when someone is listening).
  if (deviceConnected && orientationChar && (now - lastNotifyMs >= NOTIFY_INTERVAL_MS)) {
    lastNotifyMs = now;
    orientationChar->setValue(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
    orientationChar->notify();
  }

  // Refresh the OLED at 10 Hz.
  if (now - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = now;
    displayData();
  }
}
