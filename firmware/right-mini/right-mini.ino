// ESP32-C3 0.42" OLED board + MPU-6050 → BLE "Mini" hand tracker (no external screen).
//
// A compact glove unit: MPU-6050 orientation + SoftPot touch strip (grab/twist) + TWO
// buttons (X-ray on/off, X-ray screen capture), with status on the board's own 72x40 OLED.
// Two boards from the SAME sketch — only IS_LEFT_HAND differs.
//
// IMU NOTE — the MPU-6050 is 6-DOF (gyro + accel) with NO magnetometer, and unlike the
// BNO085 on the big trackers it does NO fusion of its own. So this sketch integrates the
// GYRO to produce the quaternion. What that means in practice:
//   * Rotation tracks well moment-to-moment — this is what makes the cube move.
//   * There is no absolute reference, so orientation DRIFTS on all three axes. Two things
//     keep it small: a gyro-bias calibration at boot, and a deadband that stops a still
//     board from creeping. Expect slow drift over minutes anyway — re-center in the app.
//   * Drift gets worse as the board warms (gyro bias is temperature-dependent), and these
//     boards do run warm. Re-power occasionally to re-calibrate if it gets annoying.
//   * HOLD THE BOARD STILL for ~1.5 s after power-up — that IS the bias calibration.
//   * Accelerometer is read too (same I2C transaction, free) and reported in the packet
//     for the dashboard's readouts, but it does NOT correct orientation by default. Set
//     USE_ACCEL_TILT to 1 below to have gravity pull roll/pitch straight; yaw is beyond
//     help without a magnetometer.
//
// Pins (this is the C3 board with the 0.42" OLED, ceramic antenna, Type-C):
//   MPU-6050  VCC→3V3  GND→GND  SDA→GPIO5  SCL→GPIO6  (addr 0x68; AD0/INT/XDA/XCL unused)
//             It SHARES the onboard OLED's I2C bus — different addresses (OLED 0x3C), so
//             this costs no extra pins. VCC must be 3V3, NOT 5V: the GY-521's pull-ups tie
//             SDA/SCL to VCC, and 5V on those lines would exceed the C3's 3.3V GPIOs.
//   SoftPot   V+→3V3  GND→GND  wiper→GPIO0 (ADC). Internal pulldown is enabled so no
//             external resistor is needed; if "no touch" readings jitter, add the same
//             100k wiper→GND the big trackers use.
//   X-ray button   across GPIO8 (sense) & GPIO7 (ground). GPIO8 is a STRAPPING pin, so it
//                  MUST be the INPUT_PULLUP sense pin — it then idles HIGH, which is what the
//                  chip needs at boot. GPIO7 is the driven-LOW ground (and isn't actually
//                  driven until firmware runs, so even holding the button at power-up is safe).
//                  Do NOT swap the two, or a floating GPIO8 can stop the board booting.
//   Capture button across GPIO3 & GPIO4 — GPIO4 driven LOW as the button's ground.
//   Onboard OLED   72x40 SSD1306, hardware I2C SDA=GPIO5 SCL=GPIO6 — shared with the IMU.
//   DO NOT USE: GPIO2 (strapping — a floating wiper/ground here can stop the boot),
//               GPIO9 (onboard BOOT button). GPIO8 is a strapping pin too, but is used
//               SAFELY as the X-ray sense pin (see above). GPIO1/10/RX/TX are free.
//   Power: battery + → 5V pin, battery − → GND. NEVER battery on 5V while USB is plugged in.
//
// Packet: same 32 bytes as every other board, UNCHANGED by adding the IMU. Byte 31 flips on
// each X-ray button press (= toggle shared X-ray). The CAPTURE button flips byte 28 — the
// calib slot — and the dashboard fires one screen capture per flip. NOTE that byte 28 is
// therefore NOT available for IMU calibration status on Mini boards; gyro-cal state is shown
// on the OLED and serial instead. Do not repurpose it.
//
// Libraries: NimBLE-Arduino 1.4.x (not 2.x), U8g2 (needs a version with the 72X40_ER
// constructor, v2.34+ — update U8g2 if the constructor is not found), Adafruit MPU6050
// (which pulls in Adafruit Unified Sensor).
// Board: "ESP32C3 Dev Module", core 2.0.17, USB CDC On Boot: Enabled, 115200 baud.

#include <Wire.h>
#include <Adafruit_MPU6050.h>
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
  float   w, x, y, z;     // orientation, integrated from the gyro (see IMU NOTE)
  float   ax, ay, az;     // accel, m/s^2 — reported for the dashboard readouts
  uint8_t calib;          // REPURPOSED on Mini boards: capture bit, flips per capture press.
                          // NOT IMU calibration status — see the header note.
  uint8_t touchStart;     // SoftPot position where the touch began (0 = no touch)
  uint8_t touchCurrent;   // current SoftPot position while touched (0 = no touch)
  uint8_t xrayOn;         // flips 0/1 on each X-ray button press
};
static_assert(sizeof(OrientationPacket) == 32, "packet must be 32 bytes");

static OrientationPacket pkt = { 1, 0, 0, 0,  0, 0, 0,  0, 0, 0, 0 };

// Onboard 72x40 OLED, hardware I2C — SHARED with the MPU-6050 (different addresses).
U8G2_SSD1306_72X40_ER_F_HW_I2C display(U8G2_R0, /*reset=*/U8X8_PIN_NONE,
                                       /*clock=*/PIN_OLED_SCL, /*data=*/PIN_OLED_SDA);

// ---- IMU ----
static constexpr uint8_t MPU_ADDR = 0x68;   // AD0 low; 0x69 if AD0 is tied high
Adafruit_MPU6050 mpu;
static bool imuOK = false;                  // false = run without orientation, don't halt

// Integrate on a FIXED cadence so dt is a known constant rather than whatever the loop
// happened to take. The loop spins faster than this and gates on millis().
static constexpr uint32_t IMU_HZ          = 100;
static constexpr uint32_t IMU_INTERVAL_MS = 1000 / IMU_HZ;
static constexpr float    IMU_DT          = 1.0f / (float)IMU_HZ;
static uint32_t lastImuMs = 0;

// Averaged at boot while the board is held still, then subtracted from every reading.
// Without this an MPU-6050 can sit at whole degrees per second and visibly spin the cube.
static float gyroBias[3] = { 0, 0, 0 };

// Residual bias still creeps after calibration, which would slowly rotate the cube on its
// own. Below this rate we call it "not moving" and zero it. Deliberate hand rotation is
// orders of magnitude faster, so this costs nothing real.
static constexpr float GYRO_DEADBAND_RADS = 0.012f;   // ~0.7 deg/s

// Optional gravity correction for roll/pitch. OFF by default: the requirement here is
// "gyro, to see the cube rotate". Set to 1 if drift becomes annoying in use — it bounds
// roll and pitch, but yaw is unfixable without a magnetometer.
#define USE_ACCEL_TILT 0
#if USE_ACCEL_TILT
static constexpr float TILT_GAIN = 0.02f;   // per step; higher = firmer, more accel noise
#endif

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
  // P3 (not the P9 max the big trackers use): every BLE transmit is a current SPIKE, and
  // on battery power (LiPo → 5V pin) the spikes sag the 3.3V rail and flicker the OLED.
  // P3 halves the spike and is plenty for glove-to-headset/PC range.
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(SERVICE_UUID);
  orientationChar = service->createCharacteristic(ORIENTATION_UUID, NIMBLE_PROPERTY::NOTIFY);
  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  // Advertise less often (units of 0.625 ms → ~320-400 ms): fewer radio bursts per second
  // while waiting for a connection = less rail sag on battery. Discovery still takes <2 s.
  adv->setMinInterval(512);
  adv->setMaxInterval(640);
  NimBLEAdvertisementData scanData;
  scanData.setName(DEVICE_NAME);
  adv->setScanResponseData(scanData);
  adv->start();
  Serial.println("[BLE] advertising as " DEVICE_NAME);
}

// Log every device answering on the shared I2C bus. Expect BOTH 0x3C (OLED) and 0x68
// (MPU-6050) — this is the wiring check after adding the IMU.
void i2cScan() {
  Serial.println("[I2C] scanning bus...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      const char* who = (addr == MPU_ADDR) ? "  <- MPU-6050"
                      : (addr == 0x3C)     ? "  <- OLED" : "";
      Serial.printf("[I2C]   device at 0x%02X%s\n", addr, who);
      found++;
    }
  }
  if (found == 0)
    Serial.println("[I2C]   NONE found — check SDA/SCL/3V3/GND wiring");
}

// Average the gyro while the board sits still; that average IS the bias. This is the
// single biggest lever on drift, which is why the board must be left alone here.
void calibrateGyro() {
  Serial.println("[IMU] calibrating gyro bias — HOLD STILL...");
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 14, "HOLD");
  display.drawStr(0, 26, "STILL...");
  display.sendBuffer();

  const int samples = 300;                 // ~1.5 s at 5 ms/sample
  double sum[3] = { 0, 0, 0 };
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum[0] += g.gyro.x; sum[1] += g.gyro.y; sum[2] += g.gyro.z;
    delay(5);
  }
  for (int i = 0; i < 3; i++) gyroBias[i] = (float)(sum[i] / samples);
  Serial.printf("[IMU] gyro bias (rad/s): %+.4f %+.4f %+.4f\n",
                gyroBias[0], gyroBias[1], gyroBias[2]);
}

// One integration step. Quaternion q is updated by the small rotation the gyro measured
// over IMU_DT: q += 0.5 * q * omega * dt, then renormalized. Accel is copied through for
// the dashboard's readouts (it does not steer orientation unless USE_ACCEL_TILT).
void updateIMU() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float gx = g.gyro.x - gyroBias[0];       // rad/s
  float gy = g.gyro.y - gyroBias[1];
  float gz = g.gyro.z - gyroBias[2];
  if (fabsf(gx) < GYRO_DEADBAND_RADS) gx = 0;
  if (fabsf(gy) < GYRO_DEADBAND_RADS) gy = 0;
  if (fabsf(gz) < GYRO_DEADBAND_RADS) gz = 0;

  float qw = pkt.w, qx = pkt.x, qy = pkt.y, qz = pkt.z;
  const float h = 0.5f * IMU_DT;
  float nw = qw + (-qx * gx - qy * gy - qz * gz) * h;
  float nx = qx + ( qw * gx + qy * gz - qz * gy) * h;
  float ny = qy + ( qw * gy - qx * gz + qz * gx) * h;
  float nz = qz + ( qw * gz + qx * gy - qy * gx) * h;

#if USE_ACCEL_TILT
  // Nudge the estimate so the measured gravity direction lines up with the quaternion's
  // idea of "down". Only affects roll/pitch — rotation about gravity is invisible to an
  // accelerometer, so yaw is untouched.
  float amag = sqrtf(a.acceleration.x * a.acceleration.x +
                     a.acceleration.y * a.acceleration.y +
                     a.acceleration.z * a.acceleration.z);
  if (amag > 6.0f && amag < 13.0f) {       // only when close to 1 g (not being shaken)
    float axn = a.acceleration.x / amag, ayn = a.acceleration.y / amag, azn = a.acceleration.z / amag;
    // Gravity as the current quaternion predicts it (third row of the rotation matrix).
    float vx = 2.0f * (nx * nz - nw * ny);
    float vy = 2.0f * (nw * nx + ny * nz);
    float vz = nw * nw - nx * nx - ny * ny + nz * nz;
    // Error = measured x predicted; feed it back as a small extra rotation.
    float ex = ayn * vz - azn * vy;
    float ey = azn * vx - axn * vz;
    float ez = axn * vy - ayn * vx;
    nx += TILT_GAIN * ex; ny += TILT_GAIN * ey; nz += TILT_GAIN * ez;
  }
#endif

  float norm = sqrtf(nw * nw + nx * nx + ny * ny + nz * nz);
  if (norm > 1e-6f) {                      // never publish a degenerate quaternion
    pkt.w = nw / norm; pkt.x = nx / norm; pkt.y = ny / norm; pkt.z = nz / norm;
  }
  pkt.ax = a.acceleration.x; pkt.ay = a.acceleration.y; pkt.az = a.acceleration.z;
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

// Only push pixels when something CHANGED. On battery, each full-buffer redraw is a
// current burst that can dip the 3.3V rail (LiPo→5V-pin leaves the LDO little headroom)
// and make the screen flicker at the refresh rate — a static screen draws no burst.
struct OledState { bool conn; bool xray; uint32_t cap; uint8_t touch; };
static OledState lastDrawn = { false, false, 0xFFFFFFFF, 255 };   // force first draw

void drawOLED() {
  OledState now = { deviceConnected, xrayState, captureCount, pkt.touchCurrent };
  if (now.conn == lastDrawn.conn && now.xray == lastDrawn.xray &&
      now.cap == lastDrawn.cap && now.touch == lastDrawn.touch)
    return;
  lastDrawn = now;

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

  // Bring the shared I2C bus up ONCE, here, before anything that uses it. Both U8g2 and
  // Adafruit_MPU6050 would otherwise each init Wire on their own terms and fight over the
  // clock; setting it explicitly keeps that in one place.
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(400000);     // both the SSD1306 and the MPU-6050 are happy at 400 kHz

  Serial.println("[OLED] begin (72x40, hardware I2C SDA=5 SCL=6)...");
  display.begin();
  display.setContrast(64);   // ~quarter brightness — cuts OLED current a lot; still
                             // easily readable, and less rail sag on battery power

  i2cScan();                 // expect 0x3C (OLED) and 0x68 (MPU-6050)

  // The IMU is optional at runtime: a Mini with no IMU (or a broken one) should still be a
  // useful SoftPot + buttons board rather than a brick, so log and carry on instead of
  // halting the way the big trackers do.
  imuOK = mpu.begin(MPU_ADDR, &Wire);
  if (imuOK) {
    // +-500 deg/s and +-4 g suit hand motion: headroom for a brisk gesture without
    // throwing away resolution. The 21 Hz filter tames noise well above our 100 Hz rate.
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[IMU] MPU-6050 ready");
    calibrateGyro();
    lastImuMs = millis();
  } else {
    Serial.println("[IMU] MPU-6050 NOT FOUND at 0x68 — check SDA=5/SCL=6/3V3; running "
                   "without orientation (quaternion stays identity)");
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(0, 14, "NO IMU");
    display.drawStr(0, 26, "chk wiring");
    display.sendBuffer();
    delay(1500);
  }

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

  // Integrate the gyro on a fixed 100 Hz cadence (faster than the 50 Hz notify, so the
  // orientation being sent is always fresh).
  if (imuOK && (now - lastImuMs >= IMU_INTERVAL_MS)) {
    lastImuMs += IMU_INTERVAL_MS;                // fixed step: dt stays exactly IMU_DT
    if (now - lastImuMs > 5 * IMU_INTERVAL_MS) lastImuMs = now;   // resync after a stall
    updateIMU();
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
