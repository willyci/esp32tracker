// ESP32-S3-Touch-AMOLED-2.06 → CONNECTED hand-slot tracker + touchscreen pedal panel.
// THIS COPY: LEFT hand  - screen rotated 180 degrees.
// Generated from firmware/pedal-panel; differs from the other panel sketch ONLY in the
// 'Which hand slot' / 'Screen orientation' config block below.
//
// One device doing two jobs: the three foot-pedal controls (X-ray / DSA / capture) as
// full-width buttons on a 410x502 AMOLED, plus a real hand tracker — QMI8658 orientation
// and a SoftPot strip — so it fills a HAND SLOT in place of a Mini.
//
// WHY CONNECTED, when the foot pedals are deliberately connectionless: because this board
// REPLACES a hand rather than adding a device, the Vision Pro connection budget is
// unchanged (still two), so there is no CBError 11 risk. And connecting is the only way to
// get smooth orientation: broadcast advertisements are delivered by the host at only
// ~1-3 per second (measured — see the foot-pedal notes), which would step a cube along at
// 1-3 Hz. As a GATT peripheral it streams the standard 32-byte packet at 50 Hz like every
// other tracker, and it genuinely KNOWS when a central is connected, so the status screen
// reports real connection state and hands over to the buttons on connect.
//
// WIRING — only ONE GPIO is claimed; everything else rides the board's own buses:
//   SoftPot  V+→3V3  GND→GND  wiper→IO19    (ADC2_CH8)
//     * 3V3, NOT VBUS: 5 V on the wiper would exceed the ADC input range.
//     * IO19 is USB D−, so NATIVE USB IS GIVEN UP. Set "USB CDC On Boot: Disabled" and
//       use the UART0 pins (RXD/TXD = 44/43) with a USB-serial adapter for the console.
//     * To reflash over USB, unplug the SoftPot from IO19 first — a resistive divider on
//       D− stops the port enumerating. Reconnect it afterwards.
//     * ADC2 is blocked while WiFi runs. WiFi is unused here and BLE does not take that
//       lock, so this works — but if the reading goes erratic ONLY while BLE transmits,
//       that is the cause; move the strip to an ADS1115 on the I2C bus (0x48).
//   HAPTIC   → the board's own two MOTOR pads (top-left), which the schematic shows driven
//              by GPIO18. No DRV2605L is fitted, so do not expect 0x5A in the I2C scan.
//              That driver is UNIPOLAR — a switch to ground, not the H-bridge a DRV2605
//              gives — so an LRA cannot be driven at full efficiency here. Switching it at
//              the LRA's resonant frequency (HAPTIC_LRA_HZ) still moves it usefully; DC
//              would barely twitch it. For an ERM/coin motor set HAPTIC_LRA_HZ to 0.
//              To go back to a DRV2605L on I2C, set HAPTIC_USE_DRV2605 to 1.
//
// TOOLCHAIN — THIS BOARD IS THE ODD ONE OUT IN THIS REPO:
//   * Arduino-ESP32 core **3.3.11** (Waveshare's stated version; the ES8311 audio API
//     ESP_I2S.h only exists on 3.x). Every OTHER board here needs core 2.0.17, because
//     NimBLE-Arduino 1.4.x crashes on 3.x — so you must switch the CORE in Boards Manager
//     depending on which board you are flashing.
//   * BLE here uses the **core-bundled BLE library**, not NimBLE-Arduino. NimBLE 1.4.x will
//     not work on core 3.x and NimBLE 2.x would then break the 2.0.17 boards, since the
//     library version is shared across the sketchbook. Using the bundled BLE means you swap
//     only the core and never the library.
//   * Advertising uses the library's default payload: the 128-bit service UUID in the
//     advertisement (the visionOS app scans with a SERVICE FILTER, so it must be there) and
//     the device name in the scan response, because the two do not fit in one 31-byte PDU.
//
// POWER BUTTON — hold ~6 s to switch the board off. The button is wired to the AXP2101's
// PWRON pin, not to the ESP32, and the PMU ignores a long press until told otherwise, so
// setup() arms it (see "Long-press the side button"). Two things to know:
//   * It will NOT stay off while USB is connected — VBUS restarts the PMU. Unplug first.
//   * A short press does nothing; this sketch does not use the PKEY interrupt.
//
// BOARD OPTIONS — Tools menu, "ESP32S3 Dev Module", 115200 baud. Arduino keeps these per
// sketch FOLDER and stores nothing in the folder, so a copied sketch inherits whatever the
// IDE had selected — get them wrong and this board boots black with no BLE. Taken from
// Waveshare's own CI FQBN (ESP32-S3-Touch-AMOLED-2.06/docs/ci.md), which builds their
// examples on this exact hardware:
//     USBMode=hwcdc, CDCOnBoot=cdc, PSRAM=opi, FlashSize=16M,
//     PartitionScheme=app3M_fat9M_16MB, core 3.3.11
// which maps to:
//   PSRAM ............. OPI PSRAM        <- NOT QSPI. The module is a WROOM-1-N16R8 and the
//                                           R8 part is 8 MB OCTAL PSRAM; QSPI leaves it dead
//                                           and the 402 KiB LVGL buffer with no headroom.
//   Flash Size ........ 16MB (128Mb)
//   Partition Scheme .. 3MB APP / 9MB FATFS
//   USB Mode .......... Hardware CDC and JTAG
//   USB CDC On Boot ... DISABLED          <- THE ONE DELIBERATE DEVIATION from that FQBN.
//                                           Waveshare's examples do not use IO19; ours puts
//                                           the SoftPot wiper there, which is USB D-, so the
//                                           port cannot enumerate. Enforced by #error below.
//   Core .............. Arduino-ESP32 3.3.11 (see the TOOLCHAIN note above)
// Libraries: lvgl 9 + Arduino_GFX + Arduino_DriveBus + XPowersLib (all bundled with the
// Waveshare repo), Adafruit DRV2605. BLE and ESP_I2S come with the core.

#include <Wire.h>
#include <Arduino.h>

// ---------------------------------------------------------------------------
// CORE VERSION, ENFORCED.
//
// Placement is load-bearing in BOTH directions:
//   * AFTER <Arduino.h>, because ESP_ARDUINO_VERSION_MAJOR lives in esp_arduino_version.h
//     (pulled in by Arduino.h) and is NOT a -D compiler flag. Put this check above the
//     includes and the macro is simply undefined, so it fires on every core including the
//     right one. That mistake broke every panel sketch here once already.
//   * BEFORE "ESP_I2S.h", so the wrong core reports THIS message instead of
//     "ESP_I2S.h: No such file or directory" pointing at the line of <Wire.h>.
//
// THIS BOARD IS THE ONLY ONE IN THE REPO ON 3.x. Every other sketch needs 2.0.17, because
// NimBLE-Arduino 1.4.x crashes on 3.x with a Guru Meditation at BLE init — so the core gets
// swapped in Boards Manager constantly, and this catches the half that swaps wrong.
// ---------------------------------------------------------------------------
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
  #error "WRONG ESP32 CORE: this sketch needs Arduino-ESP32 3.x (3.3.11). ESP_I2S.h, the ES8311 audio API, is 3.x-only. Switch in Boards Manager: esp32 by Espressif -> 3.3.11. Every OTHER sketch in this repo needs 2.0.17, so switch back afterwards. A core change also RESETS per-sketch Tools options: re-check USB CDC On Boot Disabled, PSRAM OPI, Flash 16MB, Partition 3MB APP/9MB FATFS."
#endif

#include "pin_config.h"
#include <lvgl.h>

#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
// NB: no #include "lv_conf.h" here. LVGL pulls its own config in via
// ../../lv_conf.h (lv_conf_internal.h), and lv_conf.h lives in the libraries
// ROOT, which is not on the include path — including it directly just fails.

#include "XPowersLib.h"
#include <BLEDevice.h>          // core-bundled BLE — deliberately NOT NimBLE, see below
#include <BLEServer.h>
#include <BLE2902.h>            // CCCD, so a central can subscribe to notifications
#include "SensorQMI8658.hpp"    // onboard 6-axis IMU (SensorLib, bundled by Waveshare)
#include <Adafruit_DRV2605.h>

#include "ESP_I2S.h"
#include "esp_check.h"
#include "es8311.h"
#include "driver/gpio.h"

// ---------------------------------------------------------------------------
// Pins and constants
// ---------------------------------------------------------------------------
static constexpr int PIN_SOFTPOT = 19;            // ADC2_CH8 — see WIRING note above
static constexpr int SOFTPOT_NOTOUCH_RAW = 80;    // raw ADC below this = no touch

static constexpr int PIN_I2S_BCLK = 41;           // from examples/arduino/08_ES8311
static constexpr int PIN_I2S_WS   = 45;
static constexpr int PIN_I2S_DOUT = 40;
static constexpr int PIN_I2S_DIN  = 42;
static constexpr int PIN_I2S_MCLK = 16;
static constexpr int PIN_AUDIO_PA = 46;           // speaker amplifier enable

static constexpr uint32_t SAMPLE_RATE   = 16000;
static constexpr int      VOICE_VOLUME  = 85;     // 0-100
static constexpr int      I2C_NUM       = 0;      // Wire's port; es8311 shares it

// ---------------------------------------------------------------------------
// BOARD OPTIONS, ENFORCED.
//
// Arduino IDE keeps Tools settings per sketch FOLDER and writes nothing into the folder
// itself, so copying this sketch does NOT copy its board options — a fresh copy silently
// inherits whatever the IDE happens to have selected. Two of those options do not merely
// degrade this sketch, they stop it booting with no clue on screen or serial, so they are
// checked here instead of being left to memory.
//
// ARDUINO_USB_CDC_ON_BOOT and BOARD_HAS_PSRAM are both supplied by the ESP32 core from the
// Tools menu; there is nothing to define by hand.
// ---------------------------------------------------------------------------
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  #error "Set Tools > USB CDC On Boot: DISABLED. IO19 is the SoftPot wiper and IO19 is USB D-, so the USB port cannot enumerate; with CDC on boot, Serial is that dead port and Serial.println() runs before the display comes up, which boots to a black screen with no diagnostics. Console is UART0 on RXD/TXD (44/43) via a USB-serial adapter."
#endif

#if !defined(BOARD_HAS_PSRAM)
  #warning "Tools > PSRAM looks disabled. LVGL needs a 402 KiB frame buffer here; enable PSRAM for the headroom."
#endif

// ---- BLE identifiers — service UUID must match the app's scan filter ----
#define SERVICE_UUID       "4F7A0001-9B3E-4C2A-8D1F-0A1B2C3D4E5F"
#define ORIENTATION_UUID   "4F7A0002-9B3E-4C2A-8D1F-0A1B2C3D4E5F"

// ---- Which hand slot does this panel fill? ----  1 = left, 0 = right
// This is the ONLY block that differs between firmware/left-panel and firmware/right-panel.
// Diff the two .ino files: everything below this header must be identical.
#define PANEL_IS_LEFT 1

// ---- Screen orientation ----
// The two units are mounted mirror-image on the hands, so one of them reads upside down.
// Flipping it needs THREE changes in step, not just the rotation argument:
//   * PANEL_ROTATION below, handed to Arduino_CO5300;
//   * PANEL_COL_OFFSET2, because Arduino_TFT::setRotation takes _xStart from COL_OFFSET2 at
//     rotation 2 and from COL_OFFSET1 at rotation 0 (see the constructor);
//   * the touch axes in my_touchpad_read -- the FT3168 reports raw panel coordinates and
//     has no idea the display is rotated.
#define PANEL_ROTATE_180 1
// ALWAYS 0. Arduino_CO5300's rotation-2 path is broken on this panel: its MADCTL table has
// CO5300_MADCTL_Y_AXIS_FLIP = 0x05, which is two bits where a flip flag needs one, so
// rotation 2 sends 0x07 -- observed to flip X, not flip Y, and to shear any large address
// window. Staying at rotation 0 also keeps _xStart on COL_OFFSET1 = 22, the offset the
// upright right panel proves correct, so COL_OFFSET2 stops mattering at all.
// The 180 degree flip is done by PANEL_MADCTL_180 below instead.
#define PANEL_ROTATION 0

// Memory Access Control (0x36), written after begin() when the panel is mounted upside down.
//
// What the bench has established:
//   0x02  flips X  — confirmed.
//   0x04  does nothing — 0x06 (= 0x02|0x04) still mirrors instead of rotating.
// Arduino_GFX defines CO5300_MADCTL_Y_AXIS_FLIP as 0x05, which is 0x04|0x01. Since 0x04 is
// inert, the real vertical-flip bit in that constant is almost certainly 0x01 — the author
// kept the right bit and added a stray one. Hence 180 = X flip | V flip = 0x02|0x01 = 0x03.
//
// If 0x03 is still not a true 180, sweep in this order and match PANEL_TOUCH_FLIP_* below
// to whatever the glass actually does:
//   0x01  -> expect a VERTICAL mirror only. This is the isolating test: if it does that,
//            0x01 IS the V-flip bit and 0x03 must be right, so look elsewhere (row offset).
//            If 0x01 does nothing, the V flip is not in the low nibble at all.
//   0x07  -> 0x03 plus the inert 0x04; what Arduino_GFX's rotation-2 path meant to send.
//   0x42 / 0x82 / 0xC2  -> fallback if this part follows standard MIPI after all
//            (MX = 0x40, MY = 0x80), combined with the known-good 0x02.
// Bit 3 (0x08) is BGR on this part — leave it clear or the colours inverst.
//
// NOTE: a working V flip may shift the image VERTICALLY, because the GRAM height for this
// 502 px panel is unknown (CO5300_TFTHEIGHT claims 480). If that happens, that is
// PANEL_ROW_OFFSET1 above, not a wrong MADCTL — the flip itself would be correct.
// Bench log — X flips correctly in every case below; only the vertical half is missing:
//     0x06 = 0x02|0x04 -> mirror, not 180   => 0x04 inert
//     0x03 = 0x02|0x01 -> mirror, not 180   => 0x01 inert
// The low nibble is therefore exhausted. 0x80 (standard MIPI MY, "row address order") is the
// remaining candidate, so this is MY + the known-good X flip. If it still will not flip
// vertically, stop sweeping bits and set PANEL_FLIP_Y_IN_SOFTWARE below instead.
#define PANEL_MADCTL_180 0x82

// Fallback for the vertical half of the 180, done in software, for when no MADCTL value
// works. Reverses the frame buffer's row order around each push; together with MADCTL's
// working X flip that is a true 180 no matter what the controller supports.
//
// Costs ~1 ms per pushed frame against the ~10 ms the push itself takes, and only when the
// screen actually changed. Set PANEL_MADCTL_180 back to 0x02 (X flip only) when enabling
// this, or the two will fight if a MADCTL vertical flip is in fact working.
//
// Caveat: this rotates only what LVGL renders. The boot notes and the tracker-mode notice
// are drawn straight to the panel with GFX and bypass this buffer, so they stay upright
// while the UI is flipped. Diagnostic text only — the pedal UI is what has to be right.
#define PANEL_FLIP_Y_IN_SOFTWARE 0

// Touch flips, deliberately SEPARATE from the display flip and from each other.
//
// Touch used to invert both axes whenever PANEL_ROTATE_180 was set. That is right only once
// the display genuinely does 180; while the panel is still mirroring a single axis it
// over-corrects, and taps land nowhere near the buttons. Keeping these independent means
// touch can be made to match the glass at every step of the MADCTL sweep above.
//   true 180 (both axes) ..... X 1, Y 1
//   horizontal mirror only ... X 1, Y 0
//   vertical mirror only ..... X 0, Y 1
#define PANEL_TOUCH_FLIP_X 1
#define PANEL_TOUCH_FLIP_Y 1
#if PANEL_IS_LEFT
  #define DEVICE_NAME "Left Panel Tracker"
  #define HAND_LABEL  "LEFT"
#else
  #define DEVICE_NAME "Right Panel Tracker"
  #define HAND_LABEL  "RIGHT"
#endif

// ---- The standard 32-byte tracker packet (see ../../SPEC.md) ----
// Identical layout to the hand trackers and Minis, so this board drops into a hand slot.
// Two fields carry the pedal controls, extending the Mini convention by one bit:
//   calib  (byte 28) — CAPTURE COUNT, increments per tap (Minis already repurpose this)
//   xrayOn (byte 31) — LEVEL BITFIELD: bit0 = X-ray held, bit1 = DSA run. NOT a flip:
//                      these are hold controls, so consumers read the bits directly.
struct __attribute__((packed)) OrientationPacket {
  float   w, x, y, z;     // orientation, integrated from the QMI8658 gyro
  float   ax, ay, az;     // accel, m/s^2
  uint8_t calib;          // capture count
  uint8_t touchStart;     // SoftPot position where the touch began (0 = no touch)
  uint8_t touchCurrent;   // current SoftPot position while touched (0 = no touch)
  uint8_t xrayOn;         // level bitfield, see above
};
static_assert(sizeof(OrientationPacket) == 32, "packet must be 32 bytes");
static OrientationPacket pkt = { 1, 0, 0, 0,  0, 0, 0,  0, 0, 0, 0 };

static constexpr uint8_t LEVEL_XRAY = 0x01;
static constexpr uint8_t LEVEL_DSA  = 0x02;

// Accumulated held-time, shown on the buttons. Local to the panel — a consumer can derive
// the same thing from the levels, so it is not worth spending broadcast bytes on.
static uint32_t xrayHeldMs = 0, dsaHeldMs = 0;
static uint32_t xraySinceMs = 0, dsaSinceMs = 0;   // millis() at the last press

// ---------------------------------------------------------------------------
// EVERYTHING RUNS AT ONCE. Orientation, the SoftPot and the touch buttons are all live
// together: an earlier design made them exclusive, but in use you want to see the pedal
// buttons WHILE the tool is being tracked, and having to press a button to make the cube
// move — blanking the screen to do it — was the wrong trade.
//
// The expense was never LVGL, it was pushing all 410x502 pixels over QSPI at ~10 ms a
// frame. So lv_task_handler() runs every pass (that is where touch is sampled and the
// button callbacks fire — throttling it would drop quick CAPTURE taps) and only the PUSH is
// rate-limited to SCREEN_INTERVAL_MS. Button response does not suffer: the X-ray/DSA level
// bits are set in the LVGL callback, so BLE reacts at once and only the pixels lag.
//
// panelMode now means ONLY "is the screen awake" — a power-saving sleep, not a split brain.
// The IMU keeps integrating either way.
// ---------------------------------------------------------------------------
enum PanelMode : uint8_t { SCREEN_LIVE, SCREEN_ASLEEP };
static PanelMode panelMode = SCREEN_LIVE;
static uint32_t lastWakePollMs = 0;

// How often the framebuffer is pushed to the glass. 100 ms (10 Hz) is smooth enough for
// held-seconds counters and a SoftPot bar while leaving the loop free for the 100 Hz gyro.
static constexpr uint32_t SCREEN_INTERVAL_MS = 100;

// What SLEEP does with the panel. Either way LVGL stops and no frames are pushed, so the
// CPU/QSPI saving is identical — the choice is only about the glass. The IMU is unaffected
// and keeps streaming orientation throughout:
//   0 = FREEZE (default). The last frame stays visible. This works because the CO5300 has
//       its own GRAM and self-refreshes the panel from it, so an image needs no host
//       activity to persist. Costs the panel's pixel current, but the screen stays
//       readable — you can see what mode the board is in.
//   1 = POWER DOWN via displayOff(). Darkest and lowest current, but the screen is blank
//       and gives no clue why.
#define SLEEP_BLANKS_SCREEN 0

// Only used when freezing: dim the panel to save some current while keeping it readable
// (0-255; 0 = leave brightness alone). Pixel current dominates on an AMOLED, and the frozen
// notice is mostly black already, so this is a small win — set it if you want one.
#define SLEEP_DIM_BRIGHTNESS 0
#define NORMAL_BRIGHTNESS 255

static constexpr uint32_t AUTO_SWITCH_MS = 15000;  // status -> buttons, if not cancelled
static uint32_t bootMs = 0;
static bool autoSwitchDone = false;                // also set when the user hits BACK

// ---------------------------------------------------------------------------
// Display + touch (construction copied verbatim from examples/arduino/06_LVGL_Arduino_v9)
// ---------------------------------------------------------------------------
#define DIRECT_RENDER_MODE

uint32_t screenWidth, screenHeight, bufSize;
static volatile bool frameDirty = false;   // set by my_disp_flush, consumed in loop()
lv_display_t *disp;
lv_color_t *disp_draw_buf;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

// The CO5300 addresses a 480-column GRAM, and this 410-wide panel sits 22 columns in. When
// the image is flipped, the window has to be addressed from the OTHER end of that GRAM, so
// the offset becomes 480 - 22 - 410 = 48. Arduino_GFX's own 1.64" AMOLED entry confirms the
// scheme: a 280-wide panel there uses col_offset1 20 / col_offset2 180, and 20+280+180=480.
// Rows fill the GRAM (502), so row_offset2 stays 0 like row_offset1.
//
// If the rotated panel renders shifted sideways with a dead strip down one edge, this number
// is the one to adjust -- the width of the strip is the error, in pixels.
// Where the visible 410x502 window sits inside the controller's larger GRAM. The driver
// takes _xStart/_yStart from OFFSET1 at rotation 0 and from OFFSET2 at rotation 2
// (Arduino_TFT::setRotation), so the rotated panel needs its own pair.
//
// X is solved: the GRAM is 480 wide (CO5300_TFTWIDTH), so 480 - 410 - 22 = 48.
// Y is NOT: CO5300_TFTHEIGHT claims 480 while this panel is 502 tall, so that constant is
// wrong for this part and the GRAM height is unknown. 0 assumes the panel fills the GRAM
// exactly. If the rotated screen is shifted VERTICALLY by N pixels, put N here (or -N —
// the sign follows which way it moved); a horizontal shift means COL_OFFSET2 instead.
#define PANEL_COL_OFFSET1 22
// UNUSED now: the driver only reads COL_OFFSET2 at rotation 2, and PANEL_ROTATION is
// always 0. Both values tried there (48, then 0) sheared, because each put the address
// window partly outside the panel's valid column range and the controller clamped it —
// leaving fewer columns than the 410 px we push per row, which is what the diagonal was.
// Kept at 22 to match COL_OFFSET1 so nothing here misleads if rotation is ever revisited.
#define PANEL_COL_OFFSET2 22
#define PANEL_ROW_OFFSET1 0
#define PANEL_ROW_OFFSET2 0

Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RESET, PANEL_ROTATION,
                                      LCD_WIDTH, LCD_HEIGHT,
                                      PANEL_COL_OFFSET1, PANEL_ROW_OFFSET1,
                                      PANEL_COL_OFFSET2, PANEL_ROW_OFFSET2);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);
std::unique_ptr<Arduino_IIC> FT3168(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, TP_INT,
                                                       Arduino_IIC_Touch_Interrupt));
void Arduino_IIC_Touch_Interrupt(void) { FT3168->IIC_Interrupt_Flag = true; }

// ---------------------------------------------------------------------------
// Haptic backend. 0 = the board's own MOTOR pads on GPIO18 (what is actually wired);
//                 1 = a DRV2605L breakout on the I2C bus at 0x5A (not fitted).
// ---------------------------------------------------------------------------
#define HAPTIC_USE_DRV2605 0

static constexpr int PIN_MOTOR = 18;      // MOTOR pads, top-left of the board

// An LRA is RESONANT: it only moves properly when driven at its own frequency, so DC does
// almost nothing. Switch the unipolar driver at resonance instead — typically 170-235 Hz;
// use the figure from your LRA's datasheet. Set to 0 for an ERM/coin motor, which wants DC
// and will be driven at 20 kHz (inaudible) with the duty setting the strength.
#define HAPTIC_LRA_HZ    175
#define HAPTIC_MS         40     // pulse length; a click, not a buzz
#define HAPTIC_DUTY      128     // 8-bit, 128 = 50% — full swing for a resonant drive

// An LRA's resonance is NARROW — a few Hz off and the amplitude collapses, so a mistuned
// motor feels DEAD rather than merely weak. Take HAPTIC_LRA_HZ from the datasheet if you
// have it; 175, 205 and 235 Hz are the common ones. If you do not, set this to 1: at boot
// the panel steps 140→260 Hz, holding each for 400 ms and naming it on the screen, so you
// can feel which is strongest. Put that number in HAPTIC_LRA_HZ and set this back to 0.
// Costs ~16 s of boot time while enabled, so do not leave it on.
#define HAPTIC_SWEEP 0

XPowersPMU power;
Adafruit_DRV2605 drv;
static bool hapticsOK = false;
static uint32_t hapticStopMs = 0;         // 0 = idle; see hapticService() in loop()
static bool touchOK    = false;   // touch is optional; see the bounded retry in setup()
static bool lraCalibrated = false;

// ---- LRA parameters — SET THESE FROM YOUR ACTUAL LRA'S DATASHEET ----
// An LRA is a RESONANT device: the DRV2605 drives it closed-loop at its resonant frequency
// (typically 170-235 Hz), tracking it via back-EMF. It cannot do that until auto-calibration
// has measured the motor, so an uncalibrated LRA buzzes weakly or barely moves — which is
// exactly what "LRA mode is set but it feels wrong" looks like.
// These defaults suit a common ~2 V, ~175 Hz coin LRA. If yours differs, the datasheet
// values to plug in are rated RMS voltage, maximum (overdrive) voltage and resonant freq.
static constexpr uint8_t LRA_RATED_VOLTAGE = 0x3F;   // ~2.0 V RMS
static constexpr uint8_t LRA_OD_CLAMP      = 0x89;   // ~2.7 V peak overdrive
static constexpr uint8_t LRA_DRIVE_TIME    = 0x13;   // CONTROL1: ~175 Hz (2.4 ms half-period)

// ---- IMU: QMI8658, gyro integrated on-chip here ----
// Same approach and the same verified integration math as firmware/left-mini: this is a
// 6-axis part with no magnetometer, so it does no fusion of its own. Roll/pitch would be
// gravity-correctable, yaw is not, and everything drifts slowly. A bias calibration at
// boot and a deadband keep it usable; re-center in the app.
SensorQMI8658 qmi;
static bool imuOK = false;

// Integrate no faster than this, but ALWAYS over the time that really elapsed. A fixed
// step would be wrong here: pushing a full 402 KB frame over QSPI costs ~10 ms, so the loop
// runs at ~65 Hz, and advancing the quaternion by a hard-coded 10 ms per pass made the cube
// rotate at ~65% of true speed (worse while a click tone blocks the loop).
static constexpr uint32_t IMU_INTERVAL_MS = 10;      // 100 Hz ceiling
static constexpr float    IMU_DT_MAX      = 0.25f;   // clamp after a long stall
static uint32_t lastImuMs = 0;

static float gyroBias[3] = { 0, 0, 0 };
static constexpr float GYRO_DEADBAND_DPS = 0.7f;   // below this, treat the board as still
static constexpr float DEG2RAD_F = 0.01745329252f;
static constexpr float G_MPS2    = 9.80665f;

I2SClass i2s;
static bool audioOK = false;

BLECharacteristic *orientationChar = nullptr;
volatile bool deviceConnected = false;
volatile bool connectChimePending = false;

// LVGL objects we update later
static lv_obj_t *scrStatus = nullptr, *scrButtons = nullptr;
static lv_obj_t *lblBleState = nullptr, *lblBatt = nullptr, *lblI2C = nullptr;
static lv_obj_t *lblPower = nullptr;      // POWER OFF button's label, retitled while held
static lv_obj_t *lblSoftPot = nullptr, *lblCountdown = nullptr;
static lv_obj_t *lblXray = nullptr, *lblDsa = nullptr, *lblCapture = nullptr;

// ---------------------------------------------------------------------------
// BLE broadcast
// ---------------------------------------------------------------------------
// A central connected/disconnected. Advertising is restarted on disconnect so the
// dashboard or headset can pick us up again without a power cycle.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    deviceConnected = true;
    connectChimePending = true;   // the tone is blocking, so the loop plays it, not us
    Serial.println(">> central CONNECTED");
  }
  void onDisconnect(BLEServer *) override {
    deviceConnected = false;
    Serial.println(">> central DISCONNECTED — re-advertising");
    BLEDevice::startAdvertising();
  }
};

void setupBLE() {
  Serial.println("[BLE] init (GATT peripheral)...");
  BLEDevice::init(DEVICE_NAME);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);
  orientationChar = service->createCharacteristic(ORIENTATION_UUID,
                                                  BLECharacteristic::PROPERTY_NOTIFY);
  orientationChar->addDescriptor(new BLE2902());   // lets the central subscribe
  service->start();

  // Default payload generation: service UUID in the advertisement, name in the scan
  // response (the 128-bit UUID plus this name would not fit in one 31-byte PDU).
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();
  Serial.println("[BLE] advertising as " DEVICE_NAME);
}

// ---------------------------------------------------------------------------
// Audio — short tones generated in code (no embedded PCM)
// ---------------------------------------------------------------------------
esp_err_t es8311_codec_init(void) {
  es8311_handle_t es_handle = es8311_create(I2C_NUM, ES8311_ADDRRES_0);
  if (!es_handle) return ESP_FAIL;
  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = (int)(SAMPLE_RATE * 256),
    .sample_frequency = (int)SAMPLE_RATE
  };
  ESP_RETURN_ON_ERROR(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16),
                      "panel", "es8311 init failed");
  ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency,
                                                     es_clk.sample_frequency),
                      "panel", "es8311 sample rate failed");
  ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, VOICE_VOLUME, NULL),
                      "panel", "es8311 volume failed");
  es8311_microphone_config(es_handle, false);
  return ESP_OK;
}

// Blocking, so keep tones SHORT — a long one visibly stalls the LVGL redraw.
void playTone(uint16_t freqHz, uint16_t ms, float amplitude = 0.35f) {
  if (!audioOK) return;
  const uint32_t total = (uint32_t)SAMPLE_RATE * ms / 1000;
  static int16_t buf[512 * 2];                 // 512 stereo frames per chunk
  float phase = 0.0f;
  const float step = 2.0f * PI * freqHz / (float)SAMPLE_RATE;
  uint32_t done = 0;
  while (done < total) {
    uint32_t remain = total - done;
    uint32_t n = (remain < 512) ? remain : 512;   // NB: avoid min<>(), Arduino makes min a macro
    for (uint32_t i = 0; i < n; i++) {
      // Fade the envelope in and out so the click has no hard edge (which sounds like a pop).
      float env = 1.0f;
      uint32_t pos = done + i;
      const uint32_t fade = SAMPLE_RATE / 200;                    // ~5 ms
      if (pos < fade)               env = (float)pos / fade;
      else if (total - pos < fade)  env = (float)(total - pos) / fade;
      int16_t s = (int16_t)(sinf(phase) * 32767.0f * amplitude * env);
      buf[i * 2] = s; buf[i * 2 + 1] = s;                          // both channels
      phase += step;
      if (phase > 2.0f * PI) phase -= 2.0f * PI;
    }
    i2s.write((uint8_t *)buf, n * 2 * sizeof(int16_t));
    done += n;
  }
}

void soundClick()  { playTone(2000, 25); }
void soundReady()  { playTone(880, 70); playTone(1320, 90); }   // two-tone "BLE up" chime

// ---------------------------------------------------------------------------
// Haptics
// ---------------------------------------------------------------------------
// Run the DRV2605's auto-calibration so closed-loop LRA drive actually works. The Adafruit
// library has no helper for this, but it exposes raw register access, so this is the
// datasheet procedure: set the motor parameters, enter auto-cal mode, pulse GO, wait for it
// to clear, then read the pass/fail bit. Takes up to ~1.2 s and runs once at boot — the
// results live in volatile registers, so it is redone every power-up.
bool calibrateLRA() {
  drv.writeRegister8(DRV2605_REG_MODE, DRV2605_MODE_AUTOCAL);
  drv.useLRA();                                        // FEEDBACK_CTRL: N_ERM_LRA = 1
  drv.writeRegister8(DRV2605_REG_RATEDV, LRA_RATED_VOLTAGE);
  drv.writeRegister8(DRV2605_REG_CLAMPV, LRA_OD_CLAMP);
  drv.writeRegister8(DRV2605_REG_CONTROL1, LRA_DRIVE_TIME);
  drv.writeRegister8(DRV2605_REG_CONTROL2, 0xB5);      // bidirectional, sane brake factor
  drv.writeRegister8(DRV2605_REG_CONTROL4, 0x30);      // AUTO_CAL_TIME = longest, most reliable

  drv.writeRegister8(DRV2605_REG_GO, 1);               // start calibrating
  uint32_t deadline = millis() + 2000;                 // datasheet worst case is ~1.2 s
  while (drv.readRegister8(DRV2605_REG_GO) & 0x01) {
    if (millis() > deadline) {
      Serial.println("[HAPTIC] LRA auto-calibration TIMED OUT — is the motor connected?");
      return false;
    }
    delay(10);
  }
  // STATUS bit3 DIAG_RESULT: 0 = pass, 1 = fail (open motor, or parameters out of range)
  bool pass = (drv.readRegister8(DRV2605_REG_STATUS) & 0x08) == 0;
  Serial.printf("[HAPTIC] LRA auto-calibration %s\n",
                pass ? "PASSED" : "FAILED (check the LRA wiring and the datasheet values)");
  return pass;
}

void hapticClick(uint8_t effect = 1) {   // 1 = strong click 100%
  if (!hapticsOK) return;
#if HAPTIC_USE_DRV2605
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);                 // end of sequence
  drv.go();
#else
  // Start the pulse and return immediately — hapticService() stops it. Blocking here would
  // stall an LVGL button callback for HAPTIC_MS and hitch the 100 Hz gyro with it.
  // NB: `effect` is a DRV2605 effect ID and has no meaning for a bare motor; ignored.
  LV_UNUSED(effect);
  ledcWrite(PIN_MOTOR, HAPTIC_DUTY);
  hapticStopMs = millis() + HAPTIC_MS;
#endif
}

#if !HAPTIC_USE_DRV2605 && HAPTIC_SWEEP
// Step through the plausible LRA band, pausing between steps so each is felt separately.
// Blocking, and only ever runs from setup().
void hapticSweep() {
  char msg[40];
  for (uint32_t f = 140; f <= 260; f += 5) {
    ledcChangeFrequency(PIN_MOTOR, f, 8);
    snprintf(msg, sizeof(msg), "[HAPTIC] sweep %u Hz", (unsigned)f);
    bootNote(msg, RGB565_CYAN);
    ledcWrite(PIN_MOTOR, HAPTIC_DUTY);
    delay(400);                        // long enough to judge the strength
    ledcWrite(PIN_MOTOR, 0);
    delay(250);                        // a clear gap between steps
  }
  ledcChangeFrequency(PIN_MOTOR, HAPTIC_LRA_HZ, 8);   // leave it where it was
  bootNote("[HAPTIC] sweep done", RGB565_CYAN);
}
#endif

// Ends a pulse started by hapticClick(). Cheap enough to call every loop pass.
void hapticService(uint32_t now) {
#if !HAPTIC_USE_DRV2605
  if (hapticStopMs && (int32_t)(now - hapticStopMs) >= 0) {
    ledcWrite(PIN_MOTOR, 0);
    hapticStopMs = 0;
  }
#else
  LV_UNUSED(now);
#endif
}

// Both feedback channels together — every button press goes through here.
void pressFeedback(uint8_t effect = 1) {
  hapticClick(effect);
  soundClick();
}

// ---------------------------------------------------------------------------
// Boot diagnostics drawn straight onto the panel with GFX, before LVGL exists.
//
// This board gave up native USB to the SoftPot on IO19, so Serial goes out on UART0
// (RXD/TXD) and is invisible without a USB-serial adapter. The glass is therefore the only
// console most of the time. Seeing the first banner at all is itself the most useful single
// datum during bring-up: it proves gfx->begin() worked and the board options are sane, so
// any later failure is one of the lines printed beneath it. A screen that stays black means
// the fault is at or before the display init — board settings, not the code past that point.
// ---------------------------------------------------------------------------
static int bootNoteY = 24;

void bootNote(const char *msg, uint16_t colour = RGB565_WHITE) {
  // Screen BEFORE serial, deliberately. If the USB CDC option is wrong, Serial is a port
  // that cannot enumerate and a write to it can stall; the pixels are the diagnostic that
  // has to survive that, so they go down first.
  gfx->setTextColor(colour);
  gfx->setTextSize(2);
  gfx->setCursor(12, bootNoteY);
  gfx->print(msg);
  bootNoteY += 26;
  Serial.println(msg);
}

// ---------------------------------------------------------------------------
// SoftPot — same behaviour as firmware/left-mini (touch-down capture + EMA smoothing)
// ---------------------------------------------------------------------------
static float softpotEMA = 0;
static bool  touching = false;
static int lastSoftPotRaw = 0;

void readSoftPot() {
  int raw = analogRead(PIN_SOFTPOT);
  lastSoftPotRaw = raw;
  if (raw < SOFTPOT_NOTOUCH_RAW) {
    touching = false; pkt.touchStart = 0; pkt.touchCurrent = 0; softpotEMA = 0;
    return;
  }
  uint8_t pos = (uint8_t)constrain(map(raw, SOFTPOT_NOTOUCH_RAW, 4095, 1, 255), 1, 255);
  if (!touching) { touching = true; pkt.touchStart = pos; softpotEMA = pos; }
  else           { softpotEMA = 0.6f * softpotEMA + 0.4f * pos; }
  pkt.touchCurrent = (uint8_t)softpotEMA;
}

// ---------------------------------------------------------------------------
// IMU — gyro integration, transcribed from firmware/left-mini (where the math was
// checked numerically: 90 deg/s for 1 s gives 89.998 deg about each axis with the
// correct axis sign, and the deadband holds 0 deg over 60 s of residual bias).
// ---------------------------------------------------------------------------
bool imuRead(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
  if (!qmi.getDataReady()) return false;
  float a[3], g[3];
  if (!qmi.getAccelerometer(a[0], a[1], a[2])) return false;   // g
  if (!qmi.getGyroscope(g[0], g[1], g[2])) return false;       // deg/s
  *ax = a[0]; *ay = a[1]; *az = a[2];
  *gx = g[0]; *gy = g[1]; *gz = g[2];
  return true;
}

// Average the gyro while the board sits still; that average IS the bias. Bounded the same
// three ways as the Mini's version, so a sulking IMU cannot park us on this screen.
void calibrateGyro() {
  // Runs BEFORE lv_init(), so there is no LVGL yet — and unlike the Minis this board has
  // no U8g2 "display" object at all, so this goes onto the panel with GFX via bootNote.
  //
  // It used to fillScreen(BLACK) and draw its own big "HOLD STILL". That wiped every boot
  // note already on the glass, and since this runs before setupBLE(), any later failure
  // then had NO trace left on a board whose serial console needs a UART adapter. Appending
  // instead of clearing keeps the whole boot log readable.
  bootNote(">> gyro bias: HOLD STILL", RGB565_YELLOW);

  const int      WANTED   = 300;
  const int      MAX_BAD  = 60;                  // getDataReady() can legitimately say no
  const uint32_t DEADLINE = millis() + 4000;
  double sum[3] = { 0, 0, 0 };
  int good = 0, badRun = 0;
  while (good < WANTED && millis() < DEADLINE) {
    float ax, ay, az, gx, gy, gz;
    if (imuRead(&ax, &ay, &az, &gx, &gy, &gz)) {
      sum[0] += gx; sum[1] += gy; sum[2] += gz;
      good++; badRun = 0;
    } else if (++badRun >= MAX_BAD) {
      Serial.println("[IMU] no data during calibration — aborting");
      break;
    }
    delay(2);
  }
  if (good == 0) {
    imuOK = false;
    Serial.println("[IMU] calibration FAILED — running without orientation");
    return;
  }
  for (int i = 0; i < 3; i++) gyroBias[i] = (float)(sum[i] / good);   // divide by REAL count
  Serial.printf("[IMU] gyro bias (deg/s): %+.3f %+.3f %+.3f  (%d/%d samples)\n",
                gyroBias[0], gyroBias[1], gyroBias[2], good, WANTED);
}

// One integration step over `dt` seconds: q += 0.5 * q (x) omega * dt, renormalised.
void updateIMU(float dt) {
  float rax, ray, raz, rgx, rgy, rgz;
  if (!imuRead(&rax, &ray, &raz, &rgx, &rgy, &rgz)) return;

#if PANEL_ROTATE_180
  // The board is physically turned 180 degrees about the screen normal (Z), so its X and Y
  // axes point opposite to the other panel's. That rotation maps (x, y, z) -> (-x, -y, z).
  // Without this the display and touch are rotated but the gyro is not, and the cube's
  // pitch and roll come out BACKWARDS while yaw looks fine — which is exactly how a missing
  // frame correction presents. Applied to the raw readings so that bias subtraction, the
  // deadband, the integrator and the packet all see one consistent frame.
  rgx = -rgx;  rgy = -rgy;      // rgz unchanged: Z is the rotation axis
  rax = -rax;  ray = -ray;      // same rotation applies to the accelerometer
#endif

  float gx = rgx - gyroBias[0];                  // deg/s, de-biased
  float gy = rgy - gyroBias[1];
  float gz = rgz - gyroBias[2];
  if (fabsf(gx) < GYRO_DEADBAND_DPS) gx = 0;     // a still board must not creep
  if (fabsf(gy) < GYRO_DEADBAND_DPS) gy = 0;
  if (fabsf(gz) < GYRO_DEADBAND_DPS) gz = 0;
  gx *= DEG2RAD_F; gy *= DEG2RAD_F; gz *= DEG2RAD_F;

  float qw = pkt.w, qx = pkt.x, qy = pkt.y, qz = pkt.z;
  const float h = 0.5f * dt;
  float nw = qw + (-qx * gx - qy * gy - qz * gz) * h;
  float nx = qx + ( qw * gx + qy * gz - qz * gy) * h;
  float ny = qy + ( qw * gy - qx * gz + qz * gx) * h;
  float nz = qz + ( qw * gz + qx * gy - qy * gx) * h;

  float norm = sqrtf(nw * nw + nx * nx + ny * ny + nz * nz);
  if (norm > 1e-6f) {
    pkt.w = nw / norm; pkt.x = nx / norm; pkt.y = ny / norm; pkt.z = nz / norm;
  }
  pkt.ax = rax * G_MPS2; pkt.ay = ray * G_MPS2; pkt.az = raz * G_MPS2;   // packet wants m/s^2
}

// ---------------------------------------------------------------------------
// I2C scan — bring-up aid; expect 0x38 touch, 0x34 AXP2101, 0x6A QMI8658,
// 0x51 RTC and 0x5A DRV2605L.
// ---------------------------------------------------------------------------
static char i2cSummary[64] = "scanning...";
void i2cScan() {
  Serial.println("[I2C] scanning bus...");
  uint8_t found = 0;
  size_t used = 0;
  i2cSummary[0] = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      const char *who = (addr == 0x38) ? " touch" : (addr == 0x34) ? " pmu"
                      : (addr == 0x5A) ? " drv"   : (addr == 0x51) ? " rtc" : "";
      Serial.printf("[I2C]   device at 0x%02X%s\n", addr, who);
      if (used < sizeof(i2cSummary) - 8)
        used += snprintf(i2cSummary + used, sizeof(i2cSummary) - used, "%02X ", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("[I2C]   NONE found");
    snprintf(i2cSummary, sizeof(i2cSummary), "none!");
  }
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
// Power the panel down and hand the CPU to the IMU. Draws a parting notice with GFX (not
// LVGL — LVGL stops running in this mode) so the screen going dark is not a mystery.
void screenSleep() {
  panelMode = SCREEN_ASLEEP;

  // Drawn with GFX, not LVGL — LVGL stops running below, so this is the image that will be
  // left on the glass. It doubles as the mode indicator when the panel is frozen.
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(40, 170);
  gfx->print("SCREEN ASLEEP");
  gfx->setTextSize(2);
  gfx->setCursor(40, 220);
  gfx->print(HAND_LABEL " hand - still tracking");
  gfx->setCursor(40, 250);
  gfx->print("tap to wake");

#if SLEEP_BLANKS_SCREEN
  delay(700);                       // long enough to read before it goes dark
  gfx->displayOff();                // blank: lowest current, no clue why
#elif SLEEP_DIM_BRIGHTNESS > 0
  gfx->setBrightness(SLEEP_DIM_BRIGHTNESS);     // frozen but dimmed
#endif
  // Nothing more is pushed either way: LVGL is not run and frameDirty stays clear, so the
  // CO5300 just keeps self-refreshing whatever is in its GRAM.
  frameDirty = false;
  lastImuMs = millis();             // fresh dt, so the first step is not a lurch
  lastWakePollMs = millis();
  Serial.println(">> screen asleep: IMU, SoftPot and BLE all still running");
}

void screenWake() {
  panelMode = SCREEN_LIVE;
#if SLEEP_BLANKS_SCREEN
  gfx->displayOn();
#elif SLEEP_DIM_BRIGHTNESS > 0
  gfx->setBrightness(NORMAL_BRIGHTNESS);
#endif
  // Wake onto the STATUS screen, never straight onto the pedals: the tap that woke us is
  // very likely still under a finger, and landing on the button screen would fire X-ray
  // the instant LVGL resumes. One extra tap to reach the pedals is the safe trade.
  autoSwitchDone = true;            // and don't let the 15 s timer jump us there either
  lv_scr_load(scrStatus);
  lv_obj_invalidate(scrStatus);     // nothing was drawn while we were away
  frameDirty = true;
  Serial.println(">> PEDAL mode: screen on (status), IMU paused");
}

void showButtons() {
  autoSwitchDone = true;             // never auto-hop again this session
  lv_scr_load(scrButtons);
}

static void back_cb(lv_event_t *e) {
  LV_UNUSED(e);
  pressFeedback(24);                 // 24 = sharp tick, lighter than a pedal press
  autoSwitchDone = true;             // BACK must stick: don't bounce to buttons in 15 s
  // Top-RIGHT mirrors BACK: put the SCREEN to sleep to save power. Tracking, the SoftPot
  // and BLE all keep running — only the glass stops. Created last for the same reason BACK
  // is: the topmost object wins the click.
  lv_obj_t *trk = lv_button_create(scrButtons);
  lv_obj_set_size(trk, 104, 52);
  lv_obj_align(trk, LV_ALIGN_TOP_RIGHT, -10, 10);
  lv_obj_set_style_bg_color(trk, lv_color_hex(0x1E3A5F), LV_PART_MAIN);
  lv_obj_set_style_radius(trk, 10, LV_PART_MAIN);
  lv_obj_add_event_cb(trk, [](lv_event_t *e) {
    LV_UNUSED(e);
    pressFeedback(24);
    screenSleep();
  }, LV_EVENT_CLICKED, NULL);
  lv_obj_t *trkLbl = lv_label_create(trk);
  lv_label_set_text(trkLbl, "SLEEP");
  lv_obj_set_style_text_font(trkLbl, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_center(trkLbl);

  lv_scr_load(scrStatus);
}

// X-ray and DSA are HOLD controls (dead-man switch, like the real pedals): the level is
// true for exactly as long as a finger is down. Capture is a tap.
static void xray_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED) {
    pkt.xrayOn |= LEVEL_XRAY;
    xraySinceMs = millis();
    pressFeedback(1);
    Serial.println(">> X-RAY down");
  } else if (code == LV_EVENT_RELEASED) {
    pkt.xrayOn &= ~LEVEL_XRAY;
    if (xraySinceMs) { xrayHeldMs += millis() - xraySinceMs; xraySinceMs = 0; }
    Serial.println(">> X-RAY up");
  }
}

static void dsa_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED) {
    pkt.xrayOn |= LEVEL_DSA;
    dsaSinceMs = millis();
    pressFeedback(1);
    Serial.println(">> DSA run START");
  } else if (code == LV_EVENT_RELEASED) {
    pkt.xrayOn &= ~LEVEL_DSA;
    if (dsaSinceMs) { dsaHeldMs += millis() - dsaSinceMs; dsaSinceMs = 0; }
    Serial.println(">> DSA run END");
  }
}

static void capture_cb(lv_event_t *e) {
  LV_UNUSED(e);
  pkt.calib++;                      // byte 28 = capture count (Mini convention)
  pressFeedback(1);
  Serial.printf(">> CAPTURE #%u\n", pkt.calib);
}

static lv_obj_t *makePedalButton(lv_obj_t *parent, const char *text, lv_color_t colour,
                                 int yOffset, int height, lv_obj_t **labelOut,
                                 lv_event_cb_t cb, lv_event_code_t filter) {
  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_size(btn, LCD_WIDTH - 16, height);
  lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, yOffset);
  lv_obj_set_style_bg_color(btn, colour, LV_PART_MAIN);
  lv_obj_set_style_radius(btn, 14, LV_PART_MAIN);
  lv_obj_add_event_cb(btn, cb, filter, NULL);
  if (filter == LV_EVENT_PRESSED)          // hold buttons need both edges
    lv_obj_add_event_cb(btn, cb, LV_EVENT_RELEASED, NULL);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, text);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_32, LV_PART_MAIN);
  lv_obj_center(lbl);
  *labelOut = lbl;
  return btn;
}

// Cut every rail except VRTC — the same endpoint as holding the side button, but reachable
// from the screen. Say so on serial FIRST: with USB connected the AXP2101 restarts on VBUS,
// so the board bounces instead of staying off, and without this line that reads as a crash.
void powerOff() {
  Serial.println(">> POWER OFF requested from the status screen");
  if (power.isVbusIn())
    Serial.println("[PMU] USB is connected — VBUS will restart the PMU immediately. "
                   "Unplug USB if you want it to stay off.");
  hapticClick(1);
  delay(120);                       // let the click finish before the rails drop
  gfx->fillScreen(RGB565_BLACK);    // the CO5300 self-refreshes; don't leave a stale frame
  power.shutdown();
}

static lv_obj_t *makeStatusLine(lv_obj_t *parent, int y, const char *initial) {
  lv_obj_t *lbl = lv_label_create(parent);
  lv_label_set_text(lbl, initial);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xE8E8EA), LV_PART_MAIN);
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 14, y);
  return lbl;
}

void buildUI() {
  // ---- status screen ----
  scrStatus = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scrStatus, lv_color_hex(0x14161A), LV_PART_MAIN);

  lv_obj_t *title = lv_label_create(scrStatus);
  lv_label_set_text(title, "PEDAL PANEL");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_set_style_text_color(title, lv_color_hex(0x4ADE80), LV_PART_MAIN);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 24);

  lblBleState  = makeStatusLine(scrStatus, 90,  "BLE: starting...");
  lblBatt      = makeStatusLine(scrStatus, 130, "Batt: --");
  lblI2C       = makeStatusLine(scrStatus, 170, "I2C: --");
  lblSoftPot   = makeStatusLine(scrStatus, 210, "SoftPot: --");
  lblCountdown = makeStatusLine(scrStatus, 260, "");

  lv_obj_t *hint = lv_label_create(scrStatus);
  lv_label_set_text(hint, "Tap anywhere for the pedals");
  lv_obj_set_style_text_font(hint, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_style_text_color(hint, lv_color_hex(0x9AA0A8), LV_PART_MAIN);
  // Left, not centred: the POWER OFF button takes the bottom-right corner below.
  lv_obj_align(hint, LV_ALIGN_BOTTOM_LEFT, 14, -30);
  lv_obj_add_flag(scrStatus, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(scrStatus, [](lv_event_t *e) { LV_UNUSED(e); showButtons(); },
                      LV_EVENT_CLICKED, NULL);

  // POWER OFF — HOLD, don't tap. The screen itself is a click target that jumps to the
  // pedals (just above), so this is created LAST: LVGL hands the touch to the topmost
  // object, exactly as BACK does on the button screen. And it is a long press because a
  // stray tap here would kill the board mid-session — the hold time is set on the indev in
  // setup(), since LVGL's 400 ms default is far too twitchy for a shutdown.
  lv_obj_t *pwr = lv_button_create(scrStatus);
  lv_obj_set_size(pwr, 136, 60);
  lv_obj_align(pwr, LV_ALIGN_BOTTOM_RIGHT, -14, -14);
  lv_obj_set_style_bg_color(pwr, lv_color_hex(0x7F1D1D), LV_PART_MAIN);
  lv_obj_set_style_radius(pwr, 10, LV_PART_MAIN);
  lblPower = lv_label_create(pwr);
  lv_label_set_text(lblPower, LV_SYMBOL_POWER "  OFF");
  lv_obj_set_style_text_font(lblPower, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_center(lblPower);

  // The label narrates the hold, so a press that does nothing is not a mystery.
  lv_obj_add_event_cb(pwr, [](lv_event_t *e) {
    LV_UNUSED(e);
    lv_label_set_text(lblPower, "KEEP HOLDING");
  }, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(pwr, [](lv_event_t *e) {
    LV_UNUSED(e);
    lv_label_set_text(lblPower, LV_SYMBOL_POWER "  OFF");   // released too early
  }, LV_EVENT_RELEASED, NULL);
  lv_obj_add_event_cb(pwr, [](lv_event_t *e) {
    LV_UNUSED(e);
    powerOff();
  }, LV_EVENT_LONG_PRESSED, NULL);

  // ---- button screen ----
  scrButtons = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scrButtons, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_pad_all(scrButtons, 0, LV_PART_MAIN);
  lv_obj_clear_flag(scrButtons, LV_OBJ_FLAG_SCROLLABLE);

  const int h = (LCD_HEIGHT - 4 * 8) / 3;        // three buttons + even gaps
  makePedalButton(scrButtons, "X-RAY 0s",   lv_color_hex(0x14532D), 8,
                  h, &lblXray, xray_cb, LV_EVENT_PRESSED);
  makePedalButton(scrButtons, "DSA 0s",     lv_color_hex(0x7C2D12), 8 + h + 8,
                  h, &lblDsa, dsa_cb, LV_EVENT_PRESSED);
  makePedalButton(scrButtons, "CAPTURE 0",  lv_color_hex(0x1E3A5F), 8 + 2 * (h + 8),
                  h, &lblCapture, capture_cb, LV_EVENT_CLICKED);

  // BACK sits ON TOP of the X-ray button; LVGL gives the click to the topmost object,
  // so it is created last and therefore wins.
  lv_obj_t *back = lv_button_create(scrButtons);
  lv_obj_set_size(back, 96, 52);
  lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
  lv_obj_set_style_bg_color(back, lv_color_hex(0x2A2E36), LV_PART_MAIN);
  lv_obj_set_style_radius(back, 10, LV_PART_MAIN);
  lv_obj_add_event_cb(back, back_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *backLbl = lv_label_create(back);
  lv_label_set_text(backLbl, LV_SYMBOL_LEFT " Back");
  lv_obj_set_style_text_font(backLbl, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_center(backLbl);

  lv_scr_load(scrStatus);
}

// Refresh both screens' text. Runs on an LVGL timer so it cannot fight the redraw.
static void ui_tick(lv_timer_t *t) {
  LV_UNUSED(t);
  char buf[48];

  // live held-time = accumulated + the run in progress
  uint32_t xs = xrayHeldMs + (xraySinceMs ? millis() - xraySinceMs : 0);
  uint32_t ds = dsaHeldMs  + (dsaSinceMs  ? millis() - dsaSinceMs  : 0);
  snprintf(buf, sizeof(buf), "X-RAY %us", (unsigned)(xs / 1000));  lv_label_set_text(lblXray, buf);
  snprintf(buf, sizeof(buf), "DSA %us",   (unsigned)(ds / 1000));  lv_label_set_text(lblDsa, buf);
  snprintf(buf, sizeof(buf), "CAPTURE %u", pkt.calib);          lv_label_set_text(lblCapture, buf);

  if (lv_scr_act() == scrStatus) {
    lv_label_set_text(lblBleState, deviceConnected ? "BLE: CONNECTED  (" DEVICE_NAME ")"
                                                  : "BLE: advertising as " DEVICE_NAME);
    if (power.isBatteryConnect())
      // Charging STATE, not just the voltage: "is it actually charging?" is the question you
      // have while holding a USB cable, and rising millivolts answer it far too slowly.
      snprintf(buf, sizeof(buf), "Batt: %d%%  %dmV  %s", power.getBatteryPercent(),
               power.getBattVoltage(),
               power.isCharging()  ? "CHARGING"
               : power.isVbusIn()  ? "USB (full)"
                                   : "on batt");
    else
      snprintf(buf, sizeof(buf), "Batt: not detected");
    lv_label_set_text(lblBatt, buf);

    // Picked before the call, not with a #if inside the argument list: that form puts a
    // closing paren in each branch and makes the file look unbalanced to anything reading
    // it without a preprocessor.
#if HAPTIC_USE_DRV2605
    const char *hapTag = !hapticsOK ? "(no LRA)"
                       : lraCalibrated ? "(LRA cal)" : "(LRA uncal!)";
#else
    const char *hapTag = hapticsOK ? "(motor IO18)" : "(no motor)";
#endif
    snprintf(buf, sizeof(buf), "I2C: %s%s", i2cSummary, hapTag);
    lv_label_set_text(lblI2C, buf);

    if (pkt.touchCurrent > 0)
      snprintf(buf, sizeof(buf), "SoftPot: %u  (raw %d)", pkt.touchCurrent, lastSoftPotRaw);
    else
      snprintf(buf, sizeof(buf), "SoftPot: -- (raw %d)", lastSoftPotRaw);
    lv_label_set_text(lblSoftPot, buf);

    if (autoSwitchDone) {
      lv_label_set_text(lblCountdown, "");
    } else if (!deviceConnected) {
      lv_label_set_text(lblCountdown, "Waiting for a central...");
    } else {
      uint32_t up = millis() - bootMs;
      uint32_t left = up >= AUTO_SWITCH_MS ? 0 : (AUTO_SWITCH_MS - up) / 1000 + 1;
      snprintf(buf, sizeof(buf), "Pedals in %us...", (unsigned)left);
      lv_label_set_text(lblCountdown, buf);
    }
  }
}

// ---------------------------------------------------------------------------
// LVGL plumbing (copied from examples/arduino/06_LVGL_Arduino_v9)
// ---------------------------------------------------------------------------
uint32_t millis_cb(void) { return millis(); }

// LVGL tells us here that it rendered. In DIRECT mode we push the whole buffer, but only
// ONCE per render rather than every loop pass — see frameDirty in loop().
#if PANEL_ROTATE_180 && PANEL_FLIP_Y_IN_SOFTWARE
// Reverse the frame buffer's row order in place — the vertical half of a 180, in software.
// Row-granular, so nothing inside a row moves: combined with the panel's own X flip that
// yields a full 180. One 820-byte scratch row, no allocation.
static void flipBufferRows() {
  static uint16_t rowTmp[LCD_WIDTH];
  uint16_t *buf = (uint16_t *)disp_draw_buf;
  const size_t rowBytes = (size_t)LCD_WIDTH * sizeof(uint16_t);
  for (int y = 0; y < LCD_HEIGHT / 2; y++) {
    uint16_t *a = buf + (size_t)y * LCD_WIDTH;
    uint16_t *b = buf + (size_t)(LCD_HEIGHT - 1 - y) * LCD_WIDTH;
    memcpy(rowTmp, a, rowBytes);
    memcpy(a, b, rowBytes);
    memcpy(b, rowTmp, rowBytes);
  }
}
#endif

void my_disp_flush(lv_display_t *d, const lv_area_t *area, uint8_t *px_map) {
#ifndef DIRECT_RENDER_MODE
  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
#else
  frameDirty = true;
#endif
  lv_disp_flush_ready(d);
}

void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
  if (!touchOK) { data->state = LV_INDEV_STATE_REL; return; }   // never came up; see setup()
  int32_t touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  int32_t touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
  if (FT3168->IIC_Interrupt_Flag == true) {
    FT3168->IIC_Interrupt_Flag = false;
    data->state = LV_INDEV_STATE_PR;
#if PANEL_ROTATE_180
    // Per-axis, to match what the display is ACTUALLY doing — see PANEL_TOUCH_FLIP_* above.
    // The FT3168 reports raw panel coordinates and knows nothing about any flip, so every
    // axis the glass flips has to be flipped here too, and one the glass does NOT flip must
    // be left alone or touch ends up mirrored in that axis.
#if PANEL_TOUCH_FLIP_X
    data->point.x = (LCD_WIDTH  - 1) - touchX;
#else
    data->point.x = touchX;
#endif
#if PANEL_TOUCH_FLIP_Y
    data->point.y = (LCD_HEIGHT - 1) - touchY;
#else
    data->point.y = touchY;
#endif
#else
    data->point.x = touchX;
    data->point.y = touchY;
#endif
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void rounder_event_cb(lv_event_t *e) {
  lv_area_t *area = (lv_area_t *)lv_event_get_param(e);
  area->x1 = (area->x1 >> 1) << 1;
  area->y1 = (area->y1 >> 1) << 1;
  area->x2 = ((area->x2 >> 1) << 1) + 1;
  area->y2 = ((area->y2 >> 1) << 1) + 1;
}

// ---------------------------------------------------------------------------
void setup() {
  // ORDER IS LOAD-BEARING — do not "improve" this again.
  //
  // The delay() below is the AMOLED's settle time after power-on. Its rails come up through
  // the AXP2101, and gfx->begin() must not run before they are ready: called at ~0 ms the
  // display init fails, the panel stays dark, and every bootNote() afterwards writes to a
  // dead screen. That is precisely what happened when this block was reordered to put
  // gfx->begin() first — a working board went black with no diagnostics at all.
  //
  // Serial goes first, as it always did. It is UART0 on RXD/TXD (44/43), never USB CDC:
  // IO19 is the SoftPot wiper so the USB port cannot enumerate, and the #error guard above
  // rejects a CDC-on-boot build rather than letting Serial become a dead port.
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== Touchscreen Pedal Panel (X-ray / DSA / Capture) ===");

  if (!gfx->begin()) Serial.println("[GFX] begin() failed!");
#if PANEL_ROTATE_180
  // Flip in hardware ourselves rather than via setRotation(2) — see PANEL_MADCTL_180.
  // begin() has already sent the init sequence, so this lands on top of it.
  bus->beginWrite();
  bus->writeC8D8(0x36 /* MADCTL */, PANEL_MADCTL_180);
  bus->endWrite();
  Serial.printf("[GFX] MADCTL 0x%02X written for 180 mount\n", PANEL_MADCTL_180);
#endif
  gfx->fillScreen(RGB565_BLACK);

  // First thing on the glass. If this appears the display is alive, and any real fault is
  // one of the lines printed beneath it.
  bootNote(HAND_LABEL " PANEL  booting", RGB565_GREEN);

  Wire.begin(IIC_SDA, IIC_SCL);

  // Bounded, and NON-fatal. This was `while (FT3168->begin() == false)` — an unbounded retry
  // sitting before lv_init() and setupBLE(), so a touch fault hung setup() and the board
  // presented as dead: black screen, nothing advertising, no way to tell it apart from a bad
  // flash. It was also the odd one out — a missing DRV2605L and a dead QMI8658 are already
  // survivable here. Without touch the panel still streams orientation and the SoftPot,
  // which is most of its job; only the on-screen buttons are lost.
  for (int attempt = 1; attempt <= 3 && !touchOK; attempt++) {
    touchOK = FT3168->begin();
    if (!touchOK) {
      bootNote("[TOUCH] FT3168 init failed", RGB565_YELLOW);
      delay(400);
    }
  }
  if (touchOK) {
    bootNote("[TOUCH] FT3168 ready");
    FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                   FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);
  } else {
    bootNote("[TOUCH] NO TOUCH - buttons dead,", RGB565_RED);
    bootNote("        tracker still works", RGB565_RED);
  }

  i2cScan();

  // Haptics. Optional at runtime either way: no motor must not stop the panel working.
#if HAPTIC_USE_DRV2605
  hapticsOK = drv.begin(&Wire);
  if (hapticsOK) {
    lraCalibrated = calibrateLRA();       // must happen BEFORE normal playback mode
    drv.selectLibrary(6);                 // 6 = LRA effect library
    drv.useLRA();
    drv.setMode(DRV2605_MODE_INTTRIG);    // back to internal trigger for setWaveform/go
    Serial.printf("[HAPTIC] DRV2605L ready (LRA%s)\n",
                  lraCalibrated ? ", calibrated" : ", UNCALIBRATED — expect weak clicks");
    bootNote(lraCalibrated ? "[HAPTIC] LRA calibrated" : "[HAPTIC] LRA UNCALIBRATED",
             lraCalibrated ? RGB565_WHITE : RGB565_YELLOW);
  } else {
    bootNote("[HAPTIC] no DRV2605L", RGB565_YELLOW);
  }
#else
  // Onboard MOTOR pads on GPIO18. ledcAttach is the core-3.x API and takes the PIN, not a
  // channel number. 8-bit duty; the frequency is the LRA's resonance, or 20 kHz for an ERM
  // (well above hearing, so a DC motor does not whine).
  hapticsOK = ledcAttach(PIN_MOTOR, HAPTIC_LRA_HZ > 0 ? HAPTIC_LRA_HZ : 20000, 8);
  if (hapticsOK) {
    ledcWrite(PIN_MOTOR, 0);              // make sure it is not left running
    Serial.printf("[HAPTIC] MOTOR pads on GPIO%d at %d Hz\n",
                  PIN_MOTOR, HAPTIC_LRA_HZ > 0 ? HAPTIC_LRA_HZ : 20000);
    bootNote("[HAPTIC] motor on GPIO18", RGB565_WHITE);
    // One pulse at boot, proving the wiring by feel. BLOCKING on purpose: hapticService()
    // does not run until loop() starts, and setup() still has gyro calibration to get
    // through -- the non-blocking path would leave the motor buzzing for all of it.
    ledcWrite(PIN_MOTOR, HAPTIC_DUTY);
    delay(HAPTIC_MS);
    ledcWrite(PIN_MOTOR, 0);
#if HAPTIC_SWEEP
    hapticSweep();                      // find the resonance by feel; see HAPTIC_SWEEP
#endif
  } else {
    Serial.println("[HAPTIC] ledcAttach(GPIO18) FAILED — no haptics");
    bootNote("[HAPTIC] ledcAttach FAILED", RGB565_YELLOW);
  }
#endif

  // IMU. Optional at runtime: a dead QMI8658 should still leave a working pedal panel.
  imuOK = qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);
  if (imuOK) {
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz,
                            SensorQMI8658::LPF_MODE_0);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_224_2Hz,
                        SensorQMI8658::LPF_MODE_0);
    qmi.enableAccelerometer();
    qmi.enableGyroscope();
    bootNote("[IMU] QMI8658 ready");
    calibrateGyro();
    lastImuMs = millis();
  } else {
    bootNote("[IMU] no QMI8658 - no rotation", RGB565_YELLOW);
  }

  // Battery telemetry (display rails are already on; the PMU is only read here).
  power.enableBattDetection();
  power.enableBattVoltageMeasure();
  power.enableSystemVoltageMeasure();

  // Charging. The AXP2101 charges from VBUS whenever USB is connected with the battery
  // left plugged in — that is the designed use of this board and needs no code. (Contrast
  // the SuperMini pedals and the C3 Minis, which have NO charger: on those, battery + USB
  // together back-feeds the cell.) What does need code is the target voltage: that register
  // also offers 4.35 V and 4.4 V, and a standard LiPo wants 4.2 V, so leaving it at an
  // unverified power-on default risks a slow overcharge. Waveshare's example 05 sets it
  // explicitly for the same reason.
  //
  // Charge CURRENT is deliberately NOT set here: the sane value is about half the cell's
  // capacity, so it belongs with the battery, not the board. To pin it, add e.g.
  //   power.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);   // ~500 mAh cell
  if (!power.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2)) {
    Serial.println("[PMU] WARNING: could not set the 4.2 V charge target — do not leave a "
                   "battery charging unattended until this is resolved");
  }
  bootNote("[PMU] charger configured");
  Serial.printf("[PMU] charge target: %d (want %d = 4.2V), battery %s\n",
                power.getChargeTargetVoltage(), XPOWERS_AXP2101_CHG_VOL_4V2,
                power.isBatteryConnect() ? "detected" : "NOT detected");

  // Long-press the side button to power the board down.
  //
  // That button is wired to the AXP2101's PWRON pin, not to the ESP32, so shutdown is the
  // PMU's decision alone. Out of reset the AXP2101 does NOT act on a long press — Waveshare's
  // own examples enable only the SHORT-press IRQ and expect the application to handle the
  // button — which is why it appeared dead. These three writes arm it in hardware, so it
  // still works even if this firmware has hung.
  //
  // Order matters: set the hold time, choose OFF over RESTART, then arm. Skipping the middle
  // call risks inheriting PWROFF_EN bit0 = restart, where a long press REBOOTS instead —
  // indistinguishable from "nothing happened" once the board is back on screen.
  power.setPowerKeyPressOffTime(XPOWERS_POWEROFF_6S);   // 4S / 6S / 8S / 10S
  power.setLongPressPowerOFF();                         // long press = OFF, not restart
  power.enableLongPressShutdown();
  bootNote("[PMU] power key armed");

  // Read back: these are I2C writes, and a silent failure would look exactly like the
  // original symptom. Register value 0..3 maps to 4/6/8/10 s.
  {
    const uint8_t offOpt = power.getPowerKeyPressOffTime();
    Serial.printf("[PMU] long-press power-off armed: hold %d s%s\n",
                  4 + offOpt * 2,
                  (offOpt == XPOWERS_POWEROFF_6S) ? "" : "   <-- NOT what was written");
    Serial.println("[PMU] it will not stay off while USB is connected — VBUS restarts the "
                   "PMU. Unplug USB first, then hold the button.");
  }

  // SoftPot on IO19 (ADC2). Internal pulldown so an untouched, floating wiper reads ~0.
  //
  // FIRST take the pad back from the USB PHY. IO19/IO20 are the native USB D-/D+ pair, and
  // on the ESP32-S3 the USB-Serial-JTAG PHY drives them out of reset — "USB CDC On Boot:
  // Disabled" only changes where Serial goes, it does NOT release the pads. Left attached,
  // the ADC measures the PHY instead of the strip and reads a constant 0. gpio_reset_pin()
  // special-cases these two pins and disconnects the PHY. It must run before the ADC setup,
  // since it also clears pin config.
  gpio_reset_pin((gpio_num_t)PIN_SOFTPOT);
  bootNote("[ADC] IO19 taken from USB PHY");

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOFTPOT, ADC_11db);
  analogRead(PIN_SOFTPOT);                        // let the core configure the pin first
  gpio_pulldown_en((gpio_num_t)PIN_SOFTPOT);

  // Audio
  pinMode(PIN_AUDIO_PA, OUTPUT);
  digitalWrite(PIN_AUDIO_PA, HIGH);               // enable the speaker amplifier
  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_DIN, PIN_I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                 I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("[AUDIO] I2S init failed — running silent");
  } else if (es8311_codec_init() != ESP_OK) {
    Serial.println("[AUDIO] ES8311 init failed — running silent");
  } else {
    audioOK = true;
    bootNote("[AUDIO] ES8311 ready");
  }

  // ---- LVGL ----
  // Last note before LVGL owns the glass. If the boot log ends here, the failure is in the
  // LVGL/display setup below; if the log vanishes and a UI appears, setup() got through.
  bootNote("[LVGL] starting...");
  lv_init();
  lv_tick_set_cb(millis_cb);

  screenWidth = gfx->width();
  screenHeight = gfx->height();
#ifdef DIRECT_RENDER_MODE
  bufSize = screenWidth * screenHeight;
#else
  bufSize = screenWidth * 50;
#endif
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
  if (!disp_draw_buf) {
    // The other way this board goes black with no BLE: setup() returns here, before
    // setupBLE(). 402 KiB is a big ask, so say what to change rather than just failing.
    bootNote("[LVGL] draw buffer alloc FAILED", RGB565_RED);
    bootNote("Check Tools: PSRAM enabled?", RGB565_YELLOW);
    bootNote("Partition scheme big enough?", RGB565_YELLOW);
    return;
  }

  disp = lv_display_create(screenWidth, screenHeight);
  lv_display_set_flush_cb(disp, my_disp_flush);
#ifdef DIRECT_RENDER_MODE
  lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_DIRECT);
#else
  lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);
  // 1.2 s, up from LVGL's 400 ms default: the only long-press consumer is POWER OFF on the
  // status screen, and 400 ms is short enough that a slow tap would shut the board down.
  // Nothing else uses LV_EVENT_LONG_PRESSED, so this costs the pedal buttons nothing.
  lv_indev_set_long_press_time(indev, 1200);
  lv_display_add_event_cb(disp, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);

  buildUI();
  lv_timer_create(ui_tick, 250, NULL);

  bootNote("[UI] built - starting BLE");
  setupBLE();

  bootMs = millis();
  Serial.println("=== setup complete ===");
}

void loop() {
  const uint32_t now = millis();

  if (panelMode == SCREEN_LIVE) {
    // Every pass: this is where touch is sampled and the button callbacks fire, so it must
    // NOT be throttled or a quick CAPTURE tap gets dropped. It is cheap when nothing is
    // invalidated — the costly half is the push below.
    lv_task_handler();
#ifdef DIRECT_RENDER_MODE
    // Pushing all 410x502 costs ~10 ms of QSPI. Only when LVGL actually drew something AND
    // at most every SCREEN_INTERVAL_MS, so the 100 Hz gyro gets the rest of the loop.
    // frameDirty is NOT cleared while we wait, so a render is never dropped, only deferred.
    static uint32_t lastPushMs = 0;
    if (frameDirty && (now - lastPushMs >= SCREEN_INTERVAL_MS)) {
      frameDirty = false;
      lastPushMs = now;
#if PANEL_ROTATE_180 && PANEL_FLIP_Y_IN_SOFTWARE
      // Flip, push, flip back. The restore is not optional: in DIRECT render mode LVGL keeps
      // this buffer as its canvas and only repaints changed areas, so leaving it reversed
      // would make every later partial render land in the wrong place.
      flipBufferRows();
      gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
      flipBufferRows();
#else
      gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)disp_draw_buf, screenWidth, screenHeight);
#endif
    }
#endif
  } else {
    // Asleep: no LVGL, no pixels — but the IMU below keeps running. Poll the touch
    // controller slowly, only to notice a tap asking for the screen back; the ISR has
    // already latched the flag for us.
    if (now - lastWakePollMs >= 100) {              // 10 Hz is plenty to catch a finger
      lastWakePollMs = now;
      if (touchOK && FT3168->IIC_Interrupt_Flag) {
        FT3168->IIC_Interrupt_Flag = false;
        screenWake();
      }
    }
  }

  // Integrate the gyro at up to 100 Hz, over the time that ACTUALLY elapsed. ALWAYS — the
  // screen being awake, asleep, or showing the pedal buttons makes no difference now.
  if (imuOK && (now - lastImuMs >= IMU_INTERVAL_MS)) {
    float dt = (now - lastImuMs) * 0.001f;
    if (dt > IMU_DT_MAX) dt = IMU_DT_MAX;         // don't lurch after a stall (audio, boot)
    lastImuMs = now;
    updateIMU(dt);
  }

  static uint32_t lastNotifyMs = 0;
  if (now - lastNotifyMs >= 20) {                 // 50 Hz, same cadence as the trackers
    lastNotifyMs = now;
    // Unconditional, like everything else now. It was briefly gated to the tracker half,
    // which made the status screen's raw readout permanently 0 — the gate hid the very
    // number it exists to show, and saved nothing: one analogRead costs microseconds.
    readSoftPot();
    if (deviceConnected && orientationChar) {
      orientationChar->setValue(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
      orientationChar->notify();
    }
  }

  // Chime once, from here rather than the BLE callback (playTone blocks).
  if (connectChimePending) { connectChimePending = false; soundReady(); }

  hapticService(now);            // ends a motor pulse started in a button callback

  // Hand over to the buttons 15 s after boot, but only once a central is actually
  // connected — as specified. With nothing connected there is nothing to drive, so the
  // status screen stays up; tap it to go to the buttons anyway.
  if (panelMode == SCREEN_LIVE && !autoSwitchDone && deviceConnected &&
      (now - bootMs >= AUTO_SWITCH_MS)) showButtons();

  delay(5);
}
