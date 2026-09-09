# Move the SoftPot off IO19 → GPIO2, via the microSD slot

*Status: planned, not implemented. Nothing in `firmware/` has been changed for this yet.*

## Context

The SoftPot wiper currently sits on **IO19**, which is USB D−. That one choice has cost more
time than anything else in this build:

- **No USB serial.** `USB CDC On Boot` must be Disabled, the console lives on UART0
  (RXD/TXD = 44/43) via an adapter, and debugging without it produced the blank-screen hunt.
- **The strip must be unplugged to reflash**, because a resistive load on D− stops the port
  enumerating.
- **The USB-Serial-JTAG PHY owns the pad out of reset**, which needed `gpio_reset_pin()` and
  read as a flat 0 until that was found.
- **ADC2**, which is unusable while WiFi runs (harmless today, a trap later).

Moving to **GPIO2 (ADC1_CH1)** on the microSD slot removes all four at once and gives back
native USB. Confirmed from the schematic: the SD socket carries **3V3 (pin 4)** and **GND
(pin 6)**, so a microSD breakout adapter reaches everything with no soldering to the board.
User has confirmed the SD slot is never used and an adapter will fit the enclosure.

**The complication, and the design that answers it:** all four SD signal lines carry
populated **10 kΩ pull-ups to 3V3** (`R3`–`R6`; the schematic marks unfitted parts `NC`, and
these read a plain `10K`). Wired as a conventional divider, an untouched strip would read
~4095 instead of ~0 and invert the no-touch test. So **wire the strip as a rheostat instead**
and let the board's own pull-up be the upper leg. The pull-up stops being an obstacle and
becomes the circuit.

Rejected: IO20 (identical ADC2/USB trade — buys nothing); IO14/IO15 (the shared I2C bus,
2.2 k pull-ups); RXD/TXD (no ADC at all); GPIO3 (would work, but it is a strapping pin).
There is **no unallocated pad-accessible GPIO on this board** — the SD lines are pins taken
from the card reader, not spares.

## Wiring

| SoftPot | goes to | note |
|---|---|---|
| wiper (**centre** pin) | SD socket **pin 5** (CLK → GPIO2) | via the breakout adapter |
| one outer end | SD socket **pin 6** (VSS/GND) | |
| other outer end | **NOT CONNECTED** | this is the point — see below |

**Nothing connects to 3V3.** With the far end floating there is no path from the supply
through the strip to ground, so the failure that burned a strip earlier in this project
cannot recur here.

Resulting behaviour — note the inversion:

| state | reads |
|---|---|
| untouched (wiper floating, pulled up by R5) | ~4095 |
| touched, at the grounded end | ~0 |
| touched, at the far end | ~2050 (for a 10 kΩ strip) |

No-touch is unambiguous because *any* touch is well below the untouched value.

**Verify the adapter's pinout with a meter before powering anything** — microSD breakout pad
numbering is not standardised, and pin 5 must be the one that reads continuity to GPIO2.

## Changes

Apply **identically** to `firmware/left-panel/left-panel.ino` and
`firmware/right-panel/right-panel.ino`. They currently differ on exactly 3 lines (2, 165,
175 — hand and orientation); that must still hold afterwards.

### 1. Pin and calibration constants (`right-panel.ino:124-125`)

Current text:

```cpp
static constexpr int PIN_SOFTPOT = 19;            // ADC2_CH8 — see WIRING note above
static constexpr int SOFTPOT_NOTOUCH_RAW = 80;    // raw ADC below this = no touch
```

Replace with GPIO2 plus **two** constants, because the transfer function now has both ends to
pin down:

- `SOFTPOT_NOTOUCH_MIN` — raw at or above this means no touch (start ~3400).
- `SOFTPOT_TOUCH_MAX` — raw with a finger at the far end (start ~2050).

Both are placeholders to be set on the bench, exactly as `SOFTPOT_NOTOUCH_RAW` was.

### 2. `readSoftPot()` (`right-panel.ino:610-621`)

Current body:

```cpp
  int raw = analogRead(PIN_SOFTPOT);
  lastSoftPotRaw = raw;
  if (raw < SOFTPOT_NOTOUCH_RAW) {
    touching = false; pkt.touchStart = 0; pkt.touchCurrent = 0; softpotEMA = 0;
    return;
  }
  uint8_t pos = (uint8_t)constrain(map(raw, SOFTPOT_NOTOUCH_RAW, 4095, 1, 255), 1, 255);
```

Invert the no-touch test and rescale. The rheostat response is a hyperbola, not a line —
`V = 3.3·pR/(pR + 10k)` — which compresses the far end badly, so linearise it. In raw ADC
terms the ratio `raw / (4095 − raw)` is proportional to position, giving:

```
pos = 255 * (raw / (4095 - raw)) / K       where K = TOUCH_MAX / (4095 - TOUCH_MAX)
```

Keep the existing touch-down capture (`pkt.touchStart`) and the `0.6/0.4` EMA unchanged —
only the raw→position mapping changes. Guard the division for `raw ≥ 4095`.

### 3. ADC setup (`right-panel.ino:1261-1275`)

Current text, for an exact anchor:

```cpp
  // SoftPot on IO19 (ADC2). Internal pulldown so an untouched, floating wiper reads ~0.
  //
  // FIRST take the pad back from the USB PHY. ...
  gpio_reset_pin((gpio_num_t)PIN_SOFTPOT);
  bootNote("[ADC] IO19 taken from USB PHY");

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SOFTPOT, ADC_11db);
  analogRead(PIN_SOFTPOT);                        // let the core configure the pin first
  gpio_pulldown_en((gpio_num_t)PIN_SOFTPOT);
```

- **Delete `gpio_pulldown_en()`.** Critical: the internal ~45 kΩ pulldown fighting the
  external 10 kΩ pull-up would drag the untouched reading to ~3350, colliding with the
  touched range and destroying the no-touch margin.
- **Keep `gpio_reset_pin()`** — still good hygiene, but the whole comment block above it is
  about detaching the USB PHY and becomes wrong. Rewrite it for the rheostat wiring.
- **`bootNote("[ADC] IO19 taken from USB PHY")` must change too** — it is on-glass boot
  diagnostics, the thing that made the blank-screen fault findable, so a stale message here
  actively misleads. Make it name GPIO2 and the expected untouched reading, e.g.
  `[ADC] GPIO2 (ADC1) — untouched ~4095`.
- Keep `analogReadResolution(12)` and `analogSetPinAttenuation(ADC_11db)`.
- Keep `analogRead()` before any further pin config — the core reconfigures the pad on first
  read, which is why that ordering exists.

### 4. Give USB back (`right-panel.ino:22`, `:66-74`, `:151`)

- **Invert the `#error` guard.** It currently fails the build when `USB CDC On Boot` is
  *enabled*; that reason is gone. Make it require **Enabled**, so the setting can't silently
  regress and cost another blind-debugging session.
- Board-settings block: `USB CDC On Boot` → **ENABLED**, and drop the "one deliberate
  deviation" note — the FQBN is now stock.
- Header wiring note: GPIO2 via the SD slot, the rheostat rationale, and that **IO19/IO20 are
  now free** as spare ADC2 inputs.
- The core-version `#error` at `:97` lists Tools options to re-check after a core switch —
  update `USB CDC On Boot Disabled` to `Enabled` there too.
- UART0 on 43/44 still works; it stops being mandatory.

## Verification

**Static (here):** `verify_panels.py` (in the session scratchpad) — brace/paren/string-literal
balance on both sketches, and the assertion that left and right still differ on exactly 3
lines. There is no ESP32 toolchain on this machine, so this is **not** a compile.

**Bench, in order:**

1. Meter the breakout: continuity from the wiper pad to socket pin 5, and confirm 3V3/GND on
   pins 4/6. Only then power up.
2. Flash `right-panel` over **USB** with the strip connected — this working at all is the
   headline result; it was impossible before.
3. Confirm USB serial enumerates and the boot log appears without the UART adapter.
4. Status screen `SoftPot: -- (raw N)`: untouched should sit near **4095**. If it reads mid-
   scale, the internal pulldown was not removed.
5. Press at the grounded end → near 0. Press at the far end → note that value; it is
   `SOFTPOT_TOUCH_MAX`. Set `SOFTPOT_NOTOUCH_MIN` midway between it and 4095.
6. Reflash with those two constants and sweep a finger end to end: `touchCurrent` should
   travel 1→255 at a roughly even rate. Uneven travel means the linearisation constant `K` is
   off, not the thresholds.
7. Confirm the dashboard still sees the strip: `python pc_dashboard/tracker_dashboard.py`.

**Risk to watch:** if the strip's end-to-end resistance is much above 10 kΩ, the touched
range rises toward 4095 and the no-touch margin shrinks. Step 5 measures this directly — if
`SOFTPOT_TOUCH_MAX` comes back above ~3000, say so before proceeding rather than tightening
the threshold onto noise.

## Reference — full GPIO map of this board

Derived from the schematic and `pin_config.h`. Useful beyond this change, because it shows
there is nothing spare.

| GPIO | Net | Consumer | ADC | Note |
|---|---|---|---|---|
| 0 | `GPIO0` | BOOT button | — | strapping |
| 1 | `MOSI` | microSD CMD (J4 p3) | **ADC1_CH0** | 10K pull-up R4 |
| 2 | `SCK` | microSD CLK (J4 p5) | **ADC1_CH1** | 10K pull-up R5 — **chosen** |
| 3 | `MISO` | microSD D0 (J4 p7) | **ADC1_CH2** | 10K pull-up R6, **strapping** |
| 4–7 | `QSPI_SIO0..SI3` | AMOLED QSPI | ADC1 | |
| 8 | `LCD_RESET` | AMOLED reset | ADC1_CH7 | |
| 9 | `TP_RESET` | touch reset | ADC1_CH8 | |
| 10 | `SYS_OUT` | BSS138 gate | ADC1_CH9 | not broken out |
| 11–12 | `QSPI_SCL`, `LCD_CS` | AMOLED | ADC2 | |
| 13 | `LCD_TE` | display FPC only | ADC2_CH2 | no pad |
| 14/15 | `SCL`/`SDA` | **shared I2C** | ADC2 | 2.2K pull-ups; touch+PMU+IMU+RTC+codecs |
| 16 | `I2S_MCLK` | audio | ADC2_CH5 | |
| 17 | `SDCS` | microSD CD/D3 (J4 p2) | ADC2_CH6 | 10K pull-up R3 |
| 18 | `MOTOR` | haptic, via Q1 | ADC2_CH7 | pads carry ALDO3 + collector, not the GPIO |
| 19/20 | `USB'_N`/`USB'_P` | USB D−/D+ | ADC2 | **freed by this change** |
| 21 | `QMI_INT1` | IMU interrupt | — | |
| 26, 33–37 | — | flash / in-package PSRAM | — | reserved |
| 38/39 | `TP_INT`, `RTC_INT` | touch, RTC | — | |
| 40–42 | I2S | audio | — | |
| 43/44 | `U0TXD`/`U0RXD` | UART0 header | **none** | free, but cannot do analog |
| 45 | `I2S_LRCK` | audio | — | strapping |
| 46 | `PA_CTRL` | speaker amp enable | — | strapping |

USB-C carries nothing else: CC1/CC2 are plain 5.1 k Rd resistors to GND, SBU1/SBU2 are
unconnected. Only D−/GPIO19 and D+/GPIO20 reach an IO.

## Out of scope

Still open, tracked separately:

- `PANEL_MADCTL_180` sweep for the left panel's mirrored display (currently 0x06, untested).
- `HAPTIC_LRA_HZ` resonance sweep — set `HAPTIC_SWEEP 1`, feel for the strongest step.
- `firmware/panel-selftest/` still tests haptics via the absent DRV2605L, so its
  `Haptic: NO` line is known-wrong.
- `firmware/pedal-panel/pedal-panel.ino` is an older snapshot these two were generated from;
  do not regenerate from it.
