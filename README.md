# HexaDog ZBD — ESP32 Firmware

Firmware for an 18-servo hexapod running on an ESP32. It handles walking, body leveling, scripted "emote" animations, an on-screen settings menu, idle "alive" motion, puppet mode, servo and IMU calibration, live telemetry over UDP, and an animated eye display.

You can drive it two ways: the companion **Android app over Wi-Fi**, or a **Bluetooth gamepad connected straight to the robot** (PS4, PS5, Xbox, Switch Pro, 8BitDo and others). Both can be connected at once, and both use the same button layout.

The code is organised so you only have to edit one place per concern — tuning values live in the `.ino`, motion lives in the gait file, and so on.

---

## Hardware

- **MCU:** ESP32 (dual-core; both are used)
- **Servo drivers:** 2× PCA9685 PWM boards (I²C `0x40` and `0x41`)
- **IMU:** MPU6050 (I²C `0x68`)
- **Display:** TFT panel via `TFT_eSPI`
- **Servos:** 18 (6 legs × 3 joints: hip, thigh, knee)
- **Power sensing:** voltage divider on GPIO 32
- **I²C pins:** SDA 21, SCL 22 @ 400 kHz

Wi-Fi access point: **`HEXAPOD_ESP32`**, password `12345678`, UDP port `5000`.

> **IMU sourcing matters.** A lot of MPU6050 modules on the market are clones or mislabelled, and body leveling quality depends directly on the sensor. If leveling behaves badly, suspect the module before the code.

---

## Board setup — read this before flashing

Bluetooth gamepad support needs a **different board package** from the stock ESP32 one.

**For Bluetooth gamepad support:**

1. Arduino IDE → **File → Preferences → Additional boards manager URLs**, add:
   `https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json`
2. **Boards Manager** → install **`esp32_bluepad32`**
3. **Tools → Board → ESP32 + Bluepad32 Arduino → ESP32 Dev Module**
4. **Tools → Partition Scheme → Huge APP (3MB No OTA/1MB SPIFFS)**

**Without it:** the firmware still builds and runs on the stock **ESP32 Dev Module** board. Bluetooth support compiles out cleanly and the robot is app-only — it prints a message at boot saying so. Nothing else changes.

---

## File layout

All files sit in one sketch folder.

| File | What's in it |
|---|---|
| `Hexapod_Firmwarev12.ino` | Every tunable constant (geometry, gait timing, leveling gains, idle motion, puppet mode, network, pins), hardware instances, shared state, `setup()`. **Start here to tune anything.** |
| `Robot_Gait_Mechanism.h` | Inverse kinematics, gait blending, the `KinematicsTask` on Core 1, IMU Kalman filter and auto-calibration, body leveling, idle motion, puppet geometry, servo I/O with flash-persisted offsets. |
| `Robot_Emotes.h` | The emote catalog and every scripted animation, plus the per-tick runner. |
| `Robot_Bluetooth.h` | Bluetooth gamepad support (Bluepad32). Compiles to no-ops on the stock board package. |
| `Screen_Settings.h` | TFT UI, settings menu, eye engine, battery reading, and the `TelemetryTask` on Core 0 (also sends UDP telemetry). |

Include order in the `.ino` is **Emotes → Bluetooth → Gait → Screen** because of cross-references. Don't reorder them.

---

## Where to edit what

- **Tuning values** (heights, speeds, deadzones, gains, idle motion, puppet limits, pins, Wi-Fi): `Hexapod_Firmwarev12.ino`
- **Walking, leveling, idle motion, puppet maths:** `Robot_Gait_Mechanism.h`
- **Add or change an emote:** `Robot_Emotes.h` — write `emote_yourthing(float t, EmoteTargets &out)`, then add an entry to `EMOTES[]` at the bottom. Also add the name to the app's list, in the same order.
- **Screen, menus, eye shapes and colours:** `Screen_Settings.h`
- **Gamepad button mapping:** `Robot_Bluetooth.h`

---

## Controls

The same layout on the app and on a Bluetooth gamepad.

| Control | While standing | In the on-screen menus |
|---|---|---|
| Left stick | walk forward/back, strafe | — |
| Right stick X | spin on the spot | — |
| **L1 / R1** | body height down / up | move selection up / down |
| **L1 + R1 held 2 s** | stand up / sit down | — |
| **L2 / R2** | pitch trim nose down / up | change value left / right |
| **A** | toggle body leveling | open / confirm / save |
| **B** | toggle dirt mode | back / cancel |

One job per button, everywhere. Anything you change in a menu is discarded if you press **B**, and only written to flash when you press **A**.

---

## On-screen settings menu

While the robot is **asleep**, its screen becomes a settings menu:

| Row | What it does |
|---|---|
| **Emote Mode** | Stands the robot up, then shows the emote list |
| **Active Gait Settings** | COG offset, walk height, step height, idle motion on/off |
| **IMU Calibration** | *Zero Level Now*, or *Run Calibration Dance* |
| **Servo Calibration** | Per-joint offsets in 0.5° steps |
| **Face Settings** | 20 eye shapes × 10 colours |
| **Theme Mode** | Dark / light screen theme |

Everything saved here persists across reboots (ESP32 NVS flash).

**Emote list:** L1/R1 pick, A plays, B exits. The animated face appears on screen while an emote runs, and the list returns when it finishes. Exiting leaves the robot standing, ready to walk.

---

## Runtime behaviour

### Boot
- ~2.5 s sit-still window while gyro bias is measured (200 samples, all three axes).
- Screen shows "Calibrating IMU gyro…" then the measured bias.
- Once standing on stable footing, the **bow/tilt calibration dance** runs automatically. This is required — **body leveling will not engage until calibration has completed.** If a fresh dance fails, the firmware falls back to the last known-good values saved in flash.

### Modes
- **Asleep** — servos unpowered, settings menu on screen.
- **Standing / walking** — normal mode, animated eyes on screen.
- **Body leveling** — active leg-shift compensation for pitch and roll. Needs calibration first.
- **Dirt mode** — higher step clearance for rough terrain.
- **Emote mode** — walking disabled, robot rises to a fixed height and plays animations.
- **Servo calibration** — reached from the menu while asleep.
- **Puppet mode** — the body mirrors the phone's attitude (app only, see below).

### Idle "alive" motion
After **5 seconds** with no input, the robot starts breathing and drifting gently so it doesn't look switched off. Breathing is periodic; the postural drift is filtered noise rather than a repeating pattern, so it doesn't read as mechanical. It fades out within a quarter-second of any input, and can be turned off in *Active Gait Settings*.

### Auto-sleep
After **60 seconds** with no input the robot sits down by itself, with a countdown on the screen for the last 5 seconds. It will not do this during emotes, puppet mode, calibration, or while body leveling is on (sitting down unprompted on a slope is a good way to topple it). Set `AUTO_SLEEP_ENABLED = false` in the `.ino` to disable.

### Puppet mode (Android app only)
Hold the phone in **portrait** — the top of the screen is the robot's nose. Tilt it and the body copies you; twist it and the robot leans into the turn and eases back. Height is fixed at 20 cm, and body leveling, idle motion, height trim and pitch trim are all disabled while it runs. If the phone stops sending for half a second the robot returns to level by itself.

---

## Protocol

### Inbound (controller → ESP32, little-endian)

The firmware picks its parsing tier by **packet length**, so older senders keep working:

| Size | Tier |
|---|---|
| 22 B | legacy: sticks, triggers, buttons1, emote id |
| 48 B | + buttons2 (kill switch, manual re-cal, pitch trim) and 6 tunable floats |
| 61 B | + puppet mode block |

```
[0:4]   joy_fwd    float
[4:8]   joy_side   float
[8:12]  joy_spin   float
[12:16] norm_lt    float    (height down)
[16:20] norm_rt    float    (height up)
[20]    buttons1   uint8    bit0=A bit1=B bit2=L1 bit3=R1
                            bit4=emote_mode_toggle bit5=emote_play bit6=emote_stop
[21]    selected_emote_id   uint8
[22]    buttons2   uint8    bit0=manual_recal  bit1=kill_switch
                            bit3=tunables_valid
                            bit6=pitch_nose_down  bit7=pitch_nose_up
[23]    reserved
[24:48] 6 × float  live leveling tunables (only read if bit3 set)
[48]    puppet flags uint8  bit0=puppet_on  bit1=re-zero
[49:53] phone roll  float (deg)
[53:57] phone pitch float (deg)
[57:61] phone gyro Z float (deg/s)
```

### Outbound (ESP32 → app, 28 bytes, ~20 Hz)

IMU X/Y/Z (3× float), battery voltage (float), selected joint (int32), joint offset (float), display state (u8: 0 sit, 1 stand, 2 calibrate, 3 emote), leveling on/off (u8), dirt on/off (u8), playing emote id (u8, 255 = none).

---

## Emotes

Curious Head Tilt, Shy Peek-a-boo, Cautious Object Tap, The Wiggle, Play Bow, Happy Dance into Sneak, Stadium Ripple Wave, Breathing into Foot Stomp, Itch Scratch, Matrix Gyro Roll, Push-Ups, Sit & Wave Hello, Victory Wave, Battle Mode / Intimidate.

Each `EMOTES[]` entry has a name, duration, intro fade-in, outro fade-out, and a pointer to its animation function. They're ported from a PyBullet reference simulation, which is why the timing is consistent across the catalog.

---

## Dependencies

Arduino IDE Library Manager:

- `Adafruit PWM Servo Driver Library`
- `Adafruit MPU6050`
- `Adafruit Unified Sensor`
- `TFT_eSPI` (configure `User_Setup.h` for your display's wiring)
- `Preferences`, `WiFi`, `WiFiUdp`, `Wire`, `SPI` (bundled with the ESP32 core)

Bluepad32 comes with the ESP32 + Bluepad32 **board package** — it is not a Library Manager entry.

---

## Building & flashing

1. Open `Hexapod_Firmwarev12.ino`. All four `.h` files must sit next to it.
2. Configure `TFT_eSPI`'s `User_Setup.h` for your display.
3. Select the board (see **Board setup** above) and the correct serial port.
4. Upload.
5. **App:** connect to the `HEXAPOD_ESP32` Wi-Fi network and start sending packets to `192.168.4.1:5000`.
   **Gamepad:** put the controller in pairing mode with the robot powered. It connects on its own and reconnects automatically afterwards.

---

## Calibration workflow

**IMU (do this first — leveling depends on it):**
1. Stand the robot up on **level ground**.
2. It should run the bow/tilt dance by itself. To run it manually: *Settings → IMU Calibration → Run Calibration Dance*, or the RECALL button in the app.
3. Leave it alone while it runs; the manual path deliberately skips the "is it still?" check.
4. It stays standing afterwards, so you can enable leveling with **A** and drive off.

*Zero Level Now* on the same page is different: it takes the robot's current pose as "flat", for trimming out a small standing lean. It does not replace the dance.

**Servos:**
1. While asleep: *Settings → Servo Calibration*.
2. L1/R1 select the joint, L2/R2 nudge its offset in 0.5° steps.
3. **A** saves to flash, **B** cancels and restores the previous values.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Body leveling does nothing | Calibration hasn't completed. Run the dance; leveling is gated on it. |
| Leveling oscillates | Raise `LEVEL_SMOOTH_TAU_S` (try 0.2). Also check the IMU module isn't a bad clone. |
| Gamepad never connects | Wrong board package — you need ESP32 + Bluepad32, not stock ESP32. Serial says which is built. |
| Gamepad connects then drops during pairing | Some cheap pads answer the HID descriptor query too slowly for Bluepad32's 13 s limit. Use a supported controller. |
| Robot sits down on its own | Auto-sleep after 60 s idle. Disable with `AUTO_SLEEP_ENABLED`. |
| Robot won't stop fidgeting | Idle motion. Turn it off in *Active Gait Settings*. |
| Nothing on the app but the robot works | Check the packet length your sender uses — features are length-gated (22 / 48 / 61). |

---

## Safety notes

- Servos are actively disabled (`shutDownServosHardware()`) whenever the robot is fully sitting and not in calibration or emote mode — no holding torque, no buzz.
- Height is clamped between `MIN_HEIGHT` and `MAX_HEIGHT` (0.14 m – 0.23 m).
- Leveling and puppet offsets are clamped by remaining leg reach, so a commanded pose can never push a leg past its workspace.
- Leveling ramps in over 1 s and has a deadband so it ignores small IMU noise.
- Emote playback won't start until the body has reached emote height, so animations never jerk mid-ramp.
- Auto-sleep is suppressed while leveling is on, during emotes, puppet mode and calibration.
- A dropped Bluetooth controller releases all inputs rather than leaving the last command applied.

---

## Licensing & Commercial Use

Licensed under **Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to remix, adapt and build upon this design for non-commercial purposes, with appropriate credit. Commercial use of any kind is not permitted.

https://creativecommons.org/licenses/by-nc/4.0/
