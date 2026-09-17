#ifndef ROBOT_BLUETOOTH_H
#define ROBOT_BLUETOOTH_H

// ============================================================
// BLUETOOTH GAMEPAD -- second control source, alongside the mobile app.
//
// Uses Bluepad32, which on the original ESP32 speaks BOTH Bluetooth
// Classic and BLE -- so it doesn't matter which one the Rampage Hydra
// uses in Bluetooth mode (budget "Android/PC" pads vary).
//
// BUILD SETUP (one time, Arduino IDE):
//   1. File > Preferences > Additional boards manager URLs, add:
//        https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
//   2. Boards Manager: install "esp32_bluepad32"
//   3. Tools > Board > "ESP32 + Bluepad32 Arduino" > ESP32 Dev Module
//   4. Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)"
// With the normal ESP32 board package this file compiles to no-ops and
// the robot runs app-only, exactly as before.
//
// PAIRING: put the Hydra in Bluetooth pairing mode (see its manual) while
// the robot is powered. The ESP32 accepts new controllers whenever none is
// connected; after the first pairing it reconnects automatically.
//
// CONTROLLER LAYOUT (same logic as the app -- it's the same code path):
//   Left stick ........ walk (forward/back, strafe)
//   Right stick X ..... turn on the spot
//   L1 / R1 ........... body height down / up        (Menu: up / down)
//                       hold BOTH 2 s = stand / sit
//   L2 / R2 ........... pitch trim nose down / up    (Menu: left / right)
//                       (digital button or analog trigger pull)
//   A ................. balance on/off                (Menu: open / save)
//   B ................. dirt mode on/off               (Menu: back / cancel)
//   Y ................. emote mode on/off
//   X ................. play / stop the selected emote
//   D-pad left/right .. choose emote (in emote mode)
//   R3 (click R stick)  re-run IMU auto-calibration (standing only)
//   SELECT / BACK ..... EMERGENCY STOP while held
//
// COEXISTING WITH THE APP: the gamepad only feeds the robot while it is
// actually being used (stick moved / button or trigger held), plus one
// "all released" frame afterwards. An idle connected controller therefore
// never blocks the app. If both are used at the same moment they simply
// take turns -- no protection beyond that, by design.
// ============================================================

#if ENABLE_BT_CONTROLLER && defined(__has_include)
#  if __has_include(<Bluepad32.h>)
#    include <Bluepad32.h>
// Not exposed by the Arduino wrapper, but present in the core's Bluepad32
// library -- lets us pick the transport and pairing security from here.
extern "C" {
#      include <bt/uni_bt.h>
#      include <bt/uni_bt_le.h>
#      include <bt/uni_bt_bredr.h>
}
#    define HEXAPOD_BT_ACTIVE 1
#  endif
#endif
#ifndef HEXAPOD_BT_ACTIVE
#  define HEXAPOD_BT_ACTIVE 0
#endif

volatile bool btControllerConnected = false;   // shown in the Settings header

// Prints a "[BT] ..." status line every BT_STATUS_MS while no controller is
// connected. The boot banner alone is easy to miss: the IMU/level debug
// prints scroll it away within a second. Set to 0 once pairing works.
#define BT_STATUS_MS 3000

// Set to 1 for ONE flash to wipe stored pairings, then set back to 0.
// A pairing that got part-way through (e.g. dropped during the HID query)
// can leave half-valid keys on the ESP32 that make later attempts fail.
#define BT_FORGET_PAIRINGS_ON_BOOT 0

// ---- WHICH BLUETOOTH TRANSPORT TO USE -------------------------------
//   0 = both (default, recommended)
//   1 = BLE preferred: skips registering the Classic HID/SDP services.
//       NOTE: it does NOT stop Classic discovery in this library version,
//       so Classic connection attempts (and SDP errors) can still occur.
//   2 = Classic only (BR/EDR)
#define BT_TRANSPORT 0

// GAP security level for Classic pairing: -1 keeps the library default (2).
// Clone/no-name pads often fail authentication at the default level, which
// shows up as the connection dropping during the SDP/HID-descriptor step.
// Set to 0 to disable authentication -- this is the main remaining lever.
#define BT_GAP_SECURITY_LEVEL -1

#if HEXAPOD_BT_ACTIVE

static ControllerPtr btCtl = nullptr;
static char btLocalAddr[18] = "--:--:--:--:--:--";   // this ESP32's BT address
static volatile bool btNeedsReleaseFrame = false;   // failsafe after disconnect
static bool btWasActive = false;

static void btOnConnected(ControllerPtr ctl) {
    if (btCtl != nullptr) {
        Serial.println("[BT] Extra controller ignored -- one at a time.");
        return;
    }
    btCtl = ctl;
    btControllerConnected = true;
    ControllerProperties p = ctl->getProperties();
    const char* kind = ctl->isGamepad()  ? "GAMEPAD"
                     : ctl->isMouse()    ? "MOUSE"
                     : ctl->isKeyboard() ? "KEYBOARD"
                     : "OTHER";
    // The class matters for diagnosis: a mouse or keyboard pairing proves the
    // ESP32's Classic HID + SDP path works, which isolates a gamepad failure
    // to the gamepad itself. Non-gamepads connect but don't drive the robot.
    Serial.printf("[BT] CONNECTED: %s [%s] (VID 0x%04x PID 0x%04x)\n",
                  ctl->getModelName().c_str(), kind, p.vendor_id, p.product_id);
    // Stop scanning for new devices while driving: less radio time taken
    // away from the WiFi link the app uses. Re-enabled on disconnect.
    BP32.enableNewBluetoothConnections(false);
}

static void btOnDisconnected(ControllerPtr ctl) {
    if (ctl != btCtl) return;
    btCtl = nullptr;
    btControllerConnected = false;
    btNeedsReleaseFrame = true;          // make sure nothing stays "held"
    BP32.enableNewBluetoothConnections(true);
    Serial.println("[BT] Controller disconnected -- inputs released.");
}

void btSetup() {
    Serial.println();
    Serial.println("[BT] ===== Bluepad32 build: Bluetooth gamepad ENABLED =====");

    // IMPORTANT: these must be set BEFORE BP32.setup(). uni_bt_setup() reads
    // the transport flags once, and uni_bt_bredr_setup() applies the GAP
    // security level once, both during BP32.setup(). Setting them afterwards
    // has no effect at all.
#if BT_TRANSPORT == 1
    uni_bt_bredr_set_enabled(false);
    uni_bt_le_set_enabled(true);
    Serial.println("[BT] Transport: BLE preferred (Classic HID/SDP services not registered).");
    Serial.println("[BT] NOTE: this library version still runs Classic discovery, so");
    Serial.println("[BT] 'sdp_query' lines can still appear. It is not a full Classic off-switch.");
#elif BT_TRANSPORT == 2
    uni_bt_bredr_set_enabled(true);
    uni_bt_le_set_enabled(false);
    Serial.println("[BT] Transport: CLASSIC ONLY (BR/EDR).");
#else
    uni_bt_bredr_set_enabled(true);
    uni_bt_le_set_enabled(true);
    Serial.println("[BT] Transport: Classic + BLE (default).");
#endif

#if BT_GAP_SECURITY_LEVEL >= 0
    // Applied inside uni_bt_bredr_setup() via gap_set_security_level() and the
    // two l2cap_register_service() calls -- hence before BP32.setup().
    uni_bt_set_gap_security_level(BT_GAP_SECURITY_LEVEL);
#endif

    BP32.setup(&btOnConnected, &btOnDisconnected);
    Serial.printf("[BT] GAP security level in use: %d\n", uni_bt_get_gap_security_level());
    BP32.enableVirtualDevice(false);     // no virtual mouse from touchpads
#if BT_FORGET_PAIRINGS_ON_BOOT
    BP32.forgetBluetoothKeys();
    Serial.println("[BT] Stored pairings ERASED -- pair the controller again now.");
    Serial.println("[BT] Set BT_FORGET_PAIRINGS_ON_BOOT back to 0 and re-flash afterwards.");
#endif
    BP32.enableNewBluetoothConnections(true);
    const uint8_t* a = BP32.localBdAddress();
    snprintf(btLocalAddr, sizeof(btLocalAddr), "%02X:%02X:%02X:%02X:%02X:%02X",
             a[0], a[1], a[2], a[3], a[4], a[5]);
    // NOTE: the ESP32 is the Bluetooth HOST here -- Bluepad32 calls
    // gap_discoverable_control(0), so the robot is NOT discoverable and will
    // not show up in a phone's Bluetooth list. It scans and connects OUT to
    // controllers. A phone app must therefore ADVERTISE as a gamepad/keyboard
    // for the robot to find it. The address below is for identification only.
    Serial.printf("[BT] Bluepad32 %s ready, ESP32 BT address %02X:%02X:%02X:%02X:%02X:%02X\n",
                  BP32.firmwareVersion(), a[0], a[1], a[2], a[3], a[4], a[5]);
}

// Clear stored pairings (e.g. to switch to a different controller).
void btForgetPairings() { BP32.forgetBluetoothKeys(); }

static float btAxis(int32_t raw) {                // Bluepad32 sticks: -512..511
    return constrain(raw / 512.0f, -1.0f, 1.0f);
}

// Called every KinematicsTask tick. Services Bluetooth and, while the pad
// is in use, fills `out` and returns true.
bool btPoll(ControlInput &out) {
    BP32.update();                       // connect/disconnect callbacks run in here

#if BT_STATUS_MS > 0
    // Heartbeat so the status is visible even after the boot banner scrolls by.
    static unsigned long lastStatusMs = 0;
    if (!btControllerConnected && millis() - lastStatusMs >= BT_STATUS_MS) {
        lastStatusMs = millis();
        Serial.printf("[BT] Waiting for controller [transport:%s wifi:%s sec:%d] "
                      "-- put the gamepad in PAIRING mode now. "
                      "(this ESP32 is BT host %s, not discoverable)\n",
#if BT_TRANSPORT == 1
                      "BLE-only",
#elif BT_TRANSPORT == 2
                      "Classic-only",
#else
                      "Classic+BLE",
#endif
                      BT_PAIRING_TEST_NO_WIFI ? "OFF" : "AP-on",
                      uni_bt_get_gap_security_level(), btLocalAddr);
    }
#endif

    if (btNeedsReleaseFrame) {           // controller vanished mid-use
        btNeedsReleaseFrame = false;
        btWasActive = false;
        out = ControlInput();
        out.extended = true;             // also releases the kill-switch bit
        return true;
    }
    if (btCtl == nullptr || !btCtl->isConnected()) return false;
    if (!btCtl->isGamepad()) {
        // Connected, but a mouse/keyboard -- pairing works, it just can't
        // drive the robot. Say so once instead of failing silently.
        static bool warned = false;
        if (!warned) {
            warned = true;
            Serial.println("[BT] Device paired OK but is not a gamepad -- no robot control from it.");
            Serial.println("[BT] Useful result: the ESP32 Bluetooth HID path is working.");
        }
        return false;
    }

    ControlInput in;
    in.extended = true;

    // ---- sticks + triggers ----
    in.fwd  = BT_FWD_SIGN  * -btAxis(btCtl->axisY());   // stick up is negative in Bluepad32
    in.side = BT_SIDE_SIGN *  btAxis(btCtl->axisX());
    in.spin = BT_SPIN_SIGN *  btAxis(btCtl->axisRX());
    // HEIGHT is on L1/R1, exactly as in the mobile app -- digital, full
    // rate while held. L1/R1 do double duty: individually they trim height,
    // held together for 2 s they toggle stand/sit (bits 0x04/0x08 below).
    in.lt   = btCtl->l1() ? 1.0f : 0.0f;
    in.rt   = btCtl->r1() ? 1.0f : 0.0f;

    // ---- buttons1 (same bits as the app packet) ----
    if (btCtl->a())  in.buttons1 |= 0x01;
    if (btCtl->b())  in.buttons1 |= 0x02;
    if (btCtl->l1()) in.buttons1 |= 0x04;
    if (btCtl->r1()) in.buttons1 |= 0x08;
    if (btCtl->y())  in.buttons1 |= 0x10;   // emote mode toggle

    // X = play, or stop if an emote was already playing when X went down.
    // Decided on the press so a held X can't play-then-immediately-stop.
    static bool prevX = false, xMeansStop = false;
    bool x = btCtl->x();
    if (x && !prevX) xMeansStop = emote_playing;
    if (x) in.buttons1 |= xMeansStop ? 0x40 : 0x20;
    prevX = x;

    // ---- D-pad ----
    uint8_t dp = btCtl->dpad();
    static uint8_t prevDp = 0;
    uint8_t pressed = dp & ~prevDp;
    prevDp = dp;

    if (emote_mode_enabled && NUM_EMOTES > 0) {           // choose emote
        if (pressed & DPAD_RIGHT) in.emoteId = (selected_emote_id + 1) % NUM_EMOTES;
        if (pressed & DPAD_LEFT)  in.emoteId = (selected_emote_id + NUM_EMOTES - 1) % NUM_EMOTES;
    }

    // PITCH TRIM is on L2/R2, matching the app. Accepts either the digital
    // L2/R2 buttons or an analog trigger pull, since pads differ in which
    // they report. Only sent while standing: the same bits are menu
    // left/right while asleep.
    bool walking = (current_transition_progress >= 1.0f) && !in_calibration_mode;
    if (walking) {
        bool l2Down = btCtl->l2() || btCtl->brake()    > (1023 * 0.35f);
        bool r2Down = btCtl->r2() || btCtl->throttle() > (1023 * 0.35f);
        if (l2Down) in.buttons2 |= 0x40;                  // nose down
        if (r2Down) in.buttons2 |= 0x80;                  // nose up
    }

    // ---- buttons2 ----
    if (btCtl->thumbR())     in.buttons2 |= 0x01;         // manual IMU re-cal (edge)
    if (btCtl->miscSelect()) in.buttons2 |= 0x02;         // KILL SWITCH (held)

    // ---- only drive the robot while the pad is in use ----
    bool active = fabsf(in.fwd) > JOYSTICK_DEADZONE || fabsf(in.side) > JOYSTICK_DEADZONE ||
                  fabsf(in.spin) > JOYSTICK_DEADZONE ||
                  in.lt > BT_TRIGGER_ACTIVE || in.rt > BT_TRIGGER_ACTIVE ||
                  in.buttons1 || in.buttons2 || dp || in.emoteId != 0xFF;

    if (active || btWasActive) {         // last active frame is followed by one release frame
        btWasActive = active;
        out = in;
        return true;
    }
    return false;
}

#else  // ---- built without Bluepad32: app-only, zero overhead ----

void btSetup() {
#if ENABLE_BT_CONTROLLER
    Serial.println();
    Serial.println("[BT] ===== WRONG BOARD: Bluetooth gamepad is DISABLED =====");
    Serial.println("[BT] Select Tools > Board > 'ESP32 + Bluepad32 Arduino' > ESP32 Dev Module,");
    Serial.println("[BT] then Tools > Partition Scheme > 'Huge APP', and re-flash.");
#endif
}
void btForgetPairings() {}
bool btPoll(ControlInput &) {
#if ENABLE_BT_CONTROLLER && BT_STATUS_MS > 0
    static unsigned long lastStatusMs = 0;
    if (millis() - lastStatusMs >= BT_STATUS_MS) {
        lastStatusMs = millis();
        Serial.println("[BT] DISABLED -- this firmware was built without the Bluepad32 board package.");
    }
#endif
    return false;
}

#endif // HEXAPOD_BT_ACTIVE
#endif // ROBOT_BLUETOOTH_H
