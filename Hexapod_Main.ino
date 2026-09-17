#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>

// ############################################################
// #  ALL TUNABLE SETTINGS LIVE IN THIS FILE.                 #
// #  Change a value  -> here (Hexapod_Main.ino)              #
// #  Change how it moves/balances -> Robot_Gait_Mechanism.h  #
// #  Change / add emotes -> Robot_Emotes.h                   #
// #  Change what's on screen       -> Screen_Settings.h      #
// ############################################################

// ============================================================
// HARDWARE INSTANCES
// ============================================================
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm3 = Adafruit_PWMServoDriver(0x41);
Adafruit_MPU6050 mpu;
Preferences preferences;
TFT_eSPI tft = TFT_eSPI();
WiFiUDP udp;

// ============================================================
// I2C / PIN CONFIG
// ============================================================
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
const uint32_t I2C_CLOCK_HZ = 400000;
const int VOLTAGE_PIN = 32;
const float DIVIDER_RATIO = 5.0;
const float ADC_CAL_FACTOR = 1.08; // fine-trim multiplier for the voltage read

// ============================================================
// SERVO CONFIG
// ============================================================
#define USMIN 500.0
#define USMAX 2500.0
const uint8_t PWM_FREQ_HZ = 50;

// Servo Offsets (persisted to flash; edited in calibration mode)
float servoOffsets[18] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

// ============================================================
// ROBOT GEOMETRY
// ============================================================
const float L1 = 0.1000;
const float L2 = 0.1000;
const float BODY_OFFSET = 0.0544;
const float KNEE_ASSEMBLY_OFFSET = PI / 2.0;

// ============================================================
// GAIT / HEIGHT TUNING
// ============================================================
const float DT_SEC = 0.008333; // 120Hz loop period

const float MIN_HEIGHT = 0.14;
const float MAX_HEIGHT = 0.23;
const float HEIGHT_SPEED = 0.06; // m/s, manual trim rate + normal height ramp rate

const float EMOTE_HEIGHT_SPEED = 0.03;
const float STOP_BLEND_SPEED = 0.08;

const float STARTUP_DURATION = 2.0;          // stand/sit ramp seconds
const float REQUIRED_HOLD_DURATION = 2.0;    // L1+R1 hold-to-toggle seconds

// Directional shift low-pass filter (anti-shake, from the PyBullet script)
const float URDF_X_ALPHA = 0.04; // 0.02 = smoother, 0.08 = faster

// ---- Settings-menu editable body parameters (persisted to NVS "hexapod-ui";
// the values below are only the first-boot defaults). ----
// COG_OFFSET    : added on top of the per-direction urdf_x offsets (m)
// NEUTRAL_HEIGHT: body height the robot stands up at (m); triggers still trim live
// STEP_HEIGHT   : forward-walk step height (m); the other directions scale
//                 proportionally so their relative tuning is preserved.
//                 Dirt mode keeps its own fixed 0.020 m.
float COG_OFFSET     = 0.0f;
float NEUTRAL_HEIGHT = 0.20f;
float STEP_HEIGHT    = 0.0125f;   // == DIR_FORWARD.step_height -> gait unchanged by default

const float COG_OFFSET_MIN = -0.02f,  COG_OFFSET_MAX = 0.02f,  COG_OFFSET_STEP = 0.0025f;
const float NEUTRAL_H_MIN  =  0.18f,  NEUTRAL_H_MAX  = 0.22f,  NEUTRAL_H_STEP  = 0.02f;
const float STEP_H_MIN     =  0.005f, STEP_H_MAX     = 0.02f,  STEP_H_STEP     = 0.0025f;

// ============================================================
// BALANCE / IMU / BODY-LEVELING TUNING  (v10 -- Kalman-based)
// ============================================================


const unsigned long BOOT_DURATION_MS = 2500;
const int   GYRO_CALIB_SAMPLES  = 200;
const float GYRO_CALIB_SAMPLE_MS = 10.0f;


float KALMAN_Q_ANGLE   = 0.001f;   // process noise: trust in the angle prediction
float KALMAN_Q_BIAS    = 0.003f;   // process noise: expected gyro-bias drift rate
float KALMAN_R_MEASURE = 0.03f;    // measurement noise: trust in the accel-derived angle


const float LEVEL_GAIN_DEFAULT = 1.0f;
float LEVEL_GAIN = LEVEL_GAIN_DEFAULT;


const float BALANCE_BLEND_RAMP_SEC = 1.0f;

// Target body height forced while balancing (m).
const float BALANCE_TARGET_HEIGHT = 0.18f;

// Hard deadband: tilt readings smaller than this are treated as exactly
// level (zero compensation, not a scaled-down one). 
float LEVEL_DEADBAND_DEG = 2.0f;
// Low-pass on the angles used for LEVELING only (the Kalman output itself is
// untouched). Puts back the smoothing that slow IMU sampling used to provide
// by accident. Raise if leveling still oscillates, lower for a snappier
// response, 0 to disable entirely.
float LEVEL_SMOOTH_TAU_S = 0.20f;

// Hard cap on how far leveling will attempt to compensate for.
float LEVEL_MAX_TILT_DEG = 15.0f;

// Fixed safety ceilings on commanded x/y/z leg-target compensation.
const float LEVEL_MAX_DELTA_Z  = 0.08f;
const float LEVEL_MAX_DELTA_XY = 0.03f;

// ---- Manual pitch trim -- L2(nose down)/R2(nose up), WALKING MODE ONLY.

const float PITCH_TRIM_SPEED_DEG_S = 8.0f;
const float PITCH_TRIM_MAX_DEG     = 20.0f;

// ---- Reach-margin dynamic clamp ----
// (LEG_MAX_REACH mirrors solve_leg_ik_3dof's own 0.99*(L1+L2) limit)
const float LEG_MAX_REACH = (0.1000f + 0.1000f) * 0.99f;

// ---- Bow/tilt auto-calibration dance -- 
const float CAL_ZERO_DURATION_S   = 1.0f;
const float CAL_BOW_DURATION_S    = 1.0f;
const float CAL_HOLD_DURATION_S   = 0.5f;
const float CAL_RETURN_DURATION_S = 1.0f;
const float CAL_TILT_MAGNITUDE_M  = 0.020f;
const float CAL_MIN_DETECT_RAD    = 0.0349f;   // ~2 deg
const float CAL_MIN_GYRO_INT_RAD  = 0.0262f;   // ~1.5 deg
const float STARTUP_CAL_SETTLE_S  = 0.4f;
const float FOOTING_ACCEL_TOL_MS2 = 1.5f;
const float FOOTING_GYRO_MAX_RAD_S = 0.0873f;  // ~5 deg/s
const float FOOTING_STABLE_S       = 0.5f;

// ============================================================
// CONTROL / INPUT CONFIG
// ============================================================
const float JOYSTICK_DEADZONE = 0.12;

// ----- Stick axis snap (see processControlInput) -----

const float AXIS_SNAP_DEG      = 10.0f;
const float AXIS_SNAP_HYST_DEG = 2.0f;   // release angle = SNAP + HYST

const float TRIGGER_THRESHOLD = 0.05;

// ============================================================
// IDLE "ALIVE" MOTION  
// ============================================================

bool idle_motion_enabled = true;

const float IDLE_BREATH_PERIOD_S   = 4.2f;
const float IDLE_BREATH_HEIGHT_M   = 0.0062f;
const float IDLE_BREATH_PITCH_DEG  = 0.40f;
const float IDLE_BREATH_INHALE_FRAC= 0.42f;   // <0.5 = quick in, slow out

const float IDLE_DRIFT_ROLL_DEG    = 1.05f;   // 1-sigma, not a hard limit
const float IDLE_DRIFT_PITCH_DEG   = 0.85f;
const float IDLE_DRIFT_HEIGHT_M    = 0.0040f;
const float IDLE_DRIFT_TAU_S       = 3.5f;    // how long an excursion lasts
const float IDLE_SMOOTH_TAU_S      = 0.9f;    // second stage, kills jitter

const float IDLE_MAX_ROLL_DEG      = 4.5f;
const float IDLE_MAX_PITCH_DEG     = 4.0f;
const float IDLE_MAX_HEIGHT_M      = 0.020f;

const float IDLE_FADE_IN_S         = 0.8f;    // returns fairly promptly
const float IDLE_FADE_OUT_S        = 0.25f;   // gets out of the way at once


const float IDLE_ATTITUDE_SIGN     = -1.0f;

// ---- Inactivity timers ----

const bool          AUTO_SLEEP_ENABLED = true;   // set false to never auto-sleep

const unsigned long IDLE_START_MS      = 5000;
const unsigned long AUTO_SLEEP_MS      = 60000;  // quiet time before sitting down
const unsigned long AUTO_SLEEP_WARN_MS = 5000;   // on-screen warning before it happens

volatile unsigned long last_activity_ms = 0;
volatile int  auto_sleep_secs_left = -1;   // >=0 while the warning is showing

// Live idle outputs, written once per gait tick.
float idle_roll_rad = 0.0f, idle_pitch_rad = 0.0f, idle_dz_m = 0.0f;
float idle_envelope = 0.0f;

// ============================================================
// NETWORK CONFIG
// ============================================================

const char* AP_SSID = "HEXAPOD_ESP32";
const char* AP_PASSWORD = "12345678";
const uint16_t UDP_PORT = 5000;


#define ENABLE_BT_CONTROLLER 1

// ---- PAIRING TEST MODE ----------------------------------------------

#define BT_PAIRING_TEST_NO_WIFI 0

// Flip any of these to -1 if that stick moves the robot the wrong way.
// Defaults mirror a phone joystick: up = +fwd, right = +side, right = +spin.
const float BT_FWD_SIGN  = +1.0f;
const float BT_SIDE_SIGN = +1.0f;
const float BT_SPIN_SIGN = +1.0f;
const float BT_TRIGGER_ACTIVE = 0.05f;   // analog trigger counts as "in use" above this

// One frame of control input, whatever the source (app packet or gamepad).
struct ControlInput {
    float   fwd = 0.0f, side = 0.0f, spin = 0.0f, lt = 0.0f, rt = 0.0f;
    bool    puppetValid = false;     // packet carried the puppet block
    bool    puppetOn = false, puppetRezero = false;
    float   puppetRoll = 0.0f, puppetPitch = 0.0f, puppetGz = 0.0f;
    uint8_t buttons1 = 0;          // same bit layout as packet byte 20
    uint8_t emoteId  = 0xFF;       // >= NUM_EMOTES means "no change"
    bool    extended = false;      // buttons2 / tunables below are valid
    uint8_t buttons2 = 0;          // same bit layout as packet byte 22
    float   tunables[6] = {0, 0, 0, 0, 0, 0};
};

const int RX_PACKET_MIN_SIZE = 22;   // legacy floor -- joystick + buttons1 + emote id
const int RX_PACKET_EXT_SIZE = 48;   // full v10 packet with buttons2 + 6 tunables
const int RX_PACKET_PUP_SIZE = 61;   // v13: + puppet flags byte and 3 floats
uint8_t networkBuffer[61];

// ============================================================
// PUPPET MODE -- the body MIRRORS the phone's attitude
// ============================================================

const float PUPPET_BODY_HEIGHT   = 0.20f;   // fixed 20 cm while in puppet mode

// Halved: two units of phone tilt give one unit of body tilt. The clamps
// below are unchanged, so full body tilt now needs ~30 deg of phone tilt --
// which also halves the sensor noise reaching the legs.
const float PUPPET_ROLL_GAIN     = 0.5f;
const float PUPPET_PITCH_GAIN    = 0.5f;
const float PUPPET_YAW_GAIN      = 0.45f;   // left as-is: halving it feels dead

// ---- One Euro filter on the incoming phone angles ----

const float PUPPET_FILT_MINCUTOFF = 1.0f;   // Hz
const float PUPPET_FILT_BETA      = 0.02f;
const float PUPPET_FILT_DCUTOFF   = 1.0f;   // Hz, for the derivative

const float PUPPET_SIGN_ROLL     = +1.0f;   // flip if it mirrors backwards
const float PUPPET_SIGN_PITCH    = +1.0f;
const float PUPPET_SIGN_YAW      = +1.0f;

const float PUPPET_MAX_ROLL_DEG  = 15.0f;
const float PUPPET_MAX_PITCH_DEG = 15.0f;
const float PUPPET_MAX_YAW_DEG   = 8.0f;    // small: the feet are planted

const float PUPPET_DEADBAND_DEG  = 1.0f;    // SUBTRACTED, not a hard gate
const float PUPPET_MAX_SLEW_DPS  = 120.0f;  // stops a dropped packet becoming a step
const float PUPPET_YAW_LEAK_TAU  = 1.2f;    // leaky yaw integrator, seconds
const float PUPPET_GZ_DEADBAND   = 1.5f;    // deg/s -- kills standing bias drift
const float PUPPET_MAX_DELTA_XY  = 0.035f;  // yaw moves feet tangentially
const float PUPPET_MAX_DELTA_Z   = 0.060f;
const float PUPPET_STALE_S       = 0.5f;    // no data -> slew back to level

volatile bool  puppet_mode_enabled = false;
volatile float puppet_cmd_roll_deg = 0.0f, puppet_cmd_pitch_deg = 0.0f, puppet_cmd_yaw_deg = 0.0f;
float puppet_tgt_roll_deg = 0.0f, puppet_tgt_pitch_deg = 0.0f, puppet_tgt_yaw_deg = 0.0f;
unsigned long puppet_last_rx_ms = 0;

// ============================================================
// DISPLAY CONFIG
// ============================================================
const uint8_t SCREEN_ROTATION = 3;   // orientation used by the UI
const bool    SCREEN_INVERT   = true;
uint16_t MY_BLACK, MY_CYAN;          // assigned in setup() via color565()
const int eyeWidth = 90, eyeHeight = 110, eyeSpacing = 50;

// ============================================================
// RUNTIME STATE (not tunable -- do not edit to change behavior)
// ============================================================

bool target_standing_state = false;
float current_transition_progress = 0.0;
bool servosArePowered = false;
bool in_calibration_mode = false;
unsigned long bootStartTime = 0;
bool bootSequenceComplete = false;
float button_hold_time = 0.0;

// Height ramp state
float user_selected_height = 0.20;
float current_body_height = 0.20;

// Mode flags
bool balance_enabled = false;
bool dirt_mode_enabled = false;


bool          emote_mode_enabled = false;
bool          emote_playing      = false;
uint8_t       emote_playing_id   = 255;
uint8_t       selected_emote_id  = 0;
unsigned long emote_start_ms     = 0;

bool          exiting_emote_ramp = false;

// IMU filtered outputs + measured gyro bias
float accelRestMag = 9.81f;
float body_roll_filtered = 0.0;
float body_pitch_filtered = 0.0;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
bool  imuCalibrationDone = false;

// Raw gyro (rad/s), kept around for the bow/tilt auto-cal dance to
// integrate independent of whatever axis/sign convention is active.
volatile float raw_gx = 0.0f, raw_gy = 0.0f, raw_gz = 0.0f;

// ---- IMU AXIS REMAP  ----
int   imuAccelPitchKind = 0;   float imuAccelPitchSign = -1.0f; // matches v8: -atan2(ay,az)
int   imuAccelRollKind  = 2;   float imuAccelRollSign  = +1.0f; // matches v8: atan2(ax, sqrt(ay^2+az^2))
int   imuGyroPitchAxis  = 1;   float imuGyroPitchSign  = -1.0f; // matches v8: -(gy - biasY)
int   imuGyroRollAxis   = 0;   float imuGyroRollSign   = -1.0f; // matches v8: -(gx - biasX)
float imuPitchBias = 0.0f, imuRollBias = 0.0f;      // mounting-tilt offset, subtracted every frame
float imuGyroBiasPitch = 0.0f, imuGyroBiasRoll = 0.0f;

float imuUserPitchZero = 0.0f, imuUserRollZero = 0.0f;

// ---- Manual pitch trim (walking mode only -- see KinematicsTask) ----
float manual_pitch_trim = 0.0f;          // radians, user-commanded via L2(down)/R2(up)

// ---- Emergency kill switch ----
volatile bool kill_switch_active = false;

// Gait runtime
float urdf_x_filtered = 0.0;
float phase_accumulator = 0.0;
float motion_blend = 0.0;

// Controller input (written by gait task, read everywhere)
volatile float joy_fwd = 0.0, joy_side = 0.0, joy_spin = 0.0;
volatile float norm_lt = 0.0, norm_rt = 0.0;

// Telemetry values
volatile float batteryVoltage = 0.0;
volatile float imu_ax = 0.0, imu_ay = 0.0, imu_az = 0.0;

// Cross-core sync
SemaphoreHandle_t i2cMutex = NULL;
TaskHandle_t KinematicsTaskHandle;
TaskHandle_t TelemetryTaskHandle;

// ---- Settings menu input (sleep screen) ----

enum UiEvent : uint8_t { UI_UP = 1, UI_DOWN, UI_LEFT, UI_RIGHT, UI_ENTER, UI_BACK };
QueueHandle_t uiInputQueue = NULL;

// ---- Emote menu handshake (screen task <-> gait task) ----

volatile bool imu_zero_request     = false;   // Settings > IMU Calibration > Zero
volatile bool imu_zero_save_request = false;  // set by gait task, saved by screen
volatile bool cal_menu_requested   = false;   // Settings > IMU Calibration > Run Dance
volatile bool emote_menu_requested = false;
volatile bool emote_play_request   = false;
volatile bool emote_exit_request   = false;

// Calibration UI state + edge-detect latches
volatile int selectedJointID = 0;
bool lastAPressed = false, lastBPressed = false;
bool lastL1Pressed = false, lastR1Pressed = false;
// Edge-detect latches for the new emote-control bits.
bool lastEmoteModeTogglePressed = false;
bool lastEmotePlayPressed       = false;
bool lastEmoteStopPressed       = false;
// v9 -- edge-detect latches for the leveling-related bits.
bool lastManualCalTriggerPressed  = false;
bool justSavedFeedback = false;
unsigned long saveFeedbackTimer = 0;

// Display eye-engine state
uint8_t lastRenderedState = 255;
unsigned long blinkTimer = 0, blinkDuration = 0, lastMovementTime = 0, eyeLookTimer = 0;
bool isBlinking = false, screenDrawnForMotion = false;
int horizontalEyeOffset = 0;

// indices 0-2 FR, 3-5 FL, 6-8 MR, 9-11 ML, 12-14 RR, 15-17 RL
const char* JOINT_NAMES[18] = {
    "FR KNEE", "FR THIGH", "FR HIP", "FL KNEE", "FL THIGH", "FL HIP",
    "MR KNEE", "MR THIGH", "MR HIP", "ML KNEE", "ML THIGH", "ML HIP",
    "RR KNEE", "RR THIGH", "RR HIP", "RL KNEE", "RL THIGH", "RL HIP"
};

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void loadOffsetsFromFlash();
void shutDownServosHardware();
void calibrateGyroBias();        // gait file -- balance sensing (boot)
void updateIMUAndBalance();      // gait file -- balance sensing (per tick)
void setupDiagnosticUI();        // screen file
void loadUiSettingsFromFlash();  // screen file -- settings-menu values
void btSetup();                  // bluetooth file
bool btPoll(ControlInput &out);  // bluetooth file -- called from KinematicsTask
void runEmoteTick();             // emotes file -- called from KinematicsTask
void KinematicsTask(void * pvParameters);
void TelemetryTask(void * pvParameters);

// v9 -- body leveling / auto-cal (gait file)
void  loadImuCalFromFlash();
bool  calRunnerIsRunning();
bool  calDone();                 // true once auto-cal has succeeded (this boot or from cache)
const char* calStatusMsg();


#include "Robot_Emotes.h"          // emote catalog + runEmoteTick()
#include "Robot_Bluetooth.h"       // optional BT gamepad -> ControlInput
#include "Robot_Gait_Mechanism.h"  // motion + balance + servo I/O
#include "Screen_Settings.h"       // display + telemetry TX

void setup() {
    Serial.begin(921600);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);

    i2cMutex = xSemaphoreCreateMutex();

    tft.init();
    tft.setRotation(SCREEN_ROTATION);
    tft.invertDisplay(SCREEN_INVERT);
    MY_BLACK = tft.color565(0, 0, 0);
    MY_CYAN  = tft.color565(0, 255, 255);
    setupDiagnosticUI();

    loadOffsetsFromFlash();
    loadImuCalFromFlash();   // v9 -- skip the bow/tilt dance if this hardware was already calibrated
    loadUiSettingsFromFlash(); // body params, face, theme, IMU zero (Settings menu)

    uiInputQueue = xQueueCreate(16, sizeof(uint8_t));

    if (!mpu.begin(0x68, &Wire)) {
        Serial.println("Failed to find MPU6050 chip");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    pinMode(VOLTAGE_PIN, INPUT);

    pwm1.begin(); pwm3.begin();
    pwm1.setPWMFreq(PWM_FREQ_HZ); pwm3.setPWMFreq(PWM_FREQ_HZ);
    shutDownServosHardware();

#if BT_PAIRING_TEST_NO_WIFI
    Serial.println("[BT] PAIRING TEST MODE: WiFi is OFF, mobile app disabled.");
    WiFi.mode(WIFI_OFF);
#else
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    udp.begin(UDP_PORT);
#endif

    btSetup();               // no-op if built without the Bluepad32 board package

    bootStartTime = millis();
    user_selected_height = NEUTRAL_HEIGHT;
    current_body_height = user_selected_height;

    xTaskCreatePinnedToCore(KinematicsTask, "GaitEngine", 8192, NULL, 3, &KinematicsTaskHandle, 1);
    xTaskCreatePinnedToCore(TelemetryTask, "TelemetryEngine", 6144, NULL, 1, &TelemetryTaskHandle, 0);
}

void loop() {
    vTaskDelete(NULL);
}
