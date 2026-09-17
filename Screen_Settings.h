#ifndef SCREEN_SETTINGS_H
#define SCREEN_SETTINGS_H

// ============================================================
// SCREEN FILE -- telemetry TX, eye engine, and the SETTINGS menu
// shown while the robot is fully asleep.
//
// Screen modes:
//   MENU  : fully asleep, or inside Servo Calibration  -> Settings GUI
//   EYES  : standing / walking (progress >= 0.5)       -> animated eyes
//   TELEM : sitting/standing transition, emote mode    -> telemetry text
//
// Controls on the Settings menu (edge-detected in the gait task):
//   L1 / R1    up / down only    -- move selection / pick servo / pick shape
//   L2 / R2    left / right only -- change the selected value
//   A          enter / confirm+save   (the ONLY way into a sub-page)
//   B          back / cancel (restores the values you entered with)
// ============================================================

// ------------------------------------------------------------
// Small helpers
// ------------------------------------------------------------
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Snap a stepped value to its grid (avoids float drift, like round() in the mock).
static float stepValue(float v, float step, int dir, float lo, float hi) {
    float n = roundf(v / step) + dir;
    return constrain(n * step, lo, hi);
}

// ------------------------------------------------------------
// Theme
// ------------------------------------------------------------
bool ui_dark_mode = true;

uint16_t uiBg()     { return ui_dark_mode ? MY_BLACK   : rgb565(200, 200, 200); }
uint16_t uiText()   { return ui_dark_mode ? TFT_WHITE  : TFT_BLACK; }
uint16_t uiSel()    { return ui_dark_mode ? TFT_GREEN  : rgb565(0, 140, 0); }   // darker green reads better on grey
uint16_t uiTitle()  { return ui_dark_mode ? TFT_YELLOW : rgb565(150, 100, 0); }
uint16_t uiAccent() { return ui_dark_mode ? TFT_CYAN   : rgb565(0, 120, 160); }
uint16_t uiHint()   { return TFT_DARKGREY; }


typedef void (*EyeDrawFn)(int lx, int rx, int my, int w, int h, uint16_t col);

struct Pt { int x, y; };


void fillPolygon(const Pt* p, int n, uint16_t col) {
    int ymin = p[0].y, ymax = p[0].y;
    for (int i = 1; i < n; i++) { ymin = min(ymin, p[i].y); ymax = max(ymax, p[i].y); }
    float xs[12];
    for (int y = ymin; y <= ymax; y++) {
        float sy = y + 0.5f;                     // sample at pixel centre
        int k = 0;
        for (int i = 0; i < n && k < 12; i++) {
            const Pt &a = p[i], &b = p[(i + 1) % n];
            if ((a.y <= sy && b.y > sy) || (b.y <= sy && a.y > sy))
                xs[k++] = a.x + (sy - a.y) * (float)(b.x - a.x) / (float)(b.y - a.y);
        }
        for (int i = 1; i < k; i++) {            // insertion sort, k is tiny
            float v = xs[i]; int j = i - 1;
            while (j >= 0 && xs[j] > v) { xs[j + 1] = xs[j]; j--; }
            xs[j + 1] = v;
        }
        for (int i = 0; i + 1 < k; i += 2) {
            int x0 = (int)lroundf(xs[i]), x1 = (int)lroundf(xs[i + 1]);
            if (x1 >= x0) tft.drawFastHLine(x0, y, x1 - x0 + 1, col);
        }
    }
}

// pygame border_radius is clamped to half the smaller side -- mirror that.
void fillRR(int x, int y, int w, int h, int r, uint16_t col) {
    if (w <= 0 || h <= 0) return;
    r = min(r, min(w, h) / 2);
    if (r < 1) tft.fillRect(x, y, w, h, col);
    else       tft.fillRoundRect(x, y, w, h, r, col);
}


void fillHalfEllipse(int x, int y, int w, int h, bool top, uint16_t col) {
    if (h < 2) h = 2;
    float a = w / 2.0f, b = h / 2.0f, cx = x + a, cy = y + b;
    int srcY0 = top ? y : y + h / 2;             // rows of the ellipse to take
    int drawAt = y;                              // where they land (pygame: topleft)
    for (int i = 0; i < h / 2; i++) {
        float dy = (srcY0 + i + 0.5f - cy) / b;
        if (dy * dy >= 1.0f) continue;
        float hw = a * sqrtf(1.0f - dy * dy);
        int xa = (int)lroundf(cx - hw), xb = (int)lroundf(cx + hw);
        if (xb > xa) tft.drawFastHLine(xa, drawAt + i, xb - xa, col);
    }
}


void shape_standard(int lx, int rx, int my, int w, int h, uint16_t c) {
    int r = max(1, (int)(15 * (h / 100.0f)));
    fillRR(lx, my - h/2, w, h, r, c);
    fillRR(rx, my - h/2, w, h, r, c);
}
void shape_anime_capsule(int lx, int rx, int my, int w, int h, uint16_t c) {
    fillRR(lx, my - h/2, w, h, w/2, c);
    fillRR(rx, my - h/2, w, h, w/2, c);
}
void shape_perfect_circles(int lx, int rx, int my, int w, int h, uint16_t c) {
    int ry = max(1, h/2);
    tft.fillEllipse(lx + w/2, my, w/2, ry, c);
    tft.fillEllipse(rx + w/2, my, w/2, ry, c);
}
void shape_angry_slanted(int lx, int rx, int my, int w, int h, uint16_t c) {
    Pt lp[] = {{lx, my - h/3}, {lx + w, my - h/2}, {lx + w, my + h/2}, {lx, my + h/4}};
    Pt rp[] = {{rx, my - h/2}, {rx + w, my - h/3}, {rx + w, my + h/4}, {rx, my + h/2}};
    fillPolygon(lp, 4, c); fillPolygon(rp, 4, c);
}
void shape_sad_worried(int lx, int rx, int my, int w, int h, uint16_t c) {
    Pt lp[] = {{lx, my - h/2}, {lx + w, my - h/4}, {lx + w, my + h/3}, {lx, my + h/2}};
    Pt rp[] = {{rx, my - h/4}, {rx + w, my - h/2}, {rx + w, my + h/2}, {rx, my + h/3}};
    fillPolygon(lp, 4, c); fillPolygon(rp, 4, c);
}
void shape_happy_arcs(int lx, int rx, int my, int w, int h, uint16_t c) {
    fillHalfEllipse(lx, my - h/2, w, h, true, c);
    fillHalfEllipse(rx, my - h/2, w, h, true, c);
}
void shape_visor_slits(int lx, int rx, int my, int w, int h, uint16_t c) {
    int sh = max(4, (int)(24 * (h / 100.0f)));
    fillRR(lx, my - sh/2, w, sh, 4, c);
    fillRR(rx, my - sh/2, w, sh, 4, c);
}
void shape_squircle(int lx, int rx, int my, int w, int h, uint16_t c) {
    int r = max(1, (int)(30 * (h / 100.0f)));
    fillRR(lx, my - h/2, w, h, r, c);
    fillRR(rx, my - h/2, w, h, r, c);
}
void shape_cyclops_visor(int lx, int rx, int my, int w, int h, uint16_t c) {
    fillRR(lx, my - h/4, (rx + w) - lx, max(2, h/2), 10, c);
}
void shape_diamond_cat(int lx, int rx, int my, int w, int h, uint16_t c) {
    Pt lp[] = {{lx, my}, {lx + w/2, my - h/2}, {lx + w, my}, {lx + w/2, my + h/2}};
    Pt rp[] = {{rx, my}, {rx + w/2, my - h/2}, {rx + w, my}, {rx + w/2, my + h/2}};
    fillPolygon(lp, 4, c); fillPolygon(rp, 4, c);
}
void shape_flat_triangles(int lx, int rx, int my, int w, int h, uint16_t c) {
    Pt lp[] = {{lx, my - h/2}, {lx + w, my + h/2}, {lx, my + h/2}};
    Pt rp[] = {{rx + w, my - h/2}, {rx, my + h/2}, {rx + w, my + h/2}};
    fillPolygon(lp, 3, c); fillPolygon(rp, 3, c);
}
void shape_glitch_bars(int lx, int rx, int my, int w, int h, uint16_t c) {
    int sh = max(1, h/5);
    tft.fillRect(lx, my - h/3,  w, sh, c);
    tft.fillRect(lx, my + h/10, w, sh, c);
    tft.fillRect(rx, my - h/3,  w, sh, c);
    tft.fillRect(rx, my + h/10, w, sh, c);
}
void shape_hourglass(int lx, int rx, int my, int w, int h, uint16_t c) {
    Pt lp[] = {{lx, my - h/2}, {lx + w, my - h/2}, {lx + w/4, my}, {lx + w, my + h/2}, {lx, my + h/2}, {lx + w*3/4, my}};
    Pt rp[] = {{rx, my - h/2}, {rx + w, my - h/2}, {rx + w*3/4, my}, {rx + w, my + h/2}, {rx, my + h/2}, {rx + w/4, my}};
    fillPolygon(lp, 6, c); fillPolygon(rp, 6, c);
}
void shape_crosses(int lx, int rx, int my, int w, int h, uint16_t c) {
    int t = max(2, (int)(16 * (h / 100.0f)));
    tft.fillRect(lx + w/2 - t/2, my - h/2, t, h, c);
    tft.fillRect(lx, my - t/2, w, t, c);
    tft.fillRect(rx + w/2 - t/2, my - h/2, t, h, c);
    tft.fillRect(rx, my - t/2, w, t, c);
}
void shape_hexagon(int lx, int rx, int my, int w, int h, uint16_t c) {
    int hh = h/2;
    Pt lp[] = {{lx, my}, {lx + w/4, my - hh}, {lx + w*3/4, my - hh}, {lx + w, my}, {lx + w*3/4, my + hh}, {lx + w/4, my + hh}};
    Pt rp[] = {{rx, my}, {rx + w/4, my - hh}, {rx + w*3/4, my - hh}, {rx + w, my}, {rx + w*3/4, my + hh}, {rx + w/4, my + hh}};
    fillPolygon(lp, 6, c); fillPolygon(rp, 6, c);
}
void shape_inverted_arcs(int lx, int rx, int my, int w, int h, uint16_t c) {
    fillHalfEllipse(lx, my - h/2, w, h, false, c);
    fillHalfEllipse(rx, my - h/2, w, h, false, c);
}
void shape_chevron_outer(int lx, int rx, int my, int w, int h, uint16_t c) {
    Pt lp[] = {{lx, my - h/2}, {lx + w, my}, {lx, my + h/2}, {lx + w/3, my}};
    Pt rp[] = {{rx + w, my - h/2}, {rx, my}, {rx + w, my + h/2}, {rx + w*2/3, my}};
    fillPolygon(lp, 4, c); fillPolygon(rp, 4, c);
}
void shape_tall_slits(int lx, int rx, int my, int w, int h, uint16_t c) {
    fillRR(lx + w/3, my - h/2, w/3, h, 6, c);
    fillRR(rx + w/3, my - h/2, w/3, h, 6, c);
}
void shape_goggles(int lx, int rx, int my, int w, int h, uint16_t c) {
    Pt lp[] = {{lx, my - h/3}, {lx + w, my - h/2}, {lx + w, my + h/3}, {lx, my + h/2}};
    Pt rp[] = {{rx, my - h/2}, {rx + w, my - h/3}, {rx + w, my + h/2}, {rx, my + h/3}};
    fillPolygon(lp, 4, c); fillPolygon(rp, 4, c);
}
// pygame.draw.rect(..., width) draws the border INSIDE the rect
void outlineBox(int x, int y, int w, int h, int t, uint16_t c) {
    if (t * 2 >= w || t * 2 >= h) { tft.fillRect(x, y, w, h, c); return; }
    tft.fillRect(x, y, w, t, c);
    tft.fillRect(x, y + h - t, w, t, c);
    tft.fillRect(x, y + t, t, h - 2*t, c);
    tft.fillRect(x + w - t, y + t, t, h - 2*t, c);
}
void shape_bracket_boxes(int lx, int rx, int my, int w, int h, uint16_t c) {
    int t = max(1, (int)(12 * (h / 100.0f)));
    outlineBox(lx, my - h/2, w, h, t, c);
    outlineBox(rx, my - h/2, w, h, t, c);
}

struct EyeShape { const char* name; EyeDrawFn draw; };
const EyeShape EYE_SHAPES[] = {
    { "Standard Round",    shape_standard        },
    { "Anime Capsule",     shape_anime_capsule   },
    { "Perfect Circle",    shape_perfect_circles },
    { "Aggressive Tilt",   shape_angry_slanted   },
    { "Worried Slant",     shape_sad_worried     },
    { "Happy Top Arcs",    shape_happy_arcs      },
    { "Cyber Visor",       shape_visor_slits     },
    { "Sci-Fi Squircle",   shape_squircle        },
    { "Mono Visor",        shape_cyclops_visor   },
    { "Diamond Cat",       shape_diamond_cat     },
    { "Flat Triangle",     shape_flat_triangles  },
    { "Glitch Bars",       shape_glitch_bars     },
    { "Hourglass",         shape_hourglass       },
    { "Crosshairs",        shape_crosses         },
    { "Tech Hexagon",      shape_hexagon         },
    { "Downward Arcs",     shape_inverted_arcs   },
    { "Chevron Matrix",    shape_chevron_outer   },
    { "Column Slits",      shape_tall_slits      },
    { "Industrial Goggle", shape_goggles         },
    { "Bracket Box",       shape_bracket_boxes   },
};
const uint8_t NUM_EYE_SHAPES = sizeof(EYE_SHAPES) / sizeof(EYE_SHAPES[0]);

struct EyeColor { const char* name; uint8_t r, g, b; };
const EyeColor EYE_COLORS[] = {
    { "Neon Cyan",        0,   255, 255 },
    { "Plasma Red",       255, 50,  50  },
    { "Laser Green",      50,  255, 50  },
    { "Matrix Amber",     240, 160, 0   },
    { "Synthetic Pink",   255, 0,   180 },
    { "Overdrive White",  240, 240, 255 },
    { "High-Volt Yellow", 255, 230, 0   },
    { "Deep Cobalt",      0,   100, 255 },
    { "Radioactive Lime", 175, 255, 0   },
    { "Vapor Purple",     160, 32,  240 },
};
const uint8_t NUM_EYE_COLORS = sizeof(EYE_COLORS) / sizeof(EYE_COLORS[0]);

uint8_t face_shape_idx = 0;
uint8_t face_color_idx = 0;

uint16_t faceColor() {
    const EyeColor &c = EYE_COLORS[face_color_idx];
    return rgb565(c.r, c.g, c.b);
}


void loadUiSettingsFromFlash() {
    Preferences p;
    p.begin("hexapod-ui", true);
    COG_OFFSET       = constrain(p.getFloat("cog",   COG_OFFSET),     COG_OFFSET_MIN, COG_OFFSET_MAX);
    NEUTRAL_HEIGHT   = constrain(p.getFloat("bodyH", NEUTRAL_HEIGHT), NEUTRAL_H_MIN,  NEUTRAL_H_MAX);
    STEP_HEIGHT      = constrain(p.getFloat("stepH", STEP_HEIGHT),    STEP_H_MIN,     STEP_H_MAX);
    idle_motion_enabled = p.getBool("idle", true);
    face_shape_idx   = p.getUChar("shape", 0);
    face_color_idx   = p.getUChar("color", 0);
    ui_dark_mode     = p.getBool("dark", true);
    imuUserPitchZero = p.getFloat("imuZP", 0.0f);
    imuUserRollZero  = p.getFloat("imuZR", 0.0f);
    p.end();
    if (face_shape_idx >= NUM_EYE_SHAPES) face_shape_idx = 0;
    if (face_color_idx >= NUM_EYE_COLORS) face_color_idx = 0;
}

void saveBodyParamsToFlash() {
    Preferences p; p.begin("hexapod-ui", false);
    p.putFloat("cog", COG_OFFSET); p.putFloat("bodyH", NEUTRAL_HEIGHT); p.putFloat("stepH", STEP_HEIGHT);
    p.putBool("idle", idle_motion_enabled);
    p.end();
}
void saveFaceToFlash() {
    Preferences p; p.begin("hexapod-ui", false);
    p.putUChar("shape", face_shape_idx); p.putUChar("color", face_color_idx);
    p.end();
}
void saveThemeToFlash() {
    Preferences p; p.begin("hexapod-ui", false);
    p.putBool("dark", ui_dark_mode);
    p.end();
}
void saveImuZeroToFlash() {
    Preferences p; p.begin("hexapod-ui", false);
    p.putFloat("imuZP", imuUserPitchZero); p.putFloat("imuZR", imuUserRollZero);
    p.end();
}


struct EyeAnimState {
    float    openRatio   = 1.0f;
    uint8_t  blinkPhase  = 0;          // 0 idle, 1 closing, 2 opening
    int      offset      = 0;
    unsigned long nextBlinkMs = 0, nextLookMs = 0, lastTickMs = 0;
    // what is currently on the glass
    int      drawnOffset = -9999, drawnH = -1;
    uint8_t  drawnShape  = 255, drawnColor = 255;
    uint16_t drawnBg     = 0;
} eyeAnim;

const float EYE_CLOSE_RATE = 15.0f;    // openRatio units / s  (~65 ms to close)
const float EYE_OPEN_RATE  = 12.0f;    // (~85 ms to open)

void eyeAnimReset() {
    eyeAnim.openRatio = 1.0f;
    eyeAnim.blinkPhase = 0;
    eyeAnim.offset = 0;
    eyeAnim.nextBlinkMs = millis() + random(3000, 6000);
    eyeAnim.nextLookMs  = millis() + random(2000, 4000);
    eyeAnim.lastTickMs  = millis();
    eyeAnim.drawnH = -1;               // force a redraw
}

bool eyeAnimBlinking() { return eyeAnim.blinkPhase != 0; }

// idle=true -> glance around + blink; idle=false -> eyes open, centered.
void eyeAnimTick(bool idle) {
    unsigned long now = millis();
    float dt = constrain((now - eyeAnim.lastTickMs) / 1000.0f, 0.0f, 0.2f);
    eyeAnim.lastTickMs = now;

    if (!idle) {
        eyeAnim.offset = 0;
        eyeAnim.openRatio = 1.0f;
        eyeAnim.blinkPhase = 0;
        eyeAnim.nextBlinkMs = now + random(5000, 10000);
        return;
    }

    if (now > eyeAnim.nextLookMs) {
        eyeAnim.nextLookMs = now + random(3000, 6000);
        int roll = random(0, 3);
        eyeAnim.offset = (roll == 0) ? -25 : (roll == 1 ? 25 : 0);
    }

    if (eyeAnim.blinkPhase == 0 && now > eyeAnim.nextBlinkMs) eyeAnim.blinkPhase = 1;
    if (eyeAnim.blinkPhase == 1) {
        eyeAnim.openRatio -= EYE_CLOSE_RATE * dt;
        if (eyeAnim.openRatio <= 0.0f) { eyeAnim.openRatio = 0.0f; eyeAnim.blinkPhase = 2; }
    } else if (eyeAnim.blinkPhase == 2) {
        eyeAnim.openRatio += EYE_OPEN_RATE * dt;
        if (eyeAnim.openRatio >= 1.0f) {
            eyeAnim.openRatio = 1.0f;
            eyeAnim.blinkPhase = 0;
            eyeAnim.nextBlinkMs = now + random(5000, 10000);
        }
    }
}

void eyeAnimDraw() {
    int h = max(4, (int)(eyeHeight * eyeAnim.openRatio));   // same floor as the mock
    uint16_t bg = uiBg();
    if (h == eyeAnim.drawnH && eyeAnim.offset == eyeAnim.drawnOffset &&
        face_shape_idx == eyeAnim.drawnShape && face_color_idx == eyeAnim.drawnColor &&
        bg == eyeAnim.drawnBg) return;

    int midX = tft.width() / 2, midY = tft.height() / 2;
    tft.fillRect(0, midY - eyeHeight / 2 - 2, tft.width(), eyeHeight + 4, bg);

    uint16_t col = faceColor();
    EYE_SHAPES[face_shape_idx].draw(midX - eyeSpacing - eyeWidth + eyeAnim.offset,
                                    midX + eyeSpacing + eyeAnim.offset,
                                    midY, eyeWidth, h, col);

    eyeAnim.drawnH = h; eyeAnim.drawnOffset = eyeAnim.offset;
    eyeAnim.drawnShape = face_shape_idx; eyeAnim.drawnColor = face_color_idx; eyeAnim.drawnBg = bg;
}

// ------------------------------------------------------------
// Settings menu state
// ------------------------------------------------------------
enum MenuLayer : uint8_t { LAYER_MAIN, LAYER_BODY, LAYER_IMU, LAYER_SERVO, LAYER_FACE, LAYER_EMOTE };

const char* MENU_MAIN_ITEMS[] = { "Emote Mode", "Active Gait Settings", "IMU Calibration", "Servo Calibration", "Face Settings", "Theme Mode" };
const char* MENU_BODY_ITEMS[] = { "COG Offset", "Walk Height", "Step Height",
                                  "Idle Motion", "Save & Return" };
const int   MENU_MAIN_COUNT = 6, MENU_BODY_COUNT = 5;
const int   BODY_ROW_IDLE = 3;     // toggled with L2/R2, like any other value


enum { ROW_EMOTE = 0, ROW_BODY, ROW_IMU, ROW_SERVO, ROW_FACE, ROW_THEME };

const char* MENU_IMU_ITEMS[] = { "Zero Level Now", "Run Calibration Dance" };
const int   MENU_IMU_COUNT = 2;
enum { IMU_ROW_ZERO = 0, IMU_ROW_DANCE = 1 };

// Emote list scrolling: 4 rows fit under the header at text size 2.
const int EMOTE_ROWS_VISIBLE = 4;
int emoteListTop = 0;

MenuLayer menuLayer = LAYER_MAIN;
int       menuIdx   = 0;
bool      menuNeedsFullRedraw = true;   // clear + header + page
bool      menuNeedsPageRedraw = true;   // page content only

// Values captured on entering a sub-page, restored on B / cancel
float   bak_cog, bak_bodyH, bak_stepH;
bool    bak_idle;
float   bak_servoOffsets[18];
uint8_t bak_shape, bak_color;

unsigned long toastUntilMs   = 0;       // "SAVED!" in the header
unsigned long imuZeroedAtMs  = 0;       // "CALIBRATED!" on the IMU page
const unsigned long TOAST_MS = 1500;

void menuGoto(MenuLayer layer, int idx) {
    menuLayer = layer;
    menuIdx = idx;
    menuNeedsFullRedraw = true;
    if (layer == LAYER_FACE) eyeAnimReset();
}

void menuToastSaved() { toastUntilMs = millis() + TOAST_MS; menuNeedsFullRedraw = true; }

void setNeutralHeight(float h) {
    NEUTRAL_HEIGHT = h;
    user_selected_height = h;          // next stand-up uses it; triggers still trim live
}

// Discard unsaved edits of the current sub-page (B, or leaving the menu).
void menuRevertCurrentPage() {
    switch (menuLayer) {
        case LAYER_BODY:
            COG_OFFSET = bak_cog; STEP_HEIGHT = bak_stepH; setNeutralHeight(bak_bodyH);
            idle_motion_enabled = bak_idle;
            break;
        case LAYER_SERVO:
            for (int i = 0; i < 18; i++) servoOffsets[i] = bak_servoOffsets[i];
            in_calibration_mode = false;
            break;
        case LAYER_FACE:
            face_shape_idx = bak_shape; face_color_idx = bak_color;
            break;
        default: break;
    }
}

void menuImuZero() {

    imu_zero_request = true;
    imuZeroedAtMs = millis();
}


void menuHandleEvent(uint8_t ev) {
    bool up = ev == UI_UP, down = ev == UI_DOWN, left = ev == UI_LEFT,
         right = ev == UI_RIGHT, enter = ev == UI_ENTER, back = ev == UI_BACK;

    switch (menuLayer) {
    case LAYER_MAIN:
        if (down)      { menuIdx = (menuIdx + 1) % MENU_MAIN_COUNT; menuNeedsPageRedraw = true; }
        else if (up)   { menuIdx = (menuIdx - 1 + MENU_MAIN_COUNT) % MENU_MAIN_COUNT; menuNeedsPageRedraw = true; }
        else if (enter) {
            switch (menuIdx) {
                case ROW_EMOTE:

                    emote_menu_requested = true;
                    emoteListTop = 0;
                    menuNeedsFullRedraw = true;
                    break;
                case ROW_BODY:
                    bak_cog = COG_OFFSET; bak_bodyH = NEUTRAL_HEIGHT; bak_stepH = STEP_HEIGHT;
                    bak_idle = idle_motion_enabled;
                    menuGoto(LAYER_BODY, 0);
                    break;
                case ROW_IMU:
                    menuGoto(LAYER_IMU, 0);
                    break;
                case ROW_SERVO:
                    for (int i = 0; i < 18; i++) bak_servoOffsets[i] = servoOffsets[i];
                    in_calibration_mode = true;       // gait task powers servos to neutral pose
                    menuGoto(LAYER_SERVO, 0);
                    break;
                case ROW_FACE:
                    bak_shape = face_shape_idx; bak_color = face_color_idx;
                    menuGoto(LAYER_FACE, 0);
                    break;
                case ROW_THEME:
                    ui_dark_mode = !ui_dark_mode;
                    saveThemeToFlash();
                    menuNeedsFullRedraw = true;
                    break;
            }
        }
        break;

    case LAYER_BODY:
        if (down)    { menuIdx = (menuIdx + 1) % MENU_BODY_COUNT; menuNeedsPageRedraw = true; }
        else if (up) { menuIdx = (menuIdx - 1 + MENU_BODY_COUNT) % MENU_BODY_COUNT; menuNeedsPageRedraw = true; }
        else if ((left || right) && menuIdx == BODY_ROW_IDLE) {
            idle_motion_enabled = !idle_motion_enabled;   // either direction flips it
            menuNeedsPageRedraw = true;
        }
        else if ((left || right) && menuIdx < 3) {
            int dir = right ? +1 : -1;
            if (menuIdx == 0) COG_OFFSET  = stepValue(COG_OFFSET,  COG_OFFSET_STEP, dir, COG_OFFSET_MIN, COG_OFFSET_MAX);
            if (menuIdx == 1) setNeutralHeight(stepValue(NEUTRAL_HEIGHT, NEUTRAL_H_STEP, dir, NEUTRAL_H_MIN, NEUTRAL_H_MAX));
            if (menuIdx == 2) STEP_HEIGHT = stepValue(STEP_HEIGHT, STEP_H_STEP, dir, STEP_H_MIN, STEP_H_MAX);
            menuNeedsPageRedraw = true;
        }
        else if (enter) {                                   // A on any row = save & return
            saveBodyParamsToFlash();
            menuGoto(LAYER_MAIN, ROW_BODY);
            menuToastSaved();
        }
        else if (back) {
            menuRevertCurrentPage();
            menuGoto(LAYER_MAIN, ROW_BODY);
        }
        break;

    case LAYER_IMU:
        if (down)      { menuIdx = (menuIdx + 1) % MENU_IMU_COUNT; menuNeedsPageRedraw = true; }
        else if (up)   { menuIdx = (menuIdx - 1 + MENU_IMU_COUNT) % MENU_IMU_COUNT; menuNeedsPageRedraw = true; }
        else if (enter) {
            if (menuIdx == IMU_ROW_ZERO) {
                menuImuZero();
            } else {

                cal_menu_requested = true;
                menuNeedsPageRedraw = true;
            }
        }
        else if (back)  menuGoto(LAYER_MAIN, ROW_IMU);
        break;

    case LAYER_SERVO:
        if (down)       { selectedJointID = (selectedJointID + 1) % 18;      menuNeedsPageRedraw = true; }
        else if (up)    { selectedJointID = (selectedJointID - 1 + 18) % 18; menuNeedsPageRedraw = true; }
        else if (right) { servoOffsets[selectedJointID] += 0.5f;             menuNeedsPageRedraw = true; }
        else if (left)  { servoOffsets[selectedJointID] -= 0.5f;             menuNeedsPageRedraw = true; }
        else if (enter) {
            saveOffsetsToFlash();
            in_calibration_mode = false;
            menuGoto(LAYER_MAIN, ROW_SERVO);
            menuToastSaved();
        }
        else if (back) {
            menuRevertCurrentPage();
            menuGoto(LAYER_MAIN, ROW_SERVO);
        }
        break;

    case LAYER_EMOTE:

        if (NUM_EMOTES == 0) {
            if (back || enter) emote_exit_request = true;
            break;
        }
        if (down)      { selected_emote_id = (selected_emote_id + 1) % NUM_EMOTES;      menuNeedsPageRedraw = true; }
        else if (up)   { selected_emote_id = (selected_emote_id + NUM_EMOTES - 1) % NUM_EMOTES; menuNeedsPageRedraw = true; }
        else if (enter) {
            emote_play_request = true;               // gait task starts it
            menuNeedsPageRedraw = true;
        }
        else if (back) {
            emote_exit_request = true;               // leave, stay standing
        }
        break;

    case LAYER_FACE:
        if (down)       face_shape_idx = (face_shape_idx + 1) % NUM_EYE_SHAPES;
        else if (up)    face_shape_idx = (face_shape_idx + NUM_EYE_SHAPES - 1) % NUM_EYE_SHAPES;
        else if (right) face_color_idx = (face_color_idx + 1) % NUM_EYE_COLORS;
        else if (left)  face_color_idx = (face_color_idx + NUM_EYE_COLORS - 1) % NUM_EYE_COLORS;
        else if (enter) { saveFaceToFlash(); menuGoto(LAYER_MAIN, ROW_FACE); menuToastSaved(); }
        else if (back)  { menuRevertCurrentPage(); menuGoto(LAYER_MAIN, ROW_FACE); }
        if (menuLayer == LAYER_FACE && (up || down || left || right)) menuNeedsPageRedraw = true;
        break;
    }
}


void menuCancelToMain() {
    if (menuLayer != LAYER_MAIN) menuRevertCurrentPage();
    menuLayer = LAYER_MAIN;
    menuIdx = 0;
    menuNeedsFullRedraw = true;
}


const int MENU_X = 15, MENU_Y0 = 44, MENU_ROW_H = 32, MENU_COLS = 25;

void menuLine(int x, int y, uint16_t fg, const char* fmt, ...) {
    char buf[48], padded[48];
    va_list args; va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    int cols = (tft.width() - x) / 12;
    if (cols > MENU_COLS) cols = MENU_COLS;
    snprintf(padded, sizeof(padded), "%-*.*s", cols, cols, buf);
    tft.setTextSize(2);
    tft.setTextColor(fg, uiBg());
    tft.setCursor(x, y);
    tft.print(padded);
}

void menuSmallLine(int x, int y, uint16_t fg, const char* text) {
    tft.setTextSize(1);
    tft.setTextColor(fg, uiBg());
    tft.setCursor(x, y);
    tft.print(text);
}

void menuDrawHeader() {
    static char lastRight[24] = "";
    static bool lastToast = false;
    uint16_t bg = uiBg();

    if (menuNeedsFullRedraw) { lastRight[0] = '\0'; }

    // Title / "SAVED!" toast
    bool toast = millis() < toastUntilMs;
    if (toast != lastToast || menuNeedsFullRedraw) {
        tft.setTextSize(2);
        tft.setCursor(10, 12);
        tft.setTextColor(toast ? uiAccent() : TFT_ORANGE, bg);
        tft.print(toast ? "SAVED!  " : (menuLayer == LAYER_EMOTE ? "EMOTES  " : "SETTINGS"));
        lastToast = toast;
    }

    // Uptime | battery %, flashing red below 15 % (3S pack: 9.6 V = 0 %, 12.6 V = 100 %)
    float v = batteryVoltage;
    int pct = (int)constrain(((v - 9.6f) / 3.0f) * 100.0f, 0.0f, 100.0f);
    unsigned long mins = millis() / 60000UL;
    bool low = pct < 15;
    bool visible = !(low && ((millis() / 200) % 2 == 0));

    char right[24];
    // "BT" prefix while a Bluetooth gamepad is connected
    if (visible) snprintf(right, sizeof(right), "%s%lum | %d%%", btControllerConnected ? "BT  " : "", mins, pct);
    else         right[0] = '\0';

    if (menuNeedsFullRedraw || strcmp(right, lastRight) != 0) {
        tft.fillRect(130, 10, tft.width() - 130, 20, bg);
        if (visible) {
            tft.setTextSize(2);
            tft.setTextColor(low ? TFT_RED : uiAccent(), bg);
            tft.setCursor(tft.width() - 10 - tft.textWidth(right), 12);
            tft.print(right);
        }
        strncpy(lastRight, right, sizeof(lastRight));
    }

    if (menuNeedsFullRedraw) tft.drawFastHLine(0, 36, tft.width(), TFT_DARKGREY);
}

void menuDrawPage() {
    char val[20];
    switch (menuLayer) {
    case LAYER_MAIN:
        if (!menuNeedsPageRedraw) break;
        for (int i = 0; i < MENU_MAIN_COUNT; i++) {
            bool sel = (i == menuIdx);
            if (i == ROW_THEME) menuLine(MENU_X, MENU_Y0 + i * MENU_ROW_H, sel ? uiSel() : uiText(),
                                 "%s%s: %s", sel ? "> " : "  ", MENU_MAIN_ITEMS[i], ui_dark_mode ? "Dark" : "Light");
            else        menuLine(MENU_X, MENU_Y0 + i * MENU_ROW_H, sel ? uiSel() : uiText(),
                                 "%s%s", sel ? "> " : "  ", MENU_MAIN_ITEMS[i]);
        }
        break;

    case LAYER_BODY:
        if (!menuNeedsPageRedraw) break;
        for (int i = 0; i < MENU_BODY_COUNT; i++) {
            bool sel = (i == menuIdx);
            if      (i == 0) snprintf(val, sizeof(val), ": %.2f cm", COG_OFFSET * 100.0f);
            else if (i == 1) snprintf(val, sizeof(val), ": %.0f cm", NEUTRAL_HEIGHT * 100.0f);
            else if (i == 2) snprintf(val, sizeof(val), ": %.2f cm", STEP_HEIGHT * 100.0f);
            else if (i == BODY_ROW_IDLE) snprintf(val, sizeof(val), ": %s",
                                                  idle_motion_enabled ? "On " : "Off");
            else             val[0] = '\0';
            menuLine(MENU_X, MENU_Y0 + i * MENU_ROW_H, sel ? uiSel() : uiText(),
                     "%s%s%s", sel ? "> " : "  ", MENU_BODY_ITEMS[i], val);
        }
        break;

    case LAYER_IMU: {
        if (menuNeedsPageRedraw) {
            for (int i = 0; i < MENU_IMU_COUNT; i++) {
                bool sel = (i == menuIdx);
                menuLine(MENU_X, MENU_Y0 + i * MENU_ROW_H, sel ? uiSel() : uiText(),
                         "%s%s", sel ? "> " : "  ", MENU_IMU_ITEMS[i]);
            }
        }
        // Live attitude, so you can see the zero take effect.
        char small[52];
        snprintf(small, sizeof(small), "ROLL %+6.2f   PITCH %+6.2f deg   ",
                 degrees(body_roll_filtered), degrees(body_pitch_filtered));
        menuSmallLine(20, 130, uiText(), small);
        snprintf(small, sizeof(small), "cal: %-24s", calStatusMsgBuf);
        menuSmallLine(20, 146, calDone() ? uiAccent() : uiHint(), small);

        if (imuZeroedAtMs != 0 && millis() - imuZeroedAtMs < TOAST_MS)
            menuLine(20, 168, uiAccent(), "ZEROED!      ");
        else
            menuLine(20, 168, uiHint(), "             ");

        if (menuNeedsPageRedraw) {
            menuSmallLine(10, 200, uiHint(), "The DANCE stands the robot up and moves it.");
            menuSmallLine(10, 212, uiHint(), "Put it on LEVEL ground and do not touch it.");
            menuSmallLine(10, 224, uiHint(), "L1/R1: choose    A: run    B: back");
        }
        break;
    }

    case LAYER_SERVO:
        if (!menuNeedsPageRedraw) break;
        menuLine(MENU_X, MENU_Y0, uiTitle(), "SERVO ID:%02d", (int)selectedJointID);
        menuLine(20, 80,  uiSel(), "%s", JOINT_NAMES[selectedJointID]);
        menuLine(20, 112, uiSel(), "Offset: %.1f", servoOffsets[selectedJointID]);
        menuLine(20, 150, uiText(), "L1/R1 select servo");
        menuLine(20, 182, uiText(), "L2/R2 adjust offset");
        menuSmallLine(20, 214, uiHint(), "A: save    B: cancel");
        break;

    case LAYER_EMOTE: {

        static bool wasPlayingEmote = false;
        bool nowPlaying = emote_playing && emote_playing_id < NUM_EMOTES;
        if (nowPlaying != wasPlayingEmote) {
            wasPlayingEmote = nowPlaying;
            tft.fillScreen(uiBg());
            eyeAnimReset();
            menuNeedsPageRedraw = true;
        }
        if (nowPlaying) {
            eyeAnimTick(true);          // blink + glance, same as standing
            eyeAnimDraw();
            if (menuNeedsPageRedraw) {
                char pl[48];
                snprintf(pl, sizeof(pl), "PLAYING: %s", EMOTES[emote_playing_id].name);
                menuSmallLine(10, 222, uiAccent(), pl);
                menuNeedsPageRedraw = false;
            }
            break;
        }


        int sel = (int)selected_emote_id;
        int oldTop = emoteListTop;
        if (sel < emoteListTop) emoteListTop = sel;
        if (sel >= emoteListTop + EMOTE_ROWS_VISIBLE) emoteListTop = sel - EMOTE_ROWS_VISIBLE + 1;
        if (emoteListTop > (int)NUM_EMOTES - EMOTE_ROWS_VISIBLE) emoteListTop = (int)NUM_EMOTES - EMOTE_ROWS_VISIBLE;
        if (emoteListTop < 0) emoteListTop = 0;
        if (oldTop != emoteListTop) menuNeedsPageRedraw = true;

        if (menuNeedsPageRedraw) {
            for (int r = 0; r < EMOTE_ROWS_VISIBLE; r++) {
                int i = emoteListTop + r;
                int y = MENU_Y0 + r * MENU_ROW_H;
                if (i >= (int)NUM_EMOTES) { menuLine(MENU_X, y, uiText(), " "); continue; }
                bool isSel = (i == sel);
                menuLine(MENU_X, y, isSel ? uiSel() : uiText(),
                         "%s%s", isSel ? "> " : "  ", EMOTES[i].name);
            }
        }

        // Status line: what is playing, or the hint. Updated every tick.
        char st[48];
        if (emote_playing && emote_playing_id < NUM_EMOTES) {
            float t_e = (millis() - emote_start_ms) / 1000.0f;
            snprintf(st, sizeof(st), "PLAYING %-10s %.1f/%.1fs   ",
                     EMOTES[emote_playing_id].name, t_e, EMOTES[emote_playing_id].duration_sec);
            menuSmallLine(10, 196, uiAccent(), st);
        } else {
            snprintf(st, sizeof(st), "%d/%d   A: play   B: exit (stay up)      ",
                     sel + 1, (int)NUM_EMOTES);
            menuSmallLine(10, 196, uiHint(), st);
        }
        if (menuNeedsPageRedraw) menuSmallLine(10, 212, uiHint(), "L1/R1: choose emote");
        break;
    }

    case LAYER_FACE:
        eyeAnimTick(true);
        eyeAnimDraw();
        if (menuNeedsPageRedraw) {
            char cap[64];
            snprintf(cap, sizeof(cap), "L1/R1 Shape %2d/%-2d %-17s", face_shape_idx + 1, NUM_EYE_SHAPES,
                     EYE_SHAPES[face_shape_idx].name);
            menuSmallLine(10, 194, uiText(), cap);
            snprintf(cap, sizeof(cap), "L2/R2 Color %2d/%-2d %-17s", face_color_idx + 1, NUM_EYE_COLORS,
                     EYE_COLORS[face_color_idx].name);
            menuSmallLine(10, 208, uiText(), cap);
            menuSmallLine(10, 224, uiHint(), "A: save    B: cancel");
        }
        break;
    }
    menuNeedsPageRedraw = false;
}

void menuRender() {

    if (menuLayer == LAYER_SERVO && !in_calibration_mode && !emote_mode_enabled) {
        menuRevertCurrentPage();
        menuGoto(LAYER_MAIN, ROW_SERVO);
    }

    if (menuNeedsFullRedraw) {
        tft.fillScreen(uiBg());
        menuNeedsPageRedraw = true;
        eyeAnim.drawnH = -1;
    }
    if (menuLayer != LAYER_FACE) menuDrawHeader();   // face preview is eyes-only, like the mock
    menuDrawPage();
    menuNeedsFullRedraw = false;
}

// ------------------------------------------------------------
// Telemetry shell (transition + emote screens)
// ------------------------------------------------------------
void setupDiagnosticUI() {
    tft.setRotation(SCREEN_ROTATION);
    tft.fillScreen(MY_BLACK); tft.setTextColor(TFT_WHITE, MY_BLACK); tft.setTextSize(2);
    tft.setCursor(10, 10); tft.println("HEXAPOD CORE TELEMETRY"); tft.drawFastHLine(0, 32, tft.width(), TFT_DARKGREY);
}


void readBattery() {
    int rawADC = analogRead(VOLTAGE_PIN);
    float measuredPinVoltage = (rawADC / 4095.0) * 3.3 * ADC_CAL_FACTOR;
    batteryVoltage = measuredPinVoltage * DIVIDER_RATIO;
}

void drawTelemetryScreen(uint8_t displayState) {
    tft.setTextSize(2);
    tft.setCursor(10, 35);
    if (displayState == 3) {
        tft.setTextColor(MY_CYAN, MY_BLACK);
        tft.print("MODE: EMOTE     ");
    } else {
        tft.setTextColor(TFT_YELLOW, MY_BLACK);
        tft.print("MODE: SHIFTING  ");
    }

    tft.setCursor(10, 65);
    if (displayState == 3) {
        tft.setTextSize(1);
        if (emote_playing && emote_playing_id < NUM_EMOTES) {
            tft.setTextColor(TFT_GREEN, MY_BLACK);
            tft.printf("PLAYING: %s                    ", EMOTES[emote_playing_id].name);
            tft.setCursor(10, 78);
            float t_e = (millis() - emote_start_ms) / 1000.0f;
            float dur = EMOTES[emote_playing_id].duration_sec;
            tft.setTextColor(TFT_YELLOW, MY_BLACK);
            tft.printf("T: %.1fs / %.1fs               ", t_e, dur);
        } else {
            tft.setTextColor(TFT_DARKGREY, MY_BLACK);
            tft.print("READY                                    ");
            tft.setCursor(10, 78);
            tft.setTextColor(TFT_GREEN, MY_BLACK);
            if (selected_emote_id < NUM_EMOTES) {
                tft.printf("SELECTED: %s                    ", EMOTES[selected_emote_id].name);
            } else {
                tft.print("SELECTED: --                              ");
            }
        }
    } else {
        tft.fillRect(10, 65, 230, 50, MY_BLACK);
        float volt_copy = batteryVoltage;
        if (volt_copy < 10.2) tft.setTextColor(TFT_RED, MY_BLACK);
        else if (volt_copy < 11.1) tft.setTextColor(TFT_YELLOW, MY_BLACK);
        else tft.setTextColor(MY_CYAN, MY_BLACK);
        tft.printf("BAT: %2.2f V  ", volt_copy);

        tft.setTextSize(1);
        tft.setCursor(10, 100);
        tft.setTextColor(balance_enabled ? MY_CYAN : TFT_DARKGREY, MY_BLACK);
        tft.printf("BALANCE: %s   ", balance_enabled ? "ON " : "OFF");
        tft.setCursor(140, 100);
        tft.setTextColor(dirt_mode_enabled ? MY_CYAN : TFT_DARKGREY, MY_BLACK);
        tft.printf("DIRT: %s   ", dirt_mode_enabled ? "ON " : "OFF");
    }

    tft.setTextSize(2);
    tft.setTextColor(TFT_GREEN, MY_BLACK);
    tft.setCursor(10, 120);
    tft.printf("IMU X: %1.2f m/s2  ", imu_ax);
    tft.setCursor(10, 145);
    tft.printf("IMU Y: %1.2f m/s2  ", imu_ay);
    tft.setCursor(10, 170);
    tft.printf("IMU Z: %1.2f m/s2  ", imu_az);
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW, MY_BLACK);
    tft.setCursor(10, 195);
    tft.printf("ROLL: %+6.1f deg   ", degrees(body_roll_filtered));
    tft.setCursor(10, 207);
    tft.printf("PITCH:%+6.1f deg   ", degrees(body_pitch_filtered));
}

// ------------------------------------------------------------
// Telemetry Rendering, Outbound Transmitter, Eye Engine & Settings GUI (Core 0)
// ------------------------------------------------------------
enum ScreenMode : uint8_t { SCREEN_MENU = 0, SCREEN_EYES = 1, SCREEN_TELEM = 2,
                            SCREEN_EMOTE = 3, SCREEN_CALDANCE = 4, SCREEN_NONE = 255 };

void TelemetryTask(void * pvParameters) {
    unsigned long lastBatteryTime = 0;
    unsigned long lastUdpTxTime   = 0;


    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW, MY_BLACK);
    tft.setCursor(10, 50);
    tft.print("Calibrating IMU gyro...");

    calibrateGyroBias();

    tft.setCursor(10, 62);
    tft.setTextColor(TFT_GREEN, MY_BLACK);
    tft.printf("Done. Bias X:%.4f Y:%.4f", gyroBiasX, gyroBiasY);
    vTaskDelay(pdMS_TO_TICKS(600));

    readBattery();                       // so the menu header has a real % straight away

    uint8_t txTelemetryBuffer[28];
    ScreenMode lastScreenMode = SCREEN_NONE;

    for(;;) {


        if (millis() - lastBatteryTime >= 2000) {
            lastBatteryTime = millis();
            readBattery();
        }


        if (imu_zero_save_request) {
            imu_zero_save_request = false;
            saveImuZeroToFlash();
        }

        uint8_t displayState;
        if (in_calibration_mode) {
            displayState = 2;
        } else if (emote_mode_enabled) {
            displayState = 3;
        } else if (target_standing_state) {
            displayState = 1;
        } else {
            displayState = 0;
        }

        // ----- UDP TELEMETRY OUTBOUND TRANSMITTER (28-BYTE PACKET, unchanged) -----
        if (millis() - lastUdpTxTime >= 50) {
            lastUdpTxTime = millis();

            IPAddress remoteIp = udp.remoteIP();
            uint16_t remotePort = udp.remotePort();

            if (remoteIp[0] != 0) {
                float currentActiveOffset = servoOffsets[selectedJointID];
                int32_t activeJointInt = (int32_t)selectedJointID;
                uint8_t playing_id_out = emote_playing ? emote_playing_id : EMOTE_NONE;

                memcpy(&txTelemetryBuffer[0],  (const void*)&imu_ax, 4);
                memcpy(&txTelemetryBuffer[4],  (const void*)&imu_ay, 4);
                memcpy(&txTelemetryBuffer[8],  (const void*)&imu_az, 4);
                memcpy(&txTelemetryBuffer[12], (const void*)&batteryVoltage, 4);
                memcpy(&txTelemetryBuffer[16], &activeJointInt, 4);
                memcpy(&txTelemetryBuffer[20], (const void*)&currentActiveOffset, 4);
                txTelemetryBuffer[24] = displayState;
                txTelemetryBuffer[25] = balance_enabled ? 1 : 0;
                txTelemetryBuffer[26] = dirt_mode_enabled ? 1 : 0;
                txTelemetryBuffer[27] = playing_id_out;

                udp.beginPacket(remoteIp, remotePort);
                udp.write(txTelemetryBuffer, 28);
                udp.endPacket();
            }
        }

        // ----- Which screen? -----
        ScreenMode mode;

        if (calRunnerIsRunning()) {
            mode = SCREEN_CALDANCE;
        } else if (displayState == 3) {
            mode = SCREEN_EMOTE;          // emote list (standing, emote mode on)
        } else if (displayState == 2 || (displayState == 0 && current_transition_progress <= 0.0f)) {
            mode = SCREEN_MENU;
        } else if (displayState == 1 && current_transition_progress >= 0.5f) {
            mode = SCREEN_EYES;
        } else {
            mode = SCREEN_TELEM;
        }

        if (mode != lastScreenMode || (mode == SCREEN_TELEM && displayState != lastRenderedState)) {
            if (lastScreenMode == SCREEN_MENU && mode != SCREEN_MENU) menuCancelToMain();
            tft.setRotation(SCREEN_ROTATION);
            if (mode == SCREEN_CALDANCE) {
                tft.fillScreen(uiBg());
                tft.setTextSize(2);
                tft.setTextColor(TFT_ORANGE, uiBg());
                tft.setCursor(10, 20);  tft.print("IMU CALIBRATION");
                tft.setTextColor(uiText(), uiBg());
                tft.setTextSize(1);
                tft.setCursor(10, 60);  tft.print("The robot will bow and tilt itself.");
                tft.setCursor(10, 74);  tft.print("Keep it on LEVEL ground.");
                tft.setCursor(10, 88);  tft.print("DO NOT TOUCH until it finishes.");
                tft.setCursor(10, 112); tft.print("It stays standing afterwards, ready");
                tft.setCursor(10, 126); tft.print("to walk.");
            }
            else if (mode == SCREEN_EMOTE) {
                menuLayer = LAYER_EMOTE;       // the emote list IS the menu here
                menuNeedsFullRedraw = true;
            }
            else if (mode == SCREEN_MENU) {
                // Coming back from emote mode: land on the Emote Mode row.
                if (lastScreenMode == SCREEN_EMOTE) { menuLayer = LAYER_MAIN; menuIdx = ROW_EMOTE; }
                menuNeedsFullRedraw = true;
            }
            else if (mode == SCREEN_EYES) { tft.fillScreen(uiBg()); eyeAnimReset(); lastMovementTime = millis(); }
            else                          { setupDiagnosticUI(); }
            lastScreenMode = mode;
        }
        lastRenderedState = displayState;

        // ================= IMU CALIBRATION DANCE =================
        if (mode == SCREEN_CALDANCE) {
            xQueueReset(uiInputQueue);            // no menu input during the dance
            char st[52];
            snprintf(st, sizeof(st), "%-30s", calStatusMsgBuf);
            tft.setTextSize(2);
            tft.setTextColor(TFT_GREEN, uiBg());
            tft.setCursor(10, 160);
            tft.print(st);
            vTaskDelay(pdMS_TO_TICKS(120));
            continue;
        }

        // ================= EMOTE LIST (standing, emote mode) =================
        if (mode == SCREEN_EMOTE) {
            uint8_t ev;
            while (xQueueReceive(uiInputQueue, &ev, 0) == pdTRUE) menuHandleEvent(ev);
            menuRender();
            if (xQueueReceive(uiInputQueue, &ev, pdMS_TO_TICKS(60)) == pdTRUE) menuHandleEvent(ev);
            continue;
        }

        // ================= SETTINGS MENU (asleep) =================
        if (mode == SCREEN_MENU) {
            uint8_t ev;
            while (xQueueReceive(uiInputQueue, &ev, 0) == pdTRUE) menuHandleEvent(ev);
            menuRender();


            int waitMs = (menuLayer == LAYER_FACE) ? (eyeAnimBlinking() ? 15 : 30) : 50;
            if (xQueueReceive(uiInputQueue, &ev, pdMS_TO_TICKS(waitMs)) == pdTRUE) menuHandleEvent(ev);
            continue;
        }

        // Not on the menu: drop any stray queued input.
        xQueueReset(uiInputQueue);

        // ================= EYES (standing / walking) =================
        if (mode == SCREEN_EYES) {
            bool moving = (sqrt(joy_fwd*joy_fwd + joy_side*joy_side) > 0.05 || abs(joy_spin) > 0.05);
            if (moving) lastMovementTime = millis();
            bool idle = (millis() - lastMovementTime) >= 1000;

            eyeAnimTick(idle);
            eyeAnimDraw();


            static int lastWarnShown = -1;
            int warn = auto_sleep_secs_left;
            if (warn != lastWarnShown) {
                lastWarnShown = warn;
                tft.fillRect(0, tft.height() - 20, tft.width(), 20, uiBg());
                if (warn >= 0) {
                    char w[32];
                    snprintf(w, sizeof(w), "SLEEPING IN %d...", warn);
                    tft.setTextSize(2);
                    tft.setTextColor(TFT_ORANGE, uiBg());
                    tft.setCursor(10, tft.height() - 18);
                    tft.print(w);
                }
            }

            if (eyeAnimBlinking())  vTaskDelay(pdMS_TO_TICKS(15));
            else if (moving)        vTaskDelay(pdMS_TO_TICKS(100));
            else                    vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }

        // ================= TELEMETRY (transitions / emote) =================
        drawTelemetryScreen(displayState);
        vTaskDelay(pdMS_TO_TICKS(displayState == 3 ? 100 : 60));
    }
}

#endif // SCREEN_SETTINGS_H
