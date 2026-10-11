#include "apex_hud_widgets.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int imax(int a, int b) { return a > b ? a : b; }
static int imin(int a, int b) { return a < b ? a : b; }

/* Reference resolution the base_size_at_540p font sizes throughout this
 * file (and the main pipeline's own diagnostic text) were chosen against
 * -- this pipeline's 960x540 default. */
#define APEX_HUD_REF_HEIGHT 540.0f

int apex_hud_font_size(const image_buffer_t* img, int base_size_at_540p) {
    int scaled = (int)(base_size_at_540p * img->height / APEX_HUD_REF_HEIGHT + 0.5f);
    return imax(scaled, 4);
}

void apex_hud_draw_crosshair(image_buffer_t* img) {
    int side = imax((int)(img->height * 0.20f), 8);
    int half = side / 2;
    int cx = img->width / 2, cy = img->height / 2;
    int x1 = cx - half, y1 = cy - half, x2 = cx + half, y2 = cy + half;

    draw_rectangle(img, x1, y1, side, side, COLOR_GREEN, 1);

    int bracket_len = imax(side / 6, 2);
    int corners[4][4] = {
        {x1, y1, bracket_len, bracket_len},
        {x2, y1, -bracket_len, bracket_len},
        {x1, y2, bracket_len, -bracket_len},
        {x2, y2, -bracket_len, -bracket_len},
    };
    for (int i = 0; i < 4; i++) {
        int corner_x = corners[i][0], corner_y = corners[i][1], dx = corners[i][2], dy = corners[i][3];
        draw_line(img, corner_x, corner_y, corner_x + dx, corner_y, COLOR_GREEN, 1);
        draw_line(img, corner_x, corner_y, corner_x, corner_y + dy, COLOR_GREEN, 1);
    }

    int center_box_half = imax((int)(side * 0.18f) / 2, 2);
    int extend = (int)(side * 0.5f + 0.5f);
    draw_line(img, x1 - extend, cy, cx - center_box_half, cy, COLOR_GREEN, 1);
    draw_line(img, cx + center_box_half, cy, x2 + extend, cy, COLOR_GREEN, 1);
    draw_line(img, cx, y1 - extend, cx, cy - center_box_half, COLOR_GREEN, 1);
    draw_line(img, cx, cy + center_box_half, cx, y2 + extend, COLOR_GREEN, 1);

    int tick_len = imax(side / 20, 1);
    float step = half / 8.0f;
    for (int i = 1; i <= 8; i++) {
        int offset = (int)(i * step + 0.5f);
        for (int sign = -1; sign <= 1; sign += 2) {
            int tx = cx + sign * offset;
            draw_line(img, tx, cy - tick_len, tx, cy + tick_len, COLOR_GREEN, 1);
            int ty = cy + sign * offset;
            draw_line(img, cx - tick_len, ty, cx + tick_len, ty, COLOR_GREEN, 1);
        }
    }

    draw_rectangle(img, cx - center_box_half, cy - center_box_half,
                    center_box_half * 2, center_box_half * 2, COLOR_GREEN, 1);
    int tiny = imax(center_box_half / 2, 1);
    draw_line(img, cx - tiny, cy, cx + tiny, cy, COLOR_GREEN, 1);
    draw_line(img, cx, cy - tiny, cx, cy + tiny, COLOR_GREEN, 1);
}

void apex_hud_draw_track_boxes(image_buffer_t* img, const HudTrack* tracks, int n_tracks,
                                float scale_x, float scale_y) {
    for (int i = 0; i < n_tracks; i++) {
        const HudTrack* t = &tracks[i];
        int dx = (int)(t->x * scale_x), dy = (int)(t->y * scale_y);
        int dw = (int)(t->w * scale_x), dh = (int)(t->h * scale_y);
        draw_rectangle(img, dx, dy, dw, dh, APEX_HUD_TRACK_COLOR, 1);

        int center_x = dx + dw / 2, center_y = dy + dh / 2;
        draw_line(img, center_x - 5, center_y, center_x + 5, center_y, APEX_HUD_TRACK_COLOR, 1);
        draw_line(img, center_x, center_y - 5, center_x, center_y + 5, APEX_HUD_TRACK_COLOR, 1);

        char label[80];
        snprintf(label, sizeof(label), "%s %.0f%%", t->class_name, t->score * 100.0f);
        int fsz = apex_hud_font_size(img, 12);
        draw_text(img, label, dx, imax(12, dy - 2), APEX_HUD_TRACK_COLOR, fsz);
    }
}

void apex_hud_draw_accelerometer(image_buffer_t* img, HudBounds b, int has_data,
                                  float accel_x_ms2, float accel_y_ms2) {
    int cx = (b.x1 + b.x2) / 2, cy = (b.y1 + b.y2) / 2;
    int w = b.x2 - b.x1, h = b.y2 - b.y1;
    int radius = (int)((imin(w, h) / 2 - 2) * 0.5f);
    if (radius <= 0) return;

    unsigned int grid_color = has_data ? COLOR_GREEN : APEX_HUD_NO_DATA_COLOR;
    unsigned int dot_color = has_data ? APEX_HUD_TRACK_COLOR : APEX_HUD_NO_DATA_COLOR;

    draw_circle(img, cx, cy, radius, grid_color, 1);
    static const float RING_FRACS[3] = {0.25f, 0.5f, 0.75f};
    for (int i = 0; i < 3; i++) {
        int ring_radius = (int)(radius * RING_FRACS[i] + 0.5f);
        if (ring_radius > 0) draw_circle(img, cx, cy, ring_radius, grid_color, 1);
    }

    static const int SPOKE_ANGLES_DEG[8] = {0, 45, 90, 135, 180, 225, 270, 315};
    for (int i = 0; i < 8; i++) {
        float rad = (SPOKE_ANGLES_DEG[i] - 90) * (float)M_PI / 180.0f;
        int ex = cx + (int)(radius * cosf(rad) + 0.5f);
        int ey = cy + (int)(radius * sinf(rad) + 0.5f);
        draw_line(img, cx, cy, ex, ey, grid_color, 1);
    }

    /* F(ront)/R(ight)/B(ack)/L(eft) labels -- approximate centering
     * (no text-metrics query available in this draw_text() API, unlike
     * cv2.getTextSize() in the original). Offset by the label's own font
     * size (not a fixed 7px) so the glyph clears the circle instead of
     * sitting half-inside it once the font scales up with resolution. */
    struct { int angle; const char* label; } cardinals[4] = {{0, "F"}, {90, "R"}, {180, "B"}, {270, "L"}};
    int cardinal_fsz = apex_hud_font_size(img, 10);
    int label_offset = radius + cardinal_fsz + 4;
    for (int i = 0; i < 4; i++) {
        float rad = (cardinals[i].angle - 90) * (float)M_PI / 180.0f;
        float cos_r = cosf(rad), sin_r = sinf(rad);
        int lx = cx + (int)(label_offset * cos_r + 0.5f);
        int ly = cy + (int)(label_offset * sin_r + 0.5f);
        /* Center the glyph on the PERPENDICULAR axis only (fabsf(sin_r)
         * picks the x-shift for the vertical F/B labels, fabsf(cos_r)
         * picks the y-shift for the horizontal L/R labels) -- never
         * shift along the label's own radial direction, or the
         * shift's component along that direction changes how far the
         * label actually sits from center. A uniform diagonal
         * (-fsz/2,-fsz/2) shift (the previous version of this fix) did
         * exactly that: it pushed F/L outward and pulled R/B inward by
         * fsz/2 each, a real, reproducible asymmetry 1697-20747 caught by
         * eye, not just a fixed-offset-too-small bug. */
        int tx = lx - (int)(fabsf(sin_r) * (cardinal_fsz / 2.0f) + 0.5f);
        int ty = ly - (int)(fabsf(cos_r) * (cardinal_fsz / 2.0f) + 0.5f);
        draw_text(img, cardinals[i].label, tx, ty, grid_color, cardinal_fsz);
    }

    float scale = radius / 8.0f; /* MAX_MS2 = 8.0 */
    float ddx = accel_x_ms2 * scale, ddy = -accel_y_ms2 * scale;
    float dist = sqrtf(ddx * ddx + ddy * ddy);
    if (dist > radius && dist > 0.0f) {
        ddx = ddx / dist * radius;
        ddy = ddy / dist * radius;
    }
    int dot_x = cx + (int)(ddx + 0.5f), dot_y = cy + (int)(ddy + 0.5f);
    draw_circle(img, dot_x, dot_y, 3, dot_color, 2);
}

static void iso_project(float x_m, float y_m, float px_per_m, float* out_x, float* out_y) {
    *out_x = (x_m - y_m) * px_per_m;
    *out_y = (x_m + y_m) * px_per_m * 0.5f;
}

void apex_hud_draw_radar_map_stub(image_buffer_t* img, HudBounds b) {
    draw_rectangle(img, b.x1, b.y1, b.x2 - b.x1, b.y2 - b.y1, APEX_HUD_NO_DATA_COLOR, 1);

    int cx = (b.x1 + b.x2) / 2, cy = (b.y1 + b.y2) / 2;
    int w = b.x2 - b.x1, h = b.y2 - b.y1;
    int margin = 6;
    float half_extent = imin(w, h) / 2.0f - margin;
    static const float RANGE_RINGS_M[3] = {1.0f, 2.0f, 3.0f};
    float px_per_m = half_extent / RANGE_RINGS_M[2];
    if (px_per_m < 1.0f) px_per_m = 1.0f;

    for (int r = 0; r < 3; r++) {
        float ring_m = RANGE_RINGS_M[r];
        float pts_x[4], pts_y[4];
        float coords[4][2] = {{ring_m, 0.0f}, {0.0f, -ring_m}, {-ring_m, 0.0f}, {0.0f, ring_m}};
        for (int i = 0; i < 4; i++) {
            float ix, iy;
            iso_project(coords[i][0], coords[i][1], px_per_m, &ix, &iy);
            pts_x[i] = cx + ix;
            pts_y[i] = cy + iy;
        }
        for (int i = 0; i < 4; i++) {
            int j = (i + 1) % 4;
            draw_line(img, (int)pts_x[i], (int)pts_y[i], (int)pts_x[j], (int)pts_y[j],
                      APEX_HUD_NO_DATA_COLOR, 1);
        }
    }
    /* No radar_objects plotted -- this board has no real x_m/y_m/z_m
     * per-object distance yet, see header comment. */
}

/* Horizon ladder, live from the MPU-6050's gravity vector when has_data
 * (2026-10-11): ticks shift with pitch (pitch up => ladder moves down, the
 * +10 line reaches the aircraft symbol at +10 deg) and the whole ladder
 * rotates opposite to roll, like a real attitude indicator. Without data:
 * pitch=0/roll=0 in NO_DATA_COLOR -- horizon.py's own "no imu_reading"
 * default. The aircraft symbol itself never moves. */
static void rotate_about(int cx, int cy, float dx, float dy, float c, float s, int* ox, int* oy) {
    *ox = cx + (int)lroundf(dx * c - dy * s);
    *oy = cy + (int)lroundf(dx * s + dy * c);
}

void apex_hud_draw_horizon(image_buffer_t* img, HudBounds b, int has_data, float pitch_deg, float roll_deg) {
    if (!has_data) { pitch_deg = 0; roll_deg = 0; }
    unsigned int color = has_data ? COLOR_GREEN : APEX_HUD_NO_DATA_COLOR;
    float scale = (0.4f / 0.55f); /* _SCALE_RELATIVE_TO_BOUNDS, bounds.scale=1.0 */
    int bw = b.x2 - b.x1, bh = b.y2 - b.y1;
    int center_x = (b.x1 + b.x2) / 2, center_y = (b.y1 + b.y2) / 2;
    float px_per_deg = (bh / 90.0f) * scale;
    float a = -roll_deg * (float)M_PI / 180.0f;
    float ca = cosf(a), sa = sinf(a);

    static const float TICKS_DEG[4] = {10.0f, 20.0f, -10.0f, -20.0f};
    static const float HALF_WIDTH_FRAC[4] = {0.12f, 0.20f, 0.12f, 0.20f};
    int fsz = apex_hud_font_size(img, 9);
    for (int i = 0; i < 4; i++) {
        float tick_dy = -(TICKS_DEG[i] - pitch_deg) * px_per_deg;
        if (fabsf(tick_dy) > bh / 2.0f) continue; /* scrolled out of the cluster */
        float half_w = bw * HALF_WIDTH_FRAC[i] * scale;
        int x1, y1, x2, y2, lx, ly;
        rotate_about(center_x, center_y, -half_w, tick_dy, ca, sa, &x1, &y1);
        rotate_about(center_x, center_y, half_w, tick_dy, ca, sa, &x2, &y2);
        rotate_about(center_x, center_y, half_w + 3, tick_dy + 3, ca, sa, &lx, &ly);
        draw_line(img, x1, y1, x2, y2, color, 1);
        char label[8];
        snprintf(label, sizeof(label), "%+.0f", TICKS_DEG[i]);
        draw_text(img, label, lx, ly, color, fsz);
    }

    /* Fixed aircraft-symbol reference -- never rotates/shifts. */
    int wing = (int)(bw * 0.06f * scale);
    draw_line(img, center_x - wing, center_y, center_x - wing / 3, center_y, color, 1);
    draw_line(img, center_x + wing / 3, center_y, center_x + wing, center_y, color, 1);
    draw_line(img, center_x - wing / 3, center_y, center_x, center_y + wing / 4, color, 1);
    draw_line(img, center_x, center_y + wing / 4, center_x + wing / 3, center_y, color, 1);
    draw_circle(img, center_x, center_y, 2, color, 2);
}

void apex_hud_draw_horizon_stub(image_buffer_t* img, HudBounds b) {
    apex_hud_draw_horizon(img, b, 0, 0, 0);
}

/* Ticks always draw in their own fixed light grey (matching roll_ring.py
 * exactly -- that color isn't data-conditional in the original either).
 * The pointer sits at the live roll angle in TRACK_COLOR when has_data,
 * else straight up in NO_DATA_COLOR. RolloverMarkerWidget is deliberately
 * not ported -- see header comment. */
void apex_hud_draw_roll_ring(image_buffer_t* img, HudBounds b, int has_data, float roll_deg) {
    int center_x = (b.x1 + b.x2) / 2, center_y = (b.y1 + b.y2) / 2;
    int bh = b.y2 - b.y1;
    int arc_radius = imax((int)(bh * (0.22f / 0.55f)), 8 + 6);

    static const int TICKS_DEG[11] = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};
    int label_fsz = apex_hud_font_size(img, 8);
    for (int i = 0; i < 11; i++) {
        int tick_deg = TICKS_DEG[i];
        float tick_rad = (tick_deg - 90) * (float)M_PI / 180.0f;
        int is_major = (((tick_deg % 30) + 30) % 30) == 0;
        int tick_len = is_major ? 6 : 3;
        int ox = center_x + (int)(arc_radius * cosf(tick_rad));
        int oy = center_y + (int)(arc_radius * sinf(tick_rad));
        int ix = center_x + (int)((arc_radius - tick_len) * cosf(tick_rad));
        int iy = center_y + (int)((arc_radius - tick_len) * sinf(tick_rad));
        draw_line(img, ox, oy, ix, iy, APEX_HUD_ROLL_ARC_COLOR, 1);
        if (is_major) {
            char label[12];
            snprintf(label, sizeof(label), "%d", abs(tick_deg));
            int lx = center_x + (int)((arc_radius + 9) * cosf(tick_rad));
            int ly = center_y + (int)((arc_radius + 9) * sinf(tick_rad));
            draw_text(img, label, lx - 3, ly - 4, APEX_HUD_ROLL_ARC_COLOR, label_fsz);
        }
    }

    /* Pointer: a small triangle just inside the arc at the roll angle,
     * clamped to the ring's +-60 deg scale. */
    if (!has_data) roll_deg = 0;
    if (roll_deg > 60) roll_deg = 60;
    if (roll_deg < -60) roll_deg = -60;
    unsigned int color = has_data ? APEX_HUD_TRACK_COLOR : APEX_HUD_NO_DATA_COLOR;
    float r = (roll_deg - 90) * (float)M_PI / 180.0f;
    float ux = cosf(r), uy = sinf(r);  /* outward unit vector */
    float px = -uy, py = ux;           /* perpendicular */
    float tip_r = arc_radius - 8, base_r = arc_radius;
    int tip_x = center_x + (int)lroundf(ux * tip_r), tip_y = center_y + (int)lroundf(uy * tip_r);
    int b1x = center_x + (int)lroundf(ux * base_r + px * 4), b1y = center_y + (int)lroundf(uy * base_r + py * 4);
    int b2x = center_x + (int)lroundf(ux * base_r - px * 4), b2y = center_y + (int)lroundf(uy * base_r - py * 4);
    draw_line(img, tip_x, tip_y, b1x, b1y, color, 1);
    draw_line(img, b1x, b1y, b2x, b2y, color, 1);
    draw_line(img, b2x, b2y, tip_x, tip_y, color, 1);
}

void apex_hud_draw_roll_ring_stub(image_buffer_t* img, HudBounds b) {
    apex_hud_draw_roll_ring(img, b, 0, 0);
}

/* Heading=0 ("unknown, default to North"), NO_DATA_COLOR throughout,
 * "---" readout -- compass_tape.py's own real no-data default. */
void apex_hud_draw_compass_tape_stub(image_buffer_t* img, HudBounds b, int position_top) {
    int center_x = (b.x1 + b.x2) / 2;
    int sign = position_top ? 1 : -1;
    int bh = b.y2 - b.y1, bw = b.x2 - b.x1;
    int tape_y = b.y1 + (int)(bh * (position_top ? 0.08f : 0.92f));
    int tape_half_width = (int)(bw * 0.30f);
    const float deg_per_px = 0.5f;

    int box_y1 = imin(tape_y - 12 * sign, tape_y + 6 * sign);
    int box_y2 = imax(tape_y - 12 * sign, tape_y + 6 * sign);
    draw_rectangle(img, center_x - tape_half_width, box_y1, tape_half_width * 2, box_y2 - box_y1,
                    APEX_HUD_NO_DATA_COLOR, 1);

    int visible_span_deg = (int)(tape_half_width * deg_per_px) + 1;
    int tick_fsz = apex_hud_font_size(img, 8);
    for (int degree = -visible_span_deg; degree < visible_span_deg; degree++) {
        int pixel_offset = (int)roundf(degree / deg_per_px);
        if (abs(pixel_offset) > tape_half_width) continue;
        int screen_x = center_x + pixel_offset;
        int wrapped = ((degree % 360) + 360) % 360;
        if (wrapped % 30 == 0) {
            draw_line(img, screen_x, tape_y - 10 * sign, screen_x, tape_y + 4 * sign,
                      APEX_HUD_NO_DATA_COLOR, 1);
            draw_text(img, "-", screen_x - 4, tape_y - 12 * sign, APEX_HUD_NO_DATA_COLOR, tick_fsz);
        } else if (wrapped % 10 == 0) {
            draw_line(img, screen_x, tape_y - 6 * sign, screen_x, tape_y + 4 * sign,
                      APEX_HUD_NO_DATA_COLOR, 1);
        }
    }

    draw_line(img, center_x, tape_y - 12 * sign, center_x, tape_y + 6 * sign, APEX_HUD_NO_DATA_COLOR, 1);
    draw_text(img, "---", center_x - 12, tape_y + 20 * sign, APEX_HUD_NO_DATA_COLOR,
              apex_hud_font_size(img, 12));
}

/* Boxed pitch/roll readouts: live whole degrees ("P+5", "R-12") in
 * TRACK_COLOR when has_data, else "P--"/"R--" -- attitude_readouts.py's
 * own no-data default. Box/font sizes scale with resolution (the Python
 * original uses fixed pixel sizes since it only ever runs at one
 * resolution; this pipeline runs at several); boxes widen to fit the text. */
void apex_hud_draw_attitude_readouts(image_buffer_t* img, HudBounds b, int has_data, float pitch_deg, float roll_deg) {
    int center_y = (b.y1 + b.y2) / 2;
    float scale = img->height / APEX_HUD_REF_HEIGHT;
    int fsz = apex_hud_font_size(img, 12);
    int box_h = imax((int)(24 * scale), 16);
    int margin = 2;

    char labels[2][12];
    if (has_data) {
        snprintf(labels[0], sizeof(labels[0]), "P%+.0f", pitch_deg);
        snprintf(labels[1], sizeof(labels[1]), "R%+.0f", roll_deg);
    } else {
        snprintf(labels[0], sizeof(labels[0]), "P--");
        snprintf(labels[1], sizeof(labels[1]), "R--");
    }
    unsigned int text_color = has_data ? APEX_HUD_TRACK_COLOR : APEX_HUD_NO_DATA_COLOR;
    for (int i = 0; i < 2; i++) {
        /* glyphs measured ~1 fsz wide in this bitmap font ("P-33" ~ 48px at fsz 12) */
        int box_w = imax(imax((int)(28 * scale), 20), (int)strlen(labels[i]) * fsz + 6);
        int x = (i == 0) ? b.x1 + margin : b.x2 - margin - box_w;
        int top_y = center_y - box_h / 2;
        draw_rectangle(img, x, top_y, box_w, box_h, APEX_HUD_BOX_COLOR, 1);
        draw_text(img, labels[i], x + 2, center_y + 5, text_color, fsz);
    }
}

void apex_hud_draw_attitude_readouts_stub(image_buffer_t* img, HudBounds b) {
    apex_hud_draw_attitude_readouts(img, b, 0, 0, 0);
}
