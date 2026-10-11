#ifndef APEX_HUD_WIDGETS_H_
#define APEX_HUD_WIDGETS_H_

/* Port of apex_ros's apex_hud package (src/apex_hud/apex_hud/widgets/*.py)
 * to plain C for the Luckfox's bare-metal pipeline -- same colors, same
 * widget geometry/math, no OpenCV/Python/ROS dependency (none of that
 * runs here; see PROJECT_MEMORY.md's 2026-09-13 "apex_hud port" entries
 * for why, and which of apex_hud's 14 widgets have a real Luckfox data
 * source vs draw a neutral/grey "no data" default vs are genuinely not
 * ported at all).
 *
 * Colors match apex_hud/style.py exactly (RGB, this pipeline's own frame
 * order): INSTRUMENT_COLOR (0,255,0) green, TRACK_COLOR (0,200,255) cyan,
 * OBSTACLE_COLOR (255,0,0) red, NO_DATA_COLOR (128,128,128) grey.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "utils/image_drawing.h"

#define APEX_HUD_TRACK_COLOR    0xFF00C8FF  /* ARGB, RGB(0,200,255) */
#define APEX_HUD_NO_DATA_COLOR  0xFF808080  /* ARGB, RGB(128,128,128) */
#define APEX_HUD_ROLL_ARC_COLOR 0xFFDCDCDC  /* ARGB, RGB(220,220,220) -- roll_ring.py's ticks are this fixed light grey even with live data */
#define APEX_HUD_BOX_COLOR      0xFFA0A0A0  /* ARGB, RGB(160,160,160) -- attitude_readouts.py's box border */

typedef struct { int x1, y1, x2, y2; } HudBounds;

static inline HudBounds apex_hud_frame_bounds(const image_buffer_t* img) {
    HudBounds b = {0, 0, img->width, img->height};
    return b;
}

/* center_cluster_bounds(w_frac=0.55, h_frac=0.55) port -- layout.py. */
static inline HudBounds apex_hud_cluster_bounds(const image_buffer_t* img) {
    int cx = img->width / 2, cy = img->height / 2;
    int half_w = (int)(img->width * 0.55f) / 2;
    int half_h = (int)(img->height * 0.55f) / 2;
    HudBounds b = {cx - half_w, cy - half_h, cx + half_w, cy + half_h};
    return b;
}

/* Font size scaled to the current display resolution, base_size_at_540p
 * chosen against this pipeline's 960x540 default -- keeps text legible
 * at both smaller (640x480) and larger (1920x1080) resolutions instead
 * of a fixed pixel size tuned for only one of them. */
int apex_hud_font_size(const image_buffer_t* img, int base_size_at_540p);

/* CrosshairWidget port (widgets/crosshair.py) -- static center-of-view
 * reticle, no live data needed. side = frame_height * 0.20, centered on
 * the frame's true center. */
void apex_hud_draw_crosshair(image_buffer_t* img);

/* TrackBoxesWidget port (widgets/track_boxes.py), reduced to what this
 * board actually has: pixel bbox + class + confidence. No azimuth/
 * distance (that needs apex_core.camera_lens's lens model / #32's
 * mono-distance work, not done on the Luckfox yet) -- omitted rather
 * than fabricated, not drawn as a fake "no data" placeholder either,
 * matching track_boxes.py's own "exempt from always-visible" rule (zero
 * detections is a legitimate state, not "sensor missing").
 * det_x/y/w/h are in MODEL_SIZE (640x640) space; scale_x/scale_y map to
 * the display frame, same convention already used by the caller. */
typedef struct {
    float x, y, w, h, score;
    const char* class_name;
} HudTrack;
void apex_hud_draw_track_boxes(image_buffer_t* img, const HudTrack* tracks, int n_tracks,
                                float scale_x, float scale_y);

/* AccelerometerWidget port (widgets/accelerometer.py) -- flat 2-axis
 * polar-grid dot (lateral/forward acceleration only, MAX_MS2=8.0 full
 * scale), F/R/B/L labels, always drawn (grey when no real reading yet).
 * accel_x_ms2/accel_y_ms2 must already be in m/s^2 (this board's own
 * accel_update() reads g's -- multiply by 9.81 before calling). */
void apex_hud_draw_accelerometer(image_buffer_t* img, HudBounds bounds, int has_data,
                                  float accel_x_ms2, float accel_y_ms2);

/* RadarMapWidget port (widgets/radar_map.py) -- ground-plane grid with
 * range rings at 1/2/3m. Drawn as a real, honest stub: the bordered box
 * and grid always draw (matching the widget's own "always visible"
 * contract), but always in NO_DATA_COLOR with zero objects plotted --
 * this board has no real x_m/y_m/z_m per-object distance yet (ACTION_ITEMS
 * #32 in apex_ros, not done). Wire real objects in once that lands
 * instead of re-deriving this widget. */
void apex_hud_draw_radar_map_stub(image_buffer_t* img, HudBounds bounds);

/* ArtificialHorizonWidget port (widgets/horizon.py) -- pitch ladder +
 * fixed aircraft-symbol reference. This board has no orientation source
 * (MPU-6050 raw accel only, no fusion/magnetometer), so this always draws
 * horizon.py's own documented "no imu_reading" neutral default: pitch=0,
 * roll=0 ("level"), in NO_DATA_COLOR -- a real, meaningful default per
 * that file's own docstring, not a fabricated reading. */
void apex_hud_draw_horizon_stub(image_buffer_t* img, HudBounds cluster_bounds);
/* Live version (2026-10-11): pitch/roll in degrees from the accelerometer's
 * gravity vector; has_data=0 draws exactly the stub above. */
void apex_hud_draw_horizon(image_buffer_t* img, HudBounds cluster_bounds, int has_data,
                           float pitch_deg, float roll_deg);

/* RollRingWidget port (widgets/roll_ring.py) -- tick-marked arc (ticks
 * always drawn in their own fixed light-grey regardless of data, same as
 * the Python original) + a pointer at roll=0 in NO_DATA_COLOR (no
 * orientation source, see apex_hud_draw_horizon_stub). RolloverMarkerWidget
 * is NOT ported: it has no meaningful neutral default (per its own
 * docstring) and this board has no rollover_angle_deg source at all --
 * faithfully matches its real "no data -> not drawn" contract by simply
 * not being called. */
void apex_hud_draw_roll_ring_stub(image_buffer_t* img, HudBounds cluster_bounds);
void apex_hud_draw_roll_ring(image_buffer_t* img, HudBounds cluster_bounds, int has_data, float roll_deg);

/* CompassTapeWidget port (widgets/compass_tape.py) -- scrolling heading
 * tape, always drawn centered on heading=0 ("unknown, default to North")
 * in NO_DATA_COLOR with a "---" numeric readout, since this board has no
 * magnetometer/heading source. position_top: 0 = bottom (this pipeline's
 * only use so far), 1 = top. */
void apex_hud_draw_compass_tape_stub(image_buffer_t* img, HudBounds frame_bounds, int position_top);

/* AttitudeReadoutWidget port (widgets/attitude_readouts.py) -- boxed
 * "P--"/"R--" numeric placeholders at the frame's left/right edges,
 * vertically centered, in NO_DATA_COLOR (no orientation source). */
void apex_hud_draw_attitude_readouts_stub(image_buffer_t* img, HudBounds frame_bounds);
void apex_hud_draw_attitude_readouts(image_buffer_t* img, HudBounds frame_bounds, int has_data,
                                     float pitch_deg, float roll_deg);

#ifdef __cplusplus
}
#endif

#endif /* APEX_HUD_WIDGETS_H_ */
