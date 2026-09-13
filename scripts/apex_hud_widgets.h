#ifndef APEX_HUD_WIDGETS_H_
#define APEX_HUD_WIDGETS_H_

/* Port of apex_ros's apex_hud package (src/apex_hud/apex_hud/widgets/*.py)
 * to plain C for the Luckfox's bare-metal pipeline -- same colors, same
 * widget geometry/math, no OpenCV/Python/ROS dependency (none of that
 * runs here; see PROJECT_MEMORY.md's 2026-09-13 "apex_hud port" entry for
 * why, and which of apex_hud's 14 widgets have a real Luckfox data source
 * vs are intentionally not ported).
 *
 * Colors match apex_hud/style.py exactly (RGB, this pipeline's own frame
 * order): INSTRUMENT_COLOR (0,255,0) green, TRACK_COLOR (0,200,255) cyan,
 * OBSTACLE_COLOR (255,0,0) red, NO_DATA_COLOR (128,128,128) grey.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "utils/image_drawing.h"

#define APEX_HUD_TRACK_COLOR   0xFF00C8FF  /* ARGB, RGB(0,200,255) */
#define APEX_HUD_NO_DATA_COLOR 0xFF808080  /* ARGB, RGB(128,128,128) */

typedef struct { int x1, y1, x2, y2; } HudBounds;

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

#ifdef __cplusplus
}
#endif

#endif /* APEX_HUD_WIDGETS_H_ */
