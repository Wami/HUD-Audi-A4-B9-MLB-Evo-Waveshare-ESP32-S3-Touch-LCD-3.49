/*
  nav_bridge.h — overlay the phone navigation groups onto HudData.

  The bridge uses the same fields as BAP/VZE so the renderer keeps one code path:
  G1 -> man_main/man_dir/man_dist, G5 -> sign/limit, G8 -> dest/tte,
  G9 -> gps_*. G6 is cached here and used by the camera block only when the
  menu source is phone or both; the local camera database remains the default.
*/
#ifndef NAV_BRIDGE_H
#define NAV_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#include "hud_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Overlay fresh phone groups onto a HudData copy. Called from the LVGL timer
   after hud_data_snapshot(), so the phone wins over CAN/BAP. */
void nav_bridge_apply(HudData *data, uint32_t now_ms);

/* True while a fresh phone G5 speed limit is active. The sign block calls this
   before BAP/VZE/PSD so the phone limit has priority. */
bool nav_bridge_speed_limit(uint16_t *kph);

/* True while a fresh phone G6 camera is present; the camera block uses it when
   the menu source is phone or both. */
bool nav_bridge_camera(float *distance_m);

#ifdef __cplusplus
}
#endif
#endif
