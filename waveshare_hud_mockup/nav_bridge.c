/*
  nav_bridge.c — map phone navigation groups to the existing BAP/VZE fields.

  This is an additive overlay: hud_mockup.c calls nav_bridge_apply() on a local
  HudData copy after hud_data_snapshot(). The struct and the CAN decoder are not
  changed. Phone data wins over BAP/VZE/PSD while it is fresh; the camera group
  is intentionally left to the local camera subsystem (menu option).

  Distances are shown as the phone sends them, in Navigator-like steps. There is
  no interpolation between snapshots; the fast sender cycle (250 ms) makes the
  steps appear promptly without inventing intermediate values.
*/
#include "nav_bridge.h"

#include <math.h>

#include "ble_nav_server.h"
#include "nav_proto.h"

static uint16_t s_phone_kph;
static float s_phone_cam_m;
static bool s_phone_cam_on;

/* BAP distance units: meters below 1 km, kilometres (x10) above. */
static void set_distance(uint32_t meters, uint32_t *x10, uint8_t *unit)
{
    if (meters >= 1000u) {
        *x10 = (meters + 50u) / 100u;
        *unit = 1;
    } else {
        *x10 = meters * 10u;
        *unit = 0;
    }
}

static void map_maneuver(HudData *data, const nav_maneuver_t *maneuver)
{
    if (!maneuver->present) return;

    uint8_t main = 0;
    uint8_t dir = 0;
    switch (maneuver->kind) {
    case NAV_MANEUVER_KIND_LEFT:         main = 0x0D; dir = 0x40; break; /* Turn left */
    case NAV_MANEUVER_KIND_RIGHT:        main = 0x0D; dir = 0xC0; break; /* Turn right */
    case NAV_MANEUVER_KIND_LEFT_WIDE:    main = 0x0D; dir = 0x30; break; /* keep left */
    case NAV_MANEUVER_KIND_RIGHT_WIDE:   main = 0x0D; dir = 0xB0; break; /* keep right */
    case NAV_MANEUVER_KIND_LEFT_SHARP:   main = 0x0D; dir = 0x50; break;
    case NAV_MANEUVER_KIND_RIGHT_SHARP:  main = 0x0D; dir = 0xD0; break;
    case NAV_MANEUVER_KIND_U_TURN_LEFT:  main = 0x19; dir = 0x40; break;
    case NAV_MANEUVER_KIND_U_TURN_RIGHT: main = 0x19; dir = 0xC0; break;
    case NAV_MANEUVER_KIND_STRAIGHT:     main = 0x0B; dir = 0x00; break; /* FollowStreet */
    case NAV_MANEUVER_KIND_FINISH:       main = 0x03; dir = 0x00; break; /* Arrived */
    case NAV_MANEUVER_KIND_REROUTING:    main = 0x09; dir = 0x00; break; /* "..." */
    /* No exit direction in v1: the phone uses a fixed upper-right exit
       (45°, sector 14) as the neutral roundabout symbol; the HUD keeps the
       exit number in the centre. */
    case NAV_MANEUVER_KIND_ROUNDABOUT:   main = 0x15; dir = 0xE0; break; /* upper-right */
    default: return;
    }

    data->rg_active = 1;
    data->man_main = main;
    data->man_dir = dir;
    data->man_z = 0;
    data->man2_main = 0;
    data->man2_dir = 0;
    set_distance(maneuver->distance_m, &data->man_dist_x10, &data->man_unit);
    data->bargraph = 0xFF;          /* the phone snapshot hides the progress bar */
    data->valid |= V_MAN | V_MDIST;
}

static void map_remaining(HudData *data, const nav_remaining_t *remaining)
{
    if (!remaining->present) return;

    uint32_t hours = remaining->time_s / 3600u;
    if (hours > 255u) hours = 255u;

    set_distance(remaining->distance_m, &data->dest_dist_x10, &data->dest_unit);
    data->tte_type = 0;             /* remaining time, not arrival time */
    data->tte_h = (uint8_t)hours;
    data->tte_min = (uint8_t)((remaining->time_s % 3600u) / 60u);
    data->valid |= V_DEST | V_TTE;
}

static void map_speed_limit(HudData *data, const nav_speed_limit_t *limit)
{
    if (!limit->present) return;

    s_phone_kph = limit->kph;
    data->sign_suppress = 0;
    data->sign_mph = 0;
    data->tsr_hint = 0;
    data->sign_warn = 0;
    /* The phone is the only sign source while G5 is fresh: do not mix stale
       BAP additional signs (no-overtake etc.) into the same row. */
    data->sign_raw2 = 0;
    data->sign_raw3 = 0;
    data->sign_sup2 = 0;
    data->sign_sup3 = 0;
    data->valid |= V_SIGN;

    /* Keep the VZE field coherent for code that reads it directly. The main
       sign block uses nav_bridge_speed_limit() so values above 150 km/h still
       work even though the VZE linear formula stops at 150. */
    if (limit->kph <= 150) {
        uint8_t code = (uint8_t)((limit->kph + 2u) / 5u);
        if (code >= 1 && code <= 30) data->sign_raw = code;
    }
}

static void map_position(HudData *data, const nav_position_t *position)
{
    data->gps_lat_e6 = (int32_t)llround(position->lat * 1e6);
    data->gps_lon_e6 = (int32_t)llround(position->lon * 1e6);
    data->gps_fix = 3;
    data->gps_sat = 8;          /* the phone position is trusted; keep the local camera gate open */
    data->gps_hdop_x10 = 10;
    data->valid |= V_GPS;
    /* A NaN phone bearing must not clobber a valid car course: keep V_HDG and
       the previous gps_hdg_x10, which the camera query can still use. */
    if (!isnan(position->bearing)) {
        data->gps_hdg_x10 = (uint16_t)lroundf(position->bearing * 10.0f);
        data->valid |= V_HDG;
    }
    /* The phone speed is informational: the HUD speed (and everything that
       depends on it, such as the camera gate) uses CAN only. */
}

void nav_bridge_apply(HudData *data, uint32_t now_ms)
{
    s_phone_kph = 0;
    s_phone_cam_on = false;
    if (data == NULL) return;

    nav_state_t state;
    if (!nav_server_copy(&state)) return;

    if (nav_proto_state_fresh(&state, now_ms) &&
        state.snapshot.session.present &&
        state.snapshot.session.mode != NAV_MODE_IDLE) {
        map_maneuver(data, &state.snapshot.maneuver);
        if (state.snapshot.maneuver.present &&
            state.snapshot.maneuver.kind == NAV_MANEUVER_KIND_ROUNDABOUT &&
            state.snapshot.roundabout_exit.present) {
            data->roundabout_exit = state.snapshot.roundabout_exit.exit_no;
            data->valid |= V_RABOUT_EXIT;
        }
        map_remaining(data, &state.snapshot.remaining);
        map_speed_limit(data, &state.snapshot.speed_limit);
        if (state.snapshot.camera.present) {
            s_phone_cam_m = (float)state.snapshot.camera.distance_m;
            s_phone_cam_on = true;
        }
    }

    /* Position is an independent stream with its own 5 s TTL. */
    if (nav_proto_position_fresh(&state, now_ms))
        map_position(data, &state.position);
}

bool nav_bridge_camera(float *distance_m)
{
    if (!s_phone_cam_on) return false;
    if (distance_m != NULL) *distance_m = s_phone_cam_m;
    return true;
}

bool nav_bridge_speed_limit(uint16_t *kph)
{
    if (s_phone_kph == 0) return false;
    if (kph != NULL) *kph = s_phone_kph;
    return true;
}
