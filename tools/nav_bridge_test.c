/*
  nav_bridge_test.c — host test for the phone -> HudData bridge.

  Build from the repository root:
    gcc -O2 -std=c11 -Wall -Wextra -Werror -o /tmp/nav_bridge_test \
        tools/nav_bridge_test.c waveshare_hud_mockup/nav_bridge.c \
        waveshare_hud_mockup/nav_proto.c -Iwaveshare_hud_mockup -lm
    /tmp/nav_bridge_test
*/
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nav_bridge.h"
#include "nav_proto.h"
#include "nav_proto_vectors.h"

static nav_state_t g_state;
static bool g_connected;
static uint32_t g_now;

/* Stub for the real GATT server copy. The test controls the decoded state. */
bool nav_server_copy(nav_state_t *out)
{
    if (!g_connected) return false;
    *out = g_state;
    nav_proto_tick(out, g_now);
    return true;
}

static int failures;

#define CHECK(condition, message)                                      \
    do {                                                               \
        if (!(condition)) {                                            \
            printf("FAIL: %s\n", (message));                           \
            failures++;                                                \
        }                                                              \
    } while (0)

static const nav_golden_vector *golden(const char *id)
{
    for (size_t i = 0; i < NAV_GOLDEN_COUNT; i++)
        if (strcmp(nav_golden[i].id, id) == 0) return &nav_golden[i];
    return NULL;
}

static void reset_state(uint32_t now)
{
    nav_proto_init(&g_state);
    g_connected = true;
    g_now = now;
}

static void feed(const char *id, uint32_t now)
{
    const nav_golden_vector *vector = golden(id);
    if (vector == NULL) {
        printf("FAIL: missing vector %s\n", id);
        failures++;
        return;
    }
    g_now = now;
    if (!nav_proto_on_write(&g_state, vector->data, vector->len, now)) {
        printf("FAIL: vector %s rejected\n", id);
        failures++;
    }
}

static void feed_position(double lat, double lon, float speed, float bearing, float accuracy,
                          int64_t timestamp, uint32_t now)
{
    uint8_t value[36];
    memcpy(value, &lat, 8);
    memcpy(value + 8, &lon, 8);
    memcpy(value + 16, &speed, 4);
    memcpy(value + 20, &bearing, 4);
    memcpy(value + 24, &accuracy, 4);
    memcpy(value + 28, &timestamp, 8);

    uint8_t message[48];
    size_t n = 0;
    message[n++] = 1;
    message[n++] = 2;
    message[n++] = 0; message[n++] = 0; message[n++] = 0; message[n++] = 0; /* sequence */
    message[n++] = 0; message[n++] = 0;                                     /* reserved */
    message[n++] = 0x0A; message[n++] = 36;
    memcpy(message + n, value, 36);
    n += 36;
    uint16_t crc = nav_proto_crc16(message, n);
    message[n++] = (uint8_t)crc;
    message[n++] = (uint8_t)(crc >> 8);
    if (!nav_proto_on_write(&g_state, message, n, now)) {
        printf("FAIL: position rejected\n");
        failures++;
    }
    g_now = now;
}

static HudData fresh_data(void)
{
    HudData data;
    memset(&data, 0, sizeof data);
    return data;
}

int main(void)
{
    /* Route: maneuver, limit, remaining. */
    {
        reset_state(1000);
        feed("route_right_300m", 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);

        CHECK(d.valid & V_MAN, "route: V_MAN");
        CHECK(d.man_main == 0x0D && d.man_dir == 0xC0, "route: right turn as BAP turn");
        CHECK(d.man_dist_x10 == 3000 && d.man_unit == 0, "route: distance in meters x10");
        CHECK(d.bargraph == 0xFF, "route: progress bar hidden");
        CHECK(d.rg_active == 1, "route: active");
        CHECK(d.valid & V_DEST, "route: V_DEST");
        CHECK(d.dest_dist_x10 == 125 && d.dest_unit == 1, "route: remaining distance in km x10");
        CHECK(d.tte_type == 0 && d.tte_h == 1 && d.tte_min == 1, "route: remaining time");
        CHECK(d.valid & V_SIGN, "route: V_SIGN");
        CHECK(d.sign_raw == 12, "route: 60 km/h as VZE code 12");
        uint16_t kph = 0;
        CHECK(nav_bridge_speed_limit(&kph) && kph == 60, "route: phone limit hook");
    }

    /* Roundabout: neutral sector, distance and remaining. */
    {
        reset_state(1000);
        feed("roundabout", 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);

        CHECK(d.valid & V_MAN, "roundabout: V_MAN");
        CHECK(d.man_main == 0x15 && d.man_dir == 0x80, "roundabout: neutral BAP sector");
        CHECK(d.man_dist_x10 == 3500 && d.man_unit == 0, "roundabout: distance in meters x10");
        CHECK(d.valid & V_DEST && d.dest_dist_x10 == 40 && d.dest_unit == 1,
              "roundabout: remaining distance in km x10");
        CHECK(d.tte_h == 0 && d.tte_min == 4, "roundabout: remaining time");
    }

    /* Camera without maneuver: limit and remaining still apply, no arrow. */
    {
        reset_state(1000);
        feed("camera_1000m", 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);

        CHECK(!(d.valid & V_MAN), "camera: no arrow");
        CHECK(d.valid & V_DEST, "camera: remaining preserved");
        CHECK(d.valid & V_SIGN && d.sign_raw == 12, "camera: limit preserved");
    }

    /* Free mode: only limit (and position when present). */
    {
        reset_state(1000);
        feed("free", 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);

        CHECK(!(d.valid & V_MAN) && !(d.valid & V_DEST), "free: no maneuver/remaining");
        CHECK(d.valid & V_SIGN && d.sign_raw == 12, "free: speed limit");
    }

    /* Camera cache for the menu source (G6 is shown only when the menu says so). */
    {
        reset_state(1000);
        feed("camera_1000m", 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);
        float dist = 0;
        CHECK(nav_bridge_camera(&dist) && dist == 1000.0f, "camera: phone G6 cached");

        reset_state(1000);
        feed("route_right_300m", 1000);
        d = fresh_data();
        nav_bridge_apply(&d, 1000);
        CHECK(!nav_bridge_camera(&dist), "camera: absent when G6 is absent");
    }

    /* Position: GPS fields and speed fallback. */
    {
        reset_state(1000);
        feed("position", 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);

        CHECK(d.valid & V_GPS, "position: V_GPS");
        CHECK(d.gps_fix == 3, "position: fix");
        CHECK(d.gps_sat == 8 && d.gps_hdop_x10 == 10, "position: camera gate values");
        CHECK(llabs((long long)d.gps_lat_e6 - 55755800LL) <= 1, "position: latitude e6");
        CHECK(llabs((long long)d.gps_lon_e6 - 37617300LL) <= 1, "position: longitude e6");
        CHECK(d.gps_hdg_x10 == 450, "position: heading x10");
        CHECK(!(d.valid & V_SPEED) && d.speed_kmh == 0, "position: phone speed ignored (CAN only)");
    }

    /* Position with NaN speed/bearing: heading marked unknown, no speed fallback. */
    {
        reset_state(1000);
        feed("position_nan", 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);

        CHECK(d.valid & V_GPS, "position_nan: V_GPS");
        CHECK(!(d.valid & V_HDG), "position_nan: no phone heading");
        CHECK(!(d.valid & V_SPEED), "position_nan: no speed fallback");
    }

    /* A NaN phone bearing must not clobber a valid car heading. */
    {
        reset_state(1000);
        feed("position_nan", 1000);
        HudData d = fresh_data();
        d.gps_hdg_x10 = 1234;
        d.valid |= V_HDG;
        nav_bridge_apply(&d, 1000);

        CHECK((d.valid & V_HDG) && d.gps_hdg_x10 == 1234, "position_nan: car heading preserved");
    }

    /* Negative coordinates, zero speed, valid bearing. */
    {
        reset_state(1000);
        feed_position(-33.8688, 151.2093, 0.0f, 180.0f, 1.5f, 0, 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);

        CHECK(d.valid & V_GPS, "coordinates: V_GPS");
        CHECK(llabs((long long)d.gps_lat_e6 + 33868800LL) <= 1, "coordinates: negative latitude e6");
        CHECK(llabs((long long)d.gps_lon_e6 - 151209300LL) <= 1, "coordinates: longitude e6");
        CHECK(d.gps_hdg_x10 == 1800, "coordinates: bearing x10");
        CHECK(!(d.valid & V_SPEED) && d.speed_kmh == 0, "coordinates: zero phone speed is not a source");
    }

    /* The phone position overrides the car GNSS. */
    {
        reset_state(1000);
        feed_position(55.75, 37.62, 12.5f, 45.0f, 3.5f, 0, 1000);
        HudData d = fresh_data();
        d.gps_lat_e6 = 1;
        d.gps_lon_e6 = 2;
        d.valid |= V_GPS | V_HDG;
        nav_bridge_apply(&d, 1000);

        CHECK(d.gps_lat_e6 == 55750000 && d.gps_lon_e6 == 37620000,
              "coordinates: phone position overrides car GNSS");
    }

    /* Phone speed is never applied: without CAN there is no speed at all, and
       with CAN the car value is untouched. */
    {
        reset_state(1000);
        feed_position(55.75, 37.62, 12.5f, 45.0f, 3.5f, 0, 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);

        CHECK(!(d.valid & V_SPEED), "coordinates: no speed without CAN");

        d = fresh_data();
        d.speed_kmh = 80;
        d.valid |= V_SPEED;
        nav_bridge_apply(&d, 1000);

        CHECK((d.valid & V_SPEED) && d.speed_kmh == 80, "coordinates: CAN speed is kept");
    }

    /* Position TTL: 5 s by BLE receive time. */
    {
        reset_state(1000);
        feed_position(55.75, 37.62, 12.5f, 45.0f, 3.5f, 0, 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 6001);

        CHECK(!(d.valid & V_GPS), "coordinates: stale position is not shown after 5 s");
    }

    /* Idle: no phone overlay (CAN/BAP stays visible). */
    {
        reset_state(1000);
        feed("idle", 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);

        CHECK(!(d.valid & (V_MAN | V_DEST | V_SIGN)), "idle: no phone groups");
        CHECK(!nav_bridge_speed_limit(NULL), "idle: no phone limit hook");
    }

    /* Disconnect: no overlay. */
    {
        reset_state(1000);
        feed("route_right_300m", 1000);
        g_connected = false;
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);

        CHECK(!(d.valid & V_MAN), "disconnect: no overlay");
    }

    /* Stale state (3 s) hides the snapshot but a fresh position still applies. */
    {
        reset_state(1000);
        feed("route_right_300m", 1000);
        feed("position", 1000);
        HudData d = fresh_data();
        nav_bridge_apply(&d, 4001);

        CHECK(!(d.valid & V_MAN), "stale: no maneuver overlay");
        CHECK(d.valid & V_GPS, "stale: independent position still applies");
    }

    /* Phone limit above the VZE linear range (160 km/h) still reaches the hook. */
    {
        reset_state(1000);
        feed("route_right_300m", 1000);
        /* Replace G5 60 with 160 in the decoded state. */
        g_state.snapshot.speed_limit.kph = 160;
        HudData d = fresh_data();
        nav_bridge_apply(&d, 1000);
        uint16_t kph = 0;
        CHECK(nav_bridge_speed_limit(&kph) && kph == 160, "limit: 160 km/h priority");
    }

    if (failures == 0) {
        printf("PASS: nav_bridge host test\n");
        return 0;
    }
    printf("FAIL: %d check(s)\n", failures);
    return 1;
}
