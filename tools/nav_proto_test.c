/*
  nav_proto_test.c — host test for the pure-C navigation decoder.

  Build from the repository root:
    gcc -O2 -std=c11 -Wall -Wextra -o /tmp/nav_proto_test \
        tools/nav_proto_test.c waveshare_hud_mockup/nav_proto.c \
        -Iwaveshare_hud_mockup -lm
    /tmp/nav_proto_test
*/
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nav_proto.h"
#include "nav_proto_vectors.h"

static int failures;

#define CHECK(condition, message)                                      \
    do {                                                               \
        if (!(condition)) {                                            \
            printf("FAIL: %s\n", (message));                           \
            failures++;                                                \
        }                                                              \
    } while (0)

static const nav_golden_vector *find_vector(const nav_golden_vector *list, size_t count,
                                            const char *id)
{
    for (size_t i = 0; i < count; i++)
        if (strcmp(list[i].id, id) == 0) return &list[i];
    return NULL;
}

static const nav_golden_vector *golden(const char *id)
{
    return find_vector(nav_golden, NAV_GOLDEN_COUNT, id);
}

static const nav_golden_vector *negative(const char *id)
{
    return find_vector(nav_negative, NAV_NEGATIVE_COUNT, id);
}

static bool feed(nav_state_t *state, const nav_golden_vector *vector, uint32_t now_ms)
{
    return nav_proto_on_write(state, vector->data, vector->len, now_ms);
}

int main(void)
{
    CHECK(nav_proto_crc16((const uint8_t *)"123456789", 9) == 0x29B1,
          "CRC-16/CCITT-FALSE check vector");

    /* idle */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("idle"), 1000), "idle: accepted");
        CHECK(state.snapshot.session.present && state.snapshot.session.mode == NAV_MODE_IDLE,
              "idle: session mode");
        CHECK(!state.snapshot.maneuver.present, "idle: no maneuver");
        CHECK(state.snapshot.meta.present, "idle: meta present");
        CHECK(state.snapshot.meta.source == NAV_G10_SOURCE_NOTIFICATION, "idle: source notification");
    }

    /* route_right_300m */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("route_right_300m"), 1000), "route: accepted");
        CHECK(state.snapshot.session.mode == NAV_MODE_ROUTE, "route: session mode");
        CHECK(state.snapshot.maneuver.present && state.snapshot.maneuver.kind == NAV_MANEUVER_KIND_RIGHT &&
              state.snapshot.maneuver.distance_m == 300, "route: maneuver");
        CHECK(state.snapshot.lanes.present && state.snapshot.lanes.count == 3,
              "route: lane count");
        CHECK(state.snapshot.lanes.lanes[0] == 0x83 && state.snapshot.lanes.lanes[1] == 0x01 &&
              state.snapshot.lanes.lanes[2] == 0x02, "route: lane directions and recommendation");
        CHECK(strcmp(state.snapshot.next_street.text, "Тверская") == 0, "route: next street UTF-8");
        CHECK(state.snapshot.next_street.length == 16, "route: next street byte length");
        CHECK(strcmp(state.snapshot.current_street.text, "Ленина") == 0, "route: current street UTF-8");
        CHECK(state.snapshot.speed_limit.present && state.snapshot.speed_limit.kph == 60,
              "route: speed limit");
        CHECK(state.snapshot.remaining.present && state.snapshot.remaining.distance_m == 12500 &&
              state.snapshot.remaining.time_s == 3690, "route: remaining");
        CHECK(state.snapshot.meta.present && state.snapshot.meta.source == NAV_G10_SOURCE_SCREEN &&
              state.snapshot.meta.flags == 1, "route: meta screen + reader");
        CHECK(state.state_sequence == 101, "route: sequence");
        CHECK(state.groups_rx[NAV_GROUP_G0_SESSION] == 1 &&
              state.groups_rx[NAV_GROUP_G1_MANEUVER] == 1 &&
              state.groups_rx[NAV_GROUP_G2_LANES] == 1 &&
              state.groups_rx[NAV_GROUP_G3_NEXT_STREET] == 1 &&
              state.groups_rx[NAV_GROUP_G4_CURRENT_STREET] == 1 &&
              state.groups_rx[NAV_GROUP_G5_SPEED_LIMIT] == 1 &&
              state.groups_rx[NAV_GROUP_G8_REMAINING] == 1 &&
              state.groups_rx[NAV_GROUP_G10_META] == 1,
              "route: per-group counters");
        CHECK(state.groups_rx[NAV_GROUP_G6_CAMERA] == 0 &&
              state.groups_rx[NAV_GROUP_G7_ROAD_EVENT] == 0,
              "route: absent groups not counted");
        feed(&state, golden("route_right_300m"), 1000);
        CHECK(state.groups_rx[NAV_GROUP_G1_MANEUVER] == 2, "route: counters accumulate");
    }

    /* camera_1000m */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("camera_1000m"), 1000), "camera: accepted");
        CHECK(!state.snapshot.maneuver.present, "camera: no maneuver");
        CHECK(state.snapshot.camera.present && state.snapshot.camera.distance_m == 1000,
              "camera: distance");
        CHECK(state.snapshot.road_event.present &&
              state.snapshot.road_event.kind == NAV_ROAD_EVENT_KIND_ACCIDENT &&
              state.snapshot.road_event.distance_m == 800, "camera: road event");
        CHECK(state.snapshot.remaining.present, "camera: remaining preserved");
    }

    /* free */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("free"), 1000), "free: accepted");
        CHECK(state.snapshot.session.mode == NAV_MODE_FREE, "free: session mode");
        CHECK(!state.snapshot.maneuver.present && !state.snapshot.remaining.present,
              "free: no maneuver and no remaining");
        CHECK(strcmp(state.snapshot.current_street.text, "Ленина") == 0, "free: current street");
        CHECK(state.snapshot.speed_limit.kph == 60, "free: speed limit");
    }

    /* far_maneuver_70000m: no meta group */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("far_maneuver_70000m"), 1000), "far: accepted");
        CHECK(state.snapshot.maneuver.present && state.snapshot.maneuver.distance_m == 70000,
              "far: distance above 65535");
        CHECK(strcmp(state.snapshot.next_street.text, "М-7") == 0, "far: next street");
        CHECK(state.snapshot.speed_limit.kph == 90, "far: speed limit");
        CHECK(!state.snapshot.meta.present, "far: meta absent");
    }

    /* rerouting */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("rerouting"), 1000), "rerouting: accepted");
        CHECK(state.snapshot.maneuver.present &&
              state.snapshot.maneuver.kind == NAV_MANEUVER_KIND_REROUTING &&
              state.snapshot.maneuver.distance_m == 0, "rerouting: kind and distance");
        CHECK(!state.snapshot.lanes.present && !state.snapshot.camera.present,
              "rerouting: no lanes or camera");
        CHECK(state.snapshot.remaining.present, "rerouting: remaining preserved");
        CHECK(!state.snapshot.meta.present, "rerouting: meta absent");
    }

    /* roundabout: kind 12 is accepted and mapped, not dropped as unknown */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("roundabout"), 1000), "roundabout: accepted");
        CHECK(state.snapshot.maneuver.present &&
              state.snapshot.maneuver.kind == NAV_MANEUVER_KIND_ROUNDABOUT &&
              state.snapshot.maneuver.distance_m == 350, "roundabout: kind and distance");
        CHECK(state.snapshot.remaining.present && state.snapshot.remaining.distance_m == 4000 &&
              state.snapshot.remaining.time_s == 240, "roundabout: remaining");
        CHECK(state.snapshot.meta.present &&
              state.snapshot.meta.source == NAV_G10_SOURCE_NOTIFICATION, "roundabout: meta");
        CHECK(state.groups_rx[NAV_GROUP_G1_MANEUVER] == 1, "roundabout: G1 counted");
    }

    /* position */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("position"), 1000), "position: accepted");
        CHECK(state.position.valid, "position: valid");
        CHECK(fabs(state.position.lat - 55.7558) < 1e-9 && fabs(state.position.lon - 37.6173) < 1e-9,
              "position: coordinates");
        CHECK(fabs(state.position.speed - 12.5f) < 1e-6f, "position: speed");
        CHECK(fabs(state.position.bearing - 45.0f) < 1e-6f, "position: bearing");
        CHECK(fabs(state.position.accuracy - 3.5f) < 1e-6f, "position: accuracy");
        CHECK(state.position_sequence == 900, "position: sequence");
        CHECK(nav_proto_position_fresh(&state, 1000), "position: fresh at receive time");
    }

    /* free_camera */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("free_camera"), 1000), "free_camera: accepted");
        CHECK(state.snapshot.session.mode == NAV_MODE_FREE, "free_camera: session mode");
        CHECK(state.snapshot.camera.present && state.snapshot.camera.distance_m == 1000,
              "free_camera: camera without maneuver");
        CHECK(strcmp(state.snapshot.current_street.text, "Ленина") == 0,
              "free_camera: current street preserved");
        CHECK(state.snapshot.speed_limit.kph == 60, "free_camera: speed limit preserved");
    }

    /* position_nan */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("position_nan"), 1000), "position_nan: accepted");
        CHECK(state.position.valid, "position_nan: valid");
        CHECK(isnan(state.position.speed) && isnan(state.position.bearing),
              "position_nan: speed and bearing are NaN");
        CHECK(fabs(state.position.accuracy - 3.5f) < 1e-6f, "position_nan: accuracy preserved");
    }

    /* fragmentation: five chunks must equal the complete route message */
    {
        nav_state_t complete, assembled;
        nav_proto_init(&complete);
        nav_proto_init(&assembled);
        CHECK(feed(&complete, golden("route_right_300m"), 1000), "fragments: complete message accepted");
        bool accepted = false;
        for (unsigned i = 0; i < NAV_FRAGMENT_COUNT; i++) {
            const nav_golden_fragment *fragment = &nav_fragments[i];
            accepted = nav_proto_on_write(&assembled, fragment->data, fragment->len,
                                          1000u + i * 50u);
        }
        CHECK(accepted, "fragments: last chunk applied");
        CHECK(assembled.snapshots_ok == 1, "fragments: one snapshot");
        CHECK(memcmp(&assembled.snapshot, &complete.snapshot, sizeof(nav_snapshot_t)) == 0,
              "fragments: reassembled snapshot equals complete message");
        CHECK(assembled.state_sequence == complete.state_sequence, "fragments: sequence preserved");
    }

    /* fragment timeout: 1 s from the last chunk (sliding) */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(!nav_proto_on_write(&state, nav_fragments[0].data, nav_fragments[0].len, 1000),
              "timeout: first chunk is incomplete");
        CHECK(!nav_proto_on_write(&state, nav_fragments[1].data, nav_fragments[1].len, 2500),
              "timeout: late chunk is rejected");
        CHECK(state.fragment_resets == 1, "timeout: fragment buffer reset");
        CHECK(state.snapshots_ok == 0, "timeout: no snapshot applied");
    }

    /* negative vectors */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(!feed(&state, negative("bad_crc"), 1000), "bad_crc: rejected");
        CHECK(state.crc_errors == 1 && state.snapshots_ok == 0, "bad_crc: counter");

        nav_proto_init(&state);
        CHECK(!feed(&state, negative("unknown_version"), 1000), "unknown_version: rejected");
        CHECK(state.bad_versions == 1, "unknown_version: counter");

        nav_proto_init(&state);
        CHECK(!feed(&state, negative("unknown_type"), 1000), "unknown_type: rejected");
        CHECK(state.bad_types == 1, "unknown_type: counter");

        nav_proto_init(&state);
        CHECK(feed(&state, negative("unknown_tlv"), 1000), "unknown_tlv: known groups applied");
        CHECK(state.unknown_tlvs == 1, "unknown_tlv: counter");
        CHECK(state.snapshot.session.mode == NAV_MODE_IDLE, "unknown_tlv: session");
        CHECK(state.snapshot.meta.present, "unknown_tlv: meta");

        nav_proto_init(&state);
        CHECK(feed(&state, negative("wrong_group_length"), 1000),
              "wrong_group_length: message applied");
        CHECK(state.bad_lengths == 1, "wrong_group_length: counter");
        CHECK(state.snapshot.session.mode == NAV_MODE_ROUTE, "wrong_group_length: session");
        CHECK(!state.snapshot.speed_limit.present, "wrong_group_length: malformed group dropped");

        nav_proto_init(&state);
        CHECK(feed(&state, negative("duplicate_group"), 1000), "duplicate_group: message applied");
        CHECK(state.snapshot.speed_limit.present && state.snapshot.speed_limit.kph == 80,
              "duplicate_group: last occurrence wins");
    }

    /* watchdog: STALE at 3 s, zones cleared at 15 s */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("route_right_300m"), 1000), "watchdog: snapshot accepted");
        nav_proto_tick(&state, 4001);
        CHECK(state.stale, "watchdog: STALE after 3 s");
        CHECK(!nav_proto_state_fresh(&state, 4001), "watchdog: state not fresh");
        CHECK(state.snapshot.maneuver.present, "watchdog: last snapshot kept while STALE");
        nav_proto_tick(&state, 16001);
        CHECK(!state.snapshot.maneuver.present && !state.snapshot.lanes.present &&
              !state.snapshot.next_street.present && !state.snapshot.current_street.present &&
              !state.snapshot.speed_limit.present && !state.snapshot.remaining.present,
              "watchdog: G1..G8 cleared after 15 s");
        CHECK(state.snapshot.session.present, "watchdog: G0 kept");
    }

    /* position TTL: 5 s by receive time */
    {
        nav_state_t state;
        nav_proto_init(&state);
        CHECK(feed(&state, golden("position"), 1000), "position TTL: accepted");
        CHECK(nav_proto_position_fresh(&state, 1000), "position TTL: fresh");
        nav_proto_tick(&state, 6001);
        CHECK(!state.position.valid, "position TTL: hidden after 5 s");
        CHECK(!nav_proto_position_fresh(&state, 6001), "position TTL: not fresh");
    }

    if (failures == 0) {
        printf("PASS: nav_proto decoder host test\n");
        return 0;
    }
    printf("FAIL: %d check(s)\n", failures);
    return 1;
}
