/*
  nav_proto_edge_test.c — edge cases for the pure-C navigation decoder.

  Build from the repository root:
    gcc -O2 -std=c11 -Wall -Wextra -Werror -o /tmp/nav_proto_edge_test \
        tools/nav_proto_edge_test.c waveshare_hud_mockup/nav_proto.c \
        -Iwaveshare_hud_mockup -lm
    /tmp/nav_proto_edge_test
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

static size_t add_tlv(uint8_t *buf, size_t n, uint8_t group, const void *value, uint8_t len)
{
    buf[n++] = group;
    buf[n++] = len;
    if (len) memcpy(buf + n, value, len);
    return n + len;
}

static size_t build(uint8_t *out, uint8_t type, uint32_t seq, const uint8_t *tlvs, size_t tlen)
{
    size_t n = 0;
    out[n++] = 1;
    out[n++] = type;
    out[n++] = (uint8_t)seq;
    out[n++] = (uint8_t)(seq >> 8);
    out[n++] = (uint8_t)(seq >> 16);
    out[n++] = (uint8_t)(seq >> 24);
    out[n++] = 0;
    out[n++] = 0;
    memcpy(out + n, tlvs, tlen);
    n += tlen;
    uint16_t crc = nav_proto_crc16(out, n);
    out[n++] = (uint8_t)crc;
    out[n++] = (uint8_t)(crc >> 8);
    return n;
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void put_g9(uint8_t *v, double lat, double lon, float speed, float bearing,
                   float accuracy, int64_t timestamp)
{
    memcpy(v, &lat, 8);
    memcpy(v + 8, &lon, 8);
    memcpy(v + 16, &speed, 4);
    memcpy(v + 20, &bearing, 4);
    memcpy(v + 24, &accuracy, 4);
    memcpy(v + 28, &timestamp, 8);
}

static uint8_t tlvs[512];
static uint8_t msg[512];
static nav_state_t state;

static size_t state_with(uint8_t *out, uint32_t seq, size_t tlen)
{
    return build(out, 1, seq, tlvs, tlen);
}

int main(void)
{
    /* type=1 without the mandatory G0 is rejected and does not commit groups. */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g5 = 60;
        t = add_tlv(tlvs, t, NAV_GROUP_G5_SPEED_LIMIT, &g5, 1);
        size_t n = state_with(msg, 1, t);
        CHECK(!nav_proto_on_write(&state, msg, n, 1000), "no G0: message rejected");
        CHECK(state.bad_snapshots == 1, "no G0: counter");
        CHECK(!state.snapshot.speed_limit.present, "no G0: groups not committed");
    }

    /* G0 with an unknown mode is treated as absent, so the snapshot is rejected. */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g0 = 3, g5 = 60;
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G5_SPEED_LIMIT, &g5, 1);
        size_t n = state_with(msg, 2, t);
        CHECK(!nav_proto_on_write(&state, msg, n, 1000), "bad mode: rejected");
        CHECK(state.bad_values >= 1 && state.bad_snapshots == 1, "bad mode: counters");
    }

    /* G9 is forbidden in type=1: ignore it, apply the other groups. */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g0 = 1, g5 = 60, g9[36];
        put_g9(g9, 55.0, 37.0, 10.0f, 90.0f, 5.0f, 0);
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G5_SPEED_LIMIT, &g5, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G9_POSITION, g9, 36);
        size_t n = state_with(msg, 3, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "G9 in type1: message applied");
        CHECK(state.snapshot.speed_limit.present, "G9 in type1: other groups applied");
        CHECK(!state.position.valid, "G9 in type1: position not applied");
        CHECK(state.bad_values == 1, "G9 in type1: counter");
    }

    /* type=2: exactly one G9 and no other groups. */
    {
        nav_proto_init(&state);
        size_t t = 0, n;
        uint8_t g9[36], g5 = 60;
        put_g9(g9, 55.75, 37.62, 12.5f, 45.0f, 3.5f, 0);
        t = add_tlv(tlvs, t, NAV_GROUP_G9_POSITION, g9, 36);
        t = add_tlv(tlvs, t, NAV_GROUP_G5_SPEED_LIMIT, &g5, 1);
        n = build(msg, 2, 10, tlvs, t);
        CHECK(!nav_proto_on_write(&state, msg, n, 1000), "position+extra: rejected");
        CHECK(state.positions_ok == 0 && state.bad_values >= 1, "position+extra: counters");

        nav_proto_init(&state);
        t = 0;
        t = add_tlv(tlvs, t, NAV_GROUP_G9_POSITION, g9, 36);
        n = build(msg, 2, 11, tlvs, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "position only: accepted");
        CHECK(state.positions_ok == 1 && state.position.valid, "position only: applied");
    }

    /* Duplicate G9 in type=2 is rejected. */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g9[36];
        put_g9(g9, 55.75, 37.62, 12.5f, 45.0f, 3.5f, 0);
        t = add_tlv(tlvs, t, NAV_GROUP_G9_POSITION, g9, 36);
        t = add_tlv(tlvs, t, NAV_GROUP_G9_POSITION, g9, 36);
        size_t n = build(msg, 2, 12, tlvs, t);
        CHECK(!nav_proto_on_write(&state, msg, n, 1000), "duplicate G9: rejected");
        CHECK(state.positions_ok == 0 && state.bad_values >= 1, "duplicate G9: counters");
    }

    /* Position validation: accuracy > 100 and lat out of range reject the stream. */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g9[36];
        put_g9(g9, 55.75, 37.62, 12.5f, 45.0f, 101.0f, 0);
        t = add_tlv(tlvs, t, NAV_GROUP_G9_POSITION, g9, 36);
        size_t n = build(msg, 2, 13, tlvs, t);
        CHECK(!nav_proto_on_write(&state, msg, n, 1000), "accuracy 101: rejected");
        CHECK(!state.position.valid, "accuracy 101: no position");

        nav_proto_init(&state);
        t = 0;
        put_g9(g9, 91.0, 37.62, 12.5f, 45.0f, 3.5f, 0);
        t = add_tlv(tlvs, t, NAV_GROUP_G9_POSITION, g9, 36);
        n = build(msg, 2, 14, tlvs, t);
        CHECK(!nav_proto_on_write(&state, msg, n, 1000), "lat 91: rejected");
    }

    /* NaN accuracy is valid; NaN speed/bearing are accepted. */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g9[36];
        put_g9(g9, 55.75, 37.62, NAN, NAN, NAN, 0);
        t = add_tlv(tlvs, t, NAV_GROUP_G9_POSITION, g9, 36);
        size_t n = build(msg, 2, 15, tlvs, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "NaN position: accepted");
        CHECK(state.position.valid && isnan(state.position.accuracy), "NaN position: applied");
    }

    /* G5 outside 5..200 means "no group", but the snapshot is still valid. */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g0 = 1, g5 = 4;
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G5_SPEED_LIMIT, &g5, 1);
        size_t n = state_with(msg, 20, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "G5=4: snapshot accepted");
        CHECK(!state.snapshot.speed_limit.present, "G5=4: group absent");

        nav_proto_init(&state);
        t = 0; g5 = 201;
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G5_SPEED_LIMIT, &g5, 1);
        n = state_with(msg, 21, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "G5=201: snapshot accepted");
        CHECK(!state.snapshot.speed_limit.present, "G5=201: group absent");
    }

    /* G6 0 / >5000 means "no camera". */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g0 = 1, g6[2];
        put_u16(g6, 0);
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G6_CAMERA, g6, 2);
        size_t n = state_with(msg, 22, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "G6=0: snapshot accepted");
        CHECK(!state.snapshot.camera.present, "G6=0: group absent");

        nav_proto_init(&state);
        t = 0; put_u16(g6, 5001);
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G6_CAMERA, g6, 2);
        n = state_with(msg, 23, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "G6=5001: snapshot accepted");
        CHECK(!state.snapshot.camera.present, "G6=5001: group absent");
    }

    /* G7 unknown kind means "no group". */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g0 = 1, g7[3] = { 6, 0, 0 };
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G7_ROAD_EVENT, g7, 3);
        size_t n = state_with(msg, 24, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "G7 kind 6: snapshot accepted");
        CHECK(!state.snapshot.road_event.present, "G7 kind 6: group absent");
    }

    /* G8 with both zero means "no remaining". */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g0 = 1, g8[8] = { 0 };
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G8_REMAINING, g8, 8);
        size_t n = state_with(msg, 25, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "G8 zero: snapshot accepted");
        CHECK(!state.snapshot.remaining.present, "G8 zero: group absent");
    }

    /* G2: invalid count, reserved bits and unknown direction drop the group. */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g0 = 1, g2[2] = { 1, 0x11 };     /* reserved bit 0x10 */
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G2_LANES, g2, 2);
        size_t n = state_with(msg, 26, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "G2 reserved: snapshot accepted");
        CHECK(!state.snapshot.lanes.present, "G2 reserved: group absent");

        nav_proto_init(&state);
        t = 0; g2[1] = 0x06;                      /* direction 6 */
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G2_LANES, g2, 2);
        n = state_with(msg, 27, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "G2 dir 6: snapshot accepted");
        CHECK(!state.snapshot.lanes.present, "G2 dir 6: group absent");

        nav_proto_init(&state);
        t = 0;
        uint8_t g2bad[3] = { 3, 0x01, 0x02 };     /* count 3, only two lanes */
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G2_LANES, g2bad, 3);
        n = state_with(msg, 28, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "G2 mismatch: snapshot accepted");
        CHECK(!state.snapshot.lanes.present, "G2 mismatch: group absent");
    }

    /* Text: invalid UTF-8 drops the group; 65 bytes trim to a code-point boundary. */
    {
        nav_proto_init(&state);
        size_t t = 0;
        uint8_t g0 = 1, bad = 0xD0;
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G3_NEXT_STREET, &bad, 1);
        size_t n = state_with(msg, 30, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "bad UTF-8: snapshot accepted");
        CHECK(!state.snapshot.next_street.present, "bad UTF-8: group absent");

        uint8_t text65[65];
        for (int i = 0; i < 32; i++) { text65[2 * i] = 0xD1; text65[2 * i + 1] = 0x8F; } /* 32 x 'я' */
        text65[64] = 0xD1;                                                               /* incomplete */
        nav_proto_init(&state);
        t = 0;
        t = add_tlv(tlvs, t, NAV_GROUP_G0_SESSION, &g0, 1);
        t = add_tlv(tlvs, t, NAV_GROUP_G3_NEXT_STREET, text65, 65);
        n = state_with(msg, 31, t);
        CHECK(nav_proto_on_write(&state, msg, n, 1000), "text 65: snapshot accepted");
        CHECK(state.snapshot.next_street.present, "text 65: group present");
        CHECK(state.snapshot.next_street.length == 64, "text 65: trimmed to 64 bytes");
        CHECK(memcmp(state.snapshot.next_street.text, text65, 64) == 0, "text 65: code-point boundary");
    }

    /* Fragment gap resets the partial buffer; a complete sequence still assembles. */
    {
        nav_proto_init(&state);
        CHECK(!nav_proto_on_write(&state, nav_fragments[0].data, nav_fragments[0].len, 1000),
              "gap: first chunk incomplete");
        CHECK(!nav_proto_on_write(&state, nav_fragments[2].data, nav_fragments[2].len, 1050),
              "gap: skipped chunk rejected");
        CHECK(state.fragment_resets == 1 && state.snapshots_ok == 0, "gap: counter and no snapshot");

        nav_proto_init(&state);
        bool accepted = false;
        for (unsigned i = 0; i < NAV_FRAGMENT_COUNT; i++)
            accepted = nav_proto_on_write(&state, nav_fragments[i].data, nav_fragments[i].len,
                                          1000u + i * 50u);
        CHECK(accepted && state.snapshots_ok == 1, "gap: complete sequence still assembles");
    }

    if (failures == 0) {
        printf("PASS: nav_proto edge-case host test\n");
        return 0;
    }
    printf("FAIL: %d check(s)\n", failures);
    return 1;
}
