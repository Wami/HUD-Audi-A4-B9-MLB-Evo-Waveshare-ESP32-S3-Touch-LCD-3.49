/*
  nav_proto.c — decoder for the YHUD Android -> ESP32 BLE navigation protocol v1.

  Pure C, no Arduino/ESP-IDF dependencies. See protocol/spec.md and nav_proto.h.
  The decoder is fail-closed: a malformed message never partially overwrites the
  current snapshot. A valid type=1 message replaces the whole snapshot (absence
  means "no data"); a valid type=2 message replaces only the position stream.
*/
#include "nav_proto.h"

#include <math.h>
#include <string.h>

typedef enum {
    GROUP_OK,
    GROUP_ABSENT,
    GROUP_BAD_LEN,
    GROUP_BAD_VALUE,
    GROUP_UNKNOWN
} group_result_t;

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int64_t rd_i64(const uint8_t *p)
{
    uint64_t u = 0;
    for (int i = 7; i >= 0; i--) u = (u << 8) | p[i];
    int64_t v;
    memcpy(&v, &u, sizeof v);
    return v;
}

static float rd_f32(const uint8_t *p)
{
    float v;
    memcpy(&v, p, sizeof v);
    return v;
}

static double rd_f64(const uint8_t *p)
{
    double v;
    memcpy(&v, p, sizeof v);
    return v;
}

uint16_t nav_proto_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint8_t utf8_valid_prefix(const uint8_t *src, uint8_t len)
{
    uint8_t i = 0;
    while (i < len) {
        uint8_t c = src[i];
        uint8_t need;
        if (c < 0x80) need = 0;
        else if ((c & 0xE0) == 0xC0) need = 1;
        else if ((c & 0xF0) == 0xE0) need = 2;
        else if ((c & 0xF8) == 0xF0) need = 3;
        else break;
        if ((size_t)i + 1 + need > len) break;
        bool ok = true;
        for (uint8_t k = 1; k <= need; k++) {
            if ((src[i + k] & 0xC0) != 0x80) { ok = false; break; }
        }
        if (!ok) break;
        i = (uint8_t)(i + 1 + need);
    }
    return i;
}

static bool text_set(nav_text_t *dst, const uint8_t *src, uint8_t len)
{
    if (len == 0) return false;
    uint8_t n = utf8_valid_prefix(src, len);
    if (n == 0) return false;
    if (n > NAV_PROTO_MAX_TEXT_BYTES) {
        n = NAV_PROTO_MAX_TEXT_BYTES;
        while (n > 0 && (src[n] & 0xC0) == 0x80) n--;
    }
    memcpy(dst->text, src, n);
    dst->text[n] = '\0';
    dst->length = n;
    dst->present = true;
    return true;
}

static void snapshot_clear(nav_snapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof *snapshot);
}

static group_result_t parse_g0(nav_snapshot_t *out, const uint8_t *v, uint8_t len, bool *has_session)
{
    if (len != 1) return GROUP_BAD_LEN;
    if (v[0] > NAV_MODE_FREE) return GROUP_BAD_VALUE;
    out->session.present = true;
    out->session.mode = v[0];
    *has_session = true;
    return GROUP_OK;
}

static group_result_t parse_g1(nav_snapshot_t *out, const uint8_t *v, uint8_t len)
{
    if (len != 5) return GROUP_BAD_LEN;
    uint8_t kind = v[0];
    if (kind == NAV_MANEUVER_KIND_RESERVED_NONE) return GROUP_ABSENT;
    if (kind > NAV_MANEUVER_KIND_REROUTING) return GROUP_ABSENT;
    uint32_t distance = rd_u32(v + 1);
    if (distance > 2000000u) return GROUP_ABSENT;
    out->maneuver.present = true;
    out->maneuver.kind = kind;
    out->maneuver.distance_m = distance;
    return GROUP_OK;
}

static group_result_t parse_g2(nav_snapshot_t *out, const uint8_t *v, uint8_t len)
{
    if (len < 2 || len > 9) return GROUP_BAD_LEN;
    uint8_t count = v[0];
    if (count == 0 || count > NAV_PROTO_LANE_COUNT || len != 1u + count) return GROUP_ABSENT;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t lane = v[1 + i];
        uint8_t direction = lane & 0x0F;
        if ((lane & 0x70) != 0) return GROUP_ABSENT;
        if (direction < NAV_LANE_DIRECTION_STRAIGHT || direction > NAV_LANE_DIRECTION_U_TURN_LEFT)
            return GROUP_ABSENT;
    }
    out->lanes.present = true;
    out->lanes.count = count;
    memcpy(out->lanes.lanes, v + 1, count);
    return GROUP_OK;
}

static group_result_t parse_text(nav_text_t *out, const uint8_t *v, uint8_t len)
{
    if (len == 0) return GROUP_BAD_LEN;
    return text_set(out, v, len) ? GROUP_OK : GROUP_ABSENT;
}

static group_result_t parse_g5(nav_snapshot_t *out, const uint8_t *v, uint8_t len)
{
    if (len != 1) return GROUP_BAD_LEN;
    if (v[0] < 5 || v[0] > 200) return GROUP_ABSENT;
    out->speed_limit.present = true;
    out->speed_limit.kph = v[0];
    return GROUP_OK;
}

static group_result_t parse_g6(nav_snapshot_t *out, const uint8_t *v, uint8_t len)
{
    if (len != 2) return GROUP_BAD_LEN;
    uint16_t distance = rd_u16(v);
    if (distance == 0 || distance > 5000) return GROUP_ABSENT;
    out->camera.present = true;
    out->camera.distance_m = distance;
    return GROUP_OK;
}

static group_result_t parse_g7(nav_snapshot_t *out, const uint8_t *v, uint8_t len)
{
    if (len != 3) return GROUP_BAD_LEN;
    uint8_t kind = v[0];
    uint16_t distance = rd_u16(v + 1);
    if (kind < NAV_ROAD_EVENT_KIND_ACCIDENT || kind > NAV_ROAD_EVENT_KIND_SPEED_BUMP)
        return GROUP_ABSENT;
    if (distance > 5000) return GROUP_ABSENT;
    out->road_event.present = true;
    out->road_event.kind = kind;
    out->road_event.distance_m = distance;
    return GROUP_OK;
}

static group_result_t parse_g8(nav_snapshot_t *out, const uint8_t *v, uint8_t len)
{
    if (len != 8) return GROUP_BAD_LEN;
    uint32_t distance = rd_u32(v);
    uint32_t time_s = rd_u32(v + 4);
    if (distance > 50000000u || time_s > 3000000u) return GROUP_ABSENT;
    if (distance == 0 && time_s == 0) return GROUP_ABSENT;
    out->remaining.present = true;
    out->remaining.distance_m = distance;
    out->remaining.time_s = time_s;
    return GROUP_OK;
}

static group_result_t parse_g10(nav_snapshot_t *out, const uint8_t *v, uint8_t len)
{
    if (len != 10) return GROUP_BAD_LEN;
    if (v[0] > NAV_G10_SOURCE_SCREEN) return GROUP_ABSENT;
    out->meta.present = true;
    out->meta.source = v[0];
    out->meta.flags = v[1];
    out->meta.observed_at_ms = rd_i64(v + 2);
    return GROUP_OK;
}

static void count_group(nav_state_t *state, uint8_t group, group_result_t result)
{
    switch (result) {
    case GROUP_OK:
        if (group < NAV_PROTO_GROUP_COUNT) state->groups_rx[group]++;
        break;
    case GROUP_BAD_LEN:
        state->bad_lengths++;
        break;
    case GROUP_BAD_VALUE:
        state->bad_values++;
        break;
    case GROUP_UNKNOWN:
        state->unknown_tlvs++;
        break;
    case GROUP_ABSENT:
        break;
    }
}

static bool parse_state(nav_state_t *state, const uint8_t *msg, size_t len, uint32_t now_ms)
{
    nav_snapshot_t parsed;
    snapshot_clear(&parsed);
    bool has_session = false;
    size_t i = 8;
    size_t end = len - 2;

    while (i + 2 <= end) {
        uint8_t group = msg[i];
        uint8_t glen = msg[i + 1];
        if (i + 2 + glen > end) { state->bad_lengths++; return false; }
        const uint8_t *v = msg + i + 2;
        group_result_t result;
        switch (group) {
        case NAV_GROUP_G0_SESSION: result = parse_g0(&parsed, v, glen, &has_session); break;
        case NAV_GROUP_G1_MANEUVER: result = parse_g1(&parsed, v, glen); break;
        case NAV_GROUP_G2_LANES: result = parse_g2(&parsed, v, glen); break;
        case NAV_GROUP_G3_NEXT_STREET: result = parse_text(&parsed.next_street, v, glen); break;
        case NAV_GROUP_G4_CURRENT_STREET: result = parse_text(&parsed.current_street, v, glen); break;
        case NAV_GROUP_G5_SPEED_LIMIT: result = parse_g5(&parsed, v, glen); break;
        case NAV_GROUP_G6_CAMERA: result = parse_g6(&parsed, v, glen); break;
        case NAV_GROUP_G7_ROAD_EVENT: result = parse_g7(&parsed, v, glen); break;
        case NAV_GROUP_G8_REMAINING: result = parse_g8(&parsed, v, glen); break;
        case NAV_GROUP_G9_POSITION: result = GROUP_BAD_VALUE; break; /* forbidden in type 1 */
        case NAV_GROUP_G10_META: result = parse_g10(&parsed, v, glen); break;
        default: result = GROUP_UNKNOWN; break;
        }
        count_group(state, group, result);
        i += 2u + glen;
    }
    if (i != end) { state->bad_lengths++; return false; }
    if (!has_session) { state->bad_snapshots++; return false; }

    state->snapshot = parsed;
    state->state_sequence = rd_u32(msg + 2);
    state->state_received_ms = now_ms ? now_ms : 1;
    state->snapshots_ok++;
    state->stale = false;
    return true;
}

static bool position_value_valid(nav_position_t *position, const uint8_t *v)
{
    double lat = rd_f64(v);
    double lon = rd_f64(v + 8);
    float speed = rd_f32(v + 16);
    float bearing = rd_f32(v + 20);
    float accuracy = rd_f32(v + 24);
    int64_t timestamp = rd_i64(v + 28);

    if (!isfinite(lat) || lat < -90.0 || lat > 90.0) return false;
    if (!isfinite(lon) || lon < -180.0 || lon > 180.0) return false;
    if (!isnan(accuracy)) {
        if (!isfinite(accuracy) || accuracy <= 0.0f || accuracy > 100.0f) return false;
    }
    if (!isnan(speed) && (!isfinite(speed) || speed < 0.0f)) speed = NAN;
    if (!isnan(bearing) && !isfinite(bearing)) bearing = NAN;

    position->valid = true;
    position->lat = lat;
    position->lon = lon;
    position->speed = speed;
    position->bearing = bearing;
    position->accuracy = accuracy;
    position->timestamp = timestamp;
    return true;
}

static bool parse_position(nav_state_t *state, const uint8_t *msg, size_t len, uint32_t now_ms)
{
    nav_position_t parsed;
    memset(&parsed, 0, sizeof parsed);
    bool has_position = false;
    bool extra_group = false;
    size_t i = 8;
    size_t end = len - 2;

    while (i + 2 <= end) {
        uint8_t group = msg[i];
        uint8_t glen = msg[i + 1];
        if (i + 2 + glen > end) { state->bad_lengths++; return false; }
        const uint8_t *v = msg + i + 2;
        if (group == NAV_GROUP_G9_POSITION) {
            if (glen != 36) {
                state->bad_lengths++;
            } else if (has_position) {
                state->bad_values++;
                extra_group = true;
            } else if (position_value_valid(&parsed, v)) {
                has_position = true;
                state->groups_rx[NAV_GROUP_G9_POSITION]++;
            } else {
                state->bad_values++;
            }
        } else {
            extra_group = true;
            state->bad_values++;
        }
        i += 2u + glen;
    }
    if (i != end) { state->bad_lengths++; return false; }
    if (!has_position || extra_group) return false;

    state->position = parsed;
    state->position_sequence = rd_u32(msg + 2);
    state->position_received_ms = now_ms ? now_ms : 1;
    state->positions_ok++;
    return true;
}

void nav_proto_init(nav_state_t *state)
{
    memset(state, 0, sizeof *state);
    state->stale = true;
}

bool nav_proto_on_write(nav_state_t *state, const uint8_t *data, size_t len, uint32_t now_ms)
{
    if (state == NULL || data == NULL || len == 0) return false;

    const uint8_t *msg = data;
    size_t msg_len = len;

    if (data[0] == NAV_PROTO_FRAGMENT_MAGIC) {
        if (len < 3) { state->fragment_resets++; return false; }
        uint8_t index = data[1];
        uint8_t total = data[2];
        if (total == 0 || index >= total) {
            state->fragment_resets++;
            state->frag_active = false;
            return false;
        }
        if (index == 0) {
            state->frag_len = 0;
            state->frag_total = total;
            state->frag_index = 0;
            state->frag_active = true;
        } else if (!state->frag_active || total != state->frag_total ||
                   index != state->frag_index ||
                   (uint32_t)(now_ms - state->frag_last_ms) > NAV_PROTO_ASSEMBLY_TIMEOUT_MS) {
            state->fragment_resets++;
            state->frag_active = false;
            return false;
        }
        size_t chunk = len - 3;
        if ((size_t)state->frag_len + chunk > NAV_PROTO_MAX_MESSAGE_BYTES) {
            state->fragment_resets++;
            state->frag_active = false;
            return false;
        }
        memcpy(state->frag_buf + state->frag_len, data + 3, chunk);
        state->frag_len = (uint16_t)(state->frag_len + chunk);
        state->frag_index++;
        state->frag_last_ms = now_ms;
        if ((uint8_t)(index + 1) < total) return false;
        msg = state->frag_buf;
        msg_len = state->frag_len;
        state->frag_active = false;
        state->frag_len = 0;
        state->frag_index = 0;
        state->frag_total = 0;
    }

    if (msg_len < 13) return false;
    if (msg[0] != NAV_PROTO_VERSION) { state->bad_versions++; return false; }
    uint8_t type = msg[1];
    if (type != 1 && type != 2) { state->bad_types++; return false; }
    if (rd_u16(msg + msg_len - 2) != nav_proto_crc16(msg, msg_len - 2)) {
        state->crc_errors++;
        return false;
    }
    return type == 1 ? parse_state(state, msg, msg_len, now_ms)
                     : parse_position(state, msg, msg_len, now_ms);
}

void nav_proto_tick(nav_state_t *state, uint32_t now_ms)
{
    if (state == NULL) return;

    if (state->state_received_ms == 0) {
        state->stale = true;
    } else {
        uint32_t age = now_ms - state->state_received_ms;
        if (age > NAV_PROTO_CLEAR_MS) {
            state->stale = true;
            state->snapshot.maneuver.present = false;
            state->snapshot.lanes.present = false;
            state->snapshot.next_street.present = false;
            state->snapshot.current_street.present = false;
            state->snapshot.speed_limit.present = false;
            state->snapshot.camera.present = false;
            state->snapshot.road_event.present = false;
            state->snapshot.remaining.present = false;
        } else {
            state->stale = age > NAV_PROTO_STALE_MS;
        }
    }

    if (state->position.valid &&
        (state->position_received_ms == 0 ||
         (uint32_t)(now_ms - state->position_received_ms) > NAV_PROTO_POSITION_TTL_MS))
        state->position.valid = false;
}

bool nav_proto_state_fresh(const nav_state_t *state, uint32_t now_ms)
{
    if (state == NULL || state->state_received_ms == 0) return false;
    return (uint32_t)(now_ms - state->state_received_ms) <= NAV_PROTO_STALE_MS;
}

bool nav_proto_position_fresh(const nav_state_t *state, uint32_t now_ms)
{
    if (state == NULL || !state->position.valid || state->position_received_ms == 0) return false;
    return (uint32_t)(now_ms - state->position_received_ms) <= NAV_PROTO_POSITION_TTL_MS;
}
