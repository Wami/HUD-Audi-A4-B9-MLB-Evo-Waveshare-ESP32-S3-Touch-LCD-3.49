/*
  ble_nav_server.cpp — BLE GATT peripheral for the phone navigation channel.

  NimBLE dual role: this server runs alongside the CAN sniffer client. The phone
  is the only central that connects to this server; a second phone is rejected.
  The CAN sniffer's BLE_PASSKEY protection is intentionally not applied here:
  the `nav` characteristic must be writable without pairing.
*/
#include "ble_nav_server.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "hud_config.h"
#include "hud_log.h"

static nav_state_t s_state;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_connected = false;
static volatile uint16_t s_phone_conn = 0xFFFF;
static bool s_started = false;
static uint32_t s_log_ms = 0;

static void state_reset_locked(void)
{
    taskENTER_CRITICAL(&s_mux);
    nav_proto_init(&s_state);
    taskEXIT_CRITICAL(&s_mux);
}

class NavServerCallbacks : public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer *server, NimBLEConnInfo &info) override
    {
        uint16_t handle = info.getConnHandle();
        if (s_phone_conn != 0xFFFF && s_phone_conn != handle) {
            if (HUD_LOG_BLE) hud_log_write("[nav] second phone connection rejected\n");
            server->disconnect(info);
            return;
        }
        s_phone_conn = handle;
        s_connected = true;
        if (HUD_LOG_BLE) hud_log_write("[nav] phone connected\n");
    }

    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &info, int reason) override
    {
        (void)server;
        (void)reason;
        if (s_phone_conn != info.getConnHandle()) return;
        s_phone_conn = 0xFFFF;
        s_connected = false;
        state_reset_locked();
        if (HUD_LOG_BLE) hud_log_write("[nav] phone disconnected\n");
    }
};

class NavCharCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &info) override
    {
        (void)info;
        NimBLEAttValue value = characteristic->getValue();
        uint32_t now = millis();
        bool applied;
        nav_snapshot_t snapshot;
        nav_position_t position;
        taskENTER_CRITICAL(&s_mux);
        applied = nav_proto_on_write(&s_state, value.data(), value.length(), now ? now : 1);
        snapshot = s_state.snapshot;
        position = s_state.position;
        taskEXIT_CRITICAL(&s_mux);
        if (HUD_LOG_BLE && applied && (uint32_t)(now - s_log_ms) >= 1000u) {
            s_log_ms = now;
            arduino_printf("[nav] G1=%d/%u/%lu G11=%d/%u G5=%d/%u G6=%d/%u G8=%d/%lu/%lu G9=%d %.5f,%.5f\n",
                           snapshot.maneuver.present, (unsigned)snapshot.maneuver.kind,
                           (unsigned long)snapshot.maneuver.distance_m,
                           snapshot.roundabout_exit.present, (unsigned)snapshot.roundabout_exit.exit_no,
                           snapshot.speed_limit.present, (unsigned)snapshot.speed_limit.kph,
                           snapshot.camera.present, (unsigned)snapshot.camera.distance_m,
                           snapshot.remaining.present, (unsigned long)snapshot.remaining.distance_m,
                           (unsigned long)snapshot.remaining.time_s,
                           position.valid, position.lat, position.lon);
        }
    }
};

void nav_server_start(void)
{
    if (s_started) return;
    s_started = true;
    nav_proto_init(&s_state);

    if (!NimBLEDevice::isInitialized()) {
        NimBLEDevice::init("HUD-3.49");
    }
    NimBLEDevice::setMTU(NAV_PROTO_MTU_REQUEST);

    NimBLEServer *server = NimBLEDevice::createServer();
    if (server == nullptr) {
        if (HUD_LOG_BLE) hud_log_write("[nav] createServer failed\n");
        return;
    }
    server->setCallbacks(new NavServerCallbacks());
    server->advertiseOnDisconnect(true);

    NimBLEService *service = server->createService(NimBLEUUID(NAV_SERVICE_UUID));
    if (service == nullptr) {
        if (HUD_LOG_BLE) hud_log_write("[nav] createService failed\n");
        return;
    }
    NimBLECharacteristic *nav = service->createCharacteristic(
        NimBLEUUID(NAV_CHAR_UUID),
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    if (nav == nullptr) {
        if (HUD_LOG_BLE) hud_log_write("[nav] createCharacteristic failed\n");
        return;
    }
    nav->setCallbacks(new NavCharCallbacks());
    server->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->enableScanResponse(true);      /* the 12-byte name goes into the 31-byte scan response */
    advertising->addServiceUUID(service->getUUID());
    advertising->setName(NAV_ADV_NAME);
    advertising->start();

    if (HUD_LOG_BLE) hud_log_write("[nav] GATT server started, advertising " NAV_ADV_NAME "\n");
}

bool nav_server_connected(void)
{
    return s_connected;
}

bool nav_server_copy(nav_state_t *out)
{
    if (out == nullptr) return false;
    uint32_t now = millis();
    if (now == 0) now = 1;
    taskENTER_CRITICAL(&s_mux);
    *out = s_state;
    taskEXIT_CRITICAL(&s_mux);
    nav_proto_tick(out, now);
    return s_connected;
}

uint32_t nav_server_snapshots(void)
{
    taskENTER_CRITICAL(&s_mux);
    uint32_t value = s_state.snapshots_ok;
    taskEXIT_CRITICAL(&s_mux);
    return value;
}

uint32_t nav_server_crc_errors(void)
{
    taskENTER_CRITICAL(&s_mux);
    uint32_t value = s_state.crc_errors;
    taskEXIT_CRITICAL(&s_mux);
    return value;
}

uint32_t nav_server_positions(void)
{
    taskENTER_CRITICAL(&s_mux);
    uint32_t value = s_state.positions_ok;
    taskEXIT_CRITICAL(&s_mux);
    return value;
}

uint32_t nav_server_bad_lengths(void)
{
    taskENTER_CRITICAL(&s_mux);
    uint32_t value = s_state.bad_lengths;
    taskEXIT_CRITICAL(&s_mux);
    return value;
}

uint32_t nav_server_group_count(uint8_t group)
{
    if (group >= NAV_PROTO_GROUP_COUNT) return 0;
    taskENTER_CRITICAL(&s_mux);
    uint32_t value = s_state.groups_rx[group];
    taskEXIT_CRITICAL(&s_mux);
    return value;
}

uint32_t nav_server_unknown_tlvs(void)
{
    taskENTER_CRITICAL(&s_mux);
    uint32_t value = s_state.unknown_tlvs;
    taskEXIT_CRITICAL(&s_mux);
    return value;
}
