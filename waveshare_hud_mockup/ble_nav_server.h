/*
  ble_nav_server.h — BLE GATT peripheral for the phone navigation channel.

  Runs independently of HUD_DATA_SOURCE and coexists with the CAN sniffer
  (central role). The phone connects as GATT central and writes protocol v1
  messages to the `nav` characteristic.
*/
#ifndef BLE_NAV_SERVER_H
#define BLE_NAV_SERVER_H

#include <stdbool.h>
#include <stdint.h>

#include "nav_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the GATT server and advertising. Call once from setup(), before the
   CAN source starts so NimBLE is initialized exactly once. */
void nav_server_start(void);

bool nav_server_connected(void);

/* Copy the latest decoded state under the internal lock and apply the watchdog.
   Returns true while a phone is connected. Use nav_proto_state_fresh() and
   nav_proto_position_fresh() to test the independent streams. */
bool nav_server_copy(nav_state_t *out);

/* Diagnostics for the HUD menu. */
uint32_t nav_server_snapshots(void);
uint32_t nav_server_positions(void);
uint32_t nav_server_crc_errors(void);
uint32_t nav_server_bad_lengths(void);
uint32_t nav_server_group_count(uint8_t group);
uint32_t nav_server_unknown_tlvs(void);

#ifdef __cplusplus
}
#endif
#endif
