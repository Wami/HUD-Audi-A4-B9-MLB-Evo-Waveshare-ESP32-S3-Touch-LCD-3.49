/*
  hud_diag.cpp — см. hud_diag.h
*/
#include <Arduino.h>
#include <Preferences.h>
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "hud_diag.h"
#include "hud_source.h"
#include "hud_mockup.h"
#include "ble_nav_server.h"

extern "C" {
hud_diag_t g_diag;
}

extern "C" void hud_diag_save(void)
{
    g_diag.ver = HUD_DIAG_VER;
    g_diag.uptime_s = millis() / 1000;
    g_diag.heap_free  = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    g_diag.psram_free = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    g_diag.reset_reason = (uint8_t)esp_reset_reason();
    g_diag.src = (int8_t)hud_src_current();
    g_diag.cams_on = hud_get_cams();
    g_diag.cam_beep = hud_get_cam_beep();
    g_diag.cam_src = hud_get_cam_src();
    g_diag.nav_connected = nav_server_connected() ? 1 : 0;
    g_diag.nav_snapshots = nav_server_snapshots();
    g_diag.nav_positions = nav_server_positions();
    g_diag.nav_crc = nav_server_crc_errors();
    g_diag.nav_unknown = nav_server_unknown_tlvs();
    g_diag.nav_bad_len = nav_server_bad_lengths();
    for (int i = 0; i < 13; i++) g_diag.nav_groups[i] = nav_server_group_count((uint8_t)i);
    Preferences p;
    if (p.begin("hudd", false)) { p.putBytes("d", &g_diag, sizeof g_diag); p.end(); }
}

extern "C" bool hud_diag_load(hud_diag_t *out)
{
    Preferences p;
    if (!p.begin("hudd", true)) return false;
    bool ok = p.getBytesLength("d") == sizeof(hud_diag_t) && p.getBytes("d", out, sizeof(hud_diag_t)) == sizeof(hud_diag_t);
    p.end();
    return ok && out->ver == HUD_DIAG_VER;
}
