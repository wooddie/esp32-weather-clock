#pragma once

struct WiFiCred {
    const char* ssid;
    const char* pass;
};

#define WIFI_NETWORKS_COUNT 3

//you wifi list (essid and pass)
const WiFiCred WIFI_NETWORKS[WIFI_NETWORKS_COUNT] = {
    {"MyHomeWiFi",   "real_pass_1"},
    {"OfficeWiFi",   "real_pass_2"},
    {"iPhoneHotspot","real_pass_3"}
};