#include <Arduino.h>
#include <WiFi.h>
#include <map>


uint64_t module_id = 0;

void set_module_id() {
    std::map<String, uint64_t> mac_to_id = {
        {"DC:54:75:D8:52:F0", 0}, // Dead
        {"F0:9E:9E:32:29:38", 1}, // Old
        {"F0:9E:9E:32:23:D4", 2}, // Old
        {"F0:9E:9E:32:2E:7C", 3}, // Dead
        {"F0:9E:9E:32:2F:88", 4}, // Dead
        {"F0:9E:9E:31:FD:E0", 5},
        {"F0:9E:9E:31:FE:14", 6},
        {"F0:9E:9E:31:FE:24", 7},
        {"F0:9E:9E:31:FD:FC", 8},
        {"F0:9E:9E:31:FE:20", 9},
        {"F0:9E:9E:31:FE:00", 10}, // UWB
        {"F0:9E:9E:31:FE:10", 11}, // UWB
        {"F0:9E:9E:31:FD:CC", 12},
        {"F0:9E:9E:31:FE:0C", 13}, // UWB
        {"F0:9E:9E:31:FE:1C", 14},
        {"F0:9E:9E:31:FE:34", 15},
        {"F0:9E:9E:31:FE:28", 16},
        {"98:3D:AE:EF:29:30", 17},  // White
        {"98:3D:AE:EF:35:08", 18},  // Motor does not work?
        {"98:3D:AE:EF:3B:C8", 20},
        {"98:3D:AE:EF:29:00", 21},
        {"98:3D:AE:EF:29:48", 22},
        {"98:3D:AE:EF:34:C4", 23},
        {"98:3D:AE:EF:21:C8", 24},
        {"98:3D:AE:EF:28:FC", 25},
        {"98:3D:AE:EF:0C:50", 26},
        {"98:3D:AE:EF:29:18", 27},

        {"F0:9E:9E:2B:F3:50", 28},
        {"F0:9E:9E:2B:F3:5C", 29},
        {"F0:9E:9E:2B:F3:58", 30},
        {"F0:9E:9E:2B:F3:24", 31},
        {"F0:9E:9E:2B:F2:F8", 32},
        {"F0:9E:9E:2B:F2:C4", 33},
        {"F0:9E:9E:2B:F3:04", 34},
        {"F0:9E:9E:2B:F2:DC", 35},
        {"F0:9E:9E:2B:F2:FC", 36},
        {"F0:9E:9E:2B:F2:D0", 37},

        {"EC:DA:3B:41:A2:18", 1000}, //handmade
        {"E0:72:A1:FC:53:98", 1001}, //handmade
        {"E0:72:A1:FC:53:94", 1002}, //handmade
        {"F4:12:FA:88:12:64", 1003}, //handmade
        {"1C:DB:D4:5B:E6:80", 1004}, //handmade
    };
    WiFi.begin();
    String mac = WiFi.macAddress();
    Serial.println("MAC Address: " + mac);
    if (mac_to_id.count(mac) > 0) {
        module_id = mac_to_id[mac];
    } else {
        module_id = 0;
    }
}