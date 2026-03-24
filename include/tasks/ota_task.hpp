#pragma once

#include "Arduino.h"

namespace Task {
    namespace OTATask {
        extern volatile bool ota_in_progress;

        void run(void *pvParameters);
    }
}
