#include "uwb.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "uwb";

void uwb_init() {
    // TODO: initialize 3 UARTs for UWB anchors
}

bool uwb_read(UWBReading &reading) {
    // TODO: poll each anchor and populate reading
    return false;
}
