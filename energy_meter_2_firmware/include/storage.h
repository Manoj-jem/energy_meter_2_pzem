#pragma once

#include <Arduino.h>
#include "telemetry.h"

struct StorageStats {
    bool ready;
    uint16_t depth;
    uint16_t capacity;
    uint32_t dropped_records;
};

bool storage_init();
bool storage_append(const TelemetryRecord& record, bool forceCheckpoint = false);
bool storage_peek(TelemetryRecord& record);
bool storage_pop();
bool storage_has_pending();
void storage_flush();
StorageStats storage_stats();
