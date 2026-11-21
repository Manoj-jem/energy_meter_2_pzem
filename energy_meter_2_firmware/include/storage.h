#pragma once
// ================================================================
// storage.h - external SPI-NOR queue + alive journal
//
// Layout (4 KB sectors):
//   0, 1   header journal (queue head/tail/count checkpoints)
//   2, 3   alive journal: one 64-byte entry per ALIVE_INTERVAL_MS. The newest
//          entry of the previous boot says when power was last on.
//   4 ...  record ring, one TelemetryRecord per 256-byte page
//
// When MQTT is connected, records go straight to the broker; this queue only
// holds what could not be sent (offline, publish failed) and write-ahead
// events. A write-ahead record that was then also sent live is marked
// delivered (storage_mark_delivered), so it is not replayed a second time.
// ================================================================
#include <Arduino.h>
#include "telemetry.h"

struct StorageStats {
    bool ready;
    uint16_t depth;
    uint16_t capacity;
    uint32_t dropped_records;
};

// Why storage is (not) available, for the BOOT / STORAGE_UNAVAILABLE events and {"cmd":"diag"}.
struct StorageHealth {
    bool     ready;
    uint8_t  jedec[3];
    uint8_t  init_attempts;
    uint32_t chip_bytes;
    uint32_t corrupt_slots;   // found at boot: a few = writes cut by power loss
    char     fail_step[24];   // "" when ready
};

// Where a record was stored, to mark it delivered later.
struct StorageRef {
    uint32_t slot;
    uint64_t order;           // 0 = not stored
};

struct AliveEntry {
    uint32_t magic;
    uint32_t sequence;        // increases across boots
    uint32_t boot_count;
    uint32_t uptime_s;
    int64_t  epoch_ms;        // 0 if the clock was not set yet
    float    voltage[3];
    uint8_t  time_quality;
    uint8_t  reserved[19];
    uint32_t crc32;
};
static_assert(sizeof(AliveEntry) == 64, "alive journal entries are 64 bytes");

bool storage_init();
bool storage_append(const TelemetryRecord& record, bool forceCheckpoint = false, StorageRef* ref = nullptr);
bool storage_mark_delivered(const StorageRef& ref);
bool storage_peek(TelemetryRecord& record);
bool storage_pop();
bool storage_has_pending();
void storage_flush();
StorageStats storage_stats();
StorageHealth storage_health();

// Alive journal. storage_last_alive() is the newest entry found at boot,
// i.e. written by the PREVIOUS boot (false if none).
bool storage_alive_write(AliveEntry entry);
bool storage_last_alive(AliveEntry& entry);

// Diagnostics. storage_selftest(true) is destructive-but-restoring; only the
// flashtest env calls it.
void storage_print_status();
bool storage_selftest(bool destructive);
