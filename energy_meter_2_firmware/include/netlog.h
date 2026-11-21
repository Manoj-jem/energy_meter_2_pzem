#pragma once
// ================================================================
// netlog.h - always-on connection log, readable remotely (fw 2.0.1)
//
// The USB debug log is only useful with a laptop on site. This keeps the last
// NETLOG_ENTRIES connection-relevant lines (why a publish failed, why the
// session was dropped, how long reconnects took, recovery steps) in RTC memory,
// independent of DEBUG_ENABLED, and {"cmd":"log"} reads them over MQTT.
//
// RTC memory survives software restarts, watchdog resets and crashes, so the
// lines leading up to a self-restart (net_loss) or a crash can still be read
// after it. A power cut clears them (the alive journal covers that case).
//
// Thread-safe; cheap enough to call from any task. Lines are cut at
// NETLOG_TEXT_MAX - 1 characters.
// ================================================================
#include <stddef.h>
#include <stdint.h>

constexpr size_t NETLOG_ENTRIES = 40;
constexpr size_t NETLOG_TEXT_MAX = 56;

struct NetLogEntry {
    uint32_t seq;        // increases across restarts until power is lost
    uint32_t uptime_s;   // since this boot
    uint32_t epoch_s;    // UTC, 0 when the clock was not set yet
    uint16_t boot;       // low 16 bits of boot_count, to tell boots apart
    char     text[NETLOG_TEXT_MAX - 2];
};

// Validates the RTC ring after a restart (clears it after a power-on) and adds
// a "boot" line. Call once from setup(), after diag_boot().
void netlog_init(uint32_t bootCount);

void netlog_add(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Oldest first: index 0 is the oldest entry still held. Returns false past the end.
bool netlog_get(size_t index, NetLogEntry& out);
size_t netlog_count();
void netlog_clear();
