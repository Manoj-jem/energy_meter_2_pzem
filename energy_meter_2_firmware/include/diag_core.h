#pragma once
// ================================================================
// diag_core.h - hardware-independent diagnostics logic (host-testable)
//
// Pure functions only: no Arduino, no FreeRTOS. diag.cpp and main.cpp feed
// them the clock and the ESP-IDF reset reason; test/test_core exercises them.
// ================================================================
#include <stddef.h>
#include <stdint.h>

// Why THIS boot happened (from esp_reset_reason()). Stored and sent as a
// number; the backend uses reset_reason_name()'s strings. Only append.
enum ResetReason : uint8_t {
    RR_UNKNOWN = 0,
    RR_POWERON,     // power applied: after a power cut (no battery on this board)
    RR_BROWNOUT,    // supply dipped below the brown-out level: weak / failing supply
    RR_PANIC,       // firmware crash (exception, abort)
    RR_INT_WDT,     // interrupt watchdog: firmware hung with interrupts off
    RR_TASK_WDT,    // task watchdog: a task stopped running for TASK_WATCHDOG_TIMEOUT_S
    RR_OTHER_WDT,   // RTC / other watchdog
    RR_SOFTWARE,    // the firmware restarted itself: RestartCause says why
    RR_EXTERNAL,    // EN / reset pin
    RR_DEEPSLEEP,
    RR_SDIO,
};

// Why the firmware restarted ITSELF (previous boot). Only append.
enum RestartCause : uint8_t {
    RC_NONE = 0,       // not a firmware restart, or the reason was not recorded
    RC_NET_LOSS,       // recovery ladder step 3: offline too long
    RC_OTA_APPLY,      // new firmware installed
    RC_OTA_TRIAL_FAIL, // new firmware did not reach AWS in time
    RC_OTA_ROLLBACK,   // went back to the previous firmware
    RC_CMD_REBOOT,     // {"cmd":"reboot"}
    RC_QUEUE_ALLOC,    // out of memory at start-up
};

// ESP-IDF esp_reset_reason_t values (esp_system.h), repeated here so this
// file builds on the host.
enum EspResetValue : int {
    ESPV_UNKNOWN = 0, ESPV_POWERON = 1, ESPV_EXT = 2, ESPV_SW = 3, ESPV_PANIC = 4,
    ESPV_INT_WDT = 5, ESPV_TASK_WDT = 6, ESPV_WDT = 7, ESPV_DEEPSLEEP = 8,
    ESPV_BROWNOUT = 9, ESPV_SDIO = 10,
};

ResetReason diag_map_reset_reason(int espReason);
const char* reset_reason_name(uint8_t reason);
const char* restart_cause_name(uint8_t cause);
bool reset_is_crash(uint8_t reason);        // panic / any watchdog
bool reset_is_power_loss(uint8_t reason);   // power-on / brown-out

// ── Network recovery ladder ──────────────────────────────────────
enum LadderAction : uint8_t {
    LADDER_NONE = 0,
    LADDER_RADIO_CYCLE,   // step 1
    LADDER_MODEM_RESET,   // step 2
    LADDER_RESTART,       // step 3
};

struct LadderConfig {
    uint32_t l1Ms;
    uint32_t l2Ms;
    uint32_t l3Ms;
};

// Next action for `offlineMs` of continuous outage. `step` is the highest step
// already taken in this outage (0 = none); it is advanced when an action is
// returned. Each step fires once per outage; reset step to 0 on reconnect.
LadderAction ladder_next(uint32_t offlineMs, uint8_t& step, const LadderConfig& config);

// ── Event rate limiter ───────────────────────────────────────────
constexpr size_t RATE_CODES = 64;
struct RateLimiter {
    uint32_t windowStartMs[RATE_CODES];
    uint8_t  count[RATE_CODES];
    uint16_t suppressed;
};
void rate_init(RateLimiter& limiter);
// True if event `code` may be sent now (max `maxPerHour` per code per rolling
// hour window). False counts the event as suppressed.
bool rate_allow(RateLimiter& limiter, uint8_t code, uint32_t nowMs, uint8_t maxPerHour);

// ── Power-outage estimate ────────────────────────────────────────
struct OutageEstimate {
    bool     known;        // both ends have a real time
    int64_t  startMs;      // last alive entry (power was still on), epoch ms
    int64_t  endMs;        // this boot (power back), epoch ms
    uint32_t durationS;
};
// lastAliveEpochMs: time of the last alive-journal entry of the previous boot
// (0 if its clock was never set). nowEpochMs / uptimeMs: now, after a time sync.
OutageEstimate outage_estimate(int64_t lastAliveEpochMs, int64_t nowEpochMs, uint32_t uptimeMs);

// Epoch ms for a record made earlier in THIS boot without a clock, from its
// uptime: now - (uptime now - uptime then). 0 if it cannot be known.
int64_t backdate_epoch_ms(int64_t nowEpochMs, uint32_t nowUptimeS, uint32_t recordUptimeS);
