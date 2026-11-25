#pragma once
// ================================================================
// diag.h - restart forensics
//
// Every boot answers "why did the meter (re)start?":
//   reset_reason   from the chip: power_on (power cut - this board has no
//                  battery), brownout, panic, task_wdt, int_wdt, software ...
//   restart_cause  when the FIRMWARE restarted itself (reset_reason software):
//                  net_loss, ota_apply, cmd_reboot ... Written to RTC memory
//                  just before the restart (survives a software reset, never
//                  wears flash) by diag_restart(), the only restart path.
//   boot_count, fast_boots and the per-reason counters live in NVS, written
//   once per boot. fast_boots counts boots that ended before STABLE_UPTIME_MS:
//   SAFE_MODE_FAST_BOOTS in a row = boot loop -> safe mode.
// ================================================================
#include <stdint.h>
#include "diag_core.h"

struct DiagBootInfo {
    uint8_t  reset_reason;      // ResetReason of this boot
    uint8_t  prev_cause;        // RestartCause (RC_NONE unless reset_reason == RR_SOFTWARE)
    uint32_t prev_uptime_s;     // uptime of the previous boot, if it restarted itself
    uint32_t prev_offline_s;    // how long it had been offline then
    uint8_t  net_loss_streak;   // NET_LOSS restarts in a row without a connection
    uint32_t boot_count;
    uint8_t  fast_boots;
    bool     safe_mode;
    uint32_t power_ons;
    uint32_t brownouts;
    uint32_t crashes;           // panic + all watchdogs
    uint32_t net_restarts;
};

// First thing in setup(): reads the reset reason and the RTC record, updates the NVS counters.
void diag_boot();
const DiagBootInfo& diag_info();

// Periodically (any task): clears fast_boots once the boot is stable.
void diag_tick();

// After an MQTT connection: the NET_LOSS streak is over.
void diag_note_connected();
uint8_t diag_net_loss_streak();

// The only way the firmware restarts itself. Flushes the flash queue first.
[[noreturn]] void diag_restart(RestartCause cause, uint32_t offlineS = 0);

uint32_t diag_uptime_s();
