#include "diag.h"

#include <Arduino.h>
#include <Preferences.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_timer.h>

#include "config.h"
#include "storage.h"

namespace {

constexpr uint32_t RTC_MAGIC = 0x4D525354UL;   // "MRST"
constexpr const char* NS = "diag";

// Survives software, watchdog and panic resets; random after power-on.
struct RtcRestartRecord {
    uint32_t magic;
    uint32_t uptimeS;
    uint32_t offlineS;
    uint8_t  cause;
    uint8_t  netLossStreak;
    uint16_t reserved;
    uint32_t check;
};
RTC_NOINIT_ATTR RtcRestartRecord rtcRecord;

uint32_t checkOf(const RtcRestartRecord& r) {
    return r.magic ^ r.uptimeS ^ (r.offlineS * 31U) ^ ((uint32_t)r.cause << 8) ^ ((uint32_t)r.netLossStreak << 16) ^ 0xA5A55A5AUL;
}

DiagBootInfo info{};
volatile uint8_t netLossStreak = 0;
bool stableMarked = false;

}  // namespace

void diag_boot() {
    info.reset_reason = diag_map_reset_reason((int)esp_reset_reason());

    const bool rtcValid = rtcRecord.magic == RTC_MAGIC && rtcRecord.check == checkOf(rtcRecord);
    if (rtcValid && info.reset_reason == RR_SOFTWARE) {
        info.prev_cause = rtcRecord.cause;
        info.prev_uptime_s = rtcRecord.uptimeS;
        info.prev_offline_s = rtcRecord.offlineS;
        info.net_loss_streak = rtcRecord.cause == RC_NET_LOSS ? rtcRecord.netLossStreak : 0;
    }
    rtcRecord.magic = 0;   // consumed: a later reset of another kind must not reuse it
    netLossStreak = info.net_loss_streak;

    Preferences prefs;
    if (prefs.begin(NS, false)) {
        info.boot_count = prefs.getUInt("boots", 0) + 1;
        // A power cut is not a boot loop: a site whose mains flickers would
        // otherwise end up in safe mode.
        info.fast_boots = info.reset_reason == RR_POWERON
                              ? 1
                              : (uint8_t)min<uint32_t>(prefs.getUChar("fast", 0) + 1U, 255U);
        info.power_ons = prefs.getUInt("pwr", 0) + (info.reset_reason == RR_POWERON ? 1 : 0);
        info.brownouts = prefs.getUInt("brown", 0) + (info.reset_reason == RR_BROWNOUT ? 1 : 0);
        info.crashes = prefs.getUInt("crash", 0) + (reset_is_crash(info.reset_reason) ? 1 : 0);
        info.net_restarts = prefs.getUInt("netr", 0) + (info.prev_cause == RC_NET_LOSS ? 1 : 0);
        prefs.putUInt("boots", info.boot_count);
        prefs.putUChar("fast", info.fast_boots);
        prefs.putUInt("pwr", info.power_ons);
        prefs.putUInt("brown", info.brownouts);
        prefs.putUInt("crash", info.crashes);
        prefs.putUInt("netr", info.net_restarts);
        prefs.end();
    }
    info.safe_mode = info.fast_boots >= SAFE_MODE_FAST_BOOTS;

    DBGF("[DIAG] boot #%lu reset=%s prev_cause=%s prev_uptime=%lus fast_boots=%u%s\n",
         (unsigned long)info.boot_count, reset_reason_name(info.reset_reason),
         restart_cause_name(info.prev_cause), (unsigned long)info.prev_uptime_s,
         (unsigned)info.fast_boots, info.safe_mode ? " SAFE MODE" : "");
}

const DiagBootInfo& diag_info() { return info; }

void diag_tick() {
    if (stableMarked || millis() < STABLE_UPTIME_MS) return;
    stableMarked = true;
    Preferences prefs;
    if (prefs.begin(NS, false)) {
        prefs.putUChar("fast", 0);
        prefs.end();
    }
}

void diag_note_connected() { netLossStreak = 0; }
uint8_t diag_net_loss_streak() { return netLossStreak; }

uint32_t diag_uptime_s() { return (uint32_t)(esp_timer_get_time() / 1000000LL); }

void diag_restart(RestartCause cause, uint32_t offlineS) {
    RtcRestartRecord r{};
    r.magic = RTC_MAGIC;
    r.uptimeS = diag_uptime_s();
    r.offlineS = offlineS;
    r.cause = cause;
    r.netLossStreak = cause == RC_NET_LOSS ? (uint8_t)min<uint32_t>(netLossStreak + 1U, 255U) : 0;
    r.check = checkOf(r);
    rtcRecord = r;
    DBGF("[DIAG] restarting: %s (uptime %lus, offline %lus)\n", restart_cause_name(cause),
         (unsigned long)r.uptimeS, (unsigned long)offlineS);
    storage_flush();
    delay(200);
    esp_restart();
    for (;;) {}
}
