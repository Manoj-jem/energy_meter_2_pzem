#include "diag_core.h"

#include <string.h>

ResetReason diag_map_reset_reason(int espReason) {
    switch (espReason) {
        case ESPV_POWERON:   return RR_POWERON;
        case ESPV_EXT:       return RR_EXTERNAL;
        case ESPV_SW:        return RR_SOFTWARE;
        case ESPV_PANIC:     return RR_PANIC;
        case ESPV_INT_WDT:   return RR_INT_WDT;
        case ESPV_TASK_WDT:  return RR_TASK_WDT;
        case ESPV_WDT:       return RR_OTHER_WDT;
        case ESPV_DEEPSLEEP: return RR_DEEPSLEEP;
        case ESPV_BROWNOUT:  return RR_BROWNOUT;
        case ESPV_SDIO:      return RR_SDIO;
        default:             return RR_UNKNOWN;
    }
}

const char* reset_reason_name(uint8_t reason) {
    static const char* const names[] = {
        "unknown", "power_on", "brownout", "panic", "int_wdt", "task_wdt",
        "other_wdt", "software", "external", "deepsleep", "sdio",
    };
    return reason < sizeof(names) / sizeof(names[0]) ? names[reason] : "unknown";
}

const char* restart_cause_name(uint8_t cause) {
    static const char* const names[] = {
        "none", "net_loss", "ota_apply", "ota_trial_fail", "ota_rollback", "cmd_reboot", "queue_alloc",
    };
    return cause < sizeof(names) / sizeof(names[0]) ? names[cause] : "unknown";
}

bool reset_is_crash(uint8_t reason) {
    return reason == RR_PANIC || reason == RR_INT_WDT || reason == RR_TASK_WDT || reason == RR_OTHER_WDT;
}

bool reset_is_power_loss(uint8_t reason) { return reason == RR_POWERON || reason == RR_BROWNOUT; }

LadderAction ladder_next(uint32_t offlineMs, uint8_t& step, const LadderConfig& config) {
    // Highest due step first: after a long blocking attempt the ladder may be
    // late, and a modem reset makes a pending radio cycle pointless.
    if (step < 3 && offlineMs >= config.l3Ms) { step = 3; return LADDER_RESTART; }
    if (step < 2 && offlineMs >= config.l2Ms) { step = 2; return LADDER_MODEM_RESET; }
    if (step < 1 && offlineMs >= config.l1Ms) { step = 1; return LADDER_RADIO_CYCLE; }
    return LADDER_NONE;
}

void rate_init(RateLimiter& limiter) { memset(&limiter, 0, sizeof(limiter)); }

bool rate_allow(RateLimiter& limiter, uint8_t code, uint32_t nowMs, uint8_t maxPerHour) {
    constexpr uint32_t HOUR_MS = 3600UL * 1000UL;
    const size_t i = code % RATE_CODES;
    if (limiter.count[i] == 0 || nowMs - limiter.windowStartMs[i] >= HOUR_MS) {
        limiter.windowStartMs[i] = nowMs;
        limiter.count[i] = 0;
    }
    if (limiter.count[i] >= maxPerHour) {
        if (limiter.suppressed < UINT16_MAX) ++limiter.suppressed;
        return false;
    }
    ++limiter.count[i];
    return true;
}

OutageEstimate outage_estimate(int64_t lastAliveEpochMs, int64_t nowEpochMs, uint32_t uptimeMs) {
    OutageEstimate result{};
    if (nowEpochMs <= 0) return result;
    result.endMs = nowEpochMs - (int64_t)uptimeMs;
    if (lastAliveEpochMs <= 0 || result.endMs <= lastAliveEpochMs) return result;
    result.startMs = lastAliveEpochMs;
    const int64_t seconds = (result.endMs - result.startMs) / 1000;
    result.durationS = seconds > (int64_t)UINT32_MAX ? UINT32_MAX : (uint32_t)seconds;
    result.known = true;
    return result;
}

int64_t backdate_epoch_ms(int64_t nowEpochMs, uint32_t nowUptimeS, uint32_t recordUptimeS) {
    if (nowEpochMs <= 0 || recordUptimeS > nowUptimeS) return 0;
    return nowEpochMs - (int64_t)(nowUptimeS - recordUptimeS) * 1000;
}
