#include "netlog.h"

#include <Arduino.h>
#include <esp_attr.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "diag.h"
#include "time_service.h"

namespace {

constexpr uint32_t MAGIC = 0x4E4C4F47UL;   // "NLOG"

struct RtcNetLog {
    uint32_t magic;
    uint32_t head;    // next slot to write
    uint32_t count;
    uint32_t seq;     // last sequence number written
    uint32_t check;
    NetLogEntry entries[NETLOG_ENTRIES];
};

RTC_NOINIT_ATTR RtcNetLog rtcLog;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
uint16_t bootTag = 0;
bool ready = false;

uint32_t checkOf(const RtcNetLog& l) {
    return l.magic ^ (l.head * 2654435761UL) ^ (l.count << 12) ^ (l.seq * 31U) ^ 0x5AA5C33CUL;
}

void resetLocked() {
    memset(&rtcLog, 0, sizeof(rtcLog));
    rtcLog.magic = MAGIC;
    rtcLog.check = checkOf(rtcLog);
}

}  // namespace

void netlog_init(uint32_t bootCount) {
    bootTag = (uint16_t)bootCount;
    portENTER_CRITICAL(&mux);
    const bool valid = rtcLog.magic == MAGIC && rtcLog.head < NETLOG_ENTRIES && rtcLog.count <= NETLOG_ENTRIES &&
                       rtcLog.check == checkOf(rtcLog);
    // After a power-on the RTC RAM holds noise that may even pass the check by luck.
    const esp_reset_reason_t why = esp_reset_reason();
    if (!valid || why == ESP_RST_POWERON) resetLocked();
    for (size_t i = 0; i < NETLOG_ENTRIES; ++i) rtcLog.entries[i].text[sizeof(rtcLog.entries[i].text) - 1] = '\0';
    ready = true;
    portEXIT_CRITICAL(&mux);
    const DiagBootInfo& boot = diag_info();
    netlog_add("boot fw=%s reset=%s cause=%s", FIRMWARE_VERSION, reset_reason_name(boot.reset_reason),
               restart_cause_name(boot.prev_cause));
}

void netlog_add(const char* fmt, ...) {
    if (!ready) return;
    NetLogEntry e{};
    va_list args;
    va_start(args, fmt);
    vsnprintf(e.text, sizeof(e.text), fmt, args);
    va_end(args);
    e.uptime_s = diag_uptime_s();
    const int64_t nowMs = time_service_now_ms();
    e.epoch_s = nowMs > 0 ? (uint32_t)(nowMs / 1000LL) : 0;
    e.boot = bootTag;
    DBGF("[NETLOG] %s\n", e.text);

    portENTER_CRITICAL(&mux);
    e.seq = ++rtcLog.seq;
    rtcLog.entries[rtcLog.head] = e;
    rtcLog.head = (rtcLog.head + 1) % NETLOG_ENTRIES;
    if (rtcLog.count < NETLOG_ENTRIES) ++rtcLog.count;
    rtcLog.check = checkOf(rtcLog);
    portEXIT_CRITICAL(&mux);
}

size_t netlog_count() {
    portENTER_CRITICAL(&mux);
    const size_t n = ready ? rtcLog.count : 0;
    portEXIT_CRITICAL(&mux);
    return n;
}

bool netlog_get(size_t index, NetLogEntry& out) {
    bool ok = false;
    portENTER_CRITICAL(&mux);
    if (ready && index < rtcLog.count) {
        const size_t oldest = (rtcLog.head + NETLOG_ENTRIES - rtcLog.count) % NETLOG_ENTRIES;
        out = rtcLog.entries[(oldest + index) % NETLOG_ENTRIES];
        ok = true;
    }
    portEXIT_CRITICAL(&mux);
    return ok;
}

void netlog_clear() {
    portENTER_CRITICAL(&mux);
    const uint32_t seq = rtcLog.seq;
    resetLocked();
    rtcLog.seq = seq;
    rtcLog.check = checkOf(rtcLog);
    portEXIT_CRITICAL(&mux);
}
