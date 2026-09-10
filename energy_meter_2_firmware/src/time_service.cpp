#include <Arduino.h>
#include <sys/time.h>
#include <time.h>
#include <esp_timer.h>

#include "at_mqtt.h"
#include "time_service.h"

namespace {

bool clockSet = false;
uint32_t lastSyncMs = 0;
constexpr uint32_t SYNCED_WINDOW_MS = 6UL * 60UL * 60UL * 1000UL;

bool parseIso8601(const char* input, time_t& epoch) {
    int year, month, day, hour, minute, second, offsetHour, offsetMinute;
    char sign = '+';
    if (sscanf(input, "%4d-%2d-%2dT%2d:%2d:%2d%c%2d:%2d",
               &year, &month, &day, &hour, &minute, &second, &sign, &offsetHour, &offsetMinute) != 9) {
        return false;
    }
    tm value{};
    value.tm_year = year - 1900;
    value.tm_mon = month - 1;
    value.tm_mday = day;
    value.tm_hour = hour;
    value.tm_min = minute;
    value.tm_sec = second;
    // Convert the calendar value to UTC without relying on non-standard
    // timegm(), which is not declared by all ESP32 toolchains.
    const int yearFromMarch = value.tm_year + 1900 - (value.tm_mon < 2 ? 1 : 0);
    const int era = (yearFromMarch >= 0 ? yearFromMarch : yearFromMarch - 399) / 400;
    const unsigned yearOfEra = static_cast<unsigned>(yearFromMarch - era * 400);
    const unsigned monthFromMarch = static_cast<unsigned>(value.tm_mon + (value.tm_mon < 2 ? 10 : -2));
    const unsigned dayOfYear = (153 * monthFromMarch + 2) / 5 + value.tm_mday - 1;
    const unsigned dayOfEra = yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
    const int64_t daysSinceEpoch = static_cast<int64_t>(era) * 146097 + dayOfEra - 719468;
    const time_t localEpoch = static_cast<time_t>(daysSinceEpoch * 86400LL
                                                   + value.tm_hour * 3600
                                                   + value.tm_min * 60
                                                   + value.tm_sec);
    const int offset = (offsetHour * 60 + offsetMinute) * 60;
    epoch = sign == '-' ? localEpoch + offset : localEpoch - offset;
    return epoch >= 1577836800;  // 2020-01-01 UTC.
}

}  // namespace

void time_service_init() {
    clockSet = time(nullptr) >= 1577836800;
    lastSyncMs = clockSet ? millis() : 0;
}

bool time_service_sync_from_modem() {
    char timestamp[32];
    at_mqtt_get_timestamp(timestamp, sizeof(timestamp));
    time_t epoch = 0;
    if (!parseIso8601(timestamp, epoch)) return false;
    timeval now{};
    now.tv_sec = epoch;
    now.tv_usec = 0;
    if (settimeofday(&now, nullptr) != 0) return false;
    clockSet = true;
    lastSyncMs = millis();
    return true;
}

int64_t time_service_now_ms() {
    timeval now{};
    gettimeofday(&now, nullptr);
    if (now.tv_sec < 1577836800) return 0;
    return static_cast<int64_t>(now.tv_sec) * 1000LL + now.tv_usec / 1000LL;
}

TimeQuality time_service_quality() {
    if (!clockSet) return TIME_UNKNOWN;
    return (millis() - lastSyncMs) <= SYNCED_WINDOW_MS ? TIME_SYNCED : TIME_ESTIMATED;
}

void time_service_format_iso8601(int64_t epochMs, char* out, size_t outLen) {
    if (epochMs <= 0) {
        strncpy(out, "1970-01-01T00:00:00Z", outLen);
        out[outLen - 1] = '\0';
        return;
    }
    const time_t seconds = static_cast<time_t>(epochMs / 1000LL);
    tm value{};
    gmtime_r(&seconds, &value);
    strftime(out, outLen, "%Y-%m-%dT%H:%M:%SZ", &value);
}
