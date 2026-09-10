#pragma once

#include <Arduino.h>
#include "telemetry.h"

void time_service_init();
bool time_service_sync_from_modem();
int64_t time_service_now_ms();
TimeQuality time_service_quality();
void time_service_format_iso8601(int64_t epochMs, char* out, size_t outLen);
