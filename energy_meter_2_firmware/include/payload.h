#pragma once
// ================================================================
// payload.h 
// ================================================================
#include <Arduino.h>
#include "telemetry.h"


size_t payload_build(const TelemetryRecord& record,
                     DeliveryMode          delivery,
                     char*             outBuf,
                     size_t            outBufLen);
