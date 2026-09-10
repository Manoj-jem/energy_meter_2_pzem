#include <Preferences.h>

#include "calibration.h"
#include "config.h"

namespace {

float gains[3] = {CT_PHASE1_GAIN, CT_PHASE2_GAIN, CT_PHASE3_GAIN};

bool validGain(float value) {
    return value >= 0.5f && value <= 1.5f;
}

}  // namespace

void calibration_init() {
    Preferences preferences;
    if (!preferences.begin("metercal", true)) return;
    const char* keys[] = {"phase1_gain", "phase2_gain", "phase3_gain"};
    for (uint8_t index = 0; index < 3; ++index) {
        const float value = preferences.getFloat(keys[index], gains[index]);
        if (validGain(value)) gains[index] = value;
    }
    preferences.end();
}

float calibration_phase_gain(uint8_t phase) {
    return phase >= 1 && phase <= 3 ? gains[phase - 1] : 1.0f;
}
