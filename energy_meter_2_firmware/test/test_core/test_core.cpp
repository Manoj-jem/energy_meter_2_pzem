// Host unit tests for the hardware-independent diagnostics core.
//   pio test -e native
#include <string.h>
#include <unity.h>

#include "diag_core.h"

void setUp() {}
void tearDown() {}

// ── reset reasons ────────────────────────────────────────────────

void test_reset_reason_mapping() {
    TEST_ASSERT_EQUAL(RR_POWERON, diag_map_reset_reason(ESPV_POWERON));
    TEST_ASSERT_EQUAL(RR_BROWNOUT, diag_map_reset_reason(ESPV_BROWNOUT));
    TEST_ASSERT_EQUAL(RR_PANIC, diag_map_reset_reason(ESPV_PANIC));
    TEST_ASSERT_EQUAL(RR_INT_WDT, diag_map_reset_reason(ESPV_INT_WDT));
    TEST_ASSERT_EQUAL(RR_TASK_WDT, diag_map_reset_reason(ESPV_TASK_WDT));
    TEST_ASSERT_EQUAL(RR_OTHER_WDT, diag_map_reset_reason(ESPV_WDT));
    TEST_ASSERT_EQUAL(RR_SOFTWARE, diag_map_reset_reason(ESPV_SW));
    TEST_ASSERT_EQUAL(RR_EXTERNAL, diag_map_reset_reason(ESPV_EXT));
    TEST_ASSERT_EQUAL(RR_UNKNOWN, diag_map_reset_reason(ESPV_UNKNOWN));
    TEST_ASSERT_EQUAL(RR_UNKNOWN, diag_map_reset_reason(99));
}

void test_reset_reason_classes() {
    TEST_ASSERT_TRUE(reset_is_power_loss(RR_POWERON));
    TEST_ASSERT_TRUE(reset_is_power_loss(RR_BROWNOUT));
    TEST_ASSERT_FALSE(reset_is_power_loss(RR_SOFTWARE));
    TEST_ASSERT_TRUE(reset_is_crash(RR_PANIC));
    TEST_ASSERT_TRUE(reset_is_crash(RR_TASK_WDT));
    TEST_ASSERT_FALSE(reset_is_crash(RR_POWERON));
    TEST_ASSERT_FALSE(reset_is_crash(RR_SOFTWARE));
}

void test_names_are_stable() {
    // The backend matches these strings: changing one is a schema change.
    TEST_ASSERT_EQUAL_STRING("power_on", reset_reason_name(RR_POWERON));
    TEST_ASSERT_EQUAL_STRING("brownout", reset_reason_name(RR_BROWNOUT));
    TEST_ASSERT_EQUAL_STRING("task_wdt", reset_reason_name(RR_TASK_WDT));
    TEST_ASSERT_EQUAL_STRING("software", reset_reason_name(RR_SOFTWARE));
    TEST_ASSERT_EQUAL_STRING("unknown", reset_reason_name(200));
    TEST_ASSERT_EQUAL_STRING("net_loss", restart_cause_name(RC_NET_LOSS));
    TEST_ASSERT_EQUAL_STRING("ota_apply", restart_cause_name(RC_OTA_APPLY));
    TEST_ASSERT_EQUAL_STRING("cmd_reboot", restart_cause_name(RC_CMD_REBOOT));
    TEST_ASSERT_EQUAL_STRING("unknown", restart_cause_name(200));
}

// ── recovery ladder ──────────────────────────────────────────────

const LadderConfig CFG{10 * 60000UL, 20 * 60000UL, 30 * 60000UL};

void test_ladder_steps_in_order_once_each() {
    uint8_t step = 0;
    TEST_ASSERT_EQUAL(LADDER_NONE, ladder_next(0, step, CFG));
    TEST_ASSERT_EQUAL(LADDER_NONE, ladder_next(9 * 60000UL, step, CFG));
    TEST_ASSERT_EQUAL(LADDER_RADIO_CYCLE, ladder_next(10 * 60000UL, step, CFG));
    TEST_ASSERT_EQUAL(1, step);
    TEST_ASSERT_EQUAL(LADDER_NONE, ladder_next(15 * 60000UL, step, CFG));   // not again
    TEST_ASSERT_EQUAL(LADDER_MODEM_RESET, ladder_next(20 * 60000UL, step, CFG));
    TEST_ASSERT_EQUAL(LADDER_NONE, ladder_next(25 * 60000UL, step, CFG));
    TEST_ASSERT_EQUAL(LADDER_RESTART, ladder_next(30 * 60000UL, step, CFG));
    TEST_ASSERT_EQUAL(3, step);
    TEST_ASSERT_EQUAL(LADDER_NONE, ladder_next(90 * 60000UL, step, CFG));
}

void test_ladder_late_check_takes_highest_due_step() {
    // A long blocking attach can make the ladder look late: go straight to the step that is due.
    uint8_t step = 0;
    TEST_ASSERT_EQUAL(LADDER_MODEM_RESET, ladder_next(21 * 60000UL, step, CFG));
    TEST_ASSERT_EQUAL(2, step);
    step = 0;
    TEST_ASSERT_EQUAL(LADDER_RESTART, ladder_next(31 * 60000UL, step, CFG));
}

void test_ladder_reset_after_reconnect() {
    uint8_t step = 0;
    ladder_next(11 * 60000UL, step, CFG);
    step = 0;   // what the network task does after a successful publish
    TEST_ASSERT_EQUAL(LADDER_NONE, ladder_next(60000UL, step, CFG));
    TEST_ASSERT_EQUAL(LADDER_RADIO_CYCLE, ladder_next(10 * 60000UL, step, CFG));
}

void test_ladder_repeat_restart_waits_longer() {
    const LadderConfig repeat{CFG.l1Ms, CFG.l2Ms, 60 * 60000UL};
    uint8_t step = 2;
    TEST_ASSERT_EQUAL(LADDER_NONE, ladder_next(45 * 60000UL, step, repeat));
    TEST_ASSERT_EQUAL(LADDER_RESTART, ladder_next(60 * 60000UL, step, repeat));
}

// ── rate limiter ─────────────────────────────────────────────────

void test_rate_limiter_caps_per_hour() {
    RateLimiter r;
    rate_init(r);
    for (int i = 0; i < 6; ++i) TEST_ASSERT_TRUE(rate_allow(r, 4, 1000 + i, 6));
    TEST_ASSERT_FALSE(rate_allow(r, 4, 2000, 6));
    TEST_ASSERT_FALSE(rate_allow(r, 4, 3000, 6));
    TEST_ASSERT_EQUAL(2, r.suppressed);
    TEST_ASSERT_TRUE(rate_allow(r, 5, 3000, 6));            // other codes unaffected
    TEST_ASSERT_TRUE(rate_allow(r, 4, 1000 + 3600000UL, 6)); // next hour
}

void test_rate_limiter_survives_millis_wrap() {
    RateLimiter r;
    rate_init(r);
    const uint32_t nearWrap = 0xFFFFFFFFUL - 1000;
    for (int i = 0; i < 6; ++i) TEST_ASSERT_TRUE(rate_allow(r, 1, nearWrap, 6));
    TEST_ASSERT_FALSE(rate_allow(r, 1, nearWrap + 2000, 6));   // wrapped, still inside the hour
    TEST_ASSERT_TRUE(rate_allow(r, 1, nearWrap + 3600000UL, 6));
}

// ── power outage ─────────────────────────────────────────────────

void test_outage_known() {
    const int64_t lastAlive = 1763700000000LL;   // power still on
    const int64_t bootAt = lastAlive + 300000;   // back 5 min later
    const uint32_t uptimeMs = 90000;             // clock synced 90 s after boot
    const OutageEstimate e = outage_estimate(lastAlive, bootAt + uptimeMs, uptimeMs);
    TEST_ASSERT_TRUE(e.known);
    TEST_ASSERT_EQUAL_INT64(lastAlive, e.startMs);
    TEST_ASSERT_EQUAL_INT64(bootAt, e.endMs);
    TEST_ASSERT_EQUAL_UINT32(300, e.durationS);
}

void test_outage_unknown_start() {
    // The previous boot never had a clock: the end is still known, the duration is not.
    const OutageEstimate e = outage_estimate(0, 1763700000000LL, 5000);
    TEST_ASSERT_FALSE(e.known);
    TEST_ASSERT_EQUAL_INT64(1763700000000LL - 5000, e.endMs);
    TEST_ASSERT_EQUAL_UINT32(0, e.durationS);
}

void test_outage_rejects_clock_going_backwards() {
    const OutageEstimate e = outage_estimate(1763700000000LL, 1763600000000LL, 1000);
    TEST_ASSERT_FALSE(e.known);
}

void test_backdate() {
    TEST_ASSERT_EQUAL_INT64(1763700000000LL - 120000, backdate_epoch_ms(1763700000000LL, 200, 80));
    TEST_ASSERT_EQUAL_INT64(0, backdate_epoch_ms(0, 200, 80));
    TEST_ASSERT_EQUAL_INT64(0, backdate_epoch_ms(1763700000000LL, 80, 200));   // record from the future
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_reset_reason_mapping);
    RUN_TEST(test_reset_reason_classes);
    RUN_TEST(test_names_are_stable);
    RUN_TEST(test_ladder_steps_in_order_once_each);
    RUN_TEST(test_ladder_late_check_takes_highest_due_step);
    RUN_TEST(test_ladder_reset_after_reconnect);
    RUN_TEST(test_ladder_repeat_restart_waits_longer);
    RUN_TEST(test_rate_limiter_caps_per_hour);
    RUN_TEST(test_rate_limiter_survives_millis_wrap);
    RUN_TEST(test_outage_known);
    RUN_TEST(test_outage_unknown_start);
    RUN_TEST(test_outage_rejects_clock_going_backwards);
    RUN_TEST(test_backdate);
    return UNITY_END();
}
