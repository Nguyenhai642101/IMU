/**
 * @file    test_ps_example.c
 * @brief   Host C99 unit tests for the PS example application (TC-APP-01..03).
 * @note    The example's helpers are file-static, so this test includes the
 *          example source directly (its main() is renamed by the Makefile
 *          with -Dmain=app_example_main) and runs against the Mock HAL.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "mock_sm5800_hal.h"

int app_example_main(void);

#include "../driver/sm5800_ps_example.c"

/* Restore the name 'main' for this test runner's own entry point */
#undef main

static uint32_t g_app_pass_count = 0U;
static uint32_t g_app_fail_count = 0U;


static void app_check(bool is_ok, char const * p_label)
{
    if (is_ok)
    {
        g_app_pass_count++;
    }
    else
    {
        g_app_fail_count++;
        (void)printf("[FAIL] %s\n", p_label);
    }
}


static sm5800_msg2_eng_t make_nominal_eng(void)
{
    sm5800_msg2_eng_t eng = { .timestamp_us = 0U };

    eng.rate_x_rad_s = 1.0f;
    eng.rate_y_rad_s = -1.0f;
    eng.rate_z_rad_s = 0.5f;
    eng.accel_z_m_s2 = 9.81f;
    eng.timestamp_us = 1000U;

    return eng;
}


static void test_self_check(void)
{
    /* TC-APP-01: example's 1 g / 1 rad/s / 1 mrad self-check passes on host */
    app_check(SM5800_OK == verify_raw_to_eng_math(),
              "TC-APP-01: verify_raw_to_eng_math() == OK");
}


static void test_rate_plausibility(void)
{
    sm5800_msg2_eng_t eng = make_nominal_eng();

    /* TC-APP-02a: nominal sample accepted and stored */
    g_rate_reject_count = 0U;
    app_check(SM5800_OK == handle_msg2_and_faults(&eng),
              "TC-APP-02a: nominal sample accepted");
    app_check((1000U == g_last_eng.timestamp_us)
              && (0U == g_rate_reject_count),
              "TC-APP-02a: nominal sample stored, no rejection");

    /* TC-APP-02b: just inside the limit on every axis is still accepted */
    eng = make_nominal_eng();
    eng.rate_x_rad_s = 18.4f;
    eng.rate_y_rad_s = -18.4f;
    eng.rate_z_rad_s = 18.4f;
    eng.timestamp_us = 2000U;
    app_check(SM5800_OK == handle_msg2_and_faults(&eng),
              "TC-APP-02b: 18.4 rad/s on all axes accepted");

    /* TC-APP-02c: saturated Z axis (+full scale) rejected, last good kept */
    eng = make_nominal_eng();
    eng.rate_z_rad_s = 18.75f;
    eng.timestamp_us = 3000U;
    app_check(SM5800_ERR_NO_SAMPLE == handle_msg2_and_faults(&eng),
              "TC-APP-02c: saturated +Z rejected");
    app_check((2000U == g_last_eng.timestamp_us)
              && (1U == g_rate_reject_count),
              "TC-APP-02c: rejected sample did not overwrite last good");

    /* TC-APP-02d: saturated Y axis (-full scale) rejected */
    eng = make_nominal_eng();
    eng.rate_y_rad_s = -18.75f;
    eng.timestamp_us = 4000U;
    app_check(SM5800_ERR_NO_SAMPLE == handle_msg2_and_faults(&eng),
              "TC-APP-02d: saturated -Y rejected");

    /* TC-APP-02e: saturated X axis (-full scale) rejected */
    eng = make_nominal_eng();
    eng.rate_x_rad_s = -18.75f;
    eng.timestamp_us = 5000U;
    app_check(SM5800_ERR_NO_SAMPLE == handle_msg2_and_faults(&eng),
              "TC-APP-02e: saturated -X rejected");
    app_check((2000U == g_last_eng.timestamp_us)
              && (3U == g_rate_reject_count),
              "TC-APP-02e: three rejections counted, last good kept");
}


static void test_irq_path_polling(void)
{
    sm5800_msg2_eng_t eng = { .timestamp_us = 0U };

    /* TC-APP-03: without the Xilinx BSP the example must still deliver
     * Msg 02 samples by polling INT_STATUS from the main loop */
    mock_hal_reset();
    g_irq_polling = false;
    app_check(SM5800_OK == init_imu_100hz(),
              "TC-APP-03: init_imu_100hz() == OK (driver + IRQ path)");
    app_check(g_irq_polling,
              "TC-APP-03: no GIC driver -> polling mode selected");

    service_irq_polling();
    app_check(!sm5800_is_msg2_pending(&g_imu_drv),
              "TC-APP-03: no sample pending while INT_STATUS == 0");

    mock_hal_set_reg(SM5800_REG_LIN_ACC_Z, 879U);
    mock_hal_set_reg(SM5800_REG_STATUS_WORD1, 0x1901U);
    mock_hal_set_reg(SM5800_REG_INT_STATUS, SM5800_INT_MSG2_RDY_MASK);
    service_irq_polling();
    app_check(sm5800_is_msg2_pending(&g_imu_drv),
              "TC-APP-03: polled MSG2_READY sets the pending flag");
    app_check(0U == mock_hal_get_reg(SM5800_REG_INT_STATUS),
              "TC-APP-03: polled ISR acknowledged INT_STATUS (W1C)");
    app_check((SM5800_OK == sm5800_read_msg2_eng(&g_imu_drv, &eng))
              && (eng.accel_z_m_s2 > 9.7f) && (eng.accel_z_m_s2 < 9.9f),
              "TC-APP-03: polled sample read and converted (1 g)");
    app_check(SM5800_OK == sm5800_deinit(&g_imu_drv),
              "TC-APP-03: clean deinit");
}


int main(void)
{
    int exit_code = 0;

    test_self_check();
    test_rate_plausibility();
    test_irq_path_polling();

    (void)printf("==================================================\n");
    (void)printf("SM-5800 PS Example Unit Tests: %u passed, %u failed\n",
                 g_app_pass_count,
                 g_app_fail_count);
    (void)printf("==================================================\n");

    if (0U != g_app_fail_count)
    {
        exit_code = 1;
    }
    else
    {
        exit_code = 0;
    }

    return exit_code;
}
