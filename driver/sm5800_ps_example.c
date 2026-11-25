/**
 * @file    sm5800_ps_example.c
 * @brief   PS example: 100 Hz Msg 02 IRQ readout, Reset/BIT recovery & verify.
 * @req     LLR-SM5800-APP-001
 * @design  SDD-SM5800-4.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sm5800_drv.h"
#include "sm5800_hal.h"
#include "sm5800_regs.h"

/* Xilinx standalone BSP (Vitis) present -> real GIC interrupts.
 * Without it (host unit tests, bare cross builds) the main loop polls
 * INT_STATUS instead. Define APP_FORCE_IRQ_POLLING to poll even with the
 * BSP: useful to tell a PL->PS IRQ wiring / IRQ ID problem from an IP or
 * sensor problem (polling works but interrupts do not -> wiring / ID). */
#if defined(__has_include)
#if __has_include("xscugic.h")
#define APP_HAVE_XSCUGIC
#endif
#endif

#if defined(APP_HAVE_XSCUGIC) && !defined(APP_FORCE_IRQ_POLLING)
#define APP_USE_GIC
#include "xparameters.h"
#include "xscugic.h"
#include "xil_exception.h"
#endif

/* GIC interrupt ID (SPI) of the IP's irq_o. Prefer the value from
 * xparameters.h (XPAR_FABRIC_<ip>_IRQ_O_INTR) or Vivado, e.g. build with
 * -DIMU_IRQ_ID=XPAR_FABRIC_SM5800_IMU_AXI_TOP_0_IRQ_O_INTR. Defaults:
 *  - Zynq-7000 IRQ_F2P[0]        = 61  (IRQ_F2P[1..7] = 62..68)
 *  - ZynqMP pl_ps_irq0[0], APU   = 121 (pl_ps_irq0[1..7] = 122..128)
 *  - ZynqMP RPU (Cortex-R5) also uses 121: define IMU_IRQ_ID=121U */
#ifndef IMU_IRQ_ID
#if defined(__aarch64__)
#define IMU_IRQ_ID              (121U)
#else
#define IMU_IRQ_ID              (61U)
#endif
#endif

/* GIC trigger for irq_o: 0x03 = rising edge (default, verified on hardware),
 * 0x01 = active-high level. Both work: the IP drops irq_o for one cycle on
 * every INT_STATUS W1C write, so a flag still pending makes a new edge. */
#define APP_GIC_PRIORITY        (0xA0U)
#ifndef APP_GIC_TRIGGER
#define APP_GIC_TRIGGER         (0x03U)
#endif

/* AXI4-Lite base address of the SM-5800 IP (Vivado Address Editor).
 * Default: Zynq-7000 M_AXI_GP0 (0x43C00000). Zynq UltraScale+ maps the PL
 * at 0xA0000000 (M_AXI_HPM0_FPD) by default: build with
 * -DIMU_AXI_BASE_ADDR=<XPAR_..._BASEADDR from xparameters.h>. */
#ifndef IMU_AXI_BASE_ADDR
#define IMU_AXI_BASE_ADDR       ((uintptr_t)0x43C00000U)
#endif

/* Bounded iteration limits for example acquisition and BIT dwell */
/* Upper bound only: the loop normally ends after APP_TARGET_SAMPLES
 * (1 s at 100 Hz). 1e6 polls took well under 1 s on a GHz core. */
#define APP_MAX_POLL_CYCLES     (400000000U)
#define APP_TARGET_SAMPLES      (100U)
#define APP_CMD_BIT_DWELL_TICKS (200000U)

/* Bench self-check vector: 1 rad/s gyro, ~1 g accel, 1 mrad, 0.0981 m/s */
#define TEST_RATE_X_LSB         (1748)
#define TEST_ACCEL_Z_LSB        (879)
#define TEST_DANG_X_LSB         (8589935)
#define TEST_DVEL_Z_LSB         (43183277)
#define TEST_SW1_NOMINAL_25C    (0x1902U)
#define TEST_SW2_VER_3          (0x0003U)
#define TEST_RATE_1RAD_MIN      (0.995f)
#define TEST_RATE_1RAD_MAX      (1.005f)
#define TEST_ACCEL_1G_MIN_M_S2  (9.75f)
#define TEST_ACCEL_1G_MAX_M_S2  (9.86f)
#define TEST_DANG_1MRAD_MIN     (0.00099f)
#define TEST_DANG_1MRAD_MAX     (0.00101f)
#define TEST_DVEL_1G_MIN_M_S    (0.0975f)
#define TEST_DVEL_1G_MAX_M_S    (0.0986f)
#define TEST_TEMP_EXPECTED_C    (25)
#define TEST_VER_EXPECTED       (3U)
/* Rate plausibility: 16-bit full scale = 2^15 * 2^-20 * 600 = 18.75 rad/s
 * (1074 deg/s). A reading at or beyond 18.5 rad/s is treated as sensor
 * saturation. Replace with the SM-5800 rated range from the datasheet. */
#define APP_RATE_LIMIT_RAD_S    (18.5f)

/* Consecutive RESET_L recovery attempts allowed for a silent sensor */
#define APP_MAX_LINK_RECOVERY   (3U)

/**
 * @brief   Module-private SM-5800 driver instance handle.
 * @req     LLR-SM5800-APP-001
 * @design  SDD-SM5800-4.0
 */
static sm5800_drv_t g_imu_drv =
{
    .state           = SM5800_STATE_UNINIT,
    .base_addr       = 0U,
    .ctrl_shadow     = 0U,
    .is_msg2_pending = false,
    .isr_msg2_count  = 0U,
    .isr_fault_flags = 0U
};

/**
 * @brief   Latest human-readable 100 Hz Message ID 02 telemetry snapshot.
 * @req     LLR-SM5800-APP-005
 * @design  SDD-SM5800-4.4
 */
static sm5800_msg2_eng_t g_last_eng =
{
    .rate_x_rad_s      = 0.0f,
    .rate_y_rad_s      = 0.0f,
    .rate_z_rad_s      = 0.0f,
    .accel_x_m_s2      = 0.0f,
    .accel_y_m_s2      = 0.0f,
    .accel_z_m_s2      = 0.0f,
    .delta_ang_x_rad   = 0.0f,
    .delta_ang_y_rad   = 0.0f,
    .delta_ang_z_rad   = 0.0f,
    .delta_vel_x_m_s   = 0.0f,
    .delta_vel_y_m_s   = 0.0f,
    .delta_vel_z_m_s   = 0.0f,
    .accel_temp_degc   = 0,
    .bit_fault_mask    = 0U,
    .seq_counter       = 0U,
    .sw_version        = 0U,
    .has_module_failed = false,
    .timestamp_us      = 0U
};

/**
 * @brief   Latest X-axis angular rate in deg/s for bench display.
 * @req     LLR-SM5800-APP-005
 * @design  SDD-SM5800-4.4
 */
static float g_last_rate_x_deg_s = 0.0f;

/**
 * @brief   Msg 02 samples discarded by the rate plausibility check.
 * @req     LLR-SM5800-APP-005
 * @design  SDD-SM5800-4.4
 */
static uint32_t g_rate_reject_count = 0U;

/**
 * @brief   Consecutive link-timeout recoveries since the last good sample.
 * @req     LLR-SM5800-APP-008
 * @design  SDD-SM5800-4.7
 */
static uint32_t g_link_recovery_count = 0U;


/* Private prototypes */
static sm5800_status_t init_imu_100hz(void);
static sm5800_status_t verify_raw_to_eng_math(void);
static sm5800_status_t handle_msg2_and_faults(
    sm5800_msg2_eng_t const * p_eng);
static sm5800_status_t run_commanded_bit_cycle(void);
static sm5800_status_t handle_link_faults(void);
static sm5800_status_t init_irq_path(void);
static void service_irq_polling(void);
#if defined(APP_USE_GIC)
static void kick_isr_once(void);
#endif
static bool is_rate_plausible(sm5800_msg2_eng_t const * p_eng);

/**
 * @brief       PS GIC Interrupt Service Routine bound to PL irq_o (100 Hz).
 * @req         LLR-SM5800-APP-002
 * @design      SDD-SM5800-4.1
 * @note        ISR-safe. Register this handler with XScuGic for FPGA irq_o.
 *              Delegates directly to sm5800_irq_isr(); no error swallowed.
 */
void sm5800_ps_gic_isr(void);


void sm5800_ps_gic_isr(void)
{
    sm5800_irq_isr(&g_imu_drv);
}


/**
 * @brief   true: no GIC connection, main loop polls INT_STATUS instead.
 * @req     LLR-SM5800-APP-002
 * @design  SDD-SM5800-4.1
 */
static bool g_irq_polling = true;

#if defined(APP_USE_GIC)
/**
 * @brief   Xilinx SCU GIC driver instance used by the example.
 * @req     LLR-SM5800-APP-002
 * @design  SDD-SM5800-4.1
 */
static XScuGic g_gic;


/**
 * @brief       CPU IRQ exception entry: dispatch to the GIC driver.
 * @req         LLR-SM5800-APP-002
 * @design      SDD-SM5800-4.1
 * @param[in]   p_ctx  &g_gic, registered with Xil_ExceptionRegisterHandler.
 */
static void gic_irq_dispatch(void * p_ctx)
{
    XScuGic_InterruptHandler((XScuGic *)p_ctx);
}


/**
 * @brief       GIC handler for the IP's irq_o: runs the driver ISR.
 * @req         LLR-SM5800-APP-002
 * @design      SDD-SM5800-4.1
 * @param[in]   p_ctx  Unused callback reference (NULL).
 */
static void imu_gic_handler(void * p_ctx)
{
    (void)p_ctx;
    sm5800_ps_gic_isr();
}
#endif


/**
 * @brief       Bare-metal PS entry point for 100 Hz Msg 02 acquisition loop.
 * @req         LLR-SM5800-APP-003
 * @design      SDD-SM5800-4.2
 * @return      0 on normal completion, 1 if self-check, init, or read fails.
 * @note        Not ISR-safe. Bounded by APP_MAX_POLL_CYCLES iterations.
 */
int main(void)
{
    int exit_code = 0;
    sm5800_status_t status = verify_raw_to_eng_math();

    if (SM5800_OK == status)
    {
        status = init_imu_100hz();
    }
    else
    {
        /* Self-check failed; skip hardware initialization */
    }

    if (SM5800_OK != status)
    {
        exit_code = 1;
    }
    else
    {
        uint32_t sample_count = 0U;
        uint32_t poll_cycles = 0U;
        bool is_running = true;

        while ((sample_count < APP_TARGET_SAMPLES)
               && (poll_cycles < APP_MAX_POLL_CYCLES)
               && is_running)
        {
            poll_cycles++;
            service_irq_polling();
            status = handle_link_faults();

            if (SM5800_OK != status)
            {
                /* Sensor stayed silent after bounded recovery attempts */
                exit_code = 1;
                is_running = false;
            }
            else if (sm5800_is_msg2_pending(&g_imu_drv))
            {
                sm5800_msg2_eng_t eng_sample = g_last_eng;

                /* Single-call atomic read + conversion to verifiable SI */
                status = sm5800_read_msg2_eng(&g_imu_drv, &eng_sample);

                if (SM5800_OK == status)
                {
                    status = handle_msg2_and_faults(&eng_sample);
                }
                else
                {
                    /* Read failed; exit acquisition loop cleanly */
                }

                if (SM5800_OK == status)
                {
                    sample_count++;
                    g_link_recovery_count = 0U;
                }
                else if (SM5800_ERR_NO_SAMPLE == status)
                {
                    /* Implausible sample discarded; keep acquiring */
                }
                else
                {
                    exit_code = 1;
                    is_running = false;
                }
            }
            else
            {
                /* Wait for next 100 Hz Message ID 02 interrupt from FPGA */
            }
        }

        if (sample_count < APP_TARGET_SAMPLES)
        {
            exit_code = 1;
        }
        else
        {
            /* Target sample count acquired within poll budget */
        }

        status = sm5800_deinit(&g_imu_drv);
        if (SM5800_OK != status)
        {
            exit_code = 1;
        }
        else
        {
            /* Clean shutdown complete */
        }
    }

    return exit_code;
}


/**
 * @brief       Build CRC-protected PDI config and init 100 Hz Msg 02 mode.
 * @req         LLR-SM5800-APP-004
 * @design      SDD-SM5800-4.3
 * @return      SM5800_OK on success, or sm5800_status_t error code.
 * @note        Not ISR-safe. Configures FPGA strictly for 100 Hz Msg ID 02.
 */
static sm5800_status_t init_imu_100hz(void)
{
    sm5800_status_t result = SM5800_OK;
    uint32_t computed_crc = 0U;
    sm5800_config_t cfg =
    {
        .schema_version  = SM5800_CFG_SCHEMA_VER,
        .base_addr       = IMU_AXI_BASE_ADDR,
        .timeout_us      = SM5800_TIMEOUT_DFLT_US,
        .is_falling_edge = false,
        .is_raw_crc_mode = false,
        .is_msg2_only    = true,
        .crc32           = 0U
    };

    result = sm5800_calc_cfg_crc32(&cfg, &computed_crc);

    if (SM5800_OK == result)
    {
        cfg.crc32 = computed_crc;
        result = sm5800_init(&g_imu_drv, &cfg);
    }
    else
    {
        /* CRC calculation failed; propagate status */
    }

    if (SM5800_OK == result)
    {
        /* Driver is READY (ISR active) before the GIC can deliver irq_o */
        result = init_irq_path();
    }
    else
    {
        /* CRC or driver initialization failed; propagate status */
    }

    return result;
}


/**
 * @brief       Verify one-call raw-to-engineering conversion with 1 g vector.
 * @req         LLR-SM5800-APP-006
 * @design      SDD-SM5800-4.5
 * @return      SM5800_OK if converted 1 g accel is within tolerance, else err.
 * @note        Not ISR-safe (uses float). Uses explicit bounds, no float ==.
 */
static sm5800_status_t verify_raw_to_eng_math(void)
{
    sm5800_status_t result = SM5800_OK;
    sm5800_msg2_raw_t test_raw =
    {
        .rate_x_lsb      = TEST_RATE_X_LSB,
        .rate_y_lsb      = 0,
        .rate_z_lsb      = 0,
        .accel_x_lsb     = 0,
        .accel_y_lsb     = 0,
        .accel_z_lsb     = TEST_ACCEL_Z_LSB,
        .delta_ang_x_lsb = TEST_DANG_X_LSB,
        .delta_ang_y_lsb = 0,
        .delta_ang_z_lsb = 0,
        .delta_vel_x_lsb = 0,
        .delta_vel_y_lsb = 0,
        .delta_vel_z_lsb = TEST_DVEL_Z_LSB,
        .status_word1    = TEST_SW1_NOMINAL_25C,
        .status_word2    = TEST_SW2_VER_3,
        .timestamp_us    = 0U
    };
    sm5800_msg2_eng_t test_eng =
    {
        .rate_x_rad_s      = 0.0f,
        .rate_y_rad_s      = 0.0f,
        .rate_z_rad_s      = 0.0f,
        .accel_x_m_s2      = 0.0f,
        .accel_y_m_s2      = 0.0f,
        .accel_z_m_s2      = 0.0f,
        .delta_ang_x_rad   = 0.0f,
        .delta_ang_y_rad   = 0.0f,
        .delta_ang_z_rad   = 0.0f,
        .delta_vel_x_m_s   = 0.0f,
        .delta_vel_y_m_s   = 0.0f,
        .delta_vel_z_m_s   = 0.0f,
        .accel_temp_degc   = 0,
        .bit_fault_mask    = 0U,
        .seq_counter       = 0U,
        .sw_version        = 0U,
        .has_module_failed = false,
        .timestamp_us      = 0U
    };

    result = sm5800_raw_to_eng_msg2(&test_raw, &test_eng);

    if (SM5800_OK == result)
    {
        if ((test_eng.rate_x_rad_s < TEST_RATE_1RAD_MIN)
            || (test_eng.rate_x_rad_s > TEST_RATE_1RAD_MAX))
        {
            result = SM5800_ERR_PARAM;
        }
        else if ((test_eng.accel_z_m_s2 < TEST_ACCEL_1G_MIN_M_S2)
                 || (test_eng.accel_z_m_s2 > TEST_ACCEL_1G_MAX_M_S2))
        {
            result = SM5800_ERR_PARAM;
        }
        else if ((test_eng.delta_ang_x_rad < TEST_DANG_1MRAD_MIN)
                 || (test_eng.delta_ang_x_rad > TEST_DANG_1MRAD_MAX))
        {
            result = SM5800_ERR_PARAM;
        }
        else if ((test_eng.delta_vel_z_m_s < TEST_DVEL_1G_MIN_M_S)
                 || (test_eng.delta_vel_z_m_s > TEST_DVEL_1G_MAX_M_S))
        {
            result = SM5800_ERR_PARAM;
        }
        else if ((TEST_TEMP_EXPECTED_C != test_eng.accel_temp_degc)
                 || (0U != test_eng.bit_fault_mask)
                 || (TEST_VER_EXPECTED != test_eng.sw_version)
                 || test_eng.has_module_failed)
        {
            result = SM5800_ERR_PARAM;
        }
        else
        {
            result = SM5800_OK;
        }
    }
    else
    {
        /* Conversion failed; propagate error */
    }

    return result;
}


/**
 * @brief       Store Msg 02 telemetry and trigger PS->FPGA Reset/BIT on fault.
 * @req         LLR-SM5800-APP-005
 * @design      SDD-SM5800-4.4
 * @param[in]   p_eng  Converted 100 Hz Msg 02 sample, guaranteed non-NULL.
 * @return      SM5800_OK or error status from Reset/BIT recovery command.
 * @note        Not ISR-safe. Commands RESET_L on latched Module Failed, or
 *              Commanded BIT (BIT_L) on active Background BIT fault flags.
 */
static sm5800_status_t handle_msg2_and_faults(
    sm5800_msg2_eng_t const * p_eng)
{
    sm5800_status_t result = SM5800_OK;

    if (!is_rate_plausible(p_eng))
    {
        /* Saturated / implausible gyro reading: discard, keep last good */
        g_rate_reject_count++;
        result = SM5800_ERR_NO_SAMPLE;
    }
    else if (p_eng->has_module_failed)
    {
        /* Severe latched fault: command FPGA to pulse SM-5800 RESET_L > 1ms */
        result = sm5800_trigger_reset(&g_imu_drv, SM5800_RESET_POLL_MAX);
    }
    else if (0U != p_eng->bit_fault_mask)
    {
        /* Active BIT fault: command FPGA to run SM-5800 Commanded BIT */
        result = run_commanded_bit_cycle();
    }
    else
    {
        /* Nominal 100 Hz sample ready for navigation / flight control */
    }

    if (SM5800_ERR_NO_SAMPLE != result)
    {
        g_last_eng = *p_eng;
        g_last_rate_x_deg_s = sm5800_rad_s_to_deg_s(p_eng->rate_x_rad_s);
    }
    else
    {
        /* Rejected sample never overwrites the last good telemetry */
    }

    return result;
}


/**
 * @brief       Execute a bounded PS->FPGA->IMU Commanded BIT cycle (BIT_L).
 * @req         LLR-SM5800-APP-007
 * @design      SDD-SM5800-4.6
 * @return      SM5800_OK or driver error status.
 * @note        Not ISR-safe. Replace dwell loop with PS hardware timer (>=200ms
 *              per SM-5800 Sec. 6.3.2) in target RTOS/bare-metal system.
 */
static sm5800_status_t run_commanded_bit_cycle(void)
{
    sm5800_status_t result = sm5800_start_cmd_bit(&g_imu_drv);

    if (SM5800_OK == result)
    {
        uint32_t dwell_tick = 0U;
        bool bit_held_ok = true;

        /* Bounded placeholder dwell; use PS TTC/GTimer for >= 200 ms hold */
        for (dwell_tick = 0U; dwell_tick < APP_CMD_BIT_DWELL_TICKS;
             dwell_tick++)
        {
            bool is_active = sm5800_is_cmd_bit_active(&g_imu_drv);
            if (!is_active)
            {
                bit_held_ok = false;
                break;
            }
            else
            {
                /* Continue holding BIT_L low during Commanded BIT window */
            }
        }

        result = sm5800_stop_cmd_bit(&g_imu_drv);
        if ((SM5800_OK == result) && (!bit_held_ok))
        {
            result = SM5800_ERR_STATE;
        }
        else
        {
            /* BIT_L held for full dwell window and released cleanly */
        }
    }
    else
    {
        /* Failed to assert BIT_L; propagate error */
    }

    return result;
}


/**
 * @brief       React to fault IRQs latched by the ISR (sensor went silent).
 * @req         LLR-SM5800-APP-008
 * @design      SDD-SM5800-4.7
 * @return      SM5800_OK, error from sm5800_trigger_reset(), or
 *              SM5800_ERR_TIMEOUT once APP_MAX_LINK_RECOVERY is exhausted.
 * @note        Not ISR-safe. On LINK_TIMEOUT pulses RESET_L (> 1 ms) so the
 *              SM-5800 reboots; the FPGA watchdog re-arms and reports again
 *              if the sensor still sends no valid data.
 */
static sm5800_status_t handle_link_faults(void)
{
    uint32_t faults = 0U;
    sm5800_status_t result = sm5800_take_irq_faults(&g_imu_drv, &faults);

    if ((SM5800_OK == result)
        && (0U != (faults & SM5800_INT_TIMEOUT_MASK)))
    {
        if (g_link_recovery_count < APP_MAX_LINK_RECOVERY)
        {
            g_link_recovery_count++;
            result = sm5800_trigger_reset(&g_imu_drv,
                                          SM5800_RESET_POLL_MAX);
        }
        else
        {
            result = SM5800_ERR_TIMEOUT;
        }
    }
    else
    {
        /* No link timeout; BIT faults are handled per Msg 02 sample */
    }

    return result;
}


/**
 * @brief       Check that all three gyro axes are inside the plausible range.
 * @req         LLR-SM5800-APP-005
 * @design      SDD-SM5800-4.4
 * @param[in]   p_eng  Converted Msg 02 sample, guaranteed non-NULL.
 * @return      true if |rate_x|, |rate_y| and |rate_z| < APP_RATE_LIMIT_RAD_S.
 * @note        ISR-safe. No libm: explicit two-sided comparisons per axis.
 */
static bool is_rate_plausible(sm5800_msg2_eng_t const * p_eng)
{
    bool is_ok = true;

    if ((p_eng->rate_x_rad_s <= -APP_RATE_LIMIT_RAD_S)
        || (p_eng->rate_x_rad_s >= APP_RATE_LIMIT_RAD_S))
    {
        is_ok = false;
    }
    else if ((p_eng->rate_y_rad_s <= -APP_RATE_LIMIT_RAD_S)
             || (p_eng->rate_y_rad_s >= APP_RATE_LIMIT_RAD_S))
    {
        is_ok = false;
    }
    else if ((p_eng->rate_z_rad_s <= -APP_RATE_LIMIT_RAD_S)
             || (p_eng->rate_z_rad_s >= APP_RATE_LIMIT_RAD_S))
    {
        is_ok = false;
    }
    else
    {
        /* All three axes inside the plausible range */
    }

    return is_ok;
}


/**
 * @brief       Route the IP's irq_o through the GIC to sm5800_ps_gic_isr().
 * @req         LLR-SM5800-APP-002
 * @design      SDD-SM5800-4.1
 * @return      SM5800_OK, or SM5800_ERR_STATE if the GIC cannot be set up.
 * @note        Not ISR-safe. Call after sm5800_init(). Without the Xilinx
 *              BSP (or with APP_FORCE_IRQ_POLLING) selects polling mode.
 */
static sm5800_status_t init_irq_path(void)
{
    sm5800_status_t result = SM5800_OK;

#if defined(APP_USE_GIC)
    XScuGic_Config * p_gic_cfg = NULL;

#if defined(SDT)
    p_gic_cfg = XScuGic_LookupConfig(XPAR_XSCUGIC_0_BASEADDR);
#else
    p_gic_cfg = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
#endif

    if (NULL == p_gic_cfg)
    {
        result = SM5800_ERR_STATE;
    }
    else if (XST_SUCCESS != XScuGic_CfgInitialize(&g_gic, p_gic_cfg,
                 (u32)p_gic_cfg->CpuBaseAddress))
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        XScuGic_SetPriorityTriggerType(&g_gic, IMU_IRQ_ID,
                                       (u8)APP_GIC_PRIORITY,
                                       (u8)APP_GIC_TRIGGER);
        Xil_ExceptionInit();
        Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                     gic_irq_dispatch, &g_gic);

        if (XST_SUCCESS != XScuGic_Connect(&g_gic, IMU_IRQ_ID,
                                           imu_gic_handler, NULL))
        {
            result = SM5800_ERR_STATE;
        }
        else
        {
            XScuGic_Enable(&g_gic, IMU_IRQ_ID);
            /* Unmask CPU IRQs (same as Xil_ExceptionEnable(): cpsie i /
             * msr daifclr,#2) via the HAL; the Xilinx macro does not
             * build warning-free under the section-12 flags on ARMv7. */
            sm5800_hal_exit_crit(0U);
            g_irq_polling = false;
            kick_isr_once();
        }
    }
#else
    /* No GIC driver available (or polling forced): poll from main loop */
    g_irq_polling = true;
#endif

    return result;
}


/**
 * @brief       Polling mode only: run the driver ISR from the main loop.
 * @req         LLR-SM5800-APP-002
 * @design      SDD-SM5800-4.1
 * @note        Not ISR-safe. No-op when the GIC delivers interrupts.
 */
static void service_irq_polling(void)
{
    if (g_irq_polling)
    {
        sm5800_irq_isr(&g_imu_drv);
    }
    else
    {
        /* Interrupt mode: sm5800_ps_gic_isr() runs from the GIC */
    }
}


#if defined(APP_USE_GIC)
/**
 * @brief       Run the driver ISR once from task context, IRQs masked.
 * @req         LLR-SM5800-APP-002
 * @design      SDD-SM5800-4.1
 * @note        Not ISR-safe. A flag raised after sm5800_init() armed
 *              INT_ENABLE but before the GIC was configured left irq_o high
 *              without an edge the GIC saw; handling it here (W1C) re-arms
 *              irq_o so every later event produces a fresh rising edge.
 */
static void kick_isr_once(void)
{
    uint32_t saved_mask = sm5800_hal_enter_crit();

    sm5800_irq_isr(&g_imu_drv);
    sm5800_hal_exit_crit(saved_mask);
}
#endif
