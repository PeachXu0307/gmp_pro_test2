/**
 * @file ctl_main.h
 * @brief Top-level controller interface for the four-switch buck-boost converter.
 */

#ifndef _FILE_CTRL_MAIN_H_
#define _FILE_CTRL_MAIN_H_

#ifndef SDPE_FSBB_SETTINGS_HEADER
#define SDPE_FSBB_SETTINGS_HEADER <sdpe_dps_fsbb_iris_settings.h>
#endif
#include SDPE_FSBB_SETTINGS_HEADER
#include <core/pm/function_scheduler.h>
#include <core/dev/pil_core.h>
#include <ctl/framework/cia402_state_machine.h>
#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/digital_power/dcdc/dcdc_core.h>
#include <ctl/component/digital_power/dcdc/fsbb.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern cia402_sm_t cia402_sm;
extern ctl_dcdc_core_t dcdc_core;
extern ctl_pid_t fsbb_iout_pid;

extern adc_channel_t adc_v_in;
extern adc_channel_t adc_v_out;
extern adc_channel_t adc_i_in;
extern adc_channel_t adc_i_L;
extern adc_channel_t adc_i_load;
extern fsbb_modulator_t fsbb_mod;

extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;
extern volatile fast_gt index_adc_calibrator;

extern ctrl_gt g_v_out_ref_user;
extern ctrl_gt g_i_limit_user;
extern ctrl_gt g_i_out_ref_user;
extern ctrl_gt v_req;

typedef enum _tag_fsbb_control_mode
{
    FSBB_CONTROL_MODE_CV = 1,
    FSBB_CONTROL_MODE_CC = 2
} fsbb_control_mode_t;

extern volatile uint16_t g_fsbb_control_mode;

typedef enum _tag_fsbb_fault
{
    FSBB_FAULT_NONE = 0,
    FSBB_FAULT_VIN_UNDERVOLTAGE = 1U << 0,
    FSBB_FAULT_VIN_OVERVOLTAGE = 1U << 1,
    FSBB_FAULT_VOUT_OVERVOLTAGE = 1U << 2,
    FSBB_FAULT_IL_POSITIVE_OVERCURRENT = 1U << 3,
    FSBB_FAULT_IL_NEGATIVE_OVERCURRENT = 1U << 4,
    FSBB_FAULT_IOUT_OVERCURRENT = 1U << 5
} fsbb_fault_t;

extern volatile uint16_t g_fsbb_faults;
extern volatile fast_gt g_fsbb_output_enabled;
extern volatile uint16_t g_fsbb_startup_settle_count;

#ifndef FSBB_STARTUP_SETTLE_CYCLES
#define FSBB_STARTUP_SETTLE_CYCLES ((uint16_t)(CONTROLLER_FREQUENCY / 10.0f))
#endif

void ctl_init(void);
void ctl_mainloop(void);
void ctl_enable_pwm(void);
void ctl_disable_pwm(void);
void clear_all_controllers(void);
gmp_task_status_t tsk_protect(gmp_task_t* tsk);

GMP_STATIC_INLINE ctrl_gt ctl_fsbb_calc_cv_inductor_limit(ctrl_gt v_target_pu, ctrl_gt iout_limit_pu)
{
    ctrl_gt v_in_safe = (adc_v_in.control_port.value > float2ctrl(1.0f / CTRL_VOLTAGE_BASE))
                            ? adc_v_in.control_port.value
                            : float2ctrl(FSBB_INPUT_VOLTAGE_NOMINAL / CTRL_VOLTAGE_BASE);
    ctrl_gt gain = ctl_div(v_target_pu, v_in_safe);
    if (gain < float2ctrl(1.0f))
        gain = float2ctrl(1.0f);

    ctrl_gt il_limit = ctl_mul(iout_limit_pu, gain);
    il_limit = ctl_mul(il_limit, float2ctrl(FSBB_CV_INDUCTOR_CURRENT_MARGIN));
    return ctl_sat(il_limit, float2ctrl(FSBB_INDUCTOR_CURRENT_CMD_MAX / CTRL_CURRENT_BASE), float2ctrl(0.0f));
}

GMP_STATIC_INLINE ctrl_gt ctl_step_fsbb_output_current_cascade(void)
{
    ctl_dcdc_internal_ingest_and_filter(&dcdc_core);
    dcdc_core.is_current_dominant = 1;

    dcdc_core.i_ramp_ref = ctl_step_slope_limiter(&dcdc_core.ramp_i, dcdc_core.i_target);
    ctrl_gt error_iout = dcdc_core.i_ramp_ref - dcdc_core.filter_i_load.out;
    ctrl_gt inner_i_ref = ctl_step_pid_ser(&fsbb_iout_pid, error_iout);

    ctl_set_pid_limit(&dcdc_core.current_pid, float2ctrl(FSBB_CONTROL_VOLTAGE_CMD_MAX / CTRL_VOLTAGE_BASE),
                      float2ctrl(0.0f));
    ctl_set_pid_int_limit(&dcdc_core.current_pid, float2ctrl(FSBB_CONTROL_VOLTAGE_CMD_MAX / CTRL_VOLTAGE_BASE),
                          float2ctrl(0.0f));

    ctrl_gt error_il = inner_i_ref - dcdc_core.filter_i_L.out;
    dcdc_core.v_out_formal = ctl_step_pid_ser(&dcdc_core.current_pid, error_il);
    dcdc_core.v_out_formal = ctl_sat(dcdc_core.v_out_formal,
                                     float2ctrl(FSBB_CONTROL_VOLTAGE_CMD_MAX / CTRL_VOLTAGE_BASE),
                                     float2ctrl(0.0f));
    return dcdc_core.v_out_formal;
}

GMP_STATIC_INLINE ctrl_gt ctl_step_fsbb_voltage_cascade_aw(void)
{
    ctl_dcdc_internal_ingest_and_filter(&dcdc_core);
    dcdc_core.is_current_dominant = 0;

    dcdc_core.v_ramp_ref = ctl_step_slope_limiter(&dcdc_core.ramp_v, dcdc_core.v_target);
    ctrl_gt error_v = dcdc_core.v_ramp_ref - dcdc_core.filter_v_out.out;
    ctrl_gt inner_i_ref = ctl_step_pid_ser(&dcdc_core.voltage_pid, error_v);

    ctrl_gt error_i = inner_i_ref - dcdc_core.filter_i_L.out;
    dcdc_core.v_out_formal = ctl_step_pid_ser(&dcdc_core.current_pid, error_i);
    dcdc_core.v_out_formal = ctl_sat(dcdc_core.v_out_formal, dcdc_core.out_max, dcdc_core.out_min);

    if (((dcdc_core.v_out_formal >= dcdc_core.current_pid.out_max) && (error_v > float2ctrl(0.0f))) ||
        ((dcdc_core.v_out_formal <= dcdc_core.current_pid.out_min) && (error_v < float2ctrl(0.0f))))
    {
        dcdc_core.is_current_dominant = 1;
    }

    return dcdc_core.v_out_formal;
}

/** Execute one control sample after the platform input callback has run. */
GMP_STATIC_INLINE void ctl_dispatch(void)
{
#if defined SPECIFY_ENABLE_ADC_CALIBRATE
    if (flag_enable_adc_calibrator)
    {
        if (index_adc_calibrator == 0)
            ctl_step_adc_calibrator(&adc_calibrator, adc_i_L.control_port.value);
#if defined FSBB_ENABLE_IOUT_SAMPLE
        else if (index_adc_calibrator == 1)
            ctl_step_adc_calibrator(&adc_calibrator, adc_i_load.control_port.value);
#endif
        return;
    }
#endif

    g_fsbb_faults = 0;


    if (g_fsbb_faults != FSBB_FAULT_NONE)
        return;

    if (g_fsbb_startup_settle_count != 0U)
    {
        ctl_dcdc_internal_ingest_and_filter(&dcdc_core);
        v_req = float2ctrl(0.0f);
        ctl_step_fsbb_modulator(&fsbb_mod, v_req, adc_v_in.control_port.value);

        g_fsbb_startup_settle_count--;
        if (g_fsbb_startup_settle_count == 0U)
        {
            ctl_set_slope_limiter_current(&dcdc_core.ramp_v, dcdc_core.filter_v_out.out);
            ctl_set_slope_limiter_current(&dcdc_core.ramp_i, dcdc_core.filter_i_load.out);
            dcdc_core.v_ramp_ref = dcdc_core.filter_v_out.out;
            dcdc_core.i_ramp_ref = dcdc_core.filter_i_load.out;
        }
        return;
    }

    {
        static uint16_t last_control_mode = FSBB_CONTROL_MODE_CV;
        ctrl_gt v_target = ctl_div(ctl_sat(g_v_out_ref_user, float2ctrl(FSBB_OUTPUT_VOLTAGE_MAX),
                                           float2ctrl(FSBB_OUTPUT_VOLTAGE_MIN)),
                                   float2ctrl(CTRL_VOLTAGE_BASE));
        ctrl_gt iout_limit = ctl_div(ctl_sat(g_i_limit_user, float2ctrl(FSBB_OUTPUT_CURRENT_LIM),
                                             float2ctrl(FSBB_OUTPUT_CURRENT_MIN)),
                                     float2ctrl(CTRL_CURRENT_BASE));
        ctrl_gt inductor_limit = ctl_fsbb_calc_cv_inductor_limit(v_target, iout_limit);

        if (last_control_mode != g_fsbb_control_mode)
        {
            ctl_clear_dcdc_core(&dcdc_core);
            ctl_clear_pid(&fsbb_iout_pid);
            last_control_mode = g_fsbb_control_mode;
        }

        if (g_fsbb_control_mode == FSBB_CONTROL_MODE_CC)
        {
            dcdc_core.mode = CTL_DCDC_MODE_CURRENTLOOP;
            dcdc_core.i_target =
                ctl_div(ctl_sat(g_i_out_ref_user, float2ctrl(FSBB_OUTPUT_CURRENT_LIM),
                                float2ctrl(FSBB_OUTPUT_CURRENT_MIN)),
                        float2ctrl(CTRL_CURRENT_BASE));
            ctl_set_pid_limit(&fsbb_iout_pid, float2ctrl(FSBB_INDUCTOR_CURRENT_CMD_MAX / CTRL_CURRENT_BASE),
                              float2ctrl(0.0f));
            ctl_set_pid_int_limit(&fsbb_iout_pid, float2ctrl(FSBB_INDUCTOR_CURRENT_CMD_MAX / CTRL_CURRENT_BASE),
                                  float2ctrl(0.0f));
            v_req = ctl_step_fsbb_output_current_cascade();
        }
        else
        {
            dcdc_core.mode = CTL_DCDC_MODE_VOLTAGELOOP;
            dcdc_core.v_target = v_target;
            ctl_set_pid_limit(&dcdc_core.voltage_pid, inductor_limit, float2ctrl(0.0f));
            ctl_set_pid_int_limit(&dcdc_core.voltage_pid, inductor_limit, float2ctrl(0.0f));
            ctl_set_pid_limit(&dcdc_core.current_pid, float2ctrl(FSBB_CONTROL_VOLTAGE_CMD_MAX / CTRL_VOLTAGE_BASE),
                              float2ctrl(0.0f));
            ctl_set_pid_int_limit(&dcdc_core.current_pid, float2ctrl(FSBB_CONTROL_VOLTAGE_CMD_MAX / CTRL_VOLTAGE_BASE),
                                  float2ctrl(0.0f));

            v_req = ctl_step_fsbb_voltage_cascade_aw();
            //v_req = v_target;
        }
    }

    ctl_step_fsbb_modulator(&fsbb_mod, v_req, adc_v_in.control_port.value);
}

#ifdef __cplusplus
}
#endif

#endif // _FILE_CTRL_MAIN_H_
