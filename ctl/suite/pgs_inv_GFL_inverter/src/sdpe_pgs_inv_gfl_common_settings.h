/**
 * @file sdpe_pgs_inv_gfl_common_settings.h
 * @brief SDPE project bindings for PGS Grid-Following Inverter Common Settings.
 * @note Platform-independent control contract shared by all pgs_inv_GFL_inverter projects. Hardware timing, sensing, board resources and BUILD_LEVEL selection remain in each platform requirement.
 */

#ifndef _PROJECT_SDPE_PGS_INV_GFL_COMMON_SETTINGS_H_
#define _PROJECT_SDPE_PGS_INV_GFL_COMMON_SETTINGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Platform-independent GFL controller settings. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_INV_GFL_COMMON_SDPE_PROJECT_ID "pgs_inv_gfl_common"
#define PGS_INV_GFL_COMMON_SDPE_PROJECT_SUITE "pgs_inv_GFL_inverter"
#define PGS_INV_GFL_COMMON_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_INV_GFL_COMMON_SDPE_PROJECT_UPDATED_AT "2026-07-15"

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Enable the existing discrete PID anti-saturation path.
 */
#define _USE_DEBUG_DISCRETE_PID

/**
 * @brief Use DSOGI PLL instead of the default SRF PLL.
 */
// #define USING_DSOGI_PLL

/**
 * @brief Use the three-level NPC modulator instead of the two-level SPWM modulator.
 */
// #define USING_NPC_MODULATOR

//=================================================================================================
/**
 * @brief Runtime.
 */

/**
 * @brief Enable startup ADC offset calibration. Only enable while all calibrated power inputs are in a known zero state.
 */
// #define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable processor-in-the-loop input/output handling.
 */
// #define ENABLE_GMP_DL_PIL_SIM

/**
 * @brief Enable CiA402/GMP framework debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
/**
 * @brief Sampling.
 */

/**
 * @brief Capacitor-current source: direct measurement, current difference, or capacitor-voltage derivative.
 *        Options: (1), (2), (3)
 */
#define GFL_CAPACITOR_CURRENT_CALCULATE_MODE (3)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Nominal grid phase-voltage magnitude in controller per unit.
 */
#define GFL_GRID_VOLTAGE_PU (0.4*1.414f)

/**
 * @brief Nominal grid frequency in hertz.
 */
#define GFL_GRID_FREQUENCY_HZ (60.0f)

/**
 * @brief Level-5 voltage-forming AC-voltage outer-loop execution frequency in hertz.
 */
#define GFL_VAC_LOOP_FREQUENCY_HZ (1000.0f)

/**
 * @brief Current-ISR to Level-5 voltage-forming outer-loop execution divider.
 */
#define GFL_VAC_LOOP_DIVIDER ((uint32_t)(CONTROLLER_FREQUENCY / GFL_VAC_LOOP_FREQUENCY_HZ))

/**
 * @brief Grid-current ADC low-pass cutoff frequency in hertz.
 */
#define GFL_CURRENT_ADC_FILTER_FC_HZ (3000.0f)

/**
 * @brief Level-5 positive-sequence voltage-forming d-axis proportional gain from Vd error PU to Id command PU.
 */
#define GFL_VAC_D_AXIS_KP (0.20f)

/**
 * @brief Level-5 positive-sequence voltage-forming d-axis integral gain in inverse seconds.
 */
#define GFL_VAC_D_AXIS_KI (10.0f)

/**
 * @brief Level-5 positive-sequence voltage-forming q-axis proportional gain from Vq error PU to Iq command PU.
 */
#define GFL_VAC_Q_AXIS_KP (0.20f)

/**
 * @brief Level-5 positive-sequence voltage-forming q-axis integral gain in inverse seconds.
 */
#define GFL_VAC_Q_AXIS_KI (10.0f)

/**
 * @brief Circular magnitude limit applied to the d/q current reference produced by the voltage-forming loop.
 */
#define GFL_VAC_CURRENT_LIMIT_PU (1.20f)

/**
 * @brief Default Level-5 positive-sequence d-axis phase-voltage reference in the internal angle frame.
 */
#define GFL_AC_VOLTAGE_D_REF_PU (GFL_GRID_VOLTAGE_PU)

/**
 * @brief Default Level-5 positive-sequence q-axis phase-voltage reference in the internal angle frame.
 */
#define GFL_AC_VOLTAGE_Q_REF_PU (0.0f)

/**
 * @brief BUILD_LEVEL 1 d-axis open-loop voltage command.
 */
#define GFL_OPEN_LOOP_VD_PU (0.3f)

/**
 * @brief BUILD_LEVEL 1 q-axis open-loop voltage command.
 */
#define GFL_OPEN_LOOP_VQ_PU (0.0f)

/**
 * @brief BUILD_LEVEL 2 d-axis current command.
 */
#define GFL_CURRENT_LEVEL2_ID_PU (0.1f)

/**
 * @brief BUILD_LEVEL 2 q-axis current command.
 */
#define GFL_CURRENT_LEVEL2_IQ_PU (0.0f)

/**
 * @brief BUILD_LEVEL 3 grid-connected d-axis current command.
 */
#define GFL_CURRENT_LEVEL3_ID_PU (0.01f)

/**
 * @brief BUILD_LEVEL 3 grid-connected q-axis current command.
 */
#define GFL_CURRENT_LEVEL3_IQ_PU (0.0f)

/**
 * @brief BUILD_LEVEL 4 d-axis current command.
 */
#define GFL_CURRENT_LEVEL4_ID_PU (0.01f)

/**
 * @brief BUILD_LEVEL 4 q-axis current command.
 */
#define GFL_CURRENT_LEVEL4_IQ_PU (0.0f)

/**
 * @brief ADC offset calibrator filter cutoff frequency.
 */
#define GFL_ADC_CALIBRATOR_FC_HZ (20.0f)

/**
 * @brief ADC offset calibrator second-order filter quality factor.
 */
#define GFL_ADC_CALIBRATOR_Q (0.707f)

/**
 * @brief Minimum CiA402 delay before Operation Enabled.
 */
#define GFL_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief PLL lock-error threshold in controller per unit.
 */
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

/**
 * @brief Slew rate for Level-5 voltage-forming voltage references in pu/s.
 */
#define GFL_VAC_REF_SLEW_PU_S (0.50f)

// User project tail code
/* Accept the historical PIL spelling while new projects use the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_INV_GFL_COMMON_SETTINGS_H_
