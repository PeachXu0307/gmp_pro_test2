/**
 * @file sdpe_pgs_sinv_rc_common_settings.h
 * @brief SDPE project bindings for PGS Single-Phase Inverter Common Control.
 * @note Platform-independent control contract shared by all pgs_sinv_rc projects.
 */

#ifndef _PROJECT_SDPE_PGS_SINV_RC_COMMON_SETTINGS_H_
#define _PROJECT_SDPE_PGS_SINV_RC_COMMON_SETTINGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
// SDPE extension point: add after_extern_open code in the Project Requirement Code page if needed.

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_SINV_RC_COMMON_SDPE_PROJECT_ID "pgs_sinv_rc_common"
#define PGS_SINV_RC_COMMON_SDPE_PROJECT_SUITE "pgs_sinv_rc"
#define PGS_SINV_RC_COMMON_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_SINV_RC_COMMON_SDPE_PROJECT_UPDATED_AT "2026-07-15"

//=================================================================================================
/**
 * @brief Control Features.
 */

/**
 * @brief Enable delayed insertion of the fixed-frequency harmonic QPR bank.
 */
#define SINV_ENABLE_FIXED_HARMONIC_COMPENSATION

/**
 * @brief Enable grid-voltage feedforward for closed-current-loop build levels.
 */
#define SINV_ENABLE_GRID_VOLTAGE_FEEDFORWARD

//=================================================================================================
/**
 * @brief Runtime.
 */

/**
 * @brief Allow ENABLE_OPERATION to advance through the complete CiA402 startup sequence.
 */
#define CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Nominal grid frequency in Hz.
 */
#define CTRL_GRID_FREQUENCY (50.0f)

/**
 * @brief Lower admission threshold with sensing and line-drop margin around the 29 V operating requirement.
 */
#define CTRL_GRID_VOLTAGE_RMS_MIN (28.0f)

/**
 * @brief Upper admission threshold with sensing tolerance around the 43 V operating requirement.
 */
#define CTRL_GRID_VOLTAGE_RMS_MAX (44.0f)

/**
 * @brief SOGI PLL proportional gain.
 */
#define CTRL_PLL_KP (10.0f)

/**
 * @brief SOGI PLL integral time constant in seconds.
 */
#define CTRL_PLL_TI (0.02f)

/**
 * @brief PLL error-filter cutoff frequency in Hz.
 */
#define CTRL_PLL_LPF_FC (20.0f)

/**
 * @brief PLL frequency-error lock threshold in PU.
 */
#define CTRL_SPLL_EPSILON (0.005f)

/**
 * @brief Minimum permitted PLL/VCO frequency.
 */
#define CTRL_PLL_FREQUENCY_MIN_HZ (49.0f)

/**
 * @brief Maximum permitted PLL/VCO frequency.
 */
#define CTRL_PLL_FREQUENCY_MAX_HZ (51.0f)

/**
 * @brief Minimum voltage magnitude used by the normalized SOGI-PLL phase detector.
 */
#define CTRL_PLL_NORM_VMIN_PU (0.18f)

/**
 * @brief Continuous in-range PLL duration required before PWM enable.
 */
#define SINV_PLL_LOCK_HOLD_MS (100)

/**
 * @brief Power measurement low-pass cutoff frequency in Hz.
 */
#define CTRL_PQ_LPF_FC (200.0f)

/**
 * @brief Peak current command limit in PU.
 */
#define CTRL_CURRENT_LIMIT_PU (0.9f)

/**
 * @brief Minimum voltage magnitude used by the P/Q reference generator.
 */
#define CTRL_GRID_VMIN_PU (0.1f)

/**
 * @brief Active-power command slew limit in PU/s.
 */
#define CTRL_P_SLEW_PU_S (5.0f)

/**
 * @brief Reactive-power command slew limit in PU/s.
 */
#define CTRL_Q_SLEW_PU_S (5.0f)

/**
 * @brief Current deadband used by PWM dead-time compensation.
 */
#define CTRL_CURRENT_DB_PU (0.01f)

/**
 * @brief QPR current-loop crossover target; 1 kHz is 5 percent of the 20 kHz sample rate and retains digital-delay margin.
 */
#define SINV_CURRENT_LOOP_BANDWIDTH_HZ (1000.0f)

/**
 * @brief Harmonic QPR bandwidth in Hz; wide enough for grid tolerance, narrow enough for selective damping.
 */
#define SINV_HARMONIC_QPR_BANDWIDTH_HZ (4.0f)

/**
 * @brief Settling time before fixed harmonic resonant compensators are inserted.
 */
#define SINV_HARMONIC_QPR_ENABLE_DELAY_MS (600)

/**
 * @brief Fixed harmonic QPR output gain scale; keeps harmonic compensators corrective instead of dominant.
 */
#define SINV_HARMONIC_QPR_GAIN_SCALE (0.15f)

/**
 * @brief Highest enabled fixed harmonic order; keep low-order compensation first for hardware robustness.
 */
#define SINV_HARMONIC_QPR_MAX_ORDER (7)

/**
 * @brief Grid-voltage feedforward lead compensation in controller samples.
 */
#define SINV_GRID_FEEDFORWARD_LEAD_STEPS (3.0f)

/**
 * @brief BUILD_LEVEL 1 sinusoidal H-bridge voltage amplitude.
 */
#define SINV_LEVEL1_VOLTAGE_REF_PU (0.35f)

/**
 * @brief BUILD_LEVEL 2 peak current command with a resistive load.
 */
#define SINV_LEVEL2_CURRENT_REF_PEAK_PU (0.20f)

/**
 * @brief BUILD_LEVEL 3 signed grid active-power command; positive exports power.
 */
#define SINV_LEVEL3_ACTIVE_POWER_REF_PU (0.10f)

/**
 * @brief BUILD_LEVEL 3 grid reactive-power command.
 */
#define SINV_LEVEL3_REACTIVE_POWER_REF_PU (0.0f)

/**
 * @brief BUILD_LEVEL 4 measured active-power closed-loop target.
 */
#define SINV_LEVEL4_ACTIVE_POWER_REF_PU (0.15f)

/**
 * @brief Active-power outer-loop proportional gain.
 */
#define SINV_POWER_LOOP_KP (0.6f)

/**
 * @brief Active-power outer-loop integral gain per second.
 */
#define SINV_POWER_LOOP_KI (8.0f)

/**
 * @brief BUILD_LEVEL 5 DC bus target, with boost/modulation headroom above 43 Vrms peak.
 */
#define SINV_DC_BUS_REF_V (72.0f)

/**
 * @brief DC-bus outer-loop proportional gain.
 */
#define SINV_DC_BUS_LOOP_KP (0.8f)

/**
 * @brief DC-bus outer-loop integral gain per second.
 */
#define SINV_DC_BUS_LOOP_KI (12.0f)

/**
 * @brief DC-bus feedback LPF cutoff; rejects unavoidable 100 Hz single-phase ripple from the voltage PI.
 */
#define SINV_DC_BUS_FEEDBACK_LPF_HZ (15.0f)

/**
 * @brief DC-bus soft-start and setpoint slew limit in V/s.
 */
#define SINV_DC_BUS_REFERENCE_SLEW_V_S (40.0f)

/**
 * @brief Minimum accepted displacement power-factor magnitude.
 */
#define SINV_POWER_FACTOR_MIN (0.10f)

/**
 * @brief Safe unity-power-factor startup default for rectifier mode.
 */
#define SINV_POWER_FACTOR_DEFAULT (1.0f)

/**
 * @brief Default reactive-power direction; user interaction may command +1 or -1.
 */
#define SINV_REACTIVE_POWER_SIGN_DEFAULT (1.0f)

/**
 * @brief Symmetric outer-loop active-power command limit.
 */
#define SINV_OUTER_LOOP_POWER_LIMIT_PU (0.65f)

/**
 * @brief Power and DC-bus outer-loop execution frequency.
 */
#define SINV_OUTER_LOOP_FREQUENCY_HZ (1000.0f)

/**
 * @brief Minimum operation-enabled transition delay.
 */
#define SINV_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief Startup interval during which controller-divergence supervision is inhibited while OVP and OCP remain active.
 */
#define SINV_CTRL_DIVERGE_GRACE_MS (500)

/**
 * @brief Consecutive in-range ADC samples required before software fast OVP/OCP is armed; hardware trip-zone protection remains independent.
 */
#define SINV_FAST_PROTECT_VALID_SAMPLES (20)

// User project tail code
// SDPE extension point: add before_footer code in the Project Requirement Code page if needed.

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_SINV_RC_COMMON_SETTINGS_H_
