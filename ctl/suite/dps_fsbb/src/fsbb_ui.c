#include <gmp_core.h>

#include "ctl_main.h"
#include "user_main.h"

#include <xplt.ctl_interface.h>
#include <oled_driver.h>
#include <stdio.h>

#define FSBB_UI_MIN_VOLTAGE_CV       1000L
#define FSBB_UI_MAX_VOLTAGE_CV       5000L
#define FSBB_UI_DEFAULT_VOLTAGE_CV   1000L
#define FSBB_UI_MIN_CURRENT_CA       100L
#define FSBB_UI_MAX_CURRENT_CA       500L
#define FSBB_UI_DEFAULT_CURRENT_CA   100L
#define FSBB_UI_OLED_REFRESH_MS      250U

#define FSBB_UI_KEY_RIGHT            1U
#define FSBB_UI_KEY_LEFT             2U
#define FSBB_UI_KEY_ENABLE           8U
#define FSBB_UI_KEY_MODE             9U
#define FSBB_UI_KEY_SUB              14U
#define FSBB_UI_KEY_ADD              15U
#define FSBB_UI_KEY_SET              21U

#define FSBB_UI_DIGIT_COUNT          8U
#define FSBB_UI_LAST_EDIT_CURSOR     2U
#define FSBB_UI_SEG_FLAG             0x80U
#define FSBB_UI_LEFT_DP_DIGIT        0U
#define FSBB_UI_RIGHT_DP_DIGIT       4U
#define FSBB_UI_LEFT_COLON_DIGIT     1U
#define FSBB_UI_RIGHT_COLON_DIGIT    5U

#define FSBB_UI_LED_MODE_GPIO        61U
#define FSBB_UI_LED_OUTPUT_GPIO      59U
#define FSBB_UI_LED_ACTIVE_LEVEL     0U
#define FSBB_UI_LED_INACTIVE_LEVEL   1U
#define FSBB_UI_KEY_PRESS_COUNT      2U
#define FSBB_UI_KEY_RELEASE_COUNT    2U

typedef enum
{
    FSBB_UI_PARAM_LOCKED = 0U,
    FSBB_UI_PARAM_SETTING = 1U
} fsbb_ui_param_state_t;

static const unsigned char fsbb_ui_led_lut[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F,
    0x40, 0x00
};

static int32_t fsbb_ui_vset_cv = FSBB_UI_DEFAULT_VOLTAGE_CV;
static int32_t fsbb_ui_iset_ca = FSBB_UI_DEFAULT_CURRENT_CA;
static int32_t fsbb_ui_edit_value = FSBB_UI_DEFAULT_VOLTAGE_CV;
static fsbb_ui_param_state_t fsbb_ui_state = FSBB_UI_PARAM_LOCKED;
static uint8_t fsbb_ui_cursor = 0U;
static uint8_t fsbb_ui_initialized = 0U;
static uint8_t fsbb_ui_enable_after_fault_reset = 0U;

static int32_t fsbb_ui_clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value)
        value = min_value;
    if (value > max_value)
        value = max_value;

    return value;
}

static int32_t fsbb_ui_force_last_digit_zero(int32_t value)
{
    return (value / 10L) * 10L;
}

static int32_t fsbb_ui_clamp_voltage_cv(int32_t value)
{
    value = fsbb_ui_force_last_digit_zero(value);
    return fsbb_ui_clamp_i32(value, FSBB_UI_MIN_VOLTAGE_CV, FSBB_UI_MAX_VOLTAGE_CV);
}

static int32_t fsbb_ui_clamp_current_ca(int32_t value)
{
    value = fsbb_ui_force_last_digit_zero(value);
    return fsbb_ui_clamp_i32(value, FSBB_UI_MIN_CURRENT_CA, FSBB_UI_MAX_CURRENT_CA);
}

static int32_t fsbb_ui_clamp_active_value(int32_t value)
{
    if (g_fsbb_control_mode == FSBB_CONTROL_MODE_CC)
        return fsbb_ui_clamp_current_ca(value);

    return fsbb_ui_clamp_voltage_cv(value);
}

static int32_t fsbb_ui_voltage_to_cv(ctrl_gt value_pu)
{
    float value_v = (float)value_pu * CTRL_VOLTAGE_BASE;

    if (value_v < 0.0f)
        value_v = 0.0f;

    return (int32_t)(value_v * 100.0f + 0.5f);
}

static int32_t fsbb_ui_current_to_ma(ctrl_gt value_pu)
{
    float value_a = (float)value_pu * CTRL_CURRENT_BASE;

    if (value_a < 0.0f)
        value_a = -value_a;

    return (int32_t)(value_a * 1000.0f + 0.5f);
}

static void fsbb_ui_apply_setpoints(void)
{
    fsbb_ui_vset_cv = fsbb_ui_clamp_voltage_cv(fsbb_ui_vset_cv);
    fsbb_ui_iset_ca = fsbb_ui_clamp_current_ca(fsbb_ui_iset_ca);

    g_v_out_ref_user = float2ctrl(((float)fsbb_ui_vset_cv / 100.0f) / CTRL_VOLTAGE_BASE);
    g_i_limit_user = float2ctrl(((float)fsbb_ui_iset_ca / 100.0f) / CTRL_CURRENT_BASE);
}

static void fsbb_ui_update_leds(void)
{
    GPIO_WritePin(FSBB_UI_LED_MODE_GPIO,
                  (g_fsbb_control_mode == FSBB_CONTROL_MODE_CV) ?
                  FSBB_UI_LED_ACTIVE_LEVEL : FSBB_UI_LED_INACTIVE_LEVEL);
    GPIO_WritePin(FSBB_UI_LED_OUTPUT_GPIO,
                  g_fsbb_output_enabled ? FSBB_UI_LED_ACTIVE_LEVEL : FSBB_UI_LED_INACTIVE_LEVEL);
}

static int32_t fsbb_ui_active_set_value(void)
{
    if (g_fsbb_control_mode == FSBB_CONTROL_MODE_CC)
        return fsbb_ui_iset_ca;

    return fsbb_ui_vset_cv;
}

static void fsbb_ui_store_active_edit(void)
{
    fsbb_ui_edit_value = fsbb_ui_clamp_active_value(fsbb_ui_edit_value);

    if (g_fsbb_control_mode == FSBB_CONTROL_MODE_CC)
        fsbb_ui_iset_ca = fsbb_ui_edit_value;
    else
        fsbb_ui_vset_cv = fsbb_ui_edit_value;

    fsbb_ui_apply_setpoints();
}

static void fsbb_ui_enter_setting_state(void)
{
    fsbb_ui_edit_value = fsbb_ui_active_set_value();
    fsbb_ui_cursor = 0U;
    fsbb_ui_state = FSBB_UI_PARAM_SETTING;
}

static void fsbb_ui_save_and_lock(void)
{
    fsbb_ui_store_active_edit();
    fsbb_ui_state = FSBB_UI_PARAM_LOCKED;
}

static int32_t fsbb_ui_digit_weight(uint8_t cursor)
{
    static const int32_t weights[4] = {1000L, 100L, 10L, 1L};
    return weights[cursor & 0x03U];
}

static int32_t fsbb_ui_set_digit(int32_t value, uint8_t cursor, int8_t digit)
{
    int32_t weight = fsbb_ui_digit_weight(cursor);

    if (cursor > FSBB_UI_LAST_EDIT_CURSOR)
        return fsbb_ui_clamp_active_value(value);

    value -= ((value / weight) % 10L) * weight;
    value += (int32_t)digit * weight;

    return fsbb_ui_clamp_active_value(value);
}

static int32_t fsbb_ui_step_digit(int32_t value, uint8_t cursor, int8_t direction)
{
    if (cursor > FSBB_UI_LAST_EDIT_CURSOR)
        return fsbb_ui_clamp_active_value(value);

    value += (int32_t)direction * fsbb_ui_digit_weight(cursor);
    return fsbb_ui_clamp_active_value(value);
}

static int8_t fsbb_ui_digit_from_key(fast_gt key_id)
{
    switch (key_id)
    {
    case 20: return 1;
    case 19: return 2;
    case 18: return 3;
    case 17: return 4;
    case 16: return 5;
    case 7:  return 6;
    case 6:  return 7;
    case 5:  return 8;
    case 4:  return 9;
    case 3:  return 0;
    default: return -1;
    }
}

static ec_gt fsbb_ui_read_raw_key(ht16k33_dev_t* dev, fast_gt* key_id_ret)
{
    data_gt key_data[HT16K33_CFG_KEY_RAM_SIZE] = {0};
    uint32_t byte_idx;
    uint32_t bit_idx;
    ec_gt ret;

    if ((dev == NULL) || (key_id_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    ret = gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, HT16K33_REG_KEY_DATA_ADDR, 1,
                               key_data, HT16K33_CFG_KEY_RAM_SIZE, HT16K33_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
    {
        *key_id_ret = 0;
        return ret;
    }

    *key_id_ret = 0;
    for (byte_idx = 0U; byte_idx < HT16K33_CFG_KEY_RAM_SIZE; byte_idx++)
    {
        if (key_data[byte_idx] == 0U)
            continue;

        for (bit_idx = 0U; bit_idx < 8U; bit_idx++)
        {
            if (key_data[byte_idx] & (1U << bit_idx))
            {
                uint32_t ks_row = byte_idx / 2U;
                uint32_t k_col = (byte_idx % 2U) * 8U + bit_idx;
                *key_id_ret = (fast_gt)((ks_row * 13U) + k_col + 1U);
                return GMP_EC_OK;
            }
        }
    }

    return GMP_EC_OK;
}

static fast_gt fsbb_ui_filter_key_event(fast_gt raw_key)
{
    static fast_gt stable_key = 0;
    static fast_gt candidate_key = 0;
    static uint8_t candidate_count = 0U;
    static uint8_t release_count = 0U;

    if (stable_key != 0)
    {
        if (raw_key == 0)
        {
            if (release_count < FSBB_UI_KEY_RELEASE_COUNT)
                release_count++;
            if (release_count >= FSBB_UI_KEY_RELEASE_COUNT)
                stable_key = 0;
        }
        else
        {
            release_count = 0U;
        }

        return 0;
    }

    release_count = 0U;

    if (raw_key == 0)
    {
        candidate_key = 0;
        candidate_count = 0U;
        return 0;
    }

    if (raw_key == candidate_key)
    {
        if (candidate_count < FSBB_UI_KEY_PRESS_COUNT)
            candidate_count++;
    }
    else
    {
        candidate_key = raw_key;
        candidate_count = 1U;
    }

    if (candidate_count >= FSBB_UI_KEY_PRESS_COUNT)
    {
        stable_key = candidate_key;
        candidate_key = 0;
        candidate_count = 0U;
        return stable_key;
    }

    return 0;
}

static void fsbb_ui_switch_mode(void)
{
    if (fsbb_ui_state == FSBB_UI_PARAM_SETTING)
        fsbb_ui_store_active_edit();

    g_fsbb_control_mode = (g_fsbb_control_mode == FSBB_CONTROL_MODE_CV) ?
                          FSBB_CONTROL_MODE_CC : FSBB_CONTROL_MODE_CV;
    fsbb_ui_cursor = 0U;
    fsbb_ui_edit_value = fsbb_ui_active_set_value();
    fsbb_ui_state = FSBB_UI_PARAM_LOCKED;
    clear_all_controllers();
    fsbb_ui_update_leds();
}

static void fsbb_ui_request_output_enable(void)
{
    if (g_fsbb_faults == FSBB_FAULT_NONE)
    {
        fsbb_ui_enable_after_fault_reset = 0U;
        cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);
        return;
    }

    if (ctl_fsbb_active_faults() == FSBB_FAULT_NONE)
    {
        fsbb_ui_enable_after_fault_reset = 1U;
        cia402_send_cmd(&cia402_sm, CIA402_CMD_FAULT_RESET);
    }
}

static void fsbb_ui_service_pending_enable(void)
{
    if (!fsbb_ui_enable_after_fault_reset)
        return;

    if (g_fsbb_faults != FSBB_FAULT_NONE)
    {
        if (ctl_fsbb_active_faults() != FSBB_FAULT_NONE)
            fsbb_ui_enable_after_fault_reset = 0U;
        return;
    }

    if (cia402_sm.current_state != CIA402_SM_FAULT)
    {
        fsbb_ui_enable_after_fault_reset = 0U;
        cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);
    }
}

static void fsbb_ui_set_display_ram(ht16k33_dev_t* dev, const uint16_t segs[FSBB_UI_DIGIT_COUNT])
{
    uint8_t i;

    for (i = 0U; i < HT16K33_CFG_DISP_RAM_SIZE; i++)
        dev->display_ram[i] = 0U;

    for (i = 0U; i < FSBB_UI_DIGIT_COUNT; i++)
        dev->display_ram[i * 2U] = (data_gt)segs[i];

    dev->is_dirty = 1;
}

static void fsbb_ui_render_number(uint16_t* segs, uint8_t offset, int32_t value)
{
    if (value < 0)
        value = 0;
    if (value > 9999L)
        value = 9999L;

    segs[offset + 0U] = fsbb_ui_led_lut[(value / 1000L) % 10L];
    segs[offset + 1U] = fsbb_ui_led_lut[(value / 100L) % 10L];
    segs[offset + 2U] = fsbb_ui_led_lut[(value / 10L) % 10L];
    segs[offset + 3U] = fsbb_ui_led_lut[value % 10L];
}

static uint16_t fsbb_ui_required_flag_for_digit(uint8_t digit)
{
    uint16_t flags = 0U;

    if ((digit == FSBB_UI_LEFT_DP_DIGIT) || (digit == FSBB_UI_RIGHT_DP_DIGIT))
        flags |= FSBB_UI_SEG_FLAG;

    if ((g_fsbb_control_mode == FSBB_CONTROL_MODE_CV) && (digit == FSBB_UI_LEFT_COLON_DIGIT))
        flags |= FSBB_UI_SEG_FLAG;

    if ((g_fsbb_control_mode == FSBB_CONTROL_MODE_CC) && (digit == FSBB_UI_RIGHT_COLON_DIGIT))
        flags |= FSBB_UI_SEG_FLAG;

    return flags;
}

static void fsbb_ui_apply_dp_and_mode_colon(uint16_t* segs)
{
    uint8_t i;

    for (i = 0U; i < FSBB_UI_DIGIT_COUNT; i++)
    {
        segs[i] &= (uint16_t)~FSBB_UI_SEG_FLAG;
        segs[i] |= fsbb_ui_required_flag_for_digit(i);
    }
}

static void fsbb_ui_render_7seg(ht16k33_dev_t* dev)
{
    uint16_t segs[FSBB_UI_DIGIT_COUNT];
    int32_t left_value = fsbb_ui_vset_cv;
    int32_t right_value = fsbb_ui_iset_ca;
    uint8_t blink_on = (uint8_t)(((gmp_base_get_system_tick() / 500U) & 0x01U) == 0U);
    uint8_t active_offset = (g_fsbb_control_mode == FSBB_CONTROL_MODE_CC) ? 4U : 0U;
    uint8_t active_digit = (uint8_t)(active_offset + fsbb_ui_cursor);

    if (fsbb_ui_state == FSBB_UI_PARAM_SETTING)
    {
        if (g_fsbb_control_mode == FSBB_CONTROL_MODE_CC)
            right_value = fsbb_ui_edit_value;
        else
            left_value = fsbb_ui_edit_value;
    }

    fsbb_ui_render_number(segs, 0U, fsbb_ui_clamp_voltage_cv(left_value));
    fsbb_ui_render_number(segs, 4U, fsbb_ui_clamp_current_ca(right_value));
    fsbb_ui_apply_dp_and_mode_colon(segs);

    if ((fsbb_ui_state == FSBB_UI_PARAM_SETTING) && !blink_on)
        segs[active_digit] = fsbb_ui_required_flag_for_digit(active_digit);

    fsbb_ui_set_display_ram(dev, segs);
}

static void fsbb_ui_render_oled(void)
{
    char str_buf[32];
    int32_t vin_cv = fsbb_ui_voltage_to_cv(adc_v_in.control_port.value);
    int32_t vout_cv = fsbb_ui_voltage_to_cv(adc_v_out.control_port.value);
    int32_t iin_ma = fsbb_ui_current_to_ma(adc_i_in.control_port.value);
    int32_t iout_ma = fsbb_ui_current_to_ma(adc_i_load.control_port.value);
    int32_t il_ma = fsbb_ui_current_to_ma(adc_i_L.control_port.value);

    if (g_fsbb_control_mode == FSBB_CONTROL_MODE_CC)
        sprintf(str_buf, "Iset: %1ld.%02ld A  ", fsbb_ui_iset_ca / 100L, fsbb_ui_iset_ca % 100L);
    else
        sprintf(str_buf, "Vset:%2ld.%02ld V  ", fsbb_ui_vset_cv / 100L, fsbb_ui_vset_cv % 100L);
    oled_show_str(0, 0, str_buf);

    sprintf(str_buf, "Vin:  %2ld.%02ld V  ", vin_cv / 100L, vin_cv % 100L);
    oled_show_str(0, 1, str_buf);

    sprintf(str_buf, "Vout: %2ld.%02ld V  ", vout_cv / 100L, vout_cv % 100L);
    oled_show_str(0, 2, str_buf);

    sprintf(str_buf, "Iin:  %4ld mA  ", iin_ma);
    oled_show_str(0, 3, str_buf);

    sprintf(str_buf, "Iout: %4ld mA  ", iout_ma);
    oled_show_str(0, 4, str_buf);

    sprintf(str_buf, "IL:   %4ld mA  ", il_ma);
    oled_show_str(0, 5, str_buf);
}

void fsbb_ui_init(void)
{
    fsbb_ui_vset_cv = FSBB_UI_DEFAULT_VOLTAGE_CV;
    fsbb_ui_iset_ca = FSBB_UI_DEFAULT_CURRENT_CA;
    g_fsbb_control_mode = FSBB_CONTROL_MODE_CV;
    fsbb_ui_apply_setpoints();
    fsbb_ui_edit_value = fsbb_ui_vset_cv;
    fsbb_ui_cursor = 0U;
    fsbb_ui_state = FSBB_UI_PARAM_LOCKED;
    fsbb_ui_enable_after_fault_reset = 0U;
    fsbb_ui_update_leds();
    fsbb_ui_initialized = 1U;
}

gmp_task_status_t tsk_fsbb_ui_key(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;
    fast_gt key_id = 0;
    fast_gt pressed_key = 0;
    int8_t digit;
    static uint32_t last_oled_tick = 0U;

    if (!fsbb_ui_initialized)
        fsbb_ui_init();

    if (fsbb_ui_read_raw_key(dev, &key_id) != GMP_EC_OK)
    {
        tsk->is_enabled = 0;
        return GMP_TASK_DONE;
    }

    pressed_key = fsbb_ui_filter_key_event(key_id);
    fsbb_ui_service_pending_enable();

    digit = fsbb_ui_digit_from_key(pressed_key);

    switch (pressed_key)
    {
    case FSBB_UI_KEY_SET:
        if (fsbb_ui_state == FSBB_UI_PARAM_SETTING)
            fsbb_ui_save_and_lock();
        else
            fsbb_ui_enter_setting_state();
        break;

    case FSBB_UI_KEY_ENABLE:
        if (g_fsbb_output_enabled)
        {
            fsbb_ui_enable_after_fault_reset = 0U;
            cia402_send_cmd(&cia402_sm, CIA402_CMD_DISABLE_VOLTAGE);
        }
        else
            fsbb_ui_request_output_enable();
        break;

    case FSBB_UI_KEY_MODE:
        fsbb_ui_switch_mode();
        break;

    default:
        if (fsbb_ui_state != FSBB_UI_PARAM_SETTING)
            break;

        switch (pressed_key)
        {
        case FSBB_UI_KEY_LEFT:
            fsbb_ui_cursor = (fsbb_ui_cursor == 0U) ? FSBB_UI_LAST_EDIT_CURSOR : (fsbb_ui_cursor - 1U);
            break;

        case FSBB_UI_KEY_RIGHT:
            fsbb_ui_cursor = (fsbb_ui_cursor >= FSBB_UI_LAST_EDIT_CURSOR) ? 0U : (fsbb_ui_cursor + 1U);
            break;

        case FSBB_UI_KEY_ADD:
            fsbb_ui_edit_value = fsbb_ui_step_digit(fsbb_ui_edit_value, fsbb_ui_cursor, 1);
            break;

        case FSBB_UI_KEY_SUB:
            fsbb_ui_edit_value = fsbb_ui_step_digit(fsbb_ui_edit_value, fsbb_ui_cursor, -1);
            break;

        default:
            if (digit >= 0)
            {
                fsbb_ui_edit_value = fsbb_ui_set_digit(fsbb_ui_edit_value, fsbb_ui_cursor, digit);
                fsbb_ui_cursor = (fsbb_ui_cursor >= FSBB_UI_LAST_EDIT_CURSOR) ? 0U : (fsbb_ui_cursor + 1U);
            }
            break;
        }
        break;
    }

    fsbb_ui_update_leds();
    fsbb_ui_render_7seg(dev);

    if (gmp_base_get_diff_system_tick(last_oled_tick) >= FSBB_UI_OLED_REFRESH_MS)
    {
        last_oled_tick = gmp_base_get_system_tick();
        fsbb_ui_render_oled();
    }

    return GMP_TASK_DONE;
}

gmp_task_status_t tsk_fsbb_ui_display(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;

    ht16k33_update_display(dev);
    return GMP_TASK_DONE;
}
