#include <gmp_core.h>

#include "ctl_main.h"
#include "user_main.h"

#include <core/dev/display/ht16k33.h>
#include <xplt.ctl_interface.h>

#define INV_UI_KEY_RIGHT             1U
#define INV_UI_KEY_LEFT              2U
#define INV_UI_KEY_ENABLE            8U
#define INV_UI_KEY_SET               21U

#define INV_UI_KEY_PRESS_COUNT       2U
#define INV_UI_KEY_RELEASE_COUNT     2U

#define INV_UI_DIGIT_COUNT           8U
#define INV_UI_FREQ_DIGITS           3U
#define INV_UI_LAST_EDIT_CURSOR      (INV_UI_FREQ_DIGITS - 1U)
#define INV_UI_FREQ_DISPLAY_OFFSET   4U
#define INV_UI_BLINK_MS              250U

#define INV_UI_FREQ_MIN_DECI_HZ      300L
#define INV_UI_FREQ_MAX_DECI_HZ      600L
#define INV_UI_FREQ_DEFAULT_DECI_HZ  600L

#define INV_UI_DECIMAL_POINT_RAM     8U
#define INV_UI_DECIMAL_POINT_BIT     0x80U

#define INV_UI_LED7_GPIO             59U
#define INV_UI_LED_ACTIVE_LEVEL      0U
#define INV_UI_LED_INACTIVE_LEVEL    1U

#define INV_UI_SEG_BLANK             0x00U
#define INV_UI_SEG_0                 0x3FU
#define INV_UI_SEG_1                 0x06U

typedef enum
{
    INV_UI_PARAM_LOCKED = 0U,
    INV_UI_PARAM_EDIT = 1U
} inv_ui_param_state_t;

static const uint16_t inv_ui_led_lut[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

static const uint8_t inv_ui_digit_ram_map[INV_UI_DIGIT_COUNT] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U
};

static uint8_t inv_ui_initialized = 0U;
static uint8_t inv_ui_output_request = 0U;
static inv_ui_param_state_t inv_ui_param_state = INV_UI_PARAM_LOCKED;
static uint8_t inv_ui_cursor = 0U;
static int32_t inv_ui_freq_deci_hz = INV_UI_FREQ_DEFAULT_DECI_HZ;
static int32_t inv_ui_edit_freq_deci_hz = INV_UI_FREQ_DEFAULT_DECI_HZ;

volatile uint16_t g_inv_ui_raw_key = 0U;
volatile uint16_t g_inv_ui_pressed_key = 0U;
volatile uint16_t g_inv_ui_edit_mode = 0U;
volatile uint16_t g_inv_ui_output_request = 0U;
volatile uint16_t g_inv_ui_last_iic_ec = 0U;
volatile uint16_t g_inv_ui_debounced_key = 0U;
volatile uint32_t g_inv_ui_key_event_count = 0U;
volatile uint32_t g_inv_ui_key8_count = 0U;
volatile int32_t g_inv_ui_freq_deci_hz = INV_UI_FREQ_DEFAULT_DECI_HZ;

static int32_t inv_ui_clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value)
        value = min_value;
    if (value > max_value)
        value = max_value;

    return value;
}

static int32_t inv_ui_clamp_freq(int32_t value)
{
    return inv_ui_clamp_i32(value, INV_UI_FREQ_MIN_DECI_HZ, INV_UI_FREQ_MAX_DECI_HZ);
}

static int32_t inv_ui_digit_weight(uint8_t cursor)
{
    static const int32_t weights[INV_UI_FREQ_DIGITS] = {100L, 10L, 1L};

    if (cursor >= INV_UI_FREQ_DIGITS)
    {
        return 1L;
    }

    return weights[cursor];
}

static int32_t inv_ui_set_digit(int32_t value, uint8_t cursor, int8_t digit)
{
    int32_t weight = inv_ui_digit_weight(cursor);

    value -= ((value / weight) % 10L) * weight;
    value += (int32_t)digit * weight;

    return inv_ui_clamp_freq(value);
}

static int8_t inv_ui_digit_from_key(fast_gt key_id)
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

static uint8_t inv_ui_is_supported_key(fast_gt key_id)
{
    if ((key_id == INV_UI_KEY_RIGHT) || (key_id == INV_UI_KEY_LEFT) ||
        (key_id == INV_UI_KEY_ENABLE) || (key_id == INV_UI_KEY_SET))
    {
        return 1U;
    }

    return (inv_ui_digit_from_key(key_id) >= 0) ? 1U : 0U;
}

static ec_gt inv_ui_read_raw_key(ht16k33_dev_t* dev, fast_gt* key_id_ret)
{
    return ht16k33_read_keys(dev, key_id_ret);
}

static fast_gt inv_ui_filter_key_event(fast_gt raw_key)
{
    static fast_gt last_key_id = 0;
    fast_gt pressed_key = 0;

    if (raw_key != last_key_id)
    {
        if (raw_key != 0)
            pressed_key = raw_key;
        last_key_id = raw_key;
    }

    g_inv_ui_debounced_key = (uint16_t)last_key_id;
    return pressed_key;
}

static void inv_ui_apply_frequency_setpoint(void)
{
    parameter_gt freq_hz;

    inv_ui_freq_deci_hz = inv_ui_clamp_freq(inv_ui_freq_deci_hz);
    freq_hz = (parameter_gt)((float)inv_ui_freq_deci_hz / 10.0f);

    gfl_init.freq_base = freq_hz;
    dq_hcm_init.freq_base = freq_hz;
    gfl_neg_init.freq_base = freq_hz;
    gfl_neg_init.seq_filter_fc = freq_hz / 2.5f;

    ctl_set_ramp_generator_slope(&inv_ctrl.rg, float2ctrl(freq_hz / CONTROLLER_FREQUENCY));
    inv_ctrl.coef_ff_decouple =
        CTL_PARAM_CONST_2PI * gfl_init.grid_filter_L * freq_hz * gfl_init.i_base / gfl_init.v_base;

    ctl_update_dq_hcm_freq(&dq_hcm, &dq_hcm_init);
    ctl_update_neg_inv_coeff(&neg_current_ctrl, &gfl_neg_init);
}

static void inv_ui_update_leds(void)
{
    GPIO_WritePin(INV_UI_LED7_GPIO,
                  cia402_sm.state_word.bits.operation_enabled ?
                  INV_UI_LED_ACTIVE_LEVEL : INV_UI_LED_INACTIVE_LEVEL);
}

static void inv_ui_set_display_ram(ht16k33_dev_t* dev, const uint16_t segs[INV_UI_DIGIT_COUNT])
{
    uint8_t i;

    for (i = 0U; i < HT16K33_CFG_DISP_RAM_SIZE; i++)
        dev->display_ram[i] = 0U;

    for (i = 0U; i < INV_UI_DIGIT_COUNT; i++)
        dev->display_ram[inv_ui_digit_ram_map[i] * 2U] = (data_gt)segs[i];

    dev->display_ram[INV_UI_DECIMAL_POINT_RAM] |= INV_UI_DECIMAL_POINT_BIT;

    dev->is_dirty = 1;
}

static void inv_ui_render_freq(uint16_t* segs, int32_t freq_deci_hz)
{
    int32_t value = inv_ui_clamp_freq(freq_deci_hz);
    uint8_t offset = INV_UI_FREQ_DISPLAY_OFFSET;

    segs[offset + 0U] = inv_ui_led_lut[(value / 100L) % 10L];
    segs[offset + 1U] = inv_ui_led_lut[(value / 10L) % 10L];
    segs[offset + 2U] = inv_ui_led_lut[value % 10L];
    segs[offset + 3U] = INV_UI_SEG_BLANK;
}

static void inv_ui_render_7seg(ht16k33_dev_t* dev)
{
    uint16_t segs[INV_UI_DIGIT_COUNT] = {
        INV_UI_SEG_BLANK, INV_UI_SEG_BLANK, INV_UI_SEG_BLANK, INV_UI_SEG_BLANK,
        INV_UI_SEG_BLANK, INV_UI_SEG_BLANK, INV_UI_SEG_BLANK, INV_UI_SEG_BLANK
    };
    uint8_t blink_on = (uint8_t)(((gmp_base_get_system_tick() / INV_UI_BLINK_MS) & 0x01U) == 0U);
    int32_t display_freq = (inv_ui_param_state == INV_UI_PARAM_EDIT) ? inv_ui_edit_freq_deci_hz : inv_ui_freq_deci_hz;

    segs[0] = cia402_sm.state_word.bits.operation_enabled ? INV_UI_SEG_1 : INV_UI_SEG_0;
    inv_ui_render_freq(segs, display_freq);

    if ((inv_ui_param_state == INV_UI_PARAM_EDIT) && !blink_on)
    {
        uint8_t active_digit = (uint8_t)(INV_UI_FREQ_DISPLAY_OFFSET + inv_ui_cursor);
        segs[active_digit] = INV_UI_SEG_BLANK;
    }

    inv_ui_set_display_ram(dev, segs);
}

static void inv_ui_request_output_enable(void)
{
    uint8_t already_enabled = (cia402_sm.current_state == CIA402_SM_OPERATION_ENABLED) &&
                              cia402_sm.state_word.bits.operation_enabled;

    cia402_sm.flag_enable_control_word = 0;
    cia402_sm.current_cmd = CIA402_CMD_ENABLE_OPERATION;
    cia402_sm.current_state = CIA402_SM_OPERATION_ENABLED;
    cia402_sm.state_word.bits.operation_enabled = 1U;
    cia402_sm.state_word.bits.ready_to_switch_on = 1U;
    cia402_sm.state_word.bits.switched_on = 1U;
    cia402_sm.state_word.bits.switch_on_disabled = 0U;

    if (!already_enabled)
    {
        cia402_sm.entry_state_tick = gmp_base_get_ctrl_tick();
        cia402_sm.current_state_counter = 2U;
        ctl_enable_pwm();
    }
}

static void inv_ui_request_output_disable(void)
{
    cia402_sm.flag_enable_control_word = 0;
    ctl_disable_pwm();
    cia402_sm.current_cmd = CIA402_CMD_DISABLE_OPERATION;
    cia402_sm.current_state = CIA402_SM_SWITCHED_ON;
    cia402_sm.current_state_counter = 0U;
    cia402_sm.entry_state_tick = gmp_base_get_ctrl_tick();
    cia402_sm.state_word.bits.operation_enabled = 0U;
    cia402_sm.state_word.bits.ready_to_switch_on = 1U;
    cia402_sm.state_word.bits.switched_on = 1U;
    cia402_sm.state_word.bits.switch_on_disabled = 0U;
}

static void inv_ui_toggle_output(void)
{
    if (inv_ui_output_request || cia402_sm.state_word.bits.operation_enabled)
    {
        inv_ui_output_request = 0U;
        inv_ui_request_output_disable();
    }
    else
    {
        inv_ui_output_request = 1U;
        inv_ui_request_output_enable();
    }
}

static void inv_ui_service_output_request(void)
{
    cia402_sm.flag_enable_control_word = 0;

    if (inv_ui_output_request)
    {
        inv_ui_request_output_enable();
    }
    else if (cia402_sm.state_word.bits.operation_enabled)
    {
        inv_ui_request_output_disable();
    }

    g_inv_ui_output_request = inv_ui_output_request;
}

fast_gt inv_ui_is_forced_output_active(void)
{
    return inv_ui_output_request ? 1 : 0;
}

void inv_ui_service_forced_output(void)
{
    if (inv_ui_output_request)
    {
        inv_ui_request_output_enable();
    }
}

static void inv_ui_enter_edit(void)
{
    inv_ui_edit_freq_deci_hz = inv_ui_freq_deci_hz;
    inv_ui_cursor = 0U;
    inv_ui_param_state = INV_UI_PARAM_EDIT;
}

static void inv_ui_save_and_lock(void)
{
    inv_ui_freq_deci_hz = inv_ui_clamp_freq(inv_ui_edit_freq_deci_hz);
    inv_ui_apply_frequency_setpoint();
    inv_ui_param_state = INV_UI_PARAM_LOCKED;
}

void inv_ui_init(void)
{
    inv_ui_freq_deci_hz = inv_ui_clamp_freq((int32_t)(gfl_init.freq_base * 10.0f + 0.5f));
    inv_ui_edit_freq_deci_hz = inv_ui_freq_deci_hz;
    inv_ui_output_request = cia402_sm.state_word.bits.operation_enabled ? 1U : 0U;
    inv_ui_param_state = INV_UI_PARAM_LOCKED;
    inv_ui_cursor = 0U;
    inv_ui_apply_frequency_setpoint();
    inv_ui_update_leds();
    g_inv_ui_edit_mode = 0U;
    g_inv_ui_output_request = inv_ui_output_request;
    g_inv_ui_freq_deci_hz = inv_ui_freq_deci_hz;
    inv_ui_initialized = 1U;
}

gmp_task_status_t tsk_inv_ui_key(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;
    fast_gt key_id = 0;
    fast_gt pressed_key;
    int8_t digit;
    ec_gt ec;

    if (!inv_ui_initialized)
        inv_ui_init();

    ec = inv_ui_read_raw_key(dev, &key_id);
    g_inv_ui_last_iic_ec = (uint16_t)ec;
    g_inv_ui_raw_key = (uint16_t)key_id;

    if (ec != GMP_EC_OK)
    {
        inv_ui_render_7seg(dev);
        return GMP_TASK_DONE;
    }

    if (!inv_ui_is_supported_key(key_id))
        key_id = 0;

    pressed_key = inv_ui_filter_key_event(key_id);
    g_inv_ui_pressed_key = (uint16_t)pressed_key;
    if (pressed_key != 0)
        g_inv_ui_key_event_count++;
    digit = inv_ui_digit_from_key(pressed_key);

    switch (pressed_key)
    {
    case INV_UI_KEY_SET:
        if (inv_ui_param_state == INV_UI_PARAM_EDIT)
            inv_ui_save_and_lock();
        else
            inv_ui_enter_edit();
        break;

    case INV_UI_KEY_ENABLE:
        g_inv_ui_key8_count++;
        inv_ui_toggle_output();
        break;

    default:
        if (inv_ui_param_state != INV_UI_PARAM_EDIT)
            break;

        if (pressed_key == INV_UI_KEY_LEFT)
            inv_ui_cursor = (inv_ui_cursor == 0U) ? INV_UI_LAST_EDIT_CURSOR : (inv_ui_cursor - 1U);
        else if (pressed_key == INV_UI_KEY_RIGHT)
            inv_ui_cursor = (inv_ui_cursor >= INV_UI_LAST_EDIT_CURSOR) ? 0U : (inv_ui_cursor + 1U);
        else if (digit >= 0)
            inv_ui_edit_freq_deci_hz = inv_ui_set_digit(inv_ui_edit_freq_deci_hz, inv_ui_cursor, digit);
        break;
    }

    inv_ui_service_output_request();
    inv_ui_update_leds();
    inv_ui_render_7seg(dev);
    g_inv_ui_edit_mode = (uint16_t)inv_ui_param_state;
    g_inv_ui_freq_deci_hz = inv_ui_freq_deci_hz;

    return GMP_TASK_DONE;
}

gmp_task_status_t tsk_inv_ui_display(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;

    ht16k33_update_display(dev);
    return GMP_TASK_DONE;
}
