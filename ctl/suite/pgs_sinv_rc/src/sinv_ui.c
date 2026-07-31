#include <gmp_core.h>

#include "ctl_main.h"
#include "user_main.h"

#include <xplt.ctl_interface.h>
#include <oled_driver.h>
#include <stdio.h>

#define SINV_UI_OLED_REFRESH_MS       250U
#define SINV_UI_KEY_ENABLE            8U
#define SINV_UI_KEY_PRESS_COUNT       1U
#define SINV_UI_KEY_RELEASE_COUNT     2U
#define SINV_UI_DIGIT_COUNT           8U

#define SINV_UI_LED8_GPIO             61U
#define SINV_UI_LED7_GPIO             59U
#define SINV_UI_LED_ACTIVE_LEVEL      0U
#define SINV_UI_LED_INACTIVE_LEVEL    1U

#define SINV_UI_SEG_BLANK             0x00U
#define SINV_UI_SEG_0                 0x3FU
#define SINV_UI_SEG_1                 0x06U

static uint8_t sinv_ui_initialized = 0U;
static uint8_t sinv_ui_output_request = 0U;

volatile uint16_t g_sinv_ui_raw_key = 0U;
volatile uint16_t g_sinv_ui_pressed_key = 0U;
volatile uint16_t g_sinv_ui_output_request = 0U;
volatile uint32_t g_sinv_ui_key8_count = 0U;
volatile uint16_t g_sinv_ui_last_iic_ec = 0U;

static ec_gt sinv_ui_read_raw_key(ht16k33_dev_t* dev, fast_gt* key_id_ret)
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

static fast_gt sinv_ui_filter_key_event(fast_gt raw_key)
{
    static fast_gt stable_key = 0;
    static fast_gt candidate_key = 0;
    static uint8_t candidate_count = 0U;
    static uint8_t release_count = 0U;

    if (stable_key != 0)
    {
        if (raw_key == 0)
        {
            if (release_count < SINV_UI_KEY_RELEASE_COUNT)
                release_count++;
            if (release_count >= SINV_UI_KEY_RELEASE_COUNT)
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
        if (candidate_count < SINV_UI_KEY_PRESS_COUNT)
            candidate_count++;
    }
    else
    {
        candidate_key = raw_key;
        candidate_count = 1U;
    }

    if (candidate_count >= SINV_UI_KEY_PRESS_COUNT)
    {
        stable_key = candidate_key;
        candidate_key = 0;
        candidate_count = 0U;
        return stable_key;
    }

    return 0;
}

static void sinv_ui_update_leds(void)
{
    GPIO_WritePin(SINV_UI_LED8_GPIO, SINV_UI_LED_INACTIVE_LEVEL);
    GPIO_WritePin(SINV_UI_LED7_GPIO,
                  cia402_sm.state_word.bits.operation_enabled ?
                  SINV_UI_LED_ACTIVE_LEVEL : SINV_UI_LED_INACTIVE_LEVEL);
}

static void sinv_ui_set_display_ram(ht16k33_dev_t* dev, const uint16_t segs[SINV_UI_DIGIT_COUNT])
{
    uint8_t i;

    for (i = 0U; i < HT16K33_CFG_DISP_RAM_SIZE; i++)
        dev->display_ram[i] = 0U;

    for (i = 0U; i < SINV_UI_DIGIT_COUNT; i++)
        dev->display_ram[i * 2U] = (data_gt)segs[i];

    dev->is_dirty = 1;
}

static void sinv_ui_render_7seg(ht16k33_dev_t* dev)
{
    uint16_t segs[SINV_UI_DIGIT_COUNT] = {
        SINV_UI_SEG_BLANK, SINV_UI_SEG_BLANK, SINV_UI_SEG_BLANK, SINV_UI_SEG_BLANK,
        SINV_UI_SEG_BLANK, SINV_UI_SEG_BLANK, SINV_UI_SEG_BLANK, SINV_UI_SEG_BLANK
    };

    if (cia402_sm.state_word.bits.operation_enabled)
    {
        segs[0] = SINV_UI_SEG_1;
    }
    else
    {
        segs[0] = SINV_UI_SEG_0;
    }

    sinv_ui_set_display_ram(dev, segs);
}

static int32_t sinv_ui_vbus_to_deci_volt(ctrl_gt value_pu)
{
    float value_v = (float)value_pu * CTRL_VOLTAGE_BASE;

    if (value_v < 0.0f)
        value_v = 0.0f;

    return (int32_t)(value_v * 10.0f + 0.5f);
}

static void sinv_ui_render_oled(void)
{
    char str_buf[32];
    int32_t vbus_dv = sinv_ui_vbus_to_deci_volt(adc_v_bus.control_port.value);

    sprintf(str_buf, "Vbus: %3ld.%1ld V  ", vbus_dv / 10L, vbus_dv % 10L);
    oled_show_str(0, 0, str_buf);
}

static void sinv_ui_request_output_enable(void)
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
        ctl_enable_pwm();
        cia402_sm.current_state_counter = 2U;
    }
}

static void sinv_ui_request_output_disable(void)
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

static void sinv_ui_toggle_output(void)
{
    if (sinv_ui_output_request || cia402_sm.state_word.bits.operation_enabled)
    {
        sinv_ui_output_request = 0U;
        sinv_ui_request_output_disable();
    }
    else
    {
        sinv_ui_output_request = 1U;
        sinv_ui_request_output_enable();
    }
}

static void sinv_ui_service_output_request(void)
{
    cia402_sm.flag_enable_control_word = 0;

    if (sinv_ui_output_request)
    {
        sinv_ui_request_output_enable();
    }
    else if (cia402_sm.state_word.bits.operation_enabled)
    {
        sinv_ui_request_output_disable();
    }

    g_sinv_ui_output_request = sinv_ui_output_request;
}

void sinv_ui_init(void)
{
    sinv_ui_output_request = cia402_sm.state_word.bits.operation_enabled ? 1U : 0U;
    g_sinv_ui_output_request = sinv_ui_output_request;
    sinv_ui_update_leds();
    sinv_ui_initialized = 1U;
}

gmp_task_status_t tsk_sinv_ui_key(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;
    fast_gt key_id = 0;
    fast_gt pressed_key;
    ec_gt ec;
    static uint32_t last_oled_tick = 0U;

    if (!sinv_ui_initialized)
        sinv_ui_init();

    ec = sinv_ui_read_raw_key(dev, &key_id);
    g_sinv_ui_last_iic_ec = (uint16_t)ec;
    g_sinv_ui_raw_key = (uint16_t)key_id;

    if (ec != GMP_EC_OK)
    {
        sinv_ui_render_7seg(dev);
        return GMP_TASK_DONE;
    }

    pressed_key = sinv_ui_filter_key_event(key_id);
    g_sinv_ui_pressed_key = (uint16_t)pressed_key;

    if (pressed_key == SINV_UI_KEY_ENABLE)
    {
        g_sinv_ui_key8_count++;
        sinv_ui_toggle_output();
    }

    sinv_ui_service_output_request();

    sinv_ui_update_leds();
    sinv_ui_render_7seg(dev);

    if (gmp_base_get_diff_system_tick(last_oled_tick) >= SINV_UI_OLED_REFRESH_MS)
    {
        last_oled_tick = gmp_base_get_system_tick();
        sinv_ui_render_oled();
    }

    return GMP_TASK_DONE;
}

gmp_task_status_t tsk_sinv_ui_display(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;

    ht16k33_update_display(dev);
    return GMP_TASK_DONE;
}
