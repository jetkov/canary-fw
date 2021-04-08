/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_gpio.h"

#include "nrf_libuarte_drv.h"
#include "nrf_queue.h"

#include "nrf_delay.h"
#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "canary_ble.h"

#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */


#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define NOT_CONNECTED                   0xFFFFFFFF

NRF_LIBUARTE_DRV_DEFINE(libuarte, 0, 1);

APP_TIMER_DEF(m_sense_timer_id);

#define PM_UART_BUFFER_LEN 64

static uint8_t pm_uarte_buffer[PM_UART_BUFFER_LEN];
static volatile uint8_t pm_uarte_rx_complete = 0;

static volatile uint8_t button_state_send_flag = 0xFF;

ble_lbs_t * p_lbs;
uint16_t m_conn_handle;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void uart_event_handler(void * context, nrf_libuarte_drv_evt_t * p_evt)
{

    nrf_libuarte_drv_t * p_libuarte = (nrf_libuarte_drv_t *)context;
    ret_code_t ret;

    switch (p_evt->type)
    {
        case NRF_LIBUARTE_DRV_EVT_RX_DATA:        ///< Data received.
            nrf_libuarte_drv_rx_stop(&libuarte);
            nrfx_timer_disable(&p_libuarte->timer);
            pm_uarte_rx_complete = 1;
            break;
        case NRF_LIBUARTE_DRV_EVT_RX_BUF_REQ:     ///< Requesting new buffer for receiving data.
            break;
        case NRF_LIBUARTE_DRV_EVT_TX_DONE:        ///< Requested TX transfer completed.
            break;
        case NRF_LIBUARTE_DRV_EVT_ERROR:          ///< Error reported by the UARTE peripheral.
            NRF_LOG_ERROR("NRF_LIBUARTE_DRV_EVT_ERROR")
            break;
        case NRF_LIBUARTE_DRV_EVT_OVERRUN_ERROR:  ///< Error reported by the driver.
            break;
    }
}

static void uart_init(void)
{
    ret_code_t err_code;

    nrf_libuarte_drv_config_t nrf_libuarte_drv_config = {
            .tx_pin        = TX_PIN_NUMBER,
            .rx_pin        = RX_PIN_NUMBER,
            .cts_pin       = NOT_CONNECTED,
            .rts_pin       = NOT_CONNECTED,
            .startrx_evt   = 0,
            .endrx_evt     = 0,
            .rxstarted_tsk = 0,
            .rxdone_tsk    = 0,
            .hwfc          = NRF_UARTE_HWFC_DISABLED,                           
            .parity        = NRF_UARTE_PARITY_EXCLUDED,
            .baudrate      = NRF_UARTE_BAUDRATE_9600,
            .irq_priority  = APP_IRQ_PRIORITY_LOW_MID,
            .pullup_rx     = 0
    };

    err_code = nrf_libuarte_drv_init(&libuarte, &nrf_libuarte_drv_config, uart_event_handler, (void *)&libuarte);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}

static volatile uint8_t sense_flag = 0;
static void sense_timer_handler(void * p_context)
{
    sense_flag = 1;
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers
    err_code = app_timer_create(&m_sense_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sense_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sense_timer_id, APP_TIMER_TICKS(10000), NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            button_state_send_flag = button_action;
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void handle_notification_error(ret_code_t err_code)
{
    if (err_code != NRF_SUCCESS &&
        err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
        err_code != NRF_ERROR_INVALID_STATE &&
        err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    {
        if (err_code == NRF_ERROR_RESOURCES)
        {
            NRF_LOG_ERROR("Notification queue not big enough!");
        }
        APP_ERROR_CHECK(err_code);
    }
}

uint16_t calculate_aqi(uint16_t * con_breaks, uint16_t * aqi_breaks, uint8_t num_breaks, uint16_t concentration)
{
    uint8_t lo_idx;
    uint16_t aqi_lo;

    for (lo_idx = 0; lo_idx < (num_breaks - 1); lo_idx++)
    {
        if ((con_breaks[lo_idx] <= concentration && concentration < con_breaks[lo_idx+1]) || ((lo_idx+1) == (num_breaks-1)) && (concentration >= con_breaks[num_breaks-1]))
        {
            aqi_lo = aqi_breaks[lo_idx];
            
            if (aqi_lo > 0)
            {
                aqi_lo += 1;
            }

            //printf("\r\nAQI_LO: %d\r\n", aqi_lo);
            //printf("AQI_HI: %d\r\n", aqi_breaks[lo_idx+1]);
            //printf("CON_LO: %d\r\n", con_breaks[lo_idx]);
            //printf("CON_HI: %d\r\n", con_breaks[lo_idx+1]);

            return ((float)(aqi_breaks[lo_idx+1] - aqi_breaks[lo_idx]))/((float)(con_breaks[lo_idx+1] - con_breaks[lo_idx])) * ((float)(concentration - con_breaks[lo_idx])) + ((float)aqi_breaks[lo_idx]);
        }
    }

}

uint16_t calculate_aqi_pm2_5(uint16_t pm2_5)
{
    uint16_t con_breaks[7] = {0, 12,  35,  55, 150, 250, 500};
    uint16_t aqi_breaks[7] = {0, 50, 100, 150, 200, 300, 500};

    return calculate_aqi(con_breaks, aqi_breaks, 7, pm2_5);
}

#define PM_UART_HEAD_1        (0x42)
#define PM_UART_HEAD_2        (0x4D)
#define PM_UART_DATA_LEN      (2*13 + 2)
#define PM_UART_FRAME_LEN     (32)

#define PM_UART_IDX_HEAD_1    (0)
#define PM_UART_IDX_HEAD_2    (1)
#define PM_UART_IDX_H_LENGTH  (2)
#define PM_UART_IDX_L_LENGTH  (3)

#define PM_UART_IDX_H_D1      (4) // PM1
#define PM_UART_IDX_L_D1      (5) // PM1
#define PM_UART_IDX_H_D2      (6) // PM2.5
#define PM_UART_IDX_L_D2      (7) // PM2.5
#define PM_UART_IDX_H_D3      (8) // PM10
#define PM_UART_IDX_L_D3      (9) // PM10

#define PM_UART_IDX_L_D13     (29) // Error [0b0ABCDEF] [C = 1 High temp alarm, D = 1 Low temp alarm, E = 1 Fan error]

#define PM_UART_IDX_H_CS      (30) // Checksum [CS = HEAD1 + HEAD2 + ... + L_D]
#define PM_UART_IDX_L_CS      (31) // Checksum

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    ret_code_t ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
  
    nrf_drv_clock_lfclk_request(NULL);

    log_init();
    leds_init();
    timers_init();
    buttons_init();
    uart_init();
    power_management_init();
    ble_stack_init();

    p_lbs = get_p_lbs();
    m_conn_handle = get_conn_handle();

    nrf_gpio_cfg(N_PM_SLEEP,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0D1,
                 NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg(N_PM_RST,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0D1,
                 NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg_output(EN_5V);

    nrf_gpio_pin_write(N_PM_SLEEP, 1);
    nrf_gpio_pin_write(N_PM_RST, 1);
    nrf_gpio_pin_write(EN_5V, 1);

    // Cannot leave floating!
    nrf_gpio_cfg_output(PIEZO_PWM);
    nrf_gpio_pin_write(PIEZO_PWM, 0);

    // Start execution.
    NRF_LOG_INFO("===== CANARY STARTED =====");
    advertising_start();

    int counterthing = 0;

    uint16_t sensor_val_pm1;
    uint16_t sensor_val_pm2_5;
    uint16_t sensor_val_pm10;
    uint16_t sensor_val_pm_aqi;

    // Enter main loop.
    while(true)
    {
        ret_code_t err_code;       

        if (sense_flag == 1)
        { 
            //nrf_gpio_pin_write(N_PM_SLEEP, 1);
            //nrf_gpio_pin_write(EN_5V, 1);
            //nrf_delay_ms(200);
            //nrf_gpio_pin_write(N_PM_RST, 1);
            //nrf_delay_ms(5000);
            //printf("5V Enabled\r\n");

            nrf_libuarte_drv_rx_start(&libuarte, pm_uarte_buffer, PM_UART_BUFFER_LEN, false);

            uint8_t timeout_counter = 0;
            while (pm_uarte_rx_complete == 0)
            {
                timeout_counter++;

                if (timeout_counter > 200) 
                {
                    NRF_LOG_WARNING("PM UART timed out!");
                    break;
                }
            }
            pm_uarte_rx_complete == 0;

            printf("\r\n");
            for (int i = 0; i < PM_UART_BUFFER_LEN; i++) {
                printf("%02X ", pm_uarte_buffer[i]);
            }
            printf("\r\n");
            
            for (int i = 0; i < PM_UART_BUFFER_LEN - PM_UART_FRAME_LEN; i++) {
                if (pm_uarte_buffer[i] == PM_UART_HEAD_1 && pm_uarte_buffer[i+1] == PM_UART_HEAD_2)
                {
                    uint16_t calc_checksum = 0;
                    for (int j = 0; j < (i + PM_UART_FRAME_LEN - 2); j++)
                    {
                        calc_checksum += pm_uarte_buffer[j];
                    }
                    
                    uint16_t rx_checksum = (pm_uarte_buffer[i+PM_UART_IDX_H_CS] << 8) | pm_uarte_buffer[i+PM_UART_IDX_L_CS];

                    printf("Calculated checksum: %04X \r\n", calc_checksum);
                    printf("Recieved   checksum: %04X \r\n", rx_checksum);

                    if (calc_checksum != rx_checksum)
                    {
                        NRF_LOG_WARNING("PM UART checksum failure!");
                        break;
                    }

                    if (pm_uarte_buffer[i+PM_UART_IDX_L_D13] != 0)
                    {
                        NRF_LOG_ERROR("PM SENSOR ERROR! [%02X]", pm_uarte_buffer[i+PM_UART_IDX_L_D13]);
                    }

                    sensor_val_pm1   = pm_uarte_buffer[i+PM_UART_IDX_H_D1]*256 + pm_uarte_buffer[i+PM_UART_IDX_L_D1];
                    sensor_val_pm2_5 = pm_uarte_buffer[i+PM_UART_IDX_H_D2]*256 + pm_uarte_buffer[i+PM_UART_IDX_L_D2];
                    sensor_val_pm10  = pm_uarte_buffer[i+PM_UART_IDX_H_D3]*256 + pm_uarte_buffer[i+PM_UART_IDX_L_D3];

                    //sensor_val_pm1   = 1+ (counterthing % 20) * 50;
                    //sensor_val_pm2_5 = 2+ (counterthing % 20) * 50;
                    //sensor_val_pm10  = 3+ (counterthing % 20) * 50;

                    sensor_val_pm_aqi = calculate_aqi_pm2_5(sensor_val_pm2_5);
                    
                    printf("PM1:   %d\r\n", sensor_val_pm1);
                    printf("PM2.5: %d\r\n", sensor_val_pm2_5);
                    printf("PM10:  %d\r\n", sensor_val_pm10);
                    printf("AQI:   %d\r\n", sensor_val_pm_aqi);

                    break;
                }
            }

            //nrf_gpio_pin_write(EN_5V, 0);
            //nrf_gpio_pin_write(N_PM_SLEEP, 0);
            //nrf_gpio_pin_write(N_PM_RST, 0);
            //printf("5V Disabled\r\n");

            err_code = ble_canary_notify_uint16(m_conn_handle, p_lbs, CANARY_UUID_PM1_CHAR,      sensor_val_pm1);
            handle_notification_error(err_code);
            nrf_delay_ms(50);    

            err_code = ble_canary_notify_uint16(m_conn_handle, p_lbs, CANARY_UUID_PM2_5_CHAR,    sensor_val_pm2_5);
            handle_notification_error(err_code);
            nrf_delay_ms(50);    

            err_code = ble_canary_notify_uint16(m_conn_handle, p_lbs, CANARY_UUID_PM10_CHAR,     sensor_val_pm10);
            handle_notification_error(err_code);
            nrf_delay_ms(50);        

            err_code = ble_canary_notify_uint16(m_conn_handle, p_lbs, CANARY_UUID_VOC_IDX_CHAR,  0);//4 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, p_lbs, CANARY_UUID_GAS_CHAR,      0);//5 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, p_lbs, CANARY_UUID_TEMP_CHAR,     0);//6 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, p_lbs, CANARY_UUID_HUMIDITY_CHAR, 0);//7 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, p_lbs, CANARY_UUID_PM_AQI_CHAR,   sensor_val_pm_aqi);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, p_lbs, CANARY_UUID_GAS_AQI_CHAR,  0);//8 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, p_lbs, CANARY_UUID_BATTERY_CHAR,  0);//9 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            sense_flag = 0;
            counterthing++;
        }

        //if (button_state_send_flag != 0xFF) 
        //{
        //    NRF_LOG_INFO("Send button state change.");
        //    err_code = ble_canary_notify(m_conn_handle, p_lbs, CANARY_UUID_BUTTON_CHAR, &button_state_send_flag, 1);
        //    handle_notification_error(err_code);

        //    button_state_send_flag = 0xFF;
        //}

        idle_state_handle();
    }
}


/**
 * @}
 */
