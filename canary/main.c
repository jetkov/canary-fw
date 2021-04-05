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
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "canary_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_libuarte_drv.h"
#include "nrf_queue.h"

#include "nrf_delay.h"
#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "Canary"                                /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define NOT_CONNECTED                   0xFFFFFFFF

BLE_LBS_DEF(m_lbs);                                                             /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/


NRF_LIBUARTE_DRV_DEFINE(libuarte, 0, 1);

APP_TIMER_DEF(m_sense_timer_id);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

#define PM_UART_BUFFER_LEN 64
static uint8_t pm_uarte_buffer[PM_UART_BUFFER_LEN];
static volatile uint8_t pm_uarte_rx_complete = 0;

static volatile uint8_t hvn_tx_complete_flag = 0;
static volatile uint8_t button_state_send_flag = 0xFF;

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

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

    err_code = app_timer_start(m_sense_timer_id, APP_TIMER_TICKS(5000), NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{CANARY_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_lbs_init_t     init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize LBS.
    //init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_on(ADVERTISING_LED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            bsp_board_led_on(CONNECTED_LED);
            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            hvn_tx_complete_flag = 1;
            //NRF_LOG_DEBUG("BLE_GATTS_EVT_HVN_TX_COMPLETE");

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
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
            //NRF_LOG_ERROR("Notification queue not big enough!");
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
        if (con_breaks[lo_idx] <= concentration && concentration < con_breaks[lo_idx+1])
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
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

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
            nrf_libuarte_drv_rx_start(&libuarte, pm_uarte_buffer, PM_UART_BUFFER_LEN, false);
            while (pm_uarte_rx_complete == 0);
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

                    sensor_val_pm_aqi = calculate_aqi_pm2_5(sensor_val_pm2_5);
                    
                    printf("PM1:   %d\r\n", sensor_val_pm1);
                    printf("PM2.5: %d\r\n", sensor_val_pm2_5);
                    printf("PM10:  %d\r\n", sensor_val_pm10);
                    printf("AQI:   %d\r\n", sensor_val_pm_aqi);

                    break;
                }
            }

            

            err_code = ble_canary_notify_uint16(m_conn_handle, &m_lbs, CANARY_UUID_PM1_CHAR,      sensor_val_pm1);
            handle_notification_error(err_code);
            nrf_delay_ms(50);    

            err_code = ble_canary_notify_uint16(m_conn_handle, &m_lbs, CANARY_UUID_PM2_5_CHAR,    sensor_val_pm2_5);
            handle_notification_error(err_code);
            nrf_delay_ms(50);    

            err_code = ble_canary_notify_uint16(m_conn_handle, &m_lbs, CANARY_UUID_PM10_CHAR,     sensor_val_pm10);
            handle_notification_error(err_code);
            nrf_delay_ms(50);        

            err_code = ble_canary_notify_uint16(m_conn_handle, &m_lbs, CANARY_UUID_VOC_IDX_CHAR,  0);//4 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, &m_lbs, CANARY_UUID_GAS_CHAR,      0);//5 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, &m_lbs, CANARY_UUID_TEMP_CHAR,     0);//6 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, &m_lbs, CANARY_UUID_HUMIDITY_CHAR, 0);//7 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, &m_lbs, CANARY_UUID_PM_AQI_CHAR,   sensor_val_pm_aqi);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, &m_lbs, CANARY_UUID_GAS_AQI_CHAR,  0);//8 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            err_code = ble_canary_notify_uint16(m_conn_handle, &m_lbs, CANARY_UUID_BATTERY_CHAR,  0);//9 + (counterthing % 5) * 100);
            handle_notification_error(err_code);
            nrf_delay_ms(50);   

            sense_flag = 0;
            counterthing++;
        }

        if (button_state_send_flag != 0xFF) 
        {
            NRF_LOG_INFO("Send button state change.");
            err_code = ble_canary_notify(m_conn_handle, &m_lbs, CANARY_UUID_BUTTON_CHAR, &button_state_send_flag, 1);
            handle_notification_error(err_code);

            button_state_send_flag = 0xFF;
        }

        idle_state_handle();
    }
}


/**
 * @}
 */
