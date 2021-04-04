/**
 * Copyright (c) 2013 - 2020, Nordic Semiconductor ASA
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
#include "canary_ble.h"

#include "sdk_common.h"
#include "ble_srv_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"


/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_lbs_t * p_lbs, ble_evt_t const * p_ble_evt)
{
    //ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    //if (   (p_evt_write->handle == p_lbs->led_char_handles.value_handle)
    //    && (p_evt_write->len == 1)
    //    && (p_lbs->led_write_handler != NULL))
    //{
    //    p_lbs->led_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_lbs, p_evt_write->data[0]);
    //}
}

ble_gatts_char_handles_t * get_canary_uuid_handle(ble_lbs_t * p_lbs, uint16_t uuid) {
    switch(uuid)
    {
        case CANARY_UUID_BUTTON_CHAR:
            return &p_lbs->canary_button_char_handles;
            break;
        case CANARY_UUID_PM1_CHAR:
            return &p_lbs->canary_pm1_char_handles;
            break;
        case CANARY_UUID_PM2_5_CHAR:
            return &p_lbs->canary_pm2_5_char_handles;
            break;
        case CANARY_UUID_PM10_CHAR:
            return &p_lbs->canary_pm10_char_handles;
            break;
        case CANARY_UUID_VOC_IDX_CHAR:
            return &p_lbs->canary_voc_idx_char_handles;
            break;
        case CANARY_UUID_GAS_CHAR:
            return &p_lbs->canary_gas_char_handles;
            break;
        case CANARY_UUID_TEMP_CHAR:
            return &p_lbs->canary_temp_char_handles;
            break;
        case CANARY_UUID_HUMIDITY_CHAR:
            return &p_lbs->canary_humidity_char_handles;
            break;
        case CANARY_UUID_PM_AQI_CHAR:
            return &p_lbs->canary_pm_aqi_char_handles;
            break;
        case CANARY_UUID_GAS_AQI_CHAR:
            return &p_lbs->canary_gas_aqi_char_handles;
            break;
        case CANARY_UUID_BATTERY_CHAR:
            return &p_lbs->canary_battery_char_handles;
            break;
    }
}


void ble_lbs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_lbs_t * p_lbs = (ble_lbs_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_lbs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_lbs_init(ble_lbs_t * p_lbs, const ble_lbs_init_t * p_lbs_init)
{
    ret_code_t              err_code;
    ble_uuid_t              ble_uuid;
    ble_add_char_params_t   add_char_params;

    // Initialize service structure.

    // Add service.
    ble_uuid128_t base_uuid = {CANARY_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_lbs->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = CANARY_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lbs->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add Canary sensor characteristics.
    for (uint16_t sensor_char_uuid = CANARY_UUID_BUTTON_CHAR; sensor_char_uuid <= CANARY_UUID_BATTERY_CHAR; sensor_char_uuid++) {
      memset(&add_char_params, 0, sizeof(add_char_params));
      add_char_params.uuid              = sensor_char_uuid;
      add_char_params.uuid_type         = p_lbs->uuid_type;
      add_char_params.init_len          = sizeof(uint16_t);
      add_char_params.max_len           = sizeof(uint16_t);
      add_char_params.char_props.read   = 1;
      add_char_params.char_props.notify = 1;

      add_char_params.read_access       = SEC_OPEN;
      add_char_params.cccd_write_access = SEC_OPEN;

      err_code = characteristic_add(p_lbs->service_handle,
                                    &add_char_params,
                                    get_canary_uuid_handle(p_lbs, sensor_char_uuid));
      
      if (err_code != NRF_SUCCESS)
      {
          return err_code;
      }
    }

    return err_code;
}

uint32_t ble_canary_notify(uint16_t conn_handle, ble_lbs_t * p_lbs, uint16_t uuid, uint8_t * p_data, uint16_t len)
{
    ret_code_t err_code;
    uint8_t retry_counter = 0;

    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type   = BLE_GATT_HVX_NOTIFICATION;
    params.handle = get_canary_uuid_handle(p_lbs, uuid)->value_handle;
    params.p_data = p_data;
    params.p_len  = &len;

    do 
    {
        err_code = sd_ble_gatts_hvx(conn_handle, &params);
        retry_counter++;

        if (err_code == NRF_ERROR_RESOURCES)
        {
            NRF_LOG_WARNING("Notification queue full! Retrying... %d", retry_counter);
            nrf_delay_ms(50);
        }
    } while (err_code == NRF_ERROR_RESOURCES && retry_counter < 10);

    return err_code;
}

uint32_t ble_canary_notify_uint16(uint16_t conn_handle, ble_lbs_t * p_lbs, uint16_t uuid, uint16_t data)
{
    uint8_t dataArray[2];
    dataArray[0] = data & 0xff;
    dataArray[1] = (data >> 8);

    return ble_canary_notify(conn_handle, p_lbs, uuid, dataArray, 2);
}

//#endif // NRF_MODULE_ENABLED(BLE_LBS)
