/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_saadc.h"
#include "nrfx_twim.h"

APP_TIMER_DEF(m_delay_timer_id);                                        /**< Delay timer. */


bool m_sensor_data_updated = false;
int16_t m_sensor_data_buffer[7] = { 0 };
int16_t m_sensor_data_length = 1;

void battery_update_callback(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        int16_t voltage = 3600 * p_event->data.done.p_buffer[0] / 4096;
        NRF_LOG_INFO("Battery updated: %d.%d", voltage / 1000, voltage % 1000);
        m_sensor_data_buffer[0] = voltage;
    }
    nrfx_saadc_uninit();
}

void battery_update(void)
{
    ret_code_t err_code;

    nrfx_saadc_config_t saadc_cfg = NRFX_SAADC_DEFAULT_CONFIG;
    saadc_cfg.oversample = NRF_SAADC_OVERSAMPLE_4X;
    saadc_cfg.resolution = NRF_SAADC_RESOLUTION_12BIT;
    saadc_cfg.low_power_mode = true;
    err_code = nrfx_saadc_init(&saadc_cfg, battery_update_callback);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t channel_cfg = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    channel_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;
    channel_cfg.acq_time = NRF_SAADC_ACQTIME_20US;
    channel_cfg.burst = NRF_SAADC_BURST_ENABLED;
    channel_cfg.gain = NRF_SAADC_GAIN1_6;
    err_code = nrfx_saadc_channel_init(0, &channel_cfg);
    APP_ERROR_CHECK(err_code);

    static nrf_saadc_value_t value;

    err_code = nrfx_saadc_buffer_convert(&value, 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_sample();
    APP_ERROR_CHECK(err_code);

    nrf_pwr_mgmt_feed();
}


static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(0);
static uint8_t dev_addr = 0x90 >> 1;
static uint8_t reg_addr = 0x0f;
static uint8_t xfer_buf[4];

static void temperature_delay_timeout_handler(void * p_context)
{
    ret_code_t err_code;
    reg_addr = 0x00;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(dev_addr, &reg_addr, 1, xfer_buf, 2);
    err_code = nrfx_twim_xfer(&m_twi, &xfer, 0);
    APP_ERROR_CHECK(err_code);
}

void twi_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    ret_code_t err_code;
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            if (p_event->xfer_desc.type == NRFX_TWIM_XFER_TXRX)
            {
              if(reg_addr == 0x0f) {
                NRF_LOG_DEBUG("Device ID: %x", xfer_buf[0] * 256 + xfer_buf[1]);
                reg_addr = 0x01;
                xfer_buf[0] = reg_addr;
                xfer_buf[1] = 0x0c;
                xfer_buf[2] = 0x04;
                nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(dev_addr, xfer_buf, 3);
                err_code = nrfx_twim_xfer(&m_twi, &xfer, 0);
                APP_ERROR_CHECK(err_code);
              } else {
                xfer_buf[2] = xfer_buf[1];
                xfer_buf[3] = xfer_buf[0];
                int16_t temperature =  *((int16_t*)&xfer_buf[2]) * 0.078125;
                NRF_LOG_INFO("Temperature updated: %d.%d", temperature / 10, temperature % 10);
                int size = sizeof(m_sensor_data_buffer) / sizeof(*m_sensor_data_buffer);
                for(int i = size; i > 2; i--) {
                  m_sensor_data_buffer[i - 1] = m_sensor_data_buffer[i - 2];
                }
                m_sensor_data_buffer[1] = temperature;
                if(m_sensor_data_length < size) {
                  m_sensor_data_length++;
                }
                nrfx_twim_uninit(&m_twi);
                m_sensor_data_updated = true;
              }
            }
            else if (p_event->xfer_desc.type == NRFX_TWIM_XFER_TX)
            {
              NRF_LOG_DEBUG("Config succeed");
              static bool timer_created = false;
              if(!timer_created) {
                timer_created = true;
                err_code = app_timer_create(&m_delay_timer_id, APP_TIMER_MODE_SINGLE_SHOT, temperature_delay_timeout_handler);
                APP_ERROR_CHECK(err_code);
              }
              err_code = app_timer_start(m_delay_timer_id, 130, NULL);
              APP_ERROR_CHECK(err_code);
            }
            break;
        case NRFX_TWIM_EVT_ADDRESS_NACK:
            NRF_LOG_INFO("I2C nak for address");
            nrfx_twim_uninit(&m_twi);
            break;
        case NRFX_TWIM_EVT_DATA_NACK:
            NRF_LOG_INFO("I2C nak for data");
            nrfx_twim_uninit(&m_twi);
            break;


        default:
            break;
    }
}

void temperature_update(void)
{
    ret_code_t err_code;

    const nrfx_twim_config_t twi_cfg = {
       .scl                = 3,
       .sda                = 4,
       .frequency          = NRF_TWIM_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .hold_bus_uninit     = false
    };

    err_code = nrfx_twim_init(&m_twi, &twi_cfg, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrfx_twim_enable(&m_twi);


    reg_addr = 0x0f;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(dev_addr, &reg_addr, 1, xfer_buf, 2);
    err_code = nrfx_twim_xfer(&m_twi, &xfer, 0);
    APP_ERROR_CHECK(err_code);

    nrf_pwr_mgmt_feed();
}

