/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_twi_mngr.h"

#include "ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_eeg.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_timer.h"
#include "app_error.h"
#include "ks1092.h"
#include "qmi8658a.h"

// #define sd_ble_gatts_hvx(x,y)           NRF_SUCCESS

// APP_TIMER_V2 APP_TIMER_V2_RTC1_ENABLED removed both C/C++ and asm.

#define DEVICE_NAME                     "Nordic_Template"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_FAST_INTERVAL           300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_FAST_DURATION           6000                                    /**< The advertising duration (60 seconds) in units of 10 milliseconds. */
#define APP_ADV_SLOW_INTERVAL           3000                                    // 1.875s
#define APP_ADV_SLOW_DURATION           0                                       // https://devzone.nordicsemi.com/f/nordic-q-a/22928/indefinite-advertise-time

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

                                        // defaults 100, 200
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)         /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)         /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                            /**< Definition of Logger thread. */
#endif

BLE_EEG_DEF(m_eeg);
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

/*
 *  NRFX_TWIM_ENABLED         1
 *  NRFX_TWI_ENABLED          1
 *  TWI_ENABLED               1
 *  TWI1_ENABLED              1
 *  TWI0_USE_EASY_DMA	        0
 *  NRF_TWI_MNGR_ENABLED (optional, if twi_mngr is used)
 */
#define TWI1_INSTANCE_ID                1
#define TWI1_MAX_PENDING_TRANSACTIONS   4
#define QMI8658A_SCL_PIN                21      // P0.21
#define QMI8658A_SDA_PIN                19      // P0.19
#define QMI8658A_INT1_PIN               20      // P0.20
#define QMI8658A_INT2_PIN               22      // P0.22

// use twi manager instead
// static const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);
NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, TWI1_MAX_PENDING_TRANSACTIONS, TWI1_INSTANCE_ID);
static nrf_drv_twi_config_t const qmi8658a_twi_config = {
    .scl                = QMI8658A_SCL_PIN,
    .sda                = QMI8658A_SDA_PIN,
    .frequency          = NRF_DRV_TWI_FREQ_400K,
    .interrupt_priority = TWI_DEFAULT_CONFIG_IRQ_PRIORITY,  // todo
    .clear_bus_init     = TWI_DEFAULT_CONFIG_CLR_BUS_INIT,
    .hold_bus_uninit    = TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT,
};

static void qmi8658a_int2_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

static qmi8658a_dev_t qmi8658a_dev = {
    .p_twi_mngr         = &m_nrf_twi_mngr,
    .p_twi_cfg          = &qmi8658a_twi_config,
    .int1_pin           = QMI8658A_INT1_PIN,
    .int1_evt_handler   = NULL,
    .int2_pin           = QMI8658A_INT2_PIN,
    .int2_evt_handler   = qmi8658a_int2_handler
};

#define SPI0_INSTANCE_ID                0
#define SPI0_MAX_PENDING_TRANSACTIONS   4

#define KS1092_RST_PIN                  6       // P0.06
#define KS1092_SS_PIN                   7       // P0.07
#define KS1092_SCK_PIN                  8       // P0.08
#define KS1092_MISO_PIN                 17      // P0.17 (PIN20)
#define KS1092_MOSI_PIN                 18      // P0.18 (PIN21)

NRF_SPI_MNGR_DEF(m_nrf_spi_mngr, SPI0_MAX_PENDING_TRANSACTIONS, SPI0_INSTANCE_ID);
static nrf_drv_spi_config_t const ks1092_spi_config = {
    .sck_pin            = KS1092_SCK_PIN,
    .mosi_pin           = KS1092_MOSI_PIN,
    .miso_pin           = KS1092_MISO_PIN,
    .ss_pin             = KS1092_SS_PIN,
    .irq_priority       = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc                = 0xFF,
    .frequency          = NRF_DRV_SPI_FREQ_1M,
    .mode               = NRF_DRV_SPI_MODE_1, // !!! important
    .bit_order          = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
};

static nrfx_gpiote_out_config_t const ks1092_reset_pin_out_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);

static ks1092_dev_t ks1092_dev = {
    .p_spi_mngr         = &m_nrf_spi_mngr,
    .p_spi_cfg          = &ks1092_spi_config,
    .reset_pin          = KS1092_RST_PIN,
    .p_reset_pin_cfg    = &ks1092_reset_pin_out_config
};

typedef enum BottomEventType
{
  BE_NONE = 0,
  BE_CONNECTED,
  BE_DISCONNECTED,
  BE_NOTIFICATION_ENABLED,
  BE_NOTIFICATION_DISABLED,
  BE_SAADC,
  BE_IMUINT2,
} BottomEventType_t;

typedef struct BottomEvent
{
  BottomEventType_t type;
  void * p_data;
} BottomEvent_t;

#define BEVENTS_IN_QUEUE        4

static TaskHandle_t             m_bottom_thread;
QueueHandle_t                   m_beq;

static void qmi8658a_int2_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    BottomEvent_t be = { .type = BE_IMUINT2 };
    xQueueSendFromISR(m_beq, &be, NULL);
}

/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */
#define SAMPLE_TYPE_EEG         0x01
#define SAMPLE_TYPE_IMU         0x02

#define SAMPLES_IN_BUFFER       120

static const nrf_drv_timer_t    m_adc_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_ppi_channel_t        m_ppi_channel;

typedef __packed struct {
    uint8_t type;
    uint8_t version;            // 1
    uint16_t length;            // fixed 244
    uint32_t seq_num;
    int16_t sample[SAMPLES_IN_BUFFER];
} sample_packet_t;

static sample_packet_t          m_eeg_packet[2] __attribute__((aligned(0x4))) = {
    { .type = SAMPLE_TYPE_EEG,
      .version = 1,
      .length = 244             // TODO use offset of
    },
    { .type = SAMPLE_TYPE_EEG,
      .version = 1,
      .length = 244
    },
};

static sample_packet_t          m_imu_packet = {
    .type = SAMPLE_TYPE_IMU,
    .version = 1,
    .length = 244
};


static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        int16_t * p_buffer = p_event->data.done.p_buffer;
        if (p_buffer == m_eeg_packet[0].sample || p_buffer == m_eeg_packet[1].sample)
        {
          BottomEvent_t be = { .type = BE_SAADC };
          be.p_data = p_buffer;
          xQueueSendFromISR(m_beq, &be, NULL);
        }
        err_code = nrf_drv_saadc_buffer_convert(p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
    }
}

static void adc_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
}

/*
 * init adc timer and ppi, allocate and assign ppi channel
 *
 * nrf_drv_saadc_buffer_convert explained
 * https://devzone.nordicsemi.com/f/nordic-q-a/60230/what-is-the-purpose-of-nrf_drv_saadc_buffer_convert
 * https://devzone.nordicsemi.com/f/nordic-q-a/46039/configuring-adc-sampling-rate-and-simultaneous-reading-from-multiple-channels
 */
static void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_0_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);

    channel_0_config.gain = NRF_SAADC_GAIN1_4;

//    nrf_saadc_channel_config_t channel_1_config =
//        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);

//    channel_1_config.gain = NRF_SAADC_GAIN1_4;

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);

//    err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
//    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert((nrf_saadc_value_t *)m_eeg_packet[0].sample, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert((nrf_saadc_value_t *)m_eeg_packet[1].sample, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_adc_timer, &timer_cfg, adc_timer_handler);
    APP_ERROR_CHECK(err_code);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_adc_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);

    APP_ERROR_CHECK(err_code);
}

/*
 * enable adc timer (according to m_eeg.sps_shadow) and ppi channel
 */
void saadc_sampling_event_enable(void)
{
    ret_code_t err_code;
    uint32_t period;
    uint8_t sps = m_eeg.sps_shadow;
    if (sps == 0)
    {
      period = 8; // 125sps
      NRF_LOG_INFO("Set sampling rate to 125sps (8ms)");
    }
    else if (sps == 1)
    {
      period = 4; // 250sps
      NRF_LOG_INFO("Set sampling rate to 250sps (4ms)");
    }
    else
    {
      period = 8; // default 125sps
      NRF_LOG_INFO("Set sampling rate to 125sps (8ms, fallback)");
    }

    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_adc_timer, period);
    nrf_drv_timer_extended_compare(&m_adc_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);

    nrf_drv_timer_enable(&m_adc_timer);
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_disable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_disable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_disable(&m_adc_timer);
}

static void on_eeg_evt(ble_eeg_t * p_eeg, ble_eeg_evt_t * p_evt)
{
}

static void eeg_sps_write_handler (uint16_t conn_handle, ble_eeg_t * p_eeg, uint8_t new_sps)
{
    ret_code_t err_code;
    ble_gatts_value_t gatts_value;

    if (new_sps > 1) return;

    gatts_value.len = sizeof(uint8_t);
    gatts_value.offset = 0;
    gatts_value.p_value = &new_sps;

    err_code = sd_ble_gatts_value_set(conn_handle, p_eeg->sps_handles.value_handle, &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
      NRF_LOG_WARNING("failed to write sps value, error code: %d", err_code);
    }
    else
    {
      m_eeg.sps_shadow = new_sps;
      NRF_LOG_INFO("sps set to %d", new_sps);
    }
}

static void eeg_gain_write_handler (uint16_t conn_handle, ble_eeg_t * p_eeg, uint8_t new_gain)
{
    ret_code_t err_code;
    ble_gatts_value_t gatts_value;

    if (new_gain > 7) return;

    gatts_value.len = sizeof(uint8_t);
    gatts_value.offset = 0;
    gatts_value.p_value = &new_gain;

    err_code = sd_ble_gatts_value_set(conn_handle, p_eeg->gain_handles.value_handle, &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
      NRF_LOG_WARNING("failed to write gain value, error code: %d", err_code);
    }
    else
    {
      m_eeg.gain_shadow = new_gain;
      NRF_LOG_INFO("gain set to %d", new_gain);
    }
}

static void eeg_sample_notification_enabled()
{
    m_eeg.sample_notifying = true;
    BottomEvent_t be = {
        .type = BE_NOTIFICATION_ENABLED,
    };
    xQueueSendFromISR(m_beq, &be, NULL);
}

/*
 * This function is idempotent for hiding sample_notifying member.
 */
static void eeg_sample_notification_disabled()
{
    if (m_eeg.sample_notifying == true) {
        BottomEvent_t be = {
            .type = BE_NOTIFICATION_DISABLED,
        };

        xQueueSendFromISR(m_beq, &be, NULL);
        m_eeg.sample_notifying = false;
    }
}

static bool eeg_sample_notifying()
{
    return m_eeg.sample_notifying;
}

static void eeg_init(void)
{
    ret_code_t      err_code;
    ble_eeg_init_t  eeg_init_obj;

    memset(&eeg_init_obj, 0, sizeof(eeg_init_obj));

    eeg_init_obj.evt_handler          = on_eeg_evt;
    eeg_init_obj.initial_sps          = 1;
    eeg_init_obj.initial_gain         = 0;

    eeg_init_obj.sps_write_handler = eeg_sps_write_handler;
    eeg_init_obj.gain_write_handler = eeg_gain_write_handler;
    eeg_init_obj.sample_notification_enabled = eeg_sample_notification_enabled;
    eeg_init_obj.sample_notification_disabled = eeg_sample_notification_disabled;

    //eeg_init_obj.
    err_code = ble_eeg_init(&m_eeg, &eeg_init_obj);
    APP_ERROR_CHECK(err_code);

    saadc_init();
    // saadc_sampling_event_init();
    // saadc_sampling_event_enable();
}


// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


static void advertising_start(void * p_erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
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

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

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


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Add code to initialize the services used by the application.
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */
     eeg_init();
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
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


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */

}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("Idle advertising. no connectable advertising ongoing.");
            // sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
    BottomEvent_t be = { .type = BE_NONE, .p_data = NULL };

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED: {
            NRF_LOG_INFO("GAP EVT Disconnected.");
            // LED indication will be changed when advertising starts.
            eeg_sample_notification_disabled();
            m_eeg.conn_handle = BLE_CONN_HANDLE_INVALID;
            be.type = BE_DISCONNECTED;
            xQueueSendFromISR(m_beq, &be, NULL);
            break;
        }
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("GAP EVT Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            m_eeg.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            be.type = BE_CONNECTED;
            xQueueSendFromISR(m_beq, &be, NULL);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("GAP EVT PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

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

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
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


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_FAST_DURATION;

    init.config.ble_adv_slow_enabled  = true;
    init.config.ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout  = APP_ADV_SLOW_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/*
 * https://devzone.nordicsemi.com/f/nordic-q-a/95398/nrf52840-correct-freertos-logging-using-nrf_log-module
 */

#if NRF_LOG_ENABLED && NRF_LOG_DEFERRED
void log_pending_hook( void )
{
    BaseType_t YieldRequired = pdFAIL;
    if ( __get_IPSR() != 0 )
    {
        YieldRequired = xTaskResumeFromISR( m_logger_thread );
        portYIELD_FROM_ISR( YieldRequired );
    }
    else
    {
        UNUSED_RETURN_VALUE(vTaskResume(m_logger_thread));
    }
}
#endif

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


/**@brief Function for starting advertising.
 */
static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

void do_something(int16_t * p_buffer, uint32_t sn)
{
    ret_code_t err_code;
    sample_packet_t * pkt = NULL;

    if (p_buffer == m_eeg_packet[0].sample)
    {
        pkt = &m_eeg_packet[0];
    }
    else
    {
        pkt = &m_eeg_packet[1];
    }

    pkt->seq_num = sn;

    uint16_t len = sizeof(m_eeg_packet[0]);

    int left = 0;
    int right = 0;
    for (int j = 0; j < SAMPLES_IN_BUFFER / 2; j++)
    {
      left += p_buffer[2 * j];
      right += p_buffer[2 * j + 1];
    }

    left = left / (SAMPLES_IN_BUFFER / 2);
    right = right / (SAMPLES_IN_BUFFER / 2);

    // NRF_LOG_INFO("sn %d, l %d, r %d", sn, left, right);

    ble_gatts_hvx_params_t hvx_params = {0};

    hvx_params.handle = m_eeg.samples_handles.value_handle; // p_hrs->hrm_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &len;                   // this is a bidirectional param
    hvx_params.p_data = (uint8_t*)pkt;

    // is this required?
    if (m_eeg.conn_handle == BLE_CONN_HANDLE_INVALID)
    {
      NRF_LOG_ERROR("BLE_CONN_HANDLE_INVALID right before calling sd_ble_gatts_hvx");
      return;
    }

    err_code = sd_ble_gatts_hvx(m_eeg.conn_handle, &hvx_params);

    /*
    * 0x3002 -> BLE_ERROR_INVALID_CONN_HANDLE; defined in ble_err.h
    * 0x000c -> NRF_ERROR_DATA_SIZE;
    */
    if (err_code != NRF_SUCCESS)
    {
      NRF_LOG_ERROR("(eeg) sd_ble_gatts_hvx error, %d", err_code);
    }

    if (err_code == NRF_ERROR_DATA_SIZE)
    {
      NRF_LOG_INFO("len is %d", len);
      NRF_LOG_INFO("*(hvx_params.p_len) is %d", *(hvx_params.p_len));
    }

    if (sn % 1000 == 0)
    {
        NRF_LOG_INFO("(eeg) packet %d sent", sn);
    }
}

static void bottom_thread(void * arg)
{
    ret_code_t  err_code;
    uint32_t    eeg_seq_num = 0;
    uint32_t    imu_seq_num = 0;
    int         imu_offset;
    uint8_t     status0;

    NRF_LOG_INFO("Bottom thread started");

    ks1092_init(&ks1092_dev);
    qmi8658a_init(&qmi8658a_dev);

    for(;;)
    {
        BottomEvent_t be;
        xQueueReceive(m_beq, &be, portMAX_DELAY);

        switch (be.type)
        {
        case BE_NONE:
            break;
        case BE_CONNECTED:
            NRF_LOG_INFO("Connected.");
            break;
        case BE_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            break;
        case BE_NOTIFICATION_ENABLED: {
            err_code = ks1092_write(&ks1092_dev, m_eeg.gain_shadow, KS1092_CHAN_OFF);
            APP_ERROR_CHECK(err_code);

            saadc_sampling_event_enable();
            nrfx_gpiote_in_event_enable(QMI8658A_INT2_PIN, true);
            // qmi8658a_reset();
            // vTaskDelay(1);
            qmi8658a_config(&qmi8658a_dev);
            // vTaskDelay(1);
            // qmi8658a_enable();
            eeg_seq_num = 0;
            imu_seq_num = 0;
            imu_offset = 0;
            NRF_LOG_INFO("Start sampling (notification on)");
        } break;
        case BE_NOTIFICATION_DISABLED: {
            qmi8658a_disable(&qmi8658a_dev);
            nrfx_gpiote_in_event_enable(QMI8658A_INT2_PIN, false);
            saadc_sampling_event_disable();
            ks1092_write(&ks1092_dev, KS1092_CHAN_OFF, KS1092_CHAN_OFF);
            NRF_LOG_INFO("Stop sampling (notification off)");
        } break;
        case BE_SAADC: {
            if (eeg_sample_notifying())
            {
                do_something((int16_t*)be.p_data, eeg_seq_num++);
            }
        } break;
        case BE_IMUINT2:
            if (eeg_sample_notifying())
            {
                qmi8658a_read(&qmi8658a_dev, QMI8658A_STATUS0, &status0, 1);
                if (status0 & 0x01)
                {
                    int16_t * p = (int16_t *)&m_imu_packet.sample[imu_offset];
                    qmi8658a_read(&qmi8658a_dev, QMI8658A_AX_L, (uint8_t *)p, QMI8658A_BYTES_IN_DATA);
                    imu_offset += QMI8658A_SAMPLES_IN_DATA;

//                    NRF_LOG_INFO(
//                        "ax %d, ay %d, az %d, gx %d, gy %d, gz %d",
//                        p[0], p[1], p[2], p[3], p[4], p[5]);

                    if (imu_offset >= SAMPLES_IN_BUFFER)
                    {
                        imu_offset = 0;
                        m_imu_packet.seq_num = imu_seq_num;

                        uint16_t len = sizeof(m_imu_packet);

                        ble_gatts_hvx_params_t hvx_params = {0};

                        hvx_params.handle = m_eeg.samples_handles.value_handle; // p_hrs->hrm_handles.value_handle;
                        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
                        hvx_params.offset = 0;
                        hvx_params.p_len  = &len;                   // this is a bidirectional param
                        hvx_params.p_data = (uint8_t*)&m_imu_packet;

                        ret_code_t err_code = sd_ble_gatts_hvx(m_eeg.conn_handle, &hvx_params);

                        /*
                        * 0x3002 -> BLE_ERROR_INVALID_CONN_HANDLE; defined in ble_err.h
                        * 0x000c -> NRF_ERROR_DATA_SIZE;
                        * 0x0013 -> NRF_ERROR_RESOURCES
                        */
                        if (err_code != NRF_SUCCESS)
                        {
                            // TODO output INVALID_CONN_HANDLE
                            NRF_LOG_ERROR("(imu) sd_ble_gatts_hvx error, 0x%04x", err_code);
                        }

                        if (err_code == NRF_ERROR_DATA_SIZE)
                        {
                            NRF_LOG_INFO("len is %d", len);
                            NRF_LOG_INFO("*(hvx_params.p_len) is %d", *(hvx_params.p_len));
                        }

                        if (imu_seq_num % 1000 == 0)
                        {
                            NRF_LOG_INFO("(imu) packet %d sent", imu_seq_num);
                        }

                        imu_seq_num++;
                    }
                }
            }
            break;
        }
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    bool erase_bonds;

    // Initialize.
    log_init();

 #if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();

    application_timers_start();

    // NRF_SDH_BLE_GATT_MAX_MTU_SIZE is definedin sdk_config.h
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    // Start execution.
    // SEGGER_RTT_WriteString(0, "\n^^^ rtt ^^^\n");
    NRF_LOG_RAW_INFO("\n^^^ sleepmon start ^^^\n");
    nrf_sdh_freertos_init(advertising_start, &erase_bonds);

    m_beq = xQueueCreate(BEVENTS_IN_QUEUE, sizeof(BottomEvent_t));
    if (m_beq == NULL)
    {
      APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }

    if (pdPASS != xTaskCreate(bottom_thread, "BOTTOM", 256, NULL, 1, &m_bottom_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    vTaskStartScheduler();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
