/*
 * Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/**
 * @brief BLE Heart Rate and Running speed Relay application main file.
 *
 * @detail This application demonstrates a simple "Relay".
 * Meaning we pass on the values that we receive. By combining a collector part on
 * one end and a sensor part on the other, we show that the s130 can function
 * simultaneously as a central and a peripheral device.
 *
 * In the figure below, the sensor ble_app_hrs connects and interacts with the relay
 * in the same manner it would connect to a heart rate collector. In this case, the Relay
 * application acts as a central.
 *
 * On the other side, a collector (such as Master Control panel or ble_app_hrs_c) connects
 * and interacts with the relay the same manner it would connect to a heart rate sensor peripheral.
 *
 * Led layout:
 * LED 1: Central side is scanning       LED 2: Central side is connected to a peripheral
 * LED 3: Peripheral side is advertising LED 4: Peripheral side is connected to a central
 *
 * @note While testing, be careful that the Sensor and Collector are actually connecting to the Relay,
 *       and not directly to each other!
 *
 *    Peripheral                  Relay                    Central
 *    +--------+        +-----------|----------+        +-----------+
 *    | Heart  |        | Heart     |   Heart  |        |           |
 *    | Rate   | -----> | Rate     -|-> Rate   | -----> | Collector |
 *    | Sensor |        | Collector |   Sensor |        |           |
 *    +--------+        +-----------|   and    |        +-----------+
 *                      | Running   |   Running|
 *    +--------+        | Speed    -|-> Speed  |
 *    | Running|------> | Collector |   Sensor |
 *    | Speed  |        +-----------|----------+
 *    | Sensor |
 *    +--------+
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "peer_manager.h"
#include "app_timer.h"
#include "app_trace.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "app_uart.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_rscs.h"
#include "ble_rscs_c.h"
#include "ble_hrs_c.h"
#include "ble_conn_state.h"
#include "nrf_log.h"
#include "fstorage.h"
#include "battery_measure_lib.h"
#include "gpio_lib.h"

#include "fds.h"

#define CENTRAL_LINK_COUNT          2                                  /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT       1                                  /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define APPL_LOG                    app_trace_log                      /**< Macro used to log debug information over UART. */
#define UART_TX_BUF_SIZE            256                                /**< Size of the UART TX buffer, in bytes. Must be a power of two. */
#define UART_RX_BUF_SIZE            1                                  /**< Size of the UART RX buffer, in bytes. Must be a power of two. */

/* Central related. */

#define CENTRAL_SCANNING_LED        BSP_LED_0_MASK
#define CENTRAL_CONNECTED_LED       BSP_LED_1_MASK

#define APP_TIMER_PRESCALER         0                                  /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS        (2+BSP_APP_TIMERS_NUMBER)          /**< Maximum number of timers used by the application. */
#define APP_TIMER_OP_QUEUE_SIZE     2                                  /**< Size of timer operation queues. */

#define SEC_PARAM_BOND              1                                  /**< Perform bonding. */
#define SEC_PARAM_MITM              0                                  /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES   BLE_GAP_IO_CAPS_NONE               /**< No I/O capabilities. */
#define SEC_PARAM_OOB               0                                  /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE      7                                  /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE      16                                 /**< Maximum encryption key size in octets. */

#define SCAN_INTERVAL               0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0050                             /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL     MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL     MSEC_TO_UNITS(30, UNIT_1_25_MS)    /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY               0                                  /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT         MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE                 2                                  /**< Size of a UUID, in bytes. */

/**@brief Macro to unpack 16bit unsigned UUID from an octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)

/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t     * p_data;    /**< Pointer to data. */
    uint16_t      data_len;  /**< Length of data. */
} data_t;

/** @brief Scan parameters requested for scanning and connection. */
static const ble_gap_scan_params_t m_scan_param =
{
    0,              // Active scanning not set.
    0,              // Selective scanning not set.
    NULL,           // No whitelist provided.
    SCAN_INTERVAL,
    SCAN_WINDOW,
    0x0000          // No timeout.
};

/**@brief Connection parameters requested for connection. */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    0,
    (uint16_t)SUPERVISION_TIMEOUT
};

static ble_hrs_c_t               m_ble_hrs_c;                                       /**< Main structure used by the Heart rate client module. */
static ble_rscs_c_t              m_ble_rsc_c;                                       /**< Main structure used by the Running speed and cadence client module. */
static uint16_t                  m_conn_handle_hrs_c  = BLE_CONN_HANDLE_INVALID;    /**< Connection handle for the HRS central application */
static uint16_t                  m_conn_handle_rscs_c = BLE_CONN_HANDLE_INVALID;    /**< Connection handle for the RSC central application */
static ble_db_discovery_t        m_ble_db_discovery_hrs;                            /**< HR service DB structure used by the database discovery module. */
static ble_db_discovery_t        m_ble_db_discovery_rsc;                            /**< RSC service DB structure used by the database discovery module. */

/* Peripheral related. */

#define PERIPHERAL_ADVERTISING_LED       BSP_LED_2_MASK
#define PERIPHERAL_CONNECTED_LED         BSP_LED_3_MASK

#define DEVICE_NAME                      "Cadence"                                  /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms). This value corresponds to 25 ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS       600000                                     /**< The advertising timeout in units of seconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define TX_INTERVAL_MS                   2000
#define TX_INTERVAL                      APP_TIMER_TICKS(TX_INTERVAL_MS, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */

static ble_rscs_t   m_rscs;                                                         /**< Main structure for the Running speed and cadence server module. */

/**@brief UUIDs which the central applications will scan for, and which will be advertised by the peripherals. */
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_RUNNING_SPEED_AND_CADENCE,  BLE_UUID_TYPE_BLE}};

// Force/Power related
static struct force_t{
    uint16_t force;
    uint8_t  bat;
}force_measurment;

static struct speed_t
{
    uint16_t speed;
    uint32_t distance;
    uint8_t  bat;
}speed_data;

APP_TIMER_DEF(m_tx_timer_id);                                                /**< CSC measurement timer. */

// Cadence related
NRF_TIMER_Type* rotation_counter;
NRF_TIMER_Type* debounce_timer;
#define BAT_PIN                         3  // Analogue pin used to sense battery level
#define READ_SW_PIN                     5  // gpio pin that the reed switch is connected to



/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling File Data Storage events.
 *
 * @param[in] p_evt  Peer Manager event.
 * @param[in] cmd
 */
static void fds_evt_handler(fds_evt_t const * const p_evt)
{
    if (p_evt->id == FDS_EVT_GC)
    {
        NRF_LOG_PRINTF("GC completed\n");
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch(p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
            break;

        case PM_EVT_LINK_SECURED:
            NRF_LOG_PRINTF("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                           ble_conn_state_role(p_evt->conn_handle),
                           p_evt->conn_handle,
                           p_evt->params.link_secured_evt.procedure);
            break;

        case PM_EVT_LINK_SECURE_FAILED:
            /** In some cases, when securing fails, it can be restarted directly. Sometimes it can
             *  be restarted, but only after changing some Security Parameters. Sometimes, it cannot
             *  be restarted until the link is disconnected and reconnected. Sometimes it is
             *  impossible, to secure the link, or the peer device does not support it. How to
             *  handle this error is highly application dependent. */
            switch (p_evt->params.link_secure_failed_evt.error)
            {
                case PM_SEC_ERROR_PIN_OR_KEY_MISSING:
                    // Rebond if one party has lost its keys.
                    err_code = pm_link_secure(p_evt->conn_handle, true);
                    if (err_code != NRF_ERROR_INVALID_STATE)
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                    break;

                default:
                    break;
            }
            break;

        case PM_EVT_STORAGE_FULL:
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case PM_EVT_ERROR_UNEXPECTED:
            // A likely fatal error occurred. Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected_evt.error);
            break;

        case PM_EVT_PEER_DATA_UPDATED:
            break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
            APP_ERROR_CHECK_BOOL(false);
            break;

        case PM_EVT_ERROR_LOCAL_DB_CACHE_APPLY:
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
            break;

        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
            break;

        case PM_EVT_SERVICE_CHANGED_INDICATION_SENT:
            break;
    }
}

/**@brief Handles events coming from the Heart Rate central module.
 */
static void hrs_c_evt_handler(ble_hrs_c_t * p_hrs_c, ble_hrs_c_evt_t * p_hrs_c_evt)
{
    switch (p_hrs_c_evt->evt_type)
    {
        case BLE_HRS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;

            //NRF_LOG_PRINTF("Heart rate service discovered\r\n");

            // Initiate bonding.
            err_code = pm_link_secure(p_hrs_c->conn_handle, false);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }

            // Heart rate service discovered. Enable notification of Heart Rate Measurement.
            err_code = ble_hrs_c_hrm_notif_enable(p_hrs_c);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_HRS_C_EVT_DISCOVERY_COMPLETE

        case BLE_HRS_C_EVT_HRM_NOTIFICATION:
        {
            //ret_code_t err_code;

            //NRF_LOG_PRINTF("Heart Rate = %d\r\n", p_hrs_c_evt->params.hrm.hr_value);

            /*err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, p_hrs_c_evt->params.hrm.hr_value);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_PACKETS) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
            {
                APP_ERROR_HANDLER(err_code);
            }*/
            force_measurment.bat = 88;
            force_measurment.force = p_hrs_c_evt->params.hrm.hr_value;
        } break; // BLE_HRS_C_EVT_HRM_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Handles events coming from  Running Speed and Cadence central module.
 */
static void rscs_c_evt_handler(ble_rscs_c_t * p_rscs_c, ble_rscs_c_evt_t * p_rscs_c_evt)
{
    switch (p_rscs_c_evt->evt_type)
    {
        case BLE_RSCS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;

            NRF_LOG_PRINTF("Running Speed and Cadence service discovered\r\n");

            // Initiate bonding.
            err_code = pm_link_secure(p_rscs_c->conn_handle, false);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }

            // Running speed cadence service discovered. Enable notifications.
            err_code = ble_rscs_c_rsc_notif_enable(p_rscs_c);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_RSCS_C_EVT_DISCOVERY_COMPLETE:

        case BLE_RSCS_C_EVT_RSC_NOTIFICATION:
        {
            speed_data.speed    = p_rscs_c_evt->params.rsc.inst_speed;
            speed_data.distance = p_rscs_c_evt->params.rsc.total_distance;
            speed_data.bat      = p_rscs_c_evt->params.rsc.inst_stride_length;

        } break; // BLE_RSCS_C_EVT_RSC_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE Stack events concerning central applications.
 *
 * @details This function keeps the connection handles of central applications up-to-date. It
 * parses scanning reports, initiating a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests. Additionally, it updates the status
 * of LEDs used to report central applications activity.
 *
 * @note        Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *              should be dispatched to the target application before invoking this function.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(const ble_evt_t * const p_ble_evt)
{
    // The addresses of peers we attempted to connect to.
    static ble_gap_addr_t periph_addr_rsc;
    static ble_gap_addr_t periph_addr_hrs;

    // For readability.
    const ble_gap_evt_t   * const p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        /** Upon connection, check which peripheral has connected (HR or RSC), initiate DB
         *  discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            uint32_t err_code;

            // For readability.
            const ble_gap_addr_t * const peer_addr = &p_gap_evt->params.connected.peer_addr;

            /** Check which peer has connected, save the connection handle and initiate DB discovery.
             *  DB discovery will invoke a callback (hrs_c_evt_handler and rscs_c_evt_handler)
             *  upon completion, which is used to enable notifications from the peer. */
            if(memcmp(&periph_addr_rsc, peer_addr, sizeof(ble_gap_addr_t)) == 0)
            {
                NRF_LOG_PRINTF("RSC central connected\r\n");
                // Reset the peer address we had saved.
                memset(&periph_addr_rsc, 0, sizeof(ble_gap_addr_t));

                m_conn_handle_rscs_c = p_gap_evt->conn_handle;

                NRF_LOG_PRINTF("Starting DB discovery for RSCS\r\n");
                err_code = ble_db_discovery_start(&m_ble_db_discovery_rsc, p_gap_evt->conn_handle);
                APP_ERROR_CHECK(err_code);
            }
            else if(memcmp(&periph_addr_hrs, peer_addr, sizeof(ble_gap_addr_t)) == 0)
            {
                //NRF_LOG_PRINTF("HRS central connected\r\n");
                // Reset the peer address we had saved.
                memset(&periph_addr_hrs, 0, sizeof(ble_gap_addr_t));

                m_conn_handle_hrs_c = p_gap_evt->conn_handle;

                //NRF_LOG_PRINTF("Starting DB discovery for HRS\r\n");
                err_code = ble_db_discovery_start(&m_ble_db_discovery_hrs, p_gap_evt->conn_handle);
                APP_ERROR_CHECK(err_code);
            }

            /** Update LEDs status, and check if we should be looking for more
             *  peripherals to connect to. */
            LEDS_ON(CENTRAL_CONNECTED_LED);
            if (ble_conn_state_n_centrals() == CENTRAL_LINK_COUNT)
            {
                LEDS_OFF(CENTRAL_SCANNING_LED);
            }
            else
            {
                // Resume scanning.
                LEDS_ON(CENTRAL_SCANNING_LED);
                scan_start();
            }
        } break; // BLE_GAP_EVT_CONNECTED

        /** Upon disconnection, reset the connection handle of the peer which disconnected, update
         * the LEDs status and start scanning again. */
        case BLE_GAP_EVT_DISCONNECTED:
        {
            uint8_t n_centrals;

            if (p_gap_evt->conn_handle == m_conn_handle_hrs_c)
            {
                //NRF_LOG_PRINTF("HRS central disconnected (reason: %d)\r\n",
                 //      p_gap_evt->params.disconnected.reason);

                m_conn_handle_hrs_c = BLE_CONN_HANDLE_INVALID;
            }
            else if(p_gap_evt->conn_handle == m_conn_handle_rscs_c)
            {
                //NRF_LOG_PRINTF("RSC central disconnected (reason: %d)\r\n",
                 //      p_gap_evt->params.disconnected.reason);

                m_conn_handle_rscs_c = BLE_CONN_HANDLE_INVALID;
            }

            // Start scanning
            // scan_start();

            // Update LEDs status.
            LEDS_ON(CENTRAL_SCANNING_LED);
            n_centrals = ble_conn_state_n_centrals();
            if (n_centrals == 0)
            {
                LEDS_OFF(CENTRAL_CONNECTED_LED);
            }
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_ADV_REPORT:
        {
            uint32_t err_code;
            data_t   adv_data;
            data_t   type_data;

            // For readibility.
            const ble_gap_addr_t  * const peer_addr = &p_gap_evt->params.adv_report.peer_addr;

            // Initialize advertisement report for parsing.
            adv_data.p_data     = (uint8_t *)p_gap_evt->params.adv_report.data;
            adv_data.data_len   = p_gap_evt->params.adv_report.dlen;

            err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
                                        &adv_data,
                                        &type_data);

            if (err_code != NRF_SUCCESS)
            {
                // Look for the services in 'complete' if it was not found in 'more available'.
                err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                                            &adv_data,
                                            &type_data);

                if (err_code != NRF_SUCCESS)
                {
                    // If we can't parse the data, then exit.
                    break;
                }
            }

            // Verify if any UUID match the Heart rate or Running speed and cadence services.
            for (uint32_t u_index = 0; u_index < (type_data.data_len / UUID16_SIZE); u_index++)
            {
                bool        do_connect = false;
                uint16_t    extracted_uuid;

                UUID16_EXTRACT(&extracted_uuid, &type_data.p_data[u_index * UUID16_SIZE]);

                /** We do not want to connect to two peripherals offering the same service, so when
                 *  a UUID is matched, we check that we are not already connected to a peer which
                 *  offers the same service. We then save the peer address, so that upon connection
                 *  we can tell which peer has connected and update its respective connection
                 *  handle. */
                if ((extracted_uuid       == BLE_UUID_RUNNING_SPEED_AND_CADENCE) &&
                    (m_conn_handle_rscs_c == BLE_CONN_HANDLE_INVALID))// &&
                    //(0 == memcmp("\n\tSpeed", type_data.p_data, type_data.data_len))) //the charaters proceding Force are experimentally obtained
                {
                    do_connect = true;
                    memcpy(&periph_addr_rsc, peer_addr, sizeof(ble_gap_addr_t));
                }
                else if ((extracted_uuid      == BLE_UUID_HEART_RATE_SERVICE) &&
                    (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID) &&
                    (0 == memcmp("\r\n\tForce", type_data.p_data, type_data.data_len))) //the charaters proceding Force are experimentally obtained
                {
                    do_connect = true;
                    memcpy(&periph_addr_hrs, peer_addr, sizeof(ble_gap_addr_t));
                }

                if (do_connect)
                {
                    // Initiate connection.
                    err_code = sd_ble_gap_connect(peer_addr, &m_scan_param, &m_connection_param);
                    if (err_code != NRF_SUCCESS)
                    {
                        APPL_LOG("[APPL]: Connection Request Failed, reason %d\r\n", err_code);
                    }
                }
            }
        } break; // BLE_GAP_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                APPL_LOG("[APPL]: Connection Request timed out.\r\n");
            }
        } break; // BLE_GAP_EVT_TIMEOUT

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            ret_code_t err_code;
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE Stack events involving peripheral applications. Manages the
 * LEDs used to report the status of the peripheral applications.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_PRINTF("Peripheral connected\r\n");
            LEDS_OFF(PERIPHERAL_ADVERTISING_LED);
            LEDS_ON(PERIPHERAL_CONNECTED_LED);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_PRINTF("Peripheral disconnected\r\n");
            LEDS_OFF(PERIPHERAL_CONNECTED_LED);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            LEDS_ON(PERIPHERAL_ADVERTISING_LED);
            break;

        case BLE_ADV_EVT_IDLE:
        {
            ret_code_t err_code;
            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 * been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    uint16_t conn_handle;
    uint16_t role;

    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);

    pm_ble_evt_handler(p_ble_evt);

    // The connection handle should really be retrievable for any event type.
    conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    role        = ble_conn_state_role(conn_handle);

    // Based on the role this device plays in the connection, dispatch to the right applications.
    if (role == BLE_GAP_ROLE_PERIPH)
    {
        // Manages peripheral LEDs.
        on_ble_peripheral_evt(p_ble_evt);

        ble_advertising_on_ble_evt(p_ble_evt);
        ble_conn_params_on_ble_evt(p_ble_evt);

        // Dispatch to peripheral applications.
        ble_rscs_on_ble_evt(&m_rscs, p_ble_evt);
    }
    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
    {
        /** on_ble_central_evt will update the connection handles, so we want to execute it
         * after dispatching to the central applications upon disconnection. */
        if (p_ble_evt->header.evt_id != BLE_GAP_EVT_DISCONNECTED)
        {
            on_ble_central_evt(p_ble_evt);
        }

        if (conn_handle == m_conn_handle_hrs_c)
        {
            ble_hrs_c_on_ble_evt(&m_ble_hrs_c, p_ble_evt);
            ble_db_discovery_on_ble_evt(&m_ble_db_discovery_hrs, p_ble_evt);
        }
        else if (conn_handle == m_conn_handle_rscs_c)
        {
            ble_rscs_c_on_ble_evt(&m_ble_rsc_c, p_ble_evt);
            ble_db_discovery_on_ble_evt(&m_ble_db_discovery_rsc, p_ble_evt);
        }

        // If the peer disconnected, we update the connection handles last.
        if (p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED)
        {
            on_ble_central_evt(p_ble_evt);
        }
    }
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    ble_advertising_on_sys_evt(sys_evt);
    /** Dispatch the system event to the Flash Storage module, where it will be
     *  dispatched to the Flash Data Storage module and from there to the Peer Manager. */
    fs_sys_event_handler(sys_evt);
}

/**@brief Heart rate collector initialization.
 */
static void rscs_c_init(void)
{
    uint32_t            err_code;
    ble_rscs_c_init_t   rscs_c_init_obj;

    rscs_c_init_obj.evt_handler = rscs_c_evt_handler;

    err_code = ble_rscs_c_init(&m_ble_rsc_c, &rscs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Heart rate collector initialization.
 */
static void hrs_c_init(void)
{
    uint32_t         err_code;
    ble_hrs_c_init_t hrs_c_init_obj;

    hrs_c_init_obj.evt_handler = hrs_c_evt_handler;

    err_code = ble_hrs_c_init(&m_ble_hrs_c, &hrs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        pm_peer_delete_all();
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond              = SEC_PARAM_BOND;
    sec_param.mitm              = SEC_PARAM_MITM;
    sec_param.io_caps           = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob               = SEC_PARAM_OOB;
    sec_param.min_key_size      = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size      = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_periph.enc  = 1;
    sec_param.kdist_periph.id   = 1;
    sec_param.kdist_central.enc = 1;
    sec_param.kdist_central.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to
 *                            wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    ret_code_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

#define CADENCE_COUNT_ARRAY_SIZE 6
/**@brief Function for populating simulated cycling speed and cadence measurements.
 */
static void cadence_measure(uint16_t * cadence)
{
    /*static uint16_t i = 50;
    i++;
    *cadence = i;
    return;*/
    static uint32_t rot_counter_last      = 0;
    static uint16_t still_counter         = 0;
    static uint32_t cadence_tracker[CADENCE_COUNT_ARRAY_SIZE];
    static uint16_t count_num             = 0;

    int32_t cadence_calc = 0;
    
    cadence_tracker[count_num] = rotation_counter->CC[0] / 2; //each pass of the magnet causes two increments
    // get the average cadence in the last 5 s
    if (count_num == CADENCE_COUNT_ARRAY_SIZE - 1)
    {
        cadence_calc = (60 * (cadence_tracker[count_num] - cadence_tracker[0])) / ((TX_INTERVAL_MS * (CADENCE_COUNT_ARRAY_SIZE - 1)) / 1000);
        if (cadence_calc < 0)
            *cadence = (uint16_t)(cadence_calc + 65536);
        else
        {
            *cadence = (uint16_t)cadence_calc;
        }
    }
    else
    {
        cadence_calc = (60 * (cadence_tracker[count_num] - cadence_tracker[count_num + 1])) / ((TX_INTERVAL_MS * (CADENCE_COUNT_ARRAY_SIZE - 1)) / 1000);
        if (cadence_calc < 0)
            *cadence = (uint16_t)(cadence_calc + 65536);
        else
        {
            *cadence = (uint16_t)cadence_calc;
        }
    }
    if (rot_counter_last == rotation_counter->CC[0])
    {
        still_counter ++;
    }
    else
    {
        still_counter = 0;
    }
    if (still_counter > 200)
    {
        //NRF_POWER->SYSTEMOFF = 1;
    }
    rot_counter_last = rotation_counter->CC[0];
    
    count_num++;
    if (count_num == CADENCE_COUNT_ARRAY_SIZE)
    {
        count_num = 0;
    }
}

static void tx_timeout_handler(void * p_context)
{
    rotation_counter->TASKS_CAPTURE[0] = 1;
    uint32_t        err_code;

    UNUSED_PARAMETER(p_context);
    
    uint16_t cadence = 0;
    cadence_measure(&cadence);
    
    ble_rscs_meas_t rscs_measurement;
    
    rscs_measurement.is_inst_stride_len_present = true;
    rscs_measurement.is_running                 = false;
    rscs_measurement.is_total_distance_present  = true;
    
//TODO TODO TODO - enable force-power equation;
    uint8_t power = ((float)force_measurment.force * (float)cadence * 3.1415 * 13.5 / (60.0 * 50.0));
    
    rscs_measurement.inst_cadence               = cadence;
    rscs_measurement.inst_speed                 = speed_data.speed;// * cadence * CRANK_RADIUS * 0.5;
    rscs_measurement.inst_stride_length         = (speed_data.bat << 8) + battery_measure(BAT_PIN);
    rscs_measurement.total_distance             = (speed_data.distance << 8) + power;
    
    err_code = ble_rscs_measurement_send(&m_rscs, &rscs_measurement);
    
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create battery timer.
    err_code = app_timer_create(&m_tx_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                tx_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_tx_timer_id, TX_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONNECTION_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONNECTION_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = SUPERVISION_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_CONN_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;  // Ignore events.
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t        err_code;
    ble_rscs_init_t rscs_init;

    // Initialize the Running Speed and Cadence Service.
    memset(&rscs_init, 0, sizeof(rscs_init));

    rscs_init.evt_handler = NULL;
    rscs_init.feature     = BLE_RSCS_FEATURE_INSTANT_STRIDE_LEN_BIT |
                            BLE_RSCS_FEATURE_TOTAL_DISTANCE_BIT;

    // Here the sec level for the Running Speed and Cadence Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&rscs_init.rsc_meas_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&rscs_init.rsc_meas_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&rscs_init.rsc_meas_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&rscs_init.rsc_feature_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&rscs_init.rsc_feature_attr_md.write_perm);

    err_code = ble_rscs_init(&m_rscs, &rscs_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

void reed_sw_init(uint32_t pin)
{
    // Sets TIMER4's counter to be incremented each time "pin" is toggled
    // This uses PPI channel 0-3 and TIMER4&3 (defined at top)
    // De-bouncing with a 4 ms 
    gpio_pin_in_init(pin);
    //GPIOTE setup
    NRF_GPIOTE->CONFIG[0] = ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) | \
                             (pin << GPIOTE_CONFIG_PSEL_Pos) | \
                             (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos));
    //Timer(Counter) setup
    rotation_counter->MODE = TIMER_MODE_MODE_Counter;
    rotation_counter->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    rotation_counter->TASKS_CLEAR = 1;
    rotation_counter->TASKS_START = 1;
    //Timer (debounce) setup
    debounce_timer->MODE = TIMER_MODE_MODE_Timer;
    debounce_timer->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    debounce_timer->PRESCALER = 4; //gives T=0.01ms
    
    debounce_timer->CC[0] = 1;  // 0.01 ms
    debounce_timer->CC[1] = 4000; // 40 ms
    
    debounce_timer->TASKS_CLEAR = 1;
    debounce_timer->TASKS_START = 1;
    //PPI setup
    NRF_PPI->CH[0].EEP = (uint32_t)&(NRF_GPIOTE->EVENTS_IN[0]);
    NRF_PPI->CH[0].TEP = (uint32_t)&(rotation_counter->TASKS_COUNT);
    
    NRF_PPI->CH[1].EEP = (uint32_t)&(NRF_GPIOTE->EVENTS_IN[0]);
    NRF_PPI->CH[1].TEP = (uint32_t)&(debounce_timer->TASKS_CLEAR);
    
    NRF_PPI->CH[2].EEP = (uint32_t)&(debounce_timer->EVENTS_COMPARE[0]);
    NRF_PPI->CH[2].TEP = (uint32_t)&(rotation_counter->TASKS_STOP);
    
    NRF_PPI->CH[3].EEP = (uint32_t)&(debounce_timer->EVENTS_COMPARE[1]);
    NRF_PPI->CH[3].TEP = (uint32_t)&(rotation_counter->TASKS_START);
    
    
    NRF_PPI->CHENSET = 0xF;

}


/** @brief Function to sleep until a BLE event is received by the application.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    ret_code_t err_code;
    bool       erase_bonds;
    
    rotation_counter = NRF_TIMER2;
    debounce_timer = NRF_TIMER1;
    force_measurment.bat = 0;
    force_measurment.force = 0;

    bat_adc_init(BAT_PIN);
    timers_init();
    reed_sw_init(READ_SW_PIN);
    
    err_code = NRF_LOG_INIT();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_PRINTF("Relay Example\r\n");

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    buttons_leds_init(&erase_bonds);

    if (erase_bonds == true)
    {
        NRF_LOG("Bonds erased!\r\n");
    }

    ble_stack_init();

    peer_manager_init(erase_bonds);

    db_discovery_init();
    rscs_c_init();
    hrs_c_init();

    gap_params_init();
    conn_params_init();
    services_init();
    advertising_init();
    application_timers_start();

    /** Start scanning for peripherals and initiate connection to devices which
     *  advertise Heart Rate or Running speed and cadence UUIDs. */
    scan_start();

    // Turn on the LED to signal scanning.
    LEDS_ON(CENTRAL_SCANNING_LED);

    // Start advertising.
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    reed_sw_init(READ_SW_PIN);

    for (;;)
    {
        // Wait for BLE events.
        power_manage();
    }
}
