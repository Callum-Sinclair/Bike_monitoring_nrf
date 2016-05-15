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
#include "ble_conn_state.h"
#include "nrf_log.h"
#include "battery_measure_lib.h"
#include "gpio_lib.h"
#include "ble_rscs.h"
#include "pstorage.h"
#include "device_manager.h"

#define CENTRAL_LINK_COUNT          0                                  /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT       1                                  /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define APPL_LOG                    app_trace_log                      /**< Macro used to log debug information over UART. */
#define UART_TX_BUF_SIZE            256                                /**< Size of the UART TX buffer, in bytes. Must be a power of two. */
#define UART_RX_BUF_SIZE            1                                  /**< Size of the UART RX buffer, in bytes. Must be a power of two. */

/* Peripheral related. */

#define PERIPHERAL_ADVERTISING_LED       BSP_LED_2_MASK
#define PERIPHERAL_CONNECTED_LED         BSP_LED_3_MASK

#define DEVICE_NAME                      "Speed"                                  /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms). This value corresponds to 25 ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS       180                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER         0                                  /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS        (2+BSP_APP_TIMERS_NUMBER)          /**< Maximum number of timers used by the application. */
#define APP_TIMER_OP_QUEUE_SIZE     2                                  /**< Size of timer operation queues. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define TX_INTERVAL_MS                   2000
#define TX_INTERVAL                      APP_TIMER_TICKS(TX_INTERVAL_MS, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */

#define MIN_CONNECTION_INTERVAL     MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL     MSEC_TO_UNITS(30, UNIT_1_25_MS)    /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY               0                                  /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT         MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 milliseconds. */

static ble_rscs_t   m_rscs;                                                         /**< Main structure for the Running speed and cadence server module. */

/**@brief UUIDs which the central applications will scan for, and which will be advertised by the peripherals. */
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_RUNNING_SPEED_AND_CADENCE,  BLE_UUID_TYPE_BLE}};

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */

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

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

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
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_rscs_on_ble_evt(&m_rscs, p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
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
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
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
static void cadence_measure(uint16_t * cadence, uint32_t * total)
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
    
    *total = rotation_counter->CC[0] / 2;
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
        NRF_POWER->SYSTEMOFF = 1;
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
    
    uint16_t speed = 0;
    uint32_t distance = 0;
    cadence_measure(&speed, &distance);
    
    ble_rscs_meas_t rscs_measurement;
    
    rscs_measurement.is_inst_stride_len_present = true;
    rscs_measurement.is_running                 = false;
    rscs_measurement.is_total_distance_present  = true;
    
    rscs_measurement.inst_cadence = 0;
//TODO TODO TODO - enable force-power equation;
    rscs_measurement.inst_speed                 = speed;// * cadence * CRANK_RADIUS * 0.5;
    rscs_measurement.inst_stride_length         = battery_measure(BAT_PIN);
    rscs_measurement.total_distance             = distance;
    
    rscs_measurement.inst_cadence = 0;
    
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

    bat_adc_init(BAT_PIN);
    timers_init();
    reed_sw_init(READ_SW_PIN);
    
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    buttons_leds_init(&erase_bonds);

    ble_stack_init();

    gap_params_init();
    conn_params_init();
    services_init();
    advertising_init();
    application_timers_start();

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
