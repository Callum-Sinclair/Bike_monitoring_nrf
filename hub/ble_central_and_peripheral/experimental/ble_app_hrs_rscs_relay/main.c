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
#include <math.h>
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
#include "ble_hrs.h"
#include "ble_rscs.h"
#include "ble_bas.h"
#include "ble_cscs.h"
#include "ble_hrs_c.h"
#include "ble_rscs_c.h"
#include "ble_conn_state.h"
#include "nrf_log.h"
#include "fstorage.h"

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

#define DEVICE_NAME                      "Bike_Hub"                                    /**< Name of device used for advertising. */
#define MANUFACTURER_NAME                "SmartBike"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms). This value corresponds to 25 ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS       180                                        /**< The advertising timeout in units of seconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

static ble_bas_t  m_bas;                                     /**< Structure used to identify the battery service. */
static ble_cscs_t m_cscs;                                    /**< Structure used to identify the cycling speed and cadence service. */
static ble_rscs_t m_rscs;                                    /**< Structure used to identify the running speed and cadence service. */
static ble_hrs_t  m_hrs;                                     /**< Structure used to identify the heart rate service. */
static bool                             m_auto_calibration_in_progress = false;            /**< Set when an autocalibration is in progress. */

typedef struct
{
    uint16_t    speed_speed;
    uint32_t    speed_distance;
    uint8_t     speed_bat;
    uint16_t    cadence_cadence;
    uint8_t     cadence_bat;
    uint16_t    cadence_power;
    uint8_t     cadence_power_bat;
    uint16_t    usr_range;
    uint32_t    usr_bat;
    uint8_t     hub_gradient;
    uint8_t     hub_temp;
    uint8_t     hub_heart_rate;
    uint8_t     hub_bat;
}bike_data_t;

static bike_data_t bike_data;

APP_TIMER_DEF(m_battery_timer_id);                                                 /**< Battery timer. */
APP_TIMER_DEF(m_csc_tx_timer_id);                                                /**< CSC measurement timer. */
APP_TIMER_DEF(m_rsc_tx_timer_id);                                                /**< RSC measurement timer. */
APP_TIMER_DEF(m_heart_rate_tx_timer_id);                                               /**< Heart rate measurement timer. */
APP_TIMER_DEF(m_rr_tx_timer_id);                                              /**< RR interval timer. */                 /**< RR interval timer. */
APP_TIMER_DEF(m_i2c_read_timer_id);                                              /**< RR interval timer. */                 /**< RR interval timer. */

#define TEMP_TIMER NRF_TIMER4

#define HEART_RATE_MEAS_INTERVAL         APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /**< Heart rate measurement interval (ticks). */
#define RR_INTERVAL_INTERVAL             APP_TIMER_TICKS(300, APP_TIMER_PRESCALER)  /**< RR interval interval (ticks). */
#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define SPEED_AND_CADENCE_MEAS_INTERVAL 1000                                       /**< Speed and cadence measurement interval (milliseconds). */
static ble_sensor_location_t supported_locations[] = {BLE_SENSOR_LOCATION_FRONT_WHEEL ,
                                                      BLE_SENSOR_LOCATION_REAR_WHEEL};          /**< supported location for the sensor location. */


/**@brief UUIDs which the central applications will scan for, and which will be advertised by the peripherals. */
static ble_uuid_t m_adv_uuids[] =  {{BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE}, \
                                    {BLE_UUID_RUNNING_SPEED_AND_CADENCE,    BLE_UUID_TYPE_BLE}, \
                                    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE}, \
                                    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE}, \
                                    {BLE_UUID_CYCLING_SPEED_AND_CADENCE,    BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */


//I2S/TWI
NRF_TWIM_Type* i2c = NRF_TWIM0;
#define I2C_SCL 11
#define I2C_SDA 12
#define I2C_RX_BUF_SIZE 20
#define I2C_TX_BUF_SIZE 20
uint32_t i2c_rx_buf[I2C_RX_BUF_SIZE];
uint32_t i2c_tx_buf[I2C_TX_BUF_SIZE];
#define ADXL345_RA_DEVID            0x00
#define ADXL345_RA_RESERVED1        0x01
#define ADXL345_RA_THRESH_TAP       0x1D
#define ADXL345_RA_OFSX             0x1E
#define ADXL345_RA_OFSY             0x1F
#define ADXL345_RA_OFSZ             0x20
#define ADXL345_RA_DUR              0x21
#define ADXL345_RA_LATENT           0x22
#define ADXL345_RA_WINDOW           0x23
#define ADXL345_RA_THRESH_ACT       0x24
#define ADXL345_RA_THRESH_INACT     0x25
#define ADXL345_RA_TIME_INACT       0x26
#define ADXL345_RA_ACT_INACT_CTL    0x27
#define ADXL345_RA_THRESH_FF        0x28
#define ADXL345_RA_TIME_FF          0x29
#define ADXL345_RA_TAP_AXES         0x2A
#define ADXL345_RA_ACT_TAP_STATUS   0x2B
#define ADXL345_RA_BW_RATE          0x2C
#define ADXL345_RA_POWER_CTL        0x2D
#define ADXL345_RA_INT_ENABLE       0x2E
#define ADXL345_RA_INT_MAP          0x2F
#define ADXL345_RA_INT_SOURCE       0x30
#define ADXL345_RA_DATA_FORMAT      0x31
#define ADXL345_RA_DATAX0           0x32
#define ADXL345_RA_DATAX1           0x33
#define ADXL345_RA_DATAY0           0x34
#define ADXL345_RA_DATAY1           0x35
#define ADXL345_RA_DATAZ0           0x36
#define ADXL345_RA_DATAZ1           0x37
#define ADXL345_RA_FIFO_CTL         0x38
#define ADXL345_RA_FIFO_STATUS      0x39

#define ADXL345_AIC_ACT_AC_BIT      7
#define ADXL345_AIC_ACT_X_BIT       6
#define ADXL345_AIC_ACT_Y_BIT       5
#define ADXL345_AIC_ACT_Z_BIT       4
#define ADXL345_AIC_INACT_AC_BIT    3
#define ADXL345_AIC_INACT_X_BIT     2
#define ADXL345_AIC_INACT_Y_BIT     1
#define ADXL345_AIC_INACT_Z_BIT     0

#define ADXL345_TAPAXIS_SUP_BIT     3
#define ADXL345_TAPAXIS_X_BIT       2
#define ADXL345_TAPAXIS_Y_BIT       1
#define ADXL345_TAPAXIS_Z_BIT       0

#define ADXL345_TAPSTAT_ACTX_BIT    6
#define ADXL345_TAPSTAT_ACTY_BIT    5
#define ADXL345_TAPSTAT_ACTZ_BIT    4
#define ADXL345_TAPSTAT_ASLEEP_BIT  3
#define ADXL345_TAPSTAT_TAPX_BIT    2
#define ADXL345_TAPSTAT_TAPY_BIT    1
#define ADXL345_TAPSTAT_TAPZ_BIT    0

#define ADXL345_BW_LOWPOWER_BIT     4
#define ADXL345_BW_RATE_BIT         3
#define ADXL345_BW_RATE_LENGTH      4

#define ADXL345_RATE_3200           0xF
#define ADXL345_RATE_1600           0xE
#define ADXL345_RATE_800            0xD
#define ADXL345_RATE_400            0xC
#define ADXL345_RATE_200            0xB
#define ADXL345_RATE_100            0xA
#define ADXL345_RATE_50             9
#define ADXL345_RATE_25             8
#define ADXL345_RATE_12P5           7
#define ADXL345_RATE_6P25           6
#define ADXL345_RATE_3P13           5
#define ADXL345_RATE_1P56           4
#define ADXL345_RATE_0P78           3
#define ADXL345_RATE_0P39           2
#define ADXL345_RATE_0P20           1
#define ADXL345_RATE_0P10           0

#define ADXL345_PCTL_LINK_BIT       5
#define ADXL345_PCTL_AUTOSLEEP_BIT  4
#define ADXL345_PCTL_MEASURE_BIT    3
#define ADXL345_PCTL_SLEEP_BIT      2
#define ADXL345_PCTL_WAKEUP_BIT     1
#define ADXL345_PCTL_WAKEUP_LENGTH  2

#define ADXL345_WAKEUP_8HZ          0
#define ADXL345_WAKEUP_4HZ          1
#define ADXL345_WAKEUP_2HZ          2
#define ADXL345_WAKEUP_1HZ          3

#define ADXL345_RANGE_2G            0
#define ADXL345_RANGE_4G            1
#define ADXL345_RANGE_8G            2
#define ADXL345_RANGE_16G           3

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}xyz_t;

// Temp defines
#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4 
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD       0x2E
#define BMP085_READPRESSURECMD   0x34
// Temp calibration data
typedef struct
{
    int16_t  ac1;
    int16_t  ac2;
    int16_t  ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t  b1;
    int16_t  b2;
    int16_t  mb;
    int16_t  mc;
    int16_t  md;
}temp_calib_data_t;

static temp_calib_data_t temp_calib_data;


#define TEMP_ADDRESS 0x77  // I2C address of BMP085
#define ACCEL_ADDRESS 0x53  // Accelerometor device address
/**
 * @brief I2C/TWI initialization.
 */
void i2c_init (void)
{
    i2c->PSEL.SCL   = I2C_SCL;
    i2c->PSEL.SDA   = I2C_SDA;
    i2c->FREQUENCY  = TWIM_FREQUENCY_FREQUENCY_K250; // 100 kbps
    i2c->RXD.PTR    = (uint32_t)&i2c_rx_buf[0];
    i2c->RXD.MAXCNT = I2C_RX_BUF_SIZE;
    i2c->RXD.LIST   = TWIM_RXD_LIST_LIST_Disabled;
    i2c->TXD.PTR    = (uint32_t)i2c_tx_buf;
    i2c->TXD.MAXCNT = I2C_TX_BUF_SIZE;
    i2c->TXD.LIST   = TWIM_TXD_LIST_LIST_Disabled;
    i2c->ENABLE     = TWIM_ENABLE_ENABLE_Enabled;
}

void i2c_tx_rx(uint32_t tx_addr, uint8_t* data_tx_addr, uint8_t tx_bytes, uint8_t* data_rx_addr, uint8_t rx_bytes)
{
    i2c->EVENTS_LASTRX = 0;
    i2c->SHORTS     = ((TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos) | \
                       (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos));
    i2c->ADDRESS = tx_addr;
    i2c->TXD.PTR = (uint32_t)data_tx_addr;
    i2c->TXD.MAXCNT = tx_bytes;
    i2c->RXD.MAXCNT = rx_bytes;
    i2c->RXD.PTR    = (uint32_t)data_rx_addr;
    i2c->TASKS_STARTTX = 1;
    while (i2c->EVENTS_LASTRX == 0);
}

void i2c_tx(uint32_t tx_addr, uint8_t* data_tx_addr, uint8_t tx_bytes)
{
    i2c->EVENTS_LASTTX = 0;
    i2c->SHORTS  = TWIM_SHORTS_LASTTX_STOP_Enabled << TWIM_SHORTS_LASTTX_STOP_Pos;
    i2c->ADDRESS = tx_addr;
    i2c->TXD.PTR = (uint32_t)data_tx_addr;
    i2c->TXD.MAXCNT = tx_bytes;
    i2c->TASKS_STARTTX = 1;
    while (i2c->EVENTS_LASTTX == 0);
}

void accel_i2c_init(void)
{
    uint8_t tx_vals[20] = {0};
    uint8_t rx_vals[20] = {0};
    
    tx_vals[0] = ADXL345_RA_DATA_FORMAT;
    tx_vals[1] = ADXL345_RANGE_2G;
    i2c_tx(ACCEL_ADDRESS, tx_vals, 2);
    
    tx_vals[0] = ADXL345_RA_POWER_CTL;
    tx_vals[1] = 0x08;
    i2c_tx(ACCEL_ADDRESS, tx_vals, 2);
#ifdef DEBUG
    printf("initialised %d\n", i2c->TXD.AMOUNT);
#endif

    tx_vals[0] = ADXL345_RA_POWER_CTL;
    i2c_tx_rx(ACCEL_ADDRESS, tx_vals, 1, rx_vals, 2);
#ifdef DEBUG
    printf("Power Crtl read: 0x%x 0x%x\n", rx_vals[0], rx_vals[1]);
    printf("got %d bytes\n", i2c->RXD.AMOUNT);
#endif
}

void accel_get_dev_id(void)
{
    uint8_t tx_vals[2] = {ADXL345_RA_DEVID};
    uint8_t rx_vals[20] = {0};
    i2c_tx_rx(ACCEL_ADDRESS, tx_vals, 1, rx_vals, 2);
#ifdef DEBUG
    printf("Dev ID: %d\n", rx_vals[0]);
    printf("got %d bytes\n", i2c->RXD.AMOUNT);
#endif
}

void accel_get_xyz(xyz_t* xyz)
{
    uint8_t tx_vals[2] = {ADXL345_RA_DATAX0};
    uint8_t rx_vals[20] = {0};
    i2c_tx_rx(ACCEL_ADDRESS, tx_vals, 1, rx_vals, 6);
#ifdef DEBUG
    printf("Raw data X0: %d %d %d %d %d %d \n", rx_vals[0], rx_vals[1], rx_vals[2], rx_vals[3], rx_vals[4], rx_vals[5]);
#endif
    xyz->x = (rx_vals[1]<<8) + rx_vals[0];
    xyz->y = (rx_vals[3]<<8) + rx_vals[2];
    xyz->z = (rx_vals[5]<<8) + rx_vals[4];
#ifdef DEBUG
    printf("Data:\n X = %d Y = %d Z = %d\n\n", xyz->x, xyz->y, xyz->z);
#endif
}


uint8_t grad_read(void)
{
    xyz_t xyz;
    float gradient;
    accel_get_xyz(&xyz);
    gradient  = atan2((float)(0 - xyz.y), (float)xyz.z) * 180 / 3.14159;
#ifdef DEBUG
    printf("Data:\n X = %d Y = %d Z = %d\n\n", xyz.x, xyz.y, xyz.z);
#endif
    //printf("Gradient: %f\n\n", gradient);
    return (uint8_t)(gradient + 50);
}

void temp_i2c_init(void)
{
    uint8_t tx_vals[20] = {0};
    uint8_t rx_vals[20] = {0};
    
    tx_vals[0] = BMP085_CAL_AC1;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.ac1 = (int16_t)((rx_vals[0] << 8) + rx_vals[1]);

    tx_vals[0] = BMP085_CAL_AC2;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.ac2 = (int16_t)((rx_vals[0] << 8) + rx_vals[1]);

    tx_vals[0] = BMP085_CAL_AC3;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.ac3 = (int16_t)((rx_vals[0] << 8) + rx_vals[1]);

    tx_vals[0] = BMP085_CAL_AC4;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.ac4 = (rx_vals[0] << 8) + rx_vals[1];

    tx_vals[0] = BMP085_CAL_AC5;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.ac5 = (rx_vals[0] << 8) + rx_vals[1];

    tx_vals[0] = BMP085_CAL_AC6;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.ac6 = (int16_t)((rx_vals[0] << 8) + rx_vals[1]);

    tx_vals[0] = BMP085_CAL_B1;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.b1 = (int16_t)((rx_vals[0] << 8) + rx_vals[1]);

    tx_vals[0] = BMP085_CAL_B2;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.b2 = (int16_t)((rx_vals[0] << 8) + rx_vals[1]);

    tx_vals[0] = BMP085_CAL_MB;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.mb = (int16_t)((rx_vals[0] << 8) + rx_vals[1]);

    tx_vals[0] = BMP085_CAL_MC;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.mc = (int16_t)((rx_vals[0] << 8) + rx_vals[1]);

    tx_vals[0] = BMP085_CAL_MD;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
    temp_calib_data.md = (int16_t)((rx_vals[0] << 8) + rx_vals[1]);

#ifdef DEBUG
    printf("ac: %i, %i, %i, %i, %i, %i \n", temp_calib_data.ac1, temp_calib_data.ac2, temp_calib_data.ac3, temp_calib_data.ac4, temp_calib_data.ac5, temp_calib_data.ac6);
    printf("b: %i, %i, m: %i, %i, %i\n", temp_calib_data.b1, temp_calib_data.b1, temp_calib_data.mb, temp_calib_data.mc, temp_calib_data.md);
#endif
}

int32_t temp_convert(int32_t raw)
{
    int32_t temp = 0;
    uint16_t ac6 = 23153;
    uint16_t ac5 = 32757;
    int16_t  mc  = -8711;
    int16_t  md  = 2868;
#ifdef DEBUG
    printf("ac5: %i, ac6: %i\n", temp_calib_data.ac5, temp_calib_data.ac6);
#endif
    float x1a = (raw - (int32_t)temp_calib_data.ac6); 
    float x1b = (float)temp_calib_data.ac5 / 32768;
    int32_t x1  = (int32_t)(x1a * x1b);
#ifdef DEBUG
    printf("x1a %f, x1b %f, x1c %d", x1a, x1b, x1);
#endif
    int32_t x2 = ((int32_t)temp_calib_data.mc << 11) / (x1 + (int32_t)temp_calib_data.md);
    //int32_t x1 = (raw - (int32_t)ac6) * (int32_t)ac5 >> 15;
    //int32_t x2 = ((int32_t)mc << 11) / (x1 + (int32_t)md);
    int32_t b5 = x1 + x2;
#ifdef DEBUG
    printf("X1 = %d\n", x1);
    printf("X2 = %d\n", x2);
    printf("B5 = %d\n", b5);
#endif
    temp = (b5 + 8) >> 4;
    return temp;
}

void temp_timer_start(void)
{
    TEMP_TIMER->MODE = TIMER_MODE_MODE_Timer;
    TEMP_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    TEMP_TIMER->PRESCALER = 4; //gives T=0.001ms
    TEMP_TIMER->SHORTS = ((TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos) | \
                          (TIMER_SHORTS_COMPARE1_STOP_Enabled << TIMER_SHORTS_COMPARE1_STOP_Pos));
    
    TEMP_TIMER->CC[0] = 500;   // 5 ms
    TEMP_TIMER->CC[1] = 501;   // 5 ms
    
    TEMP_TIMER->TASKS_CLEAR = 1;
    TEMP_TIMER->TASKS_START = 1;
}

bool temp_timer_running()
{
    if (TEMP_TIMER->EVENTS_COMPARE[0])
    {
        TEMP_TIMER->EVENTS_COMPARE[0] = 0;
        return true;
    }
    return false;
}

uint8_t temp_read(void)
{
    uint8_t tx_vals[20] = {0};
    uint8_t rx_vals[20] = {0};
    
    tx_vals[0] = BMP085_CONTROL;
    tx_vals[1] = BMP085_READTEMPCMD;
    i2c_tx(TEMP_ADDRESS, tx_vals, 2);
    
    temp_timer_start();
    while(temp_timer_running());
    
    tx_vals[0] = BMP085_TEMPDATA;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
#ifdef DEBUG
    printf("Temp raw read: 0x%x 0x%x, %d\n", rx_vals[0], rx_vals[1], (rx_vals[0]<<8) + rx_vals[1]);
#endif
    int32_t temp = temp_convert(23420);
    //printf("Actual Temp: %d.%d\'C\n", temp/10, temp%10);
    return (uint8_t)(temp + 50);
}
             

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

// TX FUNCTIONS
uint8_t measure_bat(void)
{
    //TODO - add battery measure
    return 81;
}

static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = 75;//measure_bat();

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

static void csc_tx_timeout_handler(void * p_context)
{
    uint32_t        err_code;
    ble_cscs_meas_t cscs_measurement;
        static uint16_t event_time            = 0;
    uint16_t event_time_inc = (1024 * SPEED_AND_CADENCE_MEAS_INTERVAL) / 1000;

    UNUSED_PARAMETER(p_context);
    cscs_measurement.is_crank_rev_data_present = true;
    cscs_measurement.is_wheel_rev_data_present = true;
    
    cscs_measurement.last_wheel_event_time = bike_data.speed_speed;
    cscs_measurement.cumulative_wheel_revs = bike_data.speed_distance;
    cscs_measurement.last_crank_event_time = ((bike_data.speed_bat << 8) + bike_data.cadence_bat);
    cscs_measurement.cumulative_crank_revs = bike_data.cadence_cadence;
    
    cscs_measurement.last_crank_event_time = 0;// event_time + event_time_inc;
    event_time= event_time + event_time_inc;
    err_code = ble_cscs_measurement_send(&m_cscs, &cscs_measurement);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
    if (m_auto_calibration_in_progress)
    {
        err_code = ble_sc_ctrlpt_rsp_send(&(m_cscs.ctrl_pt), BLE_SCPT_SUCCESS);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != BLE_ERROR_NO_TX_PACKETS)
            )
        {
            APP_ERROR_HANDLER(err_code);
        }
        if (err_code != BLE_ERROR_NO_TX_PACKETS)
        {
            m_auto_calibration_in_progress = false;
        }
    }
}

static void rsc_tx_timeout_handler(void * p_context)
{
    uint32_t        err_code;
    ble_rscs_meas_t rscs_measurement;

    UNUSED_PARAMETER(p_context);

    rscs_measurement.is_inst_stride_len_present  = true;
    rscs_measurement.is_total_distance_present   = true;

    rscs_measurement.inst_speed = bike_data.cadence_power;
    rscs_measurement.inst_cadence = bike_data.cadence_power_bat;
    rscs_measurement.inst_stride_length = bike_data.usr_range;
    rscs_measurement.total_distance = bike_data.usr_bat;
    
    err_code = ble_rscs_measurement_send(&m_rscs, &rscs_measurement);
    if (
        (err_code != NRF_SUCCESS)
        &&
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_PACKETS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

static void heart_rate_tx_timeout_handler(void * p_context)
{
    uint32_t        err_code;
    uint16_t        heart_rate;

    UNUSED_PARAMETER(p_context);

    heart_rate = bike_data.hub_heart_rate;

    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

static void rr_tx_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    uint16_t rr_interval;

    rr_interval = ((bike_data.hub_gradient << 8) + bike_data.hub_temp);
    ble_hrs_rr_interval_add(&m_hrs, rr_interval);
}

static void i2c_read_timout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    bike_data.hub_gradient = grad_read();
    bike_data.hub_temp     = temp_read();
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

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Create battery timer.
    err_code = app_timer_create(&m_csc_tx_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                csc_tx_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_rsc_tx_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                rsc_tx_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_heart_rate_tx_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_tx_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_rr_tx_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                rr_tx_timeout_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_i2c_read_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                i2c_read_timout_handler);
    APP_ERROR_CHECK(err_code);
}

ble_scpt_response_t sc_ctrlpt_event_handler(ble_sc_ctrlpt_t     * p_sc_ctrlpt,
                                            ble_sc_ctrlpt_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_SC_CTRLPT_EVT_SET_CUMUL_VALUE:
            break;

        case BLE_SC_CTRLPT_EVT_START_CALIBRATION:
            m_auto_calibration_in_progress = true;
            break;

        default:
            // No implementation needed.
            break;
    }
    return (BLE_SCPT_SUCCESS);
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
        //NRF_LOG_PRINTF("GC completed\n");
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
            //NRF_LOG_PRINTF("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
              //             ble_conn_state_role(p_evt->conn_handle),
              //             p_evt->conn_handle,
              //             p_evt->params.link_secured_evt.procedure);
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
            bike_data.usr_bat = 88;
            bike_data.usr_range = p_hrs_c_evt->params.hrm.hr_value;
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

            //NRF_LOG_PRINTF("Running Speed and Cadence service discovered\r\n");

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
            //uint32_t        err_code;
            //ble_rscs_meas_t rscs_measurment;

           // NRF_LOG_PRINTF("Speed      = %d\r\n", p_rscs_c_evt->params.rsc.inst_speed);

            /*rscs_measurment.is_running                  = p_rscs_c_evt->params.rsc.is_running;
            rscs_measurment.is_inst_stride_len_present  = p_rscs_c_evt->params.rsc.is_inst_stride_len_present;
            rscs_measurment.is_total_distance_present   = p_rscs_c_evt->params.rsc.is_total_distance_present;

            rscs_measurment.inst_stride_length = p_rscs_c_evt->params.rsc.inst_stride_length;
            rscs_measurment.inst_cadence       = p_rscs_c_evt->params.rsc.inst_cadence;
            rscs_measurment.inst_speed         = p_rscs_c_evt->params.rsc.inst_speed;
            rscs_measurment.total_distance     = p_rscs_c_evt->params.rsc.total_distance;

            err_code = ble_rscs_measurement_send(&m_rscs, &rscs_measurment);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_PACKETS) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
            {
                APP_ERROR_HANDLER(err_code);
            }*/
            bike_data.cadence_cadence   = p_rscs_c_evt->params.rsc.inst_stride_length;
            bike_data.cadence_power_bat = p_rscs_c_evt->params.rsc.inst_cadence;
            bike_data.cadence_power     = p_rscs_c_evt->params.rsc.inst_speed;
            bike_data.cadence_bat       = p_rscs_c_evt->params.rsc.total_distance;

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
    static ble_gap_addr_t periph_addr_hrs;
    static ble_gap_addr_t periph_addr_rsc;

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
            if(memcmp(&periph_addr_hrs, peer_addr, sizeof(ble_gap_addr_t)) == 0)
            {
                //NRF_LOG_PRINTF("HRS central connected\r\n");
                // Reset the peer address we had saved.
                memset(&periph_addr_hrs, 0, sizeof(ble_gap_addr_t));

                m_conn_handle_hrs_c = p_gap_evt->conn_handle;

                //NRF_LOG_PRINTF("Starting DB discovery for HRS\r\n");
                err_code = ble_db_discovery_start(&m_ble_db_discovery_hrs, p_gap_evt->conn_handle);
                APP_ERROR_CHECK(err_code);
            }
            else if(memcmp(&periph_addr_rsc, peer_addr, sizeof(ble_gap_addr_t)) == 0)
            {
                //NRF_LOG_PRINTF("RSC central connected\r\n");
                // Reset the peer address we had saved.
                memset(&periph_addr_rsc, 0, sizeof(ble_gap_addr_t));

                m_conn_handle_rscs_c = p_gap_evt->conn_handle;

                //NRF_LOG_PRINTF("Starting DB discovery for RSCS\r\n");
                err_code = ble_db_discovery_start(&m_ble_db_discovery_rsc, p_gap_evt->conn_handle);
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
                if ((extracted_uuid      == BLE_UUID_HEART_RATE_SERVICE) &&
                    (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID))
                {
                    do_connect = true;
                    memcpy(&periph_addr_hrs, peer_addr, sizeof(ble_gap_addr_t));
                }
                else if ((extracted_uuid       == BLE_UUID_RUNNING_SPEED_AND_CADENCE) &&
                         (m_conn_handle_rscs_c == BLE_CONN_HANDLE_INVALID))
                {
                    do_connect = true;
                    memcpy(&periph_addr_rsc, peer_addr, sizeof(ble_gap_addr_t));
                }

                if (do_connect)
                {
                    // Initiate connection.
                    err_code = sd_ble_gap_connect(peer_addr, &m_scan_param, &m_connection_param);
                    if (err_code != NRF_SUCCESS)
                    {
                        //APPL_LOG("[APPL]: Connection Request Failed, reason %d\r\n", err_code);
                    }
                }
            }
        } break; // BLE_GAP_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                //APPL_LOG("[APPL]: Connection Request timed out.\r\n");
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
            //NRF_LOG_PRINTF("Peripheral connected\r\n");
            LEDS_OFF(PERIPHERAL_ADVERTISING_LED);
            LEDS_ON(PERIPHERAL_CONNECTED_LED);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //NRF_LOG_PRINTF("Peripheral disconnected\r\n");
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
        ble_hrs_on_ble_evt (&m_hrs, p_ble_evt);
        ble_rscs_on_ble_evt(&m_rscs, p_ble_evt);
        ble_cscs_on_ble_evt(&m_cscs, p_ble_evt);
        ble_bas_on_ble_evt(&m_bas, p_ble_evt);
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
static void hrs_c_init(void)
{
    uint32_t         err_code;
    ble_hrs_c_init_t hrs_c_init_obj;

    hrs_c_init_obj.evt_handler = hrs_c_evt_handler;

    err_code = ble_hrs_c_init(&m_ble_hrs_c, &hrs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief RSC collector initialization.
 */
static void rscs_c_init(void)
{
    uint32_t            err_code;
    ble_rscs_c_init_t   rscs_c_init_obj;

    rscs_c_init_obj.evt_handler = rscs_c_evt_handler;

    err_code = ble_rscs_c_init(&m_ble_rsc_c, &rscs_c_init_obj);
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
    ble_hrs_init_t  hrs_init;
    ble_rscs_init_t rscs_init;
    ble_cscs_init_t cscs_init;
    ble_bas_init_t  bas_init;
    uint8_t         body_sensor_location;

    // Initialize the Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

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
    
    // Initialize Cycling Speed and Cadence Service.
    memset(&cscs_init, 0, sizeof(cscs_init));

    cscs_init.evt_handler = NULL;
    cscs_init.feature     = BLE_CSCS_FEATURE_WHEEL_REV_BIT | BLE_CSCS_FEATURE_CRANK_REV_BIT |
                            BLE_CSCS_FEATURE_MULTIPLE_SENSORS_BIT;

    // Here the sec level for the Cycling Speed and Cadence Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cscs_init.csc_meas_attr_md.cccd_write_perm);   // for the measurement characteristic, only the CCCD write permission can be set by the application, others are mandated by service specification
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cscs_init.csc_feature_attr_md.read_perm);      // for the feature characteristic, only the read permission can be set by the application, others are mandated by service specification
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cscs_init.csc_ctrlpt_attr_md.write_perm);      // for the SC control point characteristic, only the write permission and CCCD write can be set by the application, others are mandated by service specification
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cscs_init.csc_ctrlpt_attr_md.cccd_write_perm); // for the SC control point characteristic, only the write permission and CCCD write can be set by the application, others are mandated by service specification
    
    cscs_init.ctrplt_supported_functions    = BLE_SRV_SC_CTRLPT_CUM_VAL_OP_SUPPORTED
                                              |BLE_SRV_SC_CTRLPT_SENSOR_LOCATIONS_OP_SUPPORTED
                                              |BLE_SRV_SC_CTRLPT_START_CALIB_OP_SUPPORTED;
    cscs_init.ctrlpt_evt_handler            = sc_ctrlpt_event_handler;
    cscs_init.list_supported_locations      = supported_locations;
    cscs_init.size_list_supported_locations = sizeof(supported_locations) / sizeof(ble_sensor_location_t);            
    
    ble_sensor_location_t sensor_location = BLE_SENSOR_LOCATION_FRONT_WHEEL;                    // initializes the sensor location to add the sensor location characteristic.
    cscs_init.sensor_location = &sensor_location;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cscs_init.csc_sensor_loc_attr_md.read_perm);    // for the sensor location characteristic, only the read permission can be set by the application, others are mendated by service specification

    err_code = ble_cscs_init(&m_cscs, &cscs_init);
    APP_ERROR_CHECK(err_code);
    
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;
    uint32_t csc_meas_timer_ticks;
    uint32_t rsc_meas_timer_ticks;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    csc_meas_timer_ticks = APP_TIMER_TICKS(SPEED_AND_CADENCE_MEAS_INTERVAL, APP_TIMER_PRESCALER);

    err_code = app_timer_start(m_csc_tx_timer_id, csc_meas_timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);

    rsc_meas_timer_ticks = APP_TIMER_TICKS(SPEED_AND_CADENCE_MEAS_INTERVAL, APP_TIMER_PRESCALER);

    err_code = app_timer_start(m_rsc_tx_timer_id, rsc_meas_timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_start(m_heart_rate_tx_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_rr_tx_timer_id, RR_INTERVAL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_start(m_i2c_read_timer_id, RR_INTERVAL_INTERVAL, NULL);
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

    err_code = NRF_LOG_INIT();
    APP_ERROR_CHECK(err_code);

    i2c_init();
	temp_i2c_init();	
    accel_i2c_init();

    //NRF_LOG_PRINTF("Relay Example\r\n");

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    buttons_leds_init(&erase_bonds);

    if (erase_bonds == true)
    {
        //NRF_LOG("Bonds erased!\r\n");
    }

    ble_stack_init();

    peer_manager_init(erase_bonds);

    db_discovery_init();
    hrs_c_init();
    rscs_c_init();

    timers_init();
    gap_params_init();
    conn_params_init();
    services_init();
    advertising_init();

    /** Start scanning for peripherals and initiate connection to devices which
     *  advertise Heart Rate or Running speed and cadence UUIDs. */
    scan_start();

    // Turn on the LED to signal scanning.
    LEDS_ON(CENTRAL_SCANNING_LED);

    application_timers_start();
    // Start advertising.
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    for (;;)
    {
        // Wait for BLE events.
        power_manage();
    }
}
