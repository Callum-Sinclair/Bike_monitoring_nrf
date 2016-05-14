/*
 * Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/**
 * @brief BLE Running Speed and Cadence Collector application main file.
 *
 * This file contains the source code for a sample Running Speed and Cadence collector.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "softdevice_handler.h"
#include "app_util.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "pstorage.h"
#include "device_manager.h"
#include "app_trace.h"
#include "ble_rscs_c.h"
#include "app_util.h"
#include "app_timer.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"

#define CENTRAL_LINK_COUNT       1                                   /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT    0                                   /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/


#define UART_TX_BUF_SIZE          256                                /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE          1                                  /**< UART RX buffer size. */

#define STRING_BUFFER_LEN         50
#define BOND_DELETE_ALL_BUTTON_ID 0                                  /**< Button used for deleting all bonded centrals during startup. */

#define APP_TIMER_PRESCALER       0                                  /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE   2                                  /**< Size of timer operation queues. */

#define APPL_LOG                  app_trace_log                      /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

#define SEC_PARAM_BOND            1                                  /**< Perform bonding. */
#define SEC_PARAM_MITM            1                                  /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE               /**< No I/O capabilities. */
#define SEC_PARAM_OOB             0                                  /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE    7                                  /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE    16                                 /**< Maximum encryption key size. */

#define SCAN_INTERVAL             0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0050                             /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL   MSEC_TO_UNITS(30, UNIT_1_25_MS)    /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY             0                                  /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT       MSEC_TO_UNITS(4000, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 millisecond. */

#define TARGET_UUID               BLE_UUID_RUNNING_SPEED_AND_CADENCE /**< Target device name that application is looking for. */
#define UUID16_SIZE               2                                  /**< Size of 16 bit UUID */

/**@breif Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)

/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct
{
    uint8_t * p_data;   /**< Pointer to data. */
    uint16_t  data_len; /**< Length of data. */
}data_t;

typedef enum
{
    BLE_NO_SCAN,        /**< No advertising running. */
    BLE_WHITELIST_SCAN, /**< Advertising with whitelist. */
    BLE_FAST_SCAN,      /**< Fast advertising running. */
} ble_scan_mode_t;

static ble_db_discovery_t           m_ble_db_discovery;                  /**< Structure used to identify the DB Discovery module. */
static ble_rscs_c_t                 m_ble_rsc_c;                         /**< Structure used to identify the Running Speed and Cadence client module. */
static ble_gap_scan_params_t        m_scan_param;                        /**< Scan parameters requested for scanning and connection. */
static dm_application_instance_t    m_dm_app_id;                         /**< Application identifier. */
static dm_handle_t                  m_dm_device_handle;                  /**< Device Identifier identifier. */
static uint8_t                      m_peer_count = 0;                    /**< Number of peer's connected. */
static ble_scan_mode_t              m_scan_mode = BLE_FAST_SCAN;         /**< Scan mode used by application. */
static uint16_t                     m_conn_handle;                       /**< Current connection handle. */
static volatile bool                m_whitelist_temporarily_disabled = false; /**< True if whitelist has been temporarily disabled. */

static bool                         m_memory_access_in_progress = false; /**< Flag to keep track of ongoing operations on persistent memory. */

#define HR_BUFF_SIZE 2500
int16_t hr_vals0[HR_BUFF_SIZE];
int16_t hr_vals1[HR_BUFF_SIZE];

/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
    0,                                   // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};

static void scan_start(void);

#define APPL_LOG                        app_trace_log             /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

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


void grad_read(void)
{
    xyz_t xyz;
    float gradient;
    accel_get_xyz(&xyz);
    gradient  = atan2((float)(0 - xyz.y), (float)xyz.z) * 180 / 3.14159;
#ifdef DEBUG
    printf("Data:\n X = %d Y = %d Z = %d\n\n", xyz.x, xyz.y, xyz.z);
#endif
    printf("Gradient: %f\n\n", gradient);
}

void accel_i2c_test(void)
{
    accel_i2c_init();
    accel_get_dev_id();
    while(1)
    {
        //accel_get_xyz();
        nrf_delay_ms(500);
    }
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

void temp_read(void)
{
    uint8_t tx_vals[20] = {0};
    uint8_t rx_vals[20] = {0};
    
    tx_vals[0] = BMP085_CONTROL;
    tx_vals[1] = BMP085_READTEMPCMD;
    i2c_tx(TEMP_ADDRESS, tx_vals, 2);
    
    nrf_delay_ms(10);
    
    tx_vals[0] = BMP085_TEMPDATA;
    i2c_tx_rx(TEMP_ADDRESS, tx_vals, 1, rx_vals, 2);
#ifdef DEBUG
    printf("Temp raw read: 0x%x 0x%x, %d\n", rx_vals[0], rx_vals[1], (rx_vals[0]<<8) + rx_vals[1]);
#endif
    int32_t temp = temp_convert(23420);
    printf("Actual Temp: %d.%d\'C\n", temp/10, temp%10);
}

/**@brief Function for asserts in the SoftDevice.
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


/**@brief Callback handling device manager events.
 *
 * @details This function is called to notify the application of device manager events.
 *
 * @param[in]   p_handle      Device Manager Handle. For link related events, this parameter
 *                            identifies the peer.
 * @param[in]   p_event       Pointer to the device manager event.
 * @param[in]   event_status  Status of the event.
 */
static ret_code_t device_manager_event_handler(const dm_handle_t * p_handle,
                                               const dm_event_t  * p_event,
                                               const ret_code_t    event_result)
{
    uint32_t err_code;

    switch (p_event->event_id)
    {
        case DM_EVT_CONNECTION:
        {
            APPL_LOG("[APPL]: >> DM_EVT_CONNECTION\r\n");
#ifdef ENABLE_DEBUG_LOG_SUPPORT
            ble_gap_addr_t * peer_addr;
            peer_addr = &p_event->event_param.p_gap_param->params.connected.peer_addr;
#endif // ENABLE_DEBUG_LOG_SUPPORT
            APPL_LOG("[APPL]:[%02X %02X %02X %02X %02X %02X]: Connection Established\r\n",
                     peer_addr->addr[0], peer_addr->addr[1], peer_addr->addr[2],
                     peer_addr->addr[3], peer_addr->addr[4], peer_addr->addr[5]);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_event->event_param.p_gap_param->conn_handle;

            m_dm_device_handle = (*p_handle);

            // Discover peer's services.
            err_code = ble_db_discovery_start(&m_ble_db_discovery,
                                              p_event->event_param.p_gap_param->conn_handle);
            APP_ERROR_CHECK(err_code);

            m_peer_count++;

            if (m_peer_count < CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
            APPL_LOG("[APPL]: << DM_EVT_CONNECTION\r\n");
            break;
        }

        case DM_EVT_DISCONNECTION:
        {
            APPL_LOG("[APPL]: >> DM_EVT_DISCONNECTION\r\n");
            memset(&m_ble_db_discovery, 0, sizeof(m_ble_db_discovery));

            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);

            if (m_peer_count == CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
            m_peer_count--;
            APPL_LOG("[APPL]: << DM_EVT_DISCONNECTION\r\n");
            break;
        }

        case DM_EVT_SECURITY_SETUP:
        {
            APPL_LOG("[APPL]:[0x%02X] >> DM_EVT_SECURITY_SETUP\r\n", p_handle->connection_id);
            // Slave securtiy request received from peer, if from a non bonded device,
            // initiate security setup, else, wait for encryption to complete.
            err_code = dm_security_setup_req(&m_dm_device_handle);
            APP_ERROR_CHECK(err_code);
            APPL_LOG("[APPL]:[0x%02X] << DM_EVT_SECURITY_SETUP\r\n", p_handle->connection_id);
            break;
        }

        case DM_EVT_SECURITY_SETUP_COMPLETE:
        {
            APPL_LOG("[APPL]: >> DM_EVT_SECURITY_SETUP_COMPLETE\r\n");
            // Running Speed and Cadence service discovered. Enable Running Speed and Cadence notifications.
            err_code = ble_rscs_c_rsc_notif_enable(&m_ble_rsc_c);
            APP_ERROR_CHECK(err_code);
            APPL_LOG("[APPL]: << DM_EVT_SECURITY_SETUP_COMPLETE\r\n");
            break;
        }

        case DM_EVT_LINK_SECURED:
            APPL_LOG("[APPL]: >> DM_LINK_SECURED_IND\r\n");
            APPL_LOG("[APPL]: << DM_LINK_SECURED_IND\r\n");
            break;

        case DM_EVT_DEVICE_CONTEXT_LOADED:
            APPL_LOG("[APPL]: >> DM_EVT_LINK_SECURED\r\n");
            APP_ERROR_CHECK(event_result);
            APPL_LOG("[APPL]: << DM_EVT_DEVICE_CONTEXT_LOADED\r\n");
            break;

        case DM_EVT_DEVICE_CONTEXT_STORED:
            APPL_LOG("[APPL]: >> DM_EVT_DEVICE_CONTEXT_STORED\r\n");
            APP_ERROR_CHECK(event_result);
            APPL_LOG("[APPL]: << DM_EVT_DEVICE_CONTEXT_STORED\r\n");
            break;

        case DM_EVT_DEVICE_CONTEXT_DELETED:
            APPL_LOG("[APPL]: >> DM_EVT_DEVICE_CONTEXT_DELETED\r\n");
            APP_ERROR_CHECK(event_result);
            APPL_LOG("[APPL]: << DM_EVT_DEVICE_CONTEXT_DELETED\r\n");
            break;

        default:
            break;
    }

    return NRF_SUCCESS;
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
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            data_t adv_data;
            data_t type_data;

            // Initialize advertisement report for parsing.
            adv_data.p_data   = (uint8_t *)p_gap_evt->params.adv_report.data;
            adv_data.data_len = p_gap_evt->params.adv_report.dlen;

            err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
                                        &adv_data,
                                        &type_data);

            if (err_code != NRF_SUCCESS)
            {
                // Compare short local name in case complete name does not match.
                err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                                            &adv_data,
                                            &type_data);
            }

            // Verify if short or complete name matches target.
            if (err_code == NRF_SUCCESS)
            {
                uint16_t extracted_uuid;

                // UUIDs found, look for matching UUID
                for (uint32_t u_index = 0; u_index < (type_data.data_len / UUID16_SIZE); u_index++)
                {
                    UUID16_EXTRACT(&extracted_uuid, &type_data.p_data[u_index * UUID16_SIZE]);

                    APPL_LOG("\t[APPL]: %x\r\n", extracted_uuid);

                    if (extracted_uuid == TARGET_UUID)
                    {
                        // Stop scanning.
                        err_code = sd_ble_gap_scan_stop();

                        if (err_code != NRF_SUCCESS)
                        {
                            APPL_LOG("[APPL]: Scan stop failed, reason %d\r\n", err_code);
                        }
                        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                        APP_ERROR_CHECK(err_code);

                        m_scan_param.selective = 0;

                        // Initiate connection.
                        err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
                                                      &m_scan_param,
                                                      &m_connection_param);

                        m_whitelist_temporarily_disabled = false;

                        if (err_code != NRF_SUCCESS)
                        {
                            APPL_LOG("[APPL]: Connection Request Failed, reason %d\r\n", err_code);
                        }
                        break;
                    }
                }
            }
            break;
        }

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                APPL_LOG("[APPL]: Scan timed out.\r\n");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                APPL_LOG("[APPL]: Connection Request timed out.\r\n");
            }
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *  been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_rscs_c_on_ble_evt(&m_ble_rsc_c, p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
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
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

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


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Event handler to be registered with the module.
    register_param.evt_handler = device_manager_event_handler;

    // Service or protocol context for device manager to load, store and apply on behalf of application.
    // Here set to client as application is a GATT client.
    register_param.service_type = DM_PROTOCOL_CNTXT_GATT_CLI_ID;

    // Secuirty parameters to be used for security procedures.
    register_param.sec_param.bond             = SEC_PARAM_BOND;
    register_param.sec_param.mitm             = SEC_PARAM_MITM;
    register_param.sec_param.io_caps          = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob              = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size     = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size     = SEC_PARAM_MAX_KEY_SIZE;
    register_param.sec_param.kdist_periph.enc = 1;
    register_param.sec_param.kdist_periph.id  = 1;

    err_code = dm_register(&m_dm_app_id, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for disabling the use of whitelist for scanning.
 */
static void whitelist_disable(void)
{
    uint32_t err_code;

    if ((m_scan_mode == BLE_WHITELIST_SCAN) && !m_whitelist_temporarily_disabled)
    {
        m_whitelist_temporarily_disabled = true;

        err_code = sd_ble_gap_scan_stop();
        if (err_code == NRF_SUCCESS)
        {
            scan_start();
        }
        else if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
    m_whitelist_temporarily_disabled = true;
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            whitelist_disable();
            break;

        default:
            break;
    }
}


/**@brief Running Speed and Cadence Collector Handler.
 */
static void rscs_c_evt_handler(ble_rscs_c_t * p_rsc_c, ble_rscs_c_evt_t * p_rsc_c_evt)
{
    uint32_t err_code;

    switch (p_rsc_c_evt->evt_type)
    {
        case BLE_RSCS_C_EVT_DISCOVERY_COMPLETE:
            // Initiate bonding.
            err_code = dm_security_setup_req(&m_dm_device_handle);
            APP_ERROR_CHECK(err_code);

            // Running Speed and Cadence service discovered. Enable Running Speed and Cadence notifications.
            err_code = ble_rscs_c_rsc_notif_enable(p_rsc_c);
            APP_ERROR_CHECK(err_code);

            printf("Running Speed and Cadence service discovered \r\n");
            break;

        case BLE_RSCS_C_EVT_RSC_NOTIFICATION:
        {
            printf("\r\n");
            APPL_LOG("[APPL]: RSC Measurement received %d \r\n",
                     p_rsc_c_evt->params.rsc.inst_speed);

            printf("Instantanious Speed   = %d\r\n", p_rsc_c_evt->params.rsc.inst_speed);
            if (p_rsc_c_evt->params.rsc.is_inst_stride_len_present)
            {
                printf("Stride Length         = %d\r\n",
                       p_rsc_c_evt->params.rsc.inst_stride_length);
            }
            if (p_rsc_c_evt->params.rsc.is_total_distance_present)
            {
                printf("Total Distance = %u\r\n", (unsigned int)p_rsc_c_evt->params.rsc.total_distance);
            }
            printf("Instantanious Cadence = %d\r\n", p_rsc_c_evt->params.rsc.inst_cadence);
            printf("Flags\r\n");
            printf("  Stride Length Present = %d\r\n",
                   p_rsc_c_evt->params.rsc.is_inst_stride_len_present);
            printf("  Total Distance Present= %d\r\n",
                   p_rsc_c_evt->params.rsc.is_total_distance_present);
            printf("  Is Running            = %d\r\n", p_rsc_c_evt->params.rsc.is_running);
            break;
        }

        default:
            break;
    }
}


/**
 * @brief HRunning Speed and Cadence collector initialization.
 */
static void rscs_c_init(void)
{
    ble_rscs_c_init_t rscs_c_init_obj;

    rscs_c_init_obj.evt_handler = rscs_c_evt_handler;

    uint32_t err_code = ble_rscs_c_init(&m_ble_rsc_c, &rscs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init();

    APP_ERROR_CHECK(err_code);
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ble_gap_whitelist_t whitelist;
    ble_gap_addr_t    * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    ble_gap_irk_t     * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];
    uint32_t            err_code;
    uint32_t            count;

    // Verify if there is any flash access pending, if yes delay starting scanning until
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }

    // Initialize whitelist parameters.
    whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
    whitelist.irk_count  = 0;
    whitelist.pp_addrs   = p_whitelist_addr;
    whitelist.pp_irks    = p_whitelist_irk;

    // Request creating of whitelist.
    err_code = dm_whitelist_create(&m_dm_app_id,&whitelist);
    APP_ERROR_CHECK(err_code);

    if (((whitelist.addr_count == 0) && (whitelist.irk_count == 0)) ||
        (m_scan_mode != BLE_WHITELIST_SCAN) ||
        (m_whitelist_temporarily_disabled))
    {
        // No devices in whitelist, hence non selective performed.
        m_scan_param.active      = 0;             // Active scanning set.
        m_scan_param.selective   = 0;             // Selective scanning not set.
        m_scan_param.interval    = SCAN_INTERVAL; // Scan interval.
        m_scan_param.window      = SCAN_WINDOW;   // Scan window.
        m_scan_param.p_whitelist = NULL;          // No whitelist provided.
        m_scan_param.timeout     = 0x0000;        // No timeout.
    }
    else
    {
        // Selective scanning based on whitelist first.
        m_scan_param.active      = 0;             // Active scanning set.
        m_scan_param.selective   = 1;             // Selective scanning not set.
        m_scan_param.interval    = SCAN_INTERVAL; // Scan interval.
        m_scan_param.window      = SCAN_WINDOW;   // Scan window.
        m_scan_param.p_whitelist = &whitelist;    // Provide whitelist.
        m_scan_param.timeout     = 0x001E;        // 30 seconds timeout.
    }

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the UART.
 */
static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
       {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
           UART_BAUDRATE_BAUDRATE_Baud250000
       };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);

    app_trace_init();
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/** @brief Function for the Power manager.
 */
static void hr_poll(void)
{
    printf("new\n");
    static uint8_t hr_buf_num;
    while (NRF_SAADC->EVENTS_STARTED == 0);
    printf("tock \n");
    NRF_SAADC->EVENTS_STARTED = 0;
    static uint8_t buffer_count = 0;
    static uint8_t hr_buf[60];
    if (hr_buf_num == 0)
    {
        NRF_SAADC->RESULT.PTR = (uint32_t)hr_vals1;
        hr_buf_num = 1;
#ifdef ECG_GRAPH
        for (uint32_t i = 0; i < HR_BUFF_SIZE; i++)
        {
            printf("%d\n", hr_vals1[i] * hr_vals1[i] * hr_vals0[i]);
            nrf_delay_us(500);
        }
#else
        float average = 0;
        uint32_t peak = 0;
        for (uint32_t i = 0; i < HR_BUFF_SIZE; i++)
        {
            average = ((average * i) + hr_vals1[i]) / (i + 1);
            if (hr_vals1[i] > peak)
            {
                peak = hr_vals1[i];
            }
        }
        uint32_t th = (uint32_t)average + ((peak - average) / 2);
        bool over_th = false;
        uint16_t hb_count = 0;
        for (uint32_t i = 0; i < HR_BUFF_SIZE; i++)
        {
            if ((hr_vals1[i] > th) && !over_th)
            {
                hb_count ++;
                over_th = true;
            }
            if ((hr_vals1[i] < th) && over_th)
            {
                over_th = false;
            }
        }
        printf("hb_count = %d, pulse = %d\n", hb_count, hb_count * 6);
        if ((hb_count > 8) && (hb_count < 30))
            hr_buf[buffer_count] = hb_count;
        else if (buffer_count > 0)
        {
            hr_buf[buffer_count] = hr_buf[buffer_count - 1];
        }
        else
        {
            hr_buf[buffer_count] = 13;
        }
#endif
    }
    else
    {
        NRF_SAADC->RESULT.PTR = (uint32_t)hr_vals0;
        hr_buf_num = 0;
#ifdef ECG_GRAPH
        for (uint32_t i = 0; i < HR_BUFF_SIZE; i++)
        {
            printf("%d\n", hr_vals0[i] * hr_vals0[i] * hr_vals0[i]);
            nrf_delay_us(1000);
        }
    }
#else
        float average = 0;
        uint32_t peak = 0;
        for (uint32_t i = 0; i < HR_BUFF_SIZE; i++)
        {
            average = ((average * i) + hr_vals0[i]) / (i + 1);
            if (hr_vals0[i] > peak)
            {
                peak = hr_vals0[i];
            }
        }
        uint32_t th = (uint32_t)average + ((peak - average) / 2);
        bool over_th = false;
        uint16_t hb_count = 0;
        for (uint32_t i = 0; i < HR_BUFF_SIZE; i++)
        {
            if ((hr_vals0[i] > th) && !over_th)
            {
                hb_count ++;
                over_th = true;
            }
            if ((hr_vals0[i] < th) && over_th)
            {
                over_th = false;
            }
        }
        printf("hb_count = %d, pulse = %d\n", hb_count, hb_count * 6);
        if ((hb_count > 8) && (hb_count < 30))
            hr_buf[buffer_count] = hb_count;
        else if (buffer_count > 0)
        {
            hr_buf[buffer_count] = hr_buf[buffer_count - 1];
        }
        else
        {
            hr_buf[buffer_count] = 13;
        }
    }
    uint16_t last_40;
    if (buffer_count > 4)
    {
        last_40 = hr_buf[buffer_count] + hr_buf[buffer_count - 1] + hr_buf[buffer_count - 2] + hr_buf[buffer_count - 3];
    }
    printf("\n\nHR %d    (total %d)", (uint16_t)((float)last_40 / 4.0 * 6.0), last_40);
    buffer_count++;
#endif
}


void hr_adc_init()
{
    NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELN_PSELN_AnalogInput6;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
    NRF_SAADC->CH[0].CONFIG = ((SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) | \
                               (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |\
                               (SAADC_CH_CONFIG_GAIN_Gain1_4 << SAADC_CH_CONFIG_GAIN_Pos) | \
                               (SAADC_CH_CONFIG_REFSEL_VDD1_4 << SAADC_CH_CONFIG_GAIN_Pos) | \
                               (SAADC_CH_CONFIG_TACQ_5us << SAADC_CH_CONFIG_TACQ_Pos) | \
                               (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos));
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_10bit;
    NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;
    NRF_SAADC->SAMPLERATE = ((2047 << SAADC_SAMPLERATE_CC_Pos) | \
                             (SAADC_SAMPLERATE_MODE_Timers << SAADC_SAMPLERATE_CC_Pos));
    
    NRF_SAADC->RESULT.PTR = (uint32_t)hr_vals0;
    NRF_SAADC->RESULT.MAXCNT = HR_BUFF_SIZE;
        
    
    NRF_TIMER_Type* ticker_timer = NRF_TIMER4;
    
    ticker_timer->MODE = TIMER_MODE_MODE_Timer;
    ticker_timer->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    ticker_timer->PRESCALER = 4; //gives T=0.001ms
    
    ticker_timer->CC[0] = 50;
    ticker_timer->CC[1] = 4000; // 4 ms
    
    ticker_timer->TASKS_CLEAR = 1;
    ticker_timer->TASKS_START = 1;
    
    ticker_timer->SHORTS = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos);
    
    //PPI setup
    NRF_PPI->CH[0].EEP = (uint32_t)&(ticker_timer->EVENTS_COMPARE[0]);
    NRF_PPI->CH[0].TEP = (uint32_t)&(NRF_SAADC->TASKS_SAMPLE);
    
    NRF_PPI->CH[1].EEP = (uint32_t)&(NRF_SAADC->EVENTS_END);
    NRF_PPI->CH[1].TEP = (uint32_t)&(NRF_SAADC->TASKS_START);
    
    NRF_PPI->CHENSET = 0x3;
    
    
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;
    NRF_SAADC->TASKS_START = 1;
}
#define HR_TEST
int main(void)
{
    bool erase_bonds;

    // Initialize.
    //APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    //buttons_leds_init(&erase_bonds);
    uart_init();
    printf("\n\n\nRunning Speed collector example\r\n");
#ifdef HR_TEST
    printf("\n\n\nHR Test\r\n");
    hr_adc_init();
    while(1)
    {
        hr_poll();
    }
#endif
    //ble_stack_init();
    //device_manager_init(erase_bonds);
    //db_discovery_init();
    //rscs_c_init();
    
#ifdef I2C_TEST
    printf("\n\n\nI2C Test\r\n");
    i2c_init();
	temp_i2c_init();	
    accel_i2c_init();
    nrf_delay_ms(5);
    while(1)
    {
        temp_read();
        nrf_delay_ms(5);
        grad_read();
        nrf_delay_ms(1000);
    }
#endif

    // Start scanning for peripherals and initiate connection
    // with devices that advertise Running Speed and Cadence UUID.
    //scan_start();
    
    for (;;)
    {
        printf("\n\n\ndefine either HR_TEST or I2C_TEST\r\n");
    }
}


