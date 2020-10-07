/** @file
 *
 * This file contains the main application code to interface with the IMEC
 * analyte array sensor (pH, K+, Na+) and the Lura Health mobile application.
 *
 */

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


#include <stdint.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include "nordic_common.h"
#include "boards.h"

#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_gatt.h"
#include "nrf_saadc.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "nrfx_ppi.h"
#include "nrf_timer.h"
#include "nrfx_saadc.h"

#include "ble_nus.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"

#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "app_fifo.h"
#include "app_pwm.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Included for peer manager
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_conn_state.h"

// Included for persisent flash reads/writes
#include "fds.h"
#include "nrf_fstorage.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif



#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Lura_Test"                   /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                510                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (200 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define SEC_PARAM_BOND                  0                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define SAMPLES_IN_BUFFER               50                                          /**< SAADC buffer > */

#define DATA_INTERVAL                   1000


#define NRF_SAADC_CUSTOM_CHANNEL_CONFIG_SE(PIN_P) \
{                                                   \
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      \
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      \
    .gain       = NRF_SAADC_GAIN1_5,                \
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,     \
    .acq_time   = NRF_SAADC_ACQTIME_10US,           \
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      \
    .burst      = NRF_SAADC_BURST_DISABLED,         \
    .pin_p      = (nrf_saadc_input_t)(PIN_P),       \
    .pin_n      = NRF_SAADC_INPUT_DISABLED          \
}

/* UNDEFS FOR DEBUGGING */
#undef RX_PIN_NUMBER
#undef RTS_PIN_NUMBER
#undef LED_4          
#undef LED_STOP       

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_timer_id);

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


/* Lura Health nRF52810 port assignments */
#define ENABLE_ISFET_PIN 8
#define TMUX_A0_PIN      18
#define TMUX_A1_PIN      17
#define ENABLE_TMUX_PIN  16
#define CHIP_POWER_PIN   12
#define HW_TIMEOUT       10000

/* GLOBALS */
uint32_t AVG_PH_VAL        = 0;
uint32_t AVG_NA_VAL        = 0;
uint32_t AVG_K_VAL         = 0;
uint32_t AVG_BATT_VAL      = 0;
uint32_t AVG_TEMP_VAL      = 0;
bool     PH_IS_READ        = false;
bool     NA_IS_READ        = false;
bool     K_IS_READ         = false;
bool     BATTERY_IS_READ   = false;
bool     CONNECTION_MADE   = false;

/* GLOBALS USED FOR CALIBRATION */
#define  PH_CAL 0
#define  NA_CAL 1
#define  K_CAL  2
float    CAL_PERFORMED[3] = {0.0, 0.0, 0.0};
uint8_t CURR_ANALYTE     = 0;    // Set to PH_CAL, NA_CAL, K_CAL as appropriate

#define PT_1 0
#define PT_2 1
#define PT_3 2
double CAL_PTS_ARR[3]   = {0.0, 0.0, 0.0}; // Stores PT1, PT2, PT3 real calibration values
double MV_VALS_ARR[3]   = {0.0, 0.0, 0.0}; // Stores PT1, PT2, PT3 mv calibration values

float MVALS_ARR[3]   = {0.0, 0.0, 0.0};
float BVALS_ARR[3]   = {0.0, 0.0, 0.0};
float RVALS_ARR[3]   = {0.0, 0.0, 0.0};



bool CAL_MODE         = false;
bool PT1_READ         = false;
bool PT2_READ         = false;
bool PT3_READ         = false;
int  NUM_CAL_PTS      = 0;

/* Used for reading/writing calibration values to flash */
uint16_t MVAL_FILEID_ARR[3]    = {0x1000, 0x1001, 0x1002};
uint16_t MVAL_RECKEY_ARR[3]    = {0x1100, 0x1101, 0x1102};
uint16_t BVAL_FILEID_ARR[3]    = {0x2000, 0x2001, 0x2002};
uint16_t BVAL_RECKEY_ARR[3]    = {0x2200, 0x2201, 0x2202};
uint16_t RVAL_FILEID_ARR[3]    = {0x3000, 0x3001, 0x3002};
uint16_t RVAL_RECKEY_ARR[3]    = {0x3300, 0x3301, 0x3302};
uint16_t CALDONE_FILEID_ARR[3] = {0x4000, 0x4001, 0x4002};
uint16_t CALDONE_RECKEY_ARR[3] = {0x4400, 0x4401, 0x4402};

static volatile uint8_t write_flag = 0;

// Forward declarations
void enable_pH_voltage_reading   (void);
void disable_pH_voltage_reading  (void);
void saadc_init                  (void);
void enable_isfet_circuit        (void);
void disable_isfet_circuit       (void);
void turn_chip_power_on          (void);
void turn_chip_power_off          (void);
void restart_saadc               (void);
void restart_pH_interval_timer   (void);
void write_cal_values_to_flash    (void);
void linreg                      (int num, double x[], double y[]);
void perform_calibration         (uint8_t cal_pts);
double calculate_px_from_mv(uint32_t mv_val, uint8_t analyte);
double calculate_celsius_from_mv (uint32_t mv);
static void advertising_start    (bool erase_bonds);
uint32_t saadc_result_to_mv      (uint32_t saadc_result);

/*
 * Functions for translating from temperature sensor millivolt output 
 * to real-world temperature values. The thermistor resistance should first
 * be calculated from the temperature millivolt reading. Then, A simplified 
 * version of the Steinhart and Hart equation can be used to calculate 
 * Kelvins. Data is sent in Celsius for ease of debugging (data sheet values
 * are listed in Celsius), and the mobile application may convert to Fahrenheit
 */

// Helper function to convert millivolts to thermistor resistance
double mv_to_therm_resistance(uint32_t mv)
{
    double therm_res = 0;
    double Vtemp     = 0;
    double R1        = 10000.0;
    double Vin       = 1800;

    Vtemp = (double) mv;
    therm_res = (Vtemp * R1) / (Vin - Vtemp);

    // Catch invalid resistance values, set to 500 ohm if negative
    if(therm_res < 500)
        therm_res = 500;

    return therm_res;
}

// Helper function to convert thermistor resistance to Kelvins
double therm_resistance_to_kelvins(double therm_res)
{
    double kelvins_constant = 273.15;
    double ref_temp         = 25.0 + kelvins_constant;
    double ref_resistance   = 10000.0;
    double beta_constant    = 3380.0;
    double real_kelvins     = 0;

    real_kelvins = (beta_constant * ref_temp) / 
                      (beta_constant + (ref_temp * log(therm_res/ref_resistance)));

    return real_kelvins;
}

// Helper function to convert Kelvins to Celsius
double kelvins_to_celsius(double kelvins)
{
    double kelvins_constant = 273.15;
    return kelvins - kelvins_constant;
}

// Function to fully convert temperature millivolt output to degrees celsius
double calculate_celsius_from_mv(uint32_t mv)
{
    double therm_res = 0;
    double kelvins   = 0;
    double celsius   = 0;
    therm_res = mv_to_therm_resistance(mv);
    kelvins   = therm_resistance_to_kelvins(therm_res);
    celsius   = kelvins_to_celsius(kelvins);

    // Round celsius to nearest 0.1
    celsius = celsius * 100.0;
    if ((uint32_t)celsius % 10 > 5)
        celsius = celsius + 10.0;
    celsius = celsius / 10.0;
    celsius = round(celsius);
    celsius = celsius / 10.0 + 0.000000001; // Workaround for floating point error

    return celsius;
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. 
 *          You need to analyse how your product is supposed to react in case of 
 *          Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
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
    NRF_LOG_INFO("ENTERED PM_EVT_HANDLER");
    NRF_LOG_INFO("evt id: %u\n", p_evt->evt_id);
    //NRF_LOG_FLUSH();
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            NRF_LOG_INFO("PM_EVT_CONN_SEC_SUCCEEDED");
            NRF_LOG_FLUSH();
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            NRF_LOG_INFO("PM_EVT_PEERS_DELETE_SUCCEEDED");
            NRF_LOG_FLUSH();            
            advertising_start(false);
            break;

        case PM_EVT_BONDED_PEER_CONNECTED:  
            NRF_LOG_INFO("PM_EVT_BONDED_PEER_CONNECTED");
            NRF_LOG_FLUSH();
            break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Allow pairing request from an already bonded peer.
            NRF_LOG_INFO("PM_EVT_CONN_SEC_CONFIG_REQ");
            NRF_LOG_FLUSH();
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        default:
            break;
    }
}


/**@brief Function for initializing the timer module.
 */
void timers_init(void)
{
    uint32_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access 
 *          Profile) parameters of the device. It also sets the permissions 
 *          and appearance.
 */
void gap_params_init(void)
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

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may 
 *          need to inform the application about an error.
 *
 * @param[in] nrf_error Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

// Helper function
void substring(char s[], char sub[], int p, int l) {
   int c = 0;
   
   while (c < l) {
      sub[c] = s[p+c-1];
      c++;
   }
   sub[c] = '\0';
}

/* Takes mmol/L value and converts to pK or pNa
 */
double mmol_per_L_to_px(double mmol_L)
{
    // Convert mmol/L to mol/L
    double mol_L = mmol_L / 1000;
    return -log10(mol_L);
}

/* Takes pK or pNa value and converts to mmol/L
 */
double px_to_mmol_per_L(double px)
{
    return pow(10.0, -px) * 1000;
}


/* Sets PH_IS_READ, K_IS_READ, and NA_IS_READ to ensure tmux A0 and A1
 * pins and SAADC input read  the appropriate sensor signal
 */
void set_saadc_and_tmux_logic_for_calibration(void)
{
    if (CURR_ANALYTE == PH_CAL) {
        PH_IS_READ = false;
        NA_IS_READ = false;
        K_IS_READ  = false;
    }
    else if (CURR_ANALYTE == NA_CAL) {
        PH_IS_READ = true;
        NA_IS_READ = false;
        K_IS_READ  = false;
    }
    else if (CURR_ANALYTE == K_CAL) {
        PH_IS_READ = true;
        NA_IS_READ = true;
        K_IS_READ  = false;
    }
}

/* Reads only the SAADC output for the analyte sensor currently under
 * calibration. Converts results to mv and stores in MV_VALS_ARR
 */
void read_saadc_for_calibration(void) 
{
    int NUM_SAMPLES = 150;
    nrf_saadc_value_t temp_val = 0;
    ret_code_t err_code;
    disable_pH_voltage_reading();
    double AVG_CAL_MV_READING = 0;
    PH_IS_READ = false;
    enable_pH_voltage_reading();
    for (int i = 0; i < NUM_SAMPLES; i++) {
      err_code = nrfx_saadc_sample_convert(0, &temp_val);
      APP_ERROR_CHECK(err_code);
      AVG_CAL_MV_READING += saadc_result_to_mv(temp_val);
    }
    AVG_CAL_MV_READING = AVG_CAL_MV_READING / NUM_SAMPLES;
    // Assign averaged readings to the correct calibration point
    if(!PT1_READ){
      MV_VALS_ARR[PT_1] = AVG_CAL_MV_READING;
      PT1_READ = true;
    }
    else if (PT1_READ && !PT2_READ){
      MV_VALS_ARR[PT_2] = AVG_CAL_MV_READING;
      PT2_READ = true;
    }
    else if (PT1_READ && PT2_READ && !PT3_READ){
       MV_VALS_ARR[PT_3] = AVG_CAL_MV_READING;
       PT3_READ = true;
    }    
    disable_pH_voltage_reading();
}

/* Helper function to clear calibration global state variables
 */
void reset_calibration_state()
{
    CAL_MODE        = false;
    PT1_READ        = false;
    PT2_READ        = false;
    PT3_READ        = false;
    PH_IS_READ      = false;
    NA_IS_READ      = false;
    K_IS_READ       = false;
    BATTERY_IS_READ = false;
    CAL_PERFORMED[CURR_ANALYTE] = 1.0;
    for (int i = 0; i < 3; i++) {
        CAL_PTS_ARR[i] = 0.0;
        MV_VALS_ARR[i] = 0.0;
    }
}

/* Use the values read from read_saadc_for_calibration to reset the M, B and R values
 * to recalibrate accuracy of ISFET voltage output to pH value conversions
 */
void perform_calibration(uint8_t cal_pts)
{
  if (cal_pts == 1) {
    // Compare mV for pH value to mV calculated for same pH with current M & B values,
    // then adjust B value by the difference in mV values (shift intercept of line)
    double incorrect_pX  = calculate_px_from_mv((uint32_t)MV_VALS_ARR[PT_1], CURR_ANALYTE);
    double cal_adjustment = CAL_PTS_ARR[PT_1] - incorrect_pX;
    BVALS_ARR[CURR_ANALYTE] = BVALS_ARR[CURR_ANALYTE] + cal_adjustment;
    NRF_LOG_INFO("MVAL: " NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(MVALS_ARR[CURR_ANALYTE]));
    NRF_LOG_INFO("BVAL: " NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(BVALS_ARR[CURR_ANALYTE]));
    NRF_LOG_INFO("RVAL: " NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(RVALS_ARR[CURR_ANALYTE]));
  }
  else if (cal_pts == 2) {
    // Create arrays of pH value and corresponding mV values (change all line properties)
    double x_vals[] = {MV_VALS_ARR[PT_1], MV_VALS_ARR[PT_2]};
    double y_vals[] = {CAL_PTS_ARR[PT_1], CAL_PTS_ARR[PT_2]};
    linreg(cal_pts, x_vals, y_vals);
  }
  else if (cal_pts == 3) {
    // Create arrays of pH value and corresponding mV values (change all line properties)
    double x_vals[] = {MV_VALS_ARR[PT_1], MV_VALS_ARR[PT_2], MV_VALS_ARR[PT_3]};
    double y_vals[] = {CAL_PTS_ARR[PT_1], CAL_PTS_ARR[PT_2], CAL_PTS_ARR[PT_3]};
    for (int i = 0; i < cal_pts; i++) {
      NRF_LOG_INFO("x val %d (mv): " NRF_LOG_FLOAT_MARKER "", i, NRF_LOG_FLOAT(x_vals[i]));
      NRF_LOG_INFO("y val %d (mv): " NRF_LOG_FLOAT_MARKER "", i, NRF_LOG_FLOAT(y_vals[i]));
    }
    linreg(cal_pts, x_vals, y_vals);
  }
}

/* Parses packet starting with "STARTCAL" to set analyte for calibration,
 * and number of calibration points to record 
 *
 * Packet will be in format of "STARTCAL_NX", where N is P, K, or N
 * representing the three possible analytes and X is 1, 2, or 3
 * representing all potential calibration options
 */
void set_calibration_params(char **packet)
{
    char cal_pts_str[1];
    char cal_analyte_str[1];
    CAL_MODE = true;
    disable_pH_voltage_reading();
    // Parse integer and analyte from STARTCAL_NX packet as described above
    substring(*packet, cal_pts_str, 11, 1);
    substring(*packet, cal_analyte_str, 10, 1);
    NUM_CAL_PTS = atoi(cal_pts_str);
    if      (strstr(cal_analyte_str, "P") != NULL) {CURR_ANALYTE = PH_CAL;}
    else if (strstr(cal_analyte_str, "N") != NULL) {CURR_ANALYTE = NA_CAL;}
    else if (strstr(cal_analyte_str, "K") != NULL) {CURR_ANALYTE = K_CAL;}

    NRF_LOG_INFO("*** Current cal analyte: %d ***\n", CURR_ANALYTE);
}

/*
 * Checks packet contents to appropriately perform calibration
 */
void check_for_calibration(char **packet)
{
    uint32_t err_code;
    // Possible Strings to be received by pH device
    char *STARTCAL  = "STARTCAL_";
    char *PWROFF    = "PWROFF";
    char *PT        = "PT";
    // Possible strings to send to mobile application
    char *CALBEGIN = "CALBEGIN";
    char PT_CONFS[3][8] = {"PT1CONF", "PT2CONF", "PT3CONF"};
    // Variables to hold sizes of strings for ble_nus_send function
    uint16_t SIZE_BEGIN = 9;
    uint16_t SIZE_CONF  = 8;

    // If packet contains "PWROFF", turn off chip power
    if (strstr(*packet, PWROFF) != NULL){
        NRF_LOG_INFO("received pwroff\n");
        nrfx_gpiote_out_clear(CHIP_POWER_PIN);
    }

    // If packet starts with "STARTCAL", parse out calibration 
    if (strstr(*packet, STARTCAL) != NULL) {
        set_calibration_params(packet);
        err_code = ble_nus_data_send(&m_nus, CALBEGIN, 
                                     &SIZE_BEGIN, m_conn_handle);
        APP_ERROR_CHECK(err_code);
    }

    // If packet starts with "PT", parse out the point 1, 2 or 3 and the value
    if (strstr(*packet, PT) != NULL) {
        char pt_val_substring[5];
        char cal_pt_str[1];
        int  cal_pt = 0;
        // Parse out calibration point from packet
        substring(*packet, cal_pt_str, 3, 1);
        cal_pt = atoi(cal_pt_str); 
        // Parse out point value in appropriate length (3 for pH, 4 for Na/K) then
        // assign point value to appropriate variable, and convert 
        // from mmol/L to pK or pNa if necessary
        if (CURR_ANALYTE == PH_CAL) {
            substring(*packet, pt_val_substring, 5, 3); // PTX_A.B
            CAL_PTS_ARR[cal_pt - 1] = atof(pt_val_substring); 
        }
        else if (CURR_ANALYTE == K_CAL || CURR_ANALYTE == NA_CAL) {
            substring(*packet, pt_val_substring, 5, 4); // PTX_AB.C or PTX_A.BC
            CAL_PTS_ARR[cal_pt - 1] = mmol_per_L_to_px(atof(pt_val_substring)); 
        }
        // Read calibration data and send confirmation packet
        read_saadc_for_calibration();
        err_code = ble_nus_data_send(&m_nus, PT_CONFS[cal_pt - 1], 
                                     &SIZE_CONF, m_conn_handle);
        APP_ERROR_CHECK(err_code);
        // Perform calibration and restart normal data transmission if cal is complete
        if (NUM_CAL_PTS == cal_pt) {
          perform_calibration(cal_pt);
          write_cal_values_to_flash();
          reset_calibration_state();
          restart_pH_interval_timer();
        }
    }
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART 
 *          BLE Service and send it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        // Array to store data received by smartphone will never exceed 9 characters
        char data[10];
        // Pointer to array
        char *data_ptr = data;
        uint32_t err_code;

        NRF_LOG_INFO("Received data from BLE NUS.\n");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, 
                                            p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                // Parse data into array
                data[i] = p_evt->params.rx_data.p_data[i];
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", 
                                                                    err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        // Check pack for calibration protocol details
        check_for_calibration(&data_ptr);
    }

    if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
        ret_code_t err_code;
        err_code = app_timer_start(m_timer_id, APP_TIMER_TICKS(DATA_INTERVAL), NULL);
        APP_ERROR_CHECK(err_code); 
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection 
 *          Parameters Module which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by 
 *       simply setting the disconnect_on_fail config parameter, but instead 
 *       we use the event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    NRF_LOG_INFO("INSIDE CONN PARAMS EVT");
    NRF_LOG_INFO("evt type: %u\n", p_evt->evt_type);

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                         BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
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


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay  = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code; 

    // Go to system-off mode (function will not return; wakeup causes reset).
    //err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed 
 *          to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
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
    uint32_t err_code;

    NRF_LOG_INFO("INSIDE BLE EVT HANDLER\n");
    NRF_LOG_INFO("evt id: %u\n", p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            NRF_LOG_INFO("INSIDE SYS ATTR MISSING");
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSINg
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            CONNECTION_MADE = true;

            NRF_LOG_INFO("CONNECTION MADE (ble_gap_evt) \n");

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if(p_ble_evt->evt.gap_evt.params.disconnected.reason  == 
                                                    BLE_HCI_CONNECTION_TIMEOUT)
            {
                NRF_LOG_INFO("connection timeout\n");
            }
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            CONNECTION_MADE = false;
            NRF_LOG_INFO("DISCONNECTED\n");
            NRF_LOG_FLUSH();
            app_timer_stop(m_timer_id);
            nrfx_saadc_uninit();
            NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);  
            NVIC_ClearPendingIRQ(SAADC_IRQn);
            disable_isfet_circuit();
            // restart advertising
            advertising_start(false);

            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, 
                                                                         &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            NRF_LOG_INFO("ble gattc evt timeout");
            // Disconnect on GATT Client timeout event.
            err_code = 
                sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            CONNECTION_MADE = false;
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = 
                sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            CONNECTION_MADE = false;
            APP_ERROR_CHECK(err_code);
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
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
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, 
                                                ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 0;
    sec_param.kdist_own.id   = 0;
    sec_param.kdist_peer.enc = 0;
    sec_param.kdist_peer.id  = 0;

    err_code = pm_sec_params_set(&sec_param);
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


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags               = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
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
 * @details If there is no pending log operation, then sleep until next the 
 *          next event occurs.
 */
static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

/* Sets TMUX enable pin to HIGH and initializes logic pins */
void enable_tmux(void)
{
    nrf_drv_gpiote_out_config_t config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
    if(nrf_drv_gpiote_is_init() == false) {
          nrf_drv_gpiote_init();
    }   
    nrf_drv_gpiote_out_init(ENABLE_TMUX_PIN, &config);
    nrf_drv_gpiote_out_init(TMUX_A0_PIN, &config);
    nrf_drv_gpiote_out_init(TMUX_A1_PIN, &config);
    
    nrf_drv_gpiote_out_set(ENABLE_TMUX_PIN); 
}

void disable_tmux(void)
{
    nrf_drv_gpiote_out_clear(ENABLE_TMUX_PIN);
    nrf_drv_gpiote_out_clear(TMUX_A0_PIN);
    nrf_drv_gpiote_out_clear(TMUX_A1_PIN); 
       
    nrf_drv_gpiote_out_uninit(ENABLE_TMUX_PIN);
    nrf_drv_gpiote_out_uninit(TMUX_A0_PIN);
    nrf_drv_gpiote_out_uninit(TMUX_A1_PIN);
}

/* Reads from channel S1 of TMUX */
void read_indicating_electrode_one(void)
{
    nrf_drv_gpiote_out_clear(TMUX_A0_PIN);
    nrf_drv_gpiote_out_clear(TMUX_A1_PIN);
}

/* Reads from channel S3 of TMUX */
void read_indicating_electrode_two(void)
{
    nrf_drv_gpiote_out_clear(TMUX_A0_PIN);
    nrf_drv_gpiote_out_set(TMUX_A1_PIN);
}

/* Reads from channel S4 of TMUX */
void read_indicating_electrode_three(void)
{
    nrf_drv_gpiote_out_set(TMUX_A0_PIN);
    nrf_drv_gpiote_out_set(TMUX_A1_PIN);
}


/* This function sets enable pin for ISFET circuitry to HIGH
 */
void enable_isfet_circuit(void)
{
    nrf_drv_gpiote_out_config_t config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
    if(nrf_drv_gpiote_is_init() == false) {
          nrf_drv_gpiote_init();
    }
    nrf_drv_gpiote_out_init(ENABLE_ISFET_PIN, &config);
    nrf_drv_gpiote_out_set(ENABLE_ISFET_PIN);
}

/* This function holds POWER ON line HIGH to keep chip turned on
 */
void turn_chip_power_on(void)
{
    nrf_drv_gpiote_out_config_t config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
    if(nrf_drv_gpiote_is_init() == false) {
          nrf_drv_gpiote_init();
    }
    nrf_drv_gpiote_out_init(CHIP_POWER_PIN, &config);
    nrf_drv_gpiote_out_set(CHIP_POWER_PIN);
}

/* This functions turns POWER ON line LOW to turn the chip completely off
 */
void turn_chip_power_off(void)
{
    nrfx_gpiote_out_clear(CHIP_POWER_PIN);
}

/* This function sets enable pin for ISFET circuitry to LOW
 */
void disable_isfet_circuit(void)
{
     // Redundant, but follows design
     nrfx_gpiote_out_clear(ENABLE_ISFET_PIN);
}


void restart_saadc(void)
{
    nrfx_saadc_uninit();
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    while(nrfx_saadc_is_busy()) {
        // make sure SAADC is not busy
    }
    enable_pH_voltage_reading(); 
}

/* Calculates the real pH, pK or pNa value from millivolts using
 * stored calibration data
 */
double calculate_px_from_mv(uint32_t mv_val, uint8_t analyte)
{
    return ((double)mv_val * MVALS_ARR[analyte]) + BVALS_ARR[analyte];
}

/* Calculates the real pH value from mV value, and then packs 
 * calibrated pH value into total_packet[0-3], rounded to nearest 0.25 pH.
 * Stores each digit from packet[p] to packet[p+3]
 */
void pack_calibrated_ph_val(uint32_t ph_mv_val, uint8_t* total_packet, uint8_t p)
{
    uint32_t ASCII_DIG_BASE = 48;
    double real_pH  = calculate_px_from_mv(ph_mv_val, PH_CAL);
    double pH_decimal_vals = (real_pH - floor(real_pH)) * 100;
    // Round pH decimal values to 0.25 pH accuracy
    pH_decimal_vals = round(pH_decimal_vals / 25) * 25;
    // If decimals round to 100, increment real pH value and set decimals to 0.00
    if (pH_decimal_vals == 100) {
      real_pH = real_pH + 1.0;
      pH_decimal_vals = 0.00;
    }
    // If pH is 9.99 or lower, format with 2 decimal places (4 bytes total)
    if (real_pH < 10.0) {
      total_packet[p]   = (uint8_t) ((uint8_t)floor(real_pH) + ASCII_DIG_BASE);
      total_packet[p+1] = 46;
      total_packet[p+2] = (uint8_t) (((uint8_t)pH_decimal_vals / 10) + ASCII_DIG_BASE);
      total_packet[p+3] = (uint8_t) (((uint8_t)pH_decimal_vals % 10) + ASCII_DIG_BASE);
    }
    // If pH is 10.0 or greater, format with 1 decimal place (still 4 bytes total)
    else {
      total_packet[p]   = (uint8_t) ((uint8_t)floor(real_pH / 10) + ASCII_DIG_BASE);
      total_packet[p+1] = (uint8_t) ((uint8_t)floor((uint8_t)real_pH % 10) + ASCII_DIG_BASE);
      total_packet[p+2] = 46;
      total_packet[p+3] = (uint8_t) (((uint8_t)pH_decimal_vals / 10) + ASCII_DIG_BASE);
    }
}

/* Calculates the pK value from stored mv value, then converts from pK to mmol/L.
 * Data is rounded to nearest 0.1 mmol/L decimal place. Stores each digit from 
 * packet[p] to packet[p+3]
 */
void pack_calibrated_k_val(uint32_t k_mv_val, uint8_t* total_packet, uint8_t p)
{
    uint32_t ASCII_DIG_BASE = 48;
    double pK_value = calculate_px_from_mv(k_mv_val, K_CAL);
    double k_mmol_L = px_to_mmol_per_L(pK_value);
    double k_decimal_vals = (k_mmol_L - floor(k_mmol_L)) * 100;
    // Round K decimal values to nearest 0.1 accuracy
    k_decimal_vals = round(k_decimal_vals / 10) * 10;
    // If decimals rounds to 100, increment k_mmol_l and set decimals to 0
    if (k_decimal_vals == 100) {
        k_mmol_L = k_mmol_L + 1.0;
        k_decimal_vals = 0.00;
    }
    total_packet[p]   = (uint8_t) ((uint8_t)floor(k_mmol_L / 10) + ASCII_DIG_BASE);
    total_packet[p+1] = (uint8_t) ((uint8_t)floor((uint8_t)k_mmol_L % 10) + ASCII_DIG_BASE);
    total_packet[p+2] = 46;
    total_packet[p+3] = (uint8_t) (((uint8_t)k_decimal_vals / 10) + ASCII_DIG_BASE);
}

/* Calculates the pK value from stored mv value, then converts from pK to mmol/L.
 * Data is rounded to nearest 0.1 mmol/L decimal place. Stores each digit from 
 * packet[p] to packet[p+3]
 */
void pack_calibrated_na_val(uint32_t na_mv_val, uint8_t* total_packet, uint8_t p)
{
    uint32_t ASCII_DIG_BASE = 48;
    double pNa_value = calculate_px_from_mv(na_mv_val, NA_CAL);
    double na_mmol_L = px_to_mmol_per_L(pNa_value);
    double na_decimal_vals = (na_mmol_L - floor(na_mmol_L)) * 100;
    // Round K decimal values to nearest 0.1 accuracy
    na_decimal_vals = round(na_decimal_vals / 10) * 10;
    // If decimals rounds to 100, increment k_mmol_l and set decimals to 0
    if (na_decimal_vals == 100) {
        na_mmol_L = na_mmol_L + 1.0;
        na_decimal_vals = 0.00;
    }
    total_packet[p]   = (uint8_t) ((uint8_t)floor(na_mmol_L / 10) + ASCII_DIG_BASE);
    total_packet[p+1] = (uint8_t) ((uint8_t)floor((uint8_t)na_mmol_L % 10) + ASCII_DIG_BASE);
    total_packet[p+2] = 46;
    total_packet[p+3] = (uint8_t) (((uint8_t)na_decimal_vals / 10) + ASCII_DIG_BASE);

}

/* Packs a mV value between 0 - 3000 into char string total_packet, from 
 * total_packet[p] to total_packet[p + l - 1]. Ex: 1234 --> ['1','2','3','4']
 *
 * uint8_t p is the position of most significat digit, i.e. '1' in 1234
 * uint8_t l is the length of the number to be stored
 */
void pack_mv_value(uint32_t mv_val, uint8_t* total_packet, uint8_t p, uint8_t l)
{
    uint32_t ASCII_DIG_BASE = 48;
    uint32_t temp = 0;            // hold intermediate divisions of variables
    // Packing protocol for number abcd: 
    //  [... 0, 0, 0, d] --> [... 0, 0, c, d] --> ... --> [... a, b, c, d]
    temp = mv_val;
    for(int i = p+l-1; i >= p; i--){
        if (i == p+l-1) total_packet[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        else {
            temp = temp / 10;
            total_packet[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        }
    }
}

/* Packs temperature value into total_packet[5-8], as degrees Celsius.
 * Stores each digit from packet[p] to packet[p+3]
 */
void pack_temperature_val(uint32_t temp_val, uint8_t* total_packet, uint8_t p)
{
    uint32_t ASCII_DIG_BASE = 48;
    double real_temp = calculate_celsius_from_mv(temp_val);
    NRF_LOG_INFO("temp celsius: " NRF_LOG_FLOAT_MARKER " \n", NRF_LOG_FLOAT(real_temp));
    double temp_decimal_vals = (real_temp - floor(real_temp)) * 100;
    total_packet[p]   = (uint8_t) ((uint8_t)floor(real_temp / 10) + ASCII_DIG_BASE);
    total_packet[p+1] = (uint8_t) ((uint8_t)floor((uint8_t)real_temp % 10) + ASCII_DIG_BASE);
    total_packet[p+2] = 46;
    total_packet[p+3] = (uint8_t) (((uint8_t)temp_decimal_vals / 10) + ASCII_DIG_BASE);
}


/* Pack values into byte array to send via bluetooth
 *
 * Format: {pH_cal, Na_cal, K_cal, pH_mv, Na_mv, K_mv, temp_c, batt_mv},
 * where every value is 4 chars
 */
void create_bluetooth_packet(uint32_t ph_val,   uint32_t na_val,
                             uint32_t k_val,    uint32_t batt_val,        
                             uint32_t temp_val, uint8_t* total_packet)
{
    /*
        {48,48,48,48,44,48,48,48,48,44,    ph_cal[0-3], na_cal[5-8],
         48,48,48,48,44,48,48,48,48,44,    k_cal[10-13], ph_mv[15-18],
         48,48,48,48,44,48,48,48,48,44,    na_mv[20-23], k_mv[25-28],
         48,48,48,48,44,48,48,48,48,10};   temp_c[30-33], batt_mv[35-38]
    */
    
    if (CAL_PERFORMED[PH_CAL]) {
        pack_calibrated_ph_val(ph_val, total_packet, 0);
        NRF_LOG_INFO("PH_CAL is 1.0, packing calibrated ph");
    }
    if (CAL_PERFORMED[NA_CAL]){
        NRF_LOG_INFO("NA_CAL is 1.0, packing calibrated Na");
        pack_calibrated_na_val(na_val, total_packet, 5);
    }
    if (CAL_PERFORMED[K_CAL]) {
        pack_calibrated_k_val(k_val, total_packet, 10);
        NRF_LOG_INFO("K_CAL is 1.0, packing calibrated K");
    }
    pack_mv_value(ph_val, total_packet, 15, 4);
    pack_mv_value(na_val, total_packet, 20, 4);
    pack_mv_value(k_val,  total_packet, 25, 4);
    pack_temperature_val(temp_val, total_packet, 30);
    pack_mv_value(batt_val, total_packet, 35, 4);  
}

uint32_t saadc_result_to_mv(uint32_t saadc_result)
{
    float adc_denom     = 4096.0;
    float adc_ref_mv    = 600.0;
    float adc_prescale  = 5.0;
    float adc_res_in_mv = (((float)saadc_result*adc_ref_mv)/adc_denom) * adc_prescale;

    return (uint32_t)adc_res_in_mv;
}

// Read saadc values for temperature, battery level, and pH using blocking
void read_saadc_for_regular_protocol(void) 
{
    uint16_t   total_size = 40;
    uint32_t   avg_saadc_reading = 0;
    // Byte array to store total packet
    // {pH_cal, Na_cal, K_cal, pH_mv, Na_mv, K_mv, temp_c, batt_mv}
    uint8_t total_packet[] = {48,46,48,48,44,48,48,46,48,44,  
                              48,48,46,48,44,48,48,48,48,44,    
                              48,48,48,48,44,48,48,48,48,44,    
                              48,48,48,48,44,48,48,48,48,10};  

    int NUM_SAMPLES = 150;
    nrf_saadc_value_t temp_val = 0;
    ret_code_t err_code;
    uint32_t AVG_MV_VAL = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
      err_code = nrfx_saadc_sample_convert(0, &temp_val);
      APP_ERROR_CHECK(err_code);
      AVG_MV_VAL += saadc_result_to_mv(temp_val);
    }
    AVG_MV_VAL = AVG_MV_VAL / NUM_SAMPLES;
    if (AVG_MV_VAL > 3000) AVG_MV_VAL = 0;
    // Store pH value
    if(!PH_IS_READ){
      AVG_PH_VAL = AVG_MV_VAL;
      NRF_LOG_INFO("read pH val, restarting: %d", AVG_PH_VAL);
      PH_IS_READ = true;
      restart_saadc();
    }
    // Store Na+ value
    else if (!NA_IS_READ){
      AVG_NA_VAL = AVG_MV_VAL;
      NRF_LOG_INFO("read Na+ val, restarting: %d", AVG_NA_VAL);
      NA_IS_READ = true;
      restart_saadc();
    }
    // Store K+ value
    else if (!K_IS_READ){
      AVG_K_VAL = AVG_MV_VAL;
      NRF_LOG_INFO("read K+ val, restarting: %d", AVG_K_VAL);
      K_IS_READ = true;
      restart_saadc();
    }
    // Store battery value
    else if (!BATTERY_IS_READ){
      AVG_BATT_VAL = AVG_MV_VAL;
      NRF_LOG_INFO("read batt val, restarting: %d", AVG_BATT_VAL);
      BATTERY_IS_READ = true;
      restart_saadc();
    }
    // Store temperature value
    else {
       AVG_TEMP_VAL = AVG_MV_VAL;
       NRF_LOG_INFO("read temp val, restarting: %d", AVG_TEMP_VAL);
       PH_IS_READ = false;
       BATTERY_IS_READ = false;
       NA_IS_READ = false;
       K_IS_READ = false;
       if (!CAL_MODE) {
            // Create bluetooth data
            create_bluetooth_packet(AVG_PH_VAL, AVG_NA_VAL, AVG_K_VAL, 
                                    AVG_BATT_VAL, AVG_TEMP_VAL, total_packet);

            // Send data
            err_code = ble_nus_data_send(&m_nus, total_packet, 
                                         &total_size, m_conn_handle);
            if(err_code != NRF_ERROR_NOT_FOUND)
                APP_ERROR_CHECK(err_code);
              
            NRF_LOG_INFO("BLUETOOTH DATA SENT\n");
       }
       disable_pH_voltage_reading();
    }    
}


void saadc_blocking_callback(nrf_drv_saadc_evt_t const * p_event)
{
    // Don't need to do anything
}


void init_saadc_for_blocking_sample_conversion(nrf_saadc_channel_config_t channel_config)
{
    ret_code_t err_code;
    err_code = nrf_drv_saadc_init(NULL, saadc_blocking_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

/* Reads pH transducer output
 */
void saadc_init(void)
{
    nrf_saadc_input_t ANALOG_INPUT;
    // Change pin depending on global control booleans

    // Read pH, Na+ and K+ all from same analog pin
    if (!PH_IS_READ || !NA_IS_READ || ! K_IS_READ) {
        ANALOG_INPUT = NRF_SAADC_INPUT_AIN2;
        if (!PH_IS_READ) {
            enable_tmux();
            read_indicating_electrode_one();
        }
        else if (!NA_IS_READ)
            read_indicating_electrode_two();
        else
            read_indicating_electrode_three();
        nrf_delay_ms(100);
    }
    // Read battery after sensor analytes
    else if (!BATTERY_IS_READ) {
        disable_tmux(); // TMUX not needed after reading analytes
        ANALOG_INPUT = NRF_SAADC_INPUT_AIN3;
    }
    // Read thermistor voltage
    else {
        ANALOG_INPUT = NRF_SAADC_INPUT_AIN1;        
    }

    nrf_saadc_channel_config_t channel_config =
            NRF_SAADC_CUSTOM_CHANNEL_CONFIG_SE(ANALOG_INPUT);
    
    init_saadc_for_blocking_sample_conversion(channel_config);
}


/* This function initializes and enables SAADC sampling
 */
void enable_pH_voltage_reading(void)
{
    saadc_init();
    if (!CAL_MODE)
        read_saadc_for_regular_protocol();
}

void restart_pH_interval_timer(void)
{
      ret_code_t err_code;
      err_code = app_timer_start(m_timer_id, APP_TIMER_TICKS(DATA_INTERVAL), NULL);
      APP_ERROR_CHECK(err_code);
      nrf_pwr_mgmt_run();
}

/* Function unitializes and disables SAADC sampling, restarts 1 second timer
 */
void disable_pH_voltage_reading(void)
{
    nrfx_saadc_uninit();
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    while(nrfx_saadc_is_busy()) {}
    disable_isfet_circuit();

    if (!CAL_MODE) {
      // Restart timer
      restart_pH_interval_timer();
    }
}

void single_shot_timer_handler()
{
    // disable timer
    ret_code_t err_code;
    err_code = app_timer_stop(m_timer_id);
    APP_ERROR_CHECK(err_code);

    // Delay to ensure appropriate timing 
    enable_isfet_circuit();       
        
    // Begin SAADC initialization/start

    /* * * * * * * * * * * * * * *
     *  UNCOMMENT TO SEND DATA
     */

    enable_pH_voltage_reading();

    /*
     *  COMMENT TO NOT SEND DATA
     * * * * * * * * * * * * * * */
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && 
        (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH 
                                                                - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, 
                                                    m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);


}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 
                                               NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

// Helper function for linreg
inline static double sqr(double x) {
    return x*x;
}

/*
 * Function for running linear regression on two and three point calibration data
 */
void linreg(int num, double x[], double y[])
{
    double   sumx  = 0.0;                     /* sum of x     */
    double   sumx2 = 0.0;                     /* sum of x**2  */
    double   sumxy = 0.0;                     /* sum of x * y */
    double   sumy  = 0.0;                     /* sum of y     */
    double   sumy2 = 0.0;                     /* sum of y**2  */

    for (int i=0;i<num;i++){ 
        sumx  += x[i];       
        sumx2 += sqr(x[i]);  
        sumxy += x[i] * y[i];
        sumy  += y[i];      
        sumy2 += sqr(y[i]); 
    } 

    double denom = (num * sumx2 - sqr(sumx));

    if (denom == 0) {
        // singular matrix. can't solve the problem.
        NRF_LOG_INFO("singular matrix, cannot solve regression\n");
    }

    MVALS_ARR[CURR_ANALYTE] = (num * sumxy  -  sumx * sumy) / denom;
    BVALS_ARR[CURR_ANALYTE] = (sumy * sumx2  -  sumx * sumxy) / denom;
    RVALS_ARR[CURR_ANALYTE] = (sumxy - sumx * sumy / num) /    
                                sqrt((sumx2 - sqr(sumx)/num) *
                                 (sumy2 - sqr(sumy)/num));

    NRF_LOG_INFO("MVAL **CAL**: " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(MVALS_ARR[CURR_ANALYTE]));
    NRF_LOG_INFO("BVAL **CAL**: " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(BVALS_ARR[CURR_ANALYTE]));
    NRF_LOG_INFO("RVAL **CAL**: " NRF_LOG_FLOAT_MARKER " \n", NRF_LOG_FLOAT(RVALS_ARR[CURR_ANALYTE]));
}


void my_fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result != FDS_SUCCESS)
            {
                NRF_LOG_INFO("ERROR IN EVENT HANDLER\n");
                NRF_LOG_FLUSH();
            }
            break;
        case FDS_EVT_WRITE:
            if (p_fds_evt->result == FDS_SUCCESS)
            {
                write_flag=1;
            }
            break;
        default:
            break;
    }
}

static void fds_write(float value, uint16_t FILE_ID, uint16_t REC_KEY)
{
    fds_record_t       record;
    fds_record_desc_t  record_desc;

    // Set up record.
    record.file_id              = FILE_ID;
    record.key                 = REC_KEY;
    record.data.length_words   = 1;

    if(FILE_ID == MVAL_FILEID_ARR[CURR_ANALYTE] && REC_KEY == MVAL_RECKEY_ARR[CURR_ANALYTE]){
      record.data.p_data = &MVALS_ARR[CURR_ANALYTE];
    }
    else if(FILE_ID == BVAL_FILEID_ARR[CURR_ANALYTE] && REC_KEY == BVAL_RECKEY_ARR[CURR_ANALYTE]){
      record.data.p_data = &BVALS_ARR[CURR_ANALYTE];
    }
    else if(FILE_ID == RVAL_FILEID_ARR[CURR_ANALYTE] && REC_KEY == RVAL_RECKEY_ARR[CURR_ANALYTE]){
      record.data.p_data = &RVALS_ARR[CURR_ANALYTE];
    }
    else if(FILE_ID == CALDONE_FILEID_ARR[CURR_ANALYTE] && REC_KEY == CALDONE_RECKEY_ARR[CURR_ANALYTE]) {
      record.data.p_data = &CAL_PERFORMED[CURR_ANALYTE];
    }
                    
    ret_code_t ret = fds_record_write(&record_desc, &record);
    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_INFO("ERROR WRITING TO FLASH\n");
    }
    NRF_LOG_INFO("SUCCESS WRITING TO FLASH\n");
    NRF_LOG_FLUSH();
}

static void fds_update(float value, uint16_t FILE_ID, uint16_t REC_KEY)
{
    fds_record_t       record;
    fds_record_desc_t  record_desc;
    fds_find_token_t    ftok ={0};

    // Set up record.
    record.file_id              = FILE_ID;
    record.key                 = REC_KEY;
    record.data.length_words   = 1;

    if(FILE_ID == MVAL_FILEID_ARR[CURR_ANALYTE] && REC_KEY == MVAL_RECKEY_ARR[CURR_ANALYTE]){
      record.data.p_data = &MVALS_ARR[CURR_ANALYTE];
    }
    else if(FILE_ID == BVAL_FILEID_ARR[CURR_ANALYTE] && REC_KEY == BVAL_RECKEY_ARR[CURR_ANALYTE]){
      record.data.p_data = &BVALS_ARR[CURR_ANALYTE];
    }
    else if(FILE_ID == RVAL_FILEID_ARR[CURR_ANALYTE] && REC_KEY == RVAL_RECKEY_ARR[CURR_ANALYTE]){
      record.data.p_data = &RVALS_ARR[CURR_ANALYTE];
    }
    else if(FILE_ID == CALDONE_FILEID_ARR[CURR_ANALYTE] && REC_KEY == CALDONE_RECKEY_ARR[CURR_ANALYTE]) {
      record.data.p_data = &CAL_PERFORMED[CURR_ANALYTE];
    }

    fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok);
                    
    ret_code_t ret = fds_record_update(&record_desc, &record);
    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_INFO("ERROR WRITING UPDATE TO FLASH\n");
    }
    NRF_LOG_INFO("SUCCESS WRITING UPDATE TO FLASH\n");
    NRF_LOG_FLUSH();
}

float fds_read(uint16_t FILE_ID, uint16_t REC_KEY)
{
    fds_flash_record_t  flash_record;
    fds_record_desc_t  record_desc;
    fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
    float *p_data;
    float data;
    uint32_t err_code;
    
    NRF_LOG_INFO("Start searching... \r\n");
    // Loop until all records with the given key and file ID have been found.
    while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
    {
        err_code = fds_record_open(&record_desc, &flash_record);
        if ( err_code != FDS_SUCCESS)
        {
                NRF_LOG_INFO("COULD NOT FIND OR OPEN RECORD\n");	
                return 0.0;
        }

        p_data = (float *) flash_record.p_data;
        data = *p_data;
        for (uint8_t i=0;i<flash_record.p_header->length_words;i++)
        {
                NRF_LOG_INFO("Data read: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(data));
        }
        NRF_LOG_INFO("\r\n");
        // Access the record through the flash_record structure.
        // Close the record when done.
        err_code = fds_record_close(&record_desc);
        if (err_code != FDS_SUCCESS)
        {
                NRF_LOG_INFO("ERROR CLOSING RECORD\n");
        }
        NRF_LOG_FLUSH();
    }
    NRF_LOG_INFO("SUCCESS CLOSING RECORD\n");
    return data;
}

static void fds_find_and_delete(uint16_t FILE_ID, uint16_t REC_KEY)
{
    fds_record_desc_t  record_desc;
    fds_find_token_t    ftok;
	
    ftok.page=0;
    ftok.p_addr=NULL;
    // Loop and find records with same ID and rec key and mark them as deleted. 
    while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
    {
        fds_record_delete(&record_desc);
        NRF_LOG_INFO("Deleted record ID: %d \r\n",record_desc.record_id);
    }
    // call the garbage collector to empty them, don't need to do this all the time, 
    // this is just for demonstration
    ret_code_t ret = fds_gc();
    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_INFO("ERROR DELETING RECORD\n");
    }
    NRF_LOG_INFO("RECORD DELETED SUCCESFULLY\n");
}

static void fds_init_helper(void)
{	
    ret_code_t ret = fds_register(my_fds_evt_handler);
    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_INFO("ERROR, COULD NOT REGISTER FDS\n");
                    
    }
    ret = fds_init();
    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_INFO("ERROR, COULD NOT INIT FDS\n");
    }
    
    NRF_LOG_INFO("FDS INIT\n");	
}

/* Check words used in fds after initialization, and if more than 4 (default)
 * words are used then read MVAL, BVAL and RVAL values stored in flash. Assign
 * stored values to global variables respectively
 */
static void check_calibration_state(void)
{
    fds_stat_t fds_info;
    fds_stat(&fds_info);
    NRF_LOG_INFO("open records: %u, words used: %u\n", fds_info.open_records, 
                                                       fds_info.words_used);
    for (int ANALYTE = 0; ANALYTE < 3; ANALYTE++){
        NRF_LOG_INFO("searching flash storage for analyte %d cal data...", ANALYTE);
        // fds_read will return 0 if the CAL_DONE record does not exist, 
        // or if the stored value is 0
        if(fds_read(CALDONE_FILEID_ARR[ANALYTE], CALDONE_RECKEY_ARR[ANALYTE])) {
          CAL_PERFORMED[ANALYTE] = 1.0;
          NRF_LOG_INFO("Setting CAL_PERFORMED[PH_CAL] to true\n");
          // Read values stored in flash and set to respective global variables
          MVALS_ARR[ANALYTE] = fds_read(MVAL_FILEID_ARR[ANALYTE], MVAL_RECKEY_ARR[ANALYTE]);
          BVALS_ARR[ANALYTE] = fds_read(BVAL_FILEID_ARR[ANALYTE], BVAL_RECKEY_ARR[ANALYTE]);
          RVALS_ARR[ANALYTE] = fds_read(RVAL_FILEID_ARR[ANALYTE], RVAL_RECKEY_ARR[ANALYTE]);
          NRF_LOG_INFO("Analyte %d MVAL: " NRF_LOG_FLOAT_MARKER " \n", ANALYTE, NRF_LOG_FLOAT(MVALS_ARR[ANALYTE]));
          NRF_LOG_INFO("Analyte %d BVAL: " NRF_LOG_FLOAT_MARKER " \n", ANALYTE, NRF_LOG_FLOAT(BVALS_ARR[ANALYTE]));
          NRF_LOG_INFO("Analyte %d RVAL: " NRF_LOG_FLOAT_MARKER " \n", ANALYTE, NRF_LOG_FLOAT(RVALS_ARR[ANALYTE]));
        }
    }
}

/* If calibration has already been performed then update existing records with new 
 * values. If calibration has not already been performed, then write values to
 * new records
 */
void write_cal_values_to_flash(void) 
{
    // Update the existing flash records
    if (CAL_PERFORMED[CURR_ANALYTE]) {
        NRF_LOG_INFO("cal already performed, updating records");
        fds_update(MVALS_ARR[CURR_ANALYTE], MVAL_FILEID_ARR[CURR_ANALYTE], MVAL_RECKEY_ARR[CURR_ANALYTE]);
        fds_update(BVALS_ARR[CURR_ANALYTE], BVAL_FILEID_ARR[CURR_ANALYTE], BVAL_RECKEY_ARR[CURR_ANALYTE]);
        fds_update(RVALS_ARR[CURR_ANALYTE], RVAL_FILEID_ARR[CURR_ANALYTE], RVAL_RECKEY_ARR[CURR_ANALYTE]);
        fds_update(CAL_PERFORMED[CURR_ANALYTE], CALDONE_FILEID_ARR[CURR_ANALYTE], CALDONE_RECKEY_ARR[CURR_ANALYTE]);
    }
    // Write values to new records
    else {
        NRF_LOG_INFO("cal not performed yet, writing new values");
        fds_write(MVALS_ARR[CURR_ANALYTE], MVAL_FILEID_ARR[CURR_ANALYTE], MVAL_RECKEY_ARR[CURR_ANALYTE]);
        NRF_LOG_INFO("wrote mval to flash");
        fds_write(BVALS_ARR[CURR_ANALYTE], BVAL_FILEID_ARR[CURR_ANALYTE], BVAL_RECKEY_ARR[CURR_ANALYTE]);
        NRF_LOG_INFO("wrote bval to flash");
        fds_write(RVALS_ARR[CURR_ANALYTE], BVAL_FILEID_ARR[CURR_ANALYTE], BVAL_RECKEY_ARR[CURR_ANALYTE]);
        NRF_LOG_INFO("wrote rval to flash");
        fds_write(CAL_PERFORMED[CURR_ANALYTE], CALDONE_FILEID_ARR[CURR_ANALYTE], CALDONE_RECKEY_ARR[CURR_ANALYTE]);
        NRF_LOG_INFO("wrote caldone to flash");
    }
}
      


/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds = false;
    uint32_t err_code;

    // Call function very first to turn on the chip
    turn_chip_power_on();

    log_init();
    timers_init();
    power_management_init();

    // Initialize fds and check for calibration values
    fds_init_helper();
    check_calibration_state();

    // Continue with adjusted calibration state
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();

    // Create application timer and begin advertising
    err_code = app_timer_create(&m_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                single_shot_timer_handler);
    APP_ERROR_CHECK(err_code);
    advertising_start(erase_bonds);

    // Enter main loop.
    while (true)
    {
        idle_state_handle();
    } 
}

/*
 * @}
 */
