/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_app_cts_c_main main.c
 * @{
 * @ingroup ble_sdk_app_cts_c
 * @brief Current Time Profile sample application.
 *
 * This file contains the source code for a sample application that uses Current Time Service.
 * This is the client role of the profile, implemented on a peripheral device.
 * When a central device connects, the application will trigger a security procedure (if this is not done
 * by the central side first). Completion of the security procedure will trigger a service
 * discovery. When the Current Time Service and Characteristic have been discovered on the
 * server, pressing button 1 will trigger a read of the current time and print it on the UART.
 *
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "ble.h"
#include "ble_cts_c.h"
#include "ble_db_discovery.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "softdevice_handler.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"

// Efe Integration Headers
// ============================================================
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_twis.h"
#include "nrf_delay.h"
// ============================================================


#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or exclude the service_changed characteristic. If excluded, the server's database cannot be changed for the lifetime of the device. */

#define WAKEUP_BUTTON_ID                0                                           /**< Button used to wake up the application. */
#define BOND_DELETE_ALL_BUTTON_ID       1                                           /**< Button used to delete all bonded centrals during startup. */
#define CURRENT_TIME_READ_BUTTON_ID     0                                           /**< Button used to read the current time from the server/central. */

#define DEVICE_NAME                     "Nordic_CTS"                                /**< Name of the device. Will be included in the advertising data. */
#define APP_ADV_FAST_INTERVAL           0x0028                                      /**< Fast advertising interval (in units of 0.625 ms). The default value corresponds to 25 ms. */
#define APP_ADV_SLOW_INTERVAL           0x0C80                                      /**< Slow advertising interval (in units of 0.625 ms). The default value corresponds to 2 seconds. */
#define APP_ADV_SLOW_TIMEOUT            180                                         /**< The duration of the slow advertising period (in seconds). */
#define APP_ADV_FAST_TIMEOUT            30                                          /**< The duration of the fast advertising period (in seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SECURITY_REQUEST_DELAY          APP_TIMER_TICKS(4000, APP_TIMER_PRESCALER)  /**< Delay after connection until security request is sent, if necessary (ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Time-out for pairing request or security request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection requirement. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data availability. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// Efe Integration
// ============================================================================
#define TWIS_INSTANCE_ID                0                                           /**< The init instance for the I2C interface */
#define TWIS_ADDR                       0x35                                        /**< Slave device I2C address */

#define HBT_BUF_SIZE 64
char msg_ble_send[HBT_BUF_SIZE] = "Istikbal Goklerdedir Helllo";

static volatile int connected_to_cts;
static volatile int cts_rxed;


static ble_db_discovery_t m_ble_db_discovery;                                       /**< Structure used to identify the DB Discovery module. */
static ble_cts_c_t        m_cts;                                                    /**< Instance of Current Time Service. The instance uses this struct to store data related to the service. */
static pm_peer_id_t       m_peer_id;                                                /**< Device reference handle to the current bonded central. */
static uint16_t           m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;              /**< Handle of the current connection. */
static uint16_t           service_handle;
static uint32_t						reset_device;						

typedef enum
{
    BLE_HBT_EVT_NOTIFICATION_ENABLED,                             /**<notification enabled event. */
    BLE_HBT_EVT_NOTIFICATION_DISABLED                             /**<notification disabled event. */
} ble_hbt_evt_type_t;

typedef struct
{
	ble_hbt_evt_type_t evt_type;
} ble_hbt_evt_t;

typedef struct
{
	//ble_hbt_evt_handler_t 				evt_handler;
	uint16_t												    service_handler;
	bool 														time_synced;
	ble_srv_report_ref_t *  				                    p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic , Needed?*/
	uint8_t 												    mem_initial;
	ble_srv_cccd_security_mode_t 		                        hbt_attr_md;
	ble_gap_conn_sec_mode_t					                    hbt_report_read_perm;
	uint16_t                     		                        conn_handle;
	ble_gatts_char_handles_t				                    hbt_char_handle;
	ble_srv_utf8_str_t							                sample_str;
} ble_hbt_s;


static ble_hbt_s		  m_hbt;		
static volatile int       heartbeat_updated;
typedef enum {                                                                      /**< Responses that the slave will send to the master device */
    
    OK                                  = 0x8,
    NOT_OK                              = 0x9,
    
} responses;

typedef enum {                                                                      /**< Command Bytest that the slave will recieve from the master device*/
    CUR_TIME_REQ                        = 0x29,
    RX_CUR_TIME,
    HB_REQ,
    TX_HB,
    POW_OFF,
    RESTART,
} commands;


#define CUR_TIME_SIZE                   25
#define BUF_SIZE                        256
#define HB_SIZE                         24
#define CMD_RESP_SIZE                   1


static uint8_t m_rxbuff[BUF_SIZE];
static volatile uint32_t cur_time_ready;
volatile uint8_t current_time[CUR_TIME_SIZE];
uint8_t dummy_time[] = "10/29/1923 18:81:00\r\n";
volatile commands state;

static const nrf_drv_twis_t s_twi = NRF_DRV_TWIS_INSTANCE(TWIS_INSTANCE_ID);

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



/**@brief Function for initializing bsp module.
 */
void bsp_event_handler(bsp_event_t event);

void bsp_configuration()
{
    uint32_t err_code;
    
    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}

static uint32_t heartbeat_update(ble_hbt_s * temp_hbt, char temp_msg[])
{
	 ble_gatts_value_t gatts_value;

    // Initialize value struct.
	
		ble_srv_ascii_to_utf8(&temp_hbt->sample_str,(char *)temp_msg);
	
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = temp_hbt->sample_str.length;
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)temp_hbt->sample_str.p_str;
		NRF_LOG_INFO("this message is %s", (uint32_t)gatts_value.p_value);
    return sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID, temp_hbt->hbt_char_handle.value_handle, &gatts_value);
}

/**
 * @brief Event processing
 *
 *
 */
static void twis_event_handler(nrf_drv_twis_evt_t const * const p_event)
{
    //NRF_LOG_INFO("EVENT handler called!!!\r\n");
    responses response;
    switch (p_event->type)
    {
        case TWIS_EVT_READ_REQ:
            //NRF_LOG_INFO("TWIS_EVT_READ_REQ\r\n");
            
            switch (state)
						{
            case CUR_TIME_REQ:
                NRF_LOG_INFO("Sending ACK for CUR_REQ\r\n");
                if(cur_time_ready)
                {
                    response = OK;
                    nrf_drv_twis_tx_prepare(&s_twi, (uint8_t *)&response, CMD_RESP_SIZE);
                }
                else
                {
										// change this to NOT_OK for JOhn's version
                    response = OK;
                    nrf_drv_twis_tx_prepare(&s_twi, (uint8_t *)&response, CMD_RESP_SIZE);
                }
                break;
                
            case RX_CUR_TIME:
                NRF_LOG_INFO("Sending the current time\r\n");
                nrf_drv_twis_tx_prepare(&s_twi, (uint8_t *)current_time, CUR_TIME_SIZE);
                break;
            case HB_REQ:
                
                NRF_LOG_INFO("HB REQ ACK\r\n");
                response = OK;
                nrf_drv_twis_tx_prepare(&s_twi, (uint8_t *)&response, CMD_RESP_SIZE);
                break;
                
            case POW_OFF:
                NRF_LOG_INFO("POW_OFF ACK\r\n");
                // power of the chip
                response = OK;
                // pull the pin low
                nrf_gpio_pin_write(ARDUINO_13_PIN, 0);
                NRF_LOG_INFO("pulled the pin Low\r\n");
                nrf_drv_twis_tx_prepare(&s_twi, (uint8_t *)&response, CMD_RESP_SIZE);
            case RESTART:
                NRF_LOG_INFO("RESTART ACK\r\n");
                // power of the chip
                response = OK;
                nrf_drv_twis_tx_prepare(&s_twi, (uint8_t *)&response, CMD_RESP_SIZE);
                
            default:
                response = NOT_OK;
                nrf_drv_twis_tx_prepare(&s_twi, (uint8_t *)&response, CMD_RESP_SIZE);
                break;
        }
            break;
            
        case TWIS_EVT_READ_DONE:
            
            //NRF_LOG_INFO("TWIS_EVT_READ_DONE\r\n");
            break;
            
        case TWIS_EVT_WRITE_REQ:
            
            //NRF_LOG_INFO("TWIS_EVT_WRITE_REQ\r\n");
            
            
            if(state == HB_REQ)
            {
                (void)nrf_drv_twis_rx_prepare(&s_twi, m_rxbuff, HB_SIZE);
            }
            else
            {
                (void)nrf_drv_twis_rx_prepare(&s_twi, m_rxbuff, CMD_RESP_SIZE);
            }
            
            break;
            
        case TWIS_EVT_WRITE_DONE:
            
            //NRF_LOG_INFO("TWIS_EVT_WRITE_DONE\r\n");
            NRF_LOG_INFO("this is the number of bytes %d\r\n", p_event->data.rx_amount);
            if(p_event->data.rx_amount == CMD_RESP_SIZE) {
                switch (m_rxbuff[0])
                {
                    case CUR_TIME_REQ:
                        NRF_LOG_INFO("CUR_TIME_REQ\r\n");
                        state = CUR_TIME_REQ;
                        break;
                    case RX_CUR_TIME:
                        NRF_LOG_INFO("RX_CUR_TIME\r\n");
                        // change the state
                        state = RX_CUR_TIME;
                        break;
                    case HB_REQ:
                        NRF_LOG_INFO("HB_REQ\r\n");
                        // change the state
                        state = HB_REQ;
                        break;
                    case POW_OFF:
                        NRF_LOG_INFO("POW_OFF\r\n");
                        // change the state
                        state = POW_OFF;
                        break;
                    case RESTART:
												NRF_LOG_INFO("RESTART\r\n");
                        state = RESTART;
												NRF_LOG_FLUSH();
												reset_device = 1;
                        // change the state
                        break;
                    default:
                        break;
                }
            }
            else if(p_event->data.rx_amount == HB_SIZE) {
                
                // clear the msg_buff
                memset(msg_ble_send, 0, HBT_BUF_SIZE);
                NRF_LOG_INFO("cleared the msg buffer \r\n");
                
                for(int i=0; i<HB_SIZE; i++){
                    msg_ble_send[i] = m_rxbuff[i];
                }
               	heartbeat_updated = 1;
                NRF_LOG_INFO("msg_ble_send udpated with HBT\r\n");
                NRF_LOG_INFO("the rxed HBT is %s", (uint32_t)msg_ble_send);
                
            }
            break;
            
            // send NOT_OK response if we are here case
        case TWIS_EVT_READ_ERROR:
        case TWIS_EVT_WRITE_ERROR:
            
        case TWIS_EVT_GENERAL_ERROR:
            break;
            
        default:
            break;
            
    }
}
void twis_init(void)
{
    const nrf_drv_twis_config_t config =
    {
        .addr               = {TWIS_ADDR, 0},
        .scl                = ARDUINO_SCL_PIN,
        .scl_pull           = GPIO_PIN_CNF_PULL_Disabled,
        .sda                = ARDUINO_SDA_PIN,
        .sda_pull           = GPIO_PIN_CNF_PULL_Disabled,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    };
    
    if(NRF_SUCCESS != nrf_drv_twis_init(&s_twi, &config, twis_event_handler))
    {
        NRF_LOG_INFO("twis init failed!\r\n");
    }
    else
    {
        NRF_LOG_INFO("twis init OK!\r\n");
    }
    NRF_LOG_INFO("enabling the twis module!\r\n");
    nrf_drv_twis_enable(&s_twi);
    NRF_LOG_INFO("twis module enabled!\r\n");
}

/**@brief Function for initializing low frequency clock.
 */
void clock_initialization()
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;
    
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}

/**@brief Function for init the GPIO pin that will trigger the interrupt
 */

void init_gpio_pin(void) {
    
    nrf_gpio_cfg_output(ARDUINO_13_PIN);
    nrf_gpio_pin_write(ARDUINO_13_PIN, 0);
    
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
            err_code = sd_ble_gap_disconnect(m_cur_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;
            
        case BSP_EVENT_WHITELIST_OFF:
            if (m_cts.conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
            
            
        case BSP_EVENT_KEY_0:
            NRF_LOG_INFO("pressed the button 1\r\n");
            // raise the pin high to raise an interreupt
            nrf_gpio_pin_write(ARDUINO_13_PIN, 1);
            APP_ERROR_CHECK(err_code);
            
            break;
            
        case BSP_EVENT_KEY_1:
            
            NRF_LOG_INFO("pressed the button 2\r\n");
            // pull the line down again
            nrf_gpio_pin_write(ARDUINO_13_PIN, 0);
            break;
            
        case BSP_EVENT_KEY_3:
            NRF_LOG_INFO("pressed the button 3\r\n");
            if (m_cts.conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                NRF_LOG_INFO("We are going to read the cts\r\n");
                err_code = ble_cts_c_current_time_read(&m_cts);
                if (err_code == NRF_ERROR_NOT_FOUND)
                {
                    NRF_LOG_INFO("Current Time Service is not discovered.\r\n");
                }
            }
            else {
                NRF_LOG_INFO("invalid params \r\n");
            }
            break;
            
        default:
            // No implementation needed.
            break;
    }
}





// Efe Integration Done
// ============================================================================

// HeartBeat Integration
// ============================================================================
#define BLE_HBT_MSENSE    0x1825
#define MEM_INITIAL_VAL   250







																		/**< Instance of heartbeat value sent to Mobile App.*/


//Function for handling the write
static uint32_t char_add(uint16_t uuid, ble_hbt_s * ble_hbt_init)
{
    ble_uuid_t          ble_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
	
    char loc[] = "Heartbeat Checking.";
    ble_srv_utf8_str_t loc_utf8;
    ble_srv_ascii_to_utf8(&loc_utf8, (char *)loc);
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.p_char_user_desc = (uint8_t *)loc_utf8.p_str;
    char_md.char_user_desc_max_size = loc_utf8.length;
    char_md.char_user_desc_size = loc_utf8.length;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;
    BLE_UUID_BLE_ASSIGN(ble_uuid, uuid);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = ble_hbt_init->hbt_attr_md.read_perm;
    attr_md.write_perm = ble_hbt_init->hbt_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = ble_hbt_init->sample_str.length;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = ble_hbt_init->sample_str.length;
    attr_char_value.p_value   = ble_hbt_init->sample_str.p_str;

    return sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr_char_value, &ble_hbt_init->hbt_char_handle);
}

uint32_t ble_hbt_init(ble_hbt_s * p_hbt_init, char msg[])
{
	uint32_t err_code;
	ble_uuid_t ble_uuid;
	
	//Add service
	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_HBT_MSENSE);
	
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
	if(err_code != NRF_SUCCESS)
    {
		return err_code;
    }
	NRF_LOG_INFO("HBT Service Added.\r\n");
	
	ble_srv_ascii_to_utf8(&p_hbt_init->sample_str,(char *)msg);
	
	err_code = char_add(BLE_UUID_MANUFACTURER_NAME_STRING_CHAR,
											p_hbt_init);
	NRF_LOG_INFO("HBT Characteristic added.\r\n");
	
	if(err_code != NRF_SUCCESS)
		return err_code;
	
	NRF_LOG_INFO("HBT init complete.\r\n");
	
	
	return err_code;
}		

void ble_hbt_on_ble_evt(ble_hbt_s * p_hbt, ble_evt_t * p_ble_evt)
{
	if (p_ble_evt == NULL)
  {
		NRF_LOG_INFO("No event.\r\n");
     return;
  }
	
	switch(p_ble_evt->header.evt_id)
	{
		case BLE_GATTS_EVT_WRITE:
			NRF_LOG_INFO("HBT Write\r\n");
			break;
		case BLE_GAP_EVT_CONNECTED:
			NRF_LOG_INFO("HBT Connected.\r\n");
			p_hbt->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			break;
		case BLE_GAP_EVT_DISCONNECTED:
			NRF_LOG_INFO("HBT Disconnected.\r\n");
			break;
		case BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP:
			NRF_LOG_INFO("HBT Read RSP.\r\n");
		case SD_BLE_GATTC_CHAR_VALUE_BY_UUID_READ:
				NRF_LOG_INFO("HBT RR.\r\n");
				break;
		default:
			NRF_LOG_INFO("HBT Something else\r\n");
			break;
	}
}

uint8_t hbt_update_value(ble_hbt_s * p_hbt)
{
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &p_hbt->mem_initial;

    return sd_ble_gatts_value_set(p_hbt->conn_handle, p_hbt->hbt_char_handle.value_handle, &gatts_value);
}





// ============================================================================


APP_TIMER_DEF(m_sec_req_timer_id);                          /**< Security request timer. */

#define SCHED_MAX_EVENT_DATA_SIZE sizeof(app_timer_event_t) /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                 20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                 10                        /**< Maximum number of events in the scheduler queue. */
#endif
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_CURRENT_TIME_SERVICE, BLE_UUID_TYPE_BLE}};

static const char * day_of_week[8] = {"Unknown",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Sunday"};

static const char * month_of_year[13] = {"Unknown",
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December"};

static pm_peer_id_t   m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];  /**< List of peers currently in the whitelist. */
static uint32_t       m_whitelist_peer_cnt;                                 /**< Number of peers currently in the whitelist. */
static bool           m_is_wl_changed;                                      /**< Indicates if the whitelist has been changed since last time it has been updated in the Peer Manager. */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num  Line number of the failing ASSERT call.
 * @param[in] file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;
    
    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
    *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
    
    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;
    
    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t ret;
    
    memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
    m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));
    
    peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);
    
    ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
    APP_ERROR_CHECK(ret);
    
    // Setup the device identies list.
    // Some SoftDevices do not support this feature.
    ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
    if (ret != NRF_ERROR_NOT_SUPPORTED)
    {
        APP_ERROR_CHECK(ret);
    }
    
    m_is_wl_changed = false;
    
    ret = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;
    
    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;
            
        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
            
            m_peer_id = p_evt->peer_id;
            err_code  = ble_db_discovery_start(&m_ble_db_discovery, p_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            
            err_code = pm_peer_rank_highest(p_evt->peer_id);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
            
            // Note: You should check on what kind of white list policy your application should use.
            if (p_evt->params.conn_sec_succeeded.procedure == PM_LINK_SECURED_PROCEDURE_BONDING)
            {
                NRF_LOG_DEBUG("New Bond, add the peer to the whitelist if possible\r\n");
                NRF_LOG_DEBUG("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d\r\n",
                              m_whitelist_peer_cnt + 1,
                              BLE_GAP_WHITELIST_ADDR_MAX_COUNT);
                
                if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                {
                    // Bonded to a new peer, add it to the whitelist.
                    m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;
                    m_is_wl_changed = true;
                }
            }
        } break;
            
        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;
            
        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;
            
        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;
            
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start();
        } break;
            
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;
            
        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;
            
        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;
            
        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;
            
        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;
            
        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in]  nrf_error  Error code containing information about what went wrong.
 */
static void current_time_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the security request timer time-out.
 *
 * @details This function will be called each time the security request timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the time-out handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    uint32_t             err_code;
    pm_conn_sec_status_t status;
    
    if (m_cur_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = pm_conn_sec_status_get(m_cur_conn_handle, &status);
        APP_ERROR_CHECK(err_code);
        
        // If the link is still not secured by the peer, initiate security procedure.
        if (!status.encrypted)
        {
            err_code = pm_conn_secure(m_cur_conn_handle, false);
            APP_ERROR_CHECK(err_code);
        }
    }
}


/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in] p_evt  Event received from the Current Time Service client.
 */
static void current_time_print(ble_cts_c_evt_t * p_evt)
{
    uint32_t day, month, year, hours, minutes, seconds, fractions;
    
    NRF_LOG_INFO("current time print function called\r\n");
    /* parse the current time from the p_evt structure */
    day = p_evt->params.current_time.exact_time_256.day_date_time.date_time.day;
    month = p_evt->params.current_time.exact_time_256.day_date_time.date_time.month;
    year = p_evt->params.current_time.exact_time_256.day_date_time.date_time.year;
    hours = p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours;
    minutes = p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes;
    seconds = p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds;
    fractions = p_evt->params.current_time.exact_time_256.fractions256; 

    NRF_LOG_INFO("done parsing the struct fields\r\n");
    //"10/29/1923 18:81:00\r\n";
    sprintf(current_time, "%d/%d/%d %d:%d:%d:%d\r\n", day, month, year, hours, minutes, seconds, fractions);
    NRF_LOG_INFO("current time is ");
    NRF_LOG_INFO("%s", (uint32_t)current_time);
    NRF_LOG_INFO("done updating current time\r\n");
    NRF_LOG_FLUSH();
}


/**@brief Function for the timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;
    
    // Initialize timer module, making it use the scheduler
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);
    
    // Create security request timer.
    err_code = app_timer_create(&m_sec_req_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sec_req_timeout_handler);
    APP_ERROR_CHECK(err_code);
    
}


/**@brief Function for handling the Current Time Service client events.
 *
 * @details This function will be called for all events in the Current Time Service client that
 *          are passed to the application.
 *
 * @param[in] p_evt Event received from the Current Time Service client.
 */
static void on_cts_c_evt(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt)
{
    uint32_t err_code;
    
    switch (p_evt->evt_type)
    {
        case BLE_CTS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Current Time Service discovered on server.\r\n");
            err_code = ble_cts_c_handles_assign(&m_cts,
                                                p_evt->conn_handle,
                                                &p_evt->params.char_handles);
            APP_ERROR_CHECK(err_code);
            connected_to_cts = 1;
            break;
            
        case BLE_CTS_C_EVT_DISCOVERY_FAILED:
            NRF_LOG_INFO("Current Time Service not found on server. \r\n");
            // CTS not found in this case we just disconnect. There is no reason to stay
            // in the connection for this simple app since it all wants is to interact with CT
            if (p_evt->conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;
            
        case BLE_CTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_INFO("Disconnect Complete.\r\n");
            break;
            
        case BLE_CTS_C_EVT_CURRENT_TIME:
            NRF_LOG_INFO("Current Time received.\r\n");
            // prepare current time for transmission to camera module
            // set current time flag to be ready
            // raise the pin to begin the transmission process to camera
            current_time_print(p_evt);
						cur_time_ready = 1;
            nrf_gpio_pin_write(ARDUINO_13_PIN, 1);
            break;
            
        case BLE_CTS_C_EVT_INVALID_TIME:
            NRF_LOG_INFO("Invalid Time received.\r\n");
            break;
            
        default:
            break;
    }
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
    
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_cts_c_init_t cts_init_obj;
    
    cts_init_obj.evt_handler   = on_cts_c_evt;
    cts_init_obj.error_handler = current_time_error_handler;
    err_code                   = ble_cts_c_init(&m_cts, &cts_init_obj);
    APP_ERROR_CHECK(err_code);


    
    NRF_LOG_INFO("inside the services init function\r\n");
    NRF_LOG_INFO("initing the HBT service\r\n");
		m_hbt.mem_initial          = MEM_INITIAL_VAL;	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_hbt.hbt_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&m_hbt.hbt_attr_md.write_perm);
    err_code                   = ble_hbt_init(&m_hbt, msg_ble_send);
	
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("HBT service init complete\r\n");
    

}


/**@brief Function for handling the Connection Parameters module.
 *
 * @details This function will be called for all events in the Connection Parameters module that
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_cur_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
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


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the system event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);
    
    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for handling Database Discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective service instances.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_cts_c_on_db_disc_evt(&m_cts, p_evt);
}




/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;
    
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with WhiteList\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow advertising with WhiteList\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            err_code = ble_advertising_restart_without_whitelist();
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
            
        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            
            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist\r\n",
                          addr_cnt,
                          irk_cnt);
            
            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(whitelist_addrs, addr_cnt,
                                                       whitelist_irks,  irk_cnt);
            APP_ERROR_CHECK(err_code);
        }
            break;
            
        default:
            break;
    }
}


/**@brief Function for handling the application's BLE stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_cur_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = app_timer_start(m_sec_req_timer_id, SECURITY_REQUEST_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONNECTED
            
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.\r\n");
            m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;
            if (p_ble_evt->evt.gap_evt.conn_handle == m_cts.conn_handle)
            {
                m_cts.conn_handle = BLE_CONN_HANDLE_INVALID;
            }
            
            if (m_is_wl_changed)
            {
                // The whitelist has been modified, update it in the Peer Manager.
                err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                APP_ERROR_CHECK(err_code);
                
                err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                if (err_code != NRF_ERROR_NOT_SUPPORTED)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                m_is_wl_changed = false;
            }
            break; // BLE_GAP_EVT_DISCONNECTED
            
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT
            
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT
            
#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif
            
        default:
            // No implementation needed.
            break;
    }
}




/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_cts_c_on_ble_evt(&m_cts, p_ble_evt);
    
    ble_hbt_on_ble_evt(&m_hbt, p_ble_evt);			//Probably not required..!!
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}





/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;
    
    err_code = pm_init();
    APP_ERROR_CHECK(err_code);
    
    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }
    
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
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;
    
    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);
    
    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init()
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;
    
    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type                = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance       = true;
    advdata.flags                    = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    advdata.uuids_solicited.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_solicited.p_uuids  = m_adv_uuids;
    
    memset(&options, 0, sizeof(options));
    options.ble_adv_whitelist_enabled = true;
    options.ble_adv_fast_enabled      = true;
    options.ble_adv_fast_interval     = APP_ADV_FAST_INTERVAL;
    options.ble_adv_fast_timeout      = APP_ADV_FAST_TIMEOUT;
    options.ble_adv_slow_enabled      = true;
    options.ble_adv_slow_interval     = APP_ADV_SLOW_INTERVAL;
    options.ble_adv_slow_timeout      = APP_ADV_SLOW_TIMEOUT;
    
    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);
    
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    
    APP_ERROR_CHECK(err_code);
}

/**@brief Wrapper for blew functionality
 */
static void ble_init_start(void)
{
    
    bool       erase_bonds = false;
    ret_code_t err_code;
    
    // Initialize
    timers_init();
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    //buttons_leds_init(&erase_bonds);
    ble_stack_init();
    peer_manager_init(erase_bonds);
    if (erase_bonds == true)
    {
        NRF_LOG_INFO("Bonds erased!\r\n");
    }
    db_discovery_init();
    scheduler_init();
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
    
    // Start execution
    NRF_LOG_INFO("CTS Start!\r\n");
    advertising_start();
    
}
/**@brief
 * Wrapper for i2c init functions
 */

void i2c_init(void)
{
    clock_initialization();
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    bsp_configuration();
    init_gpio_pin();
    twis_init();
}

/**@brief
 * Wrapper for initializing everything
 */
void system_init(void)
{
    i2c_init();
    ble_init_start();
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t cts_err_code;
    system_init();
    
    // Enter main loop
    
    while(true)
    {
    	if(heartbeat_updated) 
    	{
            NRF_LOG_INFO("updating the HBT\r\n");
            uint32_t res = heartbeat_update(&m_hbt, msg_ble_send);
            NRF_LOG_INFO("the result is %d\r\n", res);
            heartbeat_updated = 0;
            NRF_LOG_FLUSH();
    	}
        if(connected_to_cts) 
        {
            NRF_LOG_INFO("We are going to read CTS inside the main loop\r\n");
            connected_to_cts = 0;
            // read the cts
            cts_err_code = ble_cts_c_current_time_read(&m_cts);
            APP_ERROR_CHECK(cts_err_code);        
        } 
        if(reset_device)
        {
            NRF_LOG_INFO("Resetting device A tout Le Monde\r\n");
            NRF_LOG_FLUSH();
            reset_device = 0;
            //NVIC_SystemReset();
        }
        
        NRF_LOG_FLUSH();
        app_sched_execute();
        //__SEV();
        //__WFE();
        //__WFE();
    }
    
}


