/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>


#define DEBUG 1
#define DEBUG_PRINT(fmt, args...) \
	do { if(DEBUG) {printf(fmt, ##args);} } while(0)



#define CUR_TIME_SIZE             25
#define HB_SIZE                   24
#define BUF_SIZE                  256
#define CMD_RESP_SIZE             1
#define SLAVE_ADDR                0x35
#define MAX_ERR_COUNT             10

#define SECONDS_FROM_1970_TO_2000 946684800
//Change this depending on timezone of compiled computer
#define HOUROFFSET                -5

/* ENUMS */

// responses that we'll rx from the ble peripheral
typedef enum
{

  OK                            = 0x8,
  NOT_OK                        = 0x9,

} responses;


// Commands that we are going to send to the BLE peripheral
typedef enum
{
  CUR_TIME_REQ                 = 0x29,
  RX_CUR_TIME,
  HB_REQ,
  TX_HB,
  POW_OFF,
  RESTART,
} commands;

// ble i2c state machine states
typedef enum state
{
  CUR_TIME_POL_TX,
  CUR_TIME_POL_RX,
  REC_CUR_TIME_TX,
  REC_CUR_TIME_RX,
  HB_REQ_TX,
  HB_REQ_RX,
  SEND_HB_TX,
  POW_OFF_TX,
  POW_OFF_RX,
  MY_RESTART,
  DONE,
} state;

/* IO (volatile) and regular variables  */

// the command variable for that we are going to send to ble peripheral 
__IO uint8_t                    command              = 0;
__IO uint32_t                   ble_tx_begin         = 0;
__IO state                      cur_state            = CUR_TIME_POL_TX;
__IO state                      next_state;
__IO uint32_t                   cts_ready            = 0;
__IO uint32_t                   i2c_error_count      = 0;
// counter for i2c error handler

// i2c comm error flag
uint32_t                        comm_error           = 0;
uint32_t                        device_synced        = 0;
/* Buffers */


// heartbeat buffer
uint8_t                         tx_buffer[]         = "This is the Habits Cam\r\n";
// i2c rx buffer
uint8_t                         rx_buffer[BUF_SIZE];



/* Peripheral Handlers */

I2C_HandleTypeDef               hi2c3;
UART_HandleTypeDef              hlpuart1;
RTC_HandleTypeDef               hrtc;
RTC_TimeTypeDef                 timer;
RTC_DateTypeDef                 dater;


/* Funtion Prototypes */

static void     MX_GPIO_Init(void);
static void     MX_I2C3_Init(void);
static void     MX_LPUART1_UART_Init(void);
static void     MX_RTC_Init(void);
static void     ble_i2c_error_handler(void);
static uint32_t toUnixtime(RTC_DateTypeDef date, RTC_TimeTypeDef time);
static uint8_t  isDST(int day, int month, int dow);
static uint8_t  dayofweek(uint8_t d, uint8_t m, uint8_t y);
static uint8_t  conv2d(const char* p);
static long     time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s);
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d);
static uint32_t toUnixtime(RTC_DateTypeDef date, RTC_TimeTypeDef time);
static void     strisng2date(const char* sDate, RTC_DateTypeDef* date);
static void     string2time(const char* sTime, RTC_TimeTypeDef* time);

void            SystemClock_Config(void);
void            handle_ble_connection(void);
void            HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void            async_ble_conn_handler(void);
void            my_cts_handler(void);
void            prepare_hbt(void);
void            camera_operation(void);
uint8_t 		    map_mont_to_rtc_month(uint32_t month);


/*
 * @brief function to enable routing UART to printf
 **/

int __io_putchar(int ch)
{

	HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

void system_init(void) 
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  prepare_hbt();

}

void loop() 
{
  while (1) 
  {
    my_cts_handler();
    async_ble_conn_handler();
    camera_operation();
  }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  system_init();
  DEBUG_PRINT("================================================\r\n");
  loop();
  
  return 0;

}

/*@brief Dummy camera functionality once the camera syncs its time the operation can resume

*/
void camera_operation(void) 
{
  if(device_synced)
  {
    DEBUG_PRINT("Device synced executing camera functionality\r\n");
    // Camera functionality in main loop would be here
    // obviously don't set device_synced to 0 here. I did it to print debug message once
    device_synced = 0;
  }

}



/* @brief You would need to implement this
  function that will prepare the hbt for transmission
*/
void prepare_hbt(void) 
{
  return;
}


/* @brief
  Event handler that will be called when the ble chip pulls the pin low
  command event handler
*/ 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // set the flag to 1 so the comm can begin
  ble_tx_begin = 1;
  return;
}



/* @brief
  Callback when the i2c tx is complete
*/ 
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) 
{
  //DEBUG_PRINT("I'm TX callback\r\n");
  cur_state = next_state;
  return;
}

/* @brief
  Callback when the i2c rx is complete
*/ 
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//DEBUG_PRINT("I'm RX callback\r\n");
  if(cur_state == REC_CUR_TIME_RX) 
  {
    cts_ready = 1;
    cur_state = next_state;
  }
  // check if the cur time not ready
  else if(cur_state == CUR_TIME_POL_RX) 
  {

    if(rx_buffer[0] == NOT_OK) 
    {
      cur_state = CUR_TIME_POL_TX; 
    }
    else if(rx_buffer[0] == OK) 
    {
      cur_state = next_state;
    }
    else
    {
      comm_error = 1;
    }
  }
  else 
  {
    if(rx_buffer[0] != OK)
    {
      comm_error = 1;
    }
    else
    {
      cur_state = next_state;
    }
  }
  
  return;
}

/*@brief
  function that will update the RTC once the curtime is ready
*/
void my_cts_handler(void)
{

  uint32_t day, month, year, hours, minutes, seconds, fractions, len, unix_timestamp;
  uint8_t mapped_month;
  
  RTC_TimeTypeDef startTime = {0};
  RTC_DateTypeDef startDate = {0};

  if(cts_ready) 
  {
    DEBUG_PRINT("CTS READY\r\n");
    DEBUG_PRINT("%s", rx_buffer);
    cts_ready = 0;
    // current time format d/m/y hour:minute:second:fraction\r\n
    // max len 25 min len 16
    // if string is emp8ty or the lenght doesn't fit
    len = strlen((char *)rx_buffer);
    printf("THE SIZE of current time is %d\r\n", len);
    if(len < 16 || len > CUR_TIME_SIZE) 
    {
      DEBUG_PRINT("current time size is wrong\r\n");
      ble_i2c_error_handler();
      return;

    }
    // parse the string
    sscanf((char *)rx_buffer, "%d/%d/%d %d:%d:%d:%d\r\n", &day, &month, &year, &hours, &minutes, &seconds, &fractions);
    
    // if the numbers do not match up
    if(day < 1 || day > 31)
    {
      DEBUG_PRINT("Day is wrong\r\n");
      ble_i2c_error_handler();
      return;
    }
    if(month < 1 || month > 12)
    {
      DEBUG_PRINT("Month is wrong\r\n");
      ble_i2c_error_handler();
      return;
    }
    // harcoded year values
    if(year < 2020  || month > 2022)
    {
      DEBUG_PRINT("year is wrong\r\n");
      ble_i2c_error_handler();
      return;
    }
    if(hours < 0  || hours > 24)
    {
      DEBUG_PRINT("hours is wrong\r\n");
      ble_i2c_error_handler();
      return; 
    }
    if(minutes < 0 || minutes > 60)
    {
      DEBUG_PRINT("minutes is wrong\r\n");
      ble_i2c_error_handler();
      return;
    }
    if(seconds < 0 || seconds > 60)
    {
      DEBUG_PRINT("seconds is wrong\r\n");
      ble_i2c_error_handler();
      return;
    }
    if(fractions < 0 || fractions > 1000)
    {
      DEBUG_PRINT("fractions is wrong\r\n");
      ble_i2c_error_handler();
      return;
    }
    DEBUG_PRINT("successfully parsed current time\r\n");


    
    HAL_RTC_GetTime(&hrtc,&timer,RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc,&dater,RTC_FORMAT_BIN);
    unix_timestamp = toUnixtime(dater, timer);
    DEBUG_PRINT("the unix time stamp for unitialized RTC %d\r\n", unix_timestamp);

     
    // update the RTC
    // set the time
    // I don't know if we want the daylight savings as well
    startTime.Hours   = hours;
    startTime.Minutes = minutes;
    startTime.Seconds = seconds;

    if (HAL_RTC_SetTime(&hrtc, &startTime, RTC_FORMAT_BIN) != HAL_OK)
    {
      Error_Handler();
    }
    DEBUG_PRINT("set the time\r\n");
    
    // map the month
    mapped_month      = map_mont_to_rtc_month(month);
    year              = year - 2000;
    
    startDate.WeekDay = RTC_WEEKDAY_MONDAY;
    startDate.Month   = mapped_month;
    startDate.Date    = day;
    startDate.Year    = year;
    
    if (HAL_RTC_SetDate(&hrtc, &startDate, RTC_FORMAT_BIN) != HAL_OK)
    {
      Error_Handler();
    }
    
    DEBUG_PRINT("set the date\r\n");

#if DEBUG == 1
    HAL_RTC_GetTime(&hrtc,&timer,RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc,&dater,RTC_FORMAT_BIN);
    unix_timestamp = toUnixtime(dater, timer);
    DEBUG_PRINT("the unix time stamp for initialized RTC %d\r\n", unix_timestamp);
#endif
  
  }

}

/*
  @brief 
  function to map cts value to rtc value

*/
uint8_t map_mont_to_rtc_month(uint32_t month)
{
  uint8_t mapped_month;
  
  switch (month)
  {
    case 1:
    {
      mapped_month = RTC_MONTH_JANUARY;
      break;
    }
    case 2:
    {
      mapped_month = RTC_MONTH_FEBRUARY;
      break;
    }
    case 3:
    {
      mapped_month = RTC_MONTH_MARCH;
      break;
    }
    case 4:
    {
      mapped_month = RTC_MONTH_APRIL;
      break;
    }
    case 5:
    {
      mapped_month = RTC_MONTH_MAY;
      break;
    }
    case 6:
    {
      mapped_month = RTC_MONTH_JUNE;
      break;
    }
    case 7:
    {
      mapped_month = RTC_MONTH_JULY;
      break;
    }
    case 8:
    {
      mapped_month = RTC_MONTH_AUGUST;
      break;
    }
    case 9:
    {
      mapped_month = RTC_MONTH_SEPTEMBER;
      break;
    }
    case 10:
    {
      mapped_month = RTC_MONTH_OCTOBER;
      break;
    }
    case 11:
    {
      mapped_month = RTC_MONTH_NOVEMBER;
      break;
    }
    case 12:
    {
      mapped_month = RTC_MONTH_DECEMBER;
      break;
    }
  }
  return mapped_month;
}

/*@brief
  function that will handle in case there is an error in the i2c comm
*/
static void ble_i2c_error_handler(void) 
{
  DEBUG_PRINT("inside the error handler\r\n");
  if(i2c_error_count == MAX_ERR_COUNT) 
  {
	DEBUG_PRINT("WORST CASE!!\r\n");
    // log the error on to the SD card.
    // restart the chip
    // or alternative start blinking the LED on the Board
    while (1);
  }
  i2c_error_count++;
  // deinit the i2c
  HAL_I2C_DeInit(&hi2c3);
  // reinit
  HAL_I2C_Init(&hi2c3);
  // reset the state machine
  cur_state    = MY_RESTART;
  // force the state machine to send the data
  ble_tx_begin = 1;
  return;
  
}

/* wait for i2c bus */
static inline void wait_for_i2c(void)
{
	while (HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY);
}

/* returns 1 if i2c bus ready */
static inline int ble_i2c_ready(void)
{
	return (HAL_I2C_GetState(&hi2c3) == HAL_I2C_STATE_READY);
}

/* wrapper to set the command variable */
static inline void set_command(uint8_t c) 
{
  command = c;
}
/* wrapper for HAL function */
static inline HAL_StatusTypeDef send_ble_i2c_command(uint8_t c) 
{
  set_command(c); 
  return HAL_I2C_Master_Transmit_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)&command, CMD_RESP_SIZE);
}

/* wrapper for checking errors */
static inline void check_for_ble_i2c_errors(HAL_StatusTypeDef e_code) 
{
  if(e_code != HAL_OK) 
  {
    ble_i2c_error_handler();
  }
}

/* wrapper for ble response */
static inline HAL_StatusTypeDef receive_ble_response(void) 
{
  return HAL_I2C_Master_Receive_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)rx_buffer, CMD_RESP_SIZE);
} 

/* wrapper for ble response */
static inline HAL_StatusTypeDef receive_cts_response(void)
{
  return HAL_I2C_Master_Receive_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)rx_buffer, CUR_TIME_SIZE);
}

static inline HAL_StatusTypeDef send_hbt_to_ble(void) 
{
  return HAL_I2C_Master_Transmit_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)tx_buffer, HB_SIZE); 
}

/* set next_state */
static inline void set_next_state(state s) 
{
  next_state = s;
}



/*@brief

  Async Connection Handler for i2c module

*/
void async_ble_conn_handler(void)
{
  HAL_StatusTypeDef err_code;
  
  if(ble_tx_begin && ble_i2c_ready()) 
  {
    switch(cur_state) 
    {
      case CUR_TIME_POL_TX:
      {
        DEBUG_PRINT("sending cur_time req\r\n");
        err_code = send_ble_i2c_command(CUR_TIME_REQ);
        check_for_ble_i2c_errors(err_code);
        set_next_state(CUR_TIME_POL_RX);
        break;
      }
      case CUR_TIME_POL_RX:
      {
        DEBUG_PRINT("CUR TIME POL response\r\n");
        err_code = receive_ble_response();
        check_for_ble_i2c_errors(err_code);
        // you should prepare for John's camera
        set_next_state(REC_CUR_TIME_TX);
        break;
      }
      case REC_CUR_TIME_TX:
      {
        DEBUG_PRINT("ble will send the current time\r\n");
        err_code = send_ble_i2c_command(RX_CUR_TIME);
        check_for_ble_i2c_errors(err_code);
        set_next_state(REC_CUR_TIME_RX);
        break;
      } 
      case REC_CUR_TIME_RX:
      {
        DEBUG_PRINT("receiving the current time\r\n"); 
        err_code = receive_cts_response();
        check_for_ble_i2c_errors(err_code);
        set_next_state(HB_REQ_TX);
        break;
      }
      case HB_REQ_TX:
      {
        DEBUG_PRINT("Sending the HB_REQ\r\n");
        err_code = send_ble_i2c_command(HB_REQ); 
        check_for_ble_i2c_errors(err_code);
        set_next_state(HB_REQ_RX);
        break;
      }
      case HB_REQ_RX:
      {
        DEBUG_PRINT("recieving the HB_REQ ACK\r\n");
        err_code = receive_ble_response();
        check_for_ble_i2c_errors(err_code); 
        set_next_state(SEND_HB_TX);
        break;
      }
      case SEND_HB_TX:
      {
        DEBUG_PRINT("sending the hearbeat\r\n");
        err_code = send_hbt_to_ble(); 
        check_for_ble_i2c_errors(err_code); 
        set_next_state(POW_OFF_TX);
        break;
      }
      case POW_OFF_TX: 
      {
        DEBUG_PRINT("Sending poweroff command \r\n");
        err_code = send_ble_i2c_command(POW_OFF);
        check_for_ble_i2c_errors(err_code);
        set_next_state(POW_OFF_RX);
        break;
      }
      case POW_OFF_RX:
      {
        DEBUG_PRINT("recieve response \r\n"); 
        err_code = receive_ble_response();
        set_next_state(DONE);
        break; 
      }
      case DONE:
      {
        ble_tx_begin = 0;
        cur_state    = CUR_TIME_POL_TX;
        // rtc and hbt transmission successfull now we can start the camera
        device_synced = 1;
        DEBUG_PRINT("Done\r\n");
        break;
      }
      case MY_RESTART:
      {
        //basically reset the state machine
        // we don't have to wait for ACK as the BLE chip will reset itself
        DEBUG_PRINT("sending restart to the ble peripheral\r\n");
        err_code = send_ble_i2c_command(RESTART);
        check_for_ble_i2c_errors(err_code);
        set_next_state(CUR_TIME_POL_TX);
        ble_tx_begin = 0;
        break;
      }
    }
  }
}

// Snychronous connection handler
void handle_ble_connection(void) {

  if(ble_tx_begin) {

  
    // Send transfer_request the request for current time
    command = CUR_TIME_REQ;
    DEBUG_PRINT("sending cur_time req\r\n");
    if(HAL_I2C_Master_Transmit_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)&command, CMD_RESP_SIZE)!= HAL_OK)
    {
    	DEBUG_PRINT("entering the error handler\r\n");
        Error_Handler();
    }
    wait_for_i2c();
    // recieve the device response
    DEBUG_PRINT("waiting for a resp\r\n");
    if(HAL_I2C_Master_Receive_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)rx_buffer, CMD_RESP_SIZE)!= HAL_OK)
    {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }
    wait_for_i2c();
    
    DEBUG_PRINT("I rxed %x\r\n", rx_buffer[0]);
    if(rx_buffer[0] == NOT_OK) {
        Error_Handler();
    }

    DEBUG_PRINT("CUR TIME REQ successfull\r\n");
    // send the cur time transmit command
    command = RX_CUR_TIME; 
    if(HAL_I2C_Master_Transmit_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)&command, CMD_RESP_SIZE)!= HAL_OK)
    {
        Error_Handler();
    }
    wait_for_i2c();
    // receive response
    if(HAL_I2C_Master_Receive_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)rx_buffer, CUR_TIME_SIZE)!= HAL_OK)
    {

      Error_Handler();
    }
    wait_for_i2c();
    //replace this function with processing the current time
    DEBUG_PRINT("%s", rx_buffer);

    // send the hearbeat command
    DEBUG_PRINT("Sending the HB_REQ\r\n");
    command = HB_REQ;
    if(HAL_I2C_Master_Transmit_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)&command, CMD_RESP_SIZE)!= HAL_OK)
    {
        Error_Handler();
    }
    wait_for_i2c();
    if(HAL_I2C_Master_Receive_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)&rx_buffer, CMD_RESP_SIZE)!= HAL_OK)
    {

      Error_Handler();
    }
    wait_for_i2c();
    
    if(rx_buffer[0] == NOT_OK) {
      Error_Handler();
    }
    DEBUG_PRINT("HB REQ SUCCESSFULL\r\n");

    // transmit HB
    if(HAL_I2C_Master_Transmit_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)tx_buffer, HB_SIZE)!= HAL_OK)
    {
        Error_Handler();
    }
    wait_for_i2c();
    DEBUG_PRINT("HB transmission successfull\r\n");
    // power off

    command = POW_OFF;
    if(HAL_I2C_Master_Transmit_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)&command, CMD_RESP_SIZE)!= HAL_OK)
    {
        Error_Handler();
    }
    wait_for_i2c();
    if(HAL_I2C_Master_Receive_IT(&hi2c3, (SLAVE_ADDR << 1), (uint8_t *)&rx_buffer, CMD_RESP_SIZE)!= HAL_OK)
    {

      Error_Handler();
    } 
    wait_for_i2c();
    if(rx_buffer[0] == NOT_OK) {
      Error_Handler();
    }
    ble_tx_begin = 0;
    DEBUG_PRINT("We are done with the comm!!\r\n");
  }

}




/* ================ Date Time helper functions ================ */

static uint8_t isDST(int day, int month, int dow)
{
        //January, february, and december are out.
        if (month < 3 || month > 11) { return 0; }
        //April to October are in
        if (month > 3 && month < 11) { return 1; }
        int previousSunday = day - dow;
        //In march, we are DST if our previous sunday was on or after the 8th.
        if (month == 3) { return previousSunday >= 8; }
        //In november we must be before the first sunday to be dst.
        //That means the previous sunday must be before the 1st.
        return previousSunday <= 0;
}
static uint8_t conv2d(const char* p)
{
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}
static uint8_t dayofweek(uint8_t d, uint8_t m, uint8_t y)
{
	static int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
	y -= m < 3;
	return ( y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

static uint8_t daysInMonth [] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += daysInMonth[i - 1];
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

static void string2date(const char* sDate, RTC_DateTypeDef* date){
	date->Year = conv2d(sDate + 9);
	  // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
	  switch (sDate[0]) {
		  case 'J': date->Month = (sDate[1] == 'a') ? 1 : ((sDate[2] == 'n') ? 6 : 7); break;
		  case 'F': date->Month = 2; break;
		  case 'A': date->Month = sDate[2] == 'r' ? 4 : 8; break;
		  case 'M': date->Month = sDate[2] == 'r' ? 3 : 5; break;
		  case 'S': date->Month = 9; break;
		  case 'O': date->Month = 10; break;
		  case 'N': date->Month = 11; break;
		  case 'D': date->Month = 12; break;
	  }
	   date->Date = conv2d(sDate + 4);
	   date->WeekDay = dayofweek(date->Date,date->Month,date->Year);
}
static void string2time(const char* sTime, RTC_TimeTypeDef* time){
	   time->Hours = conv2d(sTime);
	   time->Minutes = conv2d(sTime + 3);
	   time->Seconds = conv2d(sTime + 6);
}

//converts date and time to unix timestamp
static uint32_t toUnixtime(RTC_DateTypeDef date, RTC_TimeTypeDef time){
	uint32_t t;
	uint16_t days = date2days(date.Year, date.Month, date.Date);
	t = time2long(days, time.Hours, time.Minutes, time.Seconds);
	t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000
	return t + (HOUROFFSET*3600);
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 71;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00505B89;
  hi2c3.Init.OwnAddress1 = 48;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // This is might be whats missing from the code
  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */


  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
