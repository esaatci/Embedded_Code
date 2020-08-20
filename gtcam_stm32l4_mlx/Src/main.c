/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "ov2640.h"
#include "stm32l4xx_nucleo_144.h"
#include "SparkFun_GridEYE_Arduino_Library.h"
#include "stm32_adafruit_sd.h"
#include "salsa20.h"
#include "MLX90640_API.h"
#include "stm_init_funcs.h"




/* Private typedef -----------------------------------------------------------*/

#define LOW_POWER_THRESHOLD 1630 //A dead battery charged for half an hour, 1730 -> 3.3V
#define FULL_POWER_THRESHOLD 1920

/* Private variables ---------------------------------------------------------*/
bool cam_on=true;
uint32_t voltage=1900;
ADC_HandleTypeDef hadc1;
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;
DMA_HandleTypeDef hdma_i2c4_rx;
volatile int dma_ready = 1;
UART_HandleTypeDef hlpuart1;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
FATFS SDFatFs;
FIL MyFile;
char SDPath[4];


/* Private function prototypes -----------------------------------------------*/
static uint8_t isDST(int day, int month, int dow);
static uint8_t dayofweek(uint8_t d, uint8_t m, uint8_t y);
static uint8_t conv2d(const char* p);
static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s);
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d);
static uint32_t toUnixtime(RTC_DateTypeDef date, RTC_TimeTypeDef time);
static void string2date(const char* sDate, RTC_DateTypeDef* date);
static void string2time(const char* sTime, RTC_TimeTypeDef* time);
static void get_voltage();
static uint8_t saveDataToSD();
/* --------------------------------------------------------------------------- */



/* Private user code ---------------------------------------------------------*/

#define HOUROFFSET_MACRO isDST(dater.Date,dater.Month,dater.WeekDay % 7)?6:5
#define BUFFER_SIZE 0x9600
uint32_t HOUROFFSET;
uint16_t pBuffer[BUFFER_SIZE];
uint32_t frameSize;
volatile uint8_t frameDone;

/* Image header */
const uint32_t aBMPHeader[14]=
{0xB0364D42, 0x00000004, 0x00360000, 0x00280000, 0x01400000, 0x00F00000, 0x00010000,
 0x00000020, 0xF5400000, 0x00000006, 0x00000000, 0x00000000, 0x00000000, 0x0000};

char img_fname[20] = {0};
char encrypt_fname[20]={0};
char csv_fname[20] = {0};
char current_name[20] = {};
float pixelTable[2][64];
const char * DATE = __DATE__;
const char * TIME = __TIME__;
RTC_TimeTypeDef timer;
RTC_DateTypeDef dater;
uint32_t unix_timestamp;
uint16_t milliseconds;
unsigned long framecount = 0;


/* =========================== MLX =========================== */

/* MLX private functions */ 
static void mlx_init();
static int  is_mlx_connected();
static void get_Mlx_data();
/* MLX variables and Defines */

#define  Rate4HZ      0x03
#define  Rate8HZ      0x04
#define  Rate16HZ     0x05
#define	 RefreshRate  Rate16HZ
#define  TA_SHIFT     8 //Default shift for MLX90640 in open air might need to adjust this
#define  PAGE_COUNT   2

const unsigned short MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
static float mlx90640To[768];
int status;
paramsMLX90640 mlx90640;
uint16_t eeMLX90640[832];
uint8_t compressed_mlx[1536];

/* =========================== MLX =========================== */

cf_salsa20_ctx ctx;
//encryption key and nonce
//currently, nonce cannot be decrypted by python script
//so it gets set to 0 in the main function.
uint8_t key[32] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
		0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,
		0x1a,0x1b,0x1c,0x1d,0x1e,0x1f};
uint8_t nonce[8] = {};



/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_I2C4_Init();
  
  
  if(HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1)!=HAL_OK){
  	    Error_Handler();
  }
  HAL_Delay(100);

    /* Init camera */
  ov2640_Init(OV2640_I2C_ADDRESS,CAMERA_R320x240_JPEG);
  HAL_DCMI_DisableCrop(&hdcmi);
  HAL_Delay(10);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);


  mlx_init();

  //error initializing sd card
  int init_res=BSP_SD_Init();
    while(init_res != MSD_OK){
      init_res=BSP_SD_Init();
  	  BSP_LED_Off(LED3);
  	  HAL_Delay(1000);
  	  BSP_LED_On(LED3);
  	  HAL_Delay(1000);
    }

    for(int i = 0; i < 8; i++){
  	  nonce[i] = 0x00;
    }

    FRESULT res;
    uint32_t byteswritten;
    uint8_t success = 0;
    cf_salsa20_init(&ctx,key,32,nonce);

    char testStr[256];
    if(f_mount(&SDFatFs,(TCHAR const*)SDPath, 0) == FR_OK) {
  	  res = f_open(&MyFile,"ime.txt", FA_OPEN_ALWAYS| FA_WRITE);
  	  if(res == FR_OK)
  	  {
  		  int size=sprintf(testStr, "Reboot time of device:\nyear:%d month:%d date:%d\nhours:%d minutes:%d second:%d\n",
  		  			dater.Year, dater.Month, dater.Date,
  					timer.Hours, timer.Minutes, timer.Seconds);
  		  res = f_write(&MyFile, testStr, size, (void *)&byteswritten);

  			if((byteswritten > 0) && (res == FR_OK))
  			{
  				success = 1;
  				f_close(&MyFile);
  			}
  	  }
    }
    f_mount(0,(TCHAR const*) SDPath,0);

    frameDone = 0;
    HAL_StatusTypeDef hal_status = HAL_OK;
    hal_status = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,  (uint32_t)pBuffer , BUFFER_SIZE/2); //size=320*240*2/4

    if(hal_status != HAL_OK) {
  	  while(1);
    }


  /* Infinite loop */

  uint8_t last_minute=60;
  while (1)
  {

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==0&&voltage>=LOW_POWER_THRESHOLD&&!cam_on){
		if(HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1)!=HAL_OK){
			    Error_Handler();
		    }
		cam_on=true;
	}
	else if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)!=0||voltage<LOW_POWER_THRESHOLD)&&cam_on){
		if(HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_1)!=HAL_OK){
			    Error_Handler();
		    }
		cam_on=false;
	}
	  if(cam_on){
		  BSP_LED_On(LED2);
		  if(frameDone){

				// Sanity check for JPEG file (linear search for 0xffd9)
				uint8_t* pBuffer8 = (uint8_t*)pBuffer;
				framecount++;
				uint16_t i = 2;
				if(pBuffer8[0] == 0xFF && pBuffer8[1] == 0xD8) {
					while(!(pBuffer8[i]==0xff && pBuffer8[i+1] == 0xD9)) {
						i++;
						if(i+1 > BUFFER_SIZE)
							break;
					}
					frameSize = i + 2;
					//__NOP();
				}
				else{
					//jpeg check failed; restart frame capture
					HAL_StatusTypeDef hal_status = HAL_OK;
					frameDone = 0;

					hal_status = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,  (uint32_t)pBuffer , 0x9600/2);
					continue;

					if(hal_status != HAL_OK) {
					 while(1) {
						 printf("camera failed \r\n");
					 }
					}
				}

				HAL_RTC_GetTime(&hrtc,&timer,RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc,&dater,RTC_FORMAT_BIN);
				unix_timestamp = toUnixtime(dater, timer);
				milliseconds = (timer.SecondFraction-timer.SubSeconds)*1000/(timer.SecondFraction+1);
				if(timer.Minutes!=last_minute){
					get_voltage();
					last_minute=timer.Minutes;
				}

				//get thermal Data
				get_Mlx_data();



				//Encryption and SD Card Storage begins here
				success = saveDataToSD();
				if(success){
					BSP_LED_Toggle(LED3);
				}
				else{
					BSP_LED_Off(LED3);
				}
				//Capture new frame now that SD card has been saved to
				frameDone = 0;
				HAL_StatusTypeDef hal_status = HAL_OK;
				hal_status = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,  (uint32_t)pBuffer , 0x9600/2);
				if(hal_status != HAL_OK) {
				 while(1);
				}
		  }
	  }
	  else {
	    get_voltage();

		if(voltage>FULL_POWER_THRESHOLD){
			BSP_LED_On(LED3);
		}
		else {
			BSP_LED_Off(LED3);
		}
		BSP_LED_Off(LED2);
		HAL_Delay(2000);
	  }

  }
}



void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi_handle)
{
	HAL_DCMI_Stop(&hdcmi);
	frameDone = 1; // frame is now done; return to main
}


/* ================ Date Time helper functions ================ */

static uint8_t isDST(int day, int month, int dow)
{
        //January, february, and december are out.
        if (month < 3 || month > 11) { return false; }
        //April to October are in
        if (month > 3 && month < 11) { return true; }
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

/* ================ ================ ================ ================ */

void get_voltage(){
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,1000000)==HAL_OK){
		  voltage=HAL_ADC_GetValue(&hadc1);
	 }
	HAL_ADC_Stop(&hadc1);
}




static void mlx_init() {
	
  MLX90640_SetRefreshRate(MLX90640_address, RefreshRate);
	MLX90640_SetChessMode(MLX90640_address);
    status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
    if (status != 0)
        Error_Handler();

    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status != 0)
        Error_Handler();

    //Once params are extracted, we can release eeMLX90640 array

}

void get_Mlx_data() {

    //for (uint8_t i = 0; i < PAGE_COUNT; i++) {
        
  uint16_t mlx90640Frame[834];

  int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);

  if (status < 0) {
      // new error messages
    while(1) {
      printf("MLX error \r\n");
    }
      Error_Handler();
  }

  float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
  float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

  float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
  float emissivity = 0.95;

  MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);


}

/*

This function compresses the MLX data by
converting floating point values to fixed point 16 bit integers
and casting them to two 8 bit numbers to reduce the number of bytes written to sd card

Only works if we assume the floating point temperature values are in the range 0 - 99.99 C

ex.

temp = 23.565658

16 bit value 2356
16 bit value in hex 0x0934

msb 0x09
lsb 0x34

*/
void compress_mlx_data() {
  
  uint8_t   lsb, msb, ws;
  uint16_t  temp, packed;
  float     f_packed;
  char      debug_buf[128];
  
  for(int i=0; i<768; i++) {
    
    /* 
      Multiply the floating value by 100
      cast it to 16 bit integer
      pack it to two 8 bit integers
    */
    
    temp = (uint16_t)(mlx90640To[i] * 100);
    lsb  = 0x00FF & temp;
    msb  = temp >> 8;
    compressed_mlx[2*i]  = msb;
    compressed_mlx[2*i+1] = lsb;
  
  }

}

/**
 * Save MLX and camera image to SD card
 */
static uint8_t saveDataToSD(){
	uint8_t success = 0;
	FRESULT res;
	uint32_t byteswritten;
	char fname[20] = {0};
	char buffer[1024] = {0};
	int sz;
	uint8_t* pixel_buff_t = (uint8_t*)pBuffer;
	volatile int wcount = 0;

	uint32_t save_time = HAL_GetTick();

	sprintf(fname,"%lu.jpg",framecount);
	uint32_t temp = unix_timestamp/1000;
	sprintf(img_fname,"%lu.jpg",temp);
	sprintf(csv_fname,"%lu.raw",temp);

	uint8_t new_file = 0;
	//initialize the encryption if the file name changes
	for(int i = 0;i < sizeof current_name;i++){
		if(current_name[i] != fname[i]){
			current_name[i] = fname[i];
			new_file = 1;
		}
	}
	if(new_file == 1){
		cf_salsa20_init(&ctx,key,32,nonce);
	}

	//encrypt jpeg file from pbuffer
	uint8_t* buff_tmp = pixel_buff_t;
	for(int32_t i = frameSize; i > 0; i-= 64){
		memset(ctx.nonce,0,16);
		//fills with 0xFF
		if(i<64){
			memset(buff_tmp+i-1, 0xFF, 64-i);
			buff_tmp[63]=0xD9;
			frameSize+=64-i;
		}		cf_salsa20_cipher(&ctx,buff_tmp,buff_tmp,64);
		buff_tmp += 64;
	}

	//The following code saves data into proper locations in SD card
	f_mount(&SDFatFs,(TCHAR const*)SDPath, 0);
	res = f_open(&MyFile,img_fname, FA_OPEN_APPEND| FA_WRITE);
	if(res == FR_OK)
	{
		//write to sd cards in smaller chunks (512 is the only one I know works for sure)
		int32_t write_size = 512;
		for(int32_t buffer_idx = 0; buffer_idx <= frameSize; buffer_idx += write_size){
			wcount++;
			if(buffer_idx + write_size >= frameSize){
				write_size = frameSize - buffer_idx;
				res = f_write(&MyFile, &pixel_buff_t[buffer_idx], write_size, (void *)&byteswritten);
				break;
			}
			else
				res = f_write(&MyFile, &pixel_buff_t[buffer_idx], write_size, (void *)&byteswritten);
				HAL_Delay(10);
		}


	    if((byteswritten > 0) && (res == FR_OK))
	    {
	    	success = 1;
	        f_close(&MyFile);
	    }
	}

  /* First Step of Optimization:
    change the writes of the SD card to higher chunks
    
  */
 	
	save_time = HAL_GetTick() - save_time;
	res = f_open(&MyFile,csv_fname, FA_OPEN_APPEND|FA_WRITE);
	if(res == FR_OK){

     // compress data
    // add the unix time stamp + miliseconds + save_time
     uint8_t B1, B2, B3, B4;
     uint8_t B6, B7;
     uint8_t B8, B9, B10, B11;
     uint8_t B12, B13;
     B1 = unix_timestamp >> 24;
     B2 = unix_timestamp >> 16;
     B3 = unix_timestamp >> 8;
     B4 = unix_timestamp & 0x000000FF;
     B6 = milliseconds >> 8;
     B7 = milliseconds & 0x00FF;
     B8 = save_time >> 24;
     B9 =  save_time >> 16;
     B10 = save_time >> 8;
     B11 = save_time & 0x000000FF;
     B12 = 0xD;
     B13 = 0xA;
     compress_mlx_data();
     uint8_t rem_data[12] = {B1, B2, B3, B4, B6, B7, B8, B9, B10, B11, B12, B13};
     
     f_write(&MyFile, &compressed_mlx, 512,(void*)&byteswritten);
     f_write(&MyFile, &compressed_mlx[512], 512,(void*)&byteswritten);
     f_write(&MyFile, &compressed_mlx[1024], 512,(void*)&byteswritten);
     f_write(&MyFile, &rem_data, 12,(void*)&byteswritten);
     
     f_truncate(&MyFile);
		f_close(&MyFile);
	}
	f_mount(0,(TCHAR const*)SDPath, 0);
	return success;
}


/**
  * @brief  This functions allows you to printf() to print debuf messages to serial
  * @retval int
*/
int __io_putchar(int ch) {

	HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}




void Error_Handler(void)
{
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
