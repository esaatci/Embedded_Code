/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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

#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef timer;
extern RTC_DateTypeDef dater;
/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */     
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
	HAL_RTC_GetTime(&hrtc,&timer,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&dater,RTC_FORMAT_BIN);
	BYTE year = dater.Year + 20; //years since 1980. In this case, the stored year is since 2000. So, add 20
	BYTE month = dater.Month;
	BYTE date = dater.Date;
	BYTE hour = timer.Hours;
	BYTE minute = timer.Minutes;
	BYTE second = timer.Seconds;
	DWORD return_time;
	return_time = ((DWORD) year << 25)
				| ((DWORD) month << 21)
				| ((DWORD) date << 16)
				| ((DWORD) hour << 11)
				| ((DWORD) minute << 5)
				| ((DWORD) second >> 1);
	return return_time;
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
