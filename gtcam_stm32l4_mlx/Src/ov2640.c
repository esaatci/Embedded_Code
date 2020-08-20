/**
  ******************************************************************************
  * @file    ov2640.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file provides the OV2640 camera driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "ov2640.h"

extern I2C_HandleTypeDef hi2c1;

static uint32_t ov2640_ConvertValue(uint32_t feature, uint32_t value);

const unsigned char OV2640_QVGA[][2] =
{
	{0xff, 0x0},
	{0x2c, 0xff},
	{0x2e, 0xdf},
	{0xff, 0x1},
	{0x3c, 0x32},
	{0x11, 0x0},
	{0x9, 0x2},
	{0x4, 0xa8},
	{0x13, 0xe5},
	{0x14, 0x48},
	{0x2c, 0xc},
	{0x33, 0x78},
	{0x3a, 0x33},
	{0x3b, 0xfb},
	{0x3e, 0x0},
	{0x43, 0x11},
	{0x16, 0x10},
	{0x39, 0x2},
	{0x35, 0x88},

	{0x22, 0xa},
	{0x37, 0x40},
	{0x23, 0x0},
	{0x34, 0xa0},
	{0x6, 0x2},
	{0x6, 0x88},
	{0x7, 0xc0},
	{0xd, 0xb7},
	{0xe, 0x1},
	{0x4c, 0x0},
	{0x4a, 0x81},
	{0x21, 0x99},
	{0x24, 0x40},
	{0x25, 0x38},
	{0x26, 0x82},
	{0x5c, 0x0},
	{0x63, 0x0},
	{0x46, 0x22},
	{0xc, 0x3a},
	{0x5d, 0x55},
	{0x5e, 0x7d},
	{0x5f, 0x7d},
	{0x60, 0x55},
	{0x61, 0x70},
	{0x62, 0x80},
	{0x7c, 0x5},
	{0x20, 0x80},
	{0x28, 0x30},
	{0x6c, 0x0},
	{0x6d, 0x80},
	{0x6e, 0x0},
	{0x70, 0x2},
	{0x71, 0x94},
	{0x73, 0xc1},
	{0x3d, 0x34},
	{0x12, 0x4},
	{0x5a, 0x57},
	{0x4f, 0xbb},
	{0x50, 0x9c},
	{0xff, 0x0},
	{0xe5, 0x7f},
	{0xf9, 0xc0},
	{0x41, 0x24},
	{0xe0, 0x14},
	{0x76, 0xff},
	{0x33, 0xa0},
	{0x42, 0x20},
	{0x43, 0x18},
	{0x4c, 0x0},
	{0x87, 0xd0},
	{0x88, 0x3f},
	{0xd7, 0x3},
	{0xd9, 0x10},
	{0xd3, 0x82},
	{0xc8, 0x8},
	{0xc9, 0x80},
	{0x7c, 0x0},
	{0x7d, 0x0},
	{0x7c, 0x3},
	{0x7d, 0x48},
	{0x7d, 0x48},
	{0x7c, 0x8},
	{0x7d, 0x20},
	{0x7d, 0x10},
	{0x7d, 0xe},
	{0x90, 0x0},
	{0x91, 0xe},
	{0x91, 0x1a},
	{0x91, 0x31},
	{0x91, 0x5a},
	{0x91, 0x69},
	{0x91, 0x75},
	{0x91, 0x7e},
	{0x91, 0x88},
	{0x91, 0x8f},
	{0x91, 0x96},
	{0x91, 0xa3},
	{0x91, 0xaf},
	{0x91, 0xc4},
	{0x91, 0xd7},
	{0x91, 0xe8},
	{0x91, 0x20},
	{0x92, 0x0},

	{0x93, 0x6},
	{0x93, 0xe3},
	{0x93, 0x3},
	{0x93, 0x3},
	{0x93, 0x0},
	{0x93, 0x2},
	{0x93, 0x0},
	{0x93, 0x0},
	{0x93, 0x0},
	{0x93, 0x0},
	{0x93, 0x0},
	{0x93, 0x0},
	{0x93, 0x0},
	{0x96, 0x0},
	{0x97, 0x8},
	{0x97, 0x19},
	{0x97, 0x2},
	{0x97, 0xc},
	{0x97, 0x24},
	{0x97, 0x30},
	{0x97, 0x28},
	{0x97, 0x26},
	{0x97, 0x2},
	{0x97, 0x98},
	{0x97, 0x80},
	{0x97, 0x0},
	{0x97, 0x0},
	{0xa4, 0x0},
	{0xa8, 0x0},
	{0xc5, 0x11},
	{0xc6, 0x51},
	{0xbf, 0x80},
	{0xc7, 0x10},
	{0xb6, 0x66},
	{0xb8, 0xa5},
	{0xb7, 0x64},
	{0xb9, 0x7c},
	{0xb3, 0xaf},
	{0xb4, 0x97},
	{0xb5, 0xff},
	{0xb0, 0xc5},
	{0xb1, 0x94},
	{0xb2, 0xf},
	{0xc4, 0x5c},
	{0xa6, 0x0},
	{0xa7, 0x20},
	{0xa7, 0xd8},
	{0xa7, 0x1b},
	{0xa7, 0x31},
	{0xa7, 0x0},
	{0xa7, 0x18},
	{0xa7, 0x20},
	{0xa7, 0xd8},
	{0xa7, 0x19},
	{0xa7, 0x31},
	{0xa7, 0x0},
	{0xa7, 0x18},
	{0xa7, 0x20},
	{0xa7, 0xd8},
	{0xa7, 0x19},
	{0xa7, 0x31},
	{0xa7, 0x0},
	{0xa7, 0x18},
	{0x7f, 0x0},
	{0xe5, 0x1f},
	{0xe1, 0x77},
	{0xdd, 0x7f},
	{0xc2, 0xe},

	{0xff, 0x0},
	{0xe0, 0x4},
	{0xc0, 0xc8},
	{0xc1, 0x96},
	{0x86, 0x3d},
	{0x51, 0x90},
	{0x52, 0x2c},
	{0x53, 0x0},
	{0x54, 0x0},
	{0x55, 0x88},
	{0x57, 0x0},

	{0x50, 0x92},
	{0x5a, 0x50},
	{0x5b, 0x3c},
	{0x5c, 0x0},
	{0xd3, 0x4},
	{0xe0, 0x0},

	{0xff, 0x0},
	{0x5, 0x0},

	{0xda, 0x8},
	{0xd7, 0x3},
	{0xe0, 0x0},

	{0x5, 0x0}
};

const unsigned char OV2640_JPEG_INIT[][2] =
{
  { 0xff, 0x00 },
  { 0x2c, 0xff },
  { 0x2e, 0xdf },
  { 0xff, 0x01 },
  { 0x3c, 0x32 },
  { 0x11, 0x00 },
  { 0x09, 0x02 },
  { 0x04, 0x28 },
  { 0x13, 0xe5 },
  { 0x14, 0x48 },
  { 0x2c, 0x0c },
  { 0x33, 0x78 },
  { 0x3a, 0x33 },
  { 0x3b, 0xfB },
  { 0x3e, 0x00 },
  { 0x43, 0x11 },
  { 0x16, 0x10 },
  { 0x39, 0x92 },
  { 0x35, 0xda },
  { 0x22, 0x1a },
  { 0x37, 0xc3 },
  { 0x23, 0x00 },
  { 0x34, 0xc0 },
  { 0x36, 0x1a },
  { 0x06, 0x88 },
  { 0x07, 0xc0 },
  { 0x0d, 0x87 },
  { 0x0e, 0x41 },
  { 0x4c, 0x00 },
  { 0x48, 0x00 },
  { 0x5B, 0x00 },
  { 0x42, 0x03 },
  { 0x4a, 0x81 },
  { 0x21, 0x99 },
  { 0x24, 0x40 },
  { 0x25, 0x38 },
  { 0x26, 0x82 },
  { 0x5c, 0x00 },
  { 0x63, 0x00 },
  { 0x61, 0x70 },
  { 0x62, 0x80 },
  { 0x7c, 0x05 },
  { 0x20, 0x80 },
  { 0x28, 0x30 },
  { 0x6c, 0x00 },
  { 0x6d, 0x80 },
  { 0x6e, 0x00 },
  { 0x70, 0x02 },
  { 0x71, 0x94 },
  { 0x73, 0xc1 },
  { 0x12, 0x40 },
  { 0x17, 0x11 },
  { 0x18, 0x43 },
  { 0x19, 0x00 },
  { 0x1a, 0x4b },
  { 0x32, 0x09 },
  { 0x37, 0xc0 },
  { 0x4f, 0x60 },
  { 0x50, 0xa8 },
  { 0x6d, 0x00 },
  { 0x3d, 0x38 },
  { 0x46, 0x3f },
  { 0x4f, 0x60 },
  { 0x0c, 0x3c },
  { 0xff, 0x00 },
  { 0xe5, 0x7f },
  { 0xf9, 0xc0 },
  { 0x41, 0x24 },
  { 0xe0, 0x14 },
  { 0x76, 0xff },
  { 0x33, 0xa0 },
  { 0x42, 0x20 },
  { 0x43, 0x18 },
  { 0x4c, 0x00 },
  { 0x87, 0xd5 },
  { 0x88, 0x3f },
  { 0xd7, 0x03 },
  { 0xd9, 0x10 },
  { 0xd3, 0x82 },
  { 0xc8, 0x08 },
  { 0xc9, 0x80 },
  { 0x7c, 0x00 },
  { 0x7d, 0x00 },
  { 0x7c, 0x03 },
  { 0x7d, 0x48 },
  { 0x7d, 0x48 },
  { 0x7c, 0x08 },
  { 0x7d, 0x20 },
  { 0x7d, 0x10 },
  { 0x7d, 0x0e },
  { 0x90, 0x00 },
  { 0x91, 0x0e },
  { 0x91, 0x1a },
  { 0x91, 0x31 },
  { 0x91, 0x5a },
  { 0x91, 0x69 },
  { 0x91, 0x75 },
  { 0x91, 0x7e },
  { 0x91, 0x88 },
  { 0x91, 0x8f },
  { 0x91, 0x96 },
  { 0x91, 0xa3 },
  { 0x91, 0xaf },
  { 0x91, 0xc4 },
  { 0x91, 0xd7 },
  { 0x91, 0xe8 },
  { 0x91, 0x20 },
  { 0x92, 0x00 },
  { 0x93, 0x06 },
  { 0x93, 0xe3 },
  { 0x93, 0x05 },
  { 0x93, 0x05 },
  { 0x93, 0x00 },
  { 0x93, 0x04 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x93, 0x00 },
  { 0x96, 0x00 },
  { 0x97, 0x08 },
  { 0x97, 0x19 },
  { 0x97, 0x02 },
  { 0x97, 0x0c },
  { 0x97, 0x24 },
  { 0x97, 0x30 },
  { 0x97, 0x28 },
  { 0x97, 0x26 },
  { 0x97, 0x02 },
  { 0x97, 0x98 },
  { 0x97, 0x80 },
  { 0x97, 0x00 },
  { 0x97, 0x00 },
  { 0xc3, 0xed },
  { 0xa4, 0x00 },
  { 0xa8, 0x00 },
  { 0xc5, 0x11 },
  { 0xc6, 0x51 },
  { 0xbf, 0x80 },
  { 0xc7, 0x10 },
  { 0xb6, 0x66 },
  { 0xb8, 0xA5 },
  { 0xb7, 0x64 },
  { 0xb9, 0x7C },
  { 0xb3, 0xaf },
  { 0xb4, 0x97 },
  { 0xb5, 0xFF },
  { 0xb0, 0xC5 },
  { 0xb1, 0x94 },
  { 0xb2, 0x0f },
  { 0xc4, 0x5c },
  { 0xc0, 0x64 },
  { 0xc1, 0x4B },
  { 0x8c, 0x00 },
  { 0x86, 0x3D },
  { 0x50, 0x00 },
  { 0x51, 0xC8 },
  { 0x52, 0x96 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x5a, 0xC8 },
  { 0x5b, 0x96 },
  { 0x5c, 0x00 },
  { 0xd3, 0x00 },	//{ 0xd3, 0x7f },
  { 0xc3, 0xed },
  { 0x7f, 0x00 },
  { 0xda, 0x00 },
  { 0xe5, 0x1f },
  { 0xe1, 0x67 },
  { 0xe0, 0x00 },
  { 0xdd, 0x7f },
  { 0x05, 0x00 },

  { 0x12, 0x40 },
  { 0xd3, 0x04 },	//{ 0xd3, 0x7f },
  { 0xc0, 0x16 },
  { 0xC1, 0x12 },
  { 0x8c, 0x00 },
  { 0x86, 0x3d },
  { 0x50, 0x00 },
  { 0x51, 0x2C },
  { 0x52, 0x24 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x5A, 0x2c },
  { 0x5b, 0x24 },
  { 0x5c, 0x00 }
};

const unsigned char OV2640_YUV422[][2] =
{
  { 0xFF, 0x00 },
  { 0x05, 0x00 },
  { 0xDA, 0x10 },
  { 0xD7, 0x03 },
  { 0xDF, 0x00 },
  { 0x33, 0x80 },
  { 0x3C, 0x40 },
  { 0xe1, 0x77 },
  { 0x00, 0x00 }
};

const unsigned char OV2640_JPEG[][2] =
{
  { 0xe0, 0x14 },
  { 0xe1, 0x77 },
  { 0xe5, 0x1f },
  { 0xd7, 0x03 },
  { 0xda, 0x10 },
  { 0xe0, 0x00 },
  { 0xFF, 0x01 },
  { 0x04, 0x08 }
};

const unsigned char OV2640_320x240_JPEG[][2] =
{
  { 0xff, 0x01 },
  { 0x12, 0x40 },
  { 0x17, 0x11 },
  { 0x18, 0x43 },
  { 0x19, 0x00 },
  { 0x1a, 0x4b },
  { 0x32, 0x09 },
  { 0x4f, 0xca },
  { 0x50, 0xa8 },
  { 0x5a, 0x23 },
  { 0x6d, 0x00 },
  { 0x39, 0x12 },
  { 0x35, 0xda },
  { 0x22, 0x1a },
  { 0x37, 0xc3 },
  { 0x23, 0x00 },
  { 0x34, 0xc0 },
  { 0x36, 0x1a },
  { 0x06, 0x88 },
  { 0x07, 0xc0 },
  { 0x0d, 0x87 },
  { 0x0e, 0x41 },
  { 0x4c, 0x00 },
  { 0xff, 0x00 },
  { 0xe0, 0x04 },
  { 0xc0, 0x64 },
  { 0xc1, 0x4b },
  { 0x86, 0x35 },
  { 0x50, 0x89 },
  { 0x51, 0xc8 },
  { 0x52, 0x96 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x57, 0x00 },
  { 0x5a, 0x50 },
  { 0x5b, 0x3c },
  { 0x5c, 0x00 },
  { 0xe0, 0x00 }
};

const unsigned char OV2640_640x480_JPEG[][2]={
		{0xff, 0x01},
		{0x11, 0x01},
		{0x12, 0x00}, // Bit[6:4]: Resolution selection
		{0x17, 0x11}, // HREFST[10:3]
		{0x18, 0x75}, // HREFEND[10:3]
		{0x32, 0x36}, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
		{0x19, 0x01}, // VSTRT[9:2]
		{0x1a, 0x97}, // VEND[9:2]
		{0x03, 0x0f}, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
		{0x37, 0x40},
		{0x4f, 0xbb},
		{0x50, 0x9c},
		{0x5a, 0x57},
		{0x6d, 0x80},
		{0x3d, 0x34},
		{0x39, 0x02},
		{0x35, 0x88},
		{0x22, 0x0a},
		{0x37, 0x40},
		{0x34, 0xa0},
		{0x06, 0x02},
		{0x0d, 0xb7},
		{0x0e, 0x01},

		{0xff, 0x00},
		{0xe0, 0x04},
		{0xc0, 0xc8},
		{0xc1, 0x96},
		{0x86, 0x3d},
		{0x50, 0x89},
		{0x51, 0x90},
		{0x52, 0x2c},
		{0x53, 0x00},
		{0x54, 0x00},
		{0x55, 0x88},
		{0x57, 0x00},
		{0x5a, 0xa0},
		{0x5b, 0x78},
		{0x5c, 0x00},
		{0xd3, 0x04},
		{0xe0, 0x00},
};
/**
  * @}
  */
  
/** @defgroup OV2640_Private_Functions
  * @{
  */ 

/**
  * @brief  Camera writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  * @retval None
  */
void CAMERA_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&Value, 1, 0xFFFFF);
  /* Check the communication status */
  if(status != HAL_OK) {
      /* Error wait */
      while(1){}
    }
}

/**
  * @brief  Camera reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data
  */
uint8_t CAMERA_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Read(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1, 0xFFFFF);
  /* Check the communication status */
  if(status != HAL_OK) {
      /* Error wait */
      while(1){}
    }

  return read_value;
}

/**
  * @brief  Initializes the OV2640 CAMERA component.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  resolution: Camera resolution and type
  * @retval None
  */
void ov2640_Init(uint16_t DeviceAddr, uint32_t resolution)
{
  uint32_t index;
  
  /* Initialize I2C */
  //CAMERA_IO_Init();
  
  /* Prepare the camera to be configured */
  CAMERA_IO_Write(DeviceAddr, OV2640_DSP_RA_DLMT, 0x01);
  CAMERA_IO_Write(DeviceAddr, OV2640_SENSOR_COM7, 0x80);
  HAL_Delay(200);
  
  /* Initialize OV2640 */
  switch (resolution)
  {
  case CAMERA_R320x240:
    {
      for(index=0; index<(sizeof(OV2640_QVGA)/2); index++)
      {
        CAMERA_IO_Write(DeviceAddr, OV2640_QVGA[index][0], OV2640_QVGA[index][1]);
        HAL_Delay(1);
      } 
      break;
    }
  case CAMERA_R320x240_JPEG:
    {
      for(index=0; index<(sizeof(OV2640_JPEG_INIT)/2); index++)
      {
        CAMERA_IO_Write(DeviceAddr, OV2640_JPEG_INIT[index][0], OV2640_JPEG_INIT[index][1]);
        HAL_Delay(2);
      }
      for(index=0; index<(sizeof(OV2640_YUV422)/2); index++)
      {
        CAMERA_IO_Write(DeviceAddr, OV2640_YUV422[index][0], OV2640_YUV422[index][1]);
        HAL_Delay(2);
      }
      for(index=0; index<(sizeof(OV2640_JPEG)/2); index++)
      {
        CAMERA_IO_Write(DeviceAddr, OV2640_JPEG[index][0], OV2640_JPEG[index][1]);
        HAL_Delay(2);
      }
      CAMERA_IO_Write(DeviceAddr, OV2640_DSP_RA_DLMT, 0x01);
      CAMERA_IO_Write(DeviceAddr, OV2640_SENSOR_COM10, 0x00);
      for(index=0; index<(sizeof(OV2640_320x240_JPEG)/2); index++)
      {
        CAMERA_IO_Write(DeviceAddr, OV2640_320x240_JPEG[index][0], OV2640_320x240_JPEG[index][1]);
        HAL_Delay(2);
      }
      break;
    }
  case CAMERA_R640x480_JPEG:
    {
      for(index=0; index<(sizeof(OV2640_JPEG_INIT)/2); index++)
      {
        CAMERA_IO_Write(DeviceAddr, OV2640_JPEG_INIT[index][0], OV2640_JPEG_INIT[index][1]);
        HAL_Delay(2);
      }
      for(index=0; index<(sizeof(OV2640_YUV422)/2); index++)
      {
        CAMERA_IO_Write(DeviceAddr, OV2640_YUV422[index][0], OV2640_YUV422[index][1]);
        HAL_Delay(2);
      }
      for(index=0; index<(sizeof(OV2640_JPEG)/2); index++)
      {
        CAMERA_IO_Write(DeviceAddr, OV2640_JPEG[index][0], OV2640_JPEG[index][1]);
        HAL_Delay(2);
      }
      CAMERA_IO_Write(DeviceAddr, OV2640_DSP_RA_DLMT, 0x01);
      CAMERA_IO_Write(DeviceAddr, OV2640_SENSOR_COM10, 0x00);
      for(index=0; index<(sizeof(OV2640_640x480_JPEG)/2); index++)
      {
        CAMERA_IO_Write(DeviceAddr, OV2640_640x480_JPEG[index][0], OV2640_640x480_JPEG[index][1]);
        HAL_Delay(2);
      }
      break;
    }
  default:
    {
      break;
    }
  }

}

/**
  * @brief  Configures the OV2640 camera feature.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  feature: Camera feature to be configured
  * @param  value: Value to be configured
  * @param  brightness_value: Brightness value to be configured
  * @retval None
  */
void ov2640_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t brightness_value)
{
  uint8_t value1, value2;
  uint32_t value_tmp;
  uint32_t br_value;
  
  /* Convert the input value into ov2640 parameters */
  value_tmp = ov2640_ConvertValue(feature, value); 
  br_value = ov2640_ConvertValue(CAMERA_CONTRAST_BRIGHTNESS, brightness_value); 
    
  switch(feature)
  {
  case CAMERA_BLACK_WHITE:
    {  
      CAMERA_IO_Write(DeviceAddr, 0xff, 0x00);
      CAMERA_IO_Write(DeviceAddr, 0x7c, 0x00);
      CAMERA_IO_Write(DeviceAddr, 0x7d, value_tmp);
      CAMERA_IO_Write(DeviceAddr, 0x7c, 0x05);
      CAMERA_IO_Write(DeviceAddr, 0x7d, 0x80);
      CAMERA_IO_Write(DeviceAddr, 0x7d, 0x80);
      break;
    }
  case CAMERA_CONTRAST_BRIGHTNESS:
    {
      value1 = (uint8_t)(value_tmp);
      value2 = (uint8_t)(value_tmp >> 8);
      CAMERA_IO_Write(DeviceAddr, 0xff, 0x00);     
      CAMERA_IO_Write(DeviceAddr, 0x7c, 0x00);
      CAMERA_IO_Write(DeviceAddr, 0x7d, 0x04);
      CAMERA_IO_Write(DeviceAddr, 0x7c, 0x07);
      CAMERA_IO_Write(DeviceAddr, 0x7d, br_value);
      CAMERA_IO_Write(DeviceAddr, 0x7d, value1);
      CAMERA_IO_Write(DeviceAddr, 0x7d, value2);
      CAMERA_IO_Write(DeviceAddr, 0x7d, 0x06);
      break;
    }
  case CAMERA_COLOR_EFFECT:
    {     
      value1 = (uint8_t)(value_tmp);
      value2 = (uint8_t)(value_tmp >> 8);
      CAMERA_IO_Write(DeviceAddr, 0xff, 0x00);
      CAMERA_IO_Write(DeviceAddr, 0x7c, 0x00);
      CAMERA_IO_Write(DeviceAddr, 0x7d, 0x18);
      CAMERA_IO_Write(DeviceAddr, 0x7c, 0x05);
      CAMERA_IO_Write(DeviceAddr, 0x7d, value1);
      CAMERA_IO_Write(DeviceAddr, 0x7d, value2);
      break;
    }     
  default:
    {
      break;
    }
  }
}

/**
  * @brief  Read the OV2640 Camera identity.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval the OV2640 ID
  */
uint16_t ov2640_ReadID(uint16_t DeviceAddr)
{
  /* Initialize I2C */
  CAMERA_IO_Init();
  
  /* Prepare the sensor to read the Camera ID */
  CAMERA_IO_Write(DeviceAddr, OV2640_DSP_RA_DLMT, 0x01);
  
  /* Get the camera ID */
  return (CAMERA_IO_Read(DeviceAddr, OV2640_SENSOR_PIDH));
}

/******************************************************************************
                            Static Functions
*******************************************************************************/
/**
  * @brief  Convert input values into ov2640 parameters.
  * @param  feature: Camera feature to be configured
  * @param  value: Value to be configured
  * @retval The converted value
  */
static uint32_t ov2640_ConvertValue(uint32_t feature, uint32_t value)
{
  uint32_t ret = 0;
  
  switch(feature)
  {
  case CAMERA_BLACK_WHITE:
    {
      switch(value)
      {
      case CAMERA_BLACK_WHITE_BW:
        {
          ret =  OV2640_BLACK_WHITE_BW;
          break;
        }
      case CAMERA_BLACK_WHITE_NEGATIVE:
        {
          ret =  OV2640_BLACK_WHITE_NEGATIVE;
          break;
        }
      case CAMERA_BLACK_WHITE_BW_NEGATIVE:
        {
          ret =  OV2640_BLACK_WHITE_BW_NEGATIVE;
          break;
        }
      case CAMERA_BLACK_WHITE_NORMAL:
        {
          ret =  OV2640_BLACK_WHITE_NORMAL;
          break;
        }
      default:
        {
          ret =  OV2640_BLACK_WHITE_NORMAL;
          break;
        }
      }
      break;
    }
  case CAMERA_CONTRAST_BRIGHTNESS:
    {
      switch(value)
      {
      case CAMERA_BRIGHTNESS_LEVEL0:
        {
          ret =  OV2640_BRIGHTNESS_LEVEL0;
          break;
        }
      case CAMERA_BRIGHTNESS_LEVEL1:
        {
          ret =  OV2640_BRIGHTNESS_LEVEL1;
          break;
        }
      case CAMERA_BRIGHTNESS_LEVEL2:
        {
          ret =  OV2640_BRIGHTNESS_LEVEL2;
          break;
        }
      case CAMERA_BRIGHTNESS_LEVEL3:
        {
          ret =  OV2640_BRIGHTNESS_LEVEL3;
          break;
        }
      case CAMERA_BRIGHTNESS_LEVEL4:
        {
          ret =  OV2640_BRIGHTNESS_LEVEL4;
          break;
        }        
      case CAMERA_CONTRAST_LEVEL0:
        {
          ret =  OV2640_CONTRAST_LEVEL0;
          break;
        }
      case CAMERA_CONTRAST_LEVEL1:
        {
          ret =  OV2640_CONTRAST_LEVEL1;
          break;
        }
      case CAMERA_CONTRAST_LEVEL2:
        {
          ret =  OV2640_CONTRAST_LEVEL2;
          break;
        }
      case CAMERA_CONTRAST_LEVEL3:
        {
          ret =  OV2640_CONTRAST_LEVEL3;
          break;
        }
      case CAMERA_CONTRAST_LEVEL4:
        {
          ret =  OV2640_CONTRAST_LEVEL4;
          break;
        }
      default:
        {
          ret =  OV2640_CONTRAST_LEVEL0;
          break;
        }
      }
      break;
    }
  case CAMERA_COLOR_EFFECT:
    {
      switch(value)
      {
      case CAMERA_COLOR_EFFECT_ANTIQUE:
        {
          ret =  OV2640_COLOR_EFFECT_ANTIQUE;
          break;
        }
      case CAMERA_COLOR_EFFECT_BLUE:
        {
          ret =  OV2640_COLOR_EFFECT_BLUE;
          break;
        }
      case CAMERA_COLOR_EFFECT_GREEN:
        {
          ret =  OV2640_COLOR_EFFECT_GREEN;
          break;
        }
      case CAMERA_COLOR_EFFECT_RED:
        {
          ret =  OV2640_COLOR_EFFECT_RED;
          break;
        }
      default:
        {
          ret =  OV2640_COLOR_EFFECT_RED;
          break;
        }
      }
      break;
    default:
      {
        ret = 0;
        break;
      }    
    }
  }
  
  return ret;
}

/**
  * @brief  Get the capture size in pixels unit.
  * @param  resolution: the current resolution.
  * @retval capture size in pixels unit.
  */
uint32_t GetSize(uint32_t resolution)
{
  uint32_t size = 0;

  /* Get capture size */
  switch (resolution)
  {
    case CAMERA_R160x120:
    {
      size =  0x2580;
    }
    break;
    case CAMERA_R320x240:
    {
      size =  0x9600;
    }
    break;
    case CAMERA_R480x272:
    {
      size =  0xFF00;
    }
    break;
    case CAMERA_R640x480:
    {
      size =  0x25800;
    }
    break;
    default:
    {
      break;
    }
  }

  return size;
}

/**
  * @}
  */ 
  
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */  

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
