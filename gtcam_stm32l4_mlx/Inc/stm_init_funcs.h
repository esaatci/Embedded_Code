#include <string.h>
#include <stdio.h>
#include "ov2640.h"
#include "stm32l4xx_nucleo_144.h"
#include "SparkFun_GridEYE_Arduino_Library.h"
#include "stm32_adafruit_sd.h"
#include "salsa20.h"





void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_LPUART1_UART_Init(void);
void MX_DCMI_Init(void);
void MX_I2C1_Init(void);
void MX_TIM1_Init(void);
void MX_RTC_Init(void);
void MX_SPI1_Init(void);
void MX_ADC1_Init(void);
void MX_I2C4_Init(void);
