/*
  This is a library written for the Panasonic Grid-EYE AMG88
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14568

  Written by Nick Poole @ SparkFun Electronics, January 11th, 2018

  The GridEYE from Panasonic is an 8 by 8 thermopile array capable
  of detecting temperature remotely at 64 discrete points.
  
  This library handles communication with the GridEYE and provides
  methods for manipulating temperature registers in Celsius,
  Fahrenheit and raw values.

  https://github.com/sparkfun/SparkFun_GridEYE_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_GridEYE_Arduino_Library.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo_144.h"
#include<stdlib.h>
#include <Math.h>


//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco
void GridEYE_begin(uint8_t deviceAddress, I2C_HandleTypeDef wirePort)
{
  _deviceAddress = deviceAddress;
  _i2cPort = wirePort;
}

//Change the address we read and write to
void GridEYE_setI2CAddress(uint8_t addr)
{
  _deviceAddress = addr;
}

/********************************************************
 * Functions for retreiving the temperature of
 * a single pixel. 
 ********************************************************
 * 
 * getPixelTemperature() - returns float Celsius
 * 
 * getPixelTemperatureFahrenheit() - returns float Fahrenheit
 * 
 * getPixelTemperatureRaw() - returns int16_t contents of
 *    both pixel temperature registers concatinated
 *    
 ********************************************************/

float GridEYE_getPixelTemperature(unsigned char pixelAddr)
{

  // Temperature registers are numbered 128-255
  // Each pixel has a lower and higher register
  unsigned char pixelLowRegister = TEMPERATURE_REGISTER_START + (2 * pixelAddr);
  int16_t temperature = GridEYE_getRegister(pixelLowRegister, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesC = temperature * 0.25;

  return DegreesC;

}

float GridEYE_getPixelTemperatureFahrenheit(unsigned char pixelAddr)
{

  // Temperature registers are numbered 128-255
  // Each pixel has a lower and higher register
  unsigned char pixelLowRegister = TEMPERATURE_REGISTER_START + (2 * pixelAddr);
  int16_t temperature = GridEYE_getRegister(pixelLowRegister, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesF = (temperature * 0.25) * 1.8 + 32;

  return DegreesF;

}

int16_t GridEYE_getPixelTemperatureRaw(unsigned char pixelAddr)
{

  // Temperature registers are numbered 128-255
  // Each pixel has a lower and higher register
  unsigned char pixelLowRegister = TEMPERATURE_REGISTER_START + (2 * pixelAddr);
  int16_t temperature = GridEYE_getRegister(pixelLowRegister, 2);

  return temperature;

}

/********************************************************
 * Functions for retreiving the temperature of
 * the device according to the embedded thermistor. 
 ******************************************************** 
 * 
 * getDeviceTemperature() - returns float Celsius
 * 
 * getDeviceTemperatureFahrenheit() - returns float Fahrenheit
 * 
 * getDeviceTemperatureRaw() - returns int16_t contents of
 *    both thermistor temperature registers concatinated
 *    
 ********************************************************/

float GridEYE_getDeviceTemperature()
{

  int16_t temperature = GridEYE_getRegister(THERMISTOR_REGISTER_LSB, 2);

    // temperature is reported as 12-bit twos complement
    // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float realTemperature = temperature * 0.0625;

  return realTemperature;

}

float GridEYE_getDeviceTemperatureFahrenheit()
{

  int16_t temperature = GridEYE_getRegister(THERMISTOR_REGISTER_LSB, 2);

    // temperature is reported as 12-bit twos complement
    // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float realTemperatureF = (temperature * 0.0625) * 1.8 + 32;

  return realTemperatureF;

}

int16_t GridEYE_getDeviceTemperatureRaw()
{

  int16_t temperature = GridEYE_getRegister(THERMISTOR_REGISTER_LSB, 2);

  return temperature;

}

/********************************************************
 * Functions for manipulating Framerate
 ******************************************************** 
 * 
 * Internal framerate of the device is always 10fps
 * When operating in 1FPS mode, each frame is an average
 * of 10 readings.
 * 
 * setFramerate1FPS() - sets framerate to 1 Frame per Second
 * 
 * setFramerate10FPS() - sets framerate to 10 Frames per Second
 * 
 * isFramerate10FPS() - returns true if framerate is currently
 *    set to 10 Frames per Second (device default)
 *    
 ********************************************************/

void GridEYE_setFramerate1FPS()
{
    GridEYE_setRegister(FRAMERATE_REGISTER, 1);
}

void GridEYE_setFramerate10FPS()
{
    GridEYE_setRegister(FRAMERATE_REGISTER, 0);
}

bool GridEYE_isFramerate10FPS()
{

    if(GridEYE_getRegister(FRAMERATE_REGISTER, 1) == 0){
      return true;
    }else{
      return false;
    }
  
}

/********************************************************
 * Functions for manipulating Operating Mode
 ******************************************************** 
 * 
 * Device defaults to normal mode on reset. 
 * When the device is in standby mode, the temperature
 * register is only updated intermittently.
 * 
 * wake() - returns device to normal mode from any
 *    other state.
 *    
 * sleep() - puts device into sleep mode, temperature
 *    register is not updated
 * 
 * standby60seconds() - puts device into standby mode
 *    with 60 second update frequency
 *    
 * standby10seconds() - puts device into standby mode
 *    with 10 second update frequency
 *    
 ********************************************************/

void GridEYE_wake(){

    GridEYE_setRegister(POWER_CONTROL_REGISTER, 0x00);
  
}

void GridEYE_sleep(){

    GridEYE_setRegister(POWER_CONTROL_REGISTER, 0x10);
  
}

void GridEYE_standby60seconds(){

    GridEYE_setRegister(POWER_CONTROL_REGISTER, 0x20);
  
}

void GridEYE_standby10seconds(){

    GridEYE_setRegister(POWER_CONTROL_REGISTER, 0x21);
  
}

/********************************************************
 * Functions for manipulating Interrupt Control Register
 ******************************************************** 
 * 
 * interruptPinEnable() - Enable INT pin to pull low on 
 *    interrupt flag
 *    
 * interruptPinDisable() - Put INT pin into Hi-Z state
 * 
 * setInterruptModeAbsolute() - Set interrupt mode to
 *    "Absolute Value" mode
 *    
 * setInterruptModeDifference() - Set interrupt mode to
 *    "Difference" mode
 *    
 * interruptPinEnabled() - returns true if the INT pin
 *    is enabled. Returns false if INT pin is in Hi-Z 
 * 
 ********************************************************/

void GridEYE_interruptPinEnable(){

    int16_t ICRValue = GridEYE_getRegister(INT_CONTROL_REGISTER, 1);
    
    ICRValue |= (1 << 0);
    
    GridEYE_setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);
  
}

void GridEYE_interruptPinDisable(){

    int16_t ICRValue = GridEYE_getRegister(INT_CONTROL_REGISTER, 1);
    
    ICRValue &= ~(1 << 0);
    
    GridEYE_setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);
  
}

void GridEYE_setInterruptModeAbsolute(){

    int16_t ICRValue = GridEYE_getRegister(INT_CONTROL_REGISTER, 1);
    
    ICRValue |= (1 << 1);
    
    GridEYE_setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);
   
}

void GridEYE_setInterruptModeDifference(){

    int16_t ICRValue = GridEYE_getRegister(INT_CONTROL_REGISTER, 1);
    
    ICRValue &= ~(1 << 1);
    
    GridEYE_setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);
  
}

bool GridEYE_interruptPinEnabled(){

  int16_t ICRValue = GridEYE_getRegister(INT_CONTROL_REGISTER, 1);
  if(ICRValue & (1 << 0)){
    return true;
  }else{
    return false;
  }
  
}

/********************************************************
 * Functions for manipulating Status/Clear Registers
 ******************************************************** 
 * 
 * interruptFlagSet() - returns true if there is an 
 *    interrupt flag in the status register
 *    
 * pixelTemperatureOutputOK() - returns false if temperature
 *    output overflow flag is present in status register
 * 
 * deviceTemperatureOutputOK() - returns false if thermistor
 *    output overflow flag is present in status register
 *    
 * clearInterruptFlag() - clears interrupt flag in the 
 *    status register
 *    
 * clearPixelTemperatureOverflow() - clears temperature
 *    output overflow flag in status register
 *    
 * clearDeviceTemperatureOverflow() - clears thermistor
 *    output overflow flag in status register
 *    
 * clearAllOverflow() - clears both thermistor and 
 *    temperature overflow flags in status register but
 *    leaves interrupt flag untouched
 *    
 * clearAllStatusFlags() - clears all flags in status 
 *    register
 * 
 ********************************************************/

bool GridEYE_interruptFlagSet(){

  int16_t StatRegValue = GridEYE_getRegister(STATUS_REGISTER, 1);
  if(StatRegValue & (1 << 1)){
    return true;
  }else{
    return false;
  }  
  
}

bool GridEYE_pixelTemperatureOutputOK(){

  int16_t StatRegValue = GridEYE_getRegister(STATUS_REGISTER, 1);
  if(StatRegValue & (1 << 2)){
    return false;
  }else{
    return true;
  }  
  
}

bool GridEYE_deviceTemperatureOutputOK(){

  int16_t StatRegValue = GridEYE_getRegister(STATUS_REGISTER, 1);
  if(StatRegValue & (1 << 3)){
    return false;
  }else{
    return true;
  }  
  
}

void GridEYE_clearInterruptFlag(){
    
    GridEYE_setRegister(STATUS_CLEAR_REGISTER, 0x02);
  
}

void GridEYE_clearPixelTemperatureOverflow(){
    
    GridEYE_setRegister(STATUS_CLEAR_REGISTER, 0x04);
  
}

void GridEYE_clearDeviceTemperatureOverflow(){
    
    GridEYE_setRegister(STATUS_CLEAR_REGISTER, 0x08);
  
}

void GridEYE_clearAllOverflow(){

    GridEYE_setRegister(STATUS_CLEAR_REGISTER, 0x0C);
  
}

void GridEYE_clearAllStatusFlags(){

    GridEYE_setRegister(STATUS_CLEAR_REGISTER, 0x0E);
  
}

/********************************************************
 * Function for reading Interrupt Table Register
 ******************************************************** 
 * 
 * pixelInterruptSet() - Returns true if interrupt flag 
 * is set for the specified pixel
 * 
 ********************************************************/

bool GridEYE_pixelInterruptSet(uint8_t pixelAddr){

  unsigned char interruptTableRegister = INT_TABLE_REGISTER_INT0 + (pixelAddr / 8);
  uint8_t pixelPosition = (pixelAddr % 8);

  int16_t interruptTableRow = GridEYE_getRegister(interruptTableRegister, 1);

  if(interruptTableRow & (1 << pixelPosition)){
    return true;
  }else{
    return false;
  }    

}

/********************************************************
 * Functions for manipulating Average Register
 ******************************************************** 
 * 
 * Moving Average Mode enable and disable are only 
 * referenced in some of the documentation for this 
 * device but not in all documentation. Requires writing
 * in sequence to a reserved register. I'm not sure it 
 * does anything.
 * 
 * movingAverageEnable() - enable "Twice Moving Average" 
 * 
 * movingAverageDisable() - disable "Twice Moving Average"
 * 
 * movingAverageEnabled() - returns true if enabled
 * 
 ********************************************************/

void GridEYE_movingAverageEnable(){

    GridEYE_setRegister(RESERVED_AVERAGE_REGISTER, 0x50);
    GridEYE_setRegister(RESERVED_AVERAGE_REGISTER, 0x45);
    GridEYE_setRegister(RESERVED_AVERAGE_REGISTER, 0x57);
    GridEYE_setRegister(AVERAGE_REGISTER, 0x20);
    GridEYE_setRegister(RESERVED_AVERAGE_REGISTER, 0x00);
  
}

void GridEYE_movingAverageDisable(){

    GridEYE_setRegister(RESERVED_AVERAGE_REGISTER, 0x50);
    GridEYE_setRegister(RESERVED_AVERAGE_REGISTER, 0x45);
    GridEYE_setRegister(RESERVED_AVERAGE_REGISTER, 0x57);
    GridEYE_setRegister(AVERAGE_REGISTER, 0x00);
    GridEYE_setRegister(RESERVED_AVERAGE_REGISTER, 0x00);
  
}

bool GridEYE_movingAverageEnabled(){

  int16_t AVGRegValue = GridEYE_getRegister(AVERAGE_REGISTER, 1);
  if(AVGRegValue & (1 << 5)){
    return true;
  }else{
    return false;
  }  
  
}

/********************************************************
 * Functions for manipulating Interrupt Level Register
 ******************************************************** 
 * 
 * setUpperInterruptValue() - accepts float Celsius 
 * 
 * setUpperInterruptValueRaw() - accepts int16_t register
 *    configuration
 * 
 * setUpperInterruptValueFahrenheit() - accepts float 
 *    Fahrenheit
 *  
 * setLowerInterruptValue() - accepts float Celsius 
 * 
 * setLowerInterruptValueRaw() - accepts int16_t register
 *    configuration
 * 
 * setLowerInterruptValueFahrenheit() - accepts float 
 *    Fahrenheit
 * 
 * setInterruptHysteresis() - accepts float Celsius
 * 
 * setInterruptHysteresisRaw() - accepts int16_t register
 *    configuration
 * 
 * setInterruptHysteresisFahrenheit() - accepts float 
 *    Fahrenheit
 *    
 * getUpperInterruptValue() - returns float Celsius 
 * 
 * getUpperInterruptValueRaw() - returns int16_t register
 *    contents
 * 
 * getUpperInterruptValueFahrenheit() - returns float 
 *    Fahrenheit
 *  
 * getLowerInterruptValue() - returns float Celsius 
 * 
 * getLowerInterruptValueRaw() - returns int16_t register
 *    contents
 * 
 * getLowerInterruptValueFahrenheit() - returns float 
 *    Fahrenheit
 * 
 * getInterruptHysteresis() - returns float Celsius
 * 
 * getInterruptHysteresisRaw() - returns int16_t register
 *    contents
 * 
 * getInterruptHysteresisFahrenheit() - returns float 
 *    Fahrenheit   
 * 
 ********************************************************/

void GridEYE_setUpperInterruptValue(float DegreesC){

  bool isNegative = false;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  GridEYE_setRegister(INT_LEVEL_REGISTER_UPPER_LSB, temperature & 0xFF);
  GridEYE_setRegister(INT_LEVEL_REGISTER_UPPER_MSB, temperature >> 8);
  
}

void GridEYE_setUpperInterruptValueRaw(int16_t regValue){
  
  GridEYE_setRegister(INT_LEVEL_REGISTER_UPPER_LSB, regValue & 0xFF);
  GridEYE_setRegister(INT_LEVEL_REGISTER_UPPER_MSB, regValue >> 8);
  
}

void GridEYE_setUpperInterruptValueFahrenheit(float DegreesF){

  bool isNegative = false;

  float DegreesC = (DegreesF - 32) / 1.8;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  GridEYE_setRegister(INT_LEVEL_REGISTER_UPPER_LSB, temperature & 0xFF);
  GridEYE_setRegister(INT_LEVEL_REGISTER_UPPER_MSB, temperature >> 8);
  
}

void GridEYE_setLowerInterruptValue(float DegreesC){

  bool isNegative = false;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  GridEYE_setRegister(INT_LEVEL_REGISTER_LOWER_LSB, temperature & 0xFF);
  GridEYE_setRegister(INT_LEVEL_REGISTER_LOWER_MSB, temperature >> 8);
  
}

void GridEYE_setLowerInterruptValueRaw(int16_t regValue){

  GridEYE_setRegister(INT_LEVEL_REGISTER_LOWER_LSB, regValue & 0xFF);
  GridEYE_setRegister(INT_LEVEL_REGISTER_LOWER_MSB, regValue >> 8);
  
}

void GridEYE_setLowerInterruptValueFahrenheit(float DegreesF){

  bool isNegative = false;

  float DegreesC = (DegreesF - 32) / 1.8;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  GridEYE_setRegister(INT_LEVEL_REGISTER_LOWER_LSB, temperature & 0xFF);
  GridEYE_setRegister(INT_LEVEL_REGISTER_LOWER_MSB, temperature >> 8);
  
}

void GridEYE_setInterruptHysteresis(float DegreesC){

  bool isNegative = false;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  GridEYE_setRegister(INT_LEVEL_REGISTER_HYST_LSB, temperature & 0xFF);
  GridEYE_setRegister(INT_LEVEL_REGISTER_HYST_MSB, temperature >> 8);
  
}

void GridEYE_setInterruptHysteresisRaw(int16_t regValue){
  
  GridEYE_setRegister(INT_LEVEL_REGISTER_HYST_LSB, regValue & 0xFF);
  GridEYE_setRegister(INT_LEVEL_REGISTER_HYST_MSB, regValue >> 8);
  
}

void GridEYE_setInterruptHysteresisFahrenheit(float DegreesF){

  bool isNegative = false;

  float DegreesC = (DegreesF - 32) / 1.8;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  GridEYE_setRegister(INT_LEVEL_REGISTER_HYST_LSB, temperature & 0xFF);
  GridEYE_setRegister(INT_LEVEL_REGISTER_HYST_MSB, temperature >> 8);
  
}

float GridEYE_getUpperInterruptValue()
{

  int16_t temperature = GridEYE_getRegister(INT_LEVEL_REGISTER_UPPER_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesC = temperature * 0.25;

  return DegreesC;

}

int16_t GridEYE_getUpperInterruptValueRaw()
{

  return GridEYE_getRegister(INT_LEVEL_REGISTER_UPPER_LSB, 2);

}

float GridEYE_getUpperInterruptValueFahrenheit()
{

  int16_t temperature = GridEYE_getRegister(INT_LEVEL_REGISTER_UPPER_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesF = (temperature * 0.25) * 1.8 + 32;

  return DegreesF;

}

float GridEYE_getLowerInterruptValue()
{

  int16_t temperature = GridEYE_getRegister(INT_LEVEL_REGISTER_LOWER_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesC = temperature * 0.25;

  return DegreesC;

}

float GridEYE_getLowerInterruptValueFahrenheit()
{

  int16_t temperature = GridEYE_getRegister(INT_LEVEL_REGISTER_LOWER_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesF = (temperature * 0.25) * 1.8 + 32;

  return DegreesF;

}

int16_t GridEYE_getLowerInterruptValueRaw()
{

  return GridEYE_getRegister(INT_LEVEL_REGISTER_LOWER_LSB, 2);

}

float GridEYE_getInterruptHysteresis()
{

  int16_t temperature = GridEYE_getRegister(INT_LEVEL_REGISTER_HYST_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesC = temperature * 0.25;

  return DegreesC;

}

float GridEYE_getInterruptHysteresisFahrenheit()
{

  int16_t temperature = GridEYE_getRegister(INT_LEVEL_REGISTER_HYST_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesF = (temperature * 0.25) * 1.8 + 32;

  return DegreesF;

}

int16_t GridEYE_getInterruptHysteresisRaw()
{

  return GridEYE_getRegister(INT_LEVEL_REGISTER_HYST_LSB, 2);

}

/********************************************************
 * Functions for setting and getting registers over I2C
 ******************************************************** 
 * 
 * GridEYE_setRegister() - set unsigned char value at unsigned char register
 * 
 * GridEYE_getRegister() - get up to INT16 value from unsigned char register
 * 
 ********************************************************/

void GridEYE_GridEYE_setRegister(unsigned char reg, unsigned char val)
{

	HAL_StatusTypeDef status;
	/*
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);
    _i2cPort->write(val);
    _i2cPort->endTransmission();
    */
    
	status =  HAL_I2C_Mem_Write(&_i2cPort,_deviceAddress<<1,reg,
    		I2C_MEMADD_SIZE_8BIT,(uint8_t *)&val,sizeof(val),100);

    if(status != HAL_OK) {
    	//BSP_LED_On(LED2);
    }

}

int16_t GridEYE_getRegister(unsigned char reg, int8_t len)
{
  int16_t result = 0;
  HAL_StatusTypeDef status=0;
  /*
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);
    _i2cPort->endTransmission(false);
    _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)len);
    */

  uint8_t lsb = 0;
  uint8_t msb = 0;
  /*ad hoc implementation only works with 2 bytes or fewer*/
  if(len == 1) {
	  status = HAL_I2C_Mem_Read(&_i2cPort,_deviceAddress<<1,reg,
			  I2C_MEMADD_SIZE_8BIT,(uint8_t *)&result,len,10);
  }
  else {
	  status = HAL_I2C_Mem_Read(&_i2cPort,_deviceAddress<<1,reg,
			  I2C_MEMADD_SIZE_8BIT,(uint8_t *)&lsb,1,10);
	  status = HAL_I2C_Mem_Read(&_i2cPort,_deviceAddress<<1,reg+1,
			  I2C_MEMADD_SIZE_8BIT,(uint8_t *)&msb,1,10);
	  result = (uint16_t) msb << 8 | lsb;
  }

  if(status != HAL_OK) {
	  //BSP_LED_On(LED2);
  }

    /*
    while(_i2cPort->available())    // client may send less than requested
    {
      // Get bytes from sensor
      uint8_t lsb = _i2cPort->read(); 
      uint8_t msb = _i2cPort->read(); 
  
      // concat bytes into int
      result = (uint16_t)msb << 8 | lsb;
    }

    _i2cPort->endTransmission();
    */
  //result = (uint16_t)buffer[1] << 8 | buffer[0];

    return result;
                         
}
