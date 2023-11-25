/*
 *  file: stm32_ds3231.c
 *  Created on: 2022
 *  Author: Masoud Babaabasi
 */
//#include "stm32h7xx_hal.h"
#include "DS3231.h"

I2C_HandleTypeDef *i2c;
/*
*   @brief: a static function for conversion Binary Coded Decimal(BSD) to decimal
*   @param: input BCD number
*   @retval: decimal output number
*/
static uint8_t B2D(uint8_t bcd)
{
  return (bcd >> 4) * 10 + (bcd & 0x0F);
}
/*
*   @brief: a static function for conversion decimal to Binary Coded Decimal(BSD)
*   @param: input decimal number
*   @retval: BCD number
*/
static uint8_t D2B(uint8_t decimal)
{
  return (((decimal / 10) << 4) | (decimal % 10));
}
/*
*   @brief: Initilize the library with the I2C handle structure( Must use STM32 HAL library)
*   @param: pointer to I2C_HandleTypeDef structure which has been initilized before
*   @retval: BCD number
*/
void DS3231_Init(I2C_HandleTypeDef *handle)
{
  i2c = handle;
}
/*
*   @brief: Get data and time from DS3231 chip(must initilize RTC with CubeMX or include RTC HAL library like "stm32h7xx_hal_rtc.h")
*   @param: pointer RTC_TimeTypeDef 
*   @param: pointer RTC_DateTypeDef 
*   @retval: 1 on success, 0 on fail
*/
uint8_t DS3231_GetTime(RTC_TimeTypeDef *rtc_time , RTC_DateTypeDef *rtc_date)
{
  uint8_t startAddr = DS3231_REG_TIME;
  uint8_t buffer[7] = {0,};

  if(HAL_I2C_Master_Transmit(i2c, DS3231_ADDR, &startAddr, 1, HAL_MAX_DELAY) != HAL_OK) return 0;
  if(HAL_I2C_Master_Receive(i2c, DS3231_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return 0;

  rtc_time->Seconds = B2D(buffer[0] & 0x7F);
  rtc_time->Minutes = B2D(buffer[1] & 0x7F);
  rtc_time->Hours = B2D(buffer[2] & 0x3F);
  rtc_date->WeekDay = buffer[3] & 0x07;
  rtc_date->Date = B2D(buffer[4] & 0x3F);
  rtc_date->Month = B2D(buffer[5] & 0x1F);
  rtc_date->Year = B2D(buffer[6]);

  return 1;
}
/*
*   @brief: Set data and time to DS3231 chip(must initilize RTC with CubeMX or include RTC HAL library like "stm32h7xx_hal_rtc.h")
*   @param: pointer RTC_TimeTypeDef 
*   @param: pointer RTC_DateTypeDef 
*   @retval: 1 on success, 0 on fail
*/
uint8_t DS3231_SetTime(RTC_TimeTypeDef *rtc_time , RTC_DateTypeDef *rtc_date)
{
  uint8_t startAddr = DS3231_REG_TIME;
  uint8_t buffer[8] = {startAddr, D2B(rtc_time->Seconds), D2B(rtc_time->Minutes), D2B(rtc_time->Hours), rtc_date->WeekDay, D2B(rtc_date->Date), D2B(rtc_date->Month), D2B(rtc_date->Year)};
  if(HAL_I2C_Master_Transmit(i2c, DS3231_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return 0;
  return 1;
}
/*
*   @brief: read DS3231 internal temprature sensor information
*   @param: pointer a float number which will contain temrature data
*   @retval: 1 on success, 0 on fail
*/
uint8_t DS3231_ReadTemperature(float *temp)
{
  uint8_t startAddr = DS3231_REG_TEMP;
  uint8_t buffer[2] = {0,};

  if(HAL_I2C_Master_Transmit(i2c, DS3231_ADDR, &startAddr, 1, HAL_MAX_DELAY) != HAL_OK) return 0;
  if(HAL_I2C_Master_Receive(i2c, DS3231_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return 0;

  int16_t value = (buffer[0] << 8) | (buffer[1]);
  value = (value >> 6);

  *temp = value / 4.0f;
  return 1;
}
/*
*   @brief: set alaram1 data
*   @param: Alarmode: ALARM_MODE_ALL_MATCHED , 
*                     ALARM_MODE_HOUR_MIN_SEC_MATCHED , 
*                     ALARM_MODE_MIN_SEC_MATCHED, 
*                     ALARM_MODE_SEC_MATCHED, 
*                     ALARM_MODE_ONCE_PER_SECOND
*   @param: alarm date
*   @param: alarm hour
*   @param: alarm min
*   @param: alarm sec
*   @retval: 1 on success, 0 on fail
*/
uint8_t DS3231_SetAlarm1(AlarmMode mode, uint8_t date, uint8_t hour, uint8_t min, uint8_t sec)
{
  uint8_t alarmSecond = D2B(sec);
  uint8_t alarmMinute = D2B(min);
  uint8_t alarmHour = D2B(hour);
  uint8_t alarmDate = D2B(date);

  switch(mode)
  {
  case ALARM_MODE_ALL_MATCHED:
    break;
  case ALARM_MODE_HOUR_MIN_SEC_MATCHED:
    alarmDate |= 0x80;
    break;
  case ALARM_MODE_MIN_SEC_MATCHED:
    alarmDate |= 0x80;
    alarmHour |= 0x80;
    break;
  case ALARM_MODE_SEC_MATCHED:
    alarmDate |= 0x80;
    alarmHour |= 0x80;
    alarmMinute |= 0x80;
    break;
  case ALARM_MODE_ONCE_PER_SECOND:
    alarmDate |= 0x80;
    alarmHour |= 0x80;
    alarmMinute |= 0x80;
    alarmSecond |= 0x80;
    break;
  default:
    break;
  }

  /* Write Alarm Registers */
  uint8_t startAddr = DS3231_REG_ALARM1;
  uint8_t buffer[5] = {startAddr, alarmSecond, alarmMinute, alarmHour, alarmDate};
  if(HAL_I2C_Master_Transmit(i2c, DS3231_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return 0;

  /* Enable Alarm1 at Control Register */
  uint8_t ctrlReg = 0x00;
  ReadRegister(DS3231_REG_CONTROL, &ctrlReg);
  ctrlReg |= DS3231_CON_A1IE;
  ctrlReg |= DS3231_CON_INTCN;
  WriteRegister(DS3231_REG_CONTROL, ctrlReg);

  return 1;
}
/*
*   @brief: clear Alarm1 data
*   @retval: 1 on success, 0 on fail
*/
uint8_t DS3231_ClearAlarm1()
{
  uint8_t ctrlReg;
  uint8_t statusReg;

  /* Clear Control Register */
  ReadRegister(DS3231_REG_CONTROL, &ctrlReg);
  ctrlReg &= ~DS3231_CON_A1IE;
  WriteRegister(DS3231_REG_CONTROL, ctrlReg);

  /* Clear Status Register */
  ReadRegister(DS3231_REG_STATUS, &statusReg);
  statusReg &= ~DS3231_STA_A1F;
  WriteRegister(DS3231_REG_STATUS, statusReg);

  return 1;
}
/*
*   @brief: read a register from DS3231
*   @param: register address
*   @param: pointer to register value
*   @retval: 1 on success, 0 on fail
*/
uint8_t ReadRegister(uint8_t regAddr, uint8_t *value)
{
  if(HAL_I2C_Master_Transmit(i2c, DS3231_ADDR, &regAddr, 1, HAL_MAX_DELAY) != HAL_OK) return 0;
  if(HAL_I2C_Master_Receive(i2c, DS3231_ADDR, value, 1, HAL_MAX_DELAY) != HAL_OK) return 0;

  return 1;
}
/*
*   @brief: write a register to DS3231
*   @param: register address
*   @param: register value
*   @retval: 1 on success, 0 on fail
*/
uint8_t WriteRegister(uint8_t regAddr, uint8_t value)
{
  uint8_t buffer[2] = {regAddr, value};
  if(HAL_I2C_Master_Transmit(i2c, DS3231_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return 0;

  return 1;
}
/*****************************END OF FILE***************************************************/

