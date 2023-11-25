# DS3231 accurate RTC
### STM32 external Real Time Clock DS3231 C library
This code is written for use in stm32 family projects. It uses HAL I2C read and write functions. So You need to initilize the I2C with HAL library and then use this code for communication with the RTC chip. This library also relies on STM32 RTC library, so you need to initilize RTC via CubeMX or include the STM32 rtc header file manualy.

I recommend to use STM32CubeMX to initilize the microcontroller. You can use the my library in the main.c file like shown below:
```C
#include "DS3231.h"
I2C_HandleTypeDef hi2c1;
int main(void)
{
/*
* Start up code 
*/
  HAL_Init();
  SystemClock_Config();
/*
* CubeMX Initilization functions
*/
  MX_I2C1_Init();
/* USER CODE BEGIN 2 */
  RTC_TimeTypeDef DS_time;
  RTC_DateTypeDef DS_date;
  char str[50];
  DS3231_Init(&hi2c1);

  DS3231_GetTime(&DS_time , &DS_date);

  sprintf(str,"DS3231 = %d-%d-%d - %d:%d:%d\n",DS_date.Year,DS_date.Month ,DS_date.Date , DS_time.Hours , DS_time.Minutes , DS_time.Seconds);
/* USER CODE END 2 */
while (1)
  {

  }
}
```

