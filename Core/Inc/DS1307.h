#ifndef DS1307_H
#define DS1307_H
#include <stdint.h>

#define DS1307_ADDR  (0x68 << 1)
typedef struct 
  {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t date;
    uint8_t month;
    uint8_t year;
  }DS1307_RTC;

  uint8_t B2D( uint8_t num);
  uint8_t D2B(uint8_t num);
  void DS1307_Write(DS1307_RTC *rtc);
  void DS1307_Read(DS1307_RTC *rtc);
  void DS1307_Start(void);
  void DS1307_Stop(void);
  




#endif