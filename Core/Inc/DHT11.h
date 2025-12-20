#ifndef DHT11_H
#define DHT11_H
#include "stdint.h"


extern void delay_us(uint16_t us);
extern uint8_t DHT11_Humidity_Int;
extern uint8_t DHT11_Humidity_Dec;
extern uint8_t DHT11_Temperature_Int;
extern uint8_t DHT11_Temperature_Dec;
extern uint8_t DHT11_Checksum;



void DHT11_SetOutput(void);
void DHT11_SetInput(void);
uint8_t DHT11_Start(void);
uint8_t DHT11_ReadByte(void);
uint8_t DHT11_ReadData(void);

#endif