#include "DHT11.h"
#include "stm32f1xx_hal.h"


void DHT11_SetOutput(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void DHT11_SetInput(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


uint8_t DHT11_Start(void)
{
  uint16_t timeout = 0;
  __disable_irq();
  
  
  DHT11_SetOutput();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
  
 
  __enable_irq();
  HAL_Delay(18);  
  __disable_irq();
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
  delay_us(30);
  
  DHT11_SetInput();
  
  timeout = 80;
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET && timeout--) {
    delay_us(1);
  }
  if(timeout == 0) {
    __enable_irq(); 
    return 0;
  }
  
  timeout = 80;
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET && timeout--) {
    delay_us(1);
  }
  if(timeout == 0) {
    __enable_irq();
    return 0;
  }
  
  
  timeout = 200;
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET && timeout--) {
    delay_us(1);
  }
  if(timeout == 0) {
    __enable_irq();
    return 0;
  }
  
  return 1;  
}


uint8_t DHT11_ReadByte(void)
{
  uint8_t data = 0;
  uint16_t timeout = 0;
  
  for(uint8_t i = 0; i < 8; i++)
  {
    
    timeout = 200;
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET && timeout--) {
      delay_us(1);
    }
    if(timeout == 0) return 0;
    
    
    delay_us(40);
    
    
    data <<= 1;
    
    
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
      data |= 1;  
    }
    timeout = 200;
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET && timeout--) {
      delay_us(1);
    }
  }
  
  return data;
}


 uint8_t DHT11_Humidity_Int = 0;
 uint8_t DHT11_Humidity_Dec = 0;
 uint8_t DHT11_Temperature_Int = 0;
 uint8_t DHT11_Temperature_Dec =0;
 uint8_t DHT11_Checksum =0;



uint8_t DHT11_ReadData(void)
{
  uint8_t data[5] = {0};
  
  if(!DHT11_Start()) {
    __enable_irq();
    return 0;
  }
  
 
  for(uint8_t i = 0; i < 5; i++) {
    data[i] = DHT11_ReadByte();
  }
  

  __enable_irq();
  

  uint8_t checksum = data[0] + data[1] + data[2] + data[3];
  if(checksum != data[4]) {
    return 0; 
  }
  
  DHT11_Humidity_Int = data[0];
  DHT11_Humidity_Dec = data[1];
  DHT11_Temperature_Int = data[2];
  DHT11_Temperature_Dec = data[3];
  DHT11_Checksum = data[4];
  
  return 1;  
}

