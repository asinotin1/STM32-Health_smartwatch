
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "max30102.h"

extern I2C_HandleTypeDef hi2c1;

void maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{
	HAL_I2C_Mem_Write(&hi2c1, I2C_WRITE_ADDR, uch_addr, 1U, &uch_data, 1U, 100U);
}


void maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
{
	HAL_I2C_Mem_Read(&hi2c1, I2C_READ_ADDR, uch_addr, 1U, puch_data, 1U, 250U);
}

void maxim_max30102_init()
{
    maxim_max30102_write_reg(REG_INTR_ENABLE_1, 0xc0U); // INTR setting
    maxim_max30102_write_reg(REG_INTR_ENABLE_2, 0x00U);
    maxim_max30102_write_reg(REG_FIFO_WR_PTR, 0x00U); // //FIFO_WR_PTR[4:0]
    maxim_max30102_write_reg(REG_OVF_COUNTER, 0x00U); // //OVF_COUNTER[4:0]
    maxim_max30102_write_reg(REG_FIFO_RD_PTR, 0x00U); // //FIFO_RD_PTR[4:0]
    maxim_max30102_write_reg(REG_FIFO_CONFIG, 0x5fU); //sample avg = 4, fifo rollover=true, fifo almost full = 17
    maxim_max30102_write_reg(REG_MODE_CONFIG, 0x03U); // //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    maxim_max30102_write_reg(REG_SPO2_CONFIG, 0x23U); // ADC 18BIT SP02 : 50 HZ
    maxim_max30102_write_reg(REG_LED1_PA, 0x30);  // ≈ 9.6 mA led1
    maxim_max30102_write_reg(REG_LED2_PA, 0x30); // ≈ 9.6 mA led2
    maxim_max30102_write_reg(REG_PILOT_PA, 0x7fU);
}

void maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
  uint32_t un_temp;
  uint8_t uch_temp;
  uint8_t uch_i2c_data[6];

 
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
  maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);

  HAL_I2C_Mem_Read(&hi2c1, I2C_READ_ADDR, REG_FIFO_DATA, 1U, uch_i2c_data, 6U, 250U);

  *pun_ir_led = 0UL;
  *pun_red_led = 0UL;

  un_temp = (uint32_t)uch_i2c_data[0];
  un_temp <<= 16;
  *pun_red_led += un_temp;
  un_temp = (uint32_t)uch_i2c_data[1];
  un_temp <<= 8;
  *pun_red_led += un_temp;
  un_temp = (uint32_t)uch_i2c_data[2];
  *pun_red_led += un_temp;
  un_temp = (uint32_t)uch_i2c_data[3];
  un_temp <<= 16;
  *pun_ir_led += un_temp;
  un_temp = (uint32_t)uch_i2c_data[4];
  un_temp <<= 8;
  *pun_ir_led += un_temp;
  un_temp = (uint32_t)uch_i2c_data[5];
  *pun_ir_led += un_temp;
  *pun_red_led &= 0x03FFFF;  	// Mask MSB [23:18]
  *pun_ir_led &= 0x03FFFF;  	// Mask MSB [23:18]

}


void maxim_max30102_reset()
{
	maxim_max30102_write_reg(REG_MODE_CONFIG, 0x40U);
}

