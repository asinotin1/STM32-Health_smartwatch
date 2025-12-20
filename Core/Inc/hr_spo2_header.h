
#ifndef HEART_RATE_SPO2_H_
#define HEART_RATE_SPO2_H_

#include <stdint.h>


void add_sample(uint32_t ir_val, uint32_t red_val);

void calculate_heart_rate_spo2(void);

int32_t get_heart_rate(void);


int32_t get_spo2(void);


uint8_t is_heart_rate_valid(void);


uint8_t is_spo2_valid(void);


void reset_buffer(void);

#endif 