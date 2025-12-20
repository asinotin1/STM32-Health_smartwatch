
#include "hr_spo2_header.h"
#include <math.h>
#include <string.h>
#include <stdint.h>

#define BUFFER_SIZE 50
static uint32_t ir_buffer[BUFFER_SIZE];
static uint32_t red_buffer[BUFFER_SIZE];
static uint8_t buffer_index = 0;


static int32_t heart_rate = 0;
static int32_t spo2 = 0;
static uint8_t valid_heart_rate = 0;
static uint8_t valid_spo2 = 0;


 // Thêm sample mới vào buffer
 
void add_sample(uint32_t ir_val, uint32_t red_val)
{
    ir_buffer[buffer_index] = ir_val;
    red_buffer[buffer_index] = red_val;
    buffer_index++;
    
    if (buffer_index >= BUFFER_SIZE) {
        buffer_index = 0;
    }
}


  //Tìm peaks trong tín hiệu IR để tính nhịp tim
 
static void find_peaks_and_calculate_hr(void)
{
    uint8_t peak_count = 0;
    uint32_t peak_positions[20];
    uint32_t threshold;
    uint32_t max_val = 0, min_val = 0xFFFFFFFF;
    uint32_t mean_val = 0;
    int i;
    uint8_t min_peak_distance = 20;  // 200ms giữa các peak
    int last_peak_pos = -25;
    
    
    for (i = 0; i < BUFFER_SIZE; i++) {
        if (ir_buffer[i] > max_val) max_val = ir_buffer[i];
        if (ir_buffer[i] < min_val) min_val = ir_buffer[i];
        mean_val += ir_buffer[i];
    }
    mean_val /= BUFFER_SIZE;
    
    /* Kiểm tra tín hiệu đủ mạnh */
    if ((max_val - min_val) < 200) {
        valid_heart_rate = 0;
        heart_rate = 0;
        return;
    }
    
    /* Ngưỡng phát hiện peak: 30% */
    threshold = mean_val + ((max_val - mean_val) * 30) / 100;
    
    /* tìm peaks */
    for (i = 2; i < BUFFER_SIZE - 2; i++) {
        if (ir_buffer[i] > threshold &&
            ir_buffer[i] > ir_buffer[i-1] &&
            ir_buffer[i] >= ir_buffer[i+1]) {
            
            if ((i - last_peak_pos) >= min_peak_distance) {
                if (peak_count < 20) {
                    peak_positions[peak_count] = i;
                    last_peak_pos = i;
                    peak_count++;
                }
            }
        }
    }
    
    /* Tính heart rate từ  2 peaks */
    if (peak_count >= 2) {
        uint32_t total_distance = 0;
        float avg_distance;
        
        for (i = 1; i < peak_count; i++) {
            total_distance += (peak_positions[i] - peak_positions[i-1]);
        }
        
        avg_distance = (float)total_distance / (peak_count - 1);
        
        heart_rate = (int32_t)(2000.0f / avg_distance);
        
        /* Kiểm tra khoảng hợp lý: 40-180 BPM */
        if (heart_rate >= 40 && heart_rate <= 180) {
            valid_heart_rate = 1;
        } else {
            valid_heart_rate = 0;
            heart_rate = 0;
        }
    } else {
        valid_heart_rate = 0;
        heart_rate = 0;
    }
}


 // Tính SpO2 theo công thức R = (AC_red/DC_red) / (AC_ir/DC_ir)
 
static void calculate_spo2(void)
{
    uint32_t ir_max = 0, ir_min = 0xFFFFFFFF;
    uint32_t red_max = 0, red_min = 0xFFFFFFFF;
    uint32_t ir_ac, ir_dc, red_ac, red_dc;
    float R;
    int i;
    
    // tim max và min của ir và red
    for (i = 0; i < BUFFER_SIZE; i++) {
        if (ir_buffer[i] > ir_max) ir_max = ir_buffer[i];
        if (ir_buffer[i] < ir_min) ir_min = ir_buffer[i];
        if (red_buffer[i] > red_max) red_max = red_buffer[i];
        if (red_buffer[i] < red_min) red_min = red_buffer[i];
    }
    
    ir_ac = ir_max - ir_min;
    ir_dc = (ir_max + ir_min) / 2;
    red_ac = red_max - red_min;
    red_dc = (red_max + red_min) / 2;
    
    // Tránh chia cho 0
    if (ir_dc == 0 || ir_ac == 0) {
        valid_spo2 = 0;
        return;
    }
    
    // Tính R ratio
    R = ((float)red_ac / (float)red_dc) / ((float)ir_ac / (float)ir_dc);
    
    spo2 = (int32_t)(107.0f - 14.0f * R);
    
    // Kiểm tra SpO2 hợp lệ (70-100%)
    if (spo2 >= 70 && spo2 <= 100) {
        valid_spo2 = 1;
    } else {
        valid_spo2 = 0;
    }
}


void calculate_heart_rate_spo2(void)
{
    find_peaks_and_calculate_hr();
    calculate_spo2();
}


int32_t get_heart_rate(void)
{
    if (valid_heart_rate) {
        return heart_rate;
    }
    return -1;
}


int32_t get_spo2(void)
{
    if (valid_spo2) {
        return spo2;
    }
    return -1;
}


uint8_t is_heart_rate_valid(void)
{
    return valid_heart_rate;
}


uint8_t is_spo2_valid(void)
{
    return valid_spo2;
}


void reset_buffer(void)
{
    buffer_index = 0;
    valid_heart_rate = 0;
    valid_spo2 = 0;
    heart_rate = 0;
    spo2 = 0;
    memset(ir_buffer, 0, sizeof(ir_buffer));
    memset(red_buffer, 0, sizeof(red_buffer));
}

