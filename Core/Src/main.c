/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "stm32f1xx_hal.h"
#include "string.h"
#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"
#include "stdio.h"
#include "DS1307.h"
#include "max30102.h"
#include "hr_spo2_header.h"
#include "DHT11.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while(__HAL_TIM_GET_COUNTER(&htim2) < us);
}


static uint32_t un_min = 0x3FFFFUL; // max adc 18 bit
static uint32_t un_max = 0UL;
static uint32_t un_prev_data = 0UL; // lưu giá trị tín hiệu trước để so sánh với giá trị hiện tại
static float f_heartbeatTrace = 0.0f; // đường sóng tim
static int16_t graph_x = 6;      // Bắt đầu từ pixel 6
static int16_t lastY = 35;        // Vị trí Y giữa khung

#define MAX_HEART_BEAT_TRACE 50.0f


void DrawHeartbeatWave(uint32_t red_value)
{
    float f_temp;
    int16_t currentY;
    
    // khởi tạo
    if (un_prev_data == 0)
    {
        un_prev_data = red_value;
        un_min = red_value;
        un_max = red_value;
        return;
    }
    
    //  min and max
    if (red_value < un_min) un_min = red_value;
    if (red_value > un_max) un_max = red_value;
    
    // Tránh chia cho 0 - biên độ quá nhỏ
    if ((un_max - un_min) < 500)
    {
        return;
    }
    
    // ======= TÍNH TOÁN CHUYỂN ĐỘNG =======
    if (red_value > un_prev_data)
    {
        // Tín hiệu TĂNG → Đồ thị đi XUỐNG
        f_temp = (float)(red_value - un_prev_data);  // giá trị tăng bao nhiêu
        f_temp /= (float)(un_max - un_min); // tỉ lệ % thay đổi
        f_temp *= MAX_HEART_BEAT_TRACE; // phóng to đường line
        f_heartbeatTrace -= f_temp; 
        
        if (f_heartbeatTrace < -20.0f)
            f_heartbeatTrace = -20.0f;
    }
    else
    {
        // Tín hiệu GIẢM → Đồ thị đi LÊN
        f_temp = (float)(un_prev_data - red_value);
        f_temp /= (float)(un_max - un_min);
        f_temp *= MAX_HEART_BEAT_TRACE;
        f_heartbeatTrace += f_temp;
        
        if (f_heartbeatTrace > MAX_HEART_BEAT_TRACE + 20.0f)
            f_heartbeatTrace = MAX_HEART_BEAT_TRACE + 20.0f;
    }
    
    
    // Khung mới: Y từ 130 đến 200 (cao 70 pixel)
    // Giữa khung: Y = 165 (130 + 35)
    currentY = (int16_t)f_heartbeatTrace + 35;
    
    // Chuyển sang tọa độ màn hình thực
    int16_t screen_y_old = 165 - lastY;
    int16_t screen_y_new = 165 - currentY;
    
    // Giới hạn trong khung (131 đến 199)
    if (screen_y_old < 131) screen_y_old = 131;
    if (screen_y_old > 199) screen_y_old = 199;
    if (screen_y_new < 131) screen_y_new = 131;
    if (screen_y_new > 199) screen_y_new = 199;
    
    // VẼ ĐƯỜNG NỐI 
    ILI9341_DrawLine(graph_x, screen_y_old, graph_x + 2, screen_y_new, RED);
    
   
    lastY = currentY; // lưu vị trí y hiện tại
    graph_x += 2; // chạy sang phải 2 pixel
    
    // RESET KHI HẾT KHUNG 
    if (graph_x >= 233)  // Sắp chạm viền phải (235 - 2)
    {
        // Xóa vùng vẽ (giữ lại viền)
        ILI9341_DrawFilledRectangleCoord(6, 131, 234, 199, BLACK);
        
        // Reset vị trí X
        graph_x = 6;
        
        // Reset min/max cho chu kỳ mới
        un_min = 0x3FFFFUL;
        un_max = 0UL;
    }
    
    un_prev_data = red_value;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_Base_Start(&htim2);


  DS1307_RTC time;

  time.sec = 0;
  time.min = 40;
  time.hour = 10;
  time.date = 24;
  time.month = 11;
  time.year = 25;
  
  uint32_t ir_val,red_val;
	uint16_t sample_count = 0;
	int32_t heart_rate, spo2;
  DS1307_Write(&time);

  DS1307_Start();
  

  ILI9341_Init();
  ILI9341_SetRotation(SCREEN_VERTICAL_2);
  ILI9341_FillScreen(BLACK);
  maxim_max30102_init();
  
  ILI9341_DrawHLine(0, 35, 320, CYAN);
  
  ILI9341_DrawText("Temp:", Arial_Narrow8x12, 10, 60, YELLOW, BLACK);
  ILI9341_DrawText("humidity:", Arial_Narrow8x12, 10, 100, YELLOW, BLACK);
  ILI9341_DrawText("HR:", Arial_Narrow8x12, 115, 60, YELLOW, BLACK);
  ILI9341_DrawText("SPO2:", Arial_Narrow8x12, 115, 100, YELLOW, BLACK);
  ILI9341_DrawText("VAL_ADC:", Arial_Narrow10x13, 10, 220, YELLOW, BLACK);
  ILI9341_DrawText("UV_INDEX:", Arial_Narrow10x13, 10, 260, YELLOW, BLACK);
  
  // Vẽ khung
  ILI9341_DrawHollowRectangleCoord(5, 50, 100, 120, CYAN);
  ILI9341_DrawHollowRectangleCoord(110,50,235,120,CYAN);
  ILI9341_DrawHollowRectangleCoord(5, 130, 235, 200, CYAN);
  ILI9341_DrawHollowRectangleCoord(5,210,235,315,CYAN);
  char DHT11_TEMP[10];
  char DHT11_HUM[10];

  char DS1307[10];

  char HR[10];
  char SPO2[10];

  char VAL_ADC[10];
  char UV_INDEX[10];

  char status_uv[30];
  volatile uint16_t adc_data = 0;
  float vol = 0;
  int UV_index = 0;

  char hc05_data[50];

  HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc_data,1);

  uint32_t DHT11_RTC_update = 0;
  uint32_t max30102_update = 0;
  uint32_t UV_S12D_update = 0;
  uint32_t HC05_update =0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    if(HAL_GetTick() - DHT11_RTC_update >= 100)
    {
      DHT11_RTC_update = HAL_GetTick();
      
      DHT11_ReadData();
      HAL_Delay(5);
      DS1307_Read(&time);
      
       
      sprintf(DHT11_TEMP, "%d.%d C", DHT11_Temperature_Int, DHT11_Temperature_Dec);
      ILI9341_DrawText(DHT11_TEMP, Arial_Narrow8x12, 45, 60, GREEN, BLACK);
        
        
      sprintf(DHT11_HUM, "%d.%d %%", DHT11_Humidity_Int, DHT11_Humidity_Dec);
      ILI9341_DrawText(DHT11_HUM, Arial_Narrow8x12, 60, 100, GREEN, BLACK);
     
      sprintf(DS1307, "D:%d / M:%d / Y:%d | %d: %d: %d ",time.date,time.month,time.year,time.hour,time.min,time.sec);

      ILI9341_DrawText(DS1307, Arial_Narrow10x13, 20, 10, WHITE, BLACK);
    }

  if(HAL_GetTick() - max30102_update >= 30)  
  {
    max30102_update = HAL_GetTick();

    maxim_max30102_read_fifo(&red_val, &ir_val);
    
    if (ir_val < 10000) 
    {
        
        sample_count = 0;
        reset_buffer();
        heart_rate = 0;
        spo2 = 0;
        
        sprintf(HR, "---     ");
        ILI9341_DrawText(HR, Arial_Narrow8x12, 140, 60, RED, BLACK);
        sprintf(SPO2, "---     ");
        ILI9341_DrawText(SPO2, Arial_Narrow8x12, 160, 100, RED, BLACK);
    }
    else
    {
        
        add_sample(ir_val, red_val);
        sample_count++;
        
        if (sample_count >= 50)
        {
            calculate_heart_rate_spo2();
            heart_rate = get_heart_rate();
            spo2 = get_spo2();
            
            if (heart_rate < 0) heart_rate = 0;
            if (spo2 < 0) spo2 = 0;
            
            sample_count = 0;  // Reset về 0
        }

      
             DrawHeartbeatWave(red_val);
        
        if (heart_rate == 0 && spo2 == 0)
        {
            
            sprintf(HR, "Wait   ");
            ILI9341_DrawText(HR, Arial_Narrow8x12, 140, 60, YELLOW, BLACK);
            sprintf(SPO2, "Wait  ");
            ILI9341_DrawText(SPO2, Arial_Narrow8x12, 160, 100, YELLOW, BLACK);
        }
        else
        {
            sprintf(HR, "%3d bpm ", heart_rate);
            ILI9341_DrawText(HR, Arial_Narrow8x12, 140, 60, GREEN, BLACK);
            sprintf(SPO2, "%3d %%  ", spo2);
            ILI9341_DrawText(SPO2, Arial_Narrow8x12, 160, 100, GREEN, BLACK);
        }
    }
  }
    if(HAL_GetTick() - UV_S12D_update >= 100)
    {
        UV_S12D_update = HAL_GetTick();
        vol = (adc_data *3.3)/4095.0;
        UV_index = vol * 10.0;
        sprintf(VAL_ADC,"%d ",adc_data);
        ILI9341_DrawText(VAL_ADC, Arial_Narrow10x13, 90, 220, GREEN, BLACK); 
        sprintf(UV_INDEX,"%d ",UV_index);
        ILI9341_DrawText(UV_INDEX, Arial_Narrow10x13, 90, 260, GREEN, BLACK); 
        if(UV_index >= 6)
        {
          sprintf(status_uv,"not safe for skin");
          ILI9341_DrawText(status_uv, Arial_Narrow10x13, 130, 240, RED, BLACK); 
        }
        else
        {
          sprintf(status_uv,"safe for skin");
          ILI9341_DrawText(status_uv, Arial_Narrow10x13, 130, 240, GREEN, BLACK); 
        }
    }
    if(HAL_GetTick() - HC05_update >=1000)
    {
      HC05_update = HAL_GetTick();
      sprintf(hc05_data,"%d|%d|%d|%d|%d\n",DHT11_Temperature_Int,DHT11_Humidity_Int,heart_rate,spo2,UV_index);
      HAL_UART_Transmit(&huart1, (uint8_t*)hc05_data, strlen(hc05_data), 100);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_Pin|RST_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin RST_Pin CS_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RST_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
