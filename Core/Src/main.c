/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../BSP/lcd/lcd.h"
#include "stm32f1xx_hal.h"  // HAL库主头文件

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    LED_OFF,
    LED_ON,
    LED_BLINK
} LedMode_t;

volatile LedMode_t led_mode = LED_OFF;
uint32_t last_tick = 0;
uint32_t last_tick2 = 0;

uint32_t last_debounce_time = 0;
uint8_t last_btn_state1 = 1;
uint8_t last_btn_state2 = 1;
uint8_t last_btn_state3 = 1;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PIN GPIO_PIN_5      // PE5 (用户控制LED)
#define LED_PORT GPIOE
#define BTN_PIN1 GPIO_PIN_3     // PE3 (按键1)
#define BTN_PIN2 GPIO_PIN_4     // PE4 (按键2)
#define BTN_PORT GPIOE

#define BTN_PIN0 GPIO_PIN_0     // PA0 (按键0)
#define BTN_PORT0 GPIOA

#define PB5_LED_PIN GPIO_PIN_5  // PB5 (�??????机常亮LED)
#define PB5_LED_PORT GPIOB
#define BEEP_PIN GPIO_PIN_8
#define BEEP_PORT GPIOB
#define BEEP_SHORT_BEEP_TIME 100  // 蜂鸣时长(ms)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint8_t tx_e3 = 0xE3;
uint8_t tx_e4 = 0xE4;
uint8_t tx_a0 = 0xA0;
volatile uint8_t tx_request_byte = 0;
volatile uint8_t tx_request_flag = 0;
volatile uint8_t uart_rx_flag = 0;

uint8_t close_red = 0;  // 改为全局变量

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FSMC_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 设置日期和时�???
void Set_RTC_DateTime(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    RTC_DateTypeDef sDate = {0};
    RTC_TimeTypeDef sTime = {0};

    sDate.Year = year;
    sDate.Month = month;
    sDate.Date = day;
    sDate.WeekDay = RTC_WEEKDAY_MONDAY; // 根据实际情况设置

    sTime.Hours = hour;
    sTime.Minutes = minute;
    sTime.Seconds = second;
    //sTime.SubSeconds = 0;
    //sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    //sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        Error_Handler();
    }
}

// 获取当前日期和时�???
void Get_RTC_DateTime(uint8_t *year, uint8_t *month, uint8_t *day,
                      uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    RTC_DateTypeDef sDate = {0};
    RTC_TimeTypeDef sTime = {0};

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    *year = sDate.Year;
    *month = sDate.Month;
    *day = sDate.Date;
    *hour = sTime.Hours;
    *minute = sTime.Minutes;
    *second = sTime.Seconds;
}
// 在合适的位置添加中断处理函数

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t current_tick = HAL_GetTick();
    if((current_tick - last_debounce_time) > 50) {
        if(GPIO_Pin == BTN_PIN1) {
            tx_request_byte = 0xE3; tx_request_flag = 1;
            if(led_mode != LED_ON) { led_mode = LED_ON; HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);}
            else { led_mode = LED_OFF; HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); }
        }
        else if(GPIO_Pin == BTN_PIN2) {
            tx_request_byte = 0xE4; tx_request_flag = 1;
            led_mode = LED_BLINK;
        }
        else if(GPIO_Pin == BTN_PIN0) {
            tx_request_byte = 0xA0; tx_request_flag = 1;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        }
        last_debounce_time = current_tick;
    }
}
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

// 添加UART接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uart_rx_flag = 1;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    uint8_t lcd_id[12];

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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_FSMC_Init();
  MX_IWDG_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  lsens_init();
  HAL_UART_Receive_IT(&huart1, &close_red, 1);
  g_point_color = RED;
  sprintf((char *)lcd_id, "LCD ID:%04X", lcddev.id);  /* ½«LCD ID´òÓ¡µ½lcd_idÊý×é */
  lcd_clear(BLUE);
  lcd_show_string(10, 40, 240, 32, 32, "STM32F103ZET6", RED);
  lcd_show_string(10, 80, 240, 24, 24, "Cunyuan Wang", RED);
  lcd_show_string(10, 110, 240, 16, 16, "pingwcy@outlook.com", RED);
  lcd_show_string(10, 130, 240, 16, 16, (char *)lcd_id, RED); /* ÏÔÊ¾LCD ID */
  lcd_show_string(10, 150, 240, 16, 16, "LUT University, Lappeenranta", RED);
  lcd_show_string(10, 190, 240, 16, 16, "LSENS_VAL:", BLUE);
  int k = 0;
  short adcx;
  uint8_t year, month, day, hour, minute, second;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_IWDG_Refresh(&hiwdg);
	    uint32_t current_tick = HAL_GetTick();
	    k++;
	    if (tx_request_flag) {
	        tx_request_flag = 0;
	        // 阻塞发送很短的数据在主循环是安全的
	        HAL_UART_Transmit(&huart1, (uint8_t *)&tx_request_byte, 1, 100);
	    }
	    if (uart_rx_flag) {
	        uart_rx_flag = 0;
	        if (close_red == 0xCC) {
	            HAL_GPIO_WritePin(PB5_LED_PORT, PB5_LED_PIN, GPIO_PIN_SET);
	        } else if (close_red == 0xDD) {
	            HAL_GPIO_WritePin(PB5_LED_PORT, PB5_LED_PIN, GPIO_PIN_RESET);
	        }
	        HAL_UART_Receive_IT(&huart1, &close_red, 1);
	    }

	    // LED闪烁控制
	    if(led_mode == LED_BLINK) {
	        int blink_int = 333;

	        Get_RTC_DateTime(&year, &month, &day, &hour, &minute, &second);
	        if (second >= 31){
	            blink_int = 1000;
	        }

	        if((current_tick - last_tick) >= blink_int) {
	            last_tick = current_tick;
	            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
	        }
	    }
	    if (k==100){
	    	k = 0;
	        Get_RTC_DateTime(&year, &month, &day, &hour, &minute, &second);
	        char datetime[25]; // 格式: "YYYY-MM-DD HH:MM:SS" + null终止符

	        // 使用snprintf拼接字符串，年份加上1970
	        snprintf(datetime, sizeof(datetime),
	                 "%04d-%02d-%02d %02d:%02d:%02d",
	                 (int)year + 1970, // 年份加上1970
	                 (int)month,
	                 (int)day,
	                 (int)hour,
	                 (int)minute,
	                 (int)second);
	        lcd_show_string(10, 170, 240, 16, 16, datetime, RED);
	        adcx = lsens_get_val();
	        lcd_show_xnum(90, 190, adcx, 3, 16, 0, BLUE);

	    }
	    HAL_Delay(10); // 降低CPU占用


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x30;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_AUGUST;
  DateToUpdate.Date = 0x11;
  DateToUpdate.Year = 0x55;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
	__HAL_RCC_AFIO_CLK_ENABLE();
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
	__HAL_RCC_GPIOE_CLK_ENABLE();
HAL_Delay(1);
__HAL_RCC_GPIOB_CLK_ENABLE();
HAL_Delay(1);
//__HAL_RCC_GPIOA_CLK_ENABLE();

GPIO_InitTypeDef GPIO_InitStruct = {0};

GPIO_InitStruct.Pin = GPIO_PIN_0;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // 初始点亮

GPIO_InitStruct.Pin = BEEP_PIN;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
HAL_GPIO_Init(BEEP_PORT, &GPIO_InitStruct);

/* �??????机蜂鸣提�?????? */
//HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_SET); // �??????
HAL_Delay(BEEP_SHORT_BEEP_TIME);
HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_RESET); // �??????

GPIO_InitStruct.Pin = PB5_LED_PIN;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(PB5_LED_PORT, &GPIO_InitStruct);
HAL_GPIO_WritePin(PB5_LED_PORT, PB5_LED_PIN, GPIO_PIN_RESET); // �??????机立即点�??????
__DSB(); // 数据同步屏障，确保操作完�??????

// 配置PE5为输出（用户控制LED�??????
GPIO_InitStruct.Pin = LED_PIN;
HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

// 配置PE3/PE4为输入（按键�??????
GPIO_InitStruct.Pin = BTN_PIN1 | BTN_PIN2;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
GPIO_InitStruct.Pull = GPIO_PULLUP;  // 启用内部上拉
HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct);

GPIO_InitStruct.Pin = BTN_PIN0;
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // 启用内部下拉
HAL_GPIO_Init(BTN_PORT0, &GPIO_InitStruct);
// 设置中断优先级并使能中断
HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
HAL_NVIC_EnableIRQ(EXTI3_IRQn);  // PE3
HAL_NVIC_SetPriority(EXTI4_IRQn, 4, 0);
HAL_NVIC_EnableIRQ(EXTI4_IRQn);  // PE4
HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
HAL_NVIC_EnableIRQ(EXTI0_IRQn);  // PA0

last_tick = HAL_GetTick();
  /* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK4;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
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
