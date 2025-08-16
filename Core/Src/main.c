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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../BSP/lcd/lcd.h"
#include "stm32f1xx_hal.h"  // HAL库主头文件
#include "../../BSP/TOUCH/touch.h"
#include "../../BSP/NORFLASH/norflash.h"
#include "../../BSP/REMOTE/remote.h"
#include "../../BSP/24CXX/24cxx.h"
#include "../../BSP/LSENS/lsens.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"
#include "../../Crypto/whirlpool/Whirlpool.h"
#include <stdlib.h>
#include <time.h>

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
extern uint8_t g_remote_sta;

#define TOUCH_EVENT        (1 << 0)
#define UART_REVERT_EVENT  (1 << 1)

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

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
osThreadId_t remoteTaskHandle;
const osThreadAttr_t remoteTask_attributes = {
    .name = "remoteTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 256 * 4
};

osThreadId_t dogTaskHandle;
const osThreadAttr_t dogTask_attributes = {
    .name = "dogTask",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 256 * 4
};

osThreadId_t infoTaskHandle;
const osThreadAttr_t infoTask_attributes = {
    .name = "infoTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 256 * 4
};

osThreadId_t touchTaskHandle;
const osThreadAttr_t touchTask_attributes = {
    .name = "touchTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 256 * 4
};

osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
    .name = "uartTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 256 * 4
};

uint8_t tx_e3 = 0xE3;
uint8_t tx_e4 = 0xE4;
uint8_t tx_a0 = 0xA0;
volatile uint8_t tx_request_byte = 0;
volatile uint8_t tx_request_flag = 0;
volatile uint8_t uart_rx_flag = 0;

uint8_t close_red = 0;  // 改为全局变量
char strrand[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FSMC_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void WHIRLPOOL_add(const unsigned char * source, unsigned __int32 sourceBytes, WHIRLPOOL_CTX * const ctx);
void WHIRLPOOL_finalize(WHIRLPOOL_CTX* const ctx, unsigned char * result);
void WHIRLPOOL_init(WHIRLPOOL_CTX* const ctx);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void generate_random_string(char *buf, int len) {
    for (int i = 0; i < len; i++) {
        buf[i] = (rand() % (0x7E - 0x21 + 1)) + 0x21; // 可打印ASCII
    }
    buf[len] = '\0'; // 字符串结束符
}

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
void delay_init(uint16_t sysclk)
{
    //g_fac_us = sysclk;
}

void delay_us(uint32_t nus)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((__HAL_TIM_GET_COUNTER(&htim2) - start) < nus)
    {
        // 等待
    }
}

void delay_ms(uint16_t nms)
{

    delay_us((uint32_t)(nms * 1000));
}

void rtc_write_bkr(uint32_t bkrx, uint16_t data)
{
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, bkrx, data);
}

uint16_t rtc_read_bkr(uint32_t bkrx)
{
	HAL_PWR_EnableBkUpAccess();
    uint32_t temp = 0;
    temp = HAL_RTCEx_BKUPRead(&hrtc, bkrx);
    return (uint16_t)temp;
}
unsigned char hash_output[64];
int main2() {
    WHIRLPOOL_CTX ctx;
    char str1[29] = {0}; char str2[29] = {0}; char str3[29] = {0}; char str4[29] = {0}; char str5[17] = {0};

    // Initial Whirlpool CTX
    WHIRLPOOL_init(&ctx);

    // Input Process
    WHIRLPOOL_add((const unsigned char *)strrand, strlen(strrand), &ctx);
    lcd_show_string(170, 10, 240, 16, 16, strrand, RED);

    // Compute value
    WHIRLPOOL_finalize(&ctx, hash_output);
    split_hash_to_hex_strings(hash_output, str1, str2, str3, str4, str5);
    lcd_show_string(5, 30, 240, 16, 16, str1, RED);
    lcd_show_string(5, 50, 240, 16, 16, str2, RED);
    lcd_show_string(5, 70, 240, 16, 16, str3, RED);
    lcd_show_string(5, 90, 240, 16, 16, str4, RED);
    lcd_show_string(5, 110, 240, 16, 16, str5, RED);

    return 0;
}
void split_hash_to_hex_strings(const unsigned char *hash_output, char *str1, char *str2, char *str3, char *str4, char *str5) {
    for (int i = 0; i < 14; i++) {
        sprintf(&str1[i * 2], "%02x", hash_output[i]);
    }
    for (int i = 0; i < 14; i++) {
        sprintf(&str2[i * 2], "%02x", hash_output[i + 14]);
    }
    for (int i = 0; i < 14; i++) {
        sprintf(&str3[i * 2], "%02x", hash_output[i + 28]);
    }
    for (int i = 0; i < 14; i++) {
        sprintf(&str4[i * 2], "%02x", hash_output[i + 42]);
    }
    for (int i = 0; i < 8; i++) {
        sprintf(&str5[i * 2], "%02x", hash_output[i + 56]);
    }

}

//void HAL_Delay(uint32_t Delay)
//{
//     delay_ms(Delay);
//}
void dogTask(void *argument){
	for (;;){
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(500);
	}
}
void infoTask(void *argument){
    uint8_t year, month, day, hour, minute, second;
    short adcx;
	for (;;){
        Get_RTC_DateTime(&year, &month, &day, &hour, &minute, &second);
        char datetime[25];
        snprintf(datetime, sizeof(datetime),
                 "%04d-%02d-%02d %02d:%02d:%02d",
                 (int)year + 1970,
                 (int)month,
                 (int)day,
                 (int)hour,
                 (int)minute,
                 (int)second);
        lcd_show_string(10, 170, 240, 16, 16, datetime, RED);
        adcx = lsens_get_val();
        lcd_show_xnum(90, 190, adcx, 3, 16, 0, BLUE);
        osDelay(333);
	}
}
void touchTask(void *argument){
	for (;;){
        tp_dev.scan(0);

        if (tp_dev.sta & TP_PRES_DOWN)
        {
            lcd_show_string(10, 210, 200, 16, 16, "Screen Touched!", RED);
        }
        osDelay(100);
	}
}
void uartTask(void *argument){
	for (;;){
        if (tx_request_flag) {
            tx_request_flag = 0;
            HAL_UART_Transmit(&huart1, (uint8_t *)&tx_request_byte, 1, 100);
        }
        if (uart_rx_flag) {
            uart_rx_flag = 0;
            if (close_red == 0xCC) {
                HAL_GPIO_WritePin(PB5_LED_PORT, PB5_LED_PIN, GPIO_PIN_SET);
            } else if (close_red == 0xDD) {
                HAL_GPIO_WritePin(PB5_LED_PORT, PB5_LED_PIN, GPIO_PIN_RESET);
            } else if (close_red == 0xEE){
                lcd_show_string(10, 210, 200, 16, 16, "Reverted!      ", RED);
            }
            HAL_UART_Receive_IT(&huart1, &close_red, 1);
        }

        osDelay(50);
	}
}
void RemoteTask(void *argument)
{
    uint8_t key;
    int lastKey = 0;
    uint8_t index = 0;
    uint32_t lastKeyTime = 0;  // 记录最后一次按键时间
    const uint32_t displayTimeout = 1000;  // 1秒超时(ms)

    for(;;)
    {
    	uint32_t currentTime = osKernelGetTickCount();
        key = remote_scan();  // 获取按键
        if((currentTime - lastKeyTime) > displayTimeout)
        {
            lcd_show_string(10, 10, 240, 16, 16, "NO      INPUT", RED);
        }

        if(key != 0 && lastKey == 0)  // 有按键输入
        {
        	lastKeyTime = currentTime;
            const char *str = NULL;

            switch(key)
            {
            	case 0:
            		lcd_show_string(10, 10, 240, 16, 16, "NO      INPUT", RED);
            		break;
                case 22:
                	str = "1";
                    lcd_show_string(10, 10, 240, 16, 16, "Key 1 Pressed", RED);
                	break;
                case 25:
                	str = "2";
                    lcd_show_string(10, 10, 240, 16, 16, "Key 2 Pressed", RED);
                	break;
                case 13:
                	str = "3";
                    lcd_show_string(10, 10, 240, 16, 16, "Key 3 Pressed", RED);
                	break;
                case 12:
                	str = "4";
                    lcd_show_string(10, 10, 240, 16, 16, "Key 4 Pressed", RED);
                	break;
                case 24:
                	str = "5";
                    lcd_show_string(10, 10, 240, 16, 16, "Key 5 Pressed", RED);
                	break;
                case 94:
                	str = "6";
                    lcd_show_string(10, 10, 240, 16, 16, "Key 6 Pressed", RED);
                	break;
                case 8:
                	str = "7";
                    lcd_show_string(10, 10, 240, 16, 16, "Key 7 Pressed", RED);
                	break;
                case 28:
                	str = "8";
                    lcd_show_string(10, 10, 240, 16, 16, "Key 8 Pressed", RED);
                	break;
                case 90:
                	str = "9";
                    lcd_show_string(10, 10, 240, 16, 16, "Key 9 Pressed", RED);
                	break;
                case 66:
                	str = "0";
                    lcd_show_string(10, 10, 240, 16, 16, "Key 0 Pressed", RED);
                	break;

                case 64: // Enter 键
                	strrand[index] = '\0';  // 结束字符串
                	if (strrand[0] != '\0') main2();
                    index = 0;  // 清空，准备下次输入
                    break;

                default:
                	lcd_show_string(10, 10, 240, 16, 16, "NO      INPUT", RED);
                    break;
            }

            // 如果是数字键，并且未超过 12 位
            if(str != NULL && index < 8)
            {
                strcpy(&strrand[index], str);  // 追加字符
                index++;
                strrand[index] = '\0';        // 保证结尾
            }
        }
        if(key == 0)
            lastKey = 0;      // 松开，准备下一次输入
        else
            lastKey = key;    // 仍在按

        osDelay(20); // 适当延时，防抖/节省CPU
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
	SCB->VTOR =  0x08020000;
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //rtc_write_bkr(1, 17134);
  uint16_t rtc1data = rtc_read_bkr(1);
  lcd_init();
  lsens_init();
  at24cxx_init();
  norflash_init();
  remote_init();
  const uint8_t g_text_buf[] = {"It's a good sloth!"};
  #define TEXT_SIZE       sizeof(g_text_buf)
  uint8_t datatemp[TEXT_SIZE];

  const uint8_t g_text_buf2[] = {"A nice Sloth!"};
  #define TEXT_SIZE2 sizeof(g_text_buf2) /* TEXT×Ö·û´®³¤¶È */
  uint8_t datatemp2[TEXT_SIZE2];
  uint32_t flashsize = 16 * 1024 * 1024;

  srand(TIM2->CNT);
  uint8_t year, month, day, hour, minute, second;
  Get_RTC_DateTime(&year, &month, &day, &hour, &minute, &second);
  if ((int)year < 1){
	  Set_RTC_DateTime(0x37, 8, 0x0c, 0x13, 0x0d, 0x1e);
  }
  HAL_UART_Receive_IT(&huart1, &close_red, 1);
  g_point_color = BLUE;
  sprintf((char *)lcd_id, "LCD ID:%04X", lcddev.id);  /* ½«LCD ID´òÓ¡µ½lcd_idÊý×é */
  lcd_clear(BLUE);
  lcd_show_num(120, 10, rtc1data, 5, 16, YELLOW);
  lcd_show_string(10, 130, 240, 16, 16, (char *)lcd_id, RED); /* ÏÔÊ¾LCD ID */
  lcd_show_string(10, 150, 240, 16, 16, "Cunyuan Wang", RED);
  lcd_show_string(10, 190, 240, 16, 16, "LSENS VAL:", BLUE);
  HAL_IWDG_Refresh(&hiwdg);

  //lcd_show_string(10, 210, 200, 16, 16, "Start Write....", RED);
  //at24cxx_write(0, (uint8_t *)g_text_buf, TEXT_SIZE);
  HAL_IWDG_Refresh(&hiwdg);
  //generate_random_string(strrand, 8);

  //lcd_show_string(10, 210, 200, 16, 16, "24C02 Write Finished!", RED);   /* ÌáÊ¾´«ËÍÍê³É */
  HAL_IWDG_Refresh(&hiwdg);

  //lcd_show_string(10, 250, 200, 16, 16, "Start Read.... ", RED);
  at24cxx_read(0, datatemp, TEXT_SIZE);
  lcd_show_string(10, 230, 200, 16, 16, "EEPROM Data Readed Is:", RED);
  lcd_show_string(10, 250, 200, 16, 16, (char *)datatemp, RED);

  HAL_IWDG_Refresh(&hiwdg);

  //sprintf((char *)datatemp2, "%s%d", (char *)g_text_buf2, i);
  //norflash_write((uint8_t *)datatemp2, flashsize - 100, TEXT_SIZE2);      /* ´Óµ¹ÊýµÚ100¸öµØÖ·´¦¿ªÊ¼,Ð´ÈëSIZE³¤¶ÈµÄÊý¾Ý */
  norflash_read(datatemp2, flashsize - 100, TEXT_SIZE2);
  lcd_show_string(10, 270, 200, 16, 16, "Data Readed From Flash Is:", BLUE);
  lcd_show_string(10, 290, 200, 16, 16, (char *)datatemp2, BLUE);

  for (int i = 0; i < TEXT_SIZE2; i++)
  {
      HAL_UART_Transmit(&huart1, &datatemp2[i], 1, 100);
  }
  tp_dev.init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  dogTaskHandle = osThreadNew(dogTask, NULL, &dogTask_attributes);
  remoteTaskHandle = osThreadNew(RemoteTask, NULL, &remoteTask_attributes);
  infoTaskHandle = osThreadNew(infoTask, NULL, &infoTask_attributes);
  touchTaskHandle = osThreadNew(touchTask, NULL, &touchTask_attributes);
  uartTaskHandle = osThreadNew(uartTask, NULL, &uartTask_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 468;
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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 72 - 1;
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
  __HAL_RCC_TIM2_CLK_ENABLE();

  //TIM_HandleTypeDef htim2;
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;  // 1 MHz, 1 us per tick
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
	__HAL_RCC_AFIO_CLK_ENABLE();
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */


  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Turn on LED Screen back light

  /* Beep*/
  //HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_SET); // �??????
  HAL_Delay(BEEP_SHORT_BEEP_TIME);
  HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_RESET); // �??????

  HAL_GPIO_WritePin(PB5_LED_PORT, PB5_LED_PIN, GPIO_PIN_RESET); //PB5 Red LED

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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
		    uint8_t year, month, day, hour, minute, second;

		    for(;;) // FreeRTOS 任务循环
		    {
		        // LED 闪烁
		        uint32_t current_tick = HAL_GetTick();
		        if(led_mode == LED_BLINK) {
		            int blink_int = 333;
		            Get_RTC_DateTime(&year, &month, &day, &hour, &minute, &second);
		            if (second >= 31){
		                blink_int = 1000;
		            }
		            static uint32_t last_tick = 0;
		            if((current_tick - last_tick) >= blink_int) {
		                last_tick = current_tick;
		                HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
		            }
		        }
		        }
		        osDelay(50); // FreeRTOS 任务延时代替 HAL_Delay
}
  /* USER CODE END 5 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM4)
  {
      if (g_remote_sta & 0x80)      /* �ϴ������ݱ����յ��� */
      {
          g_remote_sta &= ~0X10;    /* ȡ���������Ѿ��������� */

          if ((g_remote_sta & 0X0F) == 0X00)
          {
              g_remote_sta |= 1 << 6; /* ����Ѿ����һ�ΰ����ļ�ֵ��Ϣ�ɼ� */
          }

          if ((g_remote_sta & 0X0F) < 14)
          {
              g_remote_sta++;
          }
          else
          {
              g_remote_sta &= ~(1 << 7);    /* ���������ʶ */
              g_remote_sta &= 0XF0;         /* ��ռ����� */
          }
      }
  }

  /* USER CODE END Callback 1 */
}

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
