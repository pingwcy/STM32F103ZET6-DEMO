/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "fatfs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
osThreadId_t remoteTaskHandle;
const osThreadAttr_t remoteTask_attributes = {
    .name = "remoteTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 256 * 16
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

osThreadId_t saveTaskHandle;
const osThreadAttr_t saveTask_attributes = {
    .name = "saveTask",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 128 * 4
};

uint8_t tx_e3 = 0xE3;
uint8_t tx_e4 = 0xE4;
uint8_t tx_a0 = 0xA0;
volatile uint8_t tx_request_byte = 0;
volatile uint8_t tx_request_flag = 0;
volatile uint8_t uart_rx_flag = 0;
volatile uint8_t remoteflag = 0;
FATFS fs;     // 文件系统对象
FIL fil;      // 文件对象
DIR dir;      // 目录对象
FILINFO fno;  // 文件信息
FRESULT res;  // 返回结果

uint8_t close_red = 0;  // 改为全局变量
char strrand[16];
unsigned char hash_output[64];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

extern I2C_HandleTypeDef hi2c1;

extern IWDG_HandleTypeDef hiwdg;

extern RTC_HandleTypeDef hrtc;

extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;

extern SRAM_HandleTypeDef hsram1;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void split_hash_to_hex_strings(unsigned char *hash_output, char *str1, char *str2, char *str3, char *str4, char *str5) {
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

int main2() {
    WHIRLPOOL_CTX ctx;
    char str1[29] = {0}; char str2[29] = {0}; char str3[29] = {0}; char str4[29] = {0}; char str5[17] = {0};

    // Initial Whirlpool CTX
    WHIRLPOOL_init(&ctx);

    // Input Process
    WHIRLPOOL_add((const unsigned char *)strrand, strlen(strrand), &ctx);

    // Compute value
    WHIRLPOOL_finalize(&ctx, hash_output);
    split_hash_to_hex_strings(hash_output, str1, str2, str3, str4, str5);
    taskENTER_CRITICAL();
    lcd_show_string(170, 10, 240, 16, 16, strrand, RED);
    lcd_show_string(10, 30, 240, 16, 16, str1, RED);
    lcd_show_string(10, 50, 240, 16, 16, str2, RED);
    lcd_show_string(10, 70, 240, 16, 16, str3, RED);
    lcd_show_string(10, 90, 240, 16, 16, str4, RED);
    lcd_show_string(10, 110, 240, 16, 16, str5, RED);
    taskEXIT_CRITICAL();
    return 0;
}

void dogTask(void *argument){
	for (;;){
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(500);
	}
}

void saveTask(void *argument){
	for (;;){
		norflash_sync();
		osDelay(30000);
	}

}
void infoTask(void *argument){
    uint8_t year, month, day, hour, minute, second;
    uint8_t i = 0;
    uint16_t adcValue = 0;
    uint32_t cpu_freq = 0;
    float vref = 3.3f;
    float vsense, temperature;
    short adcx;
    char str[12];
    res = f_mount(&fs, "", 1);  // 挂载到逻辑驱动器 0:，立即挂载
    if (res == FR_OK) {
        lcd_show_string(10, 270, 200, 16, 16, "FATFS mount OK.", BLUE);
    } else {
        lcd_show_string(10, 270, 200, 16, 16, "FATFS mount failed", BLUE);
    }

    char file_list[256];   // 最终拼好的字符串
    UINT offset = 0;       // 当前写入位置

    file_list[0] = '\0';   // 先置空字符串

    res = f_opendir(&dir, "/");  // 打开根目录
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);  // 逐个读目录项
            if (res != FR_OK || fno.fname[0] == 0) break;  // 读完

            // 将文件名拼接到 file_list
            int n = snprintf(&file_list[offset],
                             sizeof(file_list) - offset,
                             "%s ", fno.fname);
            if (n < 0 || n >= (int)(sizeof(file_list) - offset)) {
                // 防止溢出
                break;
            }
            offset += n;
        }
        f_closedir(&dir);

        lcd_show_string(10, 290, 200, 16, 16, file_list, BLUE);
    } else {
        lcd_show_string(10, 290, 200, 16, 16, "Open root dir failed", BLUE);
    }


	for (;;){
		i++;
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
        adcx = lsens_get_val();
        if (i==10){
        	i = 0;
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            adcValue = HAL_ADC_GetValue(&hadc1);
            vsense = (adcValue * vref) / 4095.0f;
            temperature = ((1.43f - vsense) / 0.0043f) + 25.0f;
            sprintf(str, "%.2f", temperature);
        }
        cpu_freq = HAL_RCC_GetSysClockFreq();

        }
        taskENTER_CRITICAL();
        lcd_show_string(10, 170, 240, 16, 16, datetime, RED);
        lcd_show_string(150, 190, 240, 16, 16, str, BLUE);
        lcd_show_xnum(90, 190, adcx, 3, 16, 0, BLUE);
        lcd_show_xnum(200, 190, cpu_freq / 1000000, 3, 16, 0, BLUE);

        taskEXIT_CRITICAL();
        osDelay(333);
	}
}
void touchTask(void *argument){
	for (;;){
        tp_dev.scan(0);

        if (tp_dev.sta & TP_PRES_DOWN)
        {
        	taskENTER_CRITICAL();
            lcd_show_string(10, 210, 200, 16, 16, "Screen Touched!", RED);
            taskEXIT_CRITICAL();
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
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            } else if (close_red == 0xDD) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            } else if (close_red == 0xEE){
            	taskENTER_CRITICAL();
                lcd_show_string(10, 210, 200, 16, 16, "Reverted!      ", RED);
                taskEXIT_CRITICAL();
            }
            HAL_UART_Receive_IT(&huart1, &close_red, 1);
        }

        osDelay(150);
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
    	if (remoteflag == 1){
    		remoteflag = 0;
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
            taskENTER_CRITICAL();
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
            taskEXIT_CRITICAL();
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
    	}

        osDelay(150); // 适当延时，防抖/节省CPU
    }
}

/* USER CODE END Application */

