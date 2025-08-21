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
    .stack_size = 256 * 8
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
    .stack_size = 256 * 8
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


// 监控任务自己的句柄
osThreadId_t monitorTaskHandle;
const osThreadAttr_t monitorTask_attributes = {
    .name = "monitorTask",
    .priority = (osPriority_t) osPriorityNormal, // 低优先级，不影响关键任务
    .stack_size = 256 * 4 // 512字节，足够它运行
};

osThreadId_t idleTaskHandle;
const osThreadAttr_t idleTask_attributes = {
	    .name = "IdleSimTask",
	    .priority = (osPriority_t) osPriorityLow, // 低优先级，不影响关键任务
	    .stack_size = 128 * 4 // 512字节，足够它运行
	};;

uint8_t tx_e3 = 0xE3;
uint8_t tx_e4 = 0xE4;
uint8_t tx_a0 = 0xA0;

volatile uint8_t tx_request_byte = 0;


osMessageQueueId_t uartQueue;
osSemaphoreId_t remoteSemaphore;
osSemaphoreId_t touchSem;


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

extern osThreadId_t defaultTaskHandle;
extern const osThreadAttr_t defaultTask_attributes;


uart_msg_t msg;
volatile uint32_t idleTicks = 0;     // 空闲钩子计数
volatile uint32_t cpuUsage = 0;      // CPU 使用率百分比
const uint32_t idleTicksMaxPer10s = 1000000; // 10秒空闲计数最大值（需实验调整）

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) __attribute__((noinline));
void vApplicationIdleHook(void)
{
	idleTicks++;
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    (void) xTask; // 消除未使用变量的警告
    taskENTER_CRITICAL();
    lcd_show_string(130, 10, 240, 16, 16, "==STACK OVERFLOW DETECTED!!", RED);
    taskEXIT_CRITICAL();

   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void IdleSimTask(void *argument)
{
	uint32_t i = 1;
	TickType_t now_ticks = xTaskGetTickCount();
	uint32_t start_s = now_ticks * portTICK_PERIOD_MS /1000;
	uint32_t end_s;
    for (;;)
    {
    	TickType_t end_ticks = xTaskGetTickCount();
    	uint32_t end_s = end_ticks * portTICK_PERIOD_MS /1000;

    	uint32_t errsnu = end_s - start_s;

    	lcd_show_xnum(190, 170, i * 100U / errsnu, 3, 16, 0, BLUE);
    	i++;
        osDelay(1000);         // 延时1ms，避免完全占CPU
    }
}

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
    //lcd_show_string(10, 50, 240, 16, 16, str2, RED);
    //lcd_show_string(10, 70, 240, 16, 16, str3, RED);
    //lcd_show_string(10, 90, 240, 16, 16, str4, RED);
    //lcd_show_string(10, 110, 240, 16, 16, str5, RED);
    taskEXIT_CRITICAL();
    return 0;
}

void monitorTask(void *argument)
{
    char cLineBuffer[4][32]; // 每行显示缓冲区

    for (;;)
    {
        // 获取每个任务的栈剩余空间（单位：word）
        UBaseType_t remoteStack = uxTaskGetStackHighWaterMark(remoteTaskHandle);
        UBaseType_t dogStack = uxTaskGetStackHighWaterMark(dogTaskHandle);
        UBaseType_t infoStack = uxTaskGetStackHighWaterMark(infoTaskHandle);
        UBaseType_t touchStack = uxTaskGetStackHighWaterMark(touchTaskHandle);
        UBaseType_t uartStack = uxTaskGetStackHighWaterMark(uartTaskHandle);
        UBaseType_t saveStack = uxTaskGetStackHighWaterMark(saveTaskHandle);
        UBaseType_t monitorStack = uxTaskGetStackHighWaterMark(monitorTaskHandle);
        UBaseType_t defaultStack = uxTaskGetStackHighWaterMark(defaultTaskHandle);

        // 栈总大小（单位：word）
        const UBaseType_t remoteTotal = remoteTask_attributes.stack_size / sizeof(StackType_t);
        const UBaseType_t dogTotal = dogTask_attributes.stack_size / sizeof(StackType_t);
        const UBaseType_t infoTotal = infoTask_attributes.stack_size / sizeof(StackType_t);
        const UBaseType_t touchTotal = touchTask_attributes.stack_size / sizeof(StackType_t);
        const UBaseType_t uartTotal = uartTask_attributes.stack_size / sizeof(StackType_t);
        const UBaseType_t saveTotal = saveTask_attributes.stack_size / sizeof(StackType_t);
        const UBaseType_t monitorTotal = monitorTask_attributes.stack_size / sizeof(StackType_t);
        const UBaseType_t defaultTotal = defaultTask_attributes.stack_size / sizeof(StackType_t);

        // 使用量 = 总栈 - 剩余栈
        snprintf(cLineBuffer[0], sizeof(cLineBuffer[0]), "rm:%02d/%02d  dg:%02d/%02d",
                 remoteTotal - remoteStack, remoteTotal,
                 dogTotal - dogStack, dogTotal);

        snprintf(cLineBuffer[1], sizeof(cLineBuffer[1]), "if:%02d/%02d  tc:%02d/%02d",
                 infoTotal - infoStack, infoTotal,
                 touchTotal - touchStack, touchTotal);

        snprintf(cLineBuffer[2], sizeof(cLineBuffer[2]), "ua:%02d/%02d  sv:%02d/%02d",
                 uartTotal - uartStack, uartTotal,
                 saveTotal - saveStack, saveTotal);

        snprintf(cLineBuffer[3], sizeof(cLineBuffer[3]), "mn:%02d/%02d  df:%02d/%02d",
                 monitorTotal - monitorStack, monitorTotal,
				 defaultTotal - defaultStack, defaultTotal);


        // LCD显示
    	taskENTER_CRITICAL();
        lcd_show_string(10, 50, 240, 16, 16, cLineBuffer[0], RED);
        lcd_show_string(10, 70, 240, 16, 16, cLineBuffer[1], RED);
        lcd_show_string(10, 90, 240, 16, 16, cLineBuffer[2], RED);
        lcd_show_string(10, 110, 240, 16, 16, cLineBuffer[3], RED);
        taskEXIT_CRITICAL();
        display_ram_info();


        osDelay(5000); // 5秒更新一次
    }
}
void dogTask(void *argument){
	for (;;){
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(1000);
	}
}

void saveTask(void *argument){
	for (;;){
		norflash_sync();
		osDelay(60000);
	}

}
void infoTask(void *argument)
{
    uint8_t year, month, day, hour, minute, second;
    uint16_t adcValue = 0;
    uint32_t cpu_freq = 0;
    float vref = 3.3f;
    float vsense, temperature;
    short adcx;
    char str[12] = "0.00";
    char datetime[25] = {0};

    // 初始化显示
    lcd_show_string(10, 170, 240, 16, 16, "----/--/-- --:--:--", RED);
    lcd_show_string(150, 190, 240, 16, 16, str, BLUE);
    lcd_show_xnum(90, 190, 0, 3, 16, 0, BLUE);
    lcd_show_xnum(200, 190, 0, 3, 16, 0, BLUE);

    TickType_t lastRtcUpdate = 0;
    TickType_t lastTempUpdate = 0;
    TickType_t lastCpuUpdate = 0;
    uint32_t lastIdleTicks = 0;

    for (;;)
    {
        TickType_t currentTime = osKernelGetTickCount();

        // 1. 实时更新光敏传感器（每次循环）
        adcx = lsens_get_val();
        lcd_show_xnum(90, 190, adcx, 3, 16, 0, BLUE);

        // 2. 每秒更新RTC时间
        if (currentTime - lastRtcUpdate >= 1000)
        {
            lastRtcUpdate = currentTime;
            Get_RTC_DateTime(&year, &month, &day, &hour, &minute, &second);
            snprintf(datetime, sizeof(datetime), "%04d-%02d-%02d %02d:%02d:%02d",
                    (int)year + 1970, (int)month, (int)day, (int)hour, (int)minute, (int)second);
            lcd_show_string(10, 170, 240, 16, 16, datetime, RED);
        }

        // 3. 每10秒更新温度
        if (currentTime - lastTempUpdate >= 10000)
        {
            lastTempUpdate = currentTime;
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
                adcValue = HAL_ADC_GetValue(&hadc1);
                vsense = (adcValue * vref) / 4095.0f;
                temperature = ((1.43f - vsense) / 0.0043f) + 25.0f;
                snprintf(str, sizeof(str), "%.2f", temperature);
                lcd_show_string(150, 190, 240, 16, 16, str, BLUE);
            }
        }

        // 4. 每20秒更新CPU频率
        if (currentTime - lastCpuUpdate >= 20000)
        {
        	lastCpuUpdate = currentTime;
        	cpu_freq = HAL_RCC_GetSysClockFreq();
        	lcd_show_xnum(200, 190, cpu_freq / 1000000, 3, 16, 0, BLUE);

        }

        osDelay(500);
    }
}

void touchTask(void *argument)
{
    for(;;)
    {
        if(osSemaphoreAcquire(touchSem, osWaitForever) == osOK)
        {
            tp_dev.scan(0);

            if(tp_dev.sta & TP_PRES_DOWN)
            {
                tp_draw_big_point(tp_dev.x[0], tp_dev.y[0], RED);

                taskENTER_CRITICAL();
                lcd_show_string(10, 210, 200, 16, 16, "Screen Touched!", RED);
                taskEXIT_CRITICAL();

                if(tp_dev.x[0] >10 && tp_dev.x[0] < 200 && tp_dev.y[0] > 200 && tp_dev.y[0] < 230)
                {
                    lcd_show_string(10, 210, 200, 16, 16, "Remove Status!!", RED);
                }
            }
        }
    }
}
void uartTask(void *argument)
{
    // 启动第一次接收
    HAL_UART_Receive_IT(&huart1, &close_red, 1);

    uart_msg_t msg;

    for (;;)
    {
        // 等待消息
        osMessageQueueGet(uartQueue, &msg, NULL, osWaitForever);

        if (msg.type == 0) // RX数据
        {
            if (msg.data == 0xCC) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            } else if (msg.data == 0xDD) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            } else if (msg.data == 0xEE) {
                lcd_show_string(10, 210, 200, 16, 16, "Reverted!      ", RED);
            }
        }
        else if (msg.type == 1) // TX请求
        {
            HAL_UART_Transmit(&huart1, &msg.data, 1, 100);
        }
    }
}
void RemoteTask(void *argument)
{
    uint8_t key, lastKey = 0;
    uint8_t index = 0;
    uint8_t keyStableCount = 0;
    const uint8_t DEBOUNCE_THRESHOLD = 2; // 连续检测次数阈值

    for(;;)
    {
        lcd_show_string(10, 10, 240, 16, 16, "NO      INPUT", RED);

        osSemaphoreAcquire(remoteSemaphore, osWaitForever);
        key = remote_scan();

        if(key == lastKey)
        {
            if(key != 0)
                keyStableCount++;
        }
        else
        {
            keyStableCount = 0; // 按键变化，重置计数
        }

        if(keyStableCount >= DEBOUNCE_THRESHOLD)
        {
            // 按键稳定，处理一次
            const char *str = NULL;
            taskENTER_CRITICAL();
            switch(key)
            {
                case 22: str = "1"; lcd_show_string(10, 10, 240, 16, 16, "Key 1 Pressed", RED); break;
                case 25: str = "2"; lcd_show_string(10, 10, 240, 16, 16, "Key 2 Pressed", RED); break;
                case 13: str = "3"; lcd_show_string(10, 10, 240, 16, 16, "Key 3 Pressed", RED); break;
                case 12: str = "4"; lcd_show_string(10, 10, 240, 16, 16, "Key 4 Pressed", RED); break;
                case 24: str = "5"; lcd_show_string(10, 10, 240, 16, 16, "Key 5 Pressed", RED); break;
                case 94: str = "6"; lcd_show_string(10, 10, 240, 16, 16, "Key 6 Pressed", RED); break;
                case 8:  str = "7"; lcd_show_string(10, 10, 240, 16, 16, "Key 7 Pressed", RED); break;
                case 28: str = "8"; lcd_show_string(10, 10, 240, 16, 16, "Key 8 Pressed", RED); break;
                case 90: str = "9"; lcd_show_string(10, 10, 240, 16, 16, "Key 9 Pressed", RED); break;
                case 66: str = "0"; lcd_show_string(10, 10, 240, 16, 16, "Key 0 Pressed", RED); break;
                case 64: // Enter
                    strrand[index] = '\0';
                    if (strrand[0] != '\0') main2();
                    index = 0;
                    break;
                default:
                    lcd_show_string(10, 10, 240, 16, 16, "NO      INPUT", RED);
                    break;
            }
            taskEXIT_CRITICAL();

            // 如果是数字键，并且未超过 8 位
            if(str != NULL && index < 8)
            {
                strcpy(&strrand[index], str);
                index++;
                strrand[index] = '\0';
            }

            keyStableCount = 0; // 只触发一次
        }

        lastKey = key;  // 更新上次按键状态
        osDelay(500);    // 小延时，保证CPU不空转
    }
}
/* USER CODE END Application */

