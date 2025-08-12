/**
 ****************************************************************************************************
 * @file        adc3.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.1
 * @date        2020-04-24
 * @brief       ADC3 ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F103������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20200424
 * ��һ�η���
 * V1.1 20200424
 * 1, �޸�adc3_init, ��Ӷ�ADC3_CHY_GPIO��صĳ�ʼ��
 * 2, ��ͷ�ļ�������� ADC3_CHY ��غ궨��
 ****************************************************************************************************
 */

#ifndef __ADC_H
#define __ADC_H

#include "stdlib.h"
#include "stm32f1xx.h"


/******************************************************************************************/
/* ADC������ ���� */

#define ADC3_CHY_GPIO_PORT                  GPIOA
#define ADC3_CHY_GPIO_PIN                   GPIO_PIN_1 
#define ADC3_CHY_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)  /* PA��ʱ��ʹ�� */

#define ADC_ADCX                            ADC3 
#define ADC3_CHY                            ADC_CHANNEL_1                                /* ͨ��Y,  0 <= Y <= 17 */ 
#define ADC3_CHY_CLK_ENABLE()               do{ __HAL_RCC_ADC3_CLK_ENABLE(); }while(0)   /* ADC1 ʱ��ʹ�� */


/******************************************************************************************/

void adc3_init(void);                               /* ADC3��ʼ�� */
void adc3_channel_set(ADC_HandleTypeDef *adc_handle, uint32_t ch, uint32_t rank, uint32_t stime);   /* ADC3ͨ������ */
uint32_t adc3_get_result(uint32_t ch);               /* ���ĳ��ͨ��ֵ  */
uint32_t adc3_get_result_average(uint32_t ch, uint8_t times);/* �õ�ĳ��ͨ����������������ƽ��ֵ */

#endif 















