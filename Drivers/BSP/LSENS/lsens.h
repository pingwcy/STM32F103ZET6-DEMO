/**
 ****************************************************************************************************
 * @file        lsens.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-24
 * @brief       ���������� ��������
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
 ****************************************************************************************************
 */

#ifndef __LSENS_H
#define __LSENS_H

#include "stdlib.h"
#include "stm32f1xx.h"


/******************************************************************************************/
/* ������������ӦADC3���������ź�ͨ�� ���� */

#define LSENS_ADC3_CHX_GPIO_PORT            GPIOF
#define LSENS_ADC3_CHX_GPIO_PIN             GPIO_PIN_8
#define LSENS_ADC3_CHX_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)   /* PF��ʱ��ʹ�� */


#define LSENS_ADC3_CHX                      ADC_CHANNEL_6       /* ͨ��Y,  0 <= Y <= 17 */ 

/******************************************************************************************/
 

void lsens_init(void);          /* ��ʼ������������ */
uint8_t lsens_get_val(void);    /* ��ȡ������������ֵ */
#endif 





















