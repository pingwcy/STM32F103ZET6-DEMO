/**
 ****************************************************************************************************
 * @file        remote.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-25
 * @brief       ����ң�ؽ��� ��������
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
 * V1.0 20200425
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __REMOTE_H
#define __REMOTE_H

#include "stdlib.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"


/******************************************************************************************/
/* �����������ż���ʱ�� ���� */

#define REMOTE_IN_GPIO_PORT                     GPIOB
#define REMOTE_IN_GPIO_PIN                      GPIO_PIN_9
#define REMOTE_IN_GPIO_CLK_ENABLE()             do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0) /* PB��ʱ��ʹ�� */


#define REMOTE_IN_TIMX                          TIM4                       
#define REMOTE_IN_TIMX_IRQn                     TIM4_IRQn
#define REMOTE_IN_TIMX_IRQHandler               TIM4_IRQHandler
#define REMOTE_IN_TIMX_CHY                      TIM_CHANNEL_4                               /* ͨ��Y,  1<= Y <=2*/ 
#define REMOTE_IN_TIMX_CCRY                     REMOTE_IN_TIMX->CCR4
#define REMOTE_IN_TIMX_CHY_CLK_ENABLE()         do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)  /* TIMX ʱ��ʹ�� */

/******************************************************************************************/


#define RDATA           HAL_GPIO_ReadPin(REMOTE_IN_GPIO_PORT, REMOTE_IN_GPIO_PIN)   /* ������������� */


/* ����ң��ʶ����(ID),ÿ��ң�����ĸ�ֵ��������һ��,��Ҳ��һ����.
 * ����ѡ�õ�ң����ʶ����Ϊ0
*/
#define REMOTE_ID       0

extern uint8_t g_remote_cnt;    /* �������µĴ��� */
    
void remote_init(void);         /* ���⴫��������ͷ���ų�ʼ�� */
uint8_t remote_scan(void);

#endif















