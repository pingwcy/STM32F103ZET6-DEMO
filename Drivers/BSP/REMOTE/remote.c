/**
 ****************************************************************************************************
 * @file        remote.c
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

#include "../../BSP/REMOTE/remote.h"
extern void delay_us(uint32_t nus);
extern void delay_ms(uint16_t nms);


extern TIM_HandleTypeDef htim4;
uint32_t g_remote_data = 0; /* ������յ������� */
uint8_t  g_remote_cnt = 0;
uint8_t g_remote_sta;

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == REMOTE_IN_TIMX)
    {
        GPIO_InitTypeDef gpio_init_struct;

        REMOTE_IN_GPIO_CLK_ENABLE();            /* �����������GPIOʱ��ʹ�� */
        REMOTE_IN_TIMX_CHY_CLK_ENABLE();        /* ��ʱ��ʱ��ʹ�� */
        __HAL_AFIO_REMAP_TIM4_DISABLE();        /* �����õ���PB9/TIM4_CH4���ο�AFIO_MAPR�Ĵ��������� */

        gpio_init_struct.Pin = REMOTE_IN_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;             /* �������� */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* ���� */
        HAL_GPIO_Init(REMOTE_IN_GPIO_PORT, &gpio_init_struct);  /* ��ʼ����ʱ��ͨ������ */

        HAL_NVIC_SetPriority(REMOTE_IN_TIMX_IRQn, 1, 3);        /* �����ж����ȼ�����ռ���ȼ�1�������ȼ�3 */
        HAL_NVIC_EnableIRQ(REMOTE_IN_TIMX_IRQn);                /* ����ITM4�ж� */
    }

}
uint8_t remote_scan(void)
{
    uint8_t sta = 0;
    uint8_t t1, t2;

    if (g_remote_sta & (1 << 6))    /* �õ�һ��������������Ϣ�� */
    {
        t1 = g_remote_data;                 /* �õ���ַ�� */
        t2 = (g_remote_data >> 8) & 0xff;   /* �õ���ַ���� */

        if ((t1 == (uint8_t)~t2) || t1 == REMOTE_ID)    /* ����ң��ʶ����(ID)����ַ */
        {
            t1 = (g_remote_data >> 16) & 0xff;
            t2 = (g_remote_data >> 24) & 0xff;

            if (t1 == (uint8_t)~t2)
            {
                sta = t1;           /* ��ֵ��ȷ */
            }
        }

        if ((sta == 0) || ((g_remote_sta & 0X80) == 0)) /* �������ݴ���/ң���Ѿ�û�а����� */
        {
            g_remote_sta &= ~(1 << 6);  /* ������յ���Ч������ʶ */
            g_remote_cnt = 0;           /* ����������������� */
        }
    }

    return sta;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == REMOTE_IN_TIMX)
    {
        uint16_t dval;  /* �½���ʱ��������ֵ */

        if (RDATA)      /* �����ز��� */
        {
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim4,REMOTE_IN_TIMX_CHY,TIM_INPUTCHANNELPOLARITY_FALLING);//CC4P=1 ����Ϊ�½��ز���
            __HAL_TIM_SET_COUNTER(&htim4, 0);  /* ��ն�ʱ��ֵ */
            g_remote_sta |= 0X10;                      /* ����������Ѿ������� */
        }
        else           /* �½��ز��� */
        {
            dval=HAL_TIM_ReadCapturedValue(&htim4, REMOTE_IN_TIMX_CHY);                /* ��ȡCCR4Ҳ������CC4IF��־λ */
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, REMOTE_IN_TIMX_CHY, TIM_INPUTCHANNELPOLARITY_RISING);/* ����TIM4ͨ��4�����ز��� */

            if (g_remote_sta & 0X10)        /* ���һ�θߵ�ƽ���� */
            {
                if (g_remote_sta & 0X80)    /* ���յ��������� */
                {

                    if (dval > 300 && dval < 800)           /* 560Ϊ��׼ֵ,560us */
                    {
                        g_remote_data >>= 1;                /* ����һλ */
                        g_remote_data &= ~(0x80000000);     /* ���յ�0 */
                    }
                    else if (dval > 1400 && dval < 1800)    /* 1680Ϊ��׼ֵ,1680us */
                    {
                        g_remote_data >>= 1;                /* ����һλ */
                        g_remote_data |= 0x80000000;        /* ���յ�1 */
                    }
                    else if (dval > 2000 && dval < 3000)    /* �õ�������ֵ���ӵ���Ϣ 2500Ϊ��׼ֵ2.5ms */
                    {
                        g_remote_cnt++;         /* ������������1�� */
                        g_remote_sta &= 0XF0;   /* ��ռ�ʱ�� */
                    }
                }
                else if (dval > 4200 && dval < 4700)    /* 4500Ϊ��׼ֵ4.5ms */
                {
                    g_remote_sta |= 1 << 7; /* ��ǳɹ����յ��������� */
                    g_remote_cnt = 0;       /* ����������������� */
                }
            }

            g_remote_sta&=~(1<<4);
        }
    }
}

extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);












