/**
 ****************************************************************************************************
 * @file        adc3.c
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

#include "./adc3.h"
#include "../../SYSTEM/delay/delay.h"


ADC_HandleTypeDef g_adc3_handle;         /* ADC��� */

/**
 * @brief       ADC3��ʼ������
 *   @note      ������֧��ADC1/ADC2����ͨ��, ���ǲ�֧��ADC3
 *              ����ʹ��12λ����, ADC����ʱ��=12M, ת��ʱ��Ϊ: �������� + 12.5��ADC����
 *              ��������������: 239.5, ��ת��ʱ�� = 252 ��ADC���� = 21us
 * @param       ��
 * @retval      ��
 */
void adc3_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    RCC_PeriphCLKInitTypeDef adc_clk_init = {0};

    ADC3_CHY_GPIO_CLK_ENABLE();                                /* IO��ʱ��ʹ�� */
    ADC3_CHY_CLK_ENABLE();                                     /* ADCʱ��ʹ�� */

    adc_clk_init.PeriphClockSelection = RCC_PERIPHCLK_ADC;     /* ADC����ʱ�� */
    adc_clk_init.AdcClockSelection = RCC_ADCPCLK2_DIV6;        /* ��Ƶ����6ʱ��Ϊ72M/6=12MHz */
    HAL_RCCEx_PeriphCLKConfig(&adc_clk_init);                  /* ����ADCʱ�� */

    /* ����AD�ɼ�ͨ����ӦIO���Ź���ģʽ */
    gpio_init_struct.Pin = ADC3_CHY_GPIO_PIN;                  /* ADCͨ����Ӧ��IO���� */
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;                  /* ģ�� */
    HAL_GPIO_Init(ADC3_CHY_GPIO_PORT, &gpio_init_struct);

    g_adc3_handle.Instance = ADC_ADCX;                         /* ѡ���ĸ�ADC */
    g_adc3_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;        /* ���ݶ��뷽ʽ���Ҷ��� */
    g_adc3_handle.Init.ScanConvMode = ADC_SCAN_DISABLE;        /* ��ɨ��ģʽ�����õ�һ��ͨ�� */
    g_adc3_handle.Init.ContinuousConvMode = DISABLE;           /* �ر�����ת��ģʽ */
    g_adc3_handle.Init.NbrOfConversion = 1;                    /* 1��ת���ڹ��������� Ҳ����ֻת����������1 */
    g_adc3_handle.Init.DiscontinuousConvMode = DISABLE;        /* ��ֹ����ͨ������ģʽ */
    g_adc3_handle.Init.NbrOfDiscConversion = 0;                /* ���ü��ģʽ�Ĺ���ͨ����������ֹ����ͨ������ģʽ�󣬴˲������� */
    g_adc3_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;  /* ����ת����ʽ��������� */
    HAL_ADC_Init(&g_adc3_handle);                              /* ��ʼ�� */

    HAL_ADCEx_Calibration_Start(&g_adc3_handle);               /* У׼ADC */
}

/**
 * @brief       ����ADCͨ������ʱ��
 * @param       adcx : adc���ָ��,ADC_HandleTypeDef
 * @param       ch   : ͨ����, ADC_CHANNEL_0~ADC_CHANNEL_17
 * @param       stime: ����ʱ��  0~7, ��Ӧ��ϵΪ:
 *   @arg       ADC_SAMPLETIME_1CYCLE_5, 1.5��ADCʱ������        ADC_SAMPLETIME_7CYCLES_5, 7.5��ADCʱ������
 *   @arg       ADC_SAMPLETIME_13CYCLES_5, 13.5��ADCʱ������     ADC_SAMPLETIME_28CYCLES_5, 28.5��ADCʱ������
 *   @arg       ADC_SAMPLETIME_41CYCLES_5, 41.5��ADCʱ������     ADC_SAMPLETIME_55CYCLES_5, 55.5��ADCʱ������
 *   @arg       ADC_SAMPLETIME_71CYCLES_5, 71.5��ADCʱ������     ADC_SAMPLETIME_239CYCLES_5, 239.5��ADCʱ������
 * @param       rank: ��ͨ���ɼ�ʱ��Ҫ���õĲɼ����,
                �����㶨��channle1��rank=1��channle2 ��rank=2��
                ��ô��Ӧ����DMA����ռ�ı�������AdcDMA[0] ��i��channle1��ת�������AdcDMA[1]����ͨ��2��ת������� 
                ��ͨ��DMA����Ϊ ADC_REGULAR_RANK_1
 *   @arg       ���1~16��ADC_REGULAR_RANK_1~ADC_REGULAR_RANK_16
 * @retval      ��
 */
void adc3_channel_set(ADC_HandleTypeDef *adc_handle, uint32_t ch, uint32_t rank, uint32_t stime)
{
    ADC_ChannelConfTypeDef adc_ch_conf;
    
    adc_ch_conf.Channel = ch;                            /* ͨ�� */
    adc_ch_conf.Rank = rank;                             /* ���� */
    adc_ch_conf.SamplingTime = stime;                    /* ����ʱ�� */
    HAL_ADC_ConfigChannel(adc_handle, &adc_ch_conf);     /* ͨ������ */
}

/**
 * @brief       ���ADCת����Ľ��
 * @param       ch: ͨ��ֵ 0~17��ȡֵ��ΧΪ��ADC_CHANNEL_0~ADC_CHANNEL_17
 * @retval      ��
 */
uint32_t adc3_get_result(uint32_t ch)
{
    adc3_channel_set(&g_adc3_handle , ch, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);    /* ����ͨ�������кͲ���ʱ�� */
    
    HAL_ADC_Start(&g_adc3_handle);                            /* ����ADC */
    HAL_ADC_PollForConversion(&g_adc3_handle, 10);            /* ��ѯת�� */
    return (uint16_t)HAL_ADC_GetValue(&g_adc3_handle);        /* �������һ��ADC1�������ת����� */
    
}

/**
 * @brief       ��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ��
 * @param       ch      : ͨ����, 0~17
 * @param       times   : ��ȡ����
 * @retval      ͨ��ch��times��ת�����ƽ��ֵ
 */
uint32_t adc3_get_result_average(uint32_t ch, uint8_t times)
{
    uint32_t temp_val = 0;
    uint8_t t;

    for (t = 0; t < times; t++)     /* ��ȡtimes������ */
    {
        temp_val += adc3_get_result(ch);
        delay_ms(5);
    }

    return temp_val / times;        /* ����ƽ��ֵ */
}






