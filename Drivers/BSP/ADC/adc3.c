#include "./adc3.h"
extern void delay_us(uint32_t nus);
extern void delay_ms(uint16_t nms);



extern ADC_HandleTypeDef hadc3;         /* ADC��� */

void adc3_channel_set(ADC_HandleTypeDef *adc_handle, uint32_t ch, uint32_t rank, uint32_t stime)
{
    ADC_ChannelConfTypeDef adc_ch_conf;

    adc_ch_conf.Channel = ch;                            /* Í¨µÀ */
    adc_ch_conf.Rank = rank;                             /* ÐòÁÐ */
    adc_ch_conf.SamplingTime = stime;                    /* ²ÉÑùÊ±¼ä */
    HAL_ADC_ConfigChannel(adc_handle, &adc_ch_conf);     /* Í¨µÀÅäÖÃ */
}

/**
 * @brief       »ñµÃADC×ª»»ºóµÄ½á¹û
 * @param       ch: Í¨µÀÖµ 0~17£¬È¡Öµ·¶Î§Îª£ºADC_CHANNEL_0~ADC_CHANNEL_17
 * @retval      ÎÞ
 */
uint32_t adc3_get_result(uint32_t ch)
{
    adc3_channel_set(&hadc3 , ch, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_239CYCLES_5);    /* ÉèÖÃÍ¨µÀ£¬ÐòÁÐºÍ²ÉÑùÊ±¼ä */

    HAL_ADC_Start(&hadc3);                            /* ¿ªÆôADC */
    HAL_ADC_PollForConversion(&hadc3, 10);            /* ÂÖÑ¯×ª»» */
    return (uint16_t)HAL_ADC_GetValue(&hadc3);        /* ·µ»Ø×î½üÒ»´ÎADC1¹æÔò×éµÄ×ª»»½á¹û */
    
}

/**
 * @brief       »ñÈ¡Í¨µÀchµÄ×ª»»Öµ£¬È¡times´Î,È»ºóÆ½¾ù
 * @param       ch      : Í¨µÀºÅ, 0~17
 * @param       times   : »ñÈ¡´ÎÊý
 * @retval      Í¨µÀchµÄtimes´Î×ª»»½á¹ûÆ½¾ùÖµ
 */
uint32_t adc3_get_result_average(uint32_t ch, uint8_t times)
{
    uint32_t temp_val = 0;
    uint8_t t;

    for (t = 0; t < times; t++)     /* »ñÈ¡times´ÎÊý¾Ý */
    {
        temp_val += adc3_get_result(ch);
        delay_ms(5);
    }

    return temp_val / times;        /* ·µ»ØÆ½¾ùÖµ */
}


