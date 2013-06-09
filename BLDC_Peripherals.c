/*******************************************************************************
  BLDC_Peripherals.c -  peripherals for BLDC control
  Author:   Swen Wahl
  Date:     29.04.2013
  License:  Released into the public domain
            uses API of Atmel Software Framework
            http://asf.atmel.com/docs/latest/api.html
*******************************************************************************/

#include "BLDC_Peripherals.h"

/* configuration for BLDC board 1 */

BldcHardware BldcBoard1 =
{
    {   /* PWM configuration */
        PWM_SW_FRQ,
        PWM1_PIN_SETTING,
        PWM1_CH_PHU,
        PWM1_CH_PHV,
        PWM1_CH_PHW,
        PWM1_CH_PHU_BIT,
        PWM1_CH_PHV_BIT,
        PWM1_CH_PHW_BIT
    },
    {   /* ADC configuration */
        ADC1_CH_CUR_PHU,
        ADC1_CH_CUR_PHV,
        ADC1_CH_VOLT_PHU,
        ADC1_CH_VOLT_PHV,
        ADC1_CH_VOLT_PHW,
        ADC1_CH_CUR_PHU_BIT,
        ADC1_CH_CUR_PHV_BIT,
        ADC1_CH_VOLT_PHU_BIT,
        ADC1_CH_VOLT_PHV_BIT,
        ADC1_CH_VOLT_PHW_BIT,
        &ADC1_CH_CUR_PHU_RESULT,
        &ADC1_CH_CUR_PHV_RESULT,
        &ADC1_CH_VOLT_PHU_RESULT,
        &ADC1_CH_VOLT_PHV_RESULT,
        &ADC1_CH_VOLT_PHW_RESULT
    },
    {
        0
    }
};

BldcHardware BldcBoard2 =
{
    {   /* PWM configuration */
        PWM_SW_FRQ,
        PWM2_PIN_SETTING,
        PWM2_CH_PHU,
        PWM2_CH_PHV,
        PWM2_CH_PHW,
        PWM2_CH_PHU_BIT,
        PWM2_CH_PHV_BIT,
        PWM2_CH_PHW_BIT
    },
    {   /* ADC configuration */
        ADC2_CH_CUR_PHU,
        ADC2_CH_CUR_PHV,
        ADC2_CH_VOLT_PHU,
        ADC2_CH_VOLT_PHV,
        ADC2_CH_VOLT_PHW,
        ADC2_CH_CUR_PHU_BIT,
        ADC2_CH_CUR_PHV_BIT,
        ADC2_CH_VOLT_PHU_BIT,
        ADC2_CH_VOLT_PHV_BIT,
        ADC2_CH_VOLT_PHW_BIT,
        &ADC2_CH_CUR_PHU_RESULT,
        &ADC2_CH_CUR_PHV_RESULT,
        &ADC2_CH_VOLT_PHU_RESULT,
        &ADC2_CH_VOLT_PHV_RESULT,
        &ADC2_CH_VOLT_PHW_RESULT
    },
    {
        0
    }
};
