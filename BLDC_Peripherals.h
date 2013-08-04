/*******************************************************************************
  BLDC_Peripherals.h - header for definitons of peripherals for BLDC control
  Author:   Swen Wahl
  Date:     29.04.2013
  License:  Released into the public domain
            uses API of Atmel Software Framework
            http://asf.atmel.com/docs/latest/api.html
*******************************************************************************/
#ifndef BLDC_PERIPHERALS_H
#define BLDC_PERIPHERALS_H

#include "Arduino.h"

#define POWER_SHIELD_V2

/*******************************************************************************
            PWM related definitions
*******************************************************************************/
#define PWM_SW_FRQ      30000            /* 30kHz switching frequency */
#define PWM_PERIOD      (1 / PWM_SW_FRQ) /* PWM period */

/* PWM 1 specific */
#define PWM1_CH_PHU     0
#define PWM1_CH_PHV     1
#define PWM1_CH_PHW     2

#define PWM1_CH_PHU_BIT  (0x1U<<PWM1_CH_PHU)
#define PWM1_CH_PHV_BIT  (0x1U<<PWM1_CH_PHV)
#define PWM1_CH_PHW_BIT  (0x1U<<PWM1_CH_PHW)

#define PWM1_PORT        PIOC
#define PWM1_PIN_UH      2   /* Port C */
#define PWM1_PIN_UL      3   /* Port C */
#define PWM1_PIN_VH      4   /* Port C */
#define PWM1_PIN_VL      5   /* Port C */
#define PWM1_PIN_WH      6   /* Port C */
#define PWM1_PIN_WL      7   /* Port C */
#define PWM1_PIN_SETTING ((0x1U<<PWM1_PIN_UH) | \
                          (0x1U<<PWM1_PIN_UL) | \
                          (0x1U<<PWM1_PIN_VH) | \
                          (0x1U<<PWM1_PIN_VL) | \
                          (0x1U<<PWM1_PIN_WH) | \
                          (0x1U<<PWM1_PIN_WL))
/* PWM 2 specific */
#define PWM2_CH_PHU     0
#define PWM2_CH_PHV     1
#define PWM2_CH_PHW     2

#define PWM2_CH_PHU_BIT  (0x1U<<PWM2_CH_PHU)
#define PWM2_CH_PHV_BIT  (0x1U<<PWM2_CH_PHV)
#define PWM2_CH_PHW_BIT  (0x1U<<PWM2_CH_PHW)

#define PWM2_PORT        PIOC
#define PWM2_PIN_UH      2   /* Port C */
#define PWM2_PIN_UL      3   /* Port C */
#define PWM2_PIN_VH      4   /* Port C */
#define PWM2_PIN_VL      5   /* Port C */
#define PWM2_PIN_WH      6   /* Port C */
#define PWM2_PIN_WL      7   /* Port C */
#define PWM2_PIN_SETTING ((0x1U<<PWM2_PIN_UH) | \
                          (0x1U<<PWM2_PIN_UL) | \
                          (0x1U<<PWM2_PIN_VH) | \
                          (0x1U<<PWM2_PIN_VL) | \
                          (0x1U<<PWM2_PIN_WH) | \
                          (0x1U<<PWM2_PIN_WL))

/*******************************************************************************
            ADC related definitions
*******************************************************************************/
#define ADC_CLOCK           1000000 /* 1MHz frequency of ADC */
#define ADC_REF             3.3     /* ADC reference voltage */
#define ADC_MAX_VAL_12BIT   4095

#if defined(POWER_SHIELD_V1)
    #define ADC_MAX_CUR 9.6 /* max value that can be mesured is 3.3V = */
                            /* 9.6 amps */
    #define ADC_VOLT_IN_AMPS 12 /* transfer gain of current sensor */
                                /* (2.5 Volts = 30 Amps) */
    #define ADC_CUR_OFFSET 3102 /* 0 amps = 2.5 Volts in current sensor */
    #define ADC_VOLT_DEVIDER    40 /* voltage divider for phase voltage */
                                   /* measurement */
#elif defined(POWER_SHIELD_V2) 
    #define ADC_MAX_CUR 25 /* max value that can be mesured is 3.3V = */
                           /* 25 amps */
    #define ADC_VOLT_IN_AMPS 15.15 /* transfer gain of current sensor */
                                   /* (1.65 Volts = 25 Amps) */
    #define ADC_CUR_OFFSET 2048 /* 0 amps = 1.65 Volts in current sensor */
    #define ADC_VOLT_DEVIDER    1 /* voltage divider for phase voltage */
                                   /* measurement */
#else
    #error "define version of power shield"
#endif

#define ADC07_PORT      PIOA
#define ADC811_PORT     PIOB
#define ADC_PIN_CH0     16      /* AD7  -> ch0 on arduino due */
#define ADC_PIN_CH1     24      /* AD6  -> ch1 on arduino due */
#define ADC_PIN_CH2     23      /* AD5  -> ch2 on arduino due */
#define ADC_PIN_CH3     22      /* AD4  -> ch3 on arduino due */
#define ADC_PIN_CH4     6       /* AD3  -> ch4 on arduino due */
#define ADC_PIN_CH5     4       /* AD2  -> ch5 on arduino due */
#define ADC_PIN_CH6     3       /* AD1  -> ch6 on arduino due */
#define ADC_PIN_CH7     2       /* AD0  -> ch7 on arduino due */

#define ADC811_PORT     PIOB
#define ADC_PIN_CH8     17     /* AD10 -> ch8 on arduino due */
#define ADC_PIN_CH9     18     /* AD11 -> ch9 on arduino due */


/* ADC 1 specific */
#define ADC1_CH_CUR_PHU     7     /* AD7  -> ch0 on arduino due */
#define ADC1_CH_CUR_PHV     6     /* AD6  -> ch1 on arduino due */
#define ADC1_CH_VOLT_PHU    5     /* AD5  -> ch2 on arduino due */
#define ADC1_CH_VOLT_PHV    4     /* AD4  -> ch3 on arduino due */
#define ADC1_CH_VOLT_PHW    3     /* AD3  -> ch4 on arduino due */

#define ADC1_CH_CUR_PHU_BIT     (0x1U<<ADC1_CH_CUR_PHU)
#define ADC1_CH_CUR_PHV_BIT     (0x1U<<ADC1_CH_CUR_PHV)
#define ADC1_CH_VOLT_PHU_BIT    (0x1U<<ADC1_CH_VOLT_PHU)
#define ADC1_CH_VOLT_PHV_BIT    (0x1U<<ADC1_CH_VOLT_PHV)
#define ADC1_CH_VOLT_PHW_BIT    (0x1U<<ADC1_CH_VOLT_PHW)

#define ADC1_CH_CUR_PHU_RESULT  (ADC->ADC_CDR[ADC1_CH_CUR_PHU])
#define ADC1_CH_CUR_PHV_RESULT  (ADC->ADC_CDR[ADC1_CH_CUR_PHV])
#define ADC1_CH_VOLT_PHU_RESULT (ADC->ADC_CDR[ADC1_CH_VOLT_PHU])
#define ADC1_CH_VOLT_PHV_RESULT (ADC->ADC_CDR[ADC1_CH_VOLT_PHV])
#define ADC1_CH_VOLT_PHW_RESULT (ADC->ADC_CDR[ADC1_CH_VOLT_PHW])

/* ADC 1 specific */
#define ADC2_CH_CUR_PHU     2     /* AD2  -> ch5 on arduino due */
#define ADC2_CH_CUR_PHV     1     /* AD1  -> ch6 on arduino due */
#define ADC2_CH_VOLT_PHU    0     /* AD0  -> ch7 on arduino due */
#define ADC2_CH_VOLT_PHV    10    /* AD10 -> ch8 on arduino due */
#define ADC2_CH_VOLT_PHW    11    /* AD11 -> ch9 on arduino due */

#define ADC2_CH_CUR_PHU_BIT     (0x1U<<ADC2_CH_CUR_PHU)
#define ADC2_CH_CUR_PHV_BIT     (0x1U<<ADC2_CH_CUR_PHV)
#define ADC2_CH_VOLT_PHU_BIT    (0x1U<<ADC2_CH_VOLT_PHU)
#define ADC2_CH_VOLT_PHV_BIT    (0x1U<<ADC2_CH_VOLT_PHV)
#define ADC2_CH_VOLT_PHW_BIT    (0x1U<<ADC2_CH_VOLT_PHW)

#define ADC2_CH_CUR_PHU_RESULT  (ADC->ADC_CDR[ADC2_CH_CUR_PHU])
#define ADC2_CH_CUR_PHV_RESULT  (ADC->ADC_CDR[ADC2_CH_CUR_PHV])
#define ADC2_CH_VOLT_PHU_RESULT (ADC->ADC_CDR[ADC2_CH_VOLT_PHU])
#define ADC2_CH_VOLT_PHV_RESULT (ADC->ADC_CDR[ADC2_CH_VOLT_PHV])
#define ADC2_CH_VOLT_PHW_RESULT (ADC->ADC_CDR[ADC2_CH_VOLT_PHW])

/* peripheral configuration */
typedef struct peripheralConfig{
    /* configuration of the PWM */
    struct PwmConfig
    {
        uint32_t    pwmSwFrq;
        uint32_t    pwmPio;
        uint8_t     pwmChU;
        uint8_t     pwmChV;
        uint8_t     pwmChW;
        uint32_t    pwmChBitU;
        uint32_t    pwmChBitV;
        uint32_t    pwmChBitW;
    }Pwm;
    /* configuration of the ADC */
    struct AdcConfig
    {
        uint8_t     adcChCurU;
        uint8_t     adcChCurV;
        uint8_t     adcChVoltU;
        uint8_t     adcChVoltV;
        uint8_t     adcChVoltW;
        uint32_t    adcChCurUBit;
        uint32_t    adcChCurVBit;
        uint32_t    adcChVoltUBit;
        uint32_t    adcChVoltVBit;
        uint32_t    adcChVoltWBit;
        uint32_t    *adcChCurUResult;
        uint32_t    *adcChCurVResult;
        uint32_t    *adcChVoltUResult;
        uint32_t    *adcChVoltVResult;
        uint32_t    *adcChVoltWResult;
    }Adc;
    /* configuration of the Ports */
    struct PioConfig
    {
        uint32_t    x;
    }Pio;
}BldcHardware;

/* configuration for BLDC boards */
extern BldcHardware BldcBoard1;
extern BldcHardware BldcBoard2;

#endif /* BLDC_PERIPHERALS_H */
