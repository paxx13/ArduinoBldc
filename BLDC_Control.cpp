/*******************************************************************************
  BLDC_Control.cpp - Library for brushless DC Motor Control.
  Author:   Swen Wahl
  Date:     07.03.2013
  License:  Released into the public domain
            uses API of Atmel Software Framework
            http://asf.atmel.com/docs/latest/api.html
*******************************************************************************/

#include "Arduino.h"
#include "BLDC_Control.h"
#include "BLDC_Peripherals.h"

/*******************************************************************************
            defines
*******************************************************************************/
#define POWER_SHIELD_V2
#define DEAD_TIME       0.0000005/* 500 ns dead time */
#define SYS_CLOCK_84MHZ 84000000 /* system main clock is assumed to be 84MHz */
#define MCK_CLOCK_42MHZ 42000000 /* arduino has prescaler for main peripheral
                                    clock of 2 from system clock */

#define CTRL_FRQ    30000       /* controller runs at 30kHz */
#define CTRL_DELTM  1/CTRL_FRQ  /* reciprocal of control frequency */
#define SEC_PER_MIN 60

/* filter constant for current feedback */
/* value Bandwidth (normalized to 1 Hz) Rise time (samples)
    1       0.1197                      3
    2       0.0466                      8
    3       0.0217                      16
    4       0.0104                      34
    5       0.0051                      69
    6       0.0026                      140
    7       0.0012                      280
    8       0.0007                      561
*/
#define FILTER_VALUE 1


/* PWM related definitions */
#define PWM_SW_FRQ 30000 /* 30kHz switching frequency */
#define PWM_PERIOD (1 / PWM_SW_FRQ) /* PWM period */

#define PWM_CHANNEL_PHU 0
#define PWM_CHANNEL_PHV 1
#define PWM_CHANNEL_PHW 2

#define PWM_CH_PHU_BIT (0x1U<<PWM_CHANNEL_PHU)
#define PWM_CH_PHV_BIT (0x1U<<PWM_CHANNEL_PHV)
#define PWM_CH_PHW_BIT (0x1U<<PWM_CHANNEL_PHW)

#define PORT_PWM PIOC
#define PIN_PWM_UH 2 /* Port C */
#define PIN_PWM_UL 3 /* Port C */
#define PIN_PWM_VH 4 /* Port C */
#define PIN_PWM_VL 5 /* Port C */
#define PIN_PWM_WH 6 /* Port C */
#define PIN_PWM_WL 7 /* Port C */
#define PIN_PWM_SETTING ((0x1U<<PIN_PWM_UH) | \
                         (0x1U<<PIN_PWM_UL) | \
                         (0x1U<<PIN_PWM_VH) | \
                         (0x1U<<PIN_PWM_VL) | \
                         (0x1U<<PIN_PWM_WH) | \
                         (0x1U<<PIN_PWM_WL))
                         
/* PIO related definitions */
#define PORT_HALL1 PIOA
#define PORT_HALL2 PIOD
#define PORT_HALL3 PIOD

#define PIN_HALL1 15
#define PIN_HALL2 1
#define PIN_HALL3 3

#define HALL1_STATE ((PORT_HALL1 -> PIO_PDSR & (0x1U<<PIN_HALL1)) >> PIN_HALL1)
#define HALL2_STATE ((PORT_HALL2 -> PIO_PDSR & (0x1U<<PIN_HALL2)) >> PIN_HALL2)
#define HALL3_STATE ((PORT_HALL3 -> PIO_PDSR & (0x1U<<PIN_HALL3)) >> PIN_HALL3)

#define PORT_DEBUG PIOC
#define PIN_DEBUG  16
#define SET_DEBUG_PIN (PORT_DEBUG->PIO_SODR |= (0x1U<<PIN_DEBUG))
#define CLR_DEBUG_PIN (PORT_DEBUG->PIO_CODR |= (0x1U<<PIN_DEBUG))
#define TGL_DEBUG_PIN (if(PORT_DEBUG->PIO_ODSR){CLEAR_DEBUG_PIN}\
                       else {SET_DEBUG_PIN})

/* ADC related definitions */
#define ADC_CLOCK 1000000 /* 1MHz frequency of ADC */
#define ADC_REF   3.3     /* ADC reference voltage */
#define ADC_MAX_VAL_12BIT 4095

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
    #define ADC_VOLT_IN_AMPS 15 /* transfer gain of current sensor */
                                /* (1.65 Volts = 25 Amps) */
    #define ADC_CUR_OFFSET 2048 /* 0 amps = 1.65 Volts in current sensor */
    #define ADC_VOLT_DEVIDER    40 /* voltage divider for phase voltage */
                                   /* measurement */
#else
    #error "define version of power shield"
#endif



#define ADC_CH_CUR_PHA  7 /* AD7 */
#define ADC_CH_CUR_PHB  6 /* AD6 */
#define ADC_CH_VOLT_PHA 5 /* AD5 */
#define ADC_CH_VOLT_PHB 4 /* AD4 */
#define ADC_CH_VOLT_PHC 3 /* AD3 */

#define BIT_ADC_CH_CUR_PHA  (0x1U<<ADC_CH_CUR_PHA)
#define BIT_ADC_CH_CUR_PHB  (0x1U<<ADC_CH_CUR_PHB)
#define BIT_ADC_CH_VOLT_PHA (0x1U<<ADC_CH_VOLT_PHA)
#define BIT_ADC_CH_VOLT_PHB (0x1U<<ADC_CH_VOLT_PHB)
#define BIT_ADC_CH_VOLT_PHC (0x1U<<ADC_CH_VOLT_PHC)

#define ADC_CH_CUR_PHU_RESULT  (ADC->ADC_CDR[ADC_CH_CUR_PHA])
#define ADC_CH_CUR_PHV_RESULT  (ADC->ADC_CDR[ADC_CH_CUR_PHB])
#define ADC_CH_VOLT_PHU_RESULT (ADC->ADC_CDR[ADC_CH_VOLT_PHA])
#define ADC_CH_VOLT_PHV_RESULT (ADC->ADC_CDR[ADC_CH_VOLT_PHB])
#define ADC_CH_VOLT_PHW_RESULT (ADC->ADC_CDR[ADC_CH_VOLT_PHC])

#define PORT_ADC PIOA
#define PIN_ADC_CH0 16 /* AD7 -> ch0 on arduino due */
#define PIN_ADC_CH1 24 /* AD6 -> ch1 on arduino due */
#define PIN_ADC_CH2 23 /* AD5 -> ch2 on arduino due */
#define PIN_ADC_CH3 22 /* AD4 -> ch3 on arduino due */
#define PIN_ADC_CH4 6  /* AD3 -> ch4 on arduino due */
#define PIN_ADC_CH5 4  /* AD2 -> ch5 on arduino due */

/*******************************************************************************
            static variables
*******************************************************************************/
static uint8_t MotorCount = 0;
static BldcControl* motors[MAX_MOTORS];

/* table for clockwise commutation */
commutationTable commutationTableCW =
{   { 0, 0, 0}, /* illegal hall state 000 */
    {-1, 0, 1}, /* 001 */
    { 1,-1, 0}, /* 010 */
    { 0,-1, 1}, /* 011 */
    { 0, 1,-1}, /* 100 */
    {-1, 1, 0}, /* 101 */
    { 1, 0,-1}, /* 110 */
    { 0, 0, 0}, /* illegal hall state 111 */
};

/* table for counter clockwise commutation */
commutationTable commutationTableCCW =
{   { 0, 0, 0}, /* illegal hall state 000 */
    { 1, 0,-1}, /* 001 */
    {-1, 1, 0}, /* 010 */
    { 0, 1,-1}, /* 011 */
    { 0,-1, 1}, /* 100 */
    { 1,-1, 0}, /* 101 */
    {-1, 0, 1}, /* 110 */
    { 0, 0, 0}, /* illegal hall state 111 */
};

/*******************************************************************************
            interrupt handler
*******************************************************************************/
#if defined (useTimer1)
void HANDLER_FOR_TIMER1(void) {
    /* clear interrupt */
    TC_FOR_TIMER1->TC_CHANNEL[CHANNEL_FOR_TIMER1].TC_SR;
    motors[0]->CommutationControl();
}
#endif
#if defined (_useTimer2)
void HANDLER_FOR_TIMER2(void) {
    /* clear interrupt */
    TC_FOR_TIMER2->TC_CHANNEL[CHANNEL_FOR_TIMER2].TC_SR;
    motors[1]->CommutationControl();
}
#endif

#if defined (useAdcInterrupt)
void ADC_Handler(void)
{
    motors[0]->CommutationControl();
}
#endif

/*******************************************************************************
            private methods
*******************************************************************************/
/*------------------------------------------------------------------------------
Name:           configurePMC
parameters:     -
description:    initializes the Power Management controller
------------------------------------------------------------------------------*/
void BldcControl::configurePMC(void)
{
    pmc_set_writeprotect(false);
    
    /* enable PWM peripheral */
    pmc_enable_periph_clk(PWM_IRQn);
    
    /* enable PIO of inputs */
    pmc_enable_periph_clk(PIOA_IRQn);
    pmc_enable_periph_clk(PIOD_IRQn);
}

/*------------------------------------------------------------------------------
Name:           startTimer
parameters:     tc - timer counter
                channel - timer channel
                irq - isr request
                frequency - frequency of inetrrupts
description:    initializes timer for periodic interrupt generation
------------------------------------------------------------------------------*/
void BldcControl::configureTimerInterrupt(Tc *tc,
                                          uint32_t channel,
                                          IRQn_Type irq,
                                          uint32_t frequency)
{
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk((uint32_t)irq);
        TC_Configure(tc,
                     channel,
                     TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC |
                        TC_CMR_TCCLKS_TIMER_CLOCK4);
        uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected
                                                 //TIMER_CLOCK4 above
        TC_SetRA(tc, channel, rc/2); //50% high, 50% low
        TC_SetRC(tc, channel, rc);
        TC_Start(tc, channel);
        tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
        tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
        NVIC_EnableIRQ(irq);
}

/*------------------------------------------------------------------------------
Name:           configurePIOC
parameters:     -
description:    initializes the PIO controller
------------------------------------------------------------------------------*/
void BldcControl::configurePIOC(void)
{
    /* general config just for first instance */
    if (this->motorIndex == 0)
    {
        /* setup debug Pin */
        PORT_DEBUG -> PIO_PUDR |= (1U<<PIN_DEBUG); /* no pull up */
        PORT_DEBUG -> PIO_PER  |= (1U<<PIN_DEBUG); /* manual port control */
        PORT_DEBUG -> PIO_OER  |= (1U<<PIN_DEBUG); /* enable output */
        PORT_DEBUG -> PIO_ODSR |= (1U<<PIN_DEBUG); /* drive 0 */
        PORT_DEBUG -> PIO_IFDR |= (1U<<PIN_DEBUG); /* disable interrupt */
        PORT_DEBUG -> PIO_MDDR |= (1U<<PIN_DEBUG); /* no multi drive on line */
    }

    /* set pio registers to enable PWM at desired PINs (PC2 to PC7) */
    PIOC -> PIO_ABSR |= PIN_PWM_SETTING; /* enables Peripherial B */
    PIOC -> PIO_PDR  |= PIN_PWM_SETTING; /* disable manual port control */
    PIOC -> PIO_MDDR |= PIN_PWM_SETTING; /* disable multi drive on line */
    PIOC -> PIO_PUER |= PIN_PWM_SETTING; /* enable pull up resistors */

    /* set registers for hall inputs */
    PORT_HALL1 -> PIO_PUER |= (1U<<PIN_HALL1); /* enable pull up resistors */
    PORT_HALL1 -> PIO_PER  |= (1U<<PIN_HALL1); /* enable manual port control */
    PORT_HALL1 -> PIO_ODR  |= (1U<<PIN_HALL1); /* enable output */
    PORT_HALL1 -> PIO_ODSR |= (1U<<PIN_HALL1); /* drive 0 */
    PORT_HALL1 -> PIO_IFDR |= (1U<<PIN_HALL1); /* disable interrupt */
    PORT_HALL1 -> PIO_MDDR |= (1U<<PIN_HALL1); /* disable multi drive on line */
    
    PORT_HALL2 -> PIO_PUER |= (1U<<PIN_HALL2); /* enable pull up resistors */
    PORT_HALL2 -> PIO_PER  |= (1U<<PIN_HALL2); /* enable manual port control */
    PORT_HALL2 -> PIO_ODR  |= (1U<<PIN_HALL2); /* enable output */
    PORT_HALL2 -> PIO_ODSR |= (1U<<PIN_HALL2); /* drive 0 */
    PORT_HALL2 -> PIO_IFDR |= (1U<<PIN_HALL2); /* disable interrupt */
    PORT_HALL2 -> PIO_MDDR |= (1U<<PIN_HALL2); /* disable multi drive on line */
    
    PORT_HALL3 -> PIO_PUER |= (1U<<PIN_HALL3); /* enable pull up resistors */
    PORT_HALL3 -> PIO_PER  |= (1U<<PIN_HALL3); /* enable manual port control */
    PORT_HALL3 -> PIO_ODR  |= (1U<<PIN_HALL3); /* enable output */
    PORT_HALL3 -> PIO_ODSR |= (1U<<PIN_HALL3); /* drive 0 */
    PORT_HALL3 -> PIO_IFDR |= (1U<<PIN_HALL3); /* disable interrupt */
    PORT_HALL3 -> PIO_MDDR |= (1U<<PIN_HALL3); /* disable multi drive on line */
}

/*------------------------------------------------------------------------------
Name:           configureADC
parameters:     -
description:    initializes the ADC controller
                - no DMA transfers
                - no startup delay
                - 12 bit resolution
                - trigger on PWM event line 0
------------------------------------------------------------------------------*/
void BldcControl::configureADC(void)
{
    uint32_t prescaler;

    /* general config just for first instance */
    if (this->motorIndex == 0)
    {
        /* Reset the controller. */
        ADC->ADC_CR = ADC_CR_SWRST;
        
        /* Reset Mode Register. */
        ADC->ADC_MR = 0;

        /* Reset PDC transfer. */
        ADC->ADC_PTCR = (ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS);
        ADC->ADC_RCR = 0;
        ADC->ADC_RNCR = 0;
        
        /* set clock prescaler */
        prescaler = MCK_CLOCK_42MHZ / (8 * ADC_CLOCK) - 1;
        ADC->ADC_MR |= ADC_MR_PRESCAL(prescaler) |
                       ADC_MR_STARTUP_SUT0; /* no startup delay */


        adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
        
        /* set 12 bit resolution */
        ADC->ADC_MR |= ADC_MR_LOWRES_BITS_12;

        /* set ADC trigger */
        ADC->ADC_MR |= ADC_MR_TRGEN_EN | /* enable trigger */
                       ADC_MR_TRGSEL_ADC_TRIG4; /*PWM event line 0 trrigger */
    }

#if defined (useAdcInterrupt)
    /* ADC enable interrupts */
    ADC->ADC_IER |= 0x80;//ADC_CH_CUR_PHA; /* interrupt on highest channel number */
    NVIC_EnableIRQ(ADC_IRQn);
#endif
}

/*------------------------------------------------------------------------------
Name:           configurePWMC
parameters:     -
description:    initializes the PWM controller
------------------------------------------------------------------------------*/
void BldcControl::configurePWMC(void)
{
    uint32_t clka = 0; /* clock A not used */
    uint32_t clkb = 0; /* clock B not used */
    uint32_t mck = MCK_CLOCK_42MHZ;
    uint16_t duty = 0;
    uint16_t deadTime;
    
    pwmPeriod = MCK_CLOCK_42MHZ / PWM_SW_FRQ;
    
    deadTime = (uint16_t)(pwmPeriod * (DEAD_TIME * PWM_SW_FRQ * 2));

    /* disable all 3 channels */
    PWMC_DisableChannel(PWM, PWM_CHANNEL_PHU);
    PWMC_DisableChannel(PWM, PWM_CHANNEL_PHV);
    PWMC_DisableChannel(PWM, PWM_CHANNEL_PHW);

    PWMC_ConfigureClocks(clka, clkb, mck);

    /* initialize all 3 channels */
    PWMC_ConfigureChannelExt(PWM,
                          PWM_CHANNEL_PHU, /* channel ID */
                          PWM_CMR_CPRE_MCK, /* use main clock */
                          PWM_CMR_CALG, /* center alligned */
                          0, /* polarity low level */
                          0, /* event counter = 0 */
                          PWM_CMR_DTE, /* enable dead time */
                          0,  /* no inversion of dead time H */
                          0); /* no inversion of dead time L */
    PWMC_ConfigureChannelExt(PWM,
                          PWM_CHANNEL_PHV, /* channel ID */
                          PWM_CMR_CPRE_MCK,
                          PWM_CMR_CALG, /* center alligned */
                          0, /* polarity low level */
                          0, /* event counter = 0 */
                          PWM_CMR_DTE, /* enable dead time */
                          0,  /* no inversion of dead time H */
                          0); /* no inversion of dead time L */
    PWMC_ConfigureChannelExt(PWM,
                          PWM_CHANNEL_PHW, /* channel ID */
                          PWM_CMR_CPRE_MCK,
                          PWM_CMR_CALG, /* center alligned */
                          0, /* polarity low level */
                          0, /* event counter = 0 */
                          PWM_CMR_DTE, /* enable dead time */
                          0,  /* no inversion of dead time H */
                          0); /* no inversion of dead time L */
                             
    PWMC_ConfigureSyncChannel(PWM,
                              PWM_SCM_SYNC0|PWM_SCM_SYNC1|PWM_SCM_SYNC2,
                              PWM_SCM_UPDM_MODE0,
                              PWM_SCM_PTRM,
                              PWM_SCM_PTRCS(0)) ;

    /* set periods */
    PWMC_SetPeriod(PWM, PWM_CHANNEL_PHU, pwmPeriod);
    PWMC_SetPeriod(PWM, PWM_CHANNEL_PHV, pwmPeriod);
    PWMC_SetPeriod(PWM, PWM_CHANNEL_PHW, pwmPeriod);

    /* set duty cycles */
    PWMC_SetDutyCycle(PWM, PWM_CHANNEL_PHU, duty);
    PWMC_SetDutyCycle(PWM, PWM_CHANNEL_PHV, duty);
    PWMC_SetDutyCycle(PWM, PWM_CHANNEL_PHW, duty);

    /* set dead times */
    PWMC_SetDeadTime(PWM, PWM_CHANNEL_PHU, deadTime, deadTime);
    PWMC_SetDeadTime(PWM, PWM_CHANNEL_PHV, deadTime, deadTime);
    PWMC_SetDeadTime(PWM, PWM_CHANNEL_PHW, deadTime, deadTime);

    /* set overwrites to 0 */
    PWMC_SetOverrideValue(PWM, 0);

    /* set event for ADC trigger */
    PWM->PWM_CMP[PWM_CHANNEL_PHU].PWM_CMPM |= PWM_CMPM_CEN | /* enable */
                                           PWM_CMPM_CTR(0) | /* each period */
                                           PWM_CMPM_CPR(0);
    PWM->PWM_CMP[PWM_CHANNEL_PHU].PWM_CMPV = pwmPeriod/2; /* set trigger */
    PWM->PWM_ELMR[0] |= PWM_ELMR_CSEL0; /* enable event line 0 */

}

/*------------------------------------------------------------------------------
Name:           pwmSwitchingCU
parameters:     hallState - states of hall sensors bit coded
                bit 1 = Hall Sensor 1
                bit 2 = Hall Sensor 2
                bit 3 = Hall Sensor 3
description:    sets PWM registers for unipolar complementary switching
------------------------------------------------------------------------------*/
void BldcControl::pwmSwitchingCU(uint8_t hallState)
{
    uint32_t setOverwrite = 0;
    uint32_t clearOverwrite = 0;

    /* set phase U freewheeling */
    if ((*(this->actualCommutation))[hallState][0] == 0)
    {
        setOverwrite |= PWM_CH_PHU_BIT | (PWM_CH_PHU_BIT<<0x10U);
    }
    else
    {
        clearOverwrite |= PWM_CH_PHU_BIT | (PWM_CH_PHU_BIT<<0x10U);
    }
    
    /* set phase V freewheeling */
    if ((*(this->actualCommutation))[hallState][1] == 0)
    {
        setOverwrite |= PWM_CH_PHV_BIT | (PWM_CH_PHV_BIT<<0x10U);
    }
    else
    {
        clearOverwrite |= PWM_CH_PHV_BIT | (PWM_CH_PHV_BIT<<0x10U);
    }
    
    /* set phase W freewheeling */
    if ((*(this->actualCommutation))[hallState][2] == 0)
    {
        setOverwrite |= PWM_CH_PHW_BIT | (PWM_CH_PHW_BIT<<0x10U);
    }
    else
    {
        clearOverwrite |= PWM_CH_PHW_BIT | (PWM_CH_PHW_BIT<<0x10U);
    }

    PWM->PWM_OSS |= setOverwrite;
    PWM->PWM_OSCUPD |= clearOverwrite;

    return;
}

/*------------------------------------------------------------------------------
Name:           readBemfState
parameters:     - 
description:    returns the state of the BEMF for phase U, V and W. the return
                value is coded:
                bit0: phase U
                bit1: phase V
                bit2: phase W
                The output is delayed by half the last commutation cycle time
                (represents 30 degrees)
------------------------------------------------------------------------------*/
uint8_t BldcControl::readBemfState(void)
{
    uint16_t halfDcLinkVolt = this->dcLinkVoltage>>1;
    uint8_t  bemfState = 0;
    uint8_t  retVal;

    /* get back EMF of each phase */
    if (ADC_CH_VOLT_PHU_RESULT >= halfDcLinkVolt)
    {
        bemfState |= 0x01;
    }
    else
    {
        bemfState &= ~0x01;
    }

    if (ADC_CH_VOLT_PHV_RESULT >= halfDcLinkVolt)
    {
        bemfState |= 0x02;
    }
    else
    {
        bemfState &= ~0x02;
    }

    if (ADC_CH_VOLT_PHW_RESULT >= halfDcLinkVolt)
    {
        bemfState |= 0x04;
    }
    else
    {
        bemfState &= ~0x04;
    }
    
    /* check if state changed*/
    if (this->prevBemfState != bemfState)
    {
        this->bemfStateDelayCnt = this->prevIntCount>>1; 
    }
    else 
    {
        this->bemfStateDelayCnt--;
    }
    
    if (this->bemfStateDelayCnt == 0)
    {
        retVal = bemfState;
        this->prevBemfState = bemfState;
    }
    else
    {
        retVal = this->prevBemfState;
    }

    return retVal;
}


/*******************************************************************************
            public methods
*******************************************************************************/
/*------------------------------------------------------------------------------
Name:           BldcControl
parameters:     -
description:    constructor
------------------------------------------------------------------------------*/
BldcControl::BldcControl(void)
{
    if (MotorCount < MAX_MOTORS)
    {
        motors[MotorCount] = this;
        this->motorIndex = MotorCount++; /* assign index to this instance */
    }
}

/*------------------------------------------------------------------------------
Name:           Config
parameters:     -
description:    initializes motor controler
------------------------------------------------------------------------------*/
void BldcControl::Config(void)
{
    /* configure Power Management */
    configurePMC();

    if (this->motorIndex < MAX_MOTORS)
    {
        /* setup timer for interrupt generation */
        #if defined (useTimer1)
        if (this->motorIndex == 0)
            configureTimerInterrupt(TC_FOR_TIMER1,
                                    CHANNEL_FOR_TIMER1,
                                    IRQn_FOR_TIMER1,
                                    CTRL_FRQ);
        #endif
        #if defined (_useTimer2)
        if (this->motorIndex == 1)
            configureTimerInterrupt(TC_FOR_TIMER2,
                                    CHANNEL_FOR_TIMER2,
                                    IRQn_FOR_TIMER2,
                                    CTRL_FRQ);
        #endif
    }

    /* setup Pin configuration */
    configurePIOC();
    
    /* setup ADC configuration */
    configureADC();

    /* setup PWM configuration */
    configurePWMC();

    /* set motor properties */
    this->actualCommutation = &commutationTableCW;
    this->motorProperties.polePairs = 4;
    this->Kprp = 2;
    this->Kint = 1;
    
    return;
}

/*------------------------------------------------------------------------------
Name:           Controller
parameters:     -
description:    inner loop control. 
                - measures DC Link voltage
                - Measures commutation by reading BEMF/Hall sensors. 
                - calculates motor current from phase currents
                - runs the current regulator to control the duty cycle of PWM
                - sets the block commutation accordingly
------------------------------------------------------------------------------*/
void BldcControl::CommutationControl(void)
{
    uint8_t  hallState, bemfState;
    uint16_t tmp_per = this->pwmPeriod/2;
    int16_t  tmp_dc;
    int16_t  Iu, Iv, Iw;
    int16_t  Ifilt;
    int16_t  currentFbk;

    debug++;
SET_DEBUG_PIN;
    this->interruptCounter++;

    /* measure DC Link Voltage */
    this->dcLinkVoltage = (ADC_CH_VOLT_PHU_RESULT + ADC_CH_VOLT_PHV_RESULT +
                           ADC_CH_VOLT_PHW_RESULT) / 3;
    /* read BEMF */
    bemfState = readBemfState();
    /* read hall sensors */
    hallState = (uint8_t)((HALL1_STATE | (HALL2_STATE<<1) | (HALL3_STATE<<2))&
                0b111U);

    /* calculate motor current */
    Iu = (((int16_t)(ADC_CH_CUR_PHU_RESULT)) - ADC_CUR_OFFSET);
    Iv = (((int16_t)(ADC_CH_CUR_PHV_RESULT)) - ADC_CUR_OFFSET);
    Iw = -Iu - Iv;
    currentFbk = (abs(Iu) + abs(Iv) + abs(Iw)) / 2;

    /* filter measured current */
    this->IfbkFilt += (currentFbk- (this->IfbkFilt >> FILTER_VALUE));
    Ifilt = this->IfbkFilt >> FILTER_VALUE;

    /* run current control */
    tmp_dc = CurrentControl(Ifilt, currentRef);

    /* change commutation if hall state changed */
    if (hallState != this->previousHallState)
    {
        /* set outputs according to the commutation cycles */
        pwmSwitchingCU(hallState);
        
        /* save current hall state */
        this->previousHallState = hallState;
        
        /* calculate rotor position */
        this->deltaPhi = (this->rotDirection == 1)?+60:-60;
        this->rotorPosition += (this->deltaPhi / 
                                this->motorProperties.polePairs);
        this->rotorPosition = this->rotorPosition%360;
        
        /* calculate speed */
        this->prevIntCount = this->interruptCounter;
        this->interruptCounter = 0;
    }
    else
    {
        /* hall state did not change */
    }

    /* update duty cycle */
    PWM->PWM_CH_NUM[PWM_CHANNEL_PHU].PWM_CDTYUPD = (int16_t)tmp_per +
                      (((*(this->actualCommutation))[hallState][0]) *
                      (int16_t)tmp_dc);
    PWM->PWM_CH_NUM[PWM_CHANNEL_PHV].PWM_CDTYUPD = (int16_t)tmp_per +
                      (((*(this->actualCommutation))[hallState][1]) *
                      (int16_t)tmp_dc);
    PWM->PWM_CH_NUM[PWM_CHANNEL_PHW].PWM_CDTYUPD = (int16_t)tmp_per +
                      (((*(this->actualCommutation))[hallState][2]) *
                      (int16_t)tmp_dc);

    /* enable update */
    PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

    /* debug output */
    analogWriteResolution(12);
    analogWrite(66,(bemfState&0x1)*4095); /* DAC0 */
    analogWrite(67,(hallState&0x1)*4095); /* DAC1 */
CLR_DEBUG_PIN;
}

/*------------------------------------------------------------------------------
Name:           CurrentControl
parameters:     iFbk - Current Feedback (raw value)
                iRef - Current Reference (raw value)
description:    PI controller to control the DC bus current
------------------------------------------------------------------------------*/
int16_t BldcControl::CurrentControl(int16_t iFbk, int16_t iRef)
{
    int16_t iErr;
    int16_t iOut;
    int16_t iPrp;
    int16_t iIntLim;

    /* calculate error */
    iErr = iRef - iFbk;
    
    /* proportional path */
    iPrp = iErr >> this->Kprp;
    
    /* integral path */
    if (iErr > 0)
    {
        iInt = iInt + Kint;
    }
    else if (iErr < 0)
    {
        iInt = iInt - Kint;
    }
    /* integral value limiter - anti wind up */
    iIntLim = (int16_t)(this->pwmPeriod>>3);
    if (iInt > iIntLim)
    {
        iInt = iIntLim;
    }
    else if (iInt < -(iIntLim))
    {
        iInt = -(iIntLim);
    }


    iOut =  iPrp + iInt;

    /* limit output */
    if (iOut > (this->pwmPeriod>>1))
    {
        iOut = (this->pwmPeriod>>1);
    }
    else if (iOut <= 0)
    {
        iOut = 0;
    }
    
    
    return iOut;
}

/*------------------------------------------------------------------------------
Name:           start
parameters:     -
description:    starts the machine control by activating ADC and PWM
------------------------------------------------------------------------------*/
void BldcControl::start(void)
{
    /* enable ADC channels */
    ADC->ADC_CHER |= BIT_ADC_CH_CUR_PHA  | BIT_ADC_CH_CUR_PHB |
                     BIT_ADC_CH_VOLT_PHA | BIT_ADC_CH_VOLT_PHB |
                     BIT_ADC_CH_VOLT_PHC;

    /* enable all the PWM channels for the 3 phases*/
    PWMC_EnableChannel(PWM, PWM_CHANNEL_PHU);
    PWMC_EnableChannel(PWM, PWM_CHANNEL_PHV);
    PWMC_EnableChannel(PWM, PWM_CHANNEL_PHW);
}

/*------------------------------------------------------------------------------
Name:           stop
parameters:     -
description:    stops the machine control by deactivating ADC and PWM
------------------------------------------------------------------------------*/
void BldcControl::stop(void)
{
    /* enable all the PWM channels for the 3 phases*/
    PWMC_DisableChannel(PWM, PWM_CHANNEL_PHU);
    PWMC_DisableChannel(PWM, PWM_CHANNEL_PHV);
    PWMC_DisableChannel(PWM, PWM_CHANNEL_PHW);

    /* enable ADC channels */
    ADC->ADC_CHDR |= BIT_ADC_CH_CUR_PHA  | BIT_ADC_CH_CUR_PHB |
                     BIT_ADC_CH_VOLT_PHA | BIT_ADC_CH_VOLT_PHB |
                     BIT_ADC_CH_VOLT_PHC;
}

/*------------------------------------------------------------------------------
Name:           setCurrentRef
parameters:     current - motor current reference in amps
description:    sets the raw value of the current reference for inner loop
                control. the input is limited to the maximum allowed current
------------------------------------------------------------------------------*/
void BldcControl::setCurrentRef(float current)
{
    /* limit current reference */
    current = (current > ADC_MAX_CUR)?  ADC_MAX_CUR:current;
    current = (current < -ADC_MAX_CUR)?-ADC_MAX_CUR:current;

    /* set rotational direction */
    this->rotDirection = (current >= 0)?1:-1;
    this->actualCommutation = (this->rotDirection == 1)?&commutationTableCW:
                                                        &commutationTableCCW;

    /* set raw value of current reference */
    this->currentRef = (int16_t)((abs(current)/ ADC_VOLT_IN_AMPS) *
                                 (ADC_MAX_VAL_12BIT / ADC_REF));
    return;
}

/*------------------------------------------------------------------------------
Name:           getActualSpeed
parameters:     -
description:    returns the actual speed of the machine in rpm
------------------------------------------------------------------------------*/
float BldcControl::getActualSpeed(void)
{
    float actualSpeed;

    actualSpeed = (float)(CTRL_FRQ / this->prevIntCount) *
               ((((float)(this->deltaPhi) / this->motorProperties.polePairs ))/
                         360) * (float)SEC_PER_MIN;

    return actualSpeed;
}

/*------------------------------------------------------------------------------
Name:           getDcLinkVoltage
parameters:     -
description:    returns the actual DC link voltage in Volts
------------------------------------------------------------------------------*/
float BldcControl::getDcLinkVoltage(void)
{
    float dcVoltage;

    dcVoltage = (ADC_REF / ADC_MAX_VAL_12BIT)*((float)(this->dcLinkVoltage)) 
                * ADC_VOLT_DEVIDER;

    return dcVoltage;
}
