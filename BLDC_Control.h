/*******************************************************************************
  BLDC_Control.h - Library for brushless DC Motor Control.
  Author:   Swen Wahl
  Date:     07.03.2013
  License:  Released into the public domain
*******************************************************************************/
#ifndef BLDC_CONTROL_H
#define BLDC_CONTROL_H

#include "Arduino.h"
#include "BLDC_Peripherals.h"


/*******************************************************************************
           defines
*******************************************************************************/
//#define useTimer1
#define useAdcInterrupt

/*
  TC0, chan 0 => TC0_Handler
  TC0, chan 1 => TC1_Handler
  TC0, chan 2 => TC2_Handler
  TC1, chan 0 => TC3_Handler
  TC1, chan 1 => TC4_Handler
  TC1, chan 2 => TC5_Handler
  TC2, chan 0 => TC6_Handler
  TC2, chan 1 => TC7_Handler
  TC2, chan 2 => TC8_Handler
*/
#if defined (useTimer1)
#define TC_FOR_TIMER1       TC1
#define CHANNEL_FOR_TIMER1  0
#define ID_TC_FOR_TIMER1    ID_TC3
#define IRQn_FOR_TIMER1     TC3_IRQn
#define HANDLER_FOR_TIMER1  TC3_Handler
#endif    
#if defined (_useTimer2)
#define TC_FOR_TIMER2       TC1
#define CHANNEL_FOR_TIMER2  1
#define ID_TC_FOR_TIMER2    ID_TC4
#define IRQn_FOR_TIMER2     TC4_IRQn
#define HANDLER_FOR_TIMER2  TC4_Handler
#endif

#define BLDC_ROT_DIR_CW      1 /* clockwise rotation */
#define BLDC_ROT_DIR_CCW    -1 /* counter clockwise rotation */


typedef enum { _timer1, _timer2, _Nbr_16timers } timer16_Sequence_t ;
typedef int8_t commutationTable[8][3];

#define MAX_MOTORS  1

/*******************************************************************************
            structures and classes
*******************************************************************************/
struct machineProperties
{
    uint8_t   polePairs;
    uint16_t  ratedSpeed;
};


class BldcControl
{
    public:
        /* member methods */
        BldcControl(void);
        void    Config(void);
        void    start(void);
        void    stop(void);
        float   getActualSpeed(void);
        float   getDcLinkVoltage(void);
        void    setCurrentRef(float current);
        void    CommutationControl(void);

        uint32_t getTimerVal(void);
        /* member variables */
        float   speedRequest;       /* speed request in RPM */
        int16_t  debug;              /* debug variable */
        int16_t  currentRef;         /* current reference */
    /*--------------------------------------------------------------------*/
    private:
        /* member methods */
        void      configureTimerInterrupt(Tc         *tc, 
                                          uint32_t   channel, 
                                          IRQn_Type  irq, 
                                          uint32_t   frequency);
        void      configurePIOC(void);
        void      configureADC(void);
        void      configurePWMC(void);
        void      configurePMC(void);
        int16_t   CurrentControl(int16_t iFbk, int16_t iRef);
        void      pwmSwitchingCU(uint8_t hallState);
        uint8_t   readBemfState(void);
        
        /* member variables */
        peripheralConfig    *periphery;
        commutationTable    *actualCommutation;
        uint16_t            intCount;           /* interrupt counter */
        uint16_t            intCountPrev;       /* amount of interrupts between last commutation cycle */
        uint16_t            intCountFilt;       /* filtered value of intCount */
        uint32_t            intCountFiltTmp;    /* integrator value of intCount filter */
        uint8_t             motorIndex;         /* index of BLDC instance */
        uint8_t             prevBemfState;      /* back EMF state */
        uint16_t            bemfStateDelayCnt;  /* back EMF delay counter */
        int8_t              rotDirCmd;          /* rotational direction */
        uint16_t            dcVoltRaw;          /* raw values of DC link voltage */
        int16_t             rotorPosition;      /* rotor position in deggree */
        int16_t             deltaPhi;           /* delta phi in degree */
        int16_t             phiElecOld;         /* last elec angle in degree */
        uint16_t            pwmPeriod;          /* PWM perriod in ticks */
        machineProperties   motorProperties;    /* properties of machine */
        uint8_t             previousHallState;  /* last Hall state */
        int16_t             Kprp;               /* prportinal gain of Current Controlelr */
        uint16_t            Kint;               /* integral gain of Current Controlelr */
        int16_t             iInt;               /* integrator of current controller */
        int32_t             iFbkFiltTmp;        /* integrator value of current feedback filter */
};

#endif /* BLDC_CONTROL_H */
