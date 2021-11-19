//#############################################################################
//
// FILE:        vienna_hal.h
//
// TITLE:       solution hardware abstraction layer header file
//              This file consists of common variables and functions
//              for a particular hardware board, like functions to read current
//              and voltage signals on the board and functions to setup the
//              basic peripherals of the board
//              This file must be settings independent, all settings dependent
//              code should reside in the parent solution project.
//
//#############################################################################
// $TI Release: TIDM_1000 v4.01.01.00 $
// $Release Date: Tue Nov  3 10:37:49 CST 2020 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef PFC3PHVIENNA_BOARD_H
#define PFC3PHVIENNA_H

#ifdef __cplusplus

extern "C" {
#endif


//
//******************** the includes *******************************************
//

//
// Include files for device support
//
#include "device.h"
#include "vienna_settings.h"

//
//********************* defines ***********************************************
//

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define VIENNA_TASKA_CTR_OVFLW_STATUS CPUTimer_getTimerOverflowStatus(CPUTIMER0_BASE)
#define VIENNA_CLEAR_TASKA_CTR_OVFLW_FLAG CPUTimer_clearOverflowFlag(CPUTIMER0_BASE)

#define VIENNA_TASKB_CTR_OVFLW_STATUS CPUTimer_getTimerOverflowStatus(CPUTIMER1_BASE)
#define VIENNA_CLEAR_TASKB_CTR_OVFLW_FLAG CPUTimer_clearOverflowFlag(CPUTIMER1_BASE)

#define VIENNA_IL1_FB_1    ADC_readResult(VIENNA_IL1_ADCRESULTREGBASE, VIENNA_IL1_ADC_SOC_NO_1)
#define VIENNA_IL1_FB_2    ADC_readResult(VIENNA_IL1_ADCRESULTREGBASE, VIENNA_IL1_ADC_SOC_NO_2)
#define VIENNA_IL1_FB_3    ADC_readResult(VIENNA_IL1_ADCRESULTREGBASE, VIENNA_IL1_ADC_SOC_NO_3)
#define VIENNA_IL1_FB_4    ADC_readResult(VIENNA_IL1_ADCRESULTREGBASE, VIENNA_IL1_ADC_SOC_NO_4)

#define VIENNA_IL2_FB_1    ADC_readResult(VIENNA_IL2_ADCRESULTREGBASE, VIENNA_IL2_ADC_SOC_NO_1)
#define VIENNA_IL2_FB_2    ADC_readResult(VIENNA_IL2_ADCRESULTREGBASE, VIENNA_IL2_ADC_SOC_NO_2)
#define VIENNA_IL2_FB_3    ADC_readResult(VIENNA_IL2_ADCRESULTREGBASE, VIENNA_IL2_ADC_SOC_NO_3)
#define VIENNA_IL2_FB_4    ADC_readResult(VIENNA_IL2_ADCRESULTREGBASE, VIENNA_IL2_ADC_SOC_NO_4)

#define VIENNA_IPV1_FB_1    ADC_readResult(VIENNA_IPV1_ADCRESULTREGBASE, VIENNA_IPV1_ADC_SOC_NO_1)
#define VIENNA_IPV1_FB_2    ADC_readResult(VIENNA_IPV1_ADCRESULTREGBASE, VIENNA_IPV1_ADC_SOC_NO_2)
#define VIENNA_IPV1_FB_3    ADC_readResult(VIENNA_IPV1_ADCRESULTREGBASE, VIENNA_IPV1_ADC_SOC_NO_3)
#define VIENNA_IPV1_FB_4    ADC_readResult(VIENNA_IPV1_ADCRESULTREGBASE, VIENNA_IPV1_ADC_SOC_NO_4)

#define VIENNA_IPV2_FB_1    ADC_readResult(VIENNA_IPV2_ADCRESULTREGBASE, VIENNA_IPV2_ADC_SOC_NO_1)
#define VIENNA_IPV2_FB_2    ADC_readResult(VIENNA_IPV2_ADCRESULTREGBASE, VIENNA_IPV2_ADC_SOC_NO_2)
#define VIENNA_IPV2_FB_3    ADC_readResult(VIENNA_IPV2_ADCRESULTREGBASE, VIENNA_IPV2_ADC_SOC_NO_3)
#define VIENNA_IPV2_FB_4    ADC_readResult(VIENNA_IPV2_ADCRESULTREGBASE, VIENNA_IPV2_ADC_SOC_NO_4)

#define VIENNA_VPV1_FB_1    ADC_readResult(VIENNA_VPV1_ADCRESULTREGBASE, VIENNA_VPV1_ADC_SOC_NO_1)
#define VIENNA_VPV1_FB_2    ADC_readResult(VIENNA_VPV1_ADCRESULTREGBASE, VIENNA_VPV1_ADC_SOC_NO_2)
#define VIENNA_VPV1_FB_3    ADC_readResult(VIENNA_VPV1_ADCRESULTREGBASE, VIENNA_VPV1_ADC_SOC_NO_3)
#define VIENNA_VPV1_FB_4    ADC_readResult(VIENNA_VPV1_ADCRESULTREGBASE, VIENNA_VPV1_ADC_SOC_NO_4)

#define VIENNA_VPV2_FB_1    ADC_readResult(VIENNA_VPV2_ADCRESULTREGBASE, VIENNA_VPV2_ADC_SOC_NO_1)
#define VIENNA_VPV2_FB_2    ADC_readResult(VIENNA_VPV2_ADCRESULTREGBASE, VIENNA_VPV2_ADC_SOC_NO_2)
#define VIENNA_VPV2_FB_3    ADC_readResult(VIENNA_VPV2_ADCRESULTREGBASE, VIENNA_VPV2_ADC_SOC_NO_3)
#define VIENNA_VPV2_FB_4    ADC_readResult(VIENNA_VPV2_ADCRESULTREGBASE, VIENNA_VPV2_ADC_SOC_NO_4)

#define VIENNA_VDC_FB_1    ADC_readResult(VIENNA_VDC_ADCRESULTREGBASE, VIENNA_VDC_ADC_SOC_NO_1)
#define VIENNA_VDC_FB_2    ADC_readResult(VIENNA_VDC_ADCRESULTREGBASE, VIENNA_VDC_ADC_SOC_NO_2)
#define VIENNA_VDC_FB_3    ADC_readResult(VIENNA_VDC_ADCRESULTREGBASE, VIENNA_VDC_ADC_SOC_NO_3)
#define VIENNA_VDC_FB_4    ADC_readResult(VIENNA_VDC_ADCRESULTREGBASE, VIENNA_VDC_ADC_SOC_NO_4)

#define VIENNA_VCC_FB_1    ADC_readResult(VIENNA_VCC_ADCRESULTREGBASE, VIENNA_VCC_ADC_SOC_NO_1)
#define VIENNA_VCC_FB_2    ADC_readResult(VIENNA_VCC_ADCRESULTREGBASE, VIENNA_VCC_ADC_SOC_NO_2)
#define VIENNA_VCC_FB_3    ADC_readResult(VIENNA_VCC_ADCRESULTREGBASE, VIENNA_VCC_ADC_SOC_NO_3)
#define VIENNA_VCC_FB_4    ADC_readResult(VIENNA_VCC_ADCRESULTREGBASE, VIENNA_VCC_ADC_SOC_NO_4)

#define VIENNA_TEMP_FB_1    ADC_readResult(VIENNA_TEMP_ADCRESULTREGBASE, VIENNA_TEMP_ADC_SOC_NO_1)
#define VIENNA_TEMP_FB_2    ADC_readResult(VIENNA_TEMP_ADCRESULTREGBASE, VIENNA_TEMP_ADC_SOC_NO_2)
#define VIENNA_TEMP_FB_3    ADC_readResult(VIENNA_TEMP_ADCRESULTREGBASE, VIENNA_TEMP_ADC_SOC_NO_3)
#define VIENNA_TEMP_FB_4    ADC_readResult(VIENNA_TEMP_ADCRESULTREGBASE, VIENNA_TEMP_ADC_SOC_NO_4)

#define VIENNA_POT_FB_1    ADC_readResult(VIENNA_POT_ADCRESULTREGBASE, VIENNA_POT_ADC_SOC_NO_1)
#define VIENNA_POT_FB_2    ADC_readResult(VIENNA_POT_ADCRESULTREGBASE, VIENNA_POT_ADC_SOC_NO_2)
#define VIENNA_POT_FB_3    ADC_readResult(VIENNA_POT_ADCRESULTREGBASE, VIENNA_POT_ADC_SOC_NO_3)
#define VIENNA_POT_FB_4    ADC_readResult(VIENNA_POT_ADCRESULTREGBASE, VIENNA_POT_ADC_SOC_NO_4)

/*

#define VIENNA_V1_FB_1    ADC_readResult(VIENNA_V1_ADCRESULTREGBASE, VIENNA_V1_ADC_SOC_NO_1)
#define VIENNA_V1_FB_2    ADC_readResult(VIENNA_V1_ADCRESULTREGBASE, VIENNA_V1_ADC_SOC_NO_2)
#define VIENNA_V1_FB_3    ADC_readResult(VIENNA_V1_ADCRESULTREGBASE, VIENNA_V1_ADC_SOC_NO_3)
#define VIENNA_V1_FB_4    ADC_readResult(VIENNA_V1_ADCRESULTREGBASE, VIENNA_V1_ADC_SOC_NO_4)

#define VIENNA_V2_FB_1    ADC_readResult(VIENNA_V2_ADCRESULTREGBASE, VIENNA_V2_ADC_SOC_NO_1)
#define VIENNA_V2_FB_2    ADC_readResult(VIENNA_V2_ADCRESULTREGBASE, VIENNA_V2_ADC_SOC_NO_2)
#define VIENNA_V2_FB_3    ADC_readResult(VIENNA_V2_ADCRESULTREGBASE, VIENNA_V2_ADC_SOC_NO_3)
#define VIENNA_V2_FB_4    ADC_readResult(VIENNA_V2_ADCRESULTREGBASE, VIENNA_V2_ADC_SOC_NO_4)

#define VIENNA_V3_FB_1    ADC_readResult(VIENNA_V3_ADCRESULTREGBASE, VIENNA_V3_ADC_SOC_NO_1)
#define VIENNA_V3_FB_2    ADC_readResult(VIENNA_V3_ADCRESULTREGBASE, VIENNA_V3_ADC_SOC_NO_2)
#define VIENNA_V3_FB_3    ADC_readResult(VIENNA_V3_ADCRESULTREGBASE, VIENNA_V3_ADC_SOC_NO_3)
#define VIENNA_V3_FB_4    ADC_readResult(VIENNA_V3_ADCRESULTREGBASE, VIENNA_V3_ADC_SOC_NO_4)

#define VIENNA_VBUSPM_FB_1 ADC_readResult(VIENNA_VBUSPM_ADCRESULTREGBASE, VIENNA_VBUSPM_ADC_SOC_NO_1)
#define VIENNA_VBUSPM_FB_2 ADC_readResult(VIENNA_VBUSPM_ADCRESULTREGBASE, VIENNA_VBUSPM_ADC_SOC_NO_2)
#define VIENNA_VBUSPM_FB_3 ADC_readResult(VIENNA_VBUSPM_ADCRESULTREGBASE, VIENNA_VBUSPM_ADC_SOC_NO_3)
#define VIENNA_VBUSPM_FB_4 ADC_readResult(VIENNA_VBUSPM_ADCRESULTREGBASE, VIENNA_VBUSPM_ADC_SOC_NO_4)

#define VIENNA_VBUSMN_FB_1 ADC_readResult(VIENNA_VBUSMN_ADCRESULTREGBASE, VIENNA_VBUSMN_ADC_SOC_NO_1)
#define VIENNA_VBUSMN_FB_2 ADC_readResult(VIENNA_VBUSMN_ADCRESULTREGBASE, VIENNA_VBUSMN_ADC_SOC_NO_2)
#define VIENNA_VBUSMN_FB_3 ADC_readResult(VIENNA_VBUSMN_ADCRESULTREGBASE, VIENNA_VBUSMN_ADC_SOC_NO_3)
#define VIENNA_VBUSMN_FB_4 ADC_readResult(VIENNA_VBUSMN_ADCRESULTREGBASE, VIENNA_VBUSMN_ADC_SOC_NO_4)
*/

#define VIENNA_PWM_TRIP_STATUS EPWM_getTripZoneFlagStatus
#define VIENNA_PWM_EMU_TRIP_STATUS(m) (VIENNA_PWM_TRIP_STATUS(m) & EPWM_TZ_INTERRUPT_CBC)
#define VIENNA_PWM_OC_TRIP_STATUS(m) (VIENNA_PWM_TRIP_STATUS(m) & EPWM_TZ_INTERRUPT_DCAEVT1)
#define VIENNA_PWM_OC1_TRIP_STATUS(m) (VIENNA_PWM_TRIP_STATUS(m) & EPWM_TZ_INTERRUPT_DCAEVT1)
#define VIENNA_PWM_OC2_TRIP_STATUS(m) (VIENNA_PWM_TRIP_STATUS(m) & EPWM_TZ_INTERRUPT_DCBEVT1)
#define VIENNA_PWM_OST_TRIP_STATUS(m) (VIENNA_PWM_TRIP_STATUS(m) & EPWM_TZ_INTERRUPT_OST)

#define VIENNA_CURR_TRIP_LATCH_STATUS(m) CMPSS_getStatus(m)

#define VIENNA_CMPSS_LATCH_CLEAR_HIGH_LOW(m)

#define VIENNA_ADC_PU_SCALE_FACTOR  ((float32_t)0.000244140625)

#define ADC_VPV_GAIN  ((float32_t)0.2497808)
#define ADC_VDC_GAIN  ((float32_t)0.20609618)
#define ADC_I_GAIN  ((float32_t)0.0261841)
#define ADC_TEMP_GAIN  ((float32_t)0.080566)
#define ADC_POT_GAIN  ((float32_t)0.00024332)
#define ADC_VCC_GAIN  ((float32_t)0.00161135)

#define GAIN_R_BY_VDCO 0.00016227
#define GAIN_1_MINUS_D_BY_VDCO 0.00081137
#define SET    1
#define CLEAR   0
#define ON 1
#define OFF 0
#define ENABLE 1
#define DISABLE 0

//#define PowerRelay  GpioDataRegs.GPADAT.bit.GPIO14
//#define DisablePWM  GpioDataRegs.GPADAT.bit.GPIO22

//
// 1/2^11
//
#define VIENNA_ADC_PU_PPB_SCALE_FACTOR ((float32_t)0.000488281250)

//
// 1/2^15
//
#define VIENNA_SD_PU_SCALE_FACTOR  ((float32_t)0.000030517578125)

//
//********************** the function prototypes *****************************
//

//
// Device setup
//
extern void VIENNA_HAL_setupDevice(void);

//
// PWM Related
//
extern void VIENNA_HAL_setupPWM(uint32_t base1, uint32_t base2, uint32_t base3,
                           uint16_t pwm_period_ticks);
extern void VIENNA_HAL_configurePWMUpDwnCnt(uint32_t base1,
                                            uint16_t pwm_period_ticks);
extern void setupPWM(uint32_t base1, uint32_t base2, uint32_t base3,
                           uint16_t pwm_period_ticks, uint16_t pwm_dbred_ticks, uint16_t pwm_dbfed_ticks);
extern void configurePWMUpDwnCnt(uint32_t base1, uint16_t pwm_period_ticks, uint16_t pwm_dbred_ticks, uint16_t pwm_dbfed_ticks);

extern void VIENNA_HAL_setPinsAsPWM();
extern void VIENNA_HAL_disablePWMClkCounting(void);
extern void VIENNA_HAL_enablePWMClkCounting(void);
extern void VIENNA_HAL_setupTriggerForADC(uint32_t base);

//
// ADC Related
//
extern void VIENNA_HAL_setupADC(void);
extern void setupADC(void);

//
// OC Protection
//
extern void VIENNA_HAL_setupCMPSS(uint32_t base1, float32_t current_limit,
                                  float32_t current_max_sense );
extern void VIENNA_HAL_setupBoardProtection(uint32_t base1, uint32_t base2,
                                            uint32_t base3,
                          float32_t current_limit, float32_t current_max_sense);
extern void VIENNA_HAL_setupPWMTrip(uint32_t base);

//
// GPIO Related
//
extern void VIENNA_HAL_setupProfilingGPIO(void);
extern void VIENNA_HAL_setupLEDGPIO(void);
extern void VIENNA_HAL_toggleLED(void);

//
// SDFM related
//
extern void VIENNA_HAL_setupSDFM(uint16_t PWM_ticks,
                                 uint16_t PWM_ticks_in_sdfm_osr,
               uint16_t SD_clk_ecap_sys_ticks, uint16_t sdfmosr);
extern void VIENNA_HAL_configureECAPforSDFMClk(uint16_t SD_clk_ecap_sys_ticks);
extern void VIENNA_HAL_configurePWMsyncforSDFM(uint16_t PWM_ticks,
                             uint16_t PWM_ticks_in_sdfm_osr);
extern void VIENNA_HAL_configurePWM1chUpDwnCnt(int16_t n, int16_t period,
                                               int16_t mode,
                             int16_t phase, int16_t db_red,
                             int16_t db_fed, int16_t phase_dir);
extern void VIENNA_HAL_configurePWM1chUpCnt(uint32_t base1, uint16_t period);

extern void VIENNA_HAL_setupCLA(void);

//
// Function Prototypes
//
//CLA C Tasks defined in Cla1Tasks_C.cla
//
__attribute__((interrupt))  void Cla1Task1();
__attribute__((interrupt))  void Cla1Task2();
__attribute__((interrupt))  void Cla1Task3();
__attribute__((interrupt))  void Cla1Task4();
__attribute__((interrupt))  void Cla1Task5();
__attribute__((interrupt))  void Cla1Task6();
__attribute__((interrupt))  void Cla1Task7();

__attribute__((interrupt))  void Cla1BackgroundTask();



extern uint16_t Cla1ProgLoadStart;
extern uint16_t Cla1ProgLoadEnd;
extern uint16_t Cla1ProgLoadSize;
extern uint16_t Cla1ProgRunStart;
extern uint16_t Cla1ProgRunEnd;
extern uint16_t Cla1ProgRunSize;

//
// ISR related
//
#if VIENNA_CONTROL_RUNNING_ON == C28x_CORE

#ifndef __TMS320C28XX_CLA__
    #pragma INTERRUPT (ISR1, HPI)
    #pragma CODE_SECTION(ISR1,"isrcodefuncs");
    interrupt void ISR1(void);
#endif

#else
#endif

#if VIENNA_INSTRUMENTATION_ISR_RUNNING_ON == C28x_CORE

#ifndef __TMS320C28XX_CLA__
    #pragma CODE_SECTION(ISR2,"ramfuncs");
    interrupt void ISR2(void);
#endif

#else
#endif


//
// ************************** Inline functions *******************************
//

//
// PWM Related
//

//
// EPWM_setCounterCompareValueOptimized()
//
static inline void VIENNA_HAL_EPWM_setCounterCompareValueOptimized(
                                 uint32_t base,
                                 EPWM_CounterCompareModule compModule,
                                 uint16_t compCount)
{
    uint32_t registerOffset;

    //
    // Get the register offset for the Counter compare
    //
    registerOffset = EPWM_O_CMPA + (uint16_t)compModule;

    //
    // Write to the counter compare registers.
    //
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    if((compModule == EPWM_COUNTER_COMPARE_A) ||
    (compModule == EPWM_COUNTER_COMPARE_B))
    {
        //
        // write to CMPA or COMPB bits
        //
        HWREGH(base + registerOffset + 1) = compCount;
    }
    else
    {
        //
        // write to COMPC or COMPD bits
        //
        HWREGH(base + registerOffset) = compCount;
    }
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

//
// updateInverterPWM()
//
static inline void VIENNA_HAL_updatePFCviennaPWM(uint32_t base1, uint32_t base2,
                               uint32_t base3, float32_t duty1PU,
                               float32_t duty2PU, float32_t duty3PU)
{
    float32_t pwm_period;
    uint16_t duty1, duty2, duty3;

    pwm_period = (float32_t)(VIENNA_PFC3PH_PWM_PERIOD / 2.0);

    duty1 =  (uint16_t) ( (float32_t) pwm_period * (float32_t) fabsf(duty1PU) );
    duty2 =  (uint16_t) ( (float32_t) pwm_period * (float32_t) fabsf(duty2PU) );
    duty3 =  (uint16_t) ( (float32_t) pwm_period * (float32_t) fabsf(duty3PU) );

    VIENNA_HAL_EPWM_setCounterCompareValueOptimized(base1,
                                                    EPWM_COUNTER_COMPARE_A,
                                                    duty1);

    VIENNA_HAL_EPWM_setCounterCompareValueOptimized(base2,
                                                    EPWM_COUNTER_COMPARE_A,
                                                    duty2);

    VIENNA_HAL_EPWM_setCounterCompareValueOptimized(base3,
                                                    EPWM_COUNTER_COMPARE_A,
                                                    duty3);

}

//
// clearPWMTrip
//
static inline void VIENNA_HAL_clearPWMTripFlags(uint32_t base)
{
    //
    // clear all the configured trip sources for the PWM module
    //
    EPWM_clearTripZoneFlag(base,
                           (
                             EPWM_TZ_INTERRUPT_OST |
                             EPWM_TZ_INTERRUPT_CBC |
                             EPWM_TZ_INTERRUPT_DCAEVT1 |
                             EPWM_TZ_INTERRUPT_DCBEVT1)
                           );
}

//
//  clearCMPSSLatchFlags
//
static inline void VIENNA_HAL_clearCMPSSLatchFlags(uint32_t base)
{
    //
    // also clear the latch of the CMPSS for analog trip path
    //
    CMPSS_clearFilterLatchHigh(base);
    CMPSS_clearFilterLatchLow(base);
}

static inline void VIENNA_HAL_clearOSTPWMTripFlag(uint32_t base)
{
    //
    // clear all the configured trip sources for the PWM module
    //
    EPWM_clearTripZoneFlag(base, EPWM_TZ_INTERRUPT_OST);
}

#pragma FUNC_ALWAYS_INLINE(VIENNA_HAL_forceOSTPWMTrip)

static inline void VIENNA_HAL_forceOSTPWMTrip(uint32_t base)
{
    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_OST);
}

//
// enable PWM Interrupt generation
//
static inline void VIENNA_HAL_enablePWMInterruptGeneration(uint32_t base,
                                                uint16_t cmpc_val)
{
    //
    // write value to CMPC
    //
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_C,
                                cmpc_val);

    EPWM_setInterruptSource(base, EPWM_INT_TBCTR_D_CMPC);
    EPWM_setInterruptEventCount(base, VIENNA_CNTRL_ISR_FREQ_RATIO);
    EPWM_enableInterrupt(base);
    EPWM_clearEventTriggerInterruptFlag(base);
}

//
// clearPWM Interrupt Flag
//
static inline void VIENNA_HAL_clearPWMInterruptFlag(uint32_t base)
{
    EPWM_clearEventTriggerInterruptFlag(base);
}

//
// getPWM Interrupt Flag
//
static inline bool VIENNA_HAL_getPWMInterruptFlag(uint32_t base)
{
    return(EPWM_getEventTriggerInterruptStatus(base));
}

//
// GPIO Related
//

//
// setProfilingGPIO
//
static inline void VIENNA_HAL_setProfilingGPIO(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPASET ) = VIENNA_GPIO_PROFILING1_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

//
// resetProfilingGPIO
//
static inline void VIENNA_HAL_resetProfilingGPIO(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR ) = VIENNA_GPIO_PROFILING1_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

//
// setProfilingGPIO
//
static inline void VIENNA_HAL_setProfilingGPIO2(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPASET ) = VIENNA_GPIO_PROFILING2_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

//
// resetProfilingGPIO
//
static inline void VIENNA_HAL_resetProfilingGPIO2(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR ) = VIENNA_GPIO_PROFILING2_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

// setPowerRelayGPIO
//
static inline void setPOWER_RELAY(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPASET ) = POWER_RELAY_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

// resetPowerRelayGPIO
//
static inline void resetPOWER_RELAY(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR ) = POWER_RELAY_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

// setDisablePWMGPIO
//
static inline void setDISABLE_PWM1(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPASET ) = DISABLE_PWM1_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

// resetDisablePWMGPIO
//
static inline void resetDISABLE_PWM1(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR ) = DISABLE_PWM1_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

// setTimeCheckGPIO
//
static inline void setDISABLE_PWM2(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPASET ) = DISABLE_PWM2_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

// resetTimeCheckGPIO
//
static inline void resetDISABLE_PWM2(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPACLEAR ) = DISABLE_PWM2_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

/*
// ToggleTimeCheckGPIO
//
static inline void toggleTimeCheck(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE + GPIO_O_GPATOGGLE ) = TIME_CHECK_TOGGLE;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}
*/

#ifndef __TMS320C28XX_CLA__

//
// clearInterrupt
//
static inline void VIENNA_HAL_clearInterrupt(uint16_t pie_group_no)
{
    Interrupt_clearACKGroup(pie_group_no);
}

static inline void VIENNA_HAL_setupInterrupt(void)
{
    CPUTimer_enableInterrupt(
            VIENNA_C28x_INSTRUMENTATION_INTERRUPT_TRIG_CPUTIMER_BASE
            );
    CPUTimer_clearOverflowFlag(
            VIENNA_C28x_INSTRUMENTATION_INTERRUPT_TRIG_CPUTIMER_BASE
            );
#if VIENNA_INSTRUMENTATION_ISR_RUNNING_ON == C28x_CORE
    Interrupt_register(VIENNA_C28x_INSTRUMENTATION_INTERRUPT, &ISR2);
    Interrupt_enable(VIENNA_C28x_INSTRUMENTATION_INTERRUPT);
#endif
#if VIENNA_CONTROL_RUNNING_ON == C28x_CORE

    //
    // PWM was already enabled to trigger ISR
    //
    Interrupt_register(VIENNA_C28x_ISR1_INTERRUPT, &ISR1);
    VIENNA_HAL_clearInterrupt(VIENNA_C28x_ISR1_INTERRUPT_PIE_GROUP_NO);
    Interrupt_enable(VIENNA_C28x_ISR1_INTERRUPT);
#endif

#if(VIENNA_CONTROL_RUNNING_ON == CLA_CORE || VIENNA_INSTRUMENTATION_ISR_RUNNING_ON == CLA_CORE)
    VIENNA_HAL_setupCLA();
#endif
    EALLOW;

    //
    // Enable Global interrupt INTM
    //
    EINT;

    //
    // Enable Global realtime interrupt DBGM
    //
    ERTM;
    EDIS;
}

#endif

#ifdef __cplusplus
}
#endif                                  /* extern "C" */


#endif
