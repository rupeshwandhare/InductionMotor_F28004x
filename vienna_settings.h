//#############################################################################
//
// FILE:   vienna_settings.h
//
// TITLE:  vienna_settings.h
//
//#############################################################################
// $TI Release: TIDM_1000 v4.01.01.00 $
// $Release Date: Tue Nov  3 10:37:49 CST 2020 $
// Copyright (C) {2015} Texas Instruments Incorporated - http://www.ti.com/
// * ALL RIGHTS RESERVED*
//#############################################################################

#ifndef _PROJSETTINGS_H
#define _PROJSETTINGS_H

#ifdef __cplusplus

extern "C" {
#endif

//*****************************************************************************
//defines
//*****************************************************************************

//
// Device Related Defines
//
#define VIENNA_CPU_SYS_CLOCK_FREQ_HZ ((float32_t)100*1000000)
#define VIENNA_PWMSYSCLOCK_FREQ_HZ ((float32_t)100*1000000)
#define VIENNA_ECAPSYSCLOCK_FREQ_HZ ((float32_t)100*1000000)

//
// Project Options
//=============================================================================
// Incremental Build options for System check-out
//=============================================================================
// BUILD 1   Open Loop Check
// BUILD 2   Closed Current Loop Check
// BUILD 3   Voltage loop with inner current loop check
// BUILD 4   Balance loop with voltage and current loop closed
//
#define VIENNA_INCR_BUILD 3

#define C28x_CORE 1
#define CLA_CORE 2
#define VIENNA_CONTROL_RUNNING_ON 2

//
// Check system under DC condition (cleanest SFRA curves) 0 is FALSE, 1 is TRUE
//
#define VIENNA_DC_CHECK 0

#define AC_FREQ_HZ       ((float32_t)60)

#if VIENNA_INCR_BUILD == 4
#define VIENNA_INDUCTOR_VOLTAGE_DROP_FEEDFORWARD 1
#define VIENNA_THIRD_HARMONIC_INJECTION 1
#else
#define VIENNA_INDUCTOR_VOLTAGE_DROP_FEEDFORWARD 0
#define VIENNA_THIRD_HARMONIC_INJECTION 0
#endif

#define VIENNA_NON_LINEAR_VOLTAGE_LOOP 0

//
//Power Stage Related Values
//
#define VIENNA_PFC3PH_PWM_SWITCHING_FREQUENCY_HZ ((float32_t)50*1000)
#define VIENNA_PFC3PH_PWM_PERIOD ((VIENNA_PWMSYSCLOCK_FREQ_HZ)/(VIENNA_PFC3PH_PWM_SWITCHING_FREQUENCY_HZ))

#define PWMSYSCLOCK_FREQ    (float32_t)(100 * 1000000)

#define INV_DEADBAND_US              ((float32_t)0.15)           //This statement is checked, it generates delay of xx usec
#define INV_DEADBAND_PWM_COUNT       (int16_t)((float32_t)                \
                                          INV_DEADBAND_US *               \
                                          (float32_t)PWMSYSCLOCK_FREQ *   \
                                          (float32_t)0.000001)

#define VIENNA_VAC_MAX_SENSE_VOLTS ((float32_t)454)
#define VIENNA_VDCBUS_MAX_SENSE_VOLTS ((float32_t)717.79)
#define VIENNA_V_MAX_SENSE_VOLTS ((float32_t)454)

#define VIENNA_I_MAX_SENSE_AMPS ((float32_t)44.75)//((float32_t)12)
#define VIENNA_I_TRIP_LIMIT_AMPS ((float32_t)40) //((float32_t)11)
#define VIENNA_VAC_TYPICAL_VOLTS ((float32_t)208)

//
// Control Loop Design
//
#define VIENNA_CNTRL_ISR_FREQ_RATIO 1
#define VIENNA_VOLTAGE_LOOP_RUN_RATIO   1

#define VIENNA_ISR_CONTROL_FREQUENCY_HZ ((VIENNA_PFC3PH_PWM_SWITCHING_FREQUENCY_HZ) / (VIENNA_CNTRL_ISR_FREQ_RATIO))
#define VIENNA_ISR_10KHZ_FREQUENCY_HZ ((float32_t)10000) / (VIENNA_CNTRL_ISR_FREQ_RATIO)

//
// 3mH
//
#define VIENNA_PFC_INDUCTOR_VALUE ((float32_t)0.003) 

//
//SFRA Options
//1 FRA for the Current Loop
//2 FRA for the Voltage Loop
//3 FRA for the Voltage Balance Loop
//
#define VIENNA_SFRA_CURRENT 1
#define VIENNA_SFRA_VOLTAGE 2
#define VIENNA_SFRA_BALANCECNTL 3
#if VIENNA_INCR_BUILD == 4
#define VIENNA_SFRA_TYPE  VIENNA_SFRA_BALANCECNTL
#elif VIENNA_INCR_BUILD == 3
#define VIENNA_SFRA_TYPE  VIENNA_SFRA_VOLTAGE
#else
#define VIENNA_SFRA_TYPE  VIENNA_SFRA_CURRENT
#endif
#define VIENNA_SFRA_ISR_FREQ_HZ  VIENNA_ISR_CONTROL_FREQUENCY_HZ

#define PI ((float32_t)3.141592653589)

#define VIENNA_GI_GAINKP ((float32_t)2.0000000000)

#define VIENNA_GV_PI_KP ((float32_t) 0.3999748673)
#define VIENNA_GV_PI_KI ((float32_t) 0.0001256716)

#define VIENNA_GS_GAINKP ((float32_t)1.0000000000)

#define VIENNA_VBUS_REF_SET_VOLTS ((float32_t)600)

//=============================================================================
// User code settings file
//=============================================================================
#include "vienna_user_settings.h"

#ifdef __cplusplus
}
#endif                                  /* extern "C" */

#endif //_PROJSETTINGS_H
