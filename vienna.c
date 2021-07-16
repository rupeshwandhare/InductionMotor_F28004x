//#############################################################################
//
// FILE:  vienna.c
//
// TITLE: This is the solution file.
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

//
//********************* the includes ******************************************
//

#include "vienna.h"

//
//*********************** C2000 SYSTEM SW FRAMEWORK  **************************
//

extern struct COMMON_FLAG common_flag;
extern struct COMMAND command;
extern uint16_t Screen_count;

//
//*********************** globals *********************************************
//

//
// Control Variables
//

#pragma SET_DATA_SECTION("controlVariables")

/*
//
//--- System Related ---
// DCL Library, voltage compensator
// DC Bus set point
//
volatile float32_t VIENNA_vBusRef_pu;
volatile float32_t VIENNA_vBusRefSlewed_pu;

VIENNA_GV VIENNA_gv;

volatile float32_t VIENNA_gv_out, VIENNA_voltage_error_pu;
volatile uint16_t VIENNA_nonLinearVoltageLoopFlag;

//
// Current compensator
// Peak value of the Ii, Io current set point
//
volatile float32_t VIENNA_iLRef_pu, VIENNA_iL1Ref_pu, VIENNA_iL2Ref_pu, VIENNA_iL3Ref_pu;
volatile float32_t VIENNA_gi_out1, VIENNA_gi_out2, VIENNA_gi_out3;
volatile float32_t VIENNA_gi_gainKp;

//
// Balance controller
//
volatile float32_t VIENNA_gs_gainKp;
volatile float32_t VIENNA_gs_out;

volatile float32_t VIENNA_vBusDiff_pu, VIENNA_vBusZero_pu;
*/

/*
//
// Sine analyzer block for RMS Volt, Curr and Power measurements
//
POWER_MEAS_SINE_ANALYZER VIENNA_sine_mains1, VIENNA_sine_mains2,
                         VIENNA_sine_mains3;
*/

//
// Measurement Variables
// Inductor Current Measurement
//
volatile float32_t VIENNA_iL1MeasADC_pu, VIENNA_iL2MeasADC_pu, VIENNA_iL3MeasADC_pu;
volatile float32_t VIENNA_iL1MeasSD_pu, VIENNA_iL2MeasSD_pu, VIENNA_iL3MeasSD_pu;
//volatile float32_t VIENNA_iL1Meas_pu, VIENNA_iL2Meas_pu, VIENNA_iL3Meas_pu;
volatile float32_t VIENNA_iL1Meas_pu, VIENNA_iL2Meas_pu, VIENNA_iL3Meas_pu, VIENNA_iPV1Meas_pu, VIENNA_iPV2Meas_pu, VIENNA_vPV1Meas_pu, VIENNA_vPV2Meas_pu, VIENNA_vDCMeas_pu, VIENNA_vCCMeas_pu, VIENNA_TEMPMeas_pu, VIENNA_POTMeas_pu;

//
// Inductor Current Measurement Offset
//
//volatile float32_t VIENNA_iL1MeasOffset_pu, VIENNA_iL2MeasOffset_pu,
//                   VIENNA_iL3MeasOffset_pu;
volatile float32_t VIENNA_iL1MeasOffset_pu, VIENNA_iL2MeasOffset_pu, VIENNA_iL3MeasOffset_pu,
                   VIENNA_iPV1MeasOffset_pu, VIENNA_iPV2MeasOffset_pu, VIENNA_vPV1MeasOffset_pu, VIENNA_vPV2MeasOffset_pu, VIENNA_vDCMeasOffset_pu;


/*
//
// Output Voltage Bus measurement
//
volatile float32_t VIENNA_vBusMNMeas_pu, VIENNA_vBusPMMeas_pu, VIENNA_vBusMeas_pu,
                   VIENNA_vBusHalfMeas_pu;
volatile float32_t VIENNA_vBusMNMeasAvg_pu, VIENNA_vBusPMMeasAvg_pu,
                   VIENNA_vBusMeasAvg_pu;

//
// variables used for calibration of output voltage measurements
//
volatile float32_t VIENNA_m_VBusMNMeas_pu, VIENNA_b_VBusMNMeas_pu;
volatile float32_t VIENNA_m_VBusPMMeas_pu, VIENNA_b_VBusPMMeas_pu;

//
// Input Grid Voltage Measurement
//
volatile float32_t VIENNA_v1Meas_pu, VIENNA_v2Meas_pu, VIENNA_v3Meas_pu;
volatile float32_t VIENNA_v1MeasOffset_pu, VIENNA_v2MeasOffset_pu,
                   VIENNA_v3MeasOffset_pu;

volatile float32_t VIENNA_vRmsMeasAvg_pu;
*/

/*
//
// Display Values
//
volatile float32_t  VIENNA_guiVbusMN_Volts, VIENNA_guiVbusPM_Volts, VIENNA_guiVbus_Volts,
                VIENNA_guiV1_Volts, VIENNA_guiV2_Volts, VIENNA_guiV3_Volts,
                VIENNA_guiIL1_Amps, VIENNA_guiIL2_Amps, VIENNA_guiIL3_Amps,
                VIENNA_guiIL1sd_Amps, VIENNA_guiIL2sd_Amps, VIENNA_guiIL3sd_Amps;

volatile float32_t VIENNA_guiACFreq_Hz;
volatile float32_t VIENNA_guiPrms1_W, VIENNA_guiPrms2_W, VIENNA_guiPrms3_W,
                   VIENNA_guiPrmsTotal_W;
volatile float32_t VIENNA_guiIrms1_Amps, VIENNA_guiIrms2_Amps, VIENNA_guiIrms3_Amps;
volatile float32_t VIENNA_guiVrms1_Volts, VIENNA_guiVrms2_Volts, VIENNA_guiVrms3_Volts;
volatile float32_t VIENNA_guiPF1, VIENNA_guiPF2, VIENNA_guiPF3;
volatile float32_t VIENNA_guiVA1_VA, VIENNA_guiVA2_VA, VIENNA_guiVA3_VA;

float32_t VIENNA_guiVbusTripLimit_Volts;

uint16_t VIENNA_guiPowerStageStart;
uint16_t VIENNA_guiPowerStageStop;
*/

/*
//
// PFC Filtered DC bus measurement
//
volatile float32_t VIENNA_vBusAvg_pu;

volatile float32_t VIENNA_iL1_CalibrationGain = 1.0; //0.95999979;
volatile float32_t VIENNA_iL2_CalibrationGain = 1.0;
volatile float32_t VIENNA_iL3_CalibrationGain = 1.0; // 0.9850000;

//
// variables for third harmonic injection
//
volatile float32_t VIENNA_vMin_pu, VIENNA_vMax_pu;
volatile float32_t VIENNA_thirdHarmonicInjection;
*/

/*
//
// individual duty cycles for each phase
//
volatile float32_t VIENNA_duty1PU, VIENNA_duty2PU, VIENNA_duty3PU;
volatile float32_t VIENNA_dutyPU_DC;

//
// Flags for clearing trips and closing the loops
//
int16_t VIENNA_closeGiLoop, VIENNA_closeGvLoop, VIENNA_closeGsLoop,
        VIENNA_clearTrip, VIENNA_firstTimeGvLoop;

volatile int VIENNA_updateCoeff = 0;

volatile int16_t VIENNA_thirdHarmonicInjectionEnable = 1;

volatile float32_t VIENNA_iL1Ref_prev_pu, VIENNA_iL2Ref_prev_pu, VIENNA_iL3Ref_prev_pu;
volatile float32_t VIENNA_inductor_voltage_drop_feedforward1,
        VIENNA_inductor_voltage_drop_feedforward2,
        VIENNA_inductor_voltage_drop_feedforward3;

volatile uint16_t VIENNA_busVoltageSlew_pu = 0;
*/

//
//-----------------------------------------------------------------------------
// Enum for build level of software and board status
//
enum VIENNA_BuildLevel_enum VIENNA_buildInfo = BuildLevel_1_OpenLoop ;

enum VIENNA_boardState_enum VIENNA_boardState = PowerStageOFF;

//enum VIENNA_boardStatus_enum VIENNA_boardStatus = boardStatus_NoFault;//boardStatus_Idle;

volatile uint16_t VIENNA_boardStatus=1;

volatile uint16_t CONTROL_STATE=0;
volatile uint16_t CONTROL_STATE2=0;

//---
//struct COMMON_VARS {
    volatile float32_t common_vars_vpv1_ref;
    volatile float32_t common_vars_vpv2_ref;
    volatile float32_t common_vars_duty1;
    volatile float32_t common_vars_duty2;
    volatile double xx;
//};
//#define COMMON_VARS_DEFAULTS {0,0,0,0}

//
//struct COMMON_FLAG {
    volatile uint16_t common_flag_clearTrip;
    volatile uint16_t common_flag_init_GlobalVariable;
//};
//#define COMMON_FLAG_DEFAULTS {0}

//struct IL1_CONTROL_VARS {
    volatile float32_t il1_control_ref;
    volatile float32_t il1_control_act;
    volatile float32_t il1_control_error;
    volatile float32_t il1_control_error_prev;
    volatile float32_t il1_control_PIout;
    volatile float32_t il1_control_feedforward;
    volatile float32_t il1_control_controlout;
//};
//#define IL1_CONTROL_VARS_DEFAULTS {0,0,0,0,0,0,0}

//struct IL2_CONTROL_VARS {
    volatile float32_t il2_control_ref;
    volatile float32_t il2_control_act;
    volatile float32_t il2_control_error;
    volatile float32_t il2_control_error_prev;
    volatile float32_t il2_control_PIout;
    volatile float32_t il2_control_feedforward;
    volatile float32_t il2_control_controlout;
//};
//#define IL2_CONTROL_VARS_DEFAULTS {0,0,0,0,0,0,0}

//struct VPV1_CONTROL_VARS {
    volatile float32_t vpv1_control_ref;
    volatile float32_t vpv1_control_act;
    volatile float32_t vpv1_control_error;
    volatile float32_t vpv1_control_error_prev;
    volatile float32_t vpv1_control_PIout;
    volatile float32_t vpv1_control_feedforward;
    volatile float32_t vpv1_control_controlout;
//};
//#define VPV1_CONTROL_VARS_DEFAULTS {0,0,0,0,0,0,0}

//struct VPV2_CONTROL_VARS {
    volatile float32_t vpv2_control_ref;
    volatile float32_t vpv2_control_act;
    volatile float32_t vpv2_control_error;
    volatile float32_t vpv2_control_error_prev;
    volatile float32_t vpv2_control_PIout;
    volatile float32_t vpv2_control_feedforward;
    volatile float32_t vpv2_control_controlout;
//};
//#define VPV2_CONTROL_VARS_DEFAULTS {0,0,0,0,0,0,0}

//struct UPDATE_CONST {
    volatile float32_t update_const_k1_PIiL1;
    volatile float32_t update_const_k2_PIiL1;
    volatile float32_t update_const_k1_PIiL2;
    volatile float32_t update_const_k2_PIiL2;
    volatile float32_t update_const_upper_limit_il1_PIout;
    volatile float32_t update_const_lower_limit_il1_PIout;
    volatile float32_t update_const_upper_limit_il2_PIout;
    volatile float32_t update_const_lower_limit_il2_PIout;
    volatile float32_t update_const_upper_limit_il1_controlout;
    volatile float32_t update_const_lower_limit_il1_controlout;
    volatile float32_t update_const_upper_limit_il2_controlout;
    volatile float32_t update_const_lower_limit_il2_controlout;
    volatile float32_t update_const_k1_PIvpv1;
    volatile float32_t update_const_k2_PIvpv1;
    volatile float32_t update_const_k1_PIvpv2;
    volatile float32_t update_const_k2_PIvpv2;
    volatile float32_t update_const_upper_limit_vpv1_PIout;
    volatile float32_t update_const_lower_limit_vpv1_PIout;
    volatile float32_t update_const_upper_limit_vpv2_PIout;
    volatile float32_t update_const_lower_limit_vpv2_PIout;
    volatile float32_t update_const_upper_limit_vpv1_controlout;
    volatile float32_t update_const_lower_limit_vpv1_controlout;
    volatile float32_t update_const_upper_limit_vpv2_controlout;
    volatile float32_t update_const_lower_limit_vpv2_controlout;
    volatile float32_t update_const_ipv1_ref_isc;
    volatile float32_t update_const_vpv1_ref_vsc;
    volatile float32_t update_const_vpv1_ref_voc;
    volatile float32_t update_const_ipv1_ref_imp;
    volatile float32_t update_const_slop1_vpv1_ref_gen;
    volatile float32_t update_const_constant1_vpv1_ref_gen;
    volatile float32_t update_const_slop2_vpv1_ref_gen;
    volatile float32_t update_const_constant2_vpv1_ref_gen;
    volatile float32_t update_const_ipv2_ref_isc;
    volatile float32_t update_const_vpv2_ref_vsc;
    volatile float32_t update_const_vpv2_ref_voc;
    volatile float32_t update_const_ipv2_ref_imp;
    volatile float32_t update_const_slop1_vpv2_ref_gen;
    volatile float32_t update_const_constant1_vpv2_ref_gen;
    volatile float32_t update_const_slop2_vpv2_ref_gen;
    volatile float32_t update_const_constant2_vpv2_ref_gen;
    volatile float32_t update_const_slew_rate1;
    volatile float32_t update_const_vpv1_threshold_slew_to_vpvcontrol;
    volatile float32_t update_const_duty1_threshold_slew_to_vpvcontrol;
    volatile float32_t update_const_slew_rate2;
    volatile float32_t update_const_vpv2_threshold_slew_to_vpvcontrol;
    volatile float32_t update_const_duty2_threshold_slew_to_vpvcontrol;
    volatile float32_t update_const_vdc_threshold_charging_to_idle;
    volatile float32_t update_const_deacceleration_rate;
    volatile float32_t update_const_upper_duty_threshold;


    volatile float32_t update_const_vpv_overvoltage_threshold;

//};
//#define UPDATE_CONST_DEFAULTS {31.44,   31.39,  31.44,  31.39,  100,    -100,   100,    -100,   0.7,    0.01,   0.7,    0.01,   0.2364, 0.2349, 0.2364, 0.2349, 25, -25,    25, -25,    25, -25,    25, -25,    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0.00002,    112,    0.2,    0.00002,    112,    0.2,    500,    0.00002,    0}

//struct SENSOR {
    volatile float32_t sensor_v_pv1;
    volatile float32_t sensor_v_pv2;
    volatile float32_t sensor_i_pv1;
    volatile float32_t sensor_i_pv2;
    volatile float32_t sensor_v_dc;
    volatile float32_t sensor_i_dc;
    volatile float32_t sensor_i_L1;
    volatile float32_t sensor_i_L2;
    volatile float32_t sensor_v_dc_fltr;
    volatile float32_t sensor_v_dc_prev;
    volatile float32_t sensor_v_pv1_fltr;
    volatile float32_t sensor_v_pv2_fltr;
    volatile float32_t sensor_v_pv1_prev;
    volatile float32_t sensor_v_pv2_prev;
    volatile float32_t sensor_i_pv1_fltr;
    volatile float32_t sensor_i_pv2_fltr;
    volatile float32_t sensor_i_pv1_prev;
    volatile float32_t sensor_i_pv2_prev;
    volatile float32_t sensor_i_L1_fltr;
    volatile float32_t sensor_i_L2_fltr;
    volatile float32_t sensor_i_L1_prev;
    volatile float32_t sensor_i_L2_prev;
    volatile float32_t sensor_temperature;
    volatile float32_t sensor_temperature_fltr;
    volatile float32_t sensor_temperature_prev;
    volatile float32_t sensor_board_temp;
    volatile float32_t sensor_board_temp_fltr;
    volatile float32_t sensor_board_temp_prev;
    volatile float32_t sensor_ref_vcc;
    volatile float32_t sensor_ref_vcc_fltr;
    volatile float32_t sensor_ref_vcc_prev;
    volatile float32_t sensor_zero_current_offset;
    volatile float32_t sensor_ppv;
    volatile float32_t sensor_ppv1;
    volatile float32_t sensor_ppv2;
//};

//#define SENSOR_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

    //
//    struct COMMAND {
    volatile uint16_t command_OnOffCh1;
    volatile uint16_t command_OnOffCh2;
    volatile uint16_t command_Clear_OCfault;
//    };
//#define COMMAND_DEFAULTS {0,0,0}

    volatile uint32_t count_charging4;
    volatile uint32_t testing_variable;

volatile float32_t Select_PVcurve1=0;
volatile float32_t Select_PVcurve2=0;
volatile float32_t vpv_ref1;
volatile float32_t Temp1;
volatile float32_t BetaV1;
volatile float32_t Ns1;
volatile float32_t Np1;
volatile float32_t ILight1;
volatile float32_t ipv1;
volatile float32_t Np1;
volatile float32_t Io1;
volatile float32_t Rs1;
volatile float32_t Nscell1;
volatile float32_t ppv_ref1;
volatile float32_t vpv_ref2;
volatile float32_t Temp2;
volatile float32_t BetaV2;
volatile float32_t Ns2;
volatile float32_t Np2;
volatile float32_t ILight2;
volatile float32_t ipv2;
volatile float32_t Np2;
volatile float32_t Io2;
volatile float32_t Rs2;
volatile float32_t Nscell2;
volatile float32_t ppv_ref2;
volatile float32_t vpv_ref1_fltr_prev;
volatile float32_t vpv_ref1_fltr;
volatile float32_t vpv_ref2_fltr_prev;
volatile float32_t vpv_ref2_fltr;
volatile float32_t k1_Fltr_vpv1n2;
volatile float32_t k2_Fltr_vpv1n2;
//---

#pragma SET_DATA_SECTION()

//
// datalogger
//
DLOG_4CH VIENNA_dLog1;
float32_t VIENNA_dBuff1[100], VIENNA_dBuff2[100], VIENNA_dBuff3[100],
          VIENNA_dBuff4[100];
float32_t VIENNA_dVal1, VIENNA_dVal2, VIENNA_dVal3, VIENNA_dVal4;

//
//--- SFRA Related Variables ----
//
float32_t VIENNA_plantMagVect[VIENNA_SFRA_FREQ_LENGTH];
float32_t VIENNA_plantPhaseVect[VIENNA_SFRA_FREQ_LENGTH];
float32_t VIENNA_olMagVect[VIENNA_SFRA_FREQ_LENGTH];
float32_t VIENNA_olPhaseVect[VIENNA_SFRA_FREQ_LENGTH];
float32_t VIENNA_freqVect[VIENNA_SFRA_FREQ_LENGTH];

SFRA_F32 VIENNA_sfra1;

//
// Variables used to calibrate measurement offsets
//Offset filter coefficient K1: 0.05/(T+0.05);
//
float32_t VIENNA_k1 = 0.998;

//
//Offset filter coefficient K2: T/(T+0.05)
//
float32_t VIENNA_k2 = 0.001999;
int16_t VIENNA_offsetCalCounter;
float32_t VIENNA_offset165;

//
// Stand Alone Flash Image Instrumentation
//
int16_t VIENNA_i;
int16_t VIENNA_timer1;

//
// updateBoardStatus()
//
void VIENNA_updateBoardStatus(void)
{
#if VIENNA_INCR_BUILD == 1
    #if VIENNA_CONTROL_RUNNING_ON == C28x_CORE
        VIENNA_buildInfo = BuildLevel_1_OpenLoop;
    #else
        VIENNA_buildInfo = BuildLevel_1_OpenLoop_CLA;
    #endif
#elif VIENNA_INCR_BUILD == 2
    #if VIENNA_CONTROL_RUNNING_ON == C28x_CORE
        VIENNA_buildInfo = BuildLevel_2_CurrentLoop;
    #else
        VIENNA_buildInfo = BuildLevel_2_CurrentLoop_CLA;
    #endif
#elif VIENNA_INCR_BUILD == 3
    #if VIENNA_CONTROL_RUNNING_ON == C28x_CORE
        VIENNA_buildInfo = BuildLevel_3_VoltageAndCurrentLoop;
    #else
        VIENNA_buildInfo = BuildLevel_3_VoltageAndCurrentLoop_CLA;
    #endif
#elif VIENNA_INCR_BUILD == 4
    #if VIENNA_CONTROL_RUNNING_ON == C28x_CORE
        VIENNA_buildInfo = BuildLevel_4_BalanceVoltageAndCurrentLoop;
    #else
        VIENNA_buildInfo = BuildLevel_4_BalanceVoltageAndCurrentLoop_CLA;
    #endif
#endif
}

void VIENNA_runSFRABackGroundTasks(void)
{

    SFRA_F32_runBackgroundTask(&VIENNA_sfra1);
    SFRA_GUI_runSerialHostComms(&VIENNA_sfra1);

}

/*
//
// setupSFRA
//
void VIENNA_setupSFRA(void)
{
    SFRA_F32_reset(&VIENNA_sfra1);
    SFRA_F32_config(&VIENNA_sfra1,
                    VIENNA_SFRA_ISR_FREQ_HZ,
                    VIENNA_SFRA_AMPLITUDE,
                    VIENNA_SFRA_FREQ_LENGTH,
                    VIENNA_SFRA_FREQ_START,
                    VIENNA_SFRA_FREQ_STEP_MULTIPLY,
                    VIENNA_plantMagVect,
                    VIENNA_plantPhaseVect,
                    VIENNA_olMagVect,
                    VIENNA_olPhaseVect,
                    NULL,
                    NULL,
                    VIENNA_freqVect,
                    1);

    SFRA_F32_resetFreqRespArray(&VIENNA_sfra1);

    SFRA_F32_initFreqArrayWithLogSteps(&VIENNA_sfra1,
                                       VIENNA_SFRA_FREQ_START,
                                       VIENNA_SFRA_FREQ_STEP_MULTIPLY);

    //
    //configures the SCI channel for communication with SFRA host GUI
    //to change SCI channel change #defines in the settings.h file
    //the GUI also changes a LED status, this can also be changed with #define
    //in the file pointed to above
    //
    SFRA_GUI_config(VIENNA_SFRA_GUI_SCI_BASE,
                    VIENNA_SCI_VBUS_CLK,
                    VIENNA_SFRA_GUI_SCI_BAUDRATE,
                    VIENNA_SFRA_GUI_SCIRX_GPIO,
                    VIENNA_SFRA_GUI_SCIRX_GPIO_PIN_CONFIG,
                    VIENNA_SFRA_GUI_SCITX_GPIO,
                    VIENNA_SFRA_GUI_SCITX_GPIO_PIN_CONFIG,
                    VIENNA_SFRA_GUI_LED_INDICATOR,
                    VIENNA_SFRA_GUI_LED_GPIO,
                    VIENNA_SFRA_GUI_LED_GPIO_PIN_CONFIG,
                    &VIENNA_sfra1,
                    1);

}
*/
//
//===========================================================================
// No more.
//===========================================================================
//

/*
//
// globalVariablesInit()
//
void VIENNA_globalVariablesInit(void)
{

    VIENNA_vBusRef_pu = 0;
    VIENNA_vBusRefSlewed_pu = 0;

    VIENNA_gv.Ki = VIENNA_GV_PI_KI;
    VIENNA_gv.Kp = VIENNA_GV_PI_KP;
    VIENNA_gv.Umax = VIENNA_GV_PI_MAX;
    VIENNA_gv.Umin = VIENNA_GV_PI_MIN;
    VIENNA_gv.i10 = 0;
    VIENNA_gv.i6 = 0;

    VIENNA_gv_out = 0;
    VIENNA_voltage_error_pu = 0;
    #if VIENNA_NON_LINEAR_VOLTAGE_LOOP == 1
        VIENNA_nonLinearVoltageLoopFlag = 1;
    #else
        VIENNA_nonLinearVoltageLoopFlag = 0;
    #endif

    VIENNA_gi_out1 = 0;
    VIENNA_gi_out2 = 0;
    VIENNA_gi_out3 = 0;
    VIENNA_gi_gainKp = VIENNA_GI_GAINKP;

    VIENNA_gs_gainKp = VIENNA_GS_GAINKP;

    //
    //sine analyzer initialization
    //
    POWER_MEAS_SINE_ANALYZER_reset(&VIENNA_sine_mains1);
    POWER_MEAS_SINE_ANALYZER_config(&VIENNA_sine_mains1,
                                    VIENNA_ISR_10KHZ_FREQUENCY_HZ,
                                    (float32_t)0.08,
                                    (float32_t)VIENNA_GRID_MAX_FREQ_HZ,
                                    (float32_t)VIENNA_GRID_MIN_FREQ_HZ);

    POWER_MEAS_SINE_ANALYZER_reset(&VIENNA_sine_mains2);
    POWER_MEAS_SINE_ANALYZER_config(&VIENNA_sine_mains2,
                                    VIENNA_ISR_10KHZ_FREQUENCY_HZ,
                                    (float32_t)0.08,
                                    (float32_t)VIENNA_GRID_MAX_FREQ_HZ,
                                    (float32_t)VIENNA_GRID_MIN_FREQ_HZ);

    POWER_MEAS_SINE_ANALYZER_reset(&VIENNA_sine_mains3);
    POWER_MEAS_SINE_ANALYZER_config(&VIENNA_sine_mains3,
                                    VIENNA_ISR_10KHZ_FREQUENCY_HZ,
                                    (float32_t)0.08,
                                    (float32_t)VIENNA_GRID_MAX_FREQ_HZ,
                                    (float32_t)VIENNA_GRID_MIN_FREQ_HZ);


    DLOG_4CH_reset(&VIENNA_dLog1);
    DLOG_4CH_config(&VIENNA_dLog1,
                    &VIENNA_dVal1,&VIENNA_dVal2,&VIENNA_dVal3,&VIENNA_dVal4,
                    VIENNA_dBuff1, VIENNA_dBuff2, VIENNA_dBuff3, VIENNA_dBuff4,
                    100, 0.05, 5);

    VIENNA_iL1MeasOffset_pu = 0.0f;
    VIENNA_iL2MeasOffset_pu = 0.0f;
    VIENNA_iL3MeasOffset_pu = 0.0f;

    VIENNA_vBusPMMeasAvg_pu = 0;
    VIENNA_vBusMNMeasAvg_pu = 0;
    VIENNA_vBusMeasAvg_pu = 0;

    VIENNA_m_VBusPMMeas_pu = 0.97495237f;
    VIENNA_b_VBusPMMeas_pu = 0.0002906f;

    VIENNA_m_VBusMNMeas_pu = 0.977087472f;
    VIENNA_b_VBusMNMeas_pu = 0.003299208f;

    VIENNA_offsetCalCounter = 0;

    VIENNA_guiPowerStageStart = 0;
    VIENNA_guiPowerStageStop = 0;

    VIENNA_vRmsMeasAvg_pu = 0.0f;
    VIENNA_vBusAvg_pu = 0;

    VIENNA_iL1Ref_pu = 0;
    VIENNA_iL2Ref_pu = 0;
    VIENNA_iL3Ref_pu = 0;
    VIENNA_iLRef_pu = 0.05f;

    VIENNA_closeGiLoop = 0;
    VIENNA_closeGsLoop = 0;
    VIENNA_closeGvLoop = 0;
    VIENNA_clearTrip = 0;

    VIENNA_duty1PU = 0.0f;
    VIENNA_duty2PU = 0.0f;
    VIENNA_duty3PU = 0.0f;

    VIENNA_dutyPU_DC = 0.5f;

    VIENNA_iL1Ref_prev_pu = 0;
    VIENNA_iL2Ref_prev_pu = 0;
    VIENNA_iL3Ref_prev_pu = 0;

    VIENNA_firstTimeGvLoop = 1;
    VIENNA_closeGvLoop = 0;
    VIENNA_closeGiLoop = 0;
    VIENNA_closeGsLoop = 0;
    VIENNA_vBusRef_pu = ((float32_t)VIENNA_VBUS_REF_SET_VOLTS /
                   (float32_t)VIENNA_V_MAX_SENSE_VOLTS );

    VIENNA_vBusZero_pu = 0.0f;

    VIENNA_guiVbusTripLimit_Volts = VIENNA_VBUS_TRIP_LIMIT_VOLTS;

}
*/

void initGlobalVariable(void)
{
    Screen_count = 1;
/*
    common_flag.clearTrip = 1;
    command.Clear_OCfault = 0;
    command.OnOffCh1=OFF;
    command.OnOffCh2=OFF;;
*/


    common_vars_vpv1_ref = 0;
    common_vars_vpv2_ref = 0;
    common_vars_duty1 = 0;
    common_vars_duty2 = 0;

    common_flag_clearTrip = 0;
//    common_flag_init_GlobalVariable = 0;

    il1_control_ref = 0;
    il1_control_act = 0;
    il1_control_error = 0;
    il1_control_error_prev = 0;
    il1_control_PIout = 0;
    il1_control_feedforward = 0;
    il1_control_controlout = 0;

    il2_control_ref = 0;
    il2_control_act = 0;
    il2_control_error = 0;
    il2_control_error_prev = 0;
    il2_control_PIout = 0;
    il2_control_feedforward = 0;
    il2_control_controlout = 0;

    vpv1_control_ref = 0;
    vpv1_control_act = 0;
    vpv1_control_error = 0;
    vpv1_control_error_prev = 0;
    vpv1_control_PIout = 0;
    vpv1_control_feedforward = 0;
    vpv1_control_controlout = 0;

    vpv2_control_ref = 0;
    vpv2_control_act = 0;
    vpv2_control_error = 0;
    vpv2_control_error_prev = 0;
    vpv2_control_PIout = 0;
    vpv2_control_feedforward = 0;
    vpv2_control_controlout = 0;

    update_const_k1_PIiL1 = 31.44;
    update_const_k2_PIiL1 = 31.39;
    update_const_k1_PIiL2 = 31.44;
    update_const_k2_PIiL2 = 31.39;
    update_const_upper_limit_il1_PIout = 100;
    update_const_lower_limit_il1_PIout = -100;
    update_const_upper_limit_il2_PIout = 100;
    update_const_lower_limit_il2_PIout = -100;
    update_const_upper_limit_il1_controlout = 0.7;
    update_const_lower_limit_il1_controlout = 0.01;
    update_const_upper_limit_il2_controlout = 0.7;
    update_const_lower_limit_il2_controlout = 0.01;
    update_const_k1_PIvpv1 = 0.2364;
    update_const_k2_PIvpv1 = 0.2349;
    update_const_k1_PIvpv2 = 0.2364;
    update_const_k2_PIvpv2 = 0.2349;
    update_const_upper_limit_vpv1_PIout = 25;
    update_const_lower_limit_vpv1_PIout = -25;
    update_const_upper_limit_vpv2_PIout = 25;
    update_const_lower_limit_vpv2_PIout = -25;
    update_const_upper_limit_vpv1_controlout = 25;
    update_const_lower_limit_vpv1_controlout = -25;
    update_const_upper_limit_vpv2_controlout = 25;
    update_const_lower_limit_vpv2_controlout = -25;
    update_const_ipv1_ref_isc = 0;
    update_const_vpv1_ref_vsc = 0;
    update_const_vpv1_ref_voc = 0;
    update_const_ipv1_ref_imp = 0;
    update_const_slop1_vpv1_ref_gen = 0;
    update_const_constant1_vpv1_ref_gen = 0;
    update_const_slop2_vpv1_ref_gen = 0;
    update_const_constant2_vpv1_ref_gen = 0;
    update_const_ipv2_ref_isc = 0;
    update_const_vpv2_ref_vsc = 0;
    update_const_vpv2_ref_voc = 0;
    update_const_ipv2_ref_imp = 0;
    update_const_slop1_vpv2_ref_gen = 0;
    update_const_constant1_vpv2_ref_gen = 0;
    update_const_slop2_vpv2_ref_gen = 0;
    update_const_constant2_vpv2_ref_gen = 0;
    update_const_slew_rate1 = 0.00002;
    update_const_vpv1_threshold_slew_to_vpvcontrol = 112;
    update_const_duty1_threshold_slew_to_vpvcontrol = 0.2;
    update_const_slew_rate2 = 0.00002;
    update_const_vpv2_threshold_slew_to_vpvcontrol = 112;
    update_const_duty2_threshold_slew_to_vpvcontrol = 0.2;
    update_const_vdc_threshold_charging_to_idle = 500;
    update_const_deacceleration_rate = 0.00002;
    update_const_upper_duty_threshold = 0;


    update_const_vpv_overvoltage_threshold = 400.0;

    sensor_v_pv1 = 0;
    sensor_v_pv2 = 0;
    sensor_i_pv1 = 0;
    sensor_i_pv2 = 0;
    sensor_v_dc = 0;
    sensor_i_dc = 0;
    sensor_i_L1 = 0;
    sensor_i_L2 = 0;
    sensor_v_dc_fltr = 0;
    sensor_v_dc_prev = 0;
    sensor_v_pv1_fltr = 0;
    sensor_v_pv2_fltr = 0;
    sensor_v_pv1_prev = 0;
    sensor_v_pv2_prev = 0;
    sensor_i_pv1_fltr = 0;
    sensor_i_pv2_fltr = 0;
    sensor_i_pv1_prev = 0;
    sensor_i_pv2_prev = 0;
    sensor_i_L1_fltr = 0;
    sensor_i_L2_fltr = 0;
    sensor_i_L1_prev = 0;
    sensor_i_L2_prev = 0;
    sensor_temperature = 0;
    sensor_temperature_fltr = 0;
    sensor_temperature_prev = 0;
    sensor_board_temp = 0;
    sensor_board_temp_fltr = 0;
    sensor_board_temp_prev = 0;
    sensor_ref_vcc = 0;
    sensor_ref_vcc_fltr = 0;
    sensor_ref_vcc_prev = 0;
    sensor_zero_current_offset = 0;
    sensor_ppv = 0;
    sensor_ppv1 = 0;
    sensor_ppv2 = 0;

    command_OnOffCh1 = 0;
    command_OnOffCh2 = 0;
    command_Clear_OCfault = 0;

//    CONTROL_STATE = ;
    count_charging4 = 0;

    common_flag_init_GlobalVariable = 0;
 
    Select_PVcurve1 = 2;
    Io1= 0.0000000001; //9.96*10^(-11);
    ILight1=9.04;
    Rs1 = 0.5;//0.219;
    vpv_ref1=0;
    ipv1=0;
    Np1=1;
    Ns1=8;
    BetaV1 = 0.40;
    Temp1 = 30;//50;
    
    Select_PVcurve2 = 0;
    Io2= 0.0000000001; //9.96*10^(-11);
    ILight2=9.04;
    Rs2 = 0.5;//0.219;
    vpv_ref2=0;
    ipv2=0;
    Np2=1;
    Ns2=8;
    BetaV2 = 0.40;
    Temp2 = 30;//50;
    vpv_ref1_fltr_prev = 0;
    vpv_ref2_fltr_prev = 0;

    k1_Fltr_vpv1n2 = 0.99373647;
    k2_Fltr_vpv1n2 = 0.00313176265;
}


//
// calibrateOffset()
//
void VIENNA_calibrateOffset()
{
    int16_t VIENNA_offsetCalCounter = 0;

    VIENNA_offsetCalCounter = 0;
    VIENNA_iL1MeasOffset_pu = 0;
    VIENNA_iL2MeasOffset_pu = 0;
    VIENNA_iPV1MeasOffset_pu = 0;
    VIENNA_iPV2MeasOffset_pu = 0;
/*

    VIENNA_vPV1MeasOffset_pu = 0;
    VIENNA_vPV2MeasOffset_pu = 0;

    VIENNA_vDCMeasOffset_pu = 0;

*/
//    VIENNA_gi_out1 = 0;

    while(VIENNA_offsetCalCounter < 25000)
    {
        if(VIENNA_HAL_getPWMInterruptFlag(
                           VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE) == 1)
        {
            if(VIENNA_offsetCalCounter > 1000)
            {
                //
                // offset of the inductor current sense
                //
                VIENNA_iL1MeasOffset_pu = VIENNA_k1 * (VIENNA_iL1MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_IL1_FB_1 + VIENNA_IL1_FB_2 +
                                VIENNA_IL1_FB_3 + VIENNA_IL1_FB_4 )
                                     * 0.25 * ADC_I_GAIN;
                VIENNA_iL2MeasOffset_pu = VIENNA_k1 * (VIENNA_iL2MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_IL2_FB_1 + VIENNA_IL2_FB_2 +
                                VIENNA_IL2_FB_3 + VIENNA_IL2_FB_4)
                                   * 0.25 * ADC_I_GAIN;
                VIENNA_iPV1MeasOffset_pu = VIENNA_k1 * (VIENNA_iPV1MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_IPV1_FB_1 + VIENNA_IPV1_FB_2 +
                                VIENNA_IPV1_FB_3 + VIENNA_IPV1_FB_4)
                                  * 0.25 * ADC_I_GAIN;
                VIENNA_iPV2MeasOffset_pu = VIENNA_k1 * (VIENNA_iPV2MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_IPV2_FB_1 + VIENNA_IPV2_FB_2 +
                                VIENNA_IPV2_FB_3 + VIENNA_IPV2_FB_4)
                                  * 0.25 * ADC_I_GAIN;

                //
                // offset of the voltage sense
                //
                VIENNA_vPV1MeasOffset_pu = VIENNA_k1 * (VIENNA_vPV1MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_VPV1_FB_1 + VIENNA_VPV1_FB_2 +
                                VIENNA_VPV1_FB_3 + VIENNA_VPV1_FB_4)
                                  * 0.25 * ADC_VPV_GAIN;
                VIENNA_vPV2MeasOffset_pu = VIENNA_k1 * (VIENNA_vPV2MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_VPV2_FB_1 + VIENNA_VPV2_FB_2 +
                                VIENNA_VPV2_FB_3 + VIENNA_VPV2_FB_4)
                                  * 0.25 * ADC_VPV_GAIN;


/*
                VIENNA_vDCMeasOffset_pu = VIENNA_k1 * (VIENNA_vDCMeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_VDC_FB_1 + VIENNA_VDC_FB_2 +
                                VIENNA_VDC_FB_3 + VIENNA_VDC_FB_4)
                                  * 0.25 * ADC_VDC_GAIN;
*/

            }
/*
            VIENNA_vPV1MeasOffset_pu = 0.0;
            VIENNA_vPV2MeasOffset_pu = 0.0;
*/
            VIENNA_vDCMeasOffset_pu = 0.0;

            VIENNA_HAL_clearPWMInterruptFlag(
                                   VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE);
            VIENNA_offsetCalCounter++;
        }
    }

    //
    // NO use using the PPB because of the over sampling
    //

}
