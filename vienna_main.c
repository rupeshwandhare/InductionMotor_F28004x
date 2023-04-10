//#############################################################################
//
// FILE:   vienna_main.c
//
// TITLE: This is the main file for the solution, following is the
//         <solution>.c -> solution source file
//         <solution>.h -> solution header file
//         <solution>_settings.h -> powerSUITE generated settings
//         <solution>_hal.c -> solution hardware abstraction layer
//         <solution>_hal.h -> solution hardware abstraction layer header file
//         <solution>_clatask.cla -> cla task file
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

// the includes
//

#include <hmi.h>

extern struct LCD_VARS lcd;
extern struct COMMON_FLAG common_flag;
extern struct COMMAND command;

//
// Variable declarations for state machine
//
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch

//
// State Machine function prototypes
//------------------------------------
// Alpha states
//
void A0(void);  //state A0
void B0(void);  //state B0

//
// A branch states
//
void A1(void);  //state A1
void A2(void);  //state A2

//
// B branch states
//
void B1(void);  //state B1
void B2(void);  //state B2
void B3(void);  //state B3


uint16_t CLA1mathTablesRunStart, CLA1mathTablesLoadStart;
uint16_t CLA1mathTablesLoadSize;

//=====



void main(void)
{
    //
    // This routine sets up the basic device configuration such as
    // initializing PLL, copying code from FLASH to RAM,
    // this routine will also initialize the CPU timers that are used in
    // the background 1, 2 & 3) task for this system (CPU time)
    VIENNA_HAL_setupDevice();

    configCLAMemory();  //CLA memory pointer for math table from flash to ram; and memory block as CLA master

    init_critical_gpio();   //Initialize GPIO for Power relay, PWM1 and PWM2 disable pins

    init_hmi_gpio();    //LCD, LED, ExternalSwitch, SCI Boot capacitor

    //
    // Stop all PWM mode clock
    VIENNA_HAL_disablePWMClkCounting();

    //
    //sets up the PWMs for the VIENNA PFC
//    VIENNA_HAL_setupPWM(VIENNA_HIGH_FREQ_PWM1_BASE, VIENNA_HIGH_FREQ_PWM2_BASE,  VIENNA_HIGH_FREQ_PWM3_BASE, VIENNA_PFC3PH_PWM_PERIOD);
    setupPWM(VIENNA_HIGH_FREQ_PWM1_BASE, VIENNA_HIGH_FREQ_PWM2_BASE, VIENNA_HIGH_FREQ_PWM3_BASE, VIENNA_PFC3PH_PWM_PERIOD, INV_DEADBAND_PWM_COUNT, INV_DEADBAND_PWM_COUNT);

    //
    // power up ADC on the device
//    VIENNA_HAL_setupADC();
    setupADC(); //Map adc channels and Activate adc-A,B,C modules and

    #if VIENNA_SDFM_SENSING == 1
        VIENNA_HAL_setupSDFM(VIENNA_PFC3PH_PWM_PERIOD, VIENNA_PWM_CLK_IN_SDFM_OSR, VIENNA_SD_CLK_COUNT, VIENNA_SDFM_OSR);
    #endif

    //
    //Profiling GPIO
    //
//    VIENNA_HAL_setupProfilingGPIO();

    //
    //configure LED GPIO
    //
//    VIENNA_HAL_setupLEDGPIO();

    //
    //initialize global variables
    //
//    VIENNA_globalVariablesInit();
    initGlobalVariable();
    //
    // Enable PWM Clock
    VIENNA_HAL_enablePWMClkCounting();

    //
    //setup PMW trigger for the ADC conversions; Rupesh's Notebook/Software2/ADC speed and EPWM and comp trip in c2000 tms320f280049 vienna rectifier
    VIENNA_HAL_setupTriggerForADC(VIENNA_HIGH_FREQ_PWM1_BASE);   //This trigger is spread near the peak of TB counter in sync up and down mode, so that adc reads value near middle of current rise

    #if VIENNA_SDFM_SENSING == 1
        VIENNA_HAL_enablePWMInterruptGeneration( VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE, (float32_t)VIENNA_PWM_CLK_IN_SDFM_OSR * (float32_t)1.5 );
    #else
        VIENNA_HAL_enablePWMInterruptGeneration( VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE, EPWM_getCounterCompareValue(VIENNA_HIGH_FREQ_PWM1_BASE, EPWM_COUNTER_COMPARE_B) );
    #endif

    //
    // Offset Calibration Routine
    // #if VIENNA_SDFM_SENSING == 0
    //  VIENNA_calibrateOffset();
    // #endif
    //
    #if SENSING_OPTION == ADC_BASED_SENSING
        VIENNA_calibrateOffset();  //calibration using EPWM flags before ISR-EPWM trigger
    #endif

    //
    // setup protection and trips for the board, using comparators over the ADC channels
    //
    VIENNA_HAL_setupBoardProtection(VIENNA_HIGH_FREQ_PWM1_BASE,
                                    VIENNA_HIGH_FREQ_PWM2_BASE,
                                    VIENNA_HIGH_FREQ_PWM3_BASE,
                                    VIENNA_I_TRIP_LIMIT_AMPS, VIENNA_I_MAX_SENSE_AMPS);


    InitECapture();  //Initialize capture unit; and gpio pin via Xbar input select for IR based remote control

    CodeSecurity();  //For UID based code protection

    InitializeLCD(); //Initialize LCD

    InitializeSCI(); //For bluetooth or wifi

    VIENNA_HAL_setPinsAsPWM();  //Set GPIO as PWM pins, safe to setup PWM pins, as ADC and comparator based trips are active now, PWM were tripped low in the previous routine

    //
    // ISR Mapping
    //
    VIENNA_HAL_setupInterrupt();

    //
    // Setup SFRA
//    VIENNA_setupSFRA();

    first_time_flash_writting_after_erase();   //First time flash writing after flash erase

    load_flash_constants(); //Load constants from flash memory

    CONTROL_STATE = VDC_CHARGING;  //Initialize DC bus charging state


    // Tasks State-machine initialization
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;

    // IDLE loop. Just sit and loop forever, periodically will branch into
    // A0-A3, B0-B3, C0-C3 tasks
    // Frequency of this branching is set in setupDevice routine
    for(;;)
    {
        //
        // State machine entry & exit point
        //
        (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)

    }
} //END MAIN CODE

//
// ISRs are named by the priority
// ISR1 is the highest priority
// ISR2 has the next highest and so forth
//

//
// control ISR Code
//
#if VIENNA_CONTROL_RUNNING_ON == C28x_CORE
    interrupt void ISR1(void)
    {
        ControlCode_PVEmu();
        //VIENNA_pfcControlCode();
        VIENNA_HAL_clearInterrupt(VIENNA_C28x_ISR1_INTERRUPT_PIE_GROUP_NO);
    }// control ISR Ends Here
#endif

//
// 10Khz ISR Code
//
#if VIENNA_INSTRUMENTATION_ISR_RUNNING_ON == C28x_CORE
    interrupt void ISR2(void)
    {
        protections();
//        VIENNA_instrumentationCode();
     }// 10Khz ISR Ends Here
#endif

//
//=============================================================================
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=============================================================================
//
//
//--------------------------------- FRAME WORK --------------------------------
//
void A0(void)
{
    //
    // loop rate synchronizer for A-tasks
    //
    if(VIENNA_TASKA_CTR_OVFLW_STATUS == 1)  //at 20kHz=50usec
    {
        VIENNA_CLEAR_TASKA_CTR_OVFLW_FLAG;    // clear flag

        //
        // jump to an A Task (A1,A2,A3,...)
        //
        (*A_Task_Ptr)();

//        vTimer0[0]++;           // virtual timer 0, instance 0 (spare)
    }
    Alpha_State_Ptr = &B0;      // Comment out to allow only A tasks
}

void B0(void)
{
    //
    // loop rate synchronizer for B-tasks
    //
    if(VIENNA_TASKB_CTR_OVFLW_STATUS  == 1) //at 2kHz=500usec
    {
        VIENNA_CLEAR_TASKB_CTR_OVFLW_FLAG;                  // clear flag

        //
        // jump to a B Task (B1,B2,B3,...)
        //
        (*B_Task_Ptr)();

//        vTimer1[0]++;           // virtual timer 1, instance 0 (spare)
    }

    //
    // Allow A state tasks
    //
    Alpha_State_Ptr = &A0;
}

//
//=============================================================================
//  A - TASKS (executed in every 1 msec)
//=============================================================================
//

void A1(void)   //2*50us=100us (10kHz)
{
    remote_operation();
//    GpioDataRegs.GPATOGGLE.bit.GPIO26 = 1;
    //    VIENNA_runSFRABackGroundTasks();
    //
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    //
    A_Task_Ptr = &A2;
}

void A2(void)    //2*50us=100us (10kHz)
{
//    protections4();
    //
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    //
    A_Task_Ptr = &A1;
}

//
//=============================================================================
//  B - TASKS (executed in every 5 msec)
//=============================================================================
//

void B1(void)    //3*500usec
{
    LoopforLCDPtr();  //in 1 millisencond subroutine
    if (lcd.LCD_ACTION==LCD_NO_ACTION) StateControl_RemoteScreen();

//    VIENNA_updateBoardStatus();

    //
    //the next time CpuTimer1 'counter' reaches Period value go to B2
    //
    B_Task_Ptr = &B2;
}

void B2(void)   //3*500usec
{
    remote_key_display_state_machine();

    //    VIENNA_HAL_toggleLED();

    //
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    //
    B_Task_Ptr = &B3;

}
Uint16 i=0;
Uint16 j=0;
void B3(void)   //3*500usec
{
    //    setTimeCheck();
    if (common_flag_init_GlobalVariable) initGlobalVariable();

    i++;
    if (i>667) {    //for 1sec
        i=0;

        reset_ecap_counter_IR_remote(); //reset eCap counter to address IR errors

        IfChangeConstants_flashWrite();  //If any HMI or SCI changes constants then save modified constants to flash

        if(LEDRed) LEDRed=OFF;  //Turn off the the LED triggered by IR sensor
    }

    j++;
    if (j>7) {    //approximate 10msec
        j=0;

        trasmit_vars();
    }

    //
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    //
    B_Task_Ptr = &B1;
//    resetTimeCheck();
}



void configCLAMemory(void)
{
    //
    // Copy over CLA Math tables from FLASH to RAM
    memcpy((uint32_t *)&CLA1mathTablesRunStart, (uint32_t *)&CLA1mathTablesLoadStart, (uint32_t)&CLA1mathTablesLoadSize);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS6, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS6, MEMCFG_CLA_MEM_DATA);
}


//******************************************************************************
// For this example, just stop here if an API error is found
//******************************************************************************
void Example_Error(Fapi_StatusType status)
{
    //  Error code will be in the status parameter
        __asm("    ESTOP0");
}

//******************************************************************************
//  For this example, once we are done just stop here
//******************************************************************************
void Example_Done(void)
{
    __asm("    ESTOP0");
}

//******************************************************************************
// For this example, just stop here if FMSTAT fail occurs
//******************************************************************************
void FMSTAT_Fail(void)
{
    //  Error code will be in the status parameter
        __asm("    ESTOP0");
}


//
// End of File
//
