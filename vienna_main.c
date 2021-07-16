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

//
// the includes
//
#include "F28x_Project.h"
#include "device.h"
#include "driverlib.h"
#include "vienna.h"
#define MATH_TYPE FLOAT_MATH
#include "IQmathLib.h"
#include "CLAmath.h"

//
//---  State Machine Related ---
//
/*
int16_t vTimer0[4];         // Virtual Timers slaved off CPU Timers (A events)
int16_t vTimer1[4];         // Virtual Timers slaved off CPU Timers (B events)
*/

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

//=====
//
// Globals
//
Uint32  ECap1IntCount;
Uint32  ECap1PassCount;
Uint32  EPwm3TimerDirection;

// Globals
//
uint32_t ecap1IntCount;
uint32_t ecap1PassCount;
uint32_t epwm3TimerDirection;
volatile uint16_t cap2Count;
volatile uint16_t cap3Count;
volatile uint16_t cap4Count;
volatile uint16_t epwm3PeriodCount;

//void setupPWM1(uint32_t base1, uint32_t base2, uint32_t base3, uint16_t pwm_period_ticks);
//void configurePWMUpDwnCnt1(uint32_t base1, uint16_t pwm_period_ticks);

//
// Function Prototypes
//
__interrupt void ecap1_isr(void);
void InitECapture(void);
void InitEPwmTimer(void);
void Fail(void);
void initECAP(void);
__interrupt void ecap1ISR(void);
Uint16 flag_start_remote = 0;
Uint16 packet_remote = 0;
unsigned long Switch=0;
unsigned long Last_Switch, Prev_Switch;
Uint16 Longpress=0;
#define UP_Key          0x00FF6996//0x01FE58A7             // Increment Key-code
#define DOWN_Key        0x00FFA956//0x01FEA05F             // Decrement Key-code
#define RIGHT_Key       0x00FFC936//0x01FEC03F
#define LEFT_Key        0x00FF49B6//0x01FE807F
#define MODE_Key        0x00FF9966//0x01FE40BF
#define OnOff_Key       0x00FF59A6

void remote_operation(void);
void ISR_remote(void);
void remote_key_display_state_machine(void);

//LCD1602
void CursorON(void);                              /* Make Cursor visible */
void CursorOFF(void);                             /* Hide cursor */
void DisplayLCD(char LineNumber,char *Message);   /* Display the given message (16 characters) at given location on LCD */
void WriteCommandLCD(unsigned char CommandByte);  /* Write the given command to LCD */
void WriteDataLCD(unsigned char DataByte);        /* Write the given data to LCD */
void InitializeLCD(void);                         /* Initialize LCD */
void LCDDelay(void);
void LCDDelay1600(void);
void SendByte(unsigned char Value);

/*
#define D0      GpioDataRegs.GPASET.bit.GPIO0
#define D0Not   GpioDataRegs.GPACLEAR.bit.GPIO0
#define D1      GpioDataRegs.GPASET.bit.GPIO1
#define D1Not   GpioDataRegs.GPACLEAR.bit.GPIO1
#define D2      GpioDataRegs.GPASET.bit.GPIO2
#define D2Not   GpioDataRegs.GPACLEAR.bit.GPIO2
#define D3      GpioDataRegs.GPASET.bit.GPIO3
#define D3Not   GpioDataRegs.GPACLEAR.bit.GPIO3
*/
#define D4      GpioDataRegs.GPBSET.bit.GPIO34
#define D4Not   GpioDataRegs.GPBCLEAR.bit.GPIO34
#define D5      GpioDataRegs.GPBSET.bit.GPIO35
#define D5Not   GpioDataRegs.GPBCLEAR.bit.GPIO35
#define D6      GpioDataRegs.GPBSET.bit.GPIO37
#define D6Not   GpioDataRegs.GPBCLEAR.bit.GPIO37
#define D7      GpioDataRegs.GPBSET.bit.GPIO39
#define D7Not   GpioDataRegs.GPBCLEAR.bit.GPIO39
#define E       GpioDataRegs.GPBSET.bit.GPIO33
#define ENot    GpioDataRegs.GPBCLEAR.bit.GPIO33
#define RS      GpioDataRegs.GPBSET.bit.GPIO32
#define RSNot   GpioDataRegs.GPBCLEAR.bit.GPIO32


void LCD_selection(void);
void Command_Process(void);
void Array_Process(void);
void Byte_Process(void);
void MSNibble_Process(void);
void LSNibble_Process(void);
void (*LCD_Ptr)(void);
void Lcd_Nibble_Port(unsigned char Value);
char LSNibble, MSNibble;
char char_LCD;
Uint16 i_LCD=0;
Uint16 j_LCD=0;
enum LCD_STATE {LCD_NO_ACTION=0, SCREEN1R1, SCREEN1R2, SCREEN2R1, SCREEN2R2, SCREEN3R1, SCREEN3R2, SCREEN4R1, SCREEN4R2, SCREEN5R1, SCREEN5R2, SCREEN6R1, SCREEN6R2, SCREEN7R1, SCREEN7R2, SCREEN8R1, SCREEN8R2, SCREEN9R1, SCREEN9R2, CLEAR_SCREEN, LINE1, LINE2, CURSOR_ON, CURSOR_OFF, SHIFT_RIGHT, SHIFT_LEFT};
char *string_LCD;
char command_LCD;
Uint16 LCD_ACTION = 0;
Uint16 New_string=0;
Uint16 New_command=0;

char screen1r1text[] =      "Ch1=   ;Vdc=   V";
char screen1r2text[] =      "V1=   V;I1=  . A";

char screen2r1text[] =      "Ch2=   ;Vdc=   V";
char screen2r2text[] =      "V2=   V;I2=  . A";

char screen3r1text[] =      "Ir1=  . ;Vr1=   ";
char screen3r2text[] =      "IL1=  . ;Vp1=   ";

char screen4r1text[] =      "Ir2=  . ;Vr2=   ";
char screen4r2text[] =      "IL2=  . ;Vp2=   ";

char screen5r1text[] =      "BoardTemp=  . C ";
char screen5r2text[] =      "Vdc=   V;Vc= .  ";

char screen6r1text[] =      "POT=  %;Dt=  . %";
char screen6r2text[] =      "Spare2          ";

char screen7r1text[] =      "To Change Para: ";
char screen7r2text[] =      "PsW:            ";

char screen8r1text[] =      "Index:     ;Md= ";
char screen8r2text[] =      "Val=            ";

char screen9r1text[] =      "Fault:          ";
char screen9r2text[] =      "Fixed it?       ";

uint16_t Screen_count=1;
Uint16 UpDownKeyFunc=1;
Uint16 Test3=0;
enum UPDOWNFUNC {KeyForScreenMov=0, KeyForPasswordConst, KeyForChangeIDofConst, KeyForChangeofConst};

Uint16 Command_Const=0;
Uint16 Command_PwD=0;

Uint16 PasswordConst = 0;

float Dis_Constant;
Uint16 ID_Const=0;
float Constant=0.0;
volatile float32_t *Ptr_Constant;

unsigned int x_int, x_deca, x_hecto, x_kelo, x_10kelo, x_100kelo, x_mega, x_10mega;

void StateControl_RemoteScreen(void);

float Pot_percent, Duty_percent;

struct COMMON_FLAG {
    uint16_t clearTrip:1;
};
#define COMMON_FLAG_DEFAULTS {0}

//
struct COMMAND {
    uint16_t OnOffCh1;
    uint16_t OnOffCh2;
    uint16_t Clear_OCfault;
};
#define COMMAND_DEFAULTS {0,0,0}

struct COMMAND command=COMMAND_DEFAULTS;

void pickup_constant(void);
long int Decimizer[]={1, 1,  1,  1,  100,    1,  1,  100,    10, 1,  100000000000,   100,    1,  1,  100,    1,  1,  100,    10, 1,  100000000000,   100,    10, 10, 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1000000,    1,  100,    1000000,    1,  100,    1,  1000000,    1,  1000,   1000,   1000,   1000,   1,  1,  1,  1,  100,    100,    100,    100,    10000,  10000,  10000,  10000,  10, 10, 10, 10, 10, 10, 1000000,    1000000};

#define PowerRelay  GpioDataRegs.GPADAT.bit.GPIO14
#define DisablePWM  GpioDataRegs.GPADAT.bit.GPIO22
#define ExtSwitch   GpioDataRegs.GPBDAT.bit.GPIO30
#define LEDRed      GpioDataRegs.GPADAT.bit.GPIO15
//#define TimeTest    GpioDataRegs.GPBDAT.bit.GPIO26
#define TimeTest    GpioDataRegs.GPASET.bit.GPIO26
#define TimeTestN   GpioDataRegs.GPACLEAR.bit.GPIO26
//#define TimeTestToggle GpioDataRegs.GPATOGGLE.bit.GPIO26;

uint16_t CLA1mathTablesRunStart, CLA1mathTablesLoadStart;
uint16_t CLA1mathTablesLoadSize;
void configCLAMemory(void);
//=====

void main(void)
{
    //
    // This routine sets up the basic device configuration such as
    // initializing PLL, copying code from FLASH to RAM,
    // this routine will also initialize the CPU timers that are used in
    // the background 1, 2 & 3) task for this system (CPU time)
    //

    VIENNA_HAL_setupDevice();
    configCLAMemory();
    //======
    EALLOW;
    //PowerRelay
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;
    //DisablePWM
    GpioCtrlRegs.GPAAMSEL.bit.GPIO22 = 0;//For GPIO22 and GPIO23 we need to manualy config them from analog to digital mode
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;
    EDIS;

#if VIENNA_CONTROL_RUNNING_ON == CLA_CORE
    GPIO_setMasterCore(14, GPIO_CORE_CPU1_CLA1);
    GPIO_setMasterCore(22, GPIO_CORE_CPU1_CLA1);
#endif

    EALLOW;
    //LEDRed
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;
    //ExtSwitch
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 2; //6sample qualification
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 0;
    EDIS;

    GpioDataRegs.GPADAT.bit.GPIO14 = 0;
    GpioDataRegs.GPADAT.bit.GPIO15 = 0;
    GpioDataRegs.GPADAT.bit.GPIO22 = 1;

    EALLOW;
    //TestPin shared with I2C
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;
    EDIS;
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
    //=====

#if VIENNA_CONTROL_RUNNING_ON == CLA_CORE
    GPIO_setMasterCore(26, GPIO_CORE_CPU1_CLA1);
#endif

    //
    // Tasks State-machine init
    //
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;

    //
    // Stop all PWM mode clock
    //
    VIENNA_HAL_disablePWMClkCounting();

    //
    //sets up the PWMs for the VIENNA PFC
    //
//    VIENNA_HAL_setupPWM(VIENNA_HIGH_FREQ_PWM1_BASE, VIENNA_HIGH_FREQ_PWM2_BASE,  VIENNA_HIGH_FREQ_PWM3_BASE, VIENNA_PFC3PH_PWM_PERIOD);
    setupPWM(VIENNA_HIGH_FREQ_PWM1_BASE, VIENNA_HIGH_FREQ_PWM2_BASE, VIENNA_HIGH_FREQ_PWM3_BASE, VIENNA_PFC3PH_PWM_PERIOD, INV_DEADBAND_PWM_COUNT, INV_DEADBAND_PWM_COUNT);

    //
    // power up ADC on the device
    //
//    VIENNA_HAL_setupADC();
    setupADC();

    #if VIENNA_SDFM_SENSING == 1
        VIENNA_HAL_setupSDFM(VIENNA_PFC3PH_PWM_PERIOD,
                             VIENNA_PWM_CLK_IN_SDFM_OSR,
                             VIENNA_SD_CLK_COUNT, VIENNA_SDFM_OSR);
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
    // Enable PWM Clocks
    //
    VIENNA_HAL_enablePWMClkCounting();

    //
    //setup PMW trigger for the ADC conversions
    //
    VIENNA_HAL_setupTriggerForADC(VIENNA_HIGH_FREQ_PWM1_BASE);   //same, checked

    #if VIENNA_SDFM_SENSING == 1
        VIENNA_HAL_enablePWMInterruptGeneration(
                                 VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE,
                                 (float32_t)VIENNA_PWM_CLK_IN_SDFM_OSR *
                                 (float32_t)1.5 );
    #else
        VIENNA_HAL_enablePWMInterruptGeneration(
                     VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE,
                     EPWM_getCounterCompareValue(VIENNA_HIGH_FREQ_PWM1_BASE,
                     EPWM_COUNTER_COMPARE_B) );
    #endif

    //
    // Offset Calibration Routine
    // #if VIENNA_SDFM_SENSING == 0
    //  VIENNA_calibrateOffset();
    // #endif
    //
    #if SENSING_OPTION == ADC_BASED_SENSING
        VIENNA_calibrateOffset();
    #endif

    //
    // setup protection and trips for the board
    //
    VIENNA_HAL_setupBoardProtection(VIENNA_HIGH_FREQ_PWM1_BASE,
                                    VIENNA_HIGH_FREQ_PWM2_BASE,
                                    VIENNA_HIGH_FREQ_PWM3_BASE,
                                    VIENNA_I_TRIP_LIMIT_AMPS, VIENNA_I_MAX_SENSE_AMPS);

    //InitECap1Gpio(16);
    {
        EALLOW;
        InputXbarRegs.INPUT7SELECT = 16;         // Set eCAP1 source to GPIO-pin
        EDIS;
    }
    GPIO_SetupPinOptions(16, GPIO_INPUT, GPIO_ASYNC);

    //LCD I/O mapping
    EALLOW;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;

    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO35 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO37 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;
    EDIS;

    GpioDataRegs.GPBDAT.bit.GPIO34 = 0;
    GpioDataRegs.GPBDAT.bit.GPIO35 = 0;
    GpioDataRegs.GPBDAT.bit.GPIO37 = 0;
    GpioDataRegs.GPBDAT.bit.GPIO39 = 0;
    GpioDataRegs.GPBDAT.bit.GPIO33 = 0;
    GpioDataRegs.GPBDAT.bit.GPIO32 = 0;

    InitECapture();

 // Initialize counters:
    ECap1IntCount = 0;
    ECap1PassCount = 0;

    InitializeLCD();
    LCD_Ptr = &LCD_selection;

    //
    // PWM were tripped low in the previous routine
    // safe to setup PWM pins
    //
    VIENNA_HAL_setPinsAsPWM();

    //
    // ISR Mapping
    //
    VIENNA_HAL_setupInterrupt();

    //
    // Setup SFRA
    //
//    VIENNA_setupSFRA();

    //
    // IDLE loop. Just sit and loop forever, periodically will branch into
    // A0-A3, B0-B3, C0-C3 tasks
    // Frequency of this branching is set in setupDevice routine:
    //

    LCD_ACTION = CLEAR_SCREEN;
    UpDownKeyFunc=KeyForScreenMov;
    CONTROL_STATE = VDC_CHARGING;

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
    if(VIENNA_TASKA_CTR_OVFLW_STATUS == 1)
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
    if(VIENNA_TASKB_CTR_OVFLW_STATUS  == 1)
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

void A1(void)
{
    remote_operation();
//    GpioDataRegs.GPATOGGLE.bit.GPIO26 = 1;
    //    VIENNA_runSFRABackGroundTasks();
    //
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    //
    A_Task_Ptr = &A2;
}

void A2(void)
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

void B1(void)
{

    (*LCD_Ptr)();  ////in 1 millisencond subroutine

    if (LCD_ACTION==LCD_NO_ACTION) {
        StateControl_RemoteScreen();
    }
//    VIENNA_updateBoardStatus();

    //
    //the next time CpuTimer1 'counter' reaches Period value go to B2
    //
    B_Task_Ptr = &B2;
}

void B2(void)
{
    remote_key_display_state_machine();

    //    VIENNA_HAL_toggleLED();

    //
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    //
    B_Task_Ptr = &B3;

}
Uint16 i=0;
void B3(void)
{
//    setTimeCheck();
    if (common_flag_init_GlobalVariable) {
        initGlobalVariable();
    }

    i++;
    if (i>667) {    //for 1sec
        i=0;

        if ( ECap1Regs.TSCTR > 0x007FFFFF) {    //reset ecap counter for IR remote
            ECap1Regs.TSCTR =0;
            ECAP_reArm(ECAP1_BASE); //ECAP_ECCTL2_REARM;
        }

        if(LEDRed) LEDRed=OFF;

    }

    //    VIENNA_HAL_toggleLED();

    //
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    //
    B_Task_Ptr = &B1;
//    resetTimeCheck();
}

/* Initializes LCD */
void InitializeLCD(void)
{
    E = SET; //GPIO_setHigh(myGpio, E);

    LCDDelay1600();
    LCDDelay1600();
    LCDDelay1600();
    LCDDelay1600();

    WriteCommandLCD(0x03);          //Command to select 4 bit interface
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay

    LCDDelay1600();
    LCDDelay1600();
    LCDDelay1600();


    WriteCommandLCD(0x03);          //Command to select 4 bit interface
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay

    WriteCommandLCD(0x03);          //Command to select 4 bit interface
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay
    LCDDelay();
    WriteCommandLCD(0x02);          //Command to select 4 bit interface
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay
    LCDDelay1600();


    WriteCommandLCD(0x02);          //Command to
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay
    WriteCommandLCD(0x08);          //Command to off cursor,display off
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay

    WriteCommandLCD(0x00);          //Command to
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay
    WriteCommandLCD(0x0C);          //Command to
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay

    LCDDelay1600();
    WriteCommandLCD(0x00);          //Command to
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay
    WriteCommandLCD(0x06);          //Command for setting entry mode
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay

    WriteCommandLCD(0x0f);          //Command to on cursor,blink cursor
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay
    WriteCommandLCD(0x02);          //Command return the cursor to home
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
    ENot = SET;
    LCDDelay();                         //Small delay
    LCDDelay();                     //Small delay

    LCDDelay1600();
}

void LCD_selection(void)
{
   switch (LCD_ACTION) {    //keep LCD_ACTION can have only one value
       case LCD_NO_ACTION:
           break;
       case SCREEN1R1:
           //char screen1r1text[] =      "Ch1=   ;Vdc=   V";

/*
           if(PowerControl_State_Ptr1 == &Vpv_Control1) {
               screen1r1text[4] = "O";
               screen1r1text[5] = "n";
               screen1r1text[6] = " ";
           } else if(PowerControl_State_Ptr1 == &Idle_State1) {
               screen1r1text[4] = "O";
               screen1r1text[5] = "f";
               screen1r1text[6] = "f";
           } else if(PowerControl_State_Ptr1 == &Slew_Control1) {
               screen1r1text[4] = "A";
               screen1r1text[5] = "c";
               screen1r1text[6] = "c";
           } else if(PowerControl_State_Ptr1 == &Fault_State) {
               screen1r1text[4] = "F";
               screen1r1text[5] = "l";
               screen1r1text[6] = "t";
           } else if(PowerControl_State_Ptr1 == &Vdc_Charging) {
               screen1r1text[4] = "C";
               screen1r1text[5] = "h";
               screen1r1text[6] = "g";
           }
*/

           if(CONTROL_STATE == VPV_CONTROL1) {
               screen1r1text[4] = 'O';
               screen1r1text[5] = 'n';
               screen1r1text[6] = ' ';
           } else if(CONTROL_STATE == IDLE_STATE1) {
               screen1r1text[4] = 'O';
               screen1r1text[5] = 'f';
               screen1r1text[6] = 'f';
           } else if(CONTROL_STATE == SLEW_CONTROL1) {
               screen1r1text[4] = 'A';
               screen1r1text[5] = 'c';
               screen1r1text[6] = 'c';
           } else if(CONTROL_STATE == FAULT_STATE) {
               screen1r1text[4] = 'F';
               screen1r1text[5] = 'l';
               screen1r1text[6] = 't';
           } else if(CONTROL_STATE == VDC_CHARGING) {
               screen1r1text[4] = 'C';
               screen1r1text[5] = 'h';
               screen1r1text[6] = 'g';
           } else if(CONTROL_STATE == DEACCELERATION1) {
               screen1r1text[4] = 'D';
               screen1r1text[5] = 'c';
               screen1r1text[6] = 'c';
           }

//           va = 23.1;
           x_hecto = (unsigned int)(VIENNA_vDCMeas_pu*0.01);
           x_deca = (unsigned int)((float)(VIENNA_vDCMeas_pu - (float)(x_hecto*100))* 0.1);
           x_int = (unsigned int) ((float)VIENNA_vDCMeas_pu - (float)(x_hecto*100) - (float)(x_deca*10));
           screen1r1text[12] = (x_hecto+'0');
           screen1r1text[13] = (x_deca+'0');
           screen1r1text[14] = (x_int+'0');

           string_LCD = screen1r1text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN1R2:
           //char screen1r2text[] =      "V1=   V;I1=  . A";

           x_hecto = (unsigned int)(sensor_v_pv1_fltr*0.01);
           x_deca = (unsigned int)((float)(sensor_v_pv1_fltr - (float)(x_hecto*100))* 0.1);
           x_int = (unsigned int) ((float)sensor_v_pv1_fltr - (float)(x_hecto*100) - (float)(x_deca*10));
           screen1r2text[3] = (x_hecto+'0');
           screen1r2text[4] = (x_deca+'0');
           screen1r2text[5] = (x_int+'0');

//           ia = 5.2;
           x_hecto = (unsigned int)(sensor_i_pv1_fltr*0.1);
           x_deca = (unsigned int)((float)(sensor_i_pv1_fltr - x_hecto*10));
           x_int = (unsigned int) (float)(sensor_i_pv1_fltr*10 - (float)(x_hecto*100) - (float)(x_deca*10));
           screen1r2text[11] = (x_hecto+'0');
           screen1r2text[12] = (x_deca+'0');
           screen1r2text[14] = (x_int+'0');

           string_LCD = screen1r2text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN2R1:
           //char screen2r1text[] =      "Ch2=   ;Vdc=   V";

/*
           if(PowerControl_State_Ptr2 = &Vpv_Control2) {
               screen1r1text[4] = "O";
               screen1r1text[5] = "n";
               screen1r1text[6] = " ";
           } else if(PowerControl_State_Ptr2 = &Idle_State2) {
               screen1r1text[4] = "O";
               screen1r1text[5] = "f";
               screen1r1text[6] = "f";
           }
           } else if(PowerControl_State_Ptr2 = &Slew_Control2) {
               screen1r1text[4] = "A";
               screen1r1text[5] = "c";
               screen1r1text[6] = "c";
           } else if(PowerControl_State_Ptr2 = &Fault_State) {
               screen1r1text[4] = "F";
               screen1r1text[5] = "l";
               screen1r1text[6] = "t";
           } else if(PowerControl_State_Ptr2 = &Vdc_Charging) {
               screen1r1text[4] = "C";
               screen1r1text[5] = "h";
               screen1r1text[6] = "g";
           }
*/

//           va = 23.1;
           x_hecto = (unsigned int)(VIENNA_vDCMeas_pu*0.01);
           x_deca = (unsigned int)((float)(VIENNA_vDCMeas_pu - (float)(x_hecto*100))* 0.1);
           x_int = (unsigned int) ((float)VIENNA_vDCMeas_pu - (float)(x_hecto*100) - (float)(x_deca*10));
           screen1r1text[12] = (x_hecto+'0');
           screen1r1text[13] = (x_deca+'0');
           screen1r1text[14] = (x_int+'0');

           string_LCD = screen2r1text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN2R2:
           //char screen2r2text[] =      "V2=   V;I2=  . A";
//           va = 231;
           x_hecto = (unsigned int)(sensor_v_pv2_fltr*0.01);
           x_deca = (unsigned int)((float)(sensor_v_pv2_fltr - (float)(x_hecto*100))* 0.1);
           x_int = (unsigned int) ((float)sensor_v_pv2_fltr - (float)(x_hecto*100) - (float)(x_deca*10));
           screen2r2text[3] = (x_hecto+'0');
           screen2r2text[4] = (x_deca+'0');
           screen2r2text[5] = (x_int+'0');

//           ia = 75.2;
           x_hecto = (unsigned int)(sensor_i_pv2_fltr*0.1);
           x_deca = (unsigned int)((float)(sensor_i_pv2_fltr - x_hecto*10));
           x_int = (unsigned int) (float)(sensor_i_pv2_fltr*10 - (float)(x_hecto*100) - (float)(x_deca*10));
           screen2r2text[11] = (x_hecto+'0');
           screen2r2text[12] = (x_deca+'0');
           screen2r2text[14] = (x_int+'0');

           string_LCD = screen2r2text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN3R1:
           //char screen3r1text[] =      "Ir1=  . ;Vr1=   ";
           x_hecto = (unsigned int)(il1_control_ref*0.1);
           x_deca = (unsigned int)((float)(il1_control_ref - x_hecto*10));
           x_int = (unsigned int) (float)(il1_control_ref*10 - (float)(x_hecto*100) - (float)(x_deca*10));
           screen3r1text[4] = (x_hecto+'0');
           screen3r1text[5] = (x_deca+'0');
           screen3r1text[7] = (x_int+'0');

           x_hecto = (unsigned int)(vpv1_control_ref*0.01);
           x_deca = (unsigned int)((float)(vpv1_control_ref - (float)(x_hecto*100))* 0.1);
           x_int = (unsigned int) ((float)vpv1_control_ref - (float)(x_hecto*100) - (float)(x_deca*10));
           screen3r1text[13] = (x_hecto+'0');
           screen3r1text[14] = (x_deca+'0');
           screen3r1text[15] = (x_int+'0');

           string_LCD = screen3r1text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN3R2:
           //char screen3r2text[] =      "IL1=  . ;Vp1=   ";
           x_hecto = (unsigned int)(VIENNA_iL1Meas_pu*0.1);
           x_deca = (unsigned int)((float)(VIENNA_iL1Meas_pu - x_hecto*10));
           x_int = (unsigned int) (float)(VIENNA_iL1Meas_pu*10 - (float)(x_hecto*100) - (float)(x_deca*10));
           screen3r2text[4] = (x_hecto+'0');
           screen3r2text[5] = (x_deca+'0');
           screen3r2text[7] = (x_int+'0');

           x_hecto = (unsigned int)(VIENNA_vPV1Meas_pu*0.01);
           x_deca = (unsigned int)((float)(VIENNA_vPV1Meas_pu - (float)(x_hecto*100))* 0.1);
           x_int = (unsigned int) ((float)VIENNA_vPV1Meas_pu - (float)(x_hecto*100) - (float)(x_deca*10));
           screen3r2text[13] = (x_hecto+'0');
           screen3r2text[14] = (x_deca+'0');
           screen3r2text[15] = (x_int+'0');

           string_LCD = screen3r2text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN4R1:
           //char screen4r1text[] =      "Ir2=  . ;Vr2=   ";
           x_hecto = (unsigned int)(il2_control_ref*0.1);
           x_deca = (unsigned int)((float)(il2_control_ref - x_hecto*10));
           x_int = (unsigned int) (float)(il2_control_ref*10 - (float)(x_hecto*100) - (float)(x_deca*10));
           screen4r1text[4] = (x_hecto+'0');
           screen4r1text[5] = (x_deca+'0');
           screen4r1text[7] = (x_int+'0');

           x_hecto = (unsigned int)(vpv2_control_ref*0.01);
           x_deca = (unsigned int)((float)(vpv2_control_ref - (float)(x_hecto*100))* 0.1);
           x_int = (unsigned int) ((float)vpv2_control_ref - (float)(x_hecto*100) - (float)(x_deca*10));
           screen4r1text[13] = (x_hecto+'0');
           screen4r1text[14] = (x_deca+'0');
           screen4r1text[15] = (x_int+'0');

           string_LCD = screen4r1text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN4R2:
           //char screen4r2text[] =      "IL2=  . ;Vp2=   ";
           x_hecto = (unsigned int)(VIENNA_iL2Meas_pu*0.1);
           x_deca = (unsigned int)((float)(VIENNA_iL2Meas_pu - x_hecto*10));
           x_int = (unsigned int) (float)(VIENNA_iL2Meas_pu*10 - (float)(x_hecto*100) - (float)(x_deca*10));
           screen4r2text[4] = (x_hecto+'0');
           screen4r2text[5] = (x_deca+'0');
           screen4r2text[7] = (x_int+'0');

           x_hecto = (unsigned int)(VIENNA_vPV2Meas_pu*0.01);
           x_deca = (unsigned int)((float)(VIENNA_vPV2Meas_pu - (float)(x_hecto*100))* 0.1);
           x_int = (unsigned int) ((float)VIENNA_vPV2Meas_pu - (float)(x_hecto*100) - (float)(x_deca*10));
           screen4r2text[13] = (x_hecto+'0');
           screen4r2text[14] = (x_deca+'0');
           screen4r2text[15] = (x_int+'0');

           string_LCD = screen4r2text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN5R1:
           //char screen5r1text[] =      "BoardTemp=  . C ";
           x_hecto = (unsigned int)(VIENNA_TEMPMeas_pu*0.1);
           x_deca = (unsigned int)((float)((VIENNA_TEMPMeas_pu - x_hecto*10)));
           x_int = (unsigned int) (float)((VIENNA_TEMPMeas_pu*10 - (float)(x_hecto*100) - (float)(x_deca*10)));
           screen5r1text[10] = (x_hecto+'0');
           screen5r1text[11] = (x_deca+'0');
           screen5r1text[13] = (x_int+'0');

           string_LCD = screen5r1text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN5R2:
           //char screen5r2text[] =      "Vdc=   V;Vc= .  ";
           x_hecto = (unsigned int)(VIENNA_vDCMeas_pu*0.01);
           x_deca = (unsigned int)((float)(VIENNA_vDCMeas_pu - (float)(x_hecto*100))* 0.1);
           x_int = (unsigned int) ((float)VIENNA_vDCMeas_pu - (float)(x_hecto*100) - (float)(x_deca*10));
           screen5r2text[4] = (x_hecto+'0');
           screen5r2text[5] = (x_deca+'0');
           screen5r2text[6] = (x_int+'0');

           x_hecto = (unsigned int)(VIENNA_vCCMeas_pu);
           x_deca = (unsigned int)((float)(VIENNA_vCCMeas_pu - (float)(x_hecto) )*10);
           x_int = (unsigned int) ((float)((VIENNA_vCCMeas_pu - (float)(x_hecto))*10 - (float)(x_deca))*10);
           screen5r2text[12] = (x_hecto+'0');
           screen5r2text[14] = (x_deca+'0');
           screen5r2text[15] = (x_int+'0');

           string_LCD = screen5r2text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;

       case SCREEN6R1:
           //char screen6r1text[] =      "POT=  %;Dt=  . %";
           Pot_percent=VIENNA_POTMeas_pu*100;
           x_hecto = (unsigned int)(Pot_percent*0.01);
           x_deca = (unsigned int)((float)(Pot_percent - (float)(x_hecto*100))* 0.1);
           x_int = (unsigned int) ((float)Pot_percent - (float)(x_hecto*100) - (float)(x_deca*10));
//           screen6r1text[4] = (x_hecto+'0');
           screen6r1text[4] = (x_deca+'0');
           screen6r1text[5] = (x_int+'0');

           Duty_percent=(common_vars_duty1-0.030)*100; //(2x2x0.15usec)/(1/fsw)  2 for updowncount and 2 for at front and at back; Even after this 1% error at 50kHz due to IC response time
           if (Duty_percent<0.0) Duty_percent=0.0;
           x_hecto = (unsigned int)(Duty_percent*0.1);
           x_deca = (unsigned int)((float)(Duty_percent - x_hecto*10));
           x_int = (unsigned int) (float)(Duty_percent*10 - (float)(x_hecto*100) - (float)(x_deca*10));
           screen6r1text[11] = (x_hecto+'0');
           screen6r1text[12] = (x_deca+'0');
           screen6r1text[14] = (x_int+'0');

           string_LCD = screen6r1text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN6R2:
           //char screen6r2text[] =      "Spare2          ";

           string_LCD = screen6r2text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;

       case SCREEN7R1:
           //char screen7r1text[] =      "To Change Para: ";
           screen7r1text[15] = ( (unsigned int)(UpDownKeyFunc)  +'0');

           string_LCD = screen7r1text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN7R2:
           //char screen7r2text[] =      "PsW:            ";
           //PasswordConst = 1243;
           x_kelo = (unsigned int)((float)(PasswordConst*0.001));
           x_hecto = (unsigned int)((float)((float)PasswordConst - (float)(x_kelo*1000))* 0.01);
           x_deca = (unsigned int) ((float)((float)PasswordConst - (float)(x_kelo*1000) - (float)(x_hecto*100))*0.1);
           x_int =  (unsigned int) ((float)((float)PasswordConst - (float)(x_kelo*1000) - (float)(x_hecto*100) - (float)(x_deca*10)) );
           screen7r2text[4] = (x_kelo+'0');
           screen7r2text[5] = (x_hecto+'0');
           screen7r2text[6] = (x_deca+'0');
           screen7r2text[7] = (x_int+'0');


           string_LCD = screen7r2text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;

       case SCREEN8R1:
           //char screen8r1text[] =      "Index:     ;Md= ";
           x_deca = (unsigned int)(ID_Const*0.1);
           x_int = (unsigned int)(float)(ID_Const - (float)(x_deca*10));
           screen8r1text[6] = (x_deca+'0');
           screen8r1text[7] = (x_int+'0');

           screen8r1text[14] = ( (unsigned int)(UpDownKeyFunc)  +'0');

           string_LCD = screen8r1text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN8R2:
           //char screen8r2text[] =      "Val=            ";

           if (Constant<0.0) Dis_Constant=-Constant;
           else Dis_Constant=Constant;
           x_10mega = (unsigned int)((float)(Dis_Constant*0.0000001));
           x_mega = (unsigned int)((float)((float)Dis_Constant - (float)(x_10mega*10000000))* 0.000001);
           x_100kelo = (unsigned int) ((float)((float)Dis_Constant - (float)(x_10mega*10000000) - (float)(x_mega*1000000))*0.00001);
           x_10kelo = (unsigned int) ((float)((float)Dis_Constant - (float)(x_10mega*10000000) - (float)(x_mega*1000000) - (float)(x_100kelo*100000))*0.0001);
           x_kelo = (unsigned int) ((float)((float)Dis_Constant - (float)(x_10mega*10000000) - (float)(x_mega*1000000) - (float)(x_100kelo*100000)- (float)(x_10kelo*10000) )*0.001 );
           x_hecto = (unsigned int) ((float)((float)Dis_Constant - (float)(x_10mega*10000000) - (float)(x_mega*1000000) - (float)(x_100kelo*100000)- (float)(x_10kelo*10000) - (float)(x_kelo*1000) )*0.01 );
           x_deca = (unsigned int) ((float)((float)Dis_Constant - (float)(x_10mega*10000000) - (float)(x_mega*1000000) - (float)(x_100kelo*100000)- (float)(x_10kelo*10000) - (float)(x_kelo*1000) - (float)(x_hecto*100))*0.1 );
           x_int = (unsigned int) ((float)((float)Dis_Constant - (float)(x_10mega*10000000) - (float)(x_mega*1000000) - (float)(x_100kelo*100000)- (float)(x_10kelo*10000) - (float)(x_kelo*1000) - (float)(x_hecto*100) - (float)(x_deca*10)));


           screen8r2text[4] = (x_10mega+'0');
           screen8r2text[5] = (x_mega+'0');
           screen8r2text[6] = (x_100kelo+'0');
           screen8r2text[7] = (x_10kelo+'0');
           screen8r2text[8] = (x_kelo+'0');
           screen8r2text[9] = (x_hecto+'0');
           screen8r2text[10] = (x_deca+'0');
           screen8r2text[11] = (x_int+'0');

           string_LCD = screen8r2text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;


       case SCREEN9R1:
           //char screen9r1text[] =      "Fault:          ";

           if(VIENNA_boardStatus == boardStatus_Idle) {
               screen9r1text[6] = 'I';
               screen9r1text[7] = 'd';
               screen9r1text[8] = 'l';
               screen9r1text[9] = 'e';
           } else if(VIENNA_boardStatus == boardStatus_OverVoltageTrip) {
               screen9r1text[6] = 'O';
               screen9r1text[7] = 'V';
               screen9r1text[8] = 'D';
               screen9r1text[9] = 'C';
           } else if(VIENNA_boardStatus == vdc_undervoltage) {
               screen9r1text[6] = 'U';
               screen9r1text[7] = 'V';
               screen9r1text[8] = 'D';
               screen9r1text[9] = 'C';
           } else if(VIENNA_boardStatus == vpv_overvoltage) {
               screen9r1text[6] = 'O';
               screen9r1text[7] = 'V';
               screen9r1text[8] = 'P';
               screen9r1text[9] = 'V';
           } else if(VIENNA_boardStatus == boardStatus_OverCurrentTrip_IL1) {
               screen9r1text[6] = 'O';
               screen9r1text[7] = 'C';
               screen9r1text[8] = 'I';
               screen9r1text[9] = '1';
           } else if(VIENNA_boardStatus == boardStatus_OverCurrentTrip_IL2) {
               screen9r1text[6] = 'O';
               screen9r1text[7] = 'C';
               screen9r1text[8] = 'I';
               screen9r1text[9] = '2';
           } else if(VIENNA_boardStatus == boardStatus_NoFault) {
               screen9r1text[6] = 'O';
               screen9r1text[7] = 'K';
               screen9r1text[8] = '.';
               screen9r1text[9] = '.';
           }

           string_LCD = screen9r1text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN9R2:
           //char screen9r2text[] =      "Fixed it?       ";

           string_LCD = screen9r2text;
           New_string = 1;
           LCD_Ptr = &Array_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;


       case CLEAR_SCREEN:
           command_LCD = 0x01;
           New_command = 1;
           LCD_Ptr = &Command_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case LINE1:
           command_LCD = 0x80;
           New_command = 1;
           LCD_Ptr = &Command_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case LINE2:
           command_LCD = 0xC0;
           New_command = 1;
           LCD_Ptr = &Command_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case CURSOR_ON:
           command_LCD = 0x0F;
           New_command = 1;
           LCD_Ptr = &Command_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case CURSOR_OFF:
           command_LCD = 0x0C;
           New_command = 1;
           LCD_Ptr = &Command_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SHIFT_RIGHT:
           command_LCD = 0x1C;
           New_command = 1;
           LCD_Ptr = &Command_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       case SHIFT_LEFT:
           command_LCD = 0x18;
           New_command = 1;
           LCD_Ptr = &Command_Process;
           LCD_ACTION = LCD_NO_ACTION;
           break;
       default:
           break;
   }
}

/*
//in 1 or 100millisecond subroutine.
{
    //update string for variable values as follows

}
*/
/*
//If above code is in 1millisecond then it can be clubbed with below set of code.

*/

void Command_Process(void)
{
    ENot = SET;
    if (j_LCD<1) {
        char_LCD = command_LCD;
        j_LCD++;
    }
    else {
        j_LCD=0;
        New_command = 0;
        LCD_Ptr = &LCD_selection;
        return;
    }
    RSNot = SET;             // => RS = 0
    LCD_Ptr = &Byte_Process;
}
void Array_Process(void)
{
    ENot = SET;
    if (i_LCD<16) {   //row length = 31
        char_LCD = *(string_LCD+i_LCD);                                    // for(i=0;str[i]!='\0';i++)
        i_LCD++;
    }
    else {
        i_LCD = 0;
        New_string = 0;
        LCD_Ptr = &LCD_selection;
        return;
    }
    RS = SET;             // => RS = 1
    LCD_Ptr = &Byte_Process;
}
void Byte_Process(void)
{
    char y;
    LSNibble = char_LCD&0x0F;
    y = char_LCD&0xF0;
    MSNibble = y>>4;
    LCD_Ptr = &MSNibble_Process;
}
void MSNibble_Process(void)
{
    ENot = SET;
    Lcd_Nibble_Port(MSNibble);             //Data transfer
    LCD_Ptr = &LSNibble_Process;
}
void LSNibble_Process(void)
{
    ENot = SET;
    Lcd_Nibble_Port(LSNibble);
    if (New_string)
        LCD_Ptr = &Array_Process;
    if (New_command)
        LCD_Ptr = &Command_Process;
}
void Lcd_Nibble_Port(unsigned char Value)
{
    if((Value & 0x01) == 0x01)  // CHECK ALTERNATIVE TO THIS LINE. As per LCD example file "if(value & 1) D4=SET;"
        D4 = SET; //GPIO_setHigh(myGpio, D4);
    else
        D4Not = SET;
    if((Value & 0x02) == 0x02)
        D5 = SET; //GPIO_setHigh(myGpio, D5);
    else
        D5Not = SET;
    if((Value & 0x04) == 0x04)
        D6 = SET; //GPIO_setHigh(myGpio, D6);
    else
        D6Not = SET;
    if((Value & 0x08) == 0x08)
        D7 = SET; //GPIO_setHigh(myGpio, D7);
    else
        D7Not = SET;
    E = SET;  //GPIO_setHigh(myGpio, E);            //Set E pin to select LCD
}
void WriteCommandLCD(unsigned char CommandByte) //only for initialization
{
    //RS = CLEAR; // GPIO_setLow(myGpio, RS);        //Clear RS pin to write command
    RSNot = SET;
    Lcd_Nibble_Port(CommandByte);
    LCDDelay();                     //Small delay
    LCDDelay();                     //Small delay
}
/*
void Lcd_Set_Cursor(unsigned char a, unsigned char b)
{
    char temp,z,y;
    if(a == 1)
    {
        temp = 0x80 + b - 1;
        zy = temp;
      //  z = temp>>4;
      //  y = temp & 0x0F;
      //  WriteCommandLCD(z);
      //  WriteCommandLCD(y);
    }
    else if(a == 2)
    {
        temp = 0xC0 + b - 1;
        zy = temp;
      //  z = temp>>4;
      //  y = temp & 0x0F;
      //  WriteCommandLCD(z);
      //  WriteCommandLCD(y);
    }
//I think we can add else if(a==3) and if(a==4) for 4x16 LCD
}
*/


/* Small delay */
void LCDDelay(void)
{
/*
    unsigned int x;
    for(x=50; x<=0; x-- );
*/
    DELAY_US(50);
}

/* Big delay */
void LCDDelay1600(void)
{
/*
    unsigned int x;
    for(x=1600; x<=0; x-- );
*/
    DELAY_US(1600);
}




void remote_operation(void)
{
    if (ECap1Regs.ECCTL2.bit.STOP_WRAP==1) {
        if (ECap1Regs.ECFLG.bit.CEVT2) {
            ISR_remote();
        }
    }
    if (ECap1Regs.ECCTL2.bit.STOP_WRAP==3) {
        if (ECap1Regs.ECFLG.bit.CEVT4) {
            ISR_remote();
        }
    }
}

void remote_key_display_state_machine(void)
{
    if (UpDownKeyFunc==KeyForScreenMov) {
        if (Last_Switch==UP_Key) {
            Last_Switch = 0;
            //Action1
            LEDRed = ON;
            Test3=1;
            Screen_count++;
            if (Screen_count>=8) Screen_count=1;
        }
        else if (Last_Switch==DOWN_Key) {
            Last_Switch = 0;
            //Action2
            LEDRed = ON;
//            LCD_ACTION = LINE2;
            Test3=2;
            Screen_count--;
            if (Screen_count==0) Screen_count=7;

        }
    }
    else if (UpDownKeyFunc==KeyForPasswordConst) {
        if (Last_Switch==UP_Key) {
            Last_Switch = 0;
            //Action1
            LEDRed = ON;

            if (Longpress)
                PasswordConst=PasswordConst+10;
            else
                PasswordConst++;

            Test3=1;
        }
        else if (Last_Switch==DOWN_Key) {
            Last_Switch = 0;
            //Action2
            LEDRed = ON;

            if (Longpress)
                PasswordConst=PasswordConst-10;
            else
                PasswordConst--;

            Test3=2;
        }
    }

    else if (UpDownKeyFunc==KeyForChangeIDofConst) {
        if (Last_Switch==UP_Key) {
            Last_Switch = 0;
            //Action1
            LEDRed = ON;

            if (Longpress)
                ID_Const=ID_Const+5;
            else {
                ID_Const++;
                if (ID_Const >= 99) {
                    Screen_count=1;
                    UpDownKeyFunc=KeyForScreenMov;
                    Command_Const=DISABLE;
                    Command_PwD=OFF;
                    ID_Const = 0;
                    PasswordConst=0;
                }
            }

            Test3=1;
        }
        else if (Last_Switch==DOWN_Key) {
            Last_Switch = 0;
            //Action2
            LEDRed = ON;

            if (Longpress) {
                ID_Const=ID_Const-5;
                if (ID_Const<=0) ID_Const=1;
            }
            else{
                ID_Const--;
                if (ID_Const<=0) ID_Const=1;
            }

            Test3=2;
        }
        pickup_constant();
        Constant=(*Ptr_Constant)*(float)Decimizer[ID_Const];
    }

    else if (UpDownKeyFunc==KeyForChangeofConst) {
        if (Last_Switch==UP_Key) {
            Last_Switch = 0;
            //Action1
            LEDRed = ON;

            if (Longpress)
                Constant=Constant+10;
            else
                Constant++;

            Test3=1;
        }
        else if (Last_Switch==DOWN_Key) {
            Last_Switch = 0;
            //Action2
            LEDRed = ON;

            if (Longpress)
                Constant=Constant-10;
            else
                Constant--;

            Test3=2;
        }

        *Ptr_Constant = Constant/((float)Decimizer[ID_Const]);
    }



    if (Last_Switch==LEFT_Key) {
        Last_Switch = 0;
        //Action3
        LEDRed = ON;
//        LCD_ACTION = SCREEN1R1;
        Test3=3;
    }
    if (Last_Switch==RIGHT_Key) {
        Last_Switch = 0;
        //Action4
        LEDRed = ON;
//        LCD_ACTION = SCREEN1R2;
        Test3=4;
    }
    if (Last_Switch==OnOff_Key) {
        Last_Switch = 0;
        //Action5
        LEDRed = ON;
        if ((command.OnOffCh1)&&(Screen_count==1)) {
            command.OnOffCh1=OFF;
            command_OnOffCh1=OFF;
        }
        else if ((!command.OnOffCh1)&&(Screen_count==1)) {
            command.OnOffCh1=ON;
            command_OnOffCh1=ON;
        }
        if ((command.OnOffCh2)&&(Screen_count==2))
            command.OnOffCh2=OFF;
        else if ((!command.OnOffCh2)&&(Screen_count==2))
            command.OnOffCh2=ON;

        if (Screen_count==9) {
            command.Clear_OCfault  = 1;
            command_Clear_OCfault  = 1;
        }

        if ((Command_PwD)&&(Screen_count==7)) {
            if (PasswordConst==85) {
                Screen_count=8;
                UpDownKeyFunc=KeyForChangeIDofConst;
            } else {
                Screen_count=1;
                UpDownKeyFunc=KeyForScreenMov;
                PasswordConst=0;
            }
            Command_PwD=OFF;
        }
        else if ((!Command_PwD)&&(Screen_count==7)) {
            UpDownKeyFunc=KeyForPasswordConst;
            Command_PwD=ON;
        }

        if ((Command_Const)&&(Screen_count==8)){
            UpDownKeyFunc=KeyForChangeofConst;
            Command_Const=DISABLE;
        } else if ((!Command_Const)&&(Screen_count==8)){
            UpDownKeyFunc=KeyForChangeIDofConst;
            Command_Const=ENABLE;
        }

//        LCD_ACTION = SCREEN1R2;
        Test3=5;
    }

    if (CONTROL_STATE == FAULT_STATE) {
        if (Screen_count != 9) {
        Screen_count = 9;
        LCD_ACTION = SCREEN9R1;
        }
    }

    if (Last_Switch==MODE_Key) {
        Last_Switch = 0;
        //Action6
//        LCD_ACTION = CLEAR_SCREEN;
        Test3=6;
        //
    }
    Last_Switch = 0;
}

Uint16 Seq_screen=1;
Uint16 Prev_Screen_count=0;
void StateControl_RemoteScreen(void)
{
    if (Prev_Screen_count!=Screen_count) {
        Prev_Screen_count=Screen_count;
        LCD_ACTION = CLEAR_SCREEN;
        Seq_screen=1;
        return;
    }

    switch (Screen_count) {
        case 1:
            switch (Seq_screen){
                case 1:
                    LCD_ACTION = SCREEN1R1;
                    Seq_screen++;
                    break;
                case 2:
                    LCD_ACTION = LINE2;
                    Seq_screen++;
                    break;
                case 3:
                    LCD_ACTION = SCREEN1R2;
                    Seq_screen++;
                    break;
                case 4:
                    LCD_ACTION = LINE1;
                    Seq_screen=1;
                    break;
                default:
                    break;
            }
            break;

        case 2:
            switch (Seq_screen){
                case 1:
                    LCD_ACTION = SCREEN2R1;
                    Seq_screen++;
                    break;
                case 2:
                    LCD_ACTION = LINE2;
                    Seq_screen++;
                    break;
                case 3:
                    LCD_ACTION = SCREEN2R2;
                    Seq_screen++;
                    break;
                case 4:
                    LCD_ACTION = LINE1;
                    Seq_screen=1;
                    break;
                default:
                    break;
            }
            break;

        case 3:
            switch (Seq_screen){
                case 1:
                    LCD_ACTION = SCREEN3R1;
                    Seq_screen++;
                    break;
                case 2:
                    LCD_ACTION = LINE2;
                    Seq_screen++;
                    break;
                case 3:
                    LCD_ACTION = SCREEN3R2;
                    Seq_screen++;
                    break;
                case 4:
                    LCD_ACTION = LINE1;
                    Seq_screen=1;
                    break;
                default:
                    break;
            }
            break;

       case 4:
           switch (Seq_screen){
                case 1:
                    LCD_ACTION = SCREEN4R1;
                    Seq_screen++;
                    break;
                case 2:
                    LCD_ACTION = LINE2;
                    Seq_screen++;
                    break;
                case 3:
                    LCD_ACTION = SCREEN4R2;
                    Seq_screen++;
                    break;
                case 4:
                    LCD_ACTION = LINE1;
                    Seq_screen=1;
                    break;
                default:
                    break;
           }
           break;

       case 5:
           switch (Seq_screen){
               case 1:
                   LCD_ACTION = SCREEN5R1;
                   Seq_screen++;
                   break;
               case 2:
                   LCD_ACTION = LINE2;
                   Seq_screen++;
                   break;
               case 3:
                   LCD_ACTION = SCREEN5R2;
                   Seq_screen++;
                   break;
               case 4:
                   LCD_ACTION = LINE1;
                   Seq_screen=1;
                   break;
               default:
                   break;
           }
           break;

       case 6:
           switch (Seq_screen){
               case 1:
                   LCD_ACTION = SCREEN6R1;
                   Seq_screen++;
                   break;
               case 2:
                   LCD_ACTION = LINE2;
                   Seq_screen++;
                   break;
               case 3:
                   LCD_ACTION = SCREEN6R2;
                   Seq_screen++;
                   break;
               case 4:
                   LCD_ACTION = LINE1;
                   Seq_screen=1;
                   break;
               default:
                   break;
           }
           break;

       case 7:
           switch (Seq_screen){
               case 1:
                   LCD_ACTION = SCREEN7R1;
                   Seq_screen++;
                   break;
               case 2:
                   LCD_ACTION = LINE2;
                   Seq_screen++;
                   break;
               case 3:
                   LCD_ACTION = SCREEN7R2;
                   Seq_screen++;
                   break;
               case 4:
                   LCD_ACTION = LINE1;
                   Seq_screen=1;
                   break;
               default:
                   break;
          }
           break;

       case 8:
           switch (Seq_screen){
               case 1:
                   LCD_ACTION = SCREEN8R1;
                   Seq_screen++;
                   break;
               case 2:
                   LCD_ACTION = LINE2;
                   Seq_screen++;
                   break;
               case 3:
                   LCD_ACTION = SCREEN8R2;
                   Seq_screen++;
                   break;
               case 4:
                   LCD_ACTION = LINE1;
                   Seq_screen=1;
                   break;
               default:
                   break;
           }
           break;


        default:
            break;
    }
}

//For assigning picking up constant to change THERE IS BETTER WAY (SHORT CODE) IF USED STRUCTURE
volatile float32_t dummy;
void pickup_constant(void)
{
    switch(ID_Const){
    case 0:
        Ptr_Constant = &Select_PVcurve1;
        break;
    case 1:
        Ptr_Constant = &Select_PVcurve2;
        break;
    case 2:
        Ptr_Constant = &vpv_ref1;
        break;
    case 3:
        Ptr_Constant = &Temp1;
        break;
    case 4:
        Ptr_Constant = &BetaV1;
        break;
    case 5:
        Ptr_Constant = &Ns1;
        break;
    case 6:
        Ptr_Constant = &Np1;
        break;
    case 7:
        Ptr_Constant = &ILight1;
        break;
    case 8:
        Ptr_Constant = &ipv1;
        break;
    case 9:
        Ptr_Constant = &Np1;
        break;
    case 10:
        Ptr_Constant = &Io1;
        break;
    case 11:
        Ptr_Constant = &Rs1;
        break;
    case 12:
        Ptr_Constant = &vpv_ref2;
        break;
    case 13:
        Ptr_Constant = &Temp2;
        break;
    case 14:
        Ptr_Constant = &BetaV2;
        break;
    case 15:
        Ptr_Constant = &Ns2;
        break;
    case 16:
        Ptr_Constant = &Np2;
        break;
    case 17:
        Ptr_Constant = &ILight2;
        break;
    case 18:
        Ptr_Constant = &ipv2;
        break;
    case 19:
        Ptr_Constant = &Np2;
        break;
    case 20:
        Ptr_Constant = &Io2;
        break;
    case 21:
        Ptr_Constant = &Rs2;
        break;
    case 22:
        Ptr_Constant = &update_const_upper_limit_vpv2_controlout;
        break;
    case 23:
        Ptr_Constant = &update_const_lower_limit_vpv2_controlout;
        break;
    case 24:
        Ptr_Constant = &update_const_ipv1_ref_isc;
        break;
    case 25:
        Ptr_Constant = &update_const_vpv1_ref_vsc;
        break;
    case 26:
        Ptr_Constant = &update_const_vpv1_ref_voc;
        break;
    case 27:
        Ptr_Constant = &update_const_ipv1_ref_imp;
        break;
    case 28:
        Ptr_Constant = &update_const_slop1_vpv1_ref_gen;
        break;
    case 29:
        Ptr_Constant = &update_const_constant1_vpv1_ref_gen;
        break;
    case 30:
        Ptr_Constant = &update_const_slop2_vpv1_ref_gen;
        break;
    case 31:
        Ptr_Constant = &update_const_constant2_vpv1_ref_gen;
        break;
    case 32:
        Ptr_Constant = &update_const_ipv2_ref_isc;
        break;
    case 33:
        Ptr_Constant = &update_const_vpv2_ref_vsc;
        break;
    case 34:
        Ptr_Constant = &update_const_vpv2_ref_voc;
        break;
    case 35:
        Ptr_Constant = &update_const_ipv2_ref_imp;
        break;
    case 36:
        Ptr_Constant = &update_const_slop1_vpv2_ref_gen;
        break;
    case 37:
        Ptr_Constant = &update_const_constant1_vpv2_ref_gen;
        break;
    case 38:
        Ptr_Constant = &update_const_slop2_vpv2_ref_gen;
        break;
    case 39:
        Ptr_Constant = &update_const_constant2_vpv2_ref_gen;
        break;
    case 40:
        Ptr_Constant = &update_const_slew_rate1;
        break;
    case 41:
        Ptr_Constant = &update_const_vpv1_threshold_slew_to_vpvcontrol;
        break;
    case 42:
        Ptr_Constant = &update_const_duty1_threshold_slew_to_vpvcontrol;
        break;
    case 43:
        Ptr_Constant = &update_const_slew_rate2;
        break;
    case 44:
        Ptr_Constant = &update_const_vpv2_threshold_slew_to_vpvcontrol;
        break;
    case 45:
        Ptr_Constant = &update_const_duty2_threshold_slew_to_vpvcontrol;
        break;
    case 46:
        Ptr_Constant = &update_const_vdc_threshold_charging_to_idle;
        break;
    case 47:
        Ptr_Constant = &update_const_deacceleration_rate;
        break;
    case 48:
        Ptr_Constant = &update_const_upper_duty_threshold;
        break;
    case 49:
        Ptr_Constant = &update_const_k1_PIiL1;
        break;
    case 50:
        Ptr_Constant = &update_const_k2_PIiL1;
        break;
    case 51:
        Ptr_Constant = &update_const_k1_PIiL2;
        break;
    case 52:
        Ptr_Constant = &update_const_k2_PIiL2;
        break;
    case 53:
        Ptr_Constant = &update_const_upper_limit_il1_PIout;
        break;
    case 54:
        Ptr_Constant = &update_const_lower_limit_il1_PIout;
        break;
    case 55:
        Ptr_Constant = &update_const_upper_limit_il2_PIout;
        break;
    case 56:
        Ptr_Constant = &update_const_lower_limit_il2_PIout;
        break;
    case 57:
        Ptr_Constant = &update_const_upper_limit_il1_controlout;
        break;
    case 58:
        Ptr_Constant = &update_const_lower_limit_il1_controlout;
        break;
    case 59:
        Ptr_Constant = &update_const_upper_limit_il2_controlout;
        break;
    case 60:
        Ptr_Constant = &update_const_lower_limit_il2_controlout;
        break;
    case 61:
        Ptr_Constant = &update_const_k1_PIvpv1;
        break;
    case 62:
        Ptr_Constant = &update_const_k2_PIvpv1;
        break;
    case 63:
        Ptr_Constant = &update_const_k1_PIvpv2;
        break;
    case 64:
        Ptr_Constant = &update_const_k2_PIvpv2;
        break;
    case 65:
        Ptr_Constant = &update_const_upper_limit_vpv1_PIout;
        break;
    case 66:
        Ptr_Constant = &update_const_lower_limit_vpv1_PIout;
        break;
    case 67:
        Ptr_Constant = &update_const_upper_limit_vpv2_PIout;
        break;
    case 68:
        Ptr_Constant = &update_const_lower_limit_vpv2_PIout;
        break;
    case 69:
        Ptr_Constant = &update_const_upper_limit_vpv1_controlout;
        break;
    case 70:
        Ptr_Constant = &update_const_lower_limit_vpv1_controlout;
        break;
    case 71:
        Ptr_Constant = &k1_Fltr_vpv1n2;
        break;
    case 72:
        Ptr_Constant = &k2_Fltr_vpv1n2;
        break;
    default:
        Ptr_Constant = &dummy;
        break;
    }
}


// InitECapture - Initialize ECAP1 configurations
void InitECapture()
{
    EALLOW;

    ECap1Regs.ECEINT.all = 0x0000;          // Disable all capture __interrupts
    ECap1Regs.ECCLR.all = 0xFFFF;           // Clear all CAP __interrupt flags

    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped
    ECap1Regs.ECCTL2.bit.CAP_APWM = 0;      //EC_CAP_MODE=0
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0;   // Countinous
    ECap1Regs.ECCTL2.bit.STOP_WRAP = 1;     // It should be 1 in IR sensor as we want to measure 9+4.5msec,0.56+0.56,0.56+2*0.56 (down-up)
   ECap1Regs.ECCTL1.bit.CAP1POL = 1;       // Falling edge, yes in IR we need
   ECap1Regs.ECCTL1.bit.CAP2POL = 1;       // fall  edge, yes in IR we need
   ECap1Regs.ECCTL1.bit.CAP3POL = 1;       // Falling edge, not required
   ECap1Regs.ECCTL1.bit.CAP4POL = 1;       // falling edge, not required
   ECap1Regs.ECCTL1.bit.CTRRST1 = 1;       // Difference operation, yes we should reset it
   ECap1Regs.ECCTL1.bit.CTRRST2 = 1;       // Difference operation, yes we should reset it
   ECap1Regs.ECCTL1.bit.CTRRST3 = 1;       // Difference operation
   ECap1Regs.ECCTL1.bit.CTRRST4 = 1;       // Difference operation

   ECap1Regs.ECCTL0.bit.INPUTSEL = 6; //connect ECap1 to INPUT7SELECT  //****THERE WAS PROBLEM HERE***//
//   ECap1Regs.ECCTL0.bit.INPUTSEL = 5; //connect ECap1 to INPUT6SELECT  //****THERE WAS PROBLEM HERE***//
//   ECAP_selectECAPInput(ECAP1_BASE, ECAP_INPUT_INPUTXBAR7);  //This statement assign ECCTL0.bit.INPUTSEL = 6.


   ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;      // Enable sync in
   ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;     // Pass through
   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter   //****PROBLEM WITH THIS STATEMENT****//
/*   ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units
   ECap1Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
*/

//   ECAP_enableLoadCounter(ECAP1_BASE); // ECAP_O_ECCTL2, ECAP_ECCTL2_SYNCI_EN;
//   ECAP_setSyncOutMode(ECAP1_BASE, ECAP_SYNC_OUT_SYNCI);  //ECAP_O_ECCTL2 , ECAP_SYNC_OUT_SYNCI pass through
   ECAP_startCounter(ECAP1_BASE); //ECAP_ECCTL2_TSCTRSTOP
   ECAP_enableTimeStampCapture(ECAP1_BASE); //ECAP_ECCTL1_CAPLDEN;
   ECAP_reArm(ECAP1_BASE); //ECAP_ECCTL2_REARM;

//   ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
//   ECap1Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = __interrupt

//>>> USE THIS LINE IF USING INTERRUPT DRIVEN REMOTE CONTROL OPERATION   ECAP_enableInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_2);  //at second event CAP4

   EDIS;

}

//
// ecap1_isr - ECAP1 ISR
//              Cap input is syc'ed to SYSCLKOUT so there may be
//              a +/- 1 cycle variation
//

//__interrupt void ecap1_isr(void)
void ISR_remote(void)
{

    if(flag_start_remote) {
        packet_remote++;
        if(ECap1Regs.CAP1 > 82000) {     //111900
            if(ECap1Regs.CAP1 < 162000) {  //for 1.12ms 112000
                Switch=Switch<<1;
            }
            else {  //for 2.25ms 225000
                Switch=Switch<<1;
                Switch|=0x01;
            }
        }
        if(ECap1Regs.CAP2 > 82000) {
            if(ECap1Regs.CAP2 < 162000) {
                Switch=Switch<<1;
            }
            else {
                Switch=Switch<<1;
                Switch|=0x01;
            }
        }
        if(ECap1Regs.CAP3 > 82000) {
            if(ECap1Regs.CAP3 < 162000) {
                Switch=Switch<<1;
            }
            else {
                Switch=Switch<<1;
                Switch|=0x01;
            }
        }
        if(ECap1Regs.CAP4 > 82000) {
            if(ECap1Regs.CAP4 < 162000) {
                Switch=Switch<<1;
            }
            else {
                Switch=Switch<<1;
                Switch|=0x01;
            }
        }
        if(packet_remote>=8) {
            Last_Switch = Switch;
            Prev_Switch = Switch;
            Switch = 0;
            packet_remote = 0;
            flag_start_remote = 0;
            ECAP_clearInterrupt(ECAP1_BASE,ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
            EALLOW;
            ECap1Regs.ECCTL2.bit.STOP_WRAP = 1;     //prepare for 1 cycle check
            ECAP_disableInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
            //>>> USE THIS LINE IF USING INTERRUPT DRIVEN REMOTE CONTROL OPERATION            ECAP_enableInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_2); //interrupt on 1 cycle
            EDIS;
        }
    }
    else {
        if(ECap1Regs.CAP2 < 1400000 && ECap1Regs.CAP2 > 1240000 )  {  // for 13.5 ms 1350000
            flag_start_remote = 1;
            packet_remote = 0;
            Longpress=DISABLE;
            ECAP_clearInterrupt(ECAP1_BASE,ECAP_ISR_SOURCE_CAPTURE_EVENT_2);
            EALLOW;
            ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;     //prepare for 4 cycle check
            EDIS;
            ECAP_reArm(ECAP1_BASE);
            EALLOW;
            ECAP_disableInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_2);
            //>>> USE THIS LINE IF USING INTERRUPT DRIVEN REMOTE CONTROL OPERATION            ECAP_enableInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4); //interrupt on 4 cycle
            EDIS;
        }
        if(ECap1Regs.CAP2 < 1230000 && ECap1Regs.CAP2 > 1000000 )  {  // for 11.25 ms 1125000
            Last_Switch = Prev_Switch;
            Longpress=ENABLE;
        }
    }




   ECap1IntCount++;
   ECap1PassCount++;

/*
   ECap1Regs.ECCLR.bit.CEVT4 = 1;
   ECap1Regs.ECCLR.bit.INT = 1;
   ECap1Regs.ECCTL2.bit.REARM = 1;

   //
   // Acknowledge this __interrupt to receive more __interrupts from group 4
   //
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
*/

   //
   // Clear interrupt flags for more interrupts.
   //
   ECAP_clearInterrupt(ECAP1_BASE,ECAP_ISR_SOURCE_CAPTURE_EVENT_2);
   ECAP_clearInterrupt(ECAP1_BASE,ECAP_ISR_SOURCE_CAPTURE_EVENT_4);

   ECAP_clearGlobalInterrupt(ECAP1_BASE);

   //
   // Start eCAP
   //
   ECAP_reArm(ECAP1_BASE);

   //
   // Acknowledge the group interrupt for more interrupts.
   //
   Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);

/****** THE PROBLEM IS WITH ACKNOWLEDGING INTERRUPT OR RESETING INTERRUPT *****/ //?????

}

void configCLAMemory(void)
{

    //
    // Copy over CLA Math tables from FLASH to RAM
    //
    memcpy((uint32_t *)&CLA1mathTablesRunStart, (uint32_t *)&CLA1mathTablesLoadStart,
            (uint32_t)&CLA1mathTablesLoadSize);

    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS6, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS6, MEMCFG_CLA_MEM_DATA);

}

//
// No more.
//
