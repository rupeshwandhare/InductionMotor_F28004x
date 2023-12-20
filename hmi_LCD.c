
#include "hmi.h"

//
struct LCD_VARS lcd=LCD_VARS_DEFAULTS;
struct COMMON_FLAG common_flag=COMMON_FLAG_DEFAULTS;
struct COMMAND command=COMMAND_DEFAULTS;
struct UID uid=UID_DEFAULTS;


//====

//===

void init_critical_gpio(void)
{
    //======
    EALLOW;
    //PowerRelay
//    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
//    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;


    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;


    //DisablePWM1 on GPIO22
    GpioCtrlRegs.GPAAMSEL.bit.GPIO22 = 0;//For GPIO22 and GPIO23 we need to manualy config them from analog to digital mode
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;

    //DisablePWM2 on GPIO26, Check whether other pin can be selected for DisablePWM2 as GPIO26 is I2C pin
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;   //TestPin shared with I2C
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;
    EDIS;



#if VIENNA_CONTROL_RUNNING_ON == CLA_CORE
    GPIO_setMasterCore(14, GPIO_CORE_CPU1_CLA1);
    GPIO_setMasterCore(22, GPIO_CORE_CPU1_CLA1);
    GPIO_setMasterCore(26, GPIO_CORE_CPU1_CLA1);
#endif

    GpioDataRegs.GPADAT.bit.GPIO14 = 0;
    GpioDataRegs.GPADAT.bit.GPIO22 = 1;
    GpioDataRegs.GPADAT.bit.GPIO26 = 1;
}


void init_hmi_gpio(void)
{
    EALLOW;
    //LEDRed
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;
//
/////////////////////////////////////////////////
//    //LEDRed 23
//    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;
//    GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;
/////////////////////////////////////////////////

    //SCI Boot capacitor
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;
    //ExtSwitch
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 2; //6sample qualification
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 0;
    EDIS;

    GpioDataRegs.GPADAT.bit.GPIO15 = 0;
    GpioDataRegs.GPADAT.bit.GPIO17 = 0;


}




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



