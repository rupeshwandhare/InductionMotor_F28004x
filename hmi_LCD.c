
#include "hmi.h"

//
struct LCD_VARS lcd=LCD_VARS_DEFAULTS;
struct COMMON_FLAG common_flag=COMMON_FLAG_DEFAULTS;
struct COMMAND command=COMMAND_DEFAULTS;
struct UID uid=UID_DEFAULTS;

void (*LCD_Ptr)(void);

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

char screen6r1text[] =      "Dt1=  . %POT=  %";
char screen6r2text[] =      "Dt2=  . %       ";

char screen7r1text[] =      "To Change Para: ";
char screen7r2text[] =      "PsW:            ";

char screen8r1text[] =      "Index:     ;Md= ";
char screen8r2text[] =      "Val=            ";

char screen9r1text[] =      "Fault:          ";
char screen9r2text[] =      "Fixed it?       ";


//====

//===

void init_critical_gpio(void)
{
    //======
    EALLOW;
    //PowerRelay
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;

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


    lcd.LCD_ACTION = CLEAR_SCREEN;

    LCD_Ptr = &LCD_selection;

    lcd.UpDownKeyFunc=KeyForScreenMov;
    lcd.Screen_count = 1;

}

void LoopforLCDPtr(void)
{
    (*LCD_Ptr)();  ////in 1 millisecond subroutine
}

void StateControl_RemoteScreen(void)  //Sequencing in a screen and different screen selection
{
    if (lcd.Prev_Screen_count!=lcd.Screen_count) {  //For command CLEAR_SCREEN if different screen is selected.
        lcd.Prev_Screen_count=lcd.Screen_count;
        lcd.LCD_ACTION = CLEAR_SCREEN;
        lcd.Seq_screen=1;
        return; //Return to clear the screen first.
    }

    switch (lcd.Screen_count) { //Different screens to display
        case 1:
            switch (lcd.Seq_screen){ //Sequencing in the screen    //1st line text, select 2nd line, 2nd line text, select 1st line
                case 1:
                    lcd.LCD_ACTION = SCREEN1R1;
                    lcd.Seq_screen++;
                    break;
                case 2:
                    lcd.LCD_ACTION = LINE2;
                    lcd.Seq_screen++;
                    break;
                case 3:
                    lcd.LCD_ACTION = SCREEN1R2;
                    lcd.Seq_screen++;
                    break;
                case 4:
                    lcd.LCD_ACTION = LINE1;
                    lcd.Seq_screen=1;
                    break;
                default:
                    break;
            }
            break;

        case 2:
            switch (lcd.Seq_screen){ //Sequencing in the screen    //1st line text, select 2nd line, 2nd line text, select 1st line
                case 1:
                    lcd.LCD_ACTION = SCREEN2R1;
                    lcd.Seq_screen++;
                    break;
                case 2:
                    lcd.LCD_ACTION = LINE2;
                    lcd.Seq_screen++;
                    break;
                case 3:
                    lcd.LCD_ACTION = SCREEN2R2;
                    lcd.Seq_screen++;
                    break;
                case 4:
                    lcd.LCD_ACTION = LINE1;
                    lcd.Seq_screen=1;
                    break;
                default:
                    break;
            }
            break;

        case 3:
            switch (lcd.Seq_screen){ //Sequencing in the screen    //1st line text, select 2nd line, 2nd line text, select 1st line
                case 1:
                    lcd.LCD_ACTION = SCREEN3R1;
                    lcd.Seq_screen++;
                    break;
                case 2:
                    lcd.LCD_ACTION = LINE2;
                    lcd.Seq_screen++;
                    break;
                case 3:
                    lcd.LCD_ACTION = SCREEN3R2;
                    lcd.Seq_screen++;
                    break;
                case 4:
                    lcd.LCD_ACTION = LINE1;
                    lcd.Seq_screen=1;
                    break;
                default:
                    break;
            }
            break;

       case 4:
           switch (lcd.Seq_screen){ //Sequencing in the screen    //1st line text, select 2nd line, 2nd line text, select 1st line
                case 1:
                    lcd.LCD_ACTION = SCREEN4R1;
                    lcd.Seq_screen++;
                    break;
                case 2:
                    lcd.LCD_ACTION = LINE2;
                    lcd.Seq_screen++;
                    break;
                case 3:
                    lcd.LCD_ACTION = SCREEN4R2;
                    lcd.Seq_screen++;
                    break;
                case 4:
                    lcd.LCD_ACTION = LINE1;
                    lcd.Seq_screen=1;
                    break;
                default:
                    break;
           }
           break;

       case 5:
           switch (lcd.Seq_screen){ //Sequencing in the screen    //1st line text, select 2nd line, 2nd line text, select 1st line
               case 1:
                   lcd.LCD_ACTION = SCREEN5R1;
                   lcd.Seq_screen++;
                   break;
               case 2:
                   lcd.LCD_ACTION = LINE2;
                   lcd.Seq_screen++;
                   break;
               case 3:
                   lcd.LCD_ACTION = SCREEN5R2;
                   lcd.Seq_screen++;
                   break;
               case 4:
                   lcd.LCD_ACTION = LINE1;
                   lcd.Seq_screen=1;
                   break;
               default:
                   break;
           }
           break;

       case 6:
           switch (lcd.Seq_screen){
               case 1:
                   lcd.LCD_ACTION = SCREEN6R1;
                   lcd.Seq_screen++;
                   break;
               case 2:
                   lcd.LCD_ACTION = LINE2;
                   lcd.Seq_screen++;
                   break;
               case 3:
                   lcd.LCD_ACTION = SCREEN6R2;
                   lcd.Seq_screen++;
                   break;
               case 4:
                   lcd.LCD_ACTION = LINE1;
                   lcd.Seq_screen=1;
                   break;
               default:
                   break;
           }
           break;

       case 7:
           switch (lcd.Seq_screen){
               case 1:
                   lcd.LCD_ACTION = SCREEN7R1;
                   lcd.Seq_screen++;
                   break;
               case 2:
                   lcd.LCD_ACTION = LINE2;
                   lcd.Seq_screen++;
                   break;
               case 3:
                   lcd.LCD_ACTION = SCREEN7R2;
                   lcd.Seq_screen++;
                   break;
               case 4:
                   lcd.LCD_ACTION = LINE1;
                   lcd.Seq_screen=1;
                   break;
               default:
                   break;
          }
           break;

       case 8:
           switch (lcd.Seq_screen){
               case 1:
                   lcd.LCD_ACTION = SCREEN8R1;
                   lcd.Seq_screen++;
                   break;
               case 2:
                   lcd.LCD_ACTION = LINE2;
                   lcd.Seq_screen++;
                   break;
               case 3:
                   lcd.LCD_ACTION = SCREEN8R2;
                   lcd.Seq_screen++;
                   break;
               case 4:
                   lcd.LCD_ACTION = LINE1;
                   lcd.Seq_screen=1;
                   break;
               default:
                   break;
           }
           break;


        default:
            break;
    }
}

void LCD_selection(void) // LCD action and Text selections
{
   switch (lcd.LCD_ACTION) {    //keep LCD_ACTION can have only one value
       case LCD_NO_ACTION:
           break;
       case SCREEN1R1:
/*
           //char screen1r1text[] =      "Ch1=   ;Vdc=   V";


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
           lcd.x_hecto = (unsigned int)(VIENNA_vDCMeas_pu*0.01);
           lcd.x_deca = (unsigned int)((float)(VIENNA_vDCMeas_pu - (float)(lcd.x_hecto*100))* 0.1);
           lcd.x_int = (unsigned int) ((float)VIENNA_vDCMeas_pu - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen1r1text[12] = (lcd.x_hecto+'0');
           screen1r1text[13] = (lcd.x_deca+'0');
           screen1r1text[14] = (lcd.x_int+'0');

           lcd.string_LCD = screen1r1text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN1R2:
           //char screen1r2text[] =      "V1=   V;I1=  . A";

           lcd.x_hecto = (unsigned int)(sensor_v_pv1_fltr*0.01);
           lcd.x_deca = (unsigned int)((float)(sensor_v_pv1_fltr - (float)(lcd.x_hecto*100))* 0.1);
           lcd.x_int = (unsigned int) ((float)sensor_v_pv1_fltr - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen1r2text[3] = (lcd.x_hecto+'0');
           screen1r2text[4] = (lcd.x_deca+'0');
           screen1r2text[5] = (lcd.x_int+'0');

//           ia = 5.2;
           lcd.x_hecto = (unsigned int)(sensor_i_pv1_fltr*0.1);
           lcd.x_deca = (unsigned int)((float)(sensor_i_pv1_fltr - lcd.x_hecto*10));
           lcd.x_int = (unsigned int) (float)(sensor_i_pv1_fltr*10 - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen1r2text[11] = (lcd.x_hecto+'0');
           screen1r2text[12] = (lcd.x_deca+'0');
           screen1r2text[14] = (lcd.x_int+'0');

           lcd.string_LCD = screen1r2text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
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


           if(CONTROL_STATE2 == VPV_CONTROL2) {
               screen2r1text[4] = 'O';
               screen2r1text[5] = 'n';
               screen2r1text[6] = ' ';
           } else if(CONTROL_STATE2 == IDLE_STATE2) {
               screen2r1text[4] = 'O';
               screen2r1text[5] = 'f';
               screen2r1text[6] = 'f';
           } else if(CONTROL_STATE2 == SLEW_CONTROL2) {
               screen2r1text[4] = 'A';
               screen2r1text[5] = 'c';
               screen2r1text[6] = 'c';
           } else if(CONTROL_STATE2 == FAULT_STATE) {
               screen2r1text[4] = 'F';
               screen2r1text[5] = 'l';
               screen2r1text[6] = 't';
           } else if(CONTROL_STATE2 == VDC_CHARGING) {
               screen2r1text[4] = 'C';
               screen2r1text[5] = 'h';
               screen2r1text[6] = 'g';
           } else if(CONTROL_STATE2 == DEACCELERATION2) {
               screen2r1text[4] = 'D';
               screen2r1text[5] = 'c';
               screen2r1text[6] = 'c';
           }

//           va = 23.1;
           lcd.x_hecto = (unsigned int)(VIENNA_vDCMeas_pu*0.01);
           lcd.x_deca = (unsigned int)((float)(VIENNA_vDCMeas_pu - (float)(lcd.x_hecto*100))* 0.1);
           lcd.x_int = (unsigned int) ((float)VIENNA_vDCMeas_pu - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen2r1text[12] = (lcd.x_hecto+'0');
           screen2r1text[13] = (lcd.x_deca+'0');
           screen2r1text[14] = (lcd.x_int+'0');

           lcd.string_LCD = screen2r1text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN2R2:
           //char screen2r2text[] =      "V2=   V;I2=  . A";
//           va = 231;
           lcd.x_hecto = (unsigned int)(sensor_v_pv2_fltr*0.01);
           lcd.x_deca = (unsigned int)((float)(sensor_v_pv2_fltr - (float)(lcd.x_hecto*100))* 0.1);
           lcd.x_int = (unsigned int) ((float)sensor_v_pv2_fltr - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen2r2text[3] = (lcd.x_hecto+'0');
           screen2r2text[4] = (lcd.x_deca+'0');
           screen2r2text[5] = (lcd.x_int+'0');

//           ia = 75.2;
           lcd.x_hecto = (unsigned int)(sensor_i_pv2_fltr*0.1);
           lcd.x_deca = (unsigned int)((float)(sensor_i_pv2_fltr - lcd.x_hecto*10));
           lcd.x_int = (unsigned int) (float)(sensor_i_pv2_fltr*10 - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen2r2text[11] = (lcd.x_hecto+'0');
           screen2r2text[12] = (lcd.x_deca+'0');
           screen2r2text[14] = (lcd.x_int+'0');

           lcd.string_LCD = screen2r2text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN3R1:
           //char screen3r1text[] =      "Ir1=  . ;Vr1=   ";
           lcd.x_hecto = (unsigned int)(il1_control_ref*0.1);
           lcd.x_deca = (unsigned int)((float)(il1_control_ref - lcd.x_hecto*10));
           lcd.x_int = (unsigned int) (float)(il1_control_ref*10 - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen3r1text[4] = (lcd.x_hecto+'0');
           screen3r1text[5] = (lcd.x_deca+'0');
           screen3r1text[7] = (lcd.x_int+'0');

           lcd.x_hecto = (unsigned int)(vpv1_control_ref*0.01);
           lcd.x_deca = (unsigned int)((float)(vpv1_control_ref - (float)(lcd.x_hecto*100))* 0.1);
           lcd.x_int = (unsigned int) ((float)vpv1_control_ref - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen3r1text[13] = (lcd.x_hecto+'0');
           screen3r1text[14] = (lcd.x_deca+'0');
           screen3r1text[15] = (lcd.x_int+'0');

           lcd.string_LCD = screen3r1text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN3R2:
           //char screen3r2text[] =      "IL1=  . ;Vp1=   ";
           lcd.x_hecto = (unsigned int)(VIENNA_iL1Meas_pu*0.1);
           lcd.x_deca = (unsigned int)((float)(VIENNA_iL1Meas_pu - lcd.x_hecto*10));
           lcd.x_int = (unsigned int) (float)(VIENNA_iL1Meas_pu*10 - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen3r2text[4] = (lcd.x_hecto+'0');
           screen3r2text[5] = (lcd.x_deca+'0');
           screen3r2text[7] = (lcd.x_int+'0');

           lcd.x_hecto = (unsigned int)(VIENNA_vPV1Meas_pu*0.01);
           lcd.x_deca = (unsigned int)((float)(VIENNA_vPV1Meas_pu - (float)(lcd.x_hecto*100))* 0.1);
           lcd.x_int = (unsigned int) ((float)VIENNA_vPV1Meas_pu - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen3r2text[13] = (lcd.x_hecto+'0');
           screen3r2text[14] = (lcd.x_deca+'0');
           screen3r2text[15] = (lcd.x_int+'0');

           lcd.string_LCD = screen3r2text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN4R1:
           //char screen4r1text[] =      "Ir2=  . ;Vr2=   ";
           lcd.x_hecto = (unsigned int)(il2_control_ref*0.1);
           lcd.x_deca = (unsigned int)((float)(il2_control_ref - lcd.x_hecto*10));
           lcd.x_int = (unsigned int) (float)(il2_control_ref*10 - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen4r1text[4] = (lcd.x_hecto+'0');
           screen4r1text[5] = (lcd.x_deca+'0');
           screen4r1text[7] = (lcd.x_int+'0');

           lcd.x_hecto = (unsigned int)(vpv2_control_ref*0.01);
           lcd.x_deca = (unsigned int)((float)(vpv2_control_ref - (float)(lcd.x_hecto*100))* 0.1);
           lcd.x_int = (unsigned int) ((float)vpv2_control_ref - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen4r1text[13] = (lcd.x_hecto+'0');
           screen4r1text[14] = (lcd.x_deca+'0');
           screen4r1text[15] = (lcd.x_int+'0');

           lcd.string_LCD = screen4r1text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN4R2:
           //char screen4r2text[] =      "IL2=  . ;Vp2=   ";
           lcd.x_hecto = (unsigned int)(VIENNA_iL2Meas_pu*0.1);
           lcd.x_deca = (unsigned int)((float)(VIENNA_iL2Meas_pu - lcd.x_hecto*10));
           lcd.x_int = (unsigned int) (float)(VIENNA_iL2Meas_pu*10 - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen4r2text[4] = (lcd.x_hecto+'0');
           screen4r2text[5] = (lcd.x_deca+'0');
           screen4r2text[7] = (lcd.x_int+'0');

           lcd.x_hecto = (unsigned int)(VIENNA_vPV2Meas_pu*0.01);
           lcd.x_deca = (unsigned int)((float)(VIENNA_vPV2Meas_pu - (float)(lcd.x_hecto*100))* 0.1);
           lcd.x_int = (unsigned int) ((float)VIENNA_vPV2Meas_pu - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen4r2text[13] = (lcd.x_hecto+'0');
           screen4r2text[14] = (lcd.x_deca+'0');
           screen4r2text[15] = (lcd.x_int+'0');

           lcd.string_LCD = screen4r2text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN5R1:
           //char screen5r1text[] =      "BoardTemp=  . C ";
           lcd.x_hecto = (unsigned int)(VIENNA_TEMPMeas_pu*0.1);
           lcd.x_deca = (unsigned int)((float)((VIENNA_TEMPMeas_pu - lcd.x_hecto*10)));
           lcd.x_int = (unsigned int) (float)((VIENNA_TEMPMeas_pu*10 - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10)));
           screen5r1text[10] = (lcd.x_hecto+'0');
           screen5r1text[11] = (lcd.x_deca+'0');
           screen5r1text[13] = (lcd.x_int+'0');

           lcd.string_LCD = screen5r1text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN5R2:
           //char screen5r2text[] =      "Vdc=   V;Vc= .  ";
           lcd.x_hecto = (unsigned int)(VIENNA_vDCMeas_pu*0.01);
           lcd.x_deca = (unsigned int)((float)(VIENNA_vDCMeas_pu - (float)(lcd.x_hecto*100))* 0.1);
           lcd.x_int = (unsigned int) ((float)VIENNA_vDCMeas_pu - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen5r2text[4] = (lcd.x_hecto+'0');
           screen5r2text[5] = (lcd.x_deca+'0');
           screen5r2text[6] = (lcd.x_int+'0');

           lcd.x_hecto = (unsigned int)(VIENNA_vCCMeas_pu);
           lcd.x_deca = (unsigned int)((float)(VIENNA_vCCMeas_pu - (float)(lcd.x_hecto) )*10);
           lcd.x_int = (unsigned int) ((float)((VIENNA_vCCMeas_pu - (float)(lcd.x_hecto))*10 - (float)(lcd.x_deca))*10);
           screen5r2text[12] = (lcd.x_hecto+'0');
           screen5r2text[14] = (lcd.x_deca+'0');
           screen5r2text[15] = (lcd.x_int+'0');

           lcd.string_LCD = screen5r2text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;

       case SCREEN6R1:
           //char screen6r1text[] =      "Dt1=  . %POT=  %";

           lcd.Duty_percent=(common_vars_duty1-0.030)*100; //(2x2x0.15usec)/(1/fsw)  2 for updowncount and 2 for at front and at back; Even after this 1% error at 50kHz due to IC response time
           if (lcd.Duty_percent<0.0) lcd.Duty_percent=0.0;
           lcd.x_hecto = (unsigned int)(lcd.Duty_percent*0.1);
           lcd.x_deca = (unsigned int)((float)(lcd.Duty_percent - lcd.x_hecto*10));
           lcd.x_int = (unsigned int) (float)(lcd.Duty_percent*10 - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen6r1text[4] = (lcd.x_hecto+'0');
           screen6r1text[5] = (lcd.x_deca+'0');
           screen6r1text[7] = (lcd.x_int+'0');

           lcd.Pot_percent=VIENNA_POTMeas_pu*100;
           lcd.x_hecto = (unsigned int)(lcd.Pot_percent*0.01);
           lcd.x_deca = (unsigned int)((float)(lcd.Pot_percent - (float)(lcd.x_hecto*100))* 0.1);
           lcd.x_int = (unsigned int) ((float)lcd.Pot_percent - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
//           screen6r1text[4] = (lcd.x_hecto+'0');
           screen6r1text[13] = (lcd.x_deca+'0');
           screen6r1text[14] = (lcd.x_int+'0');

           lcd.string_LCD = screen6r1text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN6R2:
           //char screen6r2text[] =      "Dt2=  . %       ";

           lcd.Duty_percent=(common_vars_duty2-0.030)*100; //(2x2x0.15usec)/(1/fsw)  2 for updowncount and 2 for at front and at back; Even after this 1% error at 50kHz due to IC response time
           if (lcd.Duty_percent<0.0) lcd.Duty_percent=0.0;
           lcd.x_hecto = (unsigned int)(lcd.Duty_percent*0.1);
           lcd.x_deca = (unsigned int)((float)(lcd.Duty_percent - lcd.x_hecto*10));
           lcd.x_int = (unsigned int) (float)(lcd.Duty_percent*10 - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10));
           screen6r2text[4] = (lcd.x_hecto+'0');
           screen6r2text[5] = (lcd.x_deca+'0');
           screen6r2text[7] = (lcd.x_int+'0');

           lcd.string_LCD = screen6r2text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;

       case SCREEN7R1:
           //char screen7r1text[] =      "To Change Para: ";
           screen7r1text[15] = ( (unsigned int)(lcd.UpDownKeyFunc)  +'0');

           lcd.string_LCD = screen7r1text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN7R2:
           //char screen7r2text[] =      "PsW:            ";
           //PasswordConst = 1243;
           lcd.x_kelo = (unsigned int)((float)(lcd.PasswordConst*0.001));
           lcd.x_hecto = (unsigned int)((float)((float)lcd.PasswordConst - (float)(lcd.x_kelo*1000))* 0.01);
           lcd.x_deca = (unsigned int) ((float)((float)lcd.PasswordConst - (float)(lcd.x_kelo*1000) - (float)(lcd.x_hecto*100))*0.1);
           lcd.x_int =  (unsigned int) ((float)((float)lcd.PasswordConst - (float)(lcd.x_kelo*1000) - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10)) );
           screen7r2text[4] = (lcd.x_kelo+'0');
           screen7r2text[5] = (lcd.x_hecto+'0');
           screen7r2text[6] = (lcd.x_deca+'0');
           screen7r2text[7] = (lcd.x_int+'0');


           lcd.string_LCD = screen7r2text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;

       case SCREEN8R1:
           //char screen8r1text[] =      "Index:     ;Md= ";
           lcd.x_deca = (unsigned int)(lcd.ID_Const*0.1);
           lcd.x_int = (unsigned int)(float)(lcd.ID_Const - (float)(lcd.x_deca*10));
           screen8r1text[6] = (lcd.x_deca+'0');
           screen8r1text[7] = (lcd.x_int+'0');

           screen8r1text[14] = ( (unsigned int)(lcd.UpDownKeyFunc)  +'0');

           lcd.string_LCD = screen8r1text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN8R2:
           //char screen8r2text[] =      "Val=            ";

           if (lcd.Constant<0.0) lcd.Dis_Constant=-lcd.Constant;
           else lcd.Dis_Constant=lcd.Constant;
           lcd.x_10mega = (unsigned int)((float)(lcd.Dis_Constant*0.0000001));
           lcd.x_mega = (unsigned int)((float)((float)lcd.Dis_Constant - (float)(lcd.x_10mega*10000000))* 0.000001);
           lcd.x_100kelo = (unsigned int) ((float)((float)lcd.Dis_Constant - (float)(lcd.x_10mega*10000000) - (float)(lcd.x_mega*1000000))*0.00001);
           lcd.x_10kelo = (unsigned int) ((float)((float)lcd.Dis_Constant - (float)(lcd.x_10mega*10000000) - (float)(lcd.x_mega*1000000) - (float)(lcd.x_100kelo*100000))*0.0001);
           lcd.x_kelo = (unsigned int) ((float)((float)lcd.Dis_Constant - (float)(lcd.x_10mega*10000000) - (float)(lcd.x_mega*1000000) - (float)(lcd.x_100kelo*100000)- (float)(lcd.x_10kelo*10000) )*0.001 );
           lcd.x_hecto = (unsigned int) ((float)((float)lcd.Dis_Constant - (float)(lcd.x_10mega*10000000) - (float)(lcd.x_mega*1000000) - (float)(lcd.x_100kelo*100000)- (float)(lcd.x_10kelo*10000) - (float)(lcd.x_kelo*1000) )*0.01 );
           lcd.x_deca = (unsigned int) ((float)((float)lcd.Dis_Constant - (float)(lcd.x_10mega*10000000) - (float)(lcd.x_mega*1000000) - (float)(lcd.x_100kelo*100000)- (float)(lcd.x_10kelo*10000) - (float)(lcd.x_kelo*1000) - (float)(lcd.x_hecto*100))*0.1 );
           lcd.x_int = (unsigned int) ((float)((float)lcd.Dis_Constant - (float)(lcd.x_10mega*10000000) - (float)(lcd.x_mega*1000000) - (float)(lcd.x_100kelo*100000)- (float)(lcd.x_10kelo*10000) - (float)(lcd.x_kelo*1000) - (float)(lcd.x_hecto*100) - (float)(lcd.x_deca*10)));


           screen8r2text[4] = (lcd.x_10mega+'0');
           screen8r2text[5] = (lcd.x_mega+'0');
           screen8r2text[6] = (lcd.x_100kelo+'0');
           screen8r2text[7] = (lcd.x_10kelo+'0');
           screen8r2text[8] = (lcd.x_kelo+'0');
           screen8r2text[9] = (lcd.x_hecto+'0');
           screen8r2text[10] = (lcd.x_deca+'0');
           screen8r2text[11] = (lcd.x_int+'0');

           lcd.string_LCD = screen8r2text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
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
           } else if(VIENNA_boardStatus == vpv1_overvoltage) {
               screen9r1text[6] = 'O';
               screen9r1text[7] = 'V';
               screen9r1text[8] = 'P';
               screen9r1text[9] = 'V';
           } else if(VIENNA_boardStatus == vpv2_overvoltage) {
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

           lcd.string_LCD = screen9r1text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SCREEN9R2:
           //char screen9r2text[] =      "Fixed it?       ";

           lcd.string_LCD = screen9r2text;
           lcd.New_string = 1;
           LCD_Ptr = &Array_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;


       case CLEAR_SCREEN:
           lcd.command_LCD = 0x01;
           lcd.New_command = 1;
           LCD_Ptr = &Command_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case LINE1:
           lcd.command_LCD = 0x80;
           lcd.New_command = 1;
           LCD_Ptr = &Command_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case LINE2:
           lcd.command_LCD = 0xC0;
           lcd.New_command = 1;
           LCD_Ptr = &Command_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case CURSOR_ON:
           lcd.command_LCD = 0x0F;
           lcd.New_command = 1;
           LCD_Ptr = &Command_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case CURSOR_OFF:
           lcd.command_LCD = 0x0C;
           lcd.New_command = 1;
           LCD_Ptr = &Command_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SHIFT_RIGHT:
           lcd.command_LCD = 0x1C;
           lcd.New_command = 1;
           LCD_Ptr = &Command_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       case SHIFT_LEFT:
           lcd.command_LCD = 0x18;
           lcd.New_command = 1;
           LCD_Ptr = &Command_Process;
           lcd.LCD_ACTION = LCD_NO_ACTION;
           break;
       default:
           break;
   }
}

void Command_Process(void)
{
    ENot = SET;
    if (lcd.j_LCD<1) {
        lcd.char_LCD = lcd.command_LCD;
        lcd.j_LCD++;
    }
    else {
        lcd.j_LCD=0;
        lcd.New_command = 0;
        LCD_Ptr = &LCD_selection;
        return;
    }
    RSNot = SET;             // => RS = 0
    LCD_Ptr = &Byte_Process;
}
void Array_Process(void)
{
    ENot = SET;
    if (lcd.i_LCD<16) {   //WE CAN TARGET ONLY SELECTED CHARACTERS (INSTEAD OF ALL 16 CHARA) IF WANT TO INCREASE SPEED OF DISPLAY AT SLOWER FUNCTION RATE (LCD_Ptr) //row length = 31
        lcd.char_LCD = *(lcd.string_LCD+lcd.i_LCD);                                    // for(i=0;str[i]!='\0';i++)
        lcd.i_LCD++;
    }
    else {
        lcd.i_LCD = 0;
        lcd.New_string = 0;
        LCD_Ptr = &LCD_selection;
        return;
    }
    RS = SET;             // => RS = 1
    LCD_Ptr = &Byte_Process;
}
void Byte_Process(void)
{
    char y;
    lcd.LSNibble = lcd.char_LCD&0x0F;
    y = lcd.char_LCD&0xF0;
    lcd.MSNibble = y>>4;
    LCD_Ptr = &MSNibble_Process;
}
void MSNibble_Process(void)
{
    ENot = SET;
    Lcd_Nibble_Port(lcd.MSNibble);             //Data transfer
    LCD_Ptr = &LSNibble_Process;
}
void LSNibble_Process(void)
{
    ENot = SET;
    Lcd_Nibble_Port(lcd.LSNibble);

    if (lcd.New_string) {
        LCD_Ptr = &Array_Process;
    }
    if (lcd.New_command) {
        LCD_Ptr = &Command_Process;
    }
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



