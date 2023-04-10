
#include "hmi.h"

struct IR_VARS ir=IR_VARS_DEFAULTS;

extern struct LCD_VARS lcd;
extern struct COMMON_FLAG common_flag;
extern struct COMMAND command;
extern struct UID uid;

long int Decimizer[]={1,    1,  1,  10, 1,  100,    1,  1,  100,    1,  100000, 100,    1,  1,  100,    1,  1,  100,    10, 1,  100000, 100,    10, 10, 1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1000000,    1,  100,    1000000,    1,  100,    1,  1000000,    1,  1000,   1000,   1000,   1000,   1,  1,  1,  1,  100,    100,    100,    100,    10000,  10000,  10000,  10000,  10, 10, 10, 10, 10, 10, 100000, 10000000,   1};
float32_t Constants_Default[]={0,   0,  0,  0,  30, 0.4,    2,  8,  9.04,   2,  0.0001, 0.5,    0,  30, 0.4,    2,  8,  9.04,   0,  2,  0.0001, 0.5,    25, -25,    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0.00002,    112,    0.2,    0.00002,    112,    0.2,    500,    0.00002,    0,  31.44,  31.39,  31.44,  31.39,  100,    -100,   100,    -100,   0.7,    0.01,   0.7,    0.01,   0.2364, 0.2349, 0.2364, 0.2349, 25, -25,    25, -25,    25, -25,    0.99373,    0.0031317,  0};

extern volatile float32_t *Ptr_Constant;

// InitECapture - Initialize ECAP1 configurations
void InitECapture()
{
    EALLOW;
    InputXbarRegs.INPUT7SELECT = 16;         // Set eCAP1 source to GPIO-pin
    EDIS;

    GPIO_SetupPinOptions(16, GPIO_INPUT, GPIO_ASYNC);

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

   // Initialize counters:
      ir.ECap1IntCount = 0;
      ir.ECap1PassCount = 0;
}

//
// ecap1_isr - ECAP1 ISR
//              Cap input is syc'ed to SYSCLKOUT so there may be
//              a +/- 1 cycle variation
//

void remote_operation(void)     //capturing 2nd or 4th event
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

//__interrupt void ecap1_isr(void)
void ISR_remote(void)   //IR code decode logic giving received code/switch value in "ir.Last_Switch"
{

    if(ir.flag_start_remote) {
        ir.packet_remote++;
        if(ECap1Regs.CAP1 > 82000) {     //111900
            if(ECap1Regs.CAP1 < 162000) {  //for 1.12ms 112000
                ir.Switch=ir.Switch<<1;
            }
            else {  //for 2.25ms 225000
                ir.Switch=ir.Switch<<1;
                ir.Switch|=0x01;
            }
        }
        if(ECap1Regs.CAP2 > 82000) {
            if(ECap1Regs.CAP2 < 162000) {
                ir.Switch=ir.Switch<<1;
            }
            else {
                ir.Switch=ir.Switch<<1;
                ir.Switch|=0x01;
            }
        }
        if(ECap1Regs.CAP3 > 82000) {
            if(ECap1Regs.CAP3 < 162000) {
                ir.Switch=ir.Switch<<1;
            }
            else {
                ir.Switch=ir.Switch<<1;
                ir.Switch|=0x01;
            }
        }
        if(ECap1Regs.CAP4 > 82000) {
            if(ECap1Regs.CAP4 < 162000) {
                ir.Switch=ir.Switch<<1;
            }
            else {
                ir.Switch=ir.Switch<<1;
                ir.Switch|=0x01;
            }
        }
        if(ir.packet_remote>=8) {
            ir.Last_Switch = ir.Switch;
            ir.Prev_Switch = ir.Switch;
            ir.Switch = 0;
            ir.packet_remote = 0;
            ir.flag_start_remote = 0;
            ECAP_clearInterrupt(ECAP1_BASE,ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
            EALLOW;
            ECap1Regs.ECCTL2.bit.STOP_WRAP = 1;     //prepare for 2 cycle check
            ECAP_disableInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
            //>>> USE THIS LINE IF USING INTERRUPT DRIVEN REMOTE CONTROL OPERATION            ECAP_enableInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_2); //interrupt on 1 cycle
            EDIS;
        }
    }
    else {
        if(ECap1Regs.CAP2 < 1400000 && ECap1Regs.CAP2 > 1240000 )  {  // for 13.5 ms 1350000
            ir.flag_start_remote = 1;
            ir.packet_remote = 0;
            ir.Longpress=DISABLE;
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
        else if(ECap1Regs.CAP2 < 1230000 && ECap1Regs.CAP2 > 1000000 )  {  // for 11.25 ms 1125000
            ir.Last_Switch = ir.Prev_Switch;
            ir.Longpress=ENABLE;
        }
    }


    ir.ECap1IntCount++;
    ir.ECap1PassCount++;

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

void reset_ecap_counter_IR_remote(void)
{
    if ( ECap1Regs.TSCTR > 0x007FFFFF) {    //reset ecap counter for IR remote
        ECap1Regs.TSCTR =0;
        ECAP_reArm(ECAP1_BASE); //ECAP_ECCTL2_REARM;
    }
}

void remote_key_display_state_machine(void) //STATE MACHINE for IR switch and LCD display
{

    ActionOn_UpDownKeyFunc();

    if (ir.Last_Switch==OnOff_Key) {
        ir.Last_Switch = 0;
        //Action5
        LEDRed = ON;

        ActionOn_OnOff_Key_and_Screen_count();  //Provide action on OnOff key

    }

    else if (ir.Last_Switch==LEFT_Key) { //Not used
        ir.Last_Switch = 0;
        //Action3
        LEDRed = ON;
//        LCD_ACTION = SCREEN1R1;
        //Test3=3;
    }

    else if (ir.Last_Switch==RIGHT_Key) { //Not used
        ir.Last_Switch = 0;
        //Action4
        LEDRed = ON;
//        LCD_ACTION = SCREEN1R2;
        //Test3=4;
    }

    else if (ir.Last_Switch==MODE_Key) { //Not used
        ir.Last_Switch = 0;
        //Action6
        LEDRed = ON;
//        LCD_ACTION = CLEAR_SCREEN;
       // Test3=6;
        //
    }

    ir.Last_Switch = 0;

    if (CONTROL_STATE == FAULT_STATE) { //Display fault, lock screen till it clear
        if (lcd.Screen_count != 9) {
//        lcd.Screen_count = 9;           //This will lock screen
//        lcd.LCD_ACTION = SCREEN9R1;     //
        }
    }

}

void ActionOn_UpDownKeyFunc(void)
{

    if (lcd.UpDownKeyFunc==KeyForScreenMov) {
        if (ir.Last_Switch==UP_Key) {
            ir.Last_Switch = 0;
            //Action1
            LEDRed = ON;
            //Test3=1;
            lcd.Screen_count++;
            if (lcd.Screen_count>=8) lcd.Screen_count=1;
        }
        else if (ir.Last_Switch==DOWN_Key) {
            ir.Last_Switch = 0;
            //Action2
            LEDRed = ON;
//            LCD_ACTION = LINE2;
            //Test3=2;
            lcd.Screen_count--;
            if (lcd.Screen_count==0) lcd.Screen_count=7;

        }
    }
    else if (lcd.UpDownKeyFunc==KeyForPasswordConst) {
        if (ir.Last_Switch==UP_Key) {
            ir.Last_Switch = 0;
            //Action1
            LEDRed = ON;

            if (ir.Longpress)
                lcd.PasswordConst=lcd.PasswordConst+10;
            else
                lcd.PasswordConst++;

            //Test3=1;
        }
        else if (ir.Last_Switch==DOWN_Key) {
            ir.Last_Switch = 0;
            //Action2
            LEDRed = ON;

            if (ir.Longpress)
                lcd.PasswordConst=lcd.PasswordConst-10;
            else
                lcd.PasswordConst--;

            //Test3=2;
        }
    }

    else if (lcd.UpDownKeyFunc==KeyForChangeIDofConst) {
        if (ir.Last_Switch==UP_Key) {
            ir.Last_Switch = 0;
            //Action1
            LEDRed = ON;

            if (ir.Longpress)
                lcd.ID_Const=lcd.ID_Const+5;
            else {
                lcd.ID_Const++;
                if (lcd.ID_Const >= 99) {
                    lcd.Screen_count=1;
                    lcd.UpDownKeyFunc=KeyForScreenMov;
                    lcd.Command_Const=DISABLE;
                    lcd.Command_PwD=OFF;
                    lcd.ID_Const = 0;
                    lcd.PasswordConst=0;
                }
            }

            //Test3=1;
        }
        else if (ir.Last_Switch==DOWN_Key) {
            ir.Last_Switch = 0;
            //Action2
            LEDRed = ON;

            if (ir.Longpress) {
                lcd.ID_Const=lcd.ID_Const-5;
                if (lcd.ID_Const<=0) lcd.ID_Const=1;
            }
            else{
                lcd.ID_Const--;
                if (lcd.ID_Const<=0) lcd.ID_Const=1;
            }

            //Test3=2;
        }
        pickup_constant();
        lcd.Constant=(*Ptr_Constant)*(float)Decimizer[lcd.ID_Const];  //To make it whole number for LCDdisplay purpose
    }

    else if (lcd.UpDownKeyFunc==KeyForChangeofConst) {
        if (ir.Last_Switch==UP_Key) {
            ir.Last_Switch = 0;
            //Action1
            LEDRed = ON;

            if (ir.Longpress)
                lcd.Constant=lcd.Constant+10;
            else
                lcd.Constant++;

            //Test3=1;
        }
        else if (ir.Last_Switch==DOWN_Key) {
            ir.Last_Switch = 0;
            //Action2
            LEDRed = ON;

            if (ir.Longpress)
                lcd.Constant=lcd.Constant-10;
            else
                lcd.Constant--;

            //Test3=2;
        }

        *Ptr_Constant = lcd.Constant/((float)Decimizer[lcd.ID_Const]);  //To convert back LCDdisplayed value to actual value
        lcd.HMI_Change_Constants=1;
    }

}

void ActionOn_OnOff_Key_and_Screen_count(void)  //function for OnOff button pressed
{
        if ((command.OnOffCh1)&&(lcd.Screen_count==1)) {
            command.OnOffCh1=OFF;
//            command_OnOffCh1=OFF;
        }
        else if ((!command.OnOffCh1)&&(lcd.Screen_count==1)) {
            command.OnOffCh1=ON;
//            command_OnOffCh1=ON;
        }
        else if ((command.OnOffCh2)&&(lcd.Screen_count==2)) {
            command.OnOffCh2=OFF;
//            command_OnOffCh2=OFF;
        }
        else if ((!command.OnOffCh2)&&(lcd.Screen_count==2)) {
            command.OnOffCh2=ON;
//            command_OnOffCh2=ON;
        }

        else if (lcd.Screen_count==9) {      //Clear the fault, over the fault screen if pressed OnOff button
            command.Clear_OCfault  = 1;
//            command_Clear_OCfault  = 1;
        }

        else if ((lcd.Command_PwD)&&(lcd.Screen_count==7)) {   //Check password match if pressed OnOff button, with the help of flag_commond_PwD
            if (lcd.PasswordConst==85) {
                lcd.Screen_count=8;
                lcd.UpDownKeyFunc=KeyForChangeIDofConst;  ///Configure UpDownKeyFunc for change of ID of constant
            } else {
                lcd.Screen_count=1;
                lcd.UpDownKeyFunc=KeyForScreenMov;
                lcd.PasswordConst=0;
            }
            lcd.Command_PwD=OFF;
        }

        else if ((!lcd.Command_PwD)&&(lcd.Screen_count==7)) {  //Configure UpDownKeyFunc for password
            lcd.UpDownKeyFunc=KeyForPasswordConst;
            lcd.Command_PwD=ON;
        }

        if ((lcd.Command_Const)&&(lcd.Screen_count==8)){    //Configure UpDownKeyFunc for change of constant if pressed OnOff button
            lcd.UpDownKeyFunc=KeyForChangeofConst;
            lcd.Command_Const=DISABLE;
        } else if ((!lcd.Command_Const)&&(lcd.Screen_count==8)){
            lcd.UpDownKeyFunc=KeyForChangeIDofConst;
            lcd.Command_Const=ENABLE;
        }

//        LCD_ACTION = SCREEN1R2;
//        Test3=5;

}
