
#include "hmi.h"
//#include "HVACI_Sensorless-Settings.h"

//extern struct PWM_VARS pwm;
//extern struct DEBUG_CONFIG debug;

extern float graph[200];
Uint16 ch_array_length = 200;
float ch_1=0, ch_2=0;
Uint16 sample=3, set_view=2;

float va, vb, vc;
float angle_rad=0.0;
float freq1=1.0;
float mo1=0.4;

extern float vars1;
extern float vars2;
extern float vars3;
extern float vars4;
Uint16 select_varDAC=1;

/*
void Setup_Debug(void)
{
    debug.hw_debug = 0;
    debug.sw_debug = 1;
    if (debug.hw_debug) {
        Setup_ePWM5(); //20KHz
    }
}
*/

Uint16 skip=0;
//For graph
void sw_debug(void)
{
    static Uint16 alt=0, k=0;
    //read 3rd sample only


    if (angle_rad>=(3141.59265))  //FOR TESTING GRAPH 2pi*19  // approximately 50sec reset time
        angle_rad = (0.0);
    angle_rad = 0.0314159265 * freq1 + angle_rad;  //radian= 0.039269908 for 50Hz 8kHz; radian= 0.0471238898 for 60Hz 8kHz;
    va = mo1*sinf(angle_rad);           // include 3rd harmonic injection if machine is in star connection
    vb = mo1*sinf(angle_rad-2.094395102393);
    vc = mo1*sinf(angle_rad+2.094395102393);

    if(sample > 2) {

        if(set_view==1)
        {
//            ch_1 = pwm.ma;
//            ch_2 = pwm.ma;
        }
        else if(set_view==2) {
            ch_1 = vars1;
            ch_2 = vars2;
        }
        else if(set_view==3) {
            ch_1 = vars3;
            ch_2 = vars4;
        }
        else { // set_view =0, view of mc
//            ch_1 = pwm.mc;
//            ch_2 = pwm.mc;
        }


        if(alt!=0) {
            graph[k] = ch_1;
        }
        else {
            graph[k] = ch_2;
        }
        alt=!alt;

//        graph[k] = va;

        k++;
        if(k>=199) {
            k = 0;
        }

        sample = 0;
    }
    sample++;
}

void setup_DAC_PWM(void)
{
    EALLOW;
    //EPWM5A/5B
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;

    EDIS;



    // Initialization for EPwm5A and EPwm5B
    // = = = = = = = = = = = = = = = = = = = = = = = =
    EPwm5Regs.TBPRD = 0x07D0; //0xBB8=3000; Set for 20kHz BB8/2=5DC used to shift
    //EPwm5Regs.CMPA.half.CMPA = 600; // Compare A = 350 TBCLK counts INITIAL DUTY CYCLE
    //EPwm5Regs.CMPB = 1800; // Compare B = 200 TBCLK counts INITIAL DUTY CYCLE
//    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;
//    EPwm5Regs.TBPHS.half.TBPHSHR = 0x0000;
    EPwm5Regs.TBCTR = 0; // clear TB counter
    EPwm5Regs.TBCTL.bit.CTRMODE = 0x0; //TB_COUNT_UP 0x0
    EPwm5Regs.TBCTL.bit.PHSEN = 0x0;
    EPwm5Regs.TBCTL.bit.PRDLD = 1;//TB_SHADOW;
    EPwm5Regs.TBCTL.bit.SYNCOSEL = 0x3; //TB_SYNC_DISABLE 0x3
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0x0; // TB_DIV1 0x0 TBCLK = SYSCLK, SO TBPRD is not div by 2
    EPwm5Regs.TBCTL.bit.CLKDIV = 0x0;   // TB_DIV1 0x0
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = 1; //Earlier it was CC_SHADOW (0), Now it is in immediate mode;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = 1; //Earlier it was CC_SHADOW (0), Now it is in immediate mode;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR = Zero
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR = Zero
    EPwm5Regs.AQCTLA.bit.ZRO = 0x2;  //AQ_SET 0x2
    EPwm5Regs.AQCTLA.bit.CAU = 0x1;  //AQ_CLEAR 0x1

    EPwm5Regs.AQCTLB.bit.ZRO = 0x2;  //AQ_SET 0x2
    EPwm5Regs.AQCTLB.bit.CBU = 0x1;  //AQ_CLEAR 0x1
}

void hw_debug(void)
{
    if (select_varDAC==1) {
        EPwm5Regs.CMPA.bit.CMPA = EPwm5Regs.TBPRD - (1+vars1)/2*(EPwm5Regs.TBPRD);
        EPwm5Regs.CMPB.bit.CMPB = EPwm5Regs.TBPRD - (1+vars2)/2*(EPwm5Regs.TBPRD);
    }
    if (select_varDAC==2) {
        EPwm5Regs.CMPA.bit.CMPA = EPwm5Regs.TBPRD - (1+vars3)/2*(EPwm5Regs.TBPRD);
        EPwm5Regs.CMPB.bit.CMPB = EPwm5Regs.TBPRD - (1+vars4)/2*(EPwm5Regs.TBPRD);
    }
    if (select_varDAC==3) {
        EPwm5Regs.CMPA.bit.CMPA = EPwm5Regs.TBPRD - (1+va)/2*(EPwm5Regs.TBPRD);
        EPwm5Regs.CMPB.bit.CMPB = EPwm5Regs.TBPRD - (1+vb)/2*(EPwm5Regs.TBPRD);
    }

}



                //DEBUGGING
                            /*** Output the required Plot varible thro DAC for debugging ***/
                            //y= ((ia>>6)*(0x5DC))>>(GLOBAL_Q-7);  //Signal is shifted by Period/2 for AC waveform; For DC waveform shift may be remove to achieve full scale
                            //y= 0x5DC+y;
                            //s= ((ia>>6)*(0x5DC>>1))>>(GLOBAL_Q-7);    // =do= , 10kohm, 0.1microF filter works fine fc=1kHz for fs=20kHz PWM
                            //s= 0x5DC+s;
                            //y= ((ib>>6)*(0x5DC>>1))>>(GLOBAL_Q-7);  //Signal is shifted by Period/2 for AC waveform; For DC waveform shift may be remove to achieve full scale
                            //y= 0x5DC+y;
                //          ig = _IQ(0.5);
                //          s= ((ig>>6)*(0x5DC))>>(GLOBAL_Q-7);     // =do= , 10kohm, 0.1microF filter works fine fc=1kHz for fs=20kHz PWM
                            //s= 0x5DC+s;
                            //e= ((vbmcflt>>6)*(0x5DC>>1))>>(GLOBAL_Q-7);
                            //e= 0x5DC+e;



                            //EPwm5Regs.CMPA.half.CMPA= xxxx;       //red
                            //EPwm5Regs.CMPB= yyyy;     //blue
                            //EPwm3Regs.CMPA.half.CMPA= (Uint16)abs(y);     //red
                            //EPwm3Regs.CMPB= (Uint16)abs(e);       //blue

