/* ==============================================================================
System Name:  	HVACI_Sensorless

File Name:	  	HVACI_Sensorless.C

Description:	Primary system file for the Real Implementation of Sensorless  
          		Field Orientation Control for Induction Motors  

				Supports F2803x(fixed point) and F2833x(floating point) 
=================================================================================  */

// Include header files used in the main function

//#include "PeripheralHeaderIncludes.h"
#include "HVACI_Sensorless-Settings.h"
//#include "IQmathLib.h"
//#include "HVACI_Sensorless.h"
//#include <math.h>

// Global variables used in this system

Uint16 OffsetFlag=0;
float32_t offsetA=0;
float32_t offsetB=0;
float32_t offsetC=0;
float32_t K1=(0.998);      //Offset filter coefficient K1: 0.05/(T+0.05);
float32_t K2=(0.001999);   //Offset filter coefficient K2: T/(T+0.05);
extern float32_t IQsinTable[];
extern float32_t IQcosTable[];

float32_t VdTesting = (0.2);           // Vd reference (pu)
float32_t VqTesting = (0.2);           // Vq reference (pu)
float32_t IdRef = (0.1);               // Id reference (pu)
float32_t IqRef = (0.05);              // Iq reference (pu)
float32_t SpeedRef = (0.3);            // Speed reference (pu)

float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h

Uint32 IsrTicker = 0;
Uint16 BackTicker = 0;
Uint16 lsw=0;
Uint16 TripFlagDMC=0;               //PWM trip status

volatile Uint16 EnableFlag = FALSE;

Uint16 SpeedLoopPrescaler = 10;      // Speed loop prescaler
Uint16 SpeedLoopCount = 1;           // Speed loop counter

// Instance rotor flux and speed estimations
ACIFE fe1 = ACIFE_DEFAULTS;
ACISE se1 = ACISE_DEFAULTS;

// Instance the constant calculations for rotor flux and speed estimations
ACIFE_CONST fe1_const = ACIFE_CONST_DEFAULTS;
ACISE_CONST se1_const = ACISE_CONST_DEFAULTS;

// Instance a few transform objects (ICLARKE is added in SVGEN module)
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK park1 = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;

// Instance PI regulators to regulate the d and q  axis currents, and speed
PI_CONTROLLER pi_spd = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_id  = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER pi_iq  = PI_CONTROLLER_DEFAULTS;

// Instance a PWM driver instance
//PWMGEN pwm1 = PWMGEN_DEFAULTS;

// Instance a PWM DAC driver instance
//PWMDAC pwmdac1 = PWMDAC_DEFAULTS;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs
SVGEN svgen1 = SVGEN_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;

//  Instance a ramp(sawtooth) generator to simulate an Anglele
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

//  Instance a phase voltage calculation
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;

// Instance a speed calculator based on QEP
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

// Instance a speed calculator based on capture Qep (for eQep of 280x only)
SPEED_MEAS_CAP speed2 = SPEED_MEAS_CAP_DEFAULTS;

extern Uint16 OnOffMotor;
extern float speed_ref;

void HVDMC_Protection(void);


void init_motor(void)
{


/*

    // Initialize PWM module
        pwm1.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2;  // Prescaler X1 (T1), ISR period = T x 1
        pwm1.HalfPerMax=pwm1.PeriodMax/2;
        pwm1.Deadband  = 2.0*SYSTEM_FREQUENCY;          // 120 counts -> 2.0 usec for TBCLK = SYSCLK/1
        PWM_INIT_MACRO(1,2,3,pwm1)
*/


    // Initialize the Speed module for QEP based speed calculation
        speed1.K1 = (1/(BASE_FREQ*T));
        speed1.K2 = (1/(1+T*2*PI*5));  // Low-pass cut-off frequency
        speed1.K3 = (1)-speed1.K2;
        speed1.BaseRpm = 120*(BASE_FREQ/POLES);

    // Initialize the Speed module for capture eQEP based speed calculation (low speed range)
        speed2.InputSelect = 1;
        speed2.BaseRpm = 120*(BASE_FREQ/POLES);
        speed2.SpeedScaler = 50*(SYSTEM_FREQUENCY*1000000/(1*2048*speed2.BaseRpm));

    // Initialize the RAMPGEN module
        rg1.StepAngleMax = (BASE_FREQ*T);

    // Initialize the aci flux estimator constants module
        fe1_const.Rs = RS;
        fe1_const.Rr = RR;
        fe1_const.Ls = LS;
        fe1_const.Lr = LR;
        fe1_const.Lm = LM;
        fe1_const.Ib = BASE_CURRENT;
        fe1_const.Vb = BASE_VOLTAGE;
        fe1_const.Ts = T;
        ACIFE_CONST_MACRO(fe1_const)

    // Initialize the aci flux estimator module
        fe1.K1 = (fe1_const.K1);
        fe1.K2 = (fe1_const.K2);
        fe1.K3 = (fe1_const.K3);
        fe1.K4 = (fe1_const.K4);
        fe1.K5 = (fe1_const.K5);
        fe1.K6 = (fe1_const.K6);
        fe1.K7 = (fe1_const.K7);
        fe1.K8 = (fe1_const.K8);
        fe1.Kp = (2.8);
        fe1.Ki = (T/0.45);

    // Initialize the aci speed estimator constants module
        se1_const.Rr = RR;
        se1_const.Lr = LR;
        se1_const.fb = BASE_FREQ;
        se1_const.fc = 3;
        se1_const.Ts = T;
        ACISE_CONST_MACRO(se1_const)

    // Initialize the  aci speed estimator module
        se1.K1 = (se1_const.K1);
        se1.K2 = (se1_const.K2);
        se1.K3 = (se1_const.K3);
        se1.K4 = (se1_const.K4);
        se1.BaseRpm = 120*BASE_FREQ/POLES;

    // Initialize the PI module for Id
        pi_spd.Kp=(2.0);
        pi_spd.Ki=(T*SpeedLoopPrescaler/0.5);
        pi_spd.Umax =(0.95);
        pi_spd.Umin =(-0.95);

    // Initialize the PI module for Iq
        pi_id.Kp=(1.0);
        pi_id.Ki=(T/0.004);
        pi_id.Umax =(0.3);
        pi_id.Umin =(-0.3);

    // Initialize the PI module for speed
        pi_iq.Kp=(1.0);
        pi_iq.Ki=(T/0.004);
        pi_iq.Umax =(0.8);
        pi_iq.Umin =(-0.8);

    //  Note that the vectorial sum of d-q PI outputs should be less than 1.0 which refers to maximum duty cycle for SVGEN.
    //  Another duty cycle limiting factor is current sense through shunt resistors which depends on hardware/software implementation.
    //  Depending on the application requirements 3,2 or a single shunt resistor can be used for current waveform reconstruction.
    //  The higher number of shunt resistors allow the higher duty cycle operation and better dc bus utilization.
    //  The users should adjust the PI saturation levels carefully during open loop tests (i.e pi_id.Umax, pi_iq.Umax and Umins) as in project manuals.
    //  Violation of this procedure yields distorted current waveforms and unstable closed loop operations which may damage the inverter.

    //Call HVDMC Protection function
//        HVDMC_Protection();

        clearPWM1Trip();
}

void MotorOperation(void)
{

}



/*

//=================================================================================
//	A - TASKS (executed in every 1 msec)
//=================================================================================
//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{
	if(EPwm1Regs.TZFLG.bit.OST==0x1)
	TripFlagDMC=1;      // Trip on DMC (halt and IPM fault trip )
	
	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A2
	A_Task_Ptr = &A2;
	//-------------------
}


//----------------------------------------
void C1(void) 	// Toggle GPIO-34 
//----------------------------------------
{

	if(EPwm1Regs.TZFLG.bit.OST==0x1)			// TripZ for PWMs is low (fault trip)
	  { TripFlagDMC=1;      				   
	  }	
	 

	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;	   // Turn on/off LD3 on the controlCARD
	
	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C2
	C_Task_Ptr = &C2;	
	//-----------------

}

*/


float vars1=0.0;
float vars2=0.0;

//MainISR 
void MotorISR(void)
{

// Verifying the ISR
    IsrTicker++;

    readCurrVolADCSignals();
    filter_signals();

    if (!OnOffMotor) { //NOT GOOD
        return;
    }

// =============================== LEVEL 1 ======================================
//	  Checks target independent modules, duty cycle waveforms and PWM update
//	  Keep the motors disconnected at this level	
// ==============================================================================

#if (BUILDLEVEL==LEVEL1)	 

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;		
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1)  //THIS IS NOT WORKING

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
//	There are two option for trigonometric functions:
//  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
//  IQsin/cos PU functions interpolate the data in the lookup table yielding higher resolution. 
// ------------------------------------------------------------------------------
    ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;
    
	//ipark1.Sine  =_IQ30toIQ(IQsinTable[_IQtoIQ9(rg1.Out)]);
    //ipark1.Cosine=_IQ30toIQ(IQcosTable[_IQtoIQ9(rg1.Out)]);

	ipark1.Sine=sinf(rg1.Out);
    ipark1.Cosine=cosf(rg1.Out);
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta = ipark1.Beta;	
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------

// 	EPwm1Regs.CMPA.bit.CMPAHR = _IQmpy(m.HalfPerMax,m.MfuncC1)+ m.HalfPerMax;    \
//    (*ePWM[ch2]).CMPA.half.CMPA = _IQmpy(m.HalfPerMax,m.MfuncC2)+ m.HalfPerMax;    \
//    (*ePWM[ch3]).CMPA.half.CMPA = _IQmpy(m.HalfPerMax,m.MfuncC3)+ m.HalfPerMax;    \

    VIENNA_HAL_EPWM_setCounterCompareValueOptimized(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)( (float32_t)((float32_t)VIENNA_PFC3PH_PWM_PERIOD / 2.0)*(float32_t) fabsf(1.0-svgen1.Ta)));
    VIENNA_HAL_EPWM_setCounterCompareValueOptimized(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)( (float32_t)((float32_t)VIENNA_PFC3PH_PWM_PERIOD / 2.0)*(float32_t) fabsf(1.0-svgen1.Tb)));
    VIENNA_HAL_EPWM_setCounterCompareValueOptimized(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)( (float32_t)((float32_t)VIENNA_PFC3PH_PWM_PERIOD / 2.0)*(float32_t) fabsf(1.0-svgen1.Tc)));

    vars1 = svgen1.Ta - svgen1.Tc;
    vars2 = svgen1.Tb;

    /*
    pwm1.MfuncC1 = svgen1.Ta;  
    pwm1.MfuncC2 = svgen1.Tb;   
    pwm1.MfuncC3 = svgen1.Tc; 
	PWM_MACRO(1,2,3,pwm1)							// Calculate the new PWM compare values
*/


#endif // (BUILDLEVEL==LEVEL1)

// =============================== LEVEL 2 ======================================
//	  Level 2 verifies the analog-to-digital conversion, offset compensation, 
//    clarke/park transformations (CLARKE/PARK), phase voltage calculations 
// ==============================================================================

#if (BUILDLEVEL==LEVEL2) 

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;		
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1) 

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
/*
	#ifdef DSP2833x_DEVICE_H
	clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
	clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.
	#endif														   // ((ADCmeas(q12)/2^12)-offset)*2*(3.0/3.3)			
	
	#ifdef DSP2803x_DEVICE_H
	clarke1.As = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT1)-offsetA); // Phase A curr.
	clarke1.Bs = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT2)-offsetB); // Phase B curr.	
	#endif														   // (ADCmeas(q12->q24)-offset)*2	
*/
//    clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
//    clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.

	
	CLARKE_MACRO(clarke1)  

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta = clarke1.Beta;
	park1.Angle = rg1.Out;
	park1.Sine = sinf(park1.Angle);
	park1.Cosine = cosf(park1.Angle);
	PARK_MACRO(park1) 
 
// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;
	ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
	IPARK_MACRO(ipark1) 

// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
	#ifdef DSP2833x_DEVICE_H
	volt1.DcBusVolt = ((AdcMirror.ADCRESULT7)*0.00024414)*0.909; // DC Bus voltage meas.
    #endif														 // (ADCmeas(q12)/2^12)*(3.0V/3.3V)	
    
    #ifdef DSP2803x_DEVICE_H
	volt1.DcBusVolt = _IQ12toIQ(AdcResult.ADCRESULT7);	     	 // DC Bus voltage meas.
    #endif
    
    volt1.MfuncV1 = svgen1.Ta;
    volt1.MfuncV2 = svgen1.Tb;
    volt1.MfuncV3 = svgen1.Tc;
    PHASEVOLT_MACRO(volt1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta = ipark1.Beta;
  	SVGENDQ_MACRO(svgen1)	

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------

    VIENNA_HAL_EPWM_setCounterCompareValueOptimized(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)( (float32_t)((float32_t)VIENNA_PFC3PH_PWM_PERIOD / 2.0)*(float32_t) fabsf(1.0-svgen1.Ta)));
    VIENNA_HAL_EPWM_setCounterCompareValueOptimized(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)( (float32_t)((float32_t)VIENNA_PFC3PH_PWM_PERIOD / 2.0)*(float32_t) fabsf(1.0-svgen1.Tb)));
    VIENNA_HAL_EPWM_setCounterCompareValueOptimized(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)( (float32_t)((float32_t)VIENNA_PFC3PH_PWM_PERIOD / 2.0)*(float32_t) fabsf(1.0-svgen1.Tc)));

    vars1 = svgen1.Ta - svgen1.Tc;
    vars2 = svgen1.Tb;

 	/*
    pwm1.MfuncC1 = svgen1.Ta;  
    pwm1.MfuncC2 = svgen1.Tb;   
    pwm1.MfuncC3 = svgen1.Tc; 
	PWM_MACRO(1,2,3,pwm1)							// Calculate the new PWM compare values	
*/

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------	
/*
	pwmdac1.MfuncC1 = svgen1.Ta; 
    pwmdac1.MfuncC2 = svgen1.Tb; 
    PWMDAC_MACRO(6,pwmdac1)	  						// PWMDAC 6A, 6B
    
    pwmdac1.MfuncC1 = svgen1.Tc; 
    pwmdac1.MfuncC2 = svgen1.Tb-svgen1.Tc; 
	PWMDAC_MACRO(7,pwmdac1)		  					// PWMDAC 7A, 7B  
*/



#endif // (BUILDLEVEL==LEVEL2)

// =============================== LEVEL 3 ======================================
//	Level 3 verifies the dq-axis current regulation performed by PI and speed 
//	measurement modules  
// ============================================================================== 

#if (BUILDLEVEL==LEVEL3)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;		
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1) 

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	#ifdef DSP2833x_DEVICE_H
	clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
	clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.
	#endif							   						        // ((ADCmeas(q12)/2^12)-offset)*2*(3.0/3.3)			
	
	#ifdef DSP2803x_DEVICE_H
	clarke1.As = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT1)-offsetA); // Phase A curr.
	clarke1.Bs = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT2)-offsetB); // Phase B curr.	
	#endif														   // (ADCmeas(q12->q24)-offset)*2	
	
	CLARKE_MACRO(clarke1)  

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta  = clarke1.Beta;
	park1.Angle = rg1.Out;
	park1.Sine  = _IQsinPU(park1.Angle);
	park1.Cosine= _IQcosPU(park1.Angle);
	PARK_MACRO(park1) 
 
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI IQ controller macro
// ------------------------------------------------------------------------------  
    pi_iq.Ref = IqRef;
	pi_iq.Fbk = park1.Qs;
	PI_MACRO(pi_iq)

// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI ID controller macro
// ------------------------------------------------------------------------------  
    pi_id.Ref = IdRef;
	pi_id.Fbk = park1.Ds;
	PI_MACRO(pi_id)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = pi_id.Out;
    ipark1.Qs = pi_iq.Out ;
	ipark1.Sine   = park1.Sine;
    ipark1.Cosine = park1.Cosine;
	IPARK_MACRO(ipark1) 

// ------------------------------------------------------------------------------
//  Call the QEP macro (if incremental encoder used for speed sensing)
//  Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------ 
    QEP_MACRO(1,qep1)

    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Call the CAP macro (if sprocket or spur gear used for speed sensing)
//  Connect inputs of the SPEED_PR module and call the speed calculation macro
// ------------------------------------------------------------------------------ 
	CAP_MACRO(1,cap1) 

    if(cap1.CapReturn ==0)             				     // Check the capture return
    {
        speed2.EventPeriod=(int32)(cap1.EventPeriod);    // Read out new event period
        SPEED_PR_MACRO(speed2)                 			 // Call the speed macro      
    } 

// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
	#ifdef DSP2833x_DEVICE_H
	volt1.DcBusVolt = ((AdcMirror.ADCRESULT7)*0.00024414)*0.909; // DC Bus voltage meas.
    #endif														 // (ADCmeas(q12)/2^12)*(3.0V/3.3V)	
    
    #ifdef DSP2803x_DEVICE_H
	volt1.DcBusVolt = _IQ12toIQ(AdcResult.ADCRESULT7);	     // DC Bus voltage meas.
    #endif
    
    volt1.MfuncV1 = svgen1.Ta;
    volt1.MfuncV2 = svgen1.Tb;
    volt1.MfuncV3 = svgen1.Tc;
    PHASEVOLT_MACRO(volt1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta = ipark1.Beta;
  	SVGENDQ_MACRO(svgen1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;  
    pwm1.MfuncC2 = svgen1.Tb;   
    pwm1.MfuncC3 = svgen1.Tc; 
	PWM_MACRO(1,2,3,pwm1)							// Calculate the new PWM compare values	

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------	
	pwmdac1.MfuncC1 = svgen1.Ta; 
    pwmdac1.MfuncC2 = svgen1.Tb; 
    PWMDAC_MACRO(6,pwmdac1)	  						// PWMDAC 6A, 6B
    
    pwmdac1.MfuncC1 = svgen1.Tc; 
    pwmdac1.MfuncC2 = svgen1.Tb-svgen1.Tc; 
	PWMDAC_MACRO(7,pwmdac1)		  					// PWMDAC 7A, 7B  
// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(svgen1.Ta);
    DlogCh2 = _IQtoQ15(qep1.ElecTheta);
    DlogCh3 = _IQtoQ15(clarke1.As);
    DlogCh4 = _IQtoQ15(clarke1.Bs); 


#endif // (BUILDLEVEL==LEVEL3)

// =============================== LEVEL 4 ======================================
//	  Level 4 verifies the flux estimation (ACI_FE) and open-loop 
//	  speed estimation (ACI_SE).
// ==============================================================================  

#if (BUILDLEVEL==LEVEL4)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;		
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1) 

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	#ifdef DSP2833x_DEVICE_H
	clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
	clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.
	#endif												 // ((ADCmeas(q12)/2^12)-offset)*2*(3.0/3.3)			
	
	#ifdef DSP2803x_DEVICE_H
	clarke1.As = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT1)-offsetA); // Phase A curr.
	clarke1.Bs = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT2)-offsetB); // Phase B curr.	
	#endif														   // (ADCmeas(q12->q24)-offset)*2	
	
	CLARKE_MACRO(clarke1)  

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta  = clarke1.Beta;
	park1.Angle = rg1.Out;
	park1.Sine  = _IQsinPU(park1.Angle);
	park1.Cosine= _IQcosPU(park1.Angle);
	PARK_MACRO(park1) 
 
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI IQ controller macro
// ------------------------------------------------------------------------------  
    pi_iq.Ref = IqRef;
	pi_iq.Fbk = park1.Qs;
	PI_MACRO(pi_iq)

// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI ID controller macro
// ------------------------------------------------------------------------------  
    pi_id.Ref = IdRef;
	pi_id.Fbk = park1.Ds;
	PI_MACRO(pi_id)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = pi_id.Out;
    ipark1.Qs = pi_iq.Out ;
	ipark1.Sine   = park1.Sine;
    ipark1.Cosine = park1.Cosine;
	IPARK_MACRO(ipark1) 

// ------------------------------------------------------------------------------
//  Call the QEP macro (if incremental encoder used for speed sensing)
//  Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------ 
    QEP_MACRO(1,qep1)

    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Call the CAP macro (if sprocket or spur gear used for speed sensing)
//  Connect inputs of the SPEED_PR module and call the speed calculation macro
// ------------------------------------------------------------------------------ 
	CAP_MACRO(1,cap1) 

    if(cap1.CapReturn ==0)             				     // Check the capture return
    {
        speed2.EventPeriod=(int32)(cap1.EventPeriod);    // Read out new event period
        SPEED_PR_MACRO(speed2)                 			 // Call the speed macro      
    } 

// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
	#ifdef DSP2833x_DEVICE_H
	volt1.DcBusVolt = ((AdcMirror.ADCRESULT7)*0.00024414)*0.909; // DC Bus voltage meas.
    #endif														 // (ADCmeas(q12)/2^12)*(3.0V/3.3V)	
    
    #ifdef DSP2803x_DEVICE_H
	volt1.DcBusVolt = _IQ12toIQ(AdcResult.ADCRESULT7);	         // DC Bus voltage meas.
    #endif
    
    volt1.MfuncV1 = svgen1.Ta;
    volt1.MfuncV2 = svgen1.Tb;
    volt1.MfuncV3 = svgen1.Tc;
    PHASEVOLT_MACRO(volt1)        

// ------------------------------------------------------------------------------
//    Connect inputs of the ACI module and call the flux estimation macro
// ------------------------------------------------------------------------------
 	fe1.UDsS = volt1.Valpha;
	fe1.UQsS = volt1.Vbeta;
 	fe1.IDsS = clarke1.Alpha;
	fe1.IQsS = clarke1.Beta;
	ACIFE_MACRO(fe1)

// ------------------------------------------------------------------------------
//    Connect inputs of the ACI module and call the speed estimation macro
// ------------------------------------------------------------------------------
 	se1.IDsS = clarke1.Alpha;
	se1.IQsS = clarke1.Beta;
 	se1.PsiDrS = fe1.PsiDrS;
	se1.PsiQrS = fe1.PsiQrS;
	se1.ThetaFlux = fe1.ThetaFlux; 
	ACISE_MACRO(se1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta = ipark1.Beta;
  	SVGENDQ_MACRO(svgen1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;  
    pwm1.MfuncC2 = svgen1.Tb;   
    pwm1.MfuncC3 = svgen1.Tc; 
	PWM_MACRO(1,2,3,pwm1)							// Calculate the new PWM compare values	

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------	
	pwmdac1.MfuncC1 = fe1.PsiQrS; 
    pwmdac1.MfuncC2 = fe1.PsiDrS; 
    PWMDAC_MACRO(6,pwmdac1)	  						// PWMDAC 6A, 6B
    
    pwmdac1.MfuncC1 = rg1.Out; 
    pwmdac1.MfuncC2 = fe1.ThetaFlux; 
	PWMDAC_MACRO(7,pwmdac1)		  					// PWMDAC 7A, 7B  
	
// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(fe1.PsiQrS);
    DlogCh2 = _IQtoQ15(fe1.PsiDrS);
    DlogCh3 = _IQtoQ15(fe1.ThetaFlux);
    DlogCh4 = _IQtoQ15(clarke1.As);  

#endif // (BUILDLEVEL==LEVEL4)


// =============================== LEVEL 5 ======================================
//	  Level 5 verifies the speed regulator performed by PI module. 
//	  The system speed loop is closed by using the measured speed as a feedback.
// ==============================================================================  
//  lsw=0, close the current loop.
//	lsw=1, close the speed loop using measured speed.

#if (BUILDLEVEL==LEVEL5)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;		
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1) 

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	#ifdef DSP2833x_DEVICE_H
	clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
	clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.
	#endif													        // ((ADCmeas(q12)/2^12)-offset)*2*(3.0/3.3)			
	
	#ifdef DSP2803x_DEVICE_H
	clarke1.As = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT1)-offsetA); // Phase A curr.
	clarke1.Bs = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT2)-offsetB); // Phase B curr.	
	#endif														   // (ADCmeas(q12->q24)-offset)*2												  
	
	CLARKE_MACRO(clarke1)  

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta  = clarke1.Beta;
	if(lsw==0) park1.Angle = rg1.Out;
	else park1.Angle = fe1.ThetaFlux; 
	park1.Sine  = _IQsinPU(park1.Angle);
	park1.Cosine= _IQcosPU(park1.Angle);
	PARK_MACRO(park1) 

// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI SPD controller macro
// ------------------------------------------------------------------------------  
    if (SpeedLoopCount==SpeedLoopPrescaler)
     {
      pi_spd.Ref = rc1.SetpointValue;
      pi_spd.Fbk = speed1.Speed;
	  PI_MACRO(pi_spd);
      SpeedLoopCount=1;
     }
    else SpeedLoopCount++; 
	
	if(lsw==0)	{pi_spd.ui=0; pi_spd.i1=0;}
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI IQ controller macro
// ------------------------------------------------------------------------------  
    if(lsw==0) pi_iq.Ref = IqRef;
    else pi_iq.Ref =  pi_spd.Out;
	pi_iq.Fbk = park1.Qs;
	PI_MACRO(pi_iq) 
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI ID controller macro
// ------------------------------------------------------------------------------  
    pi_id.Ref = IdRef;
	pi_id.Fbk = park1.Ds;
	PI_MACRO(pi_id)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = pi_id.Out;
    ipark1.Qs = pi_iq.Out ;
	ipark1.Sine   = park1.Sine;
    ipark1.Cosine = park1.Cosine;
	IPARK_MACRO(ipark1) 

// ------------------------------------------------------------------------------
//  Call the QEP macro (if incremental encoder used for speed sensing)
//  Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------ 
    QEP_MACRO(1,qep1)

    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Call the CAP macro (if sprocket,tacho or gear used for speed sensing)
//  Connect inputs of the SPEED_PR module and call the speed calculation macro
// ------------------------------------------------------------------------------ 
	CAP_MACRO(1,cap1) 

    if(cap1.CapReturn ==0)             				     // Check the capture return
    {
        speed2.EventPeriod=(int32)(cap1.EventPeriod);    // Read out new event period
        SPEED_PR_MACRO(speed2)                 			 // Call the speed macro      
    } 

// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
	#ifdef DSP2833x_DEVICE_H
	volt1.DcBusVolt = ((AdcMirror.ADCRESULT7)*0.00024414)*0.909; // DC Bus voltage meas.
    #endif														 // (ADCmeas(q12)/2^12)*(3.0V/3.3V)	
    
    #ifdef DSP2803x_DEVICE_H
	volt1.DcBusVolt = _IQ12toIQ(AdcResult.ADCRESULT7);	         // DC Bus voltage meas.
    #endif
    
    volt1.MfuncV1 = svgen1.Ta;
    volt1.MfuncV2 = svgen1.Tb;
    volt1.MfuncV3 = svgen1.Tc;
    PHASEVOLT_MACRO(volt1)        

// ------------------------------------------------------------------------------
//    Connect inputs of the ACI module and call the flux estimation macro
// ------------------------------------------------------------------------------
 	fe1.UDsS = volt1.Valpha;
	fe1.UQsS = volt1.Vbeta;
 	fe1.IDsS = clarke1.Alpha;
	fe1.IQsS = clarke1.Beta;
	ACIFE_MACRO(fe1)

// ------------------------------------------------------------------------------
//    Connect inputs of the ACI module and call the speed estimation macro
// ------------------------------------------------------------------------------
 	se1.IDsS = clarke1.Alpha;
	se1.IQsS = clarke1.Beta;
 	se1.PsiDrS = fe1.PsiDrS;
	se1.PsiQrS = fe1.PsiQrS;
	se1.ThetaFlux = fe1.ThetaFlux; 
	ACISE_MACRO(se1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta = ipark1.Beta;
  	SVGENDQ_MACRO(svgen1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;  
    pwm1.MfuncC2 = svgen1.Tb;   
    pwm1.MfuncC3 = svgen1.Tc; 
	PWM_MACRO(1,2,3,pwm1)							// Calculate the new PWM compare values	

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------	
	pwmdac1.MfuncC1 = fe1.PsiQrS; 
    pwmdac1.MfuncC2 = fe1.PsiDrS; 
    PWMDAC_MACRO(6,pwmdac1)	  						// PWMDAC 6A, 6B
    
    pwmdac1.MfuncC1 = rg1.Out; 
    pwmdac1.MfuncC2 = fe1.ThetaFlux; 
	PWMDAC_MACRO(7,pwmdac1)		  					// PWMDAC 7A, 7B  

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(fe1.PsiQrS);
    DlogCh2 = _IQtoQ15(fe1.ThetaFlux);
    DlogCh3 = _IQtoQ15(rg1.Out);
    DlogCh4 = _IQtoQ15(clarke1.As);  

#endif // (BUILDLEVEL==LEVEL5) 

// =============================== LEVEL 6 ======================================
//	  Level 6 verifies the speed regulator performed by PI module. 
//	  The system speed loop is closed by using the estimated speed as a feedback.
// ============================================================================== 
//  lsw=0, close the current loop
//	lsw=1, close the speed loop

#if (BUILDLEVEL==LEVEL6)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;		
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1) 

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1). 
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	#ifdef DSP2833x_DEVICE_H
	clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-offsetA)*2*0.909; // Phase A curr.
	clarke1.Bs=((AdcMirror.ADCRESULT2)*0.00024414-offsetB)*2*0.909; // Phase B curr.
	#endif													        // ((ADCmeas(q12)/2^12)-offset)*2*(3.0/3.3)			
	
	#ifdef DSP2803x_DEVICE_H
	clarke1.As = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT1)-offsetA); // Phase A curr.
	clarke1.Bs = _IQmpy2(_IQ12toIQ(AdcResult.ADCRESULT2)-offsetB); // Phase B curr.	
	#endif														   // (ADCmeas(q12->q24)-offset)*2	
														  
	
	CLARKE_MACRO(clarke1)  

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------ 
    park1.Alpha = clarke1.Alpha;
    park1.Beta  = clarke1.Beta;
	if(lsw==0) park1.Angle = rg1.Out;
	else park1.Angle = fe1.ThetaFlux;
	park1.Sine = _IQsinPU(park1.Angle);
	park1.Cosine = _IQcosPU(park1.Angle);
	PARK_MACRO(park1)
 
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI SPD controller macro
// ------------------------------------------------------------------------------   
   if (SpeedLoopCount==SpeedLoopPrescaler)
     {
      pi_spd.Ref = rc1.SetpointValue;
      pi_spd.Fbk = se1.WrHat;
	  PI_MACRO(pi_spd)
      SpeedLoopCount=1;
     }
	else SpeedLoopCount++;  

	if(lsw==0)	{pi_spd.ui=0; pi_spd.i1=0;}
// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI ID controller macro
// ------------------------------------------------------------------------------ 
    if(lsw==0) pi_iq.Ref = IqRef;
    else pi_iq.Ref =  pi_spd.Out;
	pi_iq.Fbk = park1.Qs;
	PI_MACRO(pi_iq)

// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PI ID controller macro
// ------------------------------------------------------------------------------  
    pi_id.Ref = IdRef;
	pi_id.Fbk = park1.Ds;
	PI_MACRO(pi_id)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = pi_id.Out;
    ipark1.Qs = pi_iq.Out ;
	ipark1.Sine   = park1.Sine;
    ipark1.Cosine = park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Call the QEP macro (if incremental encoder used for speed sensing)
//  Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------ 
    QEP_MACRO(1,qep1)

    speed1.ElecTheta = qep1.ElecTheta;
    speed1.DirectionQep = (int32)(qep1.DirectionQep);
    SPEED_FR_MACRO(speed1) 

// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
	#ifdef DSP2833x_DEVICE_H
	volt1.DcBusVolt = ((AdcMirror.ADCRESULT7)*0.00024414)*0.909; // DC Bus voltage meas.
    #endif														 // (ADCmeas(q12)/2^12)*(3.0V/3.3V)	
    
    #ifdef DSP2803x_DEVICE_H
	volt1.DcBusVolt = _IQ12toIQ(AdcResult.ADCRESULT7);	     // DC Bus voltage meas.
    #endif
    
    volt1.MfuncV1 = svgen1.Ta;
    volt1.MfuncV2 = svgen1.Tb;
    volt1.MfuncV3 = svgen1.Tc;
    PHASEVOLT_MACRO(volt1)        

// ------------------------------------------------------------------------------
//    Connect inputs of the ACI module and call the flux estimation macro
// ------------------------------------------------------------------------------
 	fe1.UDsS = volt1.Valpha;
	fe1.UQsS = volt1.Vbeta;
 	fe1.IDsS = clarke1.Alpha;
	fe1.IQsS = clarke1.Beta;
	ACIFE_MACRO(fe1)

// ------------------------------------------------------------------------------
//    Connect inputs of the ACI module and call the speed estimation macro
// ------------------------------------------------------------------------------
 	se1.IDsS = clarke1.Alpha;
	se1.IQsS = clarke1.Beta;
 	se1.PsiDrS = fe1.PsiDrS;
	se1.PsiQrS = fe1.PsiQrS;
	se1.ThetaFlux = fe1.ThetaFlux; 
	ACISE_MACRO(se1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta = ipark1.Beta;
  	SVGENDQ_MACRO(svgen1)        

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = svgen1.Ta;  
    pwm1.MfuncC2 = svgen1.Tb;   
    pwm1.MfuncC3 = svgen1.Tc; 
	PWM_MACRO(1,2,3,pwm1)							// Calculate the new PWM compare values	

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------	
	pwmdac1.MfuncC1 = speed1.Speed; 
    pwmdac1.MfuncC2 = se1.WrHat; 
    PWMDAC_MACRO(6,pwmdac1)	  						// PWMDAC 6A, 6B
    
    pwmdac1.MfuncC1 = rg1.Out; 
    pwmdac1.MfuncC2 = fe1.ThetaFlux; 
	PWMDAC_MACRO(7,pwmdac1)		  					// PWMDAC 7A, 7B  

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(clarke1.Bs);
    DlogCh2 = _IQtoQ15(clarke1.As);
    DlogCh3 = _IQtoQ15(fe1.ThetaFlux);
    DlogCh4 = _IQtoQ15(volt1.VphaseA);

#endif // (BUILDLEVEL==LEVEL6) 




// Enable more interrupts from this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to recieve more interrupts from PIE group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;


}// MainISR Ends Here



/**********************************************************/
/********************Offset Compensation*******************/  
/**********************************************************/

/*
interrupt void OffsetISR(void)
{
// Verifying the ISR
    IsrTicker++;
    
// DC offset measurement for ADC 

    if (IsrTicker>=5000)
    	{	
    		
    		#ifdef DSP2833x_DEVICE_H
    		offsetA= K1*offsetA + K2*(AdcMirror.ADCRESULT1)*0.00024414; 			//Phase A offset
    		offsetB= K1*offsetB + K2*(AdcMirror.ADCRESULT2)*0.00024414; 			//Phase B offset
    		offsetC= K1*offsetC + K2*(AdcMirror.ADCRESULT3)*0.00024414; ;			//Phase C offset
    		#endif
    		
    		#ifdef DSP2803x_DEVICE_H
    		offsetA= _IQmpy(K1,offsetA)+_IQmpy(K2,_IQ12toIQ(AdcResult.ADCRESULT1)); 		//Phase A offset
    		offsetB= _IQmpy(K1,offsetB)+_IQmpy(K2,_IQ12toIQ(AdcResult.ADCRESULT2));			//Phase B offset
    		offsetC= _IQmpy(K1,offsetC)+_IQmpy(K2,_IQ12toIQ(AdcResult.ADCRESULT3));			//Phase C offset
    		#endif
    	}

	if (IsrTicker > 20000)
	{
		EALLOW;
		PieVectTable.EPWM1_INT = &MainISR;		
		EDIS;
	}
    

// Enable more interrupts from this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to recieve more interrupts from PIE group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}
*/




void HVDMC_Protection(void)
{

      EALLOW;
      
// Configure Trip Mechanism for the Motor control software
// -Cycle by cycle trip on CPU halt
// -One shot IPM trip zone trip 
// These trips need to be repeated for EPWM1 ,2 & 3

//===========================================================================
//Motor Control Trip Config, EPwm1,2,3
//===========================================================================
    
// CPU Halt Trip  
      EPwm1Regs.TZSEL.bit.CBC6=0x1;
      EPwm2Regs.TZSEL.bit.CBC6=0x1;
      EPwm3Regs.TZSEL.bit.CBC6=0x1;

      EPwm1Regs.TZSEL.bit.OSHT1   = 1;  //enable TZ1 for OSHT
      EPwm2Regs.TZSEL.bit.OSHT1   = 1;  //enable TZ1 for OSHT     
      EPwm3Regs.TZSEL.bit.OSHT1   = 1;  //enable TZ1 for OSHT

// What do we want the OST/CBC events to do?
// TZA events can force EPWMxA
// TZB events can force EPWMxB

      EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low 
      EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
      
      EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low 
      EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
      
      EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low 
      EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
      
      
      EDIS;

     // Clear any spurious OV trip
      EPwm1Regs.TZCLR.bit.OST = 1;
      EPwm2Regs.TZCLR.bit.OST = 1;
      EPwm3Regs.TZCLR.bit.OST = 1;

}

