
#include "hmi.h"

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/
#define LEVEL1  1      		// Module check out, duty cycle waveforms and PWM update  
#define LEVEL2  2           // Verify ADC, park/clarke, calibrate the offset 
#define LEVEL3  3           // Two current PI regulator test, speed measurement 
#define LEVEL4  4           // Flux and speed estimator tests 
#define LEVEL5  5           // Speed PI regulator test (Sensored closed-loop FOC system) 
#define LEVEL6  6           // Sensorless closed-loop FOC system

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   BUILDLEVEL LEVEL4

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

//#define PI 3.14159265358979

// Define the system frequency (MHz)
#define SYSTEM_FREQUENCY 100

//Define system Math Type
// Select Floating Math Type for 2833x
// Select IQ Math Type for 2803x 
//#define MATH_TYPE 0

// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10

// Define the electrical motor parametes (1/4 hp Marathon Motor)
#define RS 		2.76		        // Stator resistance (ohm)
#define RR   	3.7091		        // Rotor resistance (ohm)
#define LS   	0.3216    	  	// Stator inductance (H)
#define LR   	0.3216	  		// Rotor inductance (H)
#define LM   	0.3086	   		// Magnatizing inductance (H)
#define POLES  	4					// Number of poles

// Define the base quantites for PU system conversion
#define BASE_VOLTAGE    230     // Base peak phase voltage (volt)
#define BASE_CURRENT    14.1          // Base peak phase current (amp)
#define BASE_TORQUE         		// Base torque (N.m)
#define BASE_FLUX       		    // Base flux linkage (volt.sec/rad)
#define BASE_FREQ      	50         // Base electrical frequency (Hz)
									// Note that 0.5 pu (1800 rpm) is max for Marathon motor 
									// Above 1800 rpm, field weakening is needed.


#ifndef __ACI_FE_H__
#define __ACI_FE_H__

typedef struct {  float32_t  ThetaFlux;       // Output: Rotor flux angle
                  float32_t  IQsS;            // Input: Stationary q-axis stator current
                  float32_t  IDsS;            // Input: Stationary d-axis stator current
                  float32_t  IDsE;            // Variable: Measured current in sync. reference frame
                  float32_t  K1;              // Parameter: Constant using in current model
                  float32_t  FluxDrE;         // Variable: Rotating d-axis rotor flux (current model)
                  float32_t  K2;              // Parameter: Constant using in current model
                  float32_t  FluxQrS;         // Variable: Stationary q-axis rotor flux (current model)
                  float32_t  FluxDrS;         // Variable: Stationary d-axis rotor flux (current model)
                  float32_t  K3;              // Parameter: Constant using in stator flux computation
                  float32_t  K4;              // Parameter: Constant using in stator flux computation
                  float32_t  FluxDsS;         // Variable: Stationary d-axis stator flux (current model)
                  float32_t  FluxQsS;         // Variable: Stationary q-axis stator flux (current model)
                  float32_t  PsiDsS;          // Variable: Stationary d-axis stator flux (voltage model)
                  float32_t  Kp;              // Parameter: PI proportionnal gain
                  float32_t  Error;           // Parameter: Error term
                  float32_t  UiDsS;           // Variable: Stationary d-axis integral term
                  float32_t  UCompDsS;        // Variable: Stationary d-axis compensated voltage
                  float32_t  Ki;              // Parameter: PI integral gain
                  float32_t  PsiQsS;          // Variable: Stationary q-axis stator flux (voltage model)
                  float32_t  UiQsS;           // Variable: Stationary q-axis integral term
                  float32_t  UCompQsS;        // Variable: Stationary q-axis compensated voltage
                  float32_t  EmfDsS;          // Variable: Stationary d-axis back emf
                  float32_t  UDsS;            // Input: Stationary d-axis stator voltage
                  float32_t  K5;              // Parameter: Constant using in back emf computation
                  float32_t  K6;              // Parameter: Constant using in back emf computation
                  float32_t  EmfQsS;          // Variable: Stationary q-axis back emf
                  float32_t  UQsS;            // Input: Stationary q-axis stator voltage
                  float32_t  K8;              // Parameter: Constant using in rotor flux computation
                  float32_t  K7;              // Parameter: Constant using in rotor flux computation
                  float32_t  PsiDrS;          // Output: Stationary d-axis estimated rotor flux
                  float32_t  PsiQrS;          // Output: Stationary q-axis estimated rotor flux
                  float32_t  OldEmf;          // Variable: Old back-emf term
                  float32_t  Sine;            // Variable: Sine term
                  float32_t  Cosine;          // Variable: Cosine term
                  float32_t  tmp;             // temporary variable
                 } ACIFE;

/*-----------------------------------------------------------------------------
    Default initalizer for the ACIFE object.
-----------------------------------------------------------------------------*/
#define ACIFE_DEFAULTS {  0,    /*  ThetaFlux  */   \
                          0,    /*  IDsS  */        \
                          0,    /*  IQsS  */        \
                          0,    /*  IQsE  */        \
                          0,    /*  K1 */           \
                          0,    /*  FluxDrE  */     \
                          0,    /*  K2  */          \
                          0,    /*  FluxDrS  */     \
                          0,    /*  FluxQrS  */     \
                          0,    /*  K3  */          \
                          0,    /*  K4  */          \
                          0,    /*  FluxDsS  */     \
                          0,    /*  FluxQsS  */     \
                          0,    /*  PsiDsS  */      \
                          0,    /*  Kp  */          \
                          0,    /*  Error  */       \
                          0,    /*  UiDsS  */       \
                          0,    /*  UCompDsS  */    \
                          0,    /*  Ki  */          \
                          0,    /*  PsiQsS  */      \
                          0,    /*  UiQsS  */       \
                          0,    /*  UCompQsS  */    \
                          0,    /*  EmfDsS  */      \
                          0,    /*  UDsS  */        \
                          0,    /*  K5  */          \
                          0,    /*  K6  */          \
                          0,    /*  EmfQsS  */      \
                          0,    /*  UQsS  */        \
                          0,    /*  K8  */          \
                          0,    /*  K7  */          \
                          0,    /*  PsiDrS  */      \
                          0,    /*  PsiQrS  */      \
                          0,    /*  OldEmf  */      \
                          0,    /*  Sine  */        \
                          0,    /*  Cosine  */      \
                          0,    /*  tmp  */         \
                        }

/*------------------------------------------------------------------------------
    ACI Flux Estimator MACRO Definition
------------------------------------------------------------------------------*/


#define ACIFE_MACRO(v)                                                          \
                                                                                \
/* Calculate Sine and Cosine terms for Park/IPark transformations   */          \
    v.Sine   = sinf(v.ThetaFlux);                                           \
    v.Cosine = cosf(v.ThetaFlux);                                           \
                                                                                \
/* Park transformation on the measured stator current*/                         \
    v.IDsE = (v.IQsS*v.Sine);                                             \
    v.IDsE += (v.IDsS*v.Cosine);                                          \
                                                                                \
/* The current model section (Classical Rotor Flux Vector Control Equation)*/   \
    v.FluxDrE = (v.K1*v.FluxDrE) + (v.K2*v.IDsE);                   \
                                                                                \
/* Inverse park transformation on the rotor flux from the current model*/       \
    v.FluxDrS = (v.FluxDrE*v.Cosine);                                     \
    v.FluxQrS = (v.FluxDrE*v.Sine);                                       \
                                                                                \
/* Compute the stator flux based on the rotor flux from current model*/         \
    v.FluxDsS = (v.K3*v.FluxDrS) + (v.K4*v.IDsS);                   \
    v.FluxQsS = (v.K3*v.FluxQrS) + (v.K4*v.IQsS);                   \
                                                                                \
/* Conventional PI controller section */                                        \
    v.Error =  v.PsiDsS - v.FluxDsS;                                            \
    v.UCompDsS = (v.Kp*v.Error) + v.UiDsS;                                \
    v.UiDsS = (v.Kp*(v.Ki*v.Error)) + v.UiDsS;                      \
                                                                                \
    v.Error =  v.PsiQsS - v.FluxQsS;                                            \
    v.UCompQsS = (v.Kp*v.Error) + v.UiQsS;                                \
    v.UiQsS = (v.Kp*(v.Ki*v.Error)) + v.UiQsS;                      \
                                                                                \
/* Compute the estimated stator flux based on the integral of back emf*/        \
    v.OldEmf = v.EmfDsS;                                                        \
    v.EmfDsS = v.UDsS - v.UCompDsS - (v.K5*v.IDsS);                       \
    v.PsiDsS = v.PsiDsS + 0.5*((v.K6*(v.EmfDsS + v.OldEmf)));          \
                                                                                \
    v.OldEmf = v.EmfQsS;                                                        \
    v.EmfQsS = v.UQsS - v.UCompQsS - (v.K5*v.IQsS);                       \
    v.PsiQsS = v.PsiQsS + 0.5*((v.K6*(v.EmfQsS + v.OldEmf)));          \
                                                                                \
/* Estimate the rotor flux based on stator flux from the integral of back emf*/ \
                                                                                \
    v.PsiDrS = (v.K7*v.PsiDsS) - (v.K8*v.IDsS);                     \
    v.PsiQrS = (v.K7*v.PsiQsS) - (v.K8*v.IQsS);                     \
                                                                                \
/* Compute the rotor flux angle*/                                               \
    v.tmp = ( atan2f(v.PsiQrS,v.PsiDrS) ) ;                                     \
    if (v.tmp >=0.0)                                                            \
        v.ThetaFlux = v.tmp;                                                    \
    else                                                                        \
        v.ThetaFlux = v.tmp + 6.283185307;
    //((atan2f(v.PsiQrS,v.PsiDrS)*(1.0/6.283185307)) >= 0.0 ? (atan2f(v.PsiQrS,v.PsiDrS)*(1.0/6.283185307)):1.0 + (atan2f(v.PsiQrS,v.PsiDrS)*(1.0/6.283185307)));

#endif // __ACI_FE_H__



/* =================================================================================
File name:       ACI_FE_CONST.H
====================================================================================*/
#ifndef __ACI_FE_CONST_H__
#define __ACI_FE_CONST_H__

typedef struct  { float32_t  Rs;              // Input: Stator resistance (ohm)
                  float32_t  Rr;              // Input: Rotor resistance (ohm)
                  float32_t  Ls;              // Input: Stator inductance (H)
                  float32_t  Lr;              // Input: Rotor inductance (H)
                  float32_t  Lm;              // Input: Magnetizing inductance (H)
                  float32_t  Ib;              // Input: Base phase current (amp)
                  float32_t  Vb;              // Input: Base phase voltage (volt)
                  float32_t  Ts;              // Input: Sampling period in sec
                  float32_t  Tr;              // Parameter: Rotor time constant
                  float32_t  K1;              // Output: constant using in rotor flux calculation
                  float32_t  K2;              // Output: constant using in rotor flux calculation
                  float32_t  K3;              // Output: constant using in rotor flux calculation
                  float32_t  K4;              // Output: constant using in stator current calculation
                  float32_t  K5;              // Output: constant using in stator current calculation
                  float32_t  K6;              // Output: constant using in stator current calculation
                  float32_t  K7;              // Output: constant using in stator current calculation
                  float32_t  K8;              // Output: constant using in torque calculation
                } ACIFE_CONST;

/*-----------------------------------------------------------------------------
    Default initalizer for the ACIFE_CONST object.
-----------------------------------------------------------------------------*/
#define ACIFE_CONST_DEFAULTS {0,0,0,0,      \
                              0,0,0,0,      \
                              0,0,0,0,      \
                              0,0,0,0,      \
                             }

/*------------------------------------------------------------------------------
    ACIFE_CONST MACRO Definition
------------------------------------------------------------------------------*/


#define ACIFE_CONST_MACRO(v)                    \
                                                \
/* Rotor time constant (sec)*/                  \
   v.Tr = v.Lr/v.Rr;                            \
                                                \
   v.K1 = v.Tr/(v.Tr+v.Ts);                     \
   v.K2 = v.Ts/(v.Tr+v.Ts);                     \
   v.K3 = v.Lm/v.Lr;                            \
   v.K4 = (v.Ls*v.Lr-v.Lm*v.Lm)/(v.Lr*v.Lm);    \
   v.K5 = v.Ib*v.Rs/v.Vb;                       \
   v.K6 = v.Vb*v.Ts/(v.Lm*v.Ib);                \
   v.K7 = v.Lr/v.Lm;                            \
   v.K8 = (v.Ls*v.Lr-v.Lm*v.Lm)/(v.Lm*v.Lm);

#endif


/* =================================================================================
File name:       ACI_SE.H
===================================================================================*/


#ifndef __ACI_SE_H__
#define __ACI_SE_H__

typedef struct {  float32_t   IQsS;           // Input: Stationary q-axis stator current
                  float32_t   PsiDrS;         // Input: Stationary d-axis rotor flux
                  float32_t   IDsS;           // Input: Stationary d-axis stator current
                  float32_t   PsiQrS;         // Input: Stationary q-axis rotor flux
                  float32_t   K1;             // Parameter: Constant using in speed computation
                  float32_t   SquaredPsi;     // Variable: Squared rotor flux
                  float32_t   ThetaFlux;      // Input: Rotor flux angle
                  float32_t   K2;             // Parameter: Constant using in differentiator (Q21) - independently with global Q
                  float32_t   OldThetaFlux;   // Variable: Previous rotor flux angle
                  float32_t   K3;             // Parameter: Constant using in low-pass filter
                  float32_t   WPsi;           // Variable: Synchronous rotor flux speed (Q21) - independently with global Q
                  float32_t   K4;             // Parameter: Constant using in low-pass filter
                  float32_t   WrHat;          // Output: Estimated speed in per unit
                  Uint32      BaseRpm;        // Parameter: Base rpm speed (Q0) - independently with global Q
                  int32       WrHatRpm;       // Output: Estimated speed in rpm (Q0) - independently with global Q
                  float32_t   WSlip;          // Variable: Slip
                  float32_t   WSyn;           // Variable: Synchronous speed
                 } ACISE;


/*-----------------------------------------------------------------------------
Default initalizer for the ACISE object.
----------------------------------------------------------------------------- */
#define ACISE_DEFAULTS {  0,            \
                          0,            \
                          0,            \
                          0,            \
                          (0.1),     \
                          0,            \
                          0,            \
                          (0.1),   \
                          0,            \
                          (0.1),     \
                          0,            \
                          (0.1),     \
                          0,            \
                          3600,         \
                          0,            \
                          0,            \
                          0,            \
                        }

#define DIFF_MAX_LIMIT      float32_t(0.80)
#define DIFF_MIN_LIMIT      float32_t(0.20)

/*------------------------------------------------------------------------------
    ACI Speed Estimator MACRO Definition
------------------------------------------------------------------------------ */


#define ACISE_MACRO(v)                                                          \
                                                                                \
/*  Slip computation */                                                         \
    v.SquaredPsi = (v.PsiDrS*v.PsiDrS)+(v.PsiQrS*v.PsiQrS);         \
                                                                                \
    v.WSlip= (v.K1*((v.PsiDrS*v.IQsS) - (v.PsiQrS*v.IDsS)));  \
    v.WSlip= (v.WSlip/v.SquaredPsi);                                      \
                                                                                \
/*  Synchronous speed computation   */                                          \
    if ((v.ThetaFlux < 0.8)&(v.ThetaFlux > 0.2))          \
/*  Q21 = Q21*(GLOBAL_Q-GLOBAL_Q)*/                                             \
          v.WSyn = (v.K2*(v.ThetaFlux - v.OldThetaFlux));                 \
    else  v.WSyn = v.WPsi;                                                      \
                                                                                \
/* low-pass filter, Q21 = GLOBAL_Q*Q21 + GLOBAL_Q*Q21   */                      \
    v.WPsi = (v.K3*v.WPsi) + (v.K4*v.WSyn);                         \
                                                                                \
/* Q21 = Q21 - GLOBAL_Q */                                                      \
    v.OldThetaFlux = v.ThetaFlux;                                               \
    v.WrHat = v.WPsi - (v.WSlip);                                      \
                                                                                \
/* Limit the estimated speed between -1 and 1 per-unit */                       \
    if (v.WrHat>1.0)    \
        v.WrHat=1.0;    \
    if (v.WrHat< -1.0)  \
        v.WrHat = -1.0; \
                                                                          \
/* Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q */                               \
    v.WrHatRpm = (v.BaseRpm*v.WrHat);

#endif // __ACI_SE_H__


/* =================================================================================
File name:       ACI_SE_CONST.H
===================================================================================*/
#ifndef __ACI_SE_CONST_H__
#define __ACI_SE_CONST_H__

typedef struct  { float32  Rr;              // Input: Rotor resistance (ohm)
                  float32  Lr;              // Input: Rotor inductance (H)
                  float32  Tr;              // Variable: Rotor time constant
                  float32  fb;              // Input: Base electrical frequency (Hz)
                  float32  Wb;              // Variable: Base angular speed (rad/s)
                  float32  fc;              // Input: Cut-off frequency of lowpass filter (Hz)
                  float32  Tc;              // Variable: Time constant (sec)
                  float32  Ts;              // Input: Sampling period in sec
                  float32  K1;              // Output: constant using in rotor flux calculation
                  float32  K2;              // Output: constant using in rotor flux calculation
                  float32  K3;              // Output: constant using in rotor flux calculation
                  float32  K4;              // Output: constant using in stator current calculation
                } ACISE_CONST;

/*-----------------------------------------------------------------------------
    Default initalizer for the ACISE_CONST object.
-----------------------------------------------------------------------------*/
#define ACISE_CONST_DEFAULTS {0,0,0,0,  \
                              0,0,0,0,  \
                              0,0,0,0,  \
                             }

/*------------------------------------------------------------------------------
    ACI_SE_CONST macro definition
------------------------------------------------------------------------------*/


//#define PI 3.14159265358979

#define ACISE_CONST_MACRO(v)                \
                                            \
/* Rotor time constant (sec) */             \
    v.Tr = v.Lr/v.Rr;                       \
                                            \
/* Lowpass filter time constant (sec) */    \
    v.Tc = 1/(2*PI*v.fc);                   \
                                            \
    v.Wb = 2*PI*v.fb;                       \
    v.K1 = 1/(v.Wb*v.Tr);                   \
    v.K2 = 1/(v.fb*v.Ts);                   \
    v.K3 = v.Tc/(v.Tc+v.Ts);                \
    v.K4 = v.Ts/(v.Tc+v.Ts);


#endif







/* =================================================================================
File name:       PARK.H
===================================================================================*/

#ifndef __PARK_H__
#define __PARK_H__



typedef struct {  float32_t  Alpha;       // Input: stationary d-axis stator variable
                  float32_t  Beta;        // Input: stationary q-axis stator variable
                  float32_t  Angle;       // Input: rotating angle (pu)
                  float32_t  Ds;          // Output: rotating d-axis stator variable
                  float32_t  Qs;          // Output: rotating q-axis stator variable
                  float32_t  Sine;
                  float32_t  Cosine;
                } PARK;

/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/
#define PARK_DEFAULTS {   0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          }

/*------------------------------------------------------------------------------
    PARK Transformation Macro Definition
------------------------------------------------------------------------------*/

#define PARK_MACRO(v)                                           \
                                                                \
    v.Ds = v.Alpha*v.Cosine + v.Beta*v.Sine;    \
    v.Qs = v.Beta*v.Cosine - v.Alpha*v.Sine;

#endif // __PARK_H__


/* =================================================================================
File name:       IPARK.H
===================================================================================*/

#ifndef __IPARK_H__
#define __IPARK_H__

typedef struct {  float32_t  Alpha;       // Output: stationary d-axis stator variable
                  float32_t  Beta;        // Output: stationary q-axis stator variable
                  float32_t  Angle;       // Input: rotating angle (pu)
                  float32_t  Ds;          // Input: rotating d-axis stator variable
                  float32_t  Qs;          // Input: rotating q-axis stator variable
                  float32_t  Sine;        // Input: Sine term
                  float32_t  Cosine;      // Input: Cosine term
                } IPARK;

/*-----------------------------------------------------------------------------
Default initalizer for the IPARK object.
-----------------------------------------------------------------------------*/
#define IPARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                       }

/*------------------------------------------------------------------------------
    Inverse PARK Transformation Macro Definition
------------------------------------------------------------------------------*/


#define IPARK_MACRO(v)                                      \
                                                            \
v.Alpha = v.Ds*v.Cosine - v.Qs*v.Sine;      \
v.Beta  = v.Qs*v.Cosine + v.Ds*v.Sine;

#endif // __IPARK_H__


/* =================================================================================
File name:       PI.H
===================================================================================*/


#ifndef __PI_H__
#define __PI_H__

typedef struct {  float32_t  Ref;             // Input: reference set-point
                  float32_t  Fbk;             // Input: feedback
                  float32_t  Out;             // Output: controller output
                  float32_t  Kp;              // Parameter: proportional loop gain
                  float32_t  Ki;              // Parameter: integral gain
                  float32_t  Umax;            // Parameter: upper saturation limit
                  float32_t  Umin;            // Parameter: lower saturation limit
                  float32_t  up;              // Data: proportional term
                  float32_t  ui;              // Data: integral term
                  float32_t  v1;              // Data: pre-saturated controller output
                  float32_t  i1;              // Data: integrator storage: ui(k-1)
                  float32_t  w1;              // Data: saturation record: [u(k-1) - v(k-1)]
                } PI_CONTROLLER;


/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects
-----------------------------------------------------------------------------*/

#define PI_CONTROLLER_DEFAULTS {        \
                           0,           \
                           0,           \
                           0,           \
                           (1.0),    \
                           (0.0),    \
                           (1.0),    \
                           (-1.0),   \
                           (0.0),    \
                           (0.0),    \
                           (0.0),    \
                           (0.0),    \
                           (1.0)     \
                          }


/*------------------------------------------------------------------------------
    PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define PI_MACRO(v)                                             \
                                                                \
    /* proportional term */                                     \
    v.up = v.Ref - v.Fbk;                                       \
                                                                \
    /* integral term */                                         \
    v.ui = (v.Out == v.v1)?((v.Ki*v.up)+ v.i1) : v.i1;   \
    v.i1 = v.ui;                                                \
                                                                \
    /* control output */                                        \
    v.v1 = (v.Kp* (v.up + v.ui));                         \
    v.Out= (v.v1>v.Umax) ? v.Umax : ( (v.v1<v.Umin)? v.Umin : v.v1 ) ;                        \
    //v.Out= _IQsat(v.v1, v.Umax, v.Umin);                        \
    //v.w1 = (v.Out == v.v1) ? _IQ(1.0) : _IQ(0.0);             \

#endif // __PI_H__


/* =================================================================================
File name:       CLARKE.H
===================================================================================*/


#ifndef __CLARKE_H__
#define __CLARKE_H__

typedef struct {  float32_t  As;          // Input: phase-a stator variable
                  float32_t  Bs;          // Input: phase-b stator variable
                  float32_t  Cs;          // Input: phase-c stator variable
                  float32_t  Alpha;       // Output: stationary d-axis stator variable
                  float32_t  Beta;        // Output: stationary q-axis stator variable
                } CLARKE;

/*-----------------------------------------------------------------------------
    Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/
#define CLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          0, \
                        }

/*------------------------------------------------------------------------------
    CLARKE Transformation Macro Definition
------------------------------------------------------------------------------*/


#define CLARKE_MACRO(v)                                         \
                                                                \
v.Alpha = v.As;                                                 \
v.Beta = ((v.As +2*v.Bs)*(0.57735026918963));   \

//  1/sqrt(3) = 0.57735026918963

#endif // __CLARKE_H__


/* =================================================================================
File name:       SVGEN.H
===================================================================================*/


#ifndef __SVGEN_H__
#define __SVGEN_H__

typedef struct  { float32_t  Ualpha;          // Input: reference alpha-axis phase voltage
                  float32_t  Ubeta;           // Input: reference beta-axis phase voltage
                  float32_t  Ta;              // Output: reference phase-a switching function
                  float32_t  Tb;              // Output: reference phase-b switching function
                  float32_t  Tc;              // Output: reference phase-c switching function
                  float32_t  tmp1;            // Variable: temp variable
                  float32_t  tmp2;            // Variable: temp variable
                  float32_t  tmp3;            // Variable: temp variable
                  Uint16 VecSector;     // Space vector sector
                } SVGEN;


/*-----------------------------------------------------------------------------
Default initalizer for the SVGEN object.
-----------------------------------------------------------------------------*/
#define SVGEN_DEFAULTS { 0,0,0,0,0 }

/*------------------------------------------------------------------------------
    Space Vector  Generator (SVGEN) Macro Definition
------------------------------------------------------------------------------*/


#define SVGENDQ_MACRO(v)                                                        \
    v.tmp1= v.Ubeta;                                                            \
    v.tmp2= (0.5*v.Ubeta) + (0.866*v.Ualpha);                   \
    v.tmp3= v.tmp2 - v.tmp1;                                                    \
                                                                                \
    v.VecSector=3;                                                              \
    v.VecSector=(v.tmp2> 0)?( v.VecSector-1):v.VecSector;                       \
    v.VecSector=(v.tmp3> 0)?( v.VecSector-1):v.VecSector;                       \
    v.VecSector=(v.tmp1< 0)?(7-v.VecSector) :v.VecSector;                       \
                                                                                \
    if     (v.VecSector==1 || v.VecSector==4)                                   \
      {     v.Ta= v.tmp2;                                                       \
            v.Tb= v.tmp1-v.tmp3;                                                \
            v.Tc=-v.tmp2;                                                       \
      }                                                                         \
                                                                                \
    else if(v.VecSector==2 || v.VecSector==5)                                   \
      {     v.Ta= v.tmp3+v.tmp2;                                                \
            v.Tb= v.tmp1;                                                       \
            v.Tc=-v.tmp1;                                                       \
      }                                                                         \
                                                                                \
    else                                                                        \
      {     v.Ta= v.tmp3;                                                       \
            v.Tb=-v.tmp3;                                                       \
            v.Tc=-(v.tmp1+v.tmp2);                                              \
      }                                                                         \
                                                                                \

#endif // __SVGEN_H__


/* =================================================================================
File name:        RAMPGEN.H
===================================================================================*/

#ifndef __RAMPGEN_H__
#define __RAMPGEN_H__

typedef struct { float32_t  Freq;         // Input: Ramp frequency (pu)
                 float32_t  StepAngleMax; // Parameter: Maximum step angle (pu)
                 float32_t  Angle;        // Variable: Step angle (pu)
                 float32_t  Gain;         // Input: Ramp gain (pu)
                 float32_t  Out;          // Output: Ramp signal (pu)
                 float32_t  Offset;       // Input: Ramp offset (pu)
               } RAMPGEN;

/*------------------------------------------------------------------------------
      Object Initializers
------------------------------------------------------------------------------*/
#define RAMPGEN_DEFAULTS {0,        \
                          0,        \
                          0,        \
                          (1.0),   \
                          0,        \
                          (1.0),   \
                         }

/*------------------------------------------------------------------------------
    RAMP(Sawtooh) Generator Macro Definition
------------------------------------------------------------------------------*/

#define RG_MACRO(v)                                 \
                                                    \
/* Compute the angle rate */                        \
    v.Angle = v.Angle + (2*PI*v.StepAngleMax*v.Freq);       \
                                                    \
/* Saturate the angle rate within (-1,1) */         \
    if (v.Angle>(2*PI))                           \
        v.Angle -= (2*PI);                        \
    else if (v.Angle<(-2*PI))                     \
        v.Angle += (2*PI);                        \
        v.Out=v.Angle;

// Use the code snippet below if gain/offset needed.                                                    \
/* Compute the ramp output */                       \
    v.Out = _IQmpy(v.Angle,v.Gain) + v.Offset;      \
/* Saturate the ramp output within (-1,1) */        \
    if (v.Out>_IQ(1.0))                             \
        v.Out -= _IQ(1.0);                          \
    else if (v.Out<_IQ(-1.0))                       \
        v.Out += _IQ(1.0);

#endif // __RAMPGEN_H__


/* =================================================================================
File name:        RMP_CNTL.H
===================================================================================*/


#ifndef __RMP_CNTL_H__
#define __RMP_CNTL_H__

typedef struct { float32_t    TargetValue;    // Input: Target input (pu)
                 Uint32 RampDelayMax;   // Parameter: Maximum delay rate (Q0) - independently with global Q
                 float32_t    RampLowLimit;   // Parameter: Minimum limit (pu)
                 float32_t    RampHighLimit;  // Parameter: Maximum limit (pu)
                 Uint32 RampDelayCount; // Variable: Incremental delay (Q0) - independently with global Q
                 float32_t    SetpointValue;  // Output: Target output (pu)
                 Uint32 EqualFlag;      // Output: Flag output (Q0) - independently with global Q
                 float32_t    Tmp;            // Variable: Temp variable
               } RMPCNTL;


/*-----------------------------------------------------------------------------
Default initalizer for the RMPCNTL object.
-----------------------------------------------------------------------------*/
#define RMPCNTL_DEFAULTS {  0,       \
                            5,       \
                           (-1),  \
                           (1),   \
                            0,       \
                            0,       \
                            0,       \
                            0,       \
                          }

/*------------------------------------------------------------------------------
    RAMP Controller Macro Definition
------------------------------------------------------------------------------*/

#define RC_MACRO(v)                                                                 \
    v.Tmp = v.TargetValue - v.SetpointValue;                                        \
/*  0.0000305 is resolution of Q15 */                                               \
if (v.Tmp < 0.0)                                                                    \
{                                                                                   \
    v.Tmp = - v.Tmp;                                                                \
}                                                                                   \
if (v.Tmp >= (0.0000305))                                                        \
{                                                                                   \
    v.RampDelayCount++  ;                                                           \
        if (v.RampDelayCount >= v.RampDelayMax)                                     \
        {                                                                           \
            if (v.TargetValue >= v.SetpointValue)                                   \
                v.SetpointValue += (0.0000305);                                  \
            else                                                                    \
                v.SetpointValue -= (0.0000305);                                  \
                                                                                    \
            if (v.SetpointValue>v.RampHighLimit) v.SetpointValue = v.RampHighLimit; \
            if (v.SetpointValue<v.RampLowLimit)  v.SetpointValue = v.RampLowLimit;  \
            v.RampDelayCount = 0;                                                   \
                                                                                    \
        }                                                                           \
}                                                                                   \
else v.EqualFlag = 0x7FFFFFFF;

#endif // __RMP_CNTL_H__


/* =================================================================================
File name:        VOLT_CAL.H
===================================================================================*/


#ifndef __VOLT_CAL_H__
#define __VOLT_CAL_H__

typedef struct  { float32_t  DcBusVolt;       // Input: DC-bus voltage (pu)
                  float32_t  MfuncV1;         // Input: Modulation voltage phase A (pu)
                  float32_t  MfuncV2;         // Input: Modulation voltage phase B (pu)
                  float32_t  MfuncV3;         // Input: Modulation voltage phase C (pu)
                  Uint16  OutOfPhase;   // Parameter: Out of Phase adjustment (0 or 1) (Q0) - independently with global Q
                  float32_t  VphaseA;         // Output: Phase voltage phase A (pu)
                  float32_t  VphaseB;         // Output: Phase voltage phase B (pu)
                  float32_t  VphaseC;         // Output: Phase voltage phase C (pu)
                  float32_t  Valpha;          // Output: Stationary d-axis phase voltage (pu)
                  float32_t  Vbeta;           // Output: Stationary q-axis phase voltage (pu)
                  float32_t  temp;                // Variable: temp variable
                } PHASEVOLTAGE;


/*
OutOfPhase = 1 for the out of phase correction if
* MfuncV1 is out of phase with PWM1,
* MfuncV2 is out of phase with PWM3,
* MfuncV3 is out of phase with PWM5,
otherwise, set 0 if their phases are correct.
*/

/*-----------------------------------------------------------------------------
Default initalizer for the PHASEVOLTAGE object.
-----------------------------------------------------------------------------*/
#define PHASEVOLTAGE_DEFAULTS { 0, \
                                0, \
                                0, \
                                0, \
                                1, \
                                0, \
                                0, \
                                0, \
                                0, \
                                0, \
                                }

#define ONE_THIRD  (0.33333333333333)
#define TWO_THIRD  (0.66666666666667)
#define INV_SQRT3  (0.57735026918963)
/*------------------------------------------------------------------------------
    Phase Voltage Calculation Macro Definition
------------------------------------------------------------------------------*/


#define PHASEVOLT_MACRO(v)                                                      \
                                                                                \
                                                                                \
/* Scale the incomming Modulation functions with the DC bus voltage value*/     \
/* and calculate the 3 Phase voltages */                                        \
  v.temp      = (v.DcBusVolt*ONE_THIRD);                                  \
  v.VphaseA   = (v.temp*((2*v.MfuncV1)-v.MfuncV2-v.MfuncV3));        \
  v.VphaseB   = (v.temp*((2*v.MfuncV2)-v.MfuncV1-v.MfuncV3));        \
                                                                                \
  if (v.OutOfPhase==0)                                                          \
  {   v.VphaseA=-v.VphaseA;                                                     \
      v.VphaseB=-v.VphaseB;                                                     \
  }                                                                             \
/* Voltage transformation (a,b,c)  ->  (Alpha,Beta) */                          \
  v.Valpha = v.VphaseA;                                                         \
  v.Vbeta = ((v.VphaseA + (2*v.VphaseB))*INV_SQRT3);


#endif // __VOLT_CAL_H__

// Phase C (if needed)
// v.VphaseC   = _IQmpy(v.temp,(_IQmpy2(v.MfuncV3)-v.MfuncV2-v.MfuncV1));   \


/* =================================================================================
File name:        SPEED_PR.H
===================================================================================*/


#ifndef __SPEED_PR_H__
#define __SPEED_PR_H__

typedef struct {
       Uint32 NewTimeStamp;     // Variable : New 'Timestamp' corresponding to a capture event (Q0) - independently with global Q
       Uint32 OldTimeStamp;     // Variable : Old 'Timestamp' corresponding to a capture event (Q0) - independently with global Q
       Uint32 TimeStamp;        // Input : Current 'Timestamp' corresponding to a capture event (Q0) - independently with global Q
       Uint32 SpeedScaler;      // Parameter :  Scaler converting 1/N cycles to a GLOBAL_Q speed (Q0) - independently with global Q
       int32 EventPeriod;       // Input/Variable :  Event Period (Q0) - independently with global Q
       int16 InputSelect;       // Input : Input selection between TimeStamp (InputSelect=0) and EventPeriod (InputSelect=1)
       float32_t Speed;               // Output :  speed in per-unit
       Uint32 BaseRpm;          // Parameter : Scaler converting GLOBAL_Q speed to rpm (Q0) speed - independently with global Q
       int32 SpeedRpm;          // Output : speed in r.p.m. (Q0) - independently with global Q
       } SPEED_MEAS_CAP;        // Data type created


/*-----------------------------------------------------------------------------
Default initalizer for the SPEED_MEAS_CAP object.
-----------------------------------------------------------------------------*/

#define SPEED_MEAS_CAP_DEFAULTS   { 0, \
                                    0, \
                                    0, \
                                   260, \
                                    0, \
                                    0, \
                                    0, \
                                   1800, \
                                    0, \
                                  }

/*------------------------------------------------------------------------------
    SPEED_PR Macro Definition
------------------------------------------------------------------------------*/

#define SPEED_PR_MACRO(v)                                       \
   if (v.InputSelect == 0)                                      \
   {                                                            \
     v.OldTimeStamp = v.NewTimeStamp;                           \
     v.NewTimeStamp = v.TimeStamp;                              \
     v.EventPeriod = v.NewTimeStamp - v.OldTimeStamp;           \
                                                                \
        if (v.EventPeriod < 0)                                  \
            v.EventPeriod += 32767;   /* 0x7FFF = 32767*/       \
    }                                                           \
                                                                \
     v.Speed = (v.SpeedScaler/ v.EventPeriod);             \
                                                                \
/* Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q*/                \
   v.SpeedRpm = (v.BaseRpm*v.Speed);                      \

#endif // __SPEED_PR_H__


/* =================================================================================
File name:        SPEED_FR.H
===================================================================================*/


#ifndef __SPEED_FR_H__
#define __SPEED_FR_H__

typedef struct {
       float32_t ElecTheta;       // Input: Electrical angle (pu)
       Uint32 DirectionQep; // Variable: Direction of rotation (Q0) - independently with global Q
       float32_t OldElecTheta;    // History: Electrical angle at previous step (pu)
       float32_t Speed;           // Output: Speed in per-unit  (pu)
       Uint32 BaseRpm;      // Parameter: Base speed in rpm (Q0) - independently with global Q
       float32_t K1;            // Parameter: Constant for differentiator (Q21) - independently with global Q
       float32_t K2;              // Parameter: Constant for low-pass filter (pu)
       float32_t K3;              // Parameter: Constant for low-pass filter (pu)
       int32 SpeedRpm;      // Output : Speed in rpm  (Q0) - independently with global Q
       float32_t Tmp;             //Variable: Temp variable
       } SPEED_MEAS_QEP;    // Data type created

/*-----------------------------------------------------------------------------
Default initalizer for the SPEED_MEAS_QEP object.
-----------------------------------------------------------------------------*/
#define SPEED_MEAS_QEP_DEFAULTS   { 0, \
                                    1, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                  }

/*------------------------------------------------------------------------------
 SPEED_FR Macro Definition
------------------------------------------------------------------------------*/


#define SPEED_FR_MACRO(v)                                           \
/* Differentiator*/                                                 \
/* Synchronous speed computation   */                               \
   if ((v.ElecTheta < float32_t(0.9))&(v.ElecTheta > float32_t(0.1)))           \
/* Q21 = Q21*(GLOBAL_Q-GLOBAL_Q)*/                                  \
        v.Tmp = (v.K1*(v.ElecTheta - v.OldElecTheta));        \
   else v.Tmp = (v.Speed);                                 \
/* Low-pass filter*/                                                \
/* Q21 = GLOBAL_Q*Q21 + GLOBAL_Q*Q21*/                              \
    v.Tmp = (v.K2*(v.Speed))+(v.K3*v.Tmp);     \
/* Saturate the output */                                           \
    v.Tmp=_IQsat(v.Tmp,_IQ21(1),_IQ21(-1));                         \
    v.Tmp= (v.Tmp>float32_t(1)) ? float32_t(1) : ((v.Tmp<float32_t(-1))? float32_t(-1) : v.Tmp )  \
    v.Speed = (v.Tmp);                                     \
/* Update the electrical angle */                                   \
    v.OldElecTheta = v.ElecTheta;                                   \
/* Change motor speed from pu value to rpm value (GLOBAL_Q -> Q0)*/ \
/* Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q*/                    \
    v.SpeedRpm = (v.BaseRpm*v.Speed);

#endif // __SPEED_FR_H__
