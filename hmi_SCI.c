
#include "hmi.h"

///////////////////////// added just for startup initialization for PMSM and IM in accordance with their base freq and good performance
#define IM 1
#define PMSM 2
#define MACHINE IM
/////////////////////////


extern struct LCD_VARS lcd;
extern struct COMMON_FLAG common_flag;
extern struct COMMAND command;
extern struct IR_VARS ir;

extern volatile float32_t *Ptr_Constant;
// Globals
//
uint16_t counter = 0;
unsigned char *msg;

// Function Prototypes
//
__interrupt void sciaTxISR(void);
__interrupt void sciaRxISR(void);

// Main
//

char fArray[4];
float f1 = -3.141592;
float f2 = 0;

//--------
uint16_t sharenow=0;
unsigned char byteArr[6];

unsigned char type_exchange;
unsigned char index_var;
unsigned char index_parameter;
float transmitValue;
volatile uint16_t command_OnOff=0;

// union Float to handle casting of byte arrays to float
typedef union _Float{
    float f;
    unsigned long bytes;
} Float;

Float receivedValue;

float test=0;

unsigned char detect_parameter_transfer;
unsigned char byte[4];
extern float32_t VdTesting;


extern float vars11;
extern float vars12;
extern float vars31;
extern float vars32;
extern float vars33;
extern float vars34;
extern float vars35;
extern float vars36;

//-------

void InitializeSCI(void)
{
    //// LED Init
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);


    // GPIO28 is the SCI Rx pin.
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCIRXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    //
    // GPIO29 is the SCI Tx pin.
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);

    // Map the ISR to the wake interrupt.
     //
     Interrupt_register(INT_SCIA_TX, sciaTxISR);
     Interrupt_register(INT_SCIA_RX, sciaRxISR);

     //
     // Initialize SCIA and its FIFO.
     //
     SCI_performSoftwareReset(SCIA_BASE);

     //
     // Configure SCIA for echoback.
     //
     SCI_setConfig(SCIA_BASE, 50000000, 9600, (SCI_CONFIG_WLEN_8 |
                                              SCI_CONFIG_STOP_ONE |
                                              SCI_CONFIG_PAR_NONE));
     SCI_resetChannels(SCIA_BASE);
     SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
     SCI_enableFIFO(SCIA_BASE);
     SCI_enableModule(SCIA_BASE);
     SCI_performSoftwareReset(SCIA_BASE);

     //
     // Set the transmit FIFO level to 0 and the receive FIFO level to 2.
     // Enable the TXFF and RXFF interrupts.
     //
     SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX6, SCI_FIFO_RX6);      //for transmitting 6 bytes together to generate interrupt
     SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXFF);
     SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXFF);

 #ifdef AUTOBAUD
     //
     // Perform an autobaud lock.
     // SCI expects an 'a' or 'A' to lock the baud rate.
     //
     SCI_lockAutobaud(SCIA_BASE);
 #endif

//     Float receivedValue;

     //
     // Send starting message.
     //
     msg = "\r\n\n\nHello World!\0";
     SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 17);
     msg = "\r\nYou will enter a character, and the DSP will echo it back!\n\0";
     SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 62);

/*
 //    sprintf(my_string, "%f", my_float);

     // From a float variable (f1) to a chars array (fArray):

     fArray[0] =  __mov_byte((int *)&f1,0);
     fArray[1] =  __mov_byte((int *)&f1,1);
     fArray[2] =  __mov_byte((int *)&f1,2);
     fArray[3] =  __mov_byte((int *)&f1,3);

     // From a chars array (fArray) to a float variable (f2):

     __byte((int *)&f2,0) = fArray[0];
     __byte((int *)&f2,1) = fArray[1];
     __byte((int *)&f2,2) = fArray[2];
     __byte((int *)&f2,3) = fArray[3];
*/

     //
     // Clear the SCI interrupts before enabling them.
     //
     SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
     SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);

//     SCI_disableInterrupt(SCIA_BASE, SCI_INT_TXFF);//////

     //
     // Enable the interrupts in the PIE: Group 9 interrupts 1 & 2.
     //
     Interrupt_enable(INT_SCIA_RX);
//     Interrupt_enable(INT_SCIA_TX);
     Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}

//
// sciaTxISR - Disable the TXFF interrupt and print message asking
//             for two characters.
//
__interrupt void
sciaTxISR(void)
{
    //
    // Disable the TXRDY interrupt.
    //
/*
    SCI_disableInterrupt(SCIA_BASE, SCI_INT_TXFF);


    msg = "\r\nEnter two characters: \0";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 26);

*/
    trasmit_vars();
    //
    // Acknowledge the PIE interrupt.
    //
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


//
// sciaRxISR - Read two characters from the RXBUF and echo them back.
//
uint16_t receivedChar1, receivedChar2, receivedChar3, receivedChar4, receivedChar5, receivedChar6;
char received_char[6];
char transmit_char[6];
float received_float=0.0;
float transmit_float=0.0;
Uint16 SCIBT_Change_Constants=0;
Uint16 True_password_SCIBT=0;
float *Ptr_recieve_var;
float speed_ref=0;

Uint16 CommandOnOff = 0;
extern Uint16 select_varDAC;
extern float32_t SpeedRef;
extern float32_t VqTesting;
extern float32_t IdRef;
extern float32_t IqRef;

void Process_SCI_Received_Data();

__interrupt void
sciaRxISR(void){
    //
    // Enable the TXFF interrupt again.
    //
//    SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXFF);



    // Read six characters from the FIFO.
    received_char[0] = SCI_readCharBlockingFIFO(SCIA_BASE);
    received_char[1] = SCI_readCharBlockingFIFO(SCIA_BASE);
    received_char[2] = SCI_readCharBlockingFIFO(SCIA_BASE);
    received_char[3] = SCI_readCharBlockingFIFO(SCIA_BASE);
    received_char[4] = SCI_readCharBlockingFIFO(SCIA_BASE);
    received_char[5] = SCI_readCharBlockingFIFO(SCIA_BASE);

    Process_SCI_Received_Data(&receivedValue);

    //
    // Clear the SCI RXFF interrupt and acknowledge the PIE interrupt.
    //
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}

// copy the 4 bytes stored in fvalue into byteArr
void floatToByteArray(Float * fvalue, unsigned char * byteArr, unsigned char index_var){


    byteArr[5] = index_var;
    unsigned long copy = fvalue->bytes, mask = 0xFF;
    int i = 1;
    for(; i<5; i++){
        byteArr[i] = (copy >> 8*(4-i) & mask);
    }

    byteArr[0] = 0x08;
}

//char transmit_char1[6];
Uint16 ID_Vars=0;
volatile float32_t *Ptr_Vars;
#define NosOfVars 3 //10
float32_t temp_vars;
Uint16 Enable_Vars_trasmit_SCIBT=1;
Uint16 rec_byte[6];
Uint16 trans_byte[6];

//float float_variable = 1.11;
//unsigned char bytes_array[4];
//
//*((float *)bytes_array) = float_variable;

//typedef union{
//    float a;
//    unsigned char bytes[4];
//  }thing;
//
//thing.a = 1.234;



//floatToBytes(floatValue,*byteArray);

void trasmit_vars(Float *transmitValue) //It should be in 10msec or slower loop //Transferring 6*8 bits+2bytes gap= 64bits in one run: minimum time=64/9600=6.6msec
{
/*
    if (!Enable_Vars_trasmit_SCIBT) { //Enable variable transmit over SCI along with their IDs and ack
        return;
    }
*/

    // Send one variable in one run and increment for next variable.
    if (ID_Vars>NosOfVars) {    //increment and rotation for ID_Vars
        ID_Vars = 0;
    }

    ID_Vars++;
    index_var = ID_Vars;
    pickup_vars();
    temp_vars = *Ptr_Vars;
    transmitValue->f = *Ptr_Vars;

    //converts Float variable 'receivedValue' to byte array
//    floatToByteArray(&transmitValue, byteArr, index_var);


    // Send bytes; From a float variable to a chars array and append with IDs, commands, etc:

    // Send bytes; From a float variable to a chars array and append with IDs, commands, etc:

    SCI_writeCharBlockingFIFO(SCIA_BASE, (char) 0x08);
    SCI_writeCharBlockingFIFO(SCIA_BASE, __mov_byte((int *)&temp_vars,0));
    SCI_writeCharBlockingFIFO(SCIA_BASE, __mov_byte((int *)&temp_vars,1));
    SCI_writeCharBlockingFIFO(SCIA_BASE, __mov_byte((int *)&temp_vars,2));
    SCI_writeCharBlockingFIFO(SCIA_BASE, __mov_byte((int *)&temp_vars,3));
    SCI_writeCharBlockingFIFO(SCIA_BASE, (char) ID_Vars);      //It may use for acknowledge signal

    trans_byte[0]=(char) ID_Vars;
    trans_byte[1]=__mov_byte((int *)&temp_vars,0);
    trans_byte[2]=__mov_byte((int *)&temp_vars,1);
    trans_byte[3]=__mov_byte((int *)&temp_vars,2);
    trans_byte[4]=__mov_byte((int *)&temp_vars,3);
    trans_byte[5]=(char) 0x33;

//    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);
//
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

// read floats through SCI
// reads 4 bytes at a time and cast and stores to the receivedValue Float variable.
void Process_SCI_Received_Data(Float * receivedValue){

    if (received_char[0]==0x11) {  //command 0x01 for On and Off; received_char[0] holds command
//        if (command_OnOff) command_OnOff=0;
//        else command_OnOff=1;
        command_OnOff=1;
        test=1;

///////////////////////// added just for startup initialization for PMSM and IM in accordance with their base freq and good performance
        if (MACHINE == IM){
            SpeedRef =0.35;
        }
        else{
            SpeedRef =0.2;
        }
/////////////////////////

        GPIO_writePin(23, 0);
        LEDRed=1;
//        GpioDataRegs.GPASET.bit.GPIO15=1;
        //        ir.Last_Switch=OnOff_Key;  //To synchronize with IR based OnOff button operation
    }
    else if (received_char[0]==0x12) {  //command 0x11 for turn off
        command_OnOff=0;
        test=2;
        GPIO_writePin(23, 1);
        LEDRed=0;
       }
    else if (received_char[0]==0x14) {  //command 0x02 for upscreen; received_char[0] holds command
        ir.Last_Switch=UP_Key;   //To synchronize with IR based UPKEY button operation
        SpeedRef = SpeedRef + 0.0125;

//        /////////////////////////////////////////////////////////
//        IqRef=IqRef+0.025;// only for level-3
//        /////////////////////////////////////////////////////////
    }
    else if (received_char[0]==0x15) {  //command 0x03 for downscreen; received_char[0] holds command
        ir.Last_Switch=DOWN_Key;  //To synchronize with IR based Downkey button operation
        SpeedRef = SpeedRef - 0.0125;

//        /////////////////////////////////////////////////////////
//        IqRef=IqRef-0.025;// only for level-3
//        /////////////////////////////////////////////////////////
    }
    else if (received_char[0]==0x08) {  //command 0x03 for downscreen; received_char[0] holds command
        IdRef = IdRef + 0.0125;
    }
    else if (received_char[0]==0x09) {  //command 0x03 for downscreen; received_char[0] holds command
        IdRef = IdRef - 0.0125;
    }
    else if (received_char[0]==0x05) {  //command 0x05 for password match for SCIBT to change variables; received_char[0] holds command
        if ( received_char[1]==0x62 && received_char[2]==0x84  )    //match these passwords received from app to enable constants to change.
            True_password_SCIBT = 1;
        else
            True_password_SCIBT = 0;
    }
    else if (received_char[0]==0x00) {  //command 0x04 for changing constants and loading to flash; received_char[0] holds command

//        if (!True_password_SCIBT) return; //Provide protection lock

        lcd.ID_Const = received_char[5]; //received_char[5] folds ID number
        //pickup_constant();      //This can also be used for sofware-POT operation. Good thing is that POT last value will store in flash after Off command

        byte[0] = received_char[1];
        byte[1] = received_char[2];
        byte[2] = received_char[3];
        byte[3] = received_char[4];

        receivedValue->bytes = ((unsigned long)byte[0]<<24 | (unsigned long)byte[1] << 16 | (unsigned long)byte[2] << 8 | (unsigned long)byte[3]);
//        receivedValue->bytes = ((unsigned long)byte[0]<<24 | (unsigned long)byte[1] << 16 | (unsigned long)byte[2] << 8 | (unsigned long)byte[3]);

        *Ptr_recieve_var = receivedValue->f;

//        speed_ref = (float)received_char[1];
//        if (speed_ref < -1.0) speed_ref = -1.0;
//        if (speed_ref > 1.0) speed_ref = 1.0;

        SCIBT_Change_Constants = 1;

        index_parameter = received_char[5];


// for testing only
//        if (index_parameter==0x11) {
//            //use new value of parameter=receivedValue->f with index as above
//            //transmitValue=;
//
//            //pass by reference- s changes the value in place
//            // do some calculations
//            receivedValue->f = receivedValue->f*10.0;
//            //calculation1(&receivedValue);
//            index_var = 0x01;
//        }

        if (index_parameter==0x12) {
            //use new value of parameter=receivedValue->f with index as above
            //transmitValue=;

            //pass by reference- s changes the value in place
            // do some calculations
            receivedValue->f = receivedValue->f*1000.0;
            //calculation2(&receivedValue);
            index_var = 0x02;
        }

        if (index_parameter==0x13) {
            //use new value of parameter=receivedValue->f with index as above
            //transmitValue=;

            //pass by reference- s changes the value in place
            // do some calculations
            receivedValue->f = receivedValue->f*10.0;
            //calculation1(&receivedValue);
            index_var = 0x03;
        }

        if (index_parameter==0x14) {
            //use new value of parameter=receivedValue->f with index as above
            //transmitValue=;

            //pass by reference- s changes the value in place
            // do some calculations
            receivedValue->f = receivedValue->f*(-2.0);
            //calculation1(&receivedValue);
            index_var = 0x04;
        }

        if (index_parameter==0x15) {
            //use new value of parameter=receivedValue->f with index as above
            //transmitValue=;

            //pass by reference- s changes the value in place
            // do some calculations
            receivedValue->f = receivedValue->f*1.0;
            //calculation1(&receivedValue);
            index_var = 0x05;
        }

        if (index_parameter==0x16) {
            //use new value of parameter=receivedValue->f with index as above
            //transmitValue=;

            //pass by reference- s changes the value in place
            // do some calculations
            receivedValue->f = receivedValue->f*(-30.1);
            //calculation1(&receivedValue);
            index_var = 0x06;
        }



/*
        // From a chars array (fArray) to a float variable (f2):
        __byte((int *)&received_float,0) = received_char[4];
        __byte((int *)&received_float,1) = received_char[3];
        __byte((int *)&received_float,2) = received_char[2];
        __byte((int *)&received_float,3) = received_char[1];

        *Ptr_Constant = received_float;
*/


    }
    else if (received_char[0]==0x06) {
        select_varDAC++;
        if (select_varDAC>3) select_varDAC=1;
    }
    else if (received_char[0]==0x07) {  //command 0x07 for SCI boot loading capacitor charge; received_char[0] holds command
//        if (!True_password_SCIBT) return; //Provide protection lock

        SCIBootCap = ENABLE;
    }


    if (detect_parameter_transfer==0x04) {


    }
}

float v0=10.5;
float v1=5;
float v2=10;
float v3=25.60;



//For assigning picking up variables for transmit (monitoring) THERE IS BETTER WAY (SHORT CODE) IF USED STRUCTURE
volatile float32_t dummy2;
void pickup_vars(void)
{
    switch(ID_Vars){
    case 0:
//            Ptr_Vars = &CONTROL_STATE;
            Ptr_Vars = &v0;
        break;
//    case 1:
//            Ptr_Vars = &v1;
//            break;
//    case 2:
//            Ptr_Vars = &v2;
//            break;
//    case 3:
//            Ptr_Vars = &v3;
//            break;
    case 1:
        Ptr_Vars = &VIENNA_vDCMeas_pu;
        break;
    case 2:
        Ptr_Vars = &VIENNA_iL1Meas_pu;
        break;
    case 3:
        Ptr_Vars = &VIENNA_iL2Meas_pu;
        break;
    case 4:
        Ptr_Vars = &vars33;
        break;
    case 5:
        Ptr_Vars = &vars34;
        break;
    case 6:
        Ptr_Vars = &vars11;
        break;
    case 7:
        Ptr_Vars = &vars31;
        break;
    case 8:
        Ptr_Vars = &vars12;
        break;
    case 9:
        Ptr_Vars = &vars32;
        break;
    case 10:
        Ptr_Vars = &VIENNA_TEMPMeas_pu;
        break;
    case 11:
        Ptr_Vars = &VIENNA_vCCMeas_pu;
        break;
    case 12:
        Ptr_Vars = &common_vars_duty1;
        break;
    case 13:
        Ptr_Vars = &common_vars_duty2;
        break;
    case 14:
        Ptr_Vars = &VIENNA_POTMeas_pu;
        break;
    case 15:
        Ptr_Vars = &Ns2;
        break;
    case 16:
        Ptr_Vars = &Np2;
        break;
    case 17:
        Ptr_Vars = &ILight2;
        break;
    case 18:
        Ptr_Vars = &reserved;
        break;
    case 19:
        Ptr_Vars = &Np2;
        break;
    case 20:
        Ptr_Vars = &Io2;
        break;
    case 21:
        Ptr_Vars = &Rs2;
        break;
    case 22:
        Ptr_Vars = &update_const_upper_limit_vpv2_controlout;
        break;
    case 23:
        Ptr_Vars = &update_const_lower_limit_vpv2_controlout;
        break;
    case 24:
        Ptr_Vars = &update_const_ipv1_ref_isc;
        break;
    case 25:
        Ptr_Vars = &update_const_vpv1_ref_vsc;
        break;
    case 26:
        Ptr_Vars = &update_const_vpv1_ref_voc;
        break;
    case 27:
        Ptr_Vars = &update_const_ipv1_ref_imp;
        break;
    case 28:
        Ptr_Vars = &update_const_slop1_vpv1_ref_gen;
        break;
    case 29:
        Ptr_Vars = &update_const_constant1_vpv1_ref_gen;
        break;
    case 30:
        Ptr_Vars = &update_const_slop2_vpv1_ref_gen;
        break;
    default:
        Ptr_Vars = &dummy2;
        break;
    }
}

