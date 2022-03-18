
// the includes
//

#include "F28x_Project.h"
#include "device.h"
#include "driverlib.h"
#include "vienna.h"
#define MATH_TYPE FLOAT_MATH
#include "IQmathLib.h"
#include "CLAmath.h"

// Include Flash API include file
#include "F021_F28004x_C28x.h"

// Include Flash API example header file
#include "flash_programming_f28004x.h"



#define PowerRelay  GpioDataRegs.GPADAT.bit.GPIO14
#define DisablePWM  GpioDataRegs.GPADAT.bit.GPIO22
#define ExtSwitch   GpioDataRegs.GPBDAT.bit.GPIO30
#define LEDRed      GpioDataRegs.GPADAT.bit.GPIO15
//#define TimeTest    GpioDataRegs.GPBDAT.bit.GPIO26
#define TimeTest    GpioDataRegs.GPASET.bit.GPIO26
#define TimeTestN   GpioDataRegs.GPACLEAR.bit.GPIO26
//#define TimeTestToggle GpioDataRegs.GPATOGGLE.bit.GPIO26;
#define SCIBootCap  GpioDataRegs.GPADAT.bit.GPIO17

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

#define UP_Key          0x00FF6996//0x01FE58A7             // Increment Key-code
#define DOWN_Key        0x00FFA956//0x01FEA05F             // Decrement Key-code
#define RIGHT_Key       0x00FFC936//0x01FEC03F
#define LEFT_Key        0x00FF49B6//0x01FE807F
#define MODE_Key        0x00FF9966//0x01FE40BF
#define OnOff_Key       0x00FF59A6


enum LCD_STATE {LCD_NO_ACTION=0, SCREEN1R1, SCREEN1R2, SCREEN2R1, SCREEN2R2, SCREEN3R1, SCREEN3R2, SCREEN4R1, SCREEN4R2, SCREEN5R1, SCREEN5R2, SCREEN6R1, SCREEN6R2, SCREEN7R1, SCREEN7R2, SCREEN8R1, SCREEN8R2, SCREEN9R1, SCREEN9R2, CLEAR_SCREEN, LINE1, LINE2, CURSOR_ON, CURSOR_OFF, SHIFT_RIGHT, SHIFT_LEFT};
enum UPDOWNFUNC {KeyForScreenMov=0, KeyForPasswordConst, KeyForChangeIDofConst, KeyForChangeofConst};


struct LCD_VARS {
    char *string_LCD;
    char LSNibble;
    char MSNibble;
    char char_LCD;
    Uint16 i_LCD;
    Uint16 j_LCD;
    char command_LCD;
    Uint16 LCD_ACTION;
    Uint16 Screen_count;
    Uint16 UpDownKeyFunc;
    Uint16 Command_Const;
    Uint16 Command_PwD;
    Uint16 PasswordConst;
    float Dis_Constant;
    Uint16 ID_Const;
    float Constant;
    float32_t *Ptr_Constant;
    unsigned int x_int;
    unsigned int x_deca;
    unsigned int x_hecto;
    unsigned int x_kelo;
    unsigned int x_10kelo;
    unsigned int x_100kelo;
    unsigned int x_mega;
    unsigned int x_10mega;
    float Pot_percent;
    float Duty_percent;
    Uint16 HMI_Change_Constants;
    int idx;
    Uint16 Seq_screen;
    Uint16 Prev_Screen_count;
    Uint16 New_string:1;
    Uint16 New_command:1;
    Uint16 res:14;
};
#define LCD_VARS_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0}

struct IR_VARS {
    Uint32  ECap1IntCount;
    Uint32  ECap1PassCount;
    uint32_t ecap1IntCount;
    uint32_t ecap1PassCount;
    volatile uint16_t cap2Count;
    volatile uint16_t cap3Count;
    volatile uint16_t cap4Count;

    Uint16 flag_start_remote;
    Uint16 packet_remote;
    unsigned long Switch;
    unsigned long Last_Switch;
    unsigned long Prev_Switch;
    Uint16 Longpress;
//    Uint16 UpDownKeyFunc;
};
#define IR_VARS_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0}

struct COMMON_FLAG {
    uint16_t clearTrip:1;
    uint16_t res:15;
};
#define COMMON_FLAG_DEFAULTS {0,0}

struct COMMAND {
    uint16_t OnOffCh1;
    uint16_t OnOffCh2;
    uint16_t Clear_OCfault;
};
#define COMMAND_DEFAULTS {0,0,0}

#define NosOfConst 73
#define WORDS_IN_FLASH_BUFFER   NosOfConst*2 //70*2 for float variable; // Length (in 16-bit words) of data buffer used for program

//variable for UID of the device In global variable initialization
#define device_uid_address 0x000703CC   //for F280049 UID address is  0x000703CC and 0x000703CD and long int pick up both 16bit LSB and 16bit MSB together from 0x000703CC+0x000703CD

struct UID {
    long int *Ptr_voltage_plus;     //This is actual pointer to uid
    long int voltage_plus;       //This will get actual uid after mathematical manipulation
    long int uid_dum;               //It is dummy variable to confuse
};
#define UID_DEFAULTS {0,0,0}

void init_critical_gpio(void);

void init_hmi_gpio(void);

void LCD_selection(void);
void Command_Process(void);
void Array_Process(void);
void Byte_Process(void);
void MSNibble_Process(void);
void LSNibble_Process(void);
void Lcd_Nibble_Port(unsigned char Value);
void WriteCommandLCD(unsigned char CommandByte);
void StateControl_RemoteScreen(void);
void LoopforLCDPtr(void);

void remote_operation(void);
void ISR_remote(void);
void remote_key_display_state_machine(void);
void ActionOn_UpDownKeyFunc(void);
void ActionOn_OnOff_Key_and_Screen_count(void);

//LCD1602
void CursorON(void);                              /* Make Cursor visible */
void CursorOFF(void);                             /* Hide cursor */
void DisplayLCD(char LineNumber,char *Message);   /* Display the given message (16 characters) at given location on LCD */
//void WriteCommandLCD(unsigned char CommandByte);  /* Write the given command to LCD */
void WriteDataLCD(unsigned char DataByte);        /* Write the given data to LCD */
void InitializeLCD(void);                         /* Initialize LCD */
void LCDDelay(void);
void LCDDelay1600(void);
void SendByte(unsigned char Value);

void remote_key_display_state_machine55(void);

__interrupt void ecap1_isr(void);
void InitECapture(void);
void InitEPwmTimer(void);
void Fail(void);
void initECAP(void);
__interrupt void ecap1ISR(void);






void configCLAMemory(void);
//=====
// Defines
void pickup_constant(void);

void first_time_flash_writting_after_erase(void);
void load_flash_constants(void);
// Prototype of the functions used in this example
void Example_Error(Fapi_StatusType status);
void Example_Done(void);
void Write_to_Flash(void);//void Example_CallFlashAPI(void);
void FMSTAT_Fail(void);
void fill_buffer_forflashwrite(void);


void CodeSecurity(void);


void InitializeSCI(void);
void trasmit_vars(void);
void pickup_vars(void);
