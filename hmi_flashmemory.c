
#include "hmi.h"

extern struct IR_VARS ir;
extern struct LCD_VARS lcd;
extern struct COMMON_FLAG common_flag;
extern struct COMMAND command;
extern struct UID uid;

extern long int Decimizer[];
extern float32_t Constants_Default[];
extern Uint16 True_password_SCIBT;


//Data Buffers used for program operation using the flash API program function
#pragma  DATA_SECTION(Buffer,"DataBufferSection");
//uint16   Buffer[WORDS_IN_FLASH_BUFFER + 1];
float32_t   Buffer[NosOfConst + 1];
uint32   *Buffer32 = (uint32 *)Buffer;
//float32_t Read_Buffer[WORDS_IN_FLASH_BUFFER + 1];
float32_t *Ptr_flash_constants;
volatile float32_t *Ptr_Constant;
extern Uint16 SCIBT_Change_Constants;

Uint16 errorcheck=0;

//
void load_flash_constants(void)
{
//    Ptr_flash_constants = (uint32 *)Bzero_Sector6_start; //(uint16*)0x86000;
    Ptr_flash_constants = (float32_t *)Bzero_Sector6_start; //(uint16*)0x86000;
    lcd.ID_Const=0;
    for(lcd.idx = 0; lcd.idx < (NosOfConst+1); lcd.idx++)
    {
        pickup_constant();
        *Ptr_Constant = *Ptr_flash_constants;  //Constants_Default[idx]
        lcd.ID_Const++;
        Ptr_flash_constants++;
    }
}

void first_time_flash_writting_after_erase(void)
{
    if (*(uint32_t*)Bzero_Sector6_start) { //Read a very first value in the selected sector. It is 0xFFFFFFFF after erase flash. It will become 0x0 after first time flash writing.
//        errorcheck=*(uint32_t*)Bzero_Sector6_start;
        Load_Default_Constants=1;
        fill_buffer_forflashwrite();
//        errorcheck=*(uint32_t*)Bzero_Sector6_start;
    }
}

void IfChangeConstants_flashWrite(void)
{
    if (lcd.HMI_Change_Constants){
        if (command.OnOffCh1==OFF && command.OnOffCh2==OFF && lcd.UpDownKeyFunc!=KeyForChangeofConst && lcd.UpDownKeyFunc!=KeyForChangeIDofConst){
            fill_buffer_forflashwrite();
        }
    }
}

void fill_buffer_forflashwrite(void)
{
    int i;
    // Fill a buffer with data to program into the flash.
     //
     lcd.ID_Const=0;
     for(i=0; i <= NosOfConst; i++)
     {
         if ((uint16_t)Load_Default_Constants) {
             Buffer[i]= Constants_Default[i];       //loading defaults to buffer for write to the flash memory
             pickup_constant();                     //loading defaults immediately to variables without wait for flash pulling
             *Ptr_Constant = Constants_Default[i];  //loading defaults immediately to variables without wait for flash pulling
             lcd.ID_Const++;
         }
         else if ((lcd.HMI_Change_Constants==1)|| (SCIBT_Change_Constants==1)) {    //if (HMI_Change_Constants==1)  is redandent, if used bluetooth in future
             pickup_constant();
             Buffer[i]= *Ptr_Constant;
             lcd.ID_Const++;
         }
     }
     Load_Default_Constants = 0;
     lcd.HMI_Change_Constants = 0;
     SCIBT_Change_Constants = 0;
     True_password_SCIBT = 0;
     Write_to_Flash();
}

//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(Write_to_Flash, ".TI.ramfunc");
#endif

void Write_to_Flash(void)
{
    uint32 u32Index = 0;
    uint16 i = 0;
    Fapi_StatusType  oReturnCheck;
    Fapi_FlashStatusType  oFlashStatus;
    Fapi_FlashStatusWordType  oFlashStatusWord;

    EALLOW;
    //
    // Note that wait-states are already configured in the Device_init().
    // However, if INTOSC is used as the clock source and
    // if the CPUCLK falls in the range (97,100] (check other ranges given in DS),
    // then an extra wait state is needed for FSM operations (erase/program).
    // Hence, below function call should be uncommented in case INTOSC is used.
    // At 100MHz, execution wait-states for external oscillator is 4 and hence
    // in this example, a wait-state of 5 is used below.
    // This example is using external oscillator as the clock source and hence
    // below is commented.
    //
    // This wait-state setting impacts both Flash banks. Applications which
    // perform simultaneous READ/FETCH of one bank and PROGRAM or ERASE of the other
    // bank must use the higher RWAIT setting during the PROGRAM or ERASE operation. OR
    // use a clock source or frequency with a common wait state setting
    // Example: Use 97MHz instead of 100MHz if it is acceptable for the application.
    //
    // In case, if user application increments wait-state before using API,
    // then remember to revert back to the original wait-state after the API usage
    // to avoid extra wait-state during application execution from Flash.
    //
    //
    Flash_setWaitstates(FLASH0CTRL_BASE, 5);

    // Initialize the Flash API by providing the Flash register base address
    // and operating frequency.
    // This function is required to initialize the Flash API based on System frequency
    // before any other Flash API operation can be performed.
    // This function must also be called whenever System frequency or RWAIT is changed.
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 100);

    if(oReturnCheck != Fapi_Status_Success)
    {//errorcheck=1;
        // Check Flash API documentation for possible errors
        Example_Error(oReturnCheck);
    }

    // Initialize the Flash banks and FMC for erase and program operations.
    // Fapi_setActiveFlashBank() function sets the Flash banks and FMC for further
    // Flash operations to be performed on the banks.
    // Note: It does not matter which bank is passed as the parameter to initialize.
    //       Both Banks and FMC get initialized with one function call unlike F2837xS.
    //       Hence there is no need to execute Fapi_setActiveFlashBank() for each bank.
    //       Executing for one bank is enough.
    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);

    if(oReturnCheck != Fapi_Status_Success)
    {//errorcheck=2;
        // Check Flash API documentation for possible errors
        Example_Error(oReturnCheck);
    }


    // Erase Flash Bank0 sector6
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                                        (uint32 *)Bzero_Sector6_start);

    // Wait until FSM is done with erase sector operation
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}

    if(oReturnCheck != Fapi_Status_Success)
    {//errorcheck=3;
        // Check Flash API documentation for possible errors
        Example_Error(oReturnCheck);
    }

    // Read FMSTAT register contents to know the status of FSM after
    // erase command to see if there are any erase operation related errors
    oFlashStatus = Fapi_getFsmStatus();
    if(oFlashStatus != 0)
    {//errorcheck=4;
        // Check Flash API documentation for FMSTAT and debug accordingly
        // Fapi_getFsmStatus() function gives the FMSTAT register contents.
        // Check to see if any of the EV bit, ESUSP bit, CSTAT bit or
        // VOLTSTAT bit is set (Refer to API documentation for more details).
        FMSTAT_Fail();
    }

    // Do blank check
    // Verify that Bank0 sector6 is erased.  The Erase command itself does a verify as
    // it goes.  Hence erase verify by CPU reads (Fapi_doBlankCheck()) is optional.
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Bzero_Sector6_start,
                   Sector8KB_u32length,
                   &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
    {//errorcheck=5;
        // Check Flash API documentation for error info
        Example_Error(oReturnCheck);
    }


    // A data buffer of max 8 16-bit words can be supplied to the program function.
    // Each word is programmed until the whole buffer is programmed or a
    // problem is found. However to program a buffer that has more than 8
    // words, program function can be called in a loop to program 8 words for
    // each loop iteration until the whole buffer is programmed.
    //
    // Remember that the main array flash programming must be aligned to
    // 64-bit address boundaries and each 64 bit word may only be programmed
    // once per write/erase cycle.  Meaning the length of the data buffer
    // (3rd parameter for Fapi_issueProgrammingCommand() function) passed
    // to the program function can only be either 4 or 8.
    //
    // Program data in Flash using "AutoEccGeneration" option.
    // When AutoEccGeneration opton is used, Flash API calculates ECC for the given
    // 64-bit data and programs it along with the 64-bit main array data.
    // Note that any unprovided data with in a 64-bit data slice
    // will be assumed as 1s for calculating ECC and will be programmed.
    //
    // Note that data buffer (Buffer) is aligned on 64-bit boundary for verify reasons.
    //
    // Monitor ECC address for Bank0 Sector6 while programming with AutoEcc mode.
    //


    for(i=0, u32Index = Bzero_Sector6_start;
       (u32Index < (Bzero_Sector6_start + WORDS_IN_FLASH_BUFFER)) &&
       (oReturnCheck == Fapi_Status_Success); i+= 4, u32Index+= 8 )
    {
        oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index, Buffer+i, 8, 0, 0, Fapi_AutoEccGeneration);     //float32 *pu16DataBuffer, IT WAS uint16 *pu16DataBuffer,

        // Wait until the Flash program operation is over
        while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

        if(oReturnCheck != Fapi_Status_Success)
        {//errorcheck=6;
            // Check Flash API documentation for possible errors
            Example_Error(oReturnCheck);
        }

        // Read FMSTAT register contents to know the status of FSM after
        // program command to see if there are any program operation related errors
        oFlashStatus = Fapi_getFsmStatus();
        if(oFlashStatus != 0)
        {//errorcheck=7;
            //Check FMSTAT and debug accordingly
            FMSTAT_Fail();
        }

/*
        // Verify the programmed values.  Check for any ECC errors.
        // The program command itself does a verify as it goes.
        // Hence program verify by CPU reads (Fapi_doVerify()) is optional.
        oReturnCheck = Fapi_doVerify((uint32 *)u32Index,
                                     4, Buffer32+(i/2),
                                     &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {errorcheck=8;
            // Check Flash API documentation for possible errors
            Example_Error(oReturnCheck);
        }
*/
    }

/*

    // Erase the sector that is programmed above
    // Erase Bank0 Sector6
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                   (uint32 *)Bzero_Sector6_start);

    // Wait until FSM is done with erase sector operation
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}

    if(oReturnCheck != Fapi_Status_Success)
    {
        // Check Flash API documentation for possible errors
        Example_Error(oReturnCheck);
    }

    // Read FMSTAT register contents to know the status of FSM after
    // erase command to see if there are any erase operation related errors
    oFlashStatus = Fapi_getFsmStatus();
    if(oFlashStatus != 0)
    {
        // Check Flash API documentation for FMSTAT and debug accordingly
        // Fapi_getFsmStatus() function gives the FMSTAT register contents.
        // Check to see if any of the EV bit, ESUSP bit, CSTAT bit or
        // VOLTSTAT bit is set (Refer to API documentation for more details).
        FMSTAT_Fail();
    }

    // Do blank check
    // Verify that Bank0 sector6 is erased.  The Erase command itself does a verify as
    // it goes.  Hence erase verify by CPU reads (Fapi_doBlankCheck()) is optional.
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Bzero_Sector6_start,
                   Sector8KB_u32length,
                   &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
    {
        // Check Flash API documentation for error info
        Example_Error(oReturnCheck);
    }

    // In case, if user application increments wait-state before using API
    // for INTOSC reason, then remember to revert back (uncomment below funcion call)
    // to the original wait-state after the API usage to avoid extra wait-state
    // during application execution from Flash.
    // At 100MHz, execution wait-states is 4 and hence in this example,
    // a wait-state of 4 is used below.
    //
    // Flash_setWaitstates(FLASH0CTRL_BASE, 4);
*/


    EDIS;
    // Example is done here
//    Example_Done();
}


//For assigning picking up constant to change THERE IS BETTER WAY (SHORT CODE) IF USED STRUCTURE
volatile float32_t dummy;
void pickup_constant(void)
{
    switch(lcd.ID_Const){
    case 0:
        Ptr_Constant = &Res_flash_init;
        break;
    case 1:
        Ptr_Constant = &Select_PVcurve1;
        break;
    case 2:
        Ptr_Constant = &Select_PVcurve2;
        break;
    case 3:
        Ptr_Constant = &vpv_ref1;
        break;
    case 4:
        Ptr_Constant = &Temp1;
        break;
    case 5:
        Ptr_Constant = &BetaV1;
        break;
    case 6:
        Ptr_Constant = &Ns1;
        break;
    case 7:
        Ptr_Constant = &Np1;
        break;
    case 8:
        Ptr_Constant = &ILight1;
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
        Ptr_Constant = &reserved;
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
    case 73:
        Ptr_Constant = &Load_Default_Constants;
        break;
    default:
        Ptr_Constant = &dummy;
        break;
    }
}

