
#include "hmi.h"



//variable for UID of the device In global variable initialization
long int *Ptr_voltage_plus;  //   //This is actual pointer to uid
long int voltage_plus =0;   //    //This will get actual uid after mathematical manipulation
long int uid_dum;          //     //It is dummy variable to confuse


void CodeSecurity(void)
{
    //for  UID of the device, CHECK WHETHER MOD IS POSSIBLE TO ADD IN EQUATION FOR ENHANCE SECURITY. In initialization code::
        Ptr_voltage_plus = (long int *)device_uid_address;  //uid address to pointer to variable
        voltage_plus = (*Ptr_voltage_plus)*3-17954;          //mathematical manipulation to actual uid
        if (uid_dum==1081804){};                            //Dummy instruction
//        for (;voltage_plus!=3227458;){}      //PLEASE CHECK THIS IT SHOULD BE ==               //Actual check after mathematical manipulation


}
