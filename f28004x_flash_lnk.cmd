
MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to Flash" bootloader mode   */

   BEGIN           	: origin = 0x080000, length = 0x000002
   RAMM0           	: origin = 0x0000F5, length = 0x00030B

   RAMLS2LS3LS4LS5  : origin = 0x009000, length = 0x002000

//TODO below on f2837xd named RAMD0, on f28004x named RAMLS6
   RAMLS6      : origin = 0x00B000, length = 0x000800

   RAMGS0GS1        : origin = 0x00C000, length = 0x002000


   RESET           	: origin = 0x3FFFC0, length = 0x000002

   /* Flash sectors */
   /* BANK 0 */
   FLASH_BANK0_SEC0  : origin = 0x080002, length = 0x000FFE	/* on-chip Flash */
   FLASH_BANK0_SEC1  : origin = 0x081000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC2  : origin = 0x082000, length = 0x001D00	/* on-chip Flash */
   FLASH_BANK0_SEC3  : origin = 0x083D00, length = 0x000300	/* on-chip Flash */
   FLASH_BANK0_SEC4  : origin = 0x084000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC5  : origin = 0x085000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC6  : origin = 0x086000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC7  : origin = 0x087000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC8  : origin = 0x088000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC9  : origin = 0x089000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC10 : origin = 0x08A000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC11 : origin = 0x08B000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC12 : origin = 0x08C000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC13 : origin = 0x08D000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC14 : origin = 0x08E000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK0_SEC15 : origin = 0x08F000, length = 0x001000	/* on-chip Flash */

   /* BANK 1 */
   FLASH_BANK1_SEC0  : origin = 0x090000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC1  : origin = 0x091000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC2  : origin = 0x092000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC3  : origin = 0x093000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC4  : origin = 0x094000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC5  : origin = 0x095000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC6  : origin = 0x096000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC7  : origin = 0x097000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC8_9_10  : origin = 0x098000, length = 0x03000	/* on-chip Flash */
   //FLASH_BANK1_SEC9  : origin = 0x099000, length = 0x001000	/* on-chip Flash */
   //FLASH_BANK1_SEC10 : origin = 0x09A000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC11 : origin = 0x09B000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC12 : origin = 0x09C000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC13 : origin = 0x09D000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC14 : origin = 0x09E000, length = 0x001000	/* on-chip Flash */
   FLASH_BANK1_SEC15 : origin = 0x09F000, length = 0x001000	/* on-chip Flash */

PAGE 1 :

   BOOT_RSVD       : origin = 0x000002, length = 0x0000F3     /* Part of M0, BOOT rom will use this for stack */

   /* on-chip RAM block M1 */
   /* NOTE! Address range 0x760-0x77F must be reserved when using ROM Flash API */
   /*       Address range 0x780-0x7FF must be reserved when using ROM TI-RTOS */
   RAMM1       : origin = 0x000400, length = 0x000360
   RAMM1_RESERVED_ROM_FLASHAPI  : origin = 0x000760, length = 0x000020
   RAMM1_RESERVED_ROM_TIRTOS    : origin = 0x000780, length = 0x000080

   RAMLS0LS1       : origin = 0x008000, length = 0x001000


//   RAMLS5      : origin = 0x00A800, length = 0x000800
//   RAMLS6      : origin = 0x00B000, length = 0x000800
   RAMLS7      : origin = 0x00B800, length = 0x000800

//   RAMGS0      : origin = 0x00C000, length = 0x002000
//   RAMGS1      : origin = 0x00E000, length = 0x002000
   RAMGS2      : origin = 0x010000, length = 0x002000
   RAMGS3      : origin = 0x012000, length = 0x002000
}


SECTIONS
{
   codestart        : > BEGIN,     PAGE = 0, ALIGN(4)
   .text            : >>FLASH_BANK0_SEC1 | FLASH_BANK0_SEC2 | FLASH_BANK0_SEC3,   PAGE = 0, ALIGN(4)
   .cinit           : > FLASH_BANK0_SEC2,     PAGE = 0, ALIGN(4)
   .init_array      : > FLASH_BANK0_SEC1,     PAGE = 0, ALIGN(4)
   .switch          : > FLASH_BANK0_SEC1,     PAGE = 0, ALIGN(4)
   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */

   .stack           : > RAMLS7,     PAGE = 1    //IT WAS RAMM1, PAGE=1
   .data            : > RAMLS6,     PAGE = 0 	//IT WAS RAMM1, PAGE=1

//TODO location of RAMGS3 is different between f2837xd and f28004x
   .bss            : >> RAMGS3      PAGE = 1
   .sysmem         : > RAMGS2       PAGE = 1
   .const          : > FLASH_BANK0_SEC4,    PAGE = 0, ALIGN(4)

//BELOW NEWLY ADDED RUPESH
	CLA1mathTables    : > RAMM1,                 PAGE = 1   //IT WAS RAMLS6,     PAGE = 0

   GROUP
   {
       .TI.ramfunc
       {
       -l sfra_f32_tmu_eabi.lib
       }
       ramfuncs

   } LOAD = FLASH_BANK0_SEC6,
         RUN = RAMGS0GS1,
         LOAD_START(RamfuncsLoadStart),
         LOAD_SIZE(RamfuncsLoadSize),
         LOAD_END(RamfuncsLoadEnd),
         RUN_START(RamfuncsRunStart),
         RUN_SIZE(RamfuncsRunSize),
         RUN_END(RamfuncsRunEnd),
         PAGE = 0, ALIGN(4)

    SFRA_F32_Data		: > RAMGS2, ALIGN = 64, PAGE = 1

    SFRA_Data		: > RAMGS2, ALIGN = 64, PAGE=1

 	FPUmathTables	: > FLASH_BANK1_SEC15, PAGE = 0

   .scratchpad      : > RAMLS0LS1,           PAGE = 1
   .bss_cla         : > RAMLS0LS1,           PAGE = 1
   controlVariables : > RAMLS0LS1,           PAGE = 1

//TODO on f2837xd LOAD = FLASHH :origin = 0x0A0000, while on f28004x FLASH_BANK1_SEC15 : origin = 0x09F000, length = 0x001000
   .const_cla       :  LOAD = FLASH_BANK1_SEC8_9_10,  //
                       RUN = RAMLS2LS3LS4LS5,
                       RUN_START(Cla1ConstRunStart),
                       LOAD_START(Cla1ConstLoadStart),
                       LOAD_SIZE(Cla1ConstLoadSize),
                       PAGE = 0

     GROUP
    {
        isrcodefuncs
        dclfuncs
    }    LOAD = FLASH_BANK1_SEC8_9_10,
         RUN =  RAMLS2LS3LS4LS5,
         LOAD_START(isrcodefuncsLoadStart),
         LOAD_SIZE(isrcodefuncsLoadSize),
         LOAD_END(isrcodefuncsLoadEnd),
         RUN_START(isrcodefuncsRunStart),
         RUN_SIZE(isrcodefuncsRunSize),
         RUN_END(isrcodefuncsRunEnd),
         PAGE = 0, ALIGN(4)

//BELOW NEWLY ADDED RUPESH
   CLA1mathTables    :  LOAD = FLASH_BANK0_SEC14,
                        RUN = RAMLS6,
                        RUN_START(_CLA1mathTablesRunStart),
                        LOAD_START(_CLA1mathTablesLoadStart),
                        LOAD_SIZE(_CLA1mathTablesLoadSize),
                        PAGE = 0

                       /* CLA specific sections */
    Cla1Prog        : LOAD = FLASH_BANK1_SEC8_9_10,
                      RUN = RAMLS2LS3LS4LS5,
                      LOAD_START(Cla1ProgLoadStart),
                      RUN_START(Cla1ProgRunStart),
                      LOAD_SIZE(Cla1ProgLoadSize),
                      PAGE = 0, ALIGN(4)

	DataBufferSection : > RAMM1, PAGE = 1, ALIGN(4)
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
