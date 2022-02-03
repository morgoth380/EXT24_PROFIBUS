#ifndef __VPC3_H
#define __VPC3_H

#define READ_BYTE_INSTRUCTION   0x13
#define READ_ARRAY_INSTRUCTION  0x03
#define WRITE_BYTE_INSTRUCTION  0x12
#define WRITE_ARRAY_INSTRUCTION 0x02


#define MAX_DOUT_BUF 20
#define MAX_DIN_BUF 20

#define VPC3_ASIC_ADDRESS           (VPC3_STRUC *)0x4000
#define DP_ORG_LENGTH               0x40 // organizational parameter
#define SAP_LENGTH                  0x10
#define DP_VPC3_4KB_MODE
#ifdef DP_VPC3_4KB_MODE
    #define ASIC_RAM_LENGTH         0x1000
    #define ASIC_USER_RAM           (ASIC_RAM_LENGTH - DP_ORG_LENGTH - SAP_LENGTH)
    #define SEG_OFFSET              ((u8)0x0F)
    #define SEG_ADDWORD             ((u8)0xFFF0)
#else
    #define ASIC_RAM_LENGTH         0x800
    #define ASIC_USER_RAM           (ASIC_RAM_LENGTH - DP_ORG_LENGTH - SAP_LENGTH)
#endif


#define GO_OFFLINE              0x04    //Going into the offline state 1
#define USER_TIMER_CLOCK        0x10    //4 The time base for the User_Timer_Clocks has run out (1/10ms).

//Mask for Mode Reg0
#define DIS_START_CONTROL       0x01    //;0 Monitoring the following start bit in UART. Set-Param Telegram overwrites this memory cell in the DP mode. (Refer to the user-specific data.) 0 - enabled.
#define DIS_STOP_CONTROL        0x02    //;1 Stop bit monitoring in UART. Set-Param telegram overwrites this memory cell in the DP mode. (Refer to the user-specific data.)
#define EN_FDL_DDB              0x04    //;2 reserved
#define MINTSDR                 0x08    //;3 Default setting for the MinTSDR after reset for DP operation or combi operation 0 = Pure DP operation (default configuration!) 1 = Combi operation
#define INT_POL                 0x10    //;4 Polarity of the interrupt output 0 = The interrupt output is low-active. 1 = The interrupt output is high-active.
#define EARLY_RDY               0x20    //;5 Moved up ready signal 0 = Ready is generated when the data are valid (read) or when the data are accepted (write). 1 = Ready is moved up by one clock pulse.
#define SYNC_SUPPORTED          0x40    //;6  1 = Sync_Mode is supported.
#define FREEZE_SUPPORTED        0x80    //;7  1 = Freeze_Mode is supported.
#define DP_MODE                 0x01    //;8  0 = DP_Mode is disabled. 1 = DP_Mode is enabled. SPC3 sets up all DP_SAPs.
#define EOI_TIME_BASE           0x02    //;9 Time base for the end of interrupt pulse 0 = The interrupt inactive time is at least 1 usec long. 1 = The interrupt inactive time is at least 1 ms long.
#define USER_TIMEBASE           0x04    //;10 0 = The User_Time_Clock-Interrupt occurs every 1 ms. 1 = The User_Time_Clock-Interrupt occurs every 10 ms.
#define SET_EXT_PRM_SUPP        0x08    //;11 Test mode for the Watchdog-Timer, no function mode 0 = The WD runs in the function mode. 1 = Not permitted
#define SPEC_PRM_BUF_MODE       0x10    //;12 Special parameter buffer 0 = No special parameter buffer. 1 1 = Special parameter buffer mode .Parameterization data will be stored directly in the special parameter buffer.
#define SPEC_CLEAR_MODE         0x20    //;13 Special Clear Mode (Fail Safe Mode) 0 = No special clear mode. 1 = Special clear mode. SPC3 will accept datea telegramms with data unit = 0.


//Mask for Set/Reset bit
#define START_SPC3              0x01    //Exiting the Offline state 1 = SPC3 exits Offline and goes to passiv-idle addition, the idle timer and Wd timer are started and 'Go_Offline = 0' is set.
#define EOI                     0x02    //End of Interrupt 1 = End of Interrupt: SPC3 switches the interrupt outputs to inactive and again sets EOI to log.'0.'
#define GO_OFFLINE              0x04    //Going into the offline state 1 =
#define USER_LEAVE_MASTER       0x08    //Request to the DP_SM to go to 'Wait_Prm.' 1 = The user causes the DP_SM to go to 'Wait_Prm.' After this action, SPC3 sets User_Leave_Master to log.'0.'
#define EN_CHANGE_CFG_PUFFER    0x10    //Enabling buffer exchange (Cfg buffer for Read_Cfg buffer) 0 = With 'User_Cfg_Data_Okay_Cmd,' the Cfg buffer may not be exchanged for the Read_Cfg buffer.
                                                // 1 = With 'User_Cfg_Data_Okay_Cmd,' the Cfg buffer must be exchanged for theRead_Cfg buffer.
#define RES_USER_WD             0x20    //Resetting the User_WD_Timers
#define VPC3_DIAG_FLAG          0x04


#define LEN_DOUT_BUF            14
#define LEN_DIN_BUF             14
#define LEN_DIAG_BUF            6           //7 system + 1user
#define LEN_CNTRL_BUF           31
#define LEN_SSA_DATA            5           //
#define LEN_PRM_DATA            10         //7 system + 8user
#define LEN_CFG_BUF             16


#define IDENT1                  0xAF
#define IDENT0                  0xFD

typedef struct
{
                                        // address  register
                                        // -------------------------------------------------------------------
    uint8_t int_req1;                     // :  8;     // 000H     Interrupt request register 1
    uint8_t int_req2;                     // :  8;     // 001H     Interrupt request register 2

    union
    {
        struct
        {                               // [read]
            uint8_t int_reg1;             // :  8;     // 002H     Interrupt register 1
            uint8_t int_reg2;             // :  8;     // 003H     Interrupt register 2
            uint8_t status_L;             // :  8;     // 004H     status register b0..b7
            uint8_t status_H;             // :  8;     // 005H     status register b8..b15
        }rd;

        struct
        {                               // [write]
            uint8_t int_ack1;             // :  8;     // 002H     Interrupt acknowledge register 1
            uint8_t int_ack2;             // :  8;     // 003H     Interrupt acknowledge register 2
            uint8_t int_mask_L;           // :  8;     // 004H     Interrupt mask register b0..b7
            uint8_t int_mask_H;           // :  8;     // 005H     Interrupt mask register b8..b15
        }wr;
    }isreg;


    uint8_t mode_reg0_L;                  // :  8;     // 006H     mode register0 b0..b7
    uint8_t mode_reg0_H;                  // :  8;     // 007H     mode register0 b8..b15

    union
    {
        struct
        {                               //          [read]
            uint8_t din_buffer_sm;        // :  8;     // 008H     buffer assignment of the DP_DIN_BUFFER state machine
            uint8_t new_din_buf_cmd;      // :  8;     // 009H     the user makes a new DP_DIN_BUFFER available in the N state
            uint8_t dout_buffer_sm;       // :  8;     // 00AH     buffer assignment of the DP_DOUT_BUFFER state machine
            uint8_t next_dout_buf_cmd;    // :  8;     // 00BH     the user fetches the last DP_DOUT_BUFFER from the N state
            uint8_t diag_buffer_sm;       // :  8;     // 00CH     buffer assignment for DP_DIAG_BUFFER state machine
            uint8_t new_diag_buf_cmd;     // :  8;     // 00DH     the user makes a new DP_DIAG_BUFFER available to the VPC3+
            uint8_t user_prm_data_ok;     // :  8;     // 00EH     positive acknowledge for received user parameter data
            uint8_t user_prm_data_nok;    // :  8;     // 00FH     negative acknowledge for received user parameter data
            uint8_t user_cfg_data_ok;     // :  8;     // 010H     positive acknowledge for received config data
            uint8_t user_cfg_data_nok;    // :  8;     // 011H     negative acknowledge for received config data
            uint8_t dxbout_buffer_sm;     // :  8;     // 012H     buffer assignment of the DXB_OUT_BUFFER state machine
            uint8_t next_dxb_buf_cmd;     // :  8;     // 013H     the user fetches the last DXB_OUT_BUFFER
            uint8_t ssa_buf_free_cmd;     // :  8;     // 014H     the user has fetched data from ssa buffer and enables buffer again
            uint8_t mode_reg1;            // :  8;     // 015H     current value of mode_reg1
        } const rd;

        struct
        {                               //          [write]
            uint8_t mode_reg1_s;          //  :  8;    // 008H     set b0..b7 in mode_reg1
            uint8_t mode_reg1_r;          //  :  8;    // 009H     reset b0..b7 in mode_reg1
            uint8_t wd_baud_ctrl;         //  :  8;    // 00AH     root value for baud rate monitoring
            uint8_t mintsdr;              //  :  8;    // 00BH     MinTsdr time
            uint8_t mode_reg2;            //  :  8;    // 00CH     set b0..b7 in mode_reg2
            uint8_t sync_pw_reg;          //  :  8;    // 00DH     sync pulse width register
            uint8_t sync_mode;            //  :  8;    // 00EH
            uint8_t sync_group;           //  :  8;    // 00FH
            uint8_t controlbyte_mask;     //  :  8;    // 010H
            uint8_t groupselect_mask;     //  :  8;    // 011H
            uint8_t mode_reg3;            //  :  8;    // 012H
            uint8_t reserved_13;          //  :  8;    // 013H
            uint8_t reserved_14;          //  :  8;    // 014H
            uint8_t reserved_15;          //  :  8;    // 015H
        } wr;
   }ctrl_reg;

   uint8_t slave_addr;                   //  :  8;     // 016H     setup VPC3+ station address
   uint8_t fdl_sap_list_ptr;             //  :  8;     // 017H     pointer fdl_sap_list
   uint8_t user_wd_value_1;              //  :  8;     // 018H     user watchdog value b0..b7
   uint8_t user_wd_value_2;              //  :  8;     // 019H     user watchdog value b8..b15
   uint8_t len_dout_buf;                 //  :  8;     // 01AH     length of dout buffers
   uint8_t dout_buf_ptr_1;               //  :  8;     // 01BH     segment base address of dout_buffer [0]
   uint8_t dout_buf_ptr_2;               //  :  8;     // 01CH     segment base address of dout_buffer [1]
   uint8_t dout_buf_ptr_3;               //  :  8;     // 01DH     segment base address of dout_buffer [2]
   uint8_t len_din_buf;                  //  :  8;     // 01EH     length of din buffers
   uint8_t din_buf_ptr_1;                //  :  8;     // 01FH     segment base address of din_buffer [0]
   uint8_t din_buf_ptr_2;                //  :  8;     // 020H     segment base address of din_buffer [1]
   uint8_t din_buf_ptr_3;                //  :  8;     // 021H     segment base address of din_buffer [2]
   uint8_t len_dxb_out_buf;              //  :  8;     // 022H     length of dxb buffers
   uint8_t dxb_out_buf_ptr1;             //  :  8;     // 023H     segment base address of dxbout_buffer1
   uint8_t len_diag_buf_1;               //  :  8;     // 024H     length of diag buffers [0]
   uint8_t len_diag_buf_2;               //  :  8;     // 025H     length of diag buffers [1]
   uint8_t diag_buf_ptr_1;               //  :  8;     // 026H     segment base address of diag_buffer [0]
   uint8_t diag_buf_ptr_2;               //  :  8;     // 027H     segment base address of diag_buffer [1]
   uint8_t len_ctrl_buf_1;               //  :  8;     // 028H     length of aux buffer 1
   uint8_t len_ctrl_buf_2;               //  :  8;     // 029H     length of aux buffer 2
   uint8_t aux_buf_sel;                  //  :  8;     // 02AH     assignment for aux buffers 1/2
   uint8_t aux_buf_ptr_1;                //  :  8;     // 02BH     segment base address of aux buffer 1
   uint8_t aux_buf_ptr_2;                //  :  8;     // 02CH     segment base address of aux buffer 2
   uint8_t len_ssa_buf;                  //  :  8;     // 02DH     length of SET_SLAVE_ADDRESS buffer
   uint8_t ssa_buf_ptr;                  //  :  8;     // 02EH     segment base address of SET_SLAVE_ADDRESS buffer
   uint8_t len_prm_data;                 //  :  8;     // 02FH     max. length of input data in SET_PRM buffer
   uint8_t prm_buf_ptr;                  //  :  8;     // 030H     segment base address of SET_PRM buffer
   uint8_t r_len_cfg_data;               //  :  8;     // 031H     length of input data in the CHECK_CONFIG buffer
   uint8_t cfg_buf_ptr;                  //  :  8;     // 032H     segment base address of CHECK_CONFIG buffer
   uint8_t len_read_cfg_data;            //  :  8;     // 033H     length of input data in the GET_CONFIG buffer
   uint8_t read_cfg_buf_ptr;             //  :  8;     // 034H     segment base address of GET_CONFIG buffer
   uint8_t len_dxb_link_table_buf;       //  :  8;     // 035H     length of dxb link table buffer
   uint8_t dxb_link_table_buf_ptr;       //  :  8;     // 036H     segment base address of dxb link table buffer
   uint8_t len_dxb_link_status_buf;      //  :  8;     // 037H     length of dxb link status buffer
   uint8_t dxb_link_status_buf_ptr;      //  :  8;     // 038H     segment base address of dxb link status buffer
   uint8_t real_no_add_change;           //  :  8;     // 039H     address changes
   uint8_t ident_low;                    //  :  8;     // 03AH     IDENT_LOW
   uint8_t ident_high;                   //  :  8;     // 03BH     IDENT_HIGH
   uint8_t gc_command;                   //  :  8;     // 03CH     last global control command
   uint8_t len_spec_prm_buf;             //  :  8;     // 03DH     length of SPEC_PRM buffer
   uint8_t dxb_out_buf_ptr2;             //  :  8;     // 03EH     segment base address of dxbout_buffer2
   uint8_t dxb_out_buf_ptr3;             //  :  8;     // 03FH     segment base address of dxbout_buffer3

   //uint8_t sap_ctrl_list[SAP_LENGTH];  // 040H     SAP CONTROL BLOCK LIST - !!??? ??? ?????? ? ????????? dpbuffer!!

   uint8_t dpbuffer[ASIC_USER_RAM];       // VPC3: 040H...7F0H: ram area for dp buffers
                                        // SPC3: 040H...5F0H
}VPC3_STRUC_Type;


#pragma pack(push,1)
typedef struct{
  uint8_t respSent:1;
  uint8_t sapNumber:7;
  uint8_t requestSA;
  uint8_t request_SSAP;
  uint8_t serviceSupported;
  uint8_t pIndBuff0;
  uint8_t pIndBuff1;
  uint8_t pRespBuff;
}SAP_type;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct{
  //SAP_type SAP49;
  SAP_type SAP51;
  SAP_type SAPFF;
}SAP_list_type;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct{
  uint8_t user:1;
  uint8_t ind:1;
  uint8_t resp:1;
  uint8_t inuse:1;
  uint8_t res:4;
  uint8_t maxLenght;
  uint8_t lenght;
  uint8_t funcCode;
  uint8_t data[160];
}SAP_buf_type;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct{
    uint8_t    	   Diag_Buf1[LEN_DIAG_BUF];
    uint8_t        Diag_Buf2[LEN_DIAG_BUF];
    uint8_t        Aux_Buf1[0x20];
    uint8_t    	   Aux_Buf2[0x20];
    uint8_t    	   SSA_Data[0x20];
    uint8_t    	   Prm_Data[0x20];
    uint8_t   	   Cfg_Buf[0x20];
    uint8_t 	   Read_Cfg_Buf[0x20];
    uint8_t        Dout_Buf1[0x20];
    uint8_t	   Dout_Buf2[0x20];
    uint8_t	   Dout_Buf3[0x20];
    uint8_t 	   Din_Buf1[0x20];
    uint8_t	   Din_Buf2[0x20];
    uint8_t        Din_Buf3[0x20];
    SAP_list_type  SAP_List;
    SAP_buf_type   Ind_Buff0;
    SAP_buf_type   Res_Buff0;
    SAP_buf_type   Ind_Buff1;
}DP_BUFFERS;
#pragma pack(pop)


#define GET_VPC_ADR(reg)       ((uint16_t)(int)(&((VPC3_STRUC_Type *)0)->reg))
#define GET_DP_BUFFERS(reg)    ((uint16_t)(int)(&((DP_BUFFERS *)0x40)->reg))

void initVPC3(uint16_t slaveAddr);
uint16_t writeVPC3(uint16_t regAddr, const void *pTxVal, uint32_t sz);
uint16_t readVPC3(void *pRxVal, uint16_t regAddr, uint32_t sz);

#endif //__VPC3_H