/*
  Date   : 2009/01/19
  File   : sys_ctrl.v
  Author : Vitaly Rizhevsky

   ----------------------------------------------------------------------------
   This module implements various system control functions. 
   ----------------------------------------------------------------------------
*/

`timescale 1ns / 1ps

`include "../control/argon_defines.v"
`include "../control/datecode.v"

//***********************************Entity Declaration*******************************

  module sys_ctrl (
           
    // System Bus Interface
    input              CLK,                  // system clock 80 MHz     
    input              RESET,                // system reset

    input  [31: 0]     SYS_ADDR_I,           // system address
    input  [ 0: 0]     SYS_CS_n_I,           // system CS
    input              SYS_RNW_I,            // system read/not write
    input  [31: 0]     SYS_DATA_I,           // system write data
    output [31: 0]     SYS_DATA_O,           // system read data
    output             SYS_RDY_O,            // system ready

    // CPU Reset and Interrupts
    input  [31: 0]     NEON_1_INT_STAT_I,    // Neon interrupt status
    input  [31: 0]     NEON_2_INT_STAT_I,    // Neon interrupt status
    input              XENON_INT_STAT_I,     // Xenon interrupt status
    input  [11:0]      DEVICE_TEMP_I,        // device temperature from ADC
    input              RWAVE_INT_I,          // Rwave interrupt
    input  [ 1: 0]     GALIL_INT_I,          // Galil interrupts
    input              FSEQ_INT_I,           // Frame sequencer interrupt
    output             BOARD_RST_O,          // board reset, active high
    output             ARM_HW_RST_O,         // ARM HW reset, active high
    output             ARM_SW_RST_O,         // ARM SW reset, active high
    output             BP_RST_O,             // PPC reset, active high
    output             BP_INT_O,             // BP interrupt, active high
    output             HOST_INT_O,           // Host interrupt, active high

    // Frame sequencer reset
    output             FSEQ_RST_O,           // Microblaze reset, active high

    // Neon and Xenon resets
    output             NEON_RST_N_O,         // Neon reset, active low
    output             XENON_RST_N_O,        // Xenon reset, active low

    // PCI BAR remap registers
    output  reg [31:0] PCIBAR_Remap_0,       // BAR0 remap register
    output  reg [31:0] PCIBAR_Remap_1,       // BAR1 remap register
    output  reg [31:0] PCIBAR_Remap_2,       // BAR2 remap register
    output  reg [31:0] PCIBAR_Remap_3,       // BAR2 remap register

    // Board status
    input              AC_PRESENT_I,         // AC present
    input              SYS_CLK_LOCKED_I,     // system clock locked status

    // FPGA configuration
    output             FPGA_CONFIG_O,        // FPGA config pulse
    output             FPGA_CONFIG_WR_O,     // FPGA config write data
    output [ 2: 0]     FPGA_CONFIG_SEL_O,    // FPGA config select
    output [31: 0]     FPGA_CONFIG_DATA_O,   // FPGA config data
    input              FPGA_CONFIG_RDY_I,    // FPGA config ready status
    input  [ 3: 0]     FPGA_CONFIG_DONE_I,   // FPGA config done status
    input              FPGA_CONFIG_DONE_SEL_I, // FPGA config done status for the selected FPGA

    // Acoustic DMA Interrupt
    input              ADMA_INT_I,           // Acoustic DMA interrupt

    // XMIT control
    output             DEADMAN_SHUTDOWN_O,   // deadman shutdown
    output             XMIT_STOP_O,          // transmit stop
    output             LOAD_LINE_O,          // LOAD_LINE pulse from SW

    // Line timer
    output             LINE_TIMER_EN_O,      // line timer enable
    output             LINE_TIMER_STOP_O,    // line timer stop

    // Line header
    output             LOAD_SETS_O,          // CW load sets
    output  [31:0]     CONTROL_SET_ID_O,     // control set ID
    output  [31:0]     LINE_TYPE_O,          // line type
    output  [31:0]     FRAME_INFO_O,         // frame info
    output  [31:0]     MODE_INFO_O,          // mode info

    // ComDMA Interface
    input              CDMA_BP_HOST_DONE_I,  // ComDMA from BP to Host done    
    input              CDMA_HOST_BP_DONE_I,  // ComDMA from Host to BP done    
    input              GDMA_DONE_I,          // generic dma Host to BP done    
    output  [31:0]     CDMA_HOST_SRC_ADDR_O, // ComDMA host source address     
    output  [31:0]     CDMA_HOST_SRC_LEN_O,  // ComDMA host source length    
    output  [31:0]     CDMA_HOST_DST_ADDR_O, // ComDMA host destination address
    output  [31:0]     CDMA_HOST_DST_LEN_O,  // ComDMA host destination length 
    output  [31:0]     CDMA_BP_SRC_ADDR_O,   // ComDMA BP source address
    output  [31:0]     CDMA_BP_SRC_LEN_O,    // ComDMA BP source length 
    output  [31:0]     CDMA_BP_DST_ADDR_O,   // ComDMA BP destination address  
    output  [31:0]     CDMA_BP_DST_LEN_O,    // ComDMA BP destination length   
    output  [31:0]     GDMA_HOST_SRC_ADDR_O, // generic dma host address
    output  [31:0]     GDMA_HOST_SRC_LEN_O , // generic dma host length
    output  [31:0]     GDMA_BP_DST_ADDR_O,   // generic dma bp destination address
           
    // CoefXferDMA Interface
    input              BP_COEF_DMA_DONE_I,   // DMA from BP to coef done     
    input              COEF_BP_DMA_DONE_I,   // DMA from coef to BP done     
    output  [31:0]     COEF_BP_SRC_ADDR_O,   // coef source address    
    output  [31:0]     COEF_BP_SRC_LEN_O,    // coef source length     
    output  [31:0]     BP_COEF_DST_ADDR_O,   // coef destination address
    output  [31:0]     BP_COEF_DST_LEN_O,    // coef destination length 
    output  [31:0]     BP_COEF_SRC_ADDR_O,   // BP source address
    output  [31:0]     BP_COEF_SRC_LEN_O,    // BP source length 
    output  [31:0]     COEF_BP_DST_ADDR_O,   // BP destination address  
    output  [31:0]     COEF_BP_DST_LEN_O,    // BP destination length   

    // RfcfXferDMA Interface
    input              BP_RFCF_DMA_DONE_I,   // DMA from BP to rfcf done     
    input              RFCF_BP_DMA_DONE_I,   // DMA from rfcf to BP done     
    output  [31:0]     RFCF_BP_SRC_ADDR_O,   // rfcf source address    
    output  [31:0]     RFCF_BP_SRC_LEN_O,    // rfcf source length     
    output  [31:0]     BP_RFCF_DST_ADDR_O,   // rfcf destination address
    output  [31:0]     BP_RFCF_DST_LEN_O,    // rfcf destination length 
    output  [31:0]     BP_RFCF_SRC_ADDR_O,   // BP source address
    output  [31:0]     BP_RFCF_SRC_LEN_O,    // BP source length 
    output  [31:0]     RFCF_BP_DST_ADDR_O,   // BP destination address  
    output  [31:0]     RFCF_BP_DST_LEN_O,    // BP destination length   

    // I2C Interfaces
    inout              DIAG_I2C_CLK_IO,      // I2C clock
    inout              DIAG_I2C_DAT_IO,      // I2C data
    inout              UTIL_I2C_CLK_IO,      // I2C clock
    inout              UTIL_I2C_DAT_IO,      // I2C data
    inout              BATT_I2C_CLK_IO,      // I2C clock
    inout              BATT_I2C_DAT_IO,      // I2C data

    // PIC Resets
    output  [ 4:0]     PIC_RESET_O,          // PIC resets

    // Switching phase select
    output  [ 9:0]     SW_PHASE_MASK_O,      // Switching phase mask
    output  [ 7:0]     SW_PHASE_SYNC_O,      // Switching phase sync
    output  [ 3:0]     SW_PHASE_SPARE_O,     // Switching phase spare
    output             PREG_SW_ON_O,         // Enable PREG switching clocks
    output             CB_CORE_ON_O,         // Turn-on Krypton Core Voltage

    // Motor controller
    input              MOTOR_STEP_I,         // motor step input
    input              MOTOR_DIRECTION_I,    // motor direction input
    input              MOTOR_FAULT_I,        // motor fault input
    input              MOTOR_ACQUIRE_I,      // motor aquire input
    output             MOTOR_RESET_O,        // motor controller reset
    output             MOTOR_MOVE_O,         // motor controller move
    output             MOTOR_CLK_EN_O,       // motor clock enable
    output             MOTOR_PLL_EN_O,       // motor PLL enable

    // DAS control
    output  [31:0]     DAS_BUF_ADDR_O,       // Buffer start address
    output  [ 7:0]     DAS_BUF_SIZE_O,       // Buffer size
    output  [ 7:0]     DAS_SUM_N_O,          // Number of summing samples  
    input   [ 7:0]     DAS_BUF_INDEX_I,      // Current buffer index  

    // GX Monitoring and Control
    output             GX_LINKS_RST_O,       // Disables data from GX links
    output             GX_RX_RST_O,          // Resets GX receivers
    output             GX_RX_HW_RDY_O,       // Indicates GX receivers are ready
    output             GX_TX_RST_O,          // Resets GX transmitters
    output             GX_COMMON_RST_O,      // Resets entire GX block
    output  [ 3:0]     GX_TXDIFFCTRL_O,      // GX TX diff control
    output             GX_TXDEEMPH_O,        // GX TX de-emphasis
    input   [ 9:0]     GX_RX_INIT_DONE_I,    // GX initialization status
    input   [ 9:0]     GX_TX_INIT_DONE_I,    // GX initialization status
    input   [ 9:0]     GX_CRC_ERR_I,         // CRC error indicator
    input   [ 9:0]     GX_CRC_VAL_I,         // CRC valid indicator
    input   [ 9:0]     GX_IDLE_ERR_I,        // IDLE error indicator
    input   [ 9:0]     GX_PLLLKDET_I,        // PLL lock indicator
    input   [ 9:0]     GX_RXLOSSOFSYNC_I,    // Loss of sync indicator
  
    // PCIe Status
    input              PCIE_LED_PULSE_I,     // PCIe LED pulse

    // PREG EC Signals
    output             PREG_EC_BOOT0_O,
    output             PREG_EC_RST_O,
  
    // Manual AML trigger
    output  reg        AML_TRIGGER_O,        // used in AML
  
    // IMB signals 
    inout              iBUTTON_IO,                      // from/to IMB when SVIO not installed in system
    input              PRH_PLL_LOCKED_I,
    output             AIO_USB_HUB_PWR_EN_O,  
    input              ONE_USEC_TICK_I, 
    input              AIO_CLK,   ///IMB TEST
    output  reg [1:0]  AML_TRIGGER_DIS_O,    // '1' disables AML and enables PWL support
    // Debug
    input   [31:0]     SDMA_STATUS_I,        // scan DMA status
    output  [47:0]     DBG_LED_O,            // LEDs
    output  [ 7:0]     TP_CTRL_O,            // controls test points mupliplexers
    //output             SMB_TEST_IO,
    output  [31:0]     MICTOR,
    output  [255:0]    CS_DATA_O             // chipscope
  
); 

//************************************* Parameters ***********************************

    parameter          AML_TRIGGER_LEN = 15'h0064; // 1.2usec pulse

//********************************* Wire Declarations********************************* 

    wire               range_8000_cs;
    wire               range_8000_we;
    wire               range_8000_re;

   // GX monitor (10 channels)
    wire  [ 9:0]       gx_status;
    wire  [ 9:0]       gx_rx_sync;
    wire  [ 9:0]       gx_pll_lock;
    wire  [319:0]      gx_idle_err_cnt;
    wire  [319:0]      gx_crc_err_cnt;
    wire  [319:0]      gx_frame_cnt;

   // I2C interfaces
 //  wire [  7:0] batt_smb_rd_data;
 //  wire [  7:0] batt_smb_rd_ptr;
 //  wire     batt_smb_done;
 //  wire     batt_smb_rd;
 //  wire     batt_smb_wr;
 //  wire     batt_smb_addr_err;
 //  wire     batt_smb_data_err;
 //  wire [255:0] batt_smb_csdata;
   
 //  wire     batt_i2c_clk_out;
//   wire     batt_i2c_clk_in;
 //  wire     batt_i2c_clk_tri;
//   wire     batt_i2c_dat_out;
//   wire     batt_i2c_dat_in;
//   wire     batt_i2c_dat_tri;


    wire  [271:0]      batt_i2c_rd_data;
    wire               batt_i2c_done;
    wire               batt_i2c_addr_err;
    wire               batt_i2c_data_err;
    
    wire               batt_i2c_clk_out;
    wire               batt_i2c_clk_in;
    wire               batt_i2c_clk_tri;
    wire               batt_i2c_dat_out;
    wire               batt_i2c_dat_in;
    wire               batt_i2c_dat_tri;
   
    wire  [271:0]      diag_i2c_rd_data;
    wire               diag_i2c_done;
    wire               diag_i2c_addr_err;
    wire               diag_i2c_data_err;
   
    wire               diag_i2c_clk_out;
    wire               diag_i2c_clk_in;
    wire               diag_i2c_clk_tri;
    wire               diag_i2c_dat_out;
    wire               diag_i2c_dat_in;
    wire               diag_i2c_dat_tri;
    
    wire               diag_i2c_pic_prog_sel;
    wire               diag_i2c_pic_prog_clk;
    wire               diag_i2c_pic_prog_dat;
    wire               diag_i2c_pic_prog_dir;
    
    wire  [271:0]      util_i2c_rd_data;
    wire               util_i2c_done;
    wire               util_i2c_addr_err;
    wire               util_i2c_data_err;
    
    wire               util_i2c_clk_out;
    wire               util_i2c_clk_in;
    wire               util_i2c_clk_tri;
    wire               util_i2c_dat_out;
    wire               util_i2c_dat_in;
    wire               util_i2c_dat_tri;
   
   
   /// for debug only
    wire  [7:0]        smb_i2c_state;
    wire               smb_i2c_Clk0Tck;
    wire               smb_i2c_Clk1Tck;
    wire               smb_i2c_reset;
    wire               smb_i2c_I2cAddrErr;
    wire               smb_i2c_I2cDatErr;
    wire               smb_i2c_I2cGo;
    wire  [7:0]        smb_i2c_I2cLen;
   
   
    wire               diag_i2c_clk_in2;
    wire               diag_i2c_dat_in2;
   
   
    assign MICTOR [7:0]   = smb_i2c_state;  
    assign MICTOR [8]     = smb_i2c_Clk0Tck;  //6
    assign MICTOR [9]     = smb_i2c_Clk1Tck; //7
    assign MICTOR [10]    = smb_i2c_reset;  //8
  // assign MICTOR [9] = smb_i2c_I2cAddrErr;  //9
 //  assign MICTOR [10] = smb_i2c_I2cDatErr;  //10
    assign MICTOR [11]    = smb_i2c_I2cGo;   //11
   //assign MICTOR [19:12] = smb_i2c_I2cLen;  //19
    assign MICTOR [31:12] = 0;
//***************************Internal Register Declarations***************************
   // System bus interface
    reg                rdack, rdack_p1;          // read ack
    reg                wrack, wrack_p1;          // write ack
    reg  [31:0]        data_out;                 // system data output
    reg  [31:0]        reg_out;                  // data readback
   // Date code register
    reg  [31:0]        date_reg;                 // date code register
   // HW Reset
    reg  [31:0]        hw_rst;
   // Reset control registers
    reg[`CB_FE_RST-1:0]fe_rst;           // controls front end resets
    reg  [31:0]        rst_ctl;                  // controls Argon resets
    reg                fseq_por_rst;             // frame sequencer power on reset, controlled by the hw
    reg  [19:0]        fseq_por_cnt;
   // FPGA configuration
    reg  [ 7:0]        fpga_config_ctrl;         // FPGA config control
    reg  [31:0]        fpga_config_data;         // FPGA config data
    reg  [ 2:0]        fpga_config_sel;          // FPGA config select
    reg                fpga_config_wr;           // FPGA config write pulse
   // BP interrupt registers
    reg                bp_int;                   // BP interrupt
    reg  [31:0]        bp_int_en;                // BP interrupt enable
    reg  [31:0]        bp_int_stat;              // BP interrupt status
    reg  [ 3:0]        rwave_p;                  // edge detector
    reg                rwave_int;                // R-wave interrupt
    reg                dummy_int;                // dummy interrupt
    reg                heartbeat_int;            // heartbeat interrupt
    reg  [31:0]        heartbeat_cnt;            // heartbeat counter
    reg  [31:0]        heartbeat_preset;         // heartbeat preset

   // Host interrupt registers
    reg                host_int;                 // host interrupt
    reg                host_int_out;             // host interrupt output
    reg  [31:0]        host_int_en;              // host interrupt enable
    reg  [11:0]        host_int_p;               // host interrupt pipeline
    reg  [31:0]        brd_host_int_stat;        // board host interrupt status

   // XMIT control
    reg                deadman_shutdown;         // deadman shutdown
    reg                deadman_shutdown_p1;      // deadman shutdown pipeline
    reg                deadman_shutdown_en;      // deadman shutdown enable
    reg                deadman_shutdown_int;     // deadman shutdown interrupt
    reg  [31:0]        deadman_shutdown_cnt;     // deadman shutdown counter
    reg                xmit_stop;                // transmit stop
    reg                load_line;                // load_line pulse
    reg                load_line_p1;             // load_line pulse pipeline

   // Line timer register 
    reg  [ 1:0]        line_timer_en;            // line timer enable
   
   // Line timer and ScanDMA control registers  
    reg                load_sets;                // CW load sets
    reg  [31:0]        control_set_id;           // control set ID
    reg  [31:0]        line_type;                // line type
    reg  [31:0]        frame_info;               // frame info
    reg  [31:0]        mode_info;                // mode info
   
   // ComDMA control registers
    reg  [31:0]        cdma_host_src_addr;       // ComDMA host source address   
    reg  [31:0]        cdma_host_src_len;      // ComDMA host source length    
    reg  [31:0]        cdma_host_dst_addr;     // ComDMA host destination address
    reg  [31:0]        cdma_host_dst_len;      // ComDMA host destination length 
    reg  [31:0]        cdma_host_int_stat;       // ComDMA host interrupt status
    reg  [31:0]        gdma_host_int_stat;       // GenericDMA host interrupt status
    reg  [31:0]        cdma_bp_src_addr;     // ComDMA BP source address    
    reg  [31:0]        cdma_bp_src_len;      // ComDMA BP source length     
    reg  [31:0]        cdma_bp_dst_addr;     // ComDMA BP destination address
    reg  [31:0]        cdma_bp_dst_len;      // ComDMA BP destination length  
    reg  [31:0]        cdma_bp_int_stat;         // ComDMA BP interrupt status
    reg  [31:0]        gdma_host_src_addr;       //generic dma source address              
    reg  [31:0]        gdma_host_src_len;        //generic dma source length
    reg  [31:0]        gdma_bp_dst_addr;         //generic dma BP address

   // CoefXferDMA control registers
    reg  [31:0]        coef_bp_src_addr;   
    reg  [31:0]        coef_bp_src_len;    
    reg  [31:0]        bp_coef_dst_addr;
    reg  [31:0]        bp_coef_dst_len; 
    reg  [31:0]        bp_coef_src_addr;    
    reg  [31:0]        bp_coef_src_len;   
    reg  [31:0]        coef_bp_dst_addr;
    reg  [31:0]        coef_bp_dst_len; 
    reg                coef_dma_int_stat;

   // RfcfXferDMA control registers
    reg  [31:0]        rfcf_bp_src_addr;   
    reg  [31:0]        rfcf_bp_src_len;    
    reg  [31:0]        bp_rfcf_dst_addr;
    reg  [31:0]        bp_rfcf_dst_len; 
    reg  [31:0]        bp_rfcf_src_addr;    
    reg  [31:0]        bp_rfcf_src_len;   
    reg  [31:0]        rfcf_bp_dst_addr;
    reg  [31:0]        rfcf_bp_dst_len; 
    reg                rfcf_dma_int_stat;

   // Interrupt generation
    reg                cdma_bp_host_done_p;      // BP-to-host complete
    reg                cdma_host_bp_done_p;      // host-to-BP complete
    reg                gdma_done_p;              // host-to-BP generic dma complete
    reg                coef_dma_done_p;          // coef dma complete
    reg                rfcf_dma_done_p;          // rfcf dma complete

   // GX monitor (10 channels)
    reg  [19:0]        gx_status_clr;
    reg  [19:0]        gx_status_mask;
    reg                gx_int;
    reg  [ 3:0]        gx_txdiffctrl;
    reg                gx_txdeemph;
    reg                gx_rx_rst;
    reg  [15:0]        gx_init_cnt;
    reg                neon_reset_n_p;
    wire               gx_rx_rst_hw;
    reg                gx_rx_hw_ready;
   
   // Battery I2C interface
  // reg [  7:0]  batt_smb_wr_data;
  // reg [  7:0]  batt_smb_wr_ptr;
 //  reg      batt_smb_wr_p;
//   reg      batt_smb_wr_en;
//   reg      batt_smb_rd_p;
//   reg      batt_smb_rd_en;
//   reg [  3:0]  batt_smb_protocol;
//   reg [  6:0]  batt_smb_addr;
//   reg      batt_smb_go;
 //  reg      batt_smb_sem;
 //  reg      batt_smb_reset;
 //  reg [  7:0]  batt_smb_len;
 //  reg [  7:0]  batt_smb_cmd;
 //  reg      batt_smb_addr_err_reg;
//  reg     batt_smb_data_err_reg;
 //  wire     batt_i2c_clk_dg;
 //  wire     batt_i2c_dat_dg; 
 
   // Diagnostics I2C interface
    reg  [127:0]       batt_i2c_data;
    reg  [  6:0]       batt_i2c_addr;
    reg  [  7:0]       batt_i2c_len;
    reg  [  8:0]       batt_i2c_mutex;
    reg                batt_i2c_rdwr;
    reg                batt_i2c_start;
    reg                batt_i2c_stop;
    reg                batt_i2c_rdnack;
    reg                batt_i2c_go;
    reg                batt_i2c_sem;
    reg                batt_i2c_addr_err_reg;
    reg                batt_i2c_data_err_reg;
    
    wire               batt_i2c_clk_dg;
    wire               batt_i2c_dat_dg; 
    
    reg                batt_i2c_reset;
    reg                preg_ec_reset;
    reg                preg_ec_boot0; 
    reg                debug_force_stop_invert;
    reg                debug_force_start_invert;
   

   // Diagnostics I2C interface
    reg  [127:0]       diag_i2c_data;
    reg  [  6:0]       diag_i2c_addr;
    reg  [  7:0]       diag_i2c_len;
    reg                diag_i2c_rdwr;
    reg                diag_i2c_start;
    reg                diag_i2c_stop;
    reg                diag_i2c_rdnack;
    reg                diag_i2c_go;
    reg                diag_i2c_sem;
    reg  [  8:0]       diag_i2c_mutex;                            
    reg                diag_i2c_addr_err_reg;
    reg                diag_i2c_data_err_reg;
    reg  [  8:0]       diag_i2c_pic_prog;
    wire               diag_i2c_clk_dg;
    wire               diag_i2c_dat_dg;
                                                
   // Utility I2C interface
    reg  [127:0]       util_i2c_data;
    reg  [  6:0]       util_i2c_addr;
    reg  [  7:0]       util_i2c_len;
    reg                util_i2c_rdwr;
    reg                util_i2c_start;
    reg                util_i2c_stop;
    reg                util_i2c_rdnack;
    reg                util_i2c_go;
    reg                util_i2c_sem;
    reg  [  8:0]       util_i2c_mutex;                            
    reg                util_i2c_addr_err_reg;
    reg                util_i2c_data_err_reg;
    wire               util_i2c_clk_dg;
    wire               util_i2c_dat_dg;
                             
   // motor controller
    reg  [ `CB_MOTOR_CONTROL-1:0]      motor_control;
    reg  [ `CB_MOTOR_POS-1:0]          motor_pos;
    reg  [ `CB_MOTOR_POS_CONTROL-1:0]  motor_pos_control;
    reg  [ `CB_MOTOR_TRIG_POS_REV-1:0] motor_trig_pos_rev;
    reg  [ 4:0]        motor_step_p;
    reg  [ 4:0]        motor_dir_p;
    reg  [ 4:0]        motor_fault_p;
    reg  [ 4:0]        motor_acquire_p;
    reg                motor_count_dir;
   
   // DAS control
    reg  [ `CB_DAS_BUF_ADDR-1:0]  das_buf_addr;
    reg  [ `CB_DAS_BUF_SIZE-1:0]  das_buf_size;
    reg  [ `CB_DAS_SUM_N-1:0]     das_sum_n;
    reg  [ `CB_DAS_BUF_INDEX-1:0] das_buf_index;
   
   // switching phase select
    reg [ `CB_SW_PHASE_SEL-1:0]   sw_phase_sel;
    reg [ `CB_FE_SLEEP_CTRL-1:0]  fe_sleep_ctrl; 

   // Debug
    reg  [ 3:0]        bp_led;
    reg  [ 3:0]        pcie_led;
    reg  [19:0]        hw_led;
    reg                hw_led_dis;   
    reg  [31:0]        pcie_status;
    reg  [ 7:0]        tp_ctrl;

   // AC present detection
    reg                ac_present_p;   

   // scan DMA debug
    reg  [ 2:0]        sdma_par_err;   
    reg  [11:0]        sdma_par_fifo_count;   
    reg  [ 2:0]        sdma_ser_err;   
    reg  [11:0]        sdma_ser_fifo_count;   

   // manual AML trigger (for CW)
    reg  [27: 0]       aml_pulse_cnt;      // AML pulse counter
   
   // IMB signals
    reg  [19:0]        hub_reset_active_cnt;
    reg  [19:0]        aio_usb_hub_pwr_en_tc;
    reg                aio_usb_hub_pwr_en;
    reg                sw_aio_usb_hub_pwr_dis;
    reg                sw_aio_reset_pulse;
    reg  [2:0]         ten_mhz_cnt;
    wire               ibutton_restart_sw;
    reg  [15:0]        ibutton_restart_sr;
    wire [4:0]         ibutton_status;
    wire [63:0]        ibutton_data;

   // device temp resyn registers
    reg  [11:0]        device_temp[1:0];  
   
//*********************************Main Body of Code**********************************
    assign SYS_RDY_O            =  rdack_p1 | wrack_p1;
    assign SYS_DATA_O           =  data_out;
							       
    assign NEON_RST_N_O         =  fe_rst[`CB_RST_NEON];
    assign XENON_RST_N_O        =  fe_rst[`CB_RST_XENON];
    assign FSEQ_RST_O           =  rst_ctl[`CB_RST_FSEQ] | fseq_por_rst;
    assign GX_LINKS_RST_O       =  rst_ctl[`CB_RST_GX_LINKS];
    assign GX_TX_RST_O          =  rst_ctl[`CB_RST_GX_TX];
    assign GX_RX_RST_O          =  rst_ctl[`CB_RST_GX_RX] | gx_rx_rst_hw;
    assign GX_RX_HW_RDY_O       =  gx_rx_hw_ready;
    assign GX_COMMON_RST_O      =  rst_ctl[`CB_RST_GX_COMMON];
    assign BOARD_RST_O          =  rst_ctl[`CB_RST_BOARD];
    assign ARM_HW_RST_O         =  rst_ctl[`CB_RST_ARM_HW];
    assign ARM_SW_RST_O         =  rst_ctl[`CB_RST_ARM_SW];
    assign BP_RST_O             =  rst_ctl[`CB_RST_PPC];
    assign GX_TXDIFFCTRL_O      =  gx_txdiffctrl;
    assign GX_TXDEEMPH_O        =  gx_txdeemph;
						        
    assign BP_INT_O             =  bp_int;
    assign HOST_INT_O           =  host_int_out;
							       
    assign PIC_RESET_O          =  ~diag_i2c_pic_prog[8:4]; // PIC resets are active low
							    
    assign SW_PHASE_MASK_O      =  sw_phase_sel[`CB_SW_PHASE_MASK];
    assign SW_PHASE_SYNC_O      =  sw_phase_sel[`CB_SW_PHASE_SYNC];
    assign SW_PHASE_SPARE_O     =  sw_phase_sel[`CB_SW_PHASE_SPARE];
							 	  
    assign CB_CORE_ON_O         =  fe_sleep_ctrl[0];
    assign PREG_SW_ON_O         =  fe_sleep_ctrl[1];
							    
    assign MOTOR_RESET_O        =  motor_control[`CB_MOTOR_RESET];
    assign MOTOR_MOVE_O         =  motor_control[`CB_MOTOR_MOVE];
    assign MOTOR_CLK_EN_O       =  motor_control[`CB_MOTOR_CLK_EN];
    assign MOTOR_PLL_EN_O       =  motor_control[`CB_MOTOR_PLL_EN];
							    
    assign DAS_BUF_ADDR_O       =  das_buf_addr;
    assign DAS_BUF_SIZE_O       =  das_buf_size;
    assign DAS_SUM_N_O          =  das_sum_n;
							    
    assign DEADMAN_SHUTDOWN_O   =  deadman_shutdown;   
    assign XMIT_STOP_O          =  xmit_stop;
    assign LOAD_LINE_O          =  load_line & ~load_line_p1;
    assign LOAD_SETS_O          =  load_sets;
    assign CONTROL_SET_ID_O     =  control_set_id;
    assign LINE_TYPE_O          =  line_type;
    assign FRAME_INFO_O         =  frame_info;
    assign MODE_INFO_O          =  mode_info;
    assign LINE_TIMER_EN_O      =  line_timer_en[`CB_LINE_TIMER_EN];
    assign LINE_TIMER_STOP_O    =  line_timer_en[`CB_LINE_TIMER_STOP];
							    
    assign FPGA_CONFIG_O        =  fpga_config_ctrl[0];
    assign FPGA_CONFIG_DATA_O   =  fpga_config_data;
    assign FPGA_CONFIG_WR_O     =  fpga_config_wr;
    assign FPGA_CONFIG_SEL_O    =  fpga_config_sel;
    
    assign CDMA_HOST_SRC_ADDR_O =  cdma_host_src_addr;
    assign CDMA_HOST_SRC_LEN_O  =  cdma_host_src_len;
    assign CDMA_HOST_DST_ADDR_O =  cdma_host_dst_addr;
    assign CDMA_HOST_DST_LEN_O  =  cdma_host_dst_len;
    assign CDMA_BP_SRC_ADDR_O   =  cdma_bp_src_addr;
    assign CDMA_BP_SRC_LEN_O    =  cdma_bp_src_len;
    assign CDMA_BP_DST_ADDR_O   =  cdma_bp_dst_addr;
    assign CDMA_BP_DST_LEN_O    =  cdma_bp_dst_len;
								   
    assign GDMA_HOST_SRC_ADDR_O =  gdma_host_src_addr;
    assign GDMA_HOST_SRC_LEN_O  =  gdma_host_src_len;
    assign GDMA_BP_DST_ADDR_O   =  gdma_bp_dst_addr;
								   
    assign COEF_BP_SRC_ADDR_O   =  coef_bp_src_addr;
    assign COEF_BP_SRC_LEN_O    =  coef_bp_src_len;
    assign BP_COEF_DST_ADDR_O   =  bp_coef_dst_addr;
    assign BP_COEF_DST_LEN_O    =  bp_coef_dst_len;
    assign BP_COEF_SRC_ADDR_O   =  bp_coef_src_addr;
    assign BP_COEF_SRC_LEN_O    =  bp_coef_src_len;
    assign COEF_BP_DST_ADDR_O   =  coef_bp_dst_addr;
    assign COEF_BP_DST_LEN_O    =  coef_bp_dst_len;
								   
    assign RFCF_BP_SRC_ADDR_O   =  rfcf_bp_src_addr;
    assign RFCF_BP_SRC_LEN_O    =  rfcf_bp_src_len;
    assign BP_RFCF_DST_ADDR_O   =  bp_rfcf_dst_addr;
    assign BP_RFCF_DST_LEN_O    =  bp_rfcf_dst_len;
    assign BP_RFCF_SRC_ADDR_O   =  bp_rfcf_src_addr;
    assign BP_RFCF_SRC_LEN_O    =  bp_rfcf_src_len;
    assign RFCF_BP_DST_ADDR_O   =  rfcf_bp_dst_addr;
    assign RFCF_BP_DST_LEN_O    =  rfcf_bp_dst_len;
    
    assign TP_CTRL_O            =  tp_ctrl;
								   
    assign PREG_EC_RST_O        =  preg_ec_reset;
    assign PREG_EC_BOOT0_O      =  preg_ec_boot0;
   
  
    assign DBG_LED_O = {
                        fe_sleep_ctrl[1],
                        {11'b0},
                        {~pcie_led[3],1'b0},
                        {~pcie_led[2],1'b0},
                        {~pcie_led[1],1'b0},
                        {~pcie_led[0],1'b0},
                        {~bp_led[3],1'b0},
                        {~bp_led[2],1'b0},
                        {~bp_led[1],1'b0},
                        {~bp_led[0],1'b0},
                        {gx_status[9], ~gx_status[9]}, // misc link from Neon 1
                        {gx_status[9], ~gx_status[9]}, // gx_status[8] misc link from Neon 0 // not used on protego ---not routed on the ACB.  duplicate MISC 1 status so LED works correclty without changing ROLIS.
                        {gx_status[7], gx_status[7]}, // RF data lane 7 from Neon 1  // not used on protego
                        {gx_status[6], ~gx_status[6]}, // RF data lane 6 from Neon 1
                        {gx_status[5], ~gx_status[5]}, // RF data lane 5 from Neon 1
                        {gx_status[4], ~gx_status[4]}, // RF data lane 4 from Neon 1
                        {gx_status[3], ~gx_status[3]}, // RF data lane 3 from Neon 1
                        {gx_status[2], ~gx_status[2]}, // RF data lane 2 from Neon 1
                        {gx_status[1], ~gx_status[1]}, // RF data lane 1 from Neon 1
                        {gx_status[0], ~gx_status[0]}  // RF data lane 0 from Neon 1
                       };

   // Acoustic DMA (0x8080 0400 - 0x8080 0ffc)
   // Frame sequencer (0x8080 1000 - 0x8080 1ffc)
   // Neon utilities (0x8080 2000 - 0x8080 9ffc)
   // Xenon utilities (0x8080 a000 - 0x8080 fffc)
   // Flash interface (0x8e00 0000 - 0x8fff fffc)
   // have their own decoding, exclude here 
   assign range_8000_cs = ~SYS_CS_n_I[0] & (SYS_ADDR_I[31:28] == `CA_RANGE_8000) & 
                          ~(SYS_ADDR_I[31: 8] >= `ADMA_LO_ADDR && SYS_ADDR_I[31: 8] <= `ADMA_HI_ADDR) &
                          ~(SYS_ADDR_I[31: 8] >= `CA_FSEQ_LO_ADDR && SYS_ADDR_I[31: 8] <= `CA_FSEQ_HI_ADDR) &
                          ~(SYS_ADDR_I[31: 8] >= `CA_MOT_UART_LO_ADDR && SYS_ADDR_I[31: 8] <= `CA_MOT_UART_HI_ADDR) &
                          ~(SYS_ADDR_I[31: 8] >= `CA_NEON_UTIL_LO_ADDR && SYS_ADDR_I[31: 8] <= `CA_NEON_UTIL_HI_ADDR);
   assign range_8000_we = range_8000_cs & ~SYS_RNW_I;
   assign range_8000_re = range_8000_cs &  SYS_RNW_I;

   assign diag_i2c_pic_prog_sel = diag_i2c_pic_prog[`CB_PIC_PROG_SEL];
   assign diag_i2c_pic_prog_clk = diag_i2c_pic_prog[`CB_PIC_PROG_CLK];
   assign diag_i2c_pic_prog_dat = diag_i2c_pic_prog[`CB_PIC_PROG_DAT];
   assign diag_i2c_pic_prog_dir = diag_i2c_pic_prog[`CB_PIC_PROG_DIR];
    
   // Data readback and acknowledge generation
    always @(posedge CLK) begin
    if (RESET) begin
        rdack          <=  1'b0;
        rdack_p1       <=  1'b0;
        wrack          <=  1'b0;
        wrack_p1       <=  1'b0;
        data_out       <=  0;
        reg_out        <=  0;
        date_reg       <=  `CD_DATECODE;
        device_temp[0] <=  0;     
        device_temp[1] <=  0;     
    end
    else begin
        // synchronize the DEVICE_TEMP_I input to the CLK domain.
        device_temp[0] <=  DEVICE_TEMP_I;     
        device_temp[1] <=  device_temp[0];      
        rdack_p1       <=  rdack;
        wrack_p1       <=  wrack;
        data_out       <=  rdack ? reg_out : 0;
      // generate read/write acknowledge
        if (range_8000_cs & SYS_RNW_I) begin
            rdack      <=  1'b1;
            wrack      <=  1'b0;
        end
        else if (range_8000_cs & ~SYS_RNW_I) begin
            rdack      <=  1'b0;
            wrack      <=  1'b1;
        end
        else begin
            rdack      <=  1'b0;
            wrack      <=  1'b0;
        end
        reg_out        <=  0;

      // readback internal registers
        if (range_8000_cs) begin
        casez (SYS_ADDR_I[23:0])
            `CA_PPC_HEARTBEAT      : reg_out                     <=  heartbeat_preset;
            `CA_XMIT_STOP          : reg_out[`CB_XMIT_STOP]      <=  xmit_stop;
            `CA_FE_RST             : reg_out[`CB_FE_RST-1:0]     <=  fe_rst;
            `CA_BP_INT_EN          : reg_out                     <=  bp_int_en;
            `CA_BP_INT_STAT        : reg_out                     <=  bp_int_stat;
            `CA_AML_TRIG_DIS       : reg_out[`CB_AML_TRIG_DIS]   <=  AML_TRIGGER_DIS_O;
            `CA_BP_BRD_STAT        : begin                           
                                     reg_out[`CB_AC_PRESENT]     <=  AC_PRESENT_I;
                                     reg_out[`CB_SYS_CLK_LOCKED] <=  SYS_CLK_LOCKED_I;
                                    end
            `CA_BP_INT_STAT_NEON_1 : reg_out                     <=  NEON_1_INT_STAT_I;
            `CA_BP_INT_STAT_NEON_2 : reg_out                     <=  NEON_2_INT_STAT_I;
            `CA_BP_INT_STAT_XENON  : reg_out                     <=  {XENON_INT_STAT_I,17'b0};
            `CA_DEADMAN_CNT_EN     : reg_out[`CB_DEADMAN_CNT_EN] <=  deadman_shutdown_en;
            `CA_CONTROL_SET_ID     : reg_out                     <=  control_set_id;
            `CA_LINE_TYPE          : reg_out                     <=  line_type;
            `CA_FRAME_INFO         : reg_out                     <=  frame_info;
            `CA_MODE_INFO          : reg_out                     <=  mode_info;
            `CA_LINE_TIMER_EN      : begin
                                     reg_out[`CB_LINE_TIMER_EN]  <=  line_timer_en[`CB_LINE_TIMER_EN];
                                     reg_out[`CB_LINE_TIMER_STOP]<=  line_timer_en[`CB_LINE_TIMER_STOP];
                                    end
            `CA_LOAD_LINE          : reg_out[`CB_LOAD_LINE]      <=  load_line;
            // rearrange bits for SW compability with Argon
            `CA_GX_STATUS          : reg_out                     <=  {12'b0, gx_status[7:0], 10'b0, gx_status[9:8]};
            `CA_GX_MASK            : reg_out                     <=  {12'b0, gx_status_mask};
            `CA_GX_RXSYNC          : reg_out                     <=  {12'b0, gx_rx_sync[7:0], 10'b0, gx_rx_sync[9:8]};
            `CA_GX_PLLLOCK         : reg_out                     <=  {12'b0, gx_pll_lock[7:0], 10'b0, gx_pll_lock[9:8]};
            //
            `CA_GX_TXDIFFCTRL      : reg_out                     <=  {27'b0, gx_txdeemph, gx_txdiffctrl};
            `CA_GX_INIT_STATUS     : reg_out                     <=  {6'b0, GX_TX_INIT_DONE_I, 6'b0, GX_RX_INIT_DONE_I};
            `CA_GX_RESET_STATUS    : reg_out                     <=  {31'b0, gx_rx_hw_ready};
            // rearrange address for SW compability with Argon
            // Frame Count
            `CA_GX_FRAME_CNT_W2    :  reg_out                    <=  gx_frame_cnt[ 31:  0];        // RFdata0
            `CA_GX_FRAME_CNT_W3    :  reg_out                    <=  gx_frame_cnt[ 63: 32];        // RFdata1
            `CA_GX_FRAME_CNT_W4    :  reg_out                    <=  gx_frame_cnt[ 95: 64];        // RFdata2
            `CA_GX_FRAME_CNT_W5    :  reg_out                    <=  gx_frame_cnt[127: 96];        // RFdata3
            `CA_GX_FRAME_CNT_W6    :  reg_out                    <=  gx_frame_cnt[159:128];        // RFdata4
            `CA_GX_FRAME_CNT_W7    :  reg_out                    <=  gx_frame_cnt[191:160];        // RFdata5
            `CA_GX_FRAME_CNT_W8    :  reg_out                    <=  gx_frame_cnt[223:192];        // RFdata6
            `CA_GX_FRAME_CNT_W9    :  reg_out                    <=  gx_frame_cnt[255:224];        // RFdata7
            `CA_GX_FRAME_CNT_W0    :  reg_out                    <=  gx_frame_cnt[287:256];        // Neon 1
            `CA_GX_FRAME_CNT_W1    :  reg_out                    <=  gx_frame_cnt[319:288];        // Neon 0
            // CRC Error Count
            `CA_GX_CRC_ERR_CNT_W2  :  reg_out                    <=  gx_crc_err_cnt[ 31:  0];    // RFdata0
            `CA_GX_CRC_ERR_CNT_W3  :  reg_out                    <=  gx_crc_err_cnt[ 63: 32];    // RFdata1
            `CA_GX_CRC_ERR_CNT_W4  :  reg_out                    <=  gx_crc_err_cnt[ 95: 64];    // RFdata2
            `CA_GX_CRC_ERR_CNT_W5  :  reg_out                    <=  gx_crc_err_cnt[127: 96];    // RFdata3
            `CA_GX_CRC_ERR_CNT_W6  :  reg_out                    <=  gx_crc_err_cnt[159:128];    // RFdata4
            `CA_GX_CRC_ERR_CNT_W7  :  reg_out                    <=  gx_crc_err_cnt[191:160];    // RFdata5
            `CA_GX_CRC_ERR_CNT_W8  :  reg_out                    <=  gx_crc_err_cnt[223:192];    // RFdata6
            `CA_GX_CRC_ERR_CNT_W9  :  reg_out                    <=  gx_crc_err_cnt[255:224];    // RFdata7
            `CA_GX_CRC_ERR_CNT_W0  :  reg_out                    <=  gx_crc_err_cnt[287:256];    // Neon 1 
            `CA_GX_CRC_ERR_CNT_W1  :  reg_out                    <=  gx_crc_err_cnt[319:288];    // Neon 0 
            // Idle Error Count
            `CA_GX_IDLE_ERR_CNT_W2 :  reg_out                    <=  gx_idle_err_cnt[ 31:  0];  // RFdata0
            `CA_GX_IDLE_ERR_CNT_W3 :  reg_out                    <=  gx_idle_err_cnt[ 63: 32];  // RFdata1
            `CA_GX_IDLE_ERR_CNT_W4 :  reg_out                    <=  gx_idle_err_cnt[ 95: 64];  // RFdata2
            `CA_GX_IDLE_ERR_CNT_W5 :  reg_out                    <=  gx_idle_err_cnt[127: 96];  // RFdata3
            `CA_GX_IDLE_ERR_CNT_W6 :  reg_out                    <=  gx_idle_err_cnt[159:128];  // RFdata4
            `CA_GX_IDLE_ERR_CNT_W7 :  reg_out                    <=  gx_idle_err_cnt[191:160];  // RFdata5
            `CA_GX_IDLE_ERR_CNT_W8 :  reg_out                    <=  gx_idle_err_cnt[223:192];  // RFdata6
            `CA_GX_IDLE_ERR_CNT_W9 :  reg_out                    <=  gx_idle_err_cnt[255:224];  // RFdata7
            `CA_GX_IDLE_ERR_CNT_W0 :  reg_out                    <=  gx_idle_err_cnt[287:256];  // Neon 1 
            `CA_GX_IDLE_ERR_CNT_W1 :  reg_out                    <=  gx_idle_err_cnt[319:288];  // Neon 0 
            `CA_DATE               :  reg_out                    <=  date_reg;
            `CA_TP_CTRL            :  reg_out                    <=  tp_ctrl;
            `CA_HOST_INT_EN        :  reg_out                    <=  host_int_en;
            `CA_BAR_REMAP_0        :  reg_out                    <=  PCIBAR_Remap_0;
            `CA_BAR_REMAP_1        :  reg_out                    <=  PCIBAR_Remap_1;
            `CA_BAR_REMAP_2        :  reg_out                    <=  PCIBAR_Remap_2;
            `CA_BAR_REMAP_3        :  reg_out                    <=  PCIBAR_Remap_3;
            `CA_RST_CTL            :  reg_out                    <=  rst_ctl;
            `CA_HW_RST             :  reg_out                    <=  hw_rst;

            `CA_BRD_STAT           : begin
                                     reg_out[`CB_AC_PRESENT]     <=  AC_PRESENT_I;
                                     reg_out[`CB_SYS_CLK_LOCKED] <=  SYS_CLK_LOCKED_I;
                                   end
            `CA_BRD_INT_STAT       : reg_out                     <=  brd_host_int_stat;
            `CA_PIC_PROG           : begin
                                     reg_out                     <=  diag_i2c_pic_prog;
                                      // PIC programming data is read from the pin 
                                     reg_out[`CB_PIC_PROG_DAT]   <=  diag_i2c_dat_in;
                                   end
            `CA_DIAG_I2C_DAT_W0    : reg_out                     <=  diag_i2c_data[ 31:  0];
            `CA_DIAG_I2C_DAT_W1    : reg_out                     <=  diag_i2c_data[ 63: 32];
            `CA_DIAG_I2C_DAT_W2    : reg_out                     <=  diag_i2c_data[ 95: 64];
            `CA_DIAG_I2C_DAT_W3    : reg_out                     <=  diag_i2c_data[127: 96];
            `CA_DIAG_I2C_MUTEX     : reg_out                     <=  {23'h00_0000,diag_i2c_mutex[8: 0]};                                                                    
            `CA_DIAG_I2C_CTL       : begin                           
                                     reg_out[`CB_I2C_RDWR]       <=  diag_i2c_rdwr;
                                     reg_out[`CB_I2C_ADDR]       <=  diag_i2c_addr;
                                     reg_out[`CB_I2C_LEN]        <=  diag_i2c_len;
                                     reg_out[`CB_I2C_START]      <=  diag_i2c_start;
                                     reg_out[`CB_I2C_STOP]       <=  diag_i2c_stop;
                                     reg_out[`CB_I2C_RDNACK]     <=  diag_i2c_rdnack;
                                     reg_out[`CB_I2C_GO]         <=  diag_i2c_go;
                                     reg_out[`CB_I2C_SEM]        <=  diag_i2c_sem;
                                     reg_out[`CB_I2C_ADDR_ERR]   <=  diag_i2c_addr_err_reg;
                                     reg_out[`CB_I2C_DATA_ERR]   <=  diag_i2c_data_err_reg;
                                    end
            `CA_UTIL_I2C_DAT_W0    : reg_out                     <=  util_i2c_data[ 31:  0];
            `CA_UTIL_I2C_DAT_W1    : reg_out                     <=  util_i2c_data[ 63: 32];
            `CA_UTIL_I2C_DAT_W2    : reg_out                     <=  util_i2c_data[ 95: 64];
            `CA_UTIL_I2C_DAT_W3    : reg_out                     <=  util_i2c_data[127: 96];
            `CA_UTIL_I2C_MUTEX     : reg_out                     <=  {23'h00_0000,util_i2c_mutex[8: 0]};                                                                    
            `CA_UTIL_I2C_CTL       : begin
                                     reg_out[`CB_I2C_RDWR]       <=  util_i2c_rdwr;
                                     reg_out[`CB_I2C_ADDR]       <=  util_i2c_addr;
                                     reg_out[`CB_I2C_LEN]        <=  util_i2c_len;
                                     reg_out[`CB_I2C_START]      <=  util_i2c_start;
                                     reg_out[`CB_I2C_STOP]       <=  util_i2c_stop;
                                     reg_out[`CB_I2C_RDNACK]     <=  util_i2c_rdnack;
                                     reg_out[`CB_I2C_GO]         <=  util_i2c_go;
                                     reg_out[`CB_I2C_SEM]        <=  util_i2c_sem;
                                     reg_out[`CB_I2C_ADDR_ERR]   <=  util_i2c_addr_err_reg;
                                     reg_out[`CB_I2C_DATA_ERR]   <=  util_i2c_data_err_reg;
                                    end
                    
         //`CA_BATT_SMB_CTL       : begin
                //                          reg_out[`CB_SMB_PROTOCOL]  <= batt_smb_protocol;
                 //                         reg_out[`CB_SMB_ADDR]      <= batt_smb_addr;
                  //                        reg_out[`CB_SMB_GO]        <= batt_smb_go;
                   //                       reg_out[`CB_SMB_SEM]       <= batt_smb_sem;
                    //                      reg_out[`CB_SMB_ADDR_ERR]  <= batt_smb_addr_err_reg;
                     //                     reg_out[`CB_SMB_DATA_ERR]  <= batt_smb_data_err_reg;
                      //                    reg_out[`CB_SMB_RESET]     <= batt_smb_reset;
                       //                   reg_out[`CB_SMB_LEN]       <= batt_smb_len;
                        //                  reg_out[`CB_SMB_CMD]       <= batt_smb_cmd;
                   //               end
            `CA_BATT_I2C_DAT_W0    : reg_out                     <=  batt_i2c_data[ 31:  0];
            `CA_BATT_I2C_DAT_W1    : reg_out                     <=  batt_i2c_data[ 63: 32];
            `CA_BATT_I2C_DAT_W2    : reg_out                     <=  batt_i2c_data[ 95: 64];
            `CA_BATT_I2C_DAT_W3    : reg_out                     <=  batt_i2c_data[127: 96];  
            `CA_BATT_I2C_CTL       : begin
                                     reg_out[`CB_I2C_RDWR]       <=  batt_i2c_rdwr;
                                     reg_out[`CB_I2C_ADDR]       <=  batt_i2c_addr;
                                     reg_out[`CB_I2C_LEN]        <=  batt_i2c_len;
                                     reg_out[`CB_I2C_START]      <=  batt_i2c_start;
                                     reg_out[`CB_I2C_STOP]       <=  batt_i2c_stop;
                                     reg_out[`CB_I2C_RDNACK]     <=  batt_i2c_rdnack;
                                     reg_out[`CB_I2C_GO]         <=  batt_i2c_go;
                                     reg_out[`CB_I2C_SEM]        <=  batt_i2c_sem;
                                     reg_out[`CB_I2C_ADDR_ERR]   <=  batt_i2c_addr_err_reg;
                                     reg_out[`CB_I2C_DATA_ERR]   <=  batt_i2c_data_err_reg;
                                    end                              
           `CA_BATT_I2C_RESET      : reg_out                     <=  batt_i2c_reset;
           `CA_BATT_I2C_MUTEX      : reg_out                     <=  {23'h00_0000, batt_i2c_mutex[8: 0]};
             
        // `CA_BATT_SMB_DAT       : begin 
                                     //     reg_out[`CB_SMB_DAT]       <= batt_smb_rd_data;
                                     //     reg_out[`CB_SMB_PTR]       <= batt_smb_rd_ptr;
                                     //   end
            `CA_DAS_BUF_ADDR       : reg_out[`CB_DAS_BUF_ADDR - 1:0]   <=  das_buf_addr;
            `CA_DAS_BUF_SIZE       : reg_out[`CB_DAS_BUF_SIZE - 1:0]   <=  das_buf_size;
            `CA_DAS_SUM_N          : reg_out[`CB_DAS_SUM_N - 1:0]      <=  das_sum_n;
            `CA_DAS_BUF_INDEX      : reg_out[`CB_DAS_BUF_INDEX - 1:0]  <=  DAS_BUF_INDEX_I;
            `CA_MOTOR_CONTROL      : reg_out  <=  motor_control;
            `CA_MOTOR_POS          : reg_out  <=  motor_pos;
            `CA_MOTOR_POS_CONTROL  : reg_out  <=  motor_pos_control;
            `CA_MOTOR_TRIG_POS_REV : reg_out  <=  motor_trig_pos_rev;
            `CA_DEVICE_TEMP        : reg_out  <=  {20'b0, device_temp[1]};
            `CA_PCIE_STATUS        : reg_out  <=  pcie_status;
            `CA_SW_PHASE_SEL       : reg_out  <=  sw_phase_sel;
            `CA_SDMA_DEBUG         : reg_out  <=  {SDMA_STATUS_I[31], sdma_ser_err, sdma_ser_fifo_count, SDMA_STATUS_I[15], sdma_par_err, sdma_par_fifo_count};
            `CA_FE_SLEEP_CTRL      : reg_out  <=  fe_sleep_ctrl;
            `CA_FPGA_CONFIG_CTRL   : reg_out  <=  {24'b0, FPGA_CONFIG_DONE_I, FPGA_CONFIG_RDY_I, FPGA_CONFIG_DONE_SEL_I, 1'b0, fpga_config_ctrl[0]};
            `CA_FPGA_CONFIG_DATA   : reg_out  <=  fpga_config_data;
            `CA_FPGA_CONFIG_SEL    : reg_out  <=  {29'b0, fpga_config_sel[2:0]};
            `CA_CDMA_HOST_SRC_ADDR : reg_out  <=  cdma_host_src_addr;
            `CA_CDMA_HOST_SRC_LEN  : reg_out  <=  cdma_host_src_len;
            `CA_CDMA_HOST_DST_ADDR : reg_out  <=  cdma_host_dst_addr;
            `CA_CDMA_HOST_DST_LEN  : reg_out  <=  cdma_host_dst_len;
            `CA_GDMA_HOST_SRC_ADDR : reg_out  <=  gdma_host_src_addr;
            `CA_GDMA_HOST_SRC_LEN  : reg_out  <=  gdma_host_src_len;
            `CA_GDMA_BP_DST_ADDR   : reg_out  <=  gdma_bp_dst_addr;
            `CA_CDMA_HOST_INT_STAT : reg_out  <=  cdma_host_int_stat;
            `CA_GDMA_HOST_INT_STAT : reg_out  <=  gdma_host_int_stat;
            `CA_CDMA_BP_SRC_ADDR   : reg_out  <=  cdma_bp_src_addr;
            `CA_CDMA_BP_SRC_LEN    : reg_out  <=  cdma_bp_src_len;
            `CA_CDMA_BP_DST_ADDR   : reg_out  <=  cdma_bp_dst_addr;
            `CA_CDMA_BP_DST_LEN    : reg_out  <=  cdma_bp_dst_len;
            `CA_CDMA_BP_INT_STAT   : reg_out  <=  cdma_bp_int_stat;
            `CA_COEF_BP_SRC_ADDR   : reg_out  <=  coef_bp_src_addr;
            `CA_COEF_BP_SRC_LEN    : reg_out  <=  coef_bp_src_len;
            `CA_BP_COEF_DST_ADDR   : reg_out  <=  bp_coef_dst_addr;
            `CA_BP_COEF_DST_LEN    : reg_out  <=  bp_coef_dst_len;
            `CA_BP_COEF_SRC_ADDR   : reg_out  <=  bp_coef_src_addr;
            `CA_BP_COEF_SRC_LEN    : reg_out  <=  bp_coef_src_len;
            `CA_COEF_BP_DST_ADDR   : reg_out  <=  coef_bp_dst_addr;
            `CA_COEF_BP_DST_LEN    : reg_out  <=  coef_bp_dst_len;
            `CA_COEF_DMA_INT_STAT  : reg_out  <=  {31'b0, coef_dma_int_stat};
            `CA_RFCF_BP_SRC_ADDR   : reg_out  <=  rfcf_bp_src_addr;
            `CA_RFCF_BP_SRC_LEN    : reg_out  <=  rfcf_bp_src_len;
            `CA_BP_RFCF_DST_ADDR   : reg_out  <=  bp_rfcf_dst_addr;
            `CA_BP_RFCF_DST_LEN    : reg_out  <=  bp_rfcf_dst_len;
            `CA_BP_RFCF_SRC_ADDR   : reg_out  <=  bp_rfcf_src_addr;
            `CA_BP_RFCF_SRC_LEN    : reg_out  <=  bp_rfcf_src_len;
            `CA_RFCF_BP_DST_ADDR   : reg_out  <=  rfcf_bp_dst_addr;
            `CA_RFCF_BP_DST_LEN    : reg_out  <=  rfcf_bp_dst_len;
            `CA_RFCF_DMA_INT_STAT  : reg_out  <=  {31'b0, rfcf_dma_int_stat};
            `CA_iBUTTON_STATUS     : reg_out  <=  {26'b0,1'b1,ibutton_status};  //bit 5 tells SW this Argon build supports iButton 
            `CA_iBUTTON_DATA_LOWER : reg_out  <=  ibutton_data[31:0];
            `CA_iBUTTON_DATA_UPPER : reg_out  <=  ibutton_data[63:32];
            `CA_PREG_EC_CTRL_ADDR  : reg_out  <=  {30'b0, preg_ec_boot0, preg_ec_reset};
   
            default                : reg_out  <=  0;
        endcase // case(SYS_ADDR_I[23:0])
        end
     end // else: !if(RESET)
    end // always @ (posedge CLK)

   // ComDMA registers
    always @(posedge CLK) begin
        if (RESET) begin
            cdma_host_src_addr <= 0;
            cdma_host_src_len  <= 0;
            cdma_host_dst_addr <= 0;
            cdma_host_dst_len  <= 0;
            cdma_host_int_stat <= 0;
            gdma_host_int_stat <= 0;
            cdma_bp_src_addr   <= 0;
            cdma_bp_src_len    <= 0;
            cdma_bp_dst_addr   <= `CD_CDMA_BP_DST_ADDR_DEF;
            cdma_bp_dst_len    <= `CD_CDMA_BP_DST_LEN_DEF;
            cdma_bp_int_stat   <= 0;
            cdma_bp_host_done_p <= 1'b0;
            cdma_host_bp_done_p <= 1'b0;
            gdma_host_src_addr <=0;
            gdma_host_src_len <=0;
            gdma_bp_dst_addr <=0;
            gdma_done_p <= 1'b0;
        end
        else begin
            cdma_bp_host_done_p <= CDMA_BP_HOST_DONE_I;
            cdma_host_bp_done_p <= CDMA_HOST_BP_DONE_I;
            gdma_done_p <= GDMA_DONE_I;
        // write to registers
            if (range_8000_we) begin
                case (SYS_ADDR_I[23:0])
                    `CA_CDMA_HOST_SRC_ADDR: cdma_host_src_addr <= SYS_DATA_I;
                    `CA_CDMA_HOST_SRC_LEN : cdma_host_src_len  <= SYS_DATA_I;
                    `CA_CDMA_HOST_DST_ADDR: cdma_host_dst_addr <= SYS_DATA_I;
                    `CA_CDMA_HOST_DST_LEN : cdma_host_dst_len  <= SYS_DATA_I;
                    `CA_GDMA_HOST_SRC_ADDR: gdma_host_src_addr <= SYS_DATA_I;
                    `CA_GDMA_HOST_SRC_LEN : gdma_host_src_len  <= SYS_DATA_I;
                    `CA_GDMA_BP_DST_ADDR  : gdma_bp_dst_addr   <= SYS_DATA_I;
                    `CA_CDMA_BP_SRC_ADDR  : cdma_bp_src_addr   <= SYS_DATA_I;
                    `CA_CDMA_BP_SRC_LEN   : cdma_bp_src_len    <= SYS_DATA_I;
                    `CA_CDMA_BP_DST_ADDR  : cdma_bp_dst_addr   <= SYS_DATA_I;
                    `CA_CDMA_BP_DST_LEN   : cdma_bp_dst_len    <= SYS_DATA_I;
                endcase // case(SYS_ADDR_I[23:0])
            end
      // clear logic
            if (CDMA_BP_HOST_DONE_I) begin
                cdma_bp_src_len   <= 0;
                cdma_host_dst_len <= 0;
            end
            if (CDMA_HOST_BP_DONE_I) begin
                cdma_host_src_len <= 0;
                cdma_bp_dst_len   <= 0;
            end
            if (GDMA_DONE_I)
                gdma_host_src_len <=0;

            // set and clear ComDMA interrupts
            // interrupts are cleared by writing "1" into a corresponding status bit
            if (CDMA_BP_HOST_DONE_I & ~cdma_bp_host_done_p)
                cdma_bp_int_stat[`CB_CDMA_BP_INT_TX] <= 1'b1;
            else if (range_8000_we & SYS_ADDR_I[23:0] == `CA_CDMA_BP_INT_STAT)
                cdma_bp_int_stat[`CB_CDMA_BP_INT_TX] <= cdma_bp_int_stat[`CB_CDMA_BP_INT_TX] & ~SYS_DATA_I[`CB_CDMA_BP_INT_TX];
            
            if (CDMA_HOST_BP_DONE_I & ~cdma_host_bp_done_p)
                cdma_bp_int_stat[`CB_CDMA_BP_INT_RX] <= 1'b1;
            else if (range_8000_we & SYS_ADDR_I[23:0] == `CA_CDMA_BP_INT_STAT)
                cdma_bp_int_stat[`CB_CDMA_BP_INT_RX] <= cdma_bp_int_stat[`CB_CDMA_BP_INT_RX] & ~SYS_DATA_I[`CB_CDMA_BP_INT_RX];
            
            if (CDMA_BP_HOST_DONE_I & ~cdma_bp_host_done_p)
                cdma_host_int_stat[`CB_CDMA_HOST_INT_RX] <=  1'b1;
            else if (range_8000_we & SYS_ADDR_I[23:0] == `CA_CDMA_HOST_INT_STAT)
                cdma_host_int_stat[`CB_CDMA_HOST_INT_RX] <= cdma_host_int_stat[`CB_CDMA_HOST_INT_RX] & ~SYS_DATA_I[`CB_CDMA_HOST_INT_RX];
	        
            if (CDMA_HOST_BP_DONE_I & ~cdma_host_bp_done_p)
                cdma_host_int_stat[`CB_CDMA_HOST_INT_TX] <=  1'b1;
            else if (range_8000_we & SYS_ADDR_I[23:0] == `CA_CDMA_HOST_INT_STAT)
                cdma_host_int_stat[`CB_CDMA_HOST_INT_TX] <= cdma_host_int_stat[`CB_CDMA_HOST_INT_TX] & ~SYS_DATA_I[`CB_CDMA_HOST_INT_TX];
	        
            if (GDMA_DONE_I & ~gdma_done_p)
                gdma_host_int_stat[`CB_GDMA_HOST_INT_TX] <= 1'b1;
            else if (range_8000_we & SYS_ADDR_I[23:0] == `CA_GDMA_HOST_INT_STAT)
                gdma_host_int_stat[`CB_GDMA_HOST_INT_TX] <= gdma_host_int_stat[`CB_GDMA_HOST_INT_TX] & ~SYS_DATA_I[`CB_GDMA_HOST_INT_TX];

        end // else: !if(RESET)
    end // always @ (posedge CLK)

  /* --------------------------- CoefXfeDMA Registers ------------------------------- */
   always @(posedge CLK) begin
        if (RESET) begin
            coef_bp_src_addr <= 0;
            coef_bp_src_len  <= 0;
            bp_coef_dst_addr <= 0;
            bp_coef_dst_len  <= 0;
            bp_coef_src_addr <= 0;
            bp_coef_src_len   <= 0;
            coef_bp_dst_addr    <= 0;
            coef_bp_dst_len   <= 0;
            coef_dma_int_stat   <= 0;
            coef_dma_done_p <= 1'b0;

        end
        else begin
      
            coef_dma_done_p <= BP_COEF_DMA_DONE_I | COEF_BP_DMA_DONE_I;

      // write to registers
            if (range_8000_we) begin
            case (SYS_ADDR_I[23:0])
                `CA_COEF_BP_SRC_ADDR  : coef_bp_src_addr <= SYS_DATA_I;
                `CA_COEF_BP_SRC_LEN   : coef_bp_src_len  <= SYS_DATA_I;
                `CA_BP_COEF_DST_ADDR  : bp_coef_dst_addr <= SYS_DATA_I;
                `CA_BP_COEF_DST_LEN   : bp_coef_dst_len  <= SYS_DATA_I;
                `CA_BP_COEF_SRC_ADDR  : bp_coef_src_addr <= SYS_DATA_I;
                `CA_BP_COEF_SRC_LEN   : bp_coef_src_len  <= SYS_DATA_I;
                `CA_COEF_BP_DST_ADDR  : coef_bp_dst_addr <= SYS_DATA_I;
                `CA_COEF_BP_DST_LEN   : coef_bp_dst_len  <= SYS_DATA_I;
            endcase // case(SYS_ADDR_I[23:0])
            end

      // clear logic
            if (BP_COEF_DMA_DONE_I) begin
                bp_coef_src_len   <= 0;
                bp_coef_dst_len   <= 0;
            end
            if (COEF_BP_DMA_DONE_I) begin
                coef_bp_src_len   <= 0;
                coef_bp_dst_len   <= 0;
            end
            // set and clear CoefXferDMA interrupt
            // interrupts are cleared by writing "1" into a corresponding status bit
            if ((BP_COEF_DMA_DONE_I | COEF_BP_DMA_DONE_I) & ~coef_dma_done_p)
             coef_dma_int_stat <= 1'b1;
            else if (range_8000_we & SYS_ADDR_I[23:0] == `CA_COEF_DMA_INT_STAT)
             coef_dma_int_stat <= coef_dma_int_stat & ~SYS_DATA_I[`CB_COEF_DMA_INT];                 
        end // else: !if(RESET)
    end // always @ (posedge CLK)

  /* --------------------------- RfcfXfeDMA Registers ------------------------------- */
    always @(posedge CLK) begin
        if (RESET) begin
            rfcf_bp_src_addr <= 0;
            rfcf_bp_src_len  <= 0;
            bp_rfcf_dst_addr <= 0;
            bp_rfcf_dst_len  <= 0;
            bp_rfcf_src_addr <= 0;
            bp_rfcf_src_len   <= 0;
            rfcf_bp_dst_addr    <= 0;
            rfcf_bp_dst_len   <= 0;
            rfcf_dma_int_stat   <= 0;
            rfcf_dma_done_p <= 1'b0;
        end
        else begin
            rfcf_dma_done_p <= BP_RFCF_DMA_DONE_I | RFCF_BP_DMA_DONE_I;
      // write to registers
            if (range_8000_we) begin
            case (SYS_ADDR_I[23:0])
                `CA_RFCF_BP_SRC_ADDR  : rfcf_bp_src_addr <= SYS_DATA_I;
                `CA_RFCF_BP_SRC_LEN   : rfcf_bp_src_len  <= SYS_DATA_I;
                `CA_BP_RFCF_DST_ADDR  : bp_rfcf_dst_addr <= SYS_DATA_I;
                `CA_BP_RFCF_DST_LEN   : bp_rfcf_dst_len  <= SYS_DATA_I;
                `CA_BP_RFCF_SRC_ADDR  : bp_rfcf_src_addr <= SYS_DATA_I;
                `CA_BP_RFCF_SRC_LEN   : bp_rfcf_src_len  <= SYS_DATA_I;
                `CA_RFCF_BP_DST_ADDR  : rfcf_bp_dst_addr <= SYS_DATA_I;
                `CA_RFCF_BP_DST_LEN   : rfcf_bp_dst_len  <= SYS_DATA_I;
            endcase // case(SYS_ADDR_I[23:0])
            end
            // clear logic
            if (BP_RFCF_DMA_DONE_I) begin
                bp_rfcf_src_len   <= 0;
                bp_rfcf_dst_len   <= 0;
            end
            if (RFCF_BP_DMA_DONE_I) begin
                rfcf_bp_src_len   <= 0;
                rfcf_bp_dst_len   <= 0;
            end
            // set and clear CoefXferDMA interrupt
            // interrupts are cleared by writing "1" into a corresponding status bit
            if ((BP_RFCF_DMA_DONE_I | RFCF_BP_DMA_DONE_I) & ~rfcf_dma_done_p)
                rfcf_dma_int_stat <= 1'b1;
            else if (range_8000_we & SYS_ADDR_I[23:0] == `CA_RFCF_DMA_INT_STAT)
                rfcf_dma_int_stat <= rfcf_dma_int_stat & ~SYS_DATA_I[`CB_RFCF_DMA_INT];       
        end // else: !if(RESET)
    end // always @ (posedge CLK)

  /* --------------------------- BP Interrupts ------------------------------- */

    always @(posedge CLK) begin
        if (RESET) begin
            bp_int        <= 1'b0;
            bp_int_en     <= 0;
            bp_int_stat   <= 0;
            rwave_p       <= 4'b0;
            rwave_int     <= 1'b0;
            dummy_int     <= 1'b0;
            heartbeat_int <= 1'b0;
            heartbeat_cnt <= 0;
            heartbeat_preset <= 0;  
        end
        else begin
            bp_int      <=  |(bp_int_en & bp_int_stat);
      // OR Neon and Xenon status registers with this one
            bp_int_stat <=  {
                                dummy_int,           // 31
                                3'b0,           // 30:28
                                gx_int,        // 27
                                FSEQ_INT_I,     // 26
                                7'b0,           // 25:19
                                XENON_INT_STAT_I,  // 18
                                1'b0,           // 17
                                GALIL_INT_I[1], // 16
                                heartbeat_int,  // 15
                                GALIL_INT_I[0], // 14
                                4'b0,           // 13:10
                                |cdma_bp_int_stat, // 9
                                rwave_int,      // 8
                                2'b0,           // 7:6
                                deadman_shutdown_int, // 5
                                2'b0,            // 4:3
                                rfcf_dma_int_stat, // 2
                                coef_dma_int_stat, // 1
                                1'b0            // 0
                        } | NEON_1_INT_STAT_I | NEON_2_INT_STAT_I;

            // BP interrupt enable
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_BP_INT_EN) begin
                bp_int_en <= SYS_DATA_I;
            end 
            // R-wave interrupt
            rwave_p <= {rwave_p[2:0], RWAVE_INT_I}; 
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_BP_INT_CLR_RWAVE) begin
                rwave_int <= 1'b0;
            end
            else if (rwave_p[2] & ~rwave_p[3]) begin
                rwave_int <= 1'b1;
            end
            // heartbeat interrupt
            if (heartbeat_cnt == 0)
                heartbeat_cnt <= heartbeat_preset;
            else
                heartbeat_cnt    <= heartbeat_cnt - 1;    
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_PPC_HEARTBEAT) begin
                heartbeat_preset <= SYS_DATA_I;
            end
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_BP_INT_CLR_HEARTBEAT) begin
                heartbeat_int <= 1'b0;
            end
            else if (heartbeat_cnt == 1) begin
                heartbeat_int <= 1'b1;
            end
      
            if (range_8000_we & SYS_ADDR_I[23:0] == `CA_BP_DUMMY_INT)
                dummy_int <= 1'b1;
            else if ((range_8000_we & SYS_ADDR_I[23:0] == `CA_BP_INT_STAT) && (SYS_DATA_I[`CB_BP_INT_DUMMY] == 1'b1))
                dummy_int <= 1'b0;
        end // else: !if(RESET)
    end // always @ (posedge CLK)

  /* --------------------------- Host Interrupts ----------------------------- */

    always @(posedge CLK) begin
        if (RESET) begin
            host_int     <= 0;
            host_int_en  <= 0;
            host_int_p   <= 0;
            host_int_out <= 0;
            brd_host_int_stat <= 0;
            ac_present_p <= 1;
        end
        else begin
            host_int_out <= &host_int_p;
            host_int_p   <= {host_int_p[10:0], host_int};     
            ac_present_p <= AC_PRESENT_I;
            host_int     <=  host_int_en[`CB_HOST_INT_EN_GBL]  & 
                            ((host_int_en[`CB_HOST_INT_EN_RX]  & cdma_host_int_stat[`CB_CDMA_HOST_INT_RX]) |
                            (host_int_en[`CB_HOST_INT_EN_TX]   & cdma_host_int_stat[`CB_CDMA_HOST_INT_TX]) |
                            (host_int_en[`CB_GDMA_INT_EN_TX]   & gdma_host_int_stat[`CB_GDMA_HOST_INT_TX]) |
                            (host_int_en[`CB_HOST_INT_EN_AC]   & brd_host_int_stat[`CB_AC_PRESENT]) |
                            (host_int_en[`CB_HOST_INT_EN_ADMA] & ADMA_INT_I));
      
        if (range_8000_we && SYS_ADDR_I[23:0] == `CA_HOST_INT_EN)
            host_int_en <= SYS_DATA_I;

        if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_BRD_INT_STAT) && SYS_DATA_I[`CB_AC_PRESENT])
            brd_host_int_stat[`CB_AC_PRESENT] <= 1'b0;
        else 
            brd_host_int_stat[`CB_AC_PRESENT] <= (~AC_PRESENT_I & ac_present_p) | (AC_PRESENT_I & ~ac_present_p) | brd_host_int_stat[`CB_AC_PRESENT];
        end // else: !if(RESET)
    end // always @ (posedge CLK)

  /* --------------------------- Misc control registers ---------------------- */

    always @(posedge CLK) begin
        if (RESET) begin
            line_timer_en   <= 0;
            bp_led          <= 0;
            hw_led          <= 0;
            hw_led_dis      <= 1'b0;
            sw_aio_usb_hub_pwr_dis <= 1'b0;
            sw_aio_reset_pulse <= 1'b0;
            aio_usb_hub_pwr_en_tc <= `CD_AIO_USB_PWR_EN_TC;
        // scratch_pad_reg  <= 32'h12345678;
        end
        else begin
        if (range_8000_we) begin
            case (SYS_ADDR_I[23:0])
                `CA_LINE_TIMER_EN     : begin
                              line_timer_en[`CB_LINE_TIMER_EN]   <= SYS_DATA_I[`CB_LINE_TIMER_EN];
                              line_timer_en[`CB_LINE_TIMER_STOP] <= SYS_DATA_I[`CB_LINE_TIMER_STOP];
                            end
                `CA_BP_LED             : begin 
                              bp_led     <= SYS_DATA_I[`CB_BP_LED];
                              hw_led     <= SYS_DATA_I[`CB_HW_LED];
                              hw_led_dis <= SYS_DATA_I[`CB_HW_LED_DIS];
                            end
                `CA_AIO_USB_PWR_EN_PULSE :    sw_aio_reset_pulse <= 1'b1;
                `CA_AIO_USB_PWR_DIS    :    sw_aio_usb_hub_pwr_dis <= SYS_DATA_I[0];  
                `CA_AIO_USB_PWR_EN_TC  :    aio_usb_hub_pwr_en_tc <= SYS_DATA_I[23:0];
            //`CA_SCRATCH_PAD_ADDR     :      scratch_pad_reg <= SYS_DATA_I[31:0];                    
            endcase // case(SYS_ADDR_I[23:0])
        end
        else
            sw_aio_reset_pulse <= 1'b0;
        end // else: !if(RESET)
    end // always @ (posedge CLK)

  /* --------------------------- Reset and BAR remap registers --------------- */

    always @(posedge CLK) begin
        if (RESET) begin
            fe_rst                <= 0;
            hw_rst                <= 32'h00000001;
            rst_ctl               <= 0;
            rst_ctl[`CB_RST_GX_LINKS] <= 1'b1; // keep GX links disabled on power on
            rst_ctl[`CB_RST_PPC]  <= 1'b1; // keep PPC in reset on power on
            PCIBAR_Remap_0        <= `CD_BAR_REMAP_0;
            PCIBAR_Remap_1        <= `CD_BAR_REMAP_1;
            PCIBAR_Remap_2        <= `CD_BAR_REMAP_2;
            PCIBAR_Remap_3        <= `CD_BAR_REMAP_3;
        end
        else begin
        // write to registers
            if (range_8000_we) begin
                case (SYS_ADDR_I[23:0])
                    `CA_FE_RST         : fe_rst         <= SYS_DATA_I;
                    `CA_RST_CTL        : rst_ctl        <= SYS_DATA_I;
                    `CA_HW_RST         : hw_rst         <= SYS_DATA_I;
                    `CA_BAR_REMAP_0    : PCIBAR_Remap_0 <= SYS_DATA_I;
                    `CA_BAR_REMAP_1    : PCIBAR_Remap_1 <= SYS_DATA_I;
                    `CA_BAR_REMAP_2    : PCIBAR_Remap_2 <= SYS_DATA_I;
                    `CA_BAR_REMAP_3    : PCIBAR_Remap_3 <= SYS_DATA_I;
                endcase // case(SYS_ADDR_I[23:0])
            end
        end // else: !if(RESET)
   end // always @ (posedge CLK)

  /* --------------------------- Motor controller ----------------------- */

  /* This copied from the Pathfider and describes the position counter and 
     direction funtionality
     
     Counting     
                  ___     ___     ___     ___ 
     enc_a    ___/   \___/   \___/   \___/   \
              _     ___     ___     ___     ___ 
     enc_b     \___/   \___/   \___/   \___/   \
     
     count     1 2 3 4 5 6 7 8 9 etc.
    
     Turn around one way-always sees 2 edges of same encoder
                  ___     ___     ___     ___ 
     enc_a    ___/   \___/   \___/   \___/   \
              _     _______     ___     ___ 
     enc_b     \___/       \___/   \___/   \
     
     count     7 8 9 10  9 8 7 6 5
     
              _     _______     ___     ___ 
     enc_a     \___/       \___/   \___/   \
                  ___     ___     ___     ___ 
     enc_b    ___/   \___/   \___/   \___/   \
     
     count     6 5 4 3   4 5 6 7
        
     the case which triggers a change in direction is when two
     consecutive edges are detected while the other encoder signal is '1'   
            ___     ___     ___     ___ 
     A  ___/   \___/   \___/   \___/   \
              ___         ___     ___ 
     B  _____/   \_______/   \___/   \
                        _____________
     dir ______________/
     
            ___     _______     ___ 
     B  ___/   \___/       \___/   \
              ___     ___     ___     ___ 
     A  _____/   \___/   \___/   \___/   \
                          _____________
     dir ________________/
     
     
     backlash case
            ___     _______         _____ 
     A  ___/   \___/       \_______/
              ___     ___     ___     ___ 
     B  _____/   \___/   \___/   \___/   \
                          _______
     dir ________________/       \_______
     
     Here is a simplified approach to a new quadrature decoder for the Omni probe.  It delays both input
     channels and then does an xor on a subsequent pair of both channels.  The delay is used to reduce
     metastability.  System software has the capability of reseting the count back to zero.  The counter 
     will roll over if not reset.  This is allowable as long as software samples the count fast enough and 
     keeps track of the direction of count.  Another part of this logic is the enable on the output
     latch.  When  omni_latch_ena = 1 the latch always gets the value of the counter.   When low, the latch 
     value is kept so that software can read a valid, unchanging number.

     Remember... an xor does the following:  0011=0, 1000=1, 1110=1, 0000=0, 1111=0

  */

    wire motor_count_en_pre  = motor_step_p[2] ^ motor_step_p[3] ^ motor_dir_p[2] ^ motor_dir_p[3];
    wire motor_count_en      = motor_step_p[3] ^ motor_step_p[4] ^ motor_dir_p[3] ^ motor_dir_p[4];
    wire motor_count_dir_pre = motor_step_p[2] ^ motor_dir_p[3];

    always @(posedge CLK) begin
        if (RESET) begin
            motor_count_dir <= 0; 
        end
        else if (motor_count_en_pre) begin
            if (motor_count_dir_pre)
                motor_count_dir <= 1;
            else
                motor_count_dir <= 0;
            end
        end
  
    always @(posedge CLK) begin
        if (RESET) begin
            motor_control      <= 0; 
            motor_pos          <= 0; 
            motor_pos_control  <= 0; 
            motor_trig_pos_rev <= 0; 
            motor_acquire_p    <= 0; 
            motor_step_p       <= 0; 
            motor_dir_p        <= 0; 
            motor_fault_p      <= 0;
            preg_ec_reset      <= 1'b0;
            preg_ec_boot0      <= 1'b0;
        end 
        else begin
       // motor signal reclocking and edge detection
            motor_acquire_p <= {motor_acquire_p[3:0], MOTOR_ACQUIRE_I};
            motor_step_p    <= {motor_step_p[3:0], MOTOR_STEP_I};
            motor_dir_p     <= {motor_dir_p[3:0], MOTOR_DIRECTION_I};
            motor_fault_p   <= {motor_fault_p[3:0], MOTOR_FAULT_I};        
            motor_control[`CB_MOTOR_ACQ]   <= motor_acquire_p[4];
            motor_control[`CB_MOTOR_STEP]  <= motor_step_p[4];
            motor_control[`CB_MOTOR_DIR]   <= motor_dir_p[4];
            motor_control[`CB_MOTOR_FAULT] <= motor_fault_p[4];
       
       // clear MOVE bit on a positive edge of ACQUIRE signal
            if (motor_acquire_p[3] & ~motor_acquire_p[4]) begin
                motor_control[`CB_MOTOR_MOVE]  <= 1'b0;
            end
            else if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_MOTOR_CONTROL)) begin
                motor_control[`CB_MOTOR_MOVE] <= SYS_DATA_I[`CB_MOTOR_MOVE];
            end  
            if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_MOTOR_CONTROL)) begin
                motor_control[`CB_MOTOR_RESET] <= SYS_DATA_I[`CB_MOTOR_RESET];
            end    
            if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_MOTOR_CONTROL)) begin
                motor_control[`CB_MOTOR_CLK_EN] <= SYS_DATA_I[`CB_MOTOR_CLK_EN];
            end	    
            if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_MOTOR_CONTROL)) begin
                motor_control[`CB_MOTOR_PLL_EN] <= SYS_DATA_I[`CB_MOTOR_PLL_EN];
            end
               // motor position register
            if (motor_count_en) begin
                if (motor_count_dir)
                    motor_pos <= motor_pos + 1;
                else
                    motor_pos <= motor_pos - 1;
            end
            else if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_MOTOR_POS)) begin
                motor_pos <= SYS_DATA_I[`CB_MOTOR_POS-1:0];
            end
		    
            if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_MOTOR_POS_CONTROL)) begin
                motor_pos_control <= SYS_DATA_I[`CB_MOTOR_POS_CONTROL-1:0];
            end
		    
            if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_MOTOR_TRIG_POS_REV)) begin
                motor_trig_pos_rev <= SYS_DATA_I[`CB_MOTOR_TRIG_POS_REV-1:0];
            end
		    
            if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_PREG_EC_CTRL_ADDR))
            begin
                preg_ec_reset  <= SYS_DATA_I[0];
                preg_ec_boot0   <= SYS_DATA_I[1];
            end  
        end // else: !if(RESET)
   end // always @ (posedge CLK)

  /* --------------------------- DAS control ------------------------------- */

    always @(posedge CLK) begin
        if (RESET) begin
            das_buf_addr <= 0; 
            das_buf_size <= 0; 
            das_sum_n    <= 0; 
        end 
        else begin       
        // write to registers
            if (range_8000_we) begin
                case (SYS_ADDR_I[23:0])
                 `CA_DAS_BUF_ADDR  : das_buf_addr  <= SYS_DATA_I[`CB_DAS_BUF_ADDR-1:0];
                 `CA_DAS_BUF_SIZE  : das_buf_size  <= SYS_DATA_I[`CB_DAS_BUF_SIZE-1:0];
                 `CA_DAS_SUM_N     : das_sum_n     <= SYS_DATA_I[`CB_DAS_SUM_N-1:0];
                endcase // case(SYS_ADDR_I[23:0])
            end
        end // else: !if(RESET)
    end // always @ (posedge CLK)

  /* --------------------------- Switching phase select ----------------------- */
    always @(posedge CLK) begin
        if (RESET) begin
            sw_phase_sel <= 0;  // all switchers are in sync
        end
        else begin
        // write to register
            if (range_8000_we) begin
                case (SYS_ADDR_I[23:0])
                 `CA_SW_PHASE_SEL  : sw_phase_sel  <= SYS_DATA_I[`CB_SW_PHASE_SEL-1:0];
                endcase // case(SYS_ADDR_I[23:0])
            end
        end // else: !if(RESET)
    end // always @ (posedge CLK)

  /* ---------------------------Sleep Control ----------------------- */

    always @(posedge CLK) begin
        if (RESET) begin
            fe_sleep_ctrl         <= 2'b11;
            end
        else begin
      // write to register
            if (range_8000_we) begin
                case (SYS_ADDR_I[23:0])
                    `CA_FE_SLEEP_CTRL  : fe_sleep_ctrl  <= SYS_DATA_I[`CB_FE_SLEEP_CTRL-1:0];
                endcase // case(SYS_DATA_I[23:0])
            end
        end // else: !if(RESET)
    end // always @ (posedge CLK)

  /* --------------------------- FPGA configuration -------------------------- */
    always @(posedge CLK) begin
        if (RESET) begin
            fpga_config_ctrl  <= 1;
            fpga_config_data  <= 0;
            fpga_config_sel   <= 0;
            fpga_config_wr    <= 1'b0;
        end
        else begin
            fpga_config_wr    <= 1'b0;
        // write to registers
            if (range_8000_we) begin
                case (SYS_ADDR_I[23:0])
                    `CA_FPGA_CONFIG_CTRL  : fpga_config_ctrl  <= SYS_DATA_I[7:0];
                    `CA_FPGA_CONFIG_DATA  : begin 
                                                       fpga_config_data <= SYS_DATA_I;
                                                       fpga_config_wr   <= 1'b1;
                                            end
                    `CA_FPGA_CONFIG_SEL   : fpga_config_sel   <= SYS_DATA_I[2:0];
                endcase // case(SYS_ADDR_I[23:0])
            end
        end // else: !if(RESET)
    end // always @ (posedge CLK)

  /* --------------------------- Test point control --------------------------------- */
    always @(posedge CLK) begin
        if (RESET) begin
            tp_ctrl <= 0;
        end
        else begin
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_TP_CTRL)
                tp_ctrl <= SYS_DATA_I;	       
            end // else: !if(RESET)
    end // always @ (posedge CLK)
   
  /* --------------------------- Line header --------------------------------- */
    always @(posedge CLK) begin
        if (RESET) begin
            load_sets      <= 1'b0;
            control_set_id <= 0;
            line_type      <= 0;
            frame_info     <= 0;
            mode_info      <= 0;
        end
        else begin	    
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_CONTROL_SET_ID)
                control_set_id <= SYS_DATA_I;		    
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_LINE_TYPE)
                line_type <= SYS_DATA_I;          
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_LOAD_SETS)
                load_sets <= 1'b1;
            else
                load_sets <= 1'b0;           
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_FRAME_INFO)
                frame_info <= SYS_DATA_I;	    
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_MODE_INFO)
                mode_info <= SYS_DATA_I;    
        end // else: !if(RESET)
    end // always @ (posedge CLK)
   
  /* --------------------------- XMIT control -------------------------------- */
   // clear deadman counter when BP is reset
   always @(posedge CLK) begin
        if (RESET) begin
            deadman_shutdown     <= 1'b0;
            deadman_shutdown_p1  <= 1'b0;
            deadman_shutdown_en  <= 1'b0;
            deadman_shutdown_int <= 1'b0;
            deadman_shutdown_cnt <= 0;
            xmit_stop            <= 1'b0;
            load_line            <= 1'b0;
            load_line_p1         <= 1'b0;
        end
        else begin
            deadman_shutdown_p1 <= deadman_shutdown;
            load_line_p1        <= load_line;
            // deadmann counter and shutdown
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_PPC_ALIVE)
                deadman_shutdown_cnt <= 0;
            else if (deadman_shutdown_en) begin
            if (deadman_shutdown_cnt == `CD_DEADMAN_CNT_TC)
                deadman_shutdown <= 1'b1;
            else
                deadman_shutdown_cnt <= deadman_shutdown_cnt + 1;
            end   
            // deadmann shutdown enable
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_DEADMAN_CNT_EN)
             deadman_shutdown_en <= SYS_DATA_I[`CB_DEADMAN_CNT_EN];
		    
            // deadmann shutdown interrupt
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_PPC_ALIVE)
             deadman_shutdown_int <= 1'b0;
            else
             deadman_shutdown_int <= (deadman_shutdown & ~deadman_shutdown_p1) | 
                                           deadman_shutdown_int;
            // XMIT stop
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_XMIT_STOP)
             xmit_stop <= SYS_DATA_I[`CB_XMIT_STOP];
            
            // manual LOAD_LINE generation
            if (range_8000_we && SYS_ADDR_I[23:0] == `CA_LOAD_LINE)
             load_line <= SYS_DATA_I[`CB_LOAD_LINE];       
        end // else: !if(RESET)
    end // always @ (posedge CLK)
   
  /* --------------------------- I2C Interfaces ------------------------------ */

   // Diagnostics I2C bus is shared with the PIC programming
   // I2C clock is always an output in the PIC programming mode
    ALT_IOBUF diag_i2c_clk_buf (
        .o(diag_i2c_clk_in),     
        .io(DIAG_I2C_CLK_IO),    
        .i(diag_i2c_pic_prog_sel ? diag_i2c_pic_prog_clk : diag_i2c_clk_out),    
        .oe(diag_i2c_pic_prog_sel ? 1'b0 : diag_i2c_clk_tri)     
    );
   
   // I2C data is bidirectional in the PIC programming mode
    ALT_IOBUF diag_i2c_dat_buf (
        .o(diag_i2c_dat_in),     
        .io(DIAG_I2C_DAT_IO),    
        .i(diag_i2c_pic_prog_sel ? diag_i2c_pic_prog_dat : diag_i2c_dat_out),    
        .oe(diag_i2c_pic_prog_sel ? diag_i2c_pic_prog_dir : diag_i2c_dat_tri)     
    );
   
    always @(posedge CLK) begin
        if (RESET) begin
            diag_i2c_rdwr     <= 1'b0;
            diag_i2c_start    <= 1'b0;
            diag_i2c_stop     <= 1'b0;
            diag_i2c_rdnack   <= 1'b0;
            diag_i2c_addr     <= 0;
            diag_i2c_len      <= 0;
            diag_i2c_pic_prog <= 0;
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_DIAG_I2C_CTL) begin
            diag_i2c_rdwr   <= SYS_DATA_I[`CB_I2C_RDWR];
            diag_i2c_start  <= SYS_DATA_I[`CB_I2C_START];
            diag_i2c_stop   <= SYS_DATA_I[`CB_I2C_STOP];
            diag_i2c_rdnack <= SYS_DATA_I[`CB_I2C_RDNACK];
            diag_i2c_addr   <= SYS_DATA_I[`CB_I2C_ADDR];
            diag_i2c_len    <= SYS_DATA_I[`CB_I2C_LEN];
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_PIC_PROG) begin
            diag_i2c_pic_prog <= SYS_DATA_I;
        end	    
        if (RESET | diag_i2c_done | diag_i2c_data_err_reg | diag_i2c_addr_err_reg) begin
            diag_i2c_go <= 1'b0;
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_DIAG_I2C_CTL) begin
            diag_i2c_go <= SYS_DATA_I[`CB_I2C_GO];
        end 
        // HW semaphore
        // first read returns '1', subsequent reads return '0'
        // SW must set by writing '1'
        if (RESET | (range_8000_we && SYS_ADDR_I[23:0] == `CA_DIAG_I2C_CTL && SYS_DATA_I[`CB_I2C_SEM])) begin
            diag_i2c_sem <= 1'b1;
        end
        else if (range_8000_re && SYS_ADDR_I[23:0] == `CA_DIAG_I2C_CTL) begin
            diag_i2c_sem <= 1'b0;
        end
        // HW mutex 
        // The lsb of the register is the lock bit.
        // If the lock bit is set, only a write with the ID stored in the mutex and a lock bit of zero, will unlock the mutex and set the contents to zero.
        // The mutex will only update if the lock bit stored in the mutex is zero.
        if (RESET) begin
            diag_i2c_mutex <= 0;
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_DIAG_I2C_MUTEX && SYS_DATA_I[`CB_I2C_MUTEX_ID] == diag_i2c_mutex[`CB_I2C_MUTEX_ID]) begin
            diag_i2c_mutex <= SYS_DATA_I[8:0];
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_DIAG_I2C_MUTEX && diag_i2c_mutex[`CB_I2C_MUTEX_LOCK] == 1'b0) begin
            diag_i2c_mutex <= SYS_DATA_I[8:0];
        end
        else begin
            diag_i2c_mutex <= diag_i2c_mutex;
        end            
        // address error flag
        if (RESET | (range_8000_we && SYS_ADDR_I[23:0] == `CA_DIAG_I2C_CTL && SYS_DATA_I[`CB_I2C_ADDR_ERR])) begin
            diag_i2c_addr_err_reg <= 1'b0;
        end
        else if (diag_i2c_addr_err) begin
            diag_i2c_addr_err_reg <= 1'b1;
        end
    
        // data error flag
        if (RESET | (range_8000_we && SYS_ADDR_I[23:0] == `CA_DIAG_I2C_CTL && SYS_DATA_I[`CB_I2C_DATA_ERR])) begin
            diag_i2c_data_err_reg <= 1'b0;
        end
        else if (diag_i2c_data_err) begin
            diag_i2c_data_err_reg <= 1'b1;
        end
        // write data
        if (RESET) begin
            diag_i2c_data <= 0;
        end
        else if (diag_i2c_go & diag_i2c_done & diag_i2c_rdwr) begin
            diag_i2c_data <= diag_i2c_rd_data;
        end
        else if (range_8000_we) begin
            case (SYS_ADDR_I[23:0])
                `CA_DIAG_I2C_DAT_W0 :  diag_i2c_data[ 31:  0] <= SYS_DATA_I;
                `CA_DIAG_I2C_DAT_W1 :  diag_i2c_data[ 63: 32] <= SYS_DATA_I;
                `CA_DIAG_I2C_DAT_W2 :  diag_i2c_data[ 95: 64] <= SYS_DATA_I;
                `CA_DIAG_I2C_DAT_W3 :  diag_i2c_data[127: 96] <= SYS_DATA_I;
            endcase
        end 
    end // always @ (posedge CLK)

   // deglitcher
    deglitcher
        #(
        .c_width          (5)
        )
    diag_scl_dg (
        .RST               (RESET),
        .CLK               (CLK),
        .DIN               (diag_i2c_clk_in),
        .DOUT              (diag_i2c_clk_dg)
    );

    deglitcher 
       #(
        .c_width          (5)
       )
    diag_sda_dg   
      (
        .RST               (RESET),
        .CLK               (CLK),
        .DIN               (diag_i2c_dat_in),
        .DOUT              (diag_i2c_dat_dg)
    );

    i2c diag_i2c_i (
        .clk             (CLK),
        .reset           (RESET),
        .I2cRdWr         (diag_i2c_rdwr),
        .WrDat           ({144'h000000000000000000000000000000000000, diag_i2c_data}),
        .RdDat           (diag_i2c_rd_data),
        .I2cAddr         (diag_i2c_addr),
        .I2cLen          (diag_i2c_len),
        .I2cGo           (diag_i2c_go),
        .I2cDone         (diag_i2c_done),
        .I2cStart        (diag_i2c_start),
        .I2cStop         (diag_i2c_stop),
        .I2cRdNack       (diag_i2c_rdnack),
        .I2cAddrErr      (diag_i2c_addr_err),
        .I2cDatErr       (diag_i2c_data_err),
        .i2c_scl_out     (diag_i2c_clk_out),
        .i2c_scl_in      (diag_i2c_clk_dg),
        .i2c_scl_tri     (diag_i2c_clk_tri),
        .i2c_sda_out     (diag_i2c_dat_out),
        .i2c_sda_in      (diag_i2c_dat_dg),
        .i2c_sda_tri     (diag_i2c_dat_tri),
        .i2c_en          (1'b1)
    );

    ALT_IOBUF util_i2c_clk_buf (
        .o(util_i2c_clk_in),     
        .io(UTIL_I2C_CLK_IO),    
        .i(util_i2c_clk_out),    
        .oe(util_i2c_clk_tri)     
    );
   
    ALT_IOBUF util_i2c_dat_buf (
        .o(util_i2c_dat_in),     
        .io(UTIL_I2C_DAT_IO),    
        .i(util_i2c_dat_out),    
        .oe(util_i2c_dat_tri)     
    );
   
    always @(posedge CLK) begin
        if (RESET) begin
            util_i2c_rdwr   <= 1'b0;
            util_i2c_start  <= 1'b0;
            util_i2c_stop   <= 1'b0;
            util_i2c_rdnack <= 1'b0;
            util_i2c_addr   <= 0;
            util_i2c_len    <= 0;
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_UTIL_I2C_CTL) begin
            util_i2c_rdwr      <= SYS_DATA_I[`CB_I2C_RDWR];
            util_i2c_start     <= SYS_DATA_I[`CB_I2C_START];
            util_i2c_stop      <= SYS_DATA_I[`CB_I2C_STOP];
            util_i2c_rdnack    <= SYS_DATA_I[`CB_I2C_RDNACK];
            util_i2c_addr      <= SYS_DATA_I[`CB_I2C_ADDR];
            util_i2c_len       <= SYS_DATA_I[`CB_I2C_LEN];
        end
        if (RESET | util_i2c_done | util_i2c_data_err_reg | util_i2c_addr_err_reg) begin
            util_i2c_go <= 1'b0;
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_UTIL_I2C_CTL) begin
            util_i2c_go <= SYS_DATA_I[`CB_I2C_GO];
        end    
        // HW semaphore
        // first read returns '1', subsequent reads return '0'
        // SW must set by writing '1'
        if (RESET | (range_8000_we && SYS_ADDR_I[23:0] == `CA_UTIL_I2C_CTL && SYS_DATA_I[`CB_I2C_SEM])) begin
            util_i2c_sem <= 1'b1;
        end
        else if (range_8000_re && SYS_ADDR_I[23:0] == `CA_UTIL_I2C_CTL) begin
            util_i2c_sem <= 1'b0;
        end
        // HW mutex 
        // The lsb of the register is the lock bit.
        // If the lock bit is set, only a write with the ID stored in the mutex and a lock bit of zero, will unlock the mutex and set the contents to zero.
        // The mutex will only update if the lock bit stored in the mutex is zero.
        if (RESET) begin
            util_i2c_mutex <= 0;
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_UTIL_I2C_MUTEX && SYS_DATA_I[`CB_I2C_MUTEX_ID] == util_i2c_mutex[`CB_I2C_MUTEX_ID]) begin
            util_i2c_mutex <= SYS_DATA_I[8:0];
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_UTIL_I2C_MUTEX && util_i2c_mutex[`CB_I2C_MUTEX_LOCK] == 1'b0) begin
            util_i2c_mutex <= SYS_DATA_I[8:0];
        end
        else begin
            util_i2c_mutex <= util_i2c_mutex;
        end            
        // address error flag
        if (RESET | (range_8000_we && SYS_ADDR_I[23:0] == `CA_UTIL_I2C_CTL && SYS_DATA_I[`CB_I2C_ADDR_ERR])) begin
            util_i2c_addr_err_reg <= 1'b0;
        end
        else if (util_i2c_addr_err) begin
            util_i2c_addr_err_reg <= 1'b1;
        end    
        // data error flag
        if (RESET | (range_8000_we && SYS_ADDR_I[23:0] == `CA_UTIL_I2C_CTL && SYS_DATA_I[`CB_I2C_DATA_ERR])) begin
            util_i2c_data_err_reg <= 1'b0;
        end
        else if (util_i2c_data_err) begin
            util_i2c_data_err_reg <= 1'b1;
        end  
        // write data
        if (RESET) begin
            util_i2c_data <= 0;
        end
        else if (util_i2c_go & util_i2c_done & util_i2c_rdwr) begin
            util_i2c_data <= util_i2c_rd_data;
        end
        else if (range_8000_we) begin
            case (SYS_ADDR_I[23:0])
                `CA_UTIL_I2C_DAT_W0 :  util_i2c_data[ 31:  0] <= SYS_DATA_I;
                `CA_UTIL_I2C_DAT_W1 :  util_i2c_data[ 63: 32] <= SYS_DATA_I;
                `CA_UTIL_I2C_DAT_W2 :  util_i2c_data[ 95: 64] <= SYS_DATA_I;
                `CA_UTIL_I2C_DAT_W3 :  util_i2c_data[127: 96] <= SYS_DATA_I;
            endcase
        end
    end // always @ (posedge CLK)
   
  // deglitcher
    deglitcher
        #(
        .c_width          (5)
        )
    util_scl_dg (
        .RST               (RESET),
        .CLK               (CLK),
        .DIN               (util_i2c_clk_in),
        .DOUT              (util_i2c_clk_dg)
    );

    deglitcher 
       #(
        .c_width          (5)
       )
    util_sda_dg   
      (
        .RST               (RESET),
        .CLK               (CLK),
        .DIN               (util_i2c_dat_in),
        .DOUT              (util_i2c_dat_dg)
    ); 

    i2c util_i2c_i (
        .clk             (CLK),
        .reset           (RESET),
        .I2cRdWr         (util_i2c_rdwr),
        .WrDat           ({144'h000000000000000000000000000000000000, util_i2c_data}),
        .RdDat           (util_i2c_rd_data),
        .I2cAddr         (util_i2c_addr),
        .I2cLen          (util_i2c_len),
        .I2cGo           (util_i2c_go),
        .I2cDone         (util_i2c_done),
        .I2cStart        (util_i2c_start),
        .I2cStop         (util_i2c_stop),
        .I2cRdNack       (util_i2c_rdnack),
        .I2cAddrErr      (util_i2c_addr_err),
        .I2cDatErr       (util_i2c_data_err),
        .i2c_scl_out     (util_i2c_clk_out),
        .i2c_scl_in      (util_i2c_clk_dg),
        .i2c_scl_tri     (util_i2c_clk_tri),
        .i2c_sda_out     (util_i2c_dat_out),
        .i2c_sda_in      (util_i2c_dat_dg),
        .i2c_sda_tri     (util_i2c_dat_tri),
        .i2c_en          (1'b1)
    );
 /*   REPLACE BATTERY SMB WITH I2C interface
 
   IOBUF batt_i2c_clk_buf (
      .O(batt_i2c_clk_in),     
      .IO(BATT_I2C_CLK_IO),    
      .I(batt_i2c_clk_out),    
      .T(batt_i2c_clk_tri)     
   );
   
   IOBUF batt_i2c_dat_buf (
      .O(batt_i2c_dat_in),     
      .IO(BATT_I2C_DAT_IO),    
      .I(batt_i2c_dat_out),    
      .T(batt_i2c_dat_tri)     
   );

   assign batt_smb_rd = range_8000_cs && SYS_ADDR_I[23:0] == `CA_BATT_SMB_DAT;
   assign batt_smb_wr = range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_SMB_DAT;
   
   always @(posedge CLK) begin
    if (RESET) begin
     batt_smb_protocol <= 0;
     batt_smb_addr     <= 0;
    // batt_smb_go       <= 0;
     batt_smb_sem      <= 0;
     batt_smb_reset    <= 0;
     batt_smb_len      <= 0;
     batt_smb_cmd      <= 0;
    
    
    
    end
    else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_SMB_CTL) begin//0x8080 0124
     batt_smb_protocol <= SYS_DATA_I[`CB_SMB_PROTOCOL];
     batt_smb_addr     <= SYS_DATA_I[`CB_SMB_ADDR];
     batt_smb_reset    <= SYS_DATA_I[`CB_SMB_RESET];
     batt_smb_len      <= SYS_DATA_I[`CB_SMB_LEN];
     batt_smb_cmd      <= SYS_DATA_I[`CB_SMB_CMD];
    end


   

    if (RESET | batt_smb_done | batt_smb_data_err_reg | batt_smb_addr_err_reg) begin
     batt_smb_go <= 1'b0;
    end
    else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_SMB_CTL) begin
     batt_smb_go <= SYS_DATA_I[`CB_SMB_GO];
    end

    // HW semaphore
    // first read returns '1', subsequent reads return '0'
    // SW must set by writing '1'
    if (RESET | (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_SMB_CTL && SYS_DATA_I[`CB_SMB_SEM])) begin
     batt_smb_sem <= 1'b1;
    end
    else if (range_8000_re && SYS_ADDR_I[23:0] == `CA_BATT_SMB_CTL) begin
     batt_smb_sem <= 1'b0;
    end

    // address error flag
    if (RESET | (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_SMB_CTL && SYS_DATA_I[`CB_SMB_ADDR_ERR])) begin
     batt_smb_addr_err_reg <= 1'b0;
    end
    else if (batt_smb_addr_err) begin
     batt_smb_addr_err_reg <= 1'b1;
    end
    
    // data error flag
    if (RESET | (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_SMB_CTL && SYS_DATA_I[`CB_SMB_DATA_ERR])) begin
     batt_smb_data_err_reg <= 1'b0;
    end
    else if (batt_smb_data_err) begin
     batt_smb_data_err_reg <= 1'b1;
    end
    
    // write data
    if (RESET) begin
     batt_smb_wr_p  <= 0;
     batt_smb_wr_en <= 0;
     batt_smb_wr_data <= 0;
     batt_smb_wr_ptr  <= 0;
    end
    else if (batt_smb_wr) begin
     batt_smb_wr_data <= SYS_DATA_I[`CB_SMB_DAT];
     batt_smb_wr_ptr  <= SYS_DATA_I[`CB_SMB_PTR];
     batt_smb_wr_p    <= batt_smb_wr;
     batt_smb_wr_en   <= batt_smb_wr_p & ~batt_smb_wr;
    end
    else begin
     batt_smb_wr_p  <= batt_smb_wr;
     batt_smb_wr_en <= batt_smb_wr_p & ~batt_smb_wr;
    end
    
    // read data
    if (RESET) begin
     batt_smb_rd_p  <= 0;
     batt_smb_rd_en <= 0;
    end
    else begin
     batt_smb_rd_p  <= batt_smb_rd;
     batt_smb_rd_en <= batt_smb_rd_p & ~batt_smb_rd;
    end
    
   end // always @ (posedge CLK)
  

    // deglitcher
  deglitcher
     #(
     .c_width          (5)
     )
  smb_scl_dg (
    .RST               (RESET),
    .CLK               (CLK),
    .DIN               (batt_i2c_clk_in),
    .DOUT              (batt_i2c_clk_dg)
    );

  deglitcher 
     #(
     .c_width          (5)
     )
  smb_sda_dg   
    (
    .RST               (RESET),
    .CLK               (CLK),
    .DIN               (batt_i2c_dat_in),
    .DOUT              (batt_i2c_dat_dg)
    );  

     protego_smb batt_smb_i (
      .CLK             (CLK),
      .RESET           (RESET),
      .SMB_PROTOCOL_I  (batt_smb_protocol),
      .SMB_ADDR_I      (batt_smb_addr),
      .SMB_LEN_I       (batt_smb_len),
      .SMB_CMD_I       (batt_smb_cmd),
      .SMB_WR_DATA_I   (batt_smb_wr_data),
      .SMB_WR_PTR_I    (batt_smb_wr_ptr),
      .SMB_WR_EN_I     (batt_smb_wr_en),
      .SMB_RD_EN_I     (batt_smb_rd_en),
      .SMB_GO_I        (batt_smb_go),
      .SMB_RESET_I     (batt_smb_reset),
      .SMB_ADDR_ERR_O  (batt_smb_addr_err),
      .SMB_DATA_ERR_O  (batt_smb_data_err),
      .SMB_DONE_O      (batt_smb_done),
      .SMB_RD_DATA_O   (batt_smb_rd_data),
      .SMB_RD_PTR_O    (batt_smb_rd_ptr),
      .I2C_CLK_O       (batt_i2c_clk_out),
      .I2C_CLK_TRI_O   (batt_i2c_clk_tri),
      .I2C_CLK_I       (batt_i2c_clk_dg),
      .I2C_DAT_O       (batt_i2c_dat_out),
      .I2C_DAT_TRI_O   (batt_i2c_dat_tri),
      .I2C_DAT_I       (batt_i2c_dat_dg),
     .smb_i2c_state(smb_i2c_state),
       .smb_i2c_Clk0Tck(smb_i2c_Clk0Tck),
       .smb_i2c_Clk1Tck(smb_i2c_Clk1Tck),
       .smb_i2c_I2cGo(smb_i2c_I2cGo),
    // .debug_force_start_invert_I(debug_force_start_invert),  //for debug only
    // .debug_force_stop_invert_I(debug_force_stop_invert),
      .CS_DATA_O       (batt_smb_csdata)
     );

*/     
     
                    
    ALT_IOBUF batt_i2c_clk_buf (
        .o(batt_i2c_clk_in),     
        .io(BATT_I2C_CLK_IO),    
        .i(batt_i2c_clk_out),    
        .oe(batt_i2c_clk_tri)     
    );
   
   
    ALT_IOBUF batt_i2c_dat_buf (
        .o(batt_i2c_dat_in),     
        .io(BATT_I2C_DAT_IO),    
        .i(batt_i2c_dat_out),    
        .oe(batt_i2c_dat_tri)     
    );
  
    always @(posedge CLK) 
    begin
        if (RESET) 
            batt_i2c_reset <= 1'b0;
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_I2C_RESET) 
            batt_i2c_reset <= 1'b1;
        else
            batt_i2c_reset <= 1'b0;
    end
   
    always @(posedge CLK) begin
        if (RESET | batt_i2c_reset) begin
            batt_i2c_rdwr     <= 1'b0;
            batt_i2c_start    <= 1'b0;
            batt_i2c_stop     <= 1'b0;
            batt_i2c_rdnack   <= 1'b0;
            batt_i2c_addr     <= 0;
            batt_i2c_len      <= 0;
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_I2C_CTL) begin
            batt_i2c_rdwr   <= SYS_DATA_I[`CB_I2C_RDWR];
            batt_i2c_start  <= SYS_DATA_I[`CB_I2C_START];
            batt_i2c_stop   <= SYS_DATA_I[`CB_I2C_STOP];
            batt_i2c_rdnack <= SYS_DATA_I[`CB_I2C_RDNACK];
            batt_i2c_addr   <= SYS_DATA_I[`CB_I2C_ADDR];
            batt_i2c_len    <= SYS_DATA_I[`CB_I2C_LEN];
    end
        if (RESET | batt_i2c_reset | batt_i2c_done | batt_i2c_data_err_reg | batt_i2c_addr_err_reg) begin
            batt_i2c_go <= 1'b0;
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_I2C_CTL) begin
            batt_i2c_go <= SYS_DATA_I[`CB_I2C_GO];
        end	    
        // HW semaphore
        // first read returns '1', subsequent reads return '0'
        // SW must set by writing '1'
        if (RESET | batt_i2c_reset | (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_I2C_CTL && SYS_DATA_I[`CB_I2C_SEM])) begin
            batt_i2c_sem <= 1'b1;
        end
        else if (range_8000_re && SYS_ADDR_I[23:0] == `CA_BATT_I2C_CTL) begin
            batt_i2c_sem <= 1'b0;
        end	    
        // HW mutex 
        // The lsb of the register is the lock bit.
        // If the lock bit is set, only a write with the ID stored in the mutex and a lock bit of zero, will unlock the mutex and set the contents to zero.
        // The mutex will only update if the lock bit stored in the mutex is zero.
        if (RESET) begin
            batt_i2c_mutex <= 0;
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_I2C_MUTEX && SYS_DATA_I[`CB_I2C_MUTEX_ID] == batt_i2c_mutex[`CB_I2C_MUTEX_ID]) begin
            batt_i2c_mutex <= SYS_DATA_I[8:0];
        end
        else if (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_I2C_MUTEX && batt_i2c_mutex[`CB_I2C_MUTEX_LOCK] == 1'b0) begin
            batt_i2c_mutex <= SYS_DATA_I[8:0];
        end
        else begin
            batt_i2c_mutex <= batt_i2c_mutex;
        end 
        // address error flag
        if (RESET | batt_i2c_reset | (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_I2C_CTL && SYS_DATA_I[`CB_I2C_ADDR_ERR])) begin
            batt_i2c_addr_err_reg <= 1'b0;
        end
        else if (batt_i2c_addr_err) begin
            batt_i2c_addr_err_reg <= 1'b1;
        end 
        // data error flag
        if (RESET | batt_i2c_reset | (range_8000_we && SYS_ADDR_I[23:0] == `CA_BATT_I2C_CTL && SYS_DATA_I[`CB_I2C_DATA_ERR])) begin
            batt_i2c_data_err_reg <= 1'b0;
        end
        else if (batt_i2c_data_err) begin
            batt_i2c_data_err_reg <= 1'b1;
        end  
        // write data
        if (RESET | batt_i2c_reset) begin
            batt_i2c_data <= 0;
        end
        else if (batt_i2c_go & batt_i2c_done & batt_i2c_rdwr) begin
            batt_i2c_data <= batt_i2c_rd_data;
        end
        else if (range_8000_we) begin
            case (SYS_ADDR_I[23:0])
                `CA_BATT_I2C_DAT_W0 :  batt_i2c_data[ 31:  0] <= SYS_DATA_I;
                `CA_BATT_I2C_DAT_W1 :  batt_i2c_data[ 63: 32] <= SYS_DATA_I;
                `CA_BATT_I2C_DAT_W2 :  batt_i2c_data[ 95: 64] <= SYS_DATA_I;
                `CA_BATT_I2C_DAT_W3 :  batt_i2c_data[127: 96] <= SYS_DATA_I;
            endcase
        end   
    end // always @ (posedge CLK)

    // deglitcher
    deglitcher
       #(
        .c_width          (5)
       )
    batt_scl_dg (
        .RST               (RESET | batt_i2c_reset),
        .CLK               (CLK),
        .DIN               (batt_i2c_clk_in),
        .DOUT              (batt_i2c_clk_dg)
    );

    deglitcher 
       #(
       .c_width          (5)
       )
    batt_sda_dg   
      (
        .RST               (RESET | batt_i2c_reset),
        .CLK               (CLK),
        .DIN               (batt_i2c_dat_in),
        .DOUT              (batt_i2c_dat_dg)
      );

    i2c batt_i2c_i (
        .clk             (CLK),
        .reset           (RESET | batt_i2c_reset),
        .I2cRdWr         (batt_i2c_rdwr),
        .WrDat           ({144'h000000000000000000000000000000000000, batt_i2c_data}),
        .RdDat           (batt_i2c_rd_data),
        .I2cAddr         (batt_i2c_addr),
        .I2cLen          (batt_i2c_len),
        .I2cGo           (batt_i2c_go),
        .I2cDone         (batt_i2c_done),
        .I2cStart        (batt_i2c_start),
        .I2cStop         (batt_i2c_stop),
        .I2cRdNack       (batt_i2c_rdnack),
        .I2cAddrErr      (batt_i2c_addr_err),
        .I2cDatErr       (batt_i2c_data_err),
        .i2c_scl_out     (batt_i2c_clk_out),
        .i2c_scl_in      (batt_i2c_clk_dg),
        .i2c_scl_tri     (batt_i2c_clk_tri),
        .i2c_sda_out     (batt_i2c_dat_out),
        .i2c_sda_in      (batt_i2c_dat_dg),
        .i2c_sda_tri     (batt_i2c_dat_tri),
        .i2c_en          (1'b1)
    );                   
  /* --------------------------- GX status monitoring ----------------------- */

   // GX links receivers reset
   // assert reset after the NEON reset to prevent FIFO over/underruns
      
    always @(posedge CLK) neon_reset_n_p <= NEON_RST_N_O;

    always @(posedge CLK) begin
        if (RESET | (NEON_RST_N_O & ~neon_reset_n_p)) begin
            gx_init_cnt <= 1;
        end
        else if (ONE_USEC_TICK_I) begin
            if (gx_init_cnt != 0) begin
                gx_init_cnt <= gx_init_cnt + 1;
            end
        end
    end

    assign gx_rx_rst_hw = gx_init_cnt == 16'h2710;   // 10mS
   
    always @(posedge CLK) begin
        if (RESET | (NEON_RST_N_O & ~neon_reset_n_p)) 
            gx_rx_hw_ready <= 1'b0;
        else if (gx_init_cnt == 16'h2AF8)  // 11mS
            gx_rx_hw_ready <= 1'b1;
    end

    always @(posedge CLK) begin
        if (RESET) begin
            gx_status_clr  <= 0;
            gx_status_mask <= 20'hfffff;
            gx_int         <= 0;
            gx_txdiffctrl  <= 4'b0101;  // 561mV default
            gx_txdeemph    <= 1'b0;
        end
        else begin
        // clear register is not sticky
        // force clear on initialization
            if (gx_init_cnt == 16'hffff)
                gx_status_clr <= 20'hfffff;
            else
                gx_status_clr <= 0;  
        // set GX monitor interrupt
            gx_int <= |({gx_status_mask[1:0],gx_status_mask[19:12]} & gx_status);
        // write to registers
            if (range_8000_we) begin
                case (SYS_ADDR_I[23:0])
                    `CA_GX_STATUS : gx_status_clr  <= SYS_DATA_I;
                    `CA_GX_MASK   : gx_status_mask <= SYS_DATA_I;
                    `CA_GX_TXDIFFCTRL   : begin
                                            gx_txdiffctrl <= SYS_DATA_I[3:0];
                                            gx_txdeemph   <= SYS_DATA_I[4];
                                        end
                endcase
            end
        end // else: !if(RESET)
   end // always @ (posedge CLK)
  
  // Status in/out of gx_status block format is {Neon1,Neon0,RFdata[7:0]}
  // The bits get rearranged in reg_out
   gx_status
    #(
        .C_NUM_CHAN ( 10 )
    )
    gx_status_inst(
        .CLK             (CLK),
        .RESET           (RESET),
        .STATUS_CLR_I    ({gx_status_clr[19:12],gx_status_clr[1:0]}),
        .RXLOSSOFSYNC_I  (GX_RXLOSSOFSYNC_I),
        .PLLLKDET_I      (GX_PLLLKDET_I),
        .IDLE_ERR_I      (GX_IDLE_ERR_I),
        .CRC_ERR_I       (GX_CRC_ERR_I),
        .CRC_VAL_I       (GX_CRC_VAL_I),
        .GX_STATUS_O     (gx_status),
        .RX_SYNC_O       (gx_rx_sync),
        .PLL_LOCK_O      (gx_pll_lock),
        .IDLE_ERR_CNT_O  (gx_idle_err_cnt),
        .CRC_ERR_CNT_O   (gx_crc_err_cnt),
        .FRAME_CNT_O     (gx_frame_cnt),
        .CS_DATA_O       ()
    );

  /* --------------------------- Frame sequencer power-on reset ----------------------- */
    always @(posedge CLK) begin
        if (RESET) begin
            fseq_por_cnt <= 1;
        end
        else if (ONE_USEC_TICK_I) begin
            if (fseq_por_cnt != 0) begin
                fseq_por_cnt <= fseq_por_cnt + 1;
            end
        end
    end
    always @(posedge CLK) fseq_por_rst <= |fseq_por_cnt;       
    // PCIe LED status
    always @(posedge CLK) begin   
        if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_PCIE_STATUS))
            pcie_status  <= SYS_DATA_I;    
        if (PCIE_LED_PULSE_I) begin
        if (pcie_status[`CB_PCIE_LINK_UP]) begin
         // 4x PCIe link
        if (pcie_status[`CB_PCIE_LINK_WIDTH] == 3'b100) begin
            case (pcie_led)
                4'b1110 : pcie_led <= 4'b1101;
                4'b1101 : pcie_led <= 4'b1011;
                4'b1011 : pcie_led <= 4'b0111;
                4'b0111 : pcie_led <= 4'b1110;
                default : pcie_led <= 4'b1110;
            endcase // case(pcie_led)
        end
         // 2x PCIe link
        else if (pcie_status[`CB_PCIE_LINK_WIDTH] == 3'b010) begin
            case (pcie_led)
                4'b1110 : pcie_led <= 4'b1101;
                4'b1101 : pcie_led <= 4'b1110;
                default : pcie_led <= 4'b1110;
            endcase // case(pcie_led)
        end
           // 1x PCIe link
            else begin
            case (pcie_led)
                4'b1110 : pcie_led <= 4'b1111;
                4'b1111 : pcie_led <= 4'b1110;
                default : pcie_led <= 4'b1110;
            endcase // case(pcie_led)
           end
         end
        else begin
        case (pcie_led)
            4'b0000 : pcie_led <= 4'b1111;
            4'b1111 : pcie_led <= 4'b0000;
        default : pcie_led <= 4'b0000;
            endcase // case(pcie_led)
            end
        end
    end

   // scan DMA error monitor
    always @(posedge CLK) begin
        if (RESET) begin
            sdma_par_err  <= 0;
            sdma_ser_err  <= 0;
            sdma_par_fifo_count <= 0;
            sdma_ser_fifo_count <= 0;
        end
        else begin
       // clear the error condition
            if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_SDMA_DEBUG)) begin
                sdma_par_err  <= ~SYS_DATA_I[`CB_SDMA_PAR_ERROR];
                sdma_ser_err  <= ~SYS_DATA_I[`CB_SDMA_SER_ERROR];
                sdma_par_fifo_count  <= SYS_DATA_I[`CB_SDMA_PAR_FIFO_CNT] == 12'hfff ? 0 : sdma_par_fifo_count;
                sdma_ser_fifo_count  <= SYS_DATA_I[`CB_SDMA_SER_FIFO_CNT] == 12'hfff ? 0 : sdma_ser_fifo_count;
            end
            else begin
                sdma_par_err  <= SDMA_STATUS_I[`CB_SDMA_PAR_ERROR] | sdma_par_err;
                sdma_ser_err  <= SDMA_STATUS_I[`CB_SDMA_SER_ERROR] | sdma_ser_err;
                sdma_par_fifo_count  <= SDMA_STATUS_I[`CB_SDMA_PAR_FIFO_CNT] >= sdma_par_fifo_count ? {3'b000, SDMA_STATUS_I[`CB_SDMA_PAR_FIFO_CNT]} : sdma_par_fifo_count;
                sdma_ser_fifo_count  <= SDMA_STATUS_I[`CB_SDMA_SER_FIFO_CNT] >= sdma_ser_fifo_count ? {3'b000, SDMA_STATUS_I[`CB_SDMA_SER_FIFO_CNT]} : sdma_ser_fifo_count;
            end
       
        end // else: !if(RESET)
    end // always @ (posedge CLK)

   // AML manual trigger
    always @(posedge CLK) begin
        if (RESET) begin
            aml_pulse_cnt   <= 0;
            AML_TRIGGER_O   <= 1'b1;
        end
        else begin
            if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_AML_LINE_TRIG)) begin
                aml_pulse_cnt  <= 0;
                AML_TRIGGER_O  <= 1'b0;
            end
            else if (aml_pulse_cnt == AML_TRIGGER_LEN) begin
                AML_TRIGGER_O  <= 1'b1;
            end
            else begin
                aml_pulse_cnt  <= aml_pulse_cnt + 1;
            end
        end // else: !if(RESET)
    end // always @ (posedge CLK)
 
   // AML output multiplexer
    always @(posedge CLK) begin
        if (RESET) 
		    AML_TRIGGER_DIS_O <= 2'b00;
        else if (range_8000_we && (SYS_ADDR_I[23:0] == `CA_AML_TRIG_DIS)) 
		    AML_TRIGGER_DIS_O <= SYS_DATA_I[`CB_AML_TRIG_DIS];
    end    
  //  AIO USB hub power enable cntrl.   (Reset control)

    reg [7:0] sw_aio_reset_pulse_p;
    always @(posedge CLK) begin
        if (RESET) 
            sw_aio_reset_pulse_p   <= 0;
        else 
            sw_aio_reset_pulse_p <= {sw_aio_reset_pulse_p[6:0], sw_aio_reset_pulse};
    end
     
    wire sw_aio_reset_pulse_sig;
    assign   sw_aio_reset_pulse_sig = |sw_aio_reset_pulse_p;

    always @(posedge AIO_CLK)
    begin
        if (~PRH_PLL_LOCKED_I || sw_aio_reset_pulse_sig)   
            hub_reset_active_cnt <= 0;
        else 
            hub_reset_active_cnt <= hub_reset_active_cnt + 1'b1;  
    end

    always @(posedge AIO_CLK)
    begin
        if (~PRH_PLL_LOCKED_I || sw_aio_reset_pulse_sig)
            aio_usb_hub_pwr_en <= 0;
        else begin
            if (hub_reset_active_cnt == aio_usb_hub_pwr_en_tc)     
                aio_usb_hub_pwr_en <= 1'b1;
            end   
    end
      
    assign AIO_USB_HUB_PWR_EN_O = aio_usb_hub_pwr_en && ~sw_aio_usb_hub_pwr_dis;
// ****  iButton interface *******

   //  create ibutton 10Mhz clock
    always @(posedge CLK) begin
        if (RESET)
           ten_mhz_cnt <= 0;
     else
           ten_mhz_cnt <= ten_mhz_cnt + 1'b1;   
     end
  // reset iButton any time a readback occurs   
    //assign ibutton_restart_sw = range_8000_cs &&  ((SYS_ADDR_I[23:0] == `CA_iBUTTON_STATUS) || (SYS_ADDR_I[23:0] == `CA_iBUTTON_DATA_LOWER) || (SYS_ADDR_I[23:0] == `CA_iBUTTON_DATA_UPPER));    
    assign ibutton_restart_sw = range_8000_we &&  ((SYS_ADDR_I[23:0] == `CA_iBUTTON_STATUS) || (SYS_ADDR_I[23:0] == `CA_iBUTTON_DATA_LOWER) || (SYS_ADDR_I[23:0] == `CA_iBUTTON_DATA_UPPER));
       
    always @ (posedge CLK) begin   //****clock was 20mHz on helium***
        if (RESET)
            ibutton_restart_sr <= 0;   
        else if (ibutton_restart_sw)  
            ibutton_restart_sr <= 16'hFFFF;   
        else
            ibutton_restart_sr <= {1'b0, ibutton_restart_sr[15:1]};   
    end
  
  // iButton instance 
    iButton iButton_inst  (
        .RESET                       (RESET),    //async reset 
        .CLK_10MHz                   (ten_mhz_cnt[2]),  //10MHz hence the cleaver register name
        .RESTART_i                   (ibutton_restart_sr[0]),   // extened reset to match original SVIO code
        .INTERFACE_STATUS            (ibutton_status),
        .IBUTTON_BUS                 (iBUTTON_IO),   
        .IBUTTON_DATA_o              (ibutton_data)    // 64 bit output
    );  
         
    assign CS_DATA_O[ 31:  0] = SYS_ADDR_I;
    assign CS_DATA_O[ 63: 32] = SYS_DATA_I;
    assign CS_DATA_O[ 95: 64] = SYS_DATA_O;
    assign CS_DATA_O[     96] = SYS_CS_n_I;
    assign CS_DATA_O[     97] = SYS_RNW_I;
    assign CS_DATA_O[     98] = SYS_RDY_O;
    assign CS_DATA_O[     99] = BP_RST_O;
    assign CS_DATA_O[    100] = BP_INT_O;
    assign CS_DATA_O[    101] = HOST_INT_O;
    assign CS_DATA_O[    102] = DEADMAN_SHUTDOWN_O;
    assign CS_DATA_O[    103] = XMIT_STOP_O;
    assign CS_DATA_O[    104] = RWAVE_INT_I;
    assign CS_DATA_O[    105] = LINE_TIMER_EN_O;
    assign CS_DATA_O[    106] = FSEQ_INT_I;
    assign CS_DATA_O[    107] = deadman_shutdown_p1;
    assign CS_DATA_O[    108] = deadman_shutdown_en;
    assign CS_DATA_O[    109] = deadman_shutdown_int;
    
    assign CS_DATA_O[    110] = batt_i2c_clk_out;
    assign CS_DATA_O[    111] = batt_i2c_clk_in;
    assign CS_DATA_O[    112] = batt_i2c_clk_tri;
    assign CS_DATA_O[    113] = batt_i2c_dat_out;
    assign CS_DATA_O[    114] = batt_i2c_dat_in;
    assign CS_DATA_O[    115] = batt_i2c_dat_tri;
    
    assign CS_DATA_O[    116] = diag_i2c_clk_out;
    assign CS_DATA_O[    117] = diag_i2c_clk_in;
    assign CS_DATA_O[    118] = diag_i2c_clk_tri;
    assign CS_DATA_O[    119] = diag_i2c_dat_out;
    assign CS_DATA_O[    120] = diag_i2c_dat_in;
    assign CS_DATA_O[    121] = diag_i2c_dat_tri;
       
    assign CS_DATA_O[    122] = util_i2c_clk_out;
    assign CS_DATA_O[    123] = util_i2c_clk_in;
    assign CS_DATA_O[    124] = util_i2c_clk_tri;
    assign CS_DATA_O[    125] = util_i2c_dat_out;
    assign CS_DATA_O[    126] = util_i2c_dat_in;
    assign CS_DATA_O[    127] = util_i2c_dat_tri;
    
    assign CS_DATA_O[141:138] = rwave_p;
    assign CS_DATA_O[    142] = bp_int;
    assign CS_DATA_O[    143] = rwave_int;
    assign CS_DATA_O[175:144] = bp_int_en;
    assign CS_DATA_O[207:176] = bp_int_stat;
    assign CS_DATA_O[213:208] = host_int_en;
    assign CS_DATA_O[219:214] = brd_host_int_stat;
    assign CS_DATA_O[231:220] = host_int_p;
    assign CS_DATA_O[    232] = AC_PRESENT_I;
    assign CS_DATA_O[    233] = host_int;
    assign CS_DATA_O[    234] = host_int_out;
    assign CS_DATA_O[251:236] = motor_control;
    assign CS_DATA_O[    252] = SYS_RNW_I;
    assign CS_DATA_O[    253] = SYS_RNW_I;
    assign CS_DATA_O[    254] = SYS_RNW_I;
    assign CS_DATA_O[    255] = SYS_RNW_I;

endmodule
