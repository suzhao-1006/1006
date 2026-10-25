# 1006
代码优化
====================================================================================================
 赛题四 · FPGA 实时图像边缘检测（SC2210 相机版）—— 完整 RTL 源码 已修改一部分，123，123
====================================================================================================
 器件      : Efinix Titanium Ti60F225 (C4)
 工具      : Efinity 2025.1.110.5.9
 顶层      : example_top
 工程文件  : D:\eff\efx_sc2210\Ti60_Demo.xml
 生成时间  : 2026-10-25 16:59:37
 内容来源  : 工程 design_file 列表里的 29 个文件，逐字拼接，未做任何改写

 算法链（本赛题原创部分）：
   src/edge_proc.v   算法链顶层（win3x3_stream 行缓存 / edge_chain 单通道 / edge_proc 双通道 / split_compose 分屏）
   src/rgb2gray.v    BT.601 灰度，移位加法，无除法器
   src/median3x3.v   3×3 中值滤波（含 sort3）
   src/sobel3x3.v    3×3 Sobel，|Gx|+|Gy|，阈值可配

 ★ 未在顶层例化的文件（编译但没用上）：
   - src/ddr_rw_ctrl.v
   - src/lcd_display.v
   - src/Sensor_Image_XYCrop.v
   - src/dsi/dsi_init.v
   - src/lvds/LCDDual2LVDS.v
   - src/cmos_i2c/i2c_timing_ctrl_16bit.v
   - src/cmos_i2c/i2c_timing_ctrl_reg16_dat16.v
   - src/cmos_i2c/CMOS_Capture_RAW_Gray.v
   - src/cmos_i2c/I2C_MT9M001_Gray_Config.v
   - src/cmos_i2c/I2C_AR0135_1280720_Config.v
   - src/cmos_i2c/I2C_SC130GS_12801024_4Lanes_Config.v

 ★ 约束/工程文件（不属于 RTL，单独提交）：
   Ti60_Demo.peri.xml   引脚与 IO 约束（IO bank 电压 / PLL / MIPI D-PHY / LVDS TX / GPIO）
   Ti60_Demo.pt.sdc     时序约束（clk_sys 96MHz、clk_pixel_2x 148.5MHz、clk_pixel_10x 744MHz、csi_rxc_i 6ns 等）
   Ti60_Demo.xml        Efinity 工程文件
   ip/                   Efinity IP（csi_rx / DdrCtrl / 8 个 FIFO），vendor 提供，不在此文件内

----------------------------------------------------------------------------------------------------
 目录
----------------------------------------------------------------------------------------------------
  [ 1] example_top.v
  [ 2] src/edge_proc.v
  [ 3] src/rgb2gray.v
  [ 4] src/median3x3.v
  [ 5] src/sobel3x3.v
  [ 6] src/axi/axi4_ctrl.v
  [ 7] src/axi/AXI4_AWARMux.v
  [ 8] src/lcd_driver.v
  [ 9] src/lcd_para.v
  [10] src/isp/VIP_RAW8_RGB888.v
  [11] src/isp/VIP_Matrix_Generate_3X3_8Bit.v
  [12] src/isp/Line_Shift_RAM_8Bit.v
  [13] src/hdmi_ip/rgb2dvi.v
  [14] src/hdmi_ip/tmds_channel.v
  [15] src/PWMLite.v
  [16] pri/mipi/MIPIRx4LaneFre.v
  [17] src/cmos_i2c/I2C_SC2210_19201080_4Lanes_Config.v
  [18] src/cmos_i2c/i2c_timing_ctrl_reg16_dat8_wronly.v
  [19] src/cmos_i2c/CMOS_Capture_RAW_Gray.v
  [20] src/cmos_i2c/I2C_AR0135_1280720_Config.v
  [21] src/cmos_i2c/I2C_MT9M001_Gray_Config.v
  [22] src/cmos_i2c/I2C_SC130GS_12801024_4Lanes_Config.v
  [23] src/cmos_i2c/i2c_timing_ctrl_16bit.v
  [24] src/cmos_i2c/i2c_timing_ctrl_reg16_dat16.v
  [25] src/ddr_rw_ctrl.v
  [26] src/dsi/dsi_init.v
  [27] src/lcd_display.v
  [28] src/lvds/LCDDual2LVDS.v
  [29] src/Sensor_Image_XYCrop.v

====================================================================================================
 [ 1/29] example_top.v    (1322 行, 已例化)
====================================================================================================
`timescale 1ns/1ps

//`include "ddr3_controller.vh"


module example_top 
(
	////////////////////////////////////////////////////////////////
	//	External Clock & Reset
	//input 			nrst, 			//	Button K2
	input 			clk_24m,			//	24MHz Crystal
	input 			clk_25m,			//	25MHz Crystal 
	
	
	////////////////////////////////////////////////////////////////
	//	System Clock
	output 			sys_pll_rstn_o, 		
	
	input 			clk_sys,			//	Sys PLL 96MHz 
	input 			clk_pixel,			//	Sys PLL 74.25MHz
	input 			clk_pixel_2x,		//	Sys PLL 148.5MHz
	input 			clk_pixel_10x,		//	Sys PLL 742.5MHz
	
	input 			sys_pll_lock,		//	Sys PLL Lock
	
	////////////////////////////////////////////////////////////////
	//	MIPI-DSI Clock & Reset
	output 			dsi_pll_rstn_o,
	
	input 			dsi_refclk_i,		//	48MHz Reference Clock (for DSI PLL)
	input 			dsi_byteclk_i,		//	DSI Byte Clock (1X)
	input 			dsi_serclk_i,		//	DSI Serial Clock (4X 45)
	input 			dsi_txcclk_i,		//	DSI Serial Clock (4X 135)
	
	input 			dsi_pll_lock,
	
	////////////////////////////////////////////////////////////////
	//	DDR Clock
	output 			ddr_pll_rstn_o, 
	
	input 			tdqss_clk,			
	input 			core_clk,			//	DDR PLL 200MHz
	input 			tac_clk,			
	input 			twd_clk,			
	
	input 			ddr_pll_lock,		//	DDR PLL Lock
	
	////////////////////////////////////////////////////////////////
	//	DDR PLL Phase Shift Interface
	output 	[2:0] 	shift,
	output 	[4:0] 	shift_sel,
	output 			shift_ena,
	
	
	
	////////////////////////////////////////////////////////////////
	//	LVDS Clock
	output 			lvds_pll_rstn_o, 
	
	input 			clk_lvds_1x, 
	input 			clk_lvds_7x, 
	input 			clk_27m, 			//	RGB 1X Clock (16MHz)
	input 			clk_54m, 			//	RGB 2X Clock (32MHz, for export control)
	
	input 			lvds_pll_lock, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	DDR Interface Ports
	output 	[15:0] 	addr,
	output 	[2:0] 	ba,
	output 			we,
	output 			reset,
	output 			ras,
	output 			cas,
	output 			odt,
	output 			cke,
	output 			cs,
	
	//	DQ I/O
	input 	[15:0] 	i_dq_hi,
	input 	[15:0] 	i_dq_lo,
	
	output 	[15:0] 	o_dq_hi,
	output 	[15:0] 	o_dq_lo,
	output 	[15:0] 	o_dq_oe,
	
	//	DM O
	output 	[1:0] 	o_dm_hi,
	output 	[1:0] 	o_dm_lo,
	
	//	DQS I/O
	input 	[1:0] 	i_dqs_hi,
	input 	[1:0] 	i_dqs_lo,
	
//	input 	[1:0] 	i_dqs_n_hi,
//	input 	[1:0] 	i_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_hi,
	output 	[1:0] 	o_dqs_lo,
	
//	output 	[1:0] 	o_dqs_n_hi,
//	output 	[1:0] 	o_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_oe,
	output 	[1:0] 	o_dqs_n_oe,
	
	//	CK
	output 			clk_p_hi, 
	output 			clk_p_lo, 
	output 			clk_n_hi, 
	output 			clk_n_lo, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	MIPI-CSI Ctl / I2C
	output 			csi_ctl0_o,
	output 			csi_ctl0_oe,
	input 			csi_ctl0_i,
	
	output 			csi_ctl1_o,
	output 			csi_ctl1_oe,
	input 			csi_ctl1_i,
	
	output 			csi_scl_o,
	output 			csi_scl_oe,
	input 			csi_scl_i,
	
	output 			csi_sda_o,
	output 			csi_sda_oe,
	input 			csi_sda_i,
	
	//	MIPI-CSI RXC 
	input 			csi_rxc_lp_p_i,
	input 			csi_rxc_lp_n_i,
	output 			csi_rxc_hs_en_o,
	output 			csi_rxc_hs_term_en_o,
	input 			csi_rxc_i,
	
	//	MIPI-CSI RXD0
	output 			csi_rxd0_rst_o,
	output 			csi_rxd0_hs_en_o,
	output 			csi_rxd0_hs_term_en_o,
	
	input 			csi_rxd0_lp_p_i,
	input 			csi_rxd0_lp_n_i,
	input 	[7:0] 	csi_rxd0_hs_i,
	
	//	MIPI-CSI RXD1
	output 			csi_rxd1_rst_o,
	output 			csi_rxd1_hs_en_o,
	output 			csi_rxd1_hs_term_en_o,
	
	input 			csi_rxd1_lp_n_i,
	input 			csi_rxd1_lp_p_i,
	input 	[7:0] 	csi_rxd1_hs_i,
	
	//	MIPI-CSI RXD2
	output 			csi_rxd2_rst_o,
	output 			csi_rxd2_hs_en_o,
	output 			csi_rxd2_hs_term_en_o,
	
	input 			csi_rxd2_lp_p_i,
	input 			csi_rxd2_lp_n_i,
	input 	[7:0] 	csi_rxd2_hs_i,
	
	//	MIPI-CSI RXD3
	output 			csi_rxd3_rst_o,
	output 			csi_rxd3_hs_en_o,
	output 			csi_rxd3_hs_term_en_o,
	
	input 			csi_rxd3_lp_p_i,
	input 			csi_rxd3_lp_n_i,
	input 	[7:0] 	csi_rxd3_hs_i,
	
	//output 			csi_rxd0_fifo_rd_o, 
	//input 			csi_rxd0_fifo_empty_i, 
	//output 			csi_rxd1_fifo_rd_o, 
	//input 			csi_rxd1_fifo_empty_i, 
	//output 			csi_rxd2_fifo_rd_o, 
	//input 			csi_rxd2_fifo_empty_i, 
	//output 			csi_rxd3_fifo_rd_o, 
	//input 			csi_rxd3_fifo_empty_i, 
	
	
	
	////////////////////////////////////////////////////////////////
	//	DSI PWM & Reset Control 
	output 			dsi_pwm_o,			//	MIPI-DSI LCD PWM
	output 			dsi_resetn_o,		//	MIPI-DSI LCD Reset
	
	//	MIPI-DSI TXC / TXD
	output 			dsi_txc_rst_o,
	output 			dsi_txc_lp_p_oe,
	output 			dsi_txc_lp_p_o,
	output 			dsi_txc_lp_n_oe,
	output 			dsi_txc_lp_n_o,
	output 			dsi_txc_hs_oe,
	output 	[7:0] 	dsi_txc_hs_o,
	
	output 			dsi_txd0_rst_o,
	output 			dsi_txd0_hs_oe,
	output 	[7:0] 	dsi_txd0_hs_o,
	output 			dsi_txd0_lp_p_oe,
	output 			dsi_txd0_lp_p_o,
	output 			dsi_txd0_lp_n_oe,
	output 			dsi_txd0_lp_n_o,
	
	output 			dsi_txd1_rst_o,
	output 			dsi_txd1_lp_p_oe,
	output 			dsi_txd1_lp_p_o,
	output 			dsi_txd1_lp_n_oe,
	output 			dsi_txd1_lp_n_o,
	output 			dsi_txd1_hs_oe,
	output 	[7:0] 	dsi_txd1_hs_o,
	
	output 			dsi_txd2_rst_o,
	output 			dsi_txd2_lp_p_oe,
	output 			dsi_txd2_lp_p_o,
	output 			dsi_txd2_lp_n_oe,
	output 			dsi_txd2_lp_n_o,
	output 			dsi_txd2_hs_oe,
	output 	[7:0] 	dsi_txd2_hs_o,
	
	output 			dsi_txd3_rst_o,
	output 			dsi_txd3_lp_p_oe,
	output 			dsi_txd3_lp_p_o,
	output 			dsi_txd3_lp_n_oe,
	output 			dsi_txd3_lp_n_o,
	output 			dsi_txd3_hs_oe,
	output 	[7:0] 	dsi_txd3_hs_o,
	
	input 			dsi_txd0_lp_p_i,
	input 			dsi_txd0_lp_n_i,
	input 			dsi_txd1_lp_p_i,
	input 			dsi_txd1_lp_n_i,
	input 			dsi_txd2_lp_p_i,
	input 			dsi_txd2_lp_n_i,
	input 			dsi_txd3_lp_p_i,
	input 			dsi_txd3_lp_n_i,
	
	
	////////////////////////////////////////////////////////////////
	//	UART Interface
	input 		 	uart_rx_i,			//	Support 460800-8-N-1. 
	output 		 	uart_tx_o, 
	
	
	output 	[5:0] 	led_o,			//	
	
	
	////////////////////////////////////////////////////////////////
	//	CMOS Sensor
	output 			cmos_sclk,
	input 			cmos_sdat_IN,
	output 			cmos_sdat_OUT,
	output 			cmos_sdat_OE,
	
	//	CMOS Interface
	input 			cmos_pclk,
	input 			cmos_vsync,
	input 			cmos_href,
	input 	[7:0] 	cmos_data,
	input 			cmos_ctl1,
	output 			cmos_ctl2,
	output 			cmos_ctl3,
	
	
	////////////////////////////////////////////////////////////////
	//	HDMI Interface
	output 			hdmi_txc_oe,
	output 			hdmi_txd0_oe,
	output 			hdmi_txd1_oe,
	output 			hdmi_txd2_oe,
	
	output 			hdmi_txc_rst_o,
	output 			hdmi_txd0_rst_o,
	output 			hdmi_txd1_rst_o,
	output 			hdmi_txd2_rst_o,
	
	output 	[9:0] 	hdmi_txc_o,
	output 	[9:0] 	hdmi_txd0_o,
	output 	[9:0] 	hdmi_txd1_o,
	output 	[9:0] 	hdmi_txd2_o,
	
	
	////////////////////////////////////////////////////////////////
	//	LVDS Interface
	output 			lvds_txc_oe,
	output 	[6:0] 	lvds_txc_o,
	output 			lvds_txc_rst_o,
	
	output 			lvds_txd0_oe,
	output 	[6:0] 	lvds_txd0_o,
	output 			lvds_txd0_rst_o,
	
	output 			lvds_txd1_oe,
	output 	[6:0] 	lvds_txd1_o,
	output 			lvds_txd1_rst_o,
	
	output 			lvds_txd2_oe,
	output 	[6:0] 	lvds_txd2_o,
	output 			lvds_txd2_rst_o,
	
	output 			lvds_txd3_oe,
	output 	[6:0] 	lvds_txd3_o,
	output 			lvds_txd3_rst_o,
	
	
	////////////////////////////////////////////////////////////////
	//	RGB LCD 5Inch 800x480
	output 			lcd_tp_sda_o,		//	TP SDA
	output 			lcd_tp_sda_oe,
	input 			lcd_tp_sda_i,
	
	output 			lcd_tp_scl_o,		//	TP SCL
	output 			lcd_tp_scl_oe,
	input 			lcd_tp_scl_i,
	
	output 			lcd_tp_int_o,		//	TP INT
	output 			lcd_tp_int_oe,
	input 			lcd_tp_int_i,
	
	output 			lcd_tp_rst_o,		//	TP RST
	
	output 			lcd_pwm_o,			//	Backlight
	output 			lcd_blen_o,
	
	//output 			lcd_pclk_o,			//	PCLK & SCK Mux
	output 			lcd_vs_o,			//	VS & SSN Mux. Fixed to 1. Use DE-Only mode. 
	output 			lcd_hs_o,			//	HS. Fixed to 1. Use DE-Only mode. 
	output 			lcd_de_o,			//	DE. 

	output 	[7:0] 	lcd_b7_0_o,			//	B7:B0. 
	output 	[7:0] 	lcd_g7_0_o,			//	G7:G0. Must output 8'hFF when access SPI. 
	output 	[7:0] 	lcd_r7_0_o,			//	R7:R0. 
	
	output 	[7:0] 	lcd_b7_0_oe,		//	B7:B0. 
	output 	[7:0] 	lcd_g7_0_oe,		//	G7:G0. Must output 8'hFF when access SPI. 
	output 	[7:0] 	lcd_r7_0_oe,		//	R7:R0. 

	input 	[7:0] 	lcd_b7_0_i,			//	B7:B0. 
	input 	[7:0] 	lcd_g7_0_i,			//	G7:G0. Must output 8'hFF when access SPI. 
	input 	[7:0] 	lcd_r7_0_i,			//	R7:R0. 
	
	//	SPI Pins
	output 			spi_sck_o, 
	output 			spi_ssn_o 			
);
	
	wire 			csi_rxd0_fifo_rd_o; 
	wire 			csi_rxd0_fifo_empty_i = 0;  
	wire 			csi_rxd1_fifo_rd_o;  
	wire 			csi_rxd1_fifo_empty_i = 0;  
	wire 			csi_rxd2_fifo_rd_o;  
	wire 			csi_rxd2_fifo_empty_i = 0;  
	wire 			csi_rxd3_fifo_rd_o;  
	wire 			csi_rxd3_fifo_empty_i = 0;  
	
	wire 	[1:0] 	i_dqs_n_hi;
	wire 	[1:0] 	i_dqs_n_lo;
	wire 	[1:0] 	o_dqs_n_hi;
	wire 	[1:0] 	o_dqs_n_lo;
	
	
	parameter 	SIM_DATA 	= 0; 
	
	//	Hardware Configuration
	assign clk_p_hi = 1'b0;	//	DDR3 Clock requires 180 degree shifted. 
	assign clk_p_lo = 1'b1;
	assign clk_n_hi = 1'b1;
	assign clk_n_lo = 1'b0; 
	
	//	System Clock Tree Control
	assign sys_pll_rstn_o = 1'b1; 	//	nrst; 	//	Reset whole system when nrst (K2) is pressed. 
	
	assign dsi_pll_rstn_o = sys_pll_lock; 
	assign ddr_pll_rstn_o = sys_pll_lock; 
	assign lvds_pll_rstn_o = sys_pll_lock; 
	
	wire 			w_pll_lock = sys_pll_lock && dsi_pll_lock && ddr_pll_lock && lvds_pll_lock; 
	
	//	Synchronize System Resets. 
	reg 			rstn_sys = 0, rstn_pixel = 0; 
	wire 			rst_sys = ~rstn_sys, rst_pixel = ~rstn_pixel; 
	
	reg 			rstn_dsi_refclk = 0, rstn_dsi_byteclk = 0; 
	wire 			rst_dsi_refclk = ~rstn_dsi_refclk, rst_dsi_byteclk = ~rstn_dsi_byteclk; 
	
	reg 			rstn_lvds_1x = 0; 
	wire 			rst_lvds_1x = ~rstn_lvds_1x; 
	
	reg 			rstn_27m = 0, rstn_54m = 0; 
	wire 			rst_27m = ~rstn_27m, rst_54m = ~rstn_54m; 
	
	//	Clock Gen
	always @(posedge clk_27m or negedge w_pll_lock) begin if(~w_pll_lock) rstn_27m <= 0; else rstn_27m <= 1; end
	always @(posedge clk_54m or negedge w_pll_lock) begin if(~w_pll_lock) rstn_54m <= 0; else rstn_54m <= 1; end
	always @(posedge clk_sys or negedge w_pll_lock) begin if(~w_pll_lock) rstn_sys <= 0; else rstn_sys <= 1; end
	always @(posedge clk_pixel_2x or negedge w_pll_lock) begin if(~w_pll_lock) rstn_pixel <= 0; else rstn_pixel <= 1; end
	always @(posedge dsi_refclk_i or negedge w_pll_lock) begin if(~w_pll_lock) rstn_dsi_refclk <= 0; else rstn_dsi_refclk <= 1; end
	always @(posedge dsi_byteclk_i or negedge w_pll_lock) begin if(~w_pll_lock) rstn_dsi_byteclk <= 0; else rstn_dsi_byteclk <= 1; end
	always @(posedge clk_lvds_1x or negedge w_pll_lock) begin if(~w_pll_lock) rstn_lvds_1x <= 0; else rstn_lvds_1x <= 1; end
	
	
	localparam 	CLOCK_MAIN 	= 96000000; 	//	System clock using 96MHz. 
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	Flash Burner Control
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	wire 			w_ustick, w_mstick; 
	
	wire  [7:0] 	w_dev_index_o;  
	wire  [7:0] 	w_dev_cmd_o;  
	wire  [31:0] 	w_dev_wdata_o;  
	wire  		w_dev_wvalid_o;  
	wire  		w_dev_rvalid_o;  
	wire 	[31:0] 	w_dev_rdata_i;  
	
	wire 			w_spi_ssn_o, w_spi_sck_o; 
	wire 	[3:0] 	w_spi_data_o, w_spi_data_oe; 
	wire 	[3:0] 	w_spi_data_i; 
	
	//	Flash Control
	reg 			r_flash_en = 0; 		//	0x00:0x00 Enable Flash
	
	always @(posedge clk_sys) begin
		r_flash_en <= (w_dev_wvalid_o && (w_dev_index_o == 8'h00) && (w_dev_cmd_o == 8'h00)) ? w_dev_wdata_o : r_flash_en; 
	end
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	LCD Data Mux
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	wire 	[7:0] 	w_lcd_b_o, w_lcd_g_o, w_lcd_r_o; 
	
	assign lcd_b7_0_o = r_flash_en ? {4'b0, w_spi_data_o[3:2], 2'b0} : w_lcd_b_o; 
	assign lcd_g7_0_o = r_flash_en ? {6'h0, w_spi_data_o[1:0]} : w_lcd_g_o; 
	assign lcd_r7_0_o = r_flash_en ? {8'h00} : w_lcd_r_o; 
	
	assign lcd_b7_0_oe = r_flash_en ? {4'b0, w_spi_data_oe[3:2], 2'b0} : 8'hFF; 
	assign lcd_g7_0_oe = r_flash_en ? {6'h0, w_spi_data_oe[1:0]} : 8'hFF; 
	assign lcd_r7_0_oe = r_flash_en ? {8'h00} : 8'hFF; 
	
	assign spi_sck_o = w_spi_sck_o; 
	assign spi_ssn_o = w_spi_ssn_o; 
	assign w_spi_data_i = {lcd_b7_0_i[3:2], lcd_g7_0_i[1:0]}; 
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	DDR3 Controller
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	wire			w_ddr3_ui_clk = clk_sys;
	wire			w_ddr3_ui_rst = rst_sys;
	wire			w_ddr3_ui_areset = rst_sys;
	wire			w_ddr3_ui_aresetn = rstn_sys;
	

	//	General AXI Interface 
	wire	[3:0] 	w_ddr3_awid;
	wire	[31:0]	w_ddr3_awaddr;
	wire	[7:0]		w_ddr3_awlen;
	wire			w_ddr3_awvalid;
	wire			w_ddr3_awready;
	
	wire 	[3:0]  	w_ddr3_wid;
	wire 	[127:0] 	w_ddr3_wdata;
	wire 	[15:0]	w_ddr3_wstrb;
	wire			w_ddr3_wlast;
	wire			w_ddr3_wvalid;
	wire			w_ddr3_wready;
	
	wire 	[3:0] 	w_ddr3_bid;
	wire 	[1:0] 	w_ddr3_bresp;
	wire			w_ddr3_bvalid;
	wire			w_ddr3_bready;
	
	wire	[3:0] 	w_ddr3_arid;
	wire	[31:0]	w_ddr3_araddr;
	wire	[7:0]		w_ddr3_arlen;
	wire			w_ddr3_arvalid;
	wire			w_ddr3_arready;
	
	wire 	[3:0] 	w_ddr3_rid;
	wire 	[127:0] 	w_ddr3_rdata;
	wire			w_ddr3_rlast;
	wire			w_ddr3_rvalid;
	wire			w_ddr3_rready;
	wire 	[1:0] 	w_ddr3_rresp;
	
	
	//	AXI Interface Request
	wire 	[3:0] 	w_ddr3_aid;
	wire 	[31:0] 	w_ddr3_aaddr;
	wire 	[7:0]  	w_ddr3_alen;
	wire 	[2:0]  	w_ddr3_asize;
	wire 	[1:0]  	w_ddr3_aburst;
	wire 	[1:0]  	w_ddr3_alock;
	wire			w_ddr3_avalid;
	wire			w_ddr3_aready;
	wire			w_ddr3_atype;
	
	wire 			w_ddr3_cal_done, w_ddr3_cal_pass; 
	
	//	Do not issue DDR read / write when ~cal_done. 
	reg 			r_ddr_unlock = 0; 
	always @(posedge w_ddr3_ui_clk or negedge w_ddr3_ui_aresetn) begin
		if(~w_ddr3_ui_aresetn)
			r_ddr_unlock <= 0; 
		else
			r_ddr_unlock <= w_ddr3_cal_done; 
	end
	
	DdrCtrl ddr3_ctl_axi (	
		.core_clk		(core_clk),
		.tac_clk		(tac_clk),
		.twd_clk		(twd_clk),	
		.tdqss_clk		(tdqss_clk),
		
		.reset		(reset),
		.cs			(cs),
		.ras			(ras),
		.cas			(cas),
		.we			(we),
		.cke			(cke),    
		.addr			(addr),
		.ba			(ba),
		.odt			(odt),
		
		.o_dm_hi		(o_dm_hi),
		.o_dm_lo		(o_dm_lo),
		
		.i_dq_hi		(i_dq_hi),
		.i_dq_lo		(i_dq_lo),
		.o_dq_hi		(o_dq_hi),
		.o_dq_lo		(o_dq_lo),
		.o_dq_oe		(o_dq_oe),
		
		.i_dqs_hi		(i_dqs_hi),
		.i_dqs_lo		(i_dqs_lo),
		.i_dqs_n_hi		(i_dqs_n_hi),
		.i_dqs_n_lo		(i_dqs_n_lo),
		.o_dqs_hi		(o_dqs_hi),
		.o_dqs_lo		(o_dqs_lo),
		.o_dqs_n_hi		(o_dqs_n_hi),
		.o_dqs_n_lo		(o_dqs_n_lo),
		.o_dqs_oe		(o_dqs_oe),
		.o_dqs_n_oe		(),				//	o_dqs_n_oe),
		
		.clk			(w_ddr3_ui_clk),
		.reset_n		(w_ddr3_ui_aresetn),
		
		.axi_avalid		(w_ddr3_avalid),		//	Enable command only when unlocked. 
		.axi_aready		(w_ddr3_aready),
		.axi_aaddr		(w_ddr3_aaddr),
		.axi_aid		(w_ddr3_aid),
		.axi_alen		(w_ddr3_alen),
		.axi_asize		(w_ddr3_asize),
		.axi_aburst		(w_ddr3_aburst),
		.axi_alock		(w_ddr3_alock),
		.axi_atype		(w_ddr3_atype),
		
		.axi_wid		(w_ddr3_wid),
		.axi_wvalid		(w_ddr3_wvalid),
		.axi_wready		(w_ddr3_wready),
		.axi_wdata		(w_ddr3_wdata),
		.axi_wstrb		(w_ddr3_wstrb),
		.axi_wlast		(w_ddr3_wlast),
		
		.axi_bvalid		(w_ddr3_bvalid),
		.axi_bready		(w_ddr3_bready),
		.axi_bid		(w_ddr3_bid),
		.axi_bresp		(w_ddr3_bresp),
		
		.axi_rvalid		(w_ddr3_rvalid),
		.axi_rready		(w_ddr3_rready),
		.axi_rdata		(w_ddr3_rdata),
		.axi_rid		(w_ddr3_rid),
		.axi_rresp		(w_ddr3_rresp),
		.axi_rlast		(w_ddr3_rlast),
		
		.shift		(shift),
		.shift_sel		(),
		.shift_ena		(shift_ena),
		
		.cal_ena		(1'b1),
		.cal_done		(w_ddr3_cal_done),
		.cal_pass		(w_ddr3_cal_pass)
	);
	//	The o_dqs_n_oe may not assert at negative edge. 
	assign o_dqs_n_oe = o_dqs_oe; 
	
	assign w_ddr3_bready = 1'b1; 
	assign shift_sel = 5'b00100; 		//	ddr_tac_clk always use PLLOUT[2]. 
	
	
	AXI4_AWARMux #(.AID_LEN(4), .AADDR_LEN(32)) axi4_awar_mux (
		.aclk_i			(w_ddr3_ui_clk), 
		.arst_i			(w_ddr3_ui_rst), 
		
		.awid_i			(w_ddr3_awid),
		.awaddr_i			(w_ddr3_awaddr),
		.awlen_i			(w_ddr3_awlen),
		.awvalid_i			(w_ddr3_awvalid && r_ddr_unlock),
		.awready_o			(w_ddr3_awready),
		
		.arid_i			(w_ddr3_arid),
		.araddr_i			(w_ddr3_araddr),
		.arlen_i			(w_ddr3_arlen),
		.arvalid_i			(w_ddr3_arvalid && r_ddr_unlock),
		.arready_o			(w_ddr3_arready),
		
		.aid_o			(w_ddr3_aid),
		.aaddr_o			(w_ddr3_aaddr),
		.alen_o			(w_ddr3_alen),
		.atype_o			(w_ddr3_atype),
		.avalid_o			(w_ddr3_avalid),
		.aready_i			(w_ddr3_aready)
	);
	
	assign w_ddr3_asize = 4; 		//	Fixed 128 bits (16 bytes, size = 4)
	assign w_ddr3_aburst = 1; 
	assign w_ddr3_alock = 0; 
	
	// [EDGE_PATCH] led_o[1:0] driver removed (edge_proc drives led_o[5:1]) 
	
	
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////
	//	I2C Config (SC130GS)
	
	//  i2c timing controller module of 16Bit
	wire            [ 8:0]          sc2210_i2c_config_index;
	wire            [23:0]          sc2210_i2c_config_data;
	wire            [ 8:0]          sc2210_i2c_config_size;
	wire                            sc2210_i2c_config_done;
	
	i2c_timing_ctrl_reg16_dat8_wronly
	#(
	    .CLK_FREQ           (CLOCK_MAIN),                              //  100 MHz
	    .I2C_FREQ           (50_000    )                               //  10 KHz(<= 400KHz)
	) u_i2c_timing_ctrl_16bit (
	    //  global clock
	    .clk                (clk_sys                 ),                          //  96MHz
	    .rst_n              (rstn_sys                ),                          //  system reset

	    //  i2c interface
	    .i2c_sclk           (csi_scl_o               ),                          //  i2c clock
	    .i2c_sdat_IN        (csi_sda_i               ),
	    .i2c_sdat_OUT       (csi_sda_o               ),
	    .i2c_sdat_OE        (csi_sda_oe              ),

	    //  i2c config data
	    .i2c_config_index   (sc2210_i2c_config_index        ),                          //  i2c config reg index, read 2 reg and write xx reg
	    .i2c_config_data    ({8'h60, sc2210_i2c_config_data}),                     //  i2c config data
	    .i2c_config_size    (sc2210_i2c_config_size         ),                          //  i2c config data counte
	    .i2c_config_done    (sc2210_i2c_config_done         )                          //  i2c config timing complete
	);
	assign csi_scl_oe = 1; 

	//  I2C Configure Data of SC2210
	I2C_SC2210_19201080_4Lanes_Config u_I2C_SC2210_19201080_4Lanes_Config
	(
	    .LUT_INDEX  (sc2210_i2c_config_index   ),
	    .LUT_DATA   (sc2210_i2c_config_data    ),
	    .LUT_SIZE   (sc2210_i2c_config_size    )
	);
		
	
		
	//	CMOS Interface
	//input 			cmos_pclk,
	//input 			cmos_vsync,
	//input 			cmos_href,
	//input 	[7:0] 	cmos_data,
	//input 			cmos_ctl1,
	//output 			cmos_ctl2,
	//output 			cmos_ctl3,

	
	//	Output LED
	reg 	[3:0]		r_cmos_fv_o = 0; 
	reg 	[1:0] 	r_cmos_rx_vsync0_in = 0; 
	always @(posedge cmos_pclk) begin
		r_cmos_rx_vsync0_in <= {r_cmos_rx_vsync0_in, cmos_vsync}; 
		r_cmos_fv_o <= r_cmos_fv_o + ((r_cmos_rx_vsync0_in == 2'b01) ? 1 : 0); 
	end
	// [EDGE_PATCH] led_o[5] driver removed (edge_proc drives led_o[5:1]) 
	
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////
	//	System Control. Can be removed for public. 
	
	localparam 	CLK_FREQ 	= 96_000_000; 	//	clk_sys is 96MHz. 
	localparam 	BAUD_RATE 	= 460_800; 		//	Use 460800-8-N-1. 
	
	
	//	SFR I/O Interface
	wire 	[7:0] 	w_sfr_addr_o; 	//	SFR Address (0xFF00 ~ 0xFFFF). 00:Power; 40~5F:Stream0; 60~7F:Stream1. 
	wire 	[7:0] 	w_sfr_wdata_o; 	//	SFR Write Data. 
	wire 			w_sfr_we_o; 		//	SFR WE. 
	reg 	[7:0] 	w_sfr_rdata_i; 	//	Must be valid after sfr_rd_o. 
	wire 			w_sfr_rd_o; 		//	SFR RD. 
	
	
	//	System Control Registers
	reg 			r_dsi_tx_rstn = 0; 	//	DSI TX Reset
	reg 	[7:0] 	r_dsi_pwm = 64; 		//	[6:0]PWM, [7]Pol
	reg 			r_dsi_resetn_o = 0; 	//	DSI Panel Reset
	reg 			r_dsi_data_rstn = 0; 	//	DSI TX Reset
	
	reg 	[3:0] 	r_dsi_lp_p_ovr = 0; 	
	reg 	[3:0] 	r_dsi_lp_n_ovr = 0; 	
	
	
	
	//	AXI-Lite Interface Bridge
	localparam 	CSI_AXILITE_ID 	= 0; 				//	Select DSI_TX when r_axi_sel = DSI_AXILITE_ID. 
	localparam 	DSI_AXILITE_ID 	= 1; 				//	Select DSI_TX when r_axi_sel = DSI_AXILITE_ID. 
	
	reg 	[7:0] 	r_axi_addr = 8'h18; 		//	0xE0 (RW)
	reg 	[31:0] 	r_axi_wdata = 32'h0000000A; 	//	0xE1~0xE4 (RW)
	wire 	[31:0] 	w_axi_rdata; 			//	0xE5~0xE8 (RO)
	reg 	[0:0] 	r_axi_sel = 1; 			//	0xE9[7:2] (RW)
	reg 			r_axi_r1w0 = 0; 			//	0xE9[1] (RW)
	reg 			r_axi_req = 0; 			//	0xE9[0] (WO, Single Cycle)
	reg 			r_axi_req_o = 0; 			//	Delayed of r_axi_req. 
	
	
	//	Buffered AXI Read Data
	reg 	[31:0] 	r_axi_rdata = 0; 		//	Use state machine
	reg 			r_axi_idle = 0; 		//	AXI Idle 
	
	reg 	[3:0] 	rs_axilite = 0; 		//	AXI Access
	wire 	[3:0] 	ws_axilite_idle = 0; 		
	wire 	[3:0] 	ws_axilite_write = 1; 
	wire 	[3:0] 	ws_axilite_read = 2; 
	wire 	[3:0] 	ws_axilite_endread = 3; 
	
	reg 			r_axi_awvalid = 0, r_axi_wvalid = 0, r_axi_arvalid = 0; 
	wire 			w_axi_awready, w_axi_wready, w_axi_arready, w_axi_rvalid; 
	
	
	reg 	[3:0] 	rc_axi_init = 0; 

	always @(posedge clk_sys or posedge rst_sys) begin
		if(rst_sys) begin
			r_dsi_tx_rstn <= 0; 
			r_dsi_pwm <= 64; 
			r_axi_req <= 0; 
			r_dsi_resetn_o <= 0; 
			r_dsi_data_rstn <= 0; 
			
			rs_axilite <= 0; 
			r_axi_awvalid <= 0; 
			r_axi_wvalid <= 0;
			r_axi_arvalid <= 0; 
			
			r_dsi_lp_p_ovr <= 0; 
			r_dsi_lp_n_ovr <= 0; 
			
			rc_axi_init <= 0; 
			r_axi_idle <= 0; 
			
		end else begin
			r_dsi_tx_rstn <= 1; 
			r_dsi_resetn_o <= 1; 
			r_dsi_data_rstn <= 1; 
			
		end
	end
	
	assign dsi_resetn_o = r_dsi_resetn_o; 
	
	assign csi_ctl0_oe = 0; 
	assign csi_ctl1_oe = 0; 
	
	
	PWMLite dsi_pwm (		//	#(.ENABLE_TICK(0), .PWM_BITS(7)) 
		.clk_i			(clk_sys),
		.rst_i			(rst_sys),
		.pwm_i			(r_dsi_pwm[6:0]),
		.pol_i			(r_dsi_pwm[7]), 
		.pwm_o			(dsi_pwm_o)
	);






	////////////////////////////////////////////////////////////////
	//	MIPI CSI RX
	
	//	The CSI RXC shall not be inverted. Data can be inverted with swapped LP data and flipped HS data. 
	localparam 	CSI_RXD_INV 	= 4'b1111; 
	localparam 	CSI_DATA_WIDTH 	= 32; 			
	localparam 	CSI_STRB_WIDTH 	= CSI_DATA_WIDTH / 8; 

	//	Current implementation supports RAW8 only. 
	wire 			w_csi_rx_clk; 
	wire 			w_csi_rx_vsync0, w_csi_rx_hsync0, w_csi_rx_dvalid; 
	wire 	[63:0] 	w_csi_rx_data; 
	wire 	[47:0] 	w_csi_rx_data_rel_raw; 
	
	//	AXI Interface
	wire 	[31:0] 	w_csi_axi_rdata; 
	wire 			w_csi_axi_awready, w_csi_axi_wready, w_csi_axi_arready, w_csi_axi_rvalid; 
	
	
	
`define PRI_MIPI_IP
	
	
`ifdef PRI_MIPI_IP
	wire 	[7:0] 	w_mipi_d0 = (CSI_RXD_INV[0] ? 8'hFF : 8'h00) ^ csi_rxd0_hs_i; 
	wire 	[7:0] 	w_mipi_d1 = (CSI_RXD_INV[1] ? 8'hFF : 8'h00) ^ csi_rxd1_hs_i; 
	wire 	[7:0] 	w_mipi_d2 = (CSI_RXD_INV[2] ? 8'hFF : 8'h00) ^ csi_rxd2_hs_i; 
	wire 	[7:0] 	w_mipi_d3 = (CSI_RXD_INV[3] ? 8'hFF : 8'h00) ^ csi_rxd3_hs_i; 
	MIPIRx4LaneFre mipi0_rx (
		.sclk_i			(clk_sys), 
		.srst_i			(rst_sys), 
		
		.mipi_rx_byteclk_i	(csi_rxc_i), 
		.mipi_rx_d0_i		({w_mipi_d0[0], w_mipi_d0[1], w_mipi_d0[2], w_mipi_d0[3], w_mipi_d0[4], w_mipi_d0[5], w_mipi_d0[6], w_mipi_d0[7]}), 
		.mipi_rx_d1_i		({w_mipi_d1[0], w_mipi_d1[1], w_mipi_d1[2], w_mipi_d1[3], w_mipi_d1[4], w_mipi_d1[5], w_mipi_d1[6], w_mipi_d1[7]}), 
		.mipi_rx_d2_i		({w_mipi_d2[0], w_mipi_d2[1], w_mipi_d2[2], w_mipi_d2[3], w_mipi_d2[4], w_mipi_d2[5], w_mipi_d2[6], w_mipi_d2[7]}), 
		.mipi_rx_d3_i		({w_mipi_d3[0], w_mipi_d3[1], w_mipi_d3[2], w_mipi_d3[3], w_mipi_d3[4], w_mipi_d3[5], w_mipi_d3[6], w_mipi_d3[7]}), 
		.mipi_rx_clk_lp_p_i	(csi_rxc_lp_p_i), 
		.mipi_rx_clk_lp_n_i	(csi_rxc_lp_n_i), 
		.mipi_rx_d0_lp_p_i	(CSI_RXD_INV[0] ? csi_rxd0_lp_n_i : csi_rxd0_lp_p_i), 
		.mipi_rx_d0_lp_n_i	(CSI_RXD_INV[0] ? csi_rxd0_lp_p_i : csi_rxd0_lp_n_i), 
		.mipi_rx_rst_o		(), 
		.mipi_rx_rst_n_o		(), 
		.mipi_rx_clk_hs_en_o	(csi_rxc_hs_en_o), 
		.mipi_rx_dat_hs_en_o	(csi_rxd0_hs_en_o), 
		.mipi_rx_clk_term_en_o	(csi_rxc_hs_term_en_o), 
		.mipi_rx_dat_term_en_o	(csi_rxd0_hs_term_en_o), 
		
		.CLK				(w_csi_rx_clk), 
		.VSYNC			(w_csi_rx_vsync0), 
		.HSYNC			(w_csi_rx_hsync0), 
		.DE				(w_csi_rx_dvalid), 
		.DAT				(w_csi_rx_data_rel_raw)
	);
	assign w_csi_rx_data = {2{w_csi_rx_data_rel_raw[47:40], w_csi_rx_data_rel_raw[35:28], w_csi_rx_data_rel_raw[23:16], w_csi_rx_data_rel_raw[11:4]}}; 
	assign {csi_rxd3_hs_en_o, csi_rxd2_hs_en_o, csi_rxd1_hs_en_o} = {3{csi_rxd0_hs_en_o}}; 
	assign {csi_rxd3_hs_term_en_o, csi_rxd2_hs_term_en_o, csi_rxd1_hs_term_en_o} = {3{csi_rxd0_hs_term_en_o}}; 
	
	
`else	
	
	//	Reset pixel 16 cycles after ~vsync. 
	reg 			r_reset_pixen_n = 0; 
	reg 	[1:0] 	r_csi_rx_vsync0_i = 0; 
	always @(posedge w_csi_rx_clk or negedge rstn_sys) begin
		if(~rstn_sys) begin
			r_reset_pixen_n <= 0; 
			r_csi_rx_vsync0_i <= 0; 
		end else begin
			r_csi_rx_vsync0_i <= {r_csi_rx_vsync0_i, w_csi_rx_vsync0}; 
			r_reset_pixen_n <= (r_csi_rx_vsync0_i == 2'b10) ? 1'b0 : 1'b1; 
		end
	end
	
	assign w_csi_rx_clk = clk_sys; 
	csi_rx mipi_rx_0(
		.reset_n			(rstn_sys), 
		.clk				(w_csi_rx_clk), 
		.reset_byte_HS_n		(rstn_sys), 
		.clk_byte_HS		(csi_rxc_i), 
		.reset_pixel_n		(r_reset_pixen_n), 	//rstn_sys), 
		.clk_pixel			(w_csi_rx_clk), 
		.Rx_LP_CLK_P		(csi_rxc_lp_p_i), 
		.Rx_LP_CLK_N		(csi_rxc_lp_n_i), 
		
		.Rx_HS_enable_C		(csi_rxc_hs_en_o), 
		.LVDS_termen_C		(csi_rxc_hs_term_en_o), 
		
		//	Lane inversion affects HS & LP data only. 
		.Rx_LP_D_P			({CSI_RXD_INV[3] ? csi_rxd3_lp_n_i : csi_rxd3_lp_p_i, CSI_RXD_INV[2] ? csi_rxd2_lp_n_i : csi_rxd2_lp_p_i, CSI_RXD_INV[1] ? csi_rxd1_lp_n_i : csi_rxd1_lp_p_i, CSI_RXD_INV[0] ? csi_rxd0_lp_n_i : csi_rxd0_lp_p_i}), 
		.Rx_LP_D_N			({CSI_RXD_INV[3] ? csi_rxd3_lp_p_i : csi_rxd3_lp_n_i, CSI_RXD_INV[2] ? csi_rxd2_lp_p_i : csi_rxd2_lp_n_i, CSI_RXD_INV[1] ? csi_rxd1_lp_p_i : csi_rxd1_lp_n_i, CSI_RXD_INV[0] ? csi_rxd0_lp_p_i : csi_rxd0_lp_n_i}), 
		.Rx_HS_D_0			((CSI_RXD_INV[0] ? 8'hFF : 8'h00) ^ csi_rxd0_hs_i), 
		.Rx_HS_D_1			((CSI_RXD_INV[1] ? 8'hFF : 8'h00) ^ csi_rxd1_hs_i), 
		.Rx_HS_D_2			((CSI_RXD_INV[2] ? 8'hFF : 8'h00) ^ csi_rxd2_hs_i), 
		.Rx_HS_D_3			((CSI_RXD_INV[3] ? 8'hFF : 8'h00) ^ csi_rxd3_hs_i), 
		.Rx_HS_D_4			(), 
		.Rx_HS_D_5			(), 
		.Rx_HS_D_6			(), 
		.Rx_HS_D_7			(), 
		
		.Rx_HS_enable_D		({csi_rxd3_hs_en_o, csi_rxd2_hs_en_o, csi_rxd1_hs_en_o, csi_rxd0_hs_en_o}), 
		.LVDS_termen_D		({csi_rxd3_hs_term_en_o, csi_rxd2_hs_term_en_o, csi_rxd1_hs_term_en_o, csi_rxd0_hs_term_en_o}), 
		.fifo_rd_enable		({csi_rxd3_fifo_rd_o, csi_rxd2_fifo_rd_o, csi_rxd1_fifo_rd_o, csi_rxd0_fifo_rd_o}), 
		.fifo_rd_empty		({csi_rxd3_fifo_empty_i, csi_rxd2_fifo_empty_i, csi_rxd1_fifo_empty_i, csi_rxd0_fifo_empty_i}), 
		.DLY_enable_D		(), 
		.DLY_inc_D			(), 
		.u_dly_enable_D		(0), 
		.u_dly_inc_D		(), 
		
		.vsync_vc1			(), 
		.vsync_vc15			(), 
		.vsync_vc12			(), 
		.vsync_vc9			(), 
		.vsync_vc7			(), 
		.vsync_vc14			(), 
		.vsync_vc13			(), 
		.vsync_vc11			(), 
		.vsync_vc10			(), 
		.vsync_vc8			(), 
		.vsync_vc6			(), 
		.vsync_vc4			(), 
		.vsync_vc0			(w_csi_rx_vsync0), 
		.vsync_vc5			(), 
		.vsync_vc3			(), 
		.vsync_vc2			(), 
	
		.irq				(), 
		
		.pixel_data_valid		(w_csi_rx_dvalid), 
		.pixel_data			(w_csi_rx_data), 
		.pixel_per_clk		(), 
		.datatype			(), 
		.shortpkt_data_field	(), 
		.word_count			(), 
		.vcx				(), 
		.vc				(), 
		.hsync_vc3			(), 
		.hsync_vc2			(), 
		.hsync_vc8			(), 
		.hsync_vc12			(), 
		.hsync_vc7			(), 
		.hsync_vc10			(), 
		.hsync_vc1			(), 
		.hsync_vc0			(w_csi_rx_hsync0), 
		.hsync_vc13			(), 
		.hsync_vc4			(), 
		.hsync_vc11			(), 
		.hsync_vc6			(), 
		.hsync_vc9			(), 
		.hsync_vc15			(), 
		.hsync_vc14			(), 
		.hsync_vc5			(), 
		
		.axi_clk			(clk_sys), 
		.axi_reset_n		(rstn_sys), 
		
		.axi_awaddr			(r_axi_addr), 
		.axi_awvalid		((r_axi_sel == CSI_AXILITE_ID) && r_axi_awvalid), 
		.axi_awready		(w_csi_axi_awready), 
	
		.axi_wvalid			((r_axi_sel == CSI_AXILITE_ID) && r_axi_wvalid), 
		.axi_wdata			(r_axi_wdata), 
		.axi_wready			(w_csi_axi_wready), 
	
		.axi_bvalid			(), 
		.axi_bready			(1), 
		
		.axi_araddr			(r_axi_addr), 
		.axi_arvalid		((r_axi_sel == CSI_AXILITE_ID) && r_axi_arvalid), 
		.axi_arready		(w_csi_axi_arready), 
		
		.axi_rready			(1), 
		.axi_rvalid			(w_csi_axi_rvalid), 
		.axi_rdata			(w_csi_axi_rdata)
		
	);
	
	assign csi_rxd0_rst_o = rst_sys; 
	assign csi_rxd1_rst_o = rst_sys; 
	assign csi_rxd2_rst_o = rst_sys; 
	assign csi_rxd3_rst_o = rst_sys; 
`endif
	
	
	
	//	Output LED
	reg 	[5:0]		r_csi_fv_o = 0; 
	reg 	[1:0] 	r_csi_rx_vsync0_in = 0; 
	always @(posedge w_csi_rx_clk) begin
		r_csi_rx_vsync0_in <= {r_csi_rx_vsync0_in, w_csi_rx_vsync0}; 
		r_csi_fv_o <= r_csi_fv_o + ((r_csi_rx_vsync0_in == 2'b01) ? 1 : 0); 
	end
	assign led_o[0] = r_csi_fv_o[5];   // [EDGE_PATCH] LED0 = vendor frame-valid blink (kept as baseline) 
	
	
	
	////////////////////////////////////////////////////////////////
	//	MIPI-CSI Crop
	
	wire			XYCrop_frame_vsync; 
	wire			XYCrop_frame_href;
	wire			XYCrop_frame_de;
	wire	[63:0]	XYCrop_frame_Gray;

	//Sensor_Image_XYCrop
	//#(
	//	.IMAGE_HSIZE_SOURCE (1920 / CSI_STRB_WIDTH),
	//	.IMAGE_VSIZE_SOURCE (1080	 ),
	//	.IMAGE_HSIZE_TARGET (1024 / CSI_STRB_WIDTH),
	//	.IMAGE_YSIZE_TARGET (600 	 ),
	//	.PIXEL_DATA_WIDTH	(CSI_DATA_WIDTH) 		//	32		 )
	//)
	//u_Sensor_Image_XYCrop
	//(
	//	//	globel clock
	//	.clk			(w_csi_rx_clk),			//	image pixel clock
	//	.rst_n		(rstn_sys),			//	system reset
	//	
	//	//CMOS Sensor interface
	//	.image_in_vsync (w_csi_rx_vsync0		),			//H : Data Valid; L : Frame Sync(Set it by register)
	//	.image_in_href	(w_csi_rx_hsync0		),			//H : Data vaild, L : Line Sync
	//	.image_in_de	(w_csi_rx_dvalid		), 			//H : Data Enable, L : Line Sync
	//	.image_in_data	({w_csi_rx_data[7:0], w_csi_rx_data[15:8], w_csi_rx_data[23:16], w_csi_rx_data[31:24], w_csi_rx_data[39:32], w_csi_rx_data[47:40], w_csi_rx_data[55:48], w_csi_rx_data[63:56]}),			//8 bits cmos data input
	//	
	//	.image_out_vsync(XYCrop_frame_vsync ),			//H : Data Valid; L : Frame Sync(Set it by register)
	//	.image_out_href (XYCrop_frame_href	),			//H : Data vaild, L : Line Sync
	//	.image_out_de	(XYCrop_frame_de	), 			//H : Data Enable, L : Line Sync
	//	.image_out_data (XYCrop_frame_Gray	)			//8 bits cmos data input	
	//);

	reg			r_XYCrop_frame_vsync = 0; 
	reg			r_XYCrop_frame_href = 0;
	reg			r_XYCrop_frame_de = 0;
	reg	[63:0]	r_XYCrop_frame_Gray = 0;
	
	always @(posedge w_csi_rx_clk) begin
		r_XYCrop_frame_vsync <= XYCrop_frame_vsync; 
		r_XYCrop_frame_href <= XYCrop_frame_href;
		r_XYCrop_frame_de <= XYCrop_frame_de;
		r_XYCrop_frame_Gray <= XYCrop_frame_Gray;
	end
	
	//	Data Write Assignment
	//wire			cmos_frame_vsync = r_XYCrop_frame_vsync;                     //  cmos frame data vsync valid signal
	//wire			cmos_frame_href = r_XYCrop_frame_href && r_XYCrop_frame_de;	 //  cmos frame data href vaild  signal
	//wire	[63:0]	cmos_frame_Gray = r_XYCrop_frame_Gray; 
	wire			cmos_frame_vsync = w_csi_rx_vsync0;                     //  cmos frame data vsync valid signal
	wire			cmos_frame_href = w_csi_rx_hsync0 && w_csi_rx_dvalid;	 //  cmos frame data href vaild  signal
	wire	[63:0]	cmos_frame_Gray = {w_csi_rx_data[7:0], w_csi_rx_data[15:8], w_csi_rx_data[23:16], w_csi_rx_data[31:24], w_csi_rx_data[39:32], w_csi_rx_data[47:40], w_csi_rx_data[55:48], w_csi_rx_data[63:56]}; 

	












	////////////////////////////////////////////////////////////////
	//	DDR R/W Control
	

	wire                            lcd_de;
	wire                            lcd_hs;      
	wire                            lcd_vs;
	wire 					  lcd_request; 
	wire            [7:0]           lcd_red, lcd_red2;
	wire            [7:0]           lcd_green, lcd_green2;
	wire            [7:0]           lcd_blue, lcd_blue2;
	wire            [15:0]          lcd_data;


	assign w_ddr3_awid = 0; 
	assign w_ddr3_wid = 0; 
	
	wire 			w_wframe_vsync; 
	wire 	[7:0] 	w_axi_tp; 
	
	//	[EDGE_PATCH] 分屏合成：改的是「写进 DDR 的像素内容」，DDR 读写时序不动
	//	    SPLIT_BEATS = 240 → 左半屏原图 / 右半屏边缘图（推荐）
	//	    SPLIT_BEATS = 480 → 不分屏，行为完全回到 vendor 原版
	localparam	SPLIT_BEATS	= 240;
	wire	[31:0]	w_split_word;
	wire		w_split_en;
	wire		w_split_vsync;
	wire	[7:0]	w_edge_pix0;
	wire	[7:0]	w_edge_pix1;
	wire		w_edge_valid;
	
	axi4_ctrl #(.C_RD_END_ADDR(1920 * 1080), .C_W_WIDTH(CSI_DATA_WIDTH), .C_R_WIDTH(8), .C_ID_LEN(4)) u_axi4_ctrl (

		.axi_clk        (w_ddr3_ui_clk            ),
		.axi_reset      (w_ddr3_ui_rst            ),

		.axi_awaddr     (w_ddr3_awaddr       ),
		.axi_awlen      (w_ddr3_awlen        ),
		.axi_awvalid    (w_ddr3_awvalid      ),
		.axi_awready    (w_ddr3_awready      ),

		.axi_wdata      (w_ddr3_wdata        ),
		.axi_wstrb      (w_ddr3_wstrb        ),
		.axi_wlast      (w_ddr3_wlast        ),
		.axi_wvalid     (w_ddr3_wvalid       ),
		.axi_wready     (w_ddr3_wready       ),

		.axi_bid        (0          ),
		.axi_bresp      (0        ),
		.axi_bvalid     (1       ),

		.axi_arid       (w_ddr3_arid         ),
		.axi_araddr     (w_ddr3_araddr       ),
		.axi_arlen      (w_ddr3_arlen        ),
		.axi_arvalid    (w_ddr3_arvalid      ),
		.axi_arready    (w_ddr3_arready      ),

		.axi_rid        (w_ddr3_rid          ),
		.axi_rdata      (w_ddr3_rdata        ),
		.axi_rresp      (0        ),
		.axi_rlast      (w_ddr3_rlast        ),
		.axi_rvalid     (w_ddr3_rvalid       ),
		.axi_rready     (w_ddr3_rready       ),

		.wframe_pclk    (w_csi_rx_clk          ),
		.wframe_vsync   (w_split_vsync), //w_wframe_vsync   ),		//	Writter VSync. Flush on rising edge. Connect to EOF. 
		.wframe_data_en (w_split_en   ),
		.wframe_data    (w_split_word),
		
		.rframe_pclk    (clk_pixel_2x            ),
		.rframe_vsync   (~lcd_vs             ),		//	Reader VSync. Flush on rising edge. Connect to ~EOF. 
		.rframe_data_en (lcd_request             ),
		.rframe_data    (lcd_data           ),
		
		.tp_o 		(w_axi_tp)
	);
	
	//	[EDGE_PATCH] 分屏合成：左半原图 / 右半边缘图，全部在 CSI 像素时钟域
	//	    edge_proc 输出的 1 个边缘像素 → 复制成 4 字节 → 右半屏横向 4 倍显示
	split_compose
	#(
		.MAX_BEATS	(512		),
		.SPLIT_BEATS	(SPLIT_BEATS	)
	)
	u_split_compose
	(
		.clk		(w_csi_rx_clk		),
		.rst_n		(rstn_sys		),
		.vsync		(cmos_frame_vsync	),
		.de		(cmos_frame_href	),
		.orig_word	(cmos_frame_Gray[31:0]	),
		.edge_pix0	(w_edge_pix0		),
		.edge_pix1	(w_edge_pix1		),
		.edge_valid	(w_edge_valid		),
		.wr_word	(w_split_word		),
		.wr_en		(w_split_en		),
		.wr_vsync	(w_split_vsync		)
	);
	// [EDGE_PATCH] led_o[3:2] driver removed (edge_proc drives led_o[5:1]) 
	
	
	
	
	////////////////////////////////////////////////////////////////
	//  LCD Timing Driver
	
	lcd_driver u_lcd_driver
	(
	    //  global clock
	    .clk        (clk_pixel_2x   ),
	    .rst_n      (rstn_pixel), 
	    
	    //  lcd interface
	    .lcd_dclk   (               ),
	    .lcd_blank  (               ),
	    .lcd_sync   (               ),
	    .lcd_request(lcd_request    ), 	//	Request data 1 cycle ahead. 
	    .lcd_hs     (lcd_hs         ),
	    .lcd_vs     (lcd_vs         ),
	    .lcd_en     (lcd_de         ),
	    .lcd_rgb    ({lcd_red2,lcd_green2,lcd_blue2, lcd_red,lcd_green,lcd_blue}),
	    
	    //  user interface
	    .lcd_data   ({{3{lcd_data[15:8]}}, {3{lcd_data[7:0]}}}  )
	);
	
	
	

	wire 			w_rgb_vsync, w_rgb_hsync, w_rgb_href; 
	wire 	[7:0] 	w_rgb_r, w_rgb_g, w_rgb_b; 
	
	VIP_RAW8_RGB888 #(.IMG_HDISP(1920), .IMG_VDISP(1080)) bayer2rgb (
		.clk				(clk_pixel_2x),                //cmos video pixel clock
		.rst_n			(rstn_pixel),              //global reset
	    
		.mirror			(0),

		    //CMOS YCbCr444 data output
		.per_frame_vsync		(lcd_vs),    //Prepared Image data vsync valid signal. Reset on falling edge. 
		.per_frame_href		(lcd_de),     //Prepared Image data href vaild  signal
		.per_frame_hsync		(lcd_hs),     //Prepared Image data href vaild  signal
		.per_img_RAW		(lcd_green),        //Prepared Image data 8 Bit RAW Data

		.post_frame_vsync		(w_rgb_vsync),   //Processed Image data vsync valid signal
		.post_frame_href		(w_rgb_href),    //Processed Image data href vaild  signal
		.post_frame_hsync		(w_rgb_hsync),    //Processed Image data href vaild  signal
		.post_img_red		(w_rgb_r),       //Prepared Image green data to be processed 
		.post_img_green		(w_rgb_g),     //Prepared Image green data to be processed
		.post_img_blue		(w_rgb_b)       //Prepared Image blue data to be processed
	);

	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	LVDS Output 
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	HDMI requires specific timing, thus is not compatible with LCD & LVDS & DSI. Must implement standalone. 
	
	assign hdmi_txd0_rst_o = rst_pixel; 
	assign hdmi_txd1_rst_o = rst_pixel; 
	assign hdmi_txd2_rst_o = rst_pixel; 
	assign hdmi_txc_rst_o = rst_pixel; 
	
	assign hdmi_txd0_oe = 1'b1; 
	assign hdmi_txd1_oe = 1'b1; 
	assign hdmi_txd2_oe = 1'b1; 
	assign hdmi_txc_oe = 1'b1; 
	
	//-------------------------------------
	//Digilent HDMI-TX IP Modified by CB elec.
	rgb2dvi #(.ENABLE_OSERDES(0)) u_rgb2dvi 
	(
		.oe_i 		(1), 			//	Always enable output
		.bitflip_i 		(4'b0000), 		//	Reverse clock & data lanes. 
		
		.aRst			(1'b0), 
		.aRst_n		(1'b1), 
		
		.PixelClk		(clk_pixel_2x        ),//pixel clk = 74.25M
		.SerialClk		(     ),//pixel clk *5 = 371.25M
		
		.vid_pVSync		(w_rgb_vsync), 
		.vid_pHSync		(w_rgb_hsync), 
		.vid_pVDE		(w_rgb_href), 
		.vid_pData		({w_rgb_r, w_rgb_g, w_rgb_b}), 
		
		.txc_o		(hdmi_txc_o), 
		.txd0_o		(hdmi_txd0_o), 
		.txd1_o		(hdmi_txd1_o), 
		.txd2_o		(hdmi_txd2_o)
	); 
			
	
	
	////////////////////////////////////////////////////////////////
	//	[EDGE_PATCH] 赛题四算法：MIPI 相机实拍 → 灰度 / 中值滤波 / Sobel 边缘检测
	//
	//	★ 接缝：直接复用 vendor 已验证能收到帧的 cmos_frame_* 通路，不动 DDR/HDMI。
	//	    cmos_frame_href         每拍 4 个像素（CSI 像素时钟域）
	//	    cmos_frame_Gray[31:24]  固定取组内第 4 个像素 → 得到 1 像素/拍
	//	    → 480 列 × 1080 行（水平 4:1 抽取，整行保留）
	//
	//	算法链：rgb2gray → 3×3 窗口 → median3x3 → 3×3 窗口 → sobel3x3 → 每帧统计
	//
	//	LED 低有效（0 = 亮），板上从右往左（led[0] 在最右边）：
	//	    LED0 : vendor 原有 r_csi_fv_o[5] 闪烁 = 相机帧有效（基准灯，未改动）
	//	    LED1 : 流水线在跑（本帧收到过有效 3×3 窗口）
	//	    LED2 : 边缘像素占比 >= 1/8192
	//	    LED3 : 边缘像素占比 >= 1/1024
	//	    LED4 : 边缘像素占比 >= 1/128
	//	    LED5 : 边缘像素占比 >= 1/16
	//	    → 亮得越多 = 画面里边缘越多
	////////////////////////////////////////////////////////////////
	wire	[4:0]	w_edge_bar;

	edge_proc
	#(
		.MAX_W		(1024		),
		.THRESHOLD	(8'd24		)
	)
	u_edge_proc
	(
		.clk		(w_csi_rx_clk		),
		.rst_n		(rstn_sys		),
		.de		(cmos_frame_href	),
		.vsync		(cmos_frame_vsync	),
		.pix_word	(cmos_frame_Gray[31:0]	),
		.led_bar_o	(w_edge_bar		),
		.edge_pix0_o	(w_edge_pix0		),
		.edge_pix1_o	(w_edge_pix1		),
		.edge_valid_o	(w_edge_valid		)
	);

	assign led_o[5:1] = ~w_edge_bar;

endmodule

====================================================================================================
 [ 2/29] src/edge_proc.v    (460 行, 已例化)
====================================================================================================
//============================================================================
//  edge_proc.v —— 赛题四算法链（双通道，960×1080）
//
//  数据来源（vendor example_top.v 里已经验证能收到帧的那条通路）：
//      cmos_frame_href   = w_csi_rx_hsync0 && w_csi_rx_dvalid   (de，每拍 4 个像素)
//      cmos_frame_vsync  = w_csi_rx_vsync0                      (高 = 帧有效)
//      cmos_frame_Gray   = 4 个真实像素（1920 宽 → 480 拍/行）
//
//  本版相对上一版的改动
//  ① 一行 4 个像素不再只取 1 个，而是「相邻两个求平均」→ 每拍出 2 个像素
//       lane0 = (pix0 + pix1) >> 1      → 抽取列 2b
//       lane1 = (pix2 + pix3) >> 1      → 抽取列 2b+1
//     于是处理分辨率 480×1080 → **960×1080**，同时这步平均本身就是 RAW Bayer
//     相邻两色（R+G 或 G+B）的亮度近似，所以「彩色→灰度」这一步是真做了，
//     不再是 r=g=b 直通。
//  ② 行缓存从「逻辑存储器（LUTRAM）」改成 **同步读的 Block RAM**：
//     原来每个 win3x3_stream 占 8303 FF + 8037 LUT（几乎把器件吃满），
//     改完只剩几十个 FF + 8 个 BRAM，省下来的资源正好放下第二条通道。
//     代价：同步读有 1 拍延迟，窗口中心因此比 beat 再滞后约 3 列（画面看不出来）。
//
//  LED 语义不变（低有效，0 = 亮），比例门限按两通道的总窗口数算。
//============================================================================

//----------------------------------------------------------------------------
//  3×3 窗口行缓存（Block RAM 版）
//    · 读写同一时钟，写端口 + 同步读端口 → 综合成 simple dual-port RAM
//    · raddr = x-1：避开与写端口同地址，换来「读回上一行同列」的确定性
//    · 行尾按 de 下降沿判断，不依赖固定行宽
//----------------------------------------------------------------------------
module win3x3_stream #(
    parameter integer MAX_W = 1024          // 行缓存深度（960 列必须 < MAX_W）
)(
    input  wire            clk,
    input  wire            rst_n,
    input  wire            de,              // 像素有效
    input  wire            vsync,           // 帧同步（高 = 帧有效）
    input  wire      [7:0] din,
    output reg       [7:0] p00, p01, p02,
    output reg       [7:0] p10, p11, p12,
    output reg       [7:0] p20, p21, p22,
    output reg             win_valid
);

    reg [7:0] bankA [0:MAX_W-1];            // 上上行
    reg [7:0] bankB [0:MAX_W-1];            // 上一行

    reg  [10:0] x;                          // 行内像素计数
    reg         parity;                     // 当前行写入哪块
    reg  [1:0]  line_cnt;                   // 已有几行历史
    reg         de_d, vs_d;

    wire        eol     = de_d  & ~de;       // de 下降沿 = 一行结束
    wire        vs_rise = vsync & ~vs_d;     // vsync 上升沿 = 新一帧

    wire [10:0] raddr   = (x == 11'd0) ? 11'd0 : (x - 11'd1);

    reg  [7:0] qa, qb;                       // 同步读输出
    reg        parity_d1;                    // 读那一拍的 parity
    reg  [7:0] din_d1, din_d2;               // din 对齐用（2 拍）
    reg  [7:0] r0_0, r0_1, r0_2;             // 上上行 3 像素
    reg  [7:0] r1_0, r1_1, r1_2;             // 上一行 3 像素
    reg  [7:0] r2_0, r2_1, r2_2;             // 当前行 3 像素
    reg        w0, w1;

    wire [7:0] mid_r = parity_d1 ? qa : qb;  // 上一行
    wire [7:0] top_r = parity_d1 ? qb : qa;  // 上上行

    //---------------- 写端口（当前行）----------------
    always @(posedge clk) begin
        if (de) begin
            if (!parity) bankA[x] <= din;
            else         bankB[x] <= din;
        end
    end

    //---------------- 同步读端口（1 拍延迟 → 推断 BRAM）----------------
    always @(posedge clk) begin
        qa <= bankA[raddr];
        qb <= bankB[raddr];
    end

    //---------------- 列计数 / 行结束 / 帧同步 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x         <= 11'd0;
            parity    <= 1'b0;
            line_cnt  <= 2'd0;
            de_d      <= 1'b0;
            vs_d      <= 1'b0;
            parity_d1 <= 1'b0;
        end else begin
            de_d      <= de;
            vs_d      <= vsync;
            parity_d1 <= parity;

            if (vs_rise) begin
                // 新一帧：丢掉历史行，避免第 1、2 行用上一帧的数据
                x        <= 11'd0;
                parity   <= 1'b0;
                line_cnt <= 2'd0;
            end else if (eol) begin
                x       <= 11'd0;
                parity  <= ~parity;
                if (line_cnt != 2'd3) line_cnt <= line_cnt + 2'd1;
            end else if (de) begin
                if (x < MAX_W-1) x <= x + 11'd1;
            end
        end
    end

    //---------------- 矩阵寄存器 + 有效标志 ----------------
    //  RAM 输出比 din 晚 2 拍，所以 din 也延 2 拍对齐；
    //  窗口寄存器本身再打 1 拍，win_valid 要对应推迟 2 拍。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            din_d1<=8'd0; din_d2<=8'd0;
            r0_0<=8'd0; r0_1<=8'd0; r0_2<=8'd0;
            r1_0<=8'd0; r1_1<=8'd0; r1_2<=8'd0;
            r2_0<=8'd0; r2_1<=8'd0; r2_2<=8'd0;
            p00<=8'd0; p01<=8'd0; p02<=8'd0;
            p10<=8'd0; p11<=8'd0; p12<=8'd0;
            p20<=8'd0; p21<=8'd0; p22<=8'd0;
            w0<=1'b0; w1<=1'b0; win_valid<=1'b0;
        end else begin
            din_d1 <= din;
            din_d2 <= din_d1;

            //  有效标志流水 2 拍，与 p** 对齐
            w1        <= w0;
            win_valid <= w1;

            if (vs_rise) begin
                w0        <= 1'b0;
                w1        <= 1'b0;
                win_valid <= 1'b0;
            end else if (de) begin
                w0 <= (line_cnt >= 2'd2) && (x >= 11'd4);

                r0_2 <= r0_1; r0_1 <= r0_0; r0_0 <= top_r;
                r1_2 <= r1_1; r1_1 <= r1_0; r1_0 <= mid_r;
                r2_2 <= r2_1; r2_1 <= r2_0; r2_0 <= din_d2;

                p00 <= r0_2; p01 <= r0_1; p02 <= r0_0;
                p10 <= r1_2; p11 <= r1_1; p12 <= r1_0;
                p20 <= r2_2; p21 <= r2_1; p22 <= r2_0;
            end else begin
                w0 <= 1'b0;
            end
        end
    end

endmodule


//----------------------------------------------------------------------------
//  单通道算法链：rgb2gray → 3×3 → 中值 → 3×3 → Sobel
//    （两路完全一样，只是输入的像素不同，所以每个像素的流水线延迟一致）
//----------------------------------------------------------------------------
module edge_chain #(
    parameter integer MAX_W     = 1024,
    parameter [7:0]   THRESHOLD = 8'd24
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        de,
    input  wire        vsync,
    input  wire [7:0]  pix,
    output wire [7:0]  edge_pix,
    output wire        edge_valid
);

    //---------------- ① 灰度化（保持链路完整：R=G=B=输入亮度）----------------
    wire [7:0] gray;
    wire       gray_de;

    rgb2gray u_rgb2gray (
        .clk    (clk),
        .rst_n  (rst_n),
        .de     (de),
        .r      (pix),
        .g      (pix),
        .b      (pix),
        .de_out (gray_de),
        .y      (gray)
    );

    //---------------- ② 第一个 3×3 窗口 → 中值滤波 ----------------
    wire [7:0] a00,a01,a02, a10,a11,a12, a20,a21,a22;
    wire       win1_valid;

    win3x3_stream #(.MAX_W(MAX_W)) u_win1 (
        .clk       (clk),
        .rst_n     (rst_n),
        .de        (gray_de),
        .vsync     (vsync),
        .din       (gray),
        .p00(a00), .p01(a01), .p02(a02),
        .p10(a10), .p11(a11), .p12(a12),
        .p20(a20), .p21(a21), .p22(a22),
        .win_valid (win1_valid)
    );

    wire [7:0] med;
    wire       med_valid;

    median3x3 u_median3x3 (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (win1_valid),
        .p00(a00), .p01(a01), .p02(a02),
        .p10(a10), .p11(a11), .p12(a12),
        .p20(a20), .p21(a21), .p22(a22),
        .valid_out (med_valid),
        .med       (med)
    );

    //---------------- ③ 第二个 3×3 窗口 → Sobel ----------------
    wire [7:0] b00,b01,b02, b10,b11,b12, b20,b21,b22;
    wire       win2_valid;

    win3x3_stream #(.MAX_W(MAX_W)) u_win2 (
        .clk       (clk),
        .rst_n     (rst_n),
        .de        (med_valid),
        .vsync     (vsync),
        .din       (med),
        .p00(b00), .p01(b01), .p02(b02),
        .p10(b10), .p11(b11), .p12(b12),
        .p20(b20), .p21(b21), .p22(b22),
        .win_valid (win2_valid)
    );

    wire [7:0] edge_mag;

    sobel3x3 u_sobel3x3 (
        .clk       (clk),
        .rst_n     (rst_n),
        .win_valid (win2_valid),
        .p00(b00), .p01(b01), .p02(b02),
        .p10(b10), .p11(b11), .p12(b12),
        .p20(b20), .p21(b21), .p22(b22),
        .threshold (THRESHOLD),
        .valid_out (edge_valid),
        .edge_out  (edge_pix),
        .mag       (edge_mag)
    );

endmodule


//----------------------------------------------------------------------------
//  顶层：双通道算法链 + 每帧边缘比例统计 + LED 条形图
//----------------------------------------------------------------------------
module edge_proc #(
    parameter integer MAX_W     = 1024,     // 行缓存深度（960 列）
    parameter [7:0]   THRESHOLD = 8'd24     // Sobel 二值化阈值
)(
    input  wire            clk,             // = w_csi_rx_clk
    input  wire            rst_n,           // = rstn_sys
    input  wire            de,              // = cmos_frame_href
    input  wire            vsync,           // = cmos_frame_vsync
    input  wire     [31:0] pix_word,        // = cmos_frame_Gray[31:0]，一拍 4 像素

    output reg       [4:0] led_bar_o,       // 1 = 条件成立（example_top 里取反驱动 LED）
    output wire      [7:0] edge_pix0_o,     // lane0 边缘像素（抽取列 2b）
    output wire      [7:0] edge_pix1_o,     // lane1 边缘像素（抽取列 2b+1）
    output wire            edge_valid_o     // 边缘像素有效（每拍 2 个）
);

    //========================================================================
    //  2:1 水平抽取 + 相邻两像素求平均
    //    字节序：pix_word[31:24] = 组内第 1 个像素，[7:0] = 第 4 个
    //    → 0.5 的除法就是右移 1 位，一个加法器搞定，仍然没有除法器
    //========================================================================
    wire [8:0] sum0 = {1'b0, pix_word[31:24]} + {1'b0, pix_word[23:16]};
    wire [8:0] sum1 = {1'b0, pix_word[15:8]}  + {1'b0, pix_word[7:0]};
    wire [7:0] luma0 = sum0[8:1];
    wire [7:0] luma1 = sum1[8:1];

    //========================================================================
    //  两路算法链（并行，各自 1 像素/拍 → 合计 960×1080）
    //========================================================================
    edge_chain
    #(
        .MAX_W      (MAX_W),
        .THRESHOLD  (THRESHOLD)
    )
    u_lane0
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .de         (de),
        .vsync      (vsync),
        .pix        (luma0),
        .edge_pix   (edge_pix0_o),
        .edge_valid (edge_valid_o)
    );

    edge_chain
    #(
        .MAX_W      (MAX_W),
        .THRESHOLD  (THRESHOLD)
    )
    u_lane1
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .de         (de),
        .vsync      (vsync),
        .pix        (luma1),
        .edge_pix   (edge_pix1_o),
        .edge_valid ()
    );

    //========================================================================
    //  ④ 每帧统计：有效窗口数 / 边缘像素数（两路合计）
    //     960 列 × 1080 行 ≈ 1036800 → 21 bit，这里留 20 bit 够用到 60 万
    //========================================================================
    reg [20:0] cnt_tot;
    reg [20:0] cnt_edge;
    reg [20:0] tot_s;
    reg [20:0] edge_s;
    reg        vs_d2;

    wire vs_rise2   = vsync & ~vs_d2;
    //  兜底：万一 vsync 不按帧来（或一直不翻转），计数到满帧上限也强制结算一次
    //  960×1080 = 1036800，所以计数器要 21 bit（20 bit 只有 1048575，会截断）
    wire frame_done = vs_rise2 || (cnt_tot >= 21'd1200000);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_tot  <= 21'd0;
            cnt_edge <= 21'd0;
            tot_s    <= 21'd0;
            edge_s   <= 21'd0;
            vs_d2    <= 1'b0;
        end else begin
            vs_d2 <= vsync;

            if (frame_done) begin
                tot_s    <= cnt_tot;
                edge_s   <= cnt_edge;
                cnt_tot  <= 21'd0;
                cnt_edge <= 21'd0;
            end else if (edge_valid_o) begin
                cnt_tot <= cnt_tot + 21'd2;
                if (edge_pix0_o != 8'd0) cnt_edge <= cnt_edge + 21'd1;
                if (edge_pix1_o != 8'd0) cnt_edge <= cnt_edge + 21'd1;
            end
        end
    end

    //========================================================================
    //  ⑤ LED 显示边缘检测结果
    //     led_bar_o[0] : 流水线在跑（本帧收到过有效 3×3 窗口）
    //     led_bar_o[1] : 边缘像素占比 >= 1/8192
    //     led_bar_o[2] : 边缘像素占比 >= 1/1024
    //     led_bar_o[3] : 边缘像素占比 >= 1/128
    //     led_bar_o[4] : 边缘像素占比 >= 1/16
    //========================================================================
    wire [20:0] th1 = tot_s >> 13;     // 1/8192
    wire [20:0] th2 = tot_s >> 10;     // 1/1024
    wire [20:0] th3 = tot_s >> 7;      // 1/128
    wire [20:0] th4 = tot_s >> 4;      // 1/16

    wire any_data = (tot_s != 21'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            led_bar_o <= 5'b0;
        end else begin
            led_bar_o[0] <= any_data;
            led_bar_o[1] <= any_data && (edge_s >= th1);
            led_bar_o[2] <= any_data && (edge_s >= th2);
            led_bar_o[3] <= any_data && (edge_s >= th3);
            led_bar_o[4] <= any_data && (edge_s >= th4);
        end
    end

endmodule


//============================================================================
//  split_compose —— 分屏合成（原图 | 边缘图），接在 DDR 写入通道前面
//
//  ★ 本模块原本单独放在 src/split_compose.v，但 Efinity 在编译启动/收尾时会把
//    工程文件（Ti60_Demo.xml）里的 design_file 列表重写，多次把新加的条目删掉，
//    导致 "instantiating unknown module"。为保证工程自洽，这里并进 edge_proc.v
//    （这个文件本来就在工程列表里，不会被删）。
//
//  做的事情（全部在 CSI 像素时钟域，不跨时钟域、不改 DDR 读写时序）：
//    · 每行前 SPLIT_BEATS 拍：原样把 4 个原图像素写进 DDR
//    · 之后：把每拍的 2 个边缘像素各铺成 2 个显示像素写进 DDR
//      → 左半屏 = 原图左半部分（1:1）
//      → 右半屏 = 右半部分的边缘图（1:1，横向 2 像素块量化）
//      → 分界线正好在图像正中（第 960 个像素），两侧内容连续
//    · 回退：SPLIT_BEATS 设成 480 即完全回到 vendor 原版（不分屏）
//============================================================================
module split_compose #(
    parameter integer MAX_BEATS   = 512,   // 每行 beat 上限保护（480 拍/行）
    parameter integer SPLIT_BEATS = 240    // 前 240 拍 = 左半屏（1920/4/2）
)(
    input  wire        clk,           // = w_csi_rx_clk（CSI 像素时钟）
    input  wire        rst_n,         // = rstn_sys
    input  wire        vsync,         // = cmos_frame_vsync（高 = 帧有效）
    input  wire        de,            // = cmos_frame_href（高电平持续整行，一拍 4 像素）
    input  wire [31:0] orig_word,     // = cmos_frame_Gray[31:0]，4 个原图像素
    input  wire [7:0]  edge_pix0,     // 来自 edge_proc lane0（抽取列 2b）
    input  wire [7:0]  edge_pix1,     // 来自 edge_proc lane1（抽取列 2b+1）
    input  wire        edge_valid,    // 边缘像素有效
    output reg  [31:0] wr_word,       // 合成后写进 DDR 的 4 像素
    output reg         wr_en,         // 写使能（比输入晚 1 拍，和 wr_word 对齐）
    output reg         wr_vsync       // 帧同步（打一拍后给 axi4_ctrl）
);

    reg  [9:0] bc;                    // 行内 beat 计数
    reg        de_d, vs_d;

    wire       eol     = de_d & ~de;          // de 下降沿 = 一行结束
    wire       vs_rise = vsync & ~vs_d;       // vsync 上升沿 = 新一帧

    //  行内 beat 计数：行尾 / 帧头清零，饱和保护
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            bc   <= 10'd0;
            de_d <= 1'b0;
            vs_d <= 1'b0;
        end else begin
            de_d <= de;
            vs_d <= vsync;

            if (eol || vs_rise)
                bc <= 10'd0;
            else if (de)
                bc <= (bc < MAX_BEATS-1) ? bc + 10'd1 : bc;
        end
    end

    //  合成：左半屏原图 / 右半屏边缘
    //    右半屏每拍拿到 2 个边缘像素（lane0/lane1），各占 2 个显示像素：
    //      字节序 DDR 里 [7:0] 对应最左边那个显示像素
    //      → 第 4b、4b+1 个显示像素 = lane0，第 4b+2、4b+3 个 = lane1
    wire [7:0]  e0        = edge_valid ? edge_pix0 : 8'd0;
    wire [7:0]  e1        = edge_valid ? edge_pix1 : 8'd0;
    wire [31:0] edge_word = {e1, e1, e0, e0};

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            wr_word  <= 32'd0;
            wr_en    <= 1'b0;
            wr_vsync <= 1'b0;
        end else begin
            wr_en    <= de;
            wr_vsync <= vsync;
            wr_word  <= (bc < SPLIT_BEATS) ? orig_word : edge_word;
        end
    end

endmodule

====================================================================================================
 [ 3/29] src/rgb2gray.v    (55 行, 已例化)
====================================================================================================
//============================================================================
//  rgb2gray.v —— RGB888 转灰度（BT.601，定点实现）
//
//  Y = (77*R + 150*G + 29*B) >> 8
//  全部用移位+加法实现，**不使用除法器**（符合赛题四要求）
//
//    77  = 64+8+4+1   -> (R<<6)+(R<<3)+(R<<2)+R
//    150 = 128+16+4+2 -> (G<<7)+(G<<4)+(G<<2)+(G<<1)
//    29  = 32-2-1     -> (B<<5)-(B<<1)-B
//
//  流水线：2 级（乘法累加 1 级 + 移位截断 1 级）
//============================================================================
module rgb2gray
(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        de,
    input  wire [7:0]  r,
    input  wire [7:0]  g,
    input  wire [7:0]  b,
    output reg         de_out,
    output reg  [7:0]  y
);

    //---------------- 第 1 级：定点乘累加 ----------------
    reg [15:0] sum;
    reg        de_d1;

    wire [15:0] r_term = (r << 6) + (r << 3) + (r << 2) + r;              // 77R
    wire [15:0] g_term = (g << 7) + (g << 4) + (g << 2) + (g << 1);       // 150G
    wire [15:0] b_term = (b << 5) - (b << 1) - b;                          // 29B

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum   <= 16'd0;
            de_d1 <= 1'b0;
        end else begin
            sum   <= r_term + g_term + b_term;
            de_d1 <= de;
        end
    end

    //---------------- 第 2 级：右移 8 位截断 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y      <= 8'd0;
            de_out <= 1'b0;
        end else begin
            y      <= (sum[15:8] > 8'd255) ? 8'd255 : sum[15:8];
            de_out <= de_d1;
        end
    end

endmodule

====================================================================================================
 [ 4/29] src/median3x3.v    (91 行, 已例化)
====================================================================================================
//============================================================================
//  median3x3.v —— 3x3 中值滤波（抗椒盐噪声）
//
//  算法（经典 3x3 中值，只用比较交换，无排序/无除法）：
//    1) 对每一行的 3 个像素排序 → (min0,med0,max0) (min1,med1,max1) (min2,med2,max2)
//    2) 取  med_of_meds = median(med0,med1,med2)
//       取  max_of_mins = max(min0,min1,min2)
//       取  min_of_maxs = min(max0,max1,max2)
//    3) 中值 = median(max_of_mins, med_of_meds, min_of_maxs)
//
//  硬件代价：每像素 3 级比较器树（约 19 个 8bit 比较器），1 拍流水
//============================================================================

//----------------------------------------------------------------------------
//  3 输入排序（比较交换）
//----------------------------------------------------------------------------
module sort3
(
    input  wire [7:0] a,
    input  wire [7:0] b,
    input  wire [7:0] c,
    output reg  [7:0] vmax,
    output reg  [7:0] vmed,
    output reg  [7:0] vmin
);
    reg [7:0] x1, x2, y2, y3;

    always @(*) begin
        // 交换 a,b
        if (a > b) begin x1 = b; x2 = a; end
        else        begin x1 = a; x2 = b; end
        // 交换 x2,c
        if (x2 > c) begin y2 = c; y3 = x2; end
        else        begin y2 = x2; y3 = c; end
        // 交换 x1,y2
        if (x1 > y2) begin vmin = y2; vmed = x1; end
        else         begin vmin = x1; vmed = y2; end
        vmax = y3;
    end
endmodule


//----------------------------------------------------------------------------
//  3x3 中值滤波
//----------------------------------------------------------------------------
module median3x3
(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input  wire [7:0]  p00, p01, p02,
    input  wire [7:0]  p10, p11, p12,
    input  wire [7:0]  p20, p21, p22,
    output reg         valid_out,
    output reg  [7:0]  med
);

    //---- 第 1 步：三行各自排序 ----
    wire [7:0] max0, med0, min0;
    wire [7:0] max1, med1, min1;
    wire [7:0] max2, med2, min2;

    sort3 u_r0 (.a(p00), .b(p01), .c(p02), .vmax(max0), .vmed(med0), .vmin(min0));
    sort3 u_r1 (.a(p10), .b(p11), .c(p12), .vmax(max1), .vmed(med1), .vmin(min1));
    sort3 u_r2 (.a(p20), .b(p21), .c(p22), .vmax(max2), .vmed(med2), .vmin(min2));

    //---- 第 2 步：三个中间统计量 ----
    wire [7:0] mmax, mmed, mmin;                 // median of medians
    wire [7:0] xmax, xmed, xmin;
    wire [7:0] nmax, nmed, nmin;

    sort3 u_med (.a(med0), .b(med1), .c(med2), .vmax(),      .vmed(mmed), .vmin());
    sort3 u_min (.a(min0), .b(min1), .c(min2), .vmax(xmax),  .vmed(),    .vmin());
    sort3 u_max (.a(max0), .b(max1), .c(max2), .vmax(),      .vmed(),    .vmin(nmin));

    //---- 第 3 步：中值 = median(max_of_mins, med_of_meds, min_of_maxs) ----
    wire [7:0] fmax, fmed, fmin;
    sort3 u_fin (.a(xmax), .b(mmed), .c(nmin), .vmax(fmax), .vmed(fmed), .vmin(fmin));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            med       <= 8'd0;
        end else begin
            valid_out <= valid_in;
            med       <= fmed;
        end
    end

endmodule

====================================================================================================
 [ 5/29] src/sobel3x3.v    (68 行, 已例化)
====================================================================================================
//============================================================================
//  sobel3x3.v —— 3x3 Sobel 边缘检测（Gx / Gy / |Gx|+|Gy| / 可配置阈值）
//
//  Gx = [ -1 0 +1 ; -2 0 +2 ; -1 0 +1 ]
//  Gy = [ -1 -2 -1 ;  0 0  0 ; +1 +2 +1 ]
//  幅值 G = |Gx| + |Gy|（避免开方，省硬件）
//  二值化：G >= threshold ? 255 : 0   （threshold 可实时配置）
//
//  流水线：2 级
//============================================================================
module sobel3x3
(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        win_valid,
    input  wire [7:0]  p00, p01, p02,
    input  wire [7:0]  p10, p11, p12,
    input  wire [7:0]  p20, p21, p22,
    input  wire [7:0]  threshold,      // 阈值（由按键/寄存器给出）
    output reg         valid_out,
    output reg  [7:0]  edge_out,       // 二值边缘：255 / 0
    output reg  [7:0]  mag             // 幅值（截断到 8 位，便于叠加显示）
);

    //---------------- 第 1 级：算 Gx / Gy ----------------
    wire signed [11:0] gx =
          ( $signed({4'b0, p02}) + $signed({3'b0, p12, 1'b0}) + $signed({4'b0, p22}) )
        - ( $signed({4'b0, p00}) + $signed({3'b0, p10, 1'b0}) + $signed({4'b0, p20}) );

    wire signed [11:0] gy =
          ( $signed({4'b0, p00}) + $signed({3'b0, p01, 1'b0}) + $signed({4'b0, p02}) )
        - ( $signed({4'b0, p20}) + $signed({3'b0, p21, 1'b0}) + $signed({4'b0, p22}) );

    reg signed [11:0] gx_r, gy_r;
    reg               valid_d1;
    reg [7:0]         thr_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gx_r <= 12'sd0; gy_r <= 12'sd0;
            valid_d1 <= 1'b0; thr_r <= 8'd80;
        end else begin
            gx_r     <= gx;
            gy_r     <= gy;
            thr_r    <= threshold;
            valid_d1 <= win_valid;
        end
    end

    //---------------- 第 2 级：绝对值相加 + 阈值比较 ----------------
    wire [11:0] agx = gx_r[11] ? (~gx_r + 12'sd1) : gx_r;   // |Gx|
    wire [11:0] agy = gy_r[11] ? (~gy_r + 12'sd1) : gy_r;   // |Gy|
    wire [12:0] gsum = {1'b0, agx} + {1'b0, agy};           // G = |Gx|+|Gy|

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            edge_out  <= 8'd0;
            mag       <= 8'd0;
        end else begin
            valid_out <= valid_d1;
            edge_out  <= (gsum[12:0] >= {5'b0, thr_r}) ? 8'd255 : 8'd0;
            mag       <= (gsum[11:4] > 8'd255) ? 8'd255 : gsum[11:4];
        end
    end

endmodule

====================================================================================================
 [ 6/29] src/axi/axi4_ctrl.v    (609 行, 已例化)
====================================================================================================
//  by CrazyBird
module axi4_ctrl #(
	parameter 	C_ID_LEN      = 8, 
			C_DATA_LEN    = 128, 
			C_DATA_SIZE   = 4, 	//	C_DATA_LEN=8:0, 16:1, 32:2, 64:3, 128:4, 256:5, 512:6, 1024:7
			
			C_BURST_LEN   = 128,
			
			C_STRB_LEN    = C_DATA_LEN / 8, 
			C_ADDR_INC    = C_BURST_LEN * C_STRB_LEN,
			
			C_BASE_ADDR   = 32'h00000000, 	//	Start at 0x00000000 by default
			C_BUF_SIZE    = 22, 			//	Allocate 2^22 bytes (4MB) by default. The module takes 4*4MB by default. 
			C_RD_END_ADDR = 1280 * 720,
			
			C_W_WIDTH     = 64, 
			C_R_WIDTH     = 8
)(
    input  wire                 axi_clk         ,
    input  wire                 axi_reset       ,
    
    output       [C_ID_LEN-1:0]      axi_awid        ,
    output       [31:0]      axi_awaddr      ,
    output       [ 7:0]      axi_awlen       ,
    output       [ 2:0]      axi_awsize      ,
    output       [ 1:0]      axi_awburst     ,
    output                   axi_awlock      ,
    output       [ 3:0]      axi_awcache     ,
    output       [ 2:0]      axi_awprot      ,
    output       [ 3:0]      axi_awqos       ,
    output reg                  axi_awvalid     ,
    input  wire                 axi_awready     ,
    
    output       [C_DATA_LEN-1:0]      axi_wdata       ,
    output       [C_STRB_LEN-1:0]      axi_wstrb       ,
    output                   axi_wlast       ,
    output reg                  axi_wvalid      ,
    input  wire                 axi_wready      ,
    
    input  wire     [C_ID_LEN-1:0]      axi_bid         ,
    input  wire     [ 1:0]      axi_bresp       ,
    input  wire                 axi_bvalid      ,
    output                   axi_bready      ,
    
    output       [C_ID_LEN-1:0]      axi_arid        ,
    output       [31:0]      axi_araddr      ,
    output       [ 7:0]      axi_arlen       ,
    output       [ 2:0]      axi_arsize      ,
    output       [ 1:0]      axi_arburst     ,
    output                   axi_arlock      ,
    output       [ 3:0]      axi_arcache     ,
    output       [ 2:0]      axi_arprot      ,
    output       [ 3:0]      axi_arqos       ,
    output reg                  axi_arvalid     ,
    input  wire                 axi_arready     ,
    
    input  wire     [C_ID_LEN-1:0]      axi_rid         ,
    input  wire     [C_DATA_LEN-1:0]      axi_rdata       ,
    input  wire     [ 1:0]      axi_rresp       ,
    input  wire                 axi_rlast       ,
    input  wire                 axi_rvalid      ,
    output                   axi_rready      ,
    
    input  wire                 wframe_pclk     ,
    input  wire                 wframe_vsync    ,		//	Writter VSync. Flush on falling edge. Connect to EOF. 
    input  wire                 wframe_data_en  ,
    input  wire     [C_W_WIDTH-1:0]      wframe_data     ,
    
    input  wire                 rframe_pclk     ,
    input  wire                 rframe_vsync    ,		//	Reader VSync. Flush on falling edge. Connect to ~EOF. 
    input  wire                 rframe_data_en  ,
    output wire     [C_R_WIDTH-1:0]      rframe_data     ,
    
    output 		[31:0] 	tp_o
);
	
	initial begin
		axi_awvalid <= 0; 
		axi_wvalid <= 0; 
		axi_arvalid <= 0; 
	end

	assign axi_awid    = {C_ID_LEN{1'b0}};
	assign axi_awlen   = C_BURST_LEN - 1'b1;
	assign axi_awsize  = C_DATA_SIZE;
	assign axi_awburst = 2'b01;	//	INCR
	assign axi_awlock  = 1'b0;
	assign axi_awcache = 0;
	assign axi_awprot  = 3'b0;
	assign axi_awqos   = 4'b0;
	assign axi_wstrb   = 32'hFFFFFFFF;
	assign axi_bready  = 1; 
	    
	assign axi_arid    = {C_ID_LEN{1'b0}};
	assign axi_arlen   = C_BURST_LEN - 1'b1;
	assign axi_arsize  = C_DATA_SIZE;
	assign axi_arburst = 2'b01;	//	INCR
	assign axi_arlock  = 1'b0;
	assign axi_arcache = 0;
	assign axi_arprot  = 3'b0;
	assign axi_arqos   = 4'b0;
	assign axi_rready  = 1; 
	
	
	
	
	////////////////////////////////////////////////////////////////
	//	R/W Schedule
	reg	[1:0] 	rc_wframe_index;
	reg	[1:0] 	rc_rframe_index;

	//	Use 4 data buffers to simplify control. 
	wire 	[1:0] 	w_wframe_index_p1 = rc_wframe_index + 1; 
	wire 	[1:0] 	w_wframe_index_next = (w_wframe_index_p1 == rc_rframe_index) ? rc_wframe_index + 2 : rc_wframe_index + 1; 

	reg 	[1:0] 	r_wframe_index_last = 0; 
	
	reg 			r_wframe_inc = 0, r_rframe_inc = 0; 

	always @(posedge axi_clk) begin
		if(axi_reset) begin
			rc_wframe_index <= 2'b0;
			rc_rframe_index <= 2'd2;
			r_wframe_index_last <= 0; 
			
		end else begin
			rc_wframe_index <= rc_wframe_index; 
			rc_rframe_index <= rc_rframe_index; 
			
			//	When wfifo_rd_rst_busy_neg, write pointer increments. When rfifo_wr_rst_busy_neg, read pointer increments. 
			case ({r_rframe_inc, r_wframe_inc})
				2'b01: begin
						//	Write increment only. 
						rc_wframe_index <= w_wframe_index_next; 	//	Use 4 buffers. 
						r_wframe_index_last <= rc_wframe_index; 
					end
				2'b10: begin
						//	Use r_wframe_index_last. 
						rc_rframe_index <= r_wframe_index_last; 
					end
				2'b11: begin
						//	Update write & read pointer simutaneously. 
						rc_wframe_index <= w_wframe_index_next; 	//	Use 4 buffers. 
						rc_rframe_index <= rc_wframe_index; 		
					end
			endcase
			
		end
	end
	
	
	
	////////////////////////////////////////////////////////////////
	//	AXI Writter

	wire 				w_wfifo_pempty, w_wfifo_empty; 
	wire 				w_wfifo_ren; 
	wire 	[C_DATA_LEN-1:0] 	w_wfifo_rdata; 

	//	On EOF when ~empty, flush the last data then reset FIFO. 
	reg 	[3:0] 	rs_w = 0; 
	wire 	[3:0] 	ws_w_idle = 0; 
	wire 	[3:0] 	ws_w_wdata = 1; 
	wire 	[3:0] 	ws_w_winc = 2; 
	wire 	[3:0] 	ws_w_eof = 3; 

	//	Write Pointer. Burst bytes is always C_ADDR_INC. 
	reg 	[C_BUF_SIZE-1:0] 	rc_w_ptr = 0; 
	reg 	[0:0] 		rc_w_eof = 0; 
	reg 				r_w_rst = 1; 
	
	//	Burst Control
	reg 	[7:0] 	rc_burst = 0; 	//	Compare to C_BURST_LEN - 1. 
	
	//	EOF Monitor. 
	reg 	[1:0] 	r_wframe_sync = 0; 	
	reg 			r_weof_pending = 0; 

	always @(posedge axi_clk or posedge axi_reset) begin
		if(axi_reset) begin
			rs_w <= 0; 
			rc_w_ptr <= 0; 
			rc_w_eof <= 0; 
			r_w_rst <= 1; 
			axi_awvalid <= 0; 
			axi_wvalid <= 0; 
			r_wframe_sync <= 0; 
			r_weof_pending <= 0; 
			r_wframe_inc <= 0; 
			
		end else begin
			rc_w_eof <= 0; 
			r_wframe_inc <= 0; 
			
			//	Raise EOF on falling edge of VSYNC. 
			r_wframe_sync <= {r_wframe_sync, wframe_vsync}; 
			if(r_wframe_sync == 2'b10) begin
				r_weof_pending <= 1; 
			end else begin
			end
			
			if(axi_awready) begin
				axi_awvalid <= 0; 
			end else begin
			end
			
			case (rs_w)
				ws_w_idle: begin
						rc_burst <= 0; 
						r_w_rst <= 0; 
						
						if(~w_wfifo_pempty) begin
							axi_awvalid <= 1; 
							axi_wvalid <= 1; 
							rs_w <= ws_w_wdata; 
						
						end else if(r_weof_pending && (~w_wfifo_empty)) begin
							//	There's some data in the FIFO. write last. 
							axi_awvalid <= 1; 
							axi_wvalid <= 1; 
							rs_w <= ws_w_wdata; 
							
						end else if(r_weof_pending) begin
							//	EOF. Reset FIFO. Increment write pointer. 
							r_w_rst <= 1; 
							r_wframe_inc <= 1; 
							rs_w <= ws_w_eof; 
							
						end else begin
						end
					end
					
				ws_w_eof: begin
						//	Wait for some cycles to release reset. 
						r_weof_pending <= 0; 
						rc_w_ptr <= 0; 
						
						rc_w_eof <= rc_w_eof + 1; 
						if(&rc_w_eof)
							rs_w <= ws_w_idle; 
						else begin
						end
					end
					
				ws_w_wdata: begin
						rc_burst <= rc_burst + axi_wready; 
						
						//	On last transaction terminate write. 
						if(axi_wlast && axi_wready) begin
							axi_wvalid <= 0; 
							rs_w <= ws_w_winc; 
						end else begin
						end
					end
					
				ws_w_winc: begin
						rc_w_ptr <= rc_w_ptr + C_ADDR_INC; 
						rs_w <= ws_w_idle; 
					end
			endcase
		end
	end
	assign axi_awaddr = C_BASE_ADDR + {rc_wframe_index, rc_w_ptr}; 
	assign axi_wdata = w_wfifo_rdata; 
	assign axi_wlast = (rc_burst >= C_BURST_LEN - 1); 
	
	assign w_wfifo_ren = axi_wvalid && axi_wready; 
	
	
	assign tp_o[1:0] = {rs_w[1:0]}; 
	assign tp_o[2] = axi_wvalid; 
	assign tp_o[3] = wframe_data_en; 
	
	
	generate
		if(C_W_WIDTH == 64) begin: Gen_FIFO_W64
			W0_FIFO_64 u_W0_FIFO_64 
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (wframe_data_en),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end else if(C_W_WIDTH == 32) begin: Gen_FIFO_W32
			W0_FIFO_32 u_W0_FIFO_32
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (wframe_data_en),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end else if(C_W_WIDTH == 8) begin: Gen_FIFO_W8
			W0_FIFO_8 u_W0_FIFO_8 
			(
			    .wr_clk_i       (wframe_pclk    ),
			    .wr_en_i        (wframe_data_en),
			    .wdata          (wframe_data    ),
			    
			    .a_rst_i 	(r_w_rst), 

			    .rd_clk_i       (axi_clk        ),
			    .rd_en_i        (w_wfifo_ren     ),
			    .rdata          (w_wfifo_rdata    ),
			    .prog_empty_o	(w_wfifo_pempty),
			    .empty_o 		(w_wfifo_empty)
			);
		end
	endgenerate




















reg                             rframe_vsync_dly;

always @(posedge rframe_pclk)
begin
    if(axi_reset == 1'b1)
        rframe_vsync_dly <= 1'b0;
    else
        rframe_vsync_dly <= rframe_vsync;
end

wire                            rframe_vsync_neg;
assign rframe_vsync_neg =  rframe_vsync_dly & ~rframe_vsync;

reg             [4:0]           rfifo_cnt;

always @(posedge rframe_pclk)
begin
    if(axi_reset == 1'b1)
        rfifo_cnt <= 5'h1f;
    else
    begin
        if(rframe_vsync_neg == 1'b1)
            rfifo_cnt <= 5'h1f;
        else if(rfifo_cnt > 5'd0)
            rfifo_cnt <= rfifo_cnt - 1'b1;
        else
            rfifo_cnt <= 5'b0;
    end
end

reg                             rfifo_rst;

always @(posedge rframe_pclk)
begin
    if(axi_reset == 1'b1)
        rfifo_rst <= 1'b1;
    else
    begin
        if(rfifo_cnt == 5'b0)
            rfifo_rst <= 1'b0;
        else
            rfifo_rst <= 1'b1;
    end
end


wire 			w_rfifo_rst = axi_reset || rfifo_rst; 

reg                             rfifo_wenb;
reg             [C_DATA_LEN-1:0]          rfifo_wdata;
wire            [ 9:0]          rfifo_wcnt;
wire 					  rfifo_wfull; 
wire                            rfifo_wr_rst_busy;
wire 					w_rfifo_aempty; 
wire 					w_rfifo_empty; 


generate
	if(C_R_WIDTH == 8) begin: Gen_RFIFO_8
		R0_FIFO_8 u_R0_FIFO_8
		(
			.wr_clk_i		(axi_clk),
			.wr_en_i		(rfifo_wenb),
			.wdata			(rfifo_wdata),
			.a_rst_i		(w_rfifo_rst), 
			.prog_full_o		(rfifo_wfull),
			
			.rd_clk_i		(rframe_pclk),
			.rd_en_i		(rframe_data_en),
			.rdata			(rframe_data),
			.empty_o		(w_rfifo_empty),
			.almost_empty_o	(w_rfifo_aempty)
		);
	end else if(C_R_WIDTH == 16) begin: Gen_RFIFO_16	
		R0_FIFO_16 u_R0_FIFO_16
		(
			.wr_clk_i		(axi_clk),
			.wr_en_i		(rfifo_wenb),
			.wdata			(rfifo_wdata),
			.a_rst_i		(w_rfifo_rst), 
			.prog_full_o		(rfifo_wfull),
			
			.rd_clk_i		(rframe_pclk),
			.rd_en_i		(rframe_data_en),
			.rdata			(rframe_data),
			.empty_o		(w_rfifo_empty),
			.almost_empty_o	(w_rfifo_aempty)
		);
	end
endgenerate
		
reg r_rfifo_rst = 0; 
always @(posedge axi_clk)
begin
	r_rfifo_rst <= w_rfifo_rst; 
end

//assign tp_o[7:0] = {w_rfifo_empty, rfifo_wfull, rframe_data_en, rfifo_wenb, rfifo_rst, axi_reset, rframe_pclk, axi_clk}; 

reg             [15:0]               rfifo_wr_rst_busy_dly = 2'b0;

always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst == 1'b1)
        rfifo_wr_rst_busy_dly <= 0;
    else
        rfifo_wr_rst_busy_dly <= {rfifo_wr_rst_busy_dly,1'b1};
end

wire                                rfifo_wr_rst_busy_neg;
assign rfifo_wr_rst_busy_neg = (rfifo_wr_rst_busy_dly[1:0] == 2'b01) ? 1'b1 : 1'b0;

reg             [4:0]               read_ddr_delay_cnt;

always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst == 1'b1)
        read_ddr_delay_cnt <= 5'b0;
    else
    begin
        if(rfifo_wr_rst_busy_neg == 1'b1)
            read_ddr_delay_cnt <= 5'd31;
        else if(read_ddr_delay_cnt > 5'b0)
            read_ddr_delay_cnt <= read_ddr_delay_cnt - 1'b1;
        else
            read_ddr_delay_cnt <= 5'b0;
    end
end

always @(posedge axi_clk or posedge axi_reset) begin
	if(axi_reset)
		r_rframe_inc <= 0; 
	else
		r_rframe_inc <= rfifo_wr_rst_busy_neg; 
end


wire                                read_ddr_init_flag;
assign read_ddr_init_flag = (read_ddr_delay_cnt == 5'd1) ? 1'b1 : 1'b0;

//----------------------------------------------------------------------




//----------------------------------------------------------------------
localparam S_READ_IDLE = 2'd0;
localparam S_READ_ADDR = 2'd1;
localparam S_READ_DATA = 2'd2;

reg             [ 1:0]          rd_state;
reg             [ 8:0]          rdata_cnt;
reg             [C_BUF_SIZE-1:0]          araddr;
reg 					r_rd_pend = 0; 


always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst) begin
        rd_state <= S_READ_IDLE;
	  axi_arvalid <= 0; 
	  r_rd_pend <= 0; 
    end else
    begin
	
	
	
	if(axi_arready) begin
		axi_arvalid <= 0; 
	end else begin
	end
	if(axi_rvalid && axi_rlast) begin
		 r_rd_pend <= 0; 
	end else begin
	end
	
        case(rd_state)
            S_READ_IDLE : 
            begin
			//	Continue if RFIFO ~Full. 
                if(rfifo_wr_rst_busy_dly[15] && (araddr < C_RD_END_ADDR)&&(~rfifo_wfull)) begin	//	rfifo_wcnt < 10'd256))
                    rd_state <= S_READ_ADDR;
                end else
                    rd_state <= S_READ_IDLE;
            end
		
            S_READ_ADDR : 
            begin
				axi_arvalid <= 1; 
				r_rd_pend <= 1; 
                rd_state <= S_READ_DATA;
            end
            S_READ_DATA : 
            begin
			//	Return when ~arvalid && ~rd_pend. 
			if((~axi_arvalid) && (~r_rd_pend))
                //if((axi_rvalid == 1'b1)&&(axi_rready == 1'b1)&&(rdata_cnt == C_BURST_LEN))
                    rd_state <= S_READ_IDLE;
                else
                    rd_state <= S_READ_DATA;
            end
            default : 
            begin
                rd_state <= S_READ_IDLE;
            end
        endcase
    end
end

always @(posedge axi_clk or posedge r_rfifo_rst)
begin
    if(r_rfifo_rst == 1'b1)
        araddr <= 0;
    else
    begin
        if((axi_arvalid == 1'b1)&&(axi_arready == 1'b1))
            araddr <= araddr + C_ADDR_INC;
        else
            araddr <= araddr;
    end
end

assign axi_araddr = C_BASE_ADDR + {rc_rframe_index, araddr};


reg             [8:0]           rdata_cnt_dly;

always @(posedge axi_clk)
begin
    rdata_cnt_dly <= rdata_cnt;
end

always @(*)
begin
    if(rd_state == S_READ_DATA)
        begin
        if((axi_rvalid == 1'b1)&&(axi_rready == 1'b1))
            rdata_cnt = rdata_cnt_dly + 1'b1;
        else
            rdata_cnt = rdata_cnt_dly;
        end
    else
        rdata_cnt = 9'b0;
end

assign axi_rready = 1; 

always @(posedge axi_clk)
begin
    if(axi_reset == 1'b1)
        rfifo_wenb <= 1'b0;
    else
    begin
        if((axi_rvalid == 1'b1)&&(axi_rready == 1'b1))
            rfifo_wenb <= 1'b1;
        else
            rfifo_wenb <= 1'b0;
    end
end

always @(posedge axi_clk)
begin
    rfifo_wdata <= axi_rdata;
end

endmodule

====================================================================================================
 [ 7/29] src/axi/AXI4_AWARMux.v    (114 行, 已例化)
====================================================================================================
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    15:58:46 04/08/2023 
// Module Name:    AXI4_AWARMux 
//////////////////////////////////////////////////////////////////////////////////

//	AXI4 AW / AR Path Mux. 
//	This module works with Efinix AXI4-Modified Interface. 
//	atype_o = 1 for DDR Write, and atype_o = 0 for DDR Read. 

module AXI4_AWARMux #(
	parameter 	AID_LEN 	= 8, 
			AADDR_LEN 	= 32
)(
	input 				aclk_i, 
	input 				arst_i, 
	
	//	AW
	input 	[AID_LEN-1:0] 	awid_i,
	input 	[AADDR_LEN-1:0] 	awaddr_i,
	input 	[7:0] 		awlen_i,
	input 				awvalid_i,
	output reg				awready_o,
	
	//	AR
	input 	[AID_LEN-1:0] 	arid_i,
	input 	[AADDR_LEN-1:0] 	araddr_i,
	input 	[7:0] 		arlen_i,
	input 				arvalid_i,
	output reg				arready_o,
	
	//	A
	output 	[AID_LEN-1:0] 	aid_o,
	output 	[AADDR_LEN-1:0] 	aaddr_o,
	output 	[7:0] 		alen_o,
	output 				atype_o,
	output reg				avalid_o,
	input 				aready_i
);

	//	R/W Wrapper. atype_o = 1 for DDR Write, and atype_o = 0 for DDR Read. 
	//	Assert R/W in round robin mode. 
	reg 	[1:0] 	rs_req = 0; 		//	DDR3 Request State Machine
	wire 	[1:0] 	ws_req_idle = 0; 
	wire 	[1:0] 	ws_req_bridge = 1; 
	wire 	[1:0] 	ws_req_inc = 2; 
	
	reg 			r_reqtype = 0; 	//	Type = 0 for read, and type = 1 for write. Toggle when aready. 
	reg 			r_reqen = 0; 	//	Request En
	
	always @(posedge aclk_i) begin
		//	Clear AREADY by default. 
		arready_o <= 0; 
		awready_o <= 0; 
		
		case (rs_req) 
			ws_req_idle: begin
					//	By default toggle reqtype when no commands. 
					rs_req <= ws_req_inc; 
					
					//	Type = 0 for read. 
					if(r_reqtype == 0) begin
						//	When read request, bridge to AR path. 
						if(arvalid_i) begin
							avalid_o <= 1; 
							rs_req <= ws_req_bridge; 
						end else begin
						end
					end else begin
						//	When write request, bridge to AR path. 
						if(awvalid_i) begin
							avalid_o <= 1; 
							rs_req <= ws_req_bridge; 
						end else begin
						end
					end
				end
			ws_req_bridge: begin
					if(aready_i) begin
						avalid_o <= 0; 
						
						//	Respond to AR / AW. 
						arready_o <= (r_reqtype == 0); 
						awready_o <= (r_reqtype == 1); 
						rs_req <= ws_req_inc; 
					end else begin
					end
				end
			ws_req_inc: begin
					r_reqtype <= r_reqtype + 1; 
					rs_req <= ws_req_idle; 
				end
		endcase
		
		if(arst_i) begin
			rs_req <= 0; 
			r_reqtype <= 0; 
			r_reqen <= 0; 
			
			//awready_o <= 0; 
			//arready_o <= 0; 
			avalid_o <= 0; 
		end else begin
		end
	end
	
	//	Type = 0 for read. Bridge to AR path. 
	assign aid_o = (r_reqtype == 0) ? arid_i : awid_i; 
	assign aaddr_o = (r_reqtype == 0) ? araddr_i : awaddr_i; 
	assign alen_o = (r_reqtype == 0) ? arlen_i : awlen_i; 
	assign atype_o = r_reqtype; 

endmodule

====================================================================================================
 [ 8/29] src/lcd_driver.v    (151 行, 已例化)
====================================================================================================
/*-----------------------------------------------------------------------
								 \\\|///
							   \\  - -  //
								(  @ @  )
+-----------------------------oOOo-(_)-oOOo-----------------------------+
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2013-20xx CrazyBingo Corporation
The entire notice above must be reproduced on all authorized copies.
Author				:		CrazyBingo
Technology blogs 	: 		www.crazyfpga.com
Email Address 		: 		crazyfpga@vip.qq.com
Filename			:		lcd_driver.v
Date				:		2012-02-18
Description			:		LCD/VGA driver.
Modification History	:
Date			By			Version			Change Description
=========================================================================
12/02/18		CrazyBingo	1.0				Original
12/03/19		CrazyBingo	1.1				Modification
12/03/21		CrazyBingo	1.2				Modification
12/05/13		CrazyBingo	1.3				Modification
13/11/07		CrazyBingo	2.1				Modification
17/04/02		CrazyBingo	3.0				Modify for 12bit width logic
-------------------------------------------------------------------------
|                                     Oooo							|
+------------------------------oooO--(   )-----------------------------+
                              (   )   ) /
                               \ (   (_/
                                \_)
----------------------------------------------------------------------*/   

`timescale 1ns/1ps

`include "lcd_para.v"

module lcd_driver #(
	//	Setting up default timing
	parameter 	H_FRONT	= `H_FRONT,
			H_SYNC 	= `H_SYNC,
			H_BACK 	= `H_BACK,
			H_DISP	= `H_DISP,
			H_TOTAL	= `H_TOTAL,
						
			V_FRONT	= `V_FRONT,
			V_SYNC 	= `V_SYNC,
			V_BACK 	= `V_BACK,
			V_DISP 	= `V_DISP,
			V_TOTAL	= `V_TOTAL,
			
			DATA_WIDTH 	= 48
)(  	
	//global clock
	input					clk,			//system clock
	input					rst_n,     		//sync reset
	
	//lcd interface
	output				lcd_dclk,   	//lcd pixel clock
	output				lcd_blank,		//lcd blank
	output				lcd_sync,		//lcd sync
	output reg			lcd_hs,	    	//lcd horizontal sync
	output reg			lcd_vs,	    	//lcd vertical sync
	output reg			lcd_en,			//lcd display enable
	output	[DATA_WIDTH-1:0]	lcd_rgb,		//lcd display data

	//user interface
	output reg			lcd_request,	//lcd data request
	output reg	[11:0]	lcd_xpos,		//lcd horizontal coordinate
	output reg	[11:0]	lcd_ypos,		//lcd vertical coordinate
	input 	[DATA_WIDTH-1:0]	lcd_data		//lcd data
);	 

/*******************************************
		SYNC--BACK--DISP--FRONT
*******************************************/
//------------------------------------------
//h_sync counter & generator
reg [13:0] hcnt; 
always @ (posedge clk or negedge rst_n)
begin
	if (!rst_n)
		hcnt <= 12'd0;
	else
		begin
        if(hcnt < H_TOTAL - 1'b1)		//line over			
            hcnt <= hcnt + 1'b1;
        else
            hcnt <= 12'd0;
        
        lcd_hs <= (hcnt <= H_SYNC - 1'b1) ? 1'b0 : 1'b1;
		end
end 


//------------------------------------------
//v_sync counter & generator
reg [11:0] vcnt;
always@(posedge clk or negedge rst_n)
begin
	if (!rst_n)
		vcnt <= 12'b0;
	else if(hcnt == H_TOTAL - 1'b1)		//line over
		begin
		if(vcnt < V_TOTAL - 1'b1)		//frame over
			vcnt <= vcnt + 1'b1;
		else
			vcnt <= 12'd0;
            
        lcd_vs <= (vcnt <= V_SYNC - 1'b1) ? 1'b0 : 1'b1;
		end
end


//------------------------------------------
//LCELL	LCELL(.in(clk),.out(lcd_dclk));
assign	lcd_dclk = ~clk;
assign	lcd_blank = lcd_hs & lcd_vs;		
assign	lcd_sync = 1'b0;


//-----------------------------------------
reg     [DATA_WIDTH-1:0]  r_lcd_rgb = 0; 
always@(posedge clk) begin
	lcd_en		<=	(hcnt >= H_SYNC + H_BACK  && hcnt < H_SYNC + H_BACK + H_DISP) &&
						(vcnt >= V_SYNC + V_BACK  && vcnt < V_SYNC + V_BACK + V_DISP) 
						? 1'b1 : 1'b0;
    r_lcd_rgb 	<= 	lcd_data;
end 
assign lcd_rgb = lcd_en ? r_lcd_rgb : 0; 

//------------------------------------------
//ahead x clock
localparam	H_AHEAD = 	12'd1;

//	Standard FIFO Request Port
 always@(posedge clk) begin
	lcd_request	<=	(hcnt >= H_SYNC + H_BACK - H_AHEAD && hcnt < H_SYNC + H_BACK + H_DISP - H_AHEAD) &&
						 (vcnt >= V_SYNC + V_BACK && vcnt < V_SYNC + V_BACK + V_DISP) 
						 ? 1'b1 : 1'b0;

                         
// lcd xpos & ypos
	lcd_xpos	<= 	lcd_request ? (hcnt - (H_SYNC + H_BACK - H_AHEAD)) : 11'd0;
	lcd_ypos	<= 	lcd_request ? (vcnt - (V_SYNC + V_BACK)) : 12'd0;
end


endmodule

====================================================================================================
 [ 9/29] src/lcd_para.v    (202 行, 已例化)
====================================================================================================
/*-----------------------------------------------------------------------
								 \\\|///
							   \\  - -  //
								(  @ @  )
+-----------------------------oOOo-(_)-oOOo-----------------------------+
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2013-20xx CrazyBingo Corporation
The entire notice above must be reproduced on all authorized copies.
Author				:		CrazyBingo
Technology blogs 	: 		www.crazyfpga.com
Email Address 		: 		crazyfpga@vip.qq.com
Filename			:		lcd_para.v
Date				:		2012-02-18
Description			:		LCD/VGA driver parameter.
Modification History	:
Date			By			Version			Change Description
=========================================================================
12/02/18		CrazyBingo	1.0				Original
12/03/19		CrazyBingo	1.1				Modification
12/03/21		CrazyBingo	1.2				Modification
12/05/13		CrazyBingo	1.3				Modification
13/11/07		CrazyBingo	2.1				Modification
17/04/02		CrazyBingo	3.0				Modify for 720P & 1080P & 800*480
-------------------------------------------------------------------------
|                                     Oooo								|
+------------------------------oooO--(   )-----------------------------+
                              (   )   ) /
                               \ (   (_/
                                \_)
----------------------------------------------------------------------*/

`timescale 1ns/1ps

//-----------------------------------------------------------------------
//Define the color parameter RGB--8|8|8
//define colors RGB--8|8|8
`define RED		24'hFF0000   /*11111111,00000000,00000000	*/
`define GREEN	24'h00FF00   /*00000000,11111111,00000000	*/
`define BLUE  	24'h0000FF   /*00000000,00000000,11111111	*/
`define WHITE 	24'hFFFFFF   /*11111111,11111111,11111111	*/
`define BLACK 	24'h000000   /*00000000,00000000,00000000	*/
`define YELLOW	24'hFFFF00   /*11111111,11111111,00000000	*/
`define CYAN  	24'hFF00FF   /*11111111,00000000,11111111	*/
`define ROYAL 	24'h00FFFF   /*00000000,11111111,11111111	*/ 

//---------------------------------
`define	SYNC_POLARITY 1'b0

//------------------------------------
//vga parameter define
//`define VGA_854_480_60FPS_30MHz
//`define	VGA_640_480_60FPS_25MHz
//`define 	VGA_800_480_30FPS_16MHz
//`define	VGA_800_600_60FPS_40MHz
//`define	VGA_1280_720_60FPS_74_25MHz
`define	VGA_1920_1080_60FPS_148_5MHz
//`define   VGA_1024_600_60FPS_50MHz
//`define	DSI_1024_600_60FPS_50MHz

//---------------------------------
//	854 * 480
`ifdef	VGA_854_480_60FPS_30MHz
`define	H_FRONT	10'd50
`define	H_SYNC 	10'd16  
`define	H_BACK 	10'd40  
`define	H_DISP	10'd854 
`define	H_TOTAL	10'd960 	
 				
`define	V_FRONT	10'd30  
`define	V_SYNC 	10'd3  
`define	V_BACK 	10'd12 
`define	V_DISP 	10'd480   
`define	V_TOTAL	10'd525
`endif


//---------------------------------
//	640 * 480
`ifdef	VGA_640_480_60FPS_25MHz
`define	H_FRONT	12'd16
`define	H_SYNC 	12'd96  
`define	H_BACK 	12'd48  
`define	H_DISP	12'd640 
`define	H_TOTAL	12'd800 	
 				
`define	V_FRONT	12'd10  
`define	V_SYNC 	12'd2   
`define	V_BACK 	12'd33 
`define	V_DISP 	12'd480   
`define	V_TOTAL	12'd525
`endif

//---------------------------------
//	800 * 600
`ifdef VGA_800_600_60FPS_40MHz 
`define	H_FRONT	12'd40
`define	H_SYNC 	12'd128  
`define	H_BACK 	12'd88  
`define	H_DISP 	12'd800
`define	H_TOTAL	12'd1056 
				
`define	V_FRONT	12'd1 
`define	V_SYNC 	12'd4   
`define	V_BACK 	12'd23  
`define	V_DISP 	12'd600  
`define	V_TOTAL	12'd628
`endif


//---------------------------------
//	1024*600 DSI (Dual Output)
`ifdef	DSI_1024_600_60FPS_50MHz
`define	H_FRONT	151
`define	H_SYNC 	1
`define	H_BACK 	1
`define	H_DISP	512
`define	H_TOTAL	1330
 				
`define	V_FRONT	12'd32
`define	V_SYNC 	12'd5   
`define	V_BACK 	12'd8 
`define	V_DISP 	12'd600   
`define	V_TOTAL	12'd645
`endif


//---------------------------------
//	1280 * 720
`ifdef	VGA_1280_720_60FPS_74_25MHz
`define	H_FRONT	12'd110
`define	H_SYNC 	12'd40
`define	H_BACK 	12'd220
`define	H_DISP	12'd1280
`define	H_DISP_REQ	12'd1280
`define	H_TOTAL	12'd1650
 				
`define	V_FRONT	12'd5
`define	V_SYNC 	12'd5   
`define	V_BACK 	12'd20 
`define	V_DISP 	12'd720   
`define	V_DISP_REQ 	12'd720  
`define	V_TOTAL	12'd750
`endif


//---------------------------------
//	1920 * 1080
`ifdef	VGA_1920_1080_60FPS_148_5MHz
`define	H_FRONT	12'd88
`define	H_SYNC 	12'd44
`define	H_BACK 	12'd148
`define	H_DISP	12'd1920
`define	H_TOTAL	12'd2200
 					
`define	V_FRONT	12'd4
`define	V_SYNC 	12'd5   
`define	V_BACK 	12'd36
//`define	V_FRONT	12'd3
//`define	V_SYNC 	12'd2
//`define	V_BACK 	12'd3
`define	V_DISP 	12'd1080 
`define	V_TOTAL	12'd1125
//`define	V_TOTAL	12'd1088
`endif


//---------------------------------
//	1024 * 600
`ifdef	VGA_1024_600_60FPS_50MHz
`define	H_FRONT	11'd160
`define	H_SYNC 	11'd40
`define	H_BACK 	11'd120
`define	H_DISP	11'd1024
`define	H_TOTAL	11'd1344
 					
`define	V_FRONT	11'd12 
`define	V_SYNC 	11'd3   
`define	V_BACK 	11'd20 
`define	V_DISP 	11'd600   
`define	V_TOTAL	11'd635
`endif


`ifdef	VGA_800_480_30FPS_16MHz
`define	H_FRONT	11'd80
`define	H_SYNC 	11'd32
`define	H_BACK 	11'd48
`define	H_DISP	11'd800
`define	H_TOTAL	11'd960
 					
`define	V_FRONT	11'd12 
`define	V_SYNC 	11'd3   
`define	V_BACK 	11'd5
`define	V_DISP 	11'd480
`define	V_TOTAL	11'd500
`endif

====================================================================================================
 [10/29] src/isp/VIP_RAW8_RGB888.v    (332 行, 已例化)
====================================================================================================
/*-----------------------------------------------------------------------
                                 \\\|///
                               \\  - -  //
                                (  @ @  )
+-----------------------------oOOo-(_)-oOOo-----------------------------+
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2013-20xx CrazyBingo Corporation
The entire notice above must be reproduced on all authorized copies.
Author              :       CrazyBingo
Technology blogs    :       www.crazyfpga.com
Email Address       :       crazyfpga@vip.qq.com
Filename            :       VIP_YCbCr444_RGB888.v
Date                :       2014-05-03
Description         :       The testbench of VIP_YCbCr444_RGB888.
Modification History    :
Date            By          Version         Change Description
=========================================================================
14/05/03        CrazyBingo  1.0             Original
-------------------------------------------------------------------------
|                                     Oooo                              |
+-------------------------------oooO--(   )-----------------------------+
                               (   )   ) /
                                \ (   (_/
                                 \_)
-----------------------------------------------------------------------*/

`timescale 1ns/1ns
module VIP_RAW8_RGB888
#(
    parameter   [13:0]  IMG_HDISP = 640,    //640*480
    parameter   [13:0]  IMG_VDISP = 480
)
(
    //global clock
    input               clk,                //cmos video pixel clock
    input               rst_n,              //global reset
    
    input       [1:0]   mirror,

    //CMOS YCbCr444 data output
    input               per_frame_vsync,    //Prepared Image data vsync valid signal
    input               per_frame_href,     //Prepared Image data href vaild  signal
    input               per_frame_hsync, 
    input       [7:0]   per_img_RAW,        //Prepared Image data 8 Bit RAW Data

    
    //CMOS RGB888 data output
    output              post_frame_vsync,   //Processed Image data vsync valid signal
    output              post_frame_href,    //Processed Image data href vaild  signal
    output              post_frame_hsync,   //Processed Image data href vaild  signal
    output reg  [7:0]   post_img_red,       //Prepared Image green data to be processed 
    output reg  [7:0]   post_img_green,     //Prepared Image green data to be processed
    output reg  [7:0]   post_img_blue       //Prepared Image blue data to be processed
);

//----------------------------------------------------
//Generate 8Bit 3X3 Matrix for Video Image Processor.
    //Image data has been processd
wire                matrix_frame_vsync; //Prepared Image data vsync valid signal
wire                matrix_frame_href;  //Prepared Image data href vaild  signal
wire                matrix_frame_hsync; //Prepared Image data href vaild  signal
wire        [7:0]   matrix_p11, matrix_p12, matrix_p13; //3X3 Matrix output
wire        [7:0]   matrix_p21, matrix_p22, matrix_p23;
wire        [7:0]   matrix_p31, matrix_p32, matrix_p33;
VIP_Matrix_Generate_3X3_8Bit    
#(
    .IMG_HDISP  (IMG_HDISP),    //640*480
    .IMG_VDISP  (IMG_VDISP)
)
u_VIP_Matrix_Generate_3X3_8Bit
(
    //global clock
    .clk                    (clk),                  //cmos video pixel clock
    .rst_n                  (rst_n),                //global reset

    //Image data prepred to be processd
    .per_frame_vsync        (per_frame_vsync),      //Prepared Image data vsync valid signal
    .per_frame_href         (per_frame_href),       //Prepared Image data href vaild  signal
    .per_frame_hsync        (per_frame_hsync),      //Prepared Image data href vaild  signal
    .per_img_Y              (per_img_RAW),          //Prepared Image brightness input

    //Image data has been processd
    .matrix_frame_vsync     (matrix_frame_vsync),   //Processed Image data vsync valid signal
    .matrix_frame_href      (matrix_frame_href),    //Processed Image data href vaild  signal
    .matrix_frame_hsync     (matrix_frame_hsync),   //Processed Image data href vaild  signal
    .matrix_p11(matrix_p11),    .matrix_p12(matrix_p12),    .matrix_p13(matrix_p13),    //3X3 Matrix output
    .matrix_p21(matrix_p21),    .matrix_p22(matrix_p22),    .matrix_p23(matrix_p23),
    .matrix_p31(matrix_p31),    .matrix_p32(matrix_p32),    .matrix_p33(matrix_p33)
);


//-------------------------------------------------------------
//sync the frame vsync and href signal and generate frame begin & end signal
reg     matrix_frame_href_r;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        matrix_frame_href_r <= 0;
    else
        matrix_frame_href_r <= matrix_frame_href;
end
wire    matrix_frame_href_end   =   (matrix_frame_href_r & ~matrix_frame_href) ? 1'b1 : 1'b0;   //Line over signal

//----------------------------------------
//Count the frame lines
reg [10:0]  line_cnt;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        line_cnt <= 0;
    else if(matrix_frame_vsync == 1'b1) //Frame valid
        begin
        if(matrix_frame_href_end)
            line_cnt <= (line_cnt < IMG_VDISP - 1'b1) ? line_cnt + 1'b1 : 11'd0;
        else
            line_cnt <= line_cnt;
        end
    else
        line_cnt <= 0;
end

//----------------------------------------
//Count the pixels
reg [13:0]  point_cnt;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        point_cnt <= 0;
    else if(matrix_frame_href == 1'b1)  //Line valid
        point_cnt <= (line_cnt < IMG_HDISP - 1'b1) ? point_cnt + 1'b1 : 11'd0;
    else
        point_cnt <= 0;
end

//--------------------------------------
//Convet RAW 2 RGB888 Format
//
localparam  OddLINE_OddPOINT    =   2'b11;  //odd lines + odd point
localparam  OddLINE_Even_POINT  =   2'b10;  //odd lines + even point
localparam  EvenLINE_OddPOINT   =   2'b01;  //even lines + odd point
localparam  EvenLINE_EvenPOINT  =   2'b00;  //even lines + even point
reg [9:0]   post_img_red_r;
reg [9:0]   post_img_green_r;
reg [9:0]   post_img_blue_r;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
    begin
        post_img_red_r  <=  0;
        post_img_green_r<=  0;
        post_img_blue_r <=  0;
    end
    else
    begin
        case(mirror)
            2'b10 : 
            begin
                case({line_cnt[0], point_cnt[0]})
                    //-------------------------GRGR...GRGR--------------------//
                    OddLINE_OddPOINT:   //odd lines + odd point
                    begin   //Center Red
                        post_img_red_r  <=  matrix_p22;
                        post_img_green_r<=  (matrix_p12 + matrix_p21 + matrix_p23 + matrix_p32)>>2;
                        post_img_blue_r <=  (matrix_p11 + matrix_p13 + matrix_p31 + matrix_p33)>>2;
                    end
                    OddLINE_Even_POINT: //odd lines + even point
                    begin   //Center Green  
                        post_img_red_r  <=  (matrix_p21 + matrix_p23)>>1;
                        post_img_green_r<=  matrix_p22;
                        post_img_blue_r <=  (matrix_p12 + matrix_p32)>>1;
                    end
                    //-------------------------BGBG...BGBG--------------------//
                    EvenLINE_OddPOINT:  //even lines + odd point
                    begin   //Center Green
                        post_img_red_r  <=  (matrix_p12 + matrix_p32)>>1;
                        post_img_green_r<=  matrix_p22;
                        post_img_blue_r <=  (matrix_p21 + matrix_p23)>>1;
                    end
                    EvenLINE_EvenPOINT: //even lines + even point
                    begin   //Center Blue
                        post_img_red_r  <=  (matrix_p11 + matrix_p13 + matrix_p31 + matrix_p33)>>2;
                        post_img_green_r<=  (matrix_p12 + matrix_p21 + matrix_p23 + matrix_p32)>>2;
                        post_img_blue_r <=  matrix_p22;     
                    end
                endcase
            end
            2'b11 : 
            begin
                case({line_cnt[0], point_cnt[0]})
                   //-------------------------GRGR...GRGR--------------------//
                   OddLINE_OddPOINT:   //odd lines + odd point
                   begin   //Center Green  
                       post_img_red_r  <=  (matrix_p21 + matrix_p23)>>1;
                       post_img_green_r<=  matrix_p22;
                       post_img_blue_r <=  (matrix_p12 + matrix_p32)>>1;
                   end
                   OddLINE_Even_POINT: //odd lines + even point
                   begin   //Center Red
                       post_img_red_r  <=  matrix_p22;
                       post_img_green_r<=  (matrix_p12 + matrix_p21 + matrix_p23 + matrix_p32)>>2;
                       post_img_blue_r <=  (matrix_p11 + matrix_p13 + matrix_p31 + matrix_p33)>>2;
                   end
                   //-------------------------BGBG...BGBG--------------------//
                   EvenLINE_OddPOINT:  //even lines + odd point
                   begin   //Center Blue
                       post_img_red_r  <=  (matrix_p11 + matrix_p13 + matrix_p31 + matrix_p33)>>2;
                       post_img_green_r<=  (matrix_p12 + matrix_p21 + matrix_p23 + matrix_p32)>>2;
                       post_img_blue_r <=  matrix_p22;     
                   end
                   EvenLINE_EvenPOINT: //even lines + even point
                   begin   //Center Green
                       post_img_red_r  <=  (matrix_p12 + matrix_p32)>>1;
                       post_img_green_r<=  matrix_p22;
                       post_img_blue_r <=  (matrix_p21 + matrix_p23)>>1;
                   end
                endcase
            end
            2'b00 : 
            begin
                case({line_cnt[0], point_cnt[0]})
                    //-------------------------GBGB...GBGB--------------------//
                    OddLINE_OddPOINT:   //odd lines + odd point
                    begin   //Center Green
                        post_img_red_r  <=  (matrix_p12 + matrix_p32)>>1;
                        post_img_green_r<=  matrix_p22;
                        post_img_blue_r <=  (matrix_p21 + matrix_p23)>>1;
                    end
                    OddLINE_Even_POINT: //odd lines + even point
                    begin   //Center Blue
                        post_img_red_r  <=  (matrix_p11 + matrix_p13 + matrix_p31 + matrix_p33)>>2;
                        post_img_green_r<=  (matrix_p12 + matrix_p21 + matrix_p23 + matrix_p32)>>2;
                        post_img_blue_r <=  matrix_p22;
                    end
                    //-------------------------RGRG...RGRG--------------------//
                    EvenLINE_OddPOINT:  //even lines + odd point
                    begin   //Center Red
                        post_img_red_r  <=  matrix_p22;
                        post_img_green_r<=  (matrix_p12 + matrix_p21 + matrix_p23 + matrix_p32)>>2;
                        post_img_blue_r <=  (matrix_p11 + matrix_p13 + matrix_p31 + matrix_p33)>>2;
                    end
                    EvenLINE_EvenPOINT: //even lines + even point
                    begin   //Center Green  
                        post_img_red_r  <=  (matrix_p21 + matrix_p23)>>1;
                        post_img_green_r<=  matrix_p22;
                        post_img_blue_r <=  (matrix_p12 + matrix_p32)>>1;
                    end
                endcase
            end
            2'b01 : 
            begin
                case({line_cnt[0], point_cnt[0]})
                    //-------------------------BGBG...BGBG--------------------//
                    OddLINE_OddPOINT:   //odd lines + odd point
                    begin   //Center Blue
                        post_img_red_r  <=  (matrix_p11 + matrix_p13 + matrix_p31 + matrix_p33)>>2;
                        post_img_green_r<=  (matrix_p12 + matrix_p21 + matrix_p23 + matrix_p32)>>2;
                        post_img_blue_r <=  matrix_p22;     
                    end
                    OddLINE_Even_POINT: //odd lines + even point
                    begin   //Center Green
                        post_img_red_r  <=  (matrix_p12 + matrix_p32)>>1;
                        post_img_green_r<=  matrix_p22;
                        post_img_blue_r <=  (matrix_p21 + matrix_p23)>>1;
                    end
                    //-------------------------GRGR...GRGR--------------------//
                    EvenLINE_OddPOINT:  //even lines + odd point
                    begin   //Center Green  
                        post_img_red_r  <=  (matrix_p21 + matrix_p23)>>1;
                        post_img_green_r<=  matrix_p22;
                        post_img_blue_r <=  (matrix_p12 + matrix_p32)>>1;
                    end
                    EvenLINE_EvenPOINT: //even lines + even point
                    begin   //Center Red
                        post_img_red_r  <=  matrix_p22;
                        post_img_green_r<=  (matrix_p12 + matrix_p21 + matrix_p23 + matrix_p32)>>2;
                        post_img_blue_r <=  (matrix_p11 + matrix_p13 + matrix_p31 + matrix_p33)>>2;
                    end
                endcase
            end
        endcase
    end
end

// assign  post_img_red    =   post_img_red_r[7:0];
// assign  post_img_green  =   post_img_green_r[7:0];
// assign  post_img_blue   =   post_img_blue_r[7:0];

always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
    begin
        post_img_red <= 8'b0;
        post_img_green <= 8'b0;
        post_img_blue <= 8'b0;
    end
    else
    begin
        post_img_red <= post_img_red_r[7:0];
        post_img_green <= post_img_green_r[7:0];
        post_img_blue <= post_img_blue_r[7:0];
    end
end

//------------------------------------------
//lag n clocks signal sync      
reg [1:0]   post_frame_vsync_r;
reg [1:0]   post_frame_href_r;
reg [1:0]   post_frame_hsync_r;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
        post_frame_vsync_r  <=  0;
        post_frame_href_r   <=  0;
        post_frame_hsync_r  <=  0;
        end
    else
        begin
        post_frame_vsync_r  <=  {post_frame_vsync_r[0], matrix_frame_vsync};
        post_frame_href_r   <=  {post_frame_href_r[0],  matrix_frame_href};
        post_frame_hsync_r  <=  {post_frame_hsync_r[0], matrix_frame_hsync};
        end
end
assign  post_frame_vsync    =   post_frame_vsync_r[1];
assign  post_frame_href     =   post_frame_href_r[1];
assign  post_frame_hsync    =   post_frame_hsync_r[1];

endmodule

====================================================================================================
 [11/29] src/isp/VIP_Matrix_Generate_3X3_8Bit.v    (222 行, 已例化)
====================================================================================================
/*-----------------------------------------------------------------------
                                 \\\|///
                               \\  - -  //
                                (  @ @  )
+-----------------------------oOOo-(_)-oOOo-----------------------------+
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2011-20xx CrazyBingo Corporation
The entire notice above must be reproduced on all authorized copies.
Author              :       CrazyBingo
Technology blogs    :       www.crazyfpga.com
Email Address       :       crazyfpga@vip.qq.com
Filename            :       VIP_Matrix_Generate_3X3_8Bit.v
Date                :       2014-03-19
Description         :       Generate 8Bit 3X3 Matrix for Video Image Processor.
                            Give up the 1th and 2th row edge data caculate for simple process
                            Give up the 1th and 2th point of 1 line for simple process
Modification History    :
Date            By          Version         Change Description
=========================================================================
13/05/26        CrazyBingo  1.0             Original
14/03/16        CrazyBingo  2.0             Modification
22/07/04        CrazyBird   3.0             Modification
-------------------------------------------------------------------------
|                                     Oooo                              |
+-------------------------------oooO--(   )-----------------------------+
                               (   )   ) /
                                \ (   (_/
                                 \_)
-----------------------------------------------------------------------*/ 

`timescale 1ns/1ns
module VIP_Matrix_Generate_3X3_8Bit
#(
    parameter   [13:0]  IMG_HDISP = 11'd640,    //640*480
    parameter   [13:0]  IMG_VDISP = 11'd480
)
(
    //global clock
    input               clk,                //cmos video pixel clock
    input               rst_n,              //global reset

    //Image data prepred to be processd
    input               per_frame_vsync,    //Prepared Image data vsync valid signal
    input               per_frame_href,     //Prepared Image data href vaild  signal
    input               per_frame_hsync,    //Prepared Image data href vaild  signal
    input       [7:0]   per_img_Y,          //Prepared Image brightness input

    //Image data has been processd
    output              matrix_frame_vsync, //Prepared Image data vsync valid signal
    output              matrix_frame_href,  //Prepared Image data href vaild  signal
    output              matrix_frame_hsync, //Prepared Image data href vaild  signal

    output  reg [7:0]   matrix_p11, matrix_p12, matrix_p13, //3X3 Matrix output
    output  reg [7:0]   matrix_p21, matrix_p22, matrix_p23,
    output  reg [7:0]   matrix_p31, matrix_p32, matrix_p33
);


//Generate 3*3 matrix 
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//sync row3_data with per_frame_clken & row1_data & raw2_data
wire    [7:0]   row1_data;  //frame data of the 1th row
wire    [7:0]   row2_data;  //frame data of the 2th row
reg     [7:0]   row3_data;  //frame data of the 3th row
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        row3_data <= 0;
    else 
    begin
        if(per_frame_href)
            row3_data <= per_img_Y;
        else
            row3_data <= row3_data;
    end 
end

//---------------------------------------
//module of shift ram for raw data
wire    shift_clk_en = per_frame_href;

Line_Shift_RAM_8Bit
#(
    .DATA_WIDTH (8          ),
    .ADDR_WIDTH (11         ),
    .DATA_DEPTH (IMG_HDISP  ),
    .DELAY_NUM  (0          )
)
u0_Line_Shift_RAM_8Bit
(
    .clk    (clk            ),
    .rst_n  (rst_n          ),
    .clken  (shift_clk_en   ),
    .din    (per_img_Y      ),   
    .dout   (row2_data      )
);

Line_Shift_RAM_8Bit
#(
    .DATA_WIDTH (8          ),
    .ADDR_WIDTH (11         ),
    .DATA_DEPTH (IMG_HDISP  ),
    .DELAY_NUM  (1          )
)
u1_Line_Shift_RAM_8Bit
(
    .clk    (clk            ),
    .rst_n  (rst_n          ),
    .clken  (shift_clk_en   ),
    .din    (row2_data      ),   
    .dout   (row1_data      )
);

//------------------------------------------
//lag 2 clocks signal sync  
reg [2:0]   per_frame_vsync_r;
reg [2:0]   per_frame_href_r;   
reg [2:0]   per_frame_hsync_r; 
reg [2:0]   per_frame_clken_r;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
        per_frame_vsync_r <= 0;
        per_frame_href_r <= 0;
	  per_frame_hsync_r <= 0; 
        end
    else
        begin
        per_frame_vsync_r   <=  {per_frame_vsync_r[1:0],    per_frame_vsync};
        per_frame_href_r    <=  {per_frame_href_r[1:0],     per_frame_href};
	  per_frame_hsync_r   <=  {per_frame_hsync_r[1:0],    per_frame_hsync}; 
        end
end
//Give up the 1th and 2th row edge data caculate for simple process
//Give up the 1th and 2th point of 1 line for simple process
wire    read_frame_href     =   per_frame_href_r[0]|per_frame_href_r[1];    //RAM read href sync signal
assign  matrix_frame_vsync  =   per_frame_vsync_r[2];
assign  matrix_frame_href   =   per_frame_href_r[2];
assign  matrix_frame_hsync  =   per_frame_hsync_r[2]; 


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
/******************************************************************************
                    ----------  Convert Matrix  ----------
                [ P31 -> P32 -> P33 -> ]    --->    [ P11 P12 P13 ] 
                [ P21 -> P22 -> P23 -> ]    --->    [ P21 P22 P23 ]
                [ P11 -> P12 -> P11 -> ]    --->    [ P31 P32 P33 ]
******************************************************************************/
//---------------------------------------------------------------------------
//---------------------------------------------------
/***********************************************
    (1) Read data from Shift_RAM
    (2) Caculate the Sobel
    (3) Steady data after Sobel generate
************************************************/
//wire  [23:0]  matrix_row1 = {matrix_p11, matrix_p12, matrix_p13}; //Just for test
//wire  [23:0]  matrix_row2 = {matrix_p21, matrix_p22, matrix_p23};
//wire  [23:0]  matrix_row3 = {matrix_p31, matrix_p32, matrix_p33};
reg     [13:0]  pixel_cnt;
reg     [7:0]   row1_data0, row1_data1, row2_data0, row2_data1, row3_data0, row3_data1;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
        pixel_cnt <= 0;
        row1_data0 <= 0; row1_data1 <= 0;
        row2_data0 <= 0; row2_data1 <= 0;
        row3_data0 <= 0; row3_data1 <= 0;
        {matrix_p11, matrix_p12, matrix_p13} <= 24'h0;
        {matrix_p21, matrix_p22, matrix_p23} <= 24'h0;
        {matrix_p31, matrix_p32, matrix_p33} <= 24'h0;
        end
    else if(read_frame_href)
        begin
        pixel_cnt <=  (pixel_cnt < IMG_HDISP) ? pixel_cnt + 1'b1 : 10'd0;   //Point Counter
        {row1_data1, row1_data0} <= {row1_data0, row1_data};
        {row2_data1, row2_data0} <= {row2_data0, row2_data};
        {row3_data1, row3_data0} <= {row3_data0, row3_data};
        if(pixel_cnt == 0)
            begin
            {matrix_p11, matrix_p12, matrix_p13} <= 0;
            {matrix_p21, matrix_p22, matrix_p23} <= 0;
            {matrix_p31, matrix_p32, matrix_p33} <= 0;
            end
        else if(pixel_cnt == 1)         //First point
            begin   
            {matrix_p11, matrix_p12, matrix_p13} <= {row1_data, row1_data0, row1_data};         
            {matrix_p21, matrix_p22, matrix_p23} <= {row2_data, row2_data0, row2_data}; 
            {matrix_p31, matrix_p32, matrix_p33} <= {row3_data, row3_data0, row3_data}; 
            end
        else if(pixel_cnt == IMG_HDISP) //Last Point        
            begin       
            {matrix_p11, matrix_p12, matrix_p13} <= {row1_data1, row1_data, row1_data1};
            {matrix_p21, matrix_p22, matrix_p23} <= {row2_data1, row2_data, row2_data1};
            {matrix_p31, matrix_p32, matrix_p33} <= {row3_data1, row3_data, row3_data1};
            end
        else                            //2 ~ IMG_HDISP-1 Point
            begin

            {matrix_p11, matrix_p12, matrix_p13} <= {row1_data1, row1_data0, row1_data};
            {matrix_p21, matrix_p22, matrix_p23} <= {row2_data1, row2_data0, row2_data};
            {matrix_p31, matrix_p32, matrix_p33} <= {row3_data1, row3_data0, row3_data};
            end
        end
    else
        begin
        pixel_cnt <= 0;
        {matrix_p11, matrix_p12, matrix_p13} <= 24'h0;
        {matrix_p21, matrix_p22, matrix_p23} <= 24'h0;
        {matrix_p31, matrix_p32, matrix_p33} <= 24'h0;
        end
end

endmodule

====================================================================================================
 [12/29] src/isp/Line_Shift_RAM_8Bit.v    (101 行, 已例化)
====================================================================================================
//  by CrazyBird
module Line_Shift_RAM_8Bit
#(
    parameter DATA_WIDTH = 8    ,
    parameter ADDR_WIDTH = 11   ,
    parameter DATA_DEPTH = 1280 ,
    parameter DELAY_NUM  = 0
)(
    input  wire                     clk     ,
    input  wire                     rst_n   ,
    input  wire                     clken   ,
    input  wire [DATA_WIDTH-1:0]    din     ,   
    output wire [DATA_WIDTH-1:0]    dout    
);
//----------------------------------------------------------------------
localparam BRAM_DEPTH = DATA_DEPTH + 1;
localparam INIT_ADDR  = DATA_DEPTH - DELAY_NUM;

//----------------------------------------------------------------------
reg             [ADDR_WIDTH-1:0]    bram_waddr;
reg             [ADDR_WIDTH-1:0]    bram_raddr;

always @(posedge clk or negedge rst_n)
begin
    if(rst_n == 1'b0)
    begin
        bram_waddr <= INIT_ADDR;
        bram_raddr <= 0;
    end
    else
    begin
        if(clken == 1'b1)
        begin
            if(bram_waddr < DATA_DEPTH)
                bram_waddr <= bram_waddr + 1'b1;
            else
                bram_waddr <= 0;
            if(bram_raddr < DATA_DEPTH)
                bram_raddr <= bram_raddr + 1'b1;
            else
                bram_raddr <= 0;
        end
        else
        begin
            bram_waddr <= bram_waddr;
            bram_raddr <= bram_raddr;
        end
    end
end




//----------------------------------------------------------------------
wire            [DATA_WIDTH-1:0]    bram_wdata;
wire                                bram_wenb;
reg            [DATA_WIDTH-1:0]    bram_rdata;

assign bram_wdata = din;
assign bram_wenb  = clken;


localparam ADDR_MSB = 2 ** ADDR_WIDTH - 1; 
reg 	[DATA_WIDTH-1:0] 	r_ram[ADDR_MSB:0]; 

always @(posedge clk) begin
	if(bram_wenb) begin
		r_ram[bram_waddr] <= bram_wdata; 
	end else begin
	end
end
always @(posedge clk) begin
	bram_rdata <= r_ram[bram_raddr]; 
end

//shift_reg_bram
//#(
//    .DATA_WIDTH_A   (DATA_WIDTH ),
//    .ADDR_WIDTH_A   (ADDR_WIDTH ),
//    .DATA_DEPTH_A   (BRAM_DEPTH ),
//    .DATA_WIDTH_B   (DATA_WIDTH ),
//    .ADDR_WIDTH_B   (ADDR_WIDTH ),
//    .DATA_DEPTH_B   (BRAM_DEPTH )
//)
//u_shift_reg_bram
//( 
//    .clka   (clk        ),
//    .addra  (bram_waddr ),
//    .dia    (bram_wdata ),
//    .cea    (bram_wenb  ),
//    
//    .clkb   (clk        ),
//    .addrb  (bram_raddr ),
//    .ceb    (1'b1       ),
//    .dob    (bram_rdata )
//);

assign dout = bram_rdata;

endmodule

====================================================================================================
 [13/29] src/hdmi_ip/rgb2dvi.v    (85 行, 已例化)
====================================================================================================
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    11:23:23 02/27/2022 
// Module Name:    rgb2dvi 
//////////////////////////////////////////////////////////////////////////////////

module rgb2dvi #(
	parameter 	ENABLE_OSERDES 	= 1		
)(
	//	Pixel Clock (1X). 
	input 			PixelClk,
	input 			aRst,
	input 			aRst_n, 
	
	input 	[3:0] 	bitflip_i,		//	LVDS Lane Flip. [3]TXC, [2:0]TXD
	input 			oe_i, 		//	Output Enable. 
	
	//	Video Timing Generator (With Data Valid Control)
	input 			vid_pVSync,		//	VSync. High Valid. 
	input 			vid_pHSync,		//	HSync. High Valid. 
	input 			vid_pVDE, 		//	DataValid (Timing). Must be WIDTH + 2. (de & ~dvalid) will send VLG. 
	input 	[23:0] 	vid_pData,		//	RGB Data. [7:0]B(0), [15:8]G(1), [23:16]R(2). 
	
	//	HDMI Serializer (5X). 
	input 	 		SerialClk, 
	output 			TMDS_Clk_p,
	output 			TMDS_Clk_n,
	output 	[2:0] 	TMDS_Data_p,
	output 	[2:0] 	TMDS_Data_n,
	
	output 	[9:0] 	txc_o, 
	output 	[9:0] 	txd0_o, 
	output 	[9:0] 	txd1_o, 
	output 	[9:0] 	txd2_o
);
	
	wire 			w_rst = aRst || (~aRst_n); 
	
	wire 	[3:0] 	w_ctl = 4'b0000; 
	
	//////////////////////////////////////////////////////////////////////////////////
	//	TMDS Encoder
	//////////////////////////////////////////////////////////////////////////////////
	
	wire 	[9:0] 	w_tmds_0_enc, w_tmds_1_enc, w_tmds_2_enc; 
	
	tmds_channel enc_0 (
		.clk_pixel			(PixelClk),
		.video_data			(vid_pData[ 7: 0]),
		.data_island_data		(0),
		.control_data		({vid_pVSync, vid_pHSync}),
		.mode				({3'b0, vid_pVDE}),  // Mode select (0 = control, 1 = video, 2 = video guard, 3 = island, 4 = island guard)
		.tmds				(w_tmds_0_enc)
	);
	tmds_channel enc_1 (
		.clk_pixel			(PixelClk),
		.video_data			(vid_pData[15: 8]),
		.data_island_data		(0),
		.control_data		(w_ctl[1:0]),
		.mode				({3'b0, vid_pVDE}),  // Mode select (0 = control, 1 = video, 2 = video guard, 3 = island, 4 = island guard)
		.tmds				(w_tmds_1_enc)
	);
	tmds_channel enc_2 (
		.clk_pixel			(PixelClk),
		.video_data			(vid_pData[23:16]),
		.data_island_data		(0),
		.control_data		(w_ctl[3:2]),
		.mode				({3'b0, vid_pVDE}),  // Mode select (0 = control, 1 = video, 2 = video guard, 3 = island, 4 = island guard)
		.tmds				(w_tmds_2_enc)
	);
	
	
	//////////////////////////////////////////////////////////////////////////////////
	//	Serializer
	//////////////////////////////////////////////////////////////////////////////////
	
	assign txc_o = 10'b1111100000;
	assign txd0_o = w_tmds_0_enc;
	assign txd1_o = w_tmds_1_enc;
	assign txd2_o = w_tmds_2_enc; 
	
	
	
endmodule

====================================================================================================
 [14/29] src/hdmi_ip/tmds_channel.v    (170 行, 已例化)
====================================================================================================
// Implementation of HDMI Spec v1.4a Section 5.4: Encoding, Section 5.2.2.1: Video Guard Band, Section 5.2.3.3: Data Island Guard Bands.
// By Sameer Puri https://github.com/sameer

module tmds_channel
#(
    // TMDS Channel number.
    // There are only 3 possible channel numbers in HDMI 1.4a: 0, 1, 2.
    parameter  CN = 0
)
(
    input  clk_pixel,
    input  [7:0] video_data,
    input  [3:0] data_island_data,
    input  [1:0] control_data,
    input  [2:0] mode,  // Mode select (0 = control, 1 = video, 2 = video guard, 3 = island, 4 = island guard)
    output reg [9:0] tmds = 10'b1101010100
);

// See Section 5.4.4.1
// Below is a direct implementation of Figure 5-7, using the same variable names.

reg signed [4:0] acc = 5'sd0;

reg [8:0] q_m;
reg [9:0] q_out;
wire [9:0] video_coding;
assign video_coding = q_out;

reg [3:0] N1D;
reg signed [4:0] N1q_m07;
reg signed [4:0] N0q_m07;
always @(*)
begin
    N1D = video_data[0] + video_data[1] + video_data[2] + video_data[3] + video_data[4] + video_data[5] + video_data[6] + video_data[7];
    case(q_m[0] + q_m[1] + q_m[2] + q_m[3] + q_m[4] + q_m[5] + q_m[6] + q_m[7])
        4'b0000: N1q_m07 = 5'sd0;
        4'b0001: N1q_m07 = 5'sd1;
        4'b0010: N1q_m07 = 5'sd2;
        4'b0011: N1q_m07 = 5'sd3;
        4'b0100: N1q_m07 = 5'sd4;
        4'b0101: N1q_m07 = 5'sd5;
        4'b0110: N1q_m07 = 5'sd6;
        4'b0111: N1q_m07 = 5'sd7;
        4'b1000: N1q_m07 = 5'sd8;
        default: N1q_m07 = 5'sd0;
    endcase
    N0q_m07 = 5'sd8 - N1q_m07;
end

reg signed [4:0] acc_add;

integer i;

always @(*)
begin
    if (N1D > 4'd4 || (N1D == 4'd4 && video_data[0] == 1'd0))
    begin
        q_m[0] = video_data[0];
        for(i = 0; i < 7; i = i + 1)
            q_m[i + 1] = q_m[i] ~^ video_data[i + 1];
        q_m[8] = 1'b0;
    end
    else
    begin
        q_m[0] = video_data[0];
        for(i = 0; i < 7; i = i + 1)
            q_m[i + 1] = q_m[i] ^ video_data[i + 1];
        q_m[8] = 1'b1;
    end
    if (acc == 5'sd0 || (N1q_m07 == N0q_m07))
    begin
        if (q_m[8])
        begin
            acc_add = N1q_m07 - N0q_m07;
            q_out = {~q_m[8], q_m[8], q_m[7:0]};
        end
        else
        begin
            acc_add = N0q_m07 - N1q_m07;
            q_out = {~q_m[8], q_m[8], ~q_m[7:0]};
        end
    end
    else
    begin
        if ((acc > 5'sd0 && N1q_m07 > N0q_m07) || (acc < 5'sd0 && N1q_m07 < N0q_m07))
        begin
            q_out = {1'b1, q_m[8], ~q_m[7:0]};
            acc_add = (N0q_m07 - N1q_m07) + (q_m[8] ? 5'sd2 : 5'sd0);
        end
        else
        begin
            q_out = {1'b0, q_m[8], q_m[7:0]};
            acc_add = (N1q_m07 - N0q_m07) - (~q_m[8] ? 5'sd2 : 5'sd0);
        end
    end
end

always @(posedge clk_pixel) acc <= mode != 3'd1 ? 5'sd0 : acc + acc_add;

// See Section 5.4.2
reg [9:0] control_coding;
always @(*)
begin
     case(control_data)
        2'b00: control_coding = 10'b1101010100;
        2'b01: control_coding = 10'b0010101011;
        2'b10: control_coding = 10'b0101010100;
        2'b11: control_coding = 10'b1010101011;
    endcase
end

// See Section 5.4.3
reg [9:0] terc4_coding;
always @(*)
begin
     case(data_island_data)
        4'b0000 : terc4_coding = 10'b1010011100;
        4'b0001 : terc4_coding = 10'b1001100011;
        4'b0010 : terc4_coding = 10'b1011100100;
        4'b0011 : terc4_coding = 10'b1011100010;
        4'b0100 : terc4_coding = 10'b0101110001;
        4'b0101 : terc4_coding = 10'b0100011110;
        4'b0110 : terc4_coding = 10'b0110001110;
        4'b0111 : terc4_coding = 10'b0100111100;
        4'b1000 : terc4_coding = 10'b1011001100;
        4'b1001 : terc4_coding = 10'b0100111001;
        4'b1010 : terc4_coding = 10'b0110011100;
        4'b1011 : terc4_coding = 10'b1011000110;
        4'b1100 : terc4_coding = 10'b1010001110;
        4'b1101 : terc4_coding = 10'b1001110001;
        4'b1110 : terc4_coding = 10'b0101100011;
        4'b1111 : terc4_coding = 10'b1011000011;
    endcase
end

// See Section 5.2.2.1
wire [9:0] video_guard_band;
generate
    if (CN == 0 || CN == 2)
        assign video_guard_band = 10'b1011001100;
    else
        assign video_guard_band = 10'b0100110011;
endgenerate

// See Section 5.2.3.3
wire [9:0] data_guard_band;
generate
    if (CN == 1 || CN == 2)
        assign data_guard_band = 10'b0100110011;
    else
        assign data_guard_band = control_data == 2'b00 ? 10'b1010001110
            : control_data == 2'b01 ? 10'b1001110001
            : control_data == 2'b10 ? 10'b0101100011
            : 10'b1011000011;
endgenerate

// Apply selected mode.
always @(posedge clk_pixel)
begin
    case (mode)
        3'd0: tmds <= control_coding;
        3'd1: tmds <= video_coding;
        3'd2: tmds <= video_guard_band;
        3'd3: tmds <= terc4_coding;
        3'd4: tmds <= data_guard_band;
    endcase
end

endmodule

====================================================================================================
 [15/29] src/PWMLite.v    (29 行, 已例化)
====================================================================================================
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:	06:46:28 12/22/2020 
// Module Name:	PWMLite 
//////////////////////////////////////////////////////////////////////////////////

//	PWM Lite Controller. 
//	The maximum PWM frequency is 31.25KHz. 

module PWMLite (
	input 				clk_i,
	input 				rst_i,
	input 	[6:0] 	pwm_i,
	input					pol_i, 
	output 				pwm_o
);
	
	
	reg	[6:0]	rc_pwm = 0; 
	reg				r_pwm = 0; 
	always @(posedge clk_i) begin
		rc_pwm <= rc_pwm + 1; 
		r_pwm <= (pwm_i >= rc_pwm); 
	end
	
	assign pwm_o = r_pwm ^ pol_i; 
	
endmodule

====================================================================================================
 [16/29] pri/mipi/MIPIRx4LaneFre.v    (5755 行, 已例化)
====================================================================================================
/* Generated by Yosys 0.29+11 (git sha1 acfdc5cc4, x86_64-w64-mingw32-g++ 9.2.1 -Os) */

module MIPIRx4LaneFre(sclk_i, srst_i, mipi_rx_byteclk_i, mipi_rx_d0_i, mipi_rx_d1_i, mipi_rx_d2_i, mipi_rx_d3_i, mipi_rx_clk_lp_p_i, mipi_rx_clk_lp_n_i, mipi_rx_d0_lp_p_i, mipi_rx_d0_lp_n_i, mipi_rx_rst_o, mipi_rx_rst_n_o, mipi_rx_clk_hs_en_o, mipi_rx_dat_hs_en_o, mipi_rx_clk_term_en_o, mipi_rx_dat_term_en_o, CLK, VSYNC, HSYNC, DE
, DAT, INDEX);
  wire _0000_;
  wire _0001_;
  wire _0002_;
  wire _0003_;
  wire _0004_;
  wire _0005_;
  wire _0006_;
  wire _0007_;
  wire _0008_;
  wire _0009_;
  wire _0010_;
  wire _0011_;
  wire _0012_;
  wire _0013_;
  wire _0014_;
  wire _0015_;
  wire _0016_;
  wire _0017_;
  wire _0018_;
  wire _0019_;
  wire _0020_;
  wire _0021_;
  wire _0022_;
  wire _0023_;
  wire _0024_;
  wire _0025_;
  wire _0026_;
  wire _0027_;
  wire [47:0] _0028_;
  wire _0029_;
  wire [3:0] _0030_;
  wire [15:0] _0031_;
  wire [3:0] _0032_;
  wire _0033_;
  wire _0034_;
  wire [15:0] _0035_;
  wire [15:0] _0036_;
  wire _0037_;
  wire [3:0] _0038_;
  wire _0039_;
  wire _0040_;
  wire [3:0] _0041_;
  wire _0042_;
  wire _0043_;
  wire _0044_;
  wire _0045_;
  wire [7:0] _0046_;
  wire [5:0] _0047_;
  wire _0048_;
  wire _0049_;
  wire _0050_;
  wire _0051_;
  wire _0052_;
  wire _0053_;
  wire [7:0] _0054_;
  wire [5:0] _0055_;
  wire [7:0] _0056_;
  wire [5:0] _0057_;
  wire [7:0] _0058_;
  wire [5:0] _0059_;
  wire [7:0] _0060_;
  wire [5:0] _0061_;
  wire _0062_;
  wire _0063_;
  wire _0064_;
  wire [3:0] _0065_;
  wire [3:0] _0066_;
  wire [3:0] _0067_;
  wire _0068_;
  wire _0069_;
  wire [31:0] _0070_;
  wire _0071_;
  wire [23:0] _0072_;
  wire [23:0] _0073_;
  wire [23:0] _0074_;
  wire [23:0] _0075_;
  wire [23:0] _0076_;
  wire [23:0] _0077_;
  wire [23:0] _0078_;
  wire [23:0] _0079_;
  wire [23:0] _0080_;
  wire [23:0] _0081_;
  wire [23:0] _0082_;
  wire [23:0] _0083_;
  wire [5:0] _0084_;
  wire [5:0] _0085_;
  wire [5:0] _0086_;
  wire [5:0] _0087_;
  wire _0088_;
  wire _0089_;
  wire [3:0] _0090_;
  wire _0091_;
  wire _0092_;
  wire _0093_;
  wire _0094_;
  wire [31:0] _0095_;
  wire _0096_;
  wire [31:0] _0097_;
  wire [31:0] _0098_;
  wire [31:0] _0099_;
  wire _0100_;
  wire [63:0] _0101_;
  wire _0102_;
  wire [31:0] _0103_;
  wire [31:0] _0104_;
  wire [7:0] _0105_;
  wire _0106_;
  wire [255:0] _0107_;
  wire [3:0] _0108_;
  wire _0109_;
  wire [7:0] _0110_;
  wire [7:0] _0111_;
  wire [7:0] _0112_;
  wire [7:0] _0113_;
  wire [31:0] _0114_;
  wire _0115_;
  wire [31:0] _0116_;
  wire [3:0] _0117_;
  wire [3:0] _0118_;
  wire [3:0] _0119_;
  wire [3:0] _0120_;
  wire [3:0] _0121_;
  wire [3:0] _0122_;
  wire [3:0] _0123_;
  wire [3:0] _0124_;
  wire [3:0] _0125_;
  wire [3:0] _0126_;
  wire _0127_;
  wire _0128_;
  wire [7:0] _0129_;
  wire [5:0] _0130_;
  wire _0131_;
  wire _0132_;
  wire _0133_;
  wire _0134_;
  wire _0135_;
  wire _0136_;
  wire [7:0] _0137_;
  wire [5:0] _0138_;
  wire [7:0] _0139_;
  wire [5:0] _0140_;
  wire [7:0] _0141_;
  wire [5:0] _0142_;
  wire [7:0] _0143_;
  wire [5:0] _0144_;
  wire _0145_;
  wire _0146_;
  wire _0147_;
  wire [3:0] _0148_;
  wire [3:0] _0149_;
  wire _0150_;
  wire _0151_;
  wire [31:0] _0152_;
  wire _0153_;
  wire [23:0] _0154_;
  wire [23:0] _0155_;
  wire [23:0] _0156_;
  wire [23:0] _0157_;
  wire [23:0] _0158_;
  wire [23:0] _0159_;
  wire [23:0] _0160_;
  wire [23:0] _0161_;
  wire [5:0] _0162_;
  wire [5:0] _0163_;
  wire [5:0] _0164_;
  wire [5:0] _0165_;
  wire _0166_;
  wire _0167_;
  wire [3:0] _0168_;
  wire _0169_;
  wire _0170_;
  wire _0171_;
  wire _0172_;
  wire [31:0] _0173_;
  wire _0174_;
  wire [31:0] _0175_;
  wire [31:0] _0176_;
  wire [31:0] _0177_;
  wire _0178_;
  wire [15:0] _0179_;
  wire [63:0] _0180_;
  wire _0181_;
  wire [31:0] _0182_;
  wire [31:0] _0183_;
  wire [15:0] _0184_;
  wire _0185_;
  wire [255:0] _0186_;
  wire [3:0] _0187_;
  wire _0188_;
  wire [7:0] _0189_;
  wire [15:0] _0190_;
  wire [15:0] _0191_;
  wire [15:0] _0192_;
  wire [15:0] _0193_;
  wire [31:0] _0194_;
  wire _0195_;
  wire [31:0] _0196_;
  wire [3:0] _0197_;
  wire [3:0] _0198_;
  wire [3:0] _0199_;
  wire [3:0] _0200_;
  wire [3:0] _0201_;
  wire [3:0] _0202_;
  wire [3:0] _0203_;
  wire [3:0] _0204_;
  wire [3:0] _0205_;
  wire [3:0] _0206_;
  wire _0207_;
  wire _0208_;
  wire _0209_;
  wire _0210_;
  wire _0211_;
  wire [23:0] _0212_;
  wire [23:0] _0213_;
  wire [7:0] _0214_;
  wire [5:0] _0215_;
  wire _0216_;
  wire _0217_;
  wire _0218_;
  wire _0219_;
  wire _0220_;
  wire _0221_;
  wire _0222_;
  wire _0223_;
  wire _0224_;
  wire [9:0] _0225_;
  wire _0226_;
  wire _0227_;
  reg [15:0] _0228_;
  wire [3:0] _0229_;
  wire _0230_;
  wire _0231_;
  wire _0232_;
  wire [3:0] _0233_;
  wire _0234_;
  wire [7:0] _0235_;
  wire [7:0] _0236_;
  wire [7:0] _0237_;
  wire [7:0] _0238_;
  wire [7:0] _0239_;
  wire [7:0] _0240_;
  wire [7:0] _0241_;
  wire [7:0] _0242_;
  wire [7:0] _0243_;
  wire [7:0] _0244_;
  wire [7:0] _0245_;
  wire [7:0] _0246_;
  wire [7:0] _0247_;
  wire [7:0] _0248_;
  wire [7:0] _0249_;
  wire [7:0] _0250_;
  wire [7:0] _0251_;
  wire [7:0] _0252_;
  wire [7:0] _0253_;
  wire [7:0] _0254_;
  wire [7:0] _0255_;
  wire [7:0] _0256_;
  wire [7:0] _0257_;
  wire [7:0] _0258_;
  wire [7:0] _0259_;
  wire [7:0] _0260_;
  wire [7:0] _0261_;
  wire [7:0] _0262_;
  wire [7:0] _0263_;
  wire [7:0] _0264_;
  wire [7:0] _0265_;
  wire [7:0] _0266_;
  wire [23:0] _0267_;
  wire [23:0] _0268_;
  wire [7:0] _0269_;
  wire [5:0] _0270_;
  wire [7:0] _0271_;
  wire [7:0] _0272_;
  wire [7:0] _0273_;
  wire [7:0] _0274_;
  wire [7:0] _0275_;
  wire [7:0] _0276_;
  wire [7:0] _0277_;
  wire [7:0] _0278_;
  wire [23:0] _0279_;
  wire [23:0] _0280_;
  wire [7:0] _0281_;
  wire [5:0] _0282_;
  wire [7:0] _0283_;
  wire [7:0] _0284_;
  wire [7:0] _0285_;
  wire [7:0] _0286_;
  wire [7:0] _0287_;
  wire [7:0] _0288_;
  wire [7:0] _0289_;
  wire [7:0] _0290_;
  wire [23:0] _0291_;
  wire [23:0] _0292_;
  wire [7:0] _0293_;
  wire [5:0] _0294_;
  wire [7:0] _0295_;
  wire [7:0] _0296_;
  wire [7:0] _0297_;
  wire [7:0] _0298_;
  wire [7:0] _0299_;
  wire [7:0] _0300_;
  wire [7:0] _0301_;
  wire [7:0] _0302_;
  wire [23:0] _0303_;
  wire [23:0] _0304_;
  wire [7:0] _0305_;
  wire [5:0] _0306_;
  wire [7:0] _0307_;
  wire [7:0] _0308_;
  wire [7:0] _0309_;
  wire [7:0] _0310_;
  wire [7:0] _0311_;
  wire [7:0] _0312_;
  wire [7:0] _0313_;
  wire [7:0] _0314_;
  wire [1:0] _0315_;
  wire _0316_;
  wire _0317_;
  wire _0318_;
  wire [31:0] _0319_;
  wire _0320_;
  reg [3:0] _0321_ = 4'h0;
  reg [3:0] _0322_ = 4'h0;
  reg [3:0] _0323_ = 4'h0;
  reg [3:0] _0324_ = 4'h0;
  reg [1:0] _0325_ = 2'h0;
  reg [3:0] _0326_ = 4'h0;
  reg [3:0] _0327_ = 4'h0;
  reg [31:0] _0328_ = 32'd0;
  reg _0329_ = 1'h0;
  wire _0330_;
  wire [79:0] _0331_;
  wire [63:0] _0332_;
  wire [63:0] _0333_;
  wire [63:0] _0334_;
  reg _0335_ = 1'h0;
  wire [31:0] _0336_;
  wire _0337_;
  wire _0338_;
  wire [7:0] _0339_;
  wire [7:0] _0340_;
  wire [7:0] _0341_;
  wire [7:0] _0342_;
  wire [7:0] _0343_;
  wire [7:0] _0344_;
  wire [7:0] _0345_;
  wire [7:0] _0346_;
  wire [23:0] _0347_;
  wire [23:0] _0348_;
  wire [23:0] _0349_;
  wire [23:0] _0350_;
  wire [23:0] _0351_;
  wire [23:0] _0352_;
  wire [23:0] _0353_;
  wire [23:0] _0354_;
  wire [23:0] _0355_;
  wire [23:0] _0356_;
  wire [23:0] _0357_;
  wire [23:0] _0358_;
  wire [23:0] _0359_;
  wire [23:0] _0360_;
  wire [23:0] _0361_;
  wire [23:0] _0362_;
  wire [31:0] _0363_;
  wire [31:0] _0364_;
  wire [31:0] _0365_;
  wire [31:0] _0366_;
  wire [5:0] _0367_;
  wire [5:0] _0368_;
  wire [5:0] _0369_;
  wire [5:0] _0370_;
  wire [5:0] _0371_;
  wire [5:0] _0372_;
  wire [5:0] _0373_;
  wire [5:0] _0374_;
  wire [15:0] _0375_;
  wire [15:0] _0376_;
  wire [15:0] _0377_;
  wire [15:0] _0378_;
  wire [15:0] _0379_;
  wire [15:0] _0380_;
  wire [15:0] _0381_;
  wire _0382_;
  wire _0383_;
  wire _0384_;
  wire [3:0] _0385_;
  wire _0386_;
  wire _0387_;
  wire _0388_;
  wire [31:0] _0389_;
  wire _0390_;
  wire _0391_;
  wire _0392_;
  wire [31:0] _0393_;
  wire _0394_;
  wire [31:0] _0395_;
  reg _0396_ = 1'h0;
  reg [3:0] _0397_ = 4'h0;
  reg [5:0] _0398_ = 6'h00;
  wire [31:0] _0399_;
  wire [31:0] _0400_;
  wire _0401_;
  reg _0402_ = 1'h0;
  reg _0403_ = 1'h0;
  wire [7:0] _0404_;
  wire [15:0] _0405_;
  wire [15:0] _0406_;
  reg [1:0] _0407_ = 2'h0;
  wire [63:0] _0408_;
  reg _0409_ = 1'h0;
  reg [1:0] _0410_ = 2'h0;
  reg [31:0] _0411_ = 32'd0;
  reg _0412_ = 1'h0;
  wire [31:0] _0413_;
  wire [31:0] _0414_;
  wire [7:0] _0415_;
  reg [15:0] _0416_ = 16'h0000;
  reg [3:0] _0417_ = 4'h0;
  wire [31:0] _0418_;
  wire _0419_;
  wire [31:0] _0420_;
  wire [255:0] _0421_;
  wire _0422_;
  wire _0423_;
  wire _0424_;
  wire [3:0] _0425_;
  wire [1:0] _0426_;
  wire [31:0] _0427_;
  wire [31:0] _0428_;
  wire _0429_;
  wire _0430_;
  wire [23:0] _0431_;
  wire [7:0] _0432_;
  wire [7:0] _0433_;
  wire [15:0] _0434_;
  wire _0435_;
  wire [31:0] _0436_;
  wire [31:0] _0437_;
  wire [31:0] _0438_;
  wire [31:0] _0439_;
  wire _0440_;
  wire [31:0] _0441_;
  wire [3:0] _0442_;
  wire [3:0] _0443_;
  wire [3:0] _0444_;
  wire [3:0] _0445_;
  wire [3:0] _0446_;
  wire [3:0] _0447_;
  wire [3:0] _0448_;
  wire [3:0] _0449_;
  wire [3:0] _0450_;
  wire [3:0] _0451_;
  wire _0452_;
  wire [1:0] _0453_;
  wire _0454_;
  wire [7:0] _0455_;
  wire _0456_;
  wire _0457_;
  wire [3:0] _0458_;
  wire [3:0] _0459_;
  wire [3:0] _0460_;
  wire [3:0] _0461_;
  wire [31:0] _0462_;
  wire [47:0] _0463_;
  wire [3:0] _0464_;
  wire [3:0] _0465_;
  reg _0466_ = 1'h0;
  reg [3:0] _0467_ = 4'h0;
  reg [3:0] _0468_ = 4'h0;
  wire [2:0] _0469_;
  reg [47:0] _0470_ = 48'h000000000000;
  wire [47:0] _0471_;
  wire [3:0] _0472_;
  reg _0473_ = 1'h0;
  reg [1:0] _0474_ = 2'h0;
  wire [1:0] _0475_;
  reg [2:0] _0476_ = 3'h0;
  wire _0477_;
  wire _0478_;
  wire _0479_;
  wire _0480_;
  wire _0481_;
  wire _0482_;
  wire [7:0] _0483_;
  wire _0484_;
  wire _0485_;
  wire [7:0] _0486_;
  wire [7:0] _0487_;
  wire [7:0] _0488_;
  wire _0489_;
  wire _0490_;
  wire _0491_;
  wire _0492_;
  wire _0493_;
  wire _0494_;
  wire _0495_;
  wire [16:0] _0496_;
  wire [16:0] _0497_;
  reg [15:0] _0498_ = 16'h0000;
  reg [3:0] _0499_ = 4'h0;
  reg [3:0] _0500_ = 4'h0;
  reg [31:0] _0501_ = 32'd0;
  reg _0502_ = 1'h0;
  wire [7:0] _0503_;
  reg _0504_ = 1'h1;
  reg [31:0] _0505_ = 32'd0;
  reg [31:0] _0506_ = 32'd0;
  reg _0507_ = 1'h0;
  reg _0508_ = 1'h0;
  wire _0509_;
  wire _0510_;
  wire [63:0] _0511_;
  wire [3:0] _0512_;
  wire [3:0] _0513_;
  wire [47:0] _0514_;
  wire [3:0] _0515_;
  wire _0516_;
  wire _0517_;
  wire [15:0] _0518_;
  wire [3:0] _0519_;
  wire [3:0] _0520_;
  wire [31:0] _0521_;
  wire _0522_;
  wire _0523_;
  wire _0524_;
  wire [7:0] _0525_;
  wire _0526_;
  wire [7:0] _0527_;
  wire [7:0] _0528_;
  wire [7:0] _0529_;
  wire [7:0] _0530_;
  wire [7:0] _0531_;
  wire [7:0] _0532_;
  wire [7:0] _0533_;
  wire [7:0] _0534_;
  wire _0535_;
  wire _0536_;
  wire _0537_;
  wire _0538_;
  wire _0539_;
  wire _0540_;
  wire _0541_;
  wire _0542_;
  wire _0543_;
  wire _0544_;
  wire _0545_;
  wire _0546_;
  wire _0547_;
  wire _0548_;
  wire _0549_;
  wire _0550_;
  wire _0551_;
  wire _0552_;
  wire _0553_;
  wire _0554_;
  wire _0555_;
  wire _0556_;
  wire _0557_;
  wire _0558_;
  wire _0559_;
  wire _0560_;
  wire _0561_;
  wire _0562_;
  wire _0563_;
  wire _0564_;
  wire _0565_;
  wire _0566_;
  wire _0567_;
  wire _0568_;
  wire _0569_;
  wire _0570_;
  wire _0571_;
  wire _0572_;
  wire _0573_;
  wire _0574_;
  wire _0575_;
  wire _0576_;
  wire _0577_;
  wire _0578_;
  wire _0579_;
  wire _0580_;
  wire _0581_;
  wire _0582_;
  wire _0583_;
  wire _0584_;
  wire _0585_;
  wire _0586_;
  wire _0587_;
  wire _0588_;
  wire _0589_;
  wire _0590_;
  wire _0591_;
  wire _0592_;
  wire _0593_;
  wire _0594_;
  wire _0595_;
  wire _0596_;
  wire _0597_;
  wire _0598_;
  wire _0599_;
  wire _0600_;
  wire _0601_;
  wire _0602_;
  wire _0603_;
  wire _0604_;
  wire _0605_;
  wire _0606_;
  wire _0607_;
  wire _0608_;
  wire _0609_;
  wire _0610_;
  wire _0611_;
  wire _0612_;
  wire _0613_;
  wire _0614_;
  wire _0615_;
  wire _0616_;
  wire _0617_;
  wire _0618_;
  wire _0619_;
  wire _0620_;
  wire _0621_;
  wire _0622_;
  wire _0623_;
  wire [1:0] _0624_;
  wire _0625_;
  wire _0626_;
  wire _0627_;
  wire _0628_;
  wire [3:0] _0629_;
  wire [1:0] _0630_;
  wire [1:0] _0631_;
  wire _0632_;
  wire [1:0] _0633_;
  wire [1:0] _0634_;
  wire [1:0] _0635_;
  wire [1:0] _0636_;
  wire [1:0] _0637_;
  wire [1:0] _0638_;
  wire [2:0] _0639_;
  wire _0640_;
  wire _0641_;
  wire [3:0] _0642_;
  wire [1:0] _0643_;
  wire [3:0] _0644_;
  wire [1:0] _0645_;
  wire [1:0] _0646_;
  wire [1:0] _0647_;
  wire [1:0] _0648_;
  wire _0649_;
  wire _0650_;
  wire [1:0] _0651_;
  wire [1:0] _0652_;
  wire [15:0] _0653_;
  wire [7:0] _0654_;
  wire [3:0] _0655_;
  wire [1:0] _0656_;
  wire [2:0] _0657_;
  wire _0658_;
  wire [2:0] _0659_;
  wire _0660_;
  wire [2:0] _0661_;
  wire _0662_;
  wire [2:0] _0663_;
  wire _0664_;
  wire [2:0] _0665_;
  wire _0666_;
  wire [2:0] _0667_;
  wire _0668_;
  wire [2:0] _0669_;
  wire _0670_;
  wire [2:0] _0671_;
  wire _0672_;
  wire [2:0] _0673_;
  wire _0674_;
  wire [2:0] _0675_;
  wire _0676_;
  wire _0677_;
  wire _0678_;
  wire [11:0] _0679_;
  wire [5:0] _0680_;
  wire [2:0] _0681_;
  wire _0682_;
  wire [11:0] _0683_;
  wire [5:0] _0684_;
  wire [2:0] _0685_;
  wire _0686_;
  wire [11:0] _0687_;
  wire [5:0] _0688_;
  wire [2:0] _0689_;
  wire _0690_;
  wire [11:0] _0691_;
  wire [5:0] _0692_;
  wire [2:0] _0693_;
  wire _0694_;
  wire [2:0] _0695_;
  wire _0696_;
  wire [11:0] _0697_;
  wire [5:0] _0698_;
  wire [2:0] _0699_;
  wire _0700_;
  wire [11:0] _0701_;
  wire [5:0] _0702_;
  wire [2:0] _0703_;
  wire _0704_;
  wire [11:0] _0705_;
  wire [5:0] _0706_;
  wire [2:0] _0707_;
  wire _0708_;
  wire [11:0] _0709_;
  wire [5:0] _0710_;
  wire [2:0] _0711_;
  wire _0712_;
  wire _0713_;
  wire [11:0] _0714_;
  wire [5:0] _0715_;
  wire [2:0] _0716_;
  wire _0717_;
  wire [11:0] _0718_;
  wire [5:0] _0719_;
  wire [2:0] _0720_;
  wire _0721_;
  wire [11:0] _0722_;
  wire [5:0] _0723_;
  wire [2:0] _0724_;
  wire _0725_;
  wire [11:0] _0726_;
  wire [5:0] _0727_;
  wire [2:0] _0728_;
  wire _0729_;
  wire [2:0] _0730_;
  wire _0731_;
  wire [11:0] _0732_;
  wire [5:0] _0733_;
  wire [2:0] _0734_;
  wire _0735_;
  wire [11:0] _0736_;
  wire [5:0] _0737_;
  wire [2:0] _0738_;
  wire _0739_;
  wire [11:0] _0740_;
  wire [5:0] _0741_;
  wire [2:0] _0742_;
  wire _0743_;
  wire [11:0] _0744_;
  wire [5:0] _0745_;
  wire [2:0] _0746_;
  wire _0747_;
  wire _0748_;
  wire [2:0] _0749_;
  wire _0750_;
  wire [1:0] _0751_;
  wire [1:0] _0752_;
  wire [1:0] _0753_;
  wire [1:0] _0754_;
  wire [1:0] _0755_;
  wire [1:0] _0756_;
  wire [1:0] _0757_;
  wire [1:0] _0758_;
  wire [1:0] _0759_;
  wire [1:0] _0760_;
  wire [1:0] _0761_;
  wire [1:0] _0762_;
  wire [1:0] _0763_;
  wire [1:0] _0764_;
  wire [1:0] _0765_;
  wire [1:0] _0766_;
  wire [1:0] _0767_;
  wire [1:0] _0768_;
  wire [1:0] _0769_;
  wire [1:0] _0770_;
  wire [1:0] _0771_;
  wire [1:0] _0772_;
  wire [1:0] _0773_;
  wire [1:0] _0774_;
  wire [1:0] _0775_;
  wire [1:0] _0776_;
  wire [1:0] _0777_;
  wire [1:0] _0778_;
  wire [1:0] _0779_;
  wire [1:0] _0780_;
  wire [1:0] _0781_;
  wire [1:0] _0782_;
  wire [1:0] _0783_;
  wire [1:0] _0784_;
  wire [1:0] _0785_;
  wire [1:0] _0786_;
  wire [1:0] _0787_;
  wire [1:0] _0788_;
  wire [1:0] _0789_;
  wire [1:0] _0790_;
  wire [1:0] _0791_;
  wire [1:0] _0792_;
  wire [1:0] _0793_;
  wire [1:0] _0794_;
  wire [1:0] _0795_;
  wire [1:0] _0796_;
  wire [1:0] _0797_;
  wire [1:0] _0798_;
  wire [1:0] _0799_;
  wire [1:0] _0800_;
  wire [1:0] _0801_;
  wire [1:0] _0802_;
  wire [1:0] _0803_;
  wire [1:0] _0804_;
  wire [1:0] _0805_;
  wire [1:0] _0806_;
  wire [1:0] _0807_;
  wire [1:0] _0808_;
  wire [1:0] _0809_;
  wire [1:0] _0810_;
  wire [1:0] _0811_;
  wire [1:0] _0812_;
  wire [1:0] _0813_;
  wire [1:0] _0814_;
  wire [1:0] _0815_;
  wire [7:0] _0816_;
  wire [3:0] _0817_;
  wire [1:0] _0818_;
  wire [1:0] _0819_;
  wire [1:0] _0820_;
  wire [1:0] _0821_;
  wire [1:0] _0822_;
  wire [1:0] _0823_;
  wire [1:0] _0824_;
  wire [1:0] _0825_;
  wire [1:0] _0826_;
  wire [1:0] _0827_;
  wire _0828_;
  wire _0829_;
  wire [15:0] _0830_;
  wire [7:0] _0831_;
  wire [3:0] _0832_;
  wire [1:0] _0833_;
  wire _0834_;
  wire _0835_;
  wire _0836_;
  wire _0837_;
  wire _0838_;
  wire [1:0] _0839_;
  wire _0840_;
  wire _0841_;
  wire [1:0] _0842_;
  wire _0843_;
  wire _0844_;
  wire [1:0] _0845_;
  wire _0846_;
  wire _0847_;
  wire [1:0] _0848_;
  wire _0849_;
  wire _0850_;
  wire _0851_;
  wire _0852_;
  wire [7:0] _0853_;
  wire [5:0] _0854_;
  wire [5:0] _0855_;
  wire [5:0] _0856_;
  wire [5:0] _0857_;
  wire [5:0] _0858_;
  wire _0859_;
  wire _0860_;
  wire _0861_;
  wire _0862_;
  wire _0863_;
  wire _0864_;
  wire _0865_;
  wire _0866_;
  wire _0867_;
  wire _0868_;
  wire _0869_;
  wire _0870_;
  wire _0871_;
  wire _0872_;
  wire _0873_;
  wire _0874_;
  wire _0875_;
  wire _0876_;
  wire _0877_;
  wire _0878_;
  wire _0879_;
  wire _0880_;
  wire _0881_;
  wire _0882_;
  wire _0883_;
  wire _0884_;
  wire _0885_;
  wire _0886_;
  wire _0887_;
  wire _0888_;
  wire _0889_;
  wire _0890_;
  wire _0891_;
  wire _0892_;
  wire _0893_;
  wire _0894_;
  wire _0895_;
  wire _0896_;
  wire _0897_;
  wire _0898_;
  wire _0899_;
  wire _0900_;
  wire _0901_;
  wire _0902_;
  wire _0903_;
  wire _0904_;
  wire _0905_;
  wire _0906_;
  wire _0907_;
  wire _0908_;
  wire _0909_;
  wire _0910_;
  wire _0911_;
  wire _0912_;
  wire _0913_;
  wire _0914_;
  wire _0915_;
  wire _0916_;
  wire _0917_;
  wire _0918_;
  wire _0919_;
  wire _0920_;
  wire _0921_;
  wire _0922_;
  wire _0923_;
  wire _0924_;
  wire _0925_;
  wire _0926_;
  wire _0927_;
  wire _0928_;
  wire _0929_;
  wire _0930_;
  wire _0931_;
  wire _0932_;
  wire _0933_;
  wire _0934_;
  wire _0935_;
  wire _0936_;
  wire _0937_;
  wire _0938_;
  wire _0939_;
  wire _0940_;
  wire _0941_;
  wire _0942_;
  wire _0943_;
  wire _0944_;
  wire _0945_;
  wire _0946_;
  wire _0947_;
  wire _0948_;
  wire _0949_;
  wire _0950_;
  wire _0951_;
  wire _0952_;
  wire _0953_;
  wire _0954_;
  wire _0955_;
  wire _0956_;
  wire _0957_;
  wire _0958_;
  wire _0959_;
  wire _0960_;
  wire _0961_;
  wire _0962_;
  wire _0963_;
  wire _0964_;
  wire _0965_;
  wire _0966_;
  wire _0967_;
  wire _0968_;
  wire _0969_;
  wire _0970_;
  wire _0971_;
  wire _0972_;
  wire _0973_;
  wire _0974_;
  wire _0975_;
  wire _0976_;
  wire _0977_;
  wire _0978_;
  wire _0979_;
  wire _0980_;
  wire _0981_;
  wire _0982_;
  wire _0983_;
  wire _0984_;
  wire _0985_;
  wire _0986_;
  wire _0987_;
  wire _0988_;
  wire _0989_;
  wire _0990_;
  wire _0991_;
  wire _0992_;
  wire _0993_;
  wire _0994_;
  wire _0995_;
  wire _0996_;
  wire _0997_;
  wire _0998_;
  wire _0999_;
  wire _1000_;
  wire _1001_;
  wire _1002_;
  wire _1003_;
  wire _1004_;
  wire _1005_;
  wire _1006_;
  wire _1007_;
  wire _1008_;
  wire _1009_;
  wire _1010_;
  wire _1011_;
  wire _1012_;
  wire _1013_;
  wire _1014_;
  wire _1015_;
  wire _1016_;
  wire _1017_;
  wire _1018_;
  wire _1019_;
  wire _1020_;
  wire _1021_;
  wire _1022_;
  wire _1023_;
  wire _1024_;
  wire _1025_;
  wire _1026_;
  wire _1027_;
  wire _1028_;
  wire _1029_;
  wire _1030_;
  wire _1031_;
  wire _1032_;
  wire _1033_;
  wire _1034_;
  wire _1035_;
  wire _1036_;
  wire _1037_;
  wire _1038_;
  wire _1039_;
  wire _1040_;
  wire _1041_;
  wire _1042_;
  wire _1043_;
  wire _1044_;
  wire _1045_;
  wire _1046_;
  wire _1047_;
  wire _1048_;
  wire _1049_;
  wire _1050_;
  wire _1051_;
  wire _1052_;
  wire _1053_;
  wire _1054_;
  wire _1055_;
  wire _1056_;
  wire _1057_;
  wire _1058_;
  wire _1059_;
  wire _1060_;
  wire _1061_;
  wire _1062_;
  wire _1063_;
  wire _1064_;
  wire _1065_;
  wire _1066_;
  wire _1067_;
  wire _1068_;
  wire _1069_;
  wire _1070_;
  wire _1071_;
  wire _1072_;
  wire _1073_;
  wire _1074_;
  wire _1075_;
  wire _1076_;
  wire _1077_;
  wire _1078_;
  wire _1079_;
  wire _1080_;
  wire _1081_;
  wire _1082_;
  wire _1083_;
  wire _1084_;
  wire _1085_;
  wire _1086_;
  wire _1087_;
  wire _1088_;
  wire _1089_;
  wire _1090_;
  wire _1091_;
  wire _1092_;
  wire _1093_;
  wire _1094_;
  wire _1095_;
  wire _1096_;
  wire _1097_;
  wire _1098_;
  wire _1099_;
  wire _1100_;
  wire _1101_;
  wire _1102_;
  wire _1103_;
  wire _1104_;
  wire _1105_;
  wire _1106_;
  wire _1107_;
  wire _1108_;
  wire _1109_;
  wire _1110_;
  wire _1111_;
  wire _1112_;
  wire _1113_;
  wire _1114_;
  wire _1115_;
  wire _1116_;
  wire _1117_;
  wire _1118_;
  wire _1119_;
  wire _1120_;
  wire _1121_;
  wire _1122_;
  wire _1123_;
  wire _1124_;
  wire _1125_;
  wire _1126_;
  wire _1127_;
  wire _1128_;
  wire _1129_;
  wire _1130_;
  wire _1131_;
  wire _1132_;
  wire _1133_;
  wire _1134_;
  wire _1135_;
  wire _1136_;
  wire _1137_;
  wire _1138_;
  wire _1139_;
  wire _1140_;
  wire _1141_;
  wire _1142_;
  wire _1143_;
  wire _1144_;
  wire _1145_;
  wire _1146_;
  wire _1147_;
  wire _1148_;
  wire _1149_;
  wire _1150_;
  wire _1151_;
  wire _1152_;
  wire _1153_;
  wire _1154_;
  wire _1155_;
  wire _1156_;
  wire _1157_;
  wire _1158_;
  wire _1159_;
  wire _1160_;
  wire _1161_;
  wire _1162_;
  wire _1163_;
  wire _1164_;
  wire _1165_;
  wire _1166_;
  wire _1167_;
  wire _1168_;
  wire _1169_;
  wire _1170_;
  wire _1171_;
  wire _1172_;
  wire _1173_;
  wire _1174_;
  wire _1175_;
  wire _1176_;
  wire _1177_;
  wire _1178_;
  wire _1179_;
  wire _1180_;
  wire _1181_;
  wire _1182_;
  wire _1183_;
  wire _1184_;
  wire _1185_;
  wire _1186_;
  wire _1187_;
  wire _1188_;
  wire _1189_;
  wire _1190_;
  wire _1191_;
  wire _1192_;
  wire _1193_;
  wire _1194_;
  wire _1195_;
  wire _1196_;
  wire _1197_;
  wire _1198_;
  wire _1199_;
  wire _1200_;
  wire _1201_;
  wire _1202_;
  wire _1203_;
  wire _1204_;
  wire _1205_;
  wire _1206_;
  wire _1207_;
  wire _1208_;
  wire _1209_;
  wire _1210_;
  wire _1211_;
  wire _1212_;
  wire _1213_;
  wire _1214_;
  wire _1215_;
  wire _1216_;
  wire _1217_;
  wire _1218_;
  wire _1219_;
  wire _1220_;
  wire _1221_;
  wire _1222_;
  wire _1223_;
  wire _1224_;
  wire _1225_;
  wire _1226_;
  wire _1227_;
  wire _1228_;
  wire _1229_;
  wire _1230_;
  wire _1231_;
  wire _1232_;
  wire _1233_;
  wire _1234_;
  wire _1235_;
  wire _1236_;
  wire _1237_;
  wire _1238_;
  wire _1239_;
  wire _1240_;
  wire _1241_;
  wire _1242_;
  wire _1243_;
  wire _1244_;
  wire _1245_;
  wire _1246_;
  wire _1247_;
  wire _1248_;
  wire _1249_;
  wire _1250_;
  wire _1251_;
  wire _1252_;
  wire _1253_;
  wire _1254_;
  wire _1255_;
  wire _1256_;
  wire _1257_;
  wire _1258_;
  wire _1259_;
  wire _1260_;
  wire _1261_;
  wire _1262_;
  wire _1263_;
  wire _1264_;
  wire _1265_;
  wire _1266_;
  wire _1267_;
  wire _1268_;
  wire _1269_;
  wire _1270_;
  wire _1271_;
  wire _1272_;
  wire _1273_;
  wire _1274_;
  wire _1275_;
  wire _1276_;
  wire _1277_;
  wire _1278_;
  wire _1279_;
  wire _1280_;
  wire _1281_;
  wire _1282_;
  wire _1283_;
  wire _1284_;
  wire _1285_;
  wire _1286_;
  wire _1287_;
  wire _1288_;
  wire _1289_;
  wire _1290_;
  wire _1291_;
  wire _1292_;
  wire _1293_;
  wire _1294_;
  wire _1295_;
  wire _1296_;
  wire _1297_;
  wire _1298_;
  wire _1299_;
  wire _1300_;
  wire _1301_;
  wire _1302_;
  wire _1303_;
  wire _1304_;
  wire _1305_;
  wire _1306_;
  wire _1307_;
  wire _1308_;
  wire _1309_;
  wire _1310_;
  wire _1311_;
  wire _1312_;
  wire _1313_;
  wire _1314_;
  wire _1315_;
  wire _1316_;
  wire _1317_;
  wire _1318_;
  wire _1319_;
  wire _1320_;
  wire _1321_;
  wire _1322_;
  wire _1323_;
  wire _1324_;
  wire [47:0] _1325_;
  wire _1326_;
  wire _1327_;
  wire _1328_;
  wire _1329_;
  wire [1:0] _1330_;
  wire [1:0] _1331_;
  wire [1:0] _1332_;
  wire [127:0] _1333_;
  wire [3:0] _1334_;
  wire [31:0] _1335_;
  wire [31:0] _1336_;
  wire _1337_;
  wire _1338_;
  wire [7:0] _1339_;
  wire [7:0] _1340_;
  wire [7:0] _1341_;
  wire [7:0] _1342_;
  wire [7:0] _1343_;
  wire _1344_;
  wire _1345_;
  wire _1346_;
  wire _1347_;
  wire _1348_;
  wire _1349_;
  wire _1350_;
  wire [3:0] _1351_;
  wire _1352_;
  wire [3:0] _1353_;
  wire [3:0] _1354_;
  wire [3:0] _1355_;
  wire [3:0] _1356_;
  wire [3:0] _1357_;
  wire [3:0] _1358_;
  wire [3:0] _1359_;
  wire [3:0] _1360_;
  wire [3:0] _1361_;
  wire [3:0] _1362_;
  wire _1363_;
  wire [31:0] _1364_;
  wire [31:0] _1365_;
  wire [3:0] _1366_;
  wire [7:0] _1367_;
  wire [7:0] _1368_;
  wire [15:0] _1369_;
  wire [15:0] _1370_;
  wire [3:0] _1371_;
  wire [3:0] _1372_;
  wire [31:0] _1373_;
  wire [31:0] _1374_;
  wire [7:0] _1375_;
  wire [7:0] _1376_;
  wire [15:0] _1377_;
  wire [15:0] _1378_;
  wire [3:0] _1379_;
  wire [3:0] _1380_;
  wire [31:0] _1381_;
  wire [7:0] _1382_;
  wire [7:0] _1383_;
  wire [15:0] _1384_;
  wire [15:0] _1385_;
  wire [3:0] _1386_;
  wire [3:0] _1387_;
  wire [31:0] _1388_;
  wire [31:0] _1389_;
  wire [7:0] _1390_;
  wire [7:0] _1391_;
  wire [15:0] _1392_;
  wire [15:0] _1393_;
  wire [3:0] _1394_;
  wire [3:0] _1395_;
  wire [2:0] _1396_;
  wire [2:0] _1397_;
  wire [3:0] _1398_;
  wire [3:0] _1399_;
  wire [31:0] _1400_;
  wire [31:0] _1401_;
  wire [31:0] _1402_;
  wire [15:0] _1403_;
  wire [15:0] _1404_;
  wire [15:0] _1405_;
  wire [15:0] _1406_;
  wire _1407_;
  wire _1408_;
  wire _1409_;
  wire _1410_;
  wire [31:0] _1411_;
  wire [31:0] _1412_;
  wire [31:0] _1413_;
  wire [2:0] _1414_;
  wire [2:0] _1415_;
  wire [2:0] _1416_;
  wire [2:0] _1417_;
  wire _1418_;
  wire _1419_;
  wire _1420_;
  wire _1421_;
  wire _1422_;
  wire _1423_;
  wire _1424_;
  wire _1425_;
  wire _1426_;
  wire _1427_;
  wire _1428_;
  wire _1429_;
  wire _1430_;
  wire _1431_;
  wire _1432_;
  wire _1433_;
  wire _1434_;
  wire _1435_;
  wire _1436_;
  wire _1437_;
  wire _1438_;
  wire _1439_;
  wire _1440_;
  wire _1441_;
  wire _1442_;
  wire _1443_;
  wire _1444_;
  wire _1445_;
  wire _1446_;
  wire _1447_;
  wire _1448_;
  wire _1449_;
  wire _1450_;
  wire _1451_;
  wire _1452_;
  wire _1453_;
  wire _1454_;
  output CLK;
  wire CLK;
  output [47:0] DAT;
  wire [47:0] DAT;
  output DE;
  wire DE;
  output [3:0] HSYNC;
  wire [3:0] HSYNC;
  output [15:0] INDEX;
  wire [15:0] INDEX;
  output [3:0] VSYNC;
  wire [3:0] VSYNC;
  input mipi_rx_byteclk_i;
  wire mipi_rx_byteclk_i;
  output mipi_rx_clk_hs_en_o;
  wire mipi_rx_clk_hs_en_o;
  input mipi_rx_clk_lp_n_i;
  wire mipi_rx_clk_lp_n_i;
  input mipi_rx_clk_lp_p_i;
  wire mipi_rx_clk_lp_p_i;
  output mipi_rx_clk_term_en_o;
  wire mipi_rx_clk_term_en_o;
  input [7:0] mipi_rx_d0_i;
  wire [7:0] mipi_rx_d0_i;
  input mipi_rx_d0_lp_n_i;
  wire mipi_rx_d0_lp_n_i;
  input mipi_rx_d0_lp_p_i;
  wire mipi_rx_d0_lp_p_i;
  input [7:0] mipi_rx_d1_i;
  wire [7:0] mipi_rx_d1_i;
  input [7:0] mipi_rx_d2_i;
  wire [7:0] mipi_rx_d2_i;
  input [7:0] mipi_rx_d3_i;
  wire [7:0] mipi_rx_d3_i;
  output mipi_rx_dat_hs_en_o;
  wire mipi_rx_dat_hs_en_o;
  output mipi_rx_dat_term_en_o;
  wire mipi_rx_dat_term_en_o;
  output mipi_rx_rst_n_o;
  wire mipi_rx_rst_n_o;
  output mipi_rx_rst_o;
  wire mipi_rx_rst_o;
  input sclk_i;
  wire sclk_i;
  input srst_i;
  wire srst_i;
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[4] <= _1325_[4];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[5] <= _1325_[5];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[6] <= _1325_[6];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[7] <= _1325_[7];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[8] <= _1325_[8];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[9] <= _1325_[9];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[10] <= _1325_[10];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[11] <= _1325_[11];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[16] <= _1325_[16];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[17] <= _1325_[17];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[18] <= _1325_[18];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[19] <= _1325_[19];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[20] <= _1325_[20];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[21] <= _1325_[21];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[22] <= _1325_[22];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[23] <= _1325_[23];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[28] <= _1325_[28];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[29] <= _1325_[29];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[30] <= _1325_[30];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[31] <= _1325_[31];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[32] <= _1325_[32];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[33] <= _1325_[33];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[34] <= _1325_[34];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[35] <= _1325_[35];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[40] <= _1325_[40];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[41] <= _1325_[41];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[42] <= _1325_[42];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[43] <= _1325_[43];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[44] <= _1325_[44];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[45] <= _1325_[45];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[46] <= _1325_[46];
  always @(posedge mipi_rx_byteclk_i)
    if (_0000_) _0470_[47] <= _1325_[47];
  always @(posedge mipi_rx_byteclk_i)
    if (_0021_)
      if (_1044_) _0325_[0] <= 1'h0;
      else _0325_[0] <= _1332_[0];
  reg _1488_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0022_) _1488_ <= 1'h0;
    else _1488_ <= 1'h1;
  assign _0469_[1] = _1488_;
  reg _1489_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1489_ <= _0501_[0];
  assign _0471_[0] = _1489_;
  reg _1490_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1490_ <= _0501_[1];
  assign _0471_[1] = _1490_;
  reg _1491_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1491_ <= _0501_[2];
  assign _0471_[2] = _1491_;
  reg _1492_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1492_ <= _0501_[3];
  assign _0471_[3] = _1492_;
  reg _1493_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1493_ <= _0501_[4];
  assign _0471_[4] = _1493_;
  reg _1494_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1494_ <= _0501_[5];
  assign _0471_[5] = _1494_;
  reg _1495_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1495_ <= _0501_[6];
  assign _0471_[6] = _1495_;
  reg _1496_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1496_ <= _0501_[7];
  assign _0471_[7] = _1496_;
  reg _1497_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1497_ <= _0501_[8];
  assign _0471_[8] = _1497_;
  reg _1498_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1498_ <= _0501_[9];
  assign _0471_[9] = _1498_;
  reg _1499_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1499_ <= _0501_[10];
  assign _0471_[10] = _1499_;
  reg _1500_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1500_ <= _0501_[11];
  assign _0471_[11] = _1500_;
  reg _1501_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1501_ <= _0501_[12];
  assign _0471_[12] = _1501_;
  reg _1502_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1502_ <= _0501_[13];
  assign _0471_[13] = _1502_;
  reg _1503_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1503_ <= _0501_[14];
  assign _0471_[14] = _1503_;
  reg _1504_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1504_ <= _0501_[15];
  assign _0471_[15] = _1504_;
  reg _1505_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1505_ <= _0501_[16];
  assign _0471_[16] = _1505_;
  reg _1506_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1506_ <= _0501_[17];
  assign _0471_[17] = _1506_;
  reg _1507_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1507_ <= _0501_[18];
  assign _0471_[18] = _1507_;
  reg _1508_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1508_ <= _0501_[19];
  assign _0471_[19] = _1508_;
  reg _1509_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1509_ <= _0501_[20];
  assign _0471_[20] = _1509_;
  reg _1510_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1510_ <= _0501_[21];
  assign _0471_[21] = _1510_;
  reg _1511_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1511_ <= _0501_[22];
  assign _0471_[22] = _1511_;
  reg _1512_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1512_ <= _0501_[23];
  assign _0471_[23] = _1512_;
  reg _1513_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1513_ <= _0501_[24];
  assign _0471_[24] = _1513_;
  reg _1514_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1514_ <= _0501_[25];
  assign _0471_[25] = _1514_;
  reg _1515_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1515_ <= _0501_[26];
  assign _0471_[26] = _1515_;
  reg _1516_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1516_ <= _0501_[27];
  assign _0471_[27] = _1516_;
  reg _1517_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1517_ <= _0501_[28];
  assign _0471_[28] = _1517_;
  reg _1518_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1518_ <= _0501_[29];
  assign _0471_[29] = _1518_;
  reg _1519_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1519_ <= _0501_[30];
  assign _0471_[30] = _1519_;
  reg _1520_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0502_) _1520_ <= _0501_[31];
  assign _0471_[31] = _1520_;
  always @(posedge mipi_rx_byteclk_i)
    _0498_[0] <= _0228_[0];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[1] <= _0228_[1];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[2] <= _0228_[2];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[3] <= _0228_[3];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[4] <= _0228_[4];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[5] <= _0228_[5];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[6] <= _0228_[6];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[7] <= _0228_[7];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[8] <= _0228_[8];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[9] <= _0228_[9];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[10] <= _0228_[10];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[11] <= _0228_[11];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[12] <= _0228_[12];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[13] <= _0228_[13];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[14] <= _0228_[14];
  always @(posedge mipi_rx_byteclk_i)
    _0498_[15] <= _0228_[15];
  reg _1537_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    _1537_ <= _0404_[0];
  assign _0503_[0] = _1537_;
  reg _1538_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    _1538_ <= _0404_[1];
  assign _0503_[1] = _1538_;
  always @(posedge mipi_rx_byteclk_i)
    _0501_[0] <= _0411_[0];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[1] <= _0411_[1];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[2] <= _0411_[2];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[3] <= _0411_[3];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[4] <= _0411_[4];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[5] <= _0411_[5];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[6] <= _0411_[6];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[7] <= _0411_[7];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[8] <= _0411_[8];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[9] <= _0411_[9];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[10] <= _0411_[10];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[11] <= _0411_[11];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[12] <= _0411_[12];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[13] <= _0411_[13];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[14] <= _0411_[14];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[15] <= _0411_[15];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[16] <= _0411_[16];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[17] <= _0411_[17];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[18] <= _0411_[18];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[19] <= _0411_[19];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[20] <= _0411_[20];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[21] <= _0411_[21];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[22] <= _0411_[22];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[23] <= _0411_[23];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[24] <= _0411_[24];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[25] <= _0411_[25];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[26] <= _0411_[26];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[27] <= _0411_[27];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[28] <= _0411_[28];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[29] <= _0411_[29];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[30] <= _0411_[30];
  always @(posedge mipi_rx_byteclk_i)
    _0501_[31] <= _0411_[31];
  always @(posedge mipi_rx_byteclk_i)
    _0502_ <= _0412_;
  always @(posedge mipi_rx_byteclk_i)
    _0500_[0] <= _0221_;
  always @(posedge mipi_rx_byteclk_i)
    _0500_[1] <= _0233_[1];
  always @(posedge mipi_rx_byteclk_i)
    _0500_[2] <= _0233_[2];
  always @(posedge mipi_rx_byteclk_i)
    _0500_[3] <= _0233_[3];
  always @(posedge mipi_rx_byteclk_i)
    _0499_[0] <= _0397_[0];
  always @(posedge mipi_rx_byteclk_i)
    _0499_[1] <= _0397_[1];
  always @(posedge mipi_rx_byteclk_i)
    _0499_[2] <= _0397_[2];
  always @(posedge mipi_rx_byteclk_i)
    _0499_[3] <= _0397_[3];
  always @(posedge mipi_rx_byteclk_i)
    _0508_ <= _0507_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _0397_[1] <= 1'h0;
    else if (_0018_) _0397_[1] <= _1338_;
  always @(posedge mipi_rx_byteclk_i)
    _0506_[0] <= _0505_[0];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[1] <= _0505_[1];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[2] <= _0505_[2];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[3] <= _0505_[3];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[4] <= _0505_[4];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[5] <= _0505_[5];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[6] <= _0505_[6];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[7] <= _0505_[7];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[8] <= _0505_[8];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[9] <= _0505_[9];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[10] <= _0505_[10];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[11] <= _0505_[11];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[12] <= _0505_[12];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[13] <= _0505_[13];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[14] <= _0505_[14];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[15] <= _0505_[15];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[16] <= _0505_[16];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[17] <= _0505_[17];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[18] <= _0505_[18];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[19] <= _0505_[19];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[20] <= _0505_[20];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[21] <= _0505_[21];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[22] <= _0505_[22];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[23] <= _0505_[23];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[24] <= _0505_[24];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[25] <= _0505_[25];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[26] <= _0505_[26];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[27] <= _0505_[27];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[28] <= _0505_[28];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[29] <= _0505_[29];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[30] <= _0505_[30];
  always @(posedge mipi_rx_byteclk_i)
    _0506_[31] <= _0505_[31];
  always @(posedge mipi_rx_byteclk_i)
    _0507_ <= mipi_rx_dat_hs_en_o;
  always @(posedge mipi_rx_byteclk_i)
    _0505_[0] <= mipi_rx_d0_i[0];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[1] <= mipi_rx_d0_i[1];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[2] <= mipi_rx_d0_i[2];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[3] <= mipi_rx_d0_i[3];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[4] <= mipi_rx_d0_i[4];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[5] <= mipi_rx_d0_i[5];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[6] <= mipi_rx_d0_i[6];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[7] <= mipi_rx_d0_i[7];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[8] <= mipi_rx_d1_i[0];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[9] <= mipi_rx_d1_i[1];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[10] <= mipi_rx_d1_i[2];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[11] <= mipi_rx_d1_i[3];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[12] <= mipi_rx_d1_i[4];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[13] <= mipi_rx_d1_i[5];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[14] <= mipi_rx_d1_i[6];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[15] <= mipi_rx_d1_i[7];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[16] <= mipi_rx_d2_i[0];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[17] <= mipi_rx_d2_i[1];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[18] <= mipi_rx_d2_i[2];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[19] <= mipi_rx_d2_i[3];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[20] <= mipi_rx_d2_i[4];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[21] <= mipi_rx_d2_i[5];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[22] <= mipi_rx_d2_i[6];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[23] <= mipi_rx_d2_i[7];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[24] <= mipi_rx_d3_i[0];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[25] <= mipi_rx_d3_i[1];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[26] <= mipi_rx_d3_i[2];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[27] <= mipi_rx_d3_i[3];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[28] <= mipi_rx_d3_i[4];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[29] <= mipi_rx_d3_i[5];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[30] <= mipi_rx_d3_i[6];
  always @(posedge mipi_rx_byteclk_i)
    _0505_[31] <= mipi_rx_d3_i[7];
  always @(posedge mipi_rx_byteclk_i, posedge srst_i)
    if (srst_i) _0504_ <= 1'h1;
    else _0504_ <= 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _0397_[2] <= 1'h0;
    else if (_0019_) _0397_[2] <= _1338_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[0] <= 1'h0;
    else if (_0001_) _0470_[0] <= _1394_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[1] <= 1'h0;
    else if (_0001_) _0470_[1] <= _1394_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[2] <= 1'h0;
    else if (_0001_) _0470_[2] <= _1394_[2];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[3] <= 1'h0;
    else if (_0001_) _0470_[3] <= _1394_[3];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[12] <= 1'h0;
    else if (_0001_) _0470_[12] <= _1386_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[13] <= 1'h0;
    else if (_0001_) _0470_[13] <= _1386_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[14] <= 1'h0;
    else if (_0001_) _0470_[14] <= _1386_[2];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[15] <= 1'h0;
    else if (_0001_) _0470_[15] <= _1386_[3];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[24] <= 1'h0;
    else if (_0001_) _0470_[24] <= _1379_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[25] <= 1'h0;
    else if (_0001_) _0470_[25] <= _1379_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[26] <= 1'h0;
    else if (_0001_) _0470_[26] <= _1379_[2];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[27] <= 1'h0;
    else if (_0001_) _0470_[27] <= _1379_[3];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[36] <= 1'h0;
    else if (_0001_) _0470_[36] <= _1371_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[37] <= 1'h0;
    else if (_0001_) _0470_[37] <= _1371_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[38] <= 1'h0;
    else if (_0001_) _0470_[38] <= _1371_[2];
  always @(posedge mipi_rx_byteclk_i)
    if (_0466_) _0470_[39] <= 1'h0;
    else if (_0001_) _0470_[39] <= _1371_[3];
  always @(posedge mipi_rx_byteclk_i)
    if (_0024_) _0403_ <= 1'h0;
    else _0403_ <= _1348_;
  always @(posedge mipi_rx_byteclk_i)
    _0474_[0] <= _0503_[0];
  always @(posedge mipi_rx_byteclk_i)
    _0474_[1] <= _0503_[1];
  always @(posedge mipi_rx_byteclk_i)
    _0473_ <= _1326_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0504_) _0467_[0] <= 1'h0;
    else _0467_[0] <= _0499_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0504_) _0467_[1] <= 1'h0;
    else _0467_[1] <= _0499_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0504_) _0467_[2] <= 1'h0;
    else _0467_[2] <= _0499_[2];
  always @(posedge mipi_rx_byteclk_i)
    if (_0504_) _0467_[3] <= 1'h0;
    else _0467_[3] <= _0499_[3];
  always @(posedge mipi_rx_byteclk_i)
    if (_0504_) _0468_[0] <= 1'h0;
    else _0468_[0] <= _0500_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0504_) _0468_[1] <= 1'h0;
    else _0468_[1] <= _0500_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0504_) _0468_[2] <= 1'h0;
    else _0468_[2] <= _0500_[2];
  always @(posedge mipi_rx_byteclk_i)
    if (_0504_) _0468_[3] <= 1'h0;
    else _0468_[3] <= _0500_[3];
  always @(posedge mipi_rx_byteclk_i)
    if (_0023_) _0476_[0] <= 1'h0;
    else if (_0502_) _0476_[0] <= _1397_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0023_) _0476_[1] <= 1'h0;
    else if (_0502_) _0476_[1] <= _1397_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0023_) _0476_[2] <= 1'h0;
    else if (_0502_) _0476_[2] <= _1397_[2];
  always @(posedge mipi_rx_byteclk_i)
    _0466_ <= _1324_;
  reg _1681_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (!_0829_) _1681_ <= 1'h0;
    else _1681_ <= _1396_[2];
  assign _0469_[2] = _1681_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _0397_[3] <= 1'h0;
    else if (_0020_) _0397_[3] <= _1338_;
  reg _1683_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0010_) _1683_ <= _1343_[0];
  assign _0406_[8] = _1683_;
  reg _1684_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0010_) _1684_ <= _1343_[1];
  assign _0406_[9] = _1684_;
  reg _1685_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0010_) _1685_ <= _1343_[2];
  assign _0406_[10] = _1685_;
  reg _1686_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0010_) _1686_ <= _1343_[3];
  assign _0406_[11] = _1686_;
  reg _1687_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0010_) _1687_ <= _1343_[4];
  assign _0406_[12] = _1687_;
  reg _1688_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0010_) _1688_ <= _1343_[5];
  assign _0406_[13] = _1688_;
  reg _1689_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0010_) _1689_ <= _1343_[6];
  assign _0406_[14] = _1689_;
  reg _1690_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0010_) _1690_ <= _1343_[7];
  assign _0406_[15] = _1690_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[0] <= 1'h0;
      else _0416_[0] <= _1401_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[1] <= 1'h0;
      else _0416_[1] <= _1402_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[2] <= 1'h0;
      else _0416_[2] <= _1402_[2];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[3] <= 1'h0;
      else _0416_[3] <= _1402_[3];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[4] <= 1'h0;
      else _0416_[4] <= _1402_[4];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[5] <= 1'h0;
      else _0416_[5] <= _1402_[5];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[6] <= 1'h0;
      else _0416_[6] <= _1402_[6];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[7] <= 1'h0;
      else _0416_[7] <= _1402_[7];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[8] <= 1'h0;
      else _0416_[8] <= _1402_[8];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[9] <= 1'h0;
      else _0416_[9] <= _1402_[9];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[10] <= 1'h0;
      else _0416_[10] <= _1402_[10];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[11] <= 1'h0;
      else _0416_[11] <= _1402_[11];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[12] <= 1'h0;
      else _0416_[12] <= _1402_[12];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[13] <= 1'h0;
      else _0416_[13] <= _1402_[13];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[14] <= 1'h0;
      else _0416_[14] <= _1402_[14];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_)
      if (_0025_) _0416_[15] <= 1'h0;
      else _0416_[15] <= _1402_[15];
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _0402_ <= 1'h0;
    else if (_0009_) _0402_ <= _1349_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0009_) _0407_[0] <= _0328_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0009_) _0407_[1] <= _0328_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0009_) _0398_[0] <= _0328_[7];
  always @(posedge mipi_rx_byteclk_i)
    if (_0009_) _0398_[1] <= _0328_[6];
  always @(posedge mipi_rx_byteclk_i)
    if (_0009_) _0398_[2] <= _0328_[5];
  always @(posedge mipi_rx_byteclk_i)
    if (_0009_) _0398_[3] <= _0328_[4];
  always @(posedge mipi_rx_byteclk_i)
    if (_0009_) _0398_[4] <= _0328_[3];
  always @(posedge mipi_rx_byteclk_i)
    if (_0009_) _0398_[5] <= _0328_[2];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[0] <= _0328_[7];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[1] <= _0328_[6];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[2] <= _0328_[5];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[3] <= _0328_[4];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[4] <= _0328_[3];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[5] <= _0328_[2];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[6] <= _0328_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[7] <= _0328_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[8] <= _0328_[15];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[9] <= _0328_[14];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[10] <= _0328_[13];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[11] <= _0328_[12];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[12] <= _0328_[11];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[13] <= _0328_[10];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[14] <= _0328_[9];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[15] <= _0328_[8];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[16] <= _0328_[23];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[17] <= _0328_[22];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[18] <= _0328_[21];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[19] <= _0328_[20];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[20] <= _0328_[19];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[21] <= _0328_[18];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[22] <= _0328_[17];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[23] <= _0328_[16];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[24] <= _0328_[31];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[25] <= _0328_[30];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[26] <= _0328_[29];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[27] <= _0328_[28];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[28] <= _0328_[27];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[29] <= _0328_[26];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[30] <= _0328_[25];
  always @(posedge mipi_rx_byteclk_i)
    if (_0329_) _0411_[31] <= _0328_[24];
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _0417_[0] <= 1'h0;
    else if (_0012_) _0417_[0] <= _1362_[0];
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _0417_[1] <= 1'h0;
    else if (_0012_) _0417_[1] <= _1362_[1];
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _0417_[2] <= 1'h0;
    else if (_0012_) _0417_[2] <= _1362_[2];
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _0417_[3] <= 1'h0;
    else if (_0012_) _0417_[3] <= _1362_[3];
  always @(posedge mipi_rx_byteclk_i)
    if (_0013_)
      if (!_0871_) _0410_[0] <= 1'h0;
      else _0410_[0] <= _0911_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0013_)
      if (!_0871_) _0410_[1] <= 1'h0;
      else _0410_[1] <= _0872_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[0] <= _0328_[15];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[1] <= _0328_[14];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[2] <= _0328_[13];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[3] <= _0328_[12];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[4] <= _0328_[11];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[5] <= _0328_[10];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[6] <= _0328_[9];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[7] <= _0328_[8];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[8] <= _0328_[23];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[9] <= _0328_[22];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[10] <= _0328_[21];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[11] <= _0328_[20];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[12] <= _0328_[19];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[13] <= _0328_[18];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[14] <= _0328_[17];
  always @(posedge mipi_rx_byteclk_i)
    if (_0015_) _0228_[15] <= _0328_[16];
  always @(posedge mipi_rx_byteclk_i)
    _0412_ <= _0329_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _0409_ <= 1'h0;
    else if (_0014_) _0409_ <= _1363_;
  reg _1772_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1772_ <= _0506_[24];
  assign _0334_[0] = _1772_;
  reg _1773_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1773_ <= _0506_[25];
  assign _0334_[1] = _1773_;
  reg _1774_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1774_ <= _0506_[26];
  assign _0334_[2] = _1774_;
  reg _1775_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1775_ <= _0506_[27];
  assign _0334_[3] = _1775_;
  reg _1776_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1776_ <= _0506_[28];
  assign _0334_[4] = _1776_;
  reg _1777_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1777_ <= _0506_[29];
  assign _0334_[5] = _1777_;
  reg _1778_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1778_ <= _0506_[30];
  assign _0334_[6] = _1778_;
  reg _1779_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1779_ <= _0506_[31];
  assign _0334_[7] = _1779_;
  reg _1780_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1780_ <= _0334_[0];
  assign _0334_[8] = _1780_;
  reg _1781_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1781_ <= _0334_[1];
  assign _0334_[9] = _1781_;
  reg _1782_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1782_ <= _0334_[2];
  assign _0334_[10] = _1782_;
  reg _1783_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1783_ <= _0334_[3];
  assign _0334_[11] = _1783_;
  reg _1784_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1784_ <= _0334_[4];
  assign _0334_[12] = _1784_;
  reg _1785_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1785_ <= _0334_[5];
  assign _0334_[13] = _1785_;
  reg _1786_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1786_ <= _0334_[6];
  assign _0334_[14] = _1786_;
  reg _1787_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1787_ <= _0334_[7];
  assign _0334_[15] = _1787_;
  reg _1788_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1788_ <= _0334_[8];
  assign _0334_[16] = _1788_;
  reg _1789_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1789_ <= _0334_[9];
  assign _0334_[17] = _1789_;
  reg _1790_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1790_ <= _0334_[10];
  assign _0334_[18] = _1790_;
  reg _1791_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1791_ <= _0334_[11];
  assign _0334_[19] = _1791_;
  reg _1792_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1792_ <= _0334_[12];
  assign _0334_[20] = _1792_;
  reg _1793_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1793_ <= _0334_[13];
  assign _0334_[21] = _1793_;
  reg _1794_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1794_ <= _0334_[14];
  assign _0334_[22] = _1794_;
  reg _1795_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1795_ <= _0334_[15];
  assign _0334_[23] = _1795_;
  reg _1796_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1796_ <= _0334_[16];
  assign _0334_[24] = _1796_;
  reg _1797_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1797_ <= _0334_[17];
  assign _0334_[25] = _1797_;
  reg _1798_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1798_ <= _0334_[18];
  assign _0334_[26] = _1798_;
  reg _1799_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1799_ <= _0334_[19];
  assign _0334_[27] = _1799_;
  reg _1800_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1800_ <= _0334_[20];
  assign _0334_[28] = _1800_;
  reg _1801_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1801_ <= _0334_[21];
  assign _0334_[29] = _1801_;
  reg _1802_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1802_ <= _0334_[22];
  assign _0334_[30] = _1802_;
  reg _1803_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1803_ <= _0334_[23];
  assign _0334_[31] = _1803_;
  reg _1804_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1804_ <= _0334_[24];
  assign _0334_[32] = _1804_;
  reg _1805_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1805_ <= _0334_[25];
  assign _0334_[33] = _1805_;
  reg _1806_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1806_ <= _0334_[26];
  assign _0334_[34] = _1806_;
  reg _1807_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1807_ <= _0334_[27];
  assign _0334_[35] = _1807_;
  reg _1808_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1808_ <= _0334_[28];
  assign _0334_[36] = _1808_;
  reg _1809_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1809_ <= _0334_[29];
  assign _0334_[37] = _1809_;
  reg _1810_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1810_ <= _0334_[30];
  assign _0334_[38] = _1810_;
  reg _1811_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1811_ <= _0334_[31];
  assign _0334_[39] = _1811_;
  reg _1812_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1812_ <= _0334_[32];
  assign _0334_[40] = _1812_;
  reg _1813_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1813_ <= _0334_[33];
  assign _0334_[41] = _1813_;
  reg _1814_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1814_ <= _0334_[34];
  assign _0334_[42] = _1814_;
  reg _1815_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1815_ <= _0334_[35];
  assign _0334_[43] = _1815_;
  reg _1816_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1816_ <= _0334_[36];
  assign _0334_[44] = _1816_;
  reg _1817_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1817_ <= _0334_[37];
  assign _0334_[45] = _1817_;
  reg _1818_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _1818_ <= 1'h0;
    else if (_0016_) _1818_ <= _0410_[0];
  assign _0404_[0] = _1818_;
  reg _1819_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _1819_ <= 1'h0;
    else if (_0016_) _1819_ <= _0410_[1];
  assign _0404_[1] = _1819_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0396_) _0397_[0] <= 1'h0;
    else if (_0017_) _0397_[0] <= _1338_;
  always @(posedge mipi_rx_byteclk_i)
    _0396_ <= _0504_;
  always @(posedge mipi_rx_byteclk_i)
    _0327_[3] <= _1029_;
  always @(posedge mipi_rx_byteclk_i)
    _0326_[3] <= _1025_;
  always @(posedge mipi_rx_byteclk_i)
    _0327_[2] <= _1028_;
  always @(posedge mipi_rx_byteclk_i)
    _0326_[2] <= _1024_;
  always @(posedge mipi_rx_byteclk_i)
    _0327_[1] <= _1027_;
  always @(posedge mipi_rx_byteclk_i)
    _0326_[1] <= _1023_;
  always @(posedge mipi_rx_byteclk_i)
    _0327_[0] <= _1026_;
  always @(posedge mipi_rx_byteclk_i)
    _0326_[0] <= _1022_;
  always @(posedge mipi_rx_byteclk_i)
    _0324_[3] <= _1021_;
  always @(posedge mipi_rx_byteclk_i)
    _0323_[3] <= _1017_;
  always @(posedge mipi_rx_byteclk_i)
    _0322_[3] <= _1013_;
  always @(posedge mipi_rx_byteclk_i)
    _0321_[3] <= _1009_;
  always @(posedge mipi_rx_byteclk_i)
    _0324_[2] <= _1020_;
  always @(posedge mipi_rx_byteclk_i)
    _0323_[2] <= _1016_;
  always @(posedge mipi_rx_byteclk_i)
    _0322_[2] <= _1012_;
  always @(posedge mipi_rx_byteclk_i)
    _0321_[2] <= _1008_;
  always @(posedge mipi_rx_byteclk_i)
    _0324_[1] <= _1019_;
  always @(posedge mipi_rx_byteclk_i)
    _0323_[1] <= _1015_;
  always @(posedge mipi_rx_byteclk_i)
    _0322_[1] <= _1011_;
  always @(posedge mipi_rx_byteclk_i)
    _0321_[1] <= _1007_;
  always @(posedge mipi_rx_byteclk_i)
    _0324_[0] <= _1018_;
  always @(posedge mipi_rx_byteclk_i)
    _0323_[0] <= _1014_;
  always @(posedge mipi_rx_byteclk_i)
    _0322_[0] <= _1010_;
  always @(posedge mipi_rx_byteclk_i)
    _0321_[0] <= _1006_;
  always @(posedge mipi_rx_byteclk_i)
    _0328_[0] <= _1335_[0];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[1] <= _1335_[1];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[2] <= _1335_[2];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[3] <= _1335_[3];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[4] <= _1335_[4];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[5] <= _1335_[5];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[6] <= _1335_[6];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[7] <= _1335_[7];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[8] <= _1335_[8];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[9] <= _1335_[9];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[10] <= _1335_[10];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[11] <= _1335_[11];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[12] <= _1335_[12];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[13] <= _1335_[13];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[14] <= _1335_[14];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[15] <= _1335_[15];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[16] <= _1335_[16];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[17] <= _1335_[17];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[18] <= _1335_[18];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[19] <= _1335_[19];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[20] <= _1335_[20];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[21] <= _1335_[21];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[22] <= _1335_[22];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[23] <= _1335_[23];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[24] <= _1335_[24];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[25] <= _1335_[25];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[26] <= _1335_[26];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[27] <= _1335_[27];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[28] <= _1335_[28];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[29] <= _1335_[29];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[30] <= _1335_[30];
  always @(posedge mipi_rx_byteclk_i)
    _0328_[31] <= _1335_[31];
  always @(posedge mipi_rx_byteclk_i)
    _0329_ <= _0335_;
  always @(posedge mipi_rx_byteclk_i)
    if (_0021_)
      if (_0026_) _0325_[1] <= 1'h0;
      else _0325_[1] <= _1331_[1];
  always @(posedge mipi_rx_byteclk_i)
    _0335_ <= _0508_;
  reg _1881_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1881_ <= _0506_[16];
  assign _0333_[0] = _1881_;
  reg _1882_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1882_ <= _0506_[17];
  assign _0333_[1] = _1882_;
  reg _1883_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1883_ <= _0506_[18];
  assign _0333_[2] = _1883_;
  reg _1884_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1884_ <= _0506_[19];
  assign _0333_[3] = _1884_;
  reg _1885_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1885_ <= _0506_[20];
  assign _0333_[4] = _1885_;
  reg _1886_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1886_ <= _0506_[21];
  assign _0333_[5] = _1886_;
  reg _1887_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1887_ <= _0506_[22];
  assign _0333_[6] = _1887_;
  reg _1888_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1888_ <= _0506_[23];
  assign _0333_[7] = _1888_;
  reg _1889_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1889_ <= _0333_[0];
  assign _0333_[8] = _1889_;
  reg _1890_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1890_ <= _0333_[1];
  assign _0333_[9] = _1890_;
  reg _1891_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1891_ <= _0333_[2];
  assign _0333_[10] = _1891_;
  reg _1892_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1892_ <= _0333_[3];
  assign _0333_[11] = _1892_;
  reg _1893_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1893_ <= _0333_[4];
  assign _0333_[12] = _1893_;
  reg _1894_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1894_ <= _0333_[5];
  assign _0333_[13] = _1894_;
  reg _1895_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1895_ <= _0333_[6];
  assign _0333_[14] = _1895_;
  reg _1896_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1896_ <= _0333_[7];
  assign _0333_[15] = _1896_;
  reg _1897_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1897_ <= _0333_[8];
  assign _0333_[16] = _1897_;
  reg _1898_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1898_ <= _0333_[9];
  assign _0333_[17] = _1898_;
  reg _1899_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1899_ <= _0333_[10];
  assign _0333_[18] = _1899_;
  reg _1900_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1900_ <= _0333_[11];
  assign _0333_[19] = _1900_;
  reg _1901_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1901_ <= _0333_[12];
  assign _0333_[20] = _1901_;
  reg _1902_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1902_ <= _0333_[13];
  assign _0333_[21] = _1902_;
  reg _1903_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1903_ <= _0333_[14];
  assign _0333_[22] = _1903_;
  reg _1904_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1904_ <= _0333_[15];
  assign _0333_[23] = _1904_;
  reg _1905_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1905_ <= _0333_[16];
  assign _0333_[24] = _1905_;
  reg _1906_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1906_ <= _0333_[17];
  assign _0333_[25] = _1906_;
  reg _1907_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1907_ <= _0333_[18];
  assign _0333_[26] = _1907_;
  reg _1908_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1908_ <= _0333_[19];
  assign _0333_[27] = _1908_;
  reg _1909_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1909_ <= _0333_[20];
  assign _0333_[28] = _1909_;
  reg _1910_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1910_ <= _0333_[21];
  assign _0333_[29] = _1910_;
  reg _1911_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1911_ <= _0333_[22];
  assign _0333_[30] = _1911_;
  reg _1912_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1912_ <= _0333_[23];
  assign _0333_[31] = _1912_;
  reg _1913_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1913_ <= _0333_[24];
  assign _0333_[32] = _1913_;
  reg _1914_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1914_ <= _0333_[25];
  assign _0333_[33] = _1914_;
  reg _1915_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1915_ <= _0333_[26];
  assign _0333_[34] = _1915_;
  reg _1916_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1916_ <= _0333_[27];
  assign _0333_[35] = _1916_;
  reg _1917_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1917_ <= _0333_[28];
  assign _0333_[36] = _1917_;
  reg _1918_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1918_ <= _0333_[29];
  assign _0333_[37] = _1918_;
  reg _1919_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1919_ <= _0333_[30];
  assign _0333_[38] = _1919_;
  reg _1920_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1920_ <= _0333_[31];
  assign _0333_[39] = _1920_;
  reg _1921_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1921_ <= _0333_[32];
  assign _0333_[40] = _1921_;
  reg _1922_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1922_ <= _0333_[33];
  assign _0333_[41] = _1922_;
  reg _1923_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1923_ <= _0333_[34];
  assign _0333_[42] = _1923_;
  reg _1924_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1924_ <= _0333_[35];
  assign _0333_[43] = _1924_;
  reg _1925_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1925_ <= _0333_[36];
  assign _0333_[44] = _1925_;
  reg _1926_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1926_ <= _0333_[37];
  assign _0333_[45] = _1926_;
  reg _1927_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1927_ <= _0506_[8];
  assign _0332_[0] = _1927_;
  reg _1928_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1928_ <= _0506_[9];
  assign _0332_[1] = _1928_;
  reg _1929_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1929_ <= _0506_[10];
  assign _0332_[2] = _1929_;
  reg _1930_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1930_ <= _0506_[11];
  assign _0332_[3] = _1930_;
  reg _1931_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1931_ <= _0506_[12];
  assign _0332_[4] = _1931_;
  reg _1932_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1932_ <= _0506_[13];
  assign _0332_[5] = _1932_;
  reg _1933_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1933_ <= _0506_[14];
  assign _0332_[6] = _1933_;
  reg _1934_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1934_ <= _0506_[15];
  assign _0332_[7] = _1934_;
  reg _1935_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1935_ <= _0332_[0];
  assign _0332_[8] = _1935_;
  reg _1936_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1936_ <= _0332_[1];
  assign _0332_[9] = _1936_;
  reg _1937_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1937_ <= _0332_[2];
  assign _0332_[10] = _1937_;
  reg _1938_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1938_ <= _0332_[3];
  assign _0332_[11] = _1938_;
  reg _1939_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1939_ <= _0332_[4];
  assign _0332_[12] = _1939_;
  reg _1940_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1940_ <= _0332_[5];
  assign _0332_[13] = _1940_;
  reg _1941_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1941_ <= _0332_[6];
  assign _0332_[14] = _1941_;
  reg _1942_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1942_ <= _0332_[7];
  assign _0332_[15] = _1942_;
  reg _1943_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1943_ <= _0332_[8];
  assign _0332_[16] = _1943_;
  reg _1944_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1944_ <= _0332_[9];
  assign _0332_[17] = _1944_;
  reg _1945_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1945_ <= _0332_[10];
  assign _0332_[18] = _1945_;
  reg _1946_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1946_ <= _0332_[11];
  assign _0332_[19] = _1946_;
  reg _1947_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1947_ <= _0332_[12];
  assign _0332_[20] = _1947_;
  reg _1948_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1948_ <= _0332_[13];
  assign _0332_[21] = _1948_;
  reg _1949_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1949_ <= _0332_[14];
  assign _0332_[22] = _1949_;
  reg _1950_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1950_ <= _0332_[15];
  assign _0332_[23] = _1950_;
  reg _1951_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1951_ <= _0332_[16];
  assign _0332_[24] = _1951_;
  reg _1952_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1952_ <= _0332_[17];
  assign _0332_[25] = _1952_;
  reg _1953_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1953_ <= _0332_[18];
  assign _0332_[26] = _1953_;
  reg _1954_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1954_ <= _0332_[19];
  assign _0332_[27] = _1954_;
  reg _1955_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1955_ <= _0332_[20];
  assign _0332_[28] = _1955_;
  reg _1956_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1956_ <= _0332_[21];
  assign _0332_[29] = _1956_;
  reg _1957_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1957_ <= _0332_[22];
  assign _0332_[30] = _1957_;
  reg _1958_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1958_ <= _0332_[23];
  assign _0332_[31] = _1958_;
  reg _1959_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1959_ <= _0332_[24];
  assign _0332_[32] = _1959_;
  reg _1960_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1960_ <= _0332_[25];
  assign _0332_[33] = _1960_;
  reg _1961_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1961_ <= _0332_[26];
  assign _0332_[34] = _1961_;
  reg _1962_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1962_ <= _0332_[27];
  assign _0332_[35] = _1962_;
  reg _1963_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1963_ <= _0332_[28];
  assign _0332_[36] = _1963_;
  reg _1964_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1964_ <= _0332_[29];
  assign _0332_[37] = _1964_;
  reg _1965_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1965_ <= _0332_[30];
  assign _0332_[38] = _1965_;
  reg _1966_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1966_ <= _0332_[31];
  assign _0332_[39] = _1966_;
  reg _1967_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1967_ <= _0332_[32];
  assign _0332_[40] = _1967_;
  reg _1968_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1968_ <= _0332_[33];
  assign _0332_[41] = _1968_;
  reg _1969_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1969_ <= _0332_[34];
  assign _0332_[42] = _1969_;
  reg _1970_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1970_ <= _0332_[35];
  assign _0332_[43] = _1970_;
  reg _1971_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1971_ <= _0332_[36];
  assign _0332_[44] = _1971_;
  reg _1972_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1972_ <= _0332_[37];
  assign _0332_[45] = _1972_;
  reg _1973_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1973_ <= _0506_[0];
  assign _0331_[0] = _1973_;
  reg _1974_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1974_ <= _0506_[1];
  assign _0331_[1] = _1974_;
  reg _1975_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1975_ <= _0506_[2];
  assign _0331_[2] = _1975_;
  reg _1976_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1976_ <= _0506_[3];
  assign _0331_[3] = _1976_;
  reg _1977_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1977_ <= _0506_[4];
  assign _0331_[4] = _1977_;
  reg _1978_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1978_ <= _0506_[5];
  assign _0331_[5] = _1978_;
  reg _1979_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1979_ <= _0506_[6];
  assign _0331_[6] = _1979_;
  reg _1980_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1980_ <= _0506_[7];
  assign _0331_[7] = _1980_;
  reg _1981_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1981_ <= _0331_[0];
  assign _0331_[8] = _1981_;
  reg _1982_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1982_ <= _0331_[1];
  assign _0331_[9] = _1982_;
  reg _1983_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1983_ <= _0331_[2];
  assign _0331_[10] = _1983_;
  reg _1984_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1984_ <= _0331_[3];
  assign _0331_[11] = _1984_;
  reg _1985_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1985_ <= _0331_[4];
  assign _0331_[12] = _1985_;
  reg _1986_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1986_ <= _0331_[5];
  assign _0331_[13] = _1986_;
  reg _1987_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1987_ <= _0331_[6];
  assign _0331_[14] = _1987_;
  reg _1988_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1988_ <= _0331_[7];
  assign _0331_[15] = _1988_;
  reg _1989_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1989_ <= _0331_[8];
  assign _0331_[16] = _1989_;
  reg _1990_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1990_ <= _0331_[9];
  assign _0331_[17] = _1990_;
  reg _1991_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1991_ <= _0331_[10];
  assign _0331_[18] = _1991_;
  reg _1992_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1992_ <= _0331_[11];
  assign _0331_[19] = _1992_;
  reg _1993_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1993_ <= _0331_[12];
  assign _0331_[20] = _1993_;
  reg _1994_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1994_ <= _0331_[13];
  assign _0331_[21] = _1994_;
  reg _1995_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1995_ <= _0331_[14];
  assign _0331_[22] = _1995_;
  reg _1996_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1996_ <= _0331_[15];
  assign _0331_[23] = _1996_;
  reg _1997_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1997_ <= _0331_[16];
  assign _0331_[24] = _1997_;
  reg _1998_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1998_ <= _0331_[17];
  assign _0331_[25] = _1998_;
  reg _1999_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _1999_ <= _0331_[18];
  assign _0331_[26] = _1999_;
  reg _2000_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2000_ <= _0331_[19];
  assign _0331_[27] = _2000_;
  reg _2001_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2001_ <= _0331_[20];
  assign _0331_[28] = _2001_;
  reg _2002_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2002_ <= _0331_[21];
  assign _0331_[29] = _2002_;
  reg _2003_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2003_ <= _0331_[22];
  assign _0331_[30] = _2003_;
  reg _2004_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2004_ <= _0331_[23];
  assign _0331_[31] = _2004_;
  reg _2005_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2005_ <= _0331_[24];
  assign _0331_[32] = _2005_;
  reg _2006_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2006_ <= _0331_[25];
  assign _0331_[33] = _2006_;
  reg _2007_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2007_ <= _0331_[26];
  assign _0331_[34] = _2007_;
  reg _2008_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2008_ <= _0331_[27];
  assign _0331_[35] = _2008_;
  reg _2009_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2009_ <= _0331_[28];
  assign _0331_[36] = _2009_;
  reg _2010_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2010_ <= _0331_[29];
  assign _0331_[37] = _2010_;
  reg _2011_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2011_ <= _0331_[30];
  assign _0331_[38] = _2011_;
  reg _2012_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2012_ <= _0331_[31];
  assign _0331_[39] = _2012_;
  reg _2013_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2013_ <= _0331_[32];
  assign _0331_[40] = _2013_;
  reg _2014_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2014_ <= _0331_[33];
  assign _0331_[41] = _2014_;
  reg _2015_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2015_ <= _0331_[34];
  assign _0331_[42] = _2015_;
  reg _2016_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2016_ <= _0331_[35];
  assign _0331_[43] = _2016_;
  reg _2017_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2017_ <= _0331_[36];
  assign _0331_[44] = _2017_;
  reg _2018_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0508_) _2018_ <= _0331_[37];
  assign _0331_[45] = _2018_;
  reg _2019_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0011_) _2019_ <= _1340_[2];
  assign _0406_[2] = _2019_;
  reg _2020_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0011_) _2020_ <= _1340_[3];
  assign _0406_[3] = _2020_;
  reg _2021_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0011_) _2021_ <= _1340_[4];
  assign _0406_[4] = _2021_;
  reg _2022_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0011_) _2022_ <= _1340_[5];
  assign _0406_[5] = _2022_;
  reg _2023_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0011_) _2023_ <= _1340_[6];
  assign _0406_[6] = _2023_;
  reg _2024_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0011_) _2024_ <= _1340_[7];
  assign _0406_[7] = _2024_;
  reg _2025_ = 1'h0;
  always @(posedge mipi_rx_byteclk_i)
    if (_0504_) _2025_ <= 1'h0;
    else _2025_ <= _1399_[0];
  assign _0472_[0] = _2025_;
  assign _0547_ = ~_0331_[17];
  assign _0548_ = ~_0331_[18];
  assign _0551_ = ~_0332_[17];
  assign _0552_ = ~_0332_[18];
  assign _0555_ = ~_0333_[17];
  assign _0556_ = ~_0333_[18];
  assign _0559_ = ~_0334_[17];
  assign _0560_ = ~_0334_[18];
  assign _0562_ = ~_0331_[4];
  assign _0563_ = ~_0331_[2];
  assign _0564_ = ~_0417_[2];
  assign _0566_ = ~_0417_[0];
  assign _0565_ = ~_0417_[1];
  assign _0567_ = ~_0476_[1];
  assign _0568_ = ~_0476_[2];
  assign _0569_ = ~_0328_[4];
  assign _0570_ = ~_0328_[5];
  assign _0571_ = ~_0328_[11];
  assign _0572_ = ~_0328_[10];
  assign _0573_ = ~_0328_[0];
  assign _0574_ = ~_0328_[3];
  assign _0576_ = ~_0328_[26];
  assign _0577_ = ~_0328_[19];
  assign _0578_ = ~_0328_[18];
  assign _0579_ = ~_0328_[20];
  assign _0580_ = ~_0328_[8];
  assign _0581_ = ~_0328_[12];
  assign _1405_[15] = ~_0416_[15];
  assign _1405_[14] = ~_0416_[14];
  assign _1401_[0] = ~_0416_[0];
  assign _0582_ = ~_0328_[28];
  assign _0583_ = ~_0328_[16];
  assign _0584_ = ~_0328_[24];
  assign _0585_ = ~_0328_[27];
  assign _0586_ = ~_0325_[0];
  assign _0587_ = ~_0325_[1];
  assign _0588_ = ~_0328_[6];
  assign _0590_ = ~_0407_[1];
  assign _0591_ = ~_0407_[0];
  assign _0575_ = ~_0328_[2];
  assign _0589_ = ~_0328_[7];
  assign _1412_[0] = ~_0476_[0];
  assign _0592_ = ~_0474_[0];
  assign _0594_ = ~_0398_[0];
  assign _0593_ = ~_0474_[1];
  assign _0595_ = ~_0417_[3];
  assign _0596_ = ~_0398_[3];
  assign _0597_ = ~_0398_[2];
  assign _0598_ = ~_0398_[5];
  assign _0599_ = ~_0398_[1];
  assign _0600_ = ~_0331_[10];
  assign _0601_ = ~_0331_[8];
  assign _0602_ = ~_0331_[11];
  assign _0603_ = ~_0331_[12];
  assign _0604_ = ~_0332_[8];
  assign _0605_ = ~_0332_[10];
  assign _0606_ = ~_0332_[11];
  assign _0607_ = ~_0332_[12];
  assign _0608_ = ~_0333_[8];
  assign _0609_ = ~_0333_[10];
  assign _0610_ = ~_0333_[11];
  assign _0611_ = ~_0333_[12];
  assign _0612_ = ~_0334_[8];
  assign _0613_ = ~_0334_[10];
  assign _0614_ = ~_0334_[11];
  assign _0615_ = ~_0334_[12];
  assign _0616_ = ~_0331_[13];
  assign _0545_ = ~_0331_[14];
  assign _0617_ = ~_0332_[13];
  assign _0549_ = ~_0332_[14];
  assign _0618_ = ~_0333_[13];
  assign _0553_ = ~_0333_[14];
  assign _0619_ = ~_0334_[13];
  assign _0557_ = ~_0334_[14];
  assign _0561_ = ~_0331_[6];
  assign _0620_ = ~_0331_[15];
  assign _0546_ = ~_0331_[16];
  assign _0621_ = ~_0332_[15];
  assign _0550_ = ~_0332_[16];
  assign _0622_ = ~_0333_[15];
  assign _0554_ = ~_0333_[16];
  assign _0623_ = ~_0334_[15];
  assign _0558_ = ~_0334_[16];
  assign _0001_ = _0624_[0] | _0624_[1];
  assign _0000_ = _0001_ | _0466_;
  assign _0625_ = _1412_[0] | _0567_;
  assign _0859_ = _0625_ | _0476_[2];
  assign _0626_ = _0476_[0] | _0476_[1];
  assign _0860_ = _0626_ | _0568_;
  assign _0627_ = _1412_[0] | _0476_[1];
  assign _0861_ = _0627_ | _0476_[2];
  assign _0021_ = _0008_ & _1039_;
  assign _0628_ = _0476_[0] | _0567_;
  assign _0862_ = _0628_ | _0476_[2];
  assign _0007_ = _0925_ | _0926_;
  assign _0017_ = _0007_ & _0924_;
  assign _0629_[0] = _1350_ & _0544_;
  assign _0629_[1] = _0543_ & _0542_;
  assign _0629_[2] = _0541_ & _0329_;
  assign _0630_[0] = _0629_[0] & _0629_[1];
  assign _0630_[1] = _0629_[2] & _0629_[3];
  assign _0014_ = _0630_[0] & _0630_[1];
  assign _0018_ = _0007_ & _0927_;
  assign _0631_[0] = _0853_[5] & _0913_;
  assign _0631_[1] = _0329_ & _0538_;
  assign _0632_ = _0631_[0] & _0631_[1];
  assign _0015_ = _0632_ & _0537_;
  assign _0016_ = _0402_ & _0924_;
  assign _0019_ = _0007_ & _0928_;
  assign _0023_ = _0535_ | _0504_;
  assign _0020_ = _0007_ & _0929_;
  assign _0633_[0] = _1059_ | _1054_;
  assign _0026_ = _1049_ | _1044_;
  assign _0008_ = _0633_[0] | _0026_;
  assign _0634_[1] = _0417_[2] | _0595_;
  assign _0863_ = _0634_[0] | _0634_[1];
  assign _0539_ = _0635_[0] | _0635_[1];
  assign _0544_ = _0636_[0] | _0635_[1];
  assign _0635_[1] = _0564_ | _0417_[3];
  assign _0543_ = _0634_[0] | _0635_[1];
  assign _0636_[0] = _0566_ | _0417_[1];
  assign _0538_ = _0636_[0] | _0637_[1];
  assign _0635_[0] = _0417_[0] | _0565_;
  assign _0541_ = _0635_[0] | _0637_[1];
  assign _0638_[0] = _0566_ | _0565_;
  assign _0637_[1] = _0417_[2] | _0417_[3];
  assign _0542_ = _0638_[0] | _0637_[1];
  assign _0864_ = _0586_ | _0325_[1];
  assign _0865_ = _0325_[0] | _0587_;
  assign _0866_ = _0586_ | _0587_;
  assign _0022_ = _1326_ | _1327_;
  assign _0624_[0] = _1366_[2] | _1366_[1];
  assign _0624_[1] = _1366_[0] | _1366_[3];
  assign _0639_[0] = _0853_[5] & _0540_;
  assign _0639_[1] = _0931_ & _0329_;
  assign _0640_ = _0639_[0] & _0639_[1];
  assign _0013_ = _0640_ & _0629_[3];
  assign _0641_ = _0329_ & _0006_;
  assign _0012_ = _0641_ & _0005_;
  assign _0642_[0] = _0539_ | _0853_[1];
  assign _0642_[1] = _0853_[2] | _0853_[3];
  assign _0642_[2] = _0853_[4] | _0853_[5];
  assign _0642_[3] = _0853_[6] | _0853_[7];
  assign _0643_[0] = _0642_[0] | _0642_[1];
  assign _0643_[1] = _0642_[2] | _0642_[3];
  assign _0006_ = _0643_[0] | _0643_[1];
  assign _0644_[0] = _1352_ | _1350_;
  assign _0645_[0] = _0644_[0] | _0644_[1];
  assign _0645_[1] = _0644_[2] | _0004_;
  assign _0005_ = _0645_[0] | _0645_[1];
  assign _0867_ = _0474_[0] | _0593_;
  assign _0868_ = _0592_ | _0593_;
  assign _0646_[0] = _0500_[0] | _0500_[1];
  assign _0646_[1] = _0500_[2] | _0500_[3];
  assign _1329_ = _0646_[0] | _0646_[1];
  assign _0647_[0] = _0329_ & _0004_;
  assign _0011_ = _0647_[0] & _0629_[3];
  assign _0648_[0] = _0329_ & _0002_;
  assign _0648_[1] = _0003_ & _0538_;
  assign _0649_ = _0648_[0] & _0648_[1];
  assign _0010_ = _0649_ & _0537_;
  assign _0650_ = _0853_[2] | _0853_[5];
  assign _0002_ = _0650_ | _0853_[6];
  assign _0644_[1] = _0853_[1] | _0853_[2];
  assign _0004_ = _0853_[5] | _0853_[6];
  assign _0651_[0] = _0644_[1] | _0644_[2];
  assign _0651_[1] = _0004_ | _0539_;
  assign _0025_ = _0651_[0] | _0651_[1];
  assign _0652_[0] = _0853_[5] & _0329_;
  assign _0629_[3] = _0538_ & _0537_;
  assign _0009_ = _0652_[0] & _0629_[3];
  assign _0003_ = _0853_[5] | _0542_;
  assign _0644_[2] = _0853_[3] | _0853_[4];
  assign _0024_ = _0644_[2] | _0536_;
  assign _0653_[2] = _0574_ | _0575_;
  assign _0653_[3] = _0328_[1] | _0573_;
  assign _0653_[4] = _0328_[15] | _0328_[14];
  assign _0653_[5] = _0328_[13] | _0581_;
  assign _0653_[6] = _0571_ | _0572_;
  assign _0653_[7] = _0328_[9] | _0580_;
  assign _0653_[8] = _0328_[23] | _0328_[22];
  assign _0653_[9] = _0328_[21] | _0579_;
  assign _0653_[10] = _0577_ | _0578_;
  assign _0653_[11] = _0328_[17] | _0583_;
  assign _0653_[12] = _0328_[31] | _0328_[30];
  assign _0653_[13] = _0328_[29] | _0582_;
  assign _0653_[14] = _0585_ | _0576_;
  assign _0653_[15] = _0328_[25] | _0584_;
  assign _0654_[0] = _0653_[0] | _0653_[1];
  assign _0654_[1] = _0653_[2] | _0653_[3];
  assign _0654_[2] = _0653_[4] | _0653_[5];
  assign _0654_[3] = _0653_[6] | _0653_[7];
  assign _0654_[4] = _0653_[8] | _0653_[9];
  assign _0654_[5] = _0653_[10] | _0653_[11];
  assign _0654_[6] = _0653_[12] | _0653_[13];
  assign _0654_[7] = _0653_[14] | _0653_[15];
  assign _0655_[0] = _0654_[0] | _0654_[1];
  assign _0655_[1] = _0654_[2] | _0654_[3];
  assign _0655_[2] = _0654_[4] | _0654_[5];
  assign _0655_[3] = _0654_[6] | _0654_[7];
  assign _0656_[0] = _0655_[0] | _0655_[1];
  assign _0656_[1] = _0655_[2] | _0655_[3];
  assign _0869_ = _0656_[0] | _0656_[1];
  assign _0657_[0] = _0854_[0] | _0854_[1];
  assign _0657_[1] = _0854_[2] | _0854_[3];
  assign _0657_[2] = _0854_[4] | _0854_[5];
  assign _0658_ = _0657_[0] | _0657_[1];
  assign _0870_ = _0658_ | _0657_[2];
  assign _0659_[0] = _0328_[7] | _0588_;
  assign _0660_ = _0659_[0] | _0653_[1];
  assign _0871_ = _0660_ | _0659_[2];
  assign _0661_[0] = _0589_ | _0588_;
  assign _0653_[1] = _0328_[5] | _0569_;
  assign _0662_ = _0661_[0] | _0653_[1];
  assign _0872_ = _0662_ | _0659_[2];
  assign _0663_[1] = _0570_ | _0569_;
  assign _0659_[2] = _0328_[3] | _0575_;
  assign _0664_ = _0653_[0] | _0663_[1];
  assign _0873_ = _0664_ | _0659_[2];
  assign _0665_[0] = _0589_ | _0328_[6];
  assign _0665_[1] = _0328_[5] | _0328_[4];
  assign _0665_[2] = _0328_[3] | _0328_[2];
  assign _0666_ = _0665_[0] | _0665_[1];
  assign _0874_ = _0666_ | _0665_[2];
  assign _0667_[0] = _0398_[0] | _0599_;
  assign _0668_ = _0667_[0] | _0667_[1];
  assign _0875_ = _0668_ | _0667_[2];
  assign _0669_[0] = _0594_ | _0599_;
  assign _0667_[1] = _0398_[2] | _0596_;
  assign _0670_ = _0669_[0] | _0667_[1];
  assign _0876_ = _0670_ | _0667_[2];
  assign _0671_[1] = _0597_ | _0596_;
  assign _0667_[2] = _0398_[4] | _0598_;
  assign _0672_ = _0671_[0] | _0671_[1];
  assign _0877_ = _0672_ | _0667_[2];
  assign _0673_[0] = _0594_ | _0398_[1];
  assign _0673_[1] = _0398_[2] | _0398_[3];
  assign _0673_[2] = _0398_[4] | _0398_[5];
  assign _0674_ = _0673_[0] | _0673_[1];
  assign _0878_ = _0674_ | _0673_[2];
  assign _0879_ = _0591_ | _0407_[1];
  assign _0880_ = _0407_[0] | _0590_;
  assign _0881_ = _0591_ | _0590_;
  assign _0675_[0] = _0855_[0] | _0855_[1];
  assign _0675_[1] = _0855_[2] | _0855_[3];
  assign _0675_[2] = _0855_[4] | _0855_[5];
  assign _0676_ = _0675_[0] | _0675_[1];
  assign _0882_ = _0676_ | _0675_[2];
  assign _0677_ = _0562_ | _0331_[3];
  assign _0883_ = _0677_ | _0563_;
  assign _0678_ = _0561_ | _0331_[5];
  assign _0884_ = _0678_ | _0562_;
  assign _0679_[1] = _0334_[19] | _0560_;
  assign _0679_[2] = _0559_ | _0558_;
  assign _0679_[11] = _0334_[36] | _0334_[37];
  assign _0680_[0] = _0679_[0] | _0679_[1];
  assign _0680_[1] = _0679_[2] | _0679_[3];
  assign _0680_[5] = _0679_[10] | _0679_[11];
  assign _0681_[0] = _0680_[0] | _0680_[1];
  assign _0681_[1] = _0680_[2] | _0680_[3];
  assign _0681_[2] = _0680_[4] | _0680_[5];
  assign _0682_ = _0681_[0] | _0681_[1];
  assign _0885_ = _0682_ | _0681_[2];
  assign _0683_[1] = _0333_[19] | _0556_;
  assign _0683_[2] = _0555_ | _0554_;
  assign _0683_[11] = _0333_[36] | _0333_[37];
  assign _0684_[0] = _0683_[0] | _0683_[1];
  assign _0684_[1] = _0683_[2] | _0683_[3];
  assign _0684_[5] = _0683_[10] | _0683_[11];
  assign _0685_[0] = _0684_[0] | _0684_[1];
  assign _0685_[1] = _0684_[2] | _0684_[3];
  assign _0685_[2] = _0684_[4] | _0684_[5];
  assign _0686_ = _0685_[0] | _0685_[1];
  assign _0886_ = _0686_ | _0685_[2];
  assign _0687_[1] = _0332_[19] | _0552_;
  assign _0687_[2] = _0551_ | _0550_;
  assign _0687_[11] = _0332_[36] | _0332_[37];
  assign _0688_[0] = _0687_[0] | _0687_[1];
  assign _0688_[1] = _0687_[2] | _0687_[3];
  assign _0688_[5] = _0687_[10] | _0687_[11];
  assign _0689_[0] = _0688_[0] | _0688_[1];
  assign _0689_[1] = _0688_[2] | _0688_[3];
  assign _0689_[2] = _0688_[4] | _0688_[5];
  assign _0690_ = _0689_[0] | _0689_[1];
  assign _0887_ = _0690_ | _0689_[2];
  assign _0691_[1] = _0331_[19] | _0548_;
  assign _0691_[2] = _0547_ | _0546_;
  assign _0691_[11] = _0331_[36] | _0331_[37];
  assign _0692_[0] = _0691_[0] | _0691_[1];
  assign _0692_[1] = _0691_[2] | _0691_[3];
  assign _0692_[5] = _0691_[10] | _0691_[11];
  assign _0693_[0] = _0692_[0] | _0692_[1];
  assign _0693_[1] = _0692_[2] | _0692_[3];
  assign _0693_[2] = _0692_[4] | _0692_[5];
  assign _0694_ = _0693_[0] | _0693_[1];
  assign _0888_ = _0694_ | _0693_[2];
  assign _0695_[0] = _0856_[0] | _0856_[1];
  assign _0695_[1] = _0856_[2] | _0856_[3];
  assign _0695_[2] = _0856_[4] | _0856_[5];
  assign _0696_ = _0695_[0] | _0695_[1];
  assign _0889_ = _0696_ | _0695_[2];
  assign _0697_[1] = _0334_[17] | _0558_;
  assign _0697_[2] = _0623_ | _0557_;
  assign _0679_[10] = _0334_[34] | _0334_[35];
  assign _0698_[0] = _0697_[0] | _0697_[1];
  assign _0698_[1] = _0697_[2] | _0697_[3];
  assign _0698_[5] = _0679_[9] | _0679_[10];
  assign _0699_[0] = _0698_[0] | _0698_[1];
  assign _0699_[1] = _0698_[2] | _0698_[3];
  assign _0699_[2] = _0698_[4] | _0698_[5];
  assign _0700_ = _0699_[0] | _0699_[1];
  assign _0890_ = _0700_ | _0699_[2];
  assign _0701_[1] = _0333_[17] | _0554_;
  assign _0701_[2] = _0622_ | _0553_;
  assign _0683_[10] = _0333_[34] | _0333_[35];
  assign _0702_[0] = _0701_[0] | _0701_[1];
  assign _0702_[1] = _0701_[2] | _0701_[3];
  assign _0702_[5] = _0683_[9] | _0683_[10];
  assign _0703_[0] = _0702_[0] | _0702_[1];
  assign _0703_[1] = _0702_[2] | _0702_[3];
  assign _0703_[2] = _0702_[4] | _0702_[5];
  assign _0704_ = _0703_[0] | _0703_[1];
  assign _0891_ = _0704_ | _0703_[2];
  assign _0705_[1] = _0332_[17] | _0550_;
  assign _0705_[2] = _0621_ | _0549_;
  assign _0687_[10] = _0332_[34] | _0332_[35];
  assign _0706_[0] = _0705_[0] | _0705_[1];
  assign _0706_[1] = _0705_[2] | _0705_[3];
  assign _0706_[5] = _0687_[9] | _0687_[10];
  assign _0707_[0] = _0706_[0] | _0706_[1];
  assign _0707_[1] = _0706_[2] | _0706_[3];
  assign _0707_[2] = _0706_[4] | _0706_[5];
  assign _0708_ = _0707_[0] | _0707_[1];
  assign _0892_ = _0708_ | _0707_[2];
  assign _0709_[1] = _0331_[17] | _0546_;
  assign _0709_[2] = _0620_ | _0545_;
  assign _0691_[10] = _0331_[34] | _0331_[35];
  assign _0710_[0] = _0709_[0] | _0709_[1];
  assign _0710_[1] = _0709_[2] | _0709_[3];
  assign _0710_[5] = _0691_[9] | _0691_[10];
  assign _0711_[0] = _0710_[0] | _0710_[1];
  assign _0711_[1] = _0710_[2] | _0710_[3];
  assign _0711_[2] = _0710_[4] | _0710_[5];
  assign _0712_ = _0711_[0] | _0711_[1];
  assign _0893_ = _0712_ | _0711_[2];
  assign _0713_ = _0601_ | _0331_[7];
  assign _0894_ = _0713_ | _0561_;
  assign _0679_[3] = _0334_[15] | _0557_;
  assign _0714_[2] = _0619_ | _0615_;
  assign _0714_[3] = _0334_[11] | _0613_;
  assign _0679_[9] = _0334_[32] | _0334_[33];
  assign _0715_[0] = _0714_[0] | _0679_[3];
  assign _0715_[1] = _0714_[2] | _0714_[3];
  assign _0715_[2] = _0697_[0] | _0679_[0];
  assign _0680_[2] = _0679_[4] | _0679_[5];
  assign _0680_[3] = _0679_[6] | _0679_[7];
  assign _0680_[4] = _0679_[8] | _0679_[9];
  assign _0716_[0] = _0715_[0] | _0715_[1];
  assign _0716_[1] = _0715_[2] | _0680_[2];
  assign _0716_[2] = _0680_[3] | _0680_[4];
  assign _0717_ = _0716_[0] | _0716_[1];
  assign _0895_ = _0717_ | _0716_[2];
  assign _0683_[3] = _0333_[15] | _0553_;
  assign _0718_[2] = _0618_ | _0611_;
  assign _0718_[3] = _0333_[11] | _0609_;
  assign _0683_[9] = _0333_[32] | _0333_[33];
  assign _0719_[0] = _0718_[0] | _0683_[3];
  assign _0719_[1] = _0718_[2] | _0718_[3];
  assign _0719_[2] = _0701_[0] | _0683_[0];
  assign _0684_[2] = _0683_[4] | _0683_[5];
  assign _0684_[3] = _0683_[6] | _0683_[7];
  assign _0684_[4] = _0683_[8] | _0683_[9];
  assign _0720_[0] = _0719_[0] | _0719_[1];
  assign _0720_[1] = _0719_[2] | _0684_[2];
  assign _0720_[2] = _0684_[3] | _0684_[4];
  assign _0721_ = _0720_[0] | _0720_[1];
  assign _0896_ = _0721_ | _0720_[2];
  assign _0687_[3] = _0332_[15] | _0549_;
  assign _0722_[2] = _0617_ | _0607_;
  assign _0722_[3] = _0332_[11] | _0605_;
  assign _0687_[9] = _0332_[32] | _0332_[33];
  assign _0723_[0] = _0722_[0] | _0687_[3];
  assign _0723_[1] = _0722_[2] | _0722_[3];
  assign _0723_[2] = _0705_[0] | _0687_[0];
  assign _0688_[2] = _0687_[4] | _0687_[5];
  assign _0688_[3] = _0687_[6] | _0687_[7];
  assign _0688_[4] = _0687_[8] | _0687_[9];
  assign _0724_[0] = _0723_[0] | _0723_[1];
  assign _0724_[1] = _0723_[2] | _0688_[2];
  assign _0724_[2] = _0688_[3] | _0688_[4];
  assign _0725_ = _0724_[0] | _0724_[1];
  assign _0897_ = _0725_ | _0724_[2];
  assign _0691_[3] = _0331_[15] | _0545_;
  assign _0726_[2] = _0616_ | _0603_;
  assign _0726_[3] = _0331_[11] | _0600_;
  assign _0691_[9] = _0331_[32] | _0331_[33];
  assign _0727_[0] = _0726_[0] | _0691_[3];
  assign _0727_[1] = _0726_[2] | _0726_[3];
  assign _0727_[2] = _0709_[0] | _0691_[0];
  assign _0692_[2] = _0691_[4] | _0691_[5];
  assign _0692_[3] = _0691_[6] | _0691_[7];
  assign _0692_[4] = _0691_[8] | _0691_[9];
  assign _0728_[0] = _0727_[0] | _0727_[1];
  assign _0728_[1] = _0727_[2] | _0692_[2];
  assign _0728_[2] = _0692_[3] | _0692_[4];
  assign _0729_ = _0728_[0] | _0728_[1];
  assign _0898_ = _0729_ | _0728_[2];
  assign _0730_[0] = _0857_[0] | _0857_[1];
  assign _0730_[1] = _0857_[2] | _0857_[3];
  assign _0730_[2] = _0857_[4] | _0857_[5];
  assign _0731_ = _0730_[0] | _0730_[1];
  assign _0899_ = _0731_ | _0730_[2];
  assign _0732_[0] = _0334_[15] | _0334_[14];
  assign _0697_[3] = _0334_[13] | _0615_;
  assign _0732_[2] = _0614_ | _0613_;
  assign _0732_[3] = _0334_[9] | _0612_;
  assign _0714_[0] = _0334_[16] | _0334_[17];
  assign _0697_[0] = _0334_[18] | _0334_[19];
  assign _0679_[0] = _0334_[20] | _0334_[21];
  assign _0679_[4] = _0334_[22] | _0334_[23];
  assign _0679_[5] = _0334_[24] | _0334_[25];
  assign _0679_[6] = _0334_[26] | _0334_[27];
  assign _0679_[7] = _0334_[28] | _0334_[29];
  assign _0679_[8] = _0334_[30] | _0334_[31];
  assign _0733_[0] = _0732_[0] | _0697_[3];
  assign _0733_[1] = _0732_[2] | _0732_[3];
  assign _0733_[2] = _0714_[0] | _0697_[0];
  assign _0698_[2] = _0679_[0] | _0679_[4];
  assign _0698_[3] = _0679_[5] | _0679_[6];
  assign _0698_[4] = _0679_[7] | _0679_[8];
  assign _0734_[0] = _0733_[0] | _0733_[1];
  assign _0734_[1] = _0733_[2] | _0698_[2];
  assign _0734_[2] = _0698_[3] | _0698_[4];
  assign _0735_ = _0734_[0] | _0734_[1];
  assign _0900_ = _0735_ | _0734_[2];
  assign _0736_[0] = _0333_[15] | _0333_[14];
  assign _0701_[3] = _0333_[13] | _0611_;
  assign _0736_[2] = _0610_ | _0609_;
  assign _0736_[3] = _0333_[9] | _0608_;
  assign _0718_[0] = _0333_[16] | _0333_[17];
  assign _0701_[0] = _0333_[18] | _0333_[19];
  assign _0683_[0] = _0333_[20] | _0333_[21];
  assign _0683_[4] = _0333_[22] | _0333_[23];
  assign _0683_[5] = _0333_[24] | _0333_[25];
  assign _0683_[6] = _0333_[26] | _0333_[27];
  assign _0683_[7] = _0333_[28] | _0333_[29];
  assign _0683_[8] = _0333_[30] | _0333_[31];
  assign _0737_[0] = _0736_[0] | _0701_[3];
  assign _0737_[1] = _0736_[2] | _0736_[3];
  assign _0737_[2] = _0718_[0] | _0701_[0];
  assign _0702_[2] = _0683_[0] | _0683_[4];
  assign _0702_[3] = _0683_[5] | _0683_[6];
  assign _0702_[4] = _0683_[7] | _0683_[8];
  assign _0738_[0] = _0737_[0] | _0737_[1];
  assign _0738_[1] = _0737_[2] | _0702_[2];
  assign _0738_[2] = _0702_[3] | _0702_[4];
  assign _0739_ = _0738_[0] | _0738_[1];
  assign _0901_ = _0739_ | _0738_[2];
  assign _0740_[0] = _0332_[15] | _0332_[14];
  assign _0705_[3] = _0332_[13] | _0607_;
  assign _0740_[2] = _0606_ | _0605_;
  assign _0740_[3] = _0332_[9] | _0604_;
  assign _0722_[0] = _0332_[16] | _0332_[17];
  assign _0705_[0] = _0332_[18] | _0332_[19];
  assign _0687_[0] = _0332_[20] | _0332_[21];
  assign _0687_[4] = _0332_[22] | _0332_[23];
  assign _0687_[5] = _0332_[24] | _0332_[25];
  assign _0687_[6] = _0332_[26] | _0332_[27];
  assign _0687_[7] = _0332_[28] | _0332_[29];
  assign _0687_[8] = _0332_[30] | _0332_[31];
  assign _0741_[0] = _0740_[0] | _0705_[3];
  assign _0741_[1] = _0740_[2] | _0740_[3];
  assign _0741_[2] = _0722_[0] | _0705_[0];
  assign _0706_[2] = _0687_[0] | _0687_[4];
  assign _0706_[3] = _0687_[5] | _0687_[6];
  assign _0706_[4] = _0687_[7] | _0687_[8];
  assign _0742_[0] = _0741_[0] | _0741_[1];
  assign _0742_[1] = _0741_[2] | _0706_[2];
  assign _0742_[2] = _0706_[3] | _0706_[4];
  assign _0743_ = _0742_[0] | _0742_[1];
  assign _0902_ = _0743_ | _0742_[2];
  assign _0744_[0] = _0331_[15] | _0331_[14];
  assign _0709_[3] = _0331_[13] | _0603_;
  assign _0744_[2] = _0602_ | _0600_;
  assign _0744_[3] = _0331_[9] | _0601_;
  assign _0726_[0] = _0331_[16] | _0331_[17];
  assign _0709_[0] = _0331_[18] | _0331_[19];
  assign _0691_[0] = _0331_[20] | _0331_[21];
  assign _0691_[4] = _0331_[22] | _0331_[23];
  assign _0691_[5] = _0331_[24] | _0331_[25];
  assign _0691_[6] = _0331_[26] | _0331_[27];
  assign _0691_[7] = _0331_[28] | _0331_[29];
  assign _0691_[8] = _0331_[30] | _0331_[31];
  assign _0745_[0] = _0744_[0] | _0709_[3];
  assign _0745_[1] = _0744_[2] | _0744_[3];
  assign _0745_[2] = _0726_[0] | _0709_[0];
  assign _0710_[2] = _0691_[0] | _0691_[4];
  assign _0710_[3] = _0691_[5] | _0691_[6];
  assign _0710_[4] = _0691_[7] | _0691_[8];
  assign _0746_[0] = _0745_[0] | _0745_[1];
  assign _0746_[1] = _0745_[2] | _0710_[2];
  assign _0746_[2] = _0710_[3] | _0710_[4];
  assign _0747_ = _0746_[0] | _0746_[1];
  assign _0903_ = _0747_ | _0746_[2];
  assign _0748_ = _0600_ | _0331_[9];
  assign _0904_ = _0748_ | _0601_;
  assign _0749_[0] = _0858_[0] | _0858_[1];
  assign _0749_[1] = _0858_[2] | _0858_[3];
  assign _0749_[2] = _0858_[4] | _0858_[5];
  assign _0750_ = _0749_[0] | _0749_[1];
  assign _0905_ = _0750_ | _0749_[2];
  assign _0906_ = _0592_ | _0474_[1];
  assign _0751_[0] = _1370_[3] | _1370_[7];
  assign _0751_[1] = _1370_[11] | _1370_[15];
  assign _1372_[3] = _0751_[0] | _0751_[1];
  assign _0752_[0] = _1366_[0] | _1366_[1];
  assign _0752_[1] = _1366_[2] | _1366_[3];
  assign _0753_[0] = _1334_[0] | _1334_[1];
  assign _0753_[1] = _1334_[2] | _1334_[3];
  assign _1418_ = _0753_[0] | _0753_[1];
  assign _0754_[0] = _1333_[1] | _1333_[33];
  assign _0754_[1] = _1333_[65] | _1333_[97];
  assign _1336_[1] = _0754_[0] | _0754_[1];
  assign _1372_[0] = _1370_[8] | _1370_[12];
  assign _0755_[0] = _1333_[0] | _1333_[32];
  assign _0755_[1] = _1333_[64] | _1333_[96];
  assign _1336_[0] = _0755_[0] | _0755_[1];
  assign _0756_[0] = _1333_[3] | _1333_[35];
  assign _0756_[1] = _1333_[67] | _1333_[99];
  assign _1336_[3] = _0756_[0] | _0756_[1];
  assign _0757_[0] = _1333_[2] | _1333_[34];
  assign _0757_[1] = _1333_[66] | _1333_[98];
  assign _1336_[2] = _0757_[0] | _0757_[1];
  assign _0758_[0] = _1333_[5] | _1333_[37];
  assign _0758_[1] = _1333_[69] | _1333_[101];
  assign _1336_[5] = _0758_[0] | _0758_[1];
  assign _0759_[0] = _1333_[4] | _1333_[36];
  assign _0759_[1] = _1333_[68] | _1333_[100];
  assign _1336_[4] = _0759_[0] | _0759_[1];
  assign _0760_[0] = _1333_[7] | _1333_[39];
  assign _0760_[1] = _1333_[71] | _1333_[103];
  assign _1336_[7] = _0760_[0] | _0760_[1];
  assign _1372_[1] = _1370_[9] | _1370_[13];
  assign _0761_[0] = _1333_[6] | _1333_[38];
  assign _0761_[1] = _1333_[70] | _1333_[102];
  assign _1336_[6] = _0761_[0] | _0761_[1];
  assign _0762_[0] = _1333_[9] | _1333_[41];
  assign _0762_[1] = _1333_[73] | _1333_[105];
  assign _1336_[9] = _0762_[0] | _0762_[1];
  assign _0763_[0] = _1333_[8] | _1333_[40];
  assign _0763_[1] = _1333_[72] | _1333_[104];
  assign _1336_[8] = _0763_[0] | _0763_[1];
  assign _0764_[0] = _1333_[11] | _1333_[43];
  assign _0764_[1] = _1333_[75] | _1333_[107];
  assign _1336_[11] = _0764_[0] | _0764_[1];
  assign _0765_[0] = _1370_[2] | _1370_[6];
  assign _0765_[1] = _1370_[10] | _1370_[14];
  assign _1372_[2] = _0765_[0] | _0765_[1];
  assign _0766_[0] = _1333_[10] | _1333_[42];
  assign _0766_[1] = _1333_[74] | _1333_[106];
  assign _1336_[10] = _0766_[0] | _0766_[1];
  assign _0767_[0] = _1333_[13] | _1333_[45];
  assign _0767_[1] = _1333_[77] | _1333_[109];
  assign _1336_[13] = _0767_[0] | _0767_[1];
  assign _0768_[0] = _1333_[12] | _1333_[44];
  assign _0768_[1] = _1333_[76] | _1333_[108];
  assign _1336_[12] = _0768_[0] | _0768_[1];
  assign _0769_[0] = _1333_[15] | _1333_[47];
  assign _0769_[1] = _1333_[79] | _1333_[111];
  assign _1336_[15] = _0769_[0] | _0769_[1];
  assign _0770_[0] = _1333_[14] | _1333_[46];
  assign _0770_[1] = _1333_[78] | _1333_[110];
  assign _1336_[14] = _0770_[0] | _0770_[1];
  assign _0771_[0] = _1333_[17] | _1333_[49];
  assign _0771_[1] = _1333_[81] | _1333_[113];
  assign _1336_[17] = _0771_[0] | _0771_[1];
  assign _0772_[0] = _1333_[16] | _1333_[48];
  assign _0772_[1] = _1333_[80] | _1333_[112];
  assign _1336_[16] = _0772_[0] | _0772_[1];
  assign _0773_[0] = _1333_[19] | _1333_[51];
  assign _0773_[1] = _1333_[83] | _1333_[115];
  assign _1336_[19] = _0773_[0] | _0773_[1];
  assign _0774_[0] = _1333_[18] | _1333_[50];
  assign _0774_[1] = _1333_[82] | _1333_[114];
  assign _1336_[18] = _0774_[0] | _0774_[1];
  assign _0775_[0] = _1333_[20] | _1333_[52];
  assign _0775_[1] = _1333_[84] | _1333_[116];
  assign _1336_[20] = _0775_[0] | _0775_[1];
  assign _0776_[0] = _1333_[21] | _1333_[53];
  assign _0776_[1] = _1333_[85] | _1333_[117];
  assign _1336_[21] = _0776_[0] | _0776_[1];
  assign _0777_[0] = _1333_[22] | _1333_[54];
  assign _0777_[1] = _1333_[86] | _1333_[118];
  assign _1336_[22] = _0777_[0] | _0777_[1];
  assign _0778_[0] = _1333_[23] | _1333_[55];
  assign _0778_[1] = _1333_[87] | _1333_[119];
  assign _1336_[23] = _0778_[0] | _0778_[1];
  assign _0779_[0] = _1333_[24] | _1333_[56];
  assign _0779_[1] = _1333_[88] | _1333_[120];
  assign _1336_[24] = _0779_[0] | _0779_[1];
  assign _0780_[0] = _1333_[25] | _1333_[57];
  assign _0780_[1] = _1333_[89] | _1333_[121];
  assign _1336_[25] = _0780_[0] | _0780_[1];
  assign _0781_[0] = _1333_[26] | _1333_[58];
  assign _0781_[1] = _1333_[90] | _1333_[122];
  assign _1336_[26] = _0781_[0] | _0781_[1];
  assign _0782_[0] = _1333_[27] | _1333_[59];
  assign _0782_[1] = _1333_[91] | _1333_[123];
  assign _1336_[27] = _0782_[0] | _0782_[1];
  assign _0783_[0] = _1333_[28] | _1333_[60];
  assign _0783_[1] = _1333_[92] | _1333_[124];
  assign _1336_[28] = _0783_[0] | _0783_[1];
  assign _0784_[0] = _1333_[29] | _1333_[61];
  assign _0784_[1] = _1333_[93] | _1333_[125];
  assign _1336_[29] = _0784_[0] | _0784_[1];
  assign _0785_[0] = _1333_[30] | _1333_[62];
  assign _0785_[1] = _1333_[94] | _1333_[126];
  assign _1336_[30] = _0785_[0] | _0785_[1];
  assign _0786_[0] = _1333_[31] | _1333_[63];
  assign _0786_[1] = _1333_[95] | _1333_[127];
  assign _1336_[31] = _0786_[0] | _0786_[1];
  assign _0787_[0] = _1393_[2] | _1393_[6];
  assign _0787_[1] = _1393_[10] | _1393_[14];
  assign _1395_[2] = _0787_[0] | _0787_[1];
  assign _1395_[1] = _1393_[9] | _1393_[13];
  assign _1395_[0] = _1393_[8] | _1393_[12];
  assign _0788_[0] = _1374_[3] | _1374_[11];
  assign _0788_[1] = _1374_[19] | _1374_[27];
  assign _1376_[3] = _0788_[0] | _0788_[1];
  assign _0789_[0] = _1374_[2] | _1374_[10];
  assign _0789_[1] = _1374_[18] | _1374_[26];
  assign _1376_[2] = _0789_[0] | _0789_[1];
  assign _0790_[0] = _1374_[1] | _1374_[9];
  assign _0790_[1] = _1374_[17] | _1374_[25];
  assign _1376_[1] = _0790_[0] | _0790_[1];
  assign _0791_[0] = _1374_[0] | _1374_[8];
  assign _0791_[1] = _1374_[16] | _1374_[24];
  assign _1376_[0] = _0791_[0] | _0791_[1];
  assign _0792_[0] = _1378_[3] | _1378_[7];
  assign _0792_[1] = _1378_[11] | _1378_[15];
  assign _1380_[3] = _0792_[0] | _0792_[1];
  assign _0793_[0] = _1378_[2] | _1378_[6];
  assign _0793_[1] = _1378_[10] | _1378_[14];
  assign _1380_[2] = _0793_[0] | _0793_[1];
  assign _1380_[1] = _1378_[9] | _1378_[13];
  assign _1380_[0] = _1378_[8] | _1378_[12];
  assign _0794_[0] = _1374_[7] | _1374_[15];
  assign _0794_[1] = _1374_[23] | _1374_[31];
  assign _1376_[7] = _0794_[0] | _0794_[1];
  assign _0795_[0] = _1374_[6] | _1374_[14];
  assign _0795_[1] = _1374_[22] | _1374_[30];
  assign _1376_[6] = _0795_[0] | _0795_[1];
  assign _0796_[0] = _1374_[5] | _1374_[13];
  assign _0796_[1] = _1374_[21] | _1374_[29];
  assign _1376_[5] = _0796_[0] | _0796_[1];
  assign _0797_[0] = _1374_[4] | _1374_[12];
  assign _0797_[1] = _1374_[20] | _1374_[28];
  assign _1376_[4] = _0797_[0] | _0797_[1];
  assign _0798_[0] = _1381_[7] | _1381_[15];
  assign _0798_[1] = _1381_[23] | _1381_[31];
  assign _1383_[7] = _0798_[0] | _0798_[1];
  assign _0799_[0] = _1381_[6] | _1381_[14];
  assign _0799_[1] = _1381_[22] | _1381_[30];
  assign _1383_[6] = _0799_[0] | _0799_[1];
  assign _0800_[0] = _1381_[5] | _1381_[13];
  assign _0800_[1] = _1381_[21] | _1381_[29];
  assign _1383_[5] = _0800_[0] | _0800_[1];
  assign _0801_[0] = _1381_[4] | _1381_[12];
  assign _0801_[1] = _1381_[20] | _1381_[28];
  assign _1383_[4] = _0801_[0] | _0801_[1];
  assign _0802_[0] = _1381_[3] | _1381_[11];
  assign _0802_[1] = _1381_[19] | _1381_[27];
  assign _1383_[3] = _0802_[0] | _0802_[1];
  assign _0803_[0] = _1381_[2] | _1381_[10];
  assign _0803_[1] = _1381_[18] | _1381_[26];
  assign _1383_[2] = _0803_[0] | _0803_[1];
  assign _0804_[0] = _1381_[1] | _1381_[9];
  assign _0804_[1] = _1381_[17] | _1381_[25];
  assign _1383_[1] = _0804_[0] | _0804_[1];
  assign _0805_[0] = _1381_[0] | _1381_[8];
  assign _0805_[1] = _1381_[16] | _1381_[24];
  assign _1383_[0] = _0805_[0] | _0805_[1];
  assign _0806_[0] = _1385_[3] | _1385_[7];
  assign _0806_[1] = _1385_[11] | _1385_[15];
  assign _1387_[3] = _0806_[0] | _0806_[1];
  assign _0807_[0] = _1385_[2] | _1385_[6];
  assign _0807_[1] = _1385_[10] | _1385_[14];
  assign _1387_[2] = _0807_[0] | _0807_[1];
  assign _1387_[1] = _1385_[9] | _1385_[13];
  assign _1387_[0] = _1385_[8] | _1385_[12];
  assign _0808_[0] = _1389_[0] | _1389_[8];
  assign _0808_[1] = _1389_[16] | _1389_[24];
  assign _1391_[0] = _0808_[0] | _0808_[1];
  assign _0809_[0] = _1389_[1] | _1389_[9];
  assign _0809_[1] = _1389_[17] | _1389_[25];
  assign _1391_[1] = _0809_[0] | _0809_[1];
  assign _0810_[0] = _1389_[2] | _1389_[10];
  assign _0810_[1] = _1389_[18] | _1389_[26];
  assign _1391_[2] = _0810_[0] | _0810_[1];
  assign _0811_[0] = _1389_[3] | _1389_[11];
  assign _0811_[1] = _1389_[19] | _1389_[27];
  assign _1391_[3] = _0811_[0] | _0811_[1];
  assign _0812_[0] = _1389_[4] | _1389_[12];
  assign _0812_[1] = _1389_[20] | _1389_[28];
  assign _1391_[4] = _0812_[0] | _0812_[1];
  assign _0813_[0] = _1389_[5] | _1389_[13];
  assign _0813_[1] = _1389_[21] | _1389_[29];
  assign _1391_[5] = _0813_[0] | _0813_[1];
  assign _0814_[0] = _1389_[6] | _1389_[14];
  assign _0814_[1] = _1389_[22] | _1389_[30];
  assign _1391_[6] = _0814_[0] | _0814_[1];
  assign _0815_[0] = _1389_[7] | _1389_[15];
  assign _0815_[1] = _1389_[23] | _1389_[31];
  assign _1391_[7] = _0815_[0] | _0815_[1];
  assign _0853_[7] = _1408_ | _1409_;
  assign _0816_[0] = _1405_[0] & _1405_[1];
  assign _0816_[1] = _1405_[2] & _1405_[3];
  assign _0816_[2] = _1405_[4] & _1405_[5];
  assign _0816_[3] = _1405_[6] & _1405_[7];
  assign _0816_[4] = _1405_[8] & _1405_[9];
  assign _0816_[6] = _1405_[12] & _1405_[13];
  assign _0816_[7] = _1405_[14] & _1405_[15];
  assign _0817_[0] = _0816_[0] & _0816_[1];
  assign _0817_[2] = _0816_[4] & _0816_[5];
  assign _0818_[0] = _0817_[0] & _0817_[1];
  assign _0818_[1] = _0817_[2] & _0817_[3];
  assign _1409_ = _0818_[0] & _0818_[1];
  assign _0819_[0] = _1365_[6] | _1365_[14];
  assign _0819_[1] = _1365_[22] | _1365_[30];
  assign _1368_[6] = _0819_[0] | _0819_[1];
  assign _0820_[0] = _1365_[7] | _1365_[15];
  assign _0820_[1] = _1365_[23] | _1365_[31];
  assign _1368_[7] = _0820_[0] | _0820_[1];
  assign _0821_[0] = _1393_[3] | _1393_[7];
  assign _0821_[1] = _1393_[11] | _1393_[15];
  assign _1395_[3] = _0821_[0] | _0821_[1];
  assign _0822_[0] = _1365_[3] | _1365_[11];
  assign _0822_[1] = _1365_[19] | _1365_[27];
  assign _1368_[3] = _0822_[0] | _0822_[1];
  assign _0823_[0] = _1365_[4] | _1365_[12];
  assign _0823_[1] = _1365_[20] | _1365_[28];
  assign _1368_[4] = _0823_[0] | _0823_[1];
  assign _0824_[0] = _1365_[0] | _1365_[8];
  assign _0824_[1] = _1365_[16] | _1365_[24];
  assign _1368_[0] = _0824_[0] | _0824_[1];
  assign _0825_[0] = _1365_[1] | _1365_[9];
  assign _0825_[1] = _1365_[17] | _1365_[25];
  assign _1368_[1] = _0825_[0] | _0825_[1];
  assign _0826_[0] = _1365_[2] | _1365_[10];
  assign _0826_[1] = _1365_[18] | _1365_[26];
  assign _1368_[2] = _0826_[0] | _0826_[1];
  assign _1419_ = _0752_[0] | _0752_[1];
  assign _0827_[0] = _1365_[5] | _1365_[13];
  assign _0827_[1] = _1365_[21] | _1365_[29];
  assign _1368_[5] = _0827_[0] | _0827_[1];
  assign _0634_[0] = _0417_[0] | _0417_[1];
  assign _0537_ = _0634_[0] | _0637_[1];
  assign _0828_ = _0325_[0] | _0325_[1];
  assign _0829_ = _0474_[0] | _0474_[1];
  assign _0830_[3] = _0328_[1] | _0328_[0];
  assign _0830_[5] = _0328_[13] | _0328_[12];
  assign _0830_[6] = _0328_[11] | _0328_[10];
  assign _0830_[7] = _0328_[9] | _0328_[8];
  assign _0830_[9] = _0328_[21] | _0328_[20];
  assign _0830_[10] = _0328_[19] | _0328_[18];
  assign _0830_[11] = _0328_[17] | _0328_[16];
  assign _0830_[13] = _0328_[29] | _0328_[28];
  assign _0830_[14] = _0328_[27] | _0328_[26];
  assign _0830_[15] = _0328_[25] | _0328_[24];
  assign _0831_[1] = _0665_[2] | _0830_[3];
  assign _0831_[2] = _0653_[4] | _0830_[5];
  assign _0831_[3] = _0830_[6] | _0830_[7];
  assign _0831_[4] = _0653_[8] | _0830_[9];
  assign _0831_[5] = _0830_[10] | _0830_[11];
  assign _0831_[6] = _0653_[12] | _0830_[13];
  assign _0831_[7] = _0830_[14] | _0830_[15];
  assign _0832_[0] = _0831_[0] | _0831_[1];
  assign _0832_[1] = _0831_[2] | _0831_[3];
  assign _0832_[2] = _0831_[4] | _0831_[5];
  assign _0832_[3] = _0831_[6] | _0831_[7];
  assign _0833_[0] = _0832_[0] | _0832_[1];
  assign _0833_[1] = _0832_[2] | _0832_[3];
  assign _0834_ = _0833_[0] | _0833_[1];
  assign _0653_[0] = _0328_[7] | _0328_[6];
  assign _0831_[0] = _0653_[0] | _0665_[1];
  assign _0835_ = _0831_[0] | _0665_[2];
  assign _0671_[0] = _0398_[0] | _0398_[1];
  assign _0836_ = _0671_[0] | _0673_[1];
  assign _0837_ = _0836_ | _0673_[2];
  assign _0838_ = _0407_[0] | _0407_[1];
  assign _0839_[1] = _0331_[4] | _0331_[3];
  assign _0840_ = _0839_[0] | _0839_[1];
  assign _0841_ = _0840_ | _0331_[2];
  assign _0839_[0] = _0331_[6] | _0331_[5];
  assign _0843_ = _0842_[0] | _0839_[0];
  assign _0844_ = _0843_ | _0331_[4];
  assign _0842_[0] = _0331_[8] | _0331_[7];
  assign _0846_ = _0845_[0] | _0842_[0];
  assign _0847_ = _0846_ | _0331_[6];
  assign _0848_[0] = _0331_[12] | _0331_[11];
  assign _0845_[0] = _0331_[10] | _0331_[9];
  assign _0849_ = _0848_[0] | _0845_[0];
  assign _0850_ = _0849_ | _0331_[8];
  assign _0851_ = mipi_rx_d0_lp_n_i | mipi_rx_d0_lp_p_i;
  assign _0852_ = mipi_rx_clk_lp_n_i | mipi_rx_clk_lp_p_i;
  assign _1366_[1] = ~_0859_;
  assign _1366_[0] = ~_0860_;
  assign _1366_[3] = ~_0861_;
  assign _1366_[2] = ~_0862_;
  assign _1352_ = ~_0863_;
  assign _1350_ = ~_0539_;
  assign _0853_[1] = ~_0544_;
  assign _0853_[2] = ~_0543_;
  assign _0853_[3] = ~_0537_;
  assign _0853_[4] = ~_0538_;
  assign _0853_[5] = ~_0541_;
  assign _0853_[6] = ~_0542_;
  assign _1334_[3] = ~_0828_;
  assign _1334_[2] = ~_0864_;
  assign _1334_[1] = ~_0865_;
  assign _1334_[0] = ~_0866_;
  assign _1327_ = ~_0829_;
  assign _1328_ = ~_0868_;
  assign _0909_ = ~_0870_;
  assign _0910_ = ~_0871_;
  assign _0912_ = ~_0873_;
  assign _0913_ = ~_0835_;
  assign _0914_ = ~_0874_;
  assign _0915_ = ~_0875_;
  assign _0916_ = ~_0876_;
  assign _0917_ = ~_0877_;
  assign _0423_ = ~_0837_;
  assign _0422_ = ~_0878_;
  assign _0918_ = ~_0838_;
  assign _0919_ = ~_0879_;
  assign _0920_ = ~_0880_;
  assign _0921_ = ~_0881_;
  assign _1026_ = ~_0882_;
  assign _1031_ = ~_0883_;
  assign _1030_ = ~_0841_;
  assign _1032_ = ~_0844_;
  assign _1033_ = ~_0884_;
  assign _1021_ = ~_0885_;
  assign _1017_ = ~_0886_;
  assign _1013_ = ~_0887_;
  assign _1009_ = ~_0888_;
  assign _1027_ = ~_0889_;
  assign _1020_ = ~_0890_;
  assign _1016_ = ~_0891_;
  assign _1012_ = ~_0892_;
  assign _1008_ = ~_0893_;
  assign _1034_ = ~_0847_;
  assign _1035_ = ~_0894_;
  assign _1019_ = ~_0895_;
  assign _1015_ = ~_0896_;
  assign _1011_ = ~_0897_;
  assign _1007_ = ~_0898_;
  assign _1028_ = ~_0899_;
  assign _1018_ = ~_0900_;
  assign _1014_ = ~_0901_;
  assign _1010_ = ~_0902_;
  assign _1006_ = ~_0903_;
  assign _1036_ = ~_0850_;
  assign _1037_ = ~_0904_;
  assign _1029_ = ~_0905_;
  assign _1326_ = ~_0906_;
  assign mipi_rx_dat_hs_en_o = ~_0851_;
  assign mipi_rx_clk_hs_en_o = ~_0852_;
  assign _1324_ = _1327_ | _1328_;
  assign _0930_ = _0910_ | _0911_;
  assign _0931_ = _0930_ | _0912_;
  assign _0932_ = _0931_ | _0913_;
  assign _0933_ = _0932_ | _0914_;
  assign _0934_ = _0913_ | _0914_;
  assign _0922_ = _0909_ & _0402_;
  assign _0935_ = _0915_ | _0916_;
  assign _0936_ = _0935_ | _0917_;
  assign _0937_ = _0936_ | _0423_;
  assign _0938_ = _0937_ | _0422_;
  assign _0923_ = _0909_ & _0938_;
  assign _0924_ = _0403_ & _0918_;
  assign _0925_ = _0403_ & _0423_;
  assign _0926_ = _0403_ & _0422_;
  assign _0927_ = _0403_ & _0919_;
  assign _0928_ = _0403_ & _0920_;
  assign _0929_ = _0403_ & _0921_;
  assign _0221_ = _0918_ & _0409_;
  assign _0233_[1] = _0919_ & _0409_;
  assign _0233_[2] = _0920_ & _0409_;
  assign _0233_[3] = _0921_ & _0409_;
  assign _1022_ = _1030_ | _1031_;
  assign _1023_ = _1032_ | _1033_;
  assign _1024_ = _1034_ | _1035_;
  assign _1025_ = _1036_ | _1037_;
  assign _1059_ = _1058_ & _0326_[3];
  assign _1058_ = _1057_ & _0327_[3];
  assign _1057_ = _1056_ & _0324_[3];
  assign _1056_ = _1055_ & _0323_[3];
  assign _1055_ = _0321_[3] & _0322_[3];
  assign _1054_ = _1053_ & _0326_[2];
  assign _1053_ = _1052_ & _0327_[2];
  assign _1052_ = _1051_ & _0324_[2];
  assign _1051_ = _1050_ & _0323_[2];
  assign _1050_ = _0321_[2] & _0322_[2];
  assign _1049_ = _1048_ & _0326_[1];
  assign _1048_ = _1047_ & _0327_[1];
  assign _1047_ = _1046_ & _0324_[1];
  assign _1046_ = _1045_ & _0323_[1];
  assign _1045_ = _0321_[1] & _0322_[1];
  assign _1044_ = _1043_ & _0326_[0];
  assign _1043_ = _1042_ & _0327_[0];
  assign _1042_ = _1041_ & _0324_[0];
  assign _1041_ = _1040_ & _0323_[0];
  assign _1040_ = _0321_[0] & _0322_[0];
  assign _1039_ = _0335_ & _1038_;
  assign _1369_[8] = _0473_ ? 1'h0 : _0501_[28];
  assign _1369_[9] = _0473_ ? 1'h0 : _0501_[29];
  assign _1369_[10] = _0473_ ? _0501_[14] : _0501_[30];
  assign _1369_[11] = _0473_ ? _0501_[15] : _0501_[31];
  assign _1364_[16] = _0473_ ? _0501_[0] : _0501_[16];
  assign _1364_[17] = _0473_ ? _0501_[1] : _0501_[17];
  assign _1364_[18] = _0473_ ? _0501_[2] : _0501_[18];
  assign _1364_[19] = _0473_ ? _0501_[3] : _0501_[19];
  assign _1364_[20] = _0473_ ? _0501_[4] : _0501_[20];
  assign _1364_[21] = _0473_ ? _0501_[5] : _0501_[21];
  assign _1364_[22] = _0473_ ? _0501_[6] : _0501_[22];
  assign _1364_[23] = _0473_ ? _0501_[7] : _0501_[23];
  assign _1325_[40] = _0466_ ? _0501_[24] : _1367_[0];
  assign _1325_[41] = _0466_ ? _0501_[25] : _1367_[1];
  assign _1325_[42] = _0466_ ? _0501_[26] : _1367_[2];
  assign _1325_[43] = _0466_ ? _0501_[27] : _1367_[3];
  assign _1325_[44] = _0466_ ? _0501_[28] : _1367_[4];
  assign _1325_[45] = _0466_ ? _0501_[29] : _1367_[5];
  assign _1325_[46] = _0466_ ? _0501_[30] : _1367_[6];
  assign _1325_[47] = _0466_ ? _0501_[31] : _1367_[7];
  assign _1369_[12] = _0473_ ? 1'h0 : _0501_[12];
  assign _1369_[13] = _0473_ ? 1'h0 : _0501_[13];
  assign _1369_[14] = _0473_ ? _0501_[6] : _0501_[14];
  assign _1369_[15] = _0473_ ? _0501_[7] : _0501_[15];
  assign _1364_[24] = _0473_ ? _0471_[24] : _0501_[0];
  assign _1364_[25] = _0473_ ? _0471_[25] : _0501_[1];
  assign _1364_[26] = _0473_ ? _0471_[26] : _0501_[2];
  assign _1364_[27] = _0473_ ? _0471_[27] : _0501_[3];
  assign _1364_[28] = _0473_ ? _0471_[28] : _0501_[4];
  assign _1364_[29] = _0473_ ? _0471_[29] : _0501_[5];
  assign _1364_[30] = _0473_ ? _0471_[30] : _0501_[6];
  assign _1364_[31] = _0473_ ? _0471_[31] : _0501_[7];
  assign _1392_[12] = _0473_ ? 1'h0 : _0471_[16];
  assign _1392_[13] = _0473_ ? 1'h0 : _0471_[17];
  assign _1392_[14] = _0473_ ? _0501_[0] : _0471_[18];
  assign _1392_[15] = _0473_ ? _0501_[1] : _0471_[19];
  assign _1392_[8] = _0473_ ? 1'h0 : _0501_[0];
  assign _1392_[9] = _0473_ ? 1'h0 : _0501_[1];
  assign _1392_[10] = _0473_ ? _0501_[8] : _0501_[2];
  assign _1392_[11] = _0473_ ? _0501_[9] : _0501_[3];
  assign _1325_[4] = _0466_ ? _0501_[0] : _1390_[0];
  assign _1325_[5] = _0466_ ? _0501_[1] : _1390_[1];
  assign _1325_[6] = _0466_ ? _0501_[2] : _1390_[2];
  assign _1325_[7] = _0466_ ? _0501_[3] : _1390_[3];
  assign _1325_[8] = _0466_ ? _0501_[4] : _1390_[4];
  assign _1325_[9] = _0466_ ? _0501_[5] : _1390_[5];
  assign _1325_[10] = _0466_ ? _0501_[6] : _1390_[6];
  assign _1325_[11] = _0466_ ? _0501_[7] : _1390_[7];
  assign _1388_[16] = _0473_ ? _0471_[8] : _0471_[16];
  assign _1388_[17] = _0473_ ? _0471_[9] : _0471_[17];
  assign _1388_[18] = _0473_ ? _0471_[10] : _0471_[18];
  assign _1388_[19] = _0473_ ? _0471_[11] : _0471_[19];
  assign _1388_[20] = _0473_ ? _0471_[12] : _0471_[20];
  assign _1388_[21] = _0473_ ? _0471_[13] : _0471_[21];
  assign _1388_[22] = _0473_ ? _0471_[14] : _0471_[22];
  assign _1388_[23] = _0473_ ? _0471_[15] : _0471_[23];
  assign _1399_[0] = _0466_ ? _0502_ : _1398_[0];
  assign _1384_[12] = _0473_ ? 1'h0 : _0471_[20];
  assign _1384_[13] = _0473_ ? 1'h0 : _0471_[21];
  assign _1384_[14] = _0473_ ? _0501_[2] : _0471_[22];
  assign _1384_[15] = _0473_ ? _0501_[3] : _0471_[23];
  assign _1384_[8] = _0473_ ? 1'h0 : _0501_[4];
  assign _1384_[9] = _0473_ ? 1'h0 : _0501_[5];
  assign _1384_[10] = _0473_ ? _0501_[10] : _0501_[6];
  assign _1384_[11] = _0473_ ? _0501_[11] : _0501_[7];
  assign _1398_[0] = _0001_ ? _0502_ : 1'h0;
  assign _1325_[16] = _0466_ ? _0501_[8] : _1382_[0];
  assign _1325_[17] = _0466_ ? _0501_[9] : _1382_[1];
  assign _1325_[18] = _0466_ ? _0501_[10] : _1382_[2];
  assign _1325_[19] = _0466_ ? _0501_[11] : _1382_[3];
  assign _1325_[20] = _0466_ ? _0501_[12] : _1382_[4];
  assign _1325_[21] = _0466_ ? _0501_[13] : _1382_[5];
  assign _1325_[22] = _0466_ ? _0501_[14] : _1382_[6];
  assign _1325_[23] = _0466_ ? _0501_[15] : _1382_[7];
  assign _1397_[0] = _1415_[2] ? 1'h0 : _1412_[0];
  assign _1397_[1] = _1415_[2] ? 1'h0 : _1413_[1];
  assign _1397_[2] = _1415_[2] ? 1'h0 : _1413_[2];
  assign _1396_[2] = _0906_ ? _0867_ : 1'h1;
  assign _1377_[12] = _0473_ ? 1'h0 : _0501_[8];
  assign _1377_[13] = _0473_ ? 1'h0 : _0501_[9];
  assign _1377_[14] = _0473_ ? _0501_[4] : _0501_[10];
  assign _1377_[15] = _0473_ ? _0501_[5] : _0501_[11];
  assign _1373_[24] = _0473_ ? _0471_[16] : _0471_[24];
  assign _1373_[25] = _0473_ ? _0471_[17] : _0471_[25];
  assign _1373_[26] = _0473_ ? _0471_[18] : _0471_[26];
  assign _1373_[27] = _0473_ ? _0471_[19] : _0471_[27];
  assign _1373_[28] = _0473_ ? _0471_[20] : _0471_[28];
  assign _1373_[29] = _0473_ ? _0471_[21] : _0471_[29];
  assign _1373_[30] = _0473_ ? _0471_[22] : _0471_[30];
  assign _1373_[31] = _0473_ ? _0471_[23] : _0471_[31];
  assign _1373_[16] = _0473_ ? _0471_[24] : _0501_[8];
  assign _1373_[17] = _0473_ ? _0471_[25] : _0501_[9];
  assign _1373_[18] = _0473_ ? _0471_[26] : _0501_[10];
  assign _1373_[19] = _0473_ ? _0471_[27] : _0501_[11];
  assign _1373_[20] = _0473_ ? _0471_[28] : _0501_[12];
  assign _1373_[21] = _0473_ ? _0471_[29] : _0501_[13];
  assign _1373_[22] = _0473_ ? _0471_[30] : _0501_[14];
  assign _1373_[23] = _0473_ ? _0471_[31] : _0501_[15];
  assign _1325_[28] = _0466_ ? _0501_[16] : _1375_[0];
  assign _1325_[29] = _0466_ ? _0501_[17] : _1375_[1];
  assign _1325_[30] = _0466_ ? _0501_[18] : _1375_[2];
  assign _1325_[31] = _0466_ ? _0501_[19] : _1375_[3];
  assign _1325_[32] = _0466_ ? _0501_[20] : _1375_[4];
  assign _1325_[33] = _0466_ ? _0501_[21] : _1375_[5];
  assign _1325_[34] = _0466_ ? _0501_[22] : _1375_[6];
  assign _1325_[35] = _0466_ ? _0501_[23] : _1375_[7];
  assign _1377_[8] = _0473_ ? 1'h0 : _0501_[24];
  assign _1377_[9] = _0473_ ? 1'h0 : _0501_[25];
  assign _1377_[10] = _0473_ ? _0501_[12] : _0501_[26];
  assign _1377_[11] = _0473_ ? _0501_[13] : _0501_[27];
  assign _1363_ = ~_0853_[7];
  assign _0911_ = ~_0872_;
  assign _1362_[0] = _0537_ ? _1361_[0] : _0907_;
  assign _1362_[1] = _0537_ ? _1361_[1] : 1'h0;
  assign _1362_[2] = _0537_ ? _1361_[2] : 1'h0;
  assign _1362_[3] = _0537_ ? _1361_[3] : 1'h0;
  assign _1361_[0] = _0538_ ? _1359_[0] : _1360_[0];
  assign _1361_[1] = _0538_ ? _1359_[1] : _0908_;
  assign _1361_[2] = _0538_ ? _1359_[2] : 1'h0;
  assign _1361_[3] = _0538_ ? _1359_[3] : 1'h0;
  assign _1360_[0] = _0869_ ? _0907_ : 1'h0;
  assign _0908_ = ~_0869_;
  assign _0907_ = ~_0834_;
  assign _1359_[0] = _0541_ ? _1357_[0] : 1'h0;
  assign _1359_[1] = _0541_ ? _1357_[1] : _1358_[2];
  assign _1359_[2] = _0541_ ? _1357_[2] : _1358_[2];
  assign _1359_[3] = _0541_ ? _1357_[3] : 1'h0;
  assign _1358_[2] = _0870_ ? 1'h0 : _0931_;
  assign _1357_[0] = _0542_ ? _1356_[0] : 1'h0;
  assign _1357_[1] = _0542_ ? _1356_[2] : 1'h0;
  assign _1357_[2] = _0542_ ? _1356_[2] : 1'h1;
  assign _1357_[3] = _0542_ ? _1356_[3] : 1'h0;
  assign _1356_[0] = _0543_ ? _1355_[0] : 1'h0;
  assign _1356_[2] = _0543_ ? _1355_[2] : _0922_;
  assign _1356_[3] = _0543_ ? _1355_[3] : 1'h0;
  assign _1355_[0] = _0544_ ? _1354_[2] : 1'h0;
  assign _1355_[2] = _0544_ ? _1354_[2] : _0922_;
  assign _1355_[3] = _0544_ ? _1354_[3] : 1'h0;
  assign _1354_[2] = _0539_ ? _1351_[3] : _1353_[2];
  assign _1354_[3] = _0539_ ? _1351_[3] : _1353_[3];
  assign _1353_[2] = _0853_[7] ? 1'h0 : 1'hx;
  assign _1353_[3] = _0853_[7] ? 1'h1 : 1'hx;
  assign _1351_[3] = _0863_ ? 1'hx : 1'h0;
  assign _1349_ = _0934_ ? 1'h0 : _0931_;
  assign _1348_ = _0541_ ? _1346_ : _1347_;
  assign _1347_ = _0870_ ? 1'h0 : _0933_;
  assign _1346_ = _0542_ ? _1345_ : 1'h0;
  assign _1345_ = _0543_ ? _1344_ : _0923_;
  assign _1344_ = _0544_ ? 1'h0 : _0923_;
  assign _1343_[0] = _0541_ ? _1342_[0] : _0328_[23];
  assign _1343_[1] = _0541_ ? _1342_[1] : _0328_[22];
  assign _1343_[2] = _0541_ ? _1342_[2] : _0328_[21];
  assign _1343_[3] = _0541_ ? _1342_[3] : _0328_[20];
  assign _1343_[4] = _0541_ ? _1342_[4] : _0328_[19];
  assign _1343_[5] = _0541_ ? _1342_[5] : _0328_[18];
  assign _1343_[6] = _0541_ ? _1342_[6] : _0328_[17];
  assign _1343_[7] = _0541_ ? _1342_[7] : _0328_[16];
  assign _1342_[0] = _0542_ ? _1341_[0] : 1'hx;
  assign _1342_[1] = _0542_ ? _1341_[1] : 1'hx;
  assign _1342_[2] = _0542_ ? _1341_[2] : 1'hx;
  assign _1342_[3] = _0542_ ? _1341_[3] : 1'hx;
  assign _1342_[4] = _0542_ ? _1341_[4] : 1'hx;
  assign _1342_[5] = _0542_ ? _1341_[5] : 1'hx;
  assign _1342_[6] = _0542_ ? _1341_[6] : 1'hx;
  assign _1342_[7] = _0542_ ? _1341_[7] : 1'hx;
  assign _1341_[0] = _0543_ ? 1'hx : _0328_[7];
  assign _1341_[1] = _0543_ ? 1'hx : _0328_[6];
  assign _1341_[2] = _0543_ ? 1'hx : _0328_[5];
  assign _1341_[3] = _0543_ ? 1'hx : _0328_[4];
  assign _1341_[4] = _0543_ ? 1'hx : _0328_[3];
  assign _1341_[5] = _0543_ ? 1'hx : _0328_[2];
  assign _1341_[6] = _0543_ ? 1'hx : _0328_[1];
  assign _1341_[7] = _0543_ ? 1'hx : _0328_[0];
  assign _1340_[2] = _0541_ ? _1339_[2] : _0328_[13];
  assign _1340_[3] = _0541_ ? _1339_[3] : _0328_[12];
  assign _1340_[4] = _0541_ ? _1339_[4] : _0328_[11];
  assign _1340_[5] = _0541_ ? _1339_[5] : _0328_[10];
  assign _1340_[6] = _0541_ ? _1339_[6] : _0328_[9];
  assign _1340_[7] = _0541_ ? _1339_[7] : _0328_[8];
  assign _1339_[2] = _0542_ ? 1'hx : _0328_[5];
  assign _1339_[3] = _0542_ ? 1'hx : _0328_[4];
  assign _1339_[4] = _0542_ ? 1'hx : _0328_[3];
  assign _1339_[5] = _0542_ ? 1'hx : _0328_[2];
  assign _1339_[6] = _0542_ ? 1'hx : _0328_[1];
  assign _1339_[7] = _0542_ ? 1'hx : _0328_[0];
  assign _1338_ = _0925_ ? 1'h1 : _1337_;
  assign _1337_ = _0926_ ? 1'h0 : 1'hx;
  assign _1332_[0] = _1049_ ? 1'h1 : _1331_[0];
  assign _1331_[0] = _1054_ ? 1'h0 : _1330_[1];
  assign _1331_[1] = _1054_ ? 1'h1 : _1330_[1];
  assign _1330_[1] = _1059_ ? 1'h1 : 1'hx;
  assign _1371_[0] = _1419_ ? _1372_[0] : 1'hx;
  assign _1371_[1] = _1419_ ? _1372_[1] : 1'hx;
  assign _1371_[2] = _1419_ ? _1372_[2] : 1'hx;
  assign _1371_[3] = _1419_ ? _1372_[3] : 1'hx;
  assign _1335_[0] = _1418_ ? _1336_[0] : 1'hx;
  assign _1335_[1] = _1418_ ? _1336_[1] : 1'hx;
  assign _1335_[2] = _1418_ ? _1336_[2] : 1'hx;
  assign _1335_[3] = _1418_ ? _1336_[3] : 1'hx;
  assign _1335_[4] = _1418_ ? _1336_[4] : 1'hx;
  assign _1335_[5] = _1418_ ? _1336_[5] : 1'hx;
  assign _1335_[6] = _1418_ ? _1336_[6] : 1'hx;
  assign _1335_[7] = _1418_ ? _1336_[7] : 1'hx;
  assign _1335_[8] = _1418_ ? _1336_[8] : 1'hx;
  assign _1335_[9] = _1418_ ? _1336_[9] : 1'hx;
  assign _1335_[10] = _1418_ ? _1336_[10] : 1'hx;
  assign _1335_[11] = _1418_ ? _1336_[11] : 1'hx;
  assign _1335_[12] = _1418_ ? _1336_[12] : 1'hx;
  assign _1335_[13] = _1418_ ? _1336_[13] : 1'hx;
  assign _1335_[14] = _1418_ ? _1336_[14] : 1'hx;
  assign _1335_[15] = _1418_ ? _1336_[15] : 1'hx;
  assign _1335_[16] = _1418_ ? _1336_[16] : 1'hx;
  assign _1335_[17] = _1418_ ? _1336_[17] : 1'hx;
  assign _1335_[18] = _1418_ ? _1336_[18] : 1'hx;
  assign _1335_[19] = _1418_ ? _1336_[19] : 1'hx;
  assign _1335_[20] = _1418_ ? _1336_[20] : 1'hx;
  assign _1335_[21] = _1418_ ? _1336_[21] : 1'hx;
  assign _1335_[22] = _1418_ ? _1336_[22] : 1'hx;
  assign _1335_[23] = _1418_ ? _1336_[23] : 1'hx;
  assign _1335_[24] = _1418_ ? _1336_[24] : 1'hx;
  assign _1335_[25] = _1418_ ? _1336_[25] : 1'hx;
  assign _1335_[26] = _1418_ ? _1336_[26] : 1'hx;
  assign _1335_[27] = _1418_ ? _1336_[27] : 1'hx;
  assign _1335_[28] = _1418_ ? _1336_[28] : 1'hx;
  assign _1335_[29] = _1418_ ? _1336_[29] : 1'hx;
  assign _1335_[30] = _1418_ ? _1336_[30] : 1'hx;
  assign _1335_[31] = _1418_ ? _1336_[31] : 1'hx;
  assign _1394_[0] = _1419_ ? _1395_[0] : 1'hx;
  assign _1394_[1] = _1419_ ? _1395_[1] : 1'hx;
  assign _1394_[2] = _1419_ ? _1395_[2] : 1'hx;
  assign _1394_[3] = _1419_ ? _1395_[3] : 1'hx;
  assign _1375_[0] = _1419_ ? _1376_[0] : 1'hx;
  assign _1375_[1] = _1419_ ? _1376_[1] : 1'hx;
  assign _1375_[2] = _1419_ ? _1376_[2] : 1'hx;
  assign _1375_[3] = _1419_ ? _1376_[3] : 1'hx;
  assign _1375_[4] = _1419_ ? _1376_[4] : 1'hx;
  assign _1375_[5] = _1419_ ? _1376_[5] : 1'hx;
  assign _1375_[6] = _1419_ ? _1376_[6] : 1'hx;
  assign _1375_[7] = _1419_ ? _1376_[7] : 1'hx;
  assign _1379_[0] = _1419_ ? _1380_[0] : 1'hx;
  assign _1379_[1] = _1419_ ? _1380_[1] : 1'hx;
  assign _1379_[2] = _1419_ ? _1380_[2] : 1'hx;
  assign _1379_[3] = _1419_ ? _1380_[3] : 1'hx;
  assign _1382_[0] = _1419_ ? _1383_[0] : 1'hx;
  assign _1382_[1] = _1419_ ? _1383_[1] : 1'hx;
  assign _1382_[2] = _1419_ ? _1383_[2] : 1'hx;
  assign _1382_[3] = _1419_ ? _1383_[3] : 1'hx;
  assign _1382_[4] = _1419_ ? _1383_[4] : 1'hx;
  assign _1382_[5] = _1419_ ? _1383_[5] : 1'hx;
  assign _1382_[6] = _1419_ ? _1383_[6] : 1'hx;
  assign _1382_[7] = _1419_ ? _1383_[7] : 1'hx;
  assign _1386_[0] = _1419_ ? _1387_[0] : 1'hx;
  assign _1386_[1] = _1419_ ? _1387_[1] : 1'hx;
  assign _1386_[2] = _1419_ ? _1387_[2] : 1'hx;
  assign _1386_[3] = _1419_ ? _1387_[3] : 1'hx;
  assign _1390_[0] = _1419_ ? _1391_[0] : 1'hx;
  assign _1390_[1] = _1419_ ? _1391_[1] : 1'hx;
  assign _1390_[2] = _1419_ ? _1391_[2] : 1'hx;
  assign _1390_[3] = _1419_ ? _1391_[3] : 1'hx;
  assign _1390_[4] = _1419_ ? _1391_[4] : 1'hx;
  assign _1390_[5] = _1419_ ? _1391_[5] : 1'hx;
  assign _1390_[6] = _1419_ ? _1391_[6] : 1'hx;
  assign _1390_[7] = _1419_ ? _1391_[7] : 1'hx;
  assign _1367_[0] = _1419_ ? _1368_[0] : 1'hx;
  assign _1367_[1] = _1419_ ? _1368_[1] : 1'hx;
  assign _1367_[2] = _1419_ ? _1368_[2] : 1'hx;
  assign _1367_[3] = _1419_ ? _1368_[3] : 1'hx;
  assign _1367_[4] = _1419_ ? _1368_[4] : 1'hx;
  assign _1367_[5] = _1419_ ? _1368_[5] : 1'hx;
  assign _1367_[6] = _1419_ ? _1368_[6] : 1'hx;
  assign _1367_[7] = _1419_ ? _1368_[7] : 1'hx;
  assign _0535_ = ~_1329_;
  assign _0540_ = ~_0934_;
  assign _0536_ = ~_0329_;
  assign _1038_ = ~_0500_[0];
  assign _1407_ = ~_1404_[15];
  assign _1408_ = ~_1410_;
  assign _1414_[1] = ~_0469_[1];
  assign _1414_[2] = ~_0469_[2];
  assign _1403_[0] = ~_0406_[2];
  assign _1403_[1] = ~_0406_[3];
  assign _1403_[2] = ~_0406_[4];
  assign _1403_[3] = ~_0406_[5];
  assign _1403_[4] = ~_0406_[6];
  assign _1403_[5] = ~_0406_[7];
  assign _1403_[6] = ~_0406_[8];
  assign _1403_[7] = ~_0406_[9];
  assign _1403_[8] = ~_0406_[10];
  assign _1403_[9] = ~_0406_[11];
  assign _1403_[10] = ~_0406_[12];
  assign _1403_[11] = ~_0406_[13];
  assign _1403_[12] = ~_0406_[14];
  assign _1403_[13] = ~_0406_[15];
  assign _0939_ = _0328_[7] ^ _0328_[6];
  assign _0854_[0] = _0432_[0] ^ _0328_[31];
  assign _0854_[1] = _0432_[1] ^ _0328_[30];
  assign _0854_[2] = _0432_[2] ^ _0328_[29];
  assign _0854_[3] = _0432_[3] ^ _0328_[28];
  assign _0854_[4] = _0432_[4] ^ _0328_[27];
  assign _0854_[5] = _0432_[5] ^ _0328_[26];
  assign _0432_[5] = _1005_ ^ _0328_[16];
  assign _1005_ = _1004_ ^ _0328_[17];
  assign _1004_ = _1003_ ^ _0328_[18];
  assign _1003_ = _1002_ ^ _0328_[20];
  assign _1002_ = _1001_ ^ _0328_[21];
  assign _1001_ = _1000_ ^ _0328_[22];
  assign _1000_ = _0999_ ^ _0328_[23];
  assign _0999_ = _0998_ ^ _0328_[8];
  assign _0998_ = _0997_ ^ _0328_[9];
  assign _0997_ = _0996_ ^ _0328_[10];
  assign _0996_ = _0995_ ^ _0328_[11];
  assign _0995_ = _0328_[13] ^ _0328_[12];
  assign _0432_[4] = _0994_ ^ _0328_[16];
  assign _0994_ = _0993_ ^ _0328_[17];
  assign _0993_ = _0992_ ^ _0328_[19];
  assign _0992_ = _0991_ ^ _0328_[20];
  assign _0991_ = _0990_ ^ _0328_[21];
  assign _0990_ = _0989_ ^ _0328_[22];
  assign _0989_ = _0988_ ^ _0328_[23];
  assign _0988_ = _0987_ ^ _0328_[14];
  assign _0987_ = _0986_ ^ _0328_[15];
  assign _0986_ = _0985_ ^ _0328_[0];
  assign _0985_ = _0984_ ^ _0328_[1];
  assign _0984_ = _0328_[3] ^ _0328_[2];
  assign _0432_[3] = _0983_ ^ _0328_[16];
  assign _0983_ = _0982_ ^ _0328_[18];
  assign _0982_ = _0981_ ^ _0328_[19];
  assign _0981_ = _0980_ ^ _0328_[20];
  assign _0980_ = _0979_ ^ _0328_[8];
  assign _0979_ = _0978_ ^ _0328_[9];
  assign _0978_ = _0977_ ^ _0328_[10];
  assign _0977_ = _0976_ ^ _0328_[14];
  assign _0976_ = _0975_ ^ _0328_[15];
  assign _0975_ = _0974_ ^ _0328_[0];
  assign _0974_ = _0973_ ^ _0328_[4];
  assign _0973_ = _0328_[6] ^ _0328_[5];
  assign _0432_[2] = _0972_ ^ _0328_[17];
  assign _0972_ = _0971_ ^ _0328_[18];
  assign _0971_ = _0970_ ^ _0328_[19];
  assign _0970_ = _0969_ ^ _0328_[21];
  assign _0969_ = _0968_ ^ _0328_[8];
  assign _0968_ = _0967_ ^ _0328_[11];
  assign _0967_ = _0966_ ^ _0328_[12];
  assign _0966_ = _0965_ ^ _0328_[14];
  assign _0965_ = _0964_ ^ _0328_[1];
  assign _0964_ = _0963_ ^ _0328_[2];
  assign _0963_ = _0962_ ^ _0328_[4];
  assign _0962_ = _0328_[7] ^ _0328_[5];
  assign _0432_[1] = _0961_ ^ _0328_[16];
  assign _0961_ = _0960_ ^ _0328_[17];
  assign _0960_ = _0959_ ^ _0328_[18];
  assign _0959_ = _0958_ ^ _0328_[19];
  assign _0958_ = _0957_ ^ _0328_[22];
  assign _0957_ = _0956_ ^ _0328_[9];
  assign _0956_ = _0955_ ^ _0328_[11];
  assign _0955_ = _0954_ ^ _0328_[13];
  assign _0954_ = _0953_ ^ _0328_[15];
  assign _0953_ = _0952_ ^ _0328_[1];
  assign _0952_ = _0951_ ^ _0328_[3];
  assign _0951_ = _0939_ ^ _0328_[4];
  assign _0432_[0] = _0950_ ^ _0328_[16];
  assign _0950_ = _0949_ ^ _0328_[17];
  assign _0949_ = _0948_ ^ _0328_[18];
  assign _0948_ = _0947_ ^ _0328_[19];
  assign _0947_ = _0946_ ^ _0328_[23];
  assign _0946_ = _0945_ ^ _0328_[10];
  assign _0945_ = _0944_ ^ _0328_[12];
  assign _0944_ = _0943_ ^ _0328_[13];
  assign _0943_ = _0942_ ^ _0328_[0];
  assign _0942_ = _0941_ ^ _0328_[2];
  assign _0941_ = _0940_ ^ _0328_[3];
  assign _0940_ = _0939_ ^ _0328_[5];
  assign _1260_ = _1259_ ^ _0331_[11];
  assign _1261_ = _1260_ ^ _0331_[9];
  assign _1262_ = _1261_ ^ _0331_[8];
  assign _1263_ = _1262_ ^ _0331_[6];
  assign _1264_ = _1263_ ^ _0332_[11];
  assign _1265_ = _1264_ ^ _0332_[10];
  assign _1266_ = _1265_ ^ _0332_[8];
  assign _1267_ = _1266_ ^ _0333_[13];
  assign _1268_ = _1267_ ^ _0333_[9];
  assign _1269_ = _1268_ ^ _0333_[8];
  assign _1270_ = _1269_ ^ _0333_[7];
  assign _0374_[0] = _1270_ ^ _0333_[6];
  assign _1271_ = _1259_ ^ _0331_[10];
  assign _1272_ = _1271_ ^ _0331_[9];
  assign _1273_ = _1272_ ^ _0331_[7];
  assign _1274_ = _1273_ ^ _0332_[13];
  assign _1275_ = _1274_ ^ _0332_[11];
  assign _1276_ = _1275_ ^ _0332_[9];
  assign _1277_ = _1276_ ^ _0332_[7];
  assign _1278_ = _1277_ ^ _0333_[12];
  assign _1279_ = _1278_ ^ _0333_[9];
  assign _1280_ = _1279_ ^ _0333_[8];
  assign _1281_ = _1280_ ^ _0333_[7];
  assign _0374_[1] = _1281_ ^ _0333_[6];
  assign _1282_ = _0331_[13] ^ _0331_[11];
  assign _1283_ = _1282_ ^ _0331_[10];
  assign _1284_ = _1283_ ^ _0331_[8];
  assign _1285_ = _1284_ ^ _0331_[7];
  assign _1286_ = _1285_ ^ _0332_[12];
  assign _1287_ = _1286_ ^ _0332_[10];
  assign _1288_ = _1287_ ^ _0332_[9];
  assign _1289_ = _1288_ ^ _0332_[6];
  assign _1290_ = _1289_ ^ _0333_[11];
  assign _1291_ = _1290_ ^ _0333_[9];
  assign _1292_ = _1291_ ^ _0333_[8];
  assign _0374_[2] = _1292_ ^ _0333_[7];
  assign _1293_ = _0331_[12] ^ _0331_[11];
  assign _1294_ = _1293_ ^ _0331_[10];
  assign _1295_ = _1294_ ^ _0331_[6];
  assign _1296_ = _1295_ ^ _0332_[13];
  assign _1297_ = _1296_ ^ _0332_[12];
  assign _1298_ = _1297_ ^ _0332_[8];
  assign _1299_ = _1298_ ^ _0332_[7];
  assign _1300_ = _1299_ ^ _0332_[6];
  assign _1301_ = _1300_ ^ _0333_[10];
  assign _1302_ = _1301_ ^ _0333_[9];
  assign _1303_ = _1302_ ^ _0333_[8];
  assign _0374_[3] = _1303_ ^ _0333_[6];
  assign _1304_ = _1128_ ^ _0331_[6];
  assign _1305_ = _1304_ ^ _0332_[13];
  assign _1306_ = _1305_ ^ _0332_[12];
  assign _1307_ = _1306_ ^ _0333_[13];
  assign _1308_ = _1307_ ^ _0333_[12];
  assign _1309_ = _1308_ ^ _0333_[11];
  assign _1310_ = _1309_ ^ _0333_[10];
  assign _1311_ = _1310_ ^ _0333_[9];
  assign _1312_ = _1311_ ^ _0333_[7];
  assign _0374_[4] = _1312_ ^ _0333_[6];
  assign _1313_ = _0332_[11] ^ _0332_[10];
  assign _1314_ = _1313_ ^ _0332_[9];
  assign _1315_ = _1314_ ^ _0332_[8];
  assign _1316_ = _1315_ ^ _0332_[7];
  assign _1317_ = _1316_ ^ _0332_[6];
  assign _1318_ = _1317_ ^ _0333_[13];
  assign _1319_ = _1318_ ^ _0333_[12];
  assign _1320_ = _1319_ ^ _0333_[11];
  assign _1321_ = _1320_ ^ _0333_[10];
  assign _1322_ = _1321_ ^ _0333_[8];
  assign _1323_ = _1322_ ^ _0333_[7];
  assign _0374_[5] = _1323_ ^ _0333_[6];
  assign _1199_ = _1198_ ^ _0332_[9];
  assign _1200_ = _1199_ ^ _0332_[8];
  assign _1201_ = _1200_ ^ _0332_[6];
  assign _1202_ = _1201_ ^ _0333_[11];
  assign _1203_ = _1202_ ^ _0333_[7];
  assign _1204_ = _1203_ ^ _0333_[6];
  assign _1205_ = _1204_ ^ _0333_[5];
  assign _0373_[0] = _1205_ ^ _0333_[4];
  assign _1206_ = _1194_ ^ _0331_[8];
  assign _1207_ = _1206_ ^ _0331_[7];
  assign _1208_ = _1207_ ^ _0331_[5];
  assign _1209_ = _1208_ ^ _0332_[11];
  assign _1210_ = _1209_ ^ _0332_[9];
  assign _1211_ = _1210_ ^ _0332_[7];
  assign _1212_ = _1211_ ^ _0332_[5];
  assign _1213_ = _1212_ ^ _0333_[10];
  assign _1214_ = _1213_ ^ _0333_[7];
  assign _1215_ = _1214_ ^ _0333_[6];
  assign _1216_ = _1215_ ^ _0333_[5];
  assign _0373_[1] = _1216_ ^ _0333_[4];
  assign _1217_ = _0331_[11] ^ _0331_[9];
  assign _1218_ = _1217_ ^ _0331_[8];
  assign _1219_ = _1218_ ^ _0331_[6];
  assign _1220_ = _1219_ ^ _0331_[5];
  assign _1221_ = _1220_ ^ _0332_[10];
  assign _1222_ = _1221_ ^ _0332_[8];
  assign _1223_ = _1222_ ^ _0332_[7];
  assign _1224_ = _1223_ ^ _0332_[4];
  assign _1225_ = _1224_ ^ _0333_[9];
  assign _1226_ = _1225_ ^ _0333_[7];
  assign _1227_ = _1226_ ^ _0333_[6];
  assign _0373_[2] = _1227_ ^ _0333_[5];
  assign _1228_ = _0331_[10] ^ _0331_[9];
  assign _1229_ = _1228_ ^ _0331_[8];
  assign _1230_ = _1229_ ^ _0331_[4];
  assign _1231_ = _1230_ ^ _0332_[11];
  assign _1232_ = _1231_ ^ _0332_[10];
  assign _1233_ = _1232_ ^ _0332_[6];
  assign _1234_ = _1233_ ^ _0332_[5];
  assign _1235_ = _1234_ ^ _0332_[4];
  assign _1236_ = _1235_ ^ _0333_[8];
  assign _1237_ = _1236_ ^ _0333_[7];
  assign _1238_ = _1237_ ^ _0333_[6];
  assign _0373_[3] = _1238_ ^ _0333_[4];
  assign _1239_ = _1061_ ^ _0331_[4];
  assign _1240_ = _1239_ ^ _0332_[11];
  assign _1241_ = _1240_ ^ _0332_[10];
  assign _1242_ = _1241_ ^ _0333_[11];
  assign _1243_ = _1242_ ^ _0333_[10];
  assign _1244_ = _1243_ ^ _0333_[9];
  assign _1245_ = _1244_ ^ _0333_[8];
  assign _1246_ = _1245_ ^ _0333_[7];
  assign _1247_ = _1246_ ^ _0333_[5];
  assign _0373_[4] = _1247_ ^ _0333_[4];
  assign _1248_ = _0332_[9] ^ _0332_[8];
  assign _1249_ = _1248_ ^ _0332_[7];
  assign _1250_ = _1249_ ^ _0332_[6];
  assign _1251_ = _1250_ ^ _0332_[5];
  assign _1252_ = _1251_ ^ _0332_[4];
  assign _1253_ = _1252_ ^ _0333_[11];
  assign _1254_ = _1253_ ^ _0333_[10];
  assign _1255_ = _1254_ ^ _0333_[9];
  assign _1256_ = _1255_ ^ _0333_[8];
  assign _1257_ = _1256_ ^ _0333_[6];
  assign _1258_ = _1257_ ^ _0333_[5];
  assign _0373_[5] = _1258_ ^ _0333_[4];
  assign _1132_ = _1131_ ^ _0332_[7];
  assign _1133_ = _1132_ ^ _0332_[6];
  assign _1134_ = _1133_ ^ _0332_[4];
  assign _1135_ = _1134_ ^ _0333_[9];
  assign _1136_ = _1135_ ^ _0333_[5];
  assign _1137_ = _1136_ ^ _0333_[4];
  assign _1138_ = _1137_ ^ _0333_[3];
  assign _0372_[0] = _1138_ ^ _0333_[2];
  assign _1139_ = _1127_ ^ _0331_[6];
  assign _1140_ = _1139_ ^ _0331_[5];
  assign _1141_ = _1140_ ^ _0331_[3];
  assign _1142_ = _1141_ ^ _0332_[9];
  assign _1143_ = _1142_ ^ _0332_[7];
  assign _1144_ = _1143_ ^ _0332_[5];
  assign _1145_ = _1144_ ^ _0332_[3];
  assign _1146_ = _1145_ ^ _0333_[8];
  assign _1147_ = _1146_ ^ _0333_[5];
  assign _1148_ = _1147_ ^ _0333_[4];
  assign _1149_ = _1148_ ^ _0333_[3];
  assign _0372_[1] = _1149_ ^ _0333_[2];
  assign _1150_ = _0331_[9] ^ _0331_[7];
  assign _1151_ = _1150_ ^ _0331_[6];
  assign _1152_ = _1151_ ^ _0331_[4];
  assign _1153_ = _1152_ ^ _0331_[3];
  assign _1154_ = _1153_ ^ _0332_[8];
  assign _1155_ = _1154_ ^ _0332_[6];
  assign _1156_ = _1155_ ^ _0332_[5];
  assign _1157_ = _1156_ ^ _0332_[2];
  assign _1158_ = _1157_ ^ _0333_[7];
  assign _1159_ = _1158_ ^ _0333_[5];
  assign _1160_ = _1159_ ^ _0333_[4];
  assign _0372_[2] = _1160_ ^ _0333_[3];
  assign _1161_ = _0331_[8] ^ _0331_[7];
  assign _1162_ = _1161_ ^ _0331_[6];
  assign _1163_ = _1162_ ^ _0331_[2];
  assign _1164_ = _1163_ ^ _0332_[9];
  assign _1165_ = _1164_ ^ _0332_[8];
  assign _1166_ = _1165_ ^ _0332_[4];
  assign _1167_ = _1166_ ^ _0332_[3];
  assign _1168_ = _1167_ ^ _0332_[2];
  assign _1169_ = _1168_ ^ _0333_[6];
  assign _1170_ = _1169_ ^ _0333_[5];
  assign _1171_ = _1170_ ^ _0333_[4];
  assign _0372_[3] = _1171_ ^ _0333_[2];
  assign _1172_ = _0331_[5] ^ _0331_[4];
  assign _1173_ = _1172_ ^ _0331_[3];
  assign _1174_ = _1173_ ^ _0331_[2];
  assign _1175_ = _1174_ ^ _0332_[9];
  assign _1176_ = _1175_ ^ _0332_[8];
  assign _1177_ = _1176_ ^ _0333_[9];
  assign _1178_ = _1177_ ^ _0333_[8];
  assign _1179_ = _1178_ ^ _0333_[7];
  assign _1180_ = _1179_ ^ _0333_[6];
  assign _1181_ = _1180_ ^ _0333_[5];
  assign _1182_ = _1181_ ^ _0333_[3];
  assign _0372_[4] = _1182_ ^ _0333_[2];
  assign _1183_ = _0332_[7] ^ _0332_[6];
  assign _1184_ = _1183_ ^ _0332_[5];
  assign _1185_ = _1184_ ^ _0332_[4];
  assign _1186_ = _1185_ ^ _0332_[3];
  assign _1187_ = _1186_ ^ _0332_[2];
  assign _1188_ = _1187_ ^ _0333_[9];
  assign _1189_ = _1188_ ^ _0333_[8];
  assign _1190_ = _1189_ ^ _0333_[7];
  assign _1191_ = _1190_ ^ _0333_[6];
  assign _1192_ = _1191_ ^ _0333_[4];
  assign _1193_ = _1192_ ^ _0333_[3];
  assign _0372_[5] = _1193_ ^ _0333_[2];
  assign _1065_ = _1064_ ^ _0332_[5];
  assign _1066_ = _1065_ ^ _0332_[4];
  assign _1067_ = _1066_ ^ _0332_[2];
  assign _1068_ = _1067_ ^ _0333_[7];
  assign _1069_ = _1068_ ^ _0333_[3];
  assign _1070_ = _1069_ ^ _0333_[2];
  assign _1071_ = _1070_ ^ _0333_[1];
  assign _0371_[0] = _1071_ ^ _0333_[0];
  assign _1072_ = _1060_ ^ _0331_[4];
  assign _1073_ = _1072_ ^ _0331_[3];
  assign _1074_ = _1073_ ^ _0331_[1];
  assign _1075_ = _1074_ ^ _0332_[7];
  assign _1076_ = _1075_ ^ _0332_[5];
  assign _1077_ = _1076_ ^ _0332_[3];
  assign _1078_ = _1077_ ^ _0332_[1];
  assign _1079_ = _1078_ ^ _0333_[6];
  assign _1080_ = _1079_ ^ _0333_[3];
  assign _1081_ = _1080_ ^ _0333_[2];
  assign _1082_ = _1081_ ^ _0333_[1];
  assign _0371_[1] = _1082_ ^ _0333_[0];
  assign _1083_ = _0331_[7] ^ _0331_[5];
  assign _1084_ = _1083_ ^ _0331_[4];
  assign _1085_ = _1084_ ^ _0331_[2];
  assign _1086_ = _1085_ ^ _0331_[1];
  assign _1087_ = _1086_ ^ _0332_[6];
  assign _1088_ = _1087_ ^ _0332_[4];
  assign _1089_ = _1088_ ^ _0332_[3];
  assign _1090_ = _1089_ ^ _0332_[0];
  assign _1091_ = _1090_ ^ _0333_[5];
  assign _1092_ = _1091_ ^ _0333_[3];
  assign _1093_ = _1092_ ^ _0333_[2];
  assign _0371_[2] = _1093_ ^ _0333_[1];
  assign _1094_ = _0331_[6] ^ _0331_[5];
  assign _1095_ = _1094_ ^ _0331_[4];
  assign _1096_ = _1095_ ^ _0331_[0];
  assign _1097_ = _1096_ ^ _0332_[7];
  assign _1098_ = _1097_ ^ _0332_[6];
  assign _1099_ = _1098_ ^ _0332_[2];
  assign _1100_ = _1099_ ^ _0332_[1];
  assign _1101_ = _1100_ ^ _0332_[0];
  assign _1102_ = _1101_ ^ _0333_[4];
  assign _1103_ = _1102_ ^ _0333_[3];
  assign _1104_ = _1103_ ^ _0333_[2];
  assign _0371_[3] = _1104_ ^ _0333_[0];
  assign _1105_ = _0331_[3] ^ _0331_[2];
  assign _1106_ = _1105_ ^ _0331_[1];
  assign _1107_ = _1106_ ^ _0331_[0];
  assign _1108_ = _1107_ ^ _0332_[7];
  assign _1109_ = _1108_ ^ _0332_[6];
  assign _1110_ = _1109_ ^ _0333_[7];
  assign _1111_ = _1110_ ^ _0333_[6];
  assign _1112_ = _1111_ ^ _0333_[5];
  assign _1113_ = _1112_ ^ _0333_[4];
  assign _1114_ = _1113_ ^ _0333_[3];
  assign _1115_ = _1114_ ^ _0333_[1];
  assign _0371_[4] = _1115_ ^ _0333_[0];
  assign _1116_ = _0332_[5] ^ _0332_[4];
  assign _1117_ = _1116_ ^ _0332_[3];
  assign _1118_ = _1117_ ^ _0332_[2];
  assign _1119_ = _1118_ ^ _0332_[1];
  assign _1120_ = _1119_ ^ _0332_[0];
  assign _1121_ = _1120_ ^ _0333_[7];
  assign _1122_ = _1121_ ^ _0333_[6];
  assign _1123_ = _1122_ ^ _0333_[5];
  assign _1124_ = _1123_ ^ _0333_[4];
  assign _1125_ = _1124_ ^ _0333_[2];
  assign _1126_ = _1125_ ^ _0333_[1];
  assign _0371_[5] = _1126_ ^ _0333_[0];
  assign _1063_ = _1062_ ^ _0331_[2];
  assign _1062_ = _1061_ ^ _0331_[3];
  assign _1061_ = _1060_ ^ _0331_[5];
  assign _1060_ = _0331_[7] ^ _0331_[6];
  assign _0855_[0] = _0334_[7] ^ _0371_[0];
  assign _0855_[1] = _0334_[6] ^ _0371_[1];
  assign _0855_[2] = _0334_[5] ^ _0371_[2];
  assign _0855_[3] = _0334_[4] ^ _0371_[3];
  assign _0855_[4] = _0334_[3] ^ _0371_[4];
  assign _0855_[5] = _0334_[2] ^ _0371_[5];
  assign _1064_ = _1063_ ^ _0331_[0];
  assign _0856_[0] = _0334_[9] ^ _0372_[0];
  assign _0856_[1] = _0334_[8] ^ _0372_[1];
  assign _0856_[2] = _0334_[7] ^ _0372_[2];
  assign _0856_[3] = _0334_[6] ^ _0372_[3];
  assign _0856_[4] = _0334_[5] ^ _0372_[4];
  assign _0856_[5] = _0334_[4] ^ _0372_[5];
  assign _1127_ = _0331_[9] ^ _0331_[8];
  assign _1128_ = _1127_ ^ _0331_[7];
  assign _1129_ = _1128_ ^ _0331_[5];
  assign _1130_ = _1129_ ^ _0331_[4];
  assign _1131_ = _1130_ ^ _0331_[2];
  assign _0857_[0] = _0334_[11] ^ _0373_[0];
  assign _0857_[1] = _0334_[10] ^ _0373_[1];
  assign _0857_[2] = _0334_[9] ^ _0373_[2];
  assign _0857_[3] = _0334_[8] ^ _0373_[3];
  assign _0857_[4] = _0334_[7] ^ _0373_[4];
  assign _0857_[5] = _0334_[6] ^ _0373_[5];
  assign _1194_ = _0331_[11] ^ _0331_[10];
  assign _1195_ = _1194_ ^ _0331_[9];
  assign _1196_ = _1195_ ^ _0331_[7];
  assign _1197_ = _1196_ ^ _0331_[6];
  assign _1198_ = _1197_ ^ _0331_[4];
  assign _0858_[0] = _0334_[13] ^ _0374_[0];
  assign _0858_[1] = _0334_[12] ^ _0374_[1];
  assign _0858_[2] = _0334_[11] ^ _0374_[2];
  assign _0858_[3] = _0334_[10] ^ _0374_[3];
  assign _0858_[4] = _0334_[9] ^ _0374_[4];
  assign _0858_[5] = _0334_[8] ^ _0374_[5];
  assign _1259_ = _0331_[13] ^ _0331_[12];
  assign _1370_[2] = _0501_[30] & _1366_[0];
  assign _1370_[3] = _0501_[31] & _1366_[0];
  assign _1370_[6] = _0501_[22] & _1366_[1];
  assign _1370_[7] = _0501_[23] & _1366_[1];
  assign _1370_[8] = _1369_[8] & _1366_[2];
  assign _1370_[9] = _1369_[9] & _1366_[2];
  assign _1370_[10] = _1369_[10] & _1366_[2];
  assign _1370_[11] = _1369_[11] & _1366_[2];
  assign _1370_[12] = _1369_[12] & _1366_[3];
  assign _1370_[13] = _1369_[13] & _1366_[3];
  assign _1370_[14] = _1369_[14] & _1366_[3];
  assign _1370_[15] = _1369_[15] & _1366_[3];
  assign _1333_[32] = _0331_[36] & _1334_[1];
  assign _1333_[33] = _0331_[37] & _1334_[1];
  assign _1333_[34] = _0331_[38] & _1334_[1];
  assign _1333_[35] = _0331_[39] & _1334_[1];
  assign _1333_[36] = _0331_[40] & _1334_[1];
  assign _1333_[37] = _0331_[41] & _1334_[1];
  assign _1333_[38] = _0331_[42] & _1334_[1];
  assign _1333_[39] = _0331_[43] & _1334_[1];
  assign _1333_[40] = _0332_[36] & _1334_[1];
  assign _1333_[41] = _0332_[37] & _1334_[1];
  assign _1333_[42] = _0332_[38] & _1334_[1];
  assign _1333_[43] = _0332_[39] & _1334_[1];
  assign _1333_[44] = _0332_[40] & _1334_[1];
  assign _1333_[45] = _0332_[41] & _1334_[1];
  assign _1333_[46] = _0332_[42] & _1334_[1];
  assign _1333_[47] = _0332_[43] & _1334_[1];
  assign _1333_[48] = _0333_[36] & _1334_[1];
  assign _1333_[49] = _0333_[37] & _1334_[1];
  assign _1333_[50] = _0333_[38] & _1334_[1];
  assign _1333_[51] = _0333_[39] & _1334_[1];
  assign _1333_[52] = _0333_[40] & _1334_[1];
  assign _1333_[53] = _0333_[41] & _1334_[1];
  assign _1333_[54] = _0333_[42] & _1334_[1];
  assign _1333_[55] = _0333_[43] & _1334_[1];
  assign _1333_[56] = _0334_[36] & _1334_[1];
  assign _1333_[57] = _0334_[37] & _1334_[1];
  assign _1333_[58] = _0334_[38] & _1334_[1];
  assign _1333_[59] = _0334_[39] & _1334_[1];
  assign _1333_[60] = _0334_[40] & _1334_[1];
  assign _1333_[61] = _0334_[41] & _1334_[1];
  assign _1333_[62] = _0334_[42] & _1334_[1];
  assign _1333_[63] = _0334_[43] & _1334_[1];
  assign _1333_[0] = _0331_[38] & _1334_[0];
  assign _1333_[1] = _0331_[39] & _1334_[0];
  assign _1333_[2] = _0331_[40] & _1334_[0];
  assign _1333_[3] = _0331_[41] & _1334_[0];
  assign _1333_[4] = _0331_[42] & _1334_[0];
  assign _1333_[5] = _0331_[43] & _1334_[0];
  assign _1333_[6] = _0331_[44] & _1334_[0];
  assign _1333_[7] = _0331_[45] & _1334_[0];
  assign _1333_[8] = _0332_[38] & _1334_[0];
  assign _1333_[9] = _0332_[39] & _1334_[0];
  assign _1333_[10] = _0332_[40] & _1334_[0];
  assign _1333_[11] = _0332_[41] & _1334_[0];
  assign _1333_[12] = _0332_[42] & _1334_[0];
  assign _1333_[13] = _0332_[43] & _1334_[0];
  assign _1333_[14] = _0332_[44] & _1334_[0];
  assign _1333_[15] = _0332_[45] & _1334_[0];
  assign _1333_[16] = _0333_[38] & _1334_[0];
  assign _1333_[17] = _0333_[39] & _1334_[0];
  assign _1333_[18] = _0333_[40] & _1334_[0];
  assign _1333_[19] = _0333_[41] & _1334_[0];
  assign _1333_[20] = _0333_[42] & _1334_[0];
  assign _1333_[21] = _0333_[43] & _1334_[0];
  assign _1333_[22] = _0333_[44] & _1334_[0];
  assign _1333_[23] = _0333_[45] & _1334_[0];
  assign _1333_[24] = _0334_[38] & _1334_[0];
  assign _1333_[25] = _0334_[39] & _1334_[0];
  assign _1333_[26] = _0334_[40] & _1334_[0];
  assign _1333_[27] = _0334_[41] & _1334_[0];
  assign _1333_[28] = _0334_[42] & _1334_[0];
  assign _1333_[29] = _0334_[43] & _1334_[0];
  assign _1333_[30] = _0334_[44] & _1334_[0];
  assign _1333_[31] = _0334_[45] & _1334_[0];
  assign _1333_[96] = _0331_[32] & _1334_[3];
  assign _1333_[97] = _0331_[33] & _1334_[3];
  assign _1333_[98] = _0331_[34] & _1334_[3];
  assign _1333_[99] = _0331_[35] & _1334_[3];
  assign _1333_[100] = _0331_[36] & _1334_[3];
  assign _1333_[101] = _0331_[37] & _1334_[3];
  assign _1333_[102] = _0331_[38] & _1334_[3];
  assign _1333_[103] = _0331_[39] & _1334_[3];
  assign _1333_[104] = _0332_[32] & _1334_[3];
  assign _1333_[105] = _0332_[33] & _1334_[3];
  assign _1333_[106] = _0332_[34] & _1334_[3];
  assign _1333_[107] = _0332_[35] & _1334_[3];
  assign _1333_[108] = _0332_[36] & _1334_[3];
  assign _1333_[109] = _0332_[37] & _1334_[3];
  assign _1333_[110] = _0332_[38] & _1334_[3];
  assign _1333_[111] = _0332_[39] & _1334_[3];
  assign _1333_[112] = _0333_[32] & _1334_[3];
  assign _1333_[113] = _0333_[33] & _1334_[3];
  assign _1333_[114] = _0333_[34] & _1334_[3];
  assign _1333_[115] = _0333_[35] & _1334_[3];
  assign _1333_[116] = _0333_[36] & _1334_[3];
  assign _1333_[117] = _0333_[37] & _1334_[3];
  assign _1333_[118] = _0333_[38] & _1334_[3];
  assign _1333_[119] = _0333_[39] & _1334_[3];
  assign _1333_[120] = _0334_[32] & _1334_[3];
  assign _1333_[121] = _0334_[33] & _1334_[3];
  assign _1333_[122] = _0334_[34] & _1334_[3];
  assign _1333_[123] = _0334_[35] & _1334_[3];
  assign _1333_[124] = _0334_[36] & _1334_[3];
  assign _1333_[125] = _0334_[37] & _1334_[3];
  assign _1333_[126] = _0334_[38] & _1334_[3];
  assign _1333_[127] = _0334_[39] & _1334_[3];
  assign _1333_[64] = _0331_[34] & _1334_[2];
  assign _1333_[65] = _0331_[35] & _1334_[2];
  assign _1333_[66] = _0331_[36] & _1334_[2];
  assign _1333_[67] = _0331_[37] & _1334_[2];
  assign _1333_[68] = _0331_[38] & _1334_[2];
  assign _1333_[69] = _0331_[39] & _1334_[2];
  assign _1333_[70] = _0331_[40] & _1334_[2];
  assign _1333_[71] = _0331_[41] & _1334_[2];
  assign _1333_[72] = _0332_[34] & _1334_[2];
  assign _1333_[73] = _0332_[35] & _1334_[2];
  assign _1333_[74] = _0332_[36] & _1334_[2];
  assign _1333_[75] = _0332_[37] & _1334_[2];
  assign _1333_[76] = _0332_[38] & _1334_[2];
  assign _1333_[77] = _0332_[39] & _1334_[2];
  assign _1333_[78] = _0332_[40] & _1334_[2];
  assign _1333_[79] = _0332_[41] & _1334_[2];
  assign _1333_[80] = _0333_[34] & _1334_[2];
  assign _1333_[81] = _0333_[35] & _1334_[2];
  assign _1333_[82] = _0333_[36] & _1334_[2];
  assign _1333_[83] = _0333_[37] & _1334_[2];
  assign _1333_[84] = _0333_[38] & _1334_[2];
  assign _1333_[85] = _0333_[39] & _1334_[2];
  assign _1333_[86] = _0333_[40] & _1334_[2];
  assign _1333_[87] = _0333_[41] & _1334_[2];
  assign _1333_[88] = _0334_[34] & _1334_[2];
  assign _1333_[89] = _0334_[35] & _1334_[2];
  assign _1333_[90] = _0334_[36] & _1334_[2];
  assign _1333_[91] = _0334_[37] & _1334_[2];
  assign _1333_[92] = _0334_[38] & _1334_[2];
  assign _1333_[93] = _0334_[39] & _1334_[2];
  assign _1333_[94] = _0334_[40] & _1334_[2];
  assign _1333_[95] = _0334_[41] & _1334_[2];
  assign _1393_[12] = _1392_[12] & _1366_[3];
  assign _1393_[13] = _1392_[13] & _1366_[3];
  assign _1393_[14] = _1392_[14] & _1366_[3];
  assign _1393_[15] = _1392_[15] & _1366_[3];
  assign _1393_[6] = _0501_[16] & _1366_[1];
  assign _1393_[7] = _0501_[17] & _1366_[1];
  assign _1393_[8] = _1392_[8] & _1366_[2];
  assign _1393_[9] = _1392_[9] & _1366_[2];
  assign _1393_[10] = _1392_[10] & _1366_[2];
  assign _1393_[11] = _1392_[11] & _1366_[2];
  assign _1393_[2] = _0501_[24] & _1366_[0];
  assign _1393_[3] = _0501_[25] & _1366_[0];
  assign _1374_[24] = _1373_[24] & _1366_[3];
  assign _1374_[25] = _1373_[25] & _1366_[3];
  assign _1374_[26] = _1373_[26] & _1366_[3];
  assign _1374_[27] = _1373_[27] & _1366_[3];
  assign _1374_[28] = _1373_[28] & _1366_[3];
  assign _1374_[29] = _1373_[29] & _1366_[3];
  assign _1374_[30] = _1373_[30] & _1366_[3];
  assign _1374_[31] = _1373_[31] & _1366_[3];
  assign _1374_[16] = _1373_[16] & _1366_[2];
  assign _1374_[17] = _1373_[17] & _1366_[2];
  assign _1374_[18] = _1373_[18] & _1366_[2];
  assign _1374_[19] = _1373_[19] & _1366_[2];
  assign _1374_[20] = _1373_[20] & _1366_[2];
  assign _1374_[21] = _1373_[21] & _1366_[2];
  assign _1374_[22] = _1373_[22] & _1366_[2];
  assign _1374_[23] = _1373_[23] & _1366_[2];
  assign _1374_[8] = _0501_[0] & _1366_[1];
  assign _1374_[9] = _0501_[1] & _1366_[1];
  assign _1374_[10] = _0501_[2] & _1366_[1];
  assign _1374_[11] = _0501_[3] & _1366_[1];
  assign _1374_[12] = _0501_[4] & _1366_[1];
  assign _1374_[13] = _0501_[5] & _1366_[1];
  assign _1374_[14] = _0501_[6] & _1366_[1];
  assign _1374_[15] = _0501_[7] & _1366_[1];
  assign _1374_[0] = _0501_[8] & _1366_[0];
  assign _1374_[1] = _0501_[9] & _1366_[0];
  assign _1374_[2] = _0501_[10] & _1366_[0];
  assign _1374_[3] = _0501_[11] & _1366_[0];
  assign _1374_[4] = _0501_[12] & _1366_[0];
  assign _1374_[5] = _0501_[13] & _1366_[0];
  assign _1374_[6] = _0501_[14] & _1366_[0];
  assign _1374_[7] = _0501_[15] & _1366_[0];
  assign _1378_[12] = _1377_[12] & _1366_[3];
  assign _1378_[13] = _1377_[13] & _1366_[3];
  assign _1378_[14] = _1377_[14] & _1366_[3];
  assign _1378_[15] = _1377_[15] & _1366_[3];
  assign _1378_[8] = _1377_[8] & _1366_[2];
  assign _1378_[9] = _1377_[9] & _1366_[2];
  assign _1378_[10] = _1377_[10] & _1366_[2];
  assign _1378_[11] = _1377_[11] & _1366_[2];
  assign _1378_[6] = _0501_[20] & _1366_[1];
  assign _1378_[7] = _0501_[21] & _1366_[1];
  assign _1378_[2] = _0501_[28] & _1366_[0];
  assign _1378_[3] = _0501_[29] & _1366_[0];
  assign _1381_[24] = _0471_[8] & _1366_[3];
  assign _1381_[25] = _0471_[9] & _1366_[3];
  assign _1381_[26] = _0471_[10] & _1366_[3];
  assign _1381_[27] = _0471_[11] & _1366_[3];
  assign _1381_[28] = _0471_[12] & _1366_[3];
  assign _1381_[29] = _0471_[13] & _1366_[3];
  assign _1381_[30] = _0471_[14] & _1366_[3];
  assign _1381_[31] = _0471_[15] & _1366_[3];
  assign _1381_[16] = _1373_[24] & _1366_[2];
  assign _1381_[17] = _1373_[25] & _1366_[2];
  assign _1381_[18] = _1373_[26] & _1366_[2];
  assign _1381_[19] = _1373_[27] & _1366_[2];
  assign _1381_[20] = _1373_[28] & _1366_[2];
  assign _1381_[21] = _1373_[29] & _1366_[2];
  assign _1381_[22] = _1373_[30] & _1366_[2];
  assign _1381_[23] = _1373_[31] & _1366_[2];
  assign _1381_[8] = _0471_[24] & _1366_[1];
  assign _1381_[9] = _0471_[25] & _1366_[1];
  assign _1381_[10] = _0471_[26] & _1366_[1];
  assign _1381_[11] = _0471_[27] & _1366_[1];
  assign _1381_[12] = _0471_[28] & _1366_[1];
  assign _1381_[13] = _0471_[29] & _1366_[1];
  assign _1381_[14] = _0471_[30] & _1366_[1];
  assign _1381_[15] = _0471_[31] & _1366_[1];
  assign _1381_[0] = _0501_[0] & _1366_[0];
  assign _1381_[1] = _0501_[1] & _1366_[0];
  assign _1381_[2] = _0501_[2] & _1366_[0];
  assign _1381_[3] = _0501_[3] & _1366_[0];
  assign _1381_[4] = _0501_[4] & _1366_[0];
  assign _1381_[5] = _0501_[5] & _1366_[0];
  assign _1381_[6] = _0501_[6] & _1366_[0];
  assign _1381_[7] = _0501_[7] & _1366_[0];
  assign _1385_[12] = _1384_[12] & _1366_[3];
  assign _1385_[13] = _1384_[13] & _1366_[3];
  assign _1385_[14] = _1384_[14] & _1366_[3];
  assign _1385_[15] = _1384_[15] & _1366_[3];
  assign _1385_[8] = _1384_[8] & _1366_[2];
  assign _1385_[9] = _1384_[9] & _1366_[2];
  assign _1385_[10] = _1384_[10] & _1366_[2];
  assign _1385_[11] = _1384_[11] & _1366_[2];
  assign _1385_[6] = _0501_[18] & _1366_[1];
  assign _1385_[7] = _0501_[19] & _1366_[1];
  assign _1385_[2] = _0501_[26] & _1366_[0];
  assign _1385_[3] = _0501_[27] & _1366_[0];
  assign _1389_[0] = _0471_[24] & _1366_[0];
  assign _1389_[1] = _0471_[25] & _1366_[0];
  assign _1389_[2] = _0471_[26] & _1366_[0];
  assign _1389_[3] = _0471_[27] & _1366_[0];
  assign _1389_[4] = _0471_[28] & _1366_[0];
  assign _1389_[5] = _0471_[29] & _1366_[0];
  assign _1389_[6] = _0471_[30] & _1366_[0];
  assign _1389_[7] = _0471_[31] & _1366_[0];
  assign _1389_[8] = _0471_[16] & _1366_[1];
  assign _1389_[9] = _0471_[17] & _1366_[1];
  assign _1389_[10] = _0471_[18] & _1366_[1];
  assign _1389_[11] = _0471_[19] & _1366_[1];
  assign _1389_[12] = _0471_[20] & _1366_[1];
  assign _1389_[13] = _0471_[21] & _1366_[1];
  assign _1389_[14] = _0471_[22] & _1366_[1];
  assign _1389_[15] = _0471_[23] & _1366_[1];
  assign _1389_[16] = _1388_[16] & _1366_[2];
  assign _1389_[17] = _1388_[17] & _1366_[2];
  assign _1389_[18] = _1388_[18] & _1366_[2];
  assign _1389_[19] = _1388_[19] & _1366_[2];
  assign _1389_[20] = _1388_[20] & _1366_[2];
  assign _1389_[21] = _1388_[21] & _1366_[2];
  assign _1389_[22] = _1388_[22] & _1366_[2];
  assign _1389_[23] = _1388_[23] & _1366_[2];
  assign _1389_[24] = _0471_[0] & _1366_[3];
  assign _1389_[25] = _0471_[1] & _1366_[3];
  assign _1389_[26] = _0471_[2] & _1366_[3];
  assign _1389_[27] = _0471_[3] & _1366_[3];
  assign _1389_[28] = _0471_[4] & _1366_[3];
  assign _1389_[29] = _0471_[5] & _1366_[3];
  assign _1389_[30] = _0471_[6] & _1366_[3];
  assign _1389_[31] = _0471_[7] & _1366_[3];
  assign _1410_ = _1407_ | _1409_;
  assign _1365_[24] = _1364_[24] & _1366_[3];
  assign _1365_[25] = _1364_[25] & _1366_[3];
  assign _1365_[26] = _1364_[26] & _1366_[3];
  assign _1365_[27] = _1364_[27] & _1366_[3];
  assign _1365_[28] = _1364_[28] & _1366_[3];
  assign _1365_[29] = _1364_[29] & _1366_[3];
  assign _1365_[30] = _1364_[30] & _1366_[3];
  assign _1365_[31] = _1364_[31] & _1366_[3];
  assign _1365_[0] = _0501_[16] & _1366_[0];
  assign _1365_[1] = _0501_[17] & _1366_[0];
  assign _1365_[2] = _0501_[18] & _1366_[0];
  assign _1365_[3] = _0501_[19] & _1366_[0];
  assign _1365_[4] = _0501_[20] & _1366_[0];
  assign _1365_[5] = _0501_[21] & _1366_[0];
  assign _1365_[6] = _0501_[22] & _1366_[0];
  assign _1365_[7] = _0501_[23] & _1366_[0];
  assign _1365_[8] = _0501_[8] & _1366_[1];
  assign _1365_[9] = _0501_[9] & _1366_[1];
  assign _1365_[10] = _0501_[10] & _1366_[1];
  assign _1365_[11] = _0501_[11] & _1366_[1];
  assign _1365_[12] = _0501_[12] & _1366_[1];
  assign _1365_[13] = _0501_[13] & _1366_[1];
  assign _1365_[14] = _0501_[14] & _1366_[1];
  assign _1365_[15] = _0501_[15] & _1366_[1];
  assign _1365_[16] = _1364_[16] & _1366_[2];
  assign _1365_[17] = _1364_[17] & _1366_[2];
  assign _1365_[18] = _1364_[18] & _1366_[2];
  assign _1365_[19] = _1364_[19] & _1366_[2];
  assign _1365_[20] = _1364_[20] & _1366_[2];
  assign _1365_[21] = _1364_[21] & _1366_[2];
  assign _1365_[22] = _1364_[22] & _1366_[2];
  assign _1365_[23] = _1364_[23] & _1366_[2];
  assign _1402_[1] = _0416_[1] ^ _0416_[0];
  assign _1402_[2] = _0416_[2] ^ _1400_[1];
  assign _1402_[3] = _0416_[3] ^ _1400_[2];
  assign _1402_[4] = _0416_[4] ^ _1400_[3];
  assign _1402_[5] = _0416_[5] ^ _1400_[4];
  assign _1402_[6] = _0416_[6] ^ _1400_[5];
  assign _1402_[7] = _0416_[7] ^ _1400_[6];
  assign _1402_[8] = _0416_[8] ^ _1400_[7];
  assign _1402_[9] = _0416_[9] ^ _1400_[8];
  assign _1402_[10] = _0416_[10] ^ _1400_[9];
  assign _1402_[11] = _0416_[11] ^ _1400_[10];
  assign _1402_[12] = _0416_[12] ^ _1400_[11];
  assign _1402_[13] = _0416_[13] ^ _1400_[12];
  assign _1402_[14] = _0416_[14] ^ _1400_[13];
  assign _1402_[15] = _0416_[15] ^ _1400_[14];
  assign _1417_[1] = _0476_[1] & _1414_[1];
  assign _1417_[2] = _0476_[2] & _1414_[2];
  assign _1416_[1] = _0476_[1] ^ _1414_[1];
  assign _1416_[2] = _0476_[2] ^ _1414_[2];
  assign _1413_[1] = _0476_[1] ^ _0476_[0];
  assign _1413_[2] = _0476_[2] ^ _1411_[1];
  assign _1405_[0] = _0416_[0] ^ _1403_[0];
  assign _1405_[1] = _0416_[1] ^ _1403_[1];
  assign _1405_[2] = _0416_[2] ^ _1403_[2];
  assign _1405_[3] = _0416_[3] ^ _1403_[3];
  assign _1405_[4] = _0416_[4] ^ _1403_[4];
  assign _1405_[5] = _0416_[5] ^ _1403_[5];
  assign _1405_[6] = _0416_[6] ^ _1403_[6];
  assign _1405_[7] = _0416_[7] ^ _1403_[7];
  assign _1405_[8] = _0416_[8] ^ _1403_[8];
  assign _1405_[9] = _0416_[9] ^ _1403_[9];
  assign _1405_[10] = _0416_[10] ^ _1403_[10];
  assign _1405_[11] = _0416_[11] ^ _1403_[11];
  assign _1405_[12] = _0416_[12] ^ _1403_[12];
  assign _1405_[13] = _0416_[13] ^ _1403_[13];
  assign _1406_[0] = _0416_[0] & _1403_[0];
  assign _1406_[1] = _0416_[1] & _1403_[1];
  assign _1406_[2] = _0416_[2] & _1403_[2];
  assign _1406_[3] = _0416_[3] & _1403_[3];
  assign _1406_[4] = _0416_[4] & _1403_[4];
  assign _1406_[5] = _0416_[5] & _1403_[5];
  assign _1406_[6] = _0416_[6] & _1403_[6];
  assign _1406_[7] = _0416_[7] & _1403_[7];
  assign _1406_[8] = _0416_[8] & _1403_[8];
  assign _1406_[9] = _0416_[9] & _1403_[9];
  assign _1406_[10] = _0416_[10] & _1403_[10];
  assign _1406_[11] = _0416_[11] & _1403_[11];
  assign _1406_[12] = _0416_[12] & _1403_[12];
  assign _1406_[13] = _0416_[13] & _1403_[13];
  assign _1400_[1] = _0416_[1] & _0416_[0];
  assign _1400_[3] = _1420_ & _1400_[1];
  assign _1400_[7] = _1426_ & _1400_[3];
  assign _1420_ = _0416_[3] & _0416_[2];
  assign _1421_ = _0416_[5] & _0416_[4];
  assign _1422_ = _0416_[7] & _0416_[6];
  assign _1423_ = _0416_[9] & _0416_[8];
  assign _1424_ = _0416_[11] & _0416_[10];
  assign _1425_ = _0416_[13] & _0416_[12];
  assign _1426_ = _1422_ & _1421_;
  assign _1427_ = _1424_ & _1423_;
  assign _1400_[11] = _1427_ & _1400_[7];
  assign _1400_[5] = _1421_ & _1400_[3];
  assign _1400_[9] = _1423_ & _1400_[7];
  assign _1400_[13] = _1425_ & _1400_[11];
  assign _1400_[2] = _0416_[2] & _1400_[1];
  assign _1400_[4] = _0416_[4] & _1400_[3];
  assign _1400_[6] = _0416_[6] & _1400_[5];
  assign _1400_[8] = _0416_[8] & _1400_[7];
  assign _1400_[10] = _0416_[10] & _1400_[9];
  assign _1400_[12] = _0416_[12] & _1400_[11];
  assign _1400_[14] = _0416_[14] & _1400_[13];
  assign _1454_ = _1416_[2] & _1415_[1];
  assign _1415_[1] = _1417_[1] | _1416_[1];
  assign _1415_[2] = _1417_[2] | _1454_;
  assign _1404_[15] = _1453_ | _1442_;
  assign _1453_ = _1452_ | _1441_;
  assign _1404_[7] = _1450_ | _1440_;
  assign _1452_ = _1449_ | _1439_;
  assign _1451_ = _1447_ | _1438_;
  assign _1450_ = _1445_ | _1437_;
  assign _1404_[3] = _1443_ | _1436_;
  assign _1449_ = _0416_[15] | _1435_;
  assign _1448_ = _1406_[13] | _1434_;
  assign _1447_ = _1406_[11] | _1433_;
  assign _1446_ = _1406_[9] | _1432_;
  assign _1445_ = _1406_[7] | _1431_;
  assign _1444_ = _1406_[5] | _1430_;
  assign _1443_ = _1406_[3] | _1429_;
  assign _1404_[1] = _1406_[1] | _1428_;
  assign _1404_[0] = _1406_[0] | _1405_[0];
  assign _0817_[3] = _0816_[7] & _0816_[6];
  assign _0817_[1] = _0816_[3] & _0816_[2];
  assign _0816_[5] = _1405_[11] & _1405_[10];
  assign _1442_ = _0818_[1] & _1404_[7];
  assign _1441_ = _0817_[3] & _1451_;
  assign _1440_ = _0817_[1] & _1404_[3];
  assign _1439_ = _0816_[7] & _1448_;
  assign _1438_ = _0816_[5] & _1446_;
  assign _1437_ = _0816_[3] & _1444_;
  assign _1436_ = _0816_[1] & _1404_[1];
  assign _1435_ = _1405_[15] & _0416_[14];
  assign _1434_ = _1405_[13] & _1406_[12];
  assign _1433_ = _1405_[11] & _1406_[10];
  assign _1432_ = _1405_[9] & _1406_[8];
  assign _1431_ = _1405_[7] & _1406_[6];
  assign _1430_ = _1405_[5] & _1406_[4];
  assign _1429_ = _1405_[3] & _1406_[2];
  assign _1428_ = _1405_[1] & _1404_[0];
  assign _1411_[1] = _0476_[1] & _0476_[0];
  assign _0633_[1] = _0026_;
  assign _0636_[1] = _0635_[1];
  assign _0637_[0] = _0636_[0];
  assign _0638_[1] = _0637_[1];
  assign _0639_[2] = _0629_[3];
  assign _0644_[3] = _0004_;
  assign _0647_[1] = _0629_[3];
  assign _0652_[1] = _0629_[3];
  assign _0659_[1] = _0653_[1];
  assign _0661_[2:1] = { _0659_[2], _0653_[1] };
  assign { _0663_[2], _0663_[0] } = { _0659_[2], _0653_[0] };
  assign _0669_[2:1] = _0667_[2:1];
  assign _0671_[2] = _0667_[2];
  assign _0697_[11:4] = { _0679_[10:4], _0679_[0] };
  assign _0701_[11:4] = { _0683_[10:4], _0683_[0] };
  assign _0705_[11:4] = { _0687_[10:4], _0687_[0] };
  assign _0709_[11:4] = { _0691_[10:4], _0691_[0] };
  assign { _0714_[11:4], _0714_[1] } = { _0679_[9:4], _0679_[0], _0697_[0], _0679_[3] };
  assign _0715_[5:3] = _0680_[4:2];
  assign { _0718_[11:4], _0718_[1] } = { _0683_[9:4], _0683_[0], _0701_[0], _0683_[3] };
  assign _0719_[5:3] = _0684_[4:2];
  assign { _0722_[11:4], _0722_[1] } = { _0687_[9:4], _0687_[0], _0705_[0], _0687_[3] };
  assign _0723_[5:3] = _0688_[4:2];
  assign { _0726_[11:4], _0726_[1] } = { _0691_[9:4], _0691_[0], _0709_[0], _0691_[3] };
  assign _0727_[5:3] = _0692_[4:2];
  assign { _0732_[11:4], _0732_[1] } = { _0679_[8:4], _0679_[0], _0697_[0], _0714_[0], _0697_[3] };
  assign _0733_[5:3] = _0698_[4:2];
  assign { _0736_[11:4], _0736_[1] } = { _0683_[8:4], _0683_[0], _0701_[0], _0718_[0], _0701_[3] };
  assign _0737_[5:3] = _0702_[4:2];
  assign { _0740_[11:4], _0740_[1] } = { _0687_[8:4], _0687_[0], _0705_[0], _0722_[0], _0705_[3] };
  assign _0741_[5:3] = _0706_[4:2];
  assign { _0744_[11:4], _0744_[1] } = { _0691_[8:4], _0691_[0], _0709_[0], _0726_[0], _0709_[3] };
  assign _0745_[5:3] = _0710_[4:2];
  assign { _0830_[12], _0830_[8], _0830_[4], _0830_[2:0] } = { _0653_[12], _0653_[8], _0653_[4], _0665_[2:1], _0653_[0] };
  assign _0842_[1] = _0839_[0];
  assign _0845_[1] = _0842_[0];
  assign _0848_[1] = _0845_[0];
  assign _0853_[0] = _0539_;
  assign _1330_[0] = _1330_[1];
  assign _1351_[2:0] = { _1351_[3], _1351_[3], _1351_[3] };
  assign _1353_[1:0] = { _1353_[2], _1353_[2] };
  assign _1354_[1:0] = { _1354_[2], _1354_[2] };
  assign _1355_[1] = _1355_[2];
  assign _1356_[1] = _1356_[2];
  assign { _1358_[3], _1358_[1:0] } = { 1'h0, _1358_[2], 1'h0 };
  assign _1360_[3:1] = { 2'h0, _0908_ };
  assign _1364_[15:0] = { _0501_[15:8], _0501_[23:16] };
  assign _1369_[7:0] = { _0501_[23:22], 2'h0, _0501_[31:30], 2'h0 };
  assign { _1370_[5:4], _1370_[1:0] } = 4'h0;
  assign _1373_[15:0] = { _0501_[7:0], _0501_[15:8] };
  assign _1377_[7:0] = { _0501_[21:20], 2'h0, _0501_[29:28], 2'h0 };
  assign { _1378_[5:4], _1378_[1:0] } = 4'h0;
  assign _1384_[7:0] = { _0501_[19:18], 2'h0, _0501_[27:26], 2'h0 };
  assign { _1385_[5:4], _1385_[1:0] } = 4'h0;
  assign { _1388_[31:24], _1388_[15:0] } = { _0471_[7:0], _0471_[23:16], _0471_[31:24] };
  assign _1392_[7:0] = { _0501_[17:16], 2'h0, _0501_[25:24], 2'h0 };
  assign { _1393_[5:4], _1393_[1:0] } = 4'h0;
  assign _1396_[1:0] = { _0906_, 1'h0 };
  assign _1398_[3:1] = 3'h0;
  assign _1399_[3:1] = 3'h0;
  assign { _1400_[31:16], _1400_[0] } = { 16'h0000, _0416_[0] };
  assign _1401_[31:1] = { 16'h0000, _0416_[15:1] };
  assign { _1402_[31:16], _1402_[0] } = { 15'h0000, _1400_[15], _1401_[0] };
  assign _1403_[15:14] = 2'h3;
  assign _1406_[15:14] = _0416_[15:14];
  assign { _1411_[31:3], _1411_[0] } = { 29'h00000000, _0476_[0] };
  assign _1412_[31:1] = { 29'h00000000, _0476_[2:1] };
  assign { _1413_[31:3], _1413_[0] } = { 28'h0000000, _1411_[2], _1412_[0] };
  assign _1414_[0] = 1'h1;
  assign _1415_[0] = 1'h1;
  assign _1416_[0] = _1412_[0];
  assign _1417_[0] = _0476_[0];
  assign CLK = mipi_rx_byteclk_i;
  assign DAT = _0470_;
  assign DE = _0472_[0];
  assign HSYNC = _0468_;
  assign INDEX = _0498_;
  assign VSYNC = _0467_;
  assign _0027_ = mipi_rx_byteclk_i;
  assign _0028_ = _0470_;
  assign _0029_ = _0472_[0];
  assign _0030_ = _0468_;
  assign _0031_ = _0498_;
  assign _0032_ = _0467_;
  assign _0033_ = 1'h0;
  assign _0034_ = mipi_rx_byteclk_i;
  assign _0035_ = 16'hea60;
  assign _0036_ = 16'h7530;
  assign _0037_ = mipi_rx_byteclk_i;
  assign _0038_ = { 3'h0, _0412_ };
  assign _0039_ = _0504_;
  assign _0040_ = mipi_rx_byteclk_i;
  assign _0041_ = { 3'h0, _0412_ };
  assign _0042_ = _0504_;
  assign _0043_ = 1'h0;
  assign _0044_ = mipi_rx_byteclk_i;
  assign _0045_ = 1'h0;
  assign _0046_[7:6] = 2'h0;
  assign _0047_ = _0046_[5:0];
  assign _0048_ = 1'h0;
  assign _0049_ = mipi_rx_byteclk_i;
  assign _0050_ = mipi_rx_byteclk_i;
  assign _0051_ = _0412_;
  assign _0052_ = mipi_rx_byteclk_i;
  assign _0053_ = _0504_;
  assign _0054_[7:6] = 2'h0;
  assign _0055_ = _0054_[5:0];
  assign _0056_[7:6] = 2'h0;
  assign _0057_ = _0056_[5:0];
  assign _0058_[7:6] = 2'h0;
  assign _0059_ = _0058_[5:0];
  assign _0060_[7:6] = 2'h0;
  assign _0061_ = _0060_[5:0];
  assign _0062_ = mipi_rx_byteclk_i;
  assign _0063_ = 1'h0;
  assign _0064_ = _0329_;
  assign _0065_[2:0] = { _0065_[3], _0065_[3], _0065_[3] };
  assign _0066_ = { _0065_[3], _0065_[3], _0065_[3], _0065_[3] };
  assign _0067_ = { _0065_[3], _0065_[3], _0065_[3], _0065_[3] };
  assign _0068_ = _0329_;
  assign _0069_ = _0335_;
  assign _0070_ = { 24'h000000, _0506_[7:0] };
  assign _0071_ = _0508_;
  assign _0072_ = 24'h0000b8;
  assign _0073_ = 24'h0000b8;
  assign _0074_ = 24'h0000b8;
  assign _0075_ = 24'h0000b8;
  assign _0076_ = 24'h0000b8;
  assign _0077_ = 24'h0000b8;
  assign _0078_ = 24'h0000b8;
  assign _0079_ = 24'h0000b8;
  assign _0080_ = 24'h0000b8;
  assign _0081_ = 24'h0000b8;
  assign _0082_ = 24'h0000b8;
  assign _0083_ = 24'h0000b8;
  assign _0084_ = _0054_[5:0];
  assign _0085_ = _0056_[5:0];
  assign _0086_ = _0058_[5:0];
  assign _0087_ = _0060_[5:0];
  assign _0088_ = mipi_rx_byteclk_i;
  assign _0089_ = 1'h0;
  assign _0090_[0] = 1'h0;
  assign _0091_ = 1'h0;
  assign _0092_ = mipi_rx_byteclk_i;
  assign _0093_ = _0412_;
  assign _0094_ = 1'h0;
  assign _0095_ = 32'd0;
  assign _0096_ = 1'h0;
  assign _0097_ = 32'd0;
  assign _0098_[23:0] = 24'h000000;
  assign _0099_ = 32'd0;
  assign _0100_ = 1'h0;
  assign _0101_ = 64'h0000000000000000;
  assign _0102_ = _0412_;
  assign _0103_ = 32'd0;
  assign _0104_ = 32'd0;
  assign _0105_ = _0506_[7:0];
  assign _0106_ = _0508_;
  assign { _0107_[224], _0107_[31:0] } = 33'h000000000;
  assign _0108_ = 4'he;
  assign _0109_ = _0329_;
  assign _0110_ = { 2'h0, _0046_[5:0] };
  assign _0111_ = 8'h00;
  assign _0112_ = 8'h1d;
  assign _0113_ = 8'hb8;
  assign _0114_ = { 24'h000000, _0506_[7:0] };
  assign _0115_ = _0508_;
  assign _0116_ = { 24'h000000, _0506_[7:0] };
  assign _0117_ = 4'h8;
  assign _0118_ = 4'h0;
  assign _0119_ = 4'h1;
  assign _0120_ = 4'h7;
  assign _0121_ = 4'h2;
  assign _0122_ = 4'h6;
  assign _0123_ = 4'h5;
  assign _0124_ = 4'h4;
  assign _0125_ = 4'h3;
  assign _0126_ = 4'h9;
  assign _0127_ = mipi_rx_byteclk_i;
  assign _0128_ = 1'h0;
  assign _0129_[7:6] = 2'h0;
  assign _0130_ = _0129_[5:0];
  assign _0131_ = 1'h0;
  assign _0132_ = mipi_rx_byteclk_i;
  assign _0133_ = mipi_rx_byteclk_i;
  assign _0134_ = _0412_;
  assign _0135_ = mipi_rx_byteclk_i;
  assign _0136_ = _0504_;
  assign _0137_[7:6] = 2'h0;
  assign _0138_ = _0137_[5:0];
  assign _0139_[7:6] = 2'h0;
  assign _0140_ = _0139_[5:0];
  assign _0141_[7:6] = 2'h0;
  assign _0142_ = _0141_[5:0];
  assign _0143_[7:6] = 2'h0;
  assign _0144_ = _0143_[5:0];
  assign _0145_ = mipi_rx_byteclk_i;
  assign _0146_ = 1'h0;
  assign _0147_ = _0329_;
  assign _0148_ = { _0065_[3], _0065_[3], _0065_[3], _0065_[3] };
  assign _0149_ = { _0065_[3], _0065_[3], _0065_[3], _0065_[3] };
  assign _0150_ = _0329_;
  assign _0151_ = _0335_;
  assign _0152_ = { 16'h0000, _0506_[15:0] };
  assign _0153_ = _0508_;
  assign _0154_ = 24'h0000b8;
  assign _0155_ = 24'h0000b8;
  assign _0156_ = 24'h0000b8;
  assign _0157_ = 24'h0000b8;
  assign _0158_ = 24'h0000b8;
  assign _0159_ = 24'h0000b8;
  assign _0160_ = 24'h0000b8;
  assign _0161_ = 24'h0000b8;
  assign _0162_ = _0137_[5:0];
  assign _0163_ = _0139_[5:0];
  assign _0164_ = _0141_[5:0];
  assign _0165_ = _0143_[5:0];
  assign _0166_ = mipi_rx_byteclk_i;
  assign _0167_ = 1'h0;
  assign _0168_[0] = 1'h0;
  assign _0169_ = 1'h0;
  assign _0170_ = mipi_rx_byteclk_i;
  assign _0171_ = _0412_;
  assign _0172_ = 1'h0;
  assign _0173_ = 32'd0;
  assign _0174_ = 1'h0;
  assign _0175_ = 32'd0;
  assign _0176_[23:0] = 24'h000000;
  assign _0177_ = 32'd0;
  assign _0178_ = 1'h0;
  assign _0179_[15] = 1'h0;
  assign _0180_ = 64'h0000000000000000;
  assign _0181_ = _0412_;
  assign _0182_ = 32'd0;
  assign _0183_ = 32'd0;
  assign _0184_ = _0506_[15:0];
  assign _0185_ = _0508_;
  assign { _0186_[224], _0186_[31:0] } = 33'h000000000;
  assign _0187_ = 4'hc;
  assign _0188_ = _0329_;
  assign _0189_ = { 2'h0, _0129_[5:0] };
  assign _0190_[15] = 1'h0;
  assign _0191_ = 16'h0000;
  assign _0192_ = 16'h1d1d;
  assign _0193_ = 16'hb8b8;
  assign _0194_ = { 16'h0000, _0506_[15:0] };
  assign _0195_ = _0508_;
  assign _0196_ = { 16'h0000, _0506_[15:0] };
  assign _0197_ = 4'h8;
  assign _0198_ = 4'h0;
  assign _0199_ = 4'h1;
  assign _0200_ = 4'h7;
  assign _0201_ = 4'h2;
  assign _0202_ = 4'h6;
  assign _0203_ = 4'h5;
  assign _0204_ = 4'h4;
  assign _0205_ = 4'h3;
  assign _0206_ = 4'h9;
  assign _0207_ = mipi_rx_byteclk_i;
  assign _0208_ = 1'h0;
  assign _0209_ = _0396_;
  assign _0210_ = 1'h0;
  assign _0211_ = 1'h0;
  assign _0212_ = { _0328_[16], _0328_[17], _0328_[18], _0328_[19], _0328_[20], _0328_[21], _0328_[22], _0328_[23], _0328_[8], _0328_[9], _0328_[10], _0328_[11], _0328_[12], _0328_[13], _0328_[14], _0328_[15], _0328_[0], _0328_[1], _0328_[2], _0328_[3], _0328_[4], _0328_[5], _0328_[6], _0328_[7] };
  assign _0213_ = { _0328_[16], _0328_[17], _0328_[18], _0328_[19], _0328_[20], _0328_[21], _0328_[22], _0328_[23], _0328_[8], _0328_[9], _0328_[10], _0328_[11], _0328_[12], _0328_[13], _0328_[14], _0328_[15], _0328_[0], _0328_[1], _0328_[2], _0328_[3], _0328_[4], _0328_[5], _0328_[6], _0328_[7] };
  assign _0214_ = { 2'h0, _0432_[5:0] };
  assign _0215_ = _0432_[5:0];
  assign _0216_ = 1'h0;
  assign _0217_ = _0397_[0];
  assign _0218_ = mipi_rx_byteclk_i;
  assign _0219_ = _0396_;
  assign _0220_ = mipi_rx_byteclk_i;
  assign _0222_ = 1'h0;
  assign _0223_ = _0412_;
  assign _0224_ = 1'h0;
  assign _0225_ = 10'h000;
  assign _0226_ = 1'h0;
  assign _0227_ = _0221_;
  assign _0229_ = _0397_;
  assign _0230_ = _0397_[0];
  assign _0231_ = mipi_rx_byteclk_i;
  assign _0232_ = _0504_;
  assign _0233_[0] = _0221_;
  assign _0234_ = _0221_;
  assign _0235_ = _0331_[15:8];
  assign _0236_ = { _0331_[8], _0331_[9], _0331_[10], _0331_[11], _0331_[12], _0331_[13], _0331_[14], _0331_[15] };
  assign _0237_ = _0332_[15:8];
  assign _0238_ = { _0332_[8], _0332_[9], _0332_[10], _0332_[11], _0332_[12], _0332_[13], _0332_[14], _0332_[15] };
  assign _0239_ = _0333_[15:8];
  assign _0240_ = { _0333_[8], _0333_[9], _0333_[10], _0333_[11], _0333_[12], _0333_[13], _0333_[14], _0333_[15] };
  assign _0241_ = _0334_[15:8];
  assign _0242_ = { _0334_[8], _0334_[9], _0334_[10], _0334_[11], _0334_[12], _0334_[13], _0334_[14], _0334_[15] };
  assign _0243_ = _0331_[17:10];
  assign _0244_ = { _0331_[10], _0331_[11], _0331_[12], _0331_[13], _0331_[14], _0331_[15], _0331_[16], _0331_[17] };
  assign _0245_ = _0332_[17:10];
  assign _0246_ = { _0332_[10], _0332_[11], _0332_[12], _0332_[13], _0332_[14], _0332_[15], _0332_[16], _0332_[17] };
  assign _0247_ = _0333_[17:10];
  assign _0248_ = { _0333_[10], _0333_[11], _0333_[12], _0333_[13], _0333_[14], _0333_[15], _0333_[16], _0333_[17] };
  assign _0249_ = _0334_[17:10];
  assign _0250_ = { _0334_[10], _0334_[11], _0334_[12], _0334_[13], _0334_[14], _0334_[15], _0334_[16], _0334_[17] };
  assign _0251_ = _0331_[19:12];
  assign _0252_ = { _0331_[12], _0331_[13], _0331_[14], _0331_[15], _0331_[16], _0331_[17], _0331_[18], _0331_[19] };
  assign _0253_ = _0332_[19:12];
  assign _0254_ = { _0332_[12], _0332_[13], _0332_[14], _0332_[15], _0332_[16], _0332_[17], _0332_[18], _0332_[19] };
  assign _0255_ = _0333_[19:12];
  assign _0256_ = { _0333_[12], _0333_[13], _0333_[14], _0333_[15], _0333_[16], _0333_[17], _0333_[18], _0333_[19] };
  assign _0257_ = _0334_[19:12];
  assign _0258_ = { _0334_[12], _0334_[13], _0334_[14], _0334_[15], _0334_[16], _0334_[17], _0334_[18], _0334_[19] };
  assign _0259_ = _0331_[21:14];
  assign _0260_ = { _0331_[14], _0331_[15], _0331_[16], _0331_[17], _0331_[18], _0331_[19], _0331_[20], _0331_[21] };
  assign _0261_ = _0332_[21:14];
  assign _0262_ = { _0332_[14], _0332_[15], _0332_[16], _0332_[17], _0332_[18], _0332_[19], _0332_[20], _0332_[21] };
  assign _0263_ = _0333_[21:14];
  assign _0264_ = { _0333_[14], _0333_[15], _0333_[16], _0333_[17], _0333_[18], _0333_[19], _0333_[20], _0333_[21] };
  assign _0265_ = _0334_[21:14];
  assign _0266_ = { _0334_[14], _0334_[15], _0334_[16], _0334_[17], _0334_[18], _0334_[19], _0334_[20], _0334_[21] };
  assign _0267_ = { _0333_[0], _0333_[1], _0333_[2], _0333_[3], _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0332_[0], _0332_[1], _0332_[2], _0332_[3], _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0331_[0], _0331_[1], _0331_[2], _0331_[3], _0331_[4], _0331_[5], _0331_[6], _0331_[7] };
  assign _0268_ = { _0333_[0], _0333_[1], _0333_[2], _0333_[3], _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0332_[0], _0332_[1], _0332_[2], _0332_[3], _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0331_[0], _0331_[1], _0331_[2], _0331_[3], _0331_[4], _0331_[5], _0331_[6], _0331_[7] };
  assign _0269_ = { 2'h0, _0371_ };
  assign _0270_ = _0371_;
  assign _0271_ = _0331_[7:0];
  assign _0272_ = { _0331_[0], _0331_[1], _0331_[2], _0331_[3], _0331_[4], _0331_[5], _0331_[6], _0331_[7] };
  assign _0273_ = _0332_[7:0];
  assign _0274_ = { _0332_[0], _0332_[1], _0332_[2], _0332_[3], _0332_[4], _0332_[5], _0332_[6], _0332_[7] };
  assign _0275_ = _0333_[7:0];
  assign _0276_ = { _0333_[0], _0333_[1], _0333_[2], _0333_[3], _0333_[4], _0333_[5], _0333_[6], _0333_[7] };
  assign _0277_ = _0334_[7:0];
  assign _0278_ = { _0334_[0], _0334_[1], _0334_[2], _0334_[3], _0334_[4], _0334_[5], _0334_[6], _0334_[7] };
  assign _0279_ = { _0333_[2], _0333_[3], _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0332_[2], _0332_[3], _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0331_[2], _0331_[3], _0331_[4], _0331_[5], _0331_[6], _0331_[7], _0331_[8], _0331_[9] };
  assign _0280_ = { _0333_[2], _0333_[3], _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0332_[2], _0332_[3], _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0331_[2], _0331_[3], _0331_[4], _0331_[5], _0331_[6], _0331_[7], _0331_[8], _0331_[9] };
  assign _0281_ = { 2'h0, _0372_ };
  assign _0282_ = _0372_;
  assign _0283_ = _0331_[9:2];
  assign _0284_ = { _0331_[2], _0331_[3], _0331_[4], _0331_[5], _0331_[6], _0331_[7], _0331_[8], _0331_[9] };
  assign _0285_ = _0332_[9:2];
  assign _0286_ = { _0332_[2], _0332_[3], _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0332_[8], _0332_[9] };
  assign _0287_ = _0333_[9:2];
  assign _0288_ = { _0333_[2], _0333_[3], _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0333_[8], _0333_[9] };
  assign _0289_ = _0334_[9:2];
  assign _0290_ = { _0334_[2], _0334_[3], _0334_[4], _0334_[5], _0334_[6], _0334_[7], _0334_[8], _0334_[9] };
  assign _0291_ = { _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0333_[10], _0333_[11], _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0332_[10], _0332_[11], _0331_[4], _0331_[5], _0331_[6], _0331_[7], _0331_[8], _0331_[9], _0331_[10], _0331_[11] };
  assign _0292_ = { _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0333_[10], _0333_[11], _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0332_[10], _0332_[11], _0331_[4], _0331_[5], _0331_[6], _0331_[7], _0331_[8], _0331_[9], _0331_[10], _0331_[11] };
  assign _0293_ = { 2'h0, _0373_ };
  assign _0294_ = _0373_;
  assign _0295_ = _0331_[11:4];
  assign _0296_ = { _0331_[4], _0331_[5], _0331_[6], _0331_[7], _0331_[8], _0331_[9], _0331_[10], _0331_[11] };
  assign _0297_ = _0332_[11:4];
  assign _0298_ = { _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0332_[10], _0332_[11] };
  assign _0299_ = _0333_[11:4];
  assign _0300_ = { _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0333_[10], _0333_[11] };
  assign _0301_ = _0334_[11:4];
  assign _0302_ = { _0334_[4], _0334_[5], _0334_[6], _0334_[7], _0334_[8], _0334_[9], _0334_[10], _0334_[11] };
  assign _0303_ = { _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0333_[10], _0333_[11], _0333_[12], _0333_[13], _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0332_[10], _0332_[11], _0332_[12], _0332_[13], _0331_[6], _0331_[7], _0331_[8], _0331_[9], _0331_[10], _0331_[11], _0331_[12], _0331_[13] };
  assign _0304_ = { _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0333_[10], _0333_[11], _0333_[12], _0333_[13], _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0332_[10], _0332_[11], _0332_[12], _0332_[13], _0331_[6], _0331_[7], _0331_[8], _0331_[9], _0331_[10], _0331_[11], _0331_[12], _0331_[13] };
  assign _0305_ = { 2'h0, _0374_ };
  assign _0306_ = _0374_;
  assign _0307_ = _0331_[13:6];
  assign _0308_ = { _0331_[6], _0331_[7], _0331_[8], _0331_[9], _0331_[10], _0331_[11], _0331_[12], _0331_[13] };
  assign _0309_ = _0332_[13:6];
  assign _0310_ = { _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0332_[10], _0332_[11], _0332_[12], _0332_[13] };
  assign _0311_ = _0333_[13:6];
  assign _0312_ = { _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0333_[10], _0333_[11], _0333_[12], _0333_[13] };
  assign _0313_ = _0334_[13:6];
  assign _0314_ = { _0334_[6], _0334_[7], _0334_[8], _0334_[9], _0334_[10], _0334_[11], _0334_[12], _0334_[13] };
  assign _0315_ = _0325_;
  assign _0316_ = mipi_rx_byteclk_i;
  assign _0317_ = 1'h0;
  assign _0318_ = _0221_;
  assign _0319_ = { _0328_[24], _0328_[25], _0328_[26], _0328_[27], _0328_[28], _0328_[29], _0328_[30], _0328_[31], _0328_[16], _0328_[17], _0328_[18], _0328_[19], _0328_[20], _0328_[21], _0328_[22], _0328_[23], _0328_[8], _0328_[9], _0328_[10], _0328_[11], _0328_[12], _0328_[13], _0328_[14], _0328_[15], _0328_[0], _0328_[1], _0328_[2], _0328_[3], _0328_[4], _0328_[5], _0328_[6], _0328_[7] };
  assign _0320_ = _0329_;
  assign _0330_ = _0500_[0];
  assign _0336_ = _0506_;
  assign _0337_ = _0508_;
  assign _0338_ = _0396_;
  assign _0339_ = _0328_[7:0];
  assign _0340_ = { _0328_[0], _0328_[1], _0328_[2], _0328_[3], _0328_[4], _0328_[5], _0328_[6], _0328_[7] };
  assign _0341_ = _0328_[15:8];
  assign _0342_ = { _0328_[8], _0328_[9], _0328_[10], _0328_[11], _0328_[12], _0328_[13], _0328_[14], _0328_[15] };
  assign _0343_ = _0328_[23:16];
  assign _0344_ = { _0328_[16], _0328_[17], _0328_[18], _0328_[19], _0328_[20], _0328_[21], _0328_[22], _0328_[23] };
  assign _0345_ = _0328_[31:24];
  assign _0346_ = { _0328_[24], _0328_[25], _0328_[26], _0328_[27], _0328_[28], _0328_[29], _0328_[30], _0328_[31] };
  assign _0347_ = { _0331_[31:16], _0331_[8], _0331_[9], _0331_[10], _0331_[11], _0331_[12], _0331_[13], _0331_[14], _0331_[15] };
  assign _0348_ = { _0331_[33:18], _0331_[10], _0331_[11], _0331_[12], _0331_[13], _0331_[14], _0331_[15], _0331_[16], _0331_[17] };
  assign _0349_ = { _0331_[35:20], _0331_[12], _0331_[13], _0331_[14], _0331_[15], _0331_[16], _0331_[17], _0331_[18], _0331_[19] };
  assign _0350_ = { _0331_[37:22], _0331_[14], _0331_[15], _0331_[16], _0331_[17], _0331_[18], _0331_[19], _0331_[20], _0331_[21] };
  assign _0351_ = { _0332_[31:16], _0332_[8], _0332_[9], _0332_[10], _0332_[11], _0332_[12], _0332_[13], _0332_[14], _0332_[15] };
  assign _0352_ = { _0332_[33:18], _0332_[10], _0332_[11], _0332_[12], _0332_[13], _0332_[14], _0332_[15], _0332_[16], _0332_[17] };
  assign _0353_ = { _0332_[35:20], _0332_[12], _0332_[13], _0332_[14], _0332_[15], _0332_[16], _0332_[17], _0332_[18], _0332_[19] };
  assign _0354_ = { _0332_[37:22], _0332_[14], _0332_[15], _0332_[16], _0332_[17], _0332_[18], _0332_[19], _0332_[20], _0332_[21] };
  assign _0355_ = { _0333_[31:16], _0333_[8], _0333_[9], _0333_[10], _0333_[11], _0333_[12], _0333_[13], _0333_[14], _0333_[15] };
  assign _0356_ = { _0333_[33:18], _0333_[10], _0333_[11], _0333_[12], _0333_[13], _0333_[14], _0333_[15], _0333_[16], _0333_[17] };
  assign _0357_ = { _0333_[35:20], _0333_[12], _0333_[13], _0333_[14], _0333_[15], _0333_[16], _0333_[17], _0333_[18], _0333_[19] };
  assign _0358_ = { _0333_[37:22], _0333_[14], _0333_[15], _0333_[16], _0333_[17], _0333_[18], _0333_[19], _0333_[20], _0333_[21] };
  assign _0359_ = { _0334_[31:16], _0334_[8], _0334_[9], _0334_[10], _0334_[11], _0334_[12], _0334_[13], _0334_[14], _0334_[15] };
  assign _0360_ = { _0334_[33:18], _0334_[10], _0334_[11], _0334_[12], _0334_[13], _0334_[14], _0334_[15], _0334_[16], _0334_[17] };
  assign _0361_ = { _0334_[35:20], _0334_[12], _0334_[13], _0334_[14], _0334_[15], _0334_[16], _0334_[17], _0334_[18], _0334_[19] };
  assign _0362_ = { _0334_[37:22], _0334_[14], _0334_[15], _0334_[16], _0334_[17], _0334_[18], _0334_[19], _0334_[20], _0334_[21] };
  assign _0363_ = { _0334_[0], _0334_[1], _0334_[2], _0334_[3], _0334_[4], _0334_[5], _0334_[6], _0334_[7], _0333_[0], _0333_[1], _0333_[2], _0333_[3], _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0332_[0], _0332_[1], _0332_[2], _0332_[3], _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0331_[0], _0331_[1], _0331_[2], _0331_[3], _0331_[4], _0331_[5], _0331_[6], _0331_[7] };
  assign _0364_ = { _0334_[2], _0334_[3], _0334_[4], _0334_[5], _0334_[6], _0334_[7], _0334_[8], _0334_[9], _0333_[2], _0333_[3], _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0332_[2], _0332_[3], _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0331_[2], _0331_[3], _0331_[4], _0331_[5], _0331_[6], _0331_[7], _0331_[8], _0331_[9] };
  assign _0365_ = { _0334_[4], _0334_[5], _0334_[6], _0334_[7], _0334_[8], _0334_[9], _0334_[10], _0334_[11], _0333_[4], _0333_[5], _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0333_[10], _0333_[11], _0332_[4], _0332_[5], _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0332_[10], _0332_[11], _0331_[4], _0331_[5], _0331_[6], _0331_[7], _0331_[8], _0331_[9], _0331_[10], _0331_[11] };
  assign _0366_ = { _0334_[6], _0334_[7], _0334_[8], _0334_[9], _0334_[10], _0334_[11], _0334_[12], _0334_[13], _0333_[6], _0333_[7], _0333_[8], _0333_[9], _0333_[10], _0333_[11], _0333_[12], _0333_[13], _0332_[6], _0332_[7], _0332_[8], _0332_[9], _0332_[10], _0332_[11], _0332_[12], _0332_[13], _0331_[6], _0331_[7], _0331_[8], _0331_[9], _0331_[10], _0331_[11], _0331_[12], _0331_[13] };
  assign _0367_ = { _0334_[2], _0334_[3], _0334_[4], _0334_[5], _0334_[6], _0334_[7] };
  assign _0368_ = { _0334_[4], _0334_[5], _0334_[6], _0334_[7], _0334_[8], _0334_[9] };
  assign _0369_ = { _0334_[6], _0334_[7], _0334_[8], _0334_[9], _0334_[10], _0334_[11] };
  assign _0370_ = { _0334_[8], _0334_[9], _0334_[10], _0334_[11], _0334_[12], _0334_[13] };
  assign _0375_ = _0331_[79:64];
  assign _0376_ = _0331_[63:48];
  assign _0377_ = _0332_[63:48];
  assign _0378_ = _0331_[47:32];
  assign _0379_ = _0332_[47:32];
  assign _0380_ = _0333_[47:32];
  assign _0381_ = _0334_[47:32];
  assign _0382_ = mipi_rx_byteclk_i;
  assign _0383_ = 1'h0;
  assign _0384_ = 1'h0;
  assign _0385_[1:0] = 2'h0;
  assign _0386_ = 1'h0;
  assign _0387_ = 1'h0;
  assign _0388_ = mipi_rx_byteclk_i;
  assign _0389_ = _0411_;
  assign _0390_ = _0412_;
  assign _0391_ = _0396_;
  assign _0392_ = 1'h0;
  assign _0393_ = 32'd0;
  assign _0394_ = 1'h0;
  assign _0395_ = 32'd0;
  assign _0399_[23:0] = 24'h000000;
  assign _0400_ = 32'd0;
  assign _0401_ = 1'h0;
  assign _0405_[15:14] = 2'h0;
  assign _0408_ = 64'h0000000000000000;
  assign _0413_ = 32'd0;
  assign _0414_ = 32'd0;
  assign _0415_ = _0404_;
  assign _0418_ = _0506_;
  assign _0419_ = _0508_;
  assign _0420_[7:4] = { _0396_, _0422_, _0423_, _0403_ };
  assign { _0421_[224], _0421_[31:0] } = 33'h000000000;
  assign _0424_ = 1'h0;
  assign _0425_ = 4'h0;
  assign _0426_ = _0325_;
  assign _0427_ = { _0328_[24], _0328_[25], _0328_[26], _0328_[27], _0328_[28], _0328_[29], _0328_[30], _0328_[31], _0328_[16], _0328_[17], _0328_[18], _0328_[19], _0328_[20], _0328_[21], _0328_[22], _0328_[23], _0328_[8], _0328_[9], _0328_[10], _0328_[11], _0328_[12], _0328_[13], _0328_[14], _0328_[15], _0328_[0], _0328_[1], _0328_[2], _0328_[3], _0328_[4], _0328_[5], _0328_[6], _0328_[7] };
  assign _0428_ = { _0328_[24], _0328_[25], _0328_[26], _0328_[27], _0328_[28], _0328_[29], _0328_[30], _0328_[31], _0328_[16], _0328_[17], _0328_[18], _0328_[19], _0328_[20], _0328_[21], _0328_[22], _0328_[23], _0328_[8], _0328_[9], _0328_[10], _0328_[11], _0328_[12], _0328_[13], _0328_[14], _0328_[15], _0328_[0], _0328_[1], _0328_[2], _0328_[3], _0328_[4], _0328_[5], _0328_[6], _0328_[7] };
  assign _0429_ = _0329_;
  assign _0430_ = _0396_;
  assign _0431_ = { _0328_[16], _0328_[17], _0328_[18], _0328_[19], _0328_[20], _0328_[21], _0328_[22], _0328_[23], _0328_[8], _0328_[9], _0328_[10], _0328_[11], _0328_[12], _0328_[13], _0328_[14], _0328_[15], _0328_[0], _0328_[1], _0328_[2], _0328_[3], _0328_[4], _0328_[5], _0328_[6], _0328_[7] };
  assign _0432_[7:6] = 2'h0;
  assign _0433_ = { _0328_[24], _0328_[25], _0328_[26], _0328_[27], _0328_[28], _0328_[29], _0328_[30], _0328_[31] };
  assign _0434_ = { 2'h0, _0406_[15:2] };
  assign _0435_ = 1'h0;
  assign _0436_ = 32'd0;
  assign _0437_ = 32'd488447261;
  assign _0438_ = 32'd3099113656;
  assign _0439_ = _0506_;
  assign _0440_ = _0508_;
  assign _0441_ = _0506_;
  assign _0442_ = 4'h8;
  assign _0443_ = 4'h0;
  assign _0444_ = 4'h1;
  assign _0445_ = 4'h7;
  assign _0446_ = 4'h2;
  assign _0447_ = 4'h6;
  assign _0448_ = 4'h5;
  assign _0449_ = 4'h4;
  assign _0450_ = 4'h3;
  assign _0451_ = 4'h9;
  assign _0452_ = 1'h0;
  assign _0453_ = 2'h0;
  assign _0454_ = 1'h0;
  assign _0455_ = 8'he4;
  assign _0456_ = 1'h0;
  assign _0457_ = mipi_rx_byteclk_i;
  assign _0458_ = _0499_;
  assign _0459_ = _0467_;
  assign _0460_ = _0500_;
  assign _0461_ = _0468_;
  assign _0462_ = _0501_;
  assign _0463_ = _0470_;
  assign _0464_ = { 3'h0, _0502_ };
  assign _0465_ = _0472_;
  assign _0469_[0] = 1'h0;
  assign _0471_[47:32] = 16'h0000;
  assign _0475_ = _0503_[1:0];
  assign _0477_ = _0504_;
  assign _0478_ = mipi_rx_byteclk_i;
  assign _0479_ = mipi_rx_clk_hs_en_o;
  assign _0480_ = mipi_rx_clk_lp_n_i;
  assign _0481_ = mipi_rx_clk_lp_p_i;
  assign _0482_ = mipi_rx_clk_hs_en_o;
  assign _0483_ = mipi_rx_d0_i;
  assign _0484_ = mipi_rx_d0_lp_n_i;
  assign _0485_ = mipi_rx_d0_lp_p_i;
  assign _0486_ = mipi_rx_d1_i;
  assign _0487_ = mipi_rx_d2_i;
  assign _0488_ = mipi_rx_d3_i;
  assign _0489_ = mipi_rx_dat_hs_en_o;
  assign _0490_ = mipi_rx_dat_hs_en_o;
  assign _0491_ = 1'h1;
  assign _0492_ = 1'h0;
  assign _0493_ = 1'h0;
  assign _0494_ = mipi_rx_byteclk_i;
  assign _0495_ = 1'h0;
  assign _0496_ = 17'h1869f;
  assign _0497_ = 17'h0c350;
  assign _0509_ = sclk_i;
  assign _0510_ = srst_i;
  assign { _0511_[39:36], _0511_[31:24], _0511_[7:0] } = { _0396_, _0422_, _0423_, _0403_, _0500_, _0499_, _0233_[3:1], _0221_, _0397_ };
  assign _0512_ = _0467_;
  assign _0513_ = _0468_;
  assign _0514_ = _0470_;
  assign _0515_ = _0472_;
  assign _0516_ = 1'h0;
  assign _0517_ = mipi_rx_byteclk_i;
  assign _0518_ = _0228_;
  assign _0519_ = _0397_;
  assign _0520_ = { _0233_[3:1], _0221_ };
  assign _0521_ = _0411_;
  assign _0522_ = _0412_;
  assign _0523_ = _0412_;
  assign _0524_ = _0412_;
  assign _0525_ = _0404_;
  assign _0526_ = _0504_;
  assign _0527_ = mipi_rx_d0_i;
  assign _0528_ = { mipi_rx_d0_i[0], mipi_rx_d0_i[1], mipi_rx_d0_i[2], mipi_rx_d0_i[3], mipi_rx_d0_i[4], mipi_rx_d0_i[5], mipi_rx_d0_i[6], mipi_rx_d0_i[7] };
  assign _0529_ = mipi_rx_d1_i;
  assign _0530_ = { mipi_rx_d1_i[0], mipi_rx_d1_i[1], mipi_rx_d1_i[2], mipi_rx_d1_i[3], mipi_rx_d1_i[4], mipi_rx_d1_i[5], mipi_rx_d1_i[6], mipi_rx_d1_i[7] };
  assign _0531_ = mipi_rx_d2_i;
  assign _0532_ = { mipi_rx_d2_i[0], mipi_rx_d2_i[1], mipi_rx_d2_i[2], mipi_rx_d2_i[3], mipi_rx_d2_i[4], mipi_rx_d2_i[5], mipi_rx_d2_i[6], mipi_rx_d2_i[7] };
  assign _0533_ = mipi_rx_d3_i;
  assign _0534_ = { mipi_rx_d3_i[0], mipi_rx_d3_i[1], mipi_rx_d3_i[2], mipi_rx_d3_i[3], mipi_rx_d3_i[4], mipi_rx_d3_i[5], mipi_rx_d3_i[6], mipi_rx_d3_i[7] };
  assign mipi_rx_clk_term_en_o = mipi_rx_clk_hs_en_o;
  assign mipi_rx_dat_term_en_o = mipi_rx_dat_hs_en_o;
  assign mipi_rx_rst_n_o = 1'h1;
  assign mipi_rx_rst_o = 1'h0;
endmodule

====================================================================================================
 [17/29] src/cmos_i2c/I2C_SC2210_19201080_4Lanes_Config.v    (301 行, 已例化)
====================================================================================================
/*-------------------------------------------------------------------------
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo.www.cnblogs.com/crazybingo
(C) COPYRIGHT 2012 CrazyBingo. ALL RIGHTS RESERVED
Filename            :       I2C_SC130GS_12801024_Config.v
Author              :       CrazyBingo
Date                :       2019-08-03
Version             :       1.0
Description         :       I2C Configure Data of AR0135.
Modification History    :
Date            By          Version         Change Description
===========================================================================
19/08/03        CrazyBingo  1.0             Original
--------------------------------------------------------------------------*/

`timescale 1ns/1ns

module  I2C_SC2210_19201080_4Lanes_Config //1920*1080
(
    input       [8:0]   LUT_INDEX,
    output  reg [23:0]  LUT_DATA,
    output      [8:0]   LUT_SIZE
);
assign  LUT_SIZE = 264 + 1;

//-----------------------------------------------------------------
/////////////////////   Config Data LUT   //////////////////////////    
always@(*)
begin
    case(LUT_INDEX)
0:	LUT_DATA = {16'h0103,8'h01}; 
1:	LUT_DATA = {16'h0100,8'h00}; 
2:	LUT_DATA = {16'h36e9,8'h80}; 
3:	LUT_DATA = {16'h36f9,8'h80}; 
4:	LUT_DATA = {16'h3001,8'h07}; 
5:	LUT_DATA = {16'h3002,8'hc0}; 
6:	LUT_DATA = {16'h300a,8'h2c}; 
7:	LUT_DATA = {16'h300f,8'h00}; 
8:	LUT_DATA = {16'h3018,8'h73}; 
9:	LUT_DATA = {16'h3019,8'h00}; 
10:	LUT_DATA = {16'h301f,8'hac}; 
11:	LUT_DATA = {16'h3031,8'h08}; 
12:	LUT_DATA = {16'h3033,8'h20}; 
13:	LUT_DATA = {16'h3038,8'h22}; 
14:	LUT_DATA = {16'h3106,8'h81}; 
15:	LUT_DATA = {16'h3201,8'h04}; 
16:	LUT_DATA = {16'h3203,8'h04}; 
17:	LUT_DATA = {16'h3204,8'h07}; 
18:	LUT_DATA = {16'h3205,8'h8b}; 
19:	LUT_DATA = {16'h3206,8'h04}; 
20:	LUT_DATA = {16'h3207,8'h43}; 
21:	LUT_DATA = {16'h320c,8'h04}; 
22:	LUT_DATA = {16'h320d,8'h37}; 
23:	LUT_DATA = {16'h320e,8'h04}; 
24:	LUT_DATA = {16'h320f,8'h58}; 
25:	LUT_DATA = {16'h3211,8'h04}; 
26:	LUT_DATA = {16'h3213,8'h04}; 
27:	LUT_DATA = {16'h3231,8'h02}; 
28:	LUT_DATA = {16'h3253,8'h04}; 
29:	LUT_DATA = {16'h3301,8'h0a}; 
30:	LUT_DATA = {16'h3302,8'h10}; 
31:	LUT_DATA = {16'h3304,8'h58}; 
32:	LUT_DATA = {16'h3305,8'h00}; 
33:	LUT_DATA = {16'h3306,8'hb0}; 
34:	LUT_DATA = {16'h3308,8'h20}; 
35:	LUT_DATA = {16'h3309,8'h98}; 
36:	LUT_DATA = {16'h330a,8'h01}; 
37:	LUT_DATA = {16'h330b,8'h68}; 
38:	LUT_DATA = {16'h330e,8'h48}; 
39:	LUT_DATA = {16'h3314,8'h92}; 
40:	LUT_DATA = {16'h3000,8'hc0}; 
41:	LUT_DATA = {16'h331e,8'h49}; 
42:	LUT_DATA = {16'h331f,8'h89}; 
43:	LUT_DATA = {16'h334c,8'h10}; 
44:	LUT_DATA = {16'h335d,8'h60}; 
45:	LUT_DATA = {16'h335e,8'h02}; 
46:	LUT_DATA = {16'h335f,8'h06}; 
47:	LUT_DATA = {16'h3364,8'h16}; 
48:	LUT_DATA = {16'h3366,8'h92}; 
49:	LUT_DATA = {16'h3367,8'h10}; 
50:	LUT_DATA = {16'h3368,8'h04}; 
51:	LUT_DATA = {16'h3369,8'h00}; 
52:	LUT_DATA = {16'h336a,8'h00}; 
53:	LUT_DATA = {16'h336b,8'h00}; 
54:	LUT_DATA = {16'h336d,8'h03}; 
55:	LUT_DATA = {16'h337c,8'h08}; 
56:	LUT_DATA = {16'h337d,8'h0e}; 
57:	LUT_DATA = {16'h337f,8'h33}; 
58:	LUT_DATA = {16'h3390,8'h10}; 
59:	LUT_DATA = {16'h3391,8'h30}; 
60:	LUT_DATA = {16'h3392,8'h40}; 
61:	LUT_DATA = {16'h3393,8'h0a}; 
62:	LUT_DATA = {16'h3394,8'h0a}; 
63:	LUT_DATA = {16'h3395,8'h0a}; 
64:	LUT_DATA = {16'h3396,8'h08}; 
65:	LUT_DATA = {16'h3397,8'h30}; 
66:	LUT_DATA = {16'h3398,8'h3f}; 
67:	LUT_DATA = {16'h3399,8'h30}; 
68:	LUT_DATA = {16'h339a,8'h30}; 
69:	LUT_DATA = {16'h339b,8'h30}; 
70:	LUT_DATA = {16'h339c,8'h30}; 
71:	LUT_DATA = {16'h33a2,8'h0a}; 
72:	LUT_DATA = {16'h33b9,8'h0e}; 
73:	LUT_DATA = {16'h33e1,8'h08}; 
74:	LUT_DATA = {16'h33e2,8'h18}; 
75:	LUT_DATA = {16'h33e3,8'h18}; 
76:	LUT_DATA = {16'h33e4,8'h18}; 
77:	LUT_DATA = {16'h33e5,8'h10}; 
78:	LUT_DATA = {16'h33e6,8'h06}; 
79:	LUT_DATA = {16'h33e7,8'h02}; 
80:	LUT_DATA = {16'h33e8,8'h18}; 
81:	LUT_DATA = {16'h33e9,8'h10}; 
82:	LUT_DATA = {16'h33ea,8'h0c}; 
83:	LUT_DATA = {16'h33eb,8'h10}; 
84:	LUT_DATA = {16'h33ec,8'h04}; 
85:	LUT_DATA = {16'h33ed,8'h02}; 
86:	LUT_DATA = {16'h33ee,8'ha0}; 
87:	LUT_DATA = {16'h33ef,8'h08}; 
88:	LUT_DATA = {16'h33f4,8'h18}; 
89:	LUT_DATA = {16'h33f5,8'h10}; 
90:	LUT_DATA = {16'h33f6,8'h0c}; 
91:	LUT_DATA = {16'h33f7,8'h10}; 
92:	LUT_DATA = {16'h33f8,8'h06}; 
93:	LUT_DATA = {16'h33f9,8'h02}; 
94:	LUT_DATA = {16'h33fa,8'h18}; 
95:	LUT_DATA = {16'h33fb,8'h10}; 
96:	LUT_DATA = {16'h33fc,8'h0c}; 
97:	LUT_DATA = {16'h33fd,8'h10}; 
98:	LUT_DATA = {16'h33fe,8'h04}; 
99:	LUT_DATA = {16'h33ff,8'h02}; 
100:	LUT_DATA = {16'h360f,8'h01}; 
101:	LUT_DATA = {16'h3622,8'hf7}; 
102:	LUT_DATA = {16'h3625,8'h0a}; 
103:	LUT_DATA = {16'h3627,8'h02}; 
104:	LUT_DATA = {16'h3630,8'ha2}; 
105:	LUT_DATA = {16'h3631,8'h00}; 
106:	LUT_DATA = {16'h3632,8'hd8}; 
107:	LUT_DATA = {16'h3633,8'h43}; 
108:	LUT_DATA = {16'h3635,8'h20}; 
109:	LUT_DATA = {16'h3638,8'h24}; 
110:	LUT_DATA = {16'h363a,8'h80}; 
111:	LUT_DATA = {16'h363b,8'h02}; 
112:	LUT_DATA = {16'h363e,8'h22}; 
113:	LUT_DATA = {16'h3670,8'h48}; 
114:	LUT_DATA = {16'h3671,8'hf7}; 
115:	LUT_DATA = {16'h3672,8'hf7}; 
116:	LUT_DATA = {16'h3673,8'h07}; 
117:	LUT_DATA = {16'h367a,8'h40}; 
118:	LUT_DATA = {16'h367b,8'h7f}; 
119:	LUT_DATA = {16'h3690,8'h42}; 
120:	LUT_DATA = {16'h3691,8'h43}; 
121:	LUT_DATA = {16'h3692,8'h54}; 
122:	LUT_DATA = {16'h369c,8'h40}; 
123:	LUT_DATA = {16'h369d,8'h7f}; 
124:	LUT_DATA = {16'h36b5,8'h40}; 
125:	LUT_DATA = {16'h36b6,8'h7f}; 
126:	LUT_DATA = {16'h36c0,8'h80}; 
127:	LUT_DATA = {16'h36c1,8'h9f}; 
128:	LUT_DATA = {16'h36c2,8'h9f}; 
129:	LUT_DATA = {16'h36cc,8'h20}; 
130:	LUT_DATA = {16'h36cd,8'h20}; 
131:	LUT_DATA = {16'h36ce,8'h30}; 
132:	LUT_DATA = {16'h36d0,8'h20}; 
133:	LUT_DATA = {16'h36d1,8'h40}; 
134:	LUT_DATA = {16'h36d2,8'h7f}; 
135:	LUT_DATA = {16'h36ea,8'h38}; 
136:	LUT_DATA = {16'h36eb,8'h0e}; 
137:	LUT_DATA = {16'h36ec,8'h13}; 
138:	LUT_DATA = {16'h36ed,8'h14}; 
139:	LUT_DATA = {16'h36fa,8'h3a}; 
140:	LUT_DATA = {16'h36fb,8'h15}; 
141:	LUT_DATA = {16'h36fc,8'h01}; 
142:	LUT_DATA = {16'h36fd,8'h14}; 
143:	LUT_DATA = {16'h3905,8'hd8}; 
144:	LUT_DATA = {16'h3907,8'h01}; 
145:	LUT_DATA = {16'h3908,8'h11}; 
146:	LUT_DATA = {16'h391b,8'h83}; 
147:	LUT_DATA = {16'h391f,8'h00}; 
148:	LUT_DATA = {16'h3933,8'h28}; 
149:	LUT_DATA = {16'h3934,8'ha6}; 
150:	LUT_DATA = {16'h3940,8'h70}; 
151:	LUT_DATA = {16'h3942,8'h08}; 
152:	LUT_DATA = {16'h3943,8'hbc}; 
153:	LUT_DATA = {16'h3958,8'h02}; 
154:	LUT_DATA = {16'h3959,8'h04}; 
155:	LUT_DATA = {16'h3980,8'h61}; 
156:	LUT_DATA = {16'h3987,8'h0b}; 
157:	LUT_DATA = {16'h3990,8'h00}; 
158:	LUT_DATA = {16'h3991,8'h00}; 
159:	LUT_DATA = {16'h3992,8'h00}; 
160:	LUT_DATA = {16'h3993,8'h00}; 
161:	LUT_DATA = {16'h3994,8'h00}; 
162:	LUT_DATA = {16'h3995,8'h00}; 
163:	LUT_DATA = {16'h3996,8'h00}; 
164:	LUT_DATA = {16'h3997,8'h00}; 
165:	LUT_DATA = {16'h3998,8'h00}; 
166:	LUT_DATA = {16'h3999,8'h00}; 
167:	LUT_DATA = {16'h399a,8'h00}; 
168:	LUT_DATA = {16'h399b,8'h00}; 
169:	LUT_DATA = {16'h399c,8'h00}; 
170:	LUT_DATA = {16'h399d,8'h00}; 
171:	LUT_DATA = {16'h399e,8'h00}; 
172:	LUT_DATA = {16'h399f,8'h00}; 
173:	LUT_DATA = {16'h39a0,8'h00}; 
174:	LUT_DATA = {16'h39a1,8'h00}; 
175:	LUT_DATA = {16'h39a2,8'h03}; 
176:	LUT_DATA = {16'h39a3,8'h30}; 
177:	LUT_DATA = {16'h39a4,8'h03}; 
178:	LUT_DATA = {16'h39a5,8'h60}; 
179:	LUT_DATA = {16'h39a6,8'h03}; 
180:	LUT_DATA = {16'h39a7,8'ha0}; 
181:	LUT_DATA = {16'h39a8,8'h03}; 
182:	LUT_DATA = {16'h39a9,8'hb0}; 
183:	LUT_DATA = {16'h39aa,8'h00}; 
184:	LUT_DATA = {16'h39ab,8'h00}; 
185:	LUT_DATA = {16'h39ac,8'h00}; 
186:	LUT_DATA = {16'h39ad,8'h20}; 
187:	LUT_DATA = {16'h39ae,8'h00}; 
188:	LUT_DATA = {16'h39af,8'h40}; 
189:	LUT_DATA = {16'h39b0,8'h00}; 
190:	LUT_DATA = {16'h39b1,8'h60}; 
191:	LUT_DATA = {16'h39b2,8'h00}; 
192:	LUT_DATA = {16'h39b3,8'h00}; 
193:	LUT_DATA = {16'h39b4,8'h08}; 
194:	LUT_DATA = {16'h39b5,8'h14}; 
195:	LUT_DATA = {16'h39b6,8'h20}; 
196:	LUT_DATA = {16'h39b7,8'h38}; 
197:	LUT_DATA = {16'h39b8,8'h38}; 
198:	LUT_DATA = {16'h39b9,8'h20}; 
199:	LUT_DATA = {16'h39ba,8'h14}; 
200:	LUT_DATA = {16'h39bb,8'h08}; 
201:	LUT_DATA = {16'h39bc,8'h08}; 
202:	LUT_DATA = {16'h39bd,8'h10}; 
203:	LUT_DATA = {16'h39be,8'h20}; 
204:	LUT_DATA = {16'h39bf,8'h30}; 
205:	LUT_DATA = {16'h39c0,8'h30}; 
206:	LUT_DATA = {16'h39c1,8'h20}; 
207:	LUT_DATA = {16'h39c2,8'h10}; 
208:	LUT_DATA = {16'h39c3,8'h08}; 
209:	LUT_DATA = {16'h39c4,8'h00}; 
210:	LUT_DATA = {16'h39c5,8'h80}; 
211:	LUT_DATA = {16'h39c6,8'h00}; 
212:	LUT_DATA = {16'h39c7,8'h80}; 
213:	LUT_DATA = {16'h39c8,8'h00}; 
214:	LUT_DATA = {16'h39c9,8'h00}; 
215:	LUT_DATA = {16'h39ca,8'h80}; 
216:	LUT_DATA = {16'h39cb,8'h00}; 
217:	LUT_DATA = {16'h39cc,8'h00}; 
218:	LUT_DATA = {16'h39cd,8'h00}; 
219:	LUT_DATA = {16'h39ce,8'h00}; 
220:	LUT_DATA = {16'h39cf,8'h00}; 
221:	LUT_DATA = {16'h39d0,8'h00}; 
222:	LUT_DATA = {16'h39d1,8'h00}; 
223:	LUT_DATA = {16'h39e2,8'h05}; 
224:	LUT_DATA = {16'h39e3,8'heb}; 
225:	LUT_DATA = {16'h39e4,8'h07}; 
226:	LUT_DATA = {16'h39e5,8'hb6}; 
227:	LUT_DATA = {16'h39e6,8'h00}; 
228:	LUT_DATA = {16'h39e7,8'h3a}; 
229:	LUT_DATA = {16'h39e8,8'h3f}; 
230:	LUT_DATA = {16'h39e9,8'hb7}; 
231:	LUT_DATA = {16'h39ea,8'h02}; 
232:	LUT_DATA = {16'h39eb,8'h4f}; 
233:	LUT_DATA = {16'h39ec,8'h08}; 
234:	LUT_DATA = {16'h39ed,8'h00}; 
235:	LUT_DATA = {16'h3e00,8'h00}; 
236:	LUT_DATA = {16'h3e01,8'h45}; 
237:	LUT_DATA = {16'h3e02,8'h40}; 
238:	LUT_DATA = {16'h3e03,8'h0b}; 
239:	LUT_DATA = {16'h3e06,8'h00}; 
240:	LUT_DATA = {16'h3e07,8'h80}; 
241:	LUT_DATA = {16'h3e08,8'h03}; 
242:	LUT_DATA = {16'h3e09,8'h40}; 
243:	LUT_DATA = {16'h3e14,8'h31}; 
244:	LUT_DATA = {16'h3e1b,8'h3a}; 
245:	LUT_DATA = {16'h3e26,8'h40}; 
246:	LUT_DATA = {16'h3f08,8'h08}; 
247:	LUT_DATA = {16'h4401,8'h1a}; 
248:	LUT_DATA = {16'h4407,8'hc0}; 
249:	LUT_DATA = {16'h4418,8'h34}; 
250:	LUT_DATA = {16'h4500,8'h18}; 
251:	LUT_DATA = {16'h4501,8'hb4}; 
252:	LUT_DATA = {16'h4509,8'h20}; 
253:	LUT_DATA = {16'h4603,8'h00}; 
254:	LUT_DATA = {16'h4800,8'h24}; 
255:	LUT_DATA = {16'h4837,8'h25}; 
256:	LUT_DATA = {16'h5000,8'h0e}; 
257:	LUT_DATA = {16'h550f,8'h20}; 
258:	LUT_DATA = {16'h8c50,8'h40}; 
259:	LUT_DATA = {16'h36e9,8'h24}; 
260:	LUT_DATA = {16'h36f9,8'h14}; 
261:	LUT_DATA = {16'h3652,8'h44}; 
262:	LUT_DATA = {16'h3653,8'h44}; 
263:	LUT_DATA = {16'h3654,8'h44}; 
264:	LUT_DATA = {16'h0100,8'h01}; 
		default:LUT_DATA    =   {16'h0000, 8'h00};
    endcase
end

endmodule

====================================================================================================
 [18/29] src/cmos_i2c/i2c_timing_ctrl_reg16_dat8_wronly.v    (375 行, 已例化)
====================================================================================================
/*-----------------------------------------------------------------------
                                 \\\|///
                                \\  - -  //
                                (  @ @  )
+-----------------------------oOOo-(_)-oOOo-----------------------------+
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2012-20xx CrazyBingo Corporation.
The entire notice above must be reproduced on all authorized copies.
Author                  :       CrazyBingo
Technology blogs 	: 		www.crazyfpga.com
Email Address           :       crazyfpga@qq.com
Filename                :       module  i2c_timing_ctrl_reg16_dat8.v
Data                    :       2013-04-13
Description             :       The testbench of I2C Ctrl timing.
Modification History    :
Data            Author          Version     Change Description
=========================================================================
13/04/13        CrazyBingo      1.0         Original
13/04/14        CrazyBingo      1.1         add i2c_wdata for avoid bit width warming
14/05/13        CrazyBingo      2.0         Change to 16Bit Data module
22/01/28	CrazyBingo	3.0	    Change to 16bit register & 8bit data mode
-------------------------------------------------------------------------
|                                     Oooo                              |
+------------------------------oooO--(   )------------------------------+
                              (   )   ) /
                               \ (   (_/
                                \_)
-----------------------------------------------------------------------*/

`timescale 1ns/1ns
module  i2c_timing_ctrl_reg16_dat8_wronly
#(
    parameter   CLK_FREQ    =   100_000000, //100 MHz
    parameter   I2C_FREQ    =   400_000     //10 KHz(< 400KHz)
)
(
    //global clock
    input               clk,                //100MHz
    input               rst_n,              //system reset
            
    //i2c interface     
    output              i2c_sclk,           //i2c clock
    // inout               i2c_sdat,           //i2c data for bidirection
    input               i2c_sdat_IN,
    output              i2c_sdat_OUT,
    output              i2c_sdat_OE,

    //user interface
    input       [8:0]   i2c_config_size,    //i2c config data counte
    output  reg [8:0]   i2c_config_index,   //i2c config reg index, read 2 reg and write xx reg
    input       [31:0]  i2c_config_data,    //i2c config data
    output              i2c_config_done    //i2c config timing complete
);

//----------------------------------------
//Delay xxus until i2c slave is steady
reg [19:0]  delay_cnt;
//localparam  DELAY_TOP = SIMU_EN ? 20'hff : CLK_FREQ/1000;  //1ms Setting time after software/hardware reset
localparam  DELAY_TOP = CLK_FREQ/1000;  //1ms Setting time after software/hardware reset
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        delay_cnt <= 0;
    else if(delay_cnt < DELAY_TOP)
        delay_cnt <= delay_cnt + 1'b1;
    else
        delay_cnt <= delay_cnt;
end
wire    delay_done = (delay_cnt == DELAY_TOP) ? 1'b1 : 1'b0;    //81us delay


//----------------------------------------
//I2C Control Clock generate
reg [15:0]  clk_cnt;    //divide for i2c clock
/******************************************
             _______          _______
SCLK    ____|       |________|       |
         ________________ ______________
SDAT    |________________|______________
         _                _
CLK_EN  | |______________| |____________
                _                _
CAP_EN  _______| |______________| |_____
*******************************************/
reg i2c_ctrl_clk;       //i2c control clock, H: valid; L: valid
reg i2c_transfer_en;    //send i2c data before, make sure that sdat is steady when i2c_sclk is valid
reg i2c_capture_en;     //capture i2c data  while sdat is steady from cmos              
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
        clk_cnt <= 0;
        i2c_ctrl_clk <= 0;
        i2c_transfer_en <= 0;
        i2c_capture_en <= 0;
        end
    else if(delay_done)
        begin
        if(clk_cnt < (CLK_FREQ/I2C_FREQ) - 1'b1)
            clk_cnt <= clk_cnt + 1'd1;
        else
            clk_cnt <= 0;
        //i2c control clock, H: valid; L: valid
        i2c_ctrl_clk <= ((clk_cnt >= (CLK_FREQ/I2C_FREQ)/4 + 1'b1) &&
                        (clk_cnt < (3*CLK_FREQ/I2C_FREQ)/4 + 1'b1)) ? 1'b1 : 1'b0;
        //send i2c data before, make sure that sdat is steady when i2c_sclk is valid
        i2c_transfer_en <= (clk_cnt == 16'd0) ? 1'b1 : 1'b0;
        //capture i2c data  while sdat is steady from cmos                  
        i2c_capture_en <= (clk_cnt == (2*CLK_FREQ/I2C_FREQ)/4 - 1'b1) ? 1'b1 : 1'b0;
        end
    else
        begin
        clk_cnt <= 0;
        i2c_ctrl_clk <= 0;
        i2c_transfer_en <= 0;
        i2c_capture_en <= 0;
        end
end

//-----------------------------------------
//I2C Timing state Parameter
localparam  I2C_IDLE        =   4'd0;
//Write I2C: {ID_Address, REG_Address, W_REG_Data}
localparam  I2C_WR_START    =   4'd1;
localparam  I2C_WR_IDADDR   =   4'd2;
localparam  I2C_WR_ACK1     =   4'd3;
localparam  I2C_WR_REGADDR1 =   4'd4;
localparam  I2C_WR_ACK2     =   4'd5;
localparam  I2C_WR_REGADDR2 =   4'd6;
localparam  I2C_WR_ACK3     =   4'd7;
localparam  I2C_WR_REGDATA  =   4'd8;
localparam  I2C_WR_ACK4     =   4'd9;
localparam  I2C_WR_STOP     =   4'd10;


//-----------------------------------------
// FSM: always1
reg	[3:0]	current_state, next_state; //i2c write and read state  
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        current_state <= I2C_IDLE;
    else if(i2c_transfer_en)
        current_state <= next_state;
end

                                
//-----------------------------------------
wire    i2c_transfer_end = (current_state == I2C_WR_STOP) ? 1'b1 : 1'b0;
reg     i2c_ack;    //i2c slave renpose successed
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        i2c_config_index <= 0;
    else if(i2c_transfer_en)
        begin
		//	Force always step following. 
		if(i2c_transfer_end)	//	 & ~i2c_ack)
//		if(i2c_transfer_end /*& ~i2c_ack*/)											//Just for test
            begin
            if(i2c_config_index < i2c_config_size)  
                i2c_config_index <= i2c_config_index + 1'b1;
//              i2c_config_index <= {i2c_config_index[7:1], ~i2c_config_index[0]};  //Just for test, Read Only
            else
                i2c_config_index <= i2c_config_size;
            end
        else
            i2c_config_index <= i2c_config_index;
        end
    else
        i2c_config_index <= i2c_config_index;
end
assign  i2c_config_done = (i2c_config_index == i2c_config_size) ? 1'b1 : 1'b0;


//-----------------------------------------
// FSM: always2
reg [3:0]   i2c_stream_cnt; //i2c data bit stream count
always@(*)
begin
    next_state = I2C_IDLE;  //state initialization
    case(current_state)
    I2C_IDLE:       //5'd0
        begin
        if(delay_done == 1'b1)  //1ms Setting time after software/hardware reset    
            begin
            if(i2c_transfer_en)
                begin                
                if(i2c_config_index < i2c_config_size)
                    next_state = I2C_WR_START;  //Write Data to I2C
                else// if(i2c_config_index >= i2c_config_size)
                    next_state = I2C_IDLE;      //Config I2C Complete
					 end
            else
                next_state = next_state;
            end
        else
                next_state = I2C_IDLE;      //Wait I2C Bus is steady
        end
    //Write I2C: {ID_Address, REG_Address, W_REG_Data}
    I2C_WR_START:   
        if(i2c_transfer_en) next_state = I2C_WR_IDADDR;
        else                next_state = I2C_WR_START;
    I2C_WR_IDADDR:  
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK1;
        else                next_state = I2C_WR_IDADDR;
    I2C_WR_ACK1:    
        if(i2c_transfer_en) next_state = I2C_WR_REGADDR1;
        else                next_state = I2C_WR_ACK1;        
    I2C_WR_REGADDR1: 
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK2;
        else                next_state = I2C_WR_REGADDR1;
    I2C_WR_ACK2:    
        if(i2c_transfer_en) next_state = I2C_WR_REGADDR2;
        else                next_state = I2C_WR_ACK2;
    I2C_WR_REGADDR2: 
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK3;
        else                next_state = I2C_WR_REGADDR2;        
    I2C_WR_ACK3:    
        if(i2c_transfer_en) next_state = I2C_WR_REGDATA;
        else                next_state = I2C_WR_ACK3;        
    I2C_WR_REGDATA:    
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK4;
        else                next_state = I2C_WR_REGDATA;
    I2C_WR_ACK4:    
        if(i2c_transfer_en) next_state = I2C_WR_STOP;
        else                next_state = I2C_WR_ACK4;

    I2C_WR_STOP:    
        if(i2c_transfer_en) next_state = I2C_IDLE;
        else                next_state = I2C_WR_STOP;

    default:;   //default vaule     
    endcase
end


//-----------------------------------------
// FSM: always3
//reg   i2c_write_flag, i2c_read_flag;
reg i2c_sdat_out;       //i2c data output
//reg   [3:0]   i2c_stream_cnt; //i2c data bit stream count
reg [7:0]   i2c_wdata;  //i2c data prepared to transfer
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
        i2c_sdat_out <= 1'b1;
        i2c_stream_cnt <= 0;
        i2c_wdata <= 0;
        end
    else if(i2c_transfer_en)
        begin
        case(next_state)
        I2C_IDLE:   
            begin
            i2c_sdat_out <= 1'b1;       //idle state
            i2c_stream_cnt <= 0;
            i2c_wdata <= 0;
            end
        //Write I2C: {ID_Address, REG_Address, W_REG_Data}
        I2C_WR_START:   
            begin
            i2c_sdat_out <= 1'b0;
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[31:24];    //ID_Address
            end
        I2C_WR_IDADDR:  
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_WR_ACK1:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[23:16];        //REG_Address
            end            
        I2C_WR_REGADDR1: 
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_WR_ACK2:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[15:8];        //REG_Address
            end
        I2C_WR_REGADDR2: 
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end         
        I2C_WR_ACK3:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[7:0];     //W_REG_Data1
            end            
        I2C_WR_REGDATA:    
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_WR_ACK4:    
            i2c_stream_cnt <= 0;

        I2C_WR_STOP:    
            i2c_sdat_out <= 1'b0;

        endcase
        end
    else
        begin
        i2c_stream_cnt <= i2c_stream_cnt;
        i2c_sdat_out <= i2c_sdat_out;
        i2c_wdata <= i2c_wdata;
        end
end




//---------------------------------------------
//respone from slave for i2c data transfer
reg i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
		{i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4} <= 4'b1111;
        i2c_ack <= 1'b1;
        end
    else if(i2c_capture_en)
        begin
        case(next_state)
        I2C_IDLE:
        begin
        {i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4} <= 4'b1111;
        i2c_ack <= 1'b1;
        end
        //Write I2C: {ID_Address, REG_Address, W_REG_Data}
        I2C_WR_ACK1:    i2c_ack1 <= i2c_sdat_IN;
        I2C_WR_ACK2:    i2c_ack2 <= i2c_sdat_IN;
        I2C_WR_ACK3:    i2c_ack3 <= i2c_sdat_IN;
        I2C_WR_ACK4:    i2c_ack4 <= i2c_sdat_IN;
        I2C_WR_STOP:    i2c_ack <= (i2c_ack1 | i2c_ack2 | i2c_ack3);// | i2c_ack4 );

        default:;
        endcase
        end
    else
        begin
        {i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4} <= {i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4};
        i2c_ack <= i2c_ack;
        end
end 

//---------------------------------------------------
wire    bir_en =(   current_state == I2C_WR_ACK1 || current_state == I2C_WR_ACK2 || 
	            current_state == I2C_WR_ACK3 || current_state == I2C_WR_ACK4) ? 1'b1 : 1'b0;
assign  i2c_sclk = (current_state >= I2C_WR_IDADDR && current_state <= I2C_WR_ACK4) ? 
                    i2c_ctrl_clk : 1'b1;
//assign  i2c_sdat = (~bir_en) ? i2c_sdat_out : 1'bz;
assign i2c_sdat_OUT = i2c_sdat_out;
assign i2c_sdat_OE = (~rst_n) ? 0 : ~bir_en;


endmodule

====================================================================================================
 [19/29] src/cmos_i2c/CMOS_Capture_RAW_Gray.v    (189 行, 未例化)
====================================================================================================
/*-------------------------------------------------------------------------
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2011-201x CrazyBingo Corporation
The entire notice above must be reproduced on all authorized copies.
Author				:		CrazyBingo
Technology blogs 	: 		www.crazyfpga.com
Eamil Address 		: 		crazyfpga@vip.qq.com
Filename			:		CMOS_Capture_RAW_Gray.v
Date				:		2013-04-10
Version				:		1.0
Description			:		Capture cmos data from cmos senser of RAW or Gray Format
Modification History	:
Date			By			Version		Change Description
===========================================================================
13/04/10		CrazyBingo	1.0			Original
13/04/14		CrazyBingo	1.1			Complete
13/05/26		CrazyBingo	1.2			Modification
13/11/08		CrazyBingo	2.0			Modification	
14/03/16		CrazyBingo	2.0			Modification
--------------------------------------------------------------------------*/
`timescale 1ns/1ns
module CMOS_Capture_RAW_Gray
#(
	parameter			CMOS_FRAME_WAITCNT	=	4'd10,		//Wait n fps for steady(OmniVision need 10 Frame)
	parameter			CMOS_PCLK_FREQ		=	24_000000														
)
(
	//global clock
	input				clk_cmos,			//24MHz CMOS Driver clock input
	input				rst_n,				//global reset

	//CMOS Sensor Interface
	input				cmos_pclk,			//24MHz CMOS Pixel clock input
	output				cmos_xclk,			//24MHz drive clock
	input				cmos_vsync,			//H : Data Valid; L : Frame Sync(Set it by register)
	input				cmos_href,			//H : Data vaild, L : Line Sync
	input		[7:0]	cmos_data,			//8 bits cmos data input
	
	//CMOS SYNC Data output
	output				cmos_frame_vsync,	//cmos frame data vsync valid signal
	output				cmos_frame_href,	//cmos frame data href vaild  signal
	output		[7:0]	cmos_frame_data,	//cmos frame RAW output	
	
	//user interface
	output	reg	[7:0]	cmos_fps_rate,		//cmos frame output rate
	output				cmos_vsync_end,
	output	reg	[11:0]	pixel_cnt,
	output	reg	[11:0]	line_cnt
);
assign	cmos_xclk = clk_cmos;	//24MHz CMOS XCLK output

//-----------------------------------------------------
//Sensor HS & VS Vaild Capture
/**************************************************					       
         _________________________________
VS______|                                 |________
	            _______	 	     _______
HS_____________|       |__...___|       |____________
**************************************************/
//-------------------------------------------------------------
//sync the frame vsync and href signal and generate frame begin & end signal
reg	[1:0]	cmos_vsync_r, cmos_href_r;
reg	[7:0]	cmos_data_r0, cmos_data_r1;
always@(posedge cmos_pclk or negedge rst_n)
begin
	if(!rst_n)
		begin
		cmos_vsync_r <= 0;
		cmos_href_r <= 0;
		{cmos_data_r1, cmos_data_r0} <= 0;
		end
	else
		begin
		cmos_vsync_r <= {cmos_vsync_r[0], cmos_vsync};
		cmos_href_r <= {cmos_href_r[0], cmos_href};
		{cmos_data_r1, cmos_data_r0} <= {cmos_data_r0, cmos_data};
		end
end
//wire	cmos_vsync_begin 	= 	(~cmos_vsync_r[1] & cmos_vsync_r[0]) ? 1'b1 : 1'b0;	
assign	cmos_vsync_end 		= 	(cmos_vsync_r[1] & ~cmos_vsync_r[0]) ? 1'b1 : 1'b0;	
wire	cmos_href_begin		=	(cmos_vsync_r[0] & ~cmos_href_r[1] & cmos_href_r[0]) ? 1'b1 : 1'b0;	

//----------------------------------------------------------------------------------
//reg	[11:0]	pixel_cnt;	
//reg	[11:0]	line_cnt;	
always@(posedge cmos_pclk or negedge rst_n)
begin
	if(!rst_n)
		pixel_cnt <= 0;
	else if(cmos_vsync_r[0] & cmos_href_r[0])
		pixel_cnt <= pixel_cnt + 1'b1;
	else
		pixel_cnt <= 0;
end

always@(posedge cmos_pclk or negedge rst_n)
begin
	if(!rst_n)
		line_cnt <= 0;
	else if(cmos_vsync_r[0] == 1'b1)
		begin
		if(cmos_href_begin == 1'b1)
			line_cnt <= line_cnt  + 1'b1;
		else
			line_cnt <= line_cnt;
		end
	else
		line_cnt <= 0;
end


//----------------------------------------------------------------------------------
//Wait for Sensor output Data valid 10 Frame of OmniVision
reg	[3:0]	cmos_fps_cnt;
always@(posedge cmos_pclk or negedge rst_n)
begin
	if(!rst_n)
		cmos_fps_cnt <= 0;
	else	//Wait until cmos init complete
		begin
		if(cmos_fps_cnt < CMOS_FRAME_WAITCNT)	
			cmos_fps_cnt <= cmos_vsync_end ? cmos_fps_cnt + 1'b1 : cmos_fps_cnt;
		else
			cmos_fps_cnt <= CMOS_FRAME_WAITCNT;
		end
end

//----------------------------------------------------------------------------------
//Come ture frame synchronization to ignore error frame or has not capture when vsync begin
reg		frame_sync_flag;
always@(posedge cmos_pclk or negedge rst_n)
begin
	if(!rst_n)
		frame_sync_flag <= 0;
	else if(cmos_fps_cnt == CMOS_FRAME_WAITCNT && cmos_vsync_end == 1)
		frame_sync_flag <= 1;
	else
		frame_sync_flag <= frame_sync_flag;
end


assign	cmos_frame_vsync = frame_sync_flag ? cmos_vsync_r[1]  : 1'b0;//DFF 2 clocks
assign	cmos_frame_href  = frame_sync_flag ? cmos_href_r[1] & cmos_vsync_r[1]: 1'b0;	//DFF 2 clocks
assign	cmos_frame_data	 = frame_sync_flag & cmos_frame_href ? cmos_data_r1 : 8'd0;	//DFF 2 clocks

//----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------
//Delay 2s for cmos fps counter
localparam	DELAY_TOP = 2 * CMOS_PCLK_FREQ;	//2s delay
reg	[27:0]	delay_cnt;
always@(posedge cmos_pclk or negedge rst_n)
begin
	if(!rst_n)
		delay_cnt <= 0;
	else if(delay_cnt < DELAY_TOP - 1'b1)
		delay_cnt <= delay_cnt + 1'b1;
	else
		delay_cnt <= 0;
end
wire	delay_2s = (delay_cnt == DELAY_TOP - 1'b1) ? 1'b1 : 1'b0;

//-------------------------------------
//cmos image output rate counter
reg	[8:0]	cmos_fps_cnt2;
always@(posedge cmos_pclk or negedge rst_n)
begin
	if(!rst_n)
		begin
		cmos_fps_cnt2 <= 0;
		cmos_fps_rate <= 0;
		end
	else if(delay_2s == 1'b0)	//time is not reached
		begin
		cmos_fps_cnt2 <= cmos_vsync_end ? cmos_fps_cnt2 + 1'b1 : cmos_fps_cnt2;
		cmos_fps_rate <= cmos_fps_rate;
		end
	else	//time up
		begin
		cmos_fps_cnt2 <= 0;
		cmos_fps_rate <= cmos_fps_cnt2[8:1];	//divide by 2
		end
end


endmodule

====================================================================================================
 [20/29] src/cmos_i2c/I2C_AR0135_1280720_Config.v    (73 行, 未例化)
====================================================================================================
/*-------------------------------------------------------------------------
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo.www.cnblogs.com/crazybingo
(C) COPYRIGHT 2012 CrazyBingo. ALL RIGHTS RESERVED
Filename			:		I2C_AR0144_1280720_Config.v
Author				:		CrazyBingo
Date				:		2019-08-03
Version				:		1.0
Description			:		I2C Configure Data of AR0135.
Modification History	:
Date			By			Version			Change Description
===========================================================================
19/08/03		CrazyBingo	1.0				Original
--------------------------------------------------------------------------*/

`timescale 1ns/1ns
module	I2C_AR0135_1280720_Config   //1280*720@60 with AutO/Manual Exposure
(
	input		[7:0]	LUT_INDEX,
	output	reg	[31:0]	LUT_DATA,
	output		[7:0]	LUT_SIZE
);
`define PLL_EN
`define AE_EN

assign	LUT_SIZE = 1'b1 + 8'd23 ;

//-----------------------------------------------------------------
/////////////////////	Config Data LUT	  //////////////////////////	
always@(*)
begin
	case(LUT_INDEX)
//	AR0135 : 1280*720 Gray Config
	0 :		LUT_DATA	=	{16'h3000, 16'h0554};  //Chip Vesion Register
	//Write Data Index   
    1 :     LUT_DATA    =   {16'h301A, 16'h00D9};  //RESET_REGISTER
    2 :     LUT_DATA    =   {16'h0000, 16'h0000};  //DELAY= 200
    3 :     LUT_DATA    =   {16'h301A, 16'h10D8};  //RESET_REGISTER
    
    4 :     LUT_DATA    =   {16'h302C, 16'h0001};  //VT_SYS_CLK_DIV, 27MHz to 74.25MHz
    5 :     LUT_DATA    =   {16'h302A, 16'h0008};  //VT_PIX_CLK_DIV
    6 :     LUT_DATA    =   {16'h302E, 16'h0002};  //PRE_PLL_CLK_DIV
    7 :     LUT_DATA    =   {16'h3030, 16'h002C};  //PLL_MULTIPLIER
    8 :     LUT_DATA    =   {16'h30B0, 16'h04A0};  //DIGITAL_TEST, enable PLL
    9 :     LUT_DATA    =   {16'h0000, 16'h0000};  //DELAY= 200
    
    10 :    LUT_DATA    =   {16'h3002, 16'h0078};  //Y_ADDR_START
    11 :    LUT_DATA    =   {16'h3004, 16'h0000};  //X_ADDR_START
    12 :    LUT_DATA    =   {16'h3006, 16'h0347};  //Y_ADDR_END
    13 :    LUT_DATA    =   {16'h3008, 16'h04FF};  //X_ADDR_END
    14 :    LUT_DATA    =   {16'h300A, 16'h02EB};  //FRAME_LENGTH_LINES
    15 :    LUT_DATA    =   {16'h300C, 16'h0672};  //LINE_LENGTH_PCK
    
    16 :    LUT_DATA    =   {16'h30A2, 16'h0001};  //X_ODD_INC
    17 :    LUT_DATA    =   {16'h30A6, 16'h0001};  //Y_ODD_INC
    18 :    LUT_DATA    =   {16'h3040, 16'h8000};  //READ_MODE
    19 :    LUT_DATA    =   {16'h3028, 16'h0010};  //ROW_SPEED
    
    //Manual Gain & Expsoure Parameter
    20 :    LUT_DATA    =   {16'h305E, 16'h0020};   //Global Gain Defalut 0x20
    21 :    LUT_DATA    =   {16'h3012, 16'd960};   //COARSE_INTEGRATION_TIME
   
    //Register for Auto Exposure
    22 :    LUT_DATA    =   {16'h3100, 16'h0013};  //bit[4]:Auto DG, bit[1]:Auto AG; bit[0]:Auto Exposure

    23 :    LUT_DATA    =   {16'h301A, 16'h10DC};  //RESET_REGISTER - [0:00:02.440]

	default:LUT_DATA	=	{16'h0000, 16'h0000};
	endcase
end

endmodule

====================================================================================================
 [21/29] src/cmos_i2c/I2C_MT9M001_Gray_Config.v    (61 行, 未例化)
====================================================================================================
/*-------------------------------------------------------------------------
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo.www.cnblogs.com/crazybingo
(C) COPYRIGHT 2012 CrazyBingo. ALL RIGHTS RESERVED
Filename			:		I2C_MT9M001_Gray_Config.v
Author				:		CrazyBingo
Date				:		2012-05-11
Version				:		1.0
Description			:		I2C Configure Data of MT9M001.
Modification History	:
Date			By			Version			Change Description
===========================================================================
12/05/11		CrazyBingo	1.0				Original
12/05/13		CrazyBingo	1.1				Modification
12/06/01		CrazyBingo	1.4				Modification
12/06/05		CrazyBingo	1.5				Modification
13/04/08		CrazyBingo	1.6				Modification
17/08/12		CrazyBingo	2.0				Modification for MT9M001 V1.0
--------------------------------------------------------------------------*/

`timescale 1ns/1ns
module	I2C_MT9M001_Gray_Config
(
	input		[7:0]	LUT_INDEX,
	output	reg	[23:0]	LUT_DATA,
	output		[7:0]	LUT_SIZE
);

//---------------------------------------------
//  LUT size
assign  LUT_SIZE = 8'd12;

//---------------------------------------------
//  Config Data LUT
always@(*)
begin
    case(LUT_INDEX)
        0   :   LUT_DATA = {24'h0D_0001};	//Reset
        1   :   LUT_DATA = {24'h0D_0000};
        2   :   LUT_DATA = {24'h09_0419};	//Shutter Width, 	Default 0x0419(1049)
        3   :   LUT_DATA = {24'h0C_0000};	//Shutter Delay,	Default 0x0000
        4   :   LUT_DATA = {24'h01_00A4};	//Row Start, 		Default 0x000C,	12+(1024-720)/2=164
        5   :   LUT_DATA = {24'h02_0014};	//Column Start, 	Default 0x0014
        6   :   LUT_DATA = {24'h03_02CF};	//Row Width, 		Default 0x03FF,	0x02CF = 720  - 1
        7   :   LUT_DATA = {24'h04_04FF};	//Column Width, 	Default	0x04FF,	0x3FFF = 1280 - 1
        8   :   LUT_DATA = {24'h20_D104};	//Read Option1,		Defalut 0x1104,	bit[15:14] is for Row/col mirror
		
        9   :   LUT_DATA = {8'h09, 16'd960};//Shutter Width, 	Default 0x0419 = 1049
        10  :   LUT_DATA = {24'h0C_0082};	//Shutter Delay,	Default 0x0000
		
		11	:	LUT_DATA = {24'h35_0020};	//Global Gain,		Default 0x0008;	0x08-0x20:1-4;0x51-0x60:4.25-8;0x61-0x67:9-15
        default :   LUT_DATA = {24'h0D0000};
    endcase
end



endmodule

====================================================================================================
 [22/29] src/cmos_i2c/I2C_SC130GS_12801024_4Lanes_Config.v    (143 行, 未例化)
====================================================================================================
/*-------------------------------------------------------------------------
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo.www.cnblogs.com/crazybingo
(C) COPYRIGHT 2012 CrazyBingo. ALL RIGHTS RESERVED
Filename            :       I2C_SC130GS_12801024_Config.v
Author              :       CrazyBingo
Date                :       2019-08-03
Version             :       1.0
Description         :       I2C Configure Data of AR0135.
Modification History    :
Date            By          Version         Change Description
===========================================================================
19/08/03        CrazyBingo  1.0             Original
--------------------------------------------------------------------------*/

`timescale 1ns/1ns
module  I2C_SC130GS_12801024_4Lanes_Config  //1280*1024@60 with AutO/Manual Exposure
(
    input       [7:0]   LUT_INDEX,
    output  reg [23:0]  LUT_DATA,
    output      [7:0]   LUT_SIZE
);
assign  LUT_SIZE = 106 + 1;

//-----------------------------------------------------------------
/////////////////////   Config Data LUT   //////////////////////////    
always@(*)
begin
    case(LUT_INDEX)
0:	LUT_DATA = {16'h0103, 8'h01}; 
1:	LUT_DATA = {16'h0100, 8'h00}; 
2:	LUT_DATA = {16'h3039, 8'h80}; 
3:	LUT_DATA = {16'h3034, 8'h80}; 
4:	LUT_DATA = {16'h3001, 8'h00}; 
5:	LUT_DATA = {16'h3018, 8'h70}; 
6:	LUT_DATA = {16'h3019, 8'h00}; 
7:	LUT_DATA = {16'h301f, 8'h47}; 
8:	LUT_DATA = {16'h3022, 8'h10}; 
9:	LUT_DATA = {16'h302b, 8'h80}; 
10:	LUT_DATA = {16'h3030, 8'h01}; 
11:	LUT_DATA = {16'h3000, 8'h00}; 
12:	LUT_DATA = {16'h3031, 8'h08}; 
13:	LUT_DATA = {16'h3035, 8'hd2}; 
14:	LUT_DATA = {16'h3036, 8'h00}; 
15:	LUT_DATA = {16'h3038, 8'h4b}; 
16:	LUT_DATA = {16'h303a, 8'h35}; 
17:	LUT_DATA = {16'h303b, 8'h0e}; 
18:	LUT_DATA = {16'h303c, 8'h06}; 
19:	LUT_DATA = {16'h303d, 8'h03}; 
20:	LUT_DATA = {16'h303f, 8'h11}; 
21:	LUT_DATA = {16'h3202, 8'h00}; 
22:	LUT_DATA = {16'h3203, 8'h00}; 
23:	LUT_DATA = {16'h3205, 8'h8b}; 
24:	LUT_DATA = {16'h3206, 8'h02}; 
25:	LUT_DATA = {16'h3207, 8'h04}; 
26:	LUT_DATA = {16'h320a, 8'h04}; 
27:	LUT_DATA = {16'h320b, 8'h00}; 
28:	LUT_DATA = {16'h320c, 8'h03}; 
29:	LUT_DATA = {16'h320d, 8'h0c}; 
30:	LUT_DATA = {16'h320e, 8'h02}; 
31:	LUT_DATA = {16'h320f, 8'h0f}; 
32:	LUT_DATA = {16'h3211, 8'h08}; 
33:	LUT_DATA = {16'h3213, 8'h04}; 
34:	LUT_DATA = {16'h3300, 8'h20}; 
35:	LUT_DATA = {16'h3302, 8'h0c}; 
36:	LUT_DATA = {16'h3306, 8'h48}; 
37:	LUT_DATA = {16'h3308, 8'h50}; 
38:	LUT_DATA = {16'h330a, 8'h01}; 
39:	LUT_DATA = {16'h330b, 8'h20}; 
40:	LUT_DATA = {16'h330e, 8'h1a}; 
41:	LUT_DATA = {16'h3310, 8'hf0}; 
42:	LUT_DATA = {16'h3311, 8'h10}; 
43:	LUT_DATA = {16'h3319, 8'he8}; 
44:	LUT_DATA = {16'h3333, 8'h90}; 
45:	LUT_DATA = {16'h3334, 8'h30}; 
46:	LUT_DATA = {16'h3348, 8'h02}; 
47:	LUT_DATA = {16'h3349, 8'hee}; 
48:	LUT_DATA = {16'h334a, 8'h02}; 
49:	LUT_DATA = {16'h334b, 8'he0}; 
50:	LUT_DATA = {16'h335d, 8'h00}; 
51:	LUT_DATA = {16'h3380, 8'hff}; 
52:	LUT_DATA = {16'h3382, 8'he0}; 
53:	LUT_DATA = {16'h3383, 8'h0a}; 
54:	LUT_DATA = {16'h3384, 8'he4}; 
55:	LUT_DATA = {16'h3400, 8'h53}; 
56:	LUT_DATA = {16'h3416, 8'h31}; 
57:	LUT_DATA = {16'h3518, 8'h07}; 
58:	LUT_DATA = {16'h3519, 8'hc8}; 
59:	LUT_DATA = {16'h3620, 8'h24}; 
60:	LUT_DATA = {16'h3621, 8'h0a}; 
61:	LUT_DATA = {16'h3622, 8'h06}; 
62:	LUT_DATA = {16'h3623, 8'h14}; 
63:	LUT_DATA = {16'h3624, 8'h20}; 
64:	LUT_DATA = {16'h3625, 8'h00}; 
65:	LUT_DATA = {16'h3626, 8'h00}; 
66:	LUT_DATA = {16'h3627, 8'h01}; 
67:	LUT_DATA = {16'h3630, 8'h63}; 
68:	LUT_DATA = {16'h3632, 8'h74}; 
69:	LUT_DATA = {16'h3633, 8'h63}; 
70:	LUT_DATA = {16'h3634, 8'hff}; 
71:	LUT_DATA = {16'h3635, 8'h44}; 
72:	LUT_DATA = {16'h3638, 8'h82}; 
73:	LUT_DATA = {16'h3639, 8'h74}; 
74:	LUT_DATA = {16'h363a, 8'h24}; 
75:	LUT_DATA = {16'h363b, 8'h00}; 
76:	LUT_DATA = {16'h3640, 8'h03}; 
77:	LUT_DATA = {16'h3658, 8'h9a}; 
78:	LUT_DATA = {16'h3663, 8'h88}; 
79:	LUT_DATA = {16'h3664, 8'h06}; 
80:	LUT_DATA = {16'h3c00, 8'h41}; 
81:	LUT_DATA = {16'h3d08, 8'h00}; 
82:	LUT_DATA = {16'h3e01, 8'h20}; 
83:	LUT_DATA = {16'h3e02, 8'h50}; 
84:	LUT_DATA = {16'h3e03, 8'h0b}; 
85:	LUT_DATA = {16'h3e08, 8'h02}; 
86:	LUT_DATA = {16'h3e09, 8'h20}; 
87:	LUT_DATA = {16'h3e0e, 8'h00}; 
88:	LUT_DATA = {16'h3e0f, 8'h15}; 
89:	LUT_DATA = {16'h3e14, 8'hb0}; 
90:	LUT_DATA = {16'h3f08, 8'h04}; 
91:	LUT_DATA = {16'h4501, 8'hc0}; 
92:	LUT_DATA = {16'h4502, 8'h16}; 
93:	LUT_DATA = {16'h5000, 8'h01}; 
94:	LUT_DATA = {16'h5050, 8'h0c}; 
95:	LUT_DATA = {16'h5b00, 8'h02}; 
96:	LUT_DATA = {16'h5b01, 8'h03}; 
97:	LUT_DATA = {16'h5b02, 8'h01}; 
98:	LUT_DATA = {16'h5b03, 8'h01}; 
99:	LUT_DATA = {16'h3039, 8'h44}; 
100:	LUT_DATA = {16'h3034, 8'h01}; 
101:	LUT_DATA = {16'h363a, 8'h24}; 
102:	LUT_DATA = {16'h3630, 8'h63}; 
103:	LUT_DATA = {16'h3652, 8'h44}; 
104:	LUT_DATA = {16'h3653, 8'h44}; 
105:	LUT_DATA = {16'h3654, 8'h44}; 
106:	LUT_DATA = {16'h0100, 8'h01}; 

		default:LUT_DATA    =   {16'h0000, 8'h00};
    endcase
end

endmodule

====================================================================================================
 [23/29] src/cmos_i2c/i2c_timing_ctrl_16bit.v    (374 行, 未例化)
====================================================================================================
/*-----------------------------------------------------------------------
								 \\\|///
							   \\  - -  //
								(  @ @  )
+-----------------------------oOOo-(_)-oOOo-----------------------------+
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2013-20xx CrazyBingo Corporation
The entire notice above must be reproduced on all authorized copies.
Author				:		CrazyBingo
Technology blogs 	: 		http://blog.chinaaet.com/crazybingo
Email Address 		: 		thereturnofbingo@gmail.com
Filename			:		i2c_timing_ctrl.v
Data				:		2013-04-13
Description			:		The testbench of I2C Ctrl timing.
Modification History	:
Data			By			Version			Change Description
=========================================================================
13/04/13		CrazyBingo	1.0				Original
13/04/14		CrazyBingo	1.1				add i2c_wdata for avoid bit width warming
14/05/13		CrazyBingo	2.0				Change to 16Bit Data module
-------------------------------------------------------------------------
|                                     Oooo								|
+-------------------------------oooO--(   )-----------------------------+
                               (   )   ) /
                                \ (   (_/
                                 \_)
-----------------------------------------------------------------------*/
`timescale 1ns/1ns
module	i2c_timing_ctrl_16bit
#(
	parameter	CLK_FREQ	= 100_000000,	//100 MHz
	parameter	I2C_FREQ	= 400_000		//10 KHz(< 400KHz)
)
(
	//global clock
	input				clk,			//100MHz
	input				rst_n,		//system reset
	
	//i2c interface
	output			i2c_sclk,		//i2c clock
	// inout			i2c_sdat,		//i2c data for bidirection
	input               	i2c_sdat_IN,
	output              	i2c_sdat_OUT,
	output              	i2c_sdat_OE,

	//user interface
	input		[7:0]		i2c_config_size,	//i2c config data counte
	output reg	[7:0]		i2c_config_index,	//i2c config reg index, read 2 reg and write xx reg
	input		[31:0]	i2c_config_data,	//i2c config data
	output			i2c_config_done,	//i2c config timing complete
	output reg	[15:0]	i2c_rdata		//i2c register data while read i2c slave
);

//----------------------------------------
//Delay xxus until i2c slave is steady
reg [19:0]  delay_cnt;
//localparam  DELAY_TOP = SIMU_EN ? 20'hff : CLK_FREQ/1000;  //1ms Setting time after software/hardware reset
localparam  DELAY_TOP = CLK_FREQ/1000;  //1ms Setting time after software/hardware reset
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        delay_cnt <= 0;
    else if(delay_cnt < DELAY_TOP)
        delay_cnt <= delay_cnt + 1'b1;
    else
        delay_cnt <= delay_cnt;
end
wire    delay_done = (delay_cnt == DELAY_TOP) ? 1'b1 : 1'b0;    //81us delay


//----------------------------------------
//I2C Control Clock generate
reg [15:0]  clk_cnt;    //divide for i2c clock
/******************************************
             _______          _______
SCLK    ____|       |________|       |
         ________________ ______________
SDAT    |________________|______________
         _                _
CLK_EN  | |______________| |____________
                _                _
CAP_EN  _______| |______________| |_____
*******************************************/
reg i2c_ctrl_clk;       //i2c control clock, H: valid; L: valid
reg i2c_transfer_en;    //send i2c data before, make sure that sdat is steady when i2c_sclk is valid
reg i2c_capture_en;     //capture i2c data  while sdat is steady from cmos              
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
        clk_cnt <= 0;
        i2c_ctrl_clk <= 0;
        i2c_transfer_en <= 0;
        i2c_capture_en <= 0;
        end
    else if(delay_done)
        begin
        if(clk_cnt < (CLK_FREQ/I2C_FREQ) - 1'b1)
            clk_cnt <= clk_cnt + 1'd1;
        else
            clk_cnt <= 0;
        //i2c control clock, H: valid; L: valid
        i2c_ctrl_clk <= ((clk_cnt >= (CLK_FREQ/I2C_FREQ)/4 + 1'b1) &&
                        (clk_cnt < (3*CLK_FREQ/I2C_FREQ)/4 + 1'b1)) ? 1'b1 : 1'b0;
        //send i2c data before, make sure that sdat is steady when i2c_sclk is valid
        i2c_transfer_en <= (clk_cnt == 16'd0) ? 1'b1 : 1'b0;
        //capture i2c data  while sdat is steady from cmos                  
        i2c_capture_en <= (clk_cnt == (2*CLK_FREQ/I2C_FREQ)/4 - 1'b1) ? 1'b1 : 1'b0;
        end
    else
        begin
        clk_cnt <= 0;
        i2c_ctrl_clk <= 0;
        i2c_transfer_en <= 0;
        i2c_capture_en <= 0;
        end
end

//-----------------------------------------
//I2C Timing state Parameter
localparam  I2C_IDLE        =   4'd0;
//Write I2C: {ID_Address, REG_Address, W_REG_Data}
localparam  I2C_WR_START    =   4'd1;
localparam  I2C_WR_IDADDR   =   4'd2;
localparam  I2C_WR_ACK1     =   4'd3;
localparam  I2C_WR_REGADDR1 =   4'd4;
localparam  I2C_WR_ACK2     =   4'd5;
localparam  I2C_WR_REGADDR2 =   4'd6;
localparam  I2C_WR_ACK3     =   4'd7;
localparam  I2C_WR_REGDATA  =   4'd8;
localparam  I2C_WR_ACK4     =   4'd9;
localparam  I2C_WR_STOP     =   4'd10;


//-----------------------------------------
// FSM: always1
reg	[3:0]	current_state, next_state; //i2c write and read state  
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        current_state <= I2C_IDLE;
    else if(i2c_transfer_en)
        current_state <= next_state;
end

                                
//-----------------------------------------
wire    i2c_transfer_end = (current_state == I2C_WR_STOP) ? 1'b1 : 1'b0;
reg     i2c_ack;    //i2c slave renpose successed
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        i2c_config_index <= 0;
    else if(i2c_transfer_en)
        begin
		//	Force always step following. 
		if(i2c_transfer_end)	//	 & ~i2c_ack)
//		if(i2c_transfer_end /*& ~i2c_ack*/)											//Just for test
            begin
            if(i2c_config_index < i2c_config_size)  
                i2c_config_index <= i2c_config_index + 1'b1;
//              i2c_config_index <= {i2c_config_index[7:1], ~i2c_config_index[0]};  //Just for test, Read Only
            else
                i2c_config_index <= i2c_config_size;
            end
        else
            i2c_config_index <= i2c_config_index;
        end
    else
        i2c_config_index <= i2c_config_index;
end
assign  i2c_config_done = (i2c_config_index == i2c_config_size) ? 1'b1 : 1'b0;


//-----------------------------------------
// FSM: always2
reg [3:0]   i2c_stream_cnt; //i2c data bit stream count
always@(*)
begin
    next_state = I2C_IDLE;  //state initialization
    case(current_state)
    I2C_IDLE:       //5'd0
        begin
        if(delay_done == 1'b1)  //1ms Setting time after software/hardware reset    
            begin
            if(i2c_transfer_en)
                begin                
                if(i2c_config_index < i2c_config_size)
                    next_state = I2C_WR_START;  //Write Data to I2C
                else// if(i2c_config_index >= i2c_config_size)
                    next_state = I2C_IDLE;      //Config I2C Complete
					 end
            else
                next_state = next_state;
            end
        else
                next_state = I2C_IDLE;      //Wait I2C Bus is steady
        end
    //Write I2C: {ID_Address, REG_Address, W_REG_Data}
    I2C_WR_START:   
        if(i2c_transfer_en) next_state = I2C_WR_IDADDR;
        else                next_state = I2C_WR_START;
    I2C_WR_IDADDR:  
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK1;
        else                next_state = I2C_WR_IDADDR;
    I2C_WR_ACK1:    
        if(i2c_transfer_en) next_state = I2C_WR_REGADDR1;
        else                next_state = I2C_WR_ACK1;        
    I2C_WR_REGADDR1: 
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK2;
        else                next_state = I2C_WR_REGADDR1;
    I2C_WR_ACK2:    
        if(i2c_transfer_en) next_state = I2C_WR_REGADDR2;
        else                next_state = I2C_WR_ACK2;
    I2C_WR_REGADDR2: 
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK3;
        else                next_state = I2C_WR_REGADDR2;        
    I2C_WR_ACK3:    
        if(i2c_transfer_en) next_state = I2C_WR_REGDATA;
        else                next_state = I2C_WR_ACK3;        
    I2C_WR_REGDATA:    
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK4;
        else                next_state = I2C_WR_REGDATA;
    I2C_WR_ACK4:    
        if(i2c_transfer_en) next_state = I2C_WR_STOP;
        else                next_state = I2C_WR_ACK4;

    I2C_WR_STOP:    
        if(i2c_transfer_en) next_state = I2C_IDLE;
        else                next_state = I2C_WR_STOP;

    default:;   //default vaule     
    endcase
end


//-----------------------------------------
// FSM: always3
//reg   i2c_write_flag, i2c_read_flag;
reg i2c_sdat_out;       //i2c data output
//reg   [3:0]   i2c_stream_cnt; //i2c data bit stream count
reg [7:0]   i2c_wdata;  //i2c data prepared to transfer
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
        i2c_sdat_out <= 1'b1;
        i2c_stream_cnt <= 0;
        i2c_wdata <= 0;
        end
    else if(i2c_transfer_en)
        begin
        case(next_state)
        I2C_IDLE:   
            begin
            i2c_sdat_out <= 1'b1;       //idle state
            i2c_stream_cnt <= 0;
            i2c_wdata <= 0;
            end
        //Write I2C: {ID_Address, REG_Address, W_REG_Data}
        I2C_WR_START:   
            begin
            i2c_sdat_out <= 1'b0;
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[31:24];    //ID_Address
            end
        I2C_WR_IDADDR:  
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_WR_ACK1:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[23:16];        //REG_Address
            end            
        I2C_WR_REGADDR1: 
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_WR_ACK2:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[15:8];        //REG_Address
            end
        I2C_WR_REGADDR2: 
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end         
        I2C_WR_ACK3:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[7:0];     //W_REG_Data1
            end            
        I2C_WR_REGDATA:    
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_WR_ACK4:    
            i2c_stream_cnt <= 0;

        I2C_WR_STOP:    
            i2c_sdat_out <= 1'b0;

        endcase
        end
    else
        begin
        i2c_stream_cnt <= i2c_stream_cnt;
        i2c_sdat_out <= i2c_sdat_out;
        i2c_wdata <= i2c_wdata;
        end
end




//---------------------------------------------
//respone from slave for i2c data transfer
reg i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
		{i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4} <= 4'b1111;
        i2c_ack <= 1'b1;
        end
    else if(i2c_capture_en)
        begin
        case(next_state)
        I2C_IDLE:
        begin
        {i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4} <= 4'b1111;
        i2c_ack <= 1'b1;
        end
        //Write I2C: {ID_Address, REG_Address, W_REG_Data}
        I2C_WR_ACK1:    i2c_ack1 <= i2c_sdat_IN;
        I2C_WR_ACK2:    i2c_ack2 <= i2c_sdat_IN;
        I2C_WR_ACK3:    i2c_ack3 <= i2c_sdat_IN;
        I2C_WR_ACK4:    i2c_ack4 <= i2c_sdat_IN;
        I2C_WR_STOP:    i2c_ack <= (i2c_ack1 | i2c_ack2 | i2c_ack3);// | i2c_ack4 );

        default:;
        endcase
        end
    else
        begin
        {i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4} <= {i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4};
        i2c_ack <= i2c_ack;
        end
end 

//---------------------------------------------------
wire    bir_en =(   current_state == I2C_WR_ACK1 || current_state == I2C_WR_ACK2 || 
	            current_state == I2C_WR_ACK3 || current_state == I2C_WR_ACK4) ? 1'b1 : 1'b0;
assign  i2c_sclk = (current_state >= I2C_WR_IDADDR && current_state <= I2C_WR_ACK4) ? 
                    i2c_ctrl_clk : 1'b1;
//assign  i2c_sdat = (~bir_en) ? i2c_sdat_out : 1'bz;
assign i2c_sdat_OUT = i2c_sdat_out;
assign i2c_sdat_OE = ~bir_en;


endmodule

====================================================================================================
 [24/29] src/cmos_i2c/i2c_timing_ctrl_reg16_dat16.v    (566 行, 未例化)
====================================================================================================
/*-----------------------------------------------------------------------
                                 \\\|///
                                \\  - -  //
                                (  @ @  )
+-----------------------------oOOo-(_)-oOOo-----------------------------+
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2012-20xx CrazyBingo Corporation.
The entire notice above must be reproduced on all authorized copies.
Author                  :       CrazyBingo
Official Websites       :       http://www.crazyfpga.com
Email Address           :       crazyfpga@qq.com
Filename                :       module  i2c_timing_ctrl_reg16_dat16.v
Data                    :       2013-04-13
Description             :       The function of the Template.
Modification History    :
Data            Author          Version     Change Description
=========================================================================
13/04/13        CrazyBingo      1.0         Original
13/04/14        CrazyBingo      1.1         add i2c_wdata for avoid bit width warming
14/05/13        CrazyBingo      2.0         Change to 16Bit Data module
-------------------------------------------------------------------------
|                                     Oooo                              |
+------------------------------oooO--(   )------------------------------+
                              (   )   ) /
                               \ (   (_/
                                \_)
-----------------------------------------------------------------------*/

`timescale 1ns/1ns
module  i2c_timing_ctrl_reg16_dat16
#(
    parameter   SIMU_EN     =   0,
    parameter   I2C_READ_EN =   1,          //Enable i2c read I2C register
    parameter   CLK_FREQ    =   100_000000, //100 MHz
    parameter   I2C_FREQ    =   400_000     //10 KHz(< 400KHz)
)
(
    //global clock
    input               clk,                //100MHz
    input               rst_n,              //system reset
            
    //i2c interface     
    output              i2c_sclk,           //i2c clock
    //inout               i2c_sdat,           //i2c data for bidirection
    output 			i2c_sdat_o, 
    output 			i2c_sdat_oe, 
    input 			i2c_sdat_i, 

    //user interface
    input       [7:0]   i2c_config_size,    //i2c config data counte
    output  reg [7:0]   i2c_config_index,   //i2c config reg index, read 2 reg and write xx reg
    input       [39:0]  i2c_config_data,    //i2c config data
    output              i2c_config_done,    //i2c config timing complete
    output  reg [15:0]  i2c_rdata           //i2c register data while read i2c slave
);

//----------------------------------------
//Delay xxus until i2c slave is steady
reg [19:0]  delay_cnt;
localparam  DELAY_TOP = SIMU_EN ? 20'hff : CLK_FREQ/1000;  //1ms Setting time after software/hardware reset
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        delay_cnt <= 0;
    else if(delay_cnt < DELAY_TOP)
        delay_cnt <= delay_cnt + 1'b1;
    else
        delay_cnt <= delay_cnt;
end
wire    delay_done = (delay_cnt == DELAY_TOP) ? 1'b1 : 1'b0;    //81us delay


//----------------------------------------
//I2C Control Clock generate
reg [15:0]  clk_cnt;    //divide for i2c clock
/******************************************
             _______          _______
SCLK    ____|       |________|       |
         ________________ ______________
SDAT    |________________|______________
         _                _
CLK_EN  | |______________| |____________
                _                _
CAP_EN  _______| |______________| |_____
*******************************************/
reg i2c_ctrl_clk;       //i2c control clock, H: valid; L: valid
reg i2c_transfer_en;    //send i2c data before, make sure that sdat is steady when i2c_sclk is valid
reg i2c_capture_en;     //capture i2c data  while sdat is steady from cmos              
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
        clk_cnt <= 0;
        i2c_ctrl_clk <= 0;
        i2c_transfer_en <= 0;
        i2c_capture_en <= 0;
        end
    else if(delay_done)
        begin
        if(clk_cnt < (CLK_FREQ/I2C_FREQ) - 1'b1)
            clk_cnt <= clk_cnt + 1'd1;
        else
            clk_cnt <= 0;
        //i2c control clock, H: valid; L: valid
        i2c_ctrl_clk <= ((clk_cnt >= (CLK_FREQ/I2C_FREQ)/4 + 1'b1) &&
                        (clk_cnt < (3*CLK_FREQ/I2C_FREQ)/4 + 1'b1)) ? 1'b1 : 1'b0;
        //send i2c data before, make sure that sdat is steady when i2c_sclk is valid
        i2c_transfer_en <= (clk_cnt == 16'd0) ? 1'b1 : 1'b0;
        //capture i2c data  while sdat is steady from cmos                  
        i2c_capture_en <= (clk_cnt == (2*CLK_FREQ/I2C_FREQ)/4 - 1'b1) ? 1'b1 : 1'b0;
        end
    else
        begin
        clk_cnt <= 0;
        i2c_ctrl_clk <= 0;
        i2c_transfer_en <= 0;
        i2c_capture_en <= 0;
        end
end

//-----------------------------------------
//I2C Timing state Parameter
localparam  I2C_IDLE        =   5'd0;
//Write I2C: {ID_Address, REG_Address, W_REG_Data}
localparam  I2C_WR_START    =   5'd1;
localparam  I2C_WR_IDADDR   =   5'd2;
localparam  I2C_WR_ACK1     =   5'd3;
localparam  I2C_WR_REGADDR1 =   5'd4;
localparam  I2C_WR_ACK2     =   5'd5;
localparam  I2C_WR_REGADDR2 =   5'd6;
localparam  I2C_WR_ACK3     =   5'd7;
localparam  I2C_WR_REGDATA1 =   5'd8;
localparam  I2C_WR_ACK4     =   5'd9;
localparam  I2C_WR_REGDATA2 =   5'd10;   //16Bit Mode
localparam  I2C_WR_ACK5     =   5'd11;   //16Bit Mode
localparam  I2C_WR_STOP     =   5'd12;
//I2C Read: {ID_Address + REG_Address} + {ID_Address + R_REG_Data}
localparam  I2C_RD_START1   =   5'd13;      
localparam  I2C_RD_IDADDR1  =   5'd14;
localparam  I2C_RD_ACK1     =   5'd15;
localparam  I2C_RD_REGADDR1 =   5'd16;
localparam  I2C_RD_ACK2     =   5'd17;
localparam  I2C_RD_REGADDR2 =   5'd18;
localparam  I2C_RD_ACK3     =   5'd19;
localparam  I2C_RD_STOP1    =   5'd20;
localparam  I2C_RD_IDLE     =   5'd21;
localparam  I2C_RD_START2   =   5'd22;
localparam  I2C_RD_IDADDR2  =   5'd23;
localparam  I2C_RD_ACK4     =   5'd24;
localparam  I2C_RD_REGDATA1 =   5'd25;
localparam  I2C_RD_NACK1    =   5'd26;  //16Bit Mode--Read
localparam  I2C_RD_REGDATA2 =   5'd27;  //16Bit Mode--Read
localparam  I2C_RD_NACK2    =   5'd28;
localparam  I2C_RD_STOP2    =   5'd29;

//-----------------------------------------
// FSM: always1
reg [4:0]   current_state, next_state; //i2c write and read state  
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        current_state <= I2C_IDLE;
    else if(i2c_transfer_en)
        current_state <= next_state;
end

//wire    [1:0]   I2C_STATE_flag = current_state == I2C_IDLE ? 2'd0 :
//                                (current_state >= I2C_WR_START &&  current_state <= I2C_WR_STOP) ? 2'd1 :
//                                (current_state >= I2C_RD_START1 &&  current_state <= I2C_RD_STOP2) ? 2'd2 : 2'd3;
                                
//-----------------------------------------
wire    i2c_transfer_end = (current_state == I2C_WR_STOP || current_state == I2C_RD_STOP2) ? 1'b1 : 1'b0;
reg     i2c_ack;    //i2c slave renpose successed
wire    i2c_transfer_end_flag = SIMU_EN ? i2c_transfer_end : (i2c_transfer_end & ~i2c_ack);
//wire    i2c_transfer_end_flag = i2c_transfer_end;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        i2c_config_index <= 0;
    else if(i2c_transfer_en)
        begin
        if(i2c_transfer_end_flag == 1'b1)
            begin
            if(i2c_config_index < i2c_config_size)  
                i2c_config_index <= i2c_config_index + 1'b1;
//              i2c_config_index <= {i2c_config_index[7:1], ~i2c_config_index[0]};  //Just for test, Read Only
            else
                i2c_config_index <= i2c_config_size;
            end
        else
            i2c_config_index <= i2c_config_index;
        end
    else
        i2c_config_index <= i2c_config_index;
end
assign  i2c_config_done = (i2c_config_index == i2c_config_size) ? 1'b1 : 1'b0;


//-----------------------------------------
// FSM: always2
reg [3:0]   i2c_stream_cnt; //i2c data bit stream count
always@(*)
begin
    next_state = I2C_IDLE;  //state initialization
    case(current_state)
    I2C_IDLE:       //5'd0
        begin
        if(delay_done == 1'b1)  //1ms Setting time after software/hardware reset    
            begin
            if(i2c_transfer_en)
                begin
                if(I2C_READ_EN == 1)    //Read i2c Register
                    begin
                    if(i2c_config_index < 8'd1)
                        next_state = I2C_RD_START1; //Read I2C Slave ID
                    else if(i2c_config_index < i2c_config_size)
                        next_state = I2C_WR_START;  //Write Data to I2C
                    else// if(i2c_config_index >= i2c_config_size)
                        next_state = I2C_IDLE;      //Config I2C Complete
                    end
                else                    //Do not read i2c Register
                    begin
                    if(i2c_config_index < i2c_config_size)
                        next_state = I2C_WR_START;  //Write Data to I2C
                    else// if(i2c_config_index >= i2c_config_size)
                        next_state = I2C_IDLE;      //Config I2C Complete
                    end
                end
            else
                next_state = next_state;
            end
        else
                next_state = I2C_IDLE;      //Wait I2C Bus is steady
        end
    //Write I2C: {ID_Address, REG_Address, W_REG_Data}
    I2C_WR_START:   
        if(i2c_transfer_en) next_state = I2C_WR_IDADDR;
        else                next_state = I2C_WR_START;
    I2C_WR_IDADDR:  
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK1;
        else                next_state = I2C_WR_IDADDR;
    I2C_WR_ACK1:    
        if(i2c_transfer_en) next_state = I2C_WR_REGADDR1;
        else                next_state = I2C_WR_ACK1;        
    I2C_WR_REGADDR1: 
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK2;
        else                next_state = I2C_WR_REGADDR1;
    I2C_WR_ACK2:    
        if(i2c_transfer_en) next_state = I2C_WR_REGADDR2;
        else                next_state = I2C_WR_ACK2;
    I2C_WR_REGADDR2: 
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK3;
        else                next_state = I2C_WR_REGADDR2;        
    I2C_WR_ACK3:    
        if(i2c_transfer_en) next_state = I2C_WR_REGDATA1;
        else                next_state = I2C_WR_ACK3;        
    I2C_WR_REGDATA1:    
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK4;
        else                next_state = I2C_WR_REGDATA1;
    I2C_WR_ACK4:    
        if(i2c_transfer_en) next_state = I2C_WR_REGDATA2;
        else                next_state = I2C_WR_ACK4;
    I2C_WR_REGDATA2:    
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_WR_ACK5;
        else                next_state = I2C_WR_REGDATA2;
    I2C_WR_ACK5:    
        if(i2c_transfer_en) next_state = I2C_WR_STOP;
        else                next_state = I2C_WR_ACK5;
    I2C_WR_STOP:    
        if(i2c_transfer_en) next_state = I2C_IDLE;
        else                next_state = I2C_WR_STOP;
    //I2C Read: {ID_Address + REG_Address} + {ID_Address + R_REG_Data}
    I2C_RD_START1:  
        if(i2c_transfer_en) next_state = I2C_RD_IDADDR1;
        else                next_state = I2C_RD_START1;
    I2C_RD_IDADDR1: 
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_RD_ACK1;
        else                next_state = I2C_RD_IDADDR1;
    I2C_RD_ACK1:    
        if(i2c_transfer_en) next_state = I2C_RD_REGADDR1;
        else                next_state = I2C_RD_ACK1;
    I2C_RD_REGADDR1: 
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_RD_ACK2;
        else                next_state = I2C_RD_REGADDR1;
    I2C_RD_ACK2:    
        if(i2c_transfer_en) next_state = I2C_RD_REGADDR2;
        else                next_state = I2C_RD_ACK2;
    I2C_RD_REGADDR2: 
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_RD_ACK3;
        else                next_state = I2C_RD_REGADDR2;
    I2C_RD_ACK3:    
        if(i2c_transfer_en) next_state = I2C_RD_STOP1;
        else                next_state = I2C_RD_ACK3;     
    I2C_RD_STOP1:   
        if(i2c_transfer_en) next_state = I2C_RD_IDLE;
        else                next_state = I2C_RD_STOP1;
    I2C_RD_IDLE:    
        if(i2c_transfer_en) next_state = I2C_RD_START2;
        else                next_state = I2C_RD_IDLE;
    I2C_RD_START2:  
        if(i2c_transfer_en) next_state = I2C_RD_IDADDR2;
        else                next_state = I2C_RD_START2;
    I2C_RD_IDADDR2:
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_RD_ACK4;
        else                next_state = I2C_RD_IDADDR2;
    I2C_RD_ACK4:    
        if(i2c_transfer_en) next_state = I2C_RD_REGDATA1;
        else                next_state = I2C_RD_ACK4;
    I2C_RD_REGDATA1:    
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_RD_NACK1;
        else                next_state = I2C_RD_REGDATA1;
    I2C_RD_NACK1:   
        if(i2c_transfer_en) next_state = I2C_RD_REGDATA2;
        else                next_state = I2C_RD_NACK1;
    I2C_RD_REGDATA2:    
        if(i2c_transfer_en == 1'b1 && i2c_stream_cnt == 4'd8)   
                            next_state = I2C_RD_NACK2;
        else                next_state = I2C_RD_REGDATA2;       
    I2C_RD_NACK2:   
        if(i2c_transfer_en) next_state = I2C_RD_STOP2;
        else                next_state = I2C_RD_NACK2;
    I2C_RD_STOP2:   
        if(i2c_transfer_en) next_state = I2C_IDLE;
        else                next_state = I2C_RD_STOP2;
    default:;   //default vaule     
    endcase
end


//-----------------------------------------
// FSM: always3
//reg   i2c_write_flag, i2c_read_flag;
reg i2c_sdat_out;       //i2c data output
//reg   [3:0]   i2c_stream_cnt; //i2c data bit stream count
reg [7:0]   i2c_wdata;  //i2c data prepared to transfer
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
        i2c_sdat_out <= 1'b1;
        i2c_stream_cnt <= 0;
        i2c_wdata <= 0;
        end
    else if(i2c_transfer_en)
        begin
        case(next_state)
        I2C_IDLE:   
            begin
            i2c_sdat_out <= 1'b1;       //idle state
            i2c_stream_cnt <= 0;
            i2c_wdata <= 0;
            end
        //Write I2C: {ID_Address, REG_Address, W_REG_Data}
        I2C_WR_START:   
            begin
            i2c_sdat_out <= 1'b0;
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[39:32];    //ID_Address
            end
        I2C_WR_IDADDR:  
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_WR_ACK1:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[31:24];        //REG_Address
            end            
        I2C_WR_REGADDR1: 
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_WR_ACK2:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[23:16];        //REG_Address
            end
        I2C_WR_REGADDR2: 
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end         
        I2C_WR_ACK3:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[15:8];     //W_REG_Data1
            end            
        I2C_WR_REGDATA1:    
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_WR_ACK4:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[7:0];      //W_REG_Data2
            end
        I2C_WR_REGDATA2:    
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_WR_ACK5:    
            i2c_stream_cnt <= 0;
        I2C_WR_STOP:    
            i2c_sdat_out <= 1'b0;
        //I2C Read: {ID_Address + REG_Address} + {ID_Address + R_REG_Data}
        I2C_RD_START1:  
            begin
            i2c_sdat_out <= 1'b0;
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[39:32];
            end
        I2C_RD_IDADDR1: 
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            end
        I2C_RD_ACK1:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[31:24];
            end
        I2C_RD_REGADDR1: 
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];   
            end
        I2C_RD_ACK2:    
            begin
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[23:16];
            end
        I2C_RD_REGADDR2: 
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];   
            end           
        I2C_RD_ACK3:    
            i2c_stream_cnt <= 0;
        I2C_RD_STOP1:   
            i2c_sdat_out <= 1'b0;
        I2C_RD_IDLE:    
            i2c_sdat_out <= 1'b1;       //idle state
        //-------------------------
        I2C_RD_START2:  
            begin
            i2c_sdat_out <= 1'b0;
            i2c_stream_cnt <= 0;
            i2c_wdata <= i2c_config_data[39:32];    
            end
        I2C_RD_IDADDR2: 
            begin
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
            if(i2c_stream_cnt < 5'd7)
                i2c_sdat_out <= i2c_wdata[3'd7 - i2c_stream_cnt];
            else
                i2c_sdat_out <= 1'b1;   //Read flag for I2C Timing
            end
        I2C_RD_ACK4:    
            i2c_stream_cnt <= 0;
        I2C_RD_REGDATA1:    
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
        I2C_RD_NACK1:
            begin
            i2c_stream_cnt <= 0;
            i2c_sdat_out <= 1'b0;   //NACK1, It is very important!
            end
        I2C_RD_REGDATA2:    
            i2c_stream_cnt <= i2c_stream_cnt + 1'b1;
        I2C_RD_NACK2:   
            i2c_sdat_out <= 1'b1;   //NACK2
        I2C_RD_STOP2:   
            i2c_sdat_out <= 1'b0;
        endcase
        end
    else
        begin
        i2c_stream_cnt <= i2c_stream_cnt;
        i2c_sdat_out <= i2c_sdat_out;
        i2c_wdata <= i2c_wdata;
        end
end




//---------------------------------------------
//respone from slave for i2c data transfer
reg i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4, i2c_ack5;
reg [15:0]  i2c_rdata_r;
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n)
        begin
        {i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4, i2c_ack5} <= 5'b11111;
        i2c_ack <= 1'b1;
        i2c_rdata_r <= 0;
        i2c_rdata <= 0;
        end
    else if(i2c_capture_en)
        begin
        case(next_state)
        I2C_IDLE:
        begin
        {i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4, i2c_ack5} <= 5'b11111;
        i2c_ack <= 1'b1;
        end
        //Write I2C: {ID_Address, REG_Address, W_REG_Data}
        I2C_WR_ACK1:    i2c_ack1 <= i2c_sdat_i;
        I2C_WR_ACK2:    i2c_ack2 <= i2c_sdat_i;
        I2C_WR_ACK3:    i2c_ack3 <= i2c_sdat_i;
        I2C_WR_ACK4:    i2c_ack4 <= i2c_sdat_i;
        I2C_WR_ACK5:    i2c_ack5 <= i2c_sdat_i;
        I2C_WR_STOP:    i2c_ack <= (i2c_ack1 | i2c_ack2 | i2c_ack3 | i2c_ack4 | i2c_ack5);
        //I2C Read: {ID_Address + REG_Address} + {ID_Address + R_REG_Data}
        I2C_RD_ACK1:    i2c_ack1 <= i2c_sdat_i;
        I2C_RD_ACK2:    i2c_ack2 <= i2c_sdat_i;
        I2C_RD_ACK3:    i2c_ack3 <= i2c_sdat_i;
        I2C_RD_ACK4:    i2c_ack4 <= i2c_sdat_i;
        I2C_RD_STOP2:   i2c_ack <= (i2c_ack1 | i2c_ack2 | i2c_ack3 | i2c_ack4);
        I2C_RD_REGDATA1:    i2c_rdata_r[15:8] <= {i2c_rdata_r[14:8], i2c_sdat_i};
        I2C_RD_REGDATA2:    i2c_rdata_r[7:0]  <= {i2c_rdata_r[6:0], i2c_sdat_i};
        I2C_RD_NACK2:       i2c_rdata <= i2c_rdata_r;   //Update iec_rdata;
        default:;
        endcase
        end
    else
        begin
        {i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4, i2c_ack5} <= {i2c_ack1, i2c_ack2, i2c_ack3, i2c_ack4, i2c_ack5};
        i2c_ack <= i2c_ack;
        i2c_rdata <= i2c_rdata;
        end
end 

//---------------------------------------------------
wire    bir_en =(   current_state == I2C_WR_ACK1 || current_state == I2C_WR_ACK2 || current_state == I2C_WR_ACK3 || current_state == I2C_WR_ACK4 || current_state == I2C_WR_ACK5 ||
                    current_state == I2C_RD_ACK1 || current_state == I2C_RD_ACK2 || current_state == I2C_RD_ACK3 || current_state == I2C_RD_ACK4 || 
                    current_state == I2C_RD_REGDATA1 || current_state == I2C_RD_REGDATA2) ? 1'b1 : 1'b0;
assign  i2c_sclk = (current_state >= I2C_WR_IDADDR && current_state <= I2C_WR_ACK5 ||
                    current_state >= I2C_RD_IDADDR1 && current_state <= I2C_RD_ACK3 ||
                    current_state >= I2C_RD_IDADDR2 && current_state <= I2C_RD_NACK2) ? 
                    i2c_ctrl_clk : 1'b1;
			  
//assign  i2c_sdat = (~bir_en) ? i2c_sdat_out : 1'bz;
assign i2c_sdat_o = i2c_sdat_out; 
assign i2c_sdat_oe = (~bir_en); 


endmodule

====================================================================================================
 [25/29] src/ddr_rw_ctrl.v    (341 行, 未例化)
====================================================================================================
//  by CrazyBird
module ddr_rw_ctrl
(
    input  wire                 ddr_clk         ,
    input  wire                 ddr_rst         ,
    
    input  wire                 ddr_wr_busy     ,
    output wire     [127:0]     ddr_wr_data     ,
    output wire     [ 15:0]     ddr_wr_mask     ,
    output wire     [ 31:0]     ddr_wr_addr     ,
    output reg                  ddr_wr_en       ,
    input  wire                 ddr_wr_ack      ,
    
    input  wire                 ddr_rd_busy     ,
    output wire     [ 31:0]     ddr_rd_addr     ,
    output reg                  ddr_rd_en       ,
    input  wire     [127:0]     ddr_rd_data     ,
    input  wire                 ddr_rd_valid    ,
    
    input  wire                 wframe_pclk     ,
    input  wire                 wframe_vsync    ,
    input  wire                 wframe_data_en  ,
    input  wire     [ 63:0]     wframe_data     ,
    
    input  wire                 rframe_pclk     ,
    input  wire                 rframe_vsync    ,
    input  wire                 rframe_data_en  ,
    output wire     [ 15:0]     rframe_data     
);
//----------------------------------------------------------------------
parameter C_BURST_LEN = 32;
parameter C_RD_END_ADDR = 38400;                                        //  1024*600/16
assign ddr_wr_mask = 16'b0;

//----------------------------------------------------------------------
reg             [3:0]           wframe_vsync_dly;
reg             [3:0]           rframe_vsync_dly;

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
    begin
        wframe_vsync_dly <= 4'b0;
        rframe_vsync_dly <= 4'b0;
    end
    else
    begin
        wframe_vsync_dly <= {wframe_vsync_dly[2:0],wframe_vsync};
        rframe_vsync_dly <= {rframe_vsync_dly[2:0],rframe_vsync};
    end
end

reg             [4:0]           wfifo_cnt;

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        wfifo_cnt <= 5'h1f;
    else
    begin
        if(wframe_vsync_dly[3:2] == 2'b01)
            wfifo_cnt <= 5'h1f;
        else if(wfifo_cnt > 5'd0)
            wfifo_cnt <= wfifo_cnt - 1'b1;
        else
            wfifo_cnt <= 5'b0;
    end
end

reg             [4:0]           rfifo_cnt;

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        rfifo_cnt <= 5'h1f;
    else
    begin
        if(rframe_vsync_dly[3:2] == 2'b10)
            rfifo_cnt <= 5'h1f;
        else if(rfifo_cnt > 5'd0)
            rfifo_cnt <= rfifo_cnt - 1'b1;
        else
            rfifo_cnt <= 5'b0;
    end
end

reg                             wfifo_rst;

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        wfifo_rst <= 1'b1;
    else
    begin
        if(wfifo_cnt == 5'b0)
            wfifo_rst <= 1'b0;
        else
            wfifo_rst <= 1'b1;
    end
end

reg                             rfifo_rst;

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        rfifo_rst <= 1'b1;
    else
    begin
        if(rfifo_cnt == 5'b0)
            rfifo_rst <= 1'b0;
        else
            rfifo_rst <= 1'b1;
    end
end

//----------------------------------------------------------------------
wire            [127:0]         wfifo_rdata;
wire                            wfifo_renb;
wire            [  9:0]         wfifo_rcnt;

W0_FIFO u_W0_FIFO
(
    .a_rst_i        (wfifo_rst      ),
    .rst_busy       (               ),
    
    .wr_clk_i       (wframe_pclk    ),
    .wdata          (wframe_data    ),
    .wr_en_i        (wframe_data_en ),
    .full_o         (               ),
    .wr_datacount_o (               ),
    
    .rd_clk_i       (ddr_clk        ),
    .rdata          (wfifo_rdata    ),
    .rd_en_i        (wfifo_renb     ),
    .empty_o        (               ),
    .rd_datacount_o (wfifo_rcnt     )
);

wire            [7:0]           rfifo_wcnt;

R0_FIFO u_R0_FIFO
(
    .a_rst_i        (rfifo_rst      ),
    .rst_busy       (               ),
    
    .wr_clk_i       (ddr_clk        ),
    .wdata          (ddr_rd_data    ),
    .wr_en_i        (ddr_rd_valid   ),
    .full_o         (               ),
    .wr_datacount_o (rfifo_wcnt     ),
    
    .rd_clk_i       (rframe_pclk    ),
    .rdata          (rframe_data    ),
    .rd_en_i        (rframe_data_en ),
    .empty_o        (               ),
    .rd_datacount_o (               )
);

//----------------------------------------------------------------------
reg             [1:0]               wframe_index;

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        wframe_index <= 2'b0;
    else
    begin
        if(wframe_vsync_dly[3:2] == 2'b10)
        begin
            if(wframe_index < 2'd2)
                wframe_index <= wframe_index + 1'b1;
            else
                wframe_index <= 2'b0;
        end
        else
            wframe_index <= wframe_index;
    end
end

reg             [1:0]               rframe_index;

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        rframe_index <= 2'd0;
    else
    begin
        if(rframe_vsync_dly[3:2] == 2'b10)
        begin
            if(wframe_index == 2'd0)
                rframe_index <= 2'd2;
            else
                rframe_index <= wframe_index - 1'b1;
        end
        else
            rframe_index <= rframe_index;
    end
end

//----------------------------------------------------------------------
//  state machine 
localparam S_IDLE  = 2'd0;
localparam S_WRITE = 2'd1;
localparam S_READ  = 2'd2;
localparam S_FINISH= 2'd3;

reg             [1:0]           state;
reg             [8:0]           wdata_cnt;
reg             [8:0]           rdata_cnt;

reg             [18:0]          raddr;

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        state <= S_IDLE;
    else
    begin
        case(state)
            S_IDLE : 
            begin
                if(wfifo_rcnt >= C_BURST_LEN)
                    state <= S_WRITE;
                else if((raddr < C_RD_END_ADDR)&&(rfifo_wcnt < 8'd64))
                    state <= S_READ;
                else
                    state <= S_IDLE;
            end
            S_WRITE : 
            begin
                if((ddr_wr_busy == 1'b0)&&(wdata_cnt == C_BURST_LEN - 1'b1))
                    state <= S_FINISH;
                else
                    state <= S_WRITE;
            end
            S_READ : 
            begin
                if((ddr_rd_busy == 1'b0)&&(rdata_cnt == C_BURST_LEN - 1'b1))
                    state <= S_FINISH;
                else
                    state <= S_READ;
            end
            S_FINISH : state <= S_IDLE;
        endcase
    end
end

always @(posedge ddr_clk)
begin
    if(state == S_WRITE)
    begin
        if(ddr_wr_busy == 1'b0)
            wdata_cnt <= wdata_cnt + 1'b1;
        else
            wdata_cnt <= wdata_cnt;
    end
    else
        wdata_cnt <= 9'b0;
end

always @(posedge ddr_clk)
begin
    if(state == S_READ)
    begin
        if(ddr_rd_busy == 1'b0)
            rdata_cnt <= rdata_cnt + 1'b1;
        else
            rdata_cnt <= rdata_cnt;
    end
    else
        rdata_cnt <= 9'b0;
end

reg             [18:0]          waddr;

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        waddr <= 19'b0;
    else
    begin
        if(wframe_vsync_dly[3:2] == 2'b01)
            waddr <= 19'b0;
        else if((state == S_WRITE)&&(ddr_wr_busy == 1'b0))
            waddr <= waddr + 1'b1;
        else
            waddr <= waddr;
    end
end

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        raddr <= 19'b0;
    else
    begin
        if(rframe_vsync_dly[3:2] == 2'b01)
            raddr <= 19'b0;
        else if((state == S_READ)&&(ddr_rd_busy == 1'b0))
            raddr <= raddr + 1'b1;
        else
            raddr <= raddr;
    end
end

assign ddr_wr_addr = {wframe_index,waddr};
assign ddr_wr_data = wfifo_rdata;

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        ddr_wr_en <= 1'b0;
    else
    begin
        if((state == S_WRITE)&&(ddr_wr_busy == 1'b0))
            ddr_wr_en <= 1'b1;
        else
            ddr_wr_en <= 1'b0;
    end
end

assign wfifo_renb = ddr_wr_en;

assign ddr_rd_addr = {rframe_index,raddr};

always @(posedge ddr_clk)
begin
    if(ddr_rst == 1'b1)
        ddr_rd_en <= 1'b0;
    else
    begin
        if((state == S_READ)&&(ddr_rd_busy == 1'b0))
            ddr_rd_en <= 1'b1;
        else
            ddr_rd_en <= 1'b0;
    end
end

endmodule

====================================================================================================
 [26/29] src/dsi/dsi_init.v    (99 行, 未例化)
====================================================================================================
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    21:44:36 05/15/2023 
// Module Name:    dsi_init 
//////////////////////////////////////////////////////////////////////////////////
module dsi_init(
	input 			clk_i,
	input 			rst_i,
	
	input 	[27:0] 	delay_i,
	
	output reg	[7:0] 	awaddr_o,
	output reg			awvalid_o,
	input 			awready_i,
	
	output reg			wvalid_o,
	output reg	[31:0] 	wdata_o,
	input 			wready_i
);
	
	
	reg 	[27:0] 	rc_init = 0; 
	reg 			r_ready = 0; 
	
	reg 	[3:0] 	rs_ctl = 0; 
	
	wire 	[3:0] 	w_regcnt = 9; 
	reg 	[3:0] 	rc_addr = 0; 
	reg 	[39:0] 	w_regdata; 
	always @(*) begin
		w_regdata <= 0; 
		case (rc_addr)
			0:	w_regdata <= 40'h40_000014C8; 
			1:	w_regdata <= 40'h44_00000006; 
			2:	w_regdata <= 40'h48_0000000C; 
			3:	w_regdata <= 40'h4C_000002E2; 
			4:	w_regdata <= 40'h50_00000005; 
			5:	w_regdata <= 40'h54_00000008; 
			6:	w_regdata <= 40'h58_00000020; 
			7:	w_regdata <= 40'h5C_00000258; 
			8:	w_regdata <= 40'h18_0000000A; 
		endcase
	end
	
	always @(posedge clk_i or posedge rst_i) begin
		if(rst_i) begin
			rc_init <= 0; 
			r_ready <= 0; 
			
			awvalid_o <= 0; 
			wvalid_o <= 0; 
			rs_ctl <= 0; 
			rc_addr <= 0; 
		end else begin
			rc_init <= rc_init + 1; 
			r_ready <= r_ready | (rc_init >= delay_i); 
			
			//	Clear awvalid & wvalid. 
			if(awready_i)
				awvalid_o <= 0; 
			else begin
			end
			
			if(wready_i)
				wvalid_o <= 0; 
			else begin
			end
			
			case (rs_ctl)
				0: begin
						awaddr_o <= w_regdata[39:32]; 
						wdata_o <= w_regdata[31: 0]; 
						awvalid_o <= 1; 
						wvalid_o <= 1; 
						rs_ctl <= 1; 
					end
				1: begin
						if((~awvalid_o) && (~wvalid_o)) begin
							rs_ctl <= 2; 
						end else begin
						end
					end
				2: begin
						//	Write regcnt words. 
						if(rc_addr < w_regcnt - 1) begin
							rc_addr <= rc_addr + 1; 
							rs_ctl <= 0; 
						end else begin
						end
					end
			endcase
		end
	end
	
	
	
	
endmodule

====================================================================================================
 [27/29] src/lcd_display.v    (186 行, 未例化)
====================================================================================================
/*-----------------------------------------------------------------------
								 \\\|///
							   \\  - -  //
								(  @ @  )
+-----------------------------oOOo-(_)-oOOo-----------------------------+
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2013-20xx CrazyBingo Corporation
The entire notice above must be reproduced on all authorized copies.
Author				:		CrazyBingo
Technology blogs 	: 		http://blog.chinaaet.com/crazybingo
Email Address 		: 		thereturnofbingo@gmail.com
Filename			:		lcd_display.v
Data				:		2012-02-18
Description			:		LCD/VGA display simulation.
Modification History	:
Data			By			Version			Change Description
=========================================================================
12/02/18		CrazyBingo	1.0				Original
12/03/19		CrazyBingo	1.1				Modification
12/03/21		CrazyBingo	1.2				Modification
12/05/13		CrazyBingo	1.3				Modification
13/11/07		CrazyBingo	2.1				Modification
17/04/02		CrazyBingo	3.0				Modify for 12bit width logic
-------------------------------------------------------------------------
|                                     Oooo							|
+------------------------------oooO--(   )-----------------------------+
                              (   )   ) /
                               \ (   (_/
                                \_)
----------------------------------------------------------------------*/

`timescale 1ns/1ns
module lcd_display
/*#(
	parameter	[27:0]	DELAY_TOP
)*/
(
	input	 			clk,		//system clock
	input				rst_n,		//sync clock
	
	input		[11:0]	lcd_xpos,	//lcd horizontal coordinate
	input		[11:0]	lcd_ypos,	//lcd vertical coordinate
	output	reg	[23:0]	lcd_data	//lcd data
);
`include "lcd_para.v" 
`define	VGA_HORIZONTAL_COLOR
`define	VGA_VERICAL_COLOR
`define	VGA_GRAY_GRAPH
`define	VGA_GRAFTAL_GRAPH

initial begin
	lcd_data <= 0; 
end

localparam  DELAY_TOP = 40_000000;
//-------------------------------------------
`ifdef VGA_HORIZONTAL_COLOR
reg	[23:0]	lcd_data0;
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		lcd_data0 <= 0;
	else
		begin
		if	(lcd_ypos >= 0 && lcd_ypos < (`V_DISP/8)*1)
			lcd_data0 <= `RED;
		else if(lcd_ypos >= (`V_DISP/8)*1 && lcd_ypos < (`V_DISP/8)*2)
			lcd_data0 <= `GREEN;
		else if(lcd_ypos >= (`V_DISP/8)*2 && lcd_ypos < (`V_DISP/8)*3)
			lcd_data0 <= `BLUE;
		else if(lcd_ypos >= (`V_DISP/8)*3 && lcd_ypos < (`V_DISP/8)*4)
			lcd_data0 <= `WHITE;
		else if(lcd_ypos >= (`V_DISP/8)*4 && lcd_ypos < (`V_DISP/8)*5)
			lcd_data0 <= `BLACK;
		else if(lcd_ypos >= (`V_DISP/8)*5 && lcd_ypos < (`V_DISP/8)*6)
			lcd_data0 <= `YELLOW;
		else if(lcd_ypos >= (`V_DISP/8)*6 && lcd_ypos < (`V_DISP/8)*7)
			lcd_data0 <= `CYAN;
		else// if(lcd_ypos >= (`V_DISP/8)*7 && lcd_ypos < (`V_DISP/8)*8)
			lcd_data0 <= `ROYAL;
		end
end
`endif

//-------------------------------------------
`ifdef VGA_VERICAL_COLOR
reg	[23:0]	lcd_data1;
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		lcd_data1 <= 0;
	else
		begin
		if	(lcd_xpos >= 0 && lcd_xpos < (`H_DISP/8)*1)
			lcd_data1 <= `RED;
		else if(lcd_xpos >= (`H_DISP/8)*1 && lcd_xpos < (`H_DISP/8)*2)
			lcd_data1 <= `GREEN;
		else if(lcd_xpos >= (`H_DISP/8)*2 && lcd_xpos < (`H_DISP/8)*3)
			lcd_data1 <= `BLUE;
		else if(lcd_xpos >= (`H_DISP/8)*3 && lcd_xpos < (`H_DISP/8)*4)
			lcd_data1 <= `WHITE;
		else if(lcd_xpos >= (`H_DISP/8)*4 && lcd_xpos < (`H_DISP/8)*5)
			lcd_data1 <= `BLACK;
		else if(lcd_xpos >= (`H_DISP/8)*5 && lcd_xpos < (`H_DISP/8)*6)
			lcd_data1 <= `YELLOW;
		else if(lcd_xpos >= (`H_DISP/8)*6 && lcd_xpos < (`H_DISP/8)*7)
			lcd_data1 <= `CYAN;
		else// if(lcd_xpos >= (`H_DISP/8)*7 && lcd_xpos < (`H_DISP/8)*8)
			lcd_data1 <= `ROYAL;
		end
end
`endif

	
//-------------------------------------------
`ifdef VGA_GRAFTAL_GRAPH
reg	[23:0]	lcd_data2;
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		lcd_data2 <= 0;
	else
		lcd_data2 <= lcd_xpos * lcd_ypos;
end
`endif

//-------------------------------------------
`ifdef VGA_GRAY_GRAPH
reg	[23:0]	lcd_data3;
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		lcd_data3 <= 0;
	else
		begin
		if(lcd_ypos < `V_DISP/2)
			lcd_data3 <= {lcd_ypos[7:0], lcd_ypos[7:0], lcd_ypos[7:0]};
		else
			lcd_data3 <= {lcd_xpos[7:0], lcd_xpos[7:0], lcd_xpos[7:0]};
		end
end
`endif

//------------------------------------
//0.5S Delay
reg [27:0]	delay_cnt;
//localparam	DELAT_TOP 50_000000/2
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		delay_cnt <= 0;
	else
		delay_cnt <= (delay_cnt < DELAY_TOP - 1'b1) ? delay_cnt + 1'b1 : 28'd0;
end
wire	delay_0S5_flag = (delay_cnt == DELAY_TOP - 1'b1) ? 1'b1 : 1'b0;

//------------------------------------
//4 picture cycle cnt
reg	[1:0]	image_cnt;
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		image_cnt <= 0;
	else if(delay_0S5_flag)
		image_cnt <= image_cnt + 1'b1;
	else
		image_cnt <= image_cnt;
end

//------------------------------------
//4 picture cycle
always@(*)
begin
	case(image_cnt)
	0:	lcd_data <= lcd_data0;
	1:	lcd_data <= lcd_data1;
	2:	lcd_data <= lcd_data2;
	3:	lcd_data <= lcd_data3;
	endcase
end

endmodule

====================================================================================================
 [28/29] src/lvds/LCDDual2LVDS.v    (106 行, 未例化)
====================================================================================================
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    00:11:37 06/05/2023 
// Module Name:    LCDDual2LVDS 
//////////////////////////////////////////////////////////////////////////////////
module LCDDual2LVDS (
	input 			wclk_i,
	input 			wrst_i, 	
	input 			vs_i,		//	Reset on VS. 
	input 			de_i,
	input 	[47:0] 	data_i,	//	Dual lane raw rgb input (through FIFO). LCD requires B at Lsb. 
	
	input 			rclk_i,
	output 	[6:0] 	txc_o,
	output 	[6:0] 	txd0_o,
	output 	[6:0] 	txd1_o,
	output 	[6:0] 	txd2_o,
	output 	[6:0] 	txd3_o, 
	
	output 			vs_o, 	//	=0
	output 			hs_o, 	//	=0
	output 			de_o, 	//	DE
	output 	[23:0] 	data_o	//	DAT
);
	
	reg 			r_vs = 0; 
	reg 	[4:0] 	rc_rst = 0; 
	reg 			r_rst = 0; 
	always @(posedge wclk_i) begin
		r_vs <= vs_i; 
		if({r_vs, vs_i} == 2'b01) begin
			rc_rst <= 0; 
			r_rst <= 1; 
		end else begin
			rc_rst <= rc_rst + r_rst; 
			r_rst <= (&rc_rst) ? 0 : r_rst; 
		end
	end
	
	
	reg 	[2:0] 	r_de_r = 0; 
	reg 	[11:0] 	rc_rd = 0; 
	reg 			r_rd = 0; 
	wire 	[23:0] 	w_rdata; 
	
	always @(posedge rclk_i) begin
		r_de_r <= {r_de_r, de_i}; 
		
		rc_rd <= 0; 
		
		//	On falling DE start readout. 
		if(r_de_r[2:1] == 2'b10) begin
			rc_rd <= 0; 
			r_rd <= 1; 
		end else begin
			rc_rd <= rc_rd + 1; 
			
			if(rc_rd >= 1023)
				r_rd <= 0; 
			else begin
			end
		end
	end
	
	FIFO_W48R24 wfifo (
		.wr_clk_i		(wclk_i),
		.a_rst_i		(r_rst),
		.wr_en_i		(de_i),
		.wdata		(data_i),
		.wr_datacount_o	(),
		.full_o		(),
		
		.rd_clk_i		(rclk_i),
		.rd_en_i		(r_rd),
		.rdata		(w_rdata),
		.empty_o		(),
		.rd_datacount_o	(),
		
		.rst_busy		()
	);
	
	
	wire 	[7:0] 	lcd_blue  = w_rdata[ 7: 0]; 
	wire 	[7:0] 	lcd_green = w_rdata[15: 8]; 
	wire 	[7:0] 	lcd_red   = w_rdata[23:16]; 
	wire 			lcd_de    = r_rd; 
	
	wire    [7:0]   w_lvds_c0 = 7'b1100011;
	wire    [7:0]   w_lvds_d0 = {lcd_green[0],  lcd_red[5:0]};
	wire    [7:0]   w_lvds_d1 = {lcd_blue[1:0], lcd_green[5:1]};
	wire    [7:0]   w_lvds_d2 = {lcd_de, 2'b0,  lcd_blue[5:2]};   				//vs hs is reserved
	wire    [7:0]   w_lvds_d3 = {1'b0, lcd_blue[7:6], lcd_green[7:6], lcd_red[7:6]};
	
	assign  txc_o  = {w_lvds_c0[0], w_lvds_c0[1], w_lvds_c0[2], w_lvds_c0[3], w_lvds_c0[4], w_lvds_c0[5], w_lvds_c0[6]};
	assign  txd0_o = {w_lvds_d0[0], w_lvds_d0[1], w_lvds_d0[2], w_lvds_d0[3], w_lvds_d0[4], w_lvds_d0[5], w_lvds_d0[6]};
	assign  txd1_o = {w_lvds_d1[0], w_lvds_d1[1], w_lvds_d1[2], w_lvds_d1[3], w_lvds_d1[4], w_lvds_d1[5], w_lvds_d1[6]};
	assign  txd2_o = {w_lvds_d2[0], w_lvds_d2[1], w_lvds_d2[2], w_lvds_d2[3], w_lvds_d2[4], w_lvds_d2[5], w_lvds_d2[6]};
	assign  txd3_o = {w_lvds_d3[0], w_lvds_d3[1], w_lvds_d3[2], w_lvds_d3[3], w_lvds_d3[4], w_lvds_d3[5], w_lvds_d3[6]};
	
	assign vs_o = 0; 
	assign hs_o = 0; 
	assign de_o = lcd_de; 
	assign data_o = {lcd_red, lcd_green, lcd_blue}; 	//	DAT

endmodule

====================================================================================================
 [29/29] src/Sensor_Image_XYCrop.v    (136 行, 未例化)
====================================================================================================
/*-----------------------------------------------------------------------
								 \\\|///
							   \\  - -  //
								(  @ @  )
+-----------------------------oOOo-(_)-oOOo-----------------------------+
CONFIDENTIAL IN CONFIDENCE
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo (Thereturnofbingo).
In the event of publication, the following notice is applicable:
Copyright (C) 2012-20xx CrazyBingo Corporation
The entire notice above must be reproduced on all authorized copies.
Author				:		CrazyBingo
Technology blogs 	: 		www.crazyfpga.com
Email Address 		: 		crazyfpga@vip.qq.com
Filename			:		Sensor_Image_Crop.v
Date				:		2017-06-19
Description			:		Zoom X & Y for Sensor image output.
Modification History	:
Date			By			Version			Change Description
=========================================================================
17/06/19		CrazyBingo	1.0				Original
17/08/19		CrazyBingo	1.1				Modify for X & Y zoom
-------------------------------------------------------------------------
|                                     Oooo								|
+------------------------------oooO--(   )-----------------------------+
                              (   )   ) /
                               \ (   (_/
                                \_)
----------------------------------------------------------------------*/ 

`timescale 1ns / 1ns
module Sensor_Image_XYCrop
#(
	parameter	IMAGE_HSIZE_SOURCE	=	1280,
	parameter	IMAGE_VSIZE_SOURCE	=	1024,
	parameter	IMAGE_HSIZE_TARGET 	= 	1280,
	parameter	IMAGE_YSIZE_TARGET	= 	960,
	parameter 	PIXEL_DATA_WIDTH 	= 	64
)
(
	//globel clock
	input				clk,				//image pixel clock
	input				rst_n,				//system reset
	
	//CMOS Sensor interface
	input								image_in_vsync,		//H : Data Valid; L : Frame Sync(Set it by register)
	input								image_in_href,		//H : Data vaild, L : Line Sync
	input								image_in_de,		//H : Data Enable, L : Line Sync
	input		[PIXEL_DATA_WIDTH-1:0]	image_in_data,		//8 bits cmos data input
	
	output								image_out_vsync,	//H : Data Valid; L : Frame Sync(Set it by register)
	output								image_out_href,		//H : Data vaild, L : Line Sync
	output								image_out_de,		//H : Data Enable, L : Line Sync
	output		[PIXEL_DATA_WIDTH-1:0]	image_out_data		//8 bits cmos data input	
);


//-----------------------------------
//Generate href negedge signal
reg							image_in_href_r;
reg 						image_in_de_r; 
reg							image_in_vsync_r;
reg	[PIXEL_DATA_WIDTH-1:0]	image_in_data_r;
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		begin
		image_in_vsync_r <= 0;
		image_in_href_r <= 0;
		image_in_data_r <= 0;
		image_in_de_r <= 0; 
		end
	else
		begin
		image_in_vsync_r <= image_in_vsync;
		image_in_href_r <= image_in_href;
		image_in_data_r <= image_in_data;
		image_in_de_r <= image_in_de; 
		end
end
wire	image_in_href_negedge = (image_in_href_r & ~image_in_href) ? 1'b1 : 1'b0;

	
//-----------------------------------
//Image Ysize Crop
reg	[11:0] image_ypos;
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		image_ypos <= 0;
	else if(image_in_vsync == 1'b1)
		begin
		if(image_in_href_negedge == 1'b1)
			image_ypos <= image_ypos + 1'b1;
		else
			image_ypos <= image_ypos;
		end
	else
		image_ypos <= 0;
end
assign	image_out_vsync = image_in_vsync_r;

						   
						   
//-----------------------------------
//Image Hsize Crop
reg	[11:0] image_xpos;
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
		image_xpos <= 0;
	else if(image_in_href == 1'b1)
		image_xpos <= image_xpos + image_in_de;
	else
		image_xpos <= 0;
end

wire 			w_image_out_href = (image_in_href == 1'b1) && (
								  ((image_ypos >= (IMAGE_VSIZE_SOURCE - IMAGE_YSIZE_TARGET)/2) && 
								   (image_ypos <  (IMAGE_VSIZE_SOURCE - IMAGE_YSIZE_TARGET)/2 + IMAGE_YSIZE_TARGET) &&
								   (image_xpos >= (IMAGE_HSIZE_SOURCE - IMAGE_HSIZE_TARGET)/2) && 
								   (image_xpos <  (IMAGE_HSIZE_SOURCE - IMAGE_HSIZE_TARGET)/2 + IMAGE_HSIZE_TARGET))? 1'b1 : 1'b0);

reg 			image_out_href_r = 0; 
always @(posedge clk) begin	
	image_out_href_r <= w_image_out_href; 
end
assign image_out_href = image_out_href_r; 

assign 	image_out_de = image_in_de_r; 

assign	image_out_data = image_in_data_r;				   


endmodule

