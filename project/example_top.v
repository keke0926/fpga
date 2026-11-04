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
	
	input 	[1:0] 	i_dqs_n_hi,
	input 	[1:0] 	i_dqs_n_lo,
	
	output 	[1:0] 	o_dqs_hi,
	output 	[1:0] 	o_dqs_lo,
	
	output 	[1:0] 	o_dqs_n_hi,
	output 	[1:0] 	o_dqs_n_lo,
	
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
	
	input key_app_n_i,key_auto_n_i,key_view_n_i,key_plus_n_i,key_minus_n_i,key_detect_n_i, // active-low physical keys

	output 	[5:0] 	lcd_b7_0_o,			//	B7:B0. 
	output 	[7:0] 	lcd_g7_0_o,			//	G7:G0. Must output 8'hFF when access SPI. 
	output 	[7:0] 	lcd_r7_0_o,			//	R7:R0. 
	
	output 	[5:0] 	lcd_b7_0_oe,		//	B7:B0. 
	output 	[7:0] 	lcd_g7_0_oe,		//	G7:G0. Must output 8'hFF when access SPI. 
	output 	[7:0] 	lcd_r7_0_oe,		//	R7:R0. 

	input 	[5:0] 	lcd_b7_0_i,			//	B7:B0. 
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
	always @(posedge clk_pixel or negedge w_pll_lock) begin if(~w_pll_lock) rstn_pixel <= 0; else rstn_pixel <= 1; end
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
	
	assign lcd_b7_0_o = r_flash_en ? {2'b0, w_spi_data_o[3:2], 2'b0} : w_lcd_b_o[5:0]; 
	assign lcd_g7_0_o = r_flash_en ? {6'h0, w_spi_data_o[1:0]} : w_lcd_g_o; 
	assign lcd_r7_0_o = r_flash_en ? {8'h00} : w_lcd_r_o; 
	
	assign lcd_b7_0_oe = r_flash_en ? {2'b0, w_spi_data_oe[3:2], 2'b0} : 6'h3F; 
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
	

// Separate video accounting from the local analysis DDR client.
wire physical_ddr_avalid;
wire physical_ddr_aready;
wire [31:0] physical_ddr_aaddr;
wire [3:0] physical_ddr_aid;
wire [7:0] physical_ddr_alen;
wire physical_ddr_atype;
wire physical_ddr_wvalid;
wire physical_ddr_wready;
wire [127:0] physical_ddr_wdata;
wire [15:0] physical_ddr_wstrb;
wire physical_ddr_wlast;
wire [3:0] physical_ddr_wid;
wire physical_ddr_bvalid;
wire physical_ddr_bready;
wire [3:0] physical_ddr_bid;
wire [1:0] physical_ddr_bresp;
wire physical_ddr_rvalid;
wire physical_ddr_rready;
wire [3:0] physical_ddr_rid;
wire [1:0] physical_ddr_rresp;
wire [127:0] physical_ddr_rdata;
wire physical_ddr_rlast;
wire analysis_req_valid,analysis_req_ready,analysis_req_write,analysis_rsp_valid,analysis_rsp_ready,analysis_rsp_error;
wire[31:0]analysis_req_addr;wire[127:0]analysis_req_data,analysis_rsp_data;
wire held_axi,arb_rst_n,local_active,local_reference_valid,local_lease_req,local_lease_grant,local_scratch_we;
wire local_region_start,local_region_ready,local_region_idle,local_region_valid,local_region_done;
wire[8:0]local_region_x0,local_region_x1;wire[7:0]local_region_y0,local_region_y1;
wire[11:0]local_region_count;wire[3:0]local_region_error;
wire[1:0]held_bank_axi;wire[3:0]local_status;wire[15:0]local_scratch_addr;wire[7:0]local_scratch_data;
wire[7:0]local_scratch_read;
wire[15:0]local_state_addr;wire[1:0]local_state_we,local_state_data,local_state_read;
wire[11:0]local_queue_addr;wire local_queue_we;wire[15:0]local_queue_data,local_queue_read;
wire local_video_range_fault;
reset_sync_n local_arb_reset(.clk(w_ddr3_ui_clk),.arst_n(rstn_sys&&!w_ddr3_ui_rst&&rstn_lvds_1x),.rst_n(arb_rst_n));
part_ddr_arbiter local_ddr_arbiter(.clk(w_ddr3_ui_clk),.rst_n(arb_rst_n),.ddr_ready_i(r_ddr_unlock),
 .held_i(held_axi),.bank_i(held_bank_axi),
 .v_avalid(w_ddr3_avalid),
 .v_aready(w_ddr3_aready),
 .v_aaddr(w_ddr3_aaddr),
 .v_aid(w_ddr3_aid),
 .v_alen(w_ddr3_alen),
 .v_atype(w_ddr3_atype),
 .v_wvalid(w_ddr3_wvalid),
 .v_wready(w_ddr3_wready),
 .v_wdata(w_ddr3_wdata),
 .v_wstrb(w_ddr3_wstrb),
 .v_wlast(w_ddr3_wlast),
 .v_wid(w_ddr3_wid),
 .v_bvalid(w_ddr3_bvalid),
 .v_bready(w_ddr3_bready),
 .v_bid(w_ddr3_bid),
 .v_bresp(w_ddr3_bresp),
 .v_rvalid(w_ddr3_rvalid),
 .v_rready(w_ddr3_rready),
 .v_rid(w_ddr3_rid),
 .v_rresp(w_ddr3_rresp),
 .v_rdata(w_ddr3_rdata),
 .v_rlast(w_ddr3_rlast),
 .a_req_valid(analysis_req_valid),.a_req_ready(analysis_req_ready),.a_req_write(analysis_req_write),
 .a_req_addr(analysis_req_addr),.a_req_data(analysis_req_data),.a_rsp_valid(analysis_rsp_valid),
 .a_rsp_ready(analysis_rsp_ready),.a_rsp_error(analysis_rsp_error),.a_rsp_data(analysis_rsp_data),
 .p_avalid(physical_ddr_avalid),
 .p_aready(physical_ddr_aready),
 .p_aaddr(physical_ddr_aaddr),
 .p_aid(physical_ddr_aid),
 .p_alen(physical_ddr_alen),
 .p_atype(physical_ddr_atype),
 .p_wvalid(physical_ddr_wvalid),
 .p_wready(physical_ddr_wready),
 .p_wdata(physical_ddr_wdata),
 .p_wstrb(physical_ddr_wstrb),
 .p_wlast(physical_ddr_wlast),
 .p_wid(physical_ddr_wid),
 .p_bvalid(physical_ddr_bvalid),
 .p_bready(physical_ddr_bready),
 .p_bid(physical_ddr_bid),
 .p_bresp(physical_ddr_bresp),
 .p_rvalid(physical_ddr_rvalid),
 .p_rready(physical_ddr_rready),
 .p_rid(physical_ddr_rid),
 .p_rresp(physical_ddr_rresp),
 .p_rdata(physical_ddr_rdata),
 .p_rlast(physical_ddr_rlast),
 .video_range_fault_o(local_video_range_fault));
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
		.o_dqs_n_oe		(o_dqs_n_oe),
		
		.clk			(w_ddr3_ui_clk),
		.reset_n		(w_ddr3_ui_aresetn),
		
		.axi_avalid		(physical_ddr_avalid),	//	Enable command only when unlocked. 
		.axi_aready		(physical_ddr_aready),
		.axi_aaddr		(physical_ddr_aaddr),
		.axi_aid		(physical_ddr_aid),
		.axi_alen		(physical_ddr_alen),
		.axi_asize		(w_ddr3_asize),
		.axi_aburst		(w_ddr3_aburst),
		.axi_alock		(w_ddr3_alock),
		.axi_atype		(physical_ddr_atype),
		
		.axi_wid		(physical_ddr_wid),
		.axi_wvalid		(physical_ddr_wvalid),
		.axi_wready		(physical_ddr_wready),
		.axi_wdata		(physical_ddr_wdata),
		.axi_wstrb		(physical_ddr_wstrb),
		.axi_wlast		(physical_ddr_wlast),
		
		.axi_bvalid		(physical_ddr_bvalid),
		.axi_bready		(physical_ddr_bready),
		.axi_bid		(physical_ddr_bid),
		.axi_bresp		(physical_ddr_bresp),
		
		.axi_rvalid		(physical_ddr_rvalid),
		.axi_rready		(physical_ddr_rready),
		.axi_rdata		(physical_ddr_rdata),
		.axi_rid		(physical_ddr_rid),
		.axi_rresp		(physical_ddr_rresp),
		.axi_rlast		(physical_ddr_rlast),
		
		.shift		(shift),
		.shift_sel		(),
		.shift_ena		(shift_ena),
		
		.cal_ena		(1'b1),
		.cal_done		(w_ddr3_cal_done),
		.cal_pass		(w_ddr3_cal_pass)
	);
	
	assign w_ddr3_bready = 1'b1; 
	assign shift_sel = 5'b00100; 		//	ddr_tac_clk always use PLLOUT[2]. 
	
	
	AXI4_AWARMux #(.AID_LEN(4), .AADDR_LEN(32)) axi4_awar_mux (
		.aclk_i			(w_ddr3_ui_clk), 
		.arst_i			(w_ddr3_ui_rst), 
		
		.awid_i			(w_ddr3_awid),
		.awaddr_i			(w_ddr3_awaddr),
		.awlen_i			(w_ddr3_awlen),
		.awvalid_i			(w_ddr3_awvalid),
		.awready_o			(w_ddr3_awready),
		
		.arid_i			(w_ddr3_arid),
		.araddr_i			(capture_araddr),
		.arlen_i			(w_ddr3_arlen),
		.arvalid_i			(w_ddr3_arvalid),
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
	
	//assign led_o[1:0] = {w_ddr3_cal_pass, w_ddr3_cal_done}; 
	
	
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////
	//	I2C Config (SC130GS)
	
	//  i2c timing controller module of 16Bit
	wire            [ 7:0]          sc130_i2c_config_index;
	wire            [23:0]          sc130_i2c_config_data;
	wire            [ 7:0]          sc130_i2c_config_size;
	wire                            sc130_i2c_config_done;
	
	i2c_timing_ctrl_16bit
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
	    .i2c_config_index   (sc130_i2c_config_index        ),                          //  i2c config reg index, read 2 reg and write xx reg
	    .i2c_config_data    ({8'h60, sc130_i2c_config_data}),                     //  i2c config data
	    .i2c_config_size    (sc130_i2c_config_size         ),                          //  i2c config data counte
	    .i2c_config_done    (sc130_i2c_config_done         )                          //  i2c config timing complete
	);
	assign csi_scl_oe = 1; 

	//  I2C Configure Data of SC130GS
	I2C_SC130GS_12801024_4Lanes_Config u_I2C_SC130GS_12801024_4Lanes_Config 
	(
	    .LUT_INDEX  (sc130_i2c_config_index   ),
	    .LUT_DATA   (sc130_i2c_config_data    ),
	    .LUT_SIZE   (sc130_i2c_config_size    )
	);
	

	////////////////////////////////////////////////////////////////
	//	I2C Config (AR0135)
	
	//  i2c timing controller module of 16Bit
	wire            [ 7:0]          ar0135_i2c_config_index;
	wire            [31:0]          ar0135_i2c_config_data;
	wire            [ 7:0]          ar0135_i2c_config_size;
	wire                            ar0135_i2c_config_done;
	wire            [15:0]          ar0135_i2c_rdata;                              //  i2c register data

	i2c_timing_ctrl_reg16_dat16
	#(
	    .SIMU_EN            (0              ),
	    .I2C_READ_EN        (1              ),                              //  Enable I2C Register read
	    .CLK_FREQ           (CLOCK_MAIN     ),                              //  120 MHz
	    .I2C_FREQ           (50_000         )                               //  10 KHz(<= 400KHz)
	)
	u_i2c_timing_ctrl_16reg_16bit
	(
	    //global clock
	    .clk                (clk_sys                 ),                          //  96MHz
	    .rst_n              (rstn_sys                ),                          //  system reset

	    //i2c interface
	    .i2c_sclk           (cmos_sclk                  ),                  //  i2c clock
		.i2c_sdat_o 	(cmos_sdat_OUT), 
		.i2c_sdat_oe	(cmos_sdat_OE), 
		.i2c_sdat_i 	(cmos_sdat_IN), 
		
	    //i2c config data
	    .i2c_config_index   (ar0135_i2c_config_index           ),                  //  i2c config reg index, read 2 reg and write xx reg
	    .i2c_config_data    ({8'h20, ar0135_i2c_config_data}   ),                  //  i2c config data
	    .i2c_config_size    (ar0135_i2c_config_size            ),                  //  i2c config data counte
	    .i2c_config_done    (ar0135_i2c_config_done            ),                  //  i2c config timing complete
	    .i2c_rdata          (ar0135_i2c_rdata                  )                   //  i2c register data while read i2c slave
	);
	
	//----------------------------------------------------------------------
	//  I2C Configure Data of AR0135
	I2C_AR0135_1280720_Config u_I2C_AR0135_1280720_Config
	(
	    .LUT_INDEX  (ar0135_i2c_config_index   ),
	    .LUT_DATA   (ar0135_i2c_config_data    ),
	    .LUT_SIZE   (ar0135_i2c_config_size    )
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
	assign led_o[5] = r_cmos_fv_o[3]; 
	
	
	
	
	
	
	
	
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
	localparam 	CSI_DATA_WIDTH 	= 8; 			
	localparam 	CSI_STRB_WIDTH 	= CSI_DATA_WIDTH / 8; 

	
	////////////////////////////////////////////////////////////////
	//	MIPI-CSI Crop
	
	wire			XYCrop_frame_vsync; 
	wire			XYCrop_frame_href;
	wire			XYCrop_frame_de;
	wire	[63:0]	XYCrop_frame_Gray;

	Sensor_Image_XYCrop
	#(
		.IMAGE_HSIZE_SOURCE (1280 / CSI_STRB_WIDTH),
		.IMAGE_VSIZE_SOURCE (720	 ),
		.IMAGE_HSIZE_TARGET (1024 / CSI_STRB_WIDTH),
		.IMAGE_YSIZE_TARGET (600 	 ),
		.PIXEL_DATA_WIDTH	(CSI_DATA_WIDTH) 		//	32		 )
	)
	u_Sensor_Image_XYCrop
	(
		//	globel clock
		.clk			(cmos_pclk),			//	image pixel clock
		.rst_n		(rstn_sys),			//	system reset
		
		//CMOS Sensor interface
		.image_in_vsync (cmos_vsync		),			//H : Data Valid; L : Frame Sync(Set it by register)
		.image_in_href	(cmos_href		),			//H : Data vaild, L : Line Sync
		.image_in_de	(cmos_href		), 			//H : Data Enable, L : Line Sync
		.image_in_data	(cmos_data),			//8 bits cmos data input
		
		.image_out_vsync(XYCrop_frame_vsync ),			//H : Data Valid; L : Frame Sync(Set it by register)
		.image_out_href (XYCrop_frame_href	),			//H : Data vaild, L : Line Sync
		.image_out_de	(XYCrop_frame_de	), 			//H : Data Enable, L : Line Sync
		.image_out_data (XYCrop_frame_Gray	)			//8 bits cmos data input	
	);

	reg			r_XYCrop_frame_vsync = 0; 
	reg			r_XYCrop_frame_href = 0;
	reg			r_XYCrop_frame_de = 0;
	reg	[63:0]	r_XYCrop_frame_Gray = 0;
	
	always @(posedge cmos_pclk) begin
		r_XYCrop_frame_vsync <= XYCrop_frame_vsync; 
		r_XYCrop_frame_href <= XYCrop_frame_href;
		r_XYCrop_frame_de <= XYCrop_frame_de;
		r_XYCrop_frame_Gray <= XYCrop_frame_Gray;
	end
	
	//	Data Write Assignment
	wire			cmos_frame_vsync = r_XYCrop_frame_vsync;                     //  cmos frame data vsync valid signal
	wire			cmos_frame_href = r_XYCrop_frame_href && r_XYCrop_frame_de;	 //  cmos frame data href vaild  signal
	wire	[63:0]	cmos_frame_Gray = r_XYCrop_frame_Gray; 
	wire 			cmos_vsync_end;

	












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
	
    wire[6:0] fps_axi_observation;
    wire fps_lcd_underflow;
    wire[11:0] effective_fps_bcd;
	axi4_ctrl #(.C_RD_END_ADDR(1024 * 600), .C_W_WIDTH(CSI_DATA_WIDTH), .C_R_WIDTH(8), .C_ID_LEN(4)) u_axi4_ctrl (

		.axi_clk        (w_ddr3_ui_clk            ),
		.axi_reset      (w_ddr3_ui_rst            ),
        .rframe_arst_n  (w_pll_lock),

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

		.wframe_pclk    (cmos_pclk          ),
		.wframe_vsync   (capture_writer_vs), //w_wframe_vsync   ),		//	Writter VSync. Flush on rising edge. Connect to EOF. 
		.wframe_data_en (capture_writer_de   ),
		.wframe_data    (cmos_frame_Gray),
		
		.rframe_pclk    (clk_lvds_1x            ),
		.rframe_vsync   (~lcd_vs             ),		//	Reader VSync. Flush on rising edge. Connect to ~EOF. 
		.rframe_data_en (lcd_request             ),
		.rframe_data    (lcd_data           ),
        .fps_axi_observation_o(fps_axi_observation),.fps_lcd_underflow_o(fps_lcd_underflow),
		
		.tp_o 		(w_axi_tp)
	);
	assign led_o[3:0] = w_axi_tp; 
	
	
	
	////////////////////////////////////////////////////////////////
	//  LCD Timing Driver
	
	lcd_driver u_lcd_driver
	(
	    //  global clock
	    .clk        (clk_lvds_1x   ),
	    .rst_n      (rstn_lvds_1x), 
	    
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
	
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	LVDS Output 
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	assign lvds_txc_oe  	= 1'b1; 	//	r_lvds_oe; 
	assign lvds_txd0_oe 	= 1'b1; 	//	r_lvds_oe; 
	assign lvds_txd1_oe 	= 1'b1; 	//	r_lvds_oe; 
	assign lvds_txd2_oe 	= 1'b1; 	//	r_lvds_oe; 
	assign lvds_txd3_oe 	= 1'b1; 	//	r_lvds_oe; 
	assign lvds_txc_rst_o  	= 1'b0; 	//	r_lvds_rst; 
	assign lvds_txd0_rst_o 	= 1'b0; 	//	r_lvds_rst; 
	assign lvds_txd1_rst_o 	= 1'b0; 	//	r_lvds_rst; 
	assign lvds_txd2_rst_o 	= 1'b0; 	//	r_lvds_rst; 
	assign lvds_txd3_rst_o 	= 1'b0; 	//	r_lvds_rst; 
	
	wire 	[7:0] 	lcd_blue  = lcd_blue; 
	wire 	[7:0] 	lcd_green = lcd_green; 
	wire 	[7:0] 	lcd_red   = lcd_red; 
	wire 			lcd_de    = r_rd; 
	
	// BEGIN AR0135 CAPTURE
	wire capture_writer_vs, capture_writer_de;
	wire runtime_app_mode;
 wire source_locked,canny_enable;wire[31:0]source_frame,source_crc;
 wire[9:0]canny_x,canny_y;wire[7:0]canny_low,canny_high;
 wire[31:0]canny_frame,canny_crc,canny_reference,canny_tag,canny_cycles,canny_display_commits;
 wire[3:0]canny_state,canny_error;wire[14:0]canny_samples,canny_edges;

 wire assist_valid;wire[319:0] assist_boxes;wire[3:0] assist_rows;
 wire[9:0] assist_count;wire[5:0] assist_page;wire[6:0] assist_pages;
	wire runtime_auto_enable;
	wire runtime_median_enable;
	wire [1:0] runtime_view_mode;
	wire [10:0] runtime_threshold;
	wire [31:0] capture_araddr;
	ar0135_capture_link #(.BAUD(460800),.ENABLE_FPS(1),.ENABLE_LOCAL(1)) u_ar0135_capture_link (
 .local_region_start_o(local_region_start),.local_region_ready_o(local_region_ready),.local_region_idle_i(local_region_idle),.local_region_valid_i(local_region_valid),.local_region_done_i(local_region_done),.local_region_x0_i(local_region_x0),.local_region_x1_i(local_region_x1),.local_region_y0_i(local_region_y0),.local_region_y1_i(local_region_y1),.local_region_count_i(local_region_count),.local_region_error_i(local_region_error),.local_active_o(local_active),.local_status_o(local_status),.local_reference_valid_o(local_reference_valid),
 .local_lease_req_o(local_lease_req),.local_lease_grant_i(local_lease_grant),
 .local_scratch_addr_o(local_scratch_addr),.local_scratch_we_o(local_scratch_we),.local_scratch_data_o(local_scratch_data),.local_scratch_data_i(local_scratch_read),
 .local_state_addr_o(local_state_addr),.local_state_we_o(local_state_we),.local_state_data_o(local_state_data),.local_state_data_i(local_state_read),
 .local_queue_addr_o(local_queue_addr),.local_queue_we_o(local_queue_we),.local_queue_data_o(local_queue_data),.local_queue_data_i(local_queue_read),
 .analysis_req_valid_o(analysis_req_valid),.analysis_req_ready_i(analysis_req_ready),.analysis_req_write_o(analysis_req_write),
 .analysis_req_addr_o(analysis_req_addr),.analysis_req_data_o(analysis_req_data),
 .analysis_rsp_valid_i(analysis_rsp_valid),.analysis_rsp_ready_o(analysis_rsp_ready),.analysis_rsp_error_i(analysis_rsp_error),.analysis_rsp_data_i(analysis_rsp_data),
 .held_axi_o(held_axi),.held_bank_axi_o(held_bank_axi),
		.clk_cmos_i(cmos_pclk),
		.rst_cmos_n_i(rstn_sys),
		.src_vs_i(cmos_frame_vsync),
		.src_de_i(cmos_frame_href),
		.src_gray_i(cmos_frame_Gray[7:0]),
		.clk_axi_i(w_ddr3_ui_clk),
		.rst_axi_n_i(~w_ddr3_ui_rst),
		.ddr_ready_i(r_ddr_unlock),
		.awvalid_i(w_ddr3_awvalid),
		.awready_i(w_ddr3_awready),
		.arvalid_i(w_ddr3_arvalid),
		.arready_i(w_ddr3_arready),
		.raw_araddr_i(w_ddr3_araddr),
		.cmd_valid_i(w_ddr3_avalid),
		.cmd_ready_i(w_ddr3_aready),
		.cmd_write_i(w_ddr3_atype),
		.cmd_addr_i(w_ddr3_aaddr),
		.cmd_len_i(w_ddr3_alen),
		.wvalid_i(w_ddr3_wvalid),
		.wready_i(w_ddr3_wready),
		.wlast_i(w_ddr3_wlast),
		.wdata_i(w_ddr3_wdata),
		.wstrb_i(w_ddr3_wstrb),
		.bvalid_i(w_ddr3_bvalid),
		.bready_i(w_ddr3_bready),
		.bresp_i(w_ddr3_bresp),
		.bid_i(w_ddr3_bid),
		.rvalid_i(w_ddr3_rvalid),
		.rready_i(w_ddr3_rready),
		.rlast_i(w_ddr3_rlast),
		.rresp_i(w_ddr3_rresp),
		.rid_i(w_ddr3_rid),
		.writer_state_i(w_axi_tp[1:0]),
		.clk_lcd_i(clk_lvds_1x),
		.rst_lcd_n_i(rstn_lvds_1x),
		.lcd_vs_i(lcd_vs),
		.lcd_de_i(lcd_de),
		.lcd_gray_i(lcd_red),
        .fps_axi_observation_i(fps_axi_observation),.fps_lcd_underflow_i(fps_lcd_underflow),.fps_bcd_o(effective_fps_bcd),
		.uart_rx_i(uart_rx_i),
		.uart_tx_o(uart_tx_o),
		.writer_vs_o(capture_writer_vs),
		.writer_de_o(capture_writer_de),
		.capture_araddr_o(capture_araddr),
		.keys_n_i({key_minus_n_i,key_plus_n_i,key_view_n_i,key_auto_n_i,key_app_n_i}),
		.key_detect_n_i(key_detect_n_i),
		.runtime_app_mode_o(runtime_app_mode),
        .source_locked_o(source_locked),.source_frame_o(source_frame),.source_crc_o(source_crc),
        .canny_enable_o(canny_enable),.canny_x_o(canny_x),.canny_y_o(canny_y),.canny_low_o(canny_low),.canny_high_o(canny_high),
        .canny_frame_o(canny_frame),.canny_crc_o(canny_crc),.canny_reference_o(canny_reference),.canny_tag_o(canny_tag),
        .canny_state_i(canny_state),.canny_error_i(canny_error),.canny_samples_i(canny_samples),.canny_edges_i(canny_edges),
        .canny_cycles_i(canny_cycles),.canny_display_commits_i(canny_display_commits),
        .assist_sof_i(app_sof),.assist_error_i(edge_align_error),
        .assist_valid_o(assist_valid),.assist_boxes_o(assist_boxes),.assist_rows_o(assist_rows),
        .assist_count_o(assist_count),.assist_page_o(assist_page),.assist_pages_o(assist_pages),
		.runtime_auto_enable_o(runtime_auto_enable),
		.runtime_median_enable_o(runtime_median_enable),
		.runtime_view_mode_o(runtime_view_mode),
		.runtime_threshold_o(runtime_threshold)
	);
	// END AR0135 CAPTURE
	// BEGIN LVDS SOBEL
	wire edge_ready, edge_hs, edge_vs, edge_de, edge_sof, edge_eof;
	wire [11:0] edge_x, edge_y;
	wire [7:0] edge_gray;
	wire edge_bit, edge_border, edge_align_error;
	wire [23:0] edge_rgb;
	wire [7:0] edge_red = status_rgb[23:16];
	wire [7:0] edge_green = status_rgb[15:8];
	wire [7:0] edge_blue = status_rgb[7:0];
	video_median_board_front_link #(
		.IMAGE_WIDTH(1024), .IMAGE_HEIGHT(600), .H_TOTAL(1344),
		.COORD_WIDTH(12), .VS_ACTIVE(0), .SYNC_IDLE(1)
	) u_lvds_sobel_link (
		.clk(clk_lvds_1x), .rst_n(rstn_lvds_1x),
		.in_hs(lcd_hs), .in_vs(lcd_vs), .in_de(lcd_de), .in_gray(lcd_red),
		.in_median_enable(runtime_median_enable), .in_auto_enable(runtime_auto_enable), .in_threshold(runtime_threshold), .in_mode(runtime_view_mode),
		.out_ready(edge_ready), .out_hs(edge_hs), .out_vs(edge_vs), .out_de(edge_de),
		.out_sof(edge_sof), .out_eof(edge_eof), .out_x(edge_x), .out_y(edge_y),
		.out_gray(edge_gray), .out_edge(edge_bit), .out_border(edge_border),
		.out_rgb(edge_rgb), .out_align_error(edge_align_error),
		.out_magnitude(), .out_threshold(), .out_auto_enable(), .out_table_applied_valid(),
		.out_statistics_busy(), .out_statistics_late(), .out_statistics_ready()
	);
	// END LVDS SOBEL
    // Full-scene dynamic-target analysis. Main video never stalls; result is
    // shown with its matching binary overview and Chinese labels (2 cycles).
    wire app_hs, app_vs, app_de, app_sof, app_eof;
    wire [11:0] app_x, app_y;
    wire [23:0] app_rgb;
    video_roi_digit_app #(.ENABLE_LOCAL(1),.EXTERNAL_FONT(0)) u_roi_digit_app(
      .font_address_o(),.font_bits_i(16'd0),
 .local_region_start_i(local_region_start),.local_region_ready_i(local_region_ready),.local_region_idle_o(local_region_idle),.local_region_valid_o(local_region_valid),.local_region_done_o(local_region_done),.local_region_x0_o(local_region_x0),.local_region_x1_o(local_region_x1),.local_region_y0_o(local_region_y0),.local_region_y1_o(local_region_y1),.local_region_count_o(local_region_count),.local_region_error_o(local_region_error),.local_req_i(local_lease_req),.local_grant_o(local_lease_grant),
 .local_gray_addr_i(local_scratch_addr),.local_gray_we_i(local_scratch_we),.local_gray_data_i(local_scratch_data),.local_gray_data_o(local_scratch_read),
 .local_state_addr_i(local_state_addr),.local_state_we_i(local_state_we),.local_state_data_i(local_state_data),.local_state_data_o(local_state_read),
 .local_queue_addr_i(local_queue_addr),.local_queue_we_i(local_queue_we),.local_queue_data_i(local_queue_data),.local_queue_data_o(local_queue_read),
        .clk(clk_lvds_1x), .rst_n(rstn_lvds_1x),
        .in_hs(edge_hs), .in_vs(edge_vs), .in_de(edge_de),
        .in_sof(edge_sof), .in_eof(edge_eof), .in_x(edge_x), .in_y(edge_y),
        .in_gray(edge_gray), .in_rgb(edge_rgb), .in_align_error(edge_align_error),
        .cfg_app_mode(runtime_app_mode), .cfg_auto_mode(runtime_auto_enable),
        .cfg_median_enable(runtime_median_enable), .cfg_view_mode(runtime_view_mode),
        .cfg_manual_threshold(runtime_threshold),
        .source_locked_i(source_locked),.source_frame_i(source_frame),.source_crc_i(source_crc),
        .canny_enable_i(canny_enable),.canny_x_i(canny_x),.canny_y_i(canny_y),.canny_low_i(canny_low),.canny_high_i(canny_high),
        .canny_frame_i(canny_frame),.canny_crc_i(canny_crc),.canny_reference_i(canny_reference),.canny_tag_i(canny_tag),
        .canny_state_o(canny_state),.canny_error_o(canny_error),.canny_samples_o(canny_samples),.canny_edges_o(canny_edges),
        .canny_cycles_o(canny_cycles),.canny_display_commits_o(canny_display_commits),
        .out_hs(app_hs), .out_vs(app_vs), .out_de(app_de),
        .out_sof(app_sof), .out_eof(app_eof), .out_x(app_x), .out_y(app_y), .out_rgb(app_rgb));
    // Frame-bound part assist adds two matched clocks. Generic queue unchanged.
    wire part_hs,part_vs,part_de,part_sof,part_eof;wire[11:0] part_x,part_y;wire[23:0] part_rgb;
    part_assist_overlay #(.EXTERNAL_FONT(0)) u_part_assist(.font_address_o(),.font_bits_i(16'd0),.clk(clk_lvds_1x),.rst_n(rstn_lvds_1x),
      .in_hs(app_hs),.in_vs(app_vs),.in_de(app_de),.in_sof(app_sof),.in_eof(app_eof),.in_x(app_x),.in_y(app_y),.in_rgb(app_rgb),
      .display_valid(assist_valid),.display_boxes(assist_boxes),.display_rows(assist_rows),.display_count(assist_count),
      .display_page(assist_page),.display_pages(assist_pages),
      .out_hs(part_hs),.out_vs(part_vs),.out_de(part_de),.out_sof(part_sof),.out_eof(part_eof),.out_x(part_x),.out_y(part_y),.out_rgb(part_rgb));
    // Chinese status adds 2 matched cycles; app plus assist plus status total is 6 cycles.
    wire [23:0] status_rgb;
    wire status_de;
    video_status_osd #(.COORD_WIDTH(12),.ENABLE_FPS(1),.ENABLE_LOCAL(1)) u_status_osd(
        .clk(clk_lvds_1x),.rst_n(rstn_lvds_1x),
        .in_hs(part_hs),.in_vs(part_vs),.in_de(part_de),.in_sof(part_sof),.in_eof(part_eof),
        .in_x(part_x),.in_y(part_y),.in_rgb(part_rgb),
        .cfg_app_mode(runtime_app_mode),.cfg_auto_mode(runtime_auto_enable),
        .cfg_median_enable(runtime_median_enable),.cfg_view_mode(runtime_view_mode),
        .cfg_manual_threshold(runtime_threshold),.fps_bcd_i(effective_fps_bcd),
 .local_status_i(local_status),.local_reference_valid_i(local_reference_valid),
        .out_hs(),.out_vs(),.out_de(status_de),.out_sof(),.out_eof(),.out_x(),.out_y(),.out_rgb(status_rgb));

	wire    [7:0]   w_lvds_c0 = 7'b1100011;
	wire    [7:0]   w_lvds_d0 = {edge_green[0],  edge_red[5:0]};
	wire    [7:0]   w_lvds_d1 = {edge_blue[1:0], edge_green[5:1]};
	wire    [7:0]   w_lvds_d2 = {status_de, 2'b0,  edge_blue[5:2]};   				//vs hs is reserved
	wire    [7:0]   w_lvds_d3 = {1'b0, edge_blue[7:6], edge_green[7:6], edge_red[7:6]};
	
	assign  lvds_txc_o  = {w_lvds_c0[0], w_lvds_c0[1], w_lvds_c0[2], w_lvds_c0[3], w_lvds_c0[4], w_lvds_c0[5], w_lvds_c0[6]};
	assign  lvds_txd0_o = {w_lvds_d0[0], w_lvds_d0[1], w_lvds_d0[2], w_lvds_d0[3], w_lvds_d0[4], w_lvds_d0[5], w_lvds_d0[6]};
	assign  lvds_txd1_o = {w_lvds_d1[0], w_lvds_d1[1], w_lvds_d1[2], w_lvds_d1[3], w_lvds_d1[4], w_lvds_d1[5], w_lvds_d1[6]};
	assign  lvds_txd2_o = {w_lvds_d2[0], w_lvds_d2[1], w_lvds_d2[2], w_lvds_d2[3], w_lvds_d2[4], w_lvds_d2[5], w_lvds_d2[6]};
	assign  lvds_txd3_o = {w_lvds_d3[0], w_lvds_d3[1], w_lvds_d3[2], w_lvds_d3[3], w_lvds_d3[4], w_lvds_d3[5], w_lvds_d3[6]};
	
	
	
	
endmodule


