/* 
 * Copyright 2013-2016 Colin Weltin-Wu (colinww@gmail.com)
 * UC San Diego Integrated Signal Processing Group
 *
 * Licensed under GNU General Public License 3.0 or later. 
 * Some rights reserved. See LICENSE.
 * 
 * reg_slice.v
 * 
 * General purpose register bank. For correct operation, this block requires
 * a running clock on i_clk at all times.
 * 
 * i_ena high defines "normal mode", and the data on vi_regs appears at
 * vo_regs on each rising edge of i_clk. The vio_tbus is tristated.
 * 
 * In normal mode, the signals i_step and i_wr are ignored. Bringing i_rd
 * high will cause a sample of the data in the register to appear on vio_tbus.
 * This sample will be taken on the next rising edge if i_clk after i_rd
 * goes high. As long as i_rd is held high, the data which was initially
 * sampled will continue to appear on vio_tbus. One period after data appears
 * on vio_tbus, the signal o_success will go high, indicating that vio_tbus
 * can be read. This will stay high for 1.5-2 periods after i_rd goes low.
 * 
 * i_ena low defines "test mode". In this mode, the register clock is
 * synchronously disabled, and the register will retain the last value.
 * In test mode, i_rd behaves the same as in normal mode. If i_wr goes
 * high, the value on vio_tbus (driven externally) will be latched into
 * the register 1-1.5 periods after i_wr goes high. 2-2.5 periods after
 * i_wr goes high, o_success will go high, indicating the data was latched.
 * This will stay high 2.5-3 periods after i_wr goes low.
 * 
 * In test mode, i_step can also be used to allow the register to receive
 * a single rising edge. The register will latch vi_regs identically to 
 * normal operation, hence this is a method to single-step the clock.
 * o_success has a similar behavior where it goes high 1.5-2 periods after
 * i_step goes high, indicating that the register was cycled.
 * 
 * It is important that only one of the signals i_rd, i_wr, and i_step are
 * active at a time. Having more than one active at a time causes bus
 * contention and is undefined. it is also important to have AT LEAST 6
 * periods between when one signal becomes inactive and another becomes
 * active.
 * 
 */

module reg_slice (/*AUTOARG*/
		  // Outputs
		  vo_regs,
		  // Inouts
		  io_success, vio_tbus,
		  // Inputs
		  i_clk, i_rstb, i_ena, vi_regs, i_step, i_rd, i_wr
		  );
  
  parameter DWIDTH = 16;  // Register size
  
  input 	       i_clk; // Master clock
  input 	       i_rstb; // Master reset
  input 	       i_ena; // Asynchronous enable signal
  input [DWIDTH-1:0]   vi_regs; // Data to register
  output [DWIDTH-1:0]  vo_regs; // Data in register
  input 	       i_step; // Manually step the clock
  input 	       i_rd; // Put register data on test bus
  input 	       i_wr; // Load data on test bus into register
  inout 	       io_success; // Indicates a successful transaction
  inout [DWIDTH-1:0]   vio_tbus;
  
  // Synchronize the ena, rd, wr and step signals
  wire 		       ena_sync, rd_sync, wr_sync, step_sync;
  negedge_sync ena_synchro(.i_clk(i_clk), .i_async(i_ena),
			   .o_sync(ena_sync));
  negedge_sync rd_synchro(.i_clk(i_clk), .i_async(i_rd),
			  .o_sync(rd_sync));
  negedge_sync wr_synchro(.i_clk(i_clk), .i_async(i_wr),
			  .o_sync(wr_sync));
  negedge_sync step_synchro(.i_clk(i_clk), .i_async(i_step),
			    .o_sync(step_sync));
  /*
   * Generate the clock to the main register. When enabled, this clock
   * is simply i_clk. When the register is disabled, the register is 
   * clocked by clk_pulse, which is either the write or step clock.
   */
  wire 		       clk_pulse;
  wire 		       clk_regs = ena_sync ? i_clk : clk_pulse;
  reg 		       r_wr_prev, r_wr_pprev, r_step_prev;
  wire 		       clk_pulse_wr, clk_pulse_step;
  always @( posedge i_clk or negedge i_rstb ) begin : main_clk_reg
    if ( !i_rstb ) begin
      r_wr_prev <= 0;
      r_wr_pprev <= 0;
      r_step_prev <= 0;
    end
    else begin
      r_wr_prev <= wr_sync;
      r_wr_pprev <= r_wr_prev;
      r_step_prev <= step_sync;
    end
  end // block: main_clk_reg
  assign clk_pulse_wr = r_wr_prev && !r_wr_pprev;
  assign clk_pulse_step = step_sync && !r_step_prev;
  assign clk_pulse = wr_sync ? clk_pulse_wr : clk_pulse_step;
  /*
   * Generate clock to load the shadow register. This is a pulse which
   * rises on the falling edge of i_clk, and lasts half an i_clk period.
   * This is to ensure that there is no race condition between the Q output
   * of the main register (changes on the rising edge of i_clk).
   */
  reg 		       r_rd_prev;
  wire 		       clk_pulse_rd;
  always @( posedge i_clk or negedge i_rstb ) begin : shadow_clk_reg
    if ( !i_rstb )
      r_rd_prev <= 0;
    else
      r_rd_prev <= rd_sync;
  end
  assign clk_pulse_rd = rd_sync && !r_rd_prev;
  reg [DWIDTH-1:0]  rv_shadow;
  always @( posedge clk_pulse_rd or negedge i_rstb ) begin : shadow_reg
    if ( !i_rstb )
      rv_shadow <= 0;
    else
      rv_shadow <= vo_regs;
  end
  assign vio_tbus = (rd_sync && i_rd) ? rv_shadow : {DWIDTH{1'bz}};
  
  /*
   * Main register file. Input is v_regs_pre, which is multiplexed from
   * either the normal input when enabled, or the test bus when disabled.
   */
  reg [DWIDTH-1:0]  vo_regs;
  wire [DWIDTH-1:0] v_regs_pre;
  assign v_regs_pre = ( !ena_sync && wr_sync ) ? vio_tbus : vi_regs;
  always @( posedge clk_regs or negedge i_rstb ) begin : main_reg
    if ( !i_rstb )
      vo_regs <= 0;
    else
      vo_regs <= v_regs_pre;
  end
  /*
   * Generate the handshaking signal back to the register interface state
   * machine, indicating that the current action (rd, wr, step) completed
   */
  wire pre_io_success = r_rd_prev || ( !ena_sync && r_wr_pprev );
  assign io_success = ( i_rd || i_wr ) ? pre_io_success : 1'bz;
endmodule
