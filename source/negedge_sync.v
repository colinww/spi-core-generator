/*
 * Copyright 2013-2016 Colin Weltin-Wu (colinww@gmail.com)
 * UC San Diego Integrated Signal Processing Group
 *
 * Licensed under GNU General Public License 3.0 or later. 
 * Some rights reserved. See LICENSE.
 * 
 * negedge_sync.v
 * Used to synchronize a signal coming from an async domain. Signal is 
 * sampled on the rising edge, but changes only on falling edges. Used
 * for synchronizing control signals that go to rising edge logic. This
 * block has no reset.
 * 
 * The idea behind allowing its output to change on the falling edge of the
 * clock is that it is more likely for there to be positive clock skew in 
 * downstream logic so having a signal change on the rising edge risks
 * impinging on the data hold window.
 * 
 * 1/2 clock period delay.
 */

module negedge_sync (i_clk, i_async, o_sync);
   input  i_clk;
   input  i_async;
   output o_sync;

   reg 	  samp_rising;
   reg 	  o_sync;

   initial begin
      samp_rising <= 1'b0;
      o_sync <= 1'b0;
   end

   always @( posedge i_clk )
     samp_rising <= i_async;

   always @( negedge i_clk )
     o_sync <= samp_rising;

endmodule
