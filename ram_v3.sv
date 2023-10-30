/*************************************************************
**     Author: Engineer Youssef Nasser                      **
**     Inferred Single-Port Memory Class-Based Verification **                 
**     Async reset and Sync Write and Read Operation        **
**     Copyright (c) 2023                             	    **
**     All rights reserved.                         	      **
************************************************************/

module ram_v3 (
  input logic [7:0] data_in,    // Data input (8 bits)
  input logic WE, RE,           // Write enable and Read enable
  input logic [7:0] addr,       // Address input (8 bits)
  input bit clk,               // Clock input
  input bit rst,               // Reset input
  output logic [7:0] data_out,  // Data output (8 bits)
  output logic valid_out        // Valid output
);

  reg [7:0] memory [255:0];    // 256 locations, each with 8 bits
  integer i;

  // Write and Read operations
  always @(posedge clk or negedge rst) begin
    if (!rst) begin
      valid_out <= 1'b0;
      data_out  <= 8'b0;

      // Reset the memory to all zeros
      for (i = 0; i < 256; i = i + 1) begin
        memory[i] <= 8'b0;
      end
    end
    else if (WE) begin
      // Write operation
      memory[addr] <= data_in;
      valid_out <= 1'b0;
    end
    else if (RE) begin
      // Read operation
      valid_out <= 1'b1;
      data_out <= memory[addr];
    end
    else
      valid_out <= 1'b0;
  end
endmodule
