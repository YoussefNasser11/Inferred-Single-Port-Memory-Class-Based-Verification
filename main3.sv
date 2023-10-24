/*********************************************************
**     Author: Engineer Youssef Nasser                  **
**     Classical Testbench using Classes for RAM Module **
**     Copyright (c) 2023.                            	**
**     All rights reserved.                           	**
*********************************************************/
package youssef;
  parameter clk_period = 10;

  // Definition of the 'transaction' class
  class transaction;
    rand bit rst;                   // Reset signal
    rand logic [7:0] data_in;       // Data input (8 bits)
    rand logic WE;                 // Write enable
    rand logic RE;                 // Read enable
    randc logic [7:0] addr;        // Address input (8 bits)
    logic [7:0] data_out;          // Data output (8 bits)
    logic valid_out;               // Valid output

    function new;
      // Constructor for 'transaction' class
      this.rst = 1'b0;
      this.data_in = 8'b0;
      this.WE = 1'b0;
      this.RE = 1'b0;
      this.addr = 8'b0;
      this.data_out = 8'b0;
      this.valid_out = 1'b0;
    endfunction

    // Randomization Constraints
    constraint const0 { data_in inside {[0:((2**8)-1)]}; }
    constraint const1 { WE dist {0:=100, 1:=100}; }
    constraint const2 { RE dist {0:=100, 1:=100}; }
    // constraint const3 { rst dist {0:=1, 1:=10000}; }
    constraint const4 { addr inside {[0:((2**8)-1)]}; }
  endclass

  // Definition of the 'sequencer' class
  class sequencer;
    transaction t_in;

    // Constructor for 'sequencer' class
    function new();
      this.t_in = new;
    endfunction

    task Write();
      // Task to perform a write operation
      void'(t_in.randomize() with { WE == 1 && RE == 0; });
    endtask

    task Read();
      // Task to perform a read operation
      void'(t_in.randomize() with { RE == 1 && WE == 0; });
    endtask
  endclass

  // Definition of the 'driver' class
  class driver;
    virtual intf1 virtual_interface1;

    // Constructor for 'driver' class
    function new(virtual intf1 reciving);
      virtual_interface1 = reciving;
    endfunction

    task my_rst;
      // Task to reset the RAM
      virtual_interface1.rst <= 1'b0;
      #20;
      virtual_interface1.rst <= 1'b1;
    endtask

    task Send_to_RAM(transaction t_in);
      // Task to send a transaction to the RAM
      @(negedge virtual_interface1.clk)
      begin
        virtual_interface1.data_in <= t_in.data_in;
        virtual_interface1.WE <= t_in.WE;
        virtual_interface1.RE <= t_in.RE;
        virtual_interface1.addr <= t_in.addr;
        //virtual_interface1.rst <= t_in.rst;
      end
    endtask
  endclass

  // Definition of the 'subscriber' class
  typedef class subscriber;

  // Definition of the 'scoreboard' class
  typedef class scoreboard;

  // Definition of the 'monitor' class
  class monitor;
    reg [7:0] temp_reg;
    transaction t2;               // Instantiation of a transaction object
    virtual intf1 virtual_interface2;

    // Constructor for 'monitor' class
    function new(virtual intf1 reciving);
      virtual_interface2 = reciving;
      this.t2 = new;
    endfunction

    task get_from_RAM();
      // Task to get data from the RAM
      @(negedge virtual_interface2.clk)
      begin
        if (virtual_interface2.valid_out) begin
          t2.data_out <= virtual_interface2.data_out;
          t2.valid_out <= virtual_interface2.valid_out;
          t2.data_in <= virtual_interface2.data_in;
          t2.WE <= virtual_interface2.WE;
          t2.RE <= virtual_interface2.RE;
          t2.addr <= virtual_interface2.addr;
          t2.rst <= virtual_interface2.rst;
        end
      end

      $display("trans of monitor %0t %p", $time, t2);
    endtask

    task checking(scoreboard arg2);
      // Task to check data in the scoreboard
      @(posedge virtual_interface2.clk)
      begin
        arg2.my_checker(t2);
      end
    endtask

    task hacker_write(subscriber arg1);
      // Task for simulating a write operation by a subscriber
      @(posedge virtual_interface2.clk)
      begin
        fork
        begin
          arg1.subscriber_common(t2);
        end
        join
      end
    endtask

    task hacker_read(subscriber arg1);
      // Task for simulating a read operation by a subscriber
      @(posedge virtual_interface2.clk)
      begin
        fork
        begin
          arg1.subscriber_common(t2);
        end
        begin
          arg1.subscriber_read(t2);
        end
        join
      end
    endtask
  endclass

  // Definition of the 'subscriber' class continued
  class subscriber;
    transaction t2;

    covergroup cg_data_valid();
    valid_out_cp: coverpoint t2.valid_out;
    endgroup

    covergroup cg_data_out();
    data_out_cp: coverpoint t2.data_out;
    endgroup

    covergroup cross_grp();
    cv1: coverpoint t2.data_out;
    cv2: coverpoint t2.valid_out;
    cross cv1, cv2;
    endgroup

    // Constructor for 'subscriber' class
    function new();
      this.t2 = new;
      cg_data_valid = new;
      cg_data_out = new;
      cross_grp = new;
    endfunction

    task subscriber_common(transaction tx2);
      // Task to handle a transaction in the subscriber
      t2 = tx2;
      cg_data_valid.sample();
      cross_grp.sample();
      $display("time is %0t coverage_valid is %f ", $time, cg_data_valid.get_coverage());
    endtask

    task subscriber_read(transaction tx3);
      // Task to handle a read transaction in the subscriber
      t2 = tx3;
      cg_data_out.sample();
      cross_grp.sample();
      $display("time is %0t coverage is %f ", $time, cg_data_out.get_coverage());
    endtask
  endclass

  // Definition of the 'scoreboard' class continued
  class scoreboard;
    static int error_Count;
    static int successfully_test;
    reg [7:0] golden_memory [255:0];
    reg [7:0] expected_memory;
    transaction score;

    // Constructor for 'scoreboard' class
    function new();
      this.score = new;
    endfunction

    task my_checker(transaction scorex);
      // Task to check data in the scoreboard
      score = scorex;

      if (score.rst == 1'b0) begin
        expected_memory <= 0;

        for (int i = 0; i < 256; i = i + 1) begin
          golden_memory[i] <= 8'b0;
        end
      end
      else if (score.WE) begin
        golden_memory[score.addr] <= score.data_in;
      end
      else if (score.RE) begin
        expected_memory <= golden_memory[score.addr];
      end

      this.compare;
      $display("trans of score is %p at time %0t", score, $time);
    endtask

    task compare();
      // Task to compare the data
      if (score.rst == 1'b0) begin
        if (score.valid_out == 1'b0) begin
          successfully_test++;
          $display("functionality is correct");
        end
        else begin
          $display("error @ this transaction at valid_out %0t %p", $time, score);
          error_Count++;
        end
      end

      if (expected_memory != score.data_out) begin
        $display("error");
        error_Count++;
        $display("error @ this transaction %0t %p", $time, score);
      end
      else begin
        $display("test case passed successfully");
        successfully_test++;
        $display("test passed and the transaction was %p at time %0t", score, $time);
      end
    endtask

    task Errors();
      // Task to display errors and test results
      $display("number of succeed tests is %0d", successfully_test);
      if (error_Count == 0)
        $display("Simulation completed successfully!");
      else
        $display("the number of errors is %0d", error_Count);
    endtask
  endclass

  // Definition of the 'environment' class
  class environment;
    sequencer seq;
    driver drv;
    monitor mon;
    subscriber covering;
    scoreboard golden_model;

    // Constructor for 'environment' class
    function new(virtual intf1 intf);
      this.seq = new;
      this.drv = new(intf);
      this.mon = new(intf);
      this.covering = new();
      this.golden_model = new();
    endfunction

    task RAM_reset;
      // Task to reset the RAM
      drv.my_rst;
    endtask

    task Write_Generator;
      // Task to generate write operations
      seq.Write();
      drv.Send_to_RAM(seq.t_in);
      mon.get_from_RAM();
      mon.hacker_write(covering);
    endtask

    task Read_Generator;
      // Task to generate read operations
      seq.Read();
      drv.Send_to_RAM(seq.t_in);
      mon.get_from_RAM();
      mon.hacker_read(covering);
      mon.checking(golden_model);
    endtask

    task errorss();
      // Task to handle errors and test results
      golden_model.Errors();
    endtask
  endclass
endpackage

// Definition of the 'intf1' interface
interface intf1;
  parameter clk_period = 10; // 1000/32MHz
  logic [7:0] data_in = 0;    // Data input (8 bits)
  logic WE = 0;              // Write enable
  logic RE = 0;              // Read enable
  logic [7:0] addr = 0;      // Address input (8 bits)
  bit clk;                   // Clock input
  bit rst = 0;               // Reset input
  logic [7:0] data_out;
  logic valid_out;

  always #(clk_period / 2) clk = ~clk;
endinterface

module tb3;
  `timescale 1ns/1ps
  import youssef::*;

  environment env;
  intf1 intf();
  ram_v3 dut(intf.data_in, intf.WE, intf.RE, intf.addr, intf.clk, intf.rst, intf.data_out, intf.valid_out);
  static int i;

  initial begin
    $dumpfile("wave.vcd");
    $dumpvars;
  end

  int iterations_WRITE = 20;
  int iterations_READ = 20;
  initial begin
    env = new(intf);
    env.RAM_reset;
    for (i = 0; i < iterations_WRITE; i++) begin
      env.Write_Generator();
    end
    for (i = 0; i < iterations_READ; i++) begin
      env.Read_Generator();
    end
    #30;
    $finish;
  end

  final begin
    env.errorss();
    $display("number of transactions is %0d", i);
  end
endmodule
