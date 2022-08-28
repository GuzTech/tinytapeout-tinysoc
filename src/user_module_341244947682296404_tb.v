`timescale 1ns / 1ps

`define assert(signal, value) \
    if (signal !== value) begin \
        $display("ASSERTION FAILED in %m"); \
        $finish; \
    end

module user_module_341244947682296404_tb;
    // Parameters
    parameter CLK_PERIOD = 10;

    // DUT I/O signals
    reg  [7:0] io_in;
    wire [7:0] io_out;

    // DUT
    user_module_341244947682296404
    DUT (
        .io_in (io_in),
        .io_out(io_out)
    );

    // Generate a clock.
    initial begin
        io_in[0] = 1'b0;
		
        forever #(CLK_PERIOD / 2) io_in[0] = !io_in[0];
    end

    // Generate a reset.
    initial begin
        io_in[1] = 1'b1;
        #(CLK_PERIOD)
        io_in[1] = 1'b0;
    end

    // Generate stimuli.
    initial begin
        $dumpfile("user_module_341244947682296404_tb.vcd");
        $dumpvars(0, user_module_341244947682296404_tb);
        $timeformat(-6, 2, " us", 16);

        // Initial values.
        io_in[7:2] = 6'd0;

        // Wait for reset.
        @(negedge io_in[1]);

		// We need to provide our design with
		// 16 16-bit instructions in chunks of
		// 4 nibbles for each instruction.

		// imem[0] = IMM 4 -> Reg[6]
		io_in[7:4] = 4'h4; #CLK_PERIOD;
		io_in[7:4] = 4'h0; #CLK_PERIOD;
		io_in[7:4] = 4'b10_00; #CLK_PERIOD;
		io_in[7:4] = 4'b011_1; #CLK_PERIOD;

		// imem[1] = IMM 1 -> Reg[5]
		io_in[7:4] = 4'h1; #CLK_PERIOD;
		io_in[7:4] = 4'h0; #CLK_PERIOD;
		io_in[7:4] = 4'b01_00; #CLK_PERIOD;
		io_in[7:4] = 4'b011_1; #CLK_PERIOD;
		
		// imem[2] = ALU Reg[6] + Reg[5] -> Reg[6]
		io_in[7:4] = 4'h0; #CLK_PERIOD;
		io_in[7:4] = 4'b1_110; #CLK_PERIOD;
		io_in[7:4] = 4'b10_10; #CLK_PERIOD;
		io_in[7:4] = 4'b000_1; #CLK_PERIOD;

		// imem[3] = JUMP -> 0
		io_in[7:4] = 4'h0; #CLK_PERIOD;
		io_in[7:4] = 4'h0; #CLK_PERIOD;
		io_in[7:4] = 4'h0; #CLK_PERIOD;
		io_in[7:4] = 4'h8; #CLK_PERIOD;

		// imem[15:4]
		io_in[7:4] = 4'h0;
		repeat (48) #CLK_PERIOD

		repeat (16) #CLK_PERIOD;

		$finish;
    end
endmodule

