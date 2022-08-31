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

	reg [11:0] instr;

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
		// 8 11-bit instructions in chunks of
		// 2 6-bit words for each instruction.

		// imem[0] = IMM 4 -> Reg[3]
		instr = 12'b011_11_000_0100;
		io_in[7:2] = instr[5:0];  #CLK_PERIOD;
		io_in[7:2] = instr[11:6]; #CLK_PERIOD;

		// imem[1] = IMM 1 -> Reg[2]
		instr = 12'b011_10_000_0001;
		io_in[7:2] = instr[5:0];  #CLK_PERIOD;
		io_in[7:2] = instr[11:6]; #CLK_PERIOD;
		
		// imem[2] = ALU Reg[3] + Reg[2] -> Reg[3]
		instr = 12'b000_11_10_11_000;
		io_in[7:2] = instr[5:0];  #CLK_PERIOD;
		io_in[7:2] = instr[11:6]; #CLK_PERIOD;

		// imem[3] = JUMP -> [r2]
		instr = 12'b100_00_00_10_000;
		io_in[7:2] = instr[5:0];  #CLK_PERIOD;
		io_in[7:2] = instr[11:6]; #CLK_PERIOD;

		// imem[15:4]
		io_in[7:2] = 6'h0;
		repeat (48) #CLK_PERIOD

		repeat (16) #CLK_PERIOD;

		$finish;
    end
endmodule

