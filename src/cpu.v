`default_nettype none

// Super simple 4-bit CPU!

// [14 13 12][11 10 9][8 7 6][5 4 3][2 1 0]
//  I-type       rd     rs2    rs1  ALU/Addr

// I-type [14:12]:
// 000: ALU
// 001: Load
// 010: Store
// 011: Immediate
// 100: Jump
// 101: Conditional jump
// 110: undefined
// 111: undefined

// ALU type:
// [14 13 12][11 10 9][8 7 6][5 4 3][2 1 0]
//   0  0  0     rd     rs2    rs1   A B C

// ALU instruction (A B C):
// 000: Add				= [rs1] + [rs2] -> [rd]
// 001: And				= [rs1] & [rs2] -> [rd]
// 010: Or					= [rs1] | [rs2] -> [rd]
// 011: Xor				= [rs1] ^ [rs2] -> [rd]
// 100: Not				= ~[rs1] -> [rd]
// 101: Eq					= [rs1] == [rs2] -> [rd][0]
// 110: Less than			= [rs1] < [rs2] -> [rd][0]
// 111: Shift left 1 bit	= [rs1] << 1 -> [rd]

// Load type:
// [14 13 12][11 10 9][8 7 6][5 4 3][2 1 0]
//   0  0  1     rd    X X X   rs1   X X X	(X = don't care)

// [[rs1]] -> [rd]

// Store type:
// [14 13 12][11 10 9][8 7 6][5 4 3][2 1 0]
//   0  1  0     rd    X X X   rs1   X X X	(X = don't care)

// [rs1] -> [[rd]]

// Immediate type:
// [14 13 12][11 10 9][8 7 6][5 4 3][2 1 0]
//   0  1  1     rd    X X X  X X D  C B A	(X = don't care)

// DCBA -> [rd]

// Jump type:
// [14 13 12][11 10 9][8 7 6][5 4 3][2 1 0]
//   1  0  0   X  X X  X X X   rs1   X X X

// [rs1] -> PC

module ALU (
    input  wire [3:0] in1,
    input  wire [3:0] in2,
    input  wire [2:0] alu,
    output reg  [3:0] out
);
    localparam [2:0] ALU_ADD = 3'b000;
    localparam [2:0] ALU_AND = 3'b001;
    localparam [2:0] ALU_OR  = 3'b010;
    localparam [2:0] ALU_XOR = 3'b011;
    localparam [2:0] ALU_NOT = 3'b100;
    localparam [2:0] ALU_EQ  = 3'b101;
    localparam [2:0] ALU_LT  = 3'b110;
    localparam [2:0] ALU_LSH = 3'b111;

    reg int_xor_src2;
    wire int_xor_out;

    assign int_xor_out = in1 ^ int_xor_src2;

    always @(*) begin
        case (alu)
            ALU_ADD: out = in1 + in2;
            ALU_AND: out = in1 & in2;
            ALU_OR:  out = in1 | in2;
            ALU_XOR: begin
                int_xor_src2 = in2;
                out          = int_xor_out;
            end
            ALU_NOT: begin
                int_xor_src2 = 4'b1111;
                out          = int_xor_out;
            end
            ALU_EQ: begin
                int_xor_src2 = in2;
                out          = !(|int_xor_out);
            end
            ALU_LT:  out = in1;// < in2;
            ALU_LSH: out = in1 << 1;
        endcase
    end
endmodule

module RegFileSingle #(
    parameter DATA_WIDTH = 4,
    parameter ADDR_WIDTH = 4
) (
    input  wire                  clk,
    input  wire                  rst,
    input  wire [ADDR_WIDTH-1:0] r_addr,
    output wire [DATA_WIDTH-1:0] r_data,
    input  wire [ADDR_WIDTH-1:0] w_addr,
    input  wire [DATA_WIDTH-1:0] w_data,
    input  wire                  wr
);

    reg [DATA_WIDTH-1:0] int_regs [(2**ADDR_WIDTH)-1:0];

    assign r_data = int_regs[r_addr];

    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < (2**ADDR_WIDTH); i = i + 1) begin
                int_regs[i] = {DATA_WIDTH{1'b0}};
            end
        end else begin
            if (wr) begin
                int_regs[w_addr] = w_data;
            end
        end
    end
endmodule

module RegFileDual #(
    parameter DATA_WIDTH = 4,
    parameter ADDR_WIDTH = 4
) (
    input  wire                  clk,
    input  wire                  rst,
    input  wire [ADDR_WIDTH-1:0] r_addr1,
    input  wire [ADDR_WIDTH-1:0] r_addr2,
    output wire [DATA_WIDTH-1:0] r_data1,
    output wire [DATA_WIDTH-1:0] r_data2,
    input  wire [ADDR_WIDTH-1:0] w_addr,
    input  wire [DATA_WIDTH-1:0] w_data,
    input  wire                  wr
);

    reg [DATA_WIDTH-1:0] int_regs [(2**ADDR_WIDTH)-1:0];

    assign r_data1 = int_regs[r_addr1];
    assign r_data2 = int_regs[r_addr2];

    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < (2**ADDR_WIDTH); i = i + 1) begin
                int_regs[i] = {DATA_WIDTH{1'b0}};
            end
        end else begin
            if (wr) begin
                int_regs[w_addr] = w_data;
            end
        end
    end
endmodule

module CPU (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,
    input  wire [14:0] instr,
    input  wire  [3:0] i_data,
    input  wire  [2:0] i_addr,
    output wire  [3:0] d_data_o,
    input  wire  [3:0] d_data_i,
    output wire  [2:0] d_addr,
    output wire        d_wr,
    output reg   [3:0] gpo
);
// ======== Misc ======== //
	wire [2:0] int_addr;
	
	// assign int_addr = instr[3:0];
    assign int_addr = int_rdata1;

// ======== Register file ======== //
    // Source register 1 address.
    wire [2:0] int_rs1;

    // Source register 2 address.
    wire [2:0] int_rs2;

    // Destination register address.
    wire [2:0] int_rd;

    // Source register 1 data.
    wire [3:0] int_rdata1;

    // Source register 1 data.
    wire [3:0] int_rdata2;

    // Destination register data.
    reg [3:0] int_ddata;

    // Register write flag.
    wire int_wr_reg;

    assign int_rs1 = instr[5:3];
    assign int_rs2 = instr[8:6];
    assign int_rd  = instr[11:9];

    RegFileDual #(
        .DATA_WIDTH(4),
        .ADDR_WIDTH(3)
    ) i_rf (
        .clk    (clk),
        .rst    (rst),
        .r_addr1(int_rs1),
        .r_addr2(int_rs2),
        .r_data1(int_rdata1),
        .r_data2(int_rdata2),
        .w_addr (int_rd),
        .w_data (int_ddata),
        .wr     (int_wr_reg)
    );

// ======== ALU ======== //

    // The ALU instruction.
    wire [2:0] int_alu;

    // The ALU result.
    wire [3:0] int_alu_out;

    assign int_alu = instr[2:0];

    ALU i_alu (
        .in1(int_rdata1),
        .in2(int_rdata2),
        .alu(int_alu),
        .out(int_alu_out)
    );

// ======== Decoding ======== //

    localparam [2:0] TYPE_ALU   = 3'b000;
    localparam [2:0] TYPE_LOAD  = 3'b001;
    localparam [2:0] TYPE_STORE = 3'b010;
    localparam [2:0] TYPE_IMM   = 3'b011;
    localparam [2:0] TYPE_JUMP  = 3'b100;
    localparam [2:0] TYPE_CJUMP = 3'b101;

    // The instruction type.
    wire [2:0] int_type;

    // Instruction type flags.
    wire int_type_alu;
    wire int_type_load;
    wire int_type_store;
    wire int_type_imm;
    wire int_type_jump;
    wire int_type_cjump;

    assign int_type       = instr[14:12];
    assign int_type_alu   = int_type == TYPE_ALU;
    assign int_type_load  = int_type == TYPE_LOAD;
    assign int_type_store = int_type == TYPE_STORE;
    assign int_type_imm   = int_type == TYPE_IMM;
    assign int_type_jump  = int_type == TYPE_JUMP;
    assign int_type_cjump = int_type == TYPE_CJUMP;

    assign int_wr_reg = en & (int_type_alu | int_type_load | int_type_imm);

// ======== PC ======== //

    // Program counter.
    reg [2:0] pc;

	assign i_addr = pc;

    always @(posedge clk) begin
        if (rst) begin
            pc <= 3'd0;
        end else if (en) begin
            if (int_type_jump) begin
                pc <= int_rdata1;
            end else if (int_type_cjump) begin
                if (int_rdata1[0]) begin
                    pc <= int_addr;
                end else begin
                    pc <= pc + 1'b1;
                end
            end else begin
                pc <= pc + 1'b1;
            end
        end
    end

// ======== I/O ======== //

	always @(posedge clk) begin
		if (rst) begin
			gpo <= 4'd0;
		end else begin
			if (int_type_store && int_addr[3]) begin
				gpo <= int_rdata1;
			end
		end
	end

	assign d_addr   = int_addr;
	assign d_data_o = int_rdata1;
	assign d_wr     = (int_type_store && (!int_addr[3]));

	always @(*) begin
		if (int_type_store) begin
			int_ddata = d_data_i;
		end else if (int_type_imm) begin
			int_ddata = instr[3:0];
		end else begin
			int_ddata = int_alu_out;
		end	
	end

endmodule

module tinysoc (
    input  wire [7:0] io_in,
    output wire [7:0] io_out
);
    wire clk;
    wire rst;
    assign clk = io_in[0];
    assign rst = io_in[1];

    // Indicates if the rom is done being initialized.
    reg int_rom_done;

    // We load a nibble per clock cycle,
    // so keep track of which nibble we are writing.
    reg [1:0] int_quintet_cntr;

    // Instruction memory location being written to.
    reg [3:0] int_imem_addr;

    // Store the first 2 "quintets".
    reg [9:0] int_wr_data;

    // The ROM data.
    wire [14:0] int_w_data;

    // The ROM write flag.
    wire int_wr;

    // The CPU instruction address.
    wire [2:0] int_cpu_i_addr;

    // The CPU instruction data.
    wire [14:0] int_cpu_instr;

	// The CPU data memory write data.
	wire [3:0] int_cpu_d_data_o;

	// The CPU data memory read data.
	wire [3:0] int_cpu_d_data_i;

	// The CPU data memory address.
	wire [2:0] int_cpu_d_addr;

	// The CPU data memory write flag.
	wire int_cpu_d_wr;

    assign int_w_data = {io_in[7:3], int_wr_data};
    assign int_wr     = (int_quintet_cntr == 2'd2);

    always @(posedge clk) begin
        if (rst) begin
            int_rom_done     <= 1'b0;
            int_quintet_cntr <= 2'd0;
            int_imem_addr    <= 3'd0;
            int_wr_data      <= 10'd0;
        end else begin
            if (!int_rom_done) begin
                if (int_quintet_cntr == 2'd0) begin
                    int_wr_data[4:0] <= io_in[7:3];
                    int_quintet_cntr <= int_quintet_cntr + 1'b1;
                end else if (int_quintet_cntr == 2'd1) begin
                    int_wr_data[9:5] <= io_in[7:3];
                    int_quintet_cntr <= int_quintet_cntr + 1'b1;
                end else if (int_quintet_cntr == 2'd2) begin
                    int_quintet_cntr <= 2'd0;
                    int_imem_addr    <= int_imem_addr + 1'b1;
                end

                if ((int_imem_addr == 3'd7) && (int_quintet_cntr == 2'd2)) begin
                    int_rom_done <= 1'b1;
                end
            end
        end
    end

    RegFileSingle #(
        .DATA_WIDTH(15),
        .ADDR_WIDTH(3)
    ) i_mem (
        .clk   (clk),
        .rst   (rst),
        .r_addr(int_cpu_i_addr),
        .r_data(int_cpu_instr),
        .w_addr(int_imem_addr),
        .w_data(int_w_data),
        .wr    (int_wr)
    );

	RegFileSingle #(
		.DATA_WIDTH(4),
		.ADDR_WIDTH(3)
	) d_mem (
        .clk   (clk),
		.rst   (rst),
		.r_addr(int_cpu_d_addr),
		.r_data(int_cpu_d_data_i),
		.w_addr(int_cpu_d_addr),
		.w_data(int_cpu_d_data_o),
		.wr    (int_cpu_d_wr)
	);

    CPU
    i_cpu (
        .clk     (clk),
        .rst     (rst),
        .en      (int_rom_done),
        .instr   (int_cpu_instr),
        .i_data  (int_cpu_instr),
        .i_addr  (int_cpu_i_addr),
        .d_data_o(int_cpu_d_data_o),
        .d_data_i(int_cpu_d_data_i),
        .d_addr  (int_cpu_d_addr),
        .d_wr    (int_cpu_d_wr),
        .gpo     (io_out[3:0])
    );
    
    assign io_out[7:4] = 4'd0;
endmodule