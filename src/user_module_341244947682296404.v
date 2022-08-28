`default_nettype none

// Keep I/O fixed for TinyTapeout
module user_module_341244947682296404(
	input  wire [7:0] io_in,
	output wire [7:0] io_out
);

	// Our toplevel module that was
	// generated with Amaranth.
	tinysoc i_soc (
		.io_in (io_in),
		.io_out(io_out)
	);
endmodule
