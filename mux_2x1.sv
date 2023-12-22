module mux_2x1(
	input sel,
	input [31:0] input_a, input_b,
	output logic [31:0] out_y
);
	always_comb
	begin
		case(sel)
		1'd0: out_y = input_a;
		1'd1: out_y = input_b;
		default: out_y = input_a;
		endcase
	end
endmodule