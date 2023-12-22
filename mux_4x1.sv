module mux_4x1(
	input [1:0] sel,
	input [31:0] input_a, input_b, input_c, input_d,
	output logic [31:0] out_y
);
	always_comb
	begin
		case(sel)
		2'b00: out_y = input_a;
		2'b01: out_y = input_b;
		2'b10: out_y = input_c;
		2'b11: out_y = input_d;
		default: out_y = input_a;
		endcase
	end
endmodule