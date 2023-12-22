module branch_cond (
    input logic [31:0] rdata1,
    input logic [31:0] rdata2,
    input logic [2:0] br_type,
    output logic br_taken
);

    parameter DEFAULT_CASE = 3'b010;
    parameter EQUAL = 3'b000;
    parameter NOTEQUAL = 3'b001;
    parameter LESSTHAN = 3'b100;
    parameter GREATEREQ = 3'b101;
    parameter ULESSTHAN = 3'b110;
    parameter UGREATEREQ = 3'b111;

    always_comb
    begin
        case (br_type)
            EQUAL:
                br_taken = $signed(rdata1) == $signed(rdata2) ? 1 : 0;
            NOTEQUAL:
                br_taken = $signed(rdata1) != $signed(rdata2) ? 1 : 0;
            LESSTHAN:
                br_taken = $signed(rdata1) < $signed(rdata2) ? 1 : 0;
            GREATEREQ:
                br_taken = $signed(rdata1) >= $signed(rdata2) ? 1 : 0;
            ULESSTHAN:
                br_taken = $unsigned(rdata1) < $unsigned(rdata2) ? 1 : 0;
            UGREATEREQ:
                br_taken = $unsigned(rdata1) >= $unsigned(rdata2) ? 1 : 0;
            DEFAULT_CASE:
            br_taken = 0;
        endcase
    end

endmodule