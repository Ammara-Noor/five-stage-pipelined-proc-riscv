module alu
(
    input  logic [ 3:0] aluop,
    input  logic [31:0] opr_a,
    input  logic [31:0] opr_b,
    output logic [31:0] opr_res
);

    //ALU OPS
    parameter ADD = 4'b0000;
    parameter SUB = 4'b0001;
    parameter SLL = 4'b0010;
    parameter SLT = 4'b0011;
    parameter SLTU = 4'b0100;
    parameter XOR = 4'b0101;
    parameter SRL = 4'b0110;
    parameter SRA = 4'b0111;
    parameter OR = 4'b1000;
    parameter AND = 4'b1001;
    parameter NULL = 4'b1010;

    always_comb
    begin
        case(aluop)
            ADD:
                opr_res = opr_a + opr_b; //Add
            SUB:
                opr_res = opr_a - opr_b; //Subtract
            OR:
                opr_res = opr_a | opr_b; //OR
            AND:
                opr_res = opr_a & opr_b; //AND
            XOR:
                opr_res = opr_a ^ opr_b; //XOR
            SLL:
                opr_res = opr_a << opr_b; //Shift Left Logical
            SRL:
                opr_res = opr_a >> opr_b; //Shift Right Logical
            SRA:
                opr_res = opr_a >>> opr_b; //Shift Right Arithmetic ($ to tell signed)
            SLT:
                opr_res = ($signed(opr_a) < $signed(opr_b)) ? 1 : 0; //Set Less Than
            SLTU:
                opr_res = (opr_a < opr_b) ? 1 : 0; //Set Less Than Unsigned
            NULL:
                opr_res = opr_b;  //Pass Operand B
        endcase
    end

endmodule