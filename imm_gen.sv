module imm_gen (
    input logic [31:0] inst,
    output logic [31:0] imm
);

    parameter ITYPEALO = 7'b0010011; // I type Arithmetic Logic Ops
    parameter ITYPELOAD = 7'b0000011; // I type Arithmetic Logic Ops
    parameter ITYPEJALR = 7'b1101111; // I type JALR
    parameter STYPE = 7'b0100011; // S-type
    parameter BTYPE = 7'b1100011; // B-type
    parameter UTYPELUI = 7'b0110111; // U-type LUI
    parameter UTYPEAUIPC = 7'b0010111; // U-type AUIPC
    parameter JTYPE = 7'b0010111; // J-Type



    always_comb
    begin
    case (inst[6:0])
        ITYPEALO:
            imm = $signed(inst[31:20]);
        ITYPEJALR:
            imm = $signed(inst[31:20]);
        ITYPELOAD:
            imm = $signed(inst[31:20]);
        STYPE:
            imm = $signed({inst[31:25], inst[11:7]});
        BTYPE:
            imm = $signed({inst[31], inst[7], inst[30:25], inst[11:8], 1'b0});
        UTYPEAUIPC:
            imm = {inst[31:12], 12'b0};
        UTYPELUI:
            imm = {inst[31:12], 12'b0};
        JTYPE:
            imm = $signed({inst[31], inst[19:12], inst[20], inst[30:21], 1'b0});
        default:
            imm = $signed(inst[31:20]);
    endcase
    end

endmodule