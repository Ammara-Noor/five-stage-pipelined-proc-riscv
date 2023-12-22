module controller
(
    input  logic [6:0] opcode,
    input  logic [6:0] funct7,
    input  logic [2:0] funct3,
    output logic [3:0] aluop,
    output logic       rf_en,
    //selection of operand B in ALU
    output logic       sel_b,
    output logic       sel_a,
    output logic [1:0] sel_wb,
    //data memory signals
    output logic       rd_en,
    output logic       wr_en,
    output logic [2:0] mem_mode,
    output logic [2:0] br_type,
    output logic       jump,
    output logic       is_mret,
    output logic       csr_rd,
    output logic       csr_wr,
    output logic       csr_op_sel
);

    parameter RTYPE = 7'b0110011;
    parameter ITYPEALO = 7'b0010011; // I type Arithmetic Logic Ops
    parameter ITYPELOAD = 7'b0000011; // I type Arithmetic Logic Ops
    parameter ITYPEJALR = 7'b1100111; // I type JALR
    parameter STYPE = 7'b0100011; // S-type
    parameter BTYPE = 7'b1100011; // B-type
    parameter UTYPELUI = 7'b0110111; // U-type LUI
    parameter UTYPEAUIPC = 7'b0010111; // U-type AUIPC
    parameter JTYPE = 7'b1101111; // J-type
    parameter CSRI = 7'b111011; // CSR

    // ALU OPS
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
        case(opcode)
         RTYPE: //R-Type
            begin
                rf_en = 1'b1;
                sel_b = 1'b0;
                sel_a = 1'b0;
                sel_wb = 2'b00;
                jump = 0;
                br_type = 010;
                case(funct3)
                    3'b000:
                    begin
                        case(funct7)
                            7'b0000000: aluop = ADD; //ADD
                            7'b0100000: aluop = SUB; //SUB
                        endcase
                    end
                    3'b001: aluop = SLL; //Shift Left Logical
                    3'b010: aluop = SLT; //Set Less Than
                    3'b011: aluop = SLTU; //Set Less Than Unsigned
                    3'b100: aluop = XOR; //XOR
                    3'b101:
                    begin
                        case(funct7)
                            7'b0000000: aluop = SRL; //Shift Right Logical
                            7'b0100000: aluop = SRA; //Shift Right Arithmetic
                        endcase
                    end
                    3'b110: aluop = OR; //OR
                    3'b111: aluop = AND; //AND
                endcase
            end
            ITYPEALO: // I-type Arithmetic Logic Ops
            begin
                rf_en = 1'b1;
                sel_b = 1'b1;
                sel_a = 1'b0;
                sel_wb = 2'b00;
                br_type = 010;
                jump = 0;
                case (funct3)
                3'b000: aluop = ADD; //ADDI
                3'b010: aluop = SLT; //SLTI
                3'b011: aluop = SLTU; //SLTIU
                3'b100: aluop = XOR; //XORI
                3'b110: aluop = OR; //ORI
                3'b111: aluop = AND; //ANDI
                3'b001: aluop = SLL; //SLLI
                3'b101:
                begin
                    case (funct7)
                    7'b0000000: aluop = SRL; //SRLI
                    7'b0100000: aluop = SRL; //SRAI
                    endcase
                end
                endcase
            end
            ITYPELOAD: // I-type Load

            begin
                br_type = 010;
                rf_en = 1'b1;
                sel_a = 0;
                sel_b = 1'b1;
                sel_wb = 2'b01;
                rd_en = 1;
                wr_en = 0;
                jump = 0;
                aluop = ADD;
                case (funct3)
                    3'b000: //load byte
                        mem_mode = 3'b000;
                    3'b001: //load half word
                        mem_mode = 3'b001;

                    3'b010: //load word
                        mem_mode = 3'b010;

                    3'b100: //load byte unsigned
                        mem_mode = 3'b011;

                    3'b101: //load half unsigned
                        mem_mode = 3'b100;

                endcase
            end

            ITYPEJALR: //I-type (Jump And Link Return)
            begin
                rf_en = 1;
                aluop = ADD;
                sel_b = 1;
                sel_a = 0;
                sel_wb = 2'b10;
                jump = 1;
                br_type = 010;
            end

            STYPE: //S-type
            begin
                aluop = ADD;
                sel_a = 0;
                sel_b = 1;
                rf_en = 0;
                rd_en = 0;
                wr_en = 1;
                jump = 0;
                br_type = 010;
                case (funct3)

                    3'b000: //store byte
                        mem_mode = 3'b000;
                    3'b001: //store halfword
                        mem_mode = 3'b001;
                    3'b010: //store word
                        mem_mode = 3'b010;

                endcase
            end
            BTYPE: //B-type
            begin
                sel_a = 1;
                sel_b = 1;
                rf_en = 0;
                aluop = ADD;
                br_type = funct3;
                // case (funct3)
                //     3'b000: // ==
                //         br_type = 3'b000;
                //     3'b001: // !=
                //         br_type = 3'b001;
                //     3'b100: // <
                //         br_type = 3'b100;
                //     3'b101: // >=
                //         br_type = 3'b101;
                //     3'b110: // < (unsigned)
                //         br_type = 3'b110;
                //     3'b111: // >= (unsigned)
                //         br_type = 3'b111;

                //     default:
                // endcase
            end

            UTYPELUI: //U-type (Load Upper Immediate)
            begin
                rf_en = 1;
                aluop = NULL;
                sel_b = 1;
                sel_wb = 2'b00;
                jump = 0;
                br_type = 010;

            end

            UTYPEAUIPC: //U-type (Add Upper Immediate to Program Counter)
            begin
                rf_en = 1;
                aluop = ADD;
                sel_a = 1;
                sel_b = 1;
                sel_wb = 2'b00;
                jump = 0;
                br_type = 010;
            end

            JTYPE: //J-type (Jump And Link)
            begin
                $display("JAL\n");
                sel_wb = 2'b10;
                rf_en = 1;
                aluop = ADD;
                sel_b = 1;
                sel_a = 1;
                jump = 1;
                br_type = 010;
            end

            default:
            begin
                rf_en = 1'b0;
                sel_b = 1'b0;
            end

            CSRI:
            begin
                case (funct3)
                    3'b000: // MRET
                    begin
                        rf_en = 1'b0;
                        sel_a = 1'b1;
                        sel_b = 1'b0;
                        rd_en = 1'b0;
                        sel_wb = 2'b01;
                        wr_en = 1'b0;
                        mem_mode = 3'b111;
                        br_type = 3'b010;
                        csr_rd = 1'b0;
                        csr_wr = 1'b0;
                        is_mret = 1'b1;
                    end
                    3'b001: //CSRRW
                    begin
                        rf_en = 1'b1;
                        sel_a = 1'b1;
                        sel_b = 1'b0;
                        rd_en = 1'b0;
                        sel_wb = 2'b11;
                        wr_en = 1'b0;
                        mem_mode = 3'b111;
                        br_type = 3'b010;
                        csr_rd = 1'b1;
                        csr_wr = 1'b1;
                        is_mret = 1'b0;
                    end
                endcase
            end
        endcase
    end

endmodule