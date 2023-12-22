module processor
(
    input logic clk,
    input logic rst
);
    // wires
    logic        rf_en;
    logic        rf_en_id;
    logic        rf_en_mem;
    logic        rf_en_wb;
    logic        sel_a;
    logic        sel_a_ex;
    logic        sel_a_id;
    logic        sel_b;
    logic        sel_b_ex;
    logic        sel_b_id;
    logic [1:0]  sel_wb;
    logic [1:0]  sel_wb_ex;
    logic [1:0]  sel_wb_id;
    logic        rd_en;
    logic        rd_en_ex;
    logic        rd_en_id;
    logic        rd_en_mem;
    logic        wr_en;
    logic        csr_wr;
    logic        csr_rd;
    logic        t_intr;
    logic        e_intr;
    logic        is_mret;
    logic        intr;
    logic        csr_op_sel;
    logic [31:0] pc_in;
    
    logic [31:0] pc_out_if;
    logic [31:0] pc_out_id;
    logic [31:0] pc_out_ex;
    logic [31:0] pc_out_mem;
    logic [31:0] pc_out_wb;

    logic [31:0] epc;
    logic [31:0] selected_pc;

    logic [31:0] inst_if;
    logic [31:0] inst_id;
    
    logic [ 4:0] rd;
    logic [ 4:0] rd_id;
    logic [ 4:0] rd_ex;
    logic [ 4:0] rd_mem;
    logic [ 4:0] rd_wb;
    logic [ 4:0] rs1;
    logic [ 4:0] rs1_ex;
    logic [ 4:0] rs1_id;
    logic [ 4:0] rs2;
    logic [ 4:0] rs2_ex;
    logic [ 4:0] rs2_id;
    logic [ 6:0] opcode;
    logic [ 2:0] funct3;
    logic [ 6:0] funct7;
    logic [31:0] rdata1;
    logic [31:0] rdata1_ex;
    logic [31:0] rdata1_id;
    logic [31:0] rdata2;
    logic [31:0] rdata2_id;
    logic [31:0] rdata2_ex;
    logic [31:0] rdata2_mem;
    logic [31:0] opr_a;
    logic [31:0] opr_b;
    logic [31:0] imm;
    logic [31:0] imm_ex;
    logic [31:0] wdata;
    logic [31:0] wdata_id;
    logic [31:0] wdata_wb;

    logic [31:0] alu_out; // opr_res
    logic [31:0] alu_out_mem;
    logic [31:0] alu_out_if;
    logic [31:0] alu_out_wb;
    logic [31:0] alu_out_ex;

    logic [31:0] data_mem_out;
    logic [31:0] data_mem_out_mem;
    logic [31:0] data_mem_out_wb;
    logic [31:0] csr_data_out;
    logic [3 :0] aluop;
    logic [3 :0] aluop_id;
    logic [3 :0] aluop_ex;
    logic [2:0] mem_mode;
    logic [2:0] br_type;
    logic br_taken;
    logic jump;
    logic flush_ex;
    logic flush_id;
    logic stall_id;
    logic stall_if;
    logic [1:0] forward_opr_a;
    logic [1:0] forward_opr_b;
    logic [31:0] forwarded_opr_a;
    logic [31:0] forwarded_opr_b;

    // --------- Fetch -----------

    // program counter
    pc pc_i
    (
        .clk(clk),
        .rst(rst),
        .en(~stall_if),
        .pc_in(selected_pc),
        .pc_out(pc_out_if)
    );

    // instruction memory
    inst_mem inst_mem_i
    (
        .addr(pc_out_if),
        .data(inst_if)
    );

    mux_2x1 pc_sel_mux
    (
        .sel(br_taken | jump),
        .input_a(pc_out_if + 32'd4),
        .input_b(alu_out_mem),
        .out_y(pc_in)
    );

    mux_2x1 pc_final_sel
    (
        .sel(intr),
        .input_a(pc_in),
        .input_b(epc),
        .out_y(selected_pc)
    );  

     // Fetch & Decode buffer 
    always_ff @( posedge clk ) 
    begin
        if (rst)
        begin
            inst_id <= 0;
            pc_out_id <= 0;
        end 
        else if (~flush_id)
        begin
            inst_id <= 32'h00000013; // NOP
            pc_out_id <= 'b0;
        end
        else if (~stall_id) // enable
        begin
            inst_id <= inst_if;
            pc_out_id <= pc_out_if;
        end       
    end

    // --------- Decode -----------

     // instruction decoder
    inst_dec inst_dec_i
    (
        .inst(inst_id),
        .rs1(rs1_id),
        .rs2(rs2_id),
        .rd(rd_id),
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7)
    );

    // register file
    reg_file reg_file_i
    (
        .clk(clk),
        .rf_en(rf_en_id),
        .waddr(rd_wb),
        .rs1(rs1_id),
        .rs2(rs2_id),
        .rdata1(rdata1_id),
        .rdata2(rdata2_id),
        .wdata(wdata_id)
    );

    // controller
    controller controller_i
    (
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7),
        .aluop(aluop_id),
        .sel_b(sel_b_id),
        .sel_wb(sel_wb_id),
        .rd_en(rd_en_id),
        .wr_en(wr_en_id),
        .mem_mode(mem_mode),
        .br_type(br_type),
        .jump(jump),
        .csr_rd(csr_rd),
        .csr_wr(csr_wr),
        .is_mret(is_mret),
        .csr_op_sel(csr_op_sel)
    );

    //immediate generator
    imm_gen imm_gen_i
    (
        .inst(inst_id),
        .imm(imm_id)
    );

    // Decode & Execute buffer
    always_ff @( posedge clk ) 
    begin
        if (rst | flush_ex)
        begin
           
            rdata1_ex <= 0;
            rdata2_ex <= 0;      
            pc_out_ex <= 0;
            imm_ex <= 0;
            rd_en_ex <= 0;
            sel_a_ex <= 0;
            sel_b_ex <= 0;
            sel_wb_ex <= 0;
            aluop_ex <= 0;   
            rd_ex <= 0;
            rs1_ex <= 0;
            rs2_ex <= 0;
        end 
        else
        begin
            
            rd_en_ex <= rd_en_id;
            sel_a_ex <= sel_a_id;
            sel_b_ex <= sel_b_id;
            imm_ex <= imm_id;
            aluop_ex <= aluop_id;   
            rdata1_ex <= rdata1_id;
            rdata2_ex <= rdata2_id;
            pc_out_ex <= pc_out_id;
            rd_ex <= rd_id;
            rs1_ex <= rs1_id;
            rs2_ex <= rs2_id;
            sel_wb_ex <= sel_wb_id;
        end       
    end
    
    always_comb 
    begin
        pc_out_if = pc_out_id;
    end

    // --------- Execute -----------

    always_comb 
    begin
        case(forward_opr_a)
        2'b00:   forwarded_opr_a = rdata1_ex;
        2'b01:   forwarded_opr_a = alu_out_mem;
        2'b10:   forwarded_opr_a = alu_out_wb;
        default: forwarded_opr_a = 'b0;
        endcase         
    end

    always_comb 
    begin
        case(forward_opr_b)
        2'b00:   forwarded_opr_b = rdata1_ex;
        2'b01:   forwarded_opr_b = alu_out_mem;
        2'b10:   forwarded_opr_b = alu_out_wb;
        default: forwarded_opr_b = 'b0;
        endcase         
    end    

    // alu
    alu alu_i
    (
        .aluop(aluop_ex),
        .opr_a(opr_a),
        .opr_b(opr_b),
        .opr_res(alu_out_ex)
    );

    //branch comparator
    branch_cond branch_cond_i
    (
        .rdata1(rdata1),
        .rdata2(rdata2),
        .br_type(br_type),
        .br_taken(br_taken)
    );

    //sel_a_mux
    mux_2x1 sel_a_mux
    (
        .sel(sel_a_ex),
        .input_a(forwarded_opr_a),
        .input_b(pc_out_ex),
        .out_y(opr_a)
    );

    //sel_b_mux for I-type
    mux_2x1 sel_b_mux
    (
        .sel(sel_b_ex),
        .input_a(rdata2_ex),
        .input_b(imm_ex),
        .out_y(opr_b)
    );

    // Execute & Memory buffer
    always_ff @( posedge clk ) 
    begin
        if (rst)
        begin
            alu_out_mem <= 0;
            rdata2_mem <= 0;
            rd_en_mem <= 0;
            pc_out_mem <= 0;
            rd_mem <= 0;
      
        end 
        else
        begin
            alu_out_mem <= alu_out_ex;
            rdata2_mem <= rdata2_ex;     
            rd_en_mem <= rd_en_ex;
            pc_out_mem <= pc_out_ex;
            rd_mem <= rd_ex;
        end       
    end

    // --------- Memory ------------
    data_mem data_mem_i
    (
        .clk(clk),
        .rd_en(rd_en_mem),
        .wr_en(wr_en),
        .addr(alu_out_mem),
        .wdata(rdata2_mem),
        .mem_mode(mem_mode),
        .out_data(data_mem_out_mem)
    );

    always_comb
    begin
        alu_out_if = alu_out_mem;
    end

    //csr
    csr csr_i
    (
        .clk(clk),
        .rst(rst),
        .pc_input(pc_in),
        .addr(alu_out),
        .wdata(csr_op),
        .inst(inst),
        .csr_rd(csr_rd),
        .csr_wr(csr_wr),
        .t_intr(t_intr),
        .e_intr(e_intr),
        .is_mret(is_mret),
        .rdata(csr_data_out),
        .epc(epc),
        .intr(intr)
    ); 

    mux_2x1 sel_csr_op
    (
        .sel(csr_op_sel),
        .input_a(rdata1),
        .input_b(imm),
        .out_y(csr_op)
    );
    
    // --------- Writeback ---------
    //write back selection for load instructions
    mux_4x1 sel_wb_mux
    (
        .sel(sel_wb),
        .input_a(alu_out_wb),  
        .input_b(data_mem_out_wb),
        .input_c(pc_out_wb+4),
        .input_d(csr_data_out),
        .out_y(wdata)
    );

    // Memory & Writeback buffer
    always_ff @( posedge clk ) 
    begin
        if (rst)
        begin
            data_mem_out_wb <= 0;
            alu_out_wb <= 0;
            pc_out_wb <= 0;
            rd_wb <= 0;
      
        end 
        else
        begin
            data_mem_out_wb <= data_mem_out_mem;
            alu_out_wb <= alu_out_mem;
            pc_out_wb <= pc_out_mem;
            rd_wb <= rd_mem;
        end       
    end

    always_comb 
    begin
        wdata_id = wdata_wb;         
    end

    // --------- Hazard Unit ---------
    hazard_unit hazard_unit_i
    (
        .rf_en_mem(rf_en_mem),
        .rf_en_wb(rf_en_wb),
        .rs1_ex(rs1_ex),
        .rs2_ex(rs2_ex),
        .rd_mem(rd_mem),
        .rd_wb(rd_wb),
        .forward_a(forward_opr_a),
        .forward_b(forward_opr_b),
        .rd_ex(rd_ex),
        .rs1_id(rs1_id),
        .rs2_id(rs2_id),
        .sel_wb_ex(sel_wb_ex),
        .stall_if(stall_if),
        .stall_id(stall_id),
        .flush_ex(flush_ex),
        .br_taken(br_taken),
        .flush_id(flush_id)
    );

endmodule