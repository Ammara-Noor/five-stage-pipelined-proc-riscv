module tb_processor();

    logic clk;
    logic rst;

    processor dut
    (
        .clk ( clk ),
        .rst ( rst )
    );

    // clock generator
    initial
    begin
        clk = 0;
        forever
        begin
            #5 clk = 1;
            #5 clk = 0;
            // $display("pc: %b", dut.pc_out);


            // $display("oprA:%b", dut.alu_i.opr_a);
		    // $display("oprB:%b", dut.alu_i.opr_b);
		    // $display("oprRes:%b\n", dut.alu_i.opr_res);
            // $display("pc_sel_br:%b", dut.br_taken);
		    // $display("pc_sel_j:%b\n", dut.jump);
		    // $display("wb_selected:%b\n", dut.sel_wb_mux.out_y);
        end
    end

    // reset generator
    initial
    begin
        rst = 1;
        #10;
        rst = 0;
        #150;
        $display("Processor is running");
        $display("x1: %b", dut.reg_file_i.reg_mem[1]);
        $display("x2: %b", dut.reg_file_i.reg_mem[2]);
        $display("x1+x2-> x3: %b", dut.reg_file_i.reg_mem[3]);
        $display("loaded in x4: %b", dut.reg_file_i.reg_mem[4]);
        $display("JAL return address x5: %b", dut.reg_file_i.reg_mem[5]);
        $display("loaded UI in x6: %b", dut.reg_file_i.reg_mem[6]);
        $display("loaded UI+PC in x7: %b", dut.reg_file_i.reg_mem[7]);
        
        
        $finish;
    end

    // initialize memory
    initial
    begin
        $readmemh("inst.mem", dut.inst_mem_i.mem);
        $readmemb("rf.mem", dut.reg_file_i.reg_mem);
    end

    // dumping the waveform
    initial
    begin
        $dumpfile("processor.vcd");
        $dumpvars(0, dut);
    end

endmodule