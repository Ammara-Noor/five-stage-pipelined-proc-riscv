module data_mem (
    input logic clk, wr_en, rd_en,
    input logic [31:0] addr,
    input logic [31:0] wdata,
    input logic [2:0] mem_mode,
    output logic [31:0] out_data
);

    parameter BYTE = 3'b000;
    parameter HALFWORD = 3'b001;
    parameter WORD = 3'b010;
    parameter UBYTE = 3'b011;
    parameter UHALFWORD = 3'b100;

    logic [7:0] data_mem [100];

    //ASYNC LOAD
    always_comb
    begin
        if(rd_en)
        begin
        case(mem_mode)
            BYTE:
                out_data = $signed(data_mem[addr]);
            HALFWORD:
                out_data = $signed({data_mem[addr+1], data_mem[addr]});
            WORD:
                out_data = $signed({data_mem[addr+3], data_mem[addr+2], data_mem[addr+1], data_mem[addr]});
            UBYTE:
                out_data = {24'b0,{data_mem[addr]}};
            UHALFWORD:
                out_data = {16'b0,data_mem[addr], data_mem[addr+1]};
	    endcase
        end
    end

    //SYNC STORE
    always_ff @( posedge clk )
    begin
        case (mem_mode)
            BYTE:
            begin

                data_mem[addr] <= wdata[7:0];
            end
            HALFWORD:
            begin
                data_mem[addr]   <= wdata[7:0];
                data_mem[addr+1] <= wdata[15:8];

            end
            WORD:
            begin
                data_mem[addr]   <= wdata[7:0];
                data_mem[addr+1] <= wdata[15:8];
                data_mem[addr+2] <= wdata[23:16];
                data_mem[addr+3] <= wdata[31:24];
            end
        endcase

    end

endmodule