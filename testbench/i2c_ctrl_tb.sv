module i2c_ctrl_tb ();

logic clk, rst;
logic start;
logic [8-1:0] ctrl;
logic i2c_sclk_w, i2c_sclk_r;
logic i2c_sdat_w, i2c_sdat_r;
logic busy, error;

i2c_ctrl iicctrl(
    .clock_i          ( clk        ),
    .reset_n_i        ( rst        ),
    .cmd_strobe_i     ( start      ),
    .len_rd_i         ( 8'h00      ),
    .data_valid_o     (            ),
    .data_o           (            ),
    .ctrl_wr_i        ( ctrl       ),
    .data_available_i ( 1'b0       ),
    .data_read_o      (            ),
    .data_i           ( 8'h00      ),
    .i2c_sclk_o       ( i2c_sclk_w ),
    .i2c_sdat_o       ( i2c_sdat_w ),
    .i2c_sclk_i       ( i2c_sclk_r ),
    .i2c_sdat_i       ( i2c_sdat_r ),
    .busy_o           ( busy       ),
    .error_o          ( error      ),
    .ack_error_i      ( 1'b0       )
);

assign i2c_sclk_r = i2c_sclk_w;
assign i2c_sdat_r = i2c_sdat_w;

initial begin
    fork
        begin
            clk = 1'b0;
            forever begin
                #12.5ns
                clk = !clk;
            end
        end

        begin
            rst=0;
            for(int i=0; i < 20;i++) begin
                @(posedge clk);
            end
            rst=1;
        end

        begin
            ctrl=8'h00;
            start=1'b0;
            for(int i=0; i <50; i++) begin
                @(posedge(clk));
            end
            ctrl=8'h5a;
            start=1'b1;
            @(posedge clk);
            start=1'b0;
        end
    join
end

endmodule;
