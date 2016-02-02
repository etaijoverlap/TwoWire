// avli2c_eeprom, an avalon interface for a 24LC02B I2C EEPROM. Uses the i2c_master core.
// (c) 2015 Franz Schanovsky <franz.schanovsky@gmail.com> 
// 
// This software is licensed under the EUPL V 1.1
//
// This software is provided "as is" without warranty of any kind, see the 
// respective section in the EUPL. USE AT YOUR OWN RISK.

module avli2c_address_data_buffer(
    input  logic        clock_i,
    input  logic        reset_i,
    input  logic [7:0]  address_i,
    input  logic        address_valid_i,
    input  logic [31:0] data_i,
    input  logic [3:0]  data_valid_i,
    input  logic        clear_i,
    output logic        i2c_data_available_o,
    output logic [7:0]  i2c_data_o,
    input  logic        i2c_read_data_i
    );

typedef struct {
    logic [7:0]  address_buf;
    logic        address_valid;
    logic [31:0] data_buf;
    logic [3:0]  data_valid;
}register_t;

register_t pres, next;

always_ff @(posedge clock_i or negedge reset_i)
begin
    int i;
    if (!reset_i) begin
        pres.address_buf   <=  8'b0;
        pres.address_valid <=  1'b0;
        pres.data_buf      <= 32'b0;
        pres.data_valid    <=  4'b0;
    end else begin
        pres <= next;
    end
end

always_comb
begin
    logic store_data;
    int i;
    int j;
    //default
    next = pres;

    i2c_data_available_o = 1'b0;
    if (pres.address_valid) i2c_data_available_o = 1'b1;
    for (i=0; i < 4; i++) begin
        if (pres.data_valid[i]) i2c_data_available_o = 1'b1;
    end

    if (address_valid_i) begin
        next.address_buf   <= address_i;
        next.address_valid <= 1'b1;
    end
    store_data = 1'b0;
    for (i=0; i < 4; i++) begin
        if (data_valid_i[i]) begin
            next.data_valid[i] = 1'b1;
            store_data = 1'b1;
        end
    end

    if (store_data) next.data_buf = data_i;

    i2c_data_o = 8'b0;
    if (i2c_read_data_i) begin
        if (pres.address_valid) begin
            next.address_valid = 1'b0;
            i2c_data_o = pres.address_buf;
        end else begin
            for (i=0; i < 4; i++) begin
                if (pres.data_valid[i]) begin
                    for (j=0; j < 8; j++)
                        i2c_data_o[j] = pres.data_buf[j+i*8];
                    next.data_valid[i] = 1'b0;
                    break;
                end
            end
        end
    end
    if (clear_i) begin
        next.data_valid = '0;
        next.data_buf = '0;
        next.address_buf = '0;
        next.address_valid = 1'b0;
    end
end

endmodule
