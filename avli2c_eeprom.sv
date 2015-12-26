module avli2c_eeprom #(
    ADDRESS_WIDTH=15
) (
    input logic clock_i,
    input logic reset_i,
    
    output logic       avl_waitrequest_o,
    output logic[31:0] avl_readdata_o,
    output logic       avl_readdatavalid_o,
    input  logic[4:0]  avl_burstcount_i,
    input  logic[31:0] avl_writedata_i,
    input  logic[ADDRESS_WIDTH-1:0]  avl_address_i,
    input  logic       avl_write_i,
    input  logic       avl_read_i,
    input  logic[3:0]  avl_byteenable_i,

    output logic       i2c_cmd_strobe_o,
    output logic[7:0]  i2c_ctrl_wrd_o,
    output logic       i2c_data_available_o,
    input  logic       i2c_read_data_i,
    output logic[7:0]  i2c_data_o,
    output logic[7:0]  i2c_len_read_o,
    input  logic[7:0]  i2c_data_i,
    input  logic       i2c_data_valid_i,
    input  logic       i2c_busy_i,
    input  logic       i2c_error_i,
    output logic       i2c_ack_error_o
);

typedef enum {
    IDLE,
    READ_SETADDR, READ_SETADDR_WAITFOR_I2C, READ_START_READ, READ_WAITFOR_I2C,
    WRITE_START_I2C, WRITE_WAITFOR_I2C
} if_state_t;
typedef logic[ADDRESS_WIDTH-1:0] address_t;

typedef struct {
    if_state_t   if_state;
    logic [31:0] avl_data;
    address_t    address;
    logic [3:0]  byteenable;
} register_t;

const register_t RESET = '{IDLE, 32'b0, address_t'(0), 4'b0};

register_t pres, next;

logic load_address, load_data;
logic[7:0] address_buf, address_val;
logic[7:0] data_buf, data_val;
logic address_available;
logic data_available;
logic read_request_delayed;
logic[7:0] i2c_data_buffer;

always_comb
begin
    next = pres;

    avl_waitrequest_o   = 1'b0;
    avl_readdata_o      = 32'b0;
    avl_readdatavalid_o = 1'b0;

    i2c_cmd_strobe_o     = 1'b0;
    i2c_ctrl_wrd_o       = 8'b0;
    i2c_len_read_o       = 8'b0;
    i2c_ack_error_o      = 1'b0;

    i2c_cmd_strobe_o = 1'b0;
    
    address_val = 8'b0;
    data_val = 8'b0;
    load_address = 1'b0;
    load_data = 1'b0;

    avl_readdata_o = pres.avl_data;

    unique case(pres.if_state)
        IDLE: begin
            next.avl_data    = 32'b0;
            next.address = address_t'(0);
            next.byteenable = 4'b0;
            if (avl_read_i) begin
                next.address    = avl_address_i;
                next.byteenable = avl_byteenable_i;
                next.if_state   = READ_SETADDR;
            end
            if (avl_write_i) begin
                next.if_state   = WRITE_START_I2C;
                next.address    = avl_address_i;
                next.byteenable = avl_byteenable_i;
                next.avl_data   = avl_writedata_i;
            end
        end
        READ_SETADDR: begin
            avl_waitrequest_o = 1'b1;
            i2c_ctrl_wrd_o    = {pres.address[14:8],1'b0};
            i2c_cmd_strobe_o  = 1'b1;
            address_val       = pres.address[7:0];
            load_address      = 1'b1;
            next.if_state     = READ_SETADDR_WAITFOR_I2C;
        end
        READ_SETADDR_WAITFOR_I2C: begin
            avl_waitrequest_o = 1'b1;
            if (!i2c_busy_i) begin
                if (i2c_error_i) begin
                    avl_readdata_o = 32'b0;
                    avl_readdatavalid_o = 1'b1;
                    next.if_state = IDLE;
                end else begin
                    next.if_state = READ_START_READ;
                end
            end
        end
        READ_START_READ: begin
            avl_waitrequest_o = 1'b1;
            i2c_ctrl_wrd_o    = {pres.address[14:8],1'b1};
            i2c_cmd_strobe_o  = 1'b1;
            i2c_len_read_o    = 8'b1;
            next.if_state     = READ_WAITFOR_I2C;
        end
        READ_WAITFOR_I2C: begin
            avl_waitrequest_o = 1'b1; 
            if (!i2c_busy_i) begin
/*
                if (i2c_done_i) begin
                end
*/
                avl_readdata_o = {24'b0, i2c_data_buffer}; 
                avl_readdatavalid_o = 1'b1;
                next.if_state = IDLE;
            end
        end
        WRITE_START_I2C: begin
            avl_waitrequest_o = 1'b1;
            i2c_ctrl_wrd_o    = {pres.address[14:8],1'b0};
            i2c_cmd_strobe_o  = 1'b1;
            address_val       = pres.address[7:0];
            load_address      = 1'b1;
            data_val          = pres.avl_data[7:0];
            load_data         = 1'b1;
            next.if_state     = WRITE_WAITFOR_I2C;
        end
        WRITE_WAITFOR_I2C: begin
            avl_waitrequest_o = 1'b1;
            if (!i2c_busy_i) begin
                next.if_state = IDLE;
            end
        end
    endcase;
end

always_ff @(posedge clock_i or negedge reset_i)
begin
    if (!reset_i) begin
        pres <= RESET;
    end else begin
        pres <= next;
    end
end

always_ff @(posedge clock_i or negedge reset_i)
begin
    if (!reset_i) begin
        address_buf <= 8'b0;
        address_available <= 1'b0;
        data_buf <= 8'b0;
        data_available <= 1'b0;
        read_request_delayed <= 1'b0;
    end else begin
        read_request_delayed <= i2c_read_data_i;
        if (load_address) begin
            address_buf <= address_val;
            address_available <= 1'b1;
        end
        if (load_data) begin
            data_buf <= data_val;
            data_available <= 1'b1;
        end
        if (read_request_delayed) begin
            if (address_available) begin
                address_available <= 1'b0;
            end else begin
                if (data_available) begin
                    data_available <= 1'b0;
                end
            end
        end
    end
end

always_comb
begin
    if (address_available) begin
        i2c_data_o = address_buf;
    end else begin
        if (data_available) begin
            i2c_data_o = data_buf;
        end else begin
            i2c_data_o = 8'b0;
        end
    end
end

assign i2c_data_available_o = address_available || data_available;

always_ff @(posedge clock_i or negedge reset_i)
begin
    if (!reset_i) begin
        i2c_data_buffer <= '0;
    end else begin
        if (i2c_data_valid_i)
            i2c_data_buffer <= i2c_data_i;
    end
end

endmodule
