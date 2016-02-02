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

    output logic        i2c_cmd_strobe_o,
    output logic [7:0]  i2c_ctrl_wrd_o,
    output logic        i2c_data_available_o,
    input  logic        i2c_read_data_i,
    output logic [7:0]  i2c_data_o,
    output logic [7:0]  i2c_len_read_o,
    input  logic [7:0]  i2c_data_i,
    input  logic        i2c_data_valid_i,
    input  logic        i2c_busy_i,
    input  logic        i2c_error_i,
    input  logic        i2c_collision_detected_i,
    input  logic        i2c_sclk_timeout_i,
    output logic        i2c_ack_error_o,
    input  logic [31:0] i2c_acks_received_i
);

typedef enum {
    IDLE,
    READ_SETADDR, READ_SETADDR_WAITFOR_I2C, READ_START_READ, READ_WAITFOR_I2C,
    WRITE_START_I2C, WRITE_WAITFOR_I2C, ACK_I2C_ERROR
} if_state_t;
typedef logic[ADDRESS_WIDTH-1:0] address_t;

typedef struct {
    if_state_t   if_state;
    logic [31:0] avl_data;
    address_t    word_address;
    logic [3:0]  byteenable;
    if_state_t   state_reg;
} register_t;

const register_t RESET = '{IDLE, 32'b0, address_t'(0), 4'b0, IDLE};

register_t pres, next;

logic        load_address, load_data;
logic [7:0]  buf_address;
logic        buf_address_valid;
logic [31:0] buf_data;
logic [3:0]  buf_data_valid;
logic        clear_buf;
logic        address_available;
logic        data_available;
logic        read_request_delayed;
logic [31:0] i2c_data_buffer;

avli2c_address_data_buffer(
    .clock_i              (clock_i              ),
    .reset_i              (reset_i              ),
    .address_i            (buf_address          ),
    .address_valid_i      (buf_address_valid    ),
    .data_i               (buf_data             ),
    .data_valid_i         (buf_data_valid       ),
    .clear_i              (clear_buf            ),
    .i2c_data_available_o (i2c_data_available_o ),
    .i2c_data_o           (i2c_data_o           ),
    .i2c_read_data_i      (i2c_read_data_i      )
    );


always_comb
begin
    int i,j;
    logic [1:0] adr_offset;
    logic [2:0] adr_len;
    logic [6:0] adr_page;
    logic [8:0] adr_base;
    logic [ADDRESS_WIDTH-1:0] byte_address;

    next = pres;

    avl_waitrequest_o   = 1'b0;
    avl_readdata_o      = 32'b0;
    avl_readdatavalid_o = 1'b0;

    i2c_cmd_strobe_o     = 1'b0;
    i2c_ctrl_wrd_o       = 8'b0;
    i2c_len_read_o       = 8'b0;
    i2c_ack_error_o      = 1'b0;

    i2c_cmd_strobe_o = 1'b0;
    
    avl_readdata_o = pres.avl_data;

    // Decode the byteenable signal into offset and length
    adr_offset = 2'b00;
    adr_len    = 3'b00;
    if (pres.byteenable != 4'h0) begin
        for (i=0; i < 4; i++) begin
            if (pres.byteenable[i] == 1'b1) begin
                adr_offset = i;
                break;
            end
        end
        for (i=0; i < 4; i++) begin
            if (pres.byteenable[i] == 1'b1)
                adr_len++;
        end
    end
    byte_address = pres.word_address + adr_offset;
    adr_page = byte_address[14:8];
    adr_base = byte_address[7:0];

    buf_address       = 8'b0;
    buf_address_valid = 1'b0;
    buf_data          = 32'b0;
    buf_data_valid    = 4'b0;
    clear_buf         = 1'b0;
    // Avalon bus interface state machine
    unique case(pres.if_state)
        IDLE: begin
            next.avl_data     = 32'b0;
            next.word_address = address_t'(0);
            if (avl_read_i || avl_write_i) begin
                next.word_address = avl_address_i;
                next.byteenable   = avl_byteenable_i;
            end
            if (avl_read_i) begin
                if (i2c_error_i) begin
                    next.if_state  = ACK_I2C_ERROR;
                    next.state_reg = READ_SETADDR;
                end else begin
                    next.if_state = READ_SETADDR;
                end
            end
            if (avl_write_i) begin
                next.byteenable = avl_byteenable_i;
                next.avl_data   = avl_writedata_i;
                if (i2c_error_i) begin
                    next.if_state  = ACK_I2C_ERROR;
                    next.state_reg = WRITE_START_I2C;
                end else begin
                    next.if_state   = WRITE_START_I2C;
                end
            end
        end
        READ_SETADDR: begin
            avl_waitrequest_o = 1'b1;
            i2c_ctrl_wrd_o    = {adr_page, 1'b0};
            i2c_cmd_strobe_o  = 1'b1;
            i2c_len_read_o    = 8'b0;
            buf_address       = adr_base;
            buf_address_valid = 1'b1;
            next.if_state     = READ_SETADDR_WAITFOR_I2C;
        end
        READ_SETADDR_WAITFOR_I2C: begin
            avl_waitrequest_o = 1'b1;
            if (!i2c_busy_i) begin
                if (i2c_error_i) begin
                    avl_readdata_o      = 32'b0;
                    avl_readdatavalid_o = 1'b1;
                    next.if_state       = IDLE;
                end else begin
                    next.if_state = READ_START_READ;
                end
            end
        end
        READ_START_READ: begin
            avl_waitrequest_o = 1'b1;
            i2c_ctrl_wrd_o    = {adr_page,1'b1};
            i2c_cmd_strobe_o  = 1'b1;
            i2c_len_read_o    = adr_len;
            next.if_state     = READ_WAITFOR_I2C;
        end
        READ_WAITFOR_I2C: begin
            avl_waitrequest_o = 1'b1; 
            if (i2c_data_valid_i) begin
                for (i=0; i < 4; i++) begin
                    if (pres.byteenable[i]) begin
                        next.byteenable[i] = 1'b0;
                        for (j=0; j < 8; j++)
                            next.avl_data[j+i*8] = i2c_data_i[j];
                        break;
                    end
                end
            end
            if (!i2c_busy_i) begin
                avl_readdata_o      = pres.avl_data; 
                avl_readdatavalid_o = 1'b1;
                next.if_state       = IDLE;
            end
        end
        WRITE_START_I2C: begin
            avl_waitrequest_o = 1'b1;
            i2c_ctrl_wrd_o    = {adr_page, 1'b0};
            i2c_cmd_strobe_o  = 1'b1;
            buf_address       = adr_base;
            buf_address_valid = 1'b1;
            buf_data          = pres.avl_data;
            buf_data_valid    = pres.byteenable;
            next.if_state     = WRITE_WAITFOR_I2C;
        end
        WRITE_WAITFOR_I2C: begin
            avl_waitrequest_o = 1'b1;
            if (!i2c_busy_i) begin
                if (i2c_error_i) begin
                    if (!i2c_collision_detected_i && !i2c_sclk_timeout_i && i2c_acks_received_i == 32'h0) begin
                        // This most likely means that the last write process is still ongoing
                        // go back and try again. (the data-sheet calls this 'ack polling')
                        next.if_state  = ACK_I2C_ERROR;
                        next.state_reg = WRITE_START_I2C;
                    end else begin
                        // Something weird has happened. Let's go back to idle and
                        // hope the software checks the i2c_error flag.
                        next.if_state = IDLE;
                    end
                end else begin
                    // Control word writing finished without error. This means the
                    // internal write process has finished and we can release the
                    // interface, so the next write can be initiated.
                    next.if_state = IDLE;
                end
            end
        end
        ACK_I2C_ERROR: begin
            i2c_ack_error_o = 1'b1;
            next.if_state  = pres.state_reg; 
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

endmodule
