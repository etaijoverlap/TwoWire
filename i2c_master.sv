module i2c_master #(
    START_DURATION=8'h1E,
    DATA_SETUP_DELAY=8'h0,
    DATA_HOLD_DELAY=8'h1E,
    SCLK_HIGH_DURATION=8'h1E,
    ACK_DURATION=8'h1E,
    STOP_DURATION=8'h1E
)(
    /* clock & reset */
    input  logic clock_i,
    input  logic reset_n_i,

    /* Initiation strobe */
    input  logic cmd_strobe_i,

    /* Read interface */
    input  logic [32-1:0] len_rd_i, // Expected number of reads
    output logic          data_valid_o,
    output logic [8-1:0]  data_o,

    /* Write interface */
    input  logic [8-1:0] ctrl_wr_i,
    input  logic         data_available_i,
    output logic         data_read_o,
    input  logic [8-1:0] data_i,

    /* i2c driver interface */
    output logic i2c_sclk_o,
    output logic i2c_sdat_o,

    input  logic i2c_sclk_i,
    input  logic i2c_sdat_i,
    
    /* Status interface */
    output logic busy_o,
    output logic error_o,
    input  logic ack_error_i
);

typedef logic[8-1:0] delay_t;
typedef enum {
    I2C_IDLE,
    I2C_START,
    I2C_WR_BIT_SET_SCLK, I2C_WR_BIT_SET_SDAT, I2C_WR_BIT_HOLD_SDAT, 
    I2C_ACK_SET_SCLK, I2C_ACK_SET_SDAT, I2C_ACK_HOLD,
    I2C_RD_BIT_SET, I2C_RD_BIT_HOLD,
    I2C_SLAVE_ACK_SET_SCLK, I2C_SLAVE_ACK_SET_SDAT, I2C_SLAVE_ACK_HOLD,
    I2C_STOP_SET_SCLK, I2C_STOP_SET_SDAT, I2C_STOP_DELAY, I2C_STOP,
    I2C_ERROR
    } phys_state_t;

struct {
    phys_state_t phys_state;
    logic [8-1:0] len_read;
    logic [8-1:0] rw_buffer;
    logic         read_fifo;
    logic         write_fifo;
    logic         sclk;
    logic         sdat;
    int           data_idx;
    logic         error;
} pres, next;

logic [8-1:0] timer;
logic         timer_start; 
logic timer_expired;
logic [1:0]   write_fifo_delay;
logic [1:0]   sclk_synchronizer;
logic         sclk_sync;
logic [1:0]   sdat_synchronizer;
logic         sdat_sync;

multitimer #(
    .TIMER_WIDTH(8),
    .INTERRUPTS(1)
    ) mtimer (
    .reset_n_i     ( reset_n_i        ),
    .clock_i       ( clock_i          ),
    .timer_set_i   ( '{timer}    ),
    .timer_start_i ( timer_start ),
    .expired_o     ( '{timer_expired} )
    );

always_ff @(posedge clock_i, negedge reset_n_i)
begin
    if (!reset_n_i) begin
        pres.phys_state  <= I2C_IDLE;
        pres.data_idx    <= 0;
        pres.len_read    <= '0;
        pres.sclk        <= 1'b1;
        pres.sdat        <= 1'b1;
        pres.read_fifo   <= 1'b0;
        pres.write_fifo  <= 1'b0;
        pres.error       <= 1'b0;
        pres.rw_buffer   <= 8'b0;
    end else begin
        pres <= next;
    end
end

always_comb
begin
    next.phys_state = pres.phys_state;
    if (pres.phys_state == I2C_IDLE || pres.phys_state == I2C_ERROR)
        busy_o = 1'b0;
    else
        busy_o = 1'b1;

    timer_start = 1'b0;
    timer       = '0;
    error_o          = 1'b0;
    next.read_fifo   = 1'b0;
    next.write_fifo  = 1'b0;
    next.data_idx    = pres.data_idx;
    next.rw_buffer   = pres.rw_buffer;
    next.error       = pres.error;
    next.len_read    = pres.len_read;

    data_valid_o = 1'b0;

    unique case (pres.phys_state)
        I2C_IDLE: begin
            next.sclk  = 1'b1;
            next.sdat  = 1'b1;
            next.error = 1'b0;
            if (cmd_strobe_i) begin
                next.rw_buffer   = ctrl_wr_i;
                next.phys_state  = I2C_START;
                next.len_read    = len_rd_i;
                timer       = delay_t'(START_DURATION);
                timer_start = 1'b1;
                busy_o = 1'b1;
            end
        end
        I2C_START: begin
            next.sclk = 1'b1;
            next.sdat = 1'b0;

            if (timer_expired) begin
                next.phys_state = I2C_WR_BIT_SET_SCLK;
                next.data_idx   = 7;
                timer_start     = 1'b1;
                timer           = delay_t'(DATA_SETUP_DELAY);
            end
        end
        I2C_WR_BIT_SET_SCLK: begin
            next.sclk = 1'b0;
            next.sdat = pres.sdat;
            if (timer_expired) begin
                next.phys_state  = I2C_WR_BIT_SET_SDAT;
                timer       = delay_t'(DATA_HOLD_DELAY);
                timer_start = 1'b1;
            end
        end
        I2C_WR_BIT_SET_SDAT: begin
            next.sclk = timer_expired;
            next.sdat = pres.rw_buffer[pres.data_idx];
            if (sclk_sync) begin // We need to listen to the bus instead of the internal
                                 // signal to make slave-induced delays possible.
                next.phys_state = I2C_WR_BIT_HOLD_SDAT;
                timer_start     = 1'b1;
                timer           = delay_t'(SCLK_HIGH_DURATION);
            end
        end
        I2C_WR_BIT_HOLD_SDAT: begin
            next.sclk = 1'b1;
            next.sdat = pres.sdat;
            if (timer_expired) begin
                if (pres.data_idx > 0) begin
                    next.data_idx   = pres.data_idx - 1;
                    timer           = delay_t'(DATA_SETUP_DELAY);
                    timer_start     = 1'b1;
                    next.phys_state = I2C_WR_BIT_SET_SCLK;
                end else begin
                    next.phys_state = I2C_ACK_SET_SCLK;
                    timer           = delay_t'(DATA_SETUP_DELAY);
                    timer_start     = 1'b1;
                end
            end
        end
        I2C_ACK_SET_SCLK: begin
            next.sclk = 1'b0;
            next.sdat = pres.sdat;
            if (timer_expired) begin
                next.phys_state = I2C_ACK_SET_SDAT;
                timer           = delay_t'(DATA_HOLD_DELAY);
                timer_start     = 1'b1;
            end
        end
        I2C_ACK_SET_SDAT: begin
            next.sclk = timer_expired;
            next.sdat = 1'b1;
            if (sclk_sync) begin
                next.phys_state = I2C_ACK_HOLD;
                timer           = delay_t'(SCLK_HIGH_DURATION);
                timer_start     = 1'b1;
            end
        end
        I2C_ACK_HOLD: begin
            next.sclk = 1'b1;
            next.sdat = 1'b1;
            if (timer_expired) begin
                if (sdat_sync) begin // ACK unsuccessful. -> error state.
                    next.phys_state = I2C_STOP_SET_SCLK;
                    next.error = 1'b1;
                    timer = delay_t'(DATA_SETUP_DELAY);
                    timer_start = 1'b1;
                end else begin
                    if (data_available_i) begin 
                        if (!pres.read_fifo) begin
                            next.read_fifo  = 1'b1;
                        end else begin
                            next.rw_buffer  = data_i;
                            next.phys_state = I2C_WR_BIT_SET_SCLK;
                            next.data_idx   = 7;
                            timer_start     = 1'b1;
                            timer           = delay_t'(DATA_SETUP_DELAY);
                        end
                    end else begin
                        // No data left in fifo ... do reads now.
                        if (pres.len_read > 8'h0) begin
                            next.phys_state = I2C_RD_BIT_SET;
                            timer           = delay_t'(DATA_SETUP_DELAY + DATA_HOLD_DELAY);
                            timer_start     = 1'b1;
                            next.data_idx   = 7;
                            next.len_read   = pres.len_read - 8'h1;
                        end else begin
                            // No reads to be done ... stop
                            next.phys_state = I2C_STOP_SET_SCLK;
                            timer           = delay_t'(DATA_SETUP_DELAY);
                            timer_start     = 1'b1;
                        end
                    end
                end
            end
        end
        I2C_RD_BIT_SET: begin
            next.sclk = timer_expired;
            next.sdat = 1'b1;
            if (sclk_sync) begin
                next.phys_state = I2C_RD_BIT_HOLD;
                timer           = delay_t'(SCLK_HIGH_DURATION);
                timer_start     = 1'b1;
            end
        end
        I2C_RD_BIT_HOLD: begin
            next.sclk = 1'b1;
            next.sdat = 1'b1; 
            if (timer_expired) begin
                next.rw_buffer[pres.data_idx] = sdat_sync;
                if (pres.data_idx == 0) begin
                    next.write_fifo = 1'b1;
                    next.phys_state = I2C_SLAVE_ACK_SET_SCLK;
                    timer           = delay_t'(DATA_SETUP_DELAY);
                    timer_start     = 1'b1;
                end else begin
                    next.data_idx = pres.data_idx - 1; 
                    next.phys_state = I2C_RD_BIT_SET;
                    timer = delay_t'(DATA_SETUP_DELAY + DATA_HOLD_DELAY);
                    timer_start = 1'b1;
                end
            end
        end
        I2C_SLAVE_ACK_SET_SCLK: begin
            next.sclk = 1'b0;
            next.sdat = 1'b1;
            if (timer_expired) begin
                next.phys_state = I2C_SLAVE_ACK_SET_SDAT;
                timer           = delay_t'(DATA_HOLD_DELAY);
                timer_start     = 1'b1;
            end
        end
        I2C_SLAVE_ACK_SET_SDAT: begin
            next.sclk = timer_expired;
            next.sdat = 1'b0;
            if (sclk_sync) begin
                next.phys_state = I2C_SLAVE_ACK_HOLD;
                timer           = delay_t'(SCLK_HIGH_DURATION);
                timer_start     = 1'b1;
            end
        end
        I2C_SLAVE_ACK_HOLD: begin
            next.sclk = 1'b1;
            next.sdat = 1'b0;
            if (timer_expired) begin
                data_valid_o = 1'b1;
                if (pres.len_read > 8'h0) begin
                    next.len_read   = pres.len_read - 8'h1;
                    next.phys_state = I2C_RD_BIT_SET;
                    next.data_idx   = 7;
                    timer           = delay_t'(DATA_SETUP_DELAY + DATA_HOLD_DELAY);
                    timer_start     = 1'b1;
                end else begin
                    next.phys_state = I2C_STOP_SET_SCLK;
                    timer           = STOP_DURATION;
                    timer_start     = 1'b1;
                end
            end
        end
        I2C_STOP_SET_SCLK: begin
            // Intended to set up an unplanned stop signal.
            next.sclk = 1'b0;
            next.sdat = pres.sdat;
            if (timer_expired) begin
                next.phys_state = I2C_STOP_SET_SDAT;
                timer = delay_t'(DATA_HOLD_DELAY);
                timer_start = 1'b1;
            end
        end
        I2C_STOP_SET_SDAT: begin
            next.sclk = timer_expired;
            next.sdat = 1'b0;
            if (sclk_sync) begin
                next.phys_state = I2C_STOP_DELAY;
                timer = delay_t'(SCLK_HIGH_DURATION);
                timer_start = 1'b1;
            end
        end
        I2C_STOP_DELAY: begin
            // Entering this state we assume that SDAT is pulled low either by the master
            // (myself) or the slave, and that SCLK is high.
            // 
            // Then go to stop.
            next.sclk = 1'b1;
            next.sdat = 1'b0;
            if (timer_expired) begin
                next.phys_state = I2C_STOP;
                timer = delay_t'(STOP_DURATION);
                timer_start = 1'b1;
            end
        end
        I2C_STOP: begin
            // On the output side, this state is equivalent to idle. It exists just to make
            // sure that we wait some time before the next transaction begins.
            next.sclk = 1'b1;
            next.sdat = 1'b1;
            if (timer_expired) begin
                if (pres.error)
                    next.phys_state = I2C_ERROR;
                else
                    next.phys_state = I2C_IDLE;
            end
        end
        I2C_ERROR: begin
            next.sclk = 1'b1;
            next.sdat = 1'b1;
            error_o   = 1'b1;
            if (ack_error_i) begin
                next.phys_state = I2C_IDLE;
            end
        end
    endcase;
end

always_ff @(posedge clock_i or negedge reset_n_i)
begin
    if (!reset_n_i) begin
        write_fifo_delay <= '0;
    end else begin
        write_fifo_delay[0] <= pres.write_fifo;
        write_fifo_delay[1] <= write_fifo_delay[0];
    end
end

// SCLK and SDAT synchronization
always_ff @(posedge clock_i or negedge reset_n_i)
begin
    if (!reset_n_i) begin
        sclk_synchronizer <= '0;
        sdat_synchronizer <= '0;
    end else begin
        sclk_synchronizer[0] <= i2c_sclk_i           & i2c_sclk_o;
        sclk_synchronizer[1] <= sclk_synchronizer[0] & i2c_sclk_o;
        
        sdat_synchronizer[0] <= i2c_sdat_i           & i2c_sdat_o;
        sdat_synchronizer[1] <= sdat_synchronizer[0] & i2c_sdat_o;
        if (timer_start) sclk_synchronizer <= '0;
    end
end

assign i2c_sclk_o  = pres.sclk;
assign i2c_sdat_o  = pres.sdat;
assign data_read_o = pres.read_fifo;
assign data_o      = pres.rw_buffer;

assign sclk_sync = sclk_synchronizer[1];
assign sdat_sync = sdat_synchronizer[1];

endmodule
