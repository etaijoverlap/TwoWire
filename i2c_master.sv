// i2c_master, a flexible and (hopefully) easy to use I2C busmaster core.
// (c) 2015 Franz Schanovsky <franz.schanovsky@gmail.com> 
// 
// This software is licensed under the EUPL V 1.1
//
// This software is provided "as is" without warranty of any kind, see the 
// respective section in the EUPL. USE AT YOUR OWN RISK.

module i2c_master #(
    START_DURATION     = 8'h25,
    DATA_SETUP_DELAY   = 8'h05, // Minimum value needed for synchronization
    DATA_HOLD_DELAY    = 8'h42,
    SCLK_HIGH_DURATION = 8'h20,
    ACK_DURATION       = 8'h20,
    STOP_DURATION      = 8'h70,
    TIMEOUT            = 8'hBF
)(
    /* clock & reset */
    input  logic clock_i,
    input  logic reset_n_i,

    /* Initiation strobe */
    input  logic cmd_strobe_i,
    input  logic clean_strobe_i, // Issues an attempt to unblock the bus from potentially stuck slaves

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
    output logic          busy_o,
    output logic          error_o,
    output logic          collision_detected_o,
    output logic          sclk_timeout_o,
    output logic          sclk_sync_o,
    output logic          sdat_sync_o,
    input  logic          ack_error_i,
    output logic [32-1:0] acks_received_o
);

typedef logic[8-1:0] delay_t;
typedef enum {
    I2C_IDLE,
    I2C_START,
    I2C_WR_BIT_SET_SCLK, I2C_WR_BIT_SET_SDAT, I2C_WR_BIT_HOLD_SDAT, 
    I2C_ACK_SET_SCLK, I2C_ACK_SET_SDAT, I2C_ACK_HOLD,
    I2C_RD_BIT_SET, I2C_RD_BIT_HOLD,
    I2C_SLAVE_ACK_SET_SCLK, I2C_SLAVE_ACK_SET_SDAT, I2C_SLAVE_ACK_HOLD,
    I2C_PREPARE_STOP_SCLK_LOW, I2C_PREPARE_STOP_SCLK_HIGH,
    I2C_STOP_SET_SCLK, I2C_STOP_SET_SDAT, I2C_STOP_DELAY, I2C_STOP,
    I2C_ERROR
    } phys_state_t;

typedef struct {
    phys_state_t   phys_state;
    logic [8-1:0]  len_read;
    logic [8-1:0]  rw_buffer;
    logic          read_fifo;
    logic          write_fifo;
    int            data_idx;
    phys_state_t   state_after_stop;
    logic          sclk_timeout;
    logic          collision_detected;
    logic [32-1:0] ack_count;
} register_t;

register_t pres, next;

register_t RESET = '{
    I2C_IDLE,
    8'b0,
    8'b0,
    1'b0,
    1'b0,
    0,
    I2C_IDLE,
    1'b0,
    1'b0,
    32'b0
    };



delay_t       timer;
logic         timer_start; 
logic         timer_expired;
logic         timeout_occured;
logic [1:0]   write_fifo_delay;
logic [1:0]   sclk_synchronizer;
logic         sclk_sync;
logic [1:0]   sdat_synchronizer;
logic         sdat_sync;
logic         last_sdat_output;

multitimer #(
    .TIMER_WIDTH(8),
    .INTERRUPTS(2)
    ) mtimer (
    .reset_n_i     ( reset_n_i                         ),
    .clock_i       ( clock_i                           ),
    .timer_set_i   ( '{timer, delay_t'(TIMEOUT)}       ),
    .timer_start_i ( timer_start                       ),
    .expired_o     ( '{timer_expired, timeout_occured} )
    );

/*
 * Returns the timer delay for a given state. Returns 0 for unknown states.
 */
function delay_t delay_for_state(phys_state_t state);
    delay_for_state = (state == I2C_START                  ) ? delay_t'(START_DURATION                    )
                    : (state == I2C_WR_BIT_SET_SCLK        ) ? delay_t'(DATA_SETUP_DELAY                  )
                    : (state == I2C_WR_BIT_SET_SDAT        ) ? delay_t'(DATA_HOLD_DELAY                   )
                    : (state == I2C_WR_BIT_HOLD_SDAT       ) ? delay_t'(SCLK_HIGH_DURATION                )
                    : (state == I2C_ACK_SET_SCLK           ) ? delay_t'(DATA_SETUP_DELAY                  )
                    : (state == I2C_ACK_SET_SDAT           ) ? delay_t'(DATA_HOLD_DELAY                   )
                    : (state == I2C_ACK_HOLD               ) ? delay_t'(SCLK_HIGH_DURATION                )
                    : (state == I2C_RD_BIT_SET             ) ? delay_t'(DATA_SETUP_DELAY + DATA_HOLD_DELAY)
                    : (state == I2C_RD_BIT_HOLD            ) ? delay_t'(SCLK_HIGH_DURATION                )
                    : (state == I2C_SLAVE_ACK_SET_SCLK     ) ? delay_t'(DATA_SETUP_DELAY                  )
                    : (state == I2C_SLAVE_ACK_SET_SDAT     ) ? delay_t'(DATA_HOLD_DELAY                   )
                    : (state == I2C_SLAVE_ACK_HOLD         ) ? delay_t'(SCLK_HIGH_DURATION                )
                    : (state == I2C_PREPARE_STOP_SCLK_LOW  ) ? delay_t'(DATA_SETUP_DELAY + DATA_HOLD_DELAY)
                    : (state == I2C_PREPARE_STOP_SCLK_HIGH ) ? delay_t'(SCLK_HIGH_DURATION                )
                    : (state == I2C_STOP_SET_SCLK          ) ? delay_t'(DATA_SETUP_DELAY                  )
                    : (state == I2C_STOP_SET_SDAT          ) ? delay_t'(DATA_HOLD_DELAY                   )
                    : (state == I2C_STOP_DELAY             ) ? delay_t'(SCLK_HIGH_DURATION                )
                    : (state == I2C_STOP                   ) ? delay_t'(STOP_DURATION                     )
                    : delay_t'(0);
endfunction

task setup_state(input phys_state_t next_state);
    timer           = delay_for_state(next_state);
    timer_start     = 1'b1;
    next.phys_state = next_state;
endtask

task raise_error();
    next.state_after_stop = I2C_ERROR;
    setup_state(I2C_STOP_SET_SCLK);
endtask

task check_timeout();
    if (timeout_occured) begin
        next.sclk_timeout = 1'b1;
        raise_error(); 
    end
endtask

task check_sclk();
    if (i2c_sclk_o != sclk_sync) begin
        next.collision_detected = 1'b1;
        raise_error();
    end
endtask

task check_sdat();
    if (i2c_sdat_o != sdat_sync) begin
        next.collision_detected = 1'b1;
        raise_error();
    end
endtask

task check_lines();
    check_sclk();
    check_sdat();
endtask

/*
 * State machine combinatorial block.
 */
always_comb
begin
    next = pres;
    if (pres.phys_state == I2C_IDLE || pres.phys_state == I2C_ERROR)
        busy_o = 1'b0;
    else
        busy_o = 1'b1;

    timer_start           = 1'b0;
    timer                 = delay_t'(0);
    error_o               = 1'b0;
    next.read_fifo        = 1'b0;
    next.write_fifo       = 1'b0;

    data_valid_o = 1'b0;

    unique case (pres.phys_state)
        I2C_IDLE: begin
            i2c_sclk_o = 1'b1;
            i2c_sdat_o = 1'b1;
            /* Set all internal states to default */
            next.state_after_stop = I2C_IDLE;
            next.len_read         = 8'b0;
            next.rw_buffer        = 8'b0;
            next.read_fifo        = 1'b0;
            next.write_fifo       = 1'b0;
            next.data_idx         = 0;
            next.sclk_timeout     = 1'b0;
            next.collision_detected = 1'b0;
            if (cmd_strobe_i) begin
                next.rw_buffer = ctrl_wr_i;
                next.len_read  = len_rd_i;
                next.ack_count = 32'b0;
                busy_o         = 1'b1;
                setup_state(I2C_START);
            end
            if (clean_strobe_i) begin
                setup_state(I2C_PREPARE_STOP_SCLK_LOW);
            end
        end
        I2C_START: begin
            next.state_after_stop = I2C_IDLE;
            i2c_sclk_o = 1'b1;
            i2c_sdat_o = 1'b0;
            if (timer_expired) begin
                next.data_idx   = 7;
                setup_state(I2C_WR_BIT_SET_SCLK);
                check_sclk();
            end
        end
        I2C_WR_BIT_SET_SCLK: begin
            i2c_sclk_o = 1'b0;
            i2c_sdat_o = last_sdat_output;
            if (timer_expired) begin
                setup_state(I2C_WR_BIT_SET_SDAT);
                // No checks here, as we might come back from an ACK phase
                // where the SDAT line was pulled down by the slave.
            end
        end
        I2C_WR_BIT_SET_SDAT: begin
            i2c_sclk_o = timer_expired;
            i2c_sdat_o = pres.rw_buffer[pres.data_idx];
            if (sclk_sync) begin 
                setup_state(I2C_WR_BIT_HOLD_SDAT);
                check_timeout();
            end
        end
        I2C_WR_BIT_HOLD_SDAT: begin
            i2c_sclk_o = 1'b1;
            i2c_sdat_o = last_sdat_output;
            if (timer_expired) begin
                if (pres.data_idx > 0) begin
                    next.data_idx   = pres.data_idx - 1;
                    setup_state(I2C_WR_BIT_SET_SCLK);
                end else begin
                    setup_state(I2C_ACK_SET_SCLK);
                end
                check_lines();
            end
        end
        I2C_ACK_SET_SCLK: begin
            i2c_sclk_o = 1'b0;
            i2c_sdat_o = last_sdat_output;
            if (timer_expired) begin
                setup_state(I2C_ACK_SET_SDAT);
                check_sdat();
            end
        end
        I2C_ACK_SET_SDAT: begin
            i2c_sclk_o = timer_expired;
            i2c_sdat_o = 1'b1;
            if (sclk_sync) begin
                setup_state(I2C_ACK_HOLD);
                check_timeout();
            end
        end
        I2C_ACK_HOLD: begin
            i2c_sclk_o = 1'b1;
            i2c_sdat_o = 1'b1;
            if (timer_expired) begin
                if (sdat_sync) begin // ACK unsuccessful. -> error state.
                    raise_error();
                end else begin
                    next.ack_count = pres.ack_count + 32'h1;
                    if (data_available_i) begin 
                        if (!pres.read_fifo) begin
                            next.read_fifo  = 1'b1;
                        end else begin
                            next.rw_buffer  = data_i;
                            next.data_idx   = 7;
                            setup_state(I2C_WR_BIT_SET_SCLK);
                        end
                    end else begin
                        // No data left in fifo ... do reads now.
                        if (pres.len_read > 8'h0) begin
                            next.data_idx   = 7;
                            next.len_read   = pres.len_read - 8'h1;
                            setup_state(I2C_RD_BIT_SET);
                        end else begin
                            // No reads to be done ... stop
                            setup_state(I2C_STOP_SET_SCLK);
                        end
                    end
                end
                check_sclk();
            end
        end
        I2C_RD_BIT_SET: begin
            i2c_sclk_o = timer_expired;
            i2c_sdat_o = 1'b1;
            if (sclk_sync) setup_state(I2C_RD_BIT_HOLD);
            check_timeout();
        end
        I2C_RD_BIT_HOLD: begin
            i2c_sclk_o = 1'b1;
            i2c_sdat_o = 1'b1; 
            if (timer_expired) begin
                next.rw_buffer[pres.data_idx] = sdat_sync;
                if (pres.data_idx == 0) begin
                    next.write_fifo = 1'b1;
                    setup_state(I2C_SLAVE_ACK_SET_SCLK);
                end else begin
                    next.data_idx = pres.data_idx - 1; 
                    setup_state(I2C_RD_BIT_SET);
                end
                check_sclk();
            end
        end
        I2C_SLAVE_ACK_SET_SCLK: begin
            i2c_sclk_o = 1'b0;
            i2c_sdat_o = 1'b1;
            if (timer_expired) setup_state(I2C_SLAVE_ACK_SET_SDAT);
            // Add checks
        end
        I2C_SLAVE_ACK_SET_SDAT: begin
            i2c_sclk_o = timer_expired;
            i2c_sdat_o = 1'b0;
            if (sclk_sync) setup_state(I2C_SLAVE_ACK_HOLD);
            check_timeout();
            // Add checks
        end
        I2C_SLAVE_ACK_HOLD: begin
            i2c_sclk_o = 1'b1;
            i2c_sdat_o = 1'b0;
            if (timer_expired) begin
                data_valid_o = 1'b1;
                if (pres.len_read > 8'h0) begin
                    next.len_read   = pres.len_read - 8'h1;
                    next.data_idx   = 7;
                    setup_state(I2C_RD_BIT_SET);
                end else begin
                    //setup_state(I2C_STOP_SET_SCLK); This used to be the 'logical' choice for the next state... However, it didn't work as expected.
                    setup_state(I2C_STOP_DELAY);
                end
            end
            // Add checks
        end
        I2C_PREPARE_STOP_SCLK_LOW: begin
            // The prepare stop states serve to clean the bus from
            // slave-induced stalls. If the i2c master and slave get out of
            // sync, it may happen that the i2c slave gets stuck in a state where it
            // pulls down SDAT forever. As many i2c slaves seem to lack a reset method,
            // we try to pump the SCLK signal until the SDAT is released by the slave.
            // Then we set up the stop condition, so that the bus is clean again.
            i2c_sclk_o = 1'b0;
            i2c_sdat_o = 1'b1;
            if (timer_expired) setup_state(I2C_PREPARE_STOP_SCLK_HIGH);
        end
        I2C_PREPARE_STOP_SCLK_HIGH:begin
            i2c_sclk_o = 1'b1;
            i2c_sdat_o = 1'b1;
            if (timer_expired) begin
                if (sdat_sync) 
                    setup_state(I2C_STOP_SET_SCLK);
                else
                    setup_state(I2C_PREPARE_STOP_SCLK_LOW);
            end
        end
        I2C_STOP_SET_SCLK: begin
            // Intended to set up an unplanned stop signal.
            i2c_sclk_o = 1'b0;
            i2c_sdat_o = last_sdat_output;
            if (timer_expired) setup_state(I2C_STOP_SET_SDAT);
        end
        I2C_STOP_SET_SDAT: begin
            i2c_sclk_o = timer_expired;
            i2c_sdat_o = 1'b0;
            if (sclk_sync) setup_state(I2C_STOP_DELAY);
        end
        I2C_STOP_DELAY: begin
            // Entering this state we assume that SDAT is pulled low either by the master
            // (myself) or the slave, and that SCLK is high.
            // 
            // Then go to stop.
            i2c_sclk_o = 1'b1;
            i2c_sdat_o = 1'b0;
            if (timer_expired) setup_state(I2C_STOP);
        end
        I2C_STOP: begin
            // On the output side, this state is equivalent to idle. It exists just to make
            // sure that we wait some time before the next transaction begins.
            i2c_sclk_o = 1'b1;
            i2c_sdat_o = 1'b1;
            if (timer_expired) begin
                next.phys_state = pres.state_after_stop;
            end
        end
        I2C_ERROR: begin
            i2c_sclk_o = 1'b1;
            i2c_sdat_o = 1'b1;
            error_o   = 1'b1;
            if (ack_error_i) begin
                next.phys_state = I2C_IDLE;
            end
        end
    endcase;
end

/*
 * State machine register part.
 */
always_ff @(posedge clock_i, negedge reset_n_i)
begin
    if (!reset_n_i) begin
        pres <= RESET;
    end else begin
        pres <= next;
    end
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

always_ff @(posedge clock_i or negedge reset_n_i)
begin
    if (!reset_n_i)
        last_sdat_output <= 1'b0;
    else
        last_sdat_output <= i2c_sdat_o;
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
        if (timer_start) sclk_synchronizer <= '0; // TODO Introduce resynchronization delay for timer and SDAT
    end
end

assign data_read_o = pres.read_fifo;
assign data_o      = pres.rw_buffer;
assign collision_detected_o = pres.collision_detected;
assign sclk_timeout_o = pres.sclk_timeout;
assign sclk_sync_o = sclk_sync;
assign sdat_sync_o = sdat_sync;
assign acks_received_o = pres.ack_count;

assign sclk_sync = sclk_synchronizer[1];
assign sdat_sync = sdat_synchronizer[1];

endmodule
