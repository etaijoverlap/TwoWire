// i2c_master, a flexible and (hopefully) easy to use I2C busmaster core.
// (c) 2015 Franz Schanovsky <franz.schanovsky@gmail.com> 
// 
// This software is licensed under the EUPL V 1.1
//
// This software is provided "as is" without warranty of any kind, see the 
// respective section in the EUPL. USE AT YOUR OWN RISK.

module multitimer #(
    TIMER_WIDTH=8,
    INTERRUPTS=1
)(
    input  logic clock_i,
    input  logic reset_n_i,
    input  logic[TIMER_WIDTH-1:0] timer_set_i[INTERRUPTS-1:0],
    input  logic timer_start_i,
    output logic expired_o[INTERRUPTS-1:0]
);

typedef logic[TIMER_WIDTH-1:0] time_val_t;
logic timer_start_n, timer_start_p;
time_val_t timer_value_n, timer_value_p;
time_val_t interrupt_values_p[INTERRUPTS-1:0];
time_val_t interrupt_values_n[INTERRUPTS-1:0];
logic expired_n[INTERRUPTS-1:0];
logic expired_p[INTERRUPTS-1:0];

always_comb
begin
    integer i;
    //defaults
    timer_value_n = timer_value_p;
    timer_start_n = timer_start_i;

    expired_n = expired_p;

    for (i=0; i < INTERRUPTS; i++) begin
        timer_value_n = timer_value_p + time_val_t'(1);
        if (timer_value_n >= interrupt_values_p[i])
            expired_n[i] = 1'b1;
    end

    interrupt_values_n = interrupt_values_p;
    if (timer_start_n && !timer_start_p) begin
        interrupt_values_n = timer_set_i;
        timer_value_n = '0;
        for (i=0; i < INTERRUPTS; i++) expired_n[i] = 1'b0;
    end

end

always_ff @(posedge clock_i, negedge reset_n_i)
begin
    integer i;
    if (!reset_n_i) begin
        timer_value_p <= '0;
        timer_start_p <= '0;
        for (i=0; i < INTERRUPTS; i++) begin
            interrupt_values_p[i] <= '1;
            expired_p[i] <= 1'b0;
        end
    end else begin
        timer_start_p <= timer_start_n;
        timer_value_p <= timer_value_n;
        interrupt_values_p <= interrupt_values_n;
        expired_p <= expired_n;
    end
end

assign expired_o = expired_p;
endmodule
