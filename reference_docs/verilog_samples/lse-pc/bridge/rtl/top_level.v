module top_level
(
    input clk,
    output led_tst,

    input [23:1] a,
    input _bhe,
    input _ble,
    input _ads,

    inout [15:0] d,

    output _ready,
    output hold,
    input hlda,

    output intr,
    output nmi,

    output reset,
    input _lock,

    input wr,
    input dc,
    input mio,

    input uart_tx,
    output uart_rx,

    output ramwe,
    output ramcs
);

wire [15:0] dout;

// BC signals
wire [15:0] bc_dout;
wire [15:0] mem_dout;
wire [15:0] io_dout;
wire bc_ready;
wire bc_wr;
wire mc_ramcs;
wire mc_ramwe;


assign d[15:8] = !oe || wr || _bhe || !ramcs ? 8'hzz : dout[15:8]; 
assign d[7:0] = !oe || wr || _ble || !ramcs ? 8'hzz : dout[7:0]; 

// Signals Mux
assign dout = mem_dout;
assign _ready = bc_ready;
assign hold = 0;
assign intr = 0;
assign nmi = 0;
assign reset = 0;

assign bc_wr = wr;

assign ramcs = mc_ramcs;
assign ramwe = mc_ramwe;

bus_control bc
(
    .clk                (clk),
    ._ads               (_ads),
    ._ready             (bc_ready),
    .oe                 (oe),
    .bce                (bce)
);

memory_control mem
(
    .clk                (clk),
    .cs                 (bce && mio),
    .addr               (a),
    .data_in            (d),
    .data_out           (mem_dout),
    .wr                 (wr),
    .low                (!_ble),
    .high               (!_bhe),
    .ramcs              (mc_ramcs),
    .ramwe              (mc_ramwe)
);


pll pll
(
    .inclk0             (clk),
    .c1                 (sv_clk)
);

endmodule
