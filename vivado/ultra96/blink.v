`timescale 1ns / 1ps
`default_nettype none 

module blink
#(
    parameter MAX_COUNT = 100000000
) (
    input wire clock,
    input wire resetn,
    output wire blink_out
);

localparam W = $clog2(MAX_COUNT);

reg [W-1:0] counter = 0;

assign blink_out = counter < (MAX_COUNT/2) ? 1'b1 : 1'b0;

always @(posedge clock) begin
    if( !resetn ) begin
        counter <= 0; 
    end
    else begin
        counter <= counter < MAX_COUNT - 1 ? counter + 1 : 0;
    end
end 

endmodule

`default_nettype wire