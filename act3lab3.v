`timescale 1ns / 1ps

module rotaryEncoderPosition_TOP(
    input clk,
    input reset,
    input enable,
    input wire A,B,
    output wire [6:0] ssdCathode,
    output reg [3:0] ssdAnode
    );
    
    wire clockwise, counterclockwise;
    reg signed [4:0] encoderPosition = 5'b00000;
    
 rotaryEncoder countPosition (
    .clk(clk),
    .reset(reset),
    .A(A),
    .B(B),
    .clockwise(clockwise),
    .counterclockwise(counterclockwise)
 );

always @(posedge clk) begin
    if (reset == 1 || enable == 0) begin
        encoderPosition = 0;
    end else if ( enable == 1 && clockwise == 1'b1 && encoderPosition < 9 ) begin
        encoderPosition = encoderPosition + 1;
    end else if (enable == 1 && counterclockwise == 1'b1 && encoderPosition > -9) begin
        encoderPosition = encoderPosition - 1;
    end
end

reg [3:0] minusSign, absoluteEncoderPosition;

always @(*) begin
    if(encoderPosition < 0) begin
        minusSign = 4'd15;
        absoluteEncoderPosition = -encoderPosition;
    end else begin
        minusSign = 4'd14;
       absoluteEncoderPosition = encoderPosition;
    end
 end

wire clk_1KHz, clk_1KHz_risingEdge;
clockDivider #(.THRESHOLD(50_000)) CLOCK_1HZ_GENERATOR (
    .clk(clk),
    .reset(reset),
    .enable(enable),
    .dividedClk(clk_1KHz)
);

edgeDetector CLOCK_1KHZ_EDGE (
    .clk(clk),
    .signalIn(clk_1KHz),
    .signalOut(),
    .risingEdge(clk_1KHz_risingEdge),
    .fallingEdge()
);
 reg [1:0] activeDisplay = 0;
 reg [3:0] ssdNumber = 4'd14;
 
 always @(posedge clk_1KHz_risingEdge) begin
     activeDisplay = activeDisplay + 1;
 end
 
 always @(*) begin
     case(activeDisplay)
         3'd0 : begin
             ssdNumber <= absoluteEncoderPosition;
             ssdAnode <= 4'b0111;
         end
         3'd1 : begin
             ssdNumber <= minusSign;
             ssdAnode <= 4'b1011;
         end
         default : begin
             ssdAnode <= 4'b1111; // none active
         end
      endcase
 end
 
 sevenSegmentDecoder rotarypositionDisplay
 ( .bcd(ssdNumber),
   .ssd(ssdCathode)
 );
 endmodule


// edge detector

`timescale 1ns / 1ps

module edgeDetector(
    input clk,
    input signalIn,
    output wire signalOut,
    output reg risingEdge,
    output reg fallingEdge
    );
    
reg [1:0] pipeline;

always @(posedge clk) begin
     pipeline[0] <= signalIn;
     pipeline[1] <= pipeline[0];
end

always @(*) begin
    if(pipeline[0] == 1 && pipeline[1] == 0) begin
        risingEdge <= 1;
    end else if (pipeline[0] == 0 && pipeline[1] == 1) begin
        fallingEdge <=1;
    end else begin
        risingEdge <= 0;
        fallingEdge <= 0;
    end  
end       

assign signalOut = pipeline[1];

endmodule

// Latch debouncer

`timescale 1ns / 1ps
// Use this debouncer to debounce the rotary encoder

module latchDebouncer #(
parameter integer LIMIT = 20_000_000
)(
input clk,
input inputSignal,
output reg debouncedSignal
);

wire inputSignal_risingEdge;

edgeDetector INPUT_EDGE_DETECTOR (
.clk(clk),
.signalIn(inputSignal),
.signalOut(),
.risingEdge(inputSignal_risingEdge),
.fallingEdge()
);

reg holdOffLatch;

reg [$clog2(LIMIT)-1:0] holdOffCounter; // $clog2 is a ceiling'd log 2 function.

always @(posedge clk) begin
    if (inputSignal_risingEdge == 1 && holdOffLatch == 0) begin
        holdOffLatch <= 1;
    end else if (holdOffCounter >= LIMIT-1) begin
        // When holdOffCounter reaches limit, turn the latch off
        holdOffLatch <= 0;
    end
end

always @(posedge clk) begin
    if (holdOffLatch == 1) begin
        holdOffCounter <= holdOffCounter + 1;
    end else if (holdOffLatch == 0) begin
        holdOffCounter <= 0;
    end
end

always @(*) begin
    debouncedSignal <= holdOffLatch;
end

endmodule

// seven segment decoder

`timescale 1ns / 1ps

module sevenSegmentDecoder(
    input wire [3:0] bcd,
    output reg [6:0] ssd
 );
 
always @(*) begin
   if (bcd == 4'd0) begin
        ssd = 7'b0000001;
   end else if (bcd == 4'd1) begin
        ssd = 7'b1001111;
   end else if (bcd == 4'd2) begin
        ssd = 7'b0010010;
   end else if (bcd == 4'd3) begin
        ssd = 7'b0000110;
   end else if (bcd == 4'd4) begin
        ssd = 7'b1001100;
   end else if (bcd == 4'd5) begin
        ssd = 7'b0100100;
   end else if (bcd == 4'd6) begin
        ssd = 7'b0100000;
   end else if (bcd == 4'd7) begin
        ssd = 7'b0001111;
   end else if (bcd == 4'd8) begin
        ssd = 7'b0000000;
   end else if (bcd == 4'd9) begin
        ssd = 7'b0001100;
   end else if (bcd == 4'd14) begin
         ssd = 7'b1111111;
   end else if (bcd == 4'd15) begin
        ssd = 7'b1111110;
   end else begin
        ssd = 7'b1111111;
  end
end

endmodule

// Constraints
set_property -dict {PACKAGE_PIN W5 IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 10.00 [get_ports clk]
set_property -dict {PACKAGE_PIN R2 IOSTANDARD LVCMOS33} [get_ports reset]
set_property -dict {PACKAGE_PIN T1 IOSTANDARD LVCMOS33} [get_ports enable]
set_property -dict {PACKAGE_PIN W4 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[0]}]
set_property -dict {PACKAGE_PIN V4 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[1]}]
set_property -dict {PACKAGE_PIN U4 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[2]}]
set_property -dict {PACKAGE_PIN U2 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[3]}]


set_property -dict {PACKAGE_PIN W7 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[6]}]
set_property -dict {PACKAGE_PIN W6 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[5]}]
set_property -dict {PACKAGE_PIN U8 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[4]}]
set_property -dict {PACKAGE_PIN V8 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[3]}]
set_property -dict {PACKAGE_PIN U5 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[2]}]
set_property -dict {PACKAGE_PIN V5 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[1]}]
set_property -dict {PACKAGE_PIN U7 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[0]}]

set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS33} [get_ports {A}]
set_property -dict {PACKAGE_PIN M18 IOSTANDARD LVCMOS33} [get_ports {B}]




