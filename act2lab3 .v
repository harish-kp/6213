`timescale 1ns / 1ps
module multipleSevenSegmentDisplay_TOP(
    input clk,
    input reset,
    input enable,
    output [6:0] ssdCathode,
    output  [3:0] ssdAnode
    );
    wire clk_1kHz;
    wire clk_1kHz_risingEdge;
  integerClockDivider #(
     .THRESHOLD(50_000)
     ) CLOCK_1kHZ_GENERATOR (
     .clk(clk),
     .reset(1'b0),
     .enable(1'b1),
     .dividedClk(clk_1kHz)
     );  
     edgeDetector CLOCK_1kHZ_EDGE (
     .clk(clk),
     .signalIn(clk_1kHz),
     .risingEdge(clk_1kHz_risingEdge),
     .fallingEdge()
     );
     reg [1:0] activeDisplay;
     sevenSegmentDecoder  (
          .activeDisplay(activeDisplay),
          .ssd(ssdCathode),
          .ssdAnode(ssdAnode)
          );
          
     
     always @ (posedge clk_1kHz) begin
     activeDisplay <= activeDisplay + 1;
     end
     
endmodule



`timescale 1ns / 1ps
module sevenSegmentDecoder(
    input [2:0] activeDisplay,
    output reg [3:0] ssdAnode,
    output reg [6:0] ssd
    );
    reg [3:0] ssdNumber;
    always @(*)
    begin
    case (ssdNumber)
    4'd0 : ssd = 7'b0000001;
    4'd1 : ssd = 7'b1001111;
    4'd2 : ssd = 7'b0010010;
    4'd3 : ssd = 7'b0000110;
    4'd4 : ssd = 7'b1001100;
    4'd5 : ssd = 7'b0100100;
    4'd6 : ssd = 7'b0100000;
    4'd7 : ssd = 7'b0001111;
    4'd8 : ssd = 7'b0000000;
    4'd9 : ssd = 7'b0000100;
    default : ssd = 7'b1111111;
    endcase
    end
             always @(*) begin
              case(activeDisplay)
              3'd0 : begin
              ssdNumber <= 4'd0;
              ssdAnode <= 8'b1111_1110;
              end
              3'd1 : begin
              ssdNumber <= 4'd2;
              ssdAnode <= 8'b1111_1101;
              end
              3'd2 : begin
              ssdNumber <= 4'd0;
              ssdAnode <= 8'b1111_1011;
              end
              3'd3 : begin
              ssdNumber <= 4'd1;
              ssdAnode <= 8'b1111_0111;
              end
//              3'd4 : begin
//              ssdNumber <= 4'd6;
//              ssdAnode <= 8'b1110_1111;
//              end
//              3'd5 : begin
//              ssdNumber <= 4'd6;
//              ssdAnode <= 8'b1101_1111;
//              end
//              3'd6 : begin
//              ssdNumber <= 4'd3;
//              ssdAnode <= 8'b1011_1111;
//              end              
//              3'd7 : begin
//              ssdNumber <= 4'd6;
//              ssdAnode <= 8'b0111_1111;
//              end
              default : begin
              ssdNumber <= 4'd15; // undefined
              ssdAnode <= 8'b1111_1111; // none active
              end
              endcase
              end
endmodule





`timescale 1ns / 1ps
module edgeDetector (
    input wire clk,
    input wire signalIn,
    output wire signalOut,
    output reg risingEdge,
    output reg fallingEdge
);

    reg [1:0] pipeline;
    reg [3:0] counter;
    always @(*) begin
        pipeline[0] = signalIn;
    end

    always @(posedge clk) begin
        pipeline[1] <= pipeline[0];
    end

    always @(*) begin
        if (pipeline == 2'b01) begin
            risingEdge <= 1;
        end else if (pipeline == 2'b10) begin
            fallingEdge <= 1;
        end else begin
            risingEdge <= 0;
            fallingEdge <= 0;
        end
    end

    assign signalOut = pipeline[1];
    
endmodule



module integerClockDivider #(
    parameter integer THRESHOLD = 50_000_000
) (
    input clk,
    input reset,
    input enable,
    output reg dividedClk
);

    reg [34:0] counter = 0;

    always @(posedge clk) begin
        if (reset == 1 || counter >= THRESHOLD-1) begin
            counter <= 0;
        end else if (enable == 1) begin
            counter <= counter + 1;
        end
    end

    always @(posedge clk) begin
        if (reset == 1) begin
            dividedClk <= 0;
        end else if (counter >= THRESHOLD-1) begin
            dividedClk <= ~dividedClk;
        end
    end

endmodule



set_property -dict {PACKAGE_PIN W5 IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 10.00 [get_ports clk]
set_property -dict {PACKAGE_PIN R2 IOSTANDARD LVCMOS33} [get_ports reset]
set_property -dict {PACKAGE_PIN T1 IOSTANDARD LVCMOS33} [get_ports enable]
set_property -dict {PACKAGE_PIN W4 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[0]}]
set_property -dict {PACKAGE_PIN V4 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[1]}]
set_property -dict {PACKAGE_PIN U4 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[2]}]
set_property -dict {PACKAGE_PIN U2 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[3]}]
#set_property -dict {PACKAGE_PIN V4 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[4]}]
#set_property -dict {PACKAGE_PIN U4 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[5]}]
#set_property -dict {PACKAGE_PIN U2 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[6]}]
#set_property -dict {PACKAGE_PIN U2 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[7]}]

set_property -dict {PACKAGE_PIN W7 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[6]}]
set_property -dict {PACKAGE_PIN W6 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[5]}]
set_property -dict {PACKAGE_PIN U8 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[4]}]
set_property -dict {PACKAGE_PIN V8 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[3]}]
set_property -dict {PACKAGE_PIN U5 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[2]}]
set_property -dict {PACKAGE_PIN V5 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[1]}]
set_property -dict {PACKAGE_PIN U7 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[0]}]

#set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports clk]
#create_clock -period 10.00 [get_ports clk]
#set_property -dict {PACKAGE_PIN N17 IOSTANDARD LVCMOS33} [get_ports reset]
#set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33} [get_ports enable]
#set_property -dict {PACKAGE_PIN J17 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[0]}]
#set_property -dict {PACKAGE_PIN J18 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[1]}]
#set_property -dict {PACKAGE_PIN T9 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[2]}]
#set_property -dict {PACKAGE_PIN J14 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[3]}]
#set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[4]}]
#set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[5]}]
#set_property -dict {PACKAGE_PIN K2 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[6]}]
#set_property -dict {PACKAGE_PIN U13 IOSTANDARD LVCMOS33} [get_ports {ssdAnode[7]}]
#set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[6]}]
#set_property -dict {PACKAGE_PIN R10 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[5]}]
#set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[4]}]
#set_property -dict {PACKAGE_PIN K13 IOSTANDARD LVCMOS33} [get_ports {ssdCat0
#set_property -dict {PACKAGE_PIN P15 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[2]}]
#set_property -dict {PACKAGE_PIN T11 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[1]}]
#set_property -dict {PACKAGE_PIN L18 IOSTANDARD LVCMOS33} [get_ports {ssdCathode[0]}]