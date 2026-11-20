// SSRAM test on the Tang Mega 138K Pro: (NG+1) groups x 24 RAM16SDP4 on the 50 MHz board clock; the last group is the
// canary, its checker is wrong on purpose and must show errors.
// Status line on the UART (P15, 115200 8N1) about once a second:
//   S eeeee <NG+1 digits> PASS|FAIL|CANX   (eeeee = epochs of 65536 cycles, hex; digit = errors seen per group, 0..4,
//   4 = four or more; the last digit is the canary and must be 4)
module ssr_top #(parameter NG = 62, parameter [25:0] TICK = 26'd49999999, parameter [8:0] BAUD = 9'd433)
  (input clk, output uart_tx, output [5:0] led);
  localparam W = (NG + 1) * 4;
  localparam D = 9 + NG;                       // index of the space after the digits
  localparam LAST = D + 6;
  reg [7:0] r50 = 0;
  wire rst50 = (r50 != 8'hFF);
  always @(posedge clk) if (r50 != 8'hFF) r50 <= r50 + 1;
  wire [W-1:0] th;
  wire [19:0] gry;
  ssr_dom #(.NG(NG), .BASE(32'h1000)) d (.clk(clk), .rst(rst50), .therm(th), .gray(gry));
  reg [W-1:0] sa, sb;
  always @(posedge clk) begin sa <= th; sb <= sa; end
  function [19:0] g2b(input [19:0] g);
    integer j;
    begin g2b[19] = g[19]; for (j = 18; j >= 0; j = j - 1) g2b[j] = g2b[j+1] ^ g[j]; end
  endfunction
  reg fail, canary_ok;
  integer q;
  always @(posedge clk) begin
    fail <= 0;
    for (q = 0; q < NG; q = q + 1) if (sb[q*4]) fail <= 1;
    canary_ok <= sb[NG*4];
  end
  reg [25:0] tmr = 0;
  reg [7:0] n = 0;
  reg busy = 0;
  reg [19:0] e;
  reg [W-1:0] t;
  reg fl, cok;
  function [7:0] hex(input [3:0] v); hex = v < 10 ? 8'd48 + v : 8'd55 + v; endfunction
  function [7:0] dig(input [3:0] x); dig = x[3] ? "4" : x[2] ? "3" : x[1] ? "2" : x[0] ? "1" : "0"; endfunction
  reg [7:0] c;
  always @* begin
    c = " ";
    if (n == 0) c = "S";
    else if (n >= 2 && n <= 6) c = hex(e[(6-n)*4 +: 4]);
    else if (n >= 8 && n <= 8 + NG) c = dig(t[(n-8)*4 +: 4]);
    else if (n == D + 1) c = !cok ? "C" : fl ? "F" : "P";
    else if (n == D + 2) c = !cok ? "A" : "A";
    else if (n == D + 3) c = !cok ? "N" : fl ? "I" : "S";
    else if (n == D + 4) c = !cok ? "X" : fl ? "L" : "S";
    else if (n == D + 5) c = 8'h0D;
    else if (n == D + 6) c = 8'h0A;
  end
  reg [3:0] bitn = 0;
  reg txr = 1;
  reg [8:0] bd2 = 0;
  reg [7:0] cur;
  reg active = 0;
  always @(posedge clk) begin
    tmr <= tmr + 1;
    if (tmr == TICK) begin
      tmr <= 0; busy <= 1; n <= 0;
      e <= g2b(gry); t <= sb; fl <= fail; cok <= canary_ok;
    end
    if (bd2 == BAUD) begin
      bd2 <= 0;
      if (!active) begin
        if (busy) begin cur <= c; active <= 1; bitn <= 0; txr <= 0; end
      end else begin
        if (bitn < 8) begin txr <= cur[bitn[2:0]]; bitn <= bitn + 1; end
        else if (bitn == 8) begin txr <= 1; bitn <= 9; end
        else begin
          active <= 0;
          if (n == LAST) busy <= 0; else n <= n + 1;
        end
      end
    end else bd2 <= bd2 + 1;
  end
  assign uart_tx = txr;
  // LEDs (Sipeed: low = lit): 0 heartbeat, 1 pass, 2 fail
  assign led = ~{3'b000, fail, canary_ok & ~fail, tmr[25]};
endmodule
