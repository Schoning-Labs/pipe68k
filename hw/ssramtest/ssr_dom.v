// One clock domain: NG real groups + 1 canary group (checker deliberately wrong, must report errors).
// Every 2^16 cycles an epoch passes and the data mode changes; the epoch count goes out as Gray code.
module ssr_dom #(parameter NG = 32, parameter [31:0] BASE = 32'h1000)
  (input clk, input rst, output [(NG+1)*4-1:0] therm, output reg [19:0] gray);
  reg [15:0] cyc;
  reg [19:0] ep;
  reg [2:0] mode;
  reg [3:0] wlk;
  always @(posedge clk) begin
    if (rst) begin cyc <= 0; ep <= 0; mode <= 0; wlk <= 4'b0001; gray <= 0; end
    else begin
      cyc <= cyc + 1; wlk <= {wlk[2:0], wlk[3]};
      if (cyc == 16'hFFFF) begin ep <= ep + 1; mode <= ep[2:0] + 3'd1; end
      gray <= ep ^ (ep >> 1);
    end
  end
  genvar g;
  generate for (g = 0; g <= NG; g = g + 1) begin : gr
    (* syn_keep = 1 *) reg [2:0] m;
    (* syn_keep = 1 *) reg [3:0] w;
    always @(posedge clk) begin m <= mode; w <= wlk; end
    ssr_grp #(.NR(24), .SEED(BASE * 32'h9E3779B1 + g * 32'h2545F491 + 32'h1), .CANARY(g == NG))
      u (.clk(clk), .rst(rst), .mode(m), .wlk(w), .therm(therm[g*4 +: 4]));
  end endgenerate
endmodule
