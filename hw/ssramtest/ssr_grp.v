// SSRAM test group: NR RAM16SDP4 primitives, one pseudo-random write/read stream, one register model.
// Every RAM gets the same write; each RAM's asynchronous read is compared with the model.
module ssr_grp #(parameter NR = 24, parameter [31:0] SEED = 32'h1, parameter CANARY = 0)
  (input clk, input rst, input [2:0] mode, input [3:0] wlk, output reg [3:0] therm);
  reg [31:0] lf;
  reg [3:0] wad, rad, di;
  reg we;
  reg [63:0] model;                       // flat registers, not a RAM: the reference must not be SSRAM
  reg [3:0] exp_q;
  reg [3:0] do_q [0:NR-1];
  wire [3:0] dout [0:NR-1];
  reg [NR-1:0] e1;
  reg err;
  integer k;
  wire [3:0] rnd = lf[15:12];
  always @(posedge clk) begin
    if (rst) begin
      lf <= SEED; we <= 0; wad <= 0; rad <= 0; di <= 0; therm <= 4'b0; err <= 0; e1 <= 0; exp_q <= 0;
      model <= 64'h0;
      for (k = 0; k < NR; k = k + 1) do_q[k] <= 4'h0;
    end else begin
      lf  <= {lf[30:0], lf[31] ^ lf[21] ^ lf[1] ^ lf[0]};
      wad <= lf[7:4]; rad <= lf[3:0]; we <= lf[8] | lf[9];
      case (mode)
        3'd1: di <= 4'h0;
        3'd2: di <= 4'hF;
        3'd3: di <= wlk;
        3'd4: di <= ~wlk;
        3'd5: di <= wlk[0] ? 4'h5 : 4'hA;
        default: di <= rnd;
      endcase
      if (we) model[wad*4 +: 4] <= di;
      exp_q <= model[rad*4 +: 4] ^ (CANARY ? {3'b000, lf[20] & lf[19]} : 4'h0);
      for (k = 0; k < NR; k = k + 1) begin
        do_q[k] <= dout[k];
        e1[k]   <= (do_q[k] != exp_q);
      end
      err <= |e1;
      if (err) therm <= {therm[2:0], 1'b1};
    end
  end
  genvar i;
  generate for (i = 0; i < NR; i = i + 1) begin : r
    (* syn_keep = 1, keep = 1 *) RAM16SDP4 u (.DI(di), .WRE(we), .CLK(clk), .WAD(wad), .RAD(rad), .DO(dout[i]));
    defparam u.INIT_0 = 16'h0000;
    defparam u.INIT_1 = 16'h0000;
    defparam u.INIT_2 = 16'h0000;
    defparam u.INIT_3 = 16'h0000;
  end endgenerate
endmodule
