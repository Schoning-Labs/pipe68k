// Measurement top for the pipe68k pipeline sample: serial in, one serial out, a fake memory with 3 clocks latency
// that returns data derived from the address and the serial input (so nothing is constant). Timing only.
`default_nettype none
module p68_top (input wire clk, input wire din, output wire dout);
  reg [63:0] sh = 64'h5a3c96e1_0f1e2d3c;
  always @(posedge clk) sh <= {sh[62:0], din};
  reg rst_r = 1'b1;
  always @(posedge clk) rst_r <= sh[63] & sh[62] & sh[61];

  wire        bus_req, bw_en;
  wire [31:2] bus_addr, bw_addr;
  wire [31:0] bw_data, obs_a, obs_b;
  wire [3:0]  bw_be;
  wire [4:0]  obs_ccr;
  reg  [31:0] rdata;
  reg         ack = 1'b0;
  reg  [1:0]  lat = 2'd0;
  always @(posedge clk) begin
    ack <= 1'b0;
    if (bus_req && !ack) begin
      if (lat == 2'd2) begin ack <= 1'b1; lat <= 2'd0; rdata <= {bus_addr, 2'b00} ^ sh[31:0] ^ {sh[47:32], sh[63:48]}; end
      else lat <= lat + 2'd1;
    end
  end

  p68_core core (
    .clk(clk), .rst(rst_r),
    .bus_req(bus_req), .bus_addr(bus_addr), .bus_rdata(rdata), .bus_ack(ack),
    .bw_en(bw_en), .bw_addr(bw_addr), .bw_data(bw_data), .bw_be(bw_be),
    .obs_a(obs_a), .obs_b(obs_b), .obs_ccr(obs_ccr));

  reg [31:0] acc = 32'h0;
  always @(posedge clk) if (bw_en) acc <= acc ^ bw_data ^ {bw_addr, 2'b00} ^ {28'h0, bw_be};
  reg [31:0] o1; reg [7:0] o2; reg o3;
  always @(posedge clk) begin
    o1 <= obs_a ^ obs_b ^ acc ^ {27'h0, obs_ccr};
    o2 <= {^o1[31:28], ^o1[27:24], ^o1[23:20], ^o1[19:16], ^o1[15:12], ^o1[11:8], ^o1[7:4], ^o1[3:0]};
    o3 <= ^o2;
  end
  assign dout = o3;
endmodule
`default_nettype wire
