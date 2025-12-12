module peripheral_uart#(
   parameter clk_freq = 27000000,
   parameter baud     = 115200
)(
   input clk,
   input rst,
   input [7:0] d_in,
   input cs,
   input [4:0] addr,
   input wr,
   output reg [31:0] d_out,
   output uart_tx,
   input uart_rx,
   output reg ledout
);

   reg [1:0] s;
   reg [7:0] d_in_uart;
   reg [7:0] uart_ctrl;
   wire [7:0] rx_data;
   wire tx_busy;
   wire rx_error;
   wire rx_avail;

   // Address decoder
   always @(*) begin
      case (addr)
         5'h08: s = (cs) ? 2'b01 : 2'b00; // rx_data
         5'h10: s = (cs) ? 2'b10 : 2'b00; // status
         default: s = 2'b00;
      endcase
   end

   // Input registers
   always @(posedge clk) begin
      if(rst) begin
         d_in_uart <= 0;
         uart_ctrl <= 0;
         ledout <= 0;
      end else begin
         if (s[0] & wr) begin
            d_in_uart <= d_in[7:0];
         end
         if (s[1] & wr) begin
            uart_ctrl <= d_in[7:0];
            ledout <= d_in[2]; // LED control
         end
      end
   end

   // Output registers
   always @(*) begin
      case (s)
         2'b01: d_out = {24'b0, rx_data};
         2'b10: d_out = {22'b0, tx_busy, rx_avail, rx_error, 7'b0};
         default: d_out = 32'b0;
      endcase
   end

   // UART instance
   uart #(
      .freq_hz(clk_freq),
      .baud(baud)
   ) uart0(
      .reset(rst),
      .clk(clk),
      .uart_rxd(uart_rx),
      .uart_txd(uart_tx),
      .rx_data(rx_data),
      .rx_avail(rx_avail),
      .rx_error(rx_error),
      .rx_ack(uart_ctrl[0]),
      .tx_data(d_in_uart),
      .tx_wr(uart_ctrl[3]),
      .tx_busy(tx_busy)
   );

endmodule