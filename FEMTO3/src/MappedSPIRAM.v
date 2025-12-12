module MappedSPIRAM(
   input wire clk,
   input wire reset,
   input wire rd,
   input wire wr,
   input wire [19:0] word_address,
   input wire [31:0] wdata,
   output wire [31:0] rdata,
   output reg rbusy,
   output reg wbusy,
   output wire CLK,  // Cambiado a wire
   output reg CS_N,
   output wire MOSI,
   input wire MISO
);

   parameter START = 2'b00;
   parameter WAIT_INST = 2'b01;
   parameter SEND = 2'b10;
   parameter RECEIVE = 2'b11;
   parameter divisor = 54; // 27MHz -> ~500kHz SPI

   reg [1:0] state;
   reg [5:0] div_counter;
   reg clk_enable;
   reg spi_clk_reg;

   reg [5:0] snd_bitcount;
   reg [31:0] cmd_addr;
   reg [5:0] rcv_bitcount;
   reg [31:0] rcv_data;

   // Clock divider for SPI
   always @(posedge clk) begin
      if (!reset) begin
         div_counter <= 0;
         clk_enable <= 0;
         spi_clk_reg <= 0;
      end else begin
         if (div_counter >= divisor) begin
            div_counter <= 0;
            clk_enable <= 1;
            spi_clk_reg <= ~spi_clk_reg;
         end else begin
            div_counter <= div_counter + 1;
            clk_enable <= 0;
         end
      end
   end

   // Asignación correcta - sin always block
   assign CLK = spi_clk_reg;

   // Main state machine
   always @(posedge clk) begin
      if (!reset) begin
         state <= START;
         rbusy <= 1'b0;
         wbusy <= 1'b0;
         rcv_data <= 0;
         CS_N <= 1;
         cmd_addr <= 0;
         snd_bitcount <= 0;
         rcv_bitcount <= 0;
      end else begin
         case(state)
            START: begin
               CS_N <= 1'b1;
               rbusy <= 1'b0;
               wbusy <= 1'b0;
               snd_bitcount <= 6'd0;
               rcv_bitcount <= 6'd0;
               state <= WAIT_INST;
            end

            WAIT_INST: begin
               if (rd) begin
                  CS_N <= 1'b0;
                  rbusy <= 1'b1;
                  wbusy <= 1'b0;
                  snd_bitcount <= 6'd24;
                  rcv_bitcount <= 6'd32;
                  cmd_addr <= {8'h03, word_address[15:0], 8'h00};
                  state <= SEND;
               end else if (wr) begin
                  CS_N <= 1'b0;
                  rbusy <= 1'b0;
                  wbusy <= 1'b1;
                  snd_bitcount <= 6'd32;
                  rcv_bitcount <= 6'd0;
                  cmd_addr <= {8'h02, word_address[15:0], wdata[7:0]};
                  state <= SEND;
               end
            end

            SEND: begin
               if(clk_enable) begin
                  if(snd_bitcount == 1) begin
                     state <= RECEIVE;
                  end else begin
                     snd_bitcount <= snd_bitcount - 6'd1;
                     cmd_addr <= {cmd_addr[30:0], 1'b1};
                  end
               end
            end

            RECEIVE: begin
               if(clk_enable) begin
                  if(rcv_bitcount == 0) begin
                     state <= START;
                  end else begin
                     rcv_bitcount <= rcv_bitcount - 6'd1;
                     rcv_data <= {rcv_data[30:0], MISO};
                  end
               end
            end

            default: state <= START;
         endcase
      end
   end

   assign MOSI = cmd_addr[31];
   assign rdata = {rcv_data[7:0], rcv_data[15:8], rcv_data[23:16], rcv_data[31:24]};

endmodule