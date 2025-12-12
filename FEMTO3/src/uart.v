/*
module uart #(
   parameter freq_hz = 27000000,
   parameter baud    = 115200
   //parameter baud    = 115200
) (
   input reset,
   input clk,
   input uart_rxd,
   output reg uart_txd,
   output reg [7:0] rx_data,
   output reg rx_avail,
   output reg rx_error,
   input rx_ack,
   input [7:0] tx_data,
   input tx_wr,
   output reg tx_busy
);

   // Usar localparam en lugar de parameter
   localparam divisor = freq_hz / baud / 16;
   localparam divisor_trunc = (divisor[15:0] - 1);

   // Enable16 generator
   reg [15:0] enable16_counter;
   wire enable16 = (enable16_counter == 0);

   always @(posedge clk) begin
      if (reset) begin
         enable16_counter <= divisor_trunc;
      end else begin
         if (enable16_counter == 0) begin
            enable16_counter <= divisor_trunc;
         end else begin
            enable16_counter <= enable16_counter - 1;
         end
      end
   end

   // Resto del código sin cambios...
   // Synchronize uart_rxd
   reg uart_rxd1, uart_rxd2;
   always @(posedge clk) begin
      uart_rxd1 <= uart_rxd;
      uart_rxd2 <= uart_rxd1;
   end

   // UART RX Logic
   reg rx_busy;
   reg [3:0] rx_count16;
   reg [3:0] rx_bitcount;
   reg [9:0] rxd_reg;

   always @(posedge clk) begin
      if (reset) begin
         rx_busy <= 0;
         rx_count16 <= 0;
         rx_bitcount <= 0;
         rx_avail <= 0;
         rx_error <= 0;
         rxd_reg <= 0;
      end else begin
         if (rx_ack) begin
            rx_avail <= 0;
            rx_error <= 0;
         end

         if (enable16) begin
            if (!rx_busy) begin
               if (!uart_rxd2) begin
                  rx_busy <= 1;
                  rx_count16 <= 7;
                  rx_bitcount <= 0;
               end
            end else begin
               rx_count16 <= rx_count16 + 1;

               if (rx_count16 == 0) begin
                  rx_bitcount <= rx_bitcount + 1;

                  if (rx_bitcount == 0) begin
                     if (uart_rxd2) begin
                        rx_busy <= 0;
                     end
                  end else if (rx_bitcount == 9) begin
                     rx_busy <= 0;
                     if (uart_rxd2) begin
                        rx_data <= rxd_reg[8:1];
                        rx_avail <= 1;
                        rx_error <= 0;
                     end else begin
                        rx_error <= 1;
                     end
                  end else begin
                     rxd_reg <= {uart_rxd2, rxd_reg[9:1]};
                  end
               end
            end
         end
      end
   end

   // UART TX Logic
   reg [3:0] tx_bitcount;
   reg [3:0] tx_count16;
   reg [7:0] txd_reg;

   always @(posedge clk) begin
      if (reset) begin
         tx_busy <= 0;
         uart_txd <= 1;
         tx_count16 <= 0;
      end else begin
         if (tx_wr && !tx_busy) begin
            txd_reg <= tx_data;
            tx_bitcount <= 0;
            tx_count16 <= 0;
            tx_busy <= 1;
         end

         if (enable16) begin
            tx_count16 <= tx_count16 + 1;

            if ((tx_count16 == 0) && tx_busy) begin
               tx_bitcount <= tx_bitcount + 1;

               if (tx_bitcount == 0) begin
                  uart_txd <= 0;
               end else if (tx_bitcount == 9) begin
                  uart_txd <= 1;
               end else if (tx_bitcount == 10) begin
                  tx_bitcount <= 0;
                  tx_busy <= 0;
               end else begin
                  uart_txd <= txd_reg[0];
                  txd_reg <= {1'b0, txd_reg[7:1]};
               end
            end
         end
      end
   end

endmodule
*/


module uart #(
   parameter freq_hz = 27000000,
   parameter baud    = 115200
) (
   input reset,
   input clk,
   input uart_rxd,
   output reg uart_txd,
   output reg [7:0] rx_data,
   output reg rx_avail,
   output reg rx_error,
   input rx_ack,
   input [7:0] tx_data,
   input tx_wr,
   output reg tx_busy,
   output reg tx_done  // NUEVO: indica cuando terminó la transmisión
);

   // Cálculo correcto: 27MHz / 115200 = 234.375
   localparam BIT_TIME = freq_hz / baud;  // 234
   localparam BIT_TIME_CNT = BIT_TIME - 1;
   
   // Estados TX
   reg [1:0] tx_state;
   reg [15:0] tx_counter;
   reg [3:0] tx_bitcount;
   reg [7:0] tx_shift;
   
   localparam TX_IDLE = 0;
   localparam TX_START = 1;
   localparam TX_DATA = 2;
   localparam TX_STOP = 3;
   
   // Máquina de estados TX
   always @(posedge clk) begin
      if (reset) begin
         tx_state <= TX_IDLE;
         uart_txd <= 1'b1;
         tx_busy <= 1'b0;
         tx_done <= 1'b0;
         tx_counter <= 0;
      end else begin
         tx_done <= 1'b0;  // Resetear después de 1 ciclo
         
         case (tx_state)
            TX_IDLE: begin
               uart_txd <= 1'b1;
               tx_busy <= 1'b0;
               if (tx_wr && !tx_busy) begin
                  tx_state <= TX_START;
                  tx_shift <= tx_data;
                  tx_bitcount <= 0;
                  tx_counter <= BIT_TIME_CNT;
                  tx_busy <= 1'b1;
               end
            end
            
            TX_START: begin
               uart_txd <= 1'b0;  // Start bit
               if (tx_counter == 0) begin
                  tx_state <= TX_DATA;
                  tx_counter <= BIT_TIME_CNT;
               end else begin
                  tx_counter <= tx_counter - 1;
               end
            end
            
            TX_DATA: begin
               uart_txd <= tx_shift[0];  // LSB primero
               if (tx_counter == 0) begin
                  tx_shift <= {1'b0, tx_shift[7:1]};  // Shift right
                  tx_bitcount <= tx_bitcount + 1;
                  tx_counter <= BIT_TIME_CNT;
                  
                  if (tx_bitcount == 7) begin
                     tx_state <= TX_STOP;
                  end
               end else begin
                  tx_counter <= tx_counter - 1;
               end
            end
            
            TX_STOP: begin
               uart_txd <= 1'b1;  // Stop bit
               if (tx_counter == 0) begin
                  tx_state <= TX_IDLE;
                  tx_done <= 1'b1;  // ¡NUEVO! Indicar que terminó
               end else begin
                  tx_counter <= tx_counter - 1;
               end
            end
         endcase
      end
   end
   
   // RX (simplificado para este ejemplo)
   always @(posedge clk) begin
      if (reset) begin
         rx_avail <= 1'b0;
         rx_error <= 1'b0;
         rx_data <= 8'h00;
      end
   end

endmodule

