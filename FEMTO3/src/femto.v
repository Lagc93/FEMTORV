/*
module femto (
   input         clk,        // 27MHz system clock 
   input         resetn,     // reset button (active low)
   
   // SPI Flash
   output        spi_mosi,
   input         spi_miso,
   output        spi_cs_n,
   output        spi_clk,
   
   // SPI RAM  
   output        spi_clk_ram,
   output        spi_cs_n_ram,
   input         spi_miso_ram,
   output        spi_mosi_ram,
   
   // LEDs and UART
   output        LEDS,
   output        LED_EXEC,   // Nuevo LED para indicar ejecución
   input         RXD,
   output        TXD
);

   wire [31:0] mem_address;
   reg  [31:0] mem_rdata;
   wire mem_rstrb;
   wire [31:0] mem_wdata;
   wire [3:0]  mem_wmask;

   wire mapped_spi_flash_rbusy;
   wire spi_ram_rbusy;
   wire spi_ram_wbusy;
   wire [31:0] dpram_dout;
   wire [31:0] RAM_rdata;
   wire [31:0] uart_dout;
   wire [31:0] mult_dout;
   wire [31:0] firmware_dout;

   // Señales para el LED de ejecución
   reg [23:0] exec_counter;
   reg led_exec_state;
   wire cpu_active;

   wire wr = |mem_wmask;
   wire rd = mem_rstrb; 


   // Address Decoder
   reg [6:0] cs;
   always @(*) begin
      // UART en 0x00400000 - 0x0040001F
      if (mem_address >= 32'h00400000 && mem_address < 32'h00400020) begin
         cs = 7'b0100000;  // UART (bit 5)
      end 
      // MULT en 0x00420000 - 0x0042001F
      else if (mem_address >= 32'h00420000 && mem_address < 32'h00420020) begin
         cs = 7'b0001000;  // MULT (bit 3)
      end
      // RAM en 0x00010000 - 0x0001FFFF
      else if (mem_address >= 32'h00010000 && mem_address < 32'h00020000) begin
         cs = 7'b1000000;  // RAM (bit 6)
      end
      // ROM en 0x00000000 - 0x000003FF
      else if (mem_address < 32'h00000400) begin
         cs = 7'b0000001;  // ROM (bit 0)
      end
      else begin
         cs = 7'b0000000;
      end
   end

   // CPU Instance - SIN BUSY SIGNALS (usar versión original)
   FemtoRV32 CPU(
      .clk(clk),
      .reset(resetn),		 
      .mem_addr(mem_address),
      .mem_rdata(mem_rdata),
      .mem_rstrb(mem_rstrb),
      .mem_wdata(mem_wdata),
      .mem_wmask(mem_wmask),
      .mem_rbusy(1'b0), // Deshabilitado temporalmente
      .mem_wbusy(1'b0)  // Deshabilitado temporalmente
   );

   // Detector de actividad del CPU
   assign cpu_active = mem_rstrb | (|mem_wmask);

   // Contador para el LED de ejecución (parpadeo lento)
   always @(posedge clk) begin
      if (!resetn) begin
         exec_counter <= 0;
         led_exec_state <= 0;
      end else begin
         exec_counter <= exec_counter + 1;
         
         // Cambiar estado del LED cada ~0.5 segundos (27MHz / 2^24 ≈ 0.62s)
         if (exec_counter == 0) begin
            led_exec_state <= ~led_exec_state;
         end
      end
   end

   // LED de ejecución - parpadea cuando el CPU está activo
   assign LED_EXEC = cpu_active ? led_exec_state : 1'b0;

   // Firmware ROM
   firmware_flash firmware_rom(
      .address(mem_address[9:2]),
      .data(firmware_dout)
   );

   // SPI RAM
   MappedSPIRAM mapped_spi_ram(
      .clk(clk),
      .reset(resetn),
      .word_address(mem_address[21:2]),
      .wdata(mem_wdata),
      .rd(cs[6] & rd),
      .wr(cs[6] & wr),
      .rbusy(spi_ram_rbusy),
      .wbusy(spi_ram_wbusy),
      .CLK(spi_clk_ram),
      .CS_N(spi_cs_n_ram),
      .MISO(spi_miso_ram),
      .MOSI(spi_mosi_ram),
      .rdata(dpram_dout)
   );

   // UART Peripheral (ORIGINAL)
   peripheral_uart #(
     .clk_freq(27000000),
     .baud(115200)
   ) per_uart(
     .clk(clk), 
     .rst(!resetn), 
     .d_in(mem_wdata[7:0]), 
     .cs(cs[5]), 
     .addr(mem_address[4:0]), 
     .wr(wr), 
     .d_out(uart_dout), 
     .uart_tx(TXD), 
     .uart_rx(RXD), 
     .ledout(LEDS)
   );

   // Multiplier Peripheral
   peripheral_mult mult1 (
      .clk(clk), 
      .reset(!resetn), 
      .d_in(mem_wdata[15:0]), 
      .cs(cs[3]), 
      .addr(mem_address[4:0]), 
      .rd(rd), 
      .wr(wr), 
      .d_out(mult_dout) 
   );

   // Read Multiplexer
   always @(*) begin
      case (cs)
        7'b1000000: mem_rdata = dpram_dout;
        7'b0100000: mem_rdata = uart_dout;
        7'b0001000: mem_rdata = mult_dout;
        7'b0000001: mem_rdata = firmware_dout;
        default:    mem_rdata = 32'h00000000;
      endcase
   end

   // Conectar señales SPI Flash a valores por defecto
   assign spi_mosi = 1'b0;
   assign spi_cs_n = 1'b1;
   assign spi_clk = 1'b0;

endmodule
*/

module femto (
   input         clk,        // 27MHz system clock 
   input         resetn,     // reset button (active low)
   
   // SPI Flash
   output        spi_mosi,
   input         spi_miso,
   output        spi_cs_n,
   output        spi_clk,
   
   // SPI RAM  
   output        spi_clk_ram,
   output        spi_cs_n_ram,
   input         spi_miso_ram,
   output        spi_mosi_ram,
   
   // LEDs and UART
   output        LEDS,
   output        LED_EXEC,   // Nuevo LED para indicar ejecución
   input         RXD,
   output        TXD
);

   wire [31:0] mem_address;
   reg  [31:0] mem_rdata;
   wire mem_rstrb;
   wire [31:0] mem_wdata;
   wire [3:0]  mem_wmask;

   wire mapped_spi_flash_rbusy;
   wire spi_ram_rbusy;
   wire spi_ram_wbusy;
   wire [31:0] dpram_dout;
   wire [31:0] RAM_rdata;
   wire [31:0] uart_dout;
   wire [31:0] mult_dout;
   wire [31:0] firmware_dout;

   // Señales para el LED de ejecución
   reg [23:0] exec_counter;
   reg led_exec_state;
   wire cpu_active;

   wire wr = |mem_wmask;
   wire rd = mem_rstrb; 


   // Address Decoder
   reg [6:0] cs;
   always @(*) begin
      // UART en 0x00400000 - 0x0040001F
      if (mem_address >= 32'h00400000 && mem_address < 32'h00400020) begin
         cs = 7'b0100000;  // UART (bit 5)
      end 
      // MULT en 0x00420000 - 0x0042001F
      else if (mem_address >= 32'h00420000 && mem_address < 32'h00420020) begin
         cs = 7'b0001000;  // MULT (bit 3)
      end
      // RAM en 0x00010000 - 0x0001FFFF
      else if (mem_address >= 32'h00010000 && mem_address < 32'h00020000) begin
         cs = 7'b1000000;  // RAM (bit 6)
      end
      // ROM en 0x00000000 - 0x000003FF
      else if (mem_address < 32'h00000400) begin
         cs = 7'b0000001;  // ROM (bit 0)
      end
      else begin
         cs = 7'b0000000;
      end
   end

   // CPU Instance - IGNORAR mem_wbusy para UART
   FemtoRV32 CPU(
      .clk(clk),
      .reset(resetn),		 
      .mem_addr(mem_address),
      .mem_rdata(mem_rdata),
      .mem_rstrb(mem_rstrb),
      .mem_wdata(mem_wdata),
      .mem_wmask(mem_wmask),
      .mem_rbusy(spi_ram_rbusy),  // Solo RAM puede causar busy en lectura
      .mem_wbusy(1'b0)           // ¡IGNORAR completamente mem_wbusy!
   );

   // Detector de actividad del CPU
   assign cpu_active = mem_rstrb | (|mem_wmask);

   // Contador para el LED de ejecución (parpadeo lento)
   always @(posedge clk) begin
      if (!resetn) begin
         exec_counter <= 0;
         led_exec_state <= 0;
      end else begin
         exec_counter <= exec_counter + 1;
         
         // Cambiar estado del LED cada ~0.5 segundos (27MHz / 2^24 ≈ 0.62s)
         if (exec_counter == 0) begin
            led_exec_state <= ~led_exec_state;
         end
      end
   end

   // LED de ejecución - parpadea cuando el CPU está activo
   assign LED_EXEC = cpu_active ? led_exec_state : 1'b0;

   // Firmware ROM
   firmware_flash firmware_rom(
      .address(mem_address[9:2]),
      .data(firmware_dout)
   );

   // SPI RAM
   MappedSPIRAM mapped_spi_ram(
      .clk(clk),
      .reset(resetn),
      .word_address(mem_address[21:2]),
      .wdata(mem_wdata),
      .rd(cs[6] & rd),
      .wr(cs[6] & wr),
      .rbusy(spi_ram_rbusy),
      .wbusy(spi_ram_wbusy),
      .CLK(spi_clk_ram),
      .CS_N(spi_cs_n_ram),
      .MISO(spi_miso_ram),
      .MOSI(spi_mosi_ram),
      .rdata(dpram_dout)
   );

   // UART Peripheral
   peripheral_uart #(
     .clk_freq(27000000),
     .baud(115200)
   ) per_uart(
     .clk(clk), 
     .rst(!resetn), 
     .d_in(mem_wdata[7:0]), 
     .cs(cs[5]), 
     .addr(mem_address[4:0]), 
     .wr(wr), 
     .d_out(uart_dout), 
     .uart_tx(TXD), 
     .uart_rx(RXD), 
     .ledout(LEDS)
   );

   // Multiplier Peripheral
   peripheral_mult mult1 (
      .clk(clk), 
      .reset(!resetn), 
      .d_in(mem_wdata[15:0]), 
      .cs(cs[3]), 
      .addr(mem_address[4:0]), 
      .rd(rd), 
      .wr(wr), 
      .d_out(mult_dout) 
   );

   // Read Multiplexer
   always @(*) begin
      case (cs)
        7'b1000000: mem_rdata = dpram_dout;
        7'b0100000: mem_rdata = uart_dout;
        7'b0001000: mem_rdata = mult_dout;
        7'b0000001: mem_rdata = firmware_dout;
        default:    mem_rdata = 32'h00000000;
      endcase
   end

   // Conectar señales SPI Flash a valores por defecto
   assign spi_mosi = 1'b0;
   assign spi_cs_n = 1'b1;
   assign spi_clk = 1'b0;

endmodule