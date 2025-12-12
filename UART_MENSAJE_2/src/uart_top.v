module uart_top (
    input  wire clk,
    input  wire rst_n,
    input  wire uart_rx,
    output wire uart_tx
);

    // Señales UART
    wire [7:0] rx_data;
    wire       rx_done;
    reg        tx_start;
    reg [7:0]  tx_data;
    wire       tx_busy;

    // Buffer mensaje
    reg [7:0] buffer [0:31];
    reg [5:0] idx;
    reg [5:0] send_idx;
    reg       sending;
    reg [2:0] state;
    reg       done_sending;

    // RX
    uart_rx #(
        .CLK_FREQ(27000000),
        .BAUD_RATE(115200)
    ) u_rx (
        .clk     (clk),
        .rst_n   (rst_n),
        .rx      (uart_rx),
        .rx_data (rx_data),
        .rx_done (rx_done)
    );

    // TX
    uart_tx #(
        .CLK_FREQ(27000000),
        .BAUD_RATE(115200)
    ) u_tx (
        .clk     (clk),
        .rst_n   (rst_n),
        .tx_start(tx_start),
        .tx_data (tx_data),
        .tx      (uart_tx),
        .tx_busy (tx_busy)
    );

    // Lógica
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx          <= 0;
            sending      <= 0;
            tx_start     <= 0;
            send_idx     <= 0;
            state        <= 0;
            done_sending <= 0;
        end else begin
            tx_start <= 0; // default

            // Recepción de datos
            if (rx_done && !sending) begin
                if (rx_data == 8'h0D) begin
                    // ENTER → procesar
                    if (idx != 0) begin
                        sending      <= 1;
                        send_idx     <= idx - 1;   // incluir la última letra
                        done_sending <= 0;         // se marcará después

                        // primero salto de línea (CR+LF)
                        tx_data  <= 8'h0D; // CR
                        tx_start <= 1;
                        state    <= 1;
                    end else begin
                        // mensaje vacío → solo CR+LF
                        tx_data  <= 8'h0D;
                        tx_start <= 1;
                        state    <= 5;
                    end
                end else if (rx_data == 8'h0A) begin
                    // Ignorar LF
                end else begin
                    // Guardar en buffer + eco
                    buffer[idx] <= rx_data;
                    idx <= (idx == 31) ? 0 : idx + 1;

                    if (!tx_busy) begin
                        tx_data  <= rx_data;
                        tx_start <= 1;
                    end
                end
            end

            // Máquina de estados envío
            if (sending && !tx_busy && !tx_start) begin
                case (state)
                    1: begin
                        tx_data  <= 8'h0A; // LF
                        tx_start <= 1;
                        state    <= 2; // ahora invertir
                    end
                    2: begin
                        if (!done_sending) begin
                            tx_data  <= buffer[send_idx];
                            tx_start <= 1;
                            if (send_idx == 0) begin
                                done_sending <= 1;
                            end else begin
                                send_idx <= send_idx - 1;
                            end
                        end else begin
                            state <= 3; // cuando termina → CR
                        end
                    end
                    3: begin
                        tx_data  <= 8'h0D; // CR
                        tx_start <= 1;
                        state    <= 4;
                    end
                    4: begin
                        tx_data  <= 8'h0A; // LF
                        tx_start <= 1;
                        state    <= 0;
                        sending      <= 0;
                        idx          <= 0;
                        done_sending <= 0;
                    end
                    5: begin
                        tx_data  <= 8'h0A; // LF para caso vacío
                        tx_start <= 1;
                        state    <= 0;
                    end
                endcase
            end
        end
    end
endmodule
