module uart_rx #(
    parameter CLK_FREQ = 27000000,
    parameter BAUD_RATE = 115200
)(
    input  wire clk,
    input  wire rst_n,
    input  wire rx,
    output reg  [7:0] rx_data,
    output reg        rx_done
);
    localparam DIV_CNT = CLK_FREQ / BAUD_RATE;
    localparam HALF_DIV = DIV_CNT / 2;

    reg [15:0] div_cnt;
    reg [3:0]  bit_cnt;
    reg        rx_reg;
    reg [7:0]  data_buf;
    reg        sampling;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt   <= 0;
            bit_cnt   <= 0;
            sampling  <= 0;
            rx_data   <= 0;
            rx_done   <= 0;
            rx_reg    <= 1;
        end else begin
            rx_reg <= rx;
            rx_done <= 0;
            if (!sampling) begin
                if (!rx_reg) begin
                    sampling <= 1;
                    div_cnt  <= HALF_DIV;
                    bit_cnt  <= 0;
                end
            end else begin
                if (div_cnt == DIV_CNT - 1) begin
                    div_cnt <= 0;
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt >= 1 && bit_cnt <= 8) begin
                        data_buf[bit_cnt-1] <= rx_reg;
                    end else if (bit_cnt == 9) begin
                        rx_data <= data_buf;
                        rx_done <= 1;
                        sampling <= 0;
                    end
                end else begin
                    div_cnt <= div_cnt + 1;
                end
            end
        end
    end
endmodule
