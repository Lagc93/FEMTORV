module led_blink (
    input clk,
    output reg led
);

    reg [25:0] counter;   // ahora de 26 bits

    always @(posedge clk) begin
        counter <= counter + 1;
        if (counter == 26'd12000000) begin
            counter <= 0;
            led <= ~led;
        end
    end
endmodule
