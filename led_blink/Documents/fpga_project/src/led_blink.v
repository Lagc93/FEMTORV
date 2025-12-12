module led_blink_all (
    input clk,        // Reloj de 27 MHz
    output reg led2,
    output reg led3,
    output reg led4,
    output reg led5
);

    reg [23:0] counter2;   // LED2 → 1 Hz
    reg [23:0] counter3;   // LED3 → 2 Hz
    reg [23:0] counter4;   // LED4 → 4 Hz
    reg [23:0] counter5;   // LED5 → 0.5 Hz

    always @(posedge clk) begin
        // LED2 - 1 Hz
        counter2 <= counter2 + 1;
        if (counter2 == 24'd13500000) begin  // 0.5 s
            counter2 <= 0;
            led2 <= ~led2;
        end

        // LED3 - 2 Hz
        counter3 <= counter3 + 1;
        if (counter3 == 24'd6750000) begin   // 0.25 s
            counter3 <= 0;
            led3 <= ~led3;
        end

        // LED4 - 4 Hz
        counter4 <= counter4 + 1;
        if (counter4 == 24'd3375000) begin   // 0.125 s
            counter4 <= 0;
            led4 <= ~led4;
        end

        // LED5 - 0.5 Hz
        counter5 <= counter5 + 1;
        if (counter5 == 24'd27000000) begin  // 1 s
            counter5 <= 0;
            led5 <= ~led5;
        end
    end
endmodule
