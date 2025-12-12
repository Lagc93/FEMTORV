// led_pwm.v
// Control de brillo de LED con PWM usando switches
// SW0-SW3 seleccionan duty cycle (0-15)

module led_pwm (
    input  wire Clock,         // Reloj de 27 MHz
    input  wire [3:0] SW,      // 4 switches
    output reg LED             // LED controlado por PWM
);

    // Contador para generar PWM
    reg [7:0] pwm_counter = 8'd0;  // 8 bits → 256 pasos de resolución

    always @(posedge Clock) begin
        pwm_counter <= pwm_counter + 1'b1;
    end

    always @(*) begin
        // duty = switches * 16 (expande rango de 0 a 240 aprox)
        if (pwm_counter < (SW * 16))
            LED = 1'b1;
        else
            LED = 1'b0;
    end

endmodule
