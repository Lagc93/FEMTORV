// led_counter.v
// Contador binario en los 4 LEDs del Dock
// Cada ~0.5 segundos incrementa el valor (ajustable con el parámetro)

module led_counter (
    input  wire Clock,        // Reloj de 27 MHz de la placa
    output reg [3:0] IO_voltage  // LED2..LED5
);

    // Ajusta la velocidad del contador:
    // Con 27 MHz, para ~0.5s necesitas ~13_500_000 ciclos
    parameter COUNT_MAX = 13_500_000;

    reg [23:0] counter = 24'd0;  // contador de ciclos
    reg [3:0]  led_reg = 4'b0000; // registro para los LEDs

    always @(posedge Clock) begin
        if (counter < COUNT_MAX) begin
            counter <= counter + 1'b1;
        end else begin
            counter <= 24'd0;
            led_reg <= led_reg + 1'b1;  // Incrementa el contador de 4 bits
        end
    end

    // Asignar a LEDs
    always @(*) begin
        IO_voltage = led_reg;
    end

endmodule
