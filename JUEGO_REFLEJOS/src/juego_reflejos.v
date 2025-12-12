module juego_reflejos(
    input Clock,                // Reloj de 27 MHz
    input [3:0] BTN,            // Botones S0-S3
    output reg [3:0] LED        // LEDs N14, P15, M15, L14
);

    reg [15:0] lfsr = 16'hACE1;  // Registro inicial para LFSR
    reg [1:0] current_led;       // LED encendido actual

    // Generador LFSR para pseudoaleatoriedad
    always @(posedge Clock) begin
        lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
        LED <= 4'b0000;
        LED[current_led] <= 1'b1;  // Enciende solo el LED elegido

        // Si el jugador acierta → genera nuevo LED aleatorio
        if (BTN[current_led] == 1'b0) begin
            current_led <= lfsr[1:0]; // Toma 2 bits pseudoaleatorios
        end
    end
endmodule
