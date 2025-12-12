module led_buttons (
    input  wire key_C7,    // botón en pin C7
    input  wire key_D7,    // botón en pin D7
    input  wire key_T2,    // botón en pin T2
    input  wire key_T3,    // botón en pin T3
    output reg  [3:0] IO_voltage  // LED2..LED5
);

    always @(*) begin
        // Default: todos apagados
        IO_voltage = 4'b0000;

        // Mapeo directo (invirtiendo porque botón = 0 cuando se presiona)
        IO_voltage[0] = ~key_D7; // D7 -> LED2 (N14)
        IO_voltage[1] = ~key_C7; // C7 -> LED3 (N16)
        IO_voltage[2] = ~key_T2; // T2 -> LED4 (L14)
        IO_voltage[3] = ~key_T3; // T3 -> LED5 (L16)
    end

endmodule

