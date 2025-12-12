
// firmware_test_simple.v - TEST MÍNIMO
module firmware_flash(
    input wire [7:0] address,
    output reg [31:0] data
);
    
    reg [31:0] memory [0:255];
    
    integer i;
    
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            memory[i] = 32'h00000013;
        end
        
        // PROGRAMA: Solo escribe en UART una vez
        
        // 1. Escribir 'X' en UART
        memory[0] = 32'h004002B7;   // lui t0, 0x400
        
        // Escribir dato
        memory[1] = 32'h05800513;   // li a0, 'X' (88)
        memory[2] = 32'h00A2A423;   // sw a0, 8(t0)   # TX_DATA @ 0x08
        
        // Iniciar transmisión (bit 3 = 1)
        memory[3] = 32'h00800593;   // li a1, 8
        memory[4] = 32'h00B2A823;   // sw a1, 16(t0)  # CTRL @ 0x10
        
        // 2. Encender LED (bit 2 = 1)
        memory[5] = 32'h00400593;   // li a1, 4
        memory[6] = 32'h00B2A823;   // sw a1, 16(t0)
        
        // 3. Bucle infinito
        memory[7] = 32'h0000006F;   // inf: j inf
    end
    
    always @(*) begin
        data = memory[address];
    end
    
endmodule