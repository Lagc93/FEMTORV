module peripheral_mult (
   input clk,
   input reset,
   input [15:0] d_in,
   input cs,
   input [4:0] addr,
   input rd,
   input wr,
   output reg [31:0] d_out
);

   reg [1:0] s;
   reg [15:0] op_A;
   reg [15:0] op_B;
   wire mult_done;
   wire [31:0] mult_result;
   
   // Address decoder
   always @(*) begin
      case (addr)
         5'h00: s = (cs) ? 2'b01 : 2'b00; // op_A
         5'h04: s = (cs) ? 2'b10 : 2'b00; // op_B
         5'h08: s = (cs) ? 2'b11 : 2'b00; // status/result
         default: s = 2'b00;
      endcase
   end

   // Input registers
   always @(posedge clk) begin
      if(reset) begin
         op_A <= 0;
         op_B <= 0;
      end else begin
         if (s[0] & wr) begin
            op_A <= d_in;
         end
         if (s[1] & wr) begin
            op_B <= d_in;
         end
      end
   end

   // Output registers
   always @(*) begin
      case (s)
         2'b01: d_out = {16'b0, op_A};     // Read op_A
         2'b10: d_out = {16'b0, op_B};     // Read op_B
         2'b11: d_out = {31'b0, mult_done}; // Read status
         default: d_out = 32'b0;
      endcase
      
      // When reading result address, show multiplication result
      if (s[1] && rd) begin
         d_out = mult_result;
      end
   end

   // Multiplier instance
   mult multiplier_inst (
      .reset(reset),
      .clk(clk),
      .init((s[0] & wr) | (s[1] & wr)), // Init when writing operands
      .done(mult_done),
      .result(mult_result),
      .op_A(op_A),
      .op_B(op_B)
   );

endmodule