module mult (
   input reset,
   input clk,
   input init,
   output reg done,
   output reg [31:0] result,
   input [15:0] op_A,
   input [15:0] op_B
);

   parameter START  = 3'b000;
   parameter CHECK  = 3'b001;
   parameter SHIFT  = 3'b010;
   parameter ADD    = 3'b011;
   parameter END    = 3'b100;
   parameter START1 = 3'b101;

   reg [2:0] state;
   reg [15:0] A;
   reg [15:0] B;
   reg [10:0] count;

   always @(posedge clk) begin
      if (reset) begin
         done <= 0;
         result <= 0;
         state <= START;
         count <= 0;
      end else begin
         case(state)
            START: begin
               count <= 0;
               done <= 0;
               result <= 0;
               if(init) begin
                  state <= START1;
               end
            end

            START1: begin
               A <= op_A;
               B <= op_B;
               done <= 0;
               result <= 0;
               state <= CHECK;
            end

            CHECK: begin
               if(B[0]) begin
                  state <= ADD;
               end else begin
                  state <= SHIFT;
               end
            end

            SHIFT: begin
               B <= B >> 1;
               A <= A << 1;
               done <= 0;
               if(B == 0) begin
                  state <= END;
               end else begin
                  state <= CHECK;
               end
            end

            ADD: begin
               result <= result + {16'b0, A};
               done <= 0;
               state <= SHIFT;
            end

            END: begin
               done <= 1;
               count <= count + 1;
               if(count > 10) begin
                  state <= START;
               end
            end

            default: state <= START;
         endcase
      end
   end

endmodule