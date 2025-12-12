
/*
module FemtoRV32(
   input        clk,
   input        reset,
   output reg [31:0] mem_addr,
   output [31:0] mem_wdata,
   output reg [3:0]  mem_wmask,
   input  [31:0] mem_rdata,
   output reg    mem_rstrb,
   input         mem_rbusy,
   input         mem_wbusy
);

   parameter RESET_ADDR = 32'h00000000;
   parameter ADDR_WIDTH = 24;

   // Instruction decoding
   wire [4:0] rdId = instr[11:7];
   wire [7:0] funct3Is = 8'b00000001 << instr[14:12];

   wire [31:0] Uimm = {instr[31], instr[30:12], {12{1'b0}}};
   wire [31:0] Iimm = {{21{instr[31]}}, instr[30:20]};
   wire [31:0] Simm = {{21{instr[31]}}, instr[30:25], instr[11:7]};
   wire [31:0] Bimm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
   wire [31:0] Jimm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

   wire isLoad    = (instr[6:2] == 5'b00000);
   wire isALUimm  = (instr[6:2] == 5'b00100);
   wire isStore   = (instr[6:2] == 5'b01000);
   wire isALUreg  = (instr[6:2] == 5'b01100);
   wire isSYSTEM  = (instr[6:2] == 5'b11100);
   wire isJAL     = instr[3];
   wire isJALR    = (instr[6:2] == 5'b11001);
   wire isLUI     = (instr[6:2] == 5'b01101);
   wire isAUIPC   = (instr[6:2] == 5'b00101);
   wire isBranch  = (instr[6:2] == 5'b11000);

   wire isALU = isALUimm | isALUreg;

   // Register file
   reg [31:0] rs1;
   reg [31:0] rs2;
   reg [31:0] registerFile [31:0];

   always @(negedge clk) begin
     registerFile[0] <= 0;
     if (writeBack && (rdId != 0)) begin
       registerFile[rdId] <= writeBackData;
     end
   end

   // ALU
   wire [31:0] aluIn1 = rs1;
   wire [31:0] aluIn2 = (isALUreg | isBranch) ? rs2 : Iimm;

   reg [31:0] aluReg;
   reg [4:0]  aluShamt;
   wire aluBusy = |aluShamt;
   reg aluWr;

   wire [31:0] aluPlus = aluIn1 + aluIn2;
   wire [32:0] aluMinus = {1'b1, ~aluIn2} + {1'b0, aluIn1} + 33'b1;
   wire LT  = (aluIn1[31] ^ aluIn2[31]) ? aluIn1[31] : aluMinus[32];
   wire LTU = aluMinus[32];
   wire EQ  = (aluMinus[31:0] == 0);

   wire [31:0] aluOut =
     (funct3Is[0]  ? (instr[30] & instr[5] ? aluMinus[31:0] : aluPlus) : 32'b0) |
     (funct3Is[2]  ? {31'b0, LT} : 32'b0) |
     (funct3Is[3]  ? {31'b0, LTU} : 32'b0) |
     (funct3Is[4]  ? (aluIn1 ^ aluIn2) : 32'b0) |
     (funct3Is[6]  ? (aluIn1 | aluIn2) : 32'b0) |
     (funct3Is[7]  ? (aluIn1 & aluIn2) : 32'b0) |
     ((funct3Is[1] | funct3Is[5]) ? aluReg : 32'b0);

   always @(negedge clk) begin
      if (!reset) begin
        aluShamt <= 0;
      end else begin
         if(aluWr && (funct3Is[1] | funct3Is[5])) begin
            aluReg <= aluIn1;
            aluShamt <= aluIn2[4:0];
         end
         if (|aluShamt) begin
            aluShamt <= aluShamt - 1;
            aluReg <= funct3Is[1] ? (aluReg << 1) : 
                     {instr[30] & aluReg[31], aluReg[31:1]};
         end
      end
   end

   // Branch predicate
   wire predicate =
        (funct3Is[0] & EQ)  | // BEQ
        (funct3Is[1] & !EQ) | // BNE
        (funct3Is[4] & LT)  | // BLT
        (funct3Is[5] & !LT) | // BGE
        (funct3Is[6] & LTU) | // BLTU
        (funct3Is[7] & !LTU); // BGEU

   // Program counter
   reg [ADDR_WIDTH-1:0] PC;
   reg [31:2] instr;
   wire [ADDR_WIDTH-1:0] PCplus4 = PC + 4;
   wire [ADDR_WIDTH-1:0] PCplusImm = PC + (instr[3] ? Jimm[ADDR_WIDTH-1:0] :
                                          instr[4] ? Uimm[ADDR_WIDTH-1:0] :
                                          Bimm[ADDR_WIDTH-1:0]);
   wire [ADDR_WIDTH-1:0] loadstore_addr = rs1[ADDR_WIDTH-1:0] +
                     (instr[5] ? Simm[ADDR_WIDTH-1:0] : Iimm[ADDR_WIDTH-1:0]);

   // Write back data
   wire [31:0] writeBackData  =
      (isSYSTEM ? cycles : 32'b0) |
      (isLUI    ? Uimm : 32'b0) |
      (isALU    ? aluOut : 32'b0) |
      (isAUIPC  ? PCplusImm : 32'b0) |
      ((isJALR | isJAL) ? PCplus4 : 32'b0) |
      (isLoad   ? LOAD_data : 32'b0);

   // Load/Store logic
   wire mem_byteAccess = (instr[13:12] == 2'b00);
   wire mem_halfwordAccess = (instr[13:12] == 2'b01);
   wire LOAD_sign = !instr[14] & (mem_byteAccess ? LOAD_byte[7] : LOAD_halfword[15]);

   wire [31:0] LOAD_data =
         mem_byteAccess ? {{24{LOAD_sign}}, LOAD_byte} :
         mem_halfwordAccess ? {{16{LOAD_sign}}, LOAD_halfword} :
         mem_rdata;

   wire [15:0] LOAD_halfword = loadstore_addr[1] ? mem_rdata[31:16] : mem_rdata[15:0];
   wire [7:0] LOAD_byte = loadstore_addr[0] ? LOAD_halfword[15:8] : LOAD_halfword[7:0];

   assign mem_wdata[7:0]   = rs2[7:0];
   assign mem_wdata[15:8]  = loadstore_addr[0] ? rs2[7:0]  : rs2[15:8];
   assign mem_wdata[23:16] = loadstore_addr[1] ? rs2[7:0]  : rs2[23:16];
   assign mem_wdata[31:24] = loadstore_addr[0] ? rs2[7:0]  :
                            loadstore_addr[1] ? rs2[15:8] : rs2[31:24];

   wire [3:0] STORE_wmask =
      mem_byteAccess ?
            (loadstore_addr[1] ?
                  (loadstore_addr[0] ? 4'b1000 : 4'b0100) :
                  (loadstore_addr[0] ? 4'b0010 : 4'b0001)) :
      mem_halfwordAccess ?
            (loadstore_addr[1] ? 4'b1100 : 4'b0011) :
            4'b1111;

   // State machine
   localparam FETCH_INSTR = 0;
   localparam WAIT_INSTR = 1;
   localparam EXECUTE = 2;
   localparam WAIT_ALU_OR_MEM = 3;

   reg [3:0] state;
   reg writeBack;

   wire jumpToPCplusImm = isJAL | (isBranch & predicate);
   wire needToWait = isLoad | isStore | (isALU & (funct3Is[1] | funct3Is[5]));

   // Cycle counter
   reg [31:0] cycles;

   always @(posedge clk) begin
      if(!reset) begin
         state <= WAIT_ALU_OR_MEM;
         PC <= RESET_ADDR[ADDR_WIDTH-1:0];
         rs1 <= 0;
         rs2 <= 0;
         aluWr <= 0;
         mem_wmask <= 0;
         cycles <= 0;
         instr <= 0;
      end else begin
         cycles <= cycles + 1;
         
         case(state)
            FETCH_INSTR: begin
               aluWr <= 0;
               mem_wmask <= 0;
               state <= WAIT_INSTR;
            end

            WAIT_INSTR: begin
               aluWr <= 0;
               mem_wmask <= 0;
               if(!mem_rbusy) begin
                  rs1 <= registerFile[mem_rdata[19:15]];
                  rs2 <= registerFile[mem_rdata[24:20]];
                  instr <= mem_rdata[31:2];
                  state <= EXECUTE;
               end
            end

            EXECUTE: begin
               aluWr <= isALU;
               mem_wmask <= {4{isStore}} & STORE_wmask;
               
               if(isJALR) begin
                  PC <= {aluPlus[ADDR_WIDTH-1:1], 1'b0};
               end else begin
                  PC <= jumpToPCplusImm ? PCplusImm : PCplus4;
               end
               
               state <= needToWait ? WAIT_ALU_OR_MEM : FETCH_INSTR;
            end

            WAIT_ALU_OR_MEM: begin
               aluWr <= 0;
               mem_wmask <= 0;
               if(!mem_rbusy & !mem_wbusy & !aluBusy) begin
                  state <= FETCH_INSTR;
               end
            end

            default: state <= WAIT_INSTR;
         endcase
      end
   end

   // Combinatorial outputs
   always @(*) begin
      if(!reset) begin
         mem_rstrb = 0;
         writeBack = 0;
      end else begin
         case(state)
            FETCH_INSTR: begin
               mem_rstrb = 1;
               writeBack = 0;
            end
            WAIT_INSTR: begin
               mem_rstrb = 0;
               writeBack = 0;
            end
            EXECUTE: begin
               mem_rstrb = isLoad;
               writeBack = ~(isBranch | isStore);
            end
            WAIT_ALU_OR_MEM: begin
               mem_rstrb = 0;
               writeBack = ~(isBranch | isStore);
            end
            default: begin
               mem_rstrb = 0;
               writeBack = 0;
            end
         endcase
      end
   end

   // Memory address
   always @(*) begin
      if(!reset) begin
         mem_addr = 0;
      end else begin
         if ((state == WAIT_INSTR) || (state == FETCH_INSTR)) begin
            mem_addr = PC;
         end else begin
            mem_addr = loadstore_addr;
         end
      end
   end

endmodule
*/

module FemtoRV32(
   input        clk,
   input        reset,
   output reg [31:0] mem_addr,
   output [31:0] mem_wdata,
   output reg [3:0]  mem_wmask,
   input  [31:0] mem_rdata,
   output reg    mem_rstrb,
   input         mem_rbusy,
   input         mem_wbusy
);

   parameter RESET_ADDR = 32'h00000000;
   parameter ADDR_WIDTH = 24;

   // Instruction decoding
   wire [4:0] rdId = instr[11:7];
   wire [7:0] funct3Is = 8'b00000001 << instr[14:12];

   wire [31:0] Uimm = {instr[31], instr[30:12], {12{1'b0}}};
   wire [31:0] Iimm = {{21{instr[31]}}, instr[30:20]};
   wire [31:0] Simm = {{21{instr[31]}}, instr[30:25], instr[11:7]};
   wire [31:0] Bimm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
   wire [31:0] Jimm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

   wire isLoad    = (instr[6:2] == 5'b00000);
   wire isALUimm  = (instr[6:2] == 5'b00100);
   wire isStore   = (instr[6:2] == 5'b01000);
   wire isALUreg  = (instr[6:2] == 5'b01100);
   wire isSYSTEM  = (instr[6:2] == 5'b11100);
   wire isJAL     = instr[3];
   wire isJALR    = (instr[6:2] == 5'b11001);
   wire isLUI     = (instr[6:2] == 5'b01101);
   wire isAUIPC   = (instr[6:2] == 5'b00101);
   wire isBranch  = (instr[6:2] == 5'b11000);

   wire isALU = isALUimm | isALUreg;

   // Register file
   reg [31:0] rs1;
   reg [31:0] rs2;
   reg [31:0] registerFile [31:0];

   always @(negedge clk) begin
     registerFile[0] <= 0;
     if (writeBack && (rdId != 0)) begin
       registerFile[rdId] <= writeBackData;
     end
   end

   // ALU
   wire [31:0] aluIn1 = rs1;
   wire [31:0] aluIn2 = (isALUreg | isBranch) ? rs2 : Iimm;

   reg [31:0] aluReg;
   reg [4:0]  aluShamt;
   wire aluBusy = |aluShamt;
   reg aluWr;

   wire [31:0] aluPlus = aluIn1 + aluIn2;
   wire [32:0] aluMinus = {1'b1, ~aluIn2} + {1'b0, aluIn1} + 33'b1;
   wire LT  = (aluIn1[31] ^ aluIn2[31]) ? aluIn1[31] : aluMinus[32];
   wire LTU = aluMinus[32];
   wire EQ  = (aluMinus[31:0] == 0);

   wire [31:0] aluOut =
     (funct3Is[0]  ? (instr[30] & instr[5] ? aluMinus[31:0] : aluPlus) : 32'b0) |
     (funct3Is[2]  ? {31'b0, LT} : 32'b0) |
     (funct3Is[3]  ? {31'b0, LTU} : 32'b0) |
     (funct3Is[4]  ? (aluIn1 ^ aluIn2) : 32'b0) |
     (funct3Is[6]  ? (aluIn1 | aluIn2) : 32'b0) |
     (funct3Is[7]  ? (aluIn1 & aluIn2) : 32'b0) |
     ((funct3Is[1] | funct3Is[5]) ? aluReg : 32'b0);

   always @(negedge clk) begin
      if (!reset) begin
        aluShamt <= 0;
      end else begin
         if(aluWr && (funct3Is[1] | funct3Is[5])) begin
            aluReg <= aluIn1;
            aluShamt <= aluIn2[4:0];
         end
         if (|aluShamt) begin
            aluShamt <= aluShamt - 1;
            aluReg <= funct3Is[1] ? (aluReg << 1) : 
                     {instr[30] & aluReg[31], aluReg[31:1]};
         end
      end
   end

   // Branch predicate
   wire predicate =
        (funct3Is[0] & EQ)  | // BEQ
        (funct3Is[1] & !EQ) | // BNE
        (funct3Is[4] & LT)  | // BLT
        (funct3Is[5] & !LT) | // BGE
        (funct3Is[6] & LTU) | // BLTU
        (funct3Is[7] & !LTU); // BGEU

   // Program counter
   reg [ADDR_WIDTH-1:0] PC;
   reg [31:2] instr;
   wire [ADDR_WIDTH-1:0] PCplus4 = PC + 4;
   wire [ADDR_WIDTH-1:0] PCplusImm = PC + (instr[3] ? Jimm[ADDR_WIDTH-1:0] :
                                          instr[4] ? Uimm[ADDR_WIDTH-1:0] :
                                          Bimm[ADDR_WIDTH-1:0]);
   wire [ADDR_WIDTH-1:0] loadstore_addr = rs1[ADDR_WIDTH-1:0] +
                     (instr[5] ? Simm[ADDR_WIDTH-1:0] : Iimm[ADDR_WIDTH-1:0]);

   // Write back data
   wire [31:0] writeBackData  =
      (isSYSTEM ? cycles : 32'b0) |
      (isLUI    ? Uimm : 32'b0) |
      (isALU    ? aluOut : 32'b0) |
      (isAUIPC  ? PCplusImm : 32'b0) |
      ((isJALR | isJAL) ? PCplus4 : 32'b0) |
      (isLoad   ? LOAD_data : 32'b0);

   // Load/Store logic
   wire mem_byteAccess = (instr[13:12] == 2'b00);
   wire mem_halfwordAccess = (instr[13:12] == 2'b01);
   wire LOAD_sign = !instr[14] & (mem_byteAccess ? LOAD_byte[7] : LOAD_halfword[15]);

   wire [31:0] LOAD_data =
         mem_byteAccess ? {{24{LOAD_sign}}, LOAD_byte} :
         mem_halfwordAccess ? {{16{LOAD_sign}}, LOAD_halfword} :
         mem_rdata;

   wire [15:0] LOAD_halfword = loadstore_addr[1] ? mem_rdata[31:16] : mem_rdata[15:0];
   wire [7:0] LOAD_byte = loadstore_addr[0] ? LOAD_halfword[15:8] : LOAD_halfword[7:0];

   assign mem_wdata[7:0]   = rs2[7:0];
   assign mem_wdata[15:8]  = loadstore_addr[0] ? rs2[7:0]  : rs2[15:8];
   assign mem_wdata[23:16] = loadstore_addr[1] ? rs2[7:0]  : rs2[23:16];
   assign mem_wdata[31:24] = loadstore_addr[0] ? rs2[7:0]  :
                            loadstore_addr[1] ? rs2[15:8] : rs2[31:24];

   wire [3:0] STORE_wmask =
      mem_byteAccess ?
            (loadstore_addr[1] ?
                  (loadstore_addr[0] ? 4'b1000 : 4'b0100) :
                  (loadstore_addr[0] ? 4'b0010 : 4'b0001)) :
      mem_halfwordAccess ?
            (loadstore_addr[1] ? 4'b1100 : 4'b0011) :
            4'b1111;

   // State machine - MODIFICADO: NO esperar mem_wbusy para Store
   localparam FETCH_INSTR = 0;
   localparam WAIT_INSTR = 1;
   localparam EXECUTE = 2;
   localparam WAIT_ALU_OR_MEM = 3;

   reg [3:0] state;
   reg writeBack;

   wire jumpToPCplusImm = isJAL | (isBranch & predicate);
   wire needToWait = isLoad | (isALU & (funct3Is[1] | funct3Is[5]));  // ¡Store NO espera!

   // Cycle counter
   reg [31:0] cycles;

   always @(posedge clk) begin
      if(!reset) begin
         state <= WAIT_ALU_OR_MEM;
         PC <= RESET_ADDR[ADDR_WIDTH-1:0];
         rs1 <= 0;
         rs2 <= 0;
         aluWr <= 0;
         mem_wmask <= 0;
         cycles <= 0;
         instr <= 0;
      end else begin
         cycles <= cycles + 1;
         
         case(state)
            FETCH_INSTR: begin
               aluWr <= 0;
               mem_wmask <= 0;
               state <= WAIT_INSTR;
            end

            WAIT_INSTR: begin
               aluWr <= 0;
               mem_wmask <= 0;
               if(!mem_rbusy) begin
                  rs1 <= registerFile[mem_rdata[19:15]];
                  rs2 <= registerFile[mem_rdata[24:20]];
                  instr <= mem_rdata[31:2];
                  state <= EXECUTE;
               end
            end

            EXECUTE: begin
               aluWr <= isALU;
               mem_wmask <= {4{isStore}} & STORE_wmask;
               
               if(isJALR) begin
                  PC <= {aluPlus[ADDR_WIDTH-1:1], 1'b0};
               end else begin
                  PC <= jumpToPCplusImm ? PCplusImm : PCplus4;
               end
               
               state <= needToWait ? WAIT_ALU_OR_MEM : FETCH_INSTR;
            end

            WAIT_ALU_OR_MEM: begin
               aluWr <= 0;
               mem_wmask <= 0;
               if(!mem_rbusy & !aluBusy) begin  // ¡NO verificar mem_wbusy!
                  state <= FETCH_INSTR;
               end
            end

            default: state <= WAIT_INSTR;
         endcase
      end
   end

   // Combinatorial outputs
   always @(*) begin
      if(!reset) begin
         mem_rstrb = 0;
         writeBack = 0;
      end else begin
         case(state)
            FETCH_INSTR: begin
               mem_rstrb = 1;
               writeBack = 0;
            end
            WAIT_INSTR: begin
               mem_rstrb = 0;
               writeBack = 0;
            end
            EXECUTE: begin
               mem_rstrb = isLoad;
               writeBack = ~(isBranch | isStore);
            end
            WAIT_ALU_OR_MEM: begin
               mem_rstrb = 0;
               writeBack = ~(isBranch | isStore);
            end
            default: begin
               mem_rstrb = 0;
               writeBack = 0;
            end
         endcase
      end
   end

   // Memory address
   always @(*) begin
      if(!reset) begin
         mem_addr = 0;
      end else begin
         if ((state == WAIT_INSTR) || (state == FETCH_INSTR)) begin
            mem_addr = PC;
         end else begin
            mem_addr = loadstore_addr;
         end
      end
   end

endmodule
