`default_nettype none

/************************************************************/
// Datapath eh composto por:
// - Register FIle
// - ULA
/************************************************************/

module datapath #(parameter NBITS = 8, NREGS=32, WIDTH_ALUF=4) (
  input logic clock, reset,

  // Controller
  input logic  [$clog2(NREGS)-1:0] RS2,  RS1, RD,
  input logic signed [NBITS-1:0] IMM,
  input logic [WIDTH_ALUF-1:0] ALUControl,
  output logic Zero, Neg, Carry,
  input logic ALUSrc,
  input logic MemtoReg,
  input logic RegWrite,
  input logic link,  // valida pclink para ser salvo no registrador RD 
  input logic [NBITS-1:0] pclink, // valor proveniente do PC a ser salvo em registrador RD
  output logic [NBITS-1:0] PCReg, // registrador RS1 (SrcA) volta para o PC

  // Memoria ou cache
  output logic [NBITS-1:2] Address, 
  output logic [NBITS-1:0] WriteData,
  input logic [NBITS-1:0] ReadData,

  zoi z);

logic [NBITS-1:0] SrcA, SrcB;
logic signed [NBITS-1:0] SrcAs, SrcBs;  // SrcA e SrcB vistas como numeros inteiros
logic [NBITS-1:0] SUBResult;  // para poder recuperar o vai-um
logic [NBITS-1:0] ALUResult, Result;

// ****** banco de registradores

logic [NBITS-1:0] registrador [0:NREGS-1];

always_ff @(posedge clock) begin
  if (reset)
    for (int i=0; i < NREGS; i = i + 1)
      registrador[i] <= 0;
  else if( RD != 0 && RegWrite)
         registrador[RD] <= Result;

end

always_comb begin // barramentos indo para a ULA
  SrcA <= registrador[RS1];
  if(ALUSrc) SrcB <= IMM;
  else SrcB <= registrador[RS2];
  SrcAs <= SrcA;
  SrcBs <= SrcB;
  Zero <= (ALUResult == 0);
  Neg <= (ALUResult < 0);

  unique case(ALUControl)
   ADD: ALUResult <= SrcA + SrcB;

   SUB: ALUResult <= SrcAs - SrcBs;

   SUBU: ALUResult <= SrcA - SrcB;

   OR: ALUResult <= SrcA | SrcB;

   AND: ALUResult <= SrcA & SrcB;

   XOR: ALUResult <= SrcA ^ SrcB;

   SLL: ALUResult <= SrcA << SrcB;

   SRA: ALUResult <= SrcA >>> SrcB;

   SRL: ALUResult <= SrcA >> SrcB;

   SLT: ALUResult <= SrcAs < SrcBs;

   SLTU: ALUResult <= SrcA & SrcB;

  endcase

end

// ****** ULA

always_comb begin // barramentos conectados na saida da ULA
  Address <= ALUResult[7:2];
  WriteData <= registrador[RS2];
  if (MentoReg) Result <= ReadData;
  else Result <= ALUResult;
  
end

// a zoiada
always_comb begin
  z.SrcA <= SrcA;
  z.SrcB <= SrcB;
  z.ALUResult <= ALUResult;
  z.Result <= Result;
  z.WriteData <= WriteData;
  z.ReadData <= ReadData;
  z.MemtoReg <= MemtoReg;
  z.RegWrite <= RegWrite;
  z.registrador <= registrador;
end

endmodule
