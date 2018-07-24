`default_nettype none

// numero de bits da instrucao
// Este parametro e global porque nao faz sentido mudar-lo,
// ja que queremos preservar a compatibilidade do processador com a toolchain RISC-V. 
parameter NINSTR_BITS = 32;

// Interface para espiar para dentro do processador
interface zoi #(parameter NBITS = 8, NREGS = 32) ();
  logic [NBITS-1:0] pc;
  logic [NINSTR_BITS-1:0] instruction;
  logic [NBITS-1:0] registrador [0:NREGS-1];
  logic [NBITS-1:0] SrcA, SrcB;
  logic [NBITS-1:0] ALUResult, Result;
  logic [NBITS-1:0] WriteData, ReadData;
  logic MemWrite, Branch, MemtoReg, RegWrite;
endinterface

`include "cpu.sv"
`include "lcd.sv"
`include "vJTAG_interface.sv"

module DE0_Nano(

//////////// clock //////////
input logic                         CLOCK_50,

//////////// LED //////////
output logic             [7:0]      LED,

//////////// KEY //////////
input logic              [1:0]      KEY,

//////////// SW //////////
input logic              [3:0]      SW,

//////////// SDRAM //////////
output logic            [12:0]      DRAM_ADDR,
output logic             [1:0]      DRAM_BA,
output logic                        DRAM_CAS_N,
output logic                        DRAM_CKE,
output logic                        DRAM_CLK,
output logic                        DRAM_CS_N,
inout                   [15:0]      DRAM_DQ,
output logic             [1:0]      DRAM_DQM,
output logic                        DRAM_RAS_N,
output logic                        DRAM_WE_N,

//////////// EPCS //////////
output logic                        EPCS_ASDO,
input  logic                        EPCS_DATA0,
output logic                        EPCS_DCLK,
output logic                        EPCS_NCSO,

//////////// Accelerometer and EEPROM //////////
output logic                        G_SENSOR_CS_N,
input  logic                        G_SENSOR_INT,
output logic                        I2C_SCLK,
inout  logic                        I2C_SDAT,

//////////// ADC //////////
output logic                        ADC_CS_N,
output logic                        ADC_SADDR,
output logic                        ADC_SCLK,
input logic                         ADC_SDAT,

//////////// 2x13 GPIO Header //////////
inout                   [12:0]      GPIO_2,
input logic              [2:0]      GPIO_2_IN,

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout                   [33:0]      GPIO_0,
input logic              [1:0]      GPIO_0_IN,

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout                   [33:0]      GPIO_1,
input logic              [1:0]      GPIO_1_IN
);


logic [7:0] SWI; // chaves pretas
always_comb SWI <= {GPIO_0[4],GPIO_0[6],GPIO_0[8],GPIO_0[5],GPIO_0[21],GPIO_0[23],GPIO_0[29],GPIO_0[18]};

logic [7:0] SEG; // display de 7 segmentos com ponto
always_comb {GPIO_0[27],GPIO_0[3],GPIO_0[1],GPIO_0[0],GPIO_0[2],GPIO_0[25],GPIO_0[31],GPIO_0[33]} <= SEG;


// clock lento //
localparam CLOCK_DIVIDER = 25;
logic [CLOCK_DIVIDER:0] clock_50_counter;
logic pause; // para clock
logic clock; // clock de sistema lento 

always_comb begin
  pause <= SWI[7];
  clock <= clock_50_counter[CLOCK_DIVIDER]; 
  SEG[7] <= clock;
end

always_ff @(posedge CLOCK_50)
  if (~pause) clock_50_counter <= clock_50_counter + 1;

// reset //
logic reset;
always_comb reset <= SWI[6];

// a CPU
localparam NBITS = 8;   // numero de bits dos barramentos de enderecos e dados
localparam NREGS = 32;  // numero de registradores no banco de registradores
zoi #(.NBITS(NBITS)) z();
cpu #(.NBITS(NBITS), .NREGS(NREGS)) (.*);

// dispositivos de entrada e saida
localparam NIO_BITS = 5; // nao tem nem SWI nem LED suficiente para ter NIO_BITS=NBITS
logic [NBITS-1:0] entrada, saida;
always_comb begin
  entrada[NIO_BITS-1:0] <= SWI[NIO_BITS-1:0];
  LED[NIO_BITS-1:0] <= saida[NIO_BITS-1:0];
end

logic LCD_RS, LCD_E;
logic [7:0] LCD_D;
always_comb begin
   GPIO_1[21] <= 0; // RW wired to GND on connector
   GPIO_1[19] <= LCD_RS;
   GPIO_1[15] <= LCD_E;
   GPIO_1[13] <= LCD_D[0];
   GPIO_1[11] <= LCD_D[1];
   GPIO_1[ 9] <= LCD_D[2];
   GPIO_1[31] <= LCD_D[3];
   GPIO_1[ 7] <= LCD_D[4];
   GPIO_1[ 5] <= LCD_D[5];
   GPIO_1[ 3] <= LCD_D[6];
   GPIO_1[ 1] <= LCD_D[7];
   GPIO_1[ 0] <= 1; // backlight anode
end

lcd (.clk(CLOCK_50), .reset(~KEY[0]),
     .pc(z.pc), .instruct(z.instruction), .d1(z.WriteData),
     .d2a(z.SrcA), .d2b(z.SrcB), .d2c(z.ALUResult), .d2d(z.Result), .d2e(z.ReadData),
     .f1(z.MemWrite), .f2(z.Branch), .f3(z.MemtoReg), .f4(z.RegWrite),
     .*);
vJTAG_interface(z.registrador);

endmodule

