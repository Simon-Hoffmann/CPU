`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Simon Hoffmann
// 
// Create Date: 14.01.2023 13:45:04
// Design Name: 
// Module Name: hoffmann
// Project Name: CPU
// Target Devices: 
// Tool Versions: 
// Description: A 32bit CPU implementation
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - OPcodes added
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//****Opcodes****
`define NOP             6'b000000
`define ADD             6'b000001  //Ra = Rb + Rc
`define SUB             6'b000010  //Ra = Rb - Rc
`define MUL             6'b000011  //Ra = Rb * Rc
`define DIV             6'b000100  //Ra = Rb / Rc
`define AND             6'b000101  //Ra = Rb && Rc
`define OR              6'b000110  //Ra = Rb || Rc
`define XOR             6'b000111  //Ra = Rb ^ Rc
`define NOT             6'b001000
`define LSL             6'b001001  //Ra = Rb << Rc
`define LSR             6'b001010  //Ra = Rb >> Rc

`define MOV             6'b001011  //Ra = Rb
`define LDR             6'b001100  //Ra = [Rb]
`define LDR_ADDR        6'b001101  //Ra = Address
`define STR             6'b001110  //[Rb] = Ra
`define STR_ADDR        6'b001111  //Address = Ra

`define CMP             6'b010000    //Ra vs Rb
`define INC             6'b010001  //Ra++
`define DEC             6'b010010  //Ra--
`define INPUT           6'b010011  //Ra = Input
`define OUTPUT          6'b010100  //Output = Ra
`define PUSH            6'b010101  //Ra on stack
`define POP             6'b010110  //From stack to Ra
`define B               6'b010111  //Unconditional Branch
`define BGT             6'b011000  //Branch on greater than
`define BHI             6'b011001  //Branch on higher than
`define BEQ             6'b011010  //Branch on equal
`define BNE             6'b011011  //Branch on not equal
`define BLT             6'b011100  //Branch on less than
`define BLE             6'b011101  //Branch on less than or equal
`define BGE             6'b011110  //Branch on greater than or equal
`define BVS             6'b011111  //Branch if overflow set
`define BVC             6'b100000  //Branch if overflow clear (no overflow)
`define BMI             6'b100001  //Branch if minus
`define BPL             6'b100010  //Branch if positive or zero
`define BCS             6'b100011  //Branch if carry set (unsigned bigger equal than)
`define BCC             6'b100100  //Branch if carry clear (unsigned smaller than)


//****Control Logic States****
`define FETCH           4'b001
`define EXECUTE         4'b010
`define WRITE_BACK      4'b100

//****Program memory states****
`define INITIALIZE      1'b001
`define READY           1'b010
`define PROGRAM         1'b100


module hoffmann(
    input clk,
    input reset,
    input program,
    input next_program_address,
    input   [31:0] in_port,
    output  [31:0] out_port,
    output reg in_strobe,               //signal indicatior that input has been read
    output reg out_strobe,              //signal indicatior that output is ready to be read
    input  [31:0] instruction_data_bus,
    output [31:0] instruction_address_bus
    );
    
    //****CPU Registers****
    reg[31:0] r[7:0];           //Register Bank R0-R7
    reg[31:0] pc;               //Program counter to Program memory
    reg[31:0] sp;               //Stack pointer to Data memory
    reg Z;                      //Zero flag
    reg C;                      //Carry bit
    reg N;                      //Negative flag
    reg V;                      //Overflow flag 
    
    //****Data memory cache*****
    reg[31:0] mem [1023:0];       //Memory implemented as an array
    
    //****State machine****
    reg [2:0] state;
    reg [2:0] programming_state;
    
    //****Special registers****
    reg[5:0] instruction;                 //current instruction register
    //Registers used in operations 
    reg[2:0] ra;                
    reg[2:0] rb;
    reg[2:0] rc;
    reg FIRST_ARG_MSB;                    //Most significant bit comparator for first register of operation used for flags and
    reg SECOND_ARG_MSB;                   //for second register      
    reg[31:0] result;   
    
    wire[31:0] in_port_sig;
    reg[31:0] out_port_sig;
    
    //****Signals and wiring****
    wire[5:0] opcode;
    wire MODE;
    wire[2:0] sig_ra;
    wire[2:0] sig_rb;
    wire[2:0] sig_rc;
    wire[31:0] argument;
    wire ARG_RESULT_MSB;                //Result of FIRST_ARG_MSB and SECOND_ARG_MSB
    
    //Register bank
    wire[31:0] r0;    
    wire[31:0] r1;  
    wire[31:0] r2;  
    wire[31:0] r3;  
    wire[31:0] r4;  
    wire[31:0] r5;  
    wire[31:0] r6;  
    wire[31:0] r7;
    assign r0 = r[0];   
    assign r1 = r[1]; 
    assign r2 = r[2]; 
    assign r3 = r[3]; 
    assign r4 = r[4]; 
    assign r5 = r[5]; 
    assign r6 = r[6]; 
    assign r7 = r[7];     
    
    assign instruction_data_bus = mem[pc];      //is this right---------------------------------?
    assign instruction_address_bus = pc;
    assign opcode = instruction_data_bus[31:26];
    assign sig_ra = instruction_data_bus[25:21];
    assign sig_rb = instruction_data_bus[20:16];
    assign MODE = instruction_data_bus[15];
    assign sig_rc = instruction_data_bus[14:10];
    assign argument = instruction_data_bus;
    assign ARG_RESULT_MSB = result[15];
    assign in_port_sig = in_port;
    assign out_port = out_port_sig;
    
    //****Procedural Code****
    
    always @(posedge(clk)) begin
        if(reset) begin
            state   <= `FETCH;
            programming_state <= `READY;
            pc      <= 32'b0000_0000_0000_0000_0000_0000_0000_0000;
            sp      <= 32'b0000_0000_0000_0000_0011_1111_1111_1111;
            in_strobe <= 0'b1;
            out_strobe <= 0'b1;
        end
        else if(program) begin
            case(programming_state)
                //********************************************STATE 1****************************************************
                `INITIALIZE: begin
                    pc    <= 32'b0000_0000_0000_0000_0000_0000_0000_0000;
                    state <= `READY;
                end
                //********************************************STATE 2****************************************************
                `READY: begin
                    if(!next_program_address) begin
                        in_strobe = 1;
                        state <= `PROGRAM;
                    end
                end
                //********************************************STATE 3****************************************************
                `PROGRAM: begin
                    in_strobe = 0;
                    if(next_program_address) begin
                        mem[pc] <= in_port;
                        pc <= pc + 1;
                        state <= `READY;
                    end
                 end
            endcase
        end
        else begin
            case(state)
                //********************************************STATE 1****************************************************
                `FETCH: begin
                    in_strobe <= 1'b1;
                    out_strobe <= 1'b1;
                    
                    instruction <= opcode;
                    ra <= sig_ra;
                    rb <= sig_rb;
                    rc <= sig_rc;
                    
                    case(opcode)        //For the opcodes that need the next 16 bits in memory because of an Adrress or constant 
                        `ADD,
                        `SUB,
                        `MUL,
                        `DIV,
                        `AND,
                        `OR,
                        `XOR,
                        `LSL,
                        `LSR,
                        `MOV,
                        `LDR,
                        `LDR_ADDR,
                        `STR,
                        `STR_ADDR,  
                        `CMP,   
                        `B,
                        `BGT,
                        `BHI,
                        `BEQ,
                        `BNE,
                        `BLT,
                        `BLE,
                        `BGE,
                        `BVS,
                        `BVC,
                        `BMI,
                        `BPL,
                        `BCS,
                        `BCC: begin
                            if(MODE) begin
                                pc <= pc + 32'd1; 
                            end
                        end                                
                    endcase
                
                    state <= `EXECUTE;
                 end
                 //********************************************STATE 2****************************************************
                 `EXECUTE: begin
                    case(instruction)
                        `NOP: begin
                            pc <= pc + 32'd1;   
                            state <= `FETCH;
                        end
                        `ADD: begin
                            if(MODE) begin
                                result <= r[ra] + argument;
                                FIRST_ARG_MSB <= r[ra][31];
                                SECOND_ARG_MSB <= argument[31];
                            end else begin
                                result <= r[rb] + r[rc];
                                FIRST_ARG_MSB <= r[rb][31];
                                SECOND_ARG_MSB <= r[rc][31];
                            end
                            state <= `WRITE_BACK;
                        end
                        `SUB: begin
                            if(MODE) begin
                                result <= r[ra] - argument;
                                FIRST_ARG_MSB <= r[ra][31];
                                SECOND_ARG_MSB <= argument[31];
                            end else begin
                                result <= r[rb] - r[rc];
                                FIRST_ARG_MSB <= r[rb][31];
                                SECOND_ARG_MSB <= r[rc][31];
                            end
                            state <= `WRITE_BACK;
                        end
                        `MUL: begin
                            if(MODE) begin
                                result <= r[ra] * argument;
                            end else begin
                                result <= r[rb] * r[rc];
                            end
                                state <= `WRITE_BACK;
                        end
                        `DIV: begin             //not currently implemented
                            if(MODE) begin
                                pc <= pc + 32'd1;   
                            end else begin
                                pc <= pc + 32'd1;   
                            end 
                            state <= `FETCH;
                        end
                        `AND: begin
                            if(MODE) begin
                                result <= r[ra] && argument;
                                FIRST_ARG_MSB <= r[ra][31];
                                SECOND_ARG_MSB <= argument[31];
                            end else begin
                                result <= r[rb] && r[rc];
                                FIRST_ARG_MSB <= r[rb][31];
                                SECOND_ARG_MSB <= r[rc][31];
                            end
                            state <= `WRITE_BACK;
                        end
                        `OR: begin
                            if(MODE) begin
                                result <= r[ra] || argument;
                                FIRST_ARG_MSB <= r[ra][31];
                                SECOND_ARG_MSB <= argument[31];
                            end else begin
                                result <= r[rb] || r[rc];
                                FIRST_ARG_MSB <= r[rb][31];
                                SECOND_ARG_MSB <= r[rc][31];
                            end
                            state <= `WRITE_BACK;
                        end
                        `XOR: begin
                            if(MODE) begin
                                result <= r[ra] ^ argument;
                                FIRST_ARG_MSB <= r[ra][31];
                                SECOND_ARG_MSB <= argument[31];
                            end else begin
                                result <= r[rb] ^ r[rc];
                                FIRST_ARG_MSB <= r[rb][31];
                                SECOND_ARG_MSB <= r[rc][31];
                            end
                            state <= `WRITE_BACK;
                        end
                        `LSL: begin
                            if(MODE) begin
                                result <= r[ra] << argument; 
                                C <= r[ra][31];                  //might need fixinbg
                            end else begin
                                result <= r[rb] << r[rc];
                                C <= r[rb][31];                  //might need fixinbg
                            end
                            state <= `WRITE_BACK;
                        end
                        `LSR: begin
                            if(MODE) begin
                                result <= r[ra] >> argument;
                                C <= r[ra][0];                  //might need fixinbg
                            end else begin
                                result <= r[rb] >> r[rc];
                                C <= r[rb][0];                  //might need fixinbg
                            end

                            state <= `WRITE_BACK;
                        end
                        `MOV: begin
                            if(MODE) begin
                                r[ra] <= argument;
                                Z <= (argument==0)?1:0;
                                N <= argument[31];
                            end else begin
                                r[ra] <= r[rb];
                                Z <= (r[rb]==0)?1:0;
                                N <= r[rb][31];
                            end
                            V <= 0;
                            pc <= pc + 32'd1; 
                            state <= `FETCH;
                        end
                        `LDR: begin
                            if(MODE) begin
                                r[ra] <= mem[rb + argument];
                                V <= 0;
                                N <= mem[rb + argument][31];
                                Z <= (mem[rb + argument]==0)?1:0;
                            end else begin
                                r[ra] <= mem[rb];
                                V <= 0;
                                N <= mem[rb][31];
                                Z <= (mem[rb]==0)?1:0;
                            end
                            state <= `FETCH;
                         end
                        `LDR_ADDR: begin
                            if(MODE) begin
                                r[ra] <= mem[argument];
                                V <= 0;
                                N <= mem[argument][31];
                                Z <= (mem[argument]==0)?1:0;
                            end else begin
                                r[ra] <= mem[rb + r[rc]];
                                V <= 0;
                                N <= mem[rb + r[rc]][31];
                                Z <= (mem[rb + r[rc]]==0)?1:0;
                            end
                            state <= `FETCH;
                        end
                        `STR: begin
                            if(MODE) begin
                                mem[rb + argument] <= r[ra];
                                V <= 0;
                                N <= r[ra][31];
                                Z <= (r[ra]==0)?1:0;
                            end else begin
                                mem[rb] <= r[ra];
                                V <= 0;
                                N <= r[ra][31];
                                Z <= (r[ra]==0)?1:0;
                            end
                            state <= `FETCH;
                         end
                        `STR_ADDR: begin
                            if(MODE) begin
                                mem[argument] <= r[ra];
                                V <= 0;
                                N <= r[ra][31];
                                Z <= (r[ra]==0)?1:0;
                            end else begin
                                mem[rb + r[rc]] <= r[ra];
                                V <= 0;
                                N <= r[ra][31];
                                Z <= (r[ra]==0)?1:0;
                            end
                            state <= `FETCH;
                        end
                        `CMP: begin
                            if(MODE) begin
                                result <= r[ra] - argument;
                                FIRST_ARG_MSB <= r[ra][31];
                                SECOND_ARG_MSB <= argument[31];
                            end else begin
                                result <= r[ra] - r[rb];
                                FIRST_ARG_MSB <= r[ra][31];
                                SECOND_ARG_MSB <= r[rb][31];
                            end
                            pc <= pc + 32'd1; 
                            state <= `WRITE_BACK;
                        end
                        `INC: begin
                            result <= r[ra] + 1;
                            FIRST_ARG_MSB <= r[ra][31];
                            state <= `WRITE_BACK;
                        end
                        `DEC: begin
                            result <= r[ra] - 1;
                            FIRST_ARG_MSB <= r[ra][31];
                            state <= `WRITE_BACK;
                        end
                        `INPUT: begin
                            r[ra] = in_port_sig;
                            state <= `WRITE_BACK;
                        end
                        `OUTPUT: begin
                            out_port_sig = r[ra];
                            state <= `WRITE_BACK;
                        end
                        `PUSH: begin
                            mem[sp] <= r[ra];
                            sp <= sp - 32'd1;
                        end
                        `POP: begin
                            sp <= sp + 32'd1;
                            r[ra] <= mem[sp];
                        end
                        `B: begin
                            pc <= argument;
                            state <= `FETCH;
                        end
                        `BGT: begin
                            if((Z==0) && (N==V))
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BHI: begin
                            if((C==1) && (Z==0))
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BEQ: begin
                            if(Z==1)
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BNE: begin
                            if(Z==0)
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BLT: begin
                            if(N!=V)
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BLE: begin
                            if((Z==1) || (N!=V))
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BGE: begin
                            if(N==V)
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BVS: begin
                            if(V==1)
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BVC: begin
                            if(V==0)
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BMI: begin
                            if(N==1)
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BPL: begin
                            if(N==0)
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BCS: begin
                            if(C==1)
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                        `BCC: begin
                            if(C==0)
                                pc <= argument;
                            else
                                pc <= pc + 1;
                            state <= `FETCH;
                        end
                    endcase
                 end
                 //********************************************STATE 3****************************************************
                 `WRITE_BACK: begin
                    case(instruction)
                        `ADD: begin
                            r[ra] <= result;
                            V <= (FIRST_ARG_MSB&SECOND_ARG_MSB&~ARG_RESULT_MSB)|(~FIRST_ARG_MSB&~SECOND_ARG_MSB&ARG_RESULT_MSB);
                            N <= ARG_RESULT_MSB;
                            Z <= (result==32'd0)?1'b1:1'b0;
                            C <= (FIRST_ARG_MSB&SECOND_ARG_MSB)|(SECOND_ARG_MSB&~ARG_RESULT_MSB)|(~ARG_RESULT_MSB&FIRST_ARG_MSB);
                        end
                        `SUB: begin
                            r[ra] <= result;
                            V <= (FIRST_ARG_MSB&~SECOND_ARG_MSB&~ARG_RESULT_MSB)|(~FIRST_ARG_MSB&SECOND_ARG_MSB&ARG_RESULT_MSB);
                            N <= ARG_RESULT_MSB;
                            Z <= (result==32'd0)?1'b1:1'b0;
                            C <= (~FIRST_ARG_MSB&SECOND_ARG_MSB)|(SECOND_ARG_MSB&ARG_RESULT_MSB)|(ARG_RESULT_MSB&~FIRST_ARG_MSB);
                        end
                        `MUL: begin
                            r[ra] <= result;
                            Z <= (result==32'd0)?1'b1:1'b0;
                            N <= ARG_RESULT_MSB;                  //might need V as well
                        end
                        `AND,
                        `XOR: begin
                            r[ra] <= result;
                            V <= 1'b0;
                            N <= ARG_RESULT_MSB;
                            Z <= (result==32'd0)?1'b1:1'b0;
                        end
                        `LSL: begin
                            r[ra] <= result;
                            V <= ARG_RESULT_MSB ^ C;
                            N <= ARG_RESULT_MSB;
                            Z <= (result==32'd0)?1'b1:1'b0;
                        end
                        `LSR: begin
                            r[ra] <= result;
                            V <= 1'b0 ^ C;
                            N <= 1'b0;
                            Z <= (result==32'd0)?1'b1:1'b0;
                        end
                        `CMP: begin
                            V <= (FIRST_ARG_MSB&~SECOND_ARG_MSB&~ARG_RESULT_MSB)|(~FIRST_ARG_MSB&SECOND_ARG_MSB&ARG_RESULT_MSB);
                            N <= ARG_RESULT_MSB;
                            Z <= (result==32'd0)?1'b1:1'b0;
                            C <= (~FIRST_ARG_MSB&SECOND_ARG_MSB)|(SECOND_ARG_MSB&ARG_RESULT_MSB)|(ARG_RESULT_MSB&~FIRST_ARG_MSB);
                        end
                        `INC: begin
                            r[ra] <= result;
                            V <= ~FIRST_ARG_MSB&ARG_RESULT_MSB;
                            N <= ARG_RESULT_MSB;
                            Z <= (result==0)?1:0;
                        end
                        `DEC: begin
                            r[ra] <= result;
                            V <= ~ARG_RESULT_MSB&FIRST_ARG_MSB;
                            N <= ARG_RESULT_MSB;
                            Z <= (result==0)?1:0;
                        end
                        `INPUT: begin
                            in_strobe = 1;
                        end
                        `INPUT: begin
                            out_strobe = 1;
                        end
                    endcase
                    pc <= pc + 32'd1; 
                    state <= `FETCH;
                 end
                 //Incase an unknown state occurs
                 default: begin
                    state   <= `FETCH;
                    programming_state <= `INITIALIZE;
                    pc      <= 32'b0000_0000_0000_0000_0000_0000_0000_0000;
                    sp      <= 32'b0000_0000_0000_0000_0011_1111_1111_1111;
                    in_strobe <= 0'b1;
                    out_strobe <= 0'b1;
                 end
              endcase
           end
        end
endmodule
