// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Additional contributions by:                                               //
//                 Michael Gautschi - gautschi@iis.ee.ethz.ch                 //
//                                                                            //
// Design Name:    Compressed instruction decoder                             //
// Project Name:   RI5CY                                                      //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Decodes RISC-V compressed instructions into their RV32     //
//                 equivalent. This module is fully combinatorial.            //
//                 Float extensions added                                     //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

module cv32e40x_compressed_decoder import cv32e40x_pkg::*;
(
  input  inst_resp_t  instr_i,
  output inst_resp_t  instr_o,
  output logic        is_compressed_o,
  output logic        illegal_instr_o,
  output logic        illegal_uncompressed_o,
  output logic        rs1_enable,
  output logic        rs2_enable,
  input  ctrl_fsm_t   ctrl_fsm_i
);

  import cv32e40x_pkg::*;

  logic [31:0] instr;

  assign instr = instr_i.bus_resp.rdata;
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //   ____                                                 _   ____                     _            //
  //  / ___|___  _ __ ___  _ __  _ __ ___  ___ ___  ___  __| | |  _ \  ___  ___ ___   __| | ___ _ __  //
  // | |   / _ \| '_ ` _ \| '_ \| '__/ _ \/ __/ __|/ _ \/ _` | | | | |/ _ \/ __/ _ \ / _` |/ _ \ '__| //
  // | |__| (_) | | | | | | |_) | | |  __/\__ \__ \  __/ (_| | | |_| |  __/ (_| (_) | (_| |  __/ |    //
  //  \____\___/|_| |_| |_| .__/|_|  \___||___/___/\___|\__,_| |____/ \___|\___\___/ \__,_|\___|_|    //
  //                      |_|                                                                         //
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  always_comb
  begin
    illegal_instr_o  = 1'b0;
    instr_o          = instr_i;
    rs1_enable       = 1'b0;
    rs2_enable       = 1'b0;

    unique case (instr[1:0])
      // C0
      2'b00: begin
        unique case (instr[15:13])
          3'b000: begin
            // c.addi4spn -> addi rd', x2, imm
            instr_o.bus_resp.rdata = {2'b0, instr[10:7], instr[12:11], instr[5], instr[6], 2'b00, 5'h02, 3'b000, 2'b01, instr[4:2], OPCODE_OPIMM};
            if (instr[12:5] == 8'b0) begin
              illegal_instr_o = 1'b1;
              rs1_enable = 1'b1;
              rs2_enable = 1'b1;
            end else begin
              rs1_enable = 1'b1;
            end
          end

          3'b010: begin
            // c.lw -> lw rd', imm(rs1')
            instr_o.bus_resp.rdata = {5'b0, instr[5], instr[12:10], instr[6], 2'b00, 2'b01, instr[9:7], 3'b010, 2'b01, instr[4:2], OPCODE_LOAD};
            rs1_enable = 1'b1;
          end

          3'b110: begin
            // c.sw -> sw rs2', imm(rs1')
            instr_o.bus_resp.rdata = {5'b0, instr[5], instr[12], 2'b01, instr[4:2], 2'b01, instr[9:7], 3'b010, instr[11:10], instr[6], 2'b00, OPCODE_STORE};
            rs1_enable = 1'b1;
            rs2_enable = 1'b1;
          end

          3'b001,       // c.fld -> fld rd', imm(rs1')
          3'b011,       // c.flw -> flw rd', imm(rs1')
          3'b101,       // c.fsd -> fsd rs2', imm(rs1')
          3'b111: begin // c.fsw -> fsw rs2', imm(rs1')
            illegal_instr_o = 1'b1;
            rs1_enable = 1'b1;
            rs2_enable = 1'b1;
          end
          default: begin
            illegal_instr_o = 1'b1;
            rs1_enable = 1'b1;
            rs2_enable = 1'b1;
          end
        endcase
      end


      // C1
      2'b01: begin
        unique case (instr[15:13])
          3'b000: begin
            // c.addi -> addi rd, rd, nzimm
            // c.nop
            instr_o.bus_resp.rdata = {{6 {instr[12]}}, instr[12], instr[6:2], instr[11:7], 3'b0, instr[11:7], OPCODE_OPIMM};
            rs1_enable = 1'b1;
          end

          3'b001, 3'b101: begin
            // 001: c.jal -> jal x1, imm
            // 101: c.j   -> jal x0, imm
            instr_o.bus_resp.rdata = {instr[12], instr[8], instr[10:9], instr[6], instr[7], instr[2], instr[11], instr[5:3], {9 {instr[12]}}, 4'b0, ~instr[15], OPCODE_JAL};
          end

          3'b010: begin
            if (instr[11:7] == 5'b0) begin
              // Hint -> addi x0, x0, nzimm
              instr_o.bus_resp.rdata = {{6 {instr[12]}}, instr[12], instr[6:2], 5'b0, 3'b0, instr[11:7], OPCODE_OPIMM};
              rs1_enable = 1'b1;
            end else begin
              // c.li -> addi rd, x0, nzimm
              instr_o.bus_resp.rdata = {{6 {instr[12]}}, instr[12], instr[6:2], 5'b0, 3'b0, instr[11:7], OPCODE_OPIMM};
              rs1_enable = 1'b1;
            end
          end

          3'b011: begin
            if ({instr[12], instr[6:2]} == 6'b0) begin
              illegal_instr_o = 1'b1;
              rs1_enable      = 1'b1;
              rs2_enable      = 1'b1;
            end else begin
              if (instr[11:7] == 5'h02) begin
                // c.addi16sp -> addi x2, x2, nzimm
                instr_o.bus_resp.rdata = {{3 {instr[12]}}, instr[4:3], instr[5], instr[2], instr[6], 4'b0, 5'h02, 3'b000, 5'h02, OPCODE_OPIMM};
                rs1_enable = 1'b1;
              end else if (instr[11:7] == 5'b0) begin
                // Hint -> lui x0, imm
                instr_o.bus_resp.rdata = {{15 {instr[12]}}, instr[6:2], instr[11:7], OPCODE_LUI};
              end else begin
                // c.lui -> lui rd, imm
                instr_o.bus_resp.rdata = {{15 {instr[12]}}, instr[6:2], instr[11:7], OPCODE_LUI};
              end
            end
          end

          3'b100: begin
            unique case (instr[11:10])
              2'b00,
              2'b01: begin
                // 00: c.srli -> srli rd, rd, shamt
                // 01: c.srai -> srai rd, rd, shamt
                if (instr[12] == 1'b1) begin
                  // Reserved for future custom extensions (instr_o don't care)
                  instr_o.bus_resp.rdata = {1'b0, instr[10], 5'b0, instr[6:2], 2'b01, instr[9:7], 3'b101, 2'b01, instr[9:7], OPCODE_OPIMM};
                  illegal_instr_o = 1'b1;
                  rs1_enable = 1'b1;
                  rs2_enable = 1'b1;
                end else begin
                  if (instr[6:2] == 5'b0) begin
                    // Hint
                    instr_o.bus_resp.rdata = {1'b0, instr[10], 5'b0, instr[6:2], 2'b01, instr[9:7], 3'b101, 2'b01, instr[9:7], OPCODE_OPIMM};
                    rs1_enable = 1'b1;
                  end else begin
                    instr_o.bus_resp.rdata = {1'b0, instr[10], 5'b0, instr[6:2], 2'b01, instr[9:7], 3'b101, 2'b01, instr[9:7], OPCODE_OPIMM};
                    rs1_enable = 1'b1;
                  end
                end
              end

              2'b10: begin
                // c.andi -> andi rd, rd, imm
                instr_o.bus_resp.rdata = {{6 {instr[12]}}, instr[12], instr[6:2], 2'b01, instr[9:7], 3'b111, 2'b01, instr[9:7], OPCODE_OPIMM};
                rs1_enable = 1'b1;
              end

              2'b11: begin
                unique case ({instr[12], instr[6:5]})
                  3'b000: begin
                    // c.sub -> sub rd', rd', rs2'
                    instr_o.bus_resp.rdata = {2'b01, 5'b0, 2'b01, instr[4:2], 2'b01, instr[9:7], 3'b000, 2'b01, instr[9:7], OPCODE_OP};
                    rs1_enable = 1'b1;
                    rs2_enable = 1'b1;
                  end

                  3'b001: begin
                    // c.xor -> xor rd', rd', rs2'
                    instr_o.bus_resp.rdata = {7'b0, 2'b01, instr[4:2], 2'b01, instr[9:7], 3'b100, 2'b01, instr[9:7], OPCODE_OP};
                    rs1_enable = 1'b1;
                    rs2_enable = 1'b1;
                  end

                  3'b010: begin
                    // c.or  -> or  rd', rd', rs2'
                    instr_o.bus_resp.rdata = {7'b0, 2'b01, instr[4:2], 2'b01, instr[9:7], 3'b110, 2'b01, instr[9:7], OPCODE_OP};
                    rs1_enable = 1'b1;
                    rs2_enable = 1'b1;
                  end

                  3'b011: begin
                    // c.and -> and rd', rd', rs2'
                    instr_o.bus_resp.rdata = {7'b0, 2'b01, instr[4:2], 2'b01, instr[9:7], 3'b111, 2'b01, instr[9:7], OPCODE_OP};
                    rs1_enable = 1'b1;
                    rs2_enable = 1'b1;
                  end

                  3'b100,
                  3'b101,
                  3'b110,
                  3'b111: begin
                    // 100: c.subw
                    // 101: c.addw
                    illegal_instr_o = 1'b1;
                    rs1_enable = 1'b1;
                    rs2_enable = 1'b1;
                  end
                endcase
              end
            endcase
          end

          3'b110, 3'b111: begin
            // 0: c.beqz -> beq rs1', x0, imm
            // 1: c.bnez -> bne rs1', x0, imm
            instr_o.bus_resp.rdata = {{4 {instr[12]}}, instr[6:5], instr[2], 5'b0, 2'b01, instr[9:7], 2'b00, instr[13], instr[11:10], instr[4:3], instr[12], OPCODE_BRANCH};
            rs1_enable = 1'b1;
            rs2_enable = 1'b1;
          end
        endcase
      end

      // C2
      2'b10: begin
        unique case (instr[15:13])
          3'b000: begin
            if (instr[12] == 1'b1) begin
              // Reserved for future extensions (instr_o don't care)
              instr_o.bus_resp.rdata = {7'b0, instr[6:2], instr[11:7], 3'b001, instr[11:7], OPCODE_OPIMM};
              illegal_instr_o = 1'b1;
              rs1_enable = 1'b1;
              rs2_enable = 1'b1;
            end else begin
              if ((instr[6:2] == 5'b0) || (instr[11:7] == 5'b0)) begin
                // Hint -> slli rd, rd, shamt
                instr_o.bus_resp.rdata = {7'b0, instr[6:2], instr[11:7], 3'b001, instr[11:7], OPCODE_OPIMM};
                rs1_enable = 1'b1;
              end else begin
                // c.slli -> slli rd, rd, shamt
                instr_o.bus_resp.rdata = {7'b0, instr[6:2], instr[11:7], 3'b001, instr[11:7], OPCODE_OPIMM};
                rs1_enable = 1'b1;
              end
            end
          end

          3'b010: begin
            // c.lwsp -> lw rd, imm(x2)
            instr_o.bus_resp.rdata = {4'b0, instr[3:2], instr[12], instr[6:4], 2'b00, 5'h02, 3'b010, instr[11:7], OPCODE_LOAD};
            if (instr[11:7] == 5'b0) begin
              illegal_instr_o = 1'b1;
              rs1_enable = 1'b1;
              rs2_enable = 1'b1;
            end else begin
              rs1_enable = 1'b1;
            end
          end

          3'b100: begin
            if (instr[12] == 1'b0) begin
              if (instr[6:2] == 5'b0) begin
                // c.jr -> jalr x0, rd/rs1, 0
                instr_o.bus_resp.rdata = {12'b0, instr[11:7], 3'b0, 5'b0, OPCODE_JALR};
                // c.jr with rs1 = 0 is reserved
                if (instr[11:7] == 5'b0) begin
                  illegal_instr_o = 1'b1;
                  rs1_enable = 1'b1;
                  rs2_enable = 1'b1;
                end else begin
                  rs1_enable = 1'b1;
                end
              end else begin
                if (instr[11:7] == 5'b0) begin
                  // Hint -> add x0, x0, rs2
                  instr_o.bus_resp.rdata = {7'b0, instr[6:2], 5'b0, 3'b0, instr[11:7], OPCODE_OP};
                  rs1_enable = 1'b1;
                  rs2_enable = 1'b1;
                end else begin
                  // c.mv -> add rd, x0, rs2
                  instr_o.bus_resp.rdata = {7'b0, instr[6:2], 5'b0, 3'b0, instr[11:7], OPCODE_OP};
                  rs1_enable = 1'b1;
                  rs2_enable = 1'b1;
                end
              end
            end else begin
              if (instr[6:2] == 5'b0) begin
                if (instr[11:7] == 5'b0) begin
                  // c.ebreak -> ebreak
                  instr_o.bus_resp.rdata = {32'h00_10_00_73};
                end else begin
                  // c.jalr -> jalr x1, rs1, 0
                  instr_o.bus_resp.rdata = {12'b0, instr[11:7], 3'b000, 5'b00001, OPCODE_JALR};
                  rs1_enable = 1'b1;
                end
              end else begin
                if (instr[11:7] == 5'b0) begin
                  // Hint -> add x0, x0, rs2
                  instr_o.bus_resp.rdata = {7'b0, instr[6:2], instr[11:7], 3'b0, instr[11:7], OPCODE_OP};
                  rs1_enable = 1'b1;
                  rs2_enable = 1'b1;
                end else begin
                  // c.add -> add rd, rd, rs2
                  instr_o.bus_resp.rdata = {7'b0, instr[6:2], instr[11:7], 3'b0, instr[11:7], OPCODE_OP};
                  rs1_enable = 1'b1;
                  rs2_enable = 1'b1;
                end
              end
            end
          end

          3'b110: begin
            // c.swsp -> sw rs2, imm(x2)
            instr_o.bus_resp.rdata = {4'b0, instr[8:7], instr[12], instr[6:2], 5'h02, 3'b010, instr[11:9], 2'b00, OPCODE_STORE};
            rs1_enable = 1'b1;
            rs2_enable = 1'b1;
          end

          3'b001,        // c.fldsp -> fld rd, imm(x2)
          3'b011,        // c.flwsp -> flw rd, imm(x2)
          3'b101,        // c.fsdsp -> fsd rs2, imm(x2)
          3'b111: begin  // c.fswsp -> fsw rs2, imm(x2)
            illegal_instr_o = 1'b1;
            rs1_enable = 1'b1;
            rs2_enable = 1'b1;
          end
        endcase
      end

      default: begin
        // 32 bit (or more) instruction
        instr_o.bus_resp.rdata = instr_i.bus_resp.rdata;
        illegal_uncompressed_o = 1'b0;
        rs1_enable = 1'b0;
        rs2_enable = 1'b0;
        unique case (instr[6:0])

          OPCODE_JAL: begin // Jump and Link
            rs1_enable = 1'b0;
            rs2_enable = 1'b0;
          end
        
          OPCODE_JALR: begin // Jump and Link Register
            if (instr[14:12] != 3'b0) begin
              rs1_enable = 1'b1;
              rs2_enable = 1'b1;
              illegal_uncompressed_o = 1'b1;
            end
            else begin
              rs1_enable = 1'b1;
            end
          end
          
          OPCODE_BRANCH: begin // Branch
            rs1_enable = 1'b1;
            rs2_enable = 1'b1;
          end
        
          OPCODE_STORE: begin
            rs1_enable = 1'b1;
            rs2_enable = 1'b1;
          end
          
          OPCODE_LOAD: begin
            rs1_enable = 1'b1;
        
            // Reserved or RV64
            if ((instr[14:12] == 3'b111) || (instr[14:12] == 3'b110) || (instr[14:12] == 3'b011)) begin
              rs2_enable = 1'b1;
              illegal_uncompressed_o = 1'b1;
            end
          end
        
          OPCODE_LUI: begin  // Load Upper Immediate
            rs1_enable = 1'b0;
            rs2_enable = 1'b0;
          end
        
          OPCODE_AUIPC: begin  // Add Upper Immediate to PC
            rs1_enable = 1'b0;
            rs2_enable = 1'b0;
          end
        
          OPCODE_OPIMM: begin // Register-Immediate ALU Operations
            rs1_enable = 1'b1;
        
            unique case (instr[14:12])
              3'b000,
              3'b010,
              3'b011,
              3'b100,
              3'b110,
              3'b111: begin
                rs1_enable = 1'b1;
              end
              3'b001: begin
                if (instr[31:25] != 7'b0) begin
                  rs1_enable = 1'b1;
                  rs2_enable = 1'b1;
                  illegal_uncompressed_o = 1'b1;
                end
              end
        
              3'b101: begin
                if ((instr[31:25] == 7'b0)) begin
                  rs1_enable = 1'b1;
                end else if ((instr[31:25] == 7'b010_0000)) begin
                  rs1_enable = 1'b1;
                end else begin
                  rs1_enable = 1'b1;
                  rs2_enable = 1'b1;
                  illegal_uncompressed_o = 1'b1;
                end
              end
            endcase
          end
        
          OPCODE_OP: begin  // Register-Register ALU operation
            rs1_enable = 1'b1;

            if ((instr[31:30] == 2'b11) || (instr[31:30] == 2'b10)) begin
              rs1_enable = 1'b1;
              rs2_enable = 1'b1;
              illegal_uncompressed_o = 1'b1;
            end else begin
              if (~instr[28]) rs2_enable = 1'b1;
        
              unique case ({instr[30:25], instr[14:12]})
                // RV32I ALU operations
                {6'b00_0000, 3'b000},
                {6'b10_0000, 3'b000},
                {6'b00_0000, 3'b010},
                {6'b00_0000, 3'b011},
                {6'b00_0000, 3'b100},
                {6'b00_0000, 3'b110},
                {6'b00_0000, 3'b111},
                {6'b00_0000, 3'b001},
                {6'b00_0000, 3'b101},
                {6'b10_0000, 3'b101}: begin
                  rs1_enable = 1'b1;
                end
                default: begin
                  rs1_enable = 1'b1;
                  rs2_enable = 1'b1;
                  illegal_uncompressed_o = 1'b1;
                end
              endcase
            end
          end
        
          OPCODE_FENCE: begin
            unique case (instr[14:12])
              3'b000,
              3'b001: begin // FENCE.I
                rs1_enable = 1'b0;
                rs2_enable = 1'b0;
              end
        
              default: begin
                rs1_enable = 1'b1;
                rs2_enable = 1'b1;
                illegal_uncompressed_o = 1'b1;
              end
            endcase
          end
        
          OPCODE_SYSTEM: begin
            if (instr[14:12] == 3'b000)
            begin
              // non CSR related SYSTEM instructions
              // TODO: Could set 2'b00 for X_EXT=0, and 2'b11 for X_EXT=1
              if ( {instr[19:15], instr[11:7]} == '0)
              begin
                unique case (instr[31:20])
                  12'h000,  // ECALL
                  12'h001,  // ebreak
                  12'h105,  // wfi
                  12'h302: begin  // mret
                    rs1_enable = 1'b0;
                    rs2_enable = 1'b0;
                  end
                  
                  12'h7b2:  // dret
                    begin
                      if (!ctrl_fsm_i.debug_mode) begin
                        rs1_enable = 1'b1;
                        rs2_enable = 1'b1;
                        illegal_uncompressed_o = 1'b1;
                      end
                    end
                  
                  default:
                    begin
                      rs1_enable = 1'b1;
                      rs2_enable = 1'b1;
                      illegal_uncompressed_o = 1'b1;
                  end
                endcase
              end else begin // if ( {instr[19:15], instr[11:7]} == '0)
                rs1_enable = 1'b1;
                rs2_enable = 1'b1;
                illegal_uncompressed_o = 1'b1;
              end
            end // if (instr[14:12] == 3'b000)
            else
            begin
              illegal_uncompressed_o = 1'b1;
              if (instr[14] != 1'b1) begin
                rs1_enable = 1'b1;
              end
        
              // instr[19:14] = rs or immediate value
              //   if set or clear with rs==x0 or imm==0,
              //   then do not perform a write action
              if(instr[13:12] == 2'b00) begin
                rs1_enable = 1'b1;
                rs2_enable = 1'b1;
                illegal_uncompressed_o = 1'b1;
              end
            end
          end
        
          default: begin
            rs1_enable = 1'b1;
            rs2_enable = 1'b1;
            illegal_uncompressed_o = 1'b1;
          end
        endcase
      end
    endcase
  end

  assign is_compressed_o = (instr[1:0] != 2'b11);

endmodule
