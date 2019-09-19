#![warn(non_camel_case_types)]

// const declares
const NUM_X: usize = 32;

const MAX_RAM_SIZE: usize = 256 * 1024 * 1024;
const RAM_START: usize = 0x8000_0000;
const RAM_END: usize = RAM_START + MAX_RAM_SIZE - 1;

//const MAX_ROM_SIZE: usize = 1024 * 1024;
const ROM_START: usize = 0xFFFF_FFFF_FFF0_0000;
const ROM_END: usize = 0xFFFF_FFFF_FFFF_FFFF;

// const declares: CSR
/*
#define r_MISA		0xF10
#define r_MVENDORID	0xF11
#define r_MARCHID	0xF12
#define r_MIMPID	0xF13
#define r_MHARTID	0xF14

#define r_MEDELEG	0x302
#define r_MIDELEG	0x303
#define r_MIE		0x304

#define r_MSCRATCH	0x340
#define r_MIP		0x344

#define r_MTOHOST	0x780
#define r_MFROMHOST	0x781
*/

const MSTATUS:  usize = 0x300;
const MTVEC:    usize = 0x305;
const MEPC:     usize = 0x341;
const MCAUSE:   usize = 0x342;
const MBADADDR: usize = 0x343;

// use declares
extern crate clap;
//extern crate goblin;
use clap::{Arg, App};
//use goblin::{Object};
use std::io::Read;

enum Instruction {
    ADDI(u8, i32),
    AUIPC(u8, i32),
    JAL(u8, i32),
    JALR(u8, u8, i32),
    LUI(u8, i32),
    BEQ(u8, u8, i32),
    LB(u8, u8, i32),
    LBU(u8, u8, i32),
    //LH(u8, u8, i32),
    LW(u8, u8, i32),
    LD(u8, u8, i32),
    SB(u8, u8, i32),
    //SH(u8, u8, i32),
    //SW(u8, u8, i32),
    SD(u8, u8, i32),
    SLLIW(u8, u8, i32),
    OR(u8, u8, u8),
    ANDI(u8, u8, i32),
    ORI(u8, u8, i32),
    CSRRS(u8, u8, u16),
    CSRRW(u8, u8, u16),
    MUL(u8, u8, u8),

    CXADD(u8, u8),
    CXMV(u8, u8),
    CXADDI(u8, i32),
    CXADDIW(u8, i32),
    CXADDI4SPN(u8, i32),
    CXOR(u8, u8),
    CXAND(u8, u8),
    CXLI(u8, i32),
    CXLUI(u8, i32),
    CXLDSP(u8, i32),
    CXJR(u8),
    CXSD(u8, u8, i32),
    CXSDSP(u8, i32),
    CXSLLI(u8, i32),

    EBREAK,
    //NOP,
}

impl Instruction {
    fn from_word(word: u32) -> Option<Instruction> {
        let opcode = Self::get_opcode(word);
        let opcode_c = Self::get_rvc_opcode(word);
        if word == 0 || word == 0xFFFF_FFFF {
            panic!("Illegal instruction: {:#x} ({:#b})", opcode, opcode);
        } else {
            if opcode_c != 0b11 {
                println!("RVC Opcode: {:#x} ({:#b})", opcode_c, opcode_c);
                match opcode_c {
                    0 => {
                        match Self::get_rvc_funct3(word) {
                            0 => Some(Instruction::CXADDI4SPN(Self::get_rvc_rs1rd_815(word), Self::get_rvc_imm(word))),
                            1 => panic!("Unimplemented RVC Op: C.FLD"),
                            2 => panic!("Unimplemented RVC Op: C.LW"),
                            3 => panic!("Unimplemented RVC Op: C.LD"),
                            4 => panic!("Illegal RVC-Op: Reserved"),
                            5 => panic!("Unimplemented RVC Op: C.FSD"),
                            6 => panic!("Unimplemented RVC Op: C.SW"),
                            7 => Some(Instruction::CXSD(Self::get_rvc_rs1rd_815(word), Self::get_rvc_rs2_815(word), Self::get_rvc_imm(word))),
                            _ => None,
                        }
                    },
                    1 => {
                        match Self::get_rvc_funct3(word) {
                            0 => Some(Instruction::CXADDI(Self::get_rvc_rs1rd(word), Self::get_rvc_nzimm(word))),
                            1 => Some(Instruction::CXADDIW(Self::get_rvc_rs1rd(word), Self::get_rvc_imm(word))),
                            2 => Some(Instruction::CXLI(Self::get_rvc_rs1rd(word), Self::get_rvc_imm(word))),
                            3 => {
                                let rd = Self::get_rvc_rs1rd(word);
                                if rd == 2 {
                                    panic!("Unimplemented RVC Op: C.ADDI16SP")
                                } else {
                                    Some(Instruction::CXLUI(rd, Self::get_rvc_nzimm(word)))
                                }
                            },
                            4 => {
                                match Self::get_rvc_bitwise_high(word) {
                                    0 => panic!("Unimplemented RVC Op: C.SRLI64"),
                                    1 => panic!("Unimplemented RVC Op: C.SRAI64"),
                                    2 => panic!("Unimplemented RVC Op: C.ANDI"),
                                    3 => {
                                        if Self::get_rvc_bit12(word) == false {
                                            match Self::get_rvc_bitwise_low(word) {
                                                0 => panic!("Unimplemented RVC Op: C.SUB"),
                                                1 => panic!("Unimplemented RVC Op: C.XOR"),
                                                2 => Some(Instruction::CXOR(Self::get_rvc_rs1rd_815(word), Self::get_rvc_rs2_815(word))),
                                                3 => Some(Instruction::CXAND(Self::get_rvc_rs1rd_815(word), Self::get_rvc_rs2_815(word))),
                                                _ => panic!("get_rvc_bitwise_low() returned uncaught value")
                                            }
                                        } else {
                                            match Self::get_rvc_bitwise_low(word) {
                                                0 => panic!("Unimplemented RVC Op: C.SUBW"),
                                                1 => panic!("Unimplemented RVC Op: C.ADDW"),
                                                2 => panic!("Unimplemented RVC Op: reserved (low: 10)"),
                                                3 => panic!("Unimplemented RVC Op: reserved (low: 11)"),
                                                _ => panic!("get_rvc_bitwise_low() returned uncaught value")
                                            }
                                        }
                                    },
                                    _ => panic!("get_rvc_bitwise_high() returned uncaught value"),
                                }
                            },
                            5 => panic!("Unimplemented RVC Op: C.J"),
                            6 => panic!("Unimplemented RVC Op: C.BEQZ"),
                            7 => panic!("Unimplemented RVC Op: C.BNEZ"),                            
                            _ => None,
                        }
                    },
                    2 => {
                        let rs1rd = Self::get_rvc_rs1rd(word);
                        match Self::get_rvc_funct3(word) {
                            0 => Some(Instruction::CXSLLI(rs1rd, Self::get_rvc_imm(word))),
                            1 => panic!("Unimplemented RVC Op: C.FLDSP"),
                            2 => panic!("Unimplemented RVC Op: C.LWSP"),
                            3 => Some(Instruction::CXLDSP(rs1rd, Self::get_rvc_imm(word))),
                            4 => {
                                let rs2 = Self::get_rvc_rs2(word);
                                match Self::get_rvc_bit12(word) {
                                    false => {
                                        if rs2 > 0 {
                                            Some(Instruction::CXMV(rs2, Self::get_rvc_rs1rd(word)))
                                        } else {
                                            Some(Instruction::CXJR(Self::get_rvc_rs1rd(word)))
                                        }
                                    },
                                    true => {
                                        if rs2 > 0 {
                                            Some(Instruction::CXADD(rs1rd, rs2))
                                        } else {
                                            if rs1rd > 0 {
                                                panic!("Unimplemented RVC Op: C.JALR")
                                            } else {
                                                panic!("Unimplemented RVC Op: C.EBREAK")
                                            }
                                        }
                                    }
                                }
                            },
                            5 => panic!("Unimplemented RVC Op: C.FSDSP"),
                            6 => panic!("Unimplemented RVC Op: C.SWSP"),
                            7 => Some(Instruction::CXSDSP(Self::get_rvc_rs2(word), Self::get_rvc_imm(word))),
                            _ => None,
                        }
                    },
                    _ => None
                }
            } else {
                println!("Raw opcode: {:#x} ({:#b})", opcode, opcode);
                match opcode {
                    0x3 => {
                        match Self::get_funct3(word) {
                            0x0 => Some(Instruction::LB(Self::get_rs1(word), Self::get_rd(word), Self::get_s_imm(word))),
                            0x2 => Some(Instruction::LW(Self::get_rs1(word), Self::get_rd(word), Self::get_s_imm(word))),
                            0x3 => Some(Instruction::LD(Self::get_rs1(word), Self::get_rd(word), Self::get_s_imm(word))),
                            0x4 => Some(Instruction::LBU(Self::get_rs1(word), Self::get_rd(word), Self::get_s_imm(word))),
                            _ => None   
                        }
                    }
                    0x13 => {
                        match Self::get_funct3(word) {
                            0x0 => {
                                match Self::get_funct7(word) {
                                    _ => Some(Instruction::ADDI(Self::get_rs1(word), Self::get_i_imm(word)))
                                }
                            }
                            0x6 => Some(Instruction::ORI(Self::get_rs1(word), Self::get_rd(word), Self::get_i_imm(word))),
                            0x7 => Some(Instruction::ANDI(Self::get_rs1(word), Self::get_rd(word), Self::get_i_imm(word))),
                            _ => None
                        }
                    }
                    0x17 => Some(Instruction::AUIPC(Self::get_rd(word), Self::get_u_imm(word))),
                    0x1b => {
                        match Self::get_funct3(word) {
                            0 => panic!("Unimplemented opcode: ADDIW"),
                            1 => Some(Instruction::SLLIW(Self::get_rs1(word), Self::get_rd(word), Self::get_i_imm(word))),
                            5 => {
                                match Self::get_funct7(word) {
                                    0 => panic!("Unimplemented opcode: SRLIW"),
                                    0x20 => panic!("Unimplemented opcode: SRAIW"),
                                    _ => None
                                }
                            }
                            _ => None
                        }
                    }
                    0x23 => {
                        match Self::get_funct3(word) {
                            0 => Some(Instruction::SB(Self::get_rs1(word), Self::get_rs2(word), Self::get_s_imm(word))),
                            1 => None,
                            2 => None,
                            3 => Some(Instruction::SD(Self::get_rs1(word), Self::get_rs2(word), Self::get_s_imm(word))),
                            _ => None
                        }
                    }
                    0x33 => {
                        match Self::get_funct3(word) {
                            0 => {
                                match Self::get_funct7(word) {
                                    0 => panic!("Unimplemented opcode: ADD"),
                                    1 => Some(Instruction::MUL(Self::get_rs1(word), Self::get_rs2(word), Self::get_rd(word))),
                                    0x20 => panic!("Unimplemented opcode: SUB"),
                                    _ => None
                                }
                            }
                            6 => Some(Instruction::OR(Self::get_rs1(word), Self::get_rs2(word), Self::get_rd(word))),
                            _ => None
                        }
                    }
                    0x37 => Some(Instruction::LUI(Self::get_rd(word), Self::get_u_imm(word))),
                    0x63 => {
                        match Self::get_funct3(word) {
                            0 => Some(Instruction::BEQ(Self::get_rs1(word), Self::get_rs2(word), Self::get_b_imm(word))),
                            _ => None
                        }
                    }
                    0x67 => {
                        match Self::get_funct3(word) {
                            0 => Some(Instruction::JALR(Self::get_rd(word), Self::get_rs1(word), Self::get_i_imm(word))),
                            _ => None
                        }
                    }
                    0x6f => Some(Instruction::JAL(Self::get_rd(word), Self::get_j_imm(word))),
                    0x73 => {
                        let rs2 = Self::get_rs2(word);
                        match Self::get_funct3(word) {
                            0 => {
                                match Self::get_funct7(word) {
                                    0 => {
                                        if rs2 == 2 {
                                            panic!("Unimplemented opcode: URET")
                                        } else {
                                            panic!("Unknown opcode type 0x73")
                                        }
                                    },
                                    0x1 => Some(Instruction::EBREAK),
                                    0x8 => {
                                        if rs2 == 2 {
                                            panic!("Unimplemented opcode: SRET")
                                        } else if rs2 == 5 {
                                            panic!("Unimplemented opcode: WFI")
                                        } else {
                                            panic!("Unknown opcode type 0x73")
                                        }
                                    },
                                    0x9 => panic!("Unimplemented opcode: SFENCE.VMA"),
                                    0x11 => panic!("Unimplemented opcode: HFENCE.BVMA"),
                                    0x18 => panic!("Unimplemented opcode: MRET"),
                                    0x51 => panic!("Unimplemented opcode: HFENCE.GVMA"),
                                    _ => None
                                }
                            },
                            1 => {
                                Some(Instruction::CSRRW(Self::get_rs1(word), Self::get_rd(word), Self::get_funct12(word)))
                            }
                            2 => {
                                Some(Instruction::CSRRS(Self::get_rs1(word), Self::get_rd(word), Self::get_funct12(word)))
                            }
                            _ => None
                        }
                    }
                    _ => None::<Instruction>
                }
            }
        }
    }

    fn get_opcode(word: u32) -> u8 {
        (word & 0x7F) as u8
    }

    fn get_rvc_opcode(word: u32) -> u8 {
        (word & 0b11) as u8
    }

    fn get_rvc_funct3(word: u32) -> u8 {
        ((word >> 13) & 0b111) as u8
    }

    fn get_rvc_rs1rd(word: u32) -> u8 {
        ((word >> 7) & 0b11111) as u8
    }

    fn get_rvc_rs1rd_815(word: u32) -> u8 {
        (((word >> 7) & 0b111) + 8) as u8
    }

    fn get_rvc_rs2_815(word: u32) -> u8 {
        (((word >> 2) & 0b111) + 8) as u8
    }

    fn get_rvc_rs2(word: u32) -> u8 {
        ((word >> 2) & 0b11111) as u8
    }

    fn get_rvc_bitwise_low(word: u32) -> u8 {
        ((word >> 5) & 0b11) as u8
    }

    fn get_rvc_bitwise_high(word: u32) -> u8 {
        ((word >> 10) & 0b11) as u8
    }

    fn get_rvc_bit12(word: u32) -> bool {
        ((word >> 12) & 0b1) != 0
    }

    fn get_rvc_imm(word: u32) -> i32 {
        let opcode = Self::get_rvc_opcode(word);
        let funct3 = Self::get_rvc_funct3(word);
        let imm;
        if opcode == 0 {
            match funct3 {
                0 => {
                    imm = (((word >> 6) & 0b1) << 2 | ((word >> 5) & 0b1) << 3 | ((word >> 11) & 0b11) << 4 | ((word >> 9) & 0b111) << 6) as i32;
                }
                7 => {
                    imm = (((word >> 10) & 0b111) << 3 | ((word >> 5) & 0b11) << 6) as i32;
                }
                _ => panic!("get_rvc_imm(): unimplemented decoder [0]")
            }
        } else if opcode == 1 {
            match funct3 {
                1 => {
                    let mut result = (((word >> 2) & 0b11111) | ((word >> 12) & 0b1) << 5) as i32;

                    result |= -(result & 0x20);
                    imm = result;
                }
                2 => {
                    let mut result = (((word >> 2) & 0b11111) | ((word >> 12) & 0b1) << 5) as i32;

                    result |= -(result & 0x20);
                    imm = result;
                }
                4 => {
                    let mut result = (((word >> 2) & 0b11111) | ((word >> 12) & 0b1) << 5) as i32;

                    result |= -(result & 0x20);
                    imm = result;
                }                
                _ => panic!("get_rvc_imm(): unimplemented decoder [1]")
            }
        } else if opcode == 2 {
            match funct3 {
                0 => {
                    let mut result = (((word >> 2) & 0b11111) | ((word >> 12) & 0b1) << 5) as i32;

                    result |= -(result & 0x20);
                    imm = result;
                }
                3 => {
                    imm = (((word >> 5) & 0b11) << 3 | ((word >> 12) & 0b1) << 5 | ((word >> 2) & 0b111) << 6) as i32;
                }
                5 => {
                    imm = (((word >> 7) & 0b111) << 6 | ((word >> 10) & 0b111) << 3) as i32;
                }                
                7 => {
                    imm = (((word >> 7) & 0b111) << 6 | ((word >> 10) & 0b111) << 3) as i32;
                }
                _ => panic!("get_rvc_imm(): unimplemented decoder [2]")
            }
        } else {
            panic!("get_rvc_imm(): invalid opcode")
        }
        imm
    }

    fn get_rvc_nzimm(word: u32) -> i32 {
        // C.LUI: displace bits 2-6 and bit 12 and combine at bit 12
        // C.ADDI16SP: what the *fuck*
        // C.ADDI: return basic immediate
        let funct3 = Self::get_rvc_funct3(word);
        let rd = Self::get_rvc_rs1rd(word);
        if funct3 == 3 {
            if rd == 2 {
                let mut result = (((word >> 6) & 0b1) << 4 | ((word >> 2) & 0b1) << 5 | ((word >> 5) & 0b1) << 6 | ((word >> 3) & 0b11) << 7 | ((word >> 12) & 0b1) << 9) as i32;
                
                result |= -(result & 0x200);
                result
            } else {
                let mut result = (((word >> 2) & 0b11111) << 12 | ((word >> 12) & 0b1) << 17) as i32;

                result |= -(result & 0x20000);
                result
            }
        } else if funct3 == 0 {
            let mut result = (((word >> 2) & 0b11111) | ((word >> 12) & 0b1) << 5) as i32;

            result |= -(result & 0x20);
            result
        } else {
            panic!("get_rvc_nzimm(): WHY ARE WE HERE")
        }
    }

    fn get_rd(word: u32) -> u8 {
        ((word >> 7) & 0b11111) as u8
    }

    fn get_rs1(word: u32) -> u8 {
        ((word >> 15) & 0b11111) as u8
    }

    fn get_rs2(word: u32) -> u8 {
        ((word >> 20) & 0b11111) as u8
    }

    fn get_funct3(word: u32) -> u8 {
        println!("get_funct3: {:x}", ((word >> 12) & 0b111));
        ((word >> 12) & 0b111) as u8
    }

    fn get_funct7(word: u32) -> u8 {
        println!("get_funct7: {:x}", (word >> 25) as u8);
        (word >> 25) as u8
    }

    fn get_funct12(word: u32) -> u16 {
        (word >> 20) as u16
    }

    fn get_b_imm(word: u32) -> i32 {
        let mut disp12 = (((word >> 7) & 0x1E) |((word << 4) & 0x800) |((word >> 20) & 0x7E0) | ((word >> 19) & 0x1000)) as i32;

	    disp12 |= -(disp12 & 0x1000);
        disp12
    }

    fn get_u_imm(word: u32) -> i32 {
        ((word) & 0xFFFF_F000) as i32
    }

    fn get_j_imm(word: u32) -> i32 {
        let mut disp20: u32 = ((word & 0x7FE00000) >> 20) | ((word & 0x100000) >> 9) | (word & 0xFF000) | ((word & 0x80000000) >> 11);
        if disp20 & 0x100000 == 0x100000 {
            disp20 |= 0xFFF00000;
        }

        disp20 as i32
    }

    fn get_s_imm(word: u32) -> i32 {
        let mut s_imm: u32 = ((word >> 7) & 0x1F) | ((word >> 20) & 0xFE0);
        if s_imm & 0x800 == 0x800 {
            s_imm |= 0xFFFF_F000;
        }
        s_imm as i32
    }

    fn get_i_imm(word: u32) -> i32 {
        let mut disp12 = (word >> 20) as i32;
	    disp12 |= -(disp12 & 0x800);
        println!("get_i_imm: input 0x{:x}, output 0x{:x}", (word >> 20) as i32, disp12);
        disp12
    }
}

struct CSR {
    csr: [u64; 4096],
}

impl CSR {
    fn new() -> CSR {
        CSR {
            csr: [0; 4096],
        }
    }
    fn write_csr(&mut self, csr_num: usize, value: u64) {
        match csr_num {
            _ => {self.csr[csr_num] = value},
        }
    }
    fn read_csr(&mut self, csr_num: usize) -> u64 {
        match csr_num {
            _ => {self.csr[csr_num]}
        }
    }

    fn advance_by_cycles(&mut self, cycles: u64) {
        let total_cycles = self.read_csr(0xC00).wrapping_add(cycles);
        self.write_csr(0xC00, total_cycles);
        println!("Cycles: {}", self.read_csr(0xC00));
    }

    fn advance(&mut self) {
        self.advance_by_cycles(4);
    }
}

struct Registers {
    reg_x: [u64; NUM_X],
    //reg_f: [u64; NUM_X]
}

impl Registers {
    fn new() -> Registers {
        Registers {
            reg_x: [0; NUM_X],
            //reg_f: [0; NUM_X],
        }
    }
    fn write_reg(&mut self, reg_num: u8, value: u64) {
        let register = reg_num as usize;
        if register != 0 {
            self.reg_x[register] = value;
        }
        println!("Register x{} write: {:x}", register, value);
    }
    fn read_reg(&mut self, reg_num: u8) -> u64 {
        if reg_num == 0 {
            0
        } else {
            self.reg_x[reg_num as usize]
        }
    }
    /*
    fn read_fpreg(&mut self, reg_num: u8) -> u64 {
        if reg_num == 0 {
            0
        } else {
            self.reg_f[reg_num as usize]
        }
    }
    fn write_fpreg(&mut self, reg_num: u8, value: u64) {
        let register = reg_num as usize;
        if register != 0 {
            self.reg_f[register] = value;
        }
    }
    */
}

struct CPU {
    pub registers: Registers,
    pub csr: CSR,
    pc: u64,
    pub bus: MemoryBus,
    is_halted: bool,
}

impl CPU {
    fn new(boot_rom: Vec<u8>) -> CPU {
        CPU {
            registers: Registers::new(),
            csr: CSR::new(),
            pc: 0x0,
            bus: MemoryBus::new(boot_rom),
            is_halted: true,
        }
    }

    fn step(&mut self) {
        let instruction_word = self.bus.read_word(self.pc);
        
        let description = format!("0x{:x}", instruction_word);
        /*
        if instruction_word == 0 || instruction_word == 0xFFFF_FFFF {} else {
            println!("Decoding instruction found at pc: 0x{:x}: {} {:#034b}, ", self.pc, description, instruction_word);
        }
        */
        println!("PC: 0x{:x}: {} {:#034b}", self.pc, description, instruction_word);
        let next_pc: u64 = if let Some(instruction) = Instruction::from_word(instruction_word) {
            self.execute(instruction)
        } else {
            let description = format!("{:#034b}", instruction_word);
            panic!("Unknown instruction found for: {}", description);            
        };

        self.pc = next_pc;
    }

    fn reset(&mut self) {
        self.pc = 0x8000_0000;
        //self.csr.write_csr(MTVEC,  (-0x200 as i64) as u64);
        self.is_halted = false;
    }

    fn sign_extend_32_64(&self, value: i32) -> u64 {
        (value as i64) as u64
    }

    fn trap(&mut self, cause: u64) {
        //UDWORD newstat = p->csr[i_MSTATUS] & 0xFFFFFFFFFFFFE777;
	    //newstat |= (p->csr[i_MSTATUS] & 0x8) << 4;
        let mut newstat = self.csr.read_csr(MSTATUS) & 0xFFFFFFFFFFFFE777;
        newstat |= (self.csr.read_csr(MSTATUS) & 0x8) << 4;

        //p->csr[i_MCAUSE] = cause;
        //p->csr[i_MEPC] = p->csr[i_MBADADDR] = p->pc - 4;
        //p->csr[i_MSTATUS] = newstat;
        //p->pc = p->csr[i_MTVEC];
        self.csr.write_csr(MCAUSE, cause);
        self.csr.write_csr(MEPC, self.pc.wrapping_sub(4));
        self.csr.write_csr(MBADADDR, self.pc.wrapping_sub(4));
        self.csr.write_csr(MSTATUS, newstat);
        self.pc = self.csr.read_csr(MTVEC);
    }

    fn execute(&mut self, instruction: Instruction) -> u64 {
        match instruction {
            Instruction::ADDI(rs1, value) => {
                println!("Opcode ADDI (x{}, offset 0x{:x})", rs1, value);
                let result = self.registers.read_reg(rs1).wrapping_add(self.sign_extend_32_64(value));
                self.registers.write_reg(rs1, result as u64);
                self.csr.advance();
                self.pc.wrapping_add(4)
            },
            Instruction::JAL(rd, offset) => {
                println!("Opcode JAL: jumping to {:x}, offset {:x}", self.pc.wrapping_add((offset as i64) as u64), offset);
                self.registers.write_reg(rd, self.pc + 4);
                self.pc = self.pc.wrapping_add((offset as i64) as u64);
                self.csr.advance();
                self.pc
            }
            Instruction::JALR(rd, rs1, offset) => {
                let reg_rs1 = self.registers.read_reg(rs1);
                // & (-2 as i64)
                let new_offset = reg_rs1.wrapping_add(self.sign_extend_32_64(offset)) & ((-2 as i64) as u64);
                println!("Opcode JALR: offset 0x{:x}, sign_extended 0x{:x} new_offset 0x{:x}", offset, self.sign_extend_32_64(offset), new_offset);
                self.registers.write_reg(rd, self.pc + 4);
                self.pc = new_offset;
                self.csr.advance();
                self.pc
            }            
            Instruction::AUIPC(register, target_address) => {
                println!("Opcode AUIPC: adding upper immediate 0x{:x} to register", target_address);
                let result = self.sign_extend_32_64(target_address);
                self.registers.write_reg(register, result.wrapping_add(self.pc));
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::LUI(rd, imm) => {
                let uimm = self.registers.read_reg(rd).wrapping_add(self.sign_extend_32_64(imm as i32));
                self.registers.write_reg(rd, uimm);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }            
            Instruction::LBU(rs1, rd, imm_s) => {
                println!("Opcode: LBU");
                let address = self.registers.read_reg(rs1) + self.sign_extend_32_64(imm_s);
                // zero-extension, not sign-extension
                let loaded = self.bus.read_byte(address) as u64;
                self.registers.write_reg(rd, loaded as u64);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::LW(rs1, rd, imm_s) => {
                println!("Opcode: LW");
                let address = self.registers.read_reg(rs1) + self.sign_extend_32_64(imm_s);
                // sign-extend.
                let loaded = ((self.bus.read_word(address) as u64) << 32) >> 32;
                self.registers.write_reg(rd, loaded as u64);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::LD(rs1, rd, imm_s) => {
                println!("Opcode: LD");
                let address = self.registers.read_reg(rs1).wrapping_add(self.sign_extend_32_64(imm_s));
                let loaded = self.bus.read_dword(address);
                self.registers.write_reg(rd, loaded);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::SB(rs1, rs2, imm_s) => {
                println!("Opcode: SB");
                let address = self.registers.read_reg(rs1).wrapping_add(self.sign_extend_32_64(imm_s));
                let stored = self.registers.read_reg(rs2) as u8;
                self.bus.write_byte(address, stored);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::SD(rs1, rs2, imm_s) => {
                println!("Opcode: SD");
                let address = self.registers.read_reg(rs1).wrapping_add(self.sign_extend_32_64(imm_s));
                let stored = self.registers.read_reg(rs2);
                self.bus.write_dword(address, stored);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::OR(rs1, rs2, rd) => {
                println!("Opcode: OR");
                let result = self.registers.read_reg(rs1) | self.registers.read_reg(rs2);
                self.registers.write_reg(rd, result);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::ANDI(rs1, rd, imm_i) => {
                println!("Opcode: ANDI");
                let result = self.registers.read_reg(rs1) & self.sign_extend_32_64((imm_i << 20) >> 20);
                self.registers.write_reg(rd, result);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::ORI(rs1, rd, imm_i) => {
                println!("Opcode: ORI");
                let result = self.registers.read_reg(rs1) | self.sign_extend_32_64((imm_i << 20) >> 20);
                self.registers.write_reg(rd, result);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::SLLIW(rs1, rd, imm_i) => {
                let rs1_val = (self.registers.read_reg(rs1) & 0xFFFF_FFFF) as u32;
                let shamt = (imm_i & 0b111111) as u32;
                let result = rs1_val.wrapping_shl(shamt);
                self.registers.write_reg(rd, result as u64);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::EBREAK => {
                println!("Opcode: EBREAK");
                self.trap(3);
                self.csr.advance();
                self.pc
            }
            // OR the CSR with the rs1 bitmask
            Instruction::CSRRS(rs1, rd, csr) => {
                println!("Opcode: CSRRS");
                let src = self.csr.read_csr(csr as usize);
                self.registers.write_reg(rd, src);
                let dest = src | self.registers.read_reg(rs1);
                self.csr.write_csr(csr as usize, dest);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::CSRRW(rs1, rd, csr) => {
                println!("Opcode: CSRRW");
                let src = self.csr.read_csr(csr as usize);
                self.registers.write_reg(rd, src);
                let dest = self.registers.read_reg(rs1);
                self.csr.write_csr(csr as usize, dest);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            /*
            Instruction::NOP => {
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            */
            Instruction::BEQ(rs1, rs2, b_imm) => {
                println!("Opcode: BEQ");
                if self.registers.read_reg(rs1) == self.registers.read_reg(rs2) {
                    self.csr.advance();
                    self.pc.wrapping_add(self.sign_extend_32_64(b_imm))
                } else {
                    self.csr.advance();
                    self.pc.wrapping_add(4)
                }
            }
            Instruction::MUL(rs1, rs2, rd) => {
                let result = self.registers.read_reg(rs1) * self.registers.read_reg(rs2);
                self.registers.write_reg(rd, result);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }
            Instruction::CXLI(rd, imm) => {
                let uimm = self.registers.read_reg(rd) + (self.sign_extend_32_64(imm as i32) as u64);
                self.registers.write_reg(rd, uimm);
                self.csr.advance();
                self.pc.wrapping_add(2)
            }
            Instruction::CXLUI(rd, nzimm) => {
                let uimm = self.registers.read_reg(rd) + (self.sign_extend_32_64(nzimm as i32) as u64);
                self.registers.write_reg(rd, uimm);
                self.csr.advance();
                self.pc.wrapping_add(2)
            }
            Instruction::CXADDI(rd, value) => {
                println!("Opcode C.ADDI (x{}, offset {:x})", rd, value);
                let rd_val = self.registers.read_reg(rd) as u32;
                let result = self.sign_extend_32_64(rd_val.wrapping_add(value as u32) as i32);
                self.registers.write_reg(rd, result as u64);
                self.csr.advance();
                self.pc.wrapping_add(2)
            }
            Instruction::CXADDIW(rd, value) => {
                println!("Opcode C.ADDIW (x{}, offset {:x})", rd, value);
                let result = self.registers.read_reg(rd).wrapping_add(self.sign_extend_32_64(value));
                self.registers.write_reg(rd, result as u64);
                self.csr.advance();
                self.pc.wrapping_add(2)
            }            
            Instruction::CXADDI4SPN(rd, value) => {
                println!("Opcode C.ADDI4SPN (x{}, offset {:x})", rd, value);
                let result = self.registers.read_reg(2).wrapping_add(value as u64);
                self.registers.write_reg(rd, result);
                self.csr.advance();
                self.pc.wrapping_add(2)
            }            
            Instruction::CXADD(rs2, rd) => {
                println!("Opcode: C.ADD");
                let result = self.registers.read_reg(rd).wrapping_add(self.registers.read_reg(rs2)); 
                self.registers.write_reg(rd, result);
                self.csr.advance();
                self.pc.wrapping_add(2)
            }
            Instruction::CXMV(rs2, rd) => {
                println!("Opcode: C.MV");
                let rs2_val = self.registers.read_reg(rs2);
                self.registers.write_reg(rd, rs2_val);
                self.csr.advance();
                self.pc.wrapping_add(2)
            }
            Instruction::CXOR(rs2, rd) => {
                println!("Opcode: C.OR");
                let result = self.registers.read_reg(rs2) | self.registers.read_reg(rd);
                self.registers.write_reg(rd, result);
                self.csr.advance();
                self.pc.wrapping_add(2)                
            }            
            Instruction::CXAND(rs2, rd) => {
                println!("Opcode: C.AND");
                let result = self.registers.read_reg(rs2) & self.registers.read_reg(rd);
                self.registers.write_reg(rd, result);
                self.csr.advance();
                self.pc.wrapping_add(2)                
            }
            Instruction::CXJR(rs1) => {
                println!("Opcode: C.JR");
                self.pc = self.registers.read_reg(rs1);
                self.csr.advance();
                self.pc
            }
            Instruction::CXLDSP(rd, imm) => {
                let address = self.registers.read_reg(2).wrapping_add(self.sign_extend_32_64(imm));
                self.registers.write_reg(rd, self.bus.read_dword(address));
                self.csr.advance();
                self.pc.wrapping_add(2)
            }
            Instruction::CXSD(rs1, rs2, offset) => {
                let address = self.registers.read_reg(rs1).wrapping_add(self.sign_extend_32_64(offset));
                self.registers.write_reg(rs2, address);
                self.csr.advance();
                self.pc.wrapping_add(2)
            }
            Instruction::CXSDSP(rs2, imm) => {
                println!("Opcode: C.SDSP");
                let address = self.registers.read_reg(2).wrapping_add(self.sign_extend_32_64(imm));
                let stored = self.registers.read_reg(rs2);
                self.bus.write_dword(address, stored);
                self.csr.advance();
                self.pc.wrapping_add(2)
            }
            Instruction::CXSLLI(rd, imm) => {
                let rd_val = self.registers.read_reg(rd);
                let result = rd_val.wrapping_shl(imm as u32);
                self.registers.write_reg(rd, result);
                self.csr.advance();
                self.pc.wrapping_add(4)
            }            
            _ => panic!("Instruction not implemented")
        }
    }
}
struct MemoryBus {
    boot_rom: Box<[u8]>,
    ram: Box<[u8]>,
}

impl MemoryBus {
    fn new(boot_buffer: Vec<u8>) -> MemoryBus {
        let mut ram = vec![0; MAX_RAM_SIZE].into_boxed_slice();
        let boot_rom = boot_buffer.into_boxed_slice();

        let limit = boot_rom.len();

        ram[..limit].copy_from_slice(&boot_rom[..limit]);

        MemoryBus {
            boot_rom,
            ram,
        }
    }
    fn read_byte(&self, address: u64) -> u8 {
        let address = address as usize;
        match address {
            ROM_START ..= ROM_END => {
                let rom_addr = address.wrapping_sub(ROM_START);
                if rom_addr > self.boot_rom.len() {
                    0
                } else {
                    self.boot_rom[rom_addr]
                }
            }
            RAM_START ..= RAM_END => {
                self.ram[address-RAM_START] 
            }
            _ => {
                println!("Reading byte from unknown memory at address 0x{:x}", address);
                0xFF
            }
        }
    }
    fn read_word(&self, address: u64) -> u32 {
        let address = address as usize;
        match address {
            ROM_START ..= ROM_END => {
                let rom_addr = address.wrapping_sub(ROM_START);
                if rom_addr > self.boot_rom.len() {
                    0
                } else {
                    let word =  (self.boot_rom[address-ROM_START+0] as u32) << 0 | 
                                (self.boot_rom[address-ROM_START+1] as u32) << 8 | 
                                (self.boot_rom[address-ROM_START+2] as u32) << 16 | 
                                (self.boot_rom[address-ROM_START+3] as u32) << 24;
                    word
                }
            }
            RAM_START ..= RAM_END => {
                let word =  (self.ram[address-RAM_START+0] as u32) << 0 | 
                            (self.ram[address-RAM_START+1] as u32) << 8 | 
                            (self.ram[address-RAM_START+2] as u32) << 16 | 
                            (self.ram[address-RAM_START+3] as u32) << 24;
                word
            }
            _ => {
                //println!("Reading from unknown memory at address 0x{:x}", address);
                0xFFFF_FFFF
            }
        }
    }

    fn read_dword(&self, address: u64) -> u64 {
        let address = address as usize;
        match address {
            ROM_START ..= ROM_END => {
                let rom_addr = address.wrapping_sub(ROM_START);
                if rom_addr > self.boot_rom.len() {
                    0
                } else {
                    let word =  (self.boot_rom[rom_addr+0] as u64) << 0 | 
                                (self.boot_rom[rom_addr+1] as u64) << 8 | 
                                (self.boot_rom[rom_addr+2] as u64) << 16 | 
                                (self.boot_rom[rom_addr+3] as u64) << 24 | 
                                (self.boot_rom[rom_addr+4] as u64) << 32 | 
                                (self.boot_rom[rom_addr+5] as u64) << 40 | 
                                (self.boot_rom[rom_addr+6] as u64) << 48 | 
                                (self.boot_rom[rom_addr+7] as u64) << 56;
                    word
                }

            }
            RAM_START ..= RAM_END => {
                let word =  (self.ram[address-RAM_START+0] as u64) << 0 | 
                            (self.ram[address-RAM_START+1] as u64) << 8 | 
                            (self.ram[address-RAM_START+2] as u64) << 16 | 
                            (self.ram[address-RAM_START+3] as u64) << 24 | 
                            (self.ram[address-RAM_START+4] as u64) << 32 | 
                            (self.ram[address-RAM_START+5] as u64) << 40 | 
                            (self.ram[address-RAM_START+6] as u64) << 48 | 
                            (self.ram[address-RAM_START+7] as u64) << 56;
                word
            }
            _ => {
                //println!("Reading from unknown memory at address 0x{:x}", address);
                0xFFFF_FFFF_FFFF_FFFF
            }
        }

    }
    fn write_byte(&mut self, address: u64, value: u8) {
        let address = address as usize;
        match address {
            RAM_START ..= RAM_END => {
                self.ram[address-RAM_START] = value;
            }
            _ => {
                println!("Attempting to write to unknown memory at address 0x{:x}", address);
            }
        }
    }
    /*
    fn write_word(&mut self, address: u64, value: u32) {
        let address = address as usize;
        match address {
            ROM_START ..= ROM_END => {
                self.boot_rom[address-ROM_START+0] = ((value << 0) & 0xFF) as u8;
                self.boot_rom[address-ROM_START+1] = ((value << 8) & 0xFF) as u8; 
                self.boot_rom[address-ROM_START+2] = ((value << 16) & 0xFF) as u8;
                self.boot_rom[address-ROM_START+3] = ((value << 24) & 0xFF) as u8; 
            }
            RAM_START ..= RAM_END => {
                self.ram[address-RAM_START+0] = ((value << 0) & 0xFF) as u8;
                self.ram[address-RAM_START+1] = ((value << 8) & 0xFF) as u8; 
                self.ram[address-RAM_START+2] = ((value << 16) & 0xFF) as u8;
                self.ram[address-RAM_START+3] = ((value << 24) & 0xFF) as u8; 
            }
            _ => {
                println!("Attempting to write to unknown memory at address 0x{:x}", address);
            }
        }
    }
    */
    fn write_dword(&mut self, address: u64, value: u64) {
        let address = address as usize;
        match address {
            RAM_START ..= RAM_END => {
                self.ram[address-RAM_START+0] = ((value << 0) & 0xFF) as u8;
                self.ram[address-RAM_START+1] = ((value << 8) & 0xFF) as u8; 
                self.ram[address-RAM_START+2] = ((value << 16) & 0xFF) as u8;
                self.ram[address-RAM_START+3] = ((value << 24) & 0xFF) as u8;
                self.ram[address-RAM_START+4] = ((value << 32) & 0xFF) as u8;
                self.ram[address-RAM_START+5] = ((value << 40) & 0xFF) as u8; 
                self.ram[address-RAM_START+6] = ((value << 48) & 0xFF) as u8;
                self.ram[address-RAM_START+7] = ((value << 56) & 0xFF) as u8; 
            }
            _ => {
                println!("Attempting to write to unknown memory at address 0x{:x}", address);
            }
        }
    }
}

fn run(mut cpu: CPU) {
    cpu.reset();
    while !cpu.is_halted {
        cpu.step();
    }
}

fn main() {
    let matches = App::new("Praxis PC Emulator")
        .author("Michelle Darcy <mdarcy137@gmail.com")
        .arg(Arg::with_name("kernel")
            .short("b")
            .required(true)
            .default_value("./sys/kernel")
            .value_name("FILE"))
        .get_matches();

    let kernel = matches.value_of("kernel").map(|path| buffer_from_file(path)).unwrap();

    let cpu = CPU::new(kernel);
    run(cpu);

    println!("Exiting...");
}

fn buffer_from_file(path: &str) -> Vec<u8> {
    let mut file = std::fs::File::open(path).expect("File not present");
    let mut buffer = Vec::new();
    file.read_to_end(&mut buffer).expect("Could not read file");
    buffer
}
