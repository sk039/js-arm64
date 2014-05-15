// -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
// vim: set ts=8 sts=4 et sw=4 tw=99:
//
// Copyright 2013, ARM Limited
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//   * Neither the name of ARM Limited nor the names of its contributors may be
//     used to endorse or promote products derived from this software without
//     specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef jit_arm64_Architecture_arm64_h
#define jit_arm64_Architecture_arm64_h

namespace js {
namespace jit {

// In bytes: slots needed for potential memory->memory move spills.
//   +8 for cycles
//   +8 for gpr spills
//   +8 for double spills
static const uint32_t ION_FRAME_SLACK_SIZE = 24;

// TODO: Cribbed from x86_64. Check if this is right.
static const uint32_t ShadowStackSpace = 32;

// AArch64 has 32 64-bit integer registers, X0 though X31.
//  X31 is special and functions as both the stack pointer and a zero register.
//  The bottom 32 bits of each of the X register is accessible as W0 through W31.
//  The program counter is no longer accessible as a register.
// SIMD and scalar floating-point registers share a register bank.
//  32 bit float registers are S0 through S31.
//  64 bit double registers are D0 through D31.
//  128 bit SIMD registers are V0 through V31.
// e.g., S0 is the bottom 32 bits of D0, which is the bottom 64 bits of V0.

class Registers {
  public:
    enum RegisterID {
        w0 = x0 = 0,
        w1 = x1,
        w2 = x2,
        w3 = x3,
        w4 = x4,
        w5 = x5,
        w6 = x6,
        w7 = x7,
        w8 = x8,
        w9 = x9,
        w10 = x10,
        w11 = x11,
        w12 = x12,
        w13 = x13,
        w14 = x14,
        w15 = x15,
        ip0 = w16 = x16,
        ip1 = w17 = x17,
        w18 = x18,
        w19 = x19,
        w20 = x20,
        w21 = x21,
        w22 = x22,
        w23 = x23,
        w24 = x24,
        w25 = x25,
        w26 = x26,
        w27 = x27,
        w28 = x28,
        w29 = x29,
        lr = w30 = x30,
        sp = wzr = xzr = w31 = x31, // Special: both stack pointer and a zero register.
        invalid_reg
    }
    typedef RegisterID Code;

    static const char *GetName(Code code) {
        static const char *const Names[] =
            { "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8", "x9",
              "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "x18", "x19",
              "x20", "x21", "x22", "x23", "x24", "x25", "x26", "x27", "x28", "x29",
              "lr", "x31", "invalid" };
        return Names[code];
    }
    static const char *GetName(uint32_t i) {
        MOZ_ASSERT(i < Total);
        return GetName(Code(i));
    }

    static const Code StackPointer = sp;
    static const code Invalid = invalid_reg;

    static const uint32_t Total = 32;
    static const uint32_t Allocatable = 28; // No named special-function registers.

    static const uint32_t AllMask = (1 << Total) - 1;

    static const uint32_t NonAllocatableMask =
        (1 << Registers::ip0) | // scratch
        (1 << Registers::ip1) | // scratch
        (1 << Registers::lr) |
        (1 << Registers::sp);
};

} // namespace jit
} // namespace js

#endif // jit_arm64_Architecture_arm64_h
