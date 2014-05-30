// -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
// vim: set ts=8 sts=2 et sw=2 tw=99:
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

#ifndef A64_ASSEMBLER_A64_H_
#define A64_ASSEMBLER_A64_H_

#include "jit/arm64/vixl/Assembler-vixl.h"

namespace js {
namespace jit {

class Assembler : public AssemblerVIXL {
  public:
    Assembler(byte* buffer, unsigned buffer_size)
      : AssemblerVIXL(buffer, buffer_size)
    { }

    void finish() {
        JS_ASSERT(0 && "finish()");
    }
    void executableCopy(void *buffer) {
        JS_ASSERT(0 && "executableCopy()");
    }
    void copyJumpRelocationTable(uint8_t *dest) {
        JS_ASSERT(0 && "copyJumpRelocationTable()");
    }
    void copyDataRelocationTable(uint8_t *dest) {
        JS_ASSERT(0 && "copyDataRelocationTable()");
    }
    void copyPreBarrierTable(uint8_t *dest) {
        JS_ASSERT(0 && "copyPreBarrierTable()");
    }

    bool addCodeLabel(CodeLabel label) {
        JS_ASSERT(0 && "addCodeLabel()");
        return false;
    }
    size_t numCodeLabels() const {
        return codeLabels_.length();
    }
    CodeLabel codeLabel(size_t i) {
        return codeLabels_[i];
    }
    void processCodeLabels(uint8_t *rawCode) {
        JS_ASSERT(0 && "processCodeLabels()");
    }

    // Size of the jump relocation table, in bytes.
    size_t jumpRelocationTableBytes() const {
        JS_ASSERT(0 && "jumpRelocationTableBytes()");
        return 0;
    }
    size_t dataRelocationTableBytes() const {
        JS_ASSERT(0 && "dataRelocationTableBytes()");
        return 0;
    }
    size_t preBarrierTableBytes() const {
        JS_ASSERT(0 && "preBarrierTableBytes()");
        return 0;
    }
    void flushBuffer() {
        JS_ASSERT(0 && "flushBuffer()");
    }
    unsigned int bytesNeeded() {
        return buffer_size_;
    }
    int actualOffset(int curOffset) {
        return curOffset;
    }
    int actualIndex(int curOffset) {
        return curOffset;
    }
    int labelOffsetToPatchOffset(int labelOff) {
        return labelOff;
    }
    static uint8_t *PatchableJumpAddress(JitCode *code, uint32_t index) {
        JS_ASSERT(0 && "patchableJumpAddress");
    }
    void executableCopy(unsigned char *code){
        JS_ASSERT(0 && "executableCopy");
    }
    void setPrinter(Sprinter *sp) {
        JS_ASSERT(0 && "setPrinter()");
    }

    static void TraceJumpRelocations(JSTracer *trc, JitCode *code, CompactBufferReader &reader) {
        JS_ASSERT(0 && "TraceJumpRelocations()");
    }
    static void TraceDataRelocations(JSTracer *trc, JitCode *code, CompactBufferReader &reader) {
        JS_ASSERT(0 && "TraceDataRelocations()");
    }

    static uint32_t patchWrite_NearCallSize() {
        JS_ASSERT(0 && "patchWrite_NearCallSize()");
        return 0;
    }

    static uint32_t nopSize() {
        JS_ASSERT(0 && "nopSize()");
        return 4;
    }
    static void patchWrite_NearCall(CodeLocationLabel start, CodeLocationLabel toCall) {
        JS_ASSERT(0 && "patchWrite_NearCall()");
    }
    static void patchDataWithValueCheck(CodeLocationLabel label, PatchedImmPtr newValue,
                                        PatchedImmPtr expectedValue)
    {
        JS_ASSERT(0 && "patchDataWithValueCheck()");
    }
    static void patchDataWithValueCheck(CodeLocationLabel label, ImmPtr newValue,
                                        ImmPtr expectedValue)
    {
        JS_ASSERT(0 && "patchDataWithValueCheck()");
    }
    static void patchWrite_Imm32(CodeLocationLabel label, Imm32 imm) {
        JS_ASSERT(0 && "patchWrite_Imm32()");
    }
    static uint32_t alignDoubleArg(uint32_t offset) {
        JS_ASSERT(0 && "alignDoubleArg()");
        return (offset+1)&~1;
    }
    static uint8_t *nextInstruction(uint8_t *instruction, uint32_t *count = nullptr) {
        JS_ASSERT(0 && "nextInstruction()");
        return nullptr;
    }
    static uintptr_t getPointer(uint8_t *) {
        JS_ASSERT(0 && "getPointer()");
        return 0;
    }

    // Toggle a jmp or cmp emitted by toggledJump().
    static void ToggleToJmp(CodeLocationLabel inst_) {
        JS_ASSERT(0 && "ToggleToJmp()");
    }
    static void ToggleToCmp(CodeLocationLabel inst_) {
        JS_ASSERT(0 && "ToggleToCmp()");
    }

    static void ToggleCall(CodeLocationLabel inst_, bool enabled) {
        JS_ASSERT(0 && "ToggleCall()");
    }

    static void updateBoundsCheck(uint32_t logHeapSize, Instruction *inst) {
        JS_ASSERT(0 && "updateBoundsCheck()");
    }

    js::Vector<CodeLabel, 0, SystemAllocPolicy> codeLabels_;

    CompactBufferWriter jumpRelocations_;
    CompactBufferWriter dataRelocations_;
    CompactBufferWriter relocations_;
    CompactBufferWriter preBarriers_;
};

class ABIArgGenerator
{
  public:
    ABIArgGenerator()
      : intRegIndex_(0),
        floatRegIndex_(0),
        stackOffset_(0),
        current_()
    { }

    ABIArg next(MIRType argType);
    ABIArg &current() { return current_; }
    uint32_t stackBytesConsumedSoFar() const { return stackOffset_; }

  public:
    static const Register NonArgReturnVolatileReg0;
    static const Register NonArgReturnVolatileReg1;

  protected:
    unsigned intRegIndex_;
    unsigned floatRegIndex_;
    uint32_t stackOffset_;
    ABIArg current_;
};

// ugh. why is this not a static member of Assembler?
void
PatchJump(CodeLocationJump &jump_, CodeLocationLabel label) {
    JS_ASSERT(0 && "PatchJump()");
}

static inline bool
GetIntArgReg(uint32_t usedIntArgs, uint32_t usedFloatArgs, Register *out)
{
    JS_ASSERT(0 && "TODO");
    return false;
}

// Get a register in which we plan to put a quantity that will be used as an
// integer argument.  This differs from GetIntArgReg in that if we have no more
// actual argument registers to use we will fall back on using whatever
// CallTempReg* don't overlap the argument registers, and only fail once those
// run out too.
static inline bool
GetTempRegForIntArg(uint32_t usedIntArgs, uint32_t usedFloatArgs, Register *out)
{
    JS_ASSERT(0 && "TODO");
    return false;
}

static const uint32_t AlignmentAtPrologue = 0;
static const uint32_t AlignmentMidPrologue = 8;
static const Scale ScalePointer = TimesEight;

static MOZ_CONSTEXPR_VAR ARMRegister ScratchRegister64 = { Registers::ip0, 64 };
static MOZ_CONSTEXPR_VAR ARMRegister ScratchRegister32 = { Registers::ip0, 32 };


static MOZ_CONSTEXPR_VAR Register OsrFrameReg = { Registers::x3};
static MOZ_CONSTEXPR_VAR Register ArgumentsRectifierReg = { Registers::x8};
static MOZ_CONSTEXPR_VAR Register CallTempReg0 = { Registers::x5 };
static MOZ_CONSTEXPR_VAR Register CallTempReg1 = { Registers::x6 };
static MOZ_CONSTEXPR_VAR Register CallTempReg2 = { Registers::x7 };
static MOZ_CONSTEXPR_VAR Register CallTempReg3 = { Registers::x8 };
static MOZ_CONSTEXPR_VAR Register CallTempReg4 = { Registers::x0 };
static MOZ_CONSTEXPR_VAR Register CallTempReg5 = { Registers::x1 };

static MOZ_CONSTEXPR_VAR Register PreBarrierReg = { Registers::x1 };

static MOZ_CONSTEXPR_VAR Register InvalidReg = { Registers::invalid_reg };
static MOZ_CONSTEXPR_VAR FloatRegister InvalidFloatReg = { FloatRegisters::invalid_fpreg };

static MOZ_CONSTEXPR_VAR Register ReturnReg_ = { Registers::x0 };
static MOZ_CONSTEXPR_VAR Register ReturnReg = { Registers::x0 };
static MOZ_CONSTEXPR_VAR Register JSReturnReg = { Registers::x2 };
static MOZ_CONSTEXPR_VAR Register FramePointer = { Registers::fp };
static MOZ_CONSTEXPR_VAR Register StackPointer = { Registers::sp };
static MOZ_CONSTEXPR_VAR FloatRegister ReturnFloatReg = { FloatRegisters::d0 };
static MOZ_CONSTEXPR_VAR FloatRegister ScratchFloatReg = { FloatRegisters::d31 };

static MOZ_CONSTEXPR_VAR Register IntArgReg0 = { Registers::x0 };
static MOZ_CONSTEXPR_VAR Register IntArgReg1 = { Registers::x1 };
static MOZ_CONSTEXPR_VAR Register IntArgReg2 = { Registers::x2 };
static MOZ_CONSTEXPR_VAR Register IntArgReg3 = { Registers::x3 };
static MOZ_CONSTEXPR_VAR Register GlobalReg =  { Registers::x10 };
static MOZ_CONSTEXPR_VAR Register HeapReg = { Registers::x11 };

static MOZ_CONSTEXPR_VAR Register r0 = { Registers::x0 };
static MOZ_CONSTEXPR_VAR Register r1 = { Registers::x1 };
static MOZ_CONSTEXPR_VAR Register r2 = { Registers::x2 };
static MOZ_CONSTEXPR_VAR Register r3 = { Registers::x3 };
static MOZ_CONSTEXPR_VAR Register r4 = { Registers::x4 };
static MOZ_CONSTEXPR_VAR Register r5 = { Registers::x5 };
static MOZ_CONSTEXPR_VAR Register r6 = { Registers::x6 };
static MOZ_CONSTEXPR_VAR Register r7 = { Registers::x7 };
static MOZ_CONSTEXPR_VAR Register r8 = { Registers::x8 };
static MOZ_CONSTEXPR_VAR Register r9 = { Registers::x9 };
static MOZ_CONSTEXPR_VAR Register r10 = { Registers::x10};
static MOZ_CONSTEXPR_VAR Register r11 = { Registers::x11};
static MOZ_CONSTEXPR_VAR Register r12 = { Registers::x12};
static MOZ_CONSTEXPR_VAR Register r13 = { Registers::x13};
static MOZ_CONSTEXPR_VAR Register r14 = { Registers::x14};
static MOZ_CONSTEXPR_VAR Register r15 = { Registers::x15};
static MOZ_CONSTEXPR_VAR Register r16 = { Registers::x16};
static MOZ_CONSTEXPR_VAR Register r17 = { Registers::x17};
static MOZ_CONSTEXPR_VAR Register r18 = { Registers::x18};
static MOZ_CONSTEXPR_VAR Register r19 = { Registers::x19};
static MOZ_CONSTEXPR_VAR Register r20 = { Registers::x20};
static MOZ_CONSTEXPR_VAR Register r21 = { Registers::x21};
static MOZ_CONSTEXPR_VAR Register r22 = { Registers::x22};
static MOZ_CONSTEXPR_VAR Register r23 = { Registers::x23};
static MOZ_CONSTEXPR_VAR Register r24 = { Registers::x24};
static MOZ_CONSTEXPR_VAR Register r25 = { Registers::x25};
static MOZ_CONSTEXPR_VAR Register r26 = { Registers::x26};
static MOZ_CONSTEXPR_VAR Register r27 = { Registers::x27};
static MOZ_CONSTEXPR_VAR Register r28 = { Registers::x28};
static MOZ_CONSTEXPR_VAR Register r29 = { Registers::x29};
static MOZ_CONSTEXPR_VAR Register r30 = { Registers::x30};
static MOZ_CONSTEXPR_VAR ValueOperand JSReturnOperand = ValueOperand(JSReturnReg);

// Registers used in the GenerateFFIIonExit Enable Activation block.
static MOZ_CONSTEXPR_VAR Register AsmJSIonExitRegCallee = r8;
static MOZ_CONSTEXPR_VAR Register AsmJSIonExitRegE0 = r0;
static MOZ_CONSTEXPR_VAR Register AsmJSIonExitRegE1 = r1;
static MOZ_CONSTEXPR_VAR Register AsmJSIonExitRegE2 = r2;
static MOZ_CONSTEXPR_VAR Register AsmJSIonExitRegE3 = r3;

// Registers used in the GenerateFFIIonExit Disable Activation block.
// None of these may be the second scratch register.
static MOZ_CONSTEXPR_VAR Register AsmJSIonExitRegReturnData = r2;
static MOZ_CONSTEXPR_VAR Register AsmJSIonExitRegReturnType = r3;
static MOZ_CONSTEXPR_VAR Register AsmJSIonExitRegD0 = r0;
static MOZ_CONSTEXPR_VAR Register AsmJSIonExitRegD1 = r1;
static MOZ_CONSTEXPR_VAR Register AsmJSIonExitRegD2 = r4;

static MOZ_CONSTEXPR_VAR Register JSReturnReg_Type = r3;
static MOZ_CONSTEXPR_VAR Register JSReturnReg_Data = r2;

static MOZ_CONSTEXPR_VAR FloatRegister NANReg = { FloatRegisters::d14 };
} // jit
} // js
#endif
