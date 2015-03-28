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

#ifndef A64_ASSEMBLER_A64_H_
#define A64_ASSEMBLER_A64_H_

#include "jit/arm64/vixl/Assembler-vixl.h"

#include "jit/JitCompartment.h"

namespace js {
namespace jit {
static const uint32_t AlignmentAtPrologue = 0;
static const uint32_t AlignmentMidPrologue = 8;
static const Scale ScalePointer = TimesEight;
static const uint32_t AlignmentAtAsmJSPrologue = sizeof(void*);

static MOZ_CONSTEXPR_VAR Register ScratchReg = { Registers::ip0 };
static MOZ_CONSTEXPR_VAR ARMRegister ScratchReg64 = { ScratchReg, 64 };
static MOZ_CONSTEXPR_VAR ARMRegister ScratchReg32 = { ScratchReg, 32 };

static MOZ_CONSTEXPR_VAR Register ScratchReg2 = { Registers::ip1 };
static MOZ_CONSTEXPR_VAR ARMRegister ScratchReg2_64 = { ScratchReg2, 64 };
static MOZ_CONSTEXPR_VAR ARMRegister ScratchReg2_32 = { ScratchReg2, 32 };

static MOZ_CONSTEXPR_VAR FloatRegister ScratchDoubleReg = { FloatRegisters::d31 };
static MOZ_CONSTEXPR_VAR FloatRegister ReturnDoubleReg = { FloatRegisters::d0 };

static MOZ_CONSTEXPR_VAR FloatRegister ReturnFloat32Reg = { FloatRegisters::s0 , FloatRegisters::Single };
static MOZ_CONSTEXPR_VAR FloatRegister ScratchFloat32Reg = { FloatRegisters::s31 , FloatRegisters::Single };

static MOZ_CONSTEXPR_VAR Register InvalidReg = { Registers::invalid_reg };
static MOZ_CONSTEXPR_VAR FloatRegister InvalidFloatReg = { FloatRegisters::invalid_fpreg };

// TODO: these should probably have better names....
static MOZ_CONSTEXPR_VAR ARMFPRegister ScratchDoubleReg_ = { ScratchDoubleReg, 64 };
static MOZ_CONSTEXPR_VAR ARMFPRegister ReturnDoubleReg_ = { ReturnDoubleReg, 64 };

static MOZ_CONSTEXPR_VAR ARMFPRegister ReturnFloat32Reg_ = { ReturnFloat32Reg, 32 };
static MOZ_CONSTEXPR_VAR ARMFPRegister ScratchFloat32Reg_ = { ScratchFloat32Reg, 32 };

static MOZ_CONSTEXPR_VAR FloatRegister ReturnInt32x4Reg = InvalidFloatReg;
static MOZ_CONSTEXPR_VAR FloatRegister ReturnFloat32x4Reg = InvalidFloatReg;

static MOZ_CONSTEXPR_VAR Register OsrFrameReg = { Registers::x3 };
static MOZ_CONSTEXPR_VAR Register ArgumentsRectifierReg = { Registers::x8 };
static MOZ_CONSTEXPR_VAR Register CallTempReg0 = { Registers::x9 };
static MOZ_CONSTEXPR_VAR Register CallTempReg1 = { Registers::x10 };
static MOZ_CONSTEXPR_VAR Register CallTempReg2 = { Registers::x11 };
static MOZ_CONSTEXPR_VAR Register CallTempReg3 = { Registers::x12 };
static MOZ_CONSTEXPR_VAR Register CallTempReg4 = { Registers::x13 };
static MOZ_CONSTEXPR_VAR Register CallTempReg5 = { Registers::x14 };

static MOZ_CONSTEXPR_VAR Register PreBarrierReg = { Registers::x1 };


static MOZ_CONSTEXPR_VAR Register ReturnReg_ = { Registers::x0 };
static MOZ_CONSTEXPR_VAR Register ReturnReg = { Registers::x0 };
static MOZ_CONSTEXPR_VAR Register JSReturnReg = { Registers::x2 };
static MOZ_CONSTEXPR_VAR Register FramePointer = { Registers::fp };
static MOZ_CONSTEXPR_VAR Register ZeroRegister = { Registers::sp };
static MOZ_CONSTEXPR_VAR ARMRegister ZeroRegister64 = { Registers::sp, 64 };
static MOZ_CONSTEXPR_VAR ARMRegister ZeroRegister32 = { Registers::sp, 32 };

static MOZ_CONSTEXPR_VAR FloatRegister ReturnFloatReg = { FloatRegisters::d0 };
static MOZ_CONSTEXPR_VAR FloatRegister ScratchFloatReg = { FloatRegisters::d31 };

static MOZ_CONSTEXPR_VAR FloatRegister ReturnSimdReg = InvalidFloatReg;
static MOZ_CONSTEXPR_VAR FloatRegister ScratchSimdReg = InvalidFloatReg;

// StackPointer is intentionally undefined on ARM64 to prevent misuse:
//  using sp as a base register is only valid if sp % 16 == 0.
static MOZ_CONSTEXPR_VAR Register RealStackPointer = { Registers::sp };
// TODO: We're not quite there yet.
static MOZ_CONSTEXPR_VAR Register StackPointer = { Registers::sp };

static MOZ_CONSTEXPR_VAR Register PseudoStackPointer = { Registers::x28 };
static MOZ_CONSTEXPR_VAR ARMRegister PseudoStackPointer64 = { Registers::x28, 64 };
static MOZ_CONSTEXPR_VAR ARMRegister PseudoStackPointer32 = { Registers::x28, 32 };

// StackPointer for use by irregexp.
static MOZ_CONSTEXPR_VAR Register RegExpStackPointer = PseudoStackPointer;

static MOZ_CONSTEXPR_VAR Register IntArgReg0 = { Registers::x0 };
static MOZ_CONSTEXPR_VAR Register IntArgReg1 = { Registers::x1 };
static MOZ_CONSTEXPR_VAR Register IntArgReg2 = { Registers::x2 };
static MOZ_CONSTEXPR_VAR Register IntArgReg3 = { Registers::x3 };
static MOZ_CONSTEXPR_VAR Register IntArgReg4 = { Registers::x4 };
static MOZ_CONSTEXPR_VAR Register IntArgReg5 = { Registers::x5 };
static MOZ_CONSTEXPR_VAR Register IntArgReg6 = { Registers::x6 };
static MOZ_CONSTEXPR_VAR Register IntArgReg7 = { Registers::x7 };
static MOZ_CONSTEXPR_VAR Register GlobalReg =  { Registers::x20 };
static MOZ_CONSTEXPR_VAR Register HeapReg = { Registers::x21 };
static MOZ_CONSTEXPR_VAR Register HeapLenReg = { Registers::x22 };

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
static MOZ_CONSTEXPR_VAR Register r10 = { Registers::x10 };
static MOZ_CONSTEXPR_VAR Register r11 = { Registers::x11 };
static MOZ_CONSTEXPR_VAR Register r12 = { Registers::x12 };
static MOZ_CONSTEXPR_VAR Register r13 = { Registers::x13 };
static MOZ_CONSTEXPR_VAR Register r14 = { Registers::x14 };
static MOZ_CONSTEXPR_VAR Register r15 = { Registers::x15 };
static MOZ_CONSTEXPR_VAR Register r16 = { Registers::x16 };
static MOZ_CONSTEXPR_VAR Register ip0 = { Registers::x16 };
static MOZ_CONSTEXPR_VAR Register r17 = { Registers::x17 };
static MOZ_CONSTEXPR_VAR Register ip1 = { Registers::x16 };
static MOZ_CONSTEXPR_VAR Register r18 = { Registers::x18 };
static MOZ_CONSTEXPR_VAR Register r19 = { Registers::x19 };
static MOZ_CONSTEXPR_VAR Register r20 = { Registers::x20 };
static MOZ_CONSTEXPR_VAR Register r21 = { Registers::x21 };
static MOZ_CONSTEXPR_VAR Register r22 = { Registers::x22 };
static MOZ_CONSTEXPR_VAR Register r23 = { Registers::x23 };
static MOZ_CONSTEXPR_VAR Register r24 = { Registers::x24 };
static MOZ_CONSTEXPR_VAR Register r25 = { Registers::x25 };
static MOZ_CONSTEXPR_VAR Register r26 = { Registers::x26 };
static MOZ_CONSTEXPR_VAR Register r27 = { Registers::x27 };
static MOZ_CONSTEXPR_VAR Register r28 = { Registers::x28 };
static MOZ_CONSTEXPR_VAR Register r29 = { Registers::x29 };
static MOZ_CONSTEXPR_VAR Register fp  = { Registers::x30 };
static MOZ_CONSTEXPR_VAR Register r30 = { Registers::x30 };
static MOZ_CONSTEXPR_VAR Register lr  = { Registers::x30 };
static MOZ_CONSTEXPR_VAR Register r31 = { Registers::xzr };
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
// N.B. r8 isn't listed as an aapcs temp register, but we can use it as such because we never
// use return-structs.
static MOZ_CONSTEXPR_VAR Register CallTempNonArgRegs[] = { r8, r9, r10, r11, r12, r13, r14, r15 };
static const uint32_t NumCallTempNonArgRegs =
    mozilla::ArrayLength(CallTempNonArgRegs);

static MOZ_CONSTEXPR_VAR uint32_t JitStackAlignment = 16;

static MOZ_CONSTEXPR_VAR uint32_t JitStackValueAlignment = JitStackAlignment / sizeof(Value);
static_assert(JitStackAlignment % sizeof(Value) == 0 && JitStackValueAlignment >= 1,
  "Stack alignment should be a non-zero multiple of sizeof(Value)");

// This boolean indicates whether we support SIMD instructions flavoured for
// this architecture or not. Rather than a method in the LIRGenerator, it is
// here such that it is accessible from the entire codebase. Once full support
// for SIMD is reached on all tier-1 platforms, this constant can be deleted.
static MOZ_CONSTEXPR_VAR bool SupportsSimd = false;
static MOZ_CONSTEXPR_VAR uint32_t SimdMemoryAlignment = 16;

static_assert(CodeAlignment % SimdMemoryAlignment == 0,
  "Code alignment should be larger than any of the alignments which are used for "
  "the constant sections of the code buffer.  Thus it should be larger than the "
  "alignment for SIMD constants.");

static const uint32_t AsmJSStackAlignment = SimdMemoryAlignment;

static const int32_t AsmJSGlobalRegBias = 1024;
class Assembler : public AssemblerVIXL
{
  public:
    Assembler()
      : AssemblerVIXL()
    { }

    void finish();
    void trace(JSTracer *trc);

    // Emit the jump table, returning the BufferOffset to the first entry in the table.
    BufferOffset emitExtendedJumpTable();
    BufferOffset ExtendedJumpTable_;
    void executableCopy(uint8_t *buffer);

    BufferOffset immPool(ARMRegister dest, uint8_t *value, LoadLiteralOp op, ARMBuffer::PoolEntry *pe = nullptr);
    BufferOffset immPool64(ARMRegister dest, uint64_t value, ARMBuffer::PoolEntry *pe = nullptr);
    BufferOffset immPool64Branch(RepatchLabel *label, ARMBuffer::PoolEntry *pe, Condition c);
    BufferOffset fImmPool(ARMFPRegister dest, uint8_t *value, LoadLiteralOp op);
    BufferOffset fImmPool64(ARMFPRegister dest, double value);
    BufferOffset fImmPool32(ARMFPRegister dest, float value);

    void bind(Label *label) { bind(label, nextOffset()); }
    void bind(Label *label, BufferOffset boff);
    void bind(RepatchLabel* label);

    bool oom() const {
        // FIXME: Currently not possible to OOM.
        return false;
    }

    void copyJumpRelocationTable(uint8_t *dest) const {
        if (jumpRelocations_.length())
            memcpy(dest, jumpRelocations_.buffer(), jumpRelocations_.length());
    }
    void copyDataRelocationTable(uint8_t *dest) const {
        if (dataRelocations_.length())
            memcpy(dest, dataRelocations_.buffer(), dataRelocations_.length());
    }
    void copyPreBarrierTable(uint8_t *dest) const {
        if (preBarriers_.length())
            memcpy(dest, preBarriers_.buffer(), preBarriers_.length());
    }

    size_t jumpRelocationTableBytes() const {
        return jumpRelocations_.length();
    }
    size_t dataRelocationTableBytes() const {
        return dataRelocations_.length();
    }
    size_t preBarrierTableBytes() const {
        return preBarriers_.length();
    }
    size_t bytesNeeded() const {
        return SizeOfCodeGenerated() +
            jumpRelocationTableBytes() +
            dataRelocationTableBytes() +
            preBarrierTableBytes();
    }

    BufferOffset nextOffset() {
        return armbuffer_.nextOffset();
    }

    void addCodeLabel(CodeLabel label) {
        propagateOOM(codeLabels_.append(label));
    }
    size_t numCodeLabels() const {
        return codeLabels_.length();
    }
    CodeLabel codeLabel(size_t i) {
        return codeLabels_[i];
    }
    void processCodeLabels(uint8_t *rawCode) {
        for (size_t i = 0; i < codeLabels_.length(); i++) {
            CodeLabel label = codeLabels_[i];
            Bind(rawCode, label.dest(), rawCode + actualOffset(label.src()->offset()));
        }
    }

    void Bind(uint8_t *rawCode, AbsoluteLabel *label, const void *address) {
        uint32_t off = actualOffset(label->offset());
        *reinterpret_cast<const void **>(rawCode + off) = address;
    }
    bool nextLink(BufferOffset cur, BufferOffset *next) {
        Instruction *link = getInstructionAt(cur);
        uint32_t nextLinkOffset = uint32_t(link->ImmPCRawOffset());
        if (nextLinkOffset == uint32_t(LabelBase::INVALID_OFFSET))
            return false;
        *next = BufferOffset(nextLinkOffset + cur.getOffset());
        return true;
    }
    void retarget(Label *cur, Label *next);
    // Move our entire pool into the instruction stream. This is to force an
    // opportunistic dump of the pool, preferrably when it is more convenient
    // to do a dump.
    void flushBuffer() {
        armbuffer_.flushPool();
    }
    void enterNoPool(size_t maxInst) {
        armbuffer_.enterNoPool(maxInst);
    }
    void leaveNoPool() {
        armbuffer_.leaveNoPool();
    }

    // The buffer is about to be linked. Ensure any constant pools or
    // excess bookkeeping has been flushed to the instruction stream.
    void flush() {
        // TODO: MOZ_ASSERT(!isFinished);
        armbuffer_.flushPool();
    }

    int actualOffset(int curOffset) {
        return curOffset + armbuffer_.poolSizeBefore(curOffset);
    }
    int actualIndex(int curOffset) {
        ARMBuffer::PoolEntry pe(curOffset);
        return armbuffer_.poolEntryOffset(pe);
    }
    int labelOffsetToPatchOffset(int labelOff) {
        return actualOffset(labelOff);
    }
    static uint8_t *PatchableJumpAddress(JitCode *code, uint32_t index) {
        return code->raw() + index;
    }
    void setPrinter(Sprinter *sp) {
    }

    static bool SupportsFloatingPoint() { return true; }
    static bool SupportsSimd() { return js::jit::SupportsSimd; }

    // Tracks a jump that is patchable after finalization.
    void addJumpRelocation(BufferOffset src, Relocation::Kind reloc);

  protected:
    // Add a jump whose target is unknown until finalization.
    // The jump may not be patched at runtime.
    void addPendingJump(BufferOffset src, ImmPtr target, Relocation::Kind kind);

    // Add a jump whose target is unknown until finalization, and may change
    // thereafter. The jump is patchable at runtime.
    size_t addPatchableJump(BufferOffset src, Relocation::Kind kind);

  public:
    static uint32_t PatchWrite_NearCallSize() {
        return 4;
    }

    static uint32_t NopSize() {
        return 4;
    }

    static void PatchWrite_NearCall(CodeLocationLabel start, CodeLocationLabel toCall) {
        Instruction *dest = (Instruction*)start.raw();
        //printf("patching %p with call to %p\n", start.raw(), toCall.raw());
        bl(dest, ((Instruction*)toCall.raw() - dest)>>2);

    }
    static void PatchDataWithValueCheck(CodeLocationLabel label,
                                        PatchedImmPtr newValue,
                                        PatchedImmPtr expected);

    static void PatchDataWithValueCheck(CodeLocationLabel label,
                                        ImmPtr newValue,
                                        ImmPtr expected);

    static void PatchWrite_Imm32(CodeLocationLabel label, Imm32 imm) {
        // Raw is going to be the return address.
        uint32_t *raw = (uint32_t*)label.raw();
        // Overwrite the 4 bytes before the return address, which will end up being
        // the call instruction.
        *(raw - 1) = imm.value;
    }
    static uint32_t AlignDoubleArg(uint32_t offset) {
        MOZ_CRASH("AlignDoubleArg()");
    }
    static Instruction* NextInstruction(Instruction *instruction, uint32_t *count = nullptr) {
        if (count != nullptr)
            *count += 4;
        Instruction *cur = instruction;
        Instruction *next = cur + 4;
        // Artificial pool guards can only be B (rather than BR)
        if (next->IsUncondB()) {
            uint32_t *snd = (uint32_t*)(instruction + 8);
            // test both the upper 16 bits, but also bit 15, which should be unset
            // for an artificial branch guard.
            if ((*snd & 0xffff8000) == 0xffff0000) {
                // that was a guard before a pool, step over the pool.
                int poolSize =  (*snd & 0x7fff);
                return (Instruction*)(snd + poolSize);
            }
        } else if (cur->IsBR() || cur->IsUncondB()) {
            // natural pool guards can be anything
            // but they need to have bit 15 set.
            if ((next->InstructionBits() & 0xffff0000) == 0xffff0000) {
                int poolSize = (next->InstructionBits() & 0x7fff);
                Instruction *ret = (next + (poolSize << 2));
                return ret;
            }
        }
        return (instruction + 4);

    }
    static uint8_t *NextInstruction(uint8_t *instruction, uint32_t *count = nullptr) {
        return (uint8_t*)NextInstruction((Instruction*)instruction, count);
    }
    static uintptr_t GetPointer(uint8_t *ptr) {
        Instruction *i = reinterpret_cast<Instruction *>(ptr);
        uint64_t ret = i->Literal64();
        return ret;
    }

    // Toggle a jmp or cmp emitted by toggledJump().
    static void ToggleToJmp(CodeLocationLabel inst_);
    static void ToggleToCmp(CodeLocationLabel inst_);
    static void ToggleCall(CodeLocationLabel inst_, bool enabled);

    static void TraceJumpRelocations(JSTracer *trc, JitCode *code, CompactBufferReader &reader);
    static void TraceDataRelocations(JSTracer *trc, JitCode *code, CompactBufferReader &reader);

    static int32_t ExtractCodeLabelOffset(uint8_t *code);
    static void PatchInstructionImmediate(uint8_t *code, PatchedImmPtr imm);

    static void FixupNurseryObjects(JSContext *cx, JitCode *code, CompactBufferReader &reader,
                                    const ObjectVector &nurseryObjects);

    // Convert a BufferOffset to a final byte offset from the start of the code buffer.
    size_t toFinalOffset(BufferOffset offset) {
        return size_t(offset.getOffset() + armbuffer_.poolSizeBefore(offset.getOffset()));
    }

  public:
    // A Jump table entry is 2 instructions, with 8 bytes of raw data
    static const size_t SizeOfJumpTableEntry = 16;
    struct JumpTableEntry {
        uint32_t ldr;
        uint32_t br;
        void *data;
        Instruction *getLdr() {
            return reinterpret_cast<Instruction*>(&ldr);
        }
    };
    // Offset of the patchable target for the given entry.
    static const size_t OffsetOfJumpTableEntryPointer = 8;

  protected:
    // Because jumps may be relocated to a target inaccessible by a short jump,
    // each relocatable jump must have a unique entry in the extended jump table.
    // Valid relocatable targets are of type Relocation::JITCODE.
    struct JumpRelocation
    {
        BufferOffset jump; // Offset to the short jump, from the start of the code buffer.
        uint32_t extendedTableIndex; // Unique index within the extended jump table.

        JumpRelocation(BufferOffset jump, uint32_t extendedTableIndex)
          : jump(jump), extendedTableIndex(extendedTableIndex)
        { }
    };

    // Because ARM and A64 use a code buffer that allows for constant pool insertion,
    // the actual offset of each jump cannot be known until finalization.
    // These vectors store the WIP offsets.
    js::Vector<BufferOffset, 0, SystemAllocPolicy> tmpDataRelocations_;
    js::Vector<BufferOffset, 0, SystemAllocPolicy> tmpPreBarriers_;
    js::Vector<JumpRelocation, 0, SystemAllocPolicy> tmpJumpRelocations_;

    // Structure for fixing up pc-relative loads/jumps when the machine
    // code gets moved (executable copy, gc, etc.).
    struct RelativePatch
    {
        BufferOffset offset;
        void *target;
        Relocation::Kind kind;

        RelativePatch(BufferOffset offset, void *target, Relocation::Kind kind)
          : offset(offset), target(target), kind(kind)
        { }
    };

    js::Vector<CodeLabel, 0, SystemAllocPolicy> codeLabels_;

    // List of jumps for which the target is either unknown until finalization,
    // or cannot be known due to GC. Each entry here requires a unique entry
    // in the extended jump table, and is patched at finalization.
    js::Vector<RelativePatch, 8, SystemAllocPolicy> pendingJumps_;

    // Final output formatters.
    CompactBufferWriter jumpRelocations_;
    CompactBufferWriter dataRelocations_;
    CompactBufferWriter preBarriers_;
  public:
    static void UpdateBoundsCheck(uint32_t logHeapSize, Instruction *inst);
    void writeCodePointer(AbsoluteLabel *absoluteLabel) {
        MOZ_ASSERT(!absoluteLabel->bound());
        uintptr_t x = LabelBase::INVALID_OFFSET;
        BufferOffset off = EmitData(&x, sizeof(uintptr_t));

        // The x86/x64 makes general use of AbsoluteLabel and weaves a linked list
        // of uses of an AbsoluteLabel through the assembly. ARM only uses labels
        // for the case statements of switch jump tables. Thus, for simplicity, we
        // simply treat the AbsoluteLabel as a label and bind it to the offset of
        // the jump table entry that needs to be patched.
        LabelBase *label = absoluteLabel;
        label->bind(off.getOffset());
    }
    void verifyHeapAccessDisassembly(uint32_t begin, uint32_t end,
                                     const Disassembler::HeapAccess &heapAccess)
    {
        // Implement this if we implement a disassembler.
    }


};

class ABIArgGenerator
{
    static const int numIntArgRegs = 8;
    static const int numFloatArgRegs = 8;
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
    static const Register NonArgReturnReg0;
    static const Register NonArgReturnReg1;
    static const Register NonVolatileReg;
    static const Register NonArg_VolatileReg;
    static const Register NonReturn_VolatileReg0;
    static const Register NonReturn_VolatileReg1;

  protected:
    unsigned intRegIndex_;
    unsigned floatRegIndex_;
    uint32_t stackOffset_;
    ABIArg current_;
};

// FIXME: ugh. why is this not a static member of Assembler?
void PatchJump(CodeLocationJump &jump_, CodeLocationLabel label);

static inline void
PatchBackedge(CodeLocationJump &jump_, CodeLocationLabel label, JitRuntime::BackedgeTarget target)
{
    PatchJump(jump_, label);
}

static const uint32_t NumIntArgRegs = 8;
static const uint32_t NumFloatArgRegs = 8;

static inline bool
GetIntArgReg(uint32_t usedIntArgs, uint32_t usedFloatArgs, Register *out)
{
    if (usedIntArgs >= NumIntArgRegs)
        return false;
    *out = Register::FromCode(usedIntArgs);
    return true;
}

static inline bool
GetFloatArgReg(uint32_t usedIntArgs, uint32_t usedFloatArgs, FloatRegister *out)
{
    if (usedFloatArgs >= NumFloatArgRegs)
        return false;
    *out = FloatRegister::FromCode(usedFloatArgs);
    return true;
}

// Get a register in which we plan to put a quantity that will be used as an
// integer argument.  This differs from GetIntArgReg in that if we have no more
// actual argument registers to use we will fall back on using whatever
// CallTempReg* don't overlap the argument registers, and only fail once those
// run out too.
static inline bool
GetTempRegForIntArg(uint32_t usedIntArgs, uint32_t usedFloatArgs, Register *out)
{
    if (GetIntArgReg(usedIntArgs, usedFloatArgs, out))
        return true;
    // Unfortunately, we have to assume things about the point at which
    // GetIntArgReg returns false, because we need to know how many registers it
    // can allocate.
    usedIntArgs -= NumIntArgRegs;
    if (usedIntArgs >= NumCallTempNonArgRegs)
        return false;
    *out = CallTempNonArgRegs[usedIntArgs];
    return true;

}

// FIXME: Should be shared with ARM's Assembler.
class AutoForbidPools
{
    Assembler *asm_;

  public:
    AutoForbidPools(Assembler *asm_, size_t maxInst)
      : asm_(asm_)
    {
        asm_->enterNoPool(maxInst);
    }

    ~AutoForbidPools() {
        asm_->leaveNoPool();
    }
};


} // namespace jit
} // namespace js

#endif // A64_ASSEMBLER_A64_H_
