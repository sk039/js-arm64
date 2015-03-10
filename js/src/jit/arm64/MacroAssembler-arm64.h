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

#ifndef jit_arm64_MacroAssembler_arm64_h
#define jit_arm64_MacroAssembler_arm64_h

#include "jit/arm64/Assembler-arm64.h"
#include "jit/arm64/vixl/Debugger-vixl.h"
#include "jit/arm64/vixl/MacroAssembler-vixl.h"
#include "jit/arm64/vixl/VIXL-Globals-vixl.h"

#include "jit/AtomicOp.h"
#include "jit/JitFrames.h"
#include "jit/MoveResolver.h"

namespace js {
namespace jit {

struct ImmShiftedTag : public ImmWord
{
    ImmShiftedTag(JSValueShiftedTag shtag)
      : ImmWord((uintptr_t)shtag)
    { }

    ImmShiftedTag(JSValueType type)
      : ImmWord(uintptr_t(JSValueShiftedTag(JSVAL_TYPE_TO_SHIFTED_TAG(type))))
    { }
};

struct ImmTag : public Imm32
{
    ImmTag(JSValueTag tag)
      : Imm32(tag)
    { }
};

class MacroAssemblerCompat : public MacroAssemblerVIXL
{
  protected:
    bool enoughMemory_;
    uint32_t framePushed_;

    // TODO: Can this be moved out of the MacroAssembler and into some shared code?
    // TODO: All the code seems to be arch-independent, and it's weird to have this here.
    bool inCall_;
    bool usedOutParam_;
    uint32_t args_;
    uint32_t passedIntArgs_;
    uint32_t passedFloatArgs_;
    uint32_t passedArgTypes_;
    uint32_t stackForCall_;
    bool dynamicAlignment_;

    MacroAssemblerCompat()
      : MacroAssemblerVIXL(),
        enoughMemory_(true),
        framePushed_(0),
        inCall_(false),
        usedOutParam_(false),
        args_(0),
        passedIntArgs_(0),
        passedFloatArgs_(0),
        passedArgTypes_(0),
        stackForCall_(0),
        dynamicAlignment_(false)
    { }

  protected:
    MoveResolver moveResolver_;

  public:
    bool oom() const {
        // FIXME: jandem is trying to knock out enoughMemory_ now... needs rebasing.
        return Assembler::oom() || !enoughMemory_;
    }
    static MemOperand toMemOperand(Address &a) {
        return MemOperand(ARMRegister(a.base, 64), a.offset);
    }
    void doBaseIndex(const CPURegister &rt, const BaseIndex &addr, LoadStoreOp op) {
        // Can use ONLY an indexed-reg load
        if (addr.offset == 0) {
            if (addr.scale == 0 || addr.scale == static_cast<unsigned>(CalcLSDataSize(op))) {
                LoadStoreMacro(rt, MemOperand(ARMRegister(addr.base, 64),
                                              ARMRegister(addr.index, 64),
                                              LSL, unsigned(addr.scale)), op);
                return;
            } else {
                add(ScratchReg64, ARMRegister(addr.base, 64), Operand(ARMRegister(addr.index, 64), LSL, unsigned(addr.scale)));
                LoadStoreMacro(rt, MemOperand(ScratchReg64), op);
            }
        }

        // Store operations should not clobber.
        MOZ_ASSERT(!rt.Is(ScratchReg32));
        MOZ_ASSERT(!rt.Is(ScratchReg64));

        // TODO: should only add here when we can fit it into a single operand.
        Add(ScratchReg64,
            ARMRegister(addr.base, 64),
            Operand(ARMRegister(addr.index, 64),
                    LSL,
                    unsigned(addr.scale)));
        LoadStoreMacro(rt, MemOperand(ScratchReg64, addr.offset), op);
    }

    void Push(Register reg) {
        MacroAssemblerVIXL::Push(ARMRegister(reg, 64));
        adjustFrame(sizeof(intptr_t));
    }
    void Push(const Imm32 imm) {
        push(imm);
        adjustFrame(sizeof(intptr_t));
    }
    void Push(const ImmWord imm) {
        push(imm);
        adjustFrame(sizeof(intptr_t));
    }
    void Push(const ImmPtr imm) {
        push(imm);
        adjustFrame(sizeof(intptr_t));
    }
    void Push(const ImmGCPtr ptr) {
        push(ptr);
        adjustFrame(sizeof(intptr_t));
    }
    void Push(FloatRegister f) {
        push(f);
        adjustFrame(sizeof(double));
    }
    void Push(const ValueOperand &val) {
        MacroAssemblerVIXL::Push(ARMRegister(val.valueReg(), 64));
        adjustFrame(sizeof(void *));
    }

    void Pop(const Register t) {
        pop(t);
        adjustFrame(-1 * int64_t(sizeof(int64_t)));
    }
    void Pop(const ValueOperand t) {
        pop(t);
        adjustFrame(-1 * int64_t(sizeof(int64_t)));
    }

    void push(FloatRegister f) {
        MacroAssemblerVIXL::Push(ARMFPRegister(f, 64));
    }
    void push(Imm32 imm) {
        if (imm.value == 0) {
            MacroAssemblerVIXL::Push(xzr);
        } else {
            move32(imm, ScratchReg);
            MacroAssemblerVIXL::Push(ScratchReg64);
        }
    }
    void push(ImmWord imm) {
        if (imm.value == 0) {
            MacroAssemblerVIXL::Push(xzr);
        } else {
            Mov(ScratchReg64, imm.value);
            MacroAssemblerVIXL::Push(ScratchReg64);
        }
    }
    void push(ImmPtr imm) {
        if (imm.value == nullptr) {
            MacroAssemblerVIXL::Push(xzr);
        } else {
            movePtr(imm, ScratchReg);
            MacroAssemblerVIXL::Push(ScratchReg64);
        }
    }
    void push(ImmGCPtr imm) {
        if (imm.value == nullptr) {
            MacroAssemblerVIXL::Push(xzr);
        } else {
            movePtr(imm, ScratchReg);
            MacroAssemblerVIXL::Push(ScratchReg64);
        }
    }
    void push(ImmMaybeNurseryPtr imm) {
        push(noteMaybeNurseryPtr(imm));
    }
    void push(Register reg) {
        MacroAssemblerVIXL::Push(ARMRegister(reg, 64));
    }
    void push(ARMRegister reg) {
        MacroAssemblerVIXL::Push(reg);
    }
    void push(Address a) {
        loadPtr(a, ScratchReg);
        MacroAssemblerVIXL::Push(ScratchReg64);
    }

    void pushReturnAddress() {
        push(lr);
    }
    void popReturn() {
        pop(lr);
        ret();
    }
    void pop(const ValueOperand &v) {
        pop(v.valueReg());
    }
    void pop(const FloatRegister &f) {
        MacroAssemblerVIXL::Pop(ARMRegister(f.code(), 64));
    }
    void pop(Register reg) {
        MacroAssemblerVIXL::Pop(ARMRegister(reg, 64));
    }

    void implicitPop(uint32_t args) {
        MOZ_ASSERT(args % sizeof(intptr_t) == 0);
        adjustFrame(-args);
    }

    // FIXME: This is the same on every arch.
    // FIXME: If we can share framePushed_, we can share this.
    // FIXME: Or just make it at the highest level.
    CodeOffsetLabel PushWithPatch(ImmWord word) {
        framePushed_ += sizeof(word.value);
        return pushWithPatch(word);
    }
    CodeOffsetLabel PushWithPatch(ImmPtr ptr) {
        return PushWithPatch(ImmWord(uintptr_t(ptr.value)));
    }

    uint32_t framePushed() const {
        return framePushed_;
    }
    void adjustFrame(int32_t diff) {
        setFramePushed(framePushed_ + diff);
    }

    void setFramePushed(uint32_t framePushed) {
        framePushed_ = framePushed;
    }

    void reserveStack(uint32_t amount) {
        // TODO: This bumps |sp| every time we reserve using a second register.
        // It would save some instructions if we had a fixed, maximum frame size.
        MacroAssemblerVIXL::Claim(Operand(amount));
        adjustFrame(amount);
    }
    void freeStack(uint32_t amount) {
        MacroAssemblerVIXL::Drop(Operand(amount));
        adjustFrame(-((int32_t)amount));
    }
    void freeStack(Register amount) {
        MacroAssemblerVIXL::Drop(Operand(ARMRegister(amount, 64)));
    }

    // Update sp with the value of the current active stack pointer, if necessary.
    void syncStackPtr() {
        if (!GetStackPointer().Is(sp))
            Add(sp, GetStackPointer(), Operand(0));
    }
    void initStackPtr() {
        if (!GetStackPointer().Is(sp))
            Add(GetStackPointer(), sp, Operand(0));
    }
    void storeValue(ValueOperand val, const Address &dest) {
        storePtr(val.valueReg(), dest);
    }

    template <typename T>
    void storeValue(JSValueType type, Register reg, const T &dest) {
        tagValue(type, reg, ValueOperand(ScratchReg2));
        storeValue(ValueOperand(ScratchReg2), dest);
    }
    template <typename T>
    void storeValue(const Value &val, const T &dest) {
        moveValue(val, ValueOperand(ScratchReg2));
        storeValue(ValueOperand(ScratchReg2), dest);
    }
    void storeValue(ValueOperand val, BaseIndex dest) {
        storePtr(val.valueReg(), dest);
    }

    template <typename T>
    void storeUnboxedValue(ConstantOrRegister value, MIRType valueType, const T &dest,
                           MIRType slotType)
    {
        if (valueType == MIRType_Double) {
            storeDouble(value.reg().typedReg().fpu(), dest);
            return;
        }

        // For known integers and booleans, we can just store the unboxed value if
        // the slot has the same type.
        if ((valueType == MIRType_Int32 || valueType == MIRType_Boolean) && slotType == valueType) {
            if (value.constant()) {
                Value val = value.value();
                if (valueType == MIRType_Int32)
                    store32(Imm32(val.toInt32()), dest);
                else
                    store32(Imm32(val.toBoolean() ? 1 : 0), dest);
            } else {
                store32(value.reg().typedReg().gpr(), dest);
            }
            return;
        }

        if (value.constant())
            storeValue(value.value(), dest);
        else
            storeValue(ValueTypeFromMIRType(valueType), value.reg().typedReg().gpr(), dest);

    }
    void loadValue(Address src, Register val) {
        Ldr(ARMRegister(val, 64), MemOperand(src));
    }
    void loadValue(Address src, ValueOperand val) {
        Ldr(ARMRegister(val.valueReg(), 64), MemOperand(src));
    }
    void loadValue(const BaseIndex &src, ValueOperand val) {
        doBaseIndex(ARMRegister(val.valueReg(), 64), src, LDR_x);
    }
    void tagValue(JSValueType type, Register payload, ValueOperand dest) {
        // TODO: This could be more clever, but the first attempt had bugs.
        Orr(ARMRegister(dest.valueReg(), 64), ARMRegister(payload, 64), Operand(ImmShiftedTag(type).value));
    }
    void pushValue(ValueOperand val) {
        MacroAssemblerVIXL::Push(ARMRegister(val.valueReg(), 64));
    }
    void popValue(ValueOperand val) {
        MacroAssemblerVIXL::Pop(ARMRegister(val.valueReg(), 64));
    }
    void pushValue(const Value &val) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        if (val.isMarkable()) {
            BufferOffset load = movePatchablePtr(ImmPtr((void *)jv.asBits), ScratchReg2);
            writeDataRelocation(val, load);
            push(ScratchReg2);
        } else {
            moveValue(val, ScratchReg2);
            push(ScratchReg2);
        }
    }
    void pushValue(JSValueType type, Register reg) {
        tagValue(type, reg, ValueOperand(ScratchReg2));
        push(ScratchReg2);
    }
    void pushValue(const Address &addr) {
        loadValue(addr, ScratchReg2);
        push(ScratchReg2);
    }
    template <typename T>
    void storeUnboxedPayload(ValueOperand value, T address, size_t nbytes) {
        switch (nbytes) {
          case 4:
            storePtr(value.valueReg(), address);
            return;
          case 1:
            store8(value.valueReg(), address);
            return;
          default: MOZ_CRASH("Bad payload width");
        }
    }
    void moveValue(const Value &val, Register dest) {
        movePtr(ImmWord(val.asRawBits()), dest);
    }
    void moveValue(const Value &src, const ValueOperand &dest) {
        movePtr(ImmWord(src.asRawBits()), dest.valueReg());
    }
    void moveValue(const ValueOperand &src, const ValueOperand &dest) {
        if (src.valueReg() != dest.valueReg())
            movePtr(src.valueReg(), dest.valueReg());
    }

    CodeOffsetLabel pushWithPatch(ImmWord imm) {
        CodeOffsetLabel label = movWithPatch(imm, ScratchReg);
        push(ScratchReg);
        return label;
    }

    CodeOffsetLabel movWithPatch(ImmWord imm, Register dest) {
        BufferOffset off = immPool64(ARMRegister(dest, 64), imm.value);
        return CodeOffsetLabel(off.getOffset());
    }
    CodeOffsetLabel movWithPatch(ImmPtr imm, Register dest) {
        BufferOffset off = immPool64(ARMRegister(dest, 64), uint64_t(imm.value));
        return CodeOffsetLabel(off.getOffset());
    }

    void boxValue(JSValueType type, Register src, Register dest) {
        Orr(ARMRegister(dest, 64), ARMRegister(src, 64), Operand(ImmShiftedTag(type).value));
    }
    void splitTag(Register src, Register dest) {
        ubfx(ARMRegister(dest, 64), ARMRegister(src, 64), JSVAL_TAG_SHIFT, (64 - JSVAL_TAG_SHIFT));
    }
    Register extractTag(const Address &address, Register scratch) {
        loadPtr(address, scratch);
        splitTag(scratch, scratch);
        return scratch;
    }
    Register extractTag(const ValueOperand &value, Register scratch) {
        splitTag(value.valueReg(), scratch);
        return scratch;
    }
    Register extractObject(const Address &address, Register scratch) {
        loadPtr(address, scratch);
        unboxObject(scratch, scratch);
        return scratch;
    }
    Register extractObject(const ValueOperand &value, Register scratch) {
        unboxObject(value, scratch);
        return scratch;
    }
    Register extractInt32(const ValueOperand &value, Register scratch) {
        unboxInt32(value, scratch);
        return scratch;
    }
    Register extractBoolean(const ValueOperand &value, Register scratch) {
        unboxBoolean(value, scratch);
        return scratch;
    }

    // If source is a double, load into dest.
    // If source is int32, convert to double and store in dest.
    // Else, branch to failure.
    void ensureDouble(const ValueOperand &source, FloatRegister dest, Label *failure) {
        Label isDouble, done;
        Register tag = splitTagForTest(source);
        branchTestDouble(Assembler::Equal, tag, &isDouble);
        branchTestInt32(Assembler::NotEqual, tag, failure);

        convertInt32ToDouble(source.valueReg(), dest);
        jump(&done);

        bind(&isDouble);
        unboxDouble(source, dest);

        bind(&done);
    }

    void emitSet(Assembler::Condition cond, Register dest) {
        Cset(ARMRegister(dest, 64), cond);
    }

    template <typename T1, typename T2>
    void cmpPtrSet(Assembler::Condition cond, T1 lhs, T2 rhs, Register dest) {
        cmpPtr(lhs, rhs);
        emitSet(cond, dest);
    }

    template <typename T1, typename T2>
    void cmp32Set(Assembler::Condition cond, T1 lhs, T2 rhs, Register dest) {
        cmp32(lhs, rhs);
        emitSet(cond, dest);
    }

    void testNullSet(Condition cond, const ValueOperand &value, Register dest) {
        cond = testNull(cond, value);
        emitSet(cond, dest);
    }
    void testObjectSet(Condition cond, const ValueOperand &value, Register dest) {
        cond = testObject(cond, value);
        emitSet(cond, dest);
    }
    void testUndefinedSet(Condition cond, const ValueOperand &value, Register dest) {
        cond = testUndefined(cond, value);
        emitSet(cond, dest);
    }

    void convertBoolToInt32(Register source, Register dest) {
        Uxtb(ARMRegister(dest, 64), ARMRegister(source, 64));
    }

    void convertInt32ToDouble(Register src, FloatRegister dest) {
        Scvtf(ARMFPRegister(dest, 64), ARMRegister(src, 32)); // Uses FPCR rounding mode.
    }
    void convertInt32ToDouble(const Address &src, FloatRegister dest) {
        load32(src, ScratchReg2);
        convertInt32ToDouble(ScratchReg2, dest);
    }

    void convertInt32ToFloat32(Register src, FloatRegister dest) {
        Scvtf(ARMFPRegister(dest, 32), ARMRegister(src, 32)); // Uses FPCR rounding mode.
    }
    void convertInt32ToFloat32(const Address &src, FloatRegister dest) {
        load32(src, ScratchReg2);
        convertInt32ToFloat32(ScratchReg2, dest);
    }

    void convertUInt32ToDouble(Register src, FloatRegister dest) {
        Ucvtf(ARMFPRegister(dest, 64), ARMRegister(src, 32)); // Uses FPCR rounding mode.
    }
    void convertUInt32ToDouble(const Address &src, FloatRegister dest) {
        load32(src, ScratchReg2);
        convertUInt32ToDouble(ScratchReg2, dest);
    }

    void convertUInt32ToFloat32(Register src, FloatRegister dest) {
        Ucvtf(ARMFPRegister(dest, 32), ARMRegister(src, 32)); // Uses FPCR rounding mode.
    }
    void convertUInt32ToFloat32(const Address &src, FloatRegister dest) {
        load32(src, ScratchReg2);
        convertUInt32ToFloat32(ScratchReg2, dest);
    }

    void convertFloat32ToDouble(FloatRegister src, FloatRegister dest) {
        Fcvt(ARMFPRegister(dest, 64), ARMFPRegister(src, 32));
    }
    void convertDoubleToFloat32(FloatRegister src, FloatRegister dest) {
        Fcvt(ARMFPRegister(dest, 32), ARMFPRegister(src, 64));
    }

    void branchTruncateDouble(FloatRegister src, Register dest, Label *fail) {
        // An out of range integer will be saturated to the destination size.
        ARMFPRegister Src(src, 64);
        ARMRegister Dest(dest, 64);
        //breakpoint();
        Fcvtzs(Dest, Src);
        Add(ScratchReg2_64, Dest, Operand(0x7fffffffffffffff));
        Cmn(ScratchReg2_64, 3);
        B(fail, Assembler::Above);
        And(Dest, Dest, Operand(0xffffffff));
    }
    void convertDoubleToInt32(FloatRegister src, Register dest, Label *fail,
                              bool negativeZeroCheck = true)
    {
        ARMFPRegister fsrc(src, 64);
        ARMRegister dest32(dest, 32);
        ARMRegister dest64(dest, 64);
        Fcvtzs(dest32, fsrc); // Convert, rounding toward zero.
        Scvtf(ScratchDoubleReg_, dest32); // Convert back, using FPCR rounding mode.
        Fcmp(ScratchDoubleReg_, fsrc);
        B(fail, Assembler::NotEqual);

        if (negativeZeroCheck) {
            Label nonzero;
            Cbnz(dest32, &nonzero);
            Fmov(dest64, fsrc);
            Cbnz(dest64, fail);
            bind(&nonzero);
        }
    }
    void convertFloat32ToInt32(FloatRegister src, Register dest, Label *fail,
                               bool negativeZeroCheck = true)
    {
        MOZ_CRASH("convertFloat32ToInt32");
    }

    void branchTruncateFloat32(FloatRegister src, Register dest, Label *fail) {
        ARMFPRegister Src(src, 32);
        ARMRegister Dest(dest, 64);
        Fcvtzs(Dest, Src);
        Add(ScratchReg2_64, Dest, Operand(0x7fffffffffffffff));
        Cmn(ScratchReg2_64, 3);
        B(fail, Assembler::Above);
        And(Dest, Dest, Operand(0xffffffff));
    }
    void jump(Label *label) {
        B(label);
    }
    void jump(JitCode *code) {
        branch(code);
    }
    void jump(RepatchLabel *label) {
        MOZ_CRASH("jump (repatchlabel)");
    }
    void jump(Register reg) {
        Br(ARMRegister(reg, 64));
    }
    void jump(const Address &addr) {
        loadPtr(addr, ip0);
        Br(ip0_64);
    }

    void align(int alignment) {
        armbuffer_.align(alignment);
    }

    void movePtr(Register src, Register dest) {
        Mov(ARMRegister(dest, 64), ARMRegister(src, 64));
    }
    void movePtr(ImmWord imm, Register dest) {
        Mov(ARMRegister(dest, 64), (int64_t)imm.value);
    }
    void movePtr(ImmPtr imm, Register dest) {
        Mov(ARMRegister(dest, 64), (int64_t)imm.value);
    }
    void movePtr(AsmJSImmPtr imm, Register dest) {
        BufferOffset off = movePatchablePtr(ImmWord(0xffffffffffffffffULL), dest);
        append(AsmJSAbsoluteLink(CodeOffsetLabel(off.getOffset()), imm.kind()));
    }
    void movePtr(ImmGCPtr imm, Register dest) {
        BufferOffset load = movePatchablePtr(ImmPtr(imm.value), dest);
        writeDataRelocation(imm, load);
    }
    void movePtr(ImmMaybeNurseryPtr imm, Register dest) {
        movePtr(noteMaybeNurseryPtr(imm), dest);
    }
    void move32(Imm32 imm, Register dest) {
        Mov(ARMRegister(dest, 32), (int64_t)imm.value);
    }
    void move32(Register src, Register dest) {
        Mov(ARMRegister(dest, 32), ARMRegister(src, 32));
    }

    // Move a pointer using a literal pool, so that the pointer
    // may be easily patched or traced.
    // Returns the BufferOffset of the load instruction emitted.
    BufferOffset movePatchablePtr(ImmWord ptr, Register dest);
    BufferOffset movePatchablePtr(ImmPtr ptr, Register dest);

    void not32(Register reg) {
        Orn(ARMRegister(reg, 32), wzr, ARMRegister(reg, 32));
    }
    void neg32(Register reg) {
        Neg(ARMRegister(reg, 32), Operand(ARMRegister(reg, 32)));
    }

    void loadPtr(AsmJSAbsoluteAddress address, Register dest) {
        movePtr(AsmJSImmPtr(address.kind()), ScratchReg);
        Ldr(ARMRegister(dest, 64), MemOperand(ScratchReg64));
    }
    void loadPtr(AbsoluteAddress address, Register dest) {
        movePtr(ImmWord((uintptr_t)address.addr), ScratchReg);
        Ldr(ARMRegister(dest, 64), MemOperand(ScratchReg64));
    }
    void loadPtr(const Address &address, Register dest) {
        Ldr(ARMRegister(dest, 64), MemOperand(address));
    }
    void loadPtr(const BaseIndex &src, Register dest) {
        Register base = src.base;
        uint32_t scale = Imm32::ShiftOf(src.scale).value;

        if (src.offset) {
            Add(ScratchReg64, ARMRegister(base, 64), Operand(int64_t(src.offset)));
            base = ScratchReg;
        }

        ARMRegister dest64(dest, 64);
        ARMRegister base64(base, 64);
        ARMRegister index64(src.index, 64);

        Ldr(dest64, MemOperand(base64, index64, LSL, scale));
    }
    void loadPrivate(const Address &src, Register dest) {
        loadPtr(src, dest);
        lshiftPtr(Imm32(1), dest);
    }

    void store8(Register src, const Address &address) {
        Strb(ARMRegister(src, 32), MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void store8(Imm32 imm, const Address &address) {
        move32(imm, ScratchReg2);
        Strb(ScratchReg2_32, MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void store8(Register src, const BaseIndex &address) {
        doBaseIndex(ARMRegister(src, 32), address, STRB_w);
    }
    void store8(Imm32 imm, const BaseIndex &address) {
        Mov(ScratchReg2_32, Operand(imm.value));
        doBaseIndex(ScratchReg2_32, address, STRB_w);
    }

    void store16(Register src, const Address &address) {
        Strh(ARMRegister(src, 32), MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void store16(Imm32 imm, const Address &address) {
        move32(imm, ScratchReg2);
        Strh(ScratchReg2_32, MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void store16(Register src, const BaseIndex &address) {
        doBaseIndex(ARMRegister(src, 32), address, STRH_w);
    }
    void store16(Imm32 imm, const BaseIndex &address) {
        Mov(ScratchReg2_32, Operand(imm.value));
        doBaseIndex(ScratchReg2_32, address, STRH_w);
    }

    void storePtr(ImmWord imm, const Address &address) {
        movePtr(imm, ScratchReg2);
        storePtr(ScratchReg2, address);
    }
    void storePtr(ImmPtr imm, const Address &address) {
        Mov(ScratchReg2_64, uint64_t(imm.value));
        Str(ScratchReg2_64, MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void storePtr(ImmGCPtr imm, const Address &address) {
        movePtr(imm, ScratchReg2);
        storePtr(ScratchReg2, address);
    }
    void storePtr(Register src, const Address &address) {
        Str(ARMRegister(src, 64), MemOperand(ARMRegister(address.base, 64), address.offset));
    }

    void storePtr(ImmWord imm, const BaseIndex &address) {
        Mov(ScratchReg2_64, Operand(imm.value));
        doBaseIndex(ScratchReg2_64, address, STR_x);
    }
    void storePtr(ImmGCPtr imm, const BaseIndex &address) {
        MOZ_CRASH("storePtr"); // Careful -- doBaseIndex() may use both scratch regs!
    }
    void storePtr(Register src, const BaseIndex &address) {
        doBaseIndex(ARMRegister(src, 64), address, STR_x);
    }

    void storePtr(Register src, AbsoluteAddress address) {
        Mov(ScratchReg2_64, uint64_t(address.addr));
        Str(ARMRegister(src, 64), MemOperand(ScratchReg2_64));
    }

    void store32(Register src, AbsoluteAddress address) {
        Mov(ScratchReg2_64, uint64_t(address.addr));
        Str(ARMRegister(src, 32), MemOperand(ScratchReg2_64));
    }
    void store32(Imm32 imm, const Address &address) {
        Mov(ScratchReg2_32, uint64_t(imm.value));
        Str(ScratchReg2_32, MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void store32(Register r, const Address &address) {
        Str(ARMRegister(r, 32), MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void store32(Imm32 imm, const BaseIndex &address) {
        Mov(ScratchReg2_32, imm.value);
        doBaseIndex(ScratchReg2_32, address, STR_w);
    }
    void store32(Register r, const BaseIndex &address) {
        doBaseIndex(ARMRegister(r, 32), address, STR_w);
    }

    void store32_NoSecondScratch(Imm32 imm, const Address &address) {
        Mov(ScratchReg32, uint64_t(imm.value));
        Str(ScratchReg32, MemOperand(ARMRegister(address.base, 64), address.offset));
    }

    // SIMD.
    void loadAlignedInt32x4(const Address &addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void storeAlignedInt32x4(FloatRegister src, const Address &addr) { MOZ_CRASH("NYI"); }
    void loadUnalignedInt32x4(const Address &addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void storeUnalignedInt32x4(FloatRegister dest, const Address &addr) { MOZ_CRASH("NYI"); }

    void loadAlignedFloat32x4(const Address &addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void storeAlignedFloat32x4(FloatRegister src, const Address &addr) { MOZ_CRASH("NYI"); }
    void loadUnalignedFloat32x4(const Address &addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void storeUnalignedFloat32x4(FloatRegister dest, const Address &addr) { MOZ_CRASH("NYI"); }

    void rshiftPtr(Imm32 imm, Register dest) {
        Lsr(ARMRegister(dest, 64), ARMRegister(dest, 64), imm.value);
    }
    void rshiftPtr(Imm32 imm, Register src, Register dest) {
        Lsr(ARMRegister(dest, 64), ARMRegister(src, 64), imm.value);
    }

    void rshiftPtrArithmetic(Imm32 imm, Register dest) {
        Asr(ARMRegister(dest, 64), ARMRegister(dest, 64), imm.value);
    }
    void lshiftPtr(Imm32 imm, Register dest) {
        Lsl(ARMRegister(dest, 64), ARMRegister(dest, 64), imm.value);
    }
    void xorPtr(Imm32 imm, Register dest) {
        Eor(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(imm.value));
    }
    void xor32(Imm32 imm, Register dest) {
        Eor(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(imm.value));
    }

    void xorPtr(Register src, Register dest) {
        Eor(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(ARMRegister(src, 64)));
    }
    void orPtr(ImmWord imm, Register dest) {
        Orr(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(imm.value));
    }
    void orPtr(Imm32 imm, Register dest) {
        Orr(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(imm.value));
    }
    void orPtr(Register src, Register dest) {
        Orr(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(ARMRegister(src, 64)));
    }
    void or32(Imm32 imm, Register dest) {
        Orr(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(imm.value));
    }
    void or32(Register src, Register dest) {
        Orr(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(ARMRegister(src, 32)));
    }
    void or32(Imm32 imm, const Address &dest) {
        load32(dest, ScratchReg2);
        Orr(ScratchReg2_32, ScratchReg2_32, Operand(imm.value));
        store32(ScratchReg2, dest);
    }
    void andPtr(Imm32 imm, Register dest) {
        And(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(imm.value));
    }
    void andPtr(Register src, Register dest) {
        And(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(ARMRegister(src, 64)));
    }
    void and32(Imm32 imm, Register dest) {
        And(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(imm.value));
    }
    void and32(Imm32 imm, Register src, Register dest) {
        And(ARMRegister(dest, 32), ARMRegister(src, 32), Operand(imm.value));
    }

    void and32(Register src, Register dest) {
        And(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(ARMRegister(src, 32)));
    }
    void and32(Imm32 mask, Address dest) {
        load32(dest, ScratchReg2);
        And(ScratchReg2_32, ScratchReg2_32, Operand(mask.value));
        store32(ScratchReg2, dest);
    }
    void and32(Address src, Register dest) {
        load32(src, ScratchReg2);
        And(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(ScratchReg2_32));
    }

    void testPtr(Register lhs, Register rhs) {
        Tst(ARMRegister(lhs, 64), Operand(ARMRegister(rhs, 64)));
    }
    void test32(Register lhs, Register rhs) {
        Tst(ARMRegister(lhs, 32), Operand(ARMRegister(rhs, 32)));
    }
    void test32(const Address &addr, Imm32 imm) {
        load32(addr, ScratchReg2);
        Tst(ScratchReg2_32, Operand(imm.value));
    }
    void test32(Register lhs, Imm32 rhs) {
        Tst(ARMRegister(lhs, 32), Operand(rhs.value));
    }
    void cmp32(Register lhs, Imm32 rhs) {
        Cmp(ARMRegister(lhs, 32), Operand(rhs.value));
    }
    void cmp32(Register a, Register b) {
        Cmp(ARMRegister(a, 32), Operand(ARMRegister(b, 32)));
    }
    void cmp32(const Operand &lhs, Imm32 rhs) {
        Mov(ScratchReg2_32, lhs);
        Cmp(ScratchReg2_32, Operand(rhs.value));
    }
    void cmp32(const Operand &lhs, Register rhs) {
        Mov(ScratchReg2_32, lhs);
        Cmp(ScratchReg2_32, Operand(ARMRegister(rhs, 32)));
    }

    void cmpPtr(Register lhs, ImmWord rhs) {
        Cmp(ARMRegister(lhs, 64), Operand(rhs.value));
    }
    void cmpPtr(Register lhs, ImmPtr rhs) {
        Cmp(ARMRegister(lhs, 64), Operand(uint64_t(rhs.value)));
    }
    void cmpPtr(Register lhs, Register rhs) {
        Cmp(ARMRegister(lhs, 64), ARMRegister(rhs, 64));
    }
    void cmpPtr(Register lhs, ImmGCPtr rhs) {
        movePtr(rhs, ScratchReg);
        cmpPtr(lhs, ScratchReg);
    }
    void cmpPtr(Register lhs, ImmMaybeNurseryPtr rhs) {
        cmpPtr(lhs, noteMaybeNurseryPtr(rhs));
    }

    void cmpPtr(const Address &lhs, Register rhs) {
        Ldr(ScratchReg2_64, MemOperand(ARMRegister(lhs.base, 64), lhs.offset));
        Cmp(ScratchReg2_64, Operand(ARMRegister(rhs, 64)));
    }
    void cmpPtr(const Address &lhs, ImmWord rhs) {
        Ldr(ScratchReg2_64, MemOperand(ARMRegister(lhs.base, 64), lhs.offset));
        Cmp(ScratchReg2_64, Operand(rhs.value));
    }
    void cmpPtr(const Address &lhs, ImmPtr rhs) {
        Ldr(ScratchReg2_64, MemOperand(ARMRegister(lhs.base, 64), lhs.offset));
        Cmp(ScratchReg2_64, Operand(uint64_t(rhs.value)));
    }
    void cmpPtr(const Address &lhs, ImmGCPtr rhs) {
        loadPtr(lhs, ScratchReg2);
        cmpPtr(ScratchReg2, rhs);
    }

    void loadDouble(const Address &src, FloatRegister dest) {
        Ldr(ARMFPRegister(dest, 64), MemOperand(ARMRegister(src.base,64), src.offset));
    }
    void loadDouble(const BaseIndex &src, FloatRegister dest) {
        ARMRegister base(src.base, 64);
        ARMRegister index(src.index, 64);
        if (src.offset == 0) {
            Ldr(ARMFPRegister(dest, 64), MemOperand(base, index, LSL, unsigned(src.scale)));
            return;
        }
        Add(ScratchReg2_64, base, Operand(index, LSL, unsigned(src.scale)));
        Ldr(ARMFPRegister(dest, 64), MemOperand(ScratchReg2_64, src.offset));
    }
    void loadFloatAsDouble(const Address &addr, FloatRegister dest) {
        Ldr(ARMFPRegister(dest, 32), MemOperand(ARMRegister(addr.base,64), addr.offset));
        fcvt(ARMFPRegister(dest, 64), ARMFPRegister(dest, 32));
    }
    void loadFloatAsDouble(const BaseIndex &src, FloatRegister dest) {
        ARMRegister base(src.base, 64);
        ARMRegister index(src.index, 64);
        if (src.offset == 0) {
            Ldr(ARMFPRegister(dest, 32), MemOperand(base, index, LSL, unsigned(src.scale)));
        } else {
            Add(ScratchReg2_64, base, Operand(index, LSL, unsigned(src.scale)));
            Ldr(ARMFPRegister(dest, 32), MemOperand(ScratchReg2_64, src.offset));
        }
        fcvt(ARMFPRegister(dest, 64), ARMFPRegister(dest, 32));
    }

    void loadFloat32(const Address &addr, FloatRegister dest) {
        Ldr(ARMFPRegister(dest, 32), MemOperand(ARMRegister(addr.base,64), addr.offset));
    }
    void loadFloat32(const BaseIndex &src, FloatRegister dest) {
        ARMRegister base(src.base, 64);
        ARMRegister index(src.index, 64);
        if (src.offset == 0) {
            Ldr(ARMFPRegister(dest, 32), MemOperand(base, index, LSL, unsigned(src.scale)));
        } else {
            Add(ScratchReg2_64, base, Operand(index, LSL, unsigned(src.scale)));
            Ldr(ARMFPRegister(dest, 32), MemOperand(ScratchReg2_64, src.offset));
        }
    }

    void storeDouble(FloatRegister src, const Address &dest) {
        Str(ARMFPRegister(src, 64), MemOperand(ARMRegister(dest.base, 64), dest.offset));
    }
    void storeDouble(FloatRegister src, const BaseIndex &dest) {
        doBaseIndex(ARMFPRegister(src, 64), dest, STR_d);
    }
    void storeFloat32(FloatRegister src, Address addr) {
        Str(ARMFPRegister(src, 32), MemOperand(ARMRegister(addr.base, 64), addr.offset));
    }
    void storeFloat32(FloatRegister src, BaseIndex addr) {
        doBaseIndex(ARMFPRegister(src, 32), addr, STR_s);
    }

    void moveDouble(FloatRegister src, FloatRegister dest) {
        fmov(ARMFPRegister(dest, 64), ARMFPRegister(src, 64));
    }
    void zeroDouble(FloatRegister reg) {
        fmov(ARMFPRegister(reg, 64), xzr);
    }
    void zeroFloat32(FloatRegister reg) {
        fmov(ARMFPRegister(reg, 32), wzr);
    }
    void negateDouble(FloatRegister reg) {
        fneg(ARMFPRegister(reg, 64), ARMFPRegister(reg, 64));
    }
    void negateFloat(FloatRegister reg) {
        fneg(ARMFPRegister(reg, 32), ARMFPRegister(reg, 32));
    }
    void addDouble(FloatRegister src, FloatRegister dest) {
        fadd(ARMFPRegister(dest, 64), ARMFPRegister(dest, 64), ARMFPRegister(src, 64));
    }
    void subDouble(FloatRegister src, FloatRegister dest) {
        fsub(ARMFPRegister(dest, 64), ARMFPRegister(dest, 64), ARMFPRegister(src, 64));
    }
    void mulDouble(FloatRegister src, FloatRegister dest) {
        fmul(ARMFPRegister(dest, 64), ARMFPRegister(dest, 64), ARMFPRegister(src, 64));
    }
    void divDouble(FloatRegister src, FloatRegister dest) {
        fdiv(ARMFPRegister(dest, 64), ARMFPRegister(dest, 64), ARMFPRegister(src, 64));
    }

    void moveFloat32(FloatRegister src, FloatRegister dest) {
        fmov(ARMFPRegister(dest, 32), ARMFPRegister(src, 32));
    }
    void moveFloatAsDouble(Register src, FloatRegister dest) {
        MOZ_CRASH("moveFloatAsDouble");
    }

    void splitTag(const ValueOperand &operand, Register dest) {
        splitTag(operand.valueReg(), dest);
    }
    void splitTag(const Address &operand, Register dest) {
        loadPtr(operand, dest);
        splitTag(dest, dest);
    }
    void splitTag(const BaseIndex &operand, Register dest) {
        loadPtr(operand, dest);
        splitTag(dest, dest);
    }

    // Extracts the tag of a value and places it in ScratchReg.
    Register splitTagForTest(const ValueOperand &value) {
        Lsr(ScratchReg2_64, ARMRegister(value.valueReg(), 64), JSVAL_TAG_SHIFT);
        return ScratchReg2;
    }
    void cmpTag(const ValueOperand &operand, ImmTag tag) {
        MOZ_CRASH("cmpTag");
    }

    void load32(const Address &address, Register dest) {
        Ldr(ARMRegister(dest, 32), MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void load32(const BaseIndex &src, Register dest) {
        doBaseIndex(ARMRegister(dest, 32), src, LDR_w);
    }
    void load32(AbsoluteAddress address, Register dest) {
        movePtr(ImmWord((uintptr_t)address.addr), ScratchReg);
        ldr(ARMRegister(dest, 32), MemOperand(ARMRegister(ScratchReg, 64)));
    }

    void load8SignExtend(const Address &address, Register dest) {
        Ldrsb(ARMRegister(dest, 32), MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void load8SignExtend(const BaseIndex &src, Register dest) {
        doBaseIndex(ARMRegister(dest, 32), src, LDRSB_w);
    }

    void load8ZeroExtend(const Address &address, Register dest) {
        Ldrb(ARMRegister(dest, 32), MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void load8ZeroExtend(const BaseIndex &src, Register dest) {
        doBaseIndex(ARMRegister(dest, 32), src, LDRB_w);
    }

    void load16SignExtend(const Address &address, Register dest) {
        Ldrsh(ARMRegister(dest, 32), MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void load16SignExtend(const BaseIndex &src, Register dest) {
        doBaseIndex(ARMRegister(dest, 32), src, LDRSH_w);
    }

    void load16ZeroExtend(const Address &address, Register dest) {
        Ldrh(ARMRegister(dest, 32), MemOperand(ARMRegister(address.base, 64), address.offset));
    }
    void load16ZeroExtend(const BaseIndex &src, Register dest) {
        doBaseIndex(ARMRegister(dest, 32), src, LDRH_w);
    }

    void add32(Register src, Register dest) {
        Add(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(ARMRegister(src, 32)));
    }
    void add32(Imm32 imm, Register dest) {
        Add(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(imm.value));
    }
    void add32(Imm32 imm, const Address &dest) {
        Ldr(ScratchReg2_32, MemOperand(ARMRegister(dest.base, 64), dest.offset));
        Add(ScratchReg2_32, ScratchReg2_32, Operand(imm.value));
        Str(ScratchReg2_32, MemOperand(ARMRegister(dest.base, 64), dest.offset));
    }

    void adds32(Register src, Register dest) {
        Adds(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(ARMRegister(src, 32)));
    }
    void adds32(Imm32 imm, Register dest) {
        Adds(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(imm.value));
    }
    void adds32(Imm32 imm, const Address &dest) {
        Ldr(ScratchReg2_32, MemOperand(ARMRegister(dest.base, 64), dest.offset));
        Adds(ScratchReg2_32, ScratchReg2_32, Operand(imm.value));
        Str(ScratchReg2_32, MemOperand(ARMRegister(dest.base, 64), dest.offset));
    }

    void sub32(Imm32 imm, Register dest) {
        Sub(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(imm.value));
    }
    void sub32(Register src, Register dest) {
        Sub(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(ARMRegister(src, 32)));
    }

    void subs32(Imm32 imm, Register dest) {
        Subs(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(imm.value));
    }
    void subs32(Register src, Register dest) {
        Subs(ARMRegister(dest, 32), ARMRegister(dest, 32), Operand(ARMRegister(src, 32)));
    }

    void addPtr(Register src, Register dest) {
        Add(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(ARMRegister(src, 64)));
    }
    void addPtr(Register src1, Register src2, Register dest) {
        Add(ARMRegister(dest, 64), ARMRegister(src1, 64), Operand(ARMRegister(src2, 64)));
    }

    void addPtr(Imm32 imm, Register dest) {
        Add(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(imm.value));
    }
    void addPtr(Imm32 imm, Register src, Register dest) {
        Add(ARMRegister(dest, 64), ARMRegister(src, 64), Operand(imm.value));
    }

    void addPtr(Imm32 imm, const Address &dest) {
        Ldr(ScratchReg2_64, MemOperand(ARMRegister(dest.base, 64), dest.offset));
        Add(ScratchReg2_64, ScratchReg2_64, Operand(imm.value));
        Str(ScratchReg2_64, MemOperand(ARMRegister(dest.base, 64), dest.offset));
    }
    void addPtr(ImmWord imm, Register dest) {
        Add(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(imm.value));
    }
    void addPtr(ImmPtr imm, Register dest) {
        Add(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(uint64_t(imm.value)));
    }
    void addPtr(const Address &src, Register dest) {
        Ldr(ScratchReg2_64, MemOperand(ARMRegister(src.base, 64), src.offset));
        Add(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(ScratchReg2_64));
    }
    void subPtr(Imm32 imm, Register dest) {
        Sub(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(imm.value));
    }
    void subPtr(Register src, Register dest) {
        Sub(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(ARMRegister(src, 64)));
    }
    void subPtr(const Address &addr, Register dest) {
        Ldr(ScratchReg2_64, MemOperand(ARMRegister(addr.base, 64), addr.offset));
        Sub(ARMRegister(dest, 64), ARMRegister(dest, 64), Operand(ScratchReg2_64));
    }
    void subPtr(Register src, const Address &dest) {
        Ldr(ScratchReg2_64, MemOperand(ARMRegister(dest.base, 64), dest.offset));
        Sub(ScratchReg2_64, ScratchReg2_64, Operand(ARMRegister(src, 64)));
        Str(ScratchReg2_64, MemOperand(ARMRegister(dest.base, 64), dest.offset));
    }
    void mul32(Register src1, Register src2, Register dest, Label *onOver, Label *onZero) {
        Smull(ARMRegister(dest, 64), ARMRegister(src1, 32), ARMRegister(src2, 32));
        if (onOver) {
            Cmp(ARMRegister(dest, 64), Operand(ARMRegister(dest, 32), SXTW));
            B(onOver, NotEqual);
        }
        if (onZero)
            Cbz(ARMRegister(dest, 32), onZero);

        // Clear upper 32 bits.
        Mov(ARMRegister(dest, 32), ARMRegister(dest, 32));
    }
    void ret() {
        syncStackPtr(); // SP is always used to transmit the stack between calls.
        Ret(lr_64); // Branches to lr with a return hint.
    }

    void retn(Imm32 n) {
        // ip0 <- [sp]; sp += n; ret ip0
        Ldr(ip0_64, MemOperand(GetStackPointer(), ptrdiff_t(n.value), PostIndex));
        syncStackPtr(); // SP is always used to transmit the stack between calls.
        Ret(ip0_64);
    }

    void j(Condition code, Label *dest) {
        b(dest, code);
    }
    void j(Label *dest) {
        b(dest, Always);
    }

    void branch(Condition cond, Label *label) {
        b(label, cond);
    }
    void branch(JitCode *target) {
        syncStackPtr();
        addPendingJump(nextOffset(), ImmPtr(target->raw()), Relocation::JITCODE);
        b(-1); // The jump target will be patched by executableCopy().
    }

    void branch16(Condition cond, Register lhs, Register rhs, Label *label) {
        MOZ_CRASH("branch16");
    }

    void branch32(Condition cond, const Operand &lhs, Register rhs, Label *label) {
        // since rhs is an operand, do the compare backwards
        Cmp(ARMRegister(rhs, 32), lhs);
        // 
        b(label, Assembler::InvertCmpCondition(cond));
    }
    void branch32(Condition cond, const Operand &lhs, Imm32 rhs, Label *label) {
        MOZ_CRASH("branch32");
    }
    void branch32(Condition cond, Register lhs, Register rhs, Label *label) {
        cmp32(lhs, rhs);
        b(label, cond);
    }
    void branch32(Condition cond, Register lhs, Imm32 imm, Label *label) {
        cmp32(lhs, imm);
        b(label, cond);
    }
    void branch32(Condition cond, const Address &lhs, Register rhs, Label *label) {
        load32(lhs, ScratchReg);
        branch32(cond, ScratchReg, rhs, label);
    }
    void branch32(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        load32(lhs, ScratchReg);
        branch32(cond, ScratchReg, imm, label);
    }
    void branch32(Condition cond, AbsoluteAddress lhs, Register rhs, Label *label) {
        movePtr(ImmPtr(lhs.addr), ScratchReg2);
        branch32(cond, Address(ScratchReg2, 0), rhs, label);
    }
    void branch32(Condition cond, AbsoluteAddress lhs, Imm32 rhs, Label *label) {
        movePtr(ImmPtr(lhs.addr), ScratchReg2);
        branch32(cond, Address(ScratchReg2, 0), rhs, label);
    }
    void branch32(Condition cond, AsmJSAbsoluteAddress lhs, Imm32 rhs, Label *label) {
        movePtr(AsmJSImmPtr(lhs.kind()), ScratchReg2);
        branch32(cond, Address(ScratchReg2, 0), rhs, label);
    }
    void branch32(Condition cond, BaseIndex lhs, Imm32 rhs, Label *label) {
        doBaseIndex(ScratchReg2_32, lhs, LDR_w);
        branch32(cond, ScratchReg2, rhs, label);
    }

    void branchSub32(Condition cond, const Address &lhs, Register rhs, Label *label) {
        MOZ_CRASH("branchSub32");
    }
    void branchSub32(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        MOZ_CRASH("branchSub32");
    }
    void branchSub32(Condition cond, Register lhs, Imm32 imm, Label *label) {
        MOZ_CRASH("branchSub32");
    }
    void branchSub32(Condition cond, Register lhs, Register rhs, Label *label) {
        MOZ_CRASH("branchSub32");
    }
    void branchSub32(Condition cond, AbsoluteAddress lhs, Imm32 rhs, Label *label) {
        MOZ_CRASH("branchSub32");
    }
    void branchSub32(Condition cond, AbsoluteAddress lhs, Register rhs, Label *label) {
        MOZ_CRASH("branchSub32");
    }

    void branchTest16(Condition cond, Register lhs, Register rhs, Label *label) {
        MOZ_CRASH("branchTest16");
    }
    void branchTest32(Condition cond, Register lhs, Register rhs, Label *label) {
        MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed || cond == NotSigned);
        // x86 prefers |test foo, foo| to |cmp foo, #0|.
        // Convert the former to the latter for ARM.
        if (lhs == rhs && (cond == Zero || cond == NonZero))
            cmp32(lhs, Imm32(0));
        else
            test32(lhs, rhs);
        B(label, cond);
    }
    void branchTest32(Condition cond, Register lhs, Imm32 imm, Label *label) {
        MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed || cond == NotSigned);
        test32(lhs, imm);
        B(label, cond);
    }
    void branchTest32(Condition cond, const Address &address, Imm32 imm, Label *label) {
        load32(address, ScratchReg2);
        branchTest32(cond, ScratchReg2, imm, label);
    }
    void branchTest32(Condition cond, AbsoluteAddress &address, Imm32 imm, Label *label) {
        MOZ_CRASH("branchTest32");
    }
    CodeOffsetJump jumpWithPatch(RepatchLabel *label, Condition cond = Always) {
        ARMBuffer::PoolEntry pe;
        BufferOffset load_bo =  immPool64(ScratchReg2_64, (uint64_t) label, &pe);
        BufferOffset branch_bo;
        MOZ_ASSERT(!label->bound());
        if (cond != Always) {
            Label notTaken;
            b(&notTaken, Assembler::InvertCondition(cond));
            branch_bo = b(-1);
            bind(&notTaken);
        } else {
            nop();
            branch_bo = b(-1);
        }
        label->use(branch_bo.getOffset());
        return CodeOffsetJump(load_bo.getOffset(), pe.index());
    }
    CodeOffsetJump backedgeJump(RepatchLabel *label) {
        return jumpWithPatch(label);
    }
    template <typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, Register reg, T ptr, RepatchLabel *label) {
        MOZ_CRASH("branchPtrWithPatch");
    }
    template <typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, Address addr, T ptr, RepatchLabel *label) {
        loadPtr(addr, ScratchReg2);
        cmpPtr(ScratchReg2, ptr);
        jumpWithPatch(label, cond);
    }

    void branchPtr(Condition cond, AsmJSAbsoluteAddress lhs, Register rhs, Label *label) {
        loadPtr(lhs, ScratchReg2);
        branchPtr(cond, ScratchReg2, rhs, label);
    }
    void branchPtr(Condition cond, Address lhs, ImmWord ptr, Label *label) {
        loadPtr(lhs, ScratchReg2);
        branchPtr(cond, ScratchReg2, ptr, label);
    }
    void branchPtr(Condition cond, Address lhs, ImmPtr ptr, Label *label) {
        loadPtr(lhs, ScratchReg2);
        branchPtr(cond, ScratchReg2, ptr, label);
    }
    void branchPtr(Condition cond, Address lhs, Register ptr, Label *label) {
        loadPtr(lhs, ScratchReg2);
        branchPtr(cond, ScratchReg2, ptr, label);
    }
    void branchPtr(Condition cond, Register lhs, ImmWord ptr, Label *label) {
        cmpPtr(lhs, ptr);
        B(label, cond);
    }
    void branchPtr(Condition cond, Register lhs, ImmPtr rhs, Label *label) {
        cmpPtr(lhs, rhs);
        B(label, cond);
    }
    void branchPtr(Condition cond, Register lhs, ImmGCPtr ptr, Label *label) {
        movePtr(ptr, ScratchReg2);
        branchPtr(cond, lhs, ScratchReg2, label);
    }
    void branchPtr(Condition cond, Address lhs, ImmGCPtr ptr, Label *label) {
        movePtr(ptr, ScratchReg2);
        loadPtr(lhs, ScratchReg);
        cmp(ScratchReg64, ScratchReg2_64);
        B(cond, label);

    }
    void branchPtr(Condition cond, Address lhs, ImmMaybeNurseryPtr ptr, Label *label) {
        branchPtr(cond, lhs, noteMaybeNurseryPtr(ptr), label);
    }
    void branchPtr(Condition cond, Register lhs, Register rhs, Label *label) {
        Cmp(ARMRegister(lhs, 64), ARMRegister(rhs, 64));
        B(label, cond);
    }
    void branchPtr(Condition cond, AbsoluteAddress lhs, Register rhs, Label *label) {
        loadPtr(lhs, ScratchReg2);
        branchPtr(cond, ScratchReg2, rhs, label);
    }
    void branchPtr(Condition cond, AbsoluteAddress lhs, ImmWord ptr, Label *label) {
        loadPtr(lhs, ScratchReg2);
        branchPtr(cond, ScratchReg2, ptr, label);
    }

    void branchTestPtr(Condition cond, Register lhs, Register rhs, Label *label) {
        Tst(ARMRegister(lhs, 64), Operand(ARMRegister(rhs, 64)));
        B(label, cond);
    }
    void branchTestPtr(Condition cond, Register lhs, Imm32 imm, Label *label) {
        Tst(ARMRegister(lhs, 64), Operand(imm.value));
        B(label, cond);
    }
    void branchTestPtr(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        loadPtr(lhs, ScratchReg2);
        branchTestPtr(cond, ScratchReg2, imm, label);
    }
    void branchPrivatePtr(Condition cond, const Address &lhs, ImmPtr ptr, Label *label) {
        branchPtr(cond, lhs, ptr, label);
    }

    void branchPrivatePtr(Condition cond, const Address &lhs, Register ptr, Label *label) {
        branchPtr(cond, lhs, ptr, label);
    }

    void branchPrivatePtr(Condition cond, Register lhs, ImmWord ptr, Label *label) {
        branchPtr(cond, lhs, ptr, label);
    }

    void decBranchPtr(Condition cond, Register lhs, Imm32 imm, Label *label) {
        Subs(ARMRegister(lhs, 64), ARMRegister(lhs, 64), Operand(imm.value));
        B(cond, label);
    }

    void branchTestUndefined(Condition cond, Register tag, Label *label) {
        Condition c = testUndefined(cond, tag);
        B(label, c);
    }
    void branchTestInt32(Condition cond, Register tag, Label *label) {
        Condition c = testInt32(cond, tag);
        B(label, c);
    }
    void branchTestDouble(Condition cond, Register tag, Label *label) {
        Condition c = testDouble(cond, tag);
        B(label, c);
    }
    void branchTestBoolean(Condition cond, Register tag, Label *label) {
        Condition c = testBoolean(cond, tag);
        B(label, c);
    }
    void branchTestNull(Condition cond, Register tag, Label *label) {
        Condition c = testNull(cond, tag);
        B(label, c);
    }
    void branchTestString(Condition cond, Register tag, Label *label) {
        Condition c = testString(cond, tag);
        B(label, c);
    }
    void branchTestSymbol(Condition cond, Register tag, Label *label) {
        Condition c = testSymbol(cond, tag);
        B(label, c);
    }
    void branchTestObject(Condition cond, Register tag, Label *label) {
        Condition c = testObject(cond, tag);
        B(label, c);
    }
    void branchTestNumber(Condition cond, Register tag, Label *label) {
        Condition c = testNumber(cond, tag);
        B(label, c);
    }

    void branchTestUndefined(Condition cond, const Address &address, Label *label) {
        Condition c = testUndefined(cond, address);
        B(label, c);
    }
    void branchTestInt32(Condition cond, const Address &address, Label *label) {
        Condition c = testInt32(cond, address);
        B(label, c);
    }
    void branchTestDouble(Condition cond, const Address &address, Label *label) {
        Condition c = testDouble(cond, address);
        B(label, c);
    }
    void branchTestBoolean(Condition cond, const Address &address, Label *label) {
        Condition c = testDouble(cond, address);
        B(label, c);
    }
    void branchTestNull(Condition cond, const Address &address, Label *label) {
        Condition c = testNull(cond, address);
        B(label, c);
    }
    void branchTestString(Condition cond, const Address &address, Label *label) {
        Condition c = testString(cond, address);
        B(label, c);
    }
    void branchTestSymbol(Condition cond, const Address &address, Label *label) {
        Condition c = testSymbol(cond, address);
        B(label, c);
    }
    void branchTestObject(Condition cond, const Address &address, Label *label) {
        Condition c = testObject(cond, address);
        B(label, c);
    }
    void branchTestNumber(Condition cond, const Address &address, Label *label) {
        Condition c = testNumber(cond, address);
        B(label, c);
    }

    // Perform a type-test on a full Value loaded into a register.
    // Clobbers the ScratchReg.
    void branchTestUndefined(Condition cond, const ValueOperand &src, Label *label) {
        Condition c = testUndefined(cond, src);
        B(label, c);
    }
    void branchTestInt32(Condition cond, const ValueOperand &src, Label *label) {
        Condition c = testInt32(cond, src);
        B(label, c);
    }
    void branchTestBoolean(Condition cond, const ValueOperand &src, Label *label) {
        Condition c = testBoolean(cond, src);
        B(label, c);
    }
    void branchTestDouble(Condition cond, const ValueOperand &src, Label *label) {
        Condition c = testDouble(cond, src);
        B(label, c);
    }
    void branchTestNull(Condition cond, const ValueOperand &src, Label *label) {
        Condition c = testNull(cond, src);
        B(label, c);
    }
    void branchTestString(Condition cond, const ValueOperand &src, Label *label) {
        Condition c = testString(cond, src);
        B(label, c);
    }
    void branchTestSymbol(Condition cond, const ValueOperand &src, Label *label) {
        Condition c = testSymbol(cond, src);
        B(label, c);
    }
    void branchTestObject(Condition cond, const ValueOperand &src, Label *label) {
        Condition c = testObject(cond, src);
        B(label, c);
    }
    void branchTestNumber(Condition cond, const ValueOperand &src, Label *label) {
        Condition c = testNumber(cond, src);
        B(label, c);
    }

    // Perform a type-test on a Value addressed by BaseIndex.
    // Clobbers the ScratchReg.
    void branchTestUndefined(Condition cond, const BaseIndex &address, Label *label) {
        Condition c = testUndefined(cond, address);
        B(label, c);
    }
    void branchTestInt32(Condition cond, const BaseIndex &address, Label *label) {
        Condition c = testInt32(cond, address);
        B(label, c);
    }
    void branchTestBoolean(Condition cond, const BaseIndex &address, Label *label) {
        Condition c = testBoolean(cond, address);
        B(label, c);
    }
    void branchTestDouble(Condition cond, const BaseIndex &address, Label *label) {
        Condition c = testDouble(cond, address);
        B(label, c);
    }
    void branchTestNull(Condition cond, const BaseIndex &address, Label *label) {
        Condition c = testNull(cond, address);
        B(label, c);
    }
    void branchTestString(Condition cond, const BaseIndex &address, Label *label) {
        Condition c = testString(cond, address);
        B(label, c);
    }
    void branchTestSymbol(Condition cond, const BaseIndex &address, Label *label) {
        Condition c = testSymbol(cond, address);
        B(label, c);
    }
    void branchTestObject(Condition cond, const BaseIndex &address, Label *label) {
        Condition c = testObject(cond, address);
        B(label, c);
    }
    template <typename T>
    void branchTestGCThing(Condition cond, const T &src, Label *label) {
        Condition c = testGCThing(cond, src);
        B(label, c);
    }
    template <typename T>
    void branchTestPrimitive(Condition cond, const T &t, Label *label) {
        Condition c = testPrimitive(cond, t);
        B(label, c);
    }
    template <typename T>
    void branchTestMagic(Condition cond, const T &t, Label *label) {
        Condition c = testMagic(cond, t);
        B(label, c);
    }
    void branchTestMagicValue(Condition cond, const ValueOperand &val, JSWhyMagic why, Label *label) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        branchTestValue(cond, val, MagicValue(why), label);
    }
    void branchTestValue(Condition cond, const ValueOperand &value, const Value &v, Label *label) {
        moveValue(v, ValueOperand(ScratchReg2));
        Cmp(ARMRegister(value.valueReg(), 64), ScratchReg2_64);
        B(label, cond);
    }
    void branchTestValue(Condition cond, const Address &valaddr, const ValueOperand &value,
                         Label *label)
    {
        loadValue(valaddr, ScratchReg2);
        Cmp(ARMRegister(value.valueReg(), 64), Operand(ScratchReg2_64));
        B(label, cond);
    }

    void compareDouble(DoubleCondition cond, FloatRegister lhs, FloatRegister rhs) {
        Fcmp(ARMFPRegister(lhs, 64), ARMFPRegister(rhs, 64));
    }
    void branchDouble(DoubleCondition cond, FloatRegister lhs, FloatRegister rhs, Label *label) {
        compareDouble(cond, lhs, rhs);
        switch (cond) {
          case DoubleNotEqual: {
            Label unordered;
            // not equal *and* ordered
            branch(Overflow, &unordered);
            branch(NotEqual, label);
            bind(&unordered);
            break;
          }
          case DoubleEqualOrUnordered:
            branch(Overflow, label);
            branch(Equal, label);
            break;
          default:
            branch(Condition(cond), label);
        }
    }

    void compareFloat(DoubleCondition cond, FloatRegister lhs, FloatRegister rhs) {
        Fcmp(ARMFPRegister(lhs, 32), ARMFPRegister(rhs, 32));
    }
    void branchFloat(DoubleCondition cond, FloatRegister lhs, FloatRegister rhs, Label *label) {
        compareFloat(cond, lhs, rhs);
        switch (cond) {
          case DoubleNotEqual: {
            Label unordered;
            // not equal *and* ordered
            branch(Overflow, &unordered);
            branch(NotEqual, label);
            bind(&unordered);
            break;
          }
          case DoubleEqualOrUnordered:
            branch(Overflow, label);
            branch(Equal, label);
            break;
          default:
            branch(Condition(cond), label);
        }
    }

    void branchNegativeZero(FloatRegister reg, Register scratch, Label *label) {
        MOZ_CRASH("branchNegativeZero");
    }
    void branchNegativeZeroFloat32(FloatRegister reg, Register scratch, Label *label) {
        MOZ_CRASH("branchNegativeZeroFloat32");
    }

    void boxDouble(FloatRegister src, const ValueOperand &dest) {
        Fmov(ARMRegister(dest.valueReg(), 64), ARMFPRegister(src, 64));
    }
    void boxNonDouble(JSValueType type, Register src, const ValueOperand &dest) {
        boxValue(type, src, dest.valueReg());
    }

    // Note that the |dest| register here may be ScratchReg, so we shouldn't use it.
    void unboxInt32(const ValueOperand &src, Register dest) {
        move32(src.valueReg(), dest);
    }
    void unboxInt32(const Address &src, Register dest) {
        load32(src, dest);
    }
    void unboxDouble(const Address &src, FloatRegister dest) {
        loadDouble(src, dest);
    }
    void unboxDouble(const ValueOperand &src, FloatRegister dest) {
        Fmov(ARMFPRegister(dest, 64), ARMRegister(src.valueReg(), 64));
    }

    void unboxArgObjMagic(const ValueOperand &src, Register dest) {
        MOZ_CRASH("unboxArgObjMagic");
    }
    void unboxArgObjMagic(const Address &src, Register dest) {
        MOZ_CRASH("unboxArgObjMagic");
    }

    void unboxBoolean(const ValueOperand &src, Register dest) {
        move32(src.valueReg(), dest);
    }
    void unboxBoolean(const Address &src, Register dest) {
        load32(src, dest);
    }

    void unboxMagic(const ValueOperand &src, Register dest) {
        move32(src.valueReg(), dest);
    }
    // Unbox any non-double value into dest. Prefer unboxInt32 or unboxBoolean
    // instead if the source type is known.
    void unboxNonDouble(const ValueOperand &src, Register dest) {
        unboxNonDouble(src.valueReg(), dest);
    }
    void unboxNonDouble(Address src, Register dest) {
        loadPtr(src, dest);
        unboxNonDouble(dest, dest);
    }

    void unboxNonDouble(Register src, Register dest) {
        And(ARMRegister(dest, 64), ARMRegister(src, 64), Operand((1ULL << JSVAL_TAG_SHIFT) - 1ULL));
    }

    void unboxPrivate(const ValueOperand &src, Register dest) {
        ubfx(ARMRegister(dest, 64), ARMRegister(src.valueReg(), 64), 1, JSVAL_TAG_SHIFT - 1);
    }

    void notBoolean(const ValueOperand &val) {
        ARMRegister r(val.valueReg(), 64);
        eor(r, r, Operand(1));
    }
    void unboxObject(const ValueOperand &src, Register dest) {
        unboxNonDouble(src.valueReg(), dest);
    }
    void unboxObject(Register src, Register dest) {
        unboxNonDouble(src, dest);
    }
    void unboxObject(const Address &src, Register dest) {
        loadPtr(src, dest);
        unboxNonDouble(dest, dest);
    }
    void unboxObject(const BaseIndex &src, Register dest) {
        doBaseIndex(ARMRegister(dest, 64), src, LDR_x);
        unboxNonDouble(dest, dest);
    }

    void unboxValue(const ValueOperand &src, AnyRegister dest) {
        if (dest.isFloat()) {
            Label notInt32, end;
            branchTestInt32(Assembler::NotEqual, src, &notInt32);
            convertInt32ToDouble(src.valueReg(), dest.fpu());
            jump(&end);
            bind(&notInt32);
            unboxDouble(src, dest.fpu());
            bind(&end);
        } else {
            unboxNonDouble(src, dest.gpr());
        }

    }
    void unboxString(const ValueOperand &operand, Register dest) {
        unboxNonDouble(operand, dest);
    }
    void unboxString(const Address &src, Register dest) {
        unboxNonDouble(src, dest);
    }
    void unboxSymbol(const ValueOperand &operand, Register dest) {
        unboxNonDouble(operand, dest);
    }
    void unboxSymbol(const Address &src, Register dest) {
        unboxNonDouble(src, dest);
    }
    // These two functions use the low 32-bits of the full value register.
    void boolValueToDouble(const ValueOperand &operand, FloatRegister dest) {
        convertInt32ToDouble(operand.valueReg(), dest);
    }
    void int32ValueToDouble(const ValueOperand &operand, FloatRegister dest) {
        convertInt32ToDouble(operand.valueReg(), dest);
    }

    void boolValueToFloat32(const ValueOperand &operand, FloatRegister dest) {
        convertInt32ToFloat32(operand.valueReg(), dest);
    }
    void int32ValueToFloat32(const ValueOperand &operand, FloatRegister dest) {
        convertInt32ToFloat32(operand.valueReg(), dest);
    }

    void loadConstantDouble(double d, FloatRegister dest) {
        Fmov(ARMFPRegister(dest, 64), d);
    }
    void loadConstantFloat32(float f, FloatRegister dest) {
        Fmov(ARMFPRegister(dest, 32), f);
    }

    // Register-based tests.
    Condition testUndefined(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_UNDEFINED));
        return cond;
    }
    Condition testInt32(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_INT32));
        return cond;
    }
    Condition testBoolean(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_BOOLEAN));
        return cond;
    }
    Condition testNull(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_NULL));
        return cond;
    }
    Condition testString(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_STRING));
        return cond;
    }
    Condition testSymbol(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_SYMBOL));
        return cond;
    }
    Condition testObject(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_OBJECT));
        return cond;
    }
    Condition testDouble(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, Imm32(JSVAL_TAG_MAX_DOUBLE));
        return (cond == Equal) ? BelowOrEqual : Above;
    }
    Condition testNumber(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, Imm32(JSVAL_UPPER_INCL_TAG_OF_NUMBER_SET));
        return (cond == Equal) ? BelowOrEqual : Above;
    }
    Condition testGCThing(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, Imm32(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return (cond == Equal) ? AboveOrEqual : Below;
    }
    Condition testMagic(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    Condition testPrimitive(Condition cond, Register tag) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        cmp32(tag, Imm32(JSVAL_UPPER_EXCL_TAG_OF_PRIMITIVE_SET));
        return (cond == Equal) ? Below : AboveOrEqual;
    }
    Condition testError(Condition cond, Register tag) {
        return testMagic(cond, tag);
    }

    // ValueOperand-based tests.
    Condition testInt32(Condition cond, const ValueOperand &value) {
        splitTag(value, ScratchReg2);
        return testInt32(cond, ScratchReg2);
    }
    Condition testBoolean(Condition cond, const ValueOperand &value) {
        splitTag(value, ScratchReg2);
        return testBoolean(cond, ScratchReg2);
    }
    Condition testDouble(Condition cond, const ValueOperand &value) {
        splitTag(value, ScratchReg2);
        return testDouble(cond, ScratchReg2);
    }
    Condition testNull(Condition cond, const ValueOperand &value) {
        splitTag(value, ScratchReg2);
        return testNull(cond, ScratchReg2);
    }
    Condition testUndefined(Condition cond, const ValueOperand &value) {
        splitTag(value, ScratchReg2);
        return testUndefined(cond, ScratchReg2);
    }
    Condition testString(Condition cond, const ValueOperand &value) {
        splitTag(value, ScratchReg2);
        return testString(cond, ScratchReg2);
    }
    Condition testSymbol(Condition cond, const ValueOperand &value) {
        splitTag(value, ScratchReg2);
        return testSymbol(cond, ScratchReg2);
    }
    Condition testObject(Condition cond, const ValueOperand &value) {
        splitTag(value, ScratchReg2);
        return testObject(cond, ScratchReg2);
    }
    Condition testNumber(Condition cond, const ValueOperand &value) {
        splitTag(value, ScratchReg2);
        return testNumber(cond, ScratchReg2);
    }
    Condition testPrimitive(Condition cond, const ValueOperand &value) {
        splitTag(value, ScratchReg2);
        return testPrimitive(cond, ScratchReg2);
    }
    Condition testMagic(Condition cond, const ValueOperand &src) {
        splitTag(src, ScratchReg2);
        return testMagic(cond, ScratchReg2);
    }
    Condition testError(Condition cond, const ValueOperand &src) {
        return testMagic(cond, src);
    }

    // Address-based tests.
    Condition testGCThing(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testGCThing(cond, ScratchReg2);
    }
    Condition testMagic(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testMagic(cond, ScratchReg2);
    }
    Condition testInt32(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testInt32(cond, ScratchReg2);
    }
    Condition testDouble(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testDouble(cond, ScratchReg2);
    }
    Condition testBoolean(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testBoolean(cond, ScratchReg2);
    }
    Condition testNull(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testNull(cond, ScratchReg2);
    }
    Condition testUndefined(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testUndefined(cond, ScratchReg2);
    }
    Condition testString(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testString(cond, ScratchReg2);
    }
    Condition testSymbol(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testSymbol(cond, ScratchReg2);
    }
    Condition testObject(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testObject(cond, ScratchReg2);
    }
    Condition testNumber(Condition cond, const Address &address) {
        splitTag(address, ScratchReg2);
        return testNumber(cond, ScratchReg2);
    }

    // BaseIndex-based tests.
    Condition testUndefined(Condition cond, const BaseIndex &src) {
        splitTag(src, ScratchReg2);
        return testUndefined(cond, ScratchReg2);
    }
    Condition testNull(Condition cond, const BaseIndex &src) {
        splitTag(src, ScratchReg2);
        return testNull(cond, ScratchReg2);
    }
    Condition testBoolean(Condition cond, const BaseIndex &src) {
        splitTag(src, ScratchReg2);
        return testBoolean(cond, ScratchReg2);
    }
    Condition testString(Condition cond, const BaseIndex &src) {
        splitTag(src, ScratchReg2);
        return testString(cond, ScratchReg2);
    }
    Condition testSymbol(Condition cond, const BaseIndex &src) {
        splitTag(src, ScratchReg2);
        return testSymbol(cond, ScratchReg2);
    }
    Condition testInt32(Condition cond, const BaseIndex &src) {
        splitTag(src, ScratchReg2);
        return testInt32(cond, ScratchReg2);
    }
    Condition testObject(Condition cond, const BaseIndex &src) {
        splitTag(src, ScratchReg2);
        return testObject(cond, ScratchReg2);
    }
    Condition testDouble(Condition cond, const BaseIndex &src) {
        splitTag(src, ScratchReg2);
        return testDouble(cond, ScratchReg2);
    }
    Condition testMagic(Condition cond, const BaseIndex &src) {
        splitTag(src, ScratchReg2);
        return testMagic(cond, ScratchReg2);
    }
    Condition testGCThing(Condition cond, const BaseIndex &src) {
        splitTag(src, ScratchReg2);
        return testGCThing(cond, ScratchReg2);
    }

    Condition testInt32Truthy(bool truthy, const ValueOperand &operand) {
        ARMRegister payload32(operand.valueReg(), 32);
        Tst(payload32, payload32);
        return truthy ? NonZero : Zero;
    }
    void branchTestInt32Truthy(bool truthy, const ValueOperand &operand, Label *label) {
        Condition c = testInt32Truthy(truthy, operand);
        B(label, c);
    }

    void branchTestDoubleTruthy(bool truthy, FloatRegister reg, Label *label) {
        Fcmp(ARMFPRegister(reg, 64), 0.0);
        if (!truthy) {
            // falsy values are zero, and NaN.
            branch(Zero, label);
            branch(Overflow, label);
        } else {
            // truthy values are non-zero and not nan.
            // If it is overflow
            Label onFalse;
            branch(Zero, &onFalse);
            branch(Overflow, &onFalse);
            b(label);
            bind(&onFalse);
        }
    }

    Condition testBooleanTruthy(bool truthy, const ValueOperand &operand) {
        ARMRegister payload32(operand.valueReg(), 32);
        Tst(payload32, payload32);
        return truthy ? NonZero : Zero;
    }
    void branchTestBooleanTruthy(bool truthy, const ValueOperand &operand, Label *label) {
        Condition c = testBooleanTruthy(truthy, operand);
        B(label, c);
    }
    Condition testStringTruthy(bool truthy, const ValueOperand &value) {
        unboxString(value, ScratchReg2);
        Ldr(ScratchReg2_32, MemOperand(ScratchReg2_64, JSString::offsetOfLength()));
        Cmp(ScratchReg2_32, Operand(0));
        return truthy ? Condition::NonZero : Condition::Zero;
    }
    void branchTestStringTruthy(bool truthy, const ValueOperand &value, Label *label) {
        Condition c = testStringTruthy(truthy, value);
        B(label, c);
    }
    void int32OrDouble(Register src, ARMFPRegister dest) {
        Label isInt32;
        Label join;
        testInt32(Equal, src);
        B(&isInt32, Equal);
        // is double, move teh bits as is
        Fmov(dest, ARMRegister(src, 64));
        B(&join);
        bind(&isInt32);
        // is int32, do a conversion while moving
        Scvtf(dest, ARMRegister(src, 64));
        bind(&join);
    }
    void loadUnboxedValue(Address address, MIRType type, AnyRegister dest) {
        if (dest.isFloat()) {
            Ldr(ScratchReg2_64, toMemOperand(address));
            int32OrDouble(ScratchReg2, ARMFPRegister(dest.fpu(), 64));
        } else if (type == MIRType_Int32 || type == MIRType_Boolean) {
            load32(address, dest.gpr());
        } else {
            loadPtr(address, dest.gpr());
            unboxNonDouble(dest.gpr(), dest.gpr());
        }
    }

    void loadUnboxedValue(BaseIndex address, MIRType type, AnyRegister dest) {
        if (dest.isFloat()) {
            doBaseIndex(ScratchReg2_64, address, LDR_x);
            int32OrDouble(ScratchReg2, ARMFPRegister(dest.fpu(), 64));
        }  else if (type == MIRType_Int32 || type == MIRType_Boolean) {
            load32(address, dest.gpr());
        } else {
            loadPtr(address, dest.gpr());
            unboxNonDouble(dest.gpr(), dest.gpr());
        }
    }

    void loadInstructionPointerAfterCall(Register dest) {
        MOZ_CRASH("loadInstructionPointerAfterCall");
    }

    // Emit a B that can be toggled to a CMP. See ToggleToJmp(), ToggleToCmp().
    CodeOffsetLabel toggledJump(Label *label) {
        BufferOffset offset = b(label, Always);
        CodeOffsetLabel ret(offset.getOffset());
        return ret;
    }

    // load: offset to the load instruction obtained by movePatchablePtr().
    void writeDataRelocation(ImmGCPtr ptr, BufferOffset load) {
        if (ptr.value)
            tmpDataRelocations_.append(load);
    }
    void writeDataRelocation(const Value &val, BufferOffset load) {
        if (val.isMarkable()) {
            gc::Cell *cell = reinterpret_cast<gc::Cell *>(val.toGCThing());
            if (cell && gc::IsInsideNursery(cell))
                embedsNurseryPointers_ = true;
            tmpDataRelocations_.append(load);
        }
    }

    void writePrebarrierOffset(CodeOffsetLabel label) {
        tmpPreBarriers_.append(BufferOffset(label.offset()));
    }

    void computeEffectiveAddress(const Address &address, Register dest) {
        Add(ARMRegister(dest, 64), ARMRegister(address.base, 64), Operand(address.offset));
    }
    void computeEffectiveAddress(const BaseIndex &address, Register dest) {
        ARMRegister dest64(dest, 64);
        ARMRegister base64(address.base, 64);
        ARMRegister index64(address.index, 64);

        Add(dest64, base64, Operand(index64, LSL, address.scale));
        if (address.offset)
            Add(dest64, dest64, Operand(address.offset));
    }

  private:
    void setupABICall(uint32_t args);

  public:
    // Setup a call to C/C++ code, given the number of general arguments it
    // takes. Note that this only supports cdecl.
    //
    // In order for alignment to work correctly, the MacroAssembler must have a
    // consistent view of the stack displacement. It is okay to call "push"
    // manually, however, if the stack alignment were to change, the macro
    // assembler should be notified before starting a call.
    void setupAlignedABICall(uint32_t args) {
        MOZ_CRASH("setupAlignedABICall");
    }

    // Sets up an ABI call for when the alignment is not known. This may need a
    // scratch register.
    void setupUnalignedABICall(uint32_t args, Register scratch);

    // Arguments must be assigned to a C/C++ call in order. They are moved
    // in parallel immediately before performing the call. This process may
    // temporarily use more stack, in which case sp-relative addresses will be
    // automatically adjusted. It is extremely important that sp-relative
    // addresses are computed *after* setupABICall(). Furthermore, no
    // operations should be emitted while setting arguments.
    void passABIArg(const MoveOperand &from, MoveOp::Type type);
    void passABIArg(Register reg);
    void passABIArg(FloatRegister reg, MoveOp::Type type);
    void passABIOutParam(Register reg);

  private:
    void callWithABIPre(uint32_t *stackAdjust);
    void callWithABIPost(uint32_t stackAdjust, MoveOp::Type result);

  public:
    // Emits a call to a C/C++ function, resolving all argument moves.
    void callWithABI(void *fun, MoveOp::Type result = MoveOp::GENERAL);
    void callWithABI(Register fun, MoveOp::Type result = MoveOp::GENERAL);
    void callWithABI(AsmJSImmPtr imm, MoveOp::Type result = MoveOp::GENERAL);
    void callWithABI(Address fun, MoveOp::Type result = MoveOp::GENERAL);

    CodeOffsetLabel labelForPatch() {
        return CodeOffsetLabel(nextOffset().getOffset());
    }

    void handleFailureWithHandlerTail(void *handler);

    // FIXME: This is the same on all platforms. Can be common code?
    void makeFrameDescriptor(Register frameSizeReg, FrameType type) {
        lshiftPtr(Imm32(FRAMESIZE_SHIFT), frameSizeReg);
        orPtr(Imm32(type), frameSizeReg);
    }

    void callWithExitFrame(JitCode *target, Register dynStack) {
        uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);
        Push(Imm32(descriptor)); // descriptor

        call(target);
    }

    // FIXME: See CodeGeneratorX64 calls to noteAsmJSGlobalAccess.
    void patchAsmJSGlobalAccess(CodeOffsetLabel patchAt, uint8_t *code,
                                uint8_t *globalData, unsigned globalDataOffset)
    {
        MOZ_CRASH("patchAsmJSGlobalAccess");
    }

    void memIntToValue(Address Source, Address Dest) {
        load32(Source, ScratchReg2);
        storeValue(JSVAL_TYPE_INT32, ScratchReg2, Dest);
    }

    void branchPtrInNurseryRange(Condition cond, Register ptr, Register temp, Label *label);
    void branchValueIsNurseryObject(Condition cond, ValueOperand value, Register temp, Label *label);

    // Builds an exit frame on the stack, with a return address to an internal
    // non-function. Returns offset to be passed to markSafepointAt().
    void buildFakeExitFrame(Register scratch, uint32_t *offset) {
        mozilla::DebugOnly<uint32_t> initialDepth = framePushed();
        uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);

        Push(Imm32(descriptor)); // descriptor_

        enterNoPool(3);
        Label fakeCallsite;
        Adr(ARMRegister(scratch, 64), &fakeCallsite);
        Push(scratch);
        bind(&fakeCallsite);
        uint32_t pseudoReturnOffset = currentOffset();
        leaveNoPool();

        MOZ_ASSERT(framePushed() == initialDepth + ExitFrameLayout::Size());

        *offset = pseudoReturnOffset;
    }

    void callWithExitFrame(Label *target) {
        breakpoint();
    }

    void callWithExitFrame(JitCode *target) {
        uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);
        Push(Imm32(descriptor)); // descriptor

        call(target);
    }

    void callJit(Register callee) {
        // AArch64 cannot read from the PC, so pushing must be handled callee-side.
        syncStackPtr();
        Blr(ARMRegister(callee, 64));
    }

    void appendCallSite(const CallSiteDesc &desc) {
        MOZ_CRASH("appendCallSite");
    }

    void call(const CallSiteDesc &desc, Label *label) {
        syncStackPtr();
        call(label);
        append(desc, currentOffset(), framePushed_);
    }
    void call(const CallSiteDesc &desc, Register reg) {
        syncStackPtr();
        call(reg);
        append(desc, currentOffset(), framePushed_);
    }
    void call(const CallSiteDesc &desc, AsmJSImmPtr imm) {
        syncStackPtr();
        call(imm);
        append(desc, currentOffset(), framePushed_);
    }

    void call(AsmJSImmPtr imm) {
        syncStackPtr();
        movePtr(imm, ScratchReg2);
        call(ScratchReg2);
    }

    void call(Register target) {
        syncStackPtr();
        Blr(ARMRegister(target, 64));
    }
    // Call a target JitCode, which must be traceable, and may be movable.
    void call(JitCode *target) {
        syncStackPtr();
        BufferOffset off = immPool64(ScratchReg2_64, uint64_t(target->raw()));
        addPendingJump(off, ImmPtr(target->raw()), Relocation::JITCODE);
        blr(ScratchReg2_64);
    }
    // Call a target native function, which is neither traceable nor movable.
    void call(ImmPtr target) {
        syncStackPtr();
        movePtr(target, ip0);
        Blr(ip0_64);
    }
    void call(Label *target) {
        syncStackPtr();
        Bl(target);
    }
    void callExit(AsmJSImmPtr imm, uint32_t stackArgBytes) {
        MOZ_CRASH("callExit");
    }

    void callJitFromAsmJS(Register reg) {
        Blr(ARMRegister(reg, 64));
    }

    void callAndPushReturnAddress(Label *label) {
        // FIXME: Jandem said he would refactor the code to avoid making
        // this instruction required, but probably forgot about it.
        // Instead of implementing this function, we should make it unnecessary.
        Label ret;
        Adr(ScratchReg2_64, &ret);
        Push(ScratchReg2);
        Bl(label);
        bind(&ret);
    }
    void profilerEnterFrame(Register framePtr, Register scratch) {
        AbsoluteAddress activation(GetJitContext()->runtime->addressOfProfilingActivation());
        loadPtr(activation, scratch);
        storePtr(framePtr, Address(scratch, JitActivation::offsetOfLastProfilingFrame()));
        storePtr(ImmPtr(nullptr), Address(scratch, JitActivation::offsetOfLastProfilingCallSite()));
    }
    void profilerExitFrame() {
        branch(GetJitContext()->runtime->jitRuntime()->getProfilerExitFrameTail());
    }
    Address ToPayload(Address value) {
        return value;
    }
    Address ToType(Address value) {
        return value;
    }

  private:
    template<typename T>
    void compareExchange(int nbytes, bool signExtend, const T &address, Register oldval,
                         Register newval, Register output)
    {
        MOZ_CRASH("compareExchange");
    }

    template<typename T>
    void atomicFetchOp(int nbytes, bool signExtend, AtomicOp op, const Imm32 &value,
                       const T &address, Register temp, Register output)
    {
        MOZ_CRASH("atomicFetchOp");
    }

    template<typename T>
    void atomicFetchOp(int nbytes, bool signExtend, AtomicOp op, const Register &value,
                       const T &address, Register temp, Register output)
    {
        MOZ_CRASH("atomicFetchOp");
    }

  public:
    // T in {Address,BaseIndex}
    // S in {Imm32,Register}

    template<typename T>
    void compareExchange8SignExtend(const T &mem, Register oldval, Register newval, Register output)
    {
        compareExchange(1, true, mem, oldval, newval, output);
    }
    template<typename T>
    void compareExchange8ZeroExtend(const T &mem, Register oldval, Register newval, Register output)
    {
        compareExchange(1, false, mem, oldval, newval, output);
    }
    template<typename T>
    void compareExchange16SignExtend(const T &mem, Register oldval, Register newval, Register output)
    {
        compareExchange(2, true, mem, oldval, newval, output);
    }
    template<typename T>
    void compareExchange16ZeroExtend(const T &mem, Register oldval, Register newval, Register output)
    {
        compareExchange(2, false, mem, oldval, newval, output);
    }
    template<typename T>
    void compareExchange32(const T &mem, Register oldval, Register newval, Register output)  {
        compareExchange(4, false, mem, oldval, newval, output);
    }

    template<typename T, typename S>
    void atomicFetchAdd8SignExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(1, true, AtomicFetchAddOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAdd8ZeroExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(1, false, AtomicFetchAddOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAdd16SignExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(2, true, AtomicFetchAddOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAdd16ZeroExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(2, false, AtomicFetchAddOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAdd32(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(4, false, AtomicFetchAddOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchSub8SignExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(1, true, AtomicFetchSubOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchSub8ZeroExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(1, false, AtomicFetchSubOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchSub16SignExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(2, true, AtomicFetchSubOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchSub16ZeroExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(2, false, AtomicFetchSubOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchSub32(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(4, false, AtomicFetchSubOp, value, mem, temp, output);
    }

    template<typename T, typename S>
    void atomicFetchAnd8SignExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(1, true, AtomicFetchAndOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAnd8ZeroExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(1, false, AtomicFetchAndOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAnd16SignExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(2, true, AtomicFetchAndOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAnd16ZeroExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(2, false, AtomicFetchAndOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAnd32(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(4, false, AtomicFetchAndOp, value, mem, temp, output);
    }

    template<typename T, typename S>
    void atomicFetchOr8SignExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(1, true, AtomicFetchOrOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchOr8ZeroExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(1, false, AtomicFetchOrOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchOr16SignExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(2, true, AtomicFetchOrOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchOr16ZeroExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(2, false, AtomicFetchOrOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchOr32(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(4, false, AtomicFetchOrOp, value, mem, temp, output);
    }

    template<typename T, typename S>
    void atomicFetchXor8SignExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(1, true, AtomicFetchXorOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchXor8ZeroExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(1, false, AtomicFetchXorOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchXor16SignExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(2, true, AtomicFetchXorOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchXor16ZeroExtend(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(2, false, AtomicFetchXorOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchXor32(const S &value, const T &mem, Register temp, Register output) {
        atomicFetchOp(4, false, AtomicFetchXorOp, value, mem, temp, output);
    }

    // Emit a BLR or NOP instruction. ToggleCall can be used to patch
    // this instruction.
    CodeOffsetLabel toggledCall(JitCode *target, bool enabled) {
        BufferOffset offset = nextOffset();
        BufferOffset loadOffset;
        syncStackPtr();

        // TODO: Random-ass pool insertion between instructions below is terrible.
        // Unfortunately, we can't forbid pool prevention, because we're trying
        // to add an entry to a pool. So as a temporary fix, just flush the pool
        // now, so that it won't add later. If you're changing this, also
        // check ToggleCall(), which will probably break.
        armbuffer_.flushPool();

        if (enabled) {
            loadOffset = immPool64(ScratchReg2_64, uint64_t(target->raw()));
            blr(ScratchReg2_64);
        } else {
            loadOffset = immPool64(ScratchReg2_64, uint64_t(target->raw()));
            nop();
        }
        addPendingJump(loadOffset, ImmPtr(target->raw()), Relocation::JITCODE);
        CodeOffsetLabel ret(offset.getOffset());
        return ret;
    }

    static size_t ToggledCallSize(uint8_t *code) {
        // start it off as an 8 byte sequence
        int ret = 8;
        Instruction *cur = (Instruction*)code;
        uint32_t *curw = (uint32_t*)code;
        // oh god, this is bad, just hard-code the stack sync instruction
        if (*curw == 0x9100039f) {
            ret += 4;
            cur += 4;
        }
        if (cur->IsUncondB()) {
            ret += cur->ImmPCRawOffset() << kInstructionSizeLog2;
        }
        return ret;
    }

    void checkARMRegAlignment(const ARMRegister &reg) {
#ifdef DEBUG
        Label aligned;
        Add(ScratchReg2_64, reg, Operand(0)); // Move even if reg == sp. Avoids xzr.
        Tst(ScratchReg2_64, Operand(StackAlignment - 1));
        B(Zero, &aligned);
        breakpoint();
        bind(&aligned);
        Mov(ScratchReg2_64, xzr); // Clear the scratch register for sanity.
#endif
    }

    void checkStackAlignment() {
#ifdef DEBUG
        checkARMRegAlignment(GetStackPointer());

        // If another register is being used to track pushes, check sp explicitly.
        if (!GetStackPointer().Is(sp))
            checkARMRegAlignment(sp);
#endif
    }

    void abiret() {
        syncStackPtr();
        MacroAssemblerVIXL::Ret(lr_64);
    }

    void mulBy3(Register src, Register dest) {
        ARMRegister xdest(dest, 64);
        ARMRegister xsrc(src, 64);
        Add(xdest, xsrc, Operand(xsrc, LSL, 1));
    }

    template <typename T>
    void branchAdd32(Condition cond, T src, Register dest, Label *label) {
        adds32(src, dest);
        branch(cond, label);
    }

    template <typename T>
    void branchSub32(Condition cond, T src, Register dest, Label *label) {
        subs32(src, dest);
        branch(cond, label);
    }
    void clampCheck(Register r, Label *handleNotAnInt) {
        MOZ_CRASH("clampCheck");
    }

    void memMove32(Address Source, Address Dest) {
        MOZ_CRASH("memMove32");
    }
    void memMove64(Address Source, Address Dest) {
        MOZ_CRASH("memMove64");
    }

    void stackCheck(ImmWord limitAddr, Label *label) {
        MOZ_CRASH("stackCheck");
    }
    void clampIntToUint8(Register reg) {
        ARMRegister areg(reg, 32);
        Cmp(areg, Operand(areg, UXTB));
        Csel(areg, areg, wzr, Assembler::GreaterThanOrEqual);
        Mov(ScratchReg2_32, Operand(0xff));
        Csel(areg, areg, ScratchReg2_32, Assembler::LessThanOrEqual);
    }

    void incrementInt32Value(const Address &addr) {
        load32(addr, ScratchReg2);
        Add(ScratchReg2_32, ScratchReg2_32, Operand(1));
        store32(ScratchReg2, addr);
    }
    void inc64(AbsoluteAddress dest) {
        MOZ_CRASH("inc64");
    }

    void BoundsCheck(Register ptrReg, Label *onFail, CPURegister zeroMe = noReg) {
        // use tst rather than Tst to *ensure* that a single instrution is generated.
        Cmp(ARMRegister(ptrReg, 32), ARMRegister(HeapLenReg, 32));
        if (!zeroMe.IsNone()) {
            if (zeroMe.IsRegister()) {
                Csel(ARMRegister(zeroMe),
                     ARMRegister(zeroMe),
                     Operand(zeroMe.Is32Bits() ? wzr : xzr),
                     Assembler::Below);
            } else if (zeroMe.Is32Bits()) {
                Fmov(ScratchFloat32Reg_, JS::GenericNaN());
                Fcsel(ARMFPRegister(zeroMe), ARMFPRegister(zeroMe), ScratchFloat32Reg_, Assembler::Below);
            } else {
                Fmov(ScratchDoubleReg_, JS::GenericNaN());
                Fcsel(ARMFPRegister(zeroMe), ARMFPRegister(zeroMe), ScratchDoubleReg_, Assembler::Below);
            }
        }
        B(onFail, Assembler::AboveOrEqual);
    }
    void breakpoint();

    // Emits a simulator directive to save the current sp on an internal stack.
    void simulatorMarkSP() {
#ifdef JS_ARM64_SIMULATOR
        svc(kMarkStackPointer);
#endif
    }

    // Emits a simulator directive to pop from its internal stack
    // and assert that the value is equal to the current sp.
    void simulatorCheckSP() {
#ifdef JS_ARM64_SIMULATOR
        svc(kCheckStackPointer);
#endif
    }

    void loadAsmJSActivation(Register dest) {
        loadPtr(Address(GlobalReg, AsmJSActivationGlobalDataOffset - AsmJSGlobalRegBias), dest);
    }
    void loadAsmJSHeapRegisterFromGlobalData() {
        loadPtr(Address(GlobalReg, AsmJSHeapGlobalDataOffset - AsmJSGlobalRegBias), HeapReg);
        loadPtr(Address(GlobalReg, AsmJSHeapGlobalDataOffset - AsmJSGlobalRegBias + 8), HeapLenReg);
    }
    // This moves an un-tagged value from src into a
    // dest that already has the correct tag, and /anything/ in the lower bits
    void monoTagMove(Register dest, Register src) {
        Bfi(ARMRegister(dest, 64), ARMRegister(src, 64), 0, JSVAL_TAG_SHIFT);
    }
    void monoTagMove(ARMRegister dest, ARMRegister src) {
        const ARMRegister csrc = src;
        if (csrc.code() != 31) {
            Bfi(dest, src, 0, JSVAL_TAG_SHIFT);
        } else {
            And(dest, src, Operand((int64_t(1) << JSVAL_TAG_SHIFT) - int64_t(1)));
        }
    }
    // FIXME: Should be in Assembler?
    // FIXME: Should be const?
    uint32_t currentOffset() {
        uint32_t offset = nextOffset().getOffset();
        return offset;
    }

  protected:
    bool buildOOLFakeExitFrame(void *fakeReturnAddr) {
        MOZ_CRASH("buildOOLFakeExitFrame");
    }
};

typedef MacroAssemblerCompat MacroAssemblerSpecific;

} // namespace jit
} // namespace js

#endif // jit_arm64_MacroAssembler_arm64_h
