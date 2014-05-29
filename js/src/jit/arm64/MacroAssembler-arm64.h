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

#ifndef VIXL_A64_MACRO_ASSEMBLER_A64_H_
#define VIXL_A64_MACRO_ASSEMBLER_A64_H_

#include "jit/IonFrames.h"
#include "jit/MoveResolver.h"

#include "jit/arm64/vixl/VIXL-Globals-vixl.h"
#include "jit/arm64/Assembler-arm64.h"
#include "jit/arm64/vixl/Debugger-vixl.h"
class Operand {
    // lolwut? it looks like CodeGenerator is accessing this directly?
    // That should probably be changed
};
#define LS_MACRO_LIST(V)                                      \
  V(Ldrb, Register&, rt, LDRB_w)                              \
  V(Strb, Register&, rt, STRB_w)                              \
  V(Ldrsb, Register&, rt, rt.Is64Bits() ? LDRSB_x : LDRSB_w)  \
  V(Ldrh, Register&, rt, LDRH_w)                              \
  V(Strh, Register&, rt, STRH_w)                              \
  V(Ldrsh, Register&, rt, rt.Is64Bits() ? LDRSH_x : LDRSH_w)  \
  V(Ldr, CPURegister&, rt, LoadOpFor(rt))                     \
  V(Str, CPURegister&, rt, StoreOpFor(rt))                    \
  V(Ldrsw, Register&, rt, LDRSW_x)

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

class ARMOperand
{
  public:
    enum Kind {
      REG,
      FPREG,
      MEM
    };

  private:
    Kind kind_ : 2;
    int32_t reg_ : 5;
    int32_t offset_;

  public:
    explicit ARMOperand(Register reg)
      : kind_(REG),
        reg_(reg.code())
    { }
    explicit ARMOperand(FloatRegister fpreg)
      : kind_(REG),
        reg_(fpreg.code())
    { }
    explicit ARMOperand(Register base, Imm32 offset)
      : kind_(MEM),
        reg_(base.code()),
        offset_(offset.value)
    { }
    explicit ARMOperand(Register base, int32_t offset)
      : kind_(MEM),
        reg_(base.code()),
        offset_(offset)
    { }
    explicit ARMOperand(const Address &addr)
      : kind_(MEM),
        reg_(addr.base.code()),
        offset_(addr.offset)
    { }
};

class MacroAssemblerARM64 : public Assembler
{
  protected:
    uint32_t framePushed_;

  public:
    MacroAssemblerARM64()
      : Assembler(NULL, 0) // FIXME: Integrate the Assembler with some buffer.
    { }

  protected:
    MoveResolver moveResolver_;

  public:
    // FIXME: This is the size of the buffer -- should really be in Assembler.
    int32_t size() const {
        JS_ASSERT(0 && "size");
        return 0;
    }
    bool oom() const {
        JS_ASSERT(0 && "oom");
        return false;
    }

    template <typename T>
    void Push(const T &t) {
        JS_ASSERT(0 && "Push()");
    }
    CodeOffsetLabel PushWithPatch(ImmWord word) {
        JS_ASSERT(0 && "PushWithPatch");
        return CodeOffsetLabel(0x0);
    }
    CodeOffsetLabel PushWithPatch(ImmPtr ptr) {
        JS_ASSERT(0 && "PushWithPatch");
        return CodeOffsetLabel(0x0);
    }

    // FIXME: Should be in assembler, or IonMacroAssembler shouldn't use.
    template <typename T>
    void push(const T &t) {
        JS_ASSERT(0 && "push");
    }

    template <typename T>
    void Pop(const T &t) {
        JS_ASSERT(0 && "Pop()");
    }

    // FIXME: Should be in assembler, or IonMacroAssembler shouldn't use.
    template <typename T>
    void pop(const T &t) {
        JS_ASSERT(0 && "pop");
    }

    void implicitPop(uint32_t args) {
        JS_ASSERT(0 && "implicitPop");
    }
    uint32_t framePushed() const {
        JS_ASSERT(0 && "framePushed");
        return 0;
    }
    void setFramePushed(uint32_t framePushed) {
        JS_ASSERT(0 && "setFramePushed");
    }

    void reserveStack(uint32_t amount) {
        JS_ASSERT(0 && "reserveStack");
    }
    void freeStack(uint32_t amount) {
        JS_ASSERT(0 && "freeStack");
    }
    void freeStack(Register amount) {
        JS_ASSERT(0 && "freeStack");
    }

    void storeValue(ValueOperand val, ARMOperand dest) {
        JS_ASSERT(0 && "storeValue");
    }
    void storeValue(ValueOperand val, const Address &dest) {
        JS_ASSERT(0 && "storeValue");
    }
    template <typename T>
    void storeValue(JSValueType type, Register reg, const T &dest) {
        JS_ASSERT(0 && "storeValue");
    }
    template <typename T>
    void storeValue(const Value &val, const T &dest) {
        JS_ASSERT(0 && "storeValue");
    }
    void storeValue(ValueOperand val, BaseIndex dest) {
        JS_ASSERT(0 && "storeValue");
    }
    void loadValue(ARMOperand src, ValueOperand val) {
        JS_ASSERT(0 && "loadValue");
    }
    void loadValue(Address src, ValueOperand val) {
        JS_ASSERT(0 && "loadValue");
    }
    void loadValue(const BaseIndex &src, ValueOperand val) {
        JS_ASSERT(0 && "loadValue");
    }
    void tagValue(JSValueType type, Register payload, ValueOperand dest) {
        JS_ASSERT(0 && "tagValue");
    }
    void pushValue(ValueOperand val) {
        JS_ASSERT(0 && "pushValue");
    }
    void Push(const ValueOperand &val) {
        JS_ASSERT(0 && "Push");
    }
    void popValue(ValueOperand val) {
        JS_ASSERT(0 && "popValue");
    }
    void pushValue(const Value &val) {
        JS_ASSERT(0 && "pushValue");
    }
    void pushValue(JSValueType type, Register reg) {
        JS_ASSERT(0 && "pushValue");
    }
    void pushValue(const Address &addr) {
        JS_ASSERT(0 && "pushValue");
    }
    void moveValue(const Value &val, Register dest) {
        JS_ASSERT(0 && "moveValue");
    }
    void moveValue(const Value &src, const ValueOperand &dest) {
        JS_ASSERT(0 && "moveValue");
    }
    void moveValue(const ValueOperand &src, const ValueOperand &dest) {
        JS_ASSERT(0 && "moveValue");
    }

    CodeOffsetLabel pushWithPatch(ImmWord imm) {
        JS_ASSERT(0 && "pushWithPatch");
    }

    CodeOffsetLabel movWithPatch(ImmWord imm, Register dest) {
        JS_ASSERT(0 && "moveWithPatch");
    }
    CodeOffsetLabel movWithPatch(ImmPtr imm, Register dest) {
      JS_ASSERT(0 && "moveWithPatch");
    }

    void boxValue(JSValueType type, Register src, Register dest) {
        JS_ASSERT(0 && "boxValue");
    }

    Register extractTag(const Address &address, Register scratch) {
        JS_ASSERT(0 && "extractTag()");
        return scratch;
    }
    Register extractTag(const ValueOperand &value, Register scratch) {
        JS_ASSERT(0 && "extractTag()");
        return scratch;
    }
    Register extractObject(const Address &address, Register scratch) {
        JS_ASSERT(0 && "extractObject()");
        return scratch;
    }
    Register extractObject(const ValueOperand &value, Register scratch) {
        JS_ASSERT(0 && "extractObject()");
        return scratch;
    }
    Register extractInt32(const ValueOperand &value, Register scratch) {
        JS_ASSERT(0 && "extractInt32()");
        return scratch;
    }
    Register extractBoolean(const ValueOperand &value, Register scratch) {
        JS_ASSERT(0 && "extractBoolean()");
        return scratch;
    }

    // If source is a double, load into dest.
    // If source is int32, convert to double and store in dest.
    // Else, branch to failure.
    void ensureDouble(const ValueOperand &source, FloatRegister dest, Label *failure) {
        JS_ASSERT(0 && "ensureDouble()");
    }
    void
    emitSet(Assembler::Condition cond, Register dest)
    {
        JS_ASSERT(0 && "emitSet");
    }

    template <typename T1, typename T2>
    void cmpPtrSet(Assembler::Condition cond, T1 lhs, T2 rhs, Register dest)
    {
        cmpPtr(lhs, rhs);
        emitSet(cond, dest);
    }
    template <typename T1, typename T2>
    void cmp32Set(Assembler::Condition cond, T1 lhs, T2 rhs, Register dest)
    {
        cmp32(lhs, rhs);
        emitSet(cond, dest);
    }

    void testNullSet(Condition cond, const ValueOperand &value, Register dest) {
        cond = testNull(cond, value);
        emitSet(cond, dest);
    }
    void testUndefinedSet(Condition cond, const ValueOperand &value, Register dest) {
        cond = testUndefined(cond, value);
        emitSet(cond, dest);
    }
    void convertBoolToInt32(Register source, Register dest) {
        JS_ASSERT(0 && "convertBoolToInt32");
    }
    void convertInt32ToDouble(Register src, FloatRegister dest) {
        JS_ASSERT(0 && "convertInt32ToDouble");
    }
    void convertInt32ToDouble(const Address &src, FloatRegister dest) {
        JS_ASSERT(0 && "convertInt32ToDouble");
    }
    void convertInt32ToDouble(const ARMOperand &src, FloatRegister dest) {
        JS_ASSERT(0 && "convertInt32ToDouble");
    }
    void convertInt32ToFloat32(Register src, FloatRegister dest) {
        JS_ASSERT(0 && "convertInt32ToFloat32");
    }
    void convertInt32ToFloat32(const Address &src, FloatRegister dest) {
        JS_ASSERT(0 && "convertInt32ToFloat32");
    }
    void convertInt32ToFloat32(const ARMOperand &src, FloatRegister dest) {
        JS_ASSERT(0 && "convertInt32ToFloat32");
    }

    void convertUInt32ToDouble(const Address &src, FloatRegister dest) {
        JS_ASSERT(0 && "convertUInt32ToDouble");
    }
    void convertUInt32ToDouble(const Register &src, FloatRegister dest) {
        JS_ASSERT(0 && "convertUInt32ToDouble");
    }
    void convertUInt32ToFloat32(Register src, FloatRegister dest) {
        JS_ASSERT(0 && "convertUInt32ToFloat32");
    }
    void convertUInt32ToFloat32(const Address &src, FloatRegister dest) {
        JS_ASSERT(0 && "convertUInt32ToFloat32");
    }
    void convertUInt32ToFloat32(const Register &src, FloatRegister dest) {
        JS_ASSERT(0 && "convertUInt32ToFloat32");
    }

    void convertFloat32ToDouble(FloatRegister src, FloatRegister dest) {
        JS_ASSERT(0 && "convertFloat32ToDouble");
    }
    void convertDoubleToFloat32(FloatRegister src, FloatRegister dest) {
        JS_ASSERT(0 && "convertDoubleToFloat32");
    }
    void branchTruncateDouble(FloatRegister src, Register dest, Label *fail) {
        JS_ASSERT(0 && "branchTruncateDouble");
    }
    void convertDoubleToInt32(FloatRegister src, Register dest, Label *fail,
                              bool negativeZeroCheck = true) {
        JS_ASSERT(0 && "convertDoubleToInt32");
    }
    void convertFloat32ToInt32(FloatRegister src, Register dest, Label *fail,
                               bool negativeZeroCheck = true) {
        JS_ASSERT(0 && "convertFloat32ToInt32");
    }

    void branchTruncateFloat32(FloatRegister src, Register dest, Label *fail) {
        JS_ASSERT(0 && "branchTruncateFloat32");
    }
    void jump(Label *label) {
        JS_ASSERT(0 && "jump");
    }
    void jump(RepatchLabel *label) {
        JS_ASSERT(0 && "jump");
    }
    void jump(Register reg) {
        JS_ASSERT(0 && "jump");
    }
    void jump(const Address &addr) {
        JS_ASSERT(0 && "jump");
    }

    void align(int alignment) {
        JS_ASSERT(0 && "align");
    }

    void movePtr(Register src, Register dest) {
        JS_ASSERT(0 && "movePtr");
    }
    void movePtr(Register src, const ARMOperand &dest) {
        JS_ASSERT(0 && "movePtr");
    }
    void movePtr(ImmWord imm, Register dest) {
        JS_ASSERT(0 && "movePtr");
    }
    void movePtr(ImmPtr imm, Register dest) {
        JS_ASSERT(0 && "movePtr");
    }
    void movePtr(AsmJSImmPtr imm, Register dest) {
        JS_ASSERT(0 && "movePtr");
    }
    void movePtr(ImmGCPtr imm, Register dest) {
        JS_ASSERT(0 && "movePtr");
    }
    void move32(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "move32");
    }
    void move32(Register src, Register dest) {
        JS_ASSERT(0 && "move32");
    }
    void mov(ImmWord word, Register dest) {
        JS_ASSERT(0 && "mov");
    }
    void mov(ImmPtr imm, Register dest) {
        mov(ImmWord(uintptr_t(imm.value)), dest);
    }

    void mov(Register src, Register dest) {
        JS_ASSERT(0 && "mov");
    }

    void not32(Register reg) {
        JS_ASSERT(0 && "not");
    }
    void neg32(Register reg) {
        JS_ASSERT(0 && "not");
    }

    void loadPtr(AbsoluteAddress address, Register dest) {
        JS_ASSERT(0 && "loadPtr");
    }
    void loadPtr(const Address &address, Register dest) {
        JS_ASSERT(0 && "loadPtr");
    }
    void loadPtr(const ARMOperand &src, Register dest) {
        JS_ASSERT(0 && "loadPtr");
    }
    void loadPtr(const BaseIndex &src, Register dest) {
        JS_ASSERT(0 && "loadPtr");
    }
    void loadPrivate(const Address &src, Register dest) {
        JS_ASSERT(0 && "loadPrivate");
    }
    void load32(AbsoluteAddress address, Register dest) {
        JS_ASSERT(0 && "load32");
    }

    void store8(Register src, const Address &address) {
        JS_ASSERT(0 && "store8");
    }
    void store8(Imm32 imm, const Address &address) {
        JS_ASSERT(0 && "store8");
    }
    void store8(Register src, const BaseIndex &address);
    void store8(Imm32 imm, const BaseIndex &address);

    void store16(Register src, const Address &address) {
        JS_ASSERT(0 && "store16");
    }
    void store16(Imm32 imm, const Address &address) {
        JS_ASSERT(0 && "store16");
    }
    void store16(Register src, const BaseIndex &address) {
        JS_ASSERT(0 && "store16");
    }
    void store16(Imm32 imm, const BaseIndex &address) {
        JS_ASSERT(0 && "store16");
    }

    void storePtr(ImmWord imm, const Address &address) {
        JS_ASSERT(0 && "storePtr");
    }
    void storePtr(ImmPtr imm, const Address &address) {
        JS_ASSERT(0 && "storePtr");
    }
    void storePtr(ImmGCPtr imm, const Address &address) {
        JS_ASSERT(0 && "storePtr");
    }
    void storePtr(Register src, const Address &address) {
        JS_ASSERT(0 && "storePtr");
    }
    void storePtr(Register src, const BaseIndex &address) {
        JS_ASSERT(0 && "storePtr");
    }
    void storePtr(Register src, const ARMOperand &dest) {
        JS_ASSERT(0 && "storePtr");
    }
    void storePtr(Register src, AbsoluteAddress address) {
        JS_ASSERT(0 && "storePtr");
    }
    void store32(Register src, AbsoluteAddress address) {
        JS_ASSERT(0 && "store32");
    }
    void rshiftPtr(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "rshiftPtr");
    }
    void rshiftPtrArithmetic(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "rshiftPtrArithmetic");
    }
    void lshiftPtr(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "lshiftPtr");
    }
    void xorPtr(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "xorPtr");
    }
    void xor32(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "xorPtr");
    }

    void xorPtr(Register src, Register dest) {
        JS_ASSERT(0 && "xorPtr");
    }
    void orPtr(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "orPtr");
    }
    void orPtr(Register src, Register dest) {
        JS_ASSERT(0 && "orPtr");
    }
    void andPtr(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "andPtr");
    }
    void and32(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "and32");
    }
    void and32(Register src, Register dest) {
        JS_ASSERT(0 && "and32");
    }
    void and32(Imm32 mask, Address dest) {
        JS_ASSERT(0 && "and32");
    }

    void andPtr(Register src, Register dest) {
        JS_ASSERT(0 && "andPtr");
    }
    void or32(Imm32 imm, const Address &dest) {
        JS_ASSERT(0 && "or32");
    }


    void test32(Register lhs, Register rhs) {
      JS_ASSERT(0 && "test32");
    }
    void test32(const Address &addr, Imm32 imm) {
        JS_ASSERT(0 && "test32");
    }
    void test32(Register lhs, Imm32 rhs) {
        JS_ASSERT(0 && "test32");
    }
    void cmp32(Register lhs, Imm32 rhs) {
      JS_ASSERT(0 && "cmp32");
    }
    void cmp32(Register a, Register b) {
      JS_ASSERT(0 && "cmp32");
    }
    void cmp32(const Operand &lhs, Imm32 rhs) {
      JS_ASSERT(0 && "cmp32");
    }
    void cmp32(const Operand &lhs, Register rhs) {
      JS_ASSERT(0 && "cmp32");
    }

  void cmpPtr(Register lhs, ImmWord rhs) {
    JS_ASSERT(0 && "cmpPtr");
  }
  void cmpPtr(Register lhs, ImmPtr rhs) {
    JS_ASSERT(0 && "cmpPtr");
  }
  void cmpPtr(Register lhs, Register rhs) {
    JS_ASSERT(0 && "cmpPtr");
  }
  void cmpPtr(Register lhs, ImmGCPtr rhs) {
    JS_ASSERT(0 && "cmpPtr");
  }
  void cmpPtr(Register lhs, Imm32 rhs) {
    JS_ASSERT(0 && "cmpPtr");
  }
  void cmpPtr(const Address &lhs, Register rhs) {
    JS_ASSERT(0 && "cmpPtr");
  }
  void cmpPtr(const Address &lhs, ImmWord rhs) {
    JS_ASSERT(0 && "cmpPtr");
  }
  void cmpPtr(const Address &lhs, ImmPtr rhs) {
    JS_ASSERT(0 && "cmpPtr");  }

    void testPtr(Register lhs, Register rhs) {
        test32(lhs, rhs);
    }

    void loadDouble(const Address &src, FloatRegister dest) {
        JS_ASSERT(0 && "loadDouble");
    }
    void loadDouble(const BaseIndex &src, FloatRegister dest) {
        JS_ASSERT(0 && "loadDouble");
    }
    void loadFloatAsDouble(const Address &addr, FloatRegister dest) {
        JS_ASSERT(0 && "loadFloatAsDouble");
    }
    void loadFloatAsDouble(const BaseIndex &src, FloatRegister dest) {
        JS_ASSERT(0 && "loadFloatAsDouble");
    }

    void loadFloat32(const Address &addr, FloatRegister dest) {
        JS_ASSERT(0 && "loadFloat32");
    }
    void loadFloat32(const BaseIndex &src, FloatRegister dest) {
        JS_ASSERT(0 && "loadFloat32");
    }

    void storeDouble(FloatRegister src, const Address &dest) {
        JS_ASSERT(0 && "storeDouble");
    }
    void storeDouble(FloatRegister src, const BaseIndex &dest) {
        JS_ASSERT(0 && "storeDouble");
    }
    void storeFloat32(FloatRegister src, Address addr) {
        JS_ASSERT(0 && "storeFloat32");
    }
    void storeFloat32(FloatRegister src, BaseIndex addr) {
        JS_ASSERT(0 && "storeFloat32");
    }

    void moveDouble(FloatRegister src, FloatRegister dest) {
        JS_ASSERT(0 && "moveDouble");
    }
    void zeroDouble(FloatRegister reg) {
        JS_ASSERT(0 && "zeroDouble");
    }
    void zeroFloat32(FloatRegister reg) {
        JS_ASSERT(0 && "zeroFloat32");
    }
    void negateDouble(FloatRegister reg) {
        JS_ASSERT(0 && "negateDouble");
    }
    void negateFloat(FloatRegister reg) {
        JS_ASSERT(0 && "negateFloat");
    }
    void addDouble(FloatRegister src, FloatRegister dest) {
        JS_ASSERT(0 && "addDouble");
    }
    void subDouble(FloatRegister src, FloatRegister dest) {
        JS_ASSERT(0 && "subDouble");
    }
    void mulDouble(FloatRegister src, FloatRegister dest) {
        JS_ASSERT(0 && "mulDouble");
    }
    void divDouble(FloatRegister src, FloatRegister dest) {
        JS_ASSERT(0 && "divDouble");
    }


    void moveFloat32(FloatRegister src, FloatRegister dest) {
        JS_ASSERT(0 && "moveFloat32");
    }

    void moveFloatAsDouble(Register src, FloatRegister dest) {
        JS_ASSERT(0 && "moveFloatAsDouble");
    }

    void splitTag(Register src, Register dest) {
        JS_ASSERT(0 && "splitTag");
    }
    void splitTag(const ValueOperand &operand, Register dest) {
        JS_ASSERT(0 && "void ");
    }
    void splitTag(const ARMOperand &operand, Register dest) {
        JS_ASSERT(0 && "splitTag");
    }
    void splitTag(const Address &operand, Register dest) {
        JS_ASSERT(0 && "splitTag");
    }
    void splitTag(const BaseIndex &operand, Register dest) {
        JS_ASSERT(0 && "splitTag");
    }

    // Extracts the tag of a value and places it in ScratchReg.
    Register splitTagForTest(const ValueOperand &value) {
        JS_ASSERT(0 && "splitTagForTest");
        return Register::FromCode(Registers::x0);
    }
    void cmpTag(const ValueOperand &operand, ImmTag tag) {
        JS_ASSERT(0 && "cmpTag");
    }

    void load32(const Address &address, Register dest) {
        JS_ASSERT(0 && "load32");
    }
    void load32(const BaseIndex &src, Register dest) {
        JS_ASSERT(0 && "load32");
    }
    void load32(const ARMOperand &src, Register dst) {
        JS_ASSERT(0 && "load32");
    }
    void load8SignExtend(const Address &address, Register dest);
    void load8SignExtend(const BaseIndex &src, Register dest);

    void load8ZeroExtend(const Address &address, Register dest);
    void load8ZeroExtend(const BaseIndex &src, Register dest);

    void load16SignExtend(const Address &address, Register dest);
    void load16SignExtend(const BaseIndex &src, Register dest);

    void load16ZeroExtend(const Address &address, Register dest);
    void load16ZeroExtend(const BaseIndex &src, Register dest);

    template <typename S, typename T>
    void store32(const S &src, const T &dest) {
        JS_ASSERT(0 && "store32");
    }

    void add32(Register src, Register dest) {
        JS_ASSERT(0 && "add32");
    }
    void add32(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "add32");
    }
    void add32(Imm32 imm, const Address &dest) {
        JS_ASSERT(0 && "add32");
    }
    void sub32(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "sub32");
    }
    void sub32(Register src, Register dest) {
        JS_ASSERT(0 && "sub32");
    }

    void addPtr(Register src, Register dest) {
        JS_ASSERT(0 && "addPtr");
    }
    void addPtr(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "addPtr");
    }
    void addPtr(Imm32 imm, const Address &dest) {
        JS_ASSERT(0 && "addPtr");
    }
    void addPtr(Imm32 imm, const ARMOperand &dest) {
        JS_ASSERT(0 && "addPtr");
    }
    void addPtr(ImmWord imm, Register dest) {
        JS_ASSERT(0 && "addPtr");
    }
    void addPtr(ImmPtr imm, Register dest) {
        JS_ASSERT(0 && "addPtr");
    }
    void addPtr(const Address &src, Register dest) {
        JS_ASSERT(0 && "addPtr");
    }
    void subPtr(Imm32 imm, Register dest) {
        JS_ASSERT(0 && "subPtr");
    }
    void subPtr(Register src, Register dest) {
        JS_ASSERT(0 && "subPtr");
    }
    void subPtr(const Address &addr, Register dest) {
        JS_ASSERT(0 && "subPtr");
    }
    void subPtr(Register src, const Address &dest) {
        JS_ASSERT(0 && "subPtr");
    }
    void ret() {
        JS_ASSERT(0 && "ret");
    }

    void j(Condition code , Label *dest)
    {
        JS_ASSERT(0 && "j");
    }
    void j(Label *dest)
    {
        JS_ASSERT(0 && "j");
    }

    void branch(Condition cond, Label *label) {
      JS_ASSERT(0 && "branch");
    }

    void branch16(Condition cond, Register lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branch16");
    }
    void branch32(Condition cond, const Operand &lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branch32");
    }
    void branch32(Condition cond, const Operand &lhs, Imm32 rhs, Label *label) {
        JS_ASSERT(0 && "branch32");
    }
    void branch32(Condition cond, const Address &lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branch32");
    }
    void branch32(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        JS_ASSERT(0 && "branch32");
    }
    void branch32(Condition cond, Register lhs, Imm32 imm, Label *label) {
        JS_ASSERT(0 && "branch32");
    }
    void branch32(Condition cond, Register lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branch32");
    }
    void branch32(Condition cond, AbsoluteAddress lhs, Imm32 rhs, Label *label) {
        JS_ASSERT(0 && "branch32");
    }
    void branch32(Condition cond, AbsoluteAddress lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branch32");
    }


    void branchSub32(Condition cond, const ARMOperand &lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branchSub32");
    }
    void branchSub32(Condition cond, const ARMOperand &lhs, Imm32 rhs, Label *label) {
        JS_ASSERT(0 && "branchSub32");
    }
    void branchSub32(Condition cond, const Address &lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branchSub32");
    }
    void branchSub32(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        JS_ASSERT(0 && "branchSub32");
    }
    void branchSub32(Condition cond, Register lhs, Imm32 imm, Label *label) {
        JS_ASSERT(0 && "branchSub32");
    }
    void branchSub32(Condition cond, Register lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branchSub32");
    }
    void branchSub32(Condition cond, AbsoluteAddress lhs, Imm32 rhs, Label *label) {
        JS_ASSERT(0 && "branchSub32");
    }
    void branchSub32(Condition cond, AbsoluteAddress lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branchSub32");
    }

    void branchTest16(Condition cond, Register lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branchTest16");
    }
    void branchTest32(Condition cond, Register lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branchTest32");
    }
    void branchTest32(Condition cond, Register lhs, Imm32 imm, Label *label) {
        JS_ASSERT(0 && "branchTest32");
    }
    void branchTest32(Condition cond, const Address &address, Imm32 imm, Label *label) {
        JS_ASSERT(0 && "branchTest32");
    }
    CodeOffsetJump jumpWithPatch(RepatchLabel *label, Condition cond = Always) {
        JS_ASSERT(0 && "jumpWithPatch");
    }
    template <typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, Register reg, T ptr, RepatchLabel *label) {
        JS_ASSERT(0 && "branchPtrWithPatch");
    }
    template <typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, Address addr, T ptr, RepatchLabel *label) {
        JS_ASSERT(0 && "branchPtrWithPatch");
    }
    template <typename T, typename S>
    void branchPtr(Condition cond, T lhs, S ptr, Label *label) {
        JS_ASSERT(0 && "branchPtr");
    }
    void branchTestPtr(Condition cond, Register lhs, Register rhs, Label *label) {
        JS_ASSERT(0 && "branchTestPtr");
    }
    void branchTestPtr(Condition cond, Register lhs, Imm32 imm, Label *label) {
        JS_ASSERT(0 && "branchTestPtr");
    }
    void branchTestPtr(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        JS_ASSERT(0 && "branchTestPtr");
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
        JS_ASSERT(0 && "decBranchPtr");
    }

    void branchTestUndefined(Condition cond, Register tag, Label *label) {
        JS_ASSERT(0 && "branchTestUndefined()");
    }
    void branchTestInt32(Condition cond, Register tag, Label *label) {
        JS_ASSERT(0 && "branchTestInt32");
    }
    void branchTestDouble(Condition cond, Register tag, Label *label) {
        JS_ASSERT(0 && "branchTestDouble");
    }
    void branchTestBoolean(Condition cond, Register tag, Label *label) {
        JS_ASSERT(0 && "branchTestBoolean");
    }
    void branchTestNull(Condition cond, Register tag, Label *label) {
        JS_ASSERT(0 && "branchTestNull");
    }
    void branchTestString(Condition cond, Register tag, Label *label) {
        JS_ASSERT(0 && "branchTestString");
    }
    void branchTestObject(Condition cond, Register tag, Label *label) {
        JS_ASSERT(0 && "branchTestObject");
    }
    void branchTestNumber(Condition cond, Register tag, Label *label) {
        JS_ASSERT(0 && "branchTestNumber");
    }


    void branchTestUndefined(Condition cond, const ARMOperand &operand, Label *label) {
        JS_ASSERT(0 && "branchTestUndefined");
    }
    void branchTestUndefined(Condition cond, const Address &address, Label *label) {
        JS_ASSERT(0 && "branchTestUndefined");
    }
    void branchTestInt32(Condition cond, const ARMOperand &operand, Label *label) {
        JS_ASSERT(0 && "branchTestInt32");
    }
    void branchTestInt32(Condition cond, const Address &address, Label *label) {
        JS_ASSERT(0 && "branchTestInt32");
    }
    void branchTestDouble(Condition cond, const ARMOperand &operand, Label *label) {
        JS_ASSERT(0 && "branchTestDouble");
    }
    void branchTestDouble(Condition cond, const Address &address, Label *label) {
        JS_ASSERT(0 && "branchTestDouble");
    }
    void branchTestBoolean(Condition cond, const ARMOperand &operand, Label *label) {
        JS_ASSERT(0 && "branchTestBoolean");
    }
    void branchTestNull(Condition cond, const ARMOperand &operand, Label *label) {
        JS_ASSERT(0 && "branchTestNull");
    }

    // Perform a type-test on a full Value loaded into a register.
    // Clobbers the ScratchReg.
    void branchTestUndefined(Condition cond, const ValueOperand &src, Label *label) {
        JS_ASSERT(0 && "branchTestUndefined");
    }
    void branchTestInt32(Condition cond, const ValueOperand &src, Label *label) {
        JS_ASSERT(0 && "branchTestInt32");
    }
    void branchTestBoolean(Condition cond, const ValueOperand &src, Label *label) {
        JS_ASSERT(0 && "branchTestBoolean");
    }
    void branchTestDouble(Condition cond, const ValueOperand &src, Label *label) {
        JS_ASSERT(0 && "branchTestDouble");
    }
    void branchTestNull(Condition cond, const ValueOperand &src, Label *label) {
        JS_ASSERT(0 && "branchTestNull");
    }
    void branchTestString(Condition cond, const ValueOperand &src, Label *label) {
        JS_ASSERT(0 && "branchTestString");
    }
    void branchTestObject(Condition cond, const ValueOperand &src, Label *label) {
        JS_ASSERT(0 && "branchTestObject");
    }
    void branchTestNumber(Condition cond, const ValueOperand &src, Label *label) {
        JS_ASSERT(0 && "branchTestNumber");
    }

    // Perform a type-test on a Value addressed by BaseIndex.
    // Clobbers the ScratchReg.
    void branchTestUndefined(Condition cond, const BaseIndex &address, Label *label) {
        JS_ASSERT(0 && "branchTestUndefined");
    }
    void branchTestInt32(Condition cond, const BaseIndex &address, Label *label) {
        JS_ASSERT(0 && "branchTestInt32");
    }
    void branchTestBoolean(Condition cond, const BaseIndex &address, Label *label) {
        JS_ASSERT(0 && "branchTestBoolean");
    }
    void branchTestDouble(Condition cond, const BaseIndex &address, Label *label) {
        JS_ASSERT(0 && "branchTestDouble");
    }
    void branchTestNull(Condition cond, const BaseIndex &address, Label *label) {
        JS_ASSERT(0 && "branchTestNull");
    }
    void branchTestString(Condition cond, const BaseIndex &address, Label *label) {
        JS_ASSERT(0 && "branchTestString");
    }
    void branchTestObject(Condition cond, const BaseIndex &address, Label *label) {
        JS_ASSERT(0 && "branchTestObject");
    }
    template <typename T>
    void branchTestGCThing(Condition cond, const T &src, Label *label) {
        JS_ASSERT(0 && "branchTestGCThing");
    }
    template <typename T>
    void branchTestPrimitive(Condition cond, const T &t, Label *label) {
        JS_ASSERT(0 && "branchTestPrimitive");
    }
    template <typename T>
    void branchTestMagic(Condition cond, const T &t, Label *label) {
        JS_ASSERT(0 && "branchTestMagic");
    }
    void branchTestMagicValue(Condition cond, const ValueOperand &val, JSWhyMagic why, Label *label) {
        JS_ASSERT(0 && "branchTestMagicValue");
    }
    Condition testMagic(Condition cond, const ValueOperand &src) {
        JS_ASSERT(0 && "testMagic");
        return Condition::Equal;
    }

    Condition testError(Condition cond, const ValueOperand &src) {
        JS_ASSERT(0 && "testError");
        return Condition::Equal;
    }
    void branchTestValue(Condition cond, const ValueOperand &value, const Value &v, Label *label) {
        JS_ASSERT(0 && "branchTestValue");
    }
    void branchTestValue(Condition cond, const Address &valaddr, const ValueOperand &value,
                         Label *label)
    {
        JS_ASSERT(0 && "branchTestValue");
    }

    void compareDouble(DoubleCondition cond, FloatRegister lhs, FloatRegister rhs) {
        JS_ASSERT(0 && "compareDouble");
    }
    void branchDouble(DoubleCondition cond, FloatRegister lhs, FloatRegister rhs, Label *label) {
        JS_ASSERT(0 && "branchDouble");
    }

    void compareFloat(DoubleCondition cond, FloatRegister lhs, FloatRegister rhs) {
        JS_ASSERT(0 && "compareFloat");
    }
    void branchFloat(DoubleCondition cond, FloatRegister lhs, FloatRegister rhs, Label *label) {
        JS_ASSERT(0 && "branchFloat");
    }

    void branchNegativeZero(FloatRegister reg, Register scratch, Label *label) {
        JS_ASSERT(0 && "branchNegativeZero");
    }
    void branchNegativeZeroFloat32(FloatRegister reg, Register scratch, Label *label) {
        JS_ASSERT(0 && "branchNegativeZeroFloat32");
    }

    void boxDouble(FloatRegister src, const ValueOperand &dest) {
        JS_ASSERT(0 && "boxDouble");
    }
    void boxNonDouble(JSValueType type, Register src, const ValueOperand &dest) {
        JS_ASSERT(0 && "boxNonDouble");
    }

    // Note that the |dest| register here may be ScratchReg, so we shouldn't
    // use it.
    void unboxInt32(const ValueOperand &src, Register dest) {
        JS_ASSERT(0 && "unboxInt32");
    }
    void unboxInt32(const ARMOperand &src, Register dest) {
        JS_ASSERT(0 && "unboxInt32");
    }
    void unboxInt32(const Address &src, Register dest) {
        JS_ASSERT(0 && "unboxInt32");
    }
    void unboxDouble(const Address &src, FloatRegister dest) {
        JS_ASSERT(0 && "unboxDouble");
    }

    void unboxArgObjMagic(const ValueOperand &src, Register dest) {
        JS_ASSERT(0 && "unboxArgObjMagic");
    }
    void unboxArgObjMagic(const ARMOperand &src, Register dest) {
        JS_ASSERT(0 && "unboxArgObjMagic");
    }
    void unboxArgObjMagic(const Address &src, Register dest) {
        JS_ASSERT(0 && "unboxArgObjMagic");
    }

    void unboxBoolean(const ValueOperand &src, Register dest) {
        JS_ASSERT(0 && "unboxBoolean");
    }
    void unboxBoolean(const ARMOperand &src, Register dest) {
        JS_ASSERT(0 && "unboxBoolean");
    }
    void unboxBoolean(const Address &src, Register dest) {
        JS_ASSERT(0 && "unboxBoolean");
    }

    void unboxMagic(const ValueOperand &src, Register dest) {
        JS_ASSERT(0 && "unboxMagic");
    }

    void unboxDouble(const ValueOperand &src, FloatRegister dest) {
        JS_ASSERT(0 && "unboxDouble");
    }
    void unboxPrivate(const ValueOperand &src, const Register dest) {
        JS_ASSERT(0 && "unboxPrivate");
    }

    void notBoolean(const ValueOperand &val) {
        JS_ASSERT(0 && "notBoolean");
    }
    void unboxObject(const ValueOperand &src, Register dest) {
        JS_ASSERT(0 && "unboxObject");
    }
    void unboxObject(const Address &src, Register dest) {
        JS_ASSERT(0 && "unboxObject");
    }

    // Unbox any non-double value into dest. Prefer unboxInt32 or unboxBoolean
    // instead if the source type is known.
    void unboxNonDouble(const ValueOperand &src, Register dest) {
        JS_ASSERT(0 && "unboxNonDouble");
    }
    void unboxNonDouble(const ARMOperand &src, Register dest) {
        // Explicitly permits |dest| to be used in |src|.
        JS_ASSERT(0 && "unboxNonDouble");
    }

    void unboxValue(const ValueOperand &src, AnyRegister dest) {
        JS_ASSERT(0 && "unboxValue");
    }
    void unboxString(const ValueOperand &operand, Register dest) {
        JS_ASSERT(0 && "unboxSTring");
    }
    void unboxString(const Address &src, Register dest) {
        JS_ASSERT(0 && "unboxString");
    }

    // These two functions use the low 32-bits of the full value register.
    void boolValueToDouble(const ValueOperand &operand, FloatRegister dest) {
        JS_ASSERT(0 && "boolValueToDouble");
    }
    void int32ValueToDouble(const ValueOperand &operand, FloatRegister dest) {
        JS_ASSERT(0 && "int32ValueToDouble");
    }

    void boolValueToFloat32(const ValueOperand &operand, FloatRegister dest) {
        JS_ASSERT(0 && "boolValueToFloat32");
    }
    void int32ValueToFloat32(const ValueOperand &operand, FloatRegister dest) {
        JS_ASSERT(0 && "int32ValueToFloat32");
    }

    void loadConstantDouble(double d, FloatRegister dest) {
        JS_ASSERT(0 && "loadConstantDouble");
    }
    void loadConstantFloat32(float f, FloatRegister dest) {
        JS_ASSERT(0 && "loadConstantFloat32");
    }

    Condition testInt32(Condition cond, const ValueOperand &value);
    Condition testBoolean(Condition cond, const ValueOperand &value);
    Condition testDouble(Condition cond, const ValueOperand &value);
    Condition testNull(Condition cond, const ValueOperand &value);
    Condition testUndefined(Condition cond, const ValueOperand &value);
    Condition testString(Condition cond, const ValueOperand &value);
    Condition testObject(Condition cond, const ValueOperand &value);
    Condition testNumber(Condition cond, const ValueOperand &value);

    Condition testPrimitive(Condition cond, const ValueOperand &value);

    // register-based tests
    Condition testInt32(Condition cond, Register tag);
    Condition testBoolean(Condition cond, Register tag);
    Condition testNull(Condition cond, Register tag);
    Condition testUndefined(Condition cond, Register tag);
    Condition testString(Condition cond, Register tag);
    Condition testObject(Condition cond, Register tag);
    Condition testDouble(Condition cond, Register tag);
    Condition testNumber(Condition cond, Register tag);
    Condition testMagic(Condition cond, Register tag);
    Condition testPrimitive(Condition cond, Register tag);

    Condition testGCThing(Condition cond, const Address &address);
    Condition testMagic(Condition cond, const Address &address);
    Condition testInt32(Condition cond, const Address &address);
    Condition testDouble(Condition cond, const Address &address);
    Condition testBoolean(Condition cond, const Address &address);
    Condition testNull(Condition cond, const Address &address);
    Condition testUndefined(Condition cond, const Address &address);
    Condition testString(Condition cond, const Address &address);
    Condition testObject(Condition cond, const Address &address);
    Condition testNumber(Condition cond, const Address &address);

    Condition testUndefined(Condition cond, const BaseIndex &src);
    Condition testNull(Condition cond, const BaseIndex &src);
    Condition testBoolean(Condition cond, const BaseIndex &src);
    Condition testString(Condition cond, const BaseIndex &src);
    Condition testInt32(Condition cond, const BaseIndex &src);
    Condition testObject(Condition cond, const BaseIndex &src);
    Condition testDouble(Condition cond, const BaseIndex &src);
    Condition testMagic(Condition cond, const BaseIndex &src);
    Condition testGCThing(Condition cond, const BaseIndex &src);


    Condition testInt32Truthy(bool truthy, const ValueOperand &operand) {
        JS_ASSERT(0 && "testInt32Truthy");
        return Condition::Zero;
    }
    void branchTestInt32Truthy(bool truthy, const ValueOperand &operand, Label *label) {
        JS_ASSERT(0 && "branchTestInt32Truthy");
    }

    void branchTestDoubleTruthy(bool truthy, FloatRegister reg, Label *label) {
        JS_ASSERT(0 && "branchTestDoubleTruthy");
    }

    void branchTestBooleanTruthy(bool truthy, const ValueOperand &operand, Label *label) {
        JS_ASSERT(0 && "branchTestBooleanTruthy");
    }
    Condition testStringTruthy(bool truthy, const ValueOperand &value) {
        JS_ASSERT(0 && "testStringTruthy");
        return Condition::Zero;
    }
    void branchTestStringTruthy(bool truthy, const ValueOperand &value, Label *label) {
        JS_ASSERT(0 && "branchTestStringTruthy");
    }

    void loadInt32OrDouble(const ARMOperand &operand, FloatRegister dest) {
        JS_ASSERT(0 && "loadInt32OrDouble");
    }

    template <typename T>
    void loadUnboxedValue(const T &src, MIRType type, AnyRegister dest) {
        JS_ASSERT(0 && "loadUnboxedValue");
    }

    void loadInstructionPointerAfterCall(Register dest) {
        JS_ASSERT(0 && "loadInstructionPointerAfterCall");
    }

    // Emit a JMP that can be toggled to a CMP. See ToggleToJmp(), ToggleToCmp().
    CodeOffsetLabel toggledJump(Label *label) {
        JS_ASSERT(0 && "OffsetLabel ");
        CodeOffsetLabel offset(size());
        return offset;
    }

    void bind(Label *label) {
        JS_ASSERT(0 && "bind");
    }
    void bind(RepatchLabel* label) {
        JS_ASSERT(0 && "bind");
    }

    void writeDataRelocation(const Value &val) {
        JS_ASSERT(0 && "writeDataRelocation");
    }
    void writeDataRelocation(ImmGCPtr ptr) {
        JS_ASSERT(0 && "writeDataRelocation");
    }
    void writePrebarrierOffset(CodeOffsetLabel label) {
        JS_ASSERT(0 && "writePrebarrierOffset");
    }

    template <typename T>
    void computeEffectiveAddress(const T &address, Register dest) {
        JS_ASSERT(0 && "computeEffectiveAddress");
    }

    // Setup a call to C/C++ code, given the number of general arguments it
    // takes. Note that this only supports cdecl.
    //
    // In order for alignment to work correctly, the MacroAssembler must have a
    // consistent view of the stack displacement. It is okay to call "push"
    // manually, however, if the stack alignment were to change, the macro
    // assembler should be notified before starting a call.
    void setupAlignedABICall(uint32_t args) {
        JS_ASSERT(0 && "setupAlignedABICall");
    }

    // Sets up an ABI call for when the alignment is not known. This may need a
    // scratch register.
    void setupUnalignedABICall(uint32_t args, Register scratch) {
        JS_ASSERT(0 && "setupUnalignedABICall");
    }

    // Arguments must be assigned to a C/C++ call in order. They are moved
    // in parallel immediately before performing the call. This process may
    // temporarily use more stack, in which case esp-relative addresses will be
    // automatically adjusted. It is extremely important that esp-relative
    // addresses are computed *after* setupABICall(). Furthermore, no
    // operations should be emitted while setting arguments.
    void passABIArg(const MoveOperand &from, MoveOp::Type type) {
        JS_ASSERT(0 && "passABIArg");
    }
    void passABIArg(Register reg) {
        JS_ASSERT(0 && "passABIArg");
    }
    void passABIArg(FloatRegister reg, MoveOp::Type type) {
        JS_ASSERT(0 && "passABIArg");
    }

  private:
    void callWithABIPre(uint32_t *stackAdjust) {
        JS_ASSERT(0 && "callWithABIPre");
    }
    void callWithABIPost(uint32_t stackAdjust, MoveOp::Type result) {
        JS_ASSERT(0 && "callWithABIPost");
    }

  public:
    // Emits a call to a C/C++ function, resolving all argument moves.
    void callWithABI(void *fun, MoveOp::Type result = MoveOp::GENERAL) {
        JS_ASSERT(0 && "callWithABI");
    }
    void callWithABI(AsmJSImmPtr imm, MoveOp::Type result = MoveOp::GENERAL) {
        JS_ASSERT(0 && "callWithABI");
    }
    void callWithABI(Address fun, MoveOp::Type result = MoveOp::GENERAL) {
        JS_ASSERT(0 && "callWithABI");
    }



    CodeOffsetLabel labelForPatch() {
        JS_ASSERT(0 && "labelForPatch");
    }


    void handleFailureWithHandler(void *handler) {
        JS_ASSERT(0 && "handleFailureWithHandler");
    }
    void handleFailureWithHandlerTail() {
        JS_ASSERT(0 && "handleFailureWithHandlerTail");
    }

    void makeFrameDescriptor(Register frameSizeReg, FrameType type) {
        JS_ASSERT(0 && "makeFrameDescriptor");
    }

    // Save an exit frame (which must be aligned to the stack pointer) to
    // PerThreadData::jitTop of the main thread.
    void linkExitFrame() {
        JS_ASSERT(0 && "linkExitFrame");
    }

    void callWithExitFrame(JitCode *target, Register dynStack) {
        JS_ASSERT(0 && "callWithExitFrame");
    }

    // Save an exit frame to the thread data of the current thread, given a
    // register that holds a PerThreadData *.
    void linkParallelExitFrame(Register pt) {
        JS_ASSERT(0 && "linkParallelExitFrame");
    }

    // FIXME: See CodeGeneratorX64 calls to noteAsmJSGlobalAccess.
    void patchAsmJSGlobalAccess(CodeOffsetLabel patchAt, uint8_t *code, uint8_t *globalData,
                                unsigned globalDataOffset)
    {
        JS_ASSERT(0 && "patchAsmJSGlobalAccess");
    }

    // FIXME: These guys probably shouldn't be in other arch's Assemblers...
    // FIXME: Also, they should be capitalized, being static.
    static void patchDataWithValueCheck(CodeLocationLabel data, PatchedImmPtr newData,
                                        PatchedImmPtr expectedData)
    {
        JS_ASSERT(0 && "patchDataWithValueCheck");
    }
    static void patchDataWithValueCheck(CodeLocationLabel data, ImmPtr newData, ImmPtr expectedData) {
        JS_ASSERT(0 && "patchDataWithValueCheck");
    }

    void memIntToValue(Address Source, Address Dest) {
        JS_ASSERT(0 && "memIntToValue");
    }

#ifdef JSGC_GENERATIONAL
    void branchPtrInNurseryRange(Condition cond, Register ptr, Register temp, Label *label) {
        JS_ASSERT(0 && "branchPtrInNurseryRange");
    }
    void branchValueIsNurseryObject(Condition cond, ValueOperand value, Register temp, Label *label) {
        JS_ASSERT(0 && "branchValueIsNurseryObject");
    }
#endif

    // Builds an exit frame on the stack, with a return address to an internal
    // non-function. Returns offset to be passed to markSafepointAt().
    bool buildFakeExitFrame(Register scratch, uint32_t *offset) {
        JS_ASSERT(0 && "buildFakeExitFrame");
        return false;
    }
    void callWithExitFrame(JitCode *target) {
        JS_ASSERT(0 && "callWithExitFrame");
    }

    void callIon(Register callee) {
        JS_ASSERT(0 && "callIon");
    }

    void appendCallSite(const CallSiteDesc &desc) {
        JS_ASSERT(0 && "appendCallSite");
    }

    void call(const CallSiteDesc &desc, Label *label) {
        JS_ASSERT(0 && "call");
    }
    void call(const CallSiteDesc &desc, Register reg) {
        JS_ASSERT(0 && "call");
    }
    void call(const CallSiteDesc &desc, AsmJSImmPtr imm) {
        JS_ASSERT(0 && "call");
    }

    void call(JitCode *target) {
        JS_ASSERT(0 && "call");
    }
    void call(Label *target) {
        JS_ASSERT(0 && "call");
    }
    void callExit(AsmJSImmPtr imm, uint32_t stackArgBytes) {
        JS_ASSERT(0 && "callExit");
    }

    void callIonFromAsmJS(Register reg) {
        JS_ASSERT(0 && "callIonFromAsmJS");
    }

    // Emit a BLX or NOP instruction. ToggleCall can be used to patch
    // this instruction.
    CodeOffsetLabel toggledCall(JitCode *target, bool enabled);

    static size_t ToggledCallSize() {
        JS_ASSERT(0 && "ToggledCallSize");
    }

    void checkStackAlignment() {
        JS_ASSERT(0 && "checkStackAlignment");
    }

    void abiret() {
        JS_ASSERT(0 && "abiret");
    }

    void mulBy3(const Register &src, const Register &dest) {
        JS_ASSERT(0 && "mulBy3");
    }

    template <typename T>
    void branchAdd32(Condition cond, T src, Register dest, Label *label) {
        add32(src, dest);
        branch(cond, label);
    }
    template <typename T>
    void branchSub32(Condition cond, T src, Register dest, Label *label) {
        sub32(src, dest);
        branch(cond, label);
    }
    void clampCheck(Register r, Label *handleNotAnInt) {
        JS_ASSERT(0 && "clampCheck");
    }

    void memMove32(Address Source, Address Dest) {
        JS_ASSERT(0 && "memMove32");
    }
    void memMove64(Address Source, Address Dest) {
        JS_ASSERT(0 && "memMove64");
    }

    void lea(Operand addr, Register dest) {
        JS_ASSERT(0 && "lea");
    }

    void stackCheck(ImmWord limitAddr, Label *label) {
        JS_ASSERT(0 && "stackCheck");
    }
    void clampIntToUint8(Register reg) {
        JS_ASSERT(0 && "clampIntToUint8");
    }

    void incrementInt32Value(const Address &addr) {
        JS_ASSERT(0 && "IncrementInt32Value");
    }
    void inc64(AbsoluteAddress dest) {
        JS_ASSERT(0 && "inc64");
    }
    void breakpoint() {
        JS_ASSERT(0 && "breakpoint");
    }
    // FIXME: Should be in Assembler?
    // FIXME: Should be const?
    uint32_t currentOffset() {
        JS_ASSERT(0 && "currentOffset");
        return 0;
    }


  protected:
    bool buildOOLFakeExitFrame(void *fakeReturnAddr) {
        JS_ASSERT(0 && "buildOOLFakeExitFrame");
    }
};

typedef MacroAssemblerARM64 MacroAssemblerSpecific;

enum BranchType {
  // Copies of architectural conditions.
  // The associated conditions can be used in place of those, the code will
  // take care of reinterpreting them with the correct type.
  integer_eq = eq,
  integer_ne = ne,
  integer_hs = hs,
  integer_lo = lo,
  integer_mi = mi,
  integer_pl = pl,
  integer_vs = vs,
  integer_vc = vc,
  integer_hi = hi,
  integer_ls = ls,
  integer_ge = ge,
  integer_lt = lt,
  integer_gt = gt,
  integer_le = le,
  integer_al = al,
  integer_nv = nv,

  // These two are *different* from the architectural codes al and nv.
  // 'always' is used to generate unconditional branches.
  // 'never' is used to not generate a branch (generally as the inverse
  // branch type of 'always).
  always, never,
  // cbz and cbnz
  reg_zero, reg_not_zero,
  // tbz and tbnz
  reg_bit_clear, reg_bit_set,

  // Aliases.
  kBranchTypeFirstCondition = eq,
  kBranchTypeLastCondition = nv,
  kBranchTypeFirstUsingReg = reg_zero,
  kBranchTypeFirstUsingBit = reg_bit_clear
};

enum DiscardMoveMode { kDontDiscardForSameWReg, kDiscardForSameWReg };

} // namespace jit
} // namespace js

#endif  // VIXL_A64_MACRO_ASSEMBLER_A64_H_
