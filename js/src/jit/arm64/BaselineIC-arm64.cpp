/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/BaselineHelpers.h"
#include "jit/BaselineIC.h"

using namespace js;
using namespace js::jit;

namespace js {
namespace jit {

// ICCompare_Int32

bool
ICCompare_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    MOZ_ASSUME_UNREACHABLE("ICCompare_Int32 generateStubCode");
}

bool
ICCompare_Double::Compiler::generateStubCode(MacroAssembler &masm)
{
    MOZ_ASSUME_UNREACHABLE("ICCompare_Double generateStubCode");
}

// ICBinaryArith_Int32

bool
ICBinaryArith_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    // Guard that R0 is an integer and R1 is an integer.
    Label failure;
    masm.branchTestInt32(Assembler::NotEqual, R0, &failure);
    masm.branchTestInt32(Assembler::NotEqual, R1, &failure);

    // Add R0 and R1. Don't need to explicitly unbox, just use R2.
    Register Rscratch = R2_;
    ARMRegister Xscratch = ARMRegister(R2_, 64);
    ARMRegister Wscratch = ARMRegister(R2_, 64);
    // DIV and MOD need an extra non-volatile ValueOperand to hold R0.
    GeneralRegisterSet savedRegs = availableGeneralRegs(2);
    savedRegs = GeneralRegisterSet::Intersect(GeneralRegisterSet::NonVolatile(), savedRegs);
    ValueOperand savedValue = savedRegs.takeAnyValue();
    // get some more ARM-y names for the registers
    ARMRegister W0(R0_, 32);
    ARMRegister X0(R0_, 64);
    ARMRegister W1(R1_, 32);
    ARMRegister X1(R1_, 64);
    ARMRegister WTemp(ExtractTemp0, 32);
    ARMRegister XTemp(ExtractTemp0, 64);
    Label maybeNegZero, revertRegister;
    switch(op_) {
      case JSOP_ADD:

        masm.Adds(WTemp, W0, Operand(W1));

        // Just jump to failure on overflow. R0 and R1 are preserved, so we can
        // just jump to the next stub.
        masm.j(Assembler::Overflow, &failure);

        // Box the result and return. We know R0. already contains the
        // integer tag, so we just need to move the result value into place.
        masm.monoTagMove(X0, XTemp);
        break;
      case JSOP_SUB:
        masm.Subs(WTemp, W0, Operand(W1));
        masm.j(Assembler::Overflow, &failure);
        masm.monoTagMove(X0, XTemp);
        break;
      case JSOP_MUL: {
          masm.mul32(R0.valueReg(), R1.valueReg(), Rscratch,
                     &failure, &maybeNegZero);
          masm.monoTagMove(X0, Xscratch);

        break;
      }
      case JSOP_DIV:
      case JSOP_MOD: {
#if 0
        // Check for INT_MIN / -1, it results in a double.
        masm.ma_cmp(R0.payloadReg(), Imm32(INT_MIN));
        masm.ma_cmp(R1.payloadReg(), Imm32(-1), Assembler::Equal);
        masm.j(Assembler::Equal, &failure);

        // Check for both division by zero and 0 / X with X < 0 (results in -0).
        masm.ma_cmp(R1.payloadReg(), Imm32(0));
        masm.ma_cmp(R0.payloadReg(), Imm32(0), Assembler::LessThan);
        masm.j(Assembler::Equal, &failure);

        // The call will preserve registers r4-r11. Save R0 and the link
        // register.
        JS_ASSERT(R1 == ValueOperand(r5, r4));
        JS_ASSERT(R0 == ValueOperand(r3, r2));
        masm.moveValue(R0, savedValue);

        masm.setupAlignedABICall(2);
        masm.passABIArg(R0.payloadReg());
        masm.passABIArg(R1.payloadReg());
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, __aeabi_idivmod));

        // idivmod returns the quotient in r0, and the remainder in r1.
        if (op_ == JSOP_DIV) {
            // Result is a double if the remainder != 0.
            masm.branch32(Assembler::NotEqual, r1, Imm32(0), &revertRegister);
            masm.tagValue(JSVAL_TYPE_INT32, r0, R0);
        } else {
            // If X % Y == 0 and X < 0, the result is -0.
            Label done;
            masm.branch32(Assembler::NotEqual, r1, Imm32(0), &done);
            masm.branch32(Assembler::LessThan, savedValue.payloadReg(), Imm32(0), &revertRegister);
            masm.bind(&done);
            masm.tagValue(JSVAL_TYPE_INT32, r1, R0);
        }
#else
        MOZ_ASSERT(0 && "TODO: BinaryArith_Int32::DivMod");
#endif
        break;
      }
        // ORR, EOR, AND can trivially be coerced int
        // working without affecting the tag of the dest..
      case JSOP_BITOR:
        masm.Orr(X0, X0, Operand(X1));
        break;
      case JSOP_BITXOR:
        masm.Eor(X0, X0, Operand(W1, UXTW));
        break;
      case JSOP_BITAND:
        masm.And(X0, X0, Operand(X1));
        break;
        // LSH, RSH and URSH can not.
      case JSOP_LSH:
        // ARM will happily try to shift by more than 0x1f.
        masm.Lsl(Wscratch, W0, W1);
        masm.monoTagMove(R0.valueReg(), Rscratch);
        break;
      case JSOP_RSH:
        masm.Asr(Wscratch, W0, W1);
        masm.monoTagMove(R0.valueReg(), Rscratch);
        break;
      case JSOP_URSH:
        masm.Lsr(Wscratch, W0, W1);
        masm.Cmp(Wscratch, Operand(0));
        if (allowDouble_) {
#if 0
            Label toUint;
            masm.j(Assembler::LessThan, &toUint);
            // Move result and box for return.
            masm.mov(scratchReg, R0.payloadReg());
            EmitReturnFromIC(masm);

            masm.bind(&toUint);
            masm.convertUInt32ToDouble(scratchReg, ScratchDoubleReg);
            masm.boxDouble(ScratchDoubleReg, R0);
#else
            MOZ_CRASH("Unhandled op for BinaryArith_Int32.");
#endif
        } else {
            masm.j(Assembler::LessThan, &failure);
            // Move result for return.
            masm.monoTagMove(X0, Xscratch);
        }
        break;
      default:
        MOZ_CRASH("Unhandled op for BinaryArith_Int32.");
    }

    EmitReturnFromIC(masm);

    switch (op_) {
      case JSOP_MUL:
        masm.bind(&maybeNegZero);

        // Result is -0 if exactly one of lhs or rhs is negative.
        masm.Cmn(W0, W1);
        masm.j(Assembler::Signed, &failure);

        // Result is +0.
        masm.monoTagMove(X0, ARMRegister(r31, 64));
        EmitReturnFromIC(masm);
        break;
      case JSOP_DIV:
      case JSOP_MOD:
        masm.bind(&revertRegister);
        masm.moveValue(savedValue, R0);
        break;
      default:
        break;
    }

    // Failure case - jump to next stub.
    masm.bind(&failure);
    EmitStubGuardFailure(masm);

    return true;
}

bool
ICUnaryArith_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    MOZ_ASSUME_UNREACHABLE("ICUnaryArith_Int32 generateStubCode");
}

} // namespace jit
} // namespace js
