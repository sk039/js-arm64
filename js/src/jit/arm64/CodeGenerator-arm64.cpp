/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/arm64/CodeGenerator-arm64.h"

#include "mozilla/MathAlgorithms.h"

#include "jscntxt.h"
#include "jscompartment.h"
#include "jsnum.h"

#include "jit/CodeGenerator.h"
#include "jit/JitCompartment.h"
#include "jit/JitFrames.h"
#include "jit/MIR.h"
#include "jit/MIRGraph.h"
#include "vm/Shape.h"
#include "vm/TraceLogging.h"

#include "jsscriptinlines.h"

#include "jit/shared/CodeGenerator-shared-inl.h"

using namespace js;
using namespace js::jit;

using mozilla::FloorLog2;
using mozilla::NegativeInfinity;
using JS::GenericNaN;

// shared
CodeGeneratorARM64::CodeGeneratorARM64(MIRGenerator *gen, LIRGraph *graph, MacroAssembler *masm)
  : CodeGeneratorShared(gen, graph, masm)
{
}

bool
CodeGeneratorARM64::generatePrologue()
{
    MOZ_ASSERT(masm.framePushed() == 0);
    MOZ_ASSERT(!gen->compilingAsmJS());

    // FIXME: Uh, doesn't this break frameSize() and the alignment?
#ifdef JS_USE_LINK_REGISTER
    masm.pushReturnAddress();
#endif

    // Note that this automatically sets MacroAssembler::framePushed().
    masm.reserveStack(frameSize());
    masm.checkStackAlignment();
    return true;
}

bool
CodeGeneratorARM64::generateEpilogue()
{
    MOZ_ASSERT(!gen->compilingAsmJS());
    masm.bind(&returnLabel_);

#ifdef JS_TRACE_LOGGING
    //    if (gen->info().executionMode() == SequentialExecution) {
    emitTracelogStopEvent(TraceLogger_IonMonkey);
    emitTracelogScriptStop();
        //}
#endif

    masm.freeStack(frameSize());
    // FIXME: This probably doesn't work with the push(lr) in the prologue...
    MOZ_ASSERT(masm.framePushed() == 0);
    masm.pop(lr);
    masm.MacroAssemblerVIXL::Ret(lr_64);
    masm.flushBuffer();
    return true;
}

bool
CodeGeneratorARM64::generateOutOfLineCode()
{
    if (!CodeGeneratorShared::generateOutOfLineCode())
        return false;

    if (deoptLabel_.used()) {
        // All non-table-based bailouts will go here.
        masm.bind(&deoptLabel_);

        // Push the frame size, so the handler can recover the IonScript.
        masm.Mov(x30, frameSize());

        JitCode *handler = gen->jitRuntime()->getGenericBailoutHandler();
        masm.branch(handler);
    }

    return true;
}

void
CodeGeneratorARM64::emitBranch(Assembler::Condition cond, MBasicBlock *mirTrue, MBasicBlock *mirFalse)
{
    if (isNextBlock(mirFalse->lir())) {
        jumpToBlock(mirTrue, cond);
    } else {
        jumpToBlock(mirFalse, Assembler::InvertCondition(cond));
        jumpToBlock(mirTrue);
    }
}

void
OutOfLineBailout::accept(CodeGeneratorARM64 *codegen)
{
    codegen->visitOutOfLineBailout(this);
}

void
CodeGeneratorARM64::visitTestIAndBranch(LTestIAndBranch *test)
{
    const LAllocation *opd = test->getOperand(0);
    MBasicBlock *ifTrue = test->ifTrue();
    MBasicBlock *ifFalse = test->ifFalse();

    // Test the operand
    masm.cmp32(ToRegister(opd), Imm32(0));

    if (isNextBlock(ifFalse->lir())) {
        jumpToBlock(ifTrue, Assembler::NonZero);
    } else if (isNextBlock(ifTrue->lir())) {
        jumpToBlock(ifFalse, Assembler::Zero);
    } else {
        jumpToBlock(ifFalse, Assembler::Zero);
        jumpToBlock(ifTrue);
    }
}

void 
CodeGeneratorARM64::visitCompare(LCompare *comp)
{
    Assembler::Condition cond = JSOpToCondition(comp->mir()->compareType(), comp->jsop());
    const LAllocation *left = comp->getOperand(0);
    const LAllocation *right = comp->getOperand(1);
    const LDefinition *def = comp->getDef(0);

    if (right->isConstant())
        masm.cmp32(ToRegister(left), Imm32(ToInt32(right)));
    else
        masm.cmp32(ToRegister(left), ToRegister(right));
    masm.emitSet(cond, ToRegister(def));

}

void
CodeGeneratorARM64::visitCompareAndBranch(LCompareAndBranch *comp)
{
    Assembler::Condition cond = JSOpToCondition(comp->cmpMir()->compareType(), comp->jsop());
    if (comp->right()->isConstant())
        masm.cmp32(ToRegister(comp->left()), Imm32(ToInt32(comp->right())));
    else
        masm.cmp32(ToRegister(comp->left()), ToRegister(comp->right()));
    emitBranch(cond, comp->ifTrue(), comp->ifFalse());
}

void
CodeGeneratorARM64::bailoutIf(Assembler::Condition condition, LSnapshot *snapshot)
{
    encode(snapshot);

    // Though the assembler doesn't track all frame pushes, at least make sure
    // the known value makes sense. We can't use bailout tables if the stack
    // isn't properly aligned to the static frame size.
    MOZ_ASSERT_IF(frameClass_ != FrameSizeClass::None(),
                  frameClass_.frameSize() == masm.framePushed());

#if 0 // I am pretty sure this is not worthwhile here.
    if (assignBailoutId(snapshot)) {
        uint8_t *code = Assembler::BailoutTableStart(deoptTable_->raw()) + snapshot->bailoutId() * BAILOUT_TABLE_ENTRY_SIZE;
        masm.b(code, Relocation::HARDCODED, condition);
        return;
    }
#endif
    // We could not use a jump table, either because all bailout IDs were
    // reserved, or a jump table is not optimal for this frame size or
    // platform. Whatever, we will generate a lazy bailout.
    InlineScriptTree *tree = snapshot->mir()->block()->trackedTree();
    OutOfLineBailout *ool = new(alloc()) OutOfLineBailout(snapshot);

    // All bailout code is associated with the bytecodeSite of the block we are
    // bailing out from.
    addOutOfLineCode(ool, new(alloc()) BytecodeSite(tree, tree->script()->code()));

    masm.B(ool->entry(), condition);

}

void 
CodeGeneratorARM64::bailoutFrom(Label *label, LSnapshot *snapshot)
{
    MOZ_ASSERT(label->used());
    MOZ_ASSERT(!label->bound());

    encode(snapshot);

    // Though the assembler doesn't track all frame pushes, at least make sure
    // the known value makes sense. We can't use bailout tables if the stack
    // isn't properly aligned to the static frame size.
    MOZ_ASSERT_IF(frameClass_ != FrameSizeClass::None(),
                  frameClass_.frameSize() == masm.framePushed());

    // On ARM64 we don't use a bailout table.
    InlineScriptTree *tree = snapshot->mir()->block()->trackedTree();
    OutOfLineBailout *ool = new(alloc()) OutOfLineBailout(snapshot);

    // All bailout code is associated with the bytecodeSite of the block we are
    // bailing out from.
    addOutOfLineCode(ool, new(alloc()) BytecodeSite(tree, tree->script()->code()));

    masm.retarget(label, ool->entry());

}

void 
CodeGeneratorARM64::bailout(LSnapshot *snapshot)
{
    Label label;
    masm.b(&label);
    bailoutFrom(&label, snapshot);
}

void
CodeGeneratorARM64::visitOutOfLineBailout(OutOfLineBailout *ool)
{
    masm.Mov(ScratchReg2_32, Operand(ool->snapshot()->snapshotOffset()));
    masm.MacroAssemblerVIXL::Push(ScratchReg64, ScratchReg2_64); // BailoutStack::snapshotOffset_
    masm.b(&deoptLabel_);
}

void
CodeGeneratorARM64::visitMinMaxD(LMinMaxD *ins)
{
    ARMFPRegister lhs(ToFloatRegister(ins->first()), 64);
    ARMFPRegister rhs(ToFloatRegister(ins->second()), 64);
    ARMFPRegister output(ToFloatRegister(ins->output()), 64);
    if (ins->mir()->isMax())
        masm.Fmax(output, lhs, rhs);
    else
        masm.Fmin(output, lhs, rhs);
}

void
CodeGeneratorARM64::visitMinMaxF(LMinMaxF *ins)
{
    ARMFPRegister lhs(ToFloatRegister(ins->first()), 32);
    ARMFPRegister rhs(ToFloatRegister(ins->second()), 32);
    ARMFPRegister output(ToFloatRegister(ins->output()), 32);
    if (ins->mir()->isMax())
        masm.Fmax(output, lhs, rhs);
    else
        masm.Fmin(output, lhs, rhs);
}

void 
CodeGeneratorARM64::visitAbsD(LAbsD *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 64);
    masm.Fabs(input, input);
}

void 
CodeGeneratorARM64::visitAbsF(LAbsF *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 32);
    masm.Fabs(input, input);
}

void 
CodeGeneratorARM64::visitSqrtD(LSqrtD *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 64);
    ARMFPRegister output(ToFloatRegister(ins->output()), 64);
    masm.Fsqrt(output, input);
}

void 
CodeGeneratorARM64::visitSqrtF(LSqrtF *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 32);
    ARMFPRegister output(ToFloatRegister(ins->output()), 32);
    masm.Fsqrt(output, input);
}

template<typename T>
ARMRegister toWRegister(const T *a) {
    return ARMRegister(ToRegister(a), 32);
}
template<typename T>
ARMRegister toXRegister(const T *a) {
    return ARMRegister(ToRegister(a), 64);
}

js::jit::Operand toWOperand(const LAllocation *a)
{
    if (a->isConstant())
        return js::jit::Operand(ToInt32(a));
    return js::jit::Operand(toWRegister(a));
}

CPURegister
ToCPURegister(const LAllocation *a, Scalar::Type type)
{
    if (a->isFloatReg() && type == Scalar::Float64)
        return ARMFPRegister(ToFloatRegister(a), 64);
    if (a->isFloatReg() && type == Scalar::Float32)
        return ARMFPRegister(ToFloatRegister(a), 32);
    if (a->isGeneralReg())
        return ARMRegister(ToRegister(a), 32);
    MOZ_CRASH("Unknown LAllocation");
}
CPURegister
ToCPURegister(const LDefinition *d, Scalar::Type type)
{
    return ToCPURegister(d->output(), type);
}

void
CodeGeneratorARM64::visitAddI(LAddI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);
    const LDefinition *dest = ins->getDef(0);
    if (ins->snapshot()) {
        masm.Adds(toWRegister(dest), toWRegister(lhs), toWOperand(rhs));
        bailoutIf(Assembler::Overflow, ins->snapshot());
    } else {
        masm.Add(toWRegister(dest), toWRegister(lhs), toWOperand(rhs));
    }
}

void 
CodeGeneratorARM64::visitSubI(LSubI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);
    const LDefinition *dest = ins->getDef(0);
    if (ins->snapshot()) {
        masm.Subs(toWRegister(dest), toWRegister(lhs), toWOperand(rhs));
        bailoutIf(Assembler::Overflow, ins->snapshot());
    } else {
        masm.Sub(toWRegister(dest), toWRegister(lhs), toWOperand(rhs));
    }
}

void 
CodeGeneratorARM64::visitMulI(LMulI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);
    const LDefinition *dest = ins->getDef(0);
    Register rhs_reg;
    MMul *mul = ins->mir();
    if (rhs->isConstant()) {
        int32_t constant = ToInt32(rhs);
        if (mul->canBeNegativeZero() && constant <= 0) {
            Assembler::Condition bailoutCond = (constant == 0) ? Assembler::LessThan : Assembler::Equal;
            // N.B. A cbz/cbnz can be used here , if we're ok ith an OOL bailout. I think this is fine.
            masm.Cmp(toWRegister(lhs), Operand(0));
            bailoutIf(bailoutCond, ins->snapshot());
        }
        masm.move32(Imm32(constant), ScratchReg);
        rhs_reg = ScratchReg;

    } else {
        rhs_reg = ToRegister(rhs);
    }

    Label *onZero = nullptr;
    Label l;
    if (mul->canBeNegativeZero()) {
        onZero = &l;
    }
    if (mul->canOverflow()) {
        // I should /really/ be able to just bail-out directly from the macro assembler.
        // this song-and-dance with condition codes seems unweildy in this case.
        Label onOver;
        masm.mul32(ToRegister(lhs), rhs_reg, ToRegister(dest), &onOver, onZero);
        bailoutFrom(&onOver, ins->snapshot());
    } else {
        masm.mul32(ToRegister(lhs), rhs_reg, ToRegister(dest), nullptr, onZero);
    }
    if (mul->canBeNegativeZero()) {
        bailoutFrom(onZero, ins->snapshot());
    }
}


void
CodeGeneratorARM64::visitDivI(LDivI *ins)
{
    // Extract the registers from this instruction.
    ARMRegister lhs = toWRegister(ins->lhs());
    ARMRegister rhs = toWRegister(ins->rhs());
    ARMRegister temp = toWRegister(ins->getTemp(0));
    ARMRegister output = toWRegister(ins->output());
    MDiv *mir = ins->mir();
    Label done;
    if (mir->canBeNegativeOverflow()) {
        Label skip;
        // Handle INT32_MIN / -1;
        // The integer division will give INT32_MIN, but we want -(double)INT32_MIN.

        // Sets EQ if lhs == INT32_MIN.
        masm.Cmp(lhs, Operand(INT32_MIN));
        masm.B(&skip, Assembler::NotEqual);
        // If EQ (LHS == INT32_MIN), sets EQ if rhs == -1.
        masm.Cmp(rhs, Operand(-1));
        if (mir->canTruncateOverflow()) {
            // (-INT32_MIN)|0 = INT32_MIN
            masm.B(&skip, Assembler::NotEqual);
            masm.Mov(output, Operand(INT32_MIN));
            masm.B(&done);
        } else {
            MOZ_ASSERT(mir->fallible());
            bailoutIf(Assembler::Equal, ins->snapshot());
        }
        masm.bind(&skip);
    }

    // Handle divide by zero.
    if (mir->canBeDivideByZero()) {
        masm.Cmp(rhs, Operand(0));
        if (mir->canTruncateInfinities()) {
            // Infinity|0 == 0
            Label skip;
            masm.B(&skip, Assembler::NotEqual);
            masm.Mov(output, Operand(0));
            masm.B(&done);
            masm.bind(&skip);
        } else {
            MOZ_ASSERT(mir->fallible());
            bailoutIf(Assembler::Equal, ins->snapshot());
        }
    }

    // Handle negative 0.
    if (!mir->canTruncateNegativeZero() && mir->canBeNegativeZero()) {
        Label nonzero;
        masm.Tbz(rhs, 31, &nonzero);
        masm.Cmp(lhs, Operand(0));
        MOZ_ASSERT(mir->fallible());
        bailoutIf(Assembler::Equal, ins->snapshot());
        masm.bind(&nonzero);
    }
    if (mir->canTruncateRemainder()) {
        masm.Sdiv(output, lhs, rhs);
    } else {
        masm.Sdiv(ScratchReg2_32, lhs, rhs);
        masm.Mul(temp, ScratchReg2_32, rhs);
        masm.Cmp(lhs, temp);
        bailoutIf(Assembler::NotEqual, ins->snapshot());
        masm.Mov(output, ScratchReg2_32);
    }
    masm.bind(&done);

}

void 
CodeGeneratorARM64::visitDivPowTwoI(LDivPowTwoI *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitDivPowTwoI");
}

void 
CodeGeneratorARM64::modICommon(MMod *mir, Register lhs, Register rhs, Register output,
                               LSnapshot *snapshot, Label &done)
{
    MOZ_CRASH("CodeGeneratorARM64::modICommon");
}

void
CodeGeneratorARM64::visitModI(LModI *ins)
{
    MMod *mir = ins->mir();
    ARMRegister lhs = toWRegister(ins->lhs());
    ARMRegister rhs = toWRegister(ins->rhs());
    ARMRegister output = toWRegister(ins->output());
    Label done;
    if (mir->canBeDivideByZero() && !mir->isTruncated()) {
        masm.Cmp(rhs, Operand(0));
        bailoutIf(Assembler::Equal, ins->snapshot());
    } else if (mir->canBeDivideByZero()) {
        // if it is truncated, then modulus should give zero,
        masm.Mov(output, rhs);
        masm.Cbz(rhs, &done);
    }
    masm.Sdiv(output, lhs, rhs);
    // compute the remainder, out = lhs - rhs * out.
    masm.Msub(output, output, rhs, lhs);
    if (mir->canBeNegativeDividend()) {
        if (!mir->isTruncated()) {
            masm.Cbnz(output, &done);
            masm.Cmp(lhs, Operand(0));
            bailoutIf(Assembler::Signed, ins->snapshot());
        }
    }
    masm.bind(&done);
}

void
CodeGeneratorARM64::visitModPowTwoI(LModPowTwoI *ins)
{
    MMod *mir = ins->mir();
    ARMRegister in = toWRegister(ins->getOperand(0));
    ARMRegister out = toWRegister(ins->output());
    Label fin;
    masm.Subs(out, in, Operand(0));
    masm.B(&fin, Assembler::Zero);
    masm.Cneg(out, out, Assembler::Signed);
    masm.And(out, out, Operand((1 << ins->shift()) - 1));
    masm.Cneg(out, out, Assembler::Signed);
    if (mir->canBeNegativeDividend()) {
        if (!mir->isTruncated()) {
            MOZ_ASSERT(mir->fallible());
            bailoutIf(Assembler::Zero, ins->snapshot());
        } else {
            // -0|0 == 0
        }
    }
    masm.bind(&fin);
}

void 
CodeGeneratorARM64::visitModMaskI(LModMaskI *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitModMaskI");
}

void 
CodeGeneratorARM64::visitBitNotI(LBitNotI *ins)
{
    const LAllocation *input = ins->getOperand(0);
    const LDefinition *output = ins->getDef(0);
    masm.Mvn(toWRegister(output), toWOperand(input));
}

void 
CodeGeneratorARM64::visitBitOpI(LBitOpI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);
    const LDefinition *dest = ins->getDef(0);
    const ARMRegister lhsreg = toWRegister(lhs);
    const js::jit::Operand rhsop = toWOperand(rhs);
    const ARMRegister destreg = toWRegister(dest);
    switch (ins->bitop()) {
      case JSOP_BITOR:
        masm.Orr(destreg, lhsreg, rhsop);
        break;
      case JSOP_BITXOR:
        masm.Eor(destreg, lhsreg, rhsop);
        break;
      case JSOP_BITAND:
        masm.And(destreg, lhsreg, rhsop);
        break;
      default:
        MOZ_CRASH("unexpected binary opcode");
    }
}

void 
CodeGeneratorARM64::visitShiftI(LShiftI *ins)
{
    ARMRegister lhs = toWRegister(ins->lhs());
    ARMRegister dest = toWRegister(ins->output());
    if (ins->rhs()->isConstant()) {
        int32_t rhs = ToInt32(ins->rhs()) & 0x1F;
        switch (ins->bitop()) {
          case JSOP_LSH:
            masm.Lsl(dest, lhs, rhs);
            break;
          case JSOP_RSH:
            masm.Asr(dest, lhs, rhs);
            break;
          case JSOP_URSH:
            if (rhs == 0) {
                //masm.Cmp(lhs, Operand(0));
                //masm.Mov(dest, lhs);
                //                masm.breakpoint();
                if (ins->mir()->toUrsh()->fallible()) {
                    masm.Ands(dest, lhs, Operand(0xffffffff));
                    bailoutIf(Assembler::Signed, ins->snapshot());
                } else {
                    masm.Mov(dest, lhs);
                }
            } else {
                masm.Lsr(dest, lhs, rhs);
            }
            // TODO: bail if the result is negative
            break;
          default:
            MOZ_CRASH("Unexpected shift opcode");
        }
    } else {
        ARMRegister rhs = toWRegister(ins->rhs());
        Label good;
        switch (ins->bitop()) {
          case JSOP_LSH:
            masm.Lsl(dest, lhs, rhs);
            break;
          case JSOP_RSH:
            masm.Asr(dest, lhs, rhs);
            break;
          case JSOP_URSH:
            masm.Lsr(dest, lhs, rhs);
            if (ins->mir()->toUrsh()->fallible()) {
                masm.Cmp(dest, Operand(0));
                masm.Cbnz(rhs, &good);
                bailoutIf(Assembler::LessThan, ins->snapshot());
                masm.bind(&good);
            }
            break;
          default:
            MOZ_CRASH("Unexpected shift opcode");
        }
    }
}

void
CodeGeneratorARM64::visitUrshD(LUrshD *ins)
{
    ARMRegister lhs = toWRegister(ins->lhs());
    ARMRegister temp = toWRegister(ins->temp());

    const LAllocation *rhs = ins->rhs();
    FloatRegister out = ToFloatRegister(ins->output());

    if (rhs->isConstant()) {
        int32_t shift = ToInt32(rhs) & 0x1F;
        if (shift)
            masm.Lsr(temp, lhs, shift);
        else
            masm.Mov(temp, lhs);
    } else {
        masm.And(temp,  toWRegister(rhs), Operand(0x1F));
        masm.Lsr(temp, lhs, temp);
    }

    masm.convertUInt32ToDouble(ToRegister(ins->temp()), out);

}

void
CodeGeneratorARM64::visitPowHalfD(LPowHalfD *ins)
{
    ARMFPRegister input = ARMFPRegister(ToFloatRegister(ins->input()), 64);
    ARMFPRegister output = ARMFPRegister(ToFloatRegister(ins->output()), 64);

    Label done;

    // Masm.pow(-Infinity, 0.5) == Infinity.
    masm.Fmov(ScratchDoubleReg, NegativeInfinity<double>());
    masm.Fcmp(input, ScratchDoubleReg);
    masm.Fneg(output, ScratchDoubleReg);
    masm.B(&done, Assembler::Equal);

    // Math.pow(-0, 0.5) == 0 == Math.pow(0, 0.5).
    // Adding 0 converts any -0 to 0.
    masm.Fmov(ScratchDoubleReg, 0.0);
    masm.Fadd(output, ScratchDoubleReg, input);
    masm.Fsqrt(output, output);

    masm.bind(&done);

}

MoveOperand
CodeGeneratorARM64::toMoveOperand(const LAllocation *a) const
{
    if (a->isGeneralReg())
        return MoveOperand(ToRegister(a));
    if (a->isFloatReg())
        return MoveOperand(ToFloatRegister(a));
    int32_t offset = ToStackOffset(a);
    return MoveOperand(masm.GetStackPointer_(), offset);
}

class js::jit::OutOfLineTableSwitch : public OutOfLineCodeBase<CodeGeneratorARM64>
{
    MTableSwitch *mir_;
    Vector<CodeLabel, 8, JitAllocPolicy> codeLabels_;

    void accept(CodeGeneratorARM64 *codegen) {
        codegen->visitOutOfLineTableSwitch(this);
    }

  public:
    OutOfLineTableSwitch(TempAllocator &alloc, MTableSwitch *mir)
      : mir_(mir),
        codeLabels_(alloc)
    {}

    MTableSwitch *mir() const {
        return mir_;
    }

    bool addCodeLabel(CodeLabel label) {
        return codeLabels_.append(label);
    }
    CodeLabel codeLabel(unsigned i) {
        return codeLabels_[i];
    }
};

void
CodeGeneratorARM64::visitOutOfLineTableSwitch(OutOfLineTableSwitch *ool)
{
    MTableSwitch *mir = ool->mir();

    size_t numCases = mir->numCases();
    for (size_t i = 0; i < numCases; i++) {
        LBlock *caseblock = skipTrivialBlocks(mir->getCase(i))->lir();
        Label *caseheader = caseblock->label();
        uint32_t caseoffset = caseheader->offset();

        // The entries of the jump table need to be absolute addresses and thus
        // must be patched after codegen is finished.
        CodeLabel cl = ool->codeLabel(i);
        cl.src()->bind(caseoffset);
        masm.addCodeLabel(cl);
    }
}

void
CodeGeneratorARM64::emitTableSwitchDispatch(MTableSwitch *mir, Register index_, Register base_)
{
    ARMRegister index(index_, 64);
    ARMRegister index32(index_, 32);
    Label *defaultcase = skipTrivialBlocks(mir->getDefault())->lir()->label();

    int32_t cases = mir->numCases();
    // Lower value with low value.
    masm.Sub(index32, index32, Operand(mir->low()));
    masm.Cmp(index32, Operand(cases));
    //masm.breakpoint();
    masm.B(defaultcase, Assembler::AboveOrEqual);

    // Inhibit pools within the following sequence because we are indexing into
    // a pc relative table. The region will have one instruction for ma_ldr, one
    // for ma_b, and each table case takes one word.

    AutoForbidPools afp(&masm, 1 + 1 + 1 + cases*2);
    Label table;
    masm.Adr(ScratchReg2_64, &table);
    masm.Ldr(ScratchReg2_64, MemOperand(ScratchReg2_64, index, LSL, 3));
    masm.Blr(ScratchReg2_64);
    // To fill in the CodeLabels for the case entries, we need to first generate
    // the case entries (we don't yet know their offsets in the instruction
    // stream).
    OutOfLineTableSwitch *ool = new(alloc()) OutOfLineTableSwitch(alloc(), mir);
    masm.bind(&table);
    for (int32_t i = 0; i < cases; i++) {
        CodeLabel cl;
        masm.writeCodePointer(cl.dest());
        ool->addCodeLabel(cl);
    }
    addOutOfLineCode(ool, mir);
}

void 
CodeGeneratorARM64::visitMathD(LMathD *math)
{
    const ARMFPRegister src1(ToFloatRegister(math->getOperand(0)), 64);
    const ARMFPRegister src2(ToFloatRegister(math->getOperand(1)), 64);
    const ARMFPRegister output(ToFloatRegister(math->getDef(0)), 64);

    switch (math->jsop()) {
      case JSOP_ADD:
        masm.Fadd(output, src1, src2);
        break;
      case JSOP_SUB:
        masm.Fsub(output, src1, src2);
        break;
      case JSOP_MUL:
        masm.Fmul(output, src1, src2);
        break;
      case JSOP_DIV:
        masm.Fdiv(output, src1, src2);
        break;
      default:
        MOZ_CRASH("unexpected opcode");
    }

}

void
CodeGeneratorARM64::visitMathF(LMathF *math)
{
    const ARMFPRegister src1(ToFloatRegister(math->getOperand(0)), 32);
    const ARMFPRegister src2(ToFloatRegister(math->getOperand(1)), 32);
    const ARMFPRegister output(ToFloatRegister(math->getDef(0)), 32);

    switch (math->jsop()) {
      case JSOP_ADD:
        masm.Fadd(output, src1, src2);
        break;
      case JSOP_SUB:
        masm.Fsub(output, src1, src2);
        break;
      case JSOP_MUL:
        masm.Fmul(output, src1, src2);
        break;
      case JSOP_DIV:
        masm.Fdiv(output, src1, src2);
        break;
      default:
        MOZ_CRASH("unexpected opcode");
    }

}

void
CodeGeneratorARM64::visitFloor(LFloor *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    Register output = ToRegister(lir->output());
    Label bail;
    masm.floor(input, output, &bail);
    bailoutFrom(&bail, lir->snapshot());

}

void
CodeGeneratorARM64::visitFloorF(LFloorF *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    Register output = ToRegister(lir->output());
    Label bail;
    masm.floorf(input, output, &bail);
    bailoutFrom(&bail, lir->snapshot());
}

void 
CodeGeneratorARM64::visitCeil(LCeil *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCeil");
}

void 
CodeGeneratorARM64::visitCeilF(LCeilF *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCeilF");
}

void 
CodeGeneratorARM64::visitRound(LRound *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    Register output = ToRegister(lir->output());
    FloatRegister tmp = ToFloatRegister(lir->temp());
    Label bail;
    // Output is either correct, or clamped. All -0 cases have been translated
    // to a clamped case.
    // TODO: supposedly this is not the same as the round operation that we need?
    masm.convertDoubleToInt32(input, output, &bail, true);
    bailoutFrom(&bail, lir->snapshot());
}

void
CodeGeneratorARM64::visitRoundF(LRoundF *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    Register output = ToRegister(lir->output());
    FloatRegister tmp = ToFloatRegister(lir->temp());
    Label bail;
    // Output is either correct, or clamped. All -0 cases have been translated
    // to a clamped case.
    // TODO: supposedly this is not the same as the round operation that we need?
    masm.convertFloat32ToInt32(input, output, &bail, true);
    bailoutFrom(&bail, lir->snapshot());

}

void
CodeGeneratorARM64::visitClzI(LClzI *lir)
{
    ARMRegister input = toWRegister(lir->input());
    ARMRegister output = toWRegister(lir->output());
    masm.Clz(output, input);
}


void
CodeGeneratorARM64::emitRoundDouble(FloatRegister src, Register dest, Label *fail)
{
    MOZ_CRASH("CodeGeneratorARM64::emitRoundDouble");
}

void
CodeGeneratorARM64::visitTruncateDToInt32(LTruncateDToInt32 *ins)
{
    emitTruncateDouble(ToFloatRegister(ins->input()), ToRegister(ins->output()), ins->mir());
}

void
CodeGeneratorARM64::visitTruncateFToInt32(LTruncateFToInt32 *ins)
{
    emitTruncateFloat32(ToFloatRegister(ins->input()), ToRegister(ins->output()), ins->mir());
}

static const uint32_t FrameSizes[] = { 128, 256, 512, 1024 };

FrameSizeClass
FrameSizeClass::FromDepth(uint32_t frameDepth)
{
    return FrameSizeClass::None();
}

FrameSizeClass
FrameSizeClass::ClassLimit()
{
    return FrameSizeClass(0);
}

uint32_t
FrameSizeClass::frameSize() const
{
    MOZ_CRASH("arm64 does not use frame size classes");
}

ValueOperand
CodeGeneratorARM64::ToValue(LInstruction *ins, size_t pos)
{
    return ValueOperand(ToRegister(ins->getOperand(pos)));
}

ValueOperand
CodeGeneratorARM64::ToOutValue(LInstruction *ins)
{
    Register payloadReg = ToRegister(ins->getDef(0));
    return ValueOperand(payloadReg);
}

ValueOperand
CodeGeneratorARM64::ToTempValue(LInstruction *ins, size_t pos)
{
    MOZ_CRASH("CodeGeneratorARM64::ToTempValue");
}

void
CodeGeneratorARM64::visitValue(LValue *value)
{
    const ValueOperand out = ToOutValue(value);
    masm.moveValue(value->value(), out);
}

void
CodeGeneratorARM64::visitBox(LBox *box)
{
    const LAllocation *in = box->getOperand(0);
    const LDefinition *result = box->getDef(0);

    if (IsFloatingPointType(box->type())) {
        FloatRegister reg = ToFloatRegister(in);
        if (box->type() == MIRType_Float32) {
            masm.convertFloat32ToDouble(reg, ScratchDoubleReg);
            reg = ScratchDoubleReg;
        }
        masm.Fmov(ARMRegister(ToRegister(result), 64), ARMFPRegister(reg, 64));
    } else {
        masm.boxValue(ValueTypeFromMIRType(box->type()), ToRegister(in), ToRegister(result));
    }

}

void
CodeGeneratorARM64::visitUnbox(LUnbox *unbox)
{
    MUnbox *mir = unbox->mir();

    if (mir->fallible()) {
        const ValueOperand value = ToValue(unbox, LUnbox::Input);
        Assembler::Condition cond;
        switch (mir->type()) {
          case MIRType_Int32:
            cond = masm.testInt32(Assembler::NotEqual, value);
            break;
          case MIRType_Boolean:
            cond = masm.testBoolean(Assembler::NotEqual, value);
            break;
          case MIRType_Object:
            cond = masm.testObject(Assembler::NotEqual, value);
            break;
          case MIRType_String:
            cond = masm.testString(Assembler::NotEqual, value);
            break;
          case MIRType_Symbol:
            cond = masm.testSymbol(Assembler::NotEqual, value);
            break;
          default:
            MOZ_CRASH("Given MIRType cannot be unboxed.");
        }
        bailoutIf(cond, unbox->snapshot());
    }

    ValueOperand input = ToValue(unbox, LUnbox::Input);
    Register result = ToRegister(unbox->output());
    switch (mir->type()) {
      case MIRType_Int32:
        masm.unboxInt32(input, result);
        break;
      case MIRType_Boolean:
        masm.unboxBoolean(input, result);
        break;
      case MIRType_Object:
        masm.unboxObject(input, result);
        break;
      case MIRType_String:
        masm.unboxString(input, result);
        break;
      case MIRType_Symbol:
        masm.unboxSymbol(input, result);
        break;
      default:
        MOZ_CRASH("Given MIRType cannot be unboxed.");
    }
}

void 
CodeGeneratorARM64::visitDouble(LDouble *ins)
{
    const LDefinition *out = ins->getDef(0);
    masm.Fmov(ARMFPRegister(ToFloatRegister(out), 64), ins->getDouble());
}

void 
CodeGeneratorARM64::visitFloat32(LFloat32 *ins)
{
    const LDefinition *out = ins->getDef(0);
    masm.Fmov(ARMFPRegister(ToFloatRegister(out), 32), ins->getFloat());
}

Register
CodeGeneratorARM64::splitTagForTest(const ValueOperand &value)
{
    MOZ_CRASH("CodeGeneratorARM64::splitTagForTest");
    return InvalidReg;
}

void
CodeGeneratorARM64::visitTestDAndBranch(LTestDAndBranch *test)
{
    const LAllocation *opd = test->input();
    masm.Fcmp(ARMFPRegister(ToFloatRegister(opd), 64), 0.0);

    MBasicBlock *ifTrue = test->ifTrue();
    MBasicBlock *ifFalse = test->ifFalse();
    // If the compare set the 0 bit, then the result is definately false.
    jumpToBlock(ifFalse, Assembler::Zero);
    // It is also false if one of the operands is NAN, which is shown as
    // Overflow.
    jumpToBlock(ifFalse, Assembler::Overflow);
    jumpToBlock(ifTrue);
}

void
CodeGeneratorARM64::visitTestFAndBranch(LTestFAndBranch *test)
{
    const LAllocation *opd = test->input();
    masm.Fcmp(ARMFPRegister(ToFloatRegister(opd), 32), 0.0);

    MBasicBlock *ifTrue = test->ifTrue();
    MBasicBlock *ifFalse = test->ifFalse();
    // If the compare set the 0 bit, then the result is definately false.
    jumpToBlock(ifFalse, Assembler::Zero);
    // It is also false if one of the operands is NAN, which is shown as
    // Overflow.
    jumpToBlock(ifFalse, Assembler::Overflow);
    jumpToBlock(ifTrue);

}

void
CodeGeneratorARM64::visitCompareD(LCompareD *comp)
{
    ARMFPRegister lhs(ToFloatRegister(comp->left()), 64);
    ARMFPRegister rhs(ToFloatRegister(comp->right()), 64);
    ARMRegister output = toWRegister(comp->output());
    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->mir()->jsop());
    masm.Fcmp(lhs, rhs);
    masm.cset(output, Assembler::ConditionFromDoubleCondition(cond));
}

void
CodeGeneratorARM64::visitCompareF(LCompareF *comp)
{
    ARMFPRegister lhs(ToFloatRegister(comp->left()), 32);
    ARMFPRegister rhs(ToFloatRegister(comp->right()), 32);
    ARMRegister output = toWRegister(comp->output());
    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->mir()->jsop());
    masm.Fcmp(lhs, rhs);
    masm.cset(output, Assembler::ConditionFromDoubleCondition(cond));
}

void
CodeGeneratorARM64::visitCompareDAndBranch(LCompareDAndBranch *comp)
{
    ARMFPRegister lhs(ToFloatRegister(comp->left()), 64);
    ARMFPRegister rhs(ToFloatRegister(comp->right()), 64);

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->cmpMir()->jsop());
    masm.Fcmp(lhs, rhs);
    emitBranch(Assembler::ConditionFromDoubleCondition(cond), comp->ifTrue(), comp->ifFalse());
}

void
CodeGeneratorARM64::visitCompareFAndBranch(LCompareFAndBranch *comp)
{
    FloatRegister lhs = ToFloatRegister(comp->left());
    FloatRegister rhs = ToFloatRegister(comp->right());

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->cmpMir()->jsop());
    masm.compareFloat(cond, lhs, rhs);
    emitBranch(Assembler::ConditionFromDoubleCondition(cond), comp->ifTrue(), comp->ifFalse());
}

void
CodeGeneratorARM64::visitCompareB(LCompareB *lir)
{
    MCompare *mir = lir->mir();

    const ValueOperand lhs = ToValue(lir, LCompareB::Lhs);
    const LAllocation *rhs = lir->rhs();
    const Register output = ToRegister(lir->output());

    MOZ_ASSERT(mir->jsop() == JSOP_STRICTEQ || mir->jsop() == JSOP_STRICTNE);

    // Load boxed boolean in ScratchReg.
    if (rhs->isConstant())
        masm.moveValue(*rhs->toConstant(), ScratchReg2);
    else
        masm.boxValue(JSVAL_TYPE_BOOLEAN, ToRegister(rhs), ScratchReg2);

    // Perform the comparison.
    masm.cmpPtr(lhs.valueReg(), ScratchReg2);
    masm.emitSet(JSOpToCondition(mir->compareType(), mir->jsop()), output);
}

void
CodeGeneratorARM64::visitCompareBAndBranch(LCompareBAndBranch *lir)
{
    MCompare *mir = lir->cmpMir();

    const ValueOperand lhs = ToValue(lir, LCompareBAndBranch::Lhs);
    const LAllocation *rhs = lir->rhs();

    MOZ_ASSERT(mir->jsop() == JSOP_STRICTEQ || mir->jsop() == JSOP_STRICTNE);

    // Load boxed boolean in ScratchReg.
    if (rhs->isConstant())
        masm.moveValue(*rhs->toConstant(), ScratchReg);
    else
        masm.boxValue(JSVAL_TYPE_BOOLEAN, ToRegister(rhs), ScratchReg);

    // Perform the comparison.
    masm.cmpPtr(lhs.valueReg(), ScratchReg);
    emitBranch(JSOpToCondition(mir->compareType(), mir->jsop()), lir->ifTrue(), lir->ifFalse());

}

void
CodeGeneratorARM64::visitCompareV(LCompareV *lir)
{
    MCompare *mir = lir->mir();
    const ValueOperand lhs = ToValue(lir, LCompareV::LhsInput);
    const ValueOperand rhs = ToValue(lir, LCompareV::RhsInput);
    const Register output = ToRegister(lir->output());

    MOZ_ASSERT(IsEqualityOp(mir->jsop()));

    masm.cmpPtr(lhs.valueReg(), rhs.valueReg());
    masm.emitSet(JSOpToCondition(mir->compareType(), mir->jsop()), output);
}

void
CodeGeneratorARM64::visitCompareVAndBranch(LCompareVAndBranch *lir)
{
    MCompare *mir = lir->cmpMir();

    const ValueOperand lhs = ToValue(lir, LCompareVAndBranch::LhsInput);
    const ValueOperand rhs = ToValue(lir, LCompareVAndBranch::RhsInput);

    MOZ_ASSERT(mir->jsop() == JSOP_EQ || mir->jsop() == JSOP_STRICTEQ ||
               mir->jsop() == JSOP_NE || mir->jsop() == JSOP_STRICTNE);

    masm.cmpPtr(lhs.valueReg(), rhs.valueReg());
    emitBranch(JSOpToCondition(mir->compareType(), mir->jsop()), lir->ifTrue(), lir->ifFalse());
}

void
CodeGeneratorARM64::visitBitAndAndBranch(LBitAndAndBranch *baab)
{
    if (baab->right()->isConstant())
        masm.Tst(toWRegister(baab->left()), Operand(ToInt32(baab->right())));
    else
        masm.Tst(toWRegister(baab->left()), toWRegister(baab->right()));
    emitBranch(Assembler::NonZero, baab->ifTrue(), baab->ifFalse());
}

void
CodeGeneratorARM64::visitAsmJSUInt32ToDouble(LAsmJSUInt32ToDouble *lir)
{
    masm.convertUInt32ToDouble(ToRegister(lir->input()), ToFloatRegister(lir->output()));
}

void
CodeGeneratorARM64::visitAsmJSUInt32ToFloat32(LAsmJSUInt32ToFloat32 *lir)
{
    masm.convertUInt32ToFloat32(ToRegister(lir->input()), ToFloatRegister(lir->output()));
}

void
CodeGeneratorARM64::visitNotI(LNotI *ins)
{
    ARMRegister input = toWRegister(ins->input());
    ARMRegister output = toWRegister(ins->output());
    masm.Cmp(input, ZeroRegister32);
    masm.Cset(output, Assembler::Zero);
}

//        NZCV
// NAN -> 0011
// ==  -> 0110
// <   -> 1000
// >   -> 0010
void
CodeGeneratorARM64::visitNotD(LNotD *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 64);
    ARMRegister output = toWRegister(ins->output());
    masm.Fcmp(input, 0.0);
    masm.Cset(output, Assembler::Equal);
    // dest is 1 iff input == 0, 0 otherwise
    // make it 1 if overflow was set.
    masm.Csinc(output, output, ZeroRegister32, Assembler::NoOverflow);
}

void
CodeGeneratorARM64::visitNotF(LNotF *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 32);
    ARMRegister output = toWRegister(ins->output());
    masm.Fcmp(input, 0.0);
    masm.Cset(output, Assembler::Equal);
    // dest is 1 iff input == 0, 0 otherwise
    // make it 1 if overflow was set.
    masm.Csinc(output, output, ZeroRegister32, Assembler::NoOverflow);
}

void 
CodeGeneratorARM64::visitLoadSlotV(LLoadSlotV *load)
{
    MOZ_CRASH("CodeGeneratorARM64::visitLoadSlotV");
}

void 
CodeGeneratorARM64::visitLoadSlotT(LLoadSlotT *load)
{
    MOZ_CRASH("CodeGeneratorARM64::visitLoadSlotT");
}

void 
CodeGeneratorARM64::visitStoreSlotT(LStoreSlotT *store)
{
    MOZ_CRASH("CodeGeneratorARM64::visitStoreSlotT");
}

void 
CodeGeneratorARM64::visitLoadElementT(LLoadElementT *load)
{
    MOZ_CRASH("CodeGeneratorARM64::visitLoadElementT");
}

void
CodeGeneratorARM64::storeElementTyped(const LAllocation *value, MIRType valueType,
                                      MIRType elementType, Register elements, const LAllocation *index)
{
    MOZ_CRASH("CodeGeneratorARM64::storeElementTyped");
}

void
CodeGeneratorARM64::visitGuardShape(LGuardShape *guard)
{
    ARMRegister obj = toXRegister(guard->input());
    ARMRegister tmp = toXRegister(guard->tempInt());
    Register tmp_ = ToRegister(guard->tempInt());
    masm.Ldr(tmp, MemOperand(obj, JSObject::offsetOfShape()));
    masm.cmpPtr(tmp_, ImmGCPtr(guard->mir()->shape()));

    bailoutIf(Assembler::NotEqual, guard->snapshot());

}
void
CodeGeneratorARM64::visitGuardObjectGroup(LGuardObjectGroup *guard)
{
    Register obj = ToRegister(guard->input());
    Register tmp = ToRegister(guard->tempInt());

    masm.Ldr(ARMRegister(tmp, 64), MemOperand(ARMRegister(obj, 64), JSObject::offsetOfGroup()));
    masm.cmpPtr(tmp, ImmGCPtr(guard->mir()->group()));

    Assembler::Condition cond =
        guard->mir()->bailOnEquality() ? Assembler::Equal : Assembler::NotEqual;
    bailoutIf(cond, guard->snapshot());
}


void 
CodeGeneratorARM64::visitGuardClass(LGuardClass *guard)
{
    MOZ_CRASH("CodeGeneratorARM64::visitGuardClass");
}

void 
CodeGeneratorARM64::visitInterruptCheck(LInterruptCheck *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitInterruptCheck");
}

void 
CodeGeneratorARM64::generateInvalidateEpilogue()
{
    // Ensure that there is enough space in the buffer for the OsiPoint patching
    // to occur. Otherwise, we could overwrite the invalidation epilogue.
    for (size_t i = 0; i < sizeof(void *); i += Assembler::NopSize())
        masm.nop();

    masm.bind(&invalidate_);

    // Push the return address of the point that we bailed out at onto the stack.
    masm.Push(lr);

    // Push the Ion script onto the stack (when we determine what that pointer is).
    invalidateEpilogueData_ = masm.pushWithPatch(ImmWord(uintptr_t(-1)));
    JitCode *thunk = gen->jitRuntime()->getInvalidationThunk();

    masm.branch(thunk);

    // We should never reach this point in JIT code -- the invalidation thunk
    // should pop the invalidated JS frame and return directly to its caller.
    masm.assumeUnreachable("Should have returned directly to its caller instead of here.");
}

void
DispatchIonCache::initializeAddCacheState(LInstruction *ins, AddCacheState *addState)
{
    MOZ_CRASH("CodeGeneratorARM64::initializeAddCacheState");
}

template <class U>
Register
getBase(U *mir)
{
    switch (mir->base()) {
      case U::Heap: return HeapReg;
      case U::Global: return GlobalReg;
    }
    return InvalidReg;
}

void 
CodeGeneratorARM64::visitLoadTypedArrayElementStatic(LLoadTypedArrayElementStatic *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitLoadTypedArrayElementStatic");
}

void 
CodeGeneratorARM64::visitStoreTypedArrayElementStatic(LStoreTypedArrayElementStatic *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitStoreTypedArrayElementStatic");
}

void 
CodeGeneratorARM64::visitAsmJSCall(LAsmJSCall *ins)
{

    emitAsmJSCall(ins);

}

void 
CodeGeneratorARM64::visitAsmJSLoadHeap(LAsmJSLoadHeap *ins)
{
    const MAsmJSLoadHeap *mir = ins->mir();
    LoadStoreOp op;
    const LAllocation *ptr = ins->ptr();
    switch (mir->accessType()) {
      case Scalar::Int8:    op = LDRSB_w; break;
      case Scalar::Uint8:   op = LDRB_w; break;
      case Scalar::Int16:   op = LDRSH_w; break;
      case Scalar::Uint16:  op = LDRH_w; break;
      case Scalar::Int32:   op = LDR_w; break;
      case Scalar::Uint32:  op = LDR_w; break;
      case Scalar::Float64: op = LDR_d; break;
      case Scalar::Float32: op = LDR_s; break;
      default: MOZ_CRASH("unexpected array type");
    }

    CPURegister rt = ToCPURegister(ins->output(), mir->accessType());
    if (ptr->isConstant()) {
        int32_t ptrImm = ptr->toConstant()->toInt32();
        MOZ_ASSERT(ptrImm >= 0);
        masm.LoadStoreMacro(rt, MemOperand(ARMRegister(HeapReg, 64), ptrImm), op);
        //memoryBarrier(mir->barrierAfter());
        return;
    }
    Label oob;
    Register ptrReg = ToRegister(ptr);
    if (mir->needsBoundsCheck()) {
        masm.BoundsCheck(ptrReg, &oob, rt);
    }
    masm.LoadStoreMacro(rt, MemOperand(ARMRegister(HeapReg, 64), ARMRegister(ptrReg, 64)), op);
    if (mir->needsBoundsCheck()) {
        masm.bind(&oob);
    }
}

void 
CodeGeneratorARM64::visitAsmJSStoreHeap(LAsmJSStoreHeap *ins)
{
    const MAsmJSStoreHeap *mir = ins->mir();
    LoadStoreOp op;
    const LAllocation *ptr = ins->ptr();
    bool isSigned = false;
    switch (mir->accessType()) {
      case Scalar::Int8:
      case Scalar::Uint8:   op = STRB_w; break;
      case Scalar::Int16:
      case Scalar::Uint16:  op = STRH_w; break;
      case Scalar::Int32:
      case Scalar::Uint32:  isSigned = true;  op = STR_w; break;
      case Scalar::Float64: op = STR_d; break;
      case Scalar::Float32: op = STR_s; break;
      default: MOZ_CRASH("unexpected array type");
    }

    // TODO: What is the point of isSigned here?
    (void) isSigned;

    CPURegister rt = ToCPURegister(ins->value(), mir->accessType());
    if (ptr->isConstant()) {
        int32_t ptrImm = ptr->toConstant()->toInt32();
        MOZ_ASSERT(ptrImm >= 0);
        // TODO: probably don't need a bounds check here, riiiigh?
        masm.LoadStoreMacro(rt, MemOperand(ARMRegister(HeapReg, 64), ptrImm), op);
        //memoryBarrier(mir->barrierAfter());
        return;
    }
    Register ptrReg = ToRegister(ptr);
    Label noStore;
    if (mir->needsBoundsCheck()) {
        masm.BoundsCheck(ptrReg, &noStore);
    }
    BufferOffset bo = masm.LoadStoreMacro(rt, MemOperand(ARMRegister(HeapReg, 64), ARMRegister(ptrReg, 64)), op);
    if (mir->needsBoundsCheck()) {
        masm.bind(&noStore);
    }
}

void 
CodeGeneratorARM64::visitAsmJSCompareExchangeHeap(LAsmJSCompareExchangeHeap *ins)
{
    MOZ_CRASH("visitAsmJSCompareExchangeHeap");
}

void 
CodeGeneratorARM64::visitAsmJSAtomicBinopHeap(LAsmJSAtomicBinopHeap *ins)
{
    MOZ_CRASH("visitAsmJSAtomicBinopHeap");
}

void
CodeGeneratorARM64::visitAsmJSPassStackArg(LAsmJSPassStackArg *ins)
{
    const MAsmJSPassStackArg *mir = ins->mir();
    MemOperand dst(masm.GetStackPointer(), mir->spOffset());
    if (ins->arg()->isConstant()) {
        ARMRegister tmp = ScratchReg2_32;
        if (ToInt32(ins->arg()) == 0) {
            tmp = wzr;
        } else {
            masm.Mov(tmp, Operand(ToInt32(ins->arg())));
        }
        masm.Str(tmp, dst);
    } else {
        if (ins->arg()->isGeneralReg())
            masm.Str(toWRegister(ins->arg()), dst);
        else
            masm.Str(ARMFPRegister(ToFloatRegister(ins->arg()), 64), dst);
    }
}

void
CodeGeneratorARM64::visitUDiv(LUDiv *ins)
{
    ARMRegister lhs = toWRegister(ins->lhs());
    ARMRegister rhs = toWRegister(ins->rhs());
    ARMRegister output = toWRegister(ins->output());

    Label done;
    if (ins->mir()->canBeDivideByZero()) {
        masm.Cmp(rhs, Operand(0));
        if (ins->mir()->isTruncated()) {
            // Infinity|0 == 0
            Label skip;
            masm.B(&skip, Assembler::NotEqual);
            masm.Mov(output, Operand(0));
            masm.B(&done);
            masm.bind(&skip);
        } else {
            MOZ_ASSERT(ins->mir()->fallible());
            bailoutIf(Assembler::Equal, ins->snapshot());
        }
    }

    masm.Udiv(output, lhs, rhs);
    if (!ins->mir()->toDiv()->canTruncateRemainder()) {
        Label bail;
        masm.Msub(ScratchReg2_32, output, rhs, lhs);
        masm.Cbnz(ScratchReg2_32, &bail);
        bailoutFrom(&bail, ins->snapshot());
    }
    if (!ins->mir()->isTruncated()) {
        masm.Cmp(output, Operand(0));
        bailoutIf(Assembler::LessThan, ins->snapshot());
    }

    masm.bind(&done);
}

void 
CodeGeneratorARM64::visitUMod(LUMod *ins)
{
    ARMRegister lhs = toWRegister(ins->lhs());
    ARMRegister rhs = toWRegister(ins->rhs());
    ARMRegister output = toWRegister(ins->output());
    Label done;

    if (ins->mir()->canBeDivideByZero()) {
        masm.Cmp(rhs, Operand(0));
        if (ins->mir()->isTruncated()) {
            // Infinity|0 == 0
            Label skip;
            masm.B(&skip, Assembler::NotEqual);
            masm.Mov(output, Operand(0));
            masm.B(&done);
            masm.bind(&skip);
        } else {
            MOZ_ASSERT(ins->mir()->fallible());
            bailoutIf(Assembler::Equal, ins->snapshot());
        }
    }

    masm.Udiv(output, lhs, rhs);
    masm.Msub(output, output, rhs, lhs);

    if (!ins->mir()->isTruncated()) {
        masm.Cmp(output, Operand(0));
        bailoutIf(Assembler::LessThan, ins->snapshot());
    }

    masm.bind(&done);
}

void 
CodeGeneratorARM64::visitEffectiveAddress(LEffectiveAddress *ins)
{
    const MEffectiveAddress *mir = ins->mir();
    ARMRegister base = toXRegister(ins->base());
    ARMRegister index = toXRegister(ins->index());
    ARMRegister output = toXRegister(ins->output());
    masm.Add(output, base, Operand(index, LSL, mir->scale()));
    masm.Add(output, output, Operand(mir->displacement()));
}

void 
CodeGeneratorARM64::visitAsmJSLoadGlobalVar(LAsmJSLoadGlobalVar *ins)
{
    ARMRegister GlobalPtr(GlobalReg, 64);
    const MAsmJSLoadGlobalVar *mir = ins->mir();
    int32_t addr = mir->globalDataOffset() - AsmJSGlobalRegBias;
    if (mir->type() == MIRType_Int32) {
        masm.Ldr(toWRegister(ins->output()), MemOperand(GlobalPtr, addr));
    } else if (mir->type() == MIRType_Float32) {
        ARMFPRegister vd(ToFloatRegister(ins->output()), 32);
        masm.Ldr(vd, MemOperand(GlobalPtr, addr));
    } else {
        ARMFPRegister vd(ToFloatRegister(ins->output()), 64);
        masm.Ldr(vd, MemOperand(GlobalPtr, addr));
    }

}

void
CodeGeneratorARM64::visitAsmJSStoreGlobalVar(LAsmJSStoreGlobalVar *ins)
{
    const MAsmJSStoreGlobalVar *mir = ins->mir();
    ARMRegister GlobalPtr(GlobalReg, 64);

    MIRType type = mir->value()->type();
    MOZ_ASSERT(IsNumberType(type));
    int32_t addr = mir->globalDataOffset() - AsmJSGlobalRegBias;
    if (type == MIRType_Int32) {
        masm.Str(toWRegister(ins->value()), MemOperand(GlobalPtr, addr));
    } else if (type == MIRType_Float32) {
        ARMFPRegister vd(ToFloatRegister(ins->value()), 32);
        masm.Str(vd, MemOperand(GlobalPtr, addr));
    } else {
        ARMFPRegister vd(ToFloatRegister(ins->value()), 64);
        masm.Str(vd, MemOperand(GlobalPtr, addr));
    }

}

void
CodeGeneratorARM64::visitAsmJSLoadFuncPtr(LAsmJSLoadFuncPtr *ins)
{
    const MAsmJSLoadFuncPtr *mir = ins->mir();

    ARMRegister index = toXRegister(ins->index());
    ARMRegister tmp = toXRegister(ins->temp());
    ARMRegister out = toXRegister(ins->output());
    unsigned addr = mir->globalDataOffset();
    masm.Mov(tmp, int64_t(addr) - AsmJSGlobalRegBias);
    masm.Add(tmp, tmp, Operand(index, LSL, 3));
    masm.Ldr(out, MemOperand(ARMRegister(GlobalReg, 64), tmp));
}

void
CodeGeneratorARM64::visitAsmJSLoadFFIFunc(LAsmJSLoadFFIFunc *ins)
{
    const MAsmJSLoadFFIFunc *mir = ins->mir();
    masm.Ldr(toXRegister(ins->output()),
             MemOperand(ARMRegister(GlobalReg, 64),
                        int64_t(mir->globalDataOffset()) - AsmJSGlobalRegBias));
}

void
CodeGeneratorARM64::visitNegI(LNegI *ins)
{
    ARMRegister input = toWRegister(ins->input());
    ARMRegister output = toWRegister(ins->output());
    masm.Neg(output, input);
}

void 
CodeGeneratorARM64::visitNegD(LNegD *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 64);
    ARMFPRegister output(ToFloatRegister(ins->output()), 64);
    masm.Fneg(output, input);
}

void 
CodeGeneratorARM64::visitNegF(LNegF *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 32);
    ARMFPRegister output(ToFloatRegister(ins->output()), 32);
    masm.Fneg(output, input);
}
