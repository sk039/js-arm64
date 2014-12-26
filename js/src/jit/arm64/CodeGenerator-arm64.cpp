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
    if (gen->info().executionMode() == SequentialExecution) {
        if (!emitTracelogStopEvent(TraceLogger::IonMonkey))
            return false;
        if (!emitTracelogScriptStop())
            return false;
    }
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
        masm.Mov(w30, frameSize());

        JitCode *handler = gen->jitRuntime()->getGenericBailoutHandler(gen->info().executionMode());
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
    MOZ_CRASH("OutOfLineBailout::accept");
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

    MOZ_CRASH("visitCompare (validate)");
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

    MOZ_CRASH("visitCompareAndBranch (validate)");
}

void 
CodeGeneratorARM64::bailoutIf(Assembler::Condition condition, LSnapshot *snapshot)
{
    MOZ_CRASH("CodeGeneratorARM64::bailoutIf");
}

void 
CodeGeneratorARM64::bailoutFrom(Label *label, LSnapshot *snapshot)
{
    MOZ_CRASH("CodeGeneratorARM64::bailoutFrom");
}

void 
CodeGeneratorARM64::bailout(LSnapshot *snapshot)
{
    MOZ_CRASH("CodeGeneratorARM64::bailout");
}

void 
CodeGeneratorARM64::visitOutOfLineBailout(OutOfLineBailout *ool)
{
    MOZ_CRASH("CodeGeneratorARM64::visitOutOfLineBailout");
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
ToCPURegister(const LAllocation *a)
{
    if (a->isFloatReg())
        return ARMFPRegister(ToFloatRegister(a), 64);
    if (a->isGeneralReg())
        return ARMRegister(ToRegister(a), 32);
    MOZ_CRASH("Unknown LAllocation");
}
CPURegister
ToCPURegister(const LDefinition *d)
{
    return ToCPURegister(d->output());
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
            masm.move32(Imm32(constant), ScratchReg);
            rhs_reg = ScratchReg;
        }

    } else {
        rhs_reg = ToRegister(rhs);
    }

    if (mul->canOverflow()) {
        MOZ_CRASH("TODO: Handle overflow");
        // I should /really/ be able to just bail-out directly from the macro assembler.
        // this song-and-dance with condition codes seems unweildy in this case.
        masm.mul32(ToRegister(lhs), rhs_reg, ToRegister(dest), nullptr, nullptr);
    } else {
        masm.mul32(ToRegister(lhs), rhs_reg, ToRegister(dest), nullptr, nullptr);
    }
}

void 
CodeGeneratorARM64::divICommon(MDiv *mir, Register lhs, Register rhs, Register output,
                             LSnapshot *snapshot, Label &done)
{
    MOZ_CRASH("CodeGeneratorARM64::divICommon");
}

void 
CodeGeneratorARM64::visitDivI(LDivI *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitDivI");
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
    MOZ_CRASH("CodeGeneratorARM64::visitModI");
}

void 
CodeGeneratorARM64::visitModPowTwoI(LModPowTwoI *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitModPowTwoI");
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
    ARMRegister dest = toWRegister(ins->rhs());
    if (ins->rhs()->isConstant()) {
        ARMRegister rhs = toWRegister(ins->rhs());
        switch (ins->bitop()) {
          case JSOP_LSH:
            masm.Lsl(dest, lhs, rhs);
            break;
          case JSOP_RSH:
            masm.Asr(dest, lhs, rhs);
            break;
          case JSOP_URSH:
            masm.Lsr(dest, lhs, rhs);
            // TODO: bail if the result is negative
            break;
          default:
            MOZ_CRASH("Unexpected shift opcode");
        }
    } else {
        int32_t rhs = ToInt32(ins->rhs()) & 0x1F;
        switch (ins->bitop()) {
          case JSOP_LSH:
            masm.Lsl(dest, lhs, rhs);
            break;
          case JSOP_RSH:
            masm.Asr(dest, lhs, rhs);
            break;
          case JSOP_URSH:
            masm.Lsr(dest, lhs, rhs);
            // TODO: bail if the result is negative.
            break;
          default:
            MOZ_CRASH("Unexpected shift opcode");
        }
    }
}

void 
CodeGeneratorARM64::visitUrshD(LUrshD *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitUrshD");
}

void 
CodeGeneratorARM64::visitPowHalfD(LPowHalfD *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitPowHalfD");
}

MoveOperand
CodeGeneratorARM64::toMoveOperand(const LAllocation *a) const
{
    if (a->isGeneralReg())
        return MoveOperand(ToRegister(a));
    if (a->isFloatReg())
        return MoveOperand(ToFloatRegister(a));
    int32_t offset = ToStackOffset(a);
    MOZ_ASSERT((offset & 7) == 0);
    return MoveOperand(StackPointer, offset);
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
    MOZ_CRASH("CodeGeneratorARM64::visitOutOfLineTableSwitch");
}

void 
CodeGeneratorARM64::emitTableSwitchDispatch(MTableSwitch *mir, Register index, Register base)
{
    MOZ_CRASH("CodeGeneratorARM64::emitTableSwitchDispatch");
}

void 
CodeGeneratorARM64::visitMathD(LMathD *math)
{
    const ARMFPRegister src1(ToFloatRegister(math->getOperand(0)), 64);
    const ARMFPRegister src2(ToFloatRegister(math->getOperand(1)), 64);
    const ARMFPRegister output(ToFloatRegister(math->getOperand(0)), 64);

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
    MOZ_CRASH("CodeGeneratorARM64::visitMathF");
}

void 
CodeGeneratorARM64::visitFloor(LFloor *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitFloor");
}

void 
CodeGeneratorARM64::visitFloorF(LFloorF *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitFloorF");
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
    MOZ_CRASH("CodeGeneratorARM64::visitRound");
}

void 
CodeGeneratorARM64::visitRoundF(LRoundF *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitRoundF");
}

void
CodeGeneratorARM64::emitRoundDouble(FloatRegister src, Register dest, Label *fail)
{
    MOZ_CRASH("CodeGeneratorARM64::emitRoundDouble");
}

void 
CodeGeneratorARM64::visitTruncateDToInt32(LTruncateDToInt32 *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitTruncateDToInt32");
}

void 
CodeGeneratorARM64::visitTruncateFToInt32(LTruncateFToInt32 *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitTruncateFToInt32");
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
    MOZ_CRASH("CodeGeneratorARM64::ToValue");
}

ValueOperand
CodeGeneratorARM64::ToOutValue(LInstruction *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::ToOutValue");
}

ValueOperand
CodeGeneratorARM64::ToTempValue(LInstruction *ins, size_t pos)
{
    MOZ_CRASH("CodeGeneratorARM64::ToTempValue");
}

void 
CodeGeneratorARM64::visitValue(LValue *value)
{
    MOZ_CRASH("CodeGeneratorARM64::visitValue");
}

void 
CodeGeneratorARM64::visitBox(LBox *box)
{
    MOZ_CRASH("CodeGeneratorARM64::visitBox");
}

void 
CodeGeneratorARM64::visitUnbox(LUnbox *unbox)
{
    MOZ_CRASH("CodeGeneratorARM64::visitUnbox");
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
    MOZ_CRASH("CodeGeneratorARM64::visitTestDAndBranch");
}

void 
CodeGeneratorARM64::visitTestFAndBranch(LTestFAndBranch *test)
{
    MOZ_CRASH("CodeGeneratorARM64::visitTestFAndBranch");
}

void 
CodeGeneratorARM64::visitCompareD(LCompareD *comp)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareD");
}

void 
CodeGeneratorARM64::visitCompareF(LCompareF *comp)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareF");
}

void 
CodeGeneratorARM64::visitCompareDAndBranch(LCompareDAndBranch *comp)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareDAndBranch");
}

void 
CodeGeneratorARM64::visitCompareFAndBranch(LCompareFAndBranch *comp)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareFAndBranch");
}

void 
CodeGeneratorARM64::visitCompareB(LCompareB *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareB");
}

void 
CodeGeneratorARM64::visitCompareBAndBranch(LCompareBAndBranch *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareBAndBranch");
}

void 
CodeGeneratorARM64::visitCompareV(LCompareV *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareV");
}

void 
CodeGeneratorARM64::visitCompareVAndBranch(LCompareVAndBranch *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareVAndBranch");
}

void 
CodeGeneratorARM64::visitBitAndAndBranch(LBitAndAndBranch *baab)
{
    MOZ_CRASH("CodeGeneratorARM64::visitBitAndAndBranch");
}

void 
CodeGeneratorARM64::visitAsmJSUInt32ToDouble(LAsmJSUInt32ToDouble *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSUInt32ToDouble");
}

void 
CodeGeneratorARM64::visitAsmJSUInt32ToFloat32(LAsmJSUInt32ToFloat32 *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSUInt32ToFloat32");
}

void 
CodeGeneratorARM64::visitNotI(LNotI *ins)
{
    ARMRegister input = toWRegister(ins->input());
    ARMRegister output = toWRegister(ins->output());
    masm.Cmp(input, ZeroRegister32);
    masm.Cset(output, Assembler::NonZero);
    MOZ_CRASH("visitNotI (validate)");
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
    masm.Csneg(output, output, ZeroRegister32, Assembler::Overflow);
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
    masm.Csneg(output, output, ZeroRegister32, Assembler::Overflow);
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
    MOZ_CRASH("CodeGeneratorARM64::visitGuardShape");
}

void 
CodeGeneratorARM64::visitGuardObjectType(LGuardObjectType *guard)
{
    MOZ_CRASH("CodeGeneratorARM64::visitGuardObjectType");
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
    MOZ_CRASH("CodeGeneratorARM64::generateInvalidateEpilogue");
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
    MOZ_CRASH("visitAsmJSCall");
}

void 
CodeGeneratorARM64::visitAsmJSLoadHeap(LAsmJSLoadHeap *ins)
{
    const MAsmJSLoadHeap *mir = ins->mir();
    LoadStoreOp op;
    const LAllocation *ptr = ins->ptr();
    bool isSigned = false;
    switch (mir->viewType()) {
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

    CPURegister rt = ToCPURegister(ins->output());
    if (ptr->isConstant()) {
        int32_t ptrImm = ptr->toConstant()->toInt32();
        MOZ_ASSERT(ptrImm >= 0);
        masm.LoadStoreMacro(rt, MemOperand(ARMRegister(HeapReg, 64), ptrImm), op);
        //memoryBarrier(mir->barrierAfter());
        return;
    }
    Register ptrReg = ToRegister(ptr);
    BufferOffset bo = masm.LoadStoreMacro(rt, MemOperand(ARMRegister(HeapReg, 64), ARMRegister(ptrReg, 64)), op);
    if (mir->needsBoundsCheck()) {
        masm.append(AsmJSHeapAccess(bo.getOffset()));
    }
}

void 
CodeGeneratorARM64::visitAsmJSStoreHeap(LAsmJSStoreHeap *ins)
{
    const MAsmJSStoreHeap *mir = ins->mir();
    LoadStoreOp op;
    const LAllocation *ptr = ins->ptr();
    bool isSigned = false;
    switch (mir->viewType()) {
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

    CPURegister rt = ToCPURegister(ins->value());
    if (ptr->isConstant()) {
        int32_t ptrImm = ptr->toConstant()->toInt32();
        MOZ_ASSERT(ptrImm >= 0);
        masm.LoadStoreMacro(rt, MemOperand(ARMRegister(HeapReg, 64), ptrImm), op);
        //memoryBarrier(mir->barrierAfter());
        return;
    }
    Register ptrReg = ToRegister(ptr);
    BufferOffset bo = masm.LoadStoreMacro(rt, MemOperand(ARMRegister(HeapReg, 64), ARMRegister(ptrReg, 64)), op);
    if (mir->needsBoundsCheck()) {
        masm.append(AsmJSHeapAccess(bo.getOffset()));
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
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSPassStackArg");
}

void 
CodeGeneratorARM64::visitUDiv(LUDiv *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitUDiv");
}

void 
CodeGeneratorARM64::visitUMod(LUMod *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitUMod");
}

void 
CodeGeneratorARM64::visitSoftUDivOrMod(LSoftUDivOrMod *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitSoftUDivOrMod");
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
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSLoadGlobalVar");
}

void 
CodeGeneratorARM64::visitAsmJSStoreGlobalVar(LAsmJSStoreGlobalVar *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSStoreGlobalVar");
}

void 
CodeGeneratorARM64::visitAsmJSLoadFuncPtr(LAsmJSLoadFuncPtr *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSLoadFuncPtr");
}

void
CodeGeneratorARM64::visitAsmJSLoadFFIFunc(LAsmJSLoadFFIFunc *ins)
{
    const MAsmJSLoadFFIFunc *mir = ins->mir();
    masm.Ldr(toWRegister(ins->output()),
             MemOperand(ARMRegister(GlobalReg, 64),
                        mir->globalDataOffset() - AsmJSGlobalRegBias));
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

void 
CodeGeneratorARM64::visitForkJoinGetSlice(LForkJoinGetSlice *ins)
{
    MOZ_CRASH("NYI");
}

JitCode *
JitRuntime::generateForkJoinGetSliceStub(JSContext *cx)
{
    MOZ_CRASH("NYI");
}
