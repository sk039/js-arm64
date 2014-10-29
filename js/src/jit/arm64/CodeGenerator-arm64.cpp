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
#include "jit/IonFrames.h"
#include "jit/JitCompartment.h"
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
    masm.push(lr);
    return false;
}

bool
CodeGeneratorARM64::generateAsmJSPrologue(Label *stackOverflowLabel)
{
    masm.push(lr);
    return false;
}

bool
CodeGeneratorARM64::generateEpilogue()
{
    masm.pop(lr);
    return false;
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


bool
OutOfLineBailout::accept(CodeGeneratorARM64 *codegen)
{
    MOZ_CRASH("CodeGeneratorARM64::accept");
    return false;
}

bool
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
    return true;
}

bool
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
    return true;
}

bool
CodeGeneratorARM64::visitCompareAndBranch(LCompareAndBranch *comp)
{
    Assembler::Condition cond = JSOpToCondition(comp->cmpMir()->compareType(), comp->jsop());
    if (comp->right()->isConstant())
        masm.cmp32(ToRegister(comp->left()), Imm32(ToInt32(comp->right())));
    else
        masm.cmp32(ToRegister(comp->left()), ToRegister(comp->right()));
    emitBranch(cond, comp->ifTrue(), comp->ifFalse());

    return false;
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

bool
CodeGeneratorARM64::bailoutIf(Assembler::Condition condition, LSnapshot *snapshot)
{
    MOZ_CRASH("CodeGeneratorARM64::bailoutIf");
    return false;
}

bool
CodeGeneratorARM64::bailoutFrom(Label *label, LSnapshot *snapshot)
{
    MOZ_CRASH("CodeGeneratorARM64::bailoutFrom");
    return false;
}

bool
CodeGeneratorARM64::bailout(LSnapshot *snapshot)
{
    MOZ_CRASH("CodeGeneratorARM64::bailout");
    return false;
}

bool
CodeGeneratorARM64::visitOutOfLineBailout(OutOfLineBailout *ool)
{
    MOZ_CRASH("CodeGeneratorARM64::visitOutOfLineBailout");
    return false;
}

bool
CodeGeneratorARM64::visitMinMaxD(LMinMaxD *ins)
{
    ARMFPRegister lhs(ToFloatRegister(ins->first()), 64);
    ARMFPRegister rhs(ToFloatRegister(ins->second()), 64);
    ARMFPRegister output(ToFloatRegister(ins->output()), 64);
    if (ins->mir()->isMax())
        masm.Fmax(output, lhs, rhs);
    else
        masm.Fmin(output, lhs, rhs);
    return true;
}

bool
CodeGeneratorARM64::visitAbsD(LAbsD *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 64);
    masm.Fabs(input, input);
    return true;
}

bool
CodeGeneratorARM64::visitAbsF(LAbsF *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 32);
    masm.Fabs(input, input);
    return true;
}

bool
CodeGeneratorARM64::visitSqrtD(LSqrtD *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 64);
    ARMFPRegister output(ToFloatRegister(ins->output()), 64);
    masm.Fsqrt(output, input);
    return true;
}

bool
CodeGeneratorARM64::visitSqrtF(LSqrtF *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 32);
    ARMFPRegister output(ToFloatRegister(ins->output()), 32);
    masm.Fsqrt(output, input);
    return true;
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
bool
CodeGeneratorARM64::visitAddI(LAddI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);
    const LDefinition *dest = ins->getDef(0);
    if (ins->snapshot()) {
        masm.Adds(toWRegister(dest), toWRegister(lhs), toWOperand(rhs));
        if (!bailoutIf(Assembler::Overflow, ins->snapshot()))
            return false;

    } else {
        masm.Add(toWRegister(dest), toWRegister(lhs), toWOperand(rhs));
    }
    return true;
}

bool
CodeGeneratorARM64::visitSubI(LSubI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);
    const LDefinition *dest = ins->getDef(0);
    if (ins->snapshot()) {
        masm.Subs(toWRegister(dest), toWRegister(lhs), toWOperand(rhs));
        if (!bailoutIf(Assembler::Overflow, ins->snapshot()))
            return false;

    } else {
        masm.Sub(toWRegister(dest), toWRegister(lhs), toWOperand(rhs));
    }
    return true;
}

bool
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
            if (!bailoutIf(bailoutCond, ins->snapshot()))
                return false;
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
    return true;

}

bool
CodeGeneratorARM64::divICommon(MDiv *mir, Register lhs, Register rhs, Register output,
                             LSnapshot *snapshot, Label &done)
{
    MOZ_CRASH("CodeGeneratorARM64::divICommon");
    return false;
}

bool
CodeGeneratorARM64::visitDivI(LDivI *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitDivI");
    return false;
}

bool
CodeGeneratorARM64::visitDivPowTwoI(LDivPowTwoI *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitDivPowTwoI");
    return false;
}

bool
CodeGeneratorARM64::modICommon(MMod *mir, Register lhs, Register rhs, Register output,
                               LSnapshot *snapshot, Label &done)
{
    MOZ_CRASH("CodeGeneratorARM64::modICommon");
    return false;
}

bool
CodeGeneratorARM64::visitModI(LModI *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitModI");
    return false;
}

bool
CodeGeneratorARM64::visitModPowTwoI(LModPowTwoI *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitModPowTwoI");
    return false;
}

bool
CodeGeneratorARM64::visitModMaskI(LModMaskI *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitModMaskI");
    return false;
}

bool
CodeGeneratorARM64::visitBitNotI(LBitNotI *ins)
{
    const LAllocation *input = ins->getOperand(0);
    const LDefinition *output = ins->getDef(0);
    masm.Mvn(toWRegister(output), toWOperand(input));
    return true;
}

bool
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

    return true;
}

bool
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
    return true;
}

bool
CodeGeneratorARM64::visitUrshD(LUrshD *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitUrshD");
    return false;
}

bool
CodeGeneratorARM64::visitPowHalfD(LPowHalfD *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitPowHalfD");
    return false;
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
    Vector<CodeLabel, 8, IonAllocPolicy> codeLabels_;

    bool accept(CodeGeneratorARM64 *codegen) {
        return codegen->visitOutOfLineTableSwitch(this);
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

bool
CodeGeneratorARM64::visitOutOfLineTableSwitch(OutOfLineTableSwitch *ool)
{
    MOZ_CRASH("CodeGeneratorARM64::visitOutOfLineTableSwitch");
    return false;
}

bool
CodeGeneratorARM64::emitTableSwitchDispatch(MTableSwitch *mir, Register index, Register base)
{
    MOZ_CRASH("CodeGeneratorARM64::emitTableSwitchDispatch");
    return false;
}

bool
CodeGeneratorARM64::visitMathD(LMathD *math)
{
    MOZ_CRASH("CodeGeneratorARM64::visitMathD");
    return false;
}

bool
CodeGeneratorARM64::visitMathF(LMathF *math)
{
    MOZ_CRASH("CodeGeneratorARM64::visitMathF");
    return false;
}

bool
CodeGeneratorARM64::visitFloor(LFloor *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitFloor");
    return false;
}

bool
CodeGeneratorARM64::visitFloorF(LFloorF *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitFloorF");
    return false;
}

bool
CodeGeneratorARM64::visitCeil(LCeil *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCeil");
    return false;
}

bool
CodeGeneratorARM64::visitCeilF(LCeilF *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCeilF");
    return false;
}

bool
CodeGeneratorARM64::visitRound(LRound *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitRound");
    return false;
}

bool
CodeGeneratorARM64::visitRoundF(LRoundF *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitRoundF");
    return false;
}

void
CodeGeneratorARM64::emitRoundDouble(FloatRegister src, Register dest, Label *fail)
{
    MOZ_CRASH("CodeGeneratorARM64::emitRoundDouble");
}

bool
CodeGeneratorARM64::visitTruncateDToInt32(LTruncateDToInt32 *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitTruncateDToInt32");
    return false;
}

bool
CodeGeneratorARM64::visitTruncateFToInt32(LTruncateFToInt32 *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitTruncateFToInt32");
    return false;
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

bool
CodeGeneratorARM64::visitValue(LValue *value)
{
    MOZ_CRASH("CodeGeneratorARM64::visitValue");
    return false;
}

bool
CodeGeneratorARM64::visitBox(LBox *box)
{
    MOZ_CRASH("CodeGeneratorARM64::visitBox");
    return false;
}

bool
CodeGeneratorARM64::visitUnbox(LUnbox *unbox)
{
    MOZ_CRASH("CodeGeneratorARM64::visitUnbox");
    return false;
}

bool
CodeGeneratorARM64::visitDouble(LDouble *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitDouble");
    return false;
}

bool
CodeGeneratorARM64::visitFloat32(LFloat32 *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitFloat32");
    return false;
}

Register
CodeGeneratorARM64::splitTagForTest(const ValueOperand &value)
{
    MOZ_CRASH("CodeGeneratorARM64::splitTagForTest");
    return InvalidReg;
}

bool
CodeGeneratorARM64::visitTestDAndBranch(LTestDAndBranch *test)
{
    MOZ_CRASH("CodeGeneratorARM64::visitTestDAndBranch");
    return false;
}

bool
CodeGeneratorARM64::visitTestFAndBranch(LTestFAndBranch *test)
{
    MOZ_CRASH("CodeGeneratorARM64::visitTestFAndBranch");
    return false;
}

bool
CodeGeneratorARM64::visitCompareD(LCompareD *comp)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareD");
    return false;
}

bool
CodeGeneratorARM64::visitCompareF(LCompareF *comp)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareF");
    return false;
}

bool
CodeGeneratorARM64::visitCompareDAndBranch(LCompareDAndBranch *comp)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareDAndBranch");
    return false;
}

bool
CodeGeneratorARM64::visitCompareFAndBranch(LCompareFAndBranch *comp)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareFAndBranch");
    return false;
}

bool
CodeGeneratorARM64::visitCompareB(LCompareB *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareB");
    return false;
}

bool
CodeGeneratorARM64::visitCompareBAndBranch(LCompareBAndBranch *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareBAndBranch");
    return false;
}

bool
CodeGeneratorARM64::visitCompareV(LCompareV *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareV");
    return false;
}

bool
CodeGeneratorARM64::visitCompareVAndBranch(LCompareVAndBranch *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitCompareVAndBranch");
    return false;
}

bool
CodeGeneratorARM64::visitBitAndAndBranch(LBitAndAndBranch *baab)
{
    MOZ_CRASH("CodeGeneratorARM64::visitBitAndAndBranch");
    return false;
}

bool
CodeGeneratorARM64::visitAsmJSUInt32ToDouble(LAsmJSUInt32ToDouble *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSUInt32ToDouble");
    return false;
}

bool
CodeGeneratorARM64::visitAsmJSUInt32ToFloat32(LAsmJSUInt32ToFloat32 *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSUInt32ToFloat32");
    return false;
}

bool
CodeGeneratorARM64::visitNotI(LNotI *ins)
{
    ARMRegister input = toWRegister(ins->input());
    ARMRegister output = toWRegister(ins->output());
    masm.Cmp(input, ZeroRegister32);
    masm.Cset(output, Assembler::NonZero);
    return false;
}

//        NZCV
// NAN -> 0011
// ==  -> 0110
// <   -> 1000
// >   -> 0010
bool
CodeGeneratorARM64::visitNotD(LNotD *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 64);
    ARMRegister output = toWRegister(ins->output());
    masm.Fcmp(input, 0.0);
    masm.Cset(output, Assembler::Equal);
    // dest is 1 iff input == 0, 0 otherwise
    // make it 1 if overflow was set.
    masm.Csneg(output, output, ZeroRegister32, Assembler::Overflow);
    return true;
}

bool
CodeGeneratorARM64::visitNotF(LNotF *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 32);
    ARMRegister output = toWRegister(ins->output());
    masm.Fcmp(input, 0.0);
    masm.Cset(output, Assembler::Equal);
    // dest is 1 iff input == 0, 0 otherwise
    // make it 1 if overflow was set.
    masm.Csneg(output, output, ZeroRegister32, Assembler::Overflow);
    return true;
}

bool
CodeGeneratorARM64::visitLoadSlotV(LLoadSlotV *load)
{
    MOZ_CRASH("CodeGeneratorARM64::visitLoadSlotV");
    return false;
}

bool
CodeGeneratorARM64::visitLoadSlotT(LLoadSlotT *load)
{
    MOZ_CRASH("CodeGeneratorARM64::visitLoadSlotT");
    return false;
}

bool
CodeGeneratorARM64::visitStoreSlotT(LStoreSlotT *store)
{
    MOZ_CRASH("CodeGeneratorARM64::visitStoreSlotT");
    return false;
}

bool
CodeGeneratorARM64::visitLoadElementT(LLoadElementT *load)
{
    MOZ_CRASH("CodeGeneratorARM64::visitLoadElementT");
    return false;
}

void
CodeGeneratorARM64::storeElementTyped(const LAllocation *value, MIRType valueType,
                                      MIRType elementType, Register elements, const LAllocation *index)
{
    MOZ_CRASH("CodeGeneratorARM64::storeElementTyped");
}

bool
CodeGeneratorARM64::visitGuardShape(LGuardShape *guard)
{
    MOZ_CRASH("CodeGeneratorARM64::visitGuardShape");
    return false;
}

bool
CodeGeneratorARM64::visitGuardObjectType(LGuardObjectType *guard)
{
    MOZ_CRASH("CodeGeneratorARM64::visitGuardObjectType");
    return false;
}

bool
CodeGeneratorARM64::visitGuardClass(LGuardClass *guard)
{
    MOZ_CRASH("CodeGeneratorARM64::visitGuardClass");
    return false;
}

bool
CodeGeneratorARM64::visitInterruptCheck(LInterruptCheck *lir)
{
    MOZ_CRASH("CodeGeneratorARM64::visitInterruptCheck");
    return false;
}

bool
CodeGeneratorARM64::generateInvalidateEpilogue()
{
    MOZ_CRASH("CodeGeneratorARM64::generateInvalidateEpilogue");
    return false;
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

bool
CodeGeneratorARM64::visitLoadTypedArrayElementStatic(LLoadTypedArrayElementStatic *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitLoadTypedArrayElementStatic");
    return false;
}

bool
CodeGeneratorARM64::visitStoreTypedArrayElementStatic(LStoreTypedArrayElementStatic *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitStoreTypedArrayElementStatic");
    return false;
}

bool
CodeGeneratorARM64::visitAsmJSLoadHeap(LAsmJSLoadHeap *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSLoadHeap");
    return false;
}

bool
CodeGeneratorARM64::visitAsmJSStoreHeap(LAsmJSStoreHeap *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSStoreHeap");
    return false;
}

bool
CodeGeneratorARM64::visitAsmJSPassStackArg(LAsmJSPassStackArg *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSPassStackArg");
    return false;
}

bool
CodeGeneratorARM64::visitUDiv(LUDiv *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitUDiv");
    return false;
}

bool
CodeGeneratorARM64::visitUMod(LUMod *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitUMod");
    return false;
}

bool
CodeGeneratorARM64::visitSoftUDivOrMod(LSoftUDivOrMod *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitSoftUDivOrMod");
    return false;
}

bool
CodeGeneratorARM64::visitEffectiveAddress(LEffectiveAddress *ins)
{
    const MEffectiveAddress *mir = ins->mir();
    ARMRegister base = toXRegister(ins->base());
    ARMRegister index = toXRegister(ins->index());
    ARMRegister output = toXRegister(ins->output());
    masm.Add(output, base, Operand(index, LSL, mir->scale()));
    masm.Add(output, output, Operand(mir->displacement()));
    return true;
}

bool
CodeGeneratorARM64::visitAsmJSLoadGlobalVar(LAsmJSLoadGlobalVar *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSLoadGlobalVar");
    return false;
}

bool
CodeGeneratorARM64::visitAsmJSStoreGlobalVar(LAsmJSStoreGlobalVar *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSStoreGlobalVar");
    return false;
}

bool
CodeGeneratorARM64::visitAsmJSLoadFuncPtr(LAsmJSLoadFuncPtr *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSLoadFuncPtr");
    return false;
}

bool
CodeGeneratorARM64::visitAsmJSLoadFFIFunc(LAsmJSLoadFFIFunc *ins)
{
    MOZ_CRASH("CodeGeneratorARM64::visitAsmJSLoadFFIFunc");
    return false;
}

bool
CodeGeneratorARM64::visitNegI(LNegI *ins)
{
    ARMRegister input = toWRegister(ins->input());
    ARMRegister output = toWRegister(ins->output());
    masm.Neg(output, input);
    return true;
}

bool
CodeGeneratorARM64::visitNegD(LNegD *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 64);
    ARMFPRegister output(ToFloatRegister(ins->output()), 64);
    masm.Fneg(output, input);
    return false;
}

bool
CodeGeneratorARM64::visitNegF(LNegF *ins)
{
    ARMFPRegister input(ToFloatRegister(ins->input()), 32);
    ARMFPRegister output(ToFloatRegister(ins->output()), 32);
    masm.Fneg(output, input);
    return true;
}

bool
CodeGeneratorARM64::visitForkJoinGetSlice(LForkJoinGetSlice *ins)
{
    MOZ_CRASH("NYI");
}

JitCode *
JitRuntime::generateForkJoinGetSliceStub(JSContext *cx)
{
    MOZ_CRASH("NYI");
}
