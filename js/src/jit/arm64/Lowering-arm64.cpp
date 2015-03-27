/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "mozilla/MathAlgorithms.h"

#include "jit/arm64/Assembler-arm64.h"
#include "jit/Lowering.h"
#include "jit/MIR.h"

#include "jit/shared/Lowering-shared-inl.h"

using namespace js;
using namespace js::jit;

using mozilla::FloorLog2;

// FIXME: Share with Lowering-x64.
void
LIRGeneratorARM64::useBox(LInstruction *lir, size_t n, MDefinition *mir,
                          LUse::Policy policy, bool useAtStart)
{
    MOZ_ASSERT(mir->type() == MIRType_Value);
    ensureDefined(mir);
    lir->setOperand(n, LUse(mir->virtualRegister(), policy, useAtStart));
}

// FIXME: Share with Lowering-x64.
void
LIRGeneratorARM64::useBoxFixed(LInstruction *lir, size_t n, MDefinition *mir, Register reg1, Register)
{
    MOZ_ASSERT(mir->type() == MIRType_Value);
    ensureDefined(mir);
    lir->setOperand(n, LUse(reg1, mir->virtualRegister()));
}

// FIXME: Share with Lowering-arm.
LAllocation
LIRGeneratorARM64::useByteOpRegister(MDefinition *mir)
{
    return useRegister(mir);
}

// FIXME: Share with Lowering-arm.
LAllocation
LIRGeneratorARM64::useByteOpRegisterOrNonDoubleConstant(MDefinition *mir)
{
    return useRegisterOrNonDoubleConstant(mir);
}

void
LIRGeneratorARM64::lowerConstantDouble(double d, MInstruction *mir)
{
    define(new(alloc()) LDouble(d), mir);
}

void
LIRGeneratorARM64::lowerConstantFloat32(float d, MInstruction *mir)
{
    define(new(alloc()) LFloat32(d), mir);
}

void
LIRGeneratorARM64::visitConstant(MConstant *ins)
{
    if (ins->type() == MIRType_Double)
        lowerConstantDouble(ins->value().toDouble(), ins);
    else if (ins->type() == MIRType_Float32)
        lowerConstantFloat32(ins->value().toDouble(), ins);
    else if (ins->canEmitAtUses())
        emitAtUses(ins);
    else
        LIRGeneratorShared::visitConstant(ins);
}

// FIXME: Share with Lowering-x64.
void
LIRGeneratorARM64::visitBox(MBox *box)
{
    MDefinition *opd = box->getOperand(0);

    // If the operand is a constant, emit near its uses.
    if (opd->isConstant() && box->canEmitAtUses()) {
        emitAtUses(box);
        return;
    }

    if (opd->isConstant()) {
        define(new(alloc()) LValue(opd->toConstant()->value()), box, LDefinition(LDefinition::BOX));
        return;
    }

    LBox *ins = new(alloc()) LBox(useRegister(opd), opd->type());
    define(ins, box, LDefinition(LDefinition::BOX));
}

// FIXME: Share with Lowering-x64.
void
LIRGeneratorARM64::visitUnbox(MUnbox *unbox)
{
    MDefinition *box = unbox->getOperand(0);
    LUnboxBase *lir;
    if (IsFloatingPointType(unbox->type()))
        lir = new(alloc()) LUnboxFloatingPoint(useRegisterAtStart(box), unbox->type());
    else
        lir = new(alloc()) LUnbox(useRegisterAtStart(box));

    if (unbox->fallible())
        assignSnapshot(lir, unbox->bailoutKind());

    define(lir, unbox);
}

// FIXME: Share with Lowering-x64.
void
LIRGeneratorARM64::visitReturn(MReturn *ret)
{
    MDefinition *opd = ret->getOperand(0);
    MOZ_ASSERT(opd->type() == MIRType_Value);

    LReturn *ins = new(alloc()) LReturn;
    ins->setOperand(0, useFixed(opd, JSReturnReg));
    add(ins);
}

// x = !y
void
LIRGeneratorARM64::lowerForALU(LInstructionHelper<1, 1, 0> *ins, MDefinition *mir, MDefinition *input)
{
    ins->setOperand(0, ins->snapshot() ? useRegister(input) : useRegisterAtStart(input));
    define(ins, mir, LDefinition(LDefinition::TypeFrom(mir->type()), LDefinition::REGISTER));
}

// z = x+y
void
LIRGeneratorARM64::lowerForALU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, ins->snapshot() ? useRegister(lhs) : useRegisterAtStart(lhs));
    ins->setOperand(1, ins->snapshot() ? useRegisterOrConstant(rhs) :
                                         useRegisterOrConstantAtStart(rhs));
    define(ins, mir, LDefinition(LDefinition::TypeFrom(mir->type()), LDefinition::REGISTER));
}

void
LIRGeneratorARM64::lowerForFPU(LInstructionHelper<1, 1, 0> *ins, MDefinition *mir, MDefinition *input)
{
    ins->setOperand(0, useRegisterAtStart(input));
    define(ins, mir, LDefinition(LDefinition::TypeFrom(mir->type()), LDefinition::REGISTER));
}

template <size_t Temps>
void
LIRGeneratorARM64::lowerForFPU(LInstructionHelper<1, 2, Temps> *ins, MDefinition *mir,
                               MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegisterAtStart(lhs));
    ins->setOperand(1, useRegisterAtStart(rhs));
    define(ins, mir, LDefinition(LDefinition::TypeFrom(mir->type()), LDefinition::REGISTER));
}

template void LIRGeneratorARM64::lowerForFPU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir,
                                             MDefinition *lhs, MDefinition *rhs);
template void LIRGeneratorARM64::lowerForFPU(LInstructionHelper<1, 2, 1> *ins, MDefinition *mir,
                                             MDefinition *lhs, MDefinition *rhs);

void
LIRGeneratorARM64::lowerForBitAndAndBranch(LBitAndAndBranch *baab, MInstruction *mir,
                                         MDefinition *lhs, MDefinition *rhs)
{
    baab->setOperand(0, useRegisterAtStart(lhs));
    baab->setOperand(1, useRegisterOrConstantAtStart(rhs));
    add(baab, mir);
}

void
LIRGeneratorARM64::defineUntypedPhi(MPhi *phi, size_t lirIndex)
{
    defineTypedPhi(phi, lirIndex);
}

void
LIRGeneratorARM64::lowerUntypedPhiInput(MPhi *phi, uint32_t inputPosition, LBlock *block, size_t lirIndex)
{
    lowerTypedPhiInput(phi, inputPosition, block, lirIndex);

}

void
LIRGeneratorARM64::lowerForShift(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegister(lhs));
    ins->setOperand(1, useRegisterOrConstant(rhs));
    define(ins, mir);
}

void
LIRGeneratorARM64::lowerDivI(MDiv *div)
{
    if (div->isUnsigned()) {
        lowerUDiv(div);
        return;
    }

    // Division instructions are slow. Division by constant denominators can be
    // rewritten to use other instructions.
    if (div->rhs()->isConstant()) {
        int32_t rhs = div->rhs()->toConstant()->value().toInt32();
        // Check for division by a positive power of two, which is an easy and
        // important case to optimize. Note that other optimizations are also
        // possible; division by negative powers of two can be optimized in a
        // similar manner as positive powers of two, and division by other
        // constants can be optimized by a reciprocal multiplication technique.
        int32_t shift = FloorLog2(rhs);
        if (rhs > 0 && 1 << shift == rhs) {
#if 0
            LDivPowTwoI *lir = new(alloc()) LDivPowTwoI(useRegisterAtStart(div->lhs()), shift);
            if (div->fallible())
                assignSnapshot(lir, Bailout_DoubleOutput);
            define(lir, div);
            return;
#endif
        }
    }

    LDivI *lir = new(alloc()) LDivI(useRegister(div->lhs()), useRegister(div->rhs()), temp());
    if (div->fallible())
        assignSnapshot(lir, Bailout_DoubleOutput);
    define(lir, div);
    return;
}

void
LIRGeneratorARM64::lowerMulI(MMul *mul, MDefinition *lhs, MDefinition *rhs)
{
    LMulI *lir = new(alloc()) LMulI;
    if (mul->fallible())
        assignSnapshot(lir, Bailout_DoubleOutput);
    lowerForALU(lir, mul, lhs, rhs);
}

void
LIRGeneratorARM64::lowerModI(MMod *mod)
{
    if (mod->isUnsigned()) {
        lowerUMod(mod);
        return;
    }

    if (mod->rhs()->isConstant()) {
        int32_t rhs = mod->rhs()->toConstant()->value().toInt32();
        int32_t shift = FloorLog2(rhs);
        if (rhs > 0 && 1 << shift == rhs) {
            LModPowTwoI *lir = new(alloc()) LModPowTwoI(useRegister(mod->lhs()), shift);
            if (mod->fallible())
                assignSnapshot(lir, Bailout_DoubleOutput);
            define(lir, mod);
            return;
        }
    }

    LModI *lir = new(alloc()) LModI(useRegister(mod->lhs()), useRegister(mod->rhs()), temp());
    if (mod->fallible())
        assignSnapshot(lir, Bailout_DoubleOutput);
    define(lir, mod);
}

void
LIRGeneratorARM64::visitPowHalf(MPowHalf *ins)
{
    MDefinition *input = ins->input();
    MOZ_ASSERT(input->type() == MIRType_Double);
    LPowHalfD *lir = new(alloc()) LPowHalfD(useRegister(input));
    define(lir, ins);
}

LTableSwitch *
LIRGeneratorARM64::newLTableSwitch(const LAllocation &in, const LDefinition &inputCopy,
                                       MTableSwitch *tableswitch)
{
    return new(alloc()) LTableSwitch(in, inputCopy, tableswitch);
}

LTableSwitchV *
LIRGeneratorARM64::newLTableSwitchV(MTableSwitch *tableswitch)
{
    return new(alloc()) LTableSwitchV(temp(), tempDouble(), tableswitch);
}

void
LIRGeneratorARM64::visitGuardShape(MGuardShape *ins)
{
    MOZ_ASSERT(ins->obj()->type() == MIRType_Object);

    LDefinition tempObj = temp(LDefinition::OBJECT);
    LGuardShape *guard = new(alloc()) LGuardShape(useRegister(ins->obj()), tempObj);
    assignSnapshot(guard, ins->bailoutKind());
    add(guard, ins);
    redefine(ins, ins->obj());
}

void
LIRGeneratorARM64::visitGuardObjectGroup(MGuardObjectGroup *ins)
{
    MOZ_CRASH("visitGuardObjectGroup");
    #if 0
    MOZ_ASSERT(ins->obj()->type() == MIRType_Object);

    LDefinition tempObj = temp(LDefinition::OBJECT);
    LGuardObjectType *guard = new(alloc()) LGuardObjectType(useRegister(ins->obj()), tempObj);
    assignSnapshot(guard, Bailout_ObjectIdentityOrTypeGuard);
    add(guard, ins);
    redefine(ins, ins->obj());
    #endif
}

void
LIRGeneratorARM64::lowerUrshD(MUrsh *mir)
{
    MDefinition *lhs = mir->lhs();
    MDefinition *rhs = mir->rhs();

    MOZ_ASSERT(lhs->type() == MIRType_Int32);
    MOZ_ASSERT(rhs->type() == MIRType_Int32);

    LUrshD *lir = new(alloc()) LUrshD(useRegister(lhs), useRegisterOrConstant(rhs), temp());
    define(lir, mir);
}

void
LIRGeneratorARM64::visitAsmJSNeg(MAsmJSNeg *ins)
{
    if (ins->type() == MIRType_Int32) {
        define(new(alloc()) LNegI(useRegisterAtStart(ins->input())), ins);
        return;
    }

    if (ins->type() == MIRType_Float32) {
        define(new(alloc()) LNegF(useRegisterAtStart(ins->input())), ins);
        return;
    }

    MOZ_ASSERT(ins->type() == MIRType_Double);
    define(new(alloc()) LNegD(useRegisterAtStart(ins->input())), ins);
}

void
LIRGeneratorARM64::lowerUDiv(MDiv *div)
{
    MDefinition *lhs = div->getOperand(0);
    MDefinition *rhs = div->getOperand(1);

    LUDiv *lir = new(alloc()) LUDiv;
    lir->setOperand(0, useRegister(lhs));
    lir->setOperand(1, useRegister(rhs));
    if (div->fallible())
        assignSnapshot(lir, Bailout_DoubleOutput);
    define(lir, div);
}

void
LIRGeneratorARM64::lowerUMod(MMod *mod)
{
    MDefinition *lhs = mod->getOperand(0);
    MDefinition *rhs = mod->getOperand(1);

    LUMod *lir = new(alloc()) LUMod;
    lir->setOperand(0, useRegister(lhs));
    lir->setOperand(1, useRegister(rhs));
    if (mod->fallible())
        assignSnapshot(lir, Bailout_DoubleOutput);
    define(lir, mod);
}

void
LIRGeneratorARM64::visitAsmJSUnsignedToDouble(MAsmJSUnsignedToDouble *ins)
{
    MOZ_ASSERT(ins->input()->type() == MIRType_Int32);
    LAsmJSUInt32ToDouble *lir = new(alloc()) LAsmJSUInt32ToDouble(useRegisterAtStart(ins->input()));
    define(lir, ins);
}

void
LIRGeneratorARM64::visitAsmJSUnsignedToFloat32(MAsmJSUnsignedToFloat32 *ins)
{
    MOZ_ASSERT(ins->input()->type() == MIRType_Int32);
    LAsmJSUInt32ToFloat32 *lir = new(alloc()) LAsmJSUInt32ToFloat32(useRegisterAtStart(ins->input()));
    define(lir, ins);
}

void
LIRGeneratorARM64::visitAsmJSLoadHeap(MAsmJSLoadHeap *ins)
{
    MDefinition *ptr = ins->ptr();
    MOZ_ASSERT(ptr->type() == MIRType_Int32);
    LAllocation ptrAlloc;

    // For the ARM it is best to keep the 'ptr' in a register if a bounds check is needed.
    if (ptr->isConstant() && !ins->needsBoundsCheck()) {
        // A bounds check is only skipped for a positive index.
        MOZ_ASSERT(ptr->toConstant()->value().toInt32() >= 0);
        ptrAlloc = LAllocation(ptr->toConstant()->vp());
    } else {
        ptrAlloc = useRegisterAtStart(ptr);
    }

    define(new(alloc()) LAsmJSLoadHeap(ptrAlloc), ins);
}

void
LIRGeneratorARM64::visitAsmJSStoreHeap(MAsmJSStoreHeap *ins)
{
    MDefinition *ptr = ins->ptr();
    MOZ_ASSERT(ptr->type() == MIRType_Int32);
    LAllocation ptrAlloc;

    if (ptr->isConstant() && !ins->needsBoundsCheck()) {
        MOZ_ASSERT(ptr->toConstant()->value().toInt32() >= 0);
        ptrAlloc = LAllocation(ptr->toConstant()->vp());
    } else {
        ptrAlloc = useRegisterAtStart(ptr);
    }

    add(new(alloc()) LAsmJSStoreHeap(ptrAlloc, useRegisterAtStart(ins->value())), ins);
}

void
LIRGeneratorARM64::visitAsmJSLoadFuncPtr(MAsmJSLoadFuncPtr *ins)
{
    define(new(alloc()) LAsmJSLoadFuncPtr(useRegister(ins->index()), temp()), ins);
}

void
LIRGeneratorARM64::visitAsmJSCompareExchangeHeap(MAsmJSCompareExchangeHeap *ins)
{
    MOZ_CRASH("visitAsmJSCompareExchangeHeap");
}

void
LIRGeneratorARM64::visitAsmJSAtomicBinopHeap(MAsmJSAtomicBinopHeap *ins)
{
    MOZ_CRASH("visitAsmJSAtomicBinopHeap");
}

void
LIRGeneratorARM64::lowerTruncateDToInt32(MTruncateToInt32 *ins)
{
    MDefinition *opd = ins->input();
    MOZ_ASSERT(opd->type() == MIRType_Double);

    define(new(alloc()) LTruncateDToInt32(useRegister(opd), LDefinition::BogusTemp()), ins);
}

void
LIRGeneratorARM64::lowerTruncateFToInt32(MTruncateToInt32 *ins)
{
    MDefinition *opd = ins->input();
    MOZ_ASSERT(opd->type() == MIRType_Float32);

    define(new(alloc()) LTruncateFToInt32(useRegister(opd), LDefinition::BogusTemp()), ins);
}

void
LIRGeneratorARM64::visitStoreTypedArrayElementStatic(MStoreTypedArrayElementStatic *ins)
{
    MOZ_CRASH("NYI");
}

void
LIRGeneratorARM64::visitSimdBinaryArith(MSimdBinaryArith *ins)
{
    MOZ_CRASH("NYI");
}

void
LIRGeneratorARM64::visitSimdSelect(MSimdSelect *ins)
{
    MOZ_CRASH("NYI");
}

void
LIRGeneratorARM64::visitSimdSplatX4(MSimdSplatX4 *ins)
{
    MOZ_CRASH("NYI");
}

void
LIRGeneratorARM64::visitSimdValueX4(MSimdValueX4 *ins)
{
    MOZ_CRASH("NYI");
}

void
LIRGeneratorARM64::visitAtomicTypedArrayElementBinop(MAtomicTypedArrayElementBinop *ins)
{
    MOZ_CRASH("NYI");
}

void
LIRGeneratorARM64::visitCompareExchangeTypedArrayElement(MCompareExchangeTypedArrayElement *ins)
{
    MOZ_CRASH("NYI");
}

void
LIRGeneratorARM64::visitSubstr(MSubstr *ins)
{
    LSubstr *lir = new (alloc()) LSubstr(useRegister(ins->string()),
                                         useRegister(ins->begin()),
                                         useRegister(ins->length()),
                                         temp(),
                                         temp(),
                                         temp());
    define(lir, ins);
    assignSafepoint(lir, ins);
}
