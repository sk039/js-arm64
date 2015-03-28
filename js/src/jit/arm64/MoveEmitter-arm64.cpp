/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/arm64/MoveEmitter-arm64.h"

using namespace js;
using namespace js::jit;

void
MoveEmitterARM64::emit(const MoveResolver &moves)
{
    if (moves.numCycles()) {
        masm.reserveStack(sizeof(void *));
        pushedAtCycle_ = masm.framePushed();
    }

    // TODO: Optimize cycles.
    for (size_t i = 0; i < moves.numMoves(); i++)
        emitMove(moves.getMove(i));
}

void
MoveEmitterARM64::finish()
{
    MOZ_ASSERT(!inCycle_);
    masm.freeStack(masm.framePushed() - pushedAtStart_);
    MOZ_ASSERT(masm.framePushed() == pushedAtStart_);
}

void
MoveEmitterARM64::emitMove(const MoveOp &move)
{
    const MoveOperand &from = move.from();
    const MoveOperand &to = move.to();

    if (move.isCycleEnd()) {
        MOZ_ASSERT(inCycle_);
        completeCycle(from, to, move.type());
        inCycle_ = false;
        return;
    }

    // TODO: MoveEmitterX86 has logic to attempt to avoid using the stack.
    if (move.isCycleBegin()) {
        MOZ_ASSERT(!inCycle_);
        breakCycle(from, to, move.endCycleType());
        inCycle_ = true;
    }

    switch (move.type()) {
      case MoveOp::FLOAT32:
        emitFloat32Move(from, to);
        break;
      case MoveOp::DOUBLE:
        emitDoubleMove(from, to);
        break;
      case MoveOp::INT32:
        emitInt32Move(from, to);
        break;
      case MoveOp::GENERAL:
        emitGeneralMove(from, to);
        break;
      default:
        MOZ_CRASH("Unexpected move type");
    }
}

void
MoveEmitterARM64::emitFloat32Move(const MoveOperand &from, const MoveOperand &to)
{
    if (from.isFloatReg()) {
        if (to.isFloatReg())
            masm.fmov(toFPReg(to, MoveOp::FLOAT32), toFPReg(from, MoveOp::FLOAT32));
        else
            masm.Str(toFPReg(from, MoveOp::FLOAT32), toMemOperand(to));
    } else {
        if (to.isFloatReg()) {
            masm.Ldr(toFPReg(to, MoveOp::FLOAT32), toMemOperand(from));
        } else {
            masm.Ldr(ScratchFloat32Reg_, toMemOperand(from));
            masm.Str(ScratchFloat32Reg_, toMemOperand(to));
        }
    }
}

void
MoveEmitterARM64::emitDoubleMove(const MoveOperand &from, const MoveOperand &to)
{
    if (from.isFloatReg()) {
        if (to.isFloatReg())
            masm.fmov(toFPReg(to, MoveOp::DOUBLE), toFPReg(from, MoveOp::DOUBLE));
        else
            masm.Str(toFPReg(from, MoveOp::DOUBLE), toMemOperand(to));
    } else {
        if (to.isFloatReg()) {
            masm.Ldr(toFPReg(to, MoveOp::DOUBLE), toMemOperand(from));
        } else {
            masm.ldr(ScratchDoubleReg_, toMemOperand(from));
            masm.Str(ScratchDoubleReg_, toMemOperand(to));
        }
    }
}

void
MoveEmitterARM64::emitInt32Move(const MoveOperand &from, const MoveOperand &to)
{
    if (from.isGeneralReg()) {
        if (to.isGeneralReg())
            masm.mov(toARMReg32(to), toARMReg32(from));
        else
            masm.Str(toARMReg32(from), toMemOperand(to));
    } else {
        if (to.isGeneralReg()) {
            masm.Ldr(toARMReg32(to), toMemOperand(from));
        } else {
            masm.ldr(ScratchReg2_32, toMemOperand(from));
            masm.Str(ScratchReg2_32, toMemOperand(to));
        }
    }
}

void
MoveEmitterARM64::emitGeneralMove(const MoveOperand &from, const MoveOperand &to)
{

    if (from.isGeneralReg()) {
        MOZ_ASSERT(to.isGeneralReg() || to.isMemory());

        if (to.isGeneralReg())
            masm.mov(toARMReg64(to), toARMReg64(from));
        else
            masm.Str(toARMReg64(from), toMemOperand(to));
        return;
    }

    // {Memory OR EffectiveAddress} -> Register move.
    if (to.isGeneralReg()) {
        MOZ_ASSERT(from.isMemoryOrEffectiveAddress());

        if (from.isMemory())
            masm.Ldr(toARMReg64(to), toMemOperand(from));
        else
            masm.Add(toARMReg64(to), ARMRegister(from.base(), 64), Operand(from.disp()));
        return;
    }

    // Memory -> Memory move.
    if (from.isMemory()) {
        MOZ_ASSERT(to.isMemory());
        masm.Ldr(ScratchReg2_64, toMemOperand(from));
        masm.Str(ScratchReg2_64, toMemOperand(to));
        return;
    }

    // EffectiveAddress -> Memory move.
    MOZ_ASSERT(from.isEffectiveAddress());
    MOZ_ASSERT(to.isMemory());
    masm.Add(ScratchReg2_64, ARMRegister(from.base(), 64), Operand(from.disp()));
    masm.Str(ScratchReg2_64, toMemOperand(to));
}

MemOperand
MoveEmitterARM64::cycleSlot()
{
    // Using SP as stack pointer requires alignment preservation below.
    MOZ_ASSERT(!masm.GetStackPointer64().Is(sp));

    // emit() already allocated a slot for resolving the cycle.
    MOZ_ASSERT(pushedAtCycle_ != -1);

    return MemOperand(masm.GetStackPointer64(), masm.framePushed() - pushedAtCycle_);
}

void
MoveEmitterARM64::breakCycle(const MoveOperand &from, const MoveOperand &to, MoveOp::Type type)
{
    // TODO: Uh, pretty sure cycle resolution should just use a temp register.
    // TODO: Stack seems pretty overkill.
    switch (type) {
      case MoveOp::FLOAT32:
        if (to.isMemory()) {
            ARMFPRegister temp(ScratchFloat32Reg, 32);
            masm.Ldr(temp, toMemOperand(to));
            masm.Str(temp, cycleSlot());
        } else {
            masm.Str(toFPReg(to, type), cycleSlot());
        }
        break;
      case MoveOp::DOUBLE:
        if (to.isMemory()) {
            ARMFPRegister temp(ScratchFloat32Reg, 32);
            masm.Ldr(temp, toMemOperand(to));
            masm.Str(temp, cycleSlot());
        } else {
            masm.Str(toFPReg(to, type), cycleSlot());
        }
        break;
      case MoveOp::INT32:
        if (to.isMemory()) {
            masm.Ldr(ScratchReg32, toMemOperand(to));
            masm.Str(ScratchReg32, cycleSlot());
        } else {
            masm.Str(ARMRegister(to.reg(), 32), cycleSlot());
        }
        break;
      case MoveOp::GENERAL:
        if (to.isMemory()) {
            masm.Ldr(ScratchReg64, toMemOperand(to));
            masm.Str(ScratchReg64, cycleSlot());
        } else {
            masm.Str(toARMReg64(to), cycleSlot());
        }
        break;
      case MoveOp::INT32X4: // TODO
        MOZ_CRASH("TODO");
        break;
      case MoveOp::FLOAT32X4: // TODO
        MOZ_CRASH("TODO");
        break;
      default:
        MOZ_CRASH("Unexpected move type");
    }
}

void
MoveEmitterARM64::completeCycle(const MoveOperand &from, const MoveOperand &to, MoveOp::Type type)
{
    switch (type) {
      case MoveOp::FLOAT32:
        if (to.isMemory()) {
            ARMFPRegister temp(ScratchFloat32Reg, 32);
            masm.Ldr(temp, cycleSlot());
            masm.Str(temp, toMemOperand(to));
        } else {
            masm.Ldr(ARMFPRegister(to.floatReg(), 32), cycleSlot());
        }
        break;
      case MoveOp::DOUBLE:
        if (to.isMemory()) {
            ARMFPRegister temp(ScratchDoubleReg, 64);
            masm.Ldr(temp, cycleSlot());
            masm.Str(temp, toMemOperand(to));
        } else {
            masm.Ldr(ARMFPRegister(to.floatReg(), 64), cycleSlot());
        }
        break;
      case MoveOp::INT32:
        if (to.isMemory()) {
            masm.Ldr(ARMRegister(ScratchReg, 32), cycleSlot());
            masm.Str(ARMRegister(ScratchReg, 32), toMemOperand(to));
        } else {
            masm.Ldr(ARMRegister(to.reg(), 32), cycleSlot());
        }
        break;
      case MoveOp::GENERAL:
        if (to.isMemory()) {
            masm.Ldr(ARMRegister(ScratchReg, 64), cycleSlot());
            masm.Str(ARMRegister(ScratchReg, 64), toMemOperand(to));
        } else {
            masm.Ldr(ARMRegister(to.reg(), 64), cycleSlot());
        }
        break;
      case MoveOp::INT32X4: // TODO
        MOZ_CRASH("TODO");
        break;
      case MoveOp::FLOAT32X4: // TODO
        MOZ_CRASH("TODO");
        break;
    }
}
