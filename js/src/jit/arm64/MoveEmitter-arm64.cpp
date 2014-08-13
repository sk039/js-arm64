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
        masm.reserveStack(sizeof(double));
        pushedAtCycle_ = masm.framePushed();
    }

    // TODO: Optimize cycles.
    for (size_t i = 0; i < moves.numMoves(); i++)
        emitMove(moves.getMove(i));
}

void
MoveEmitterARM64::finish()
{
    masm.freeStack(masm.framePushed() - pushedAtStart_);
}

void
MoveEmitterARM64::emitMove(const MoveOp &move)
{
    const MoveOperand &from = move.from();
    const MoveOperand &to = move.to();

    if (move.isCycleEnd()) {
        JS_ASSERT(inCycle_);
        completeCycle(from, to, move.type());
        inCycle_ = false;
    }

    // TODO: MoveEmitterX86 has logic to attempt to avoid using the stack.
    if (move.isCycleBegin()) {
        JS_ASSERT(!inCycle_);
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
        MOZ_ASSUME_UNREACHABLE("Unexpected move type");
    }
}

void
MoveEmitterARM64::emitFloat32Move(const MoveOperand &from, const MoveOperand &to)
{
    // TODO: Fix.
    masm.fmov(toFPReg(to), toFPReg(from));
}

void
MoveEmitterARM64::emitDoubleMove(const MoveOperand &from, const MoveOperand &to)
{
    // TODO: Fix.
    masm.fmov(toFPReg(to), toFPReg(from));
}

void
MoveEmitterARM64::emitInt32Move(const MoveOperand &from, const MoveOperand &to)
{
    // TODO: Fix.
    masm.mov(toARMReg64(from), toARMReg64(to));
}

void
MoveEmitterARM64::emitGeneralMove(const MoveOperand &from, const MoveOperand &to)
{
    // Register -> {Register OR Memory OR EffectiveAddress} move.
    if (from.isGeneralReg()) {
        JS_ASSERT(to.isGeneralReg() || to.isMemoryOrEffectiveAddress());

        // TODO: HOO BOY ACTUALLY IMPLEMENT THIS
        JS_ASSERT(0 && "from.isGeneralReg() case of emitGeneralMove()");
        return;
    }

    // {Memory OR EffectiveAddress} -> Register move.
    if (to.isGeneralReg()) {
        JS_ASSERT(from.isMemoryOrEffectiveAddress());

        // TODO: HOO BOY ACTUALLY IMPLEMENT THIS
        JS_ASSERT(0 && "to.isGeneralReg() case of emitGeneralMove()");
        return;
    }

    // Memory -> {Memory OR EffectiveAddress} move.
    if (from.isMemory()) {
        JS_ASSERT(to.isMemoryOrEffectiveAddress());

        // TODO: HOO BOY ACTUALLY IMPLEMENT THIS
        JS_ASSERT(0 && "from.isMemory() case of emitGeneralMove()");
        return;
    }

    // EffectiveAddress -> {Memory OR EffectiveAddress} move.
    JS_ASSERT(from.isEffectiveAddress());
    JS_ASSERT(to.isMemoryOrEffectiveAddress());

    // TODO: HOO BOY ACTUALLY IMPLEMENT THIS
    JS_ASSERT(0 && "from.isEffectiveAddress() case of emitGeneralMove()");
}

MemOperand
MoveEmitterARM64::cycleSlot()
{
    if (pushedAtCycle_ == -1) {
        // Reserve stack for cycle resolution, preserving 16-byte alignment.
        masm.reserveStack(sizeof(void *) * 2);
        pushedAtCycle_ = masm.framePushed();
    }

    return MemOperand(masm.GetStackPointer(), masm.framePushed() - pushedAtCycle_);
}

void
MoveEmitterARM64::breakCycle(const MoveOperand &from, const MoveOperand &to, MoveOp::Type type)
{
    switch (type) {
      case MoveOp::FLOAT32:
        if (to.isMemory()) {
            ARMFPRegister temp(ScratchFloat32Reg, 32);
            masm.Ldr(temp, toMemOperand(to));
            masm.Str(temp, cycleSlot());
        } else {
            masm.Str(toARMReg64(to), cycleSlot());
        }
        break;
      case MoveOp::DOUBLE:
        if (to.isMemory()) {
            ARMFPRegister temp(ScratchFloat32Reg, 32);
            masm.Ldr(temp, toMemOperand(to));
            masm.Str(temp, cycleSlot());
        } else {
            masm.Str(toARMReg64(to), cycleSlot());
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
            masm.Ldr(ScratchReg32, toMemOperand(to));
            masm.push(ScratchReg);
        } else {
            masm.push(to.reg());
        }
        break;
      default:
        MOZ_ASSUME_UNREACHABLE("Unexpected move type");
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
    }
}
