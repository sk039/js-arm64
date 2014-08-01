/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_arm64_MoveEmitter_arm64_h
#define jit_arm64_MoveEmitter_arm64_h

#include "jit/IonMacroAssembler.h"
#include "jit/MoveResolver.h"
#include "jit/arm64/Assembler-arm64.h"
namespace js {
namespace jit {

class CodeGenerator;

class MoveEmitterARM64
{
    bool inCycle_;
    MacroAssemblerCompat &masm;

    // Original stack push value.
    uint32_t pushedAtStart_;

    // These store stack offsets to spill locations, snapshotting
    // codegen->framePushed_ at the time they were allocated. They are -1 if no
    // stack space has been allocated for that particular spill.
    int32_t pushedAtCycle_;
    int32_t pushedAtSpill_;

    // These are registers that are available for temporary use. They may be
    // assigned InvalidReg. If no corresponding spill space has been assigned,
    // then these registers do not need to be spilled.
    Register spilledReg_;
    FloatRegister spilledFloatReg_;

    void assertDone() {
        JS_ASSERT(!inCycle_);
    }

    Register tempReg() {
        JS_ASSERT(0 && "tempReg()");
        return InvalidReg;
    }
    FloatRegister tempFloatReg() {
        JS_ASSERT(0 && "tempFloatReg()");
        return InvalidFloatReg;
    }
    MemOperand cycleSlot() const {
        MOZ_ASSUME_UNREACHABLE("cycleSlot()");
    }
    Operand spillSlot() const {
        MOZ_ASSUME_UNREACHABLE("spillSlot()");
    }
    MemOperand toMemOperand(const MoveOperand &operand) const {
        MOZ_ASSUME_UNREACHABLE("toOperand()");
    }
    CPURegister toRegister(const MoveOperand &operand) const {
        MOZ_ASSUME_UNREACHABLE("toOperand()");
    }
    void emitMove(const MoveOperand &from, const MoveOperand &to) {
        JS_ASSERT(0 && "emitMove()");
    }
    void emitFloat32Move(const MoveOperand &from, const MoveOperand &to) {
        JS_ASSERT(0 && "emitFloat32Move()");
    }
    void emitDoubleMove(const MoveOperand &from, const MoveOperand &to) {
        JS_ASSERT(0 && "emitDoubleMove()");
    }
    void breakCycle(const MoveOperand &from, const MoveOperand &to, MoveOp::Type type) {
        switch (type) {
          case MoveOp::FLOAT32:
            if (to.isMemory()) {
                ARMFPRegister temp(ScratchFloat32Reg, 32);
                masm.Ldr(temp, toMemOperand(to));
                masm.Str(temp, cycleSlot());
            } else {
                masm.Str(toRegister(to), cycleSlot());
            }
            break;
          case MoveOp::DOUBLE:
            if (to.isMemory()) {
                ARMFPRegister temp(ScratchFloat32Reg, 32);
                masm.Ldr(temp, toMemOperand(to));
                masm.Str(temp, cycleSlot());
            } else {
                masm.Str(toRegister(to), cycleSlot());
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
    void completeCycle(const MoveOperand &from, const MoveOperand &to, MoveOp::Type type) {
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
    void emit(const MoveOp &move) {
        const MoveOperand &from = move.from();
        const MoveOperand &to = move.to();
        if (move.isCycleEnd()) {
            MOZ_ASSERT(!move.isCycleBegin());
            MOZ_ASSERT(inCycle_);
            completeCycle(from, to, move.type());
            inCycle_ = false;
        }
        if (move.isCycleEnd()) {
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
          case MoveOp::GENERAL:
            emitMove(from, to);
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("Unexpected move type");
        }
    }

  public:
    MoveEmitterARM64(MacroAssemblerCompat &masm)
      : inCycle_(false),
        masm(masm),
        pushedAtCycle_(-1),
        pushedAtSpill_(-1),
        spilledReg_(InvalidReg),
        spilledFloatReg_(InvalidFloatReg)
    {
        pushedAtStart_ = masm.framePushed();
    }

    ~MoveEmitterARM64() {
        assertDone();
    }
    void emit(const MoveResolver &moves) {
        if (moves.numCycles()) {
            masm.reserveStack(sizeof(double));
            pushedAtCycle_ = masm.framePushed();
        }
        for (size_t i = 0; i < moves.numMoves(); i++)
            emit(moves.getMove(i));
    }
    void finish() {
        masm.freeStack(masm.framePushed() - pushedAtStart_);
    }
};

typedef MoveEmitterARM64 MoveEmitter;

} // namespace jit
} // namespace js

#endif /* jit_arm64_MoveEmitter_arm64_h */
