/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_arm64_MoveEmitter_arm64_h
#define jit_arm64_MoveEmitter_arm64_h

#include "jit/IonMacroAssembler.h"
#include "jit/MoveResolver.h"

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
    Operand cycleSlot() const {
        MOZ_ASSUME_UNREACHABLE("cycleSlot()");
    }
    Operand spillSlot() const {
        MOZ_ASSUME_UNREACHABLE("spillSlot()");
    }
    Operand toOperand(const MoveOperand &operand, bool isFloat) const {
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
        JS_ASSERT(0 && "breakCycle()");
    }
    void completeCycle(const MoveOperand &from, const MoveOperand &to, MoveOp::Type type) {
        JS_ASSERT(0 && "completeCycle()");
    }
    void emit(const MoveOp &move) {
        JS_ASSERT(0 && "emit()");
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
            JS_ASSERT(0 && "emit()");
    }
    void finish() {
            JS_ASSERT(0 && "finish()");
    }
};

typedef MoveEmitterARM64 MoveEmitter;

} // namespace jit
} // namespace js

#endif /* jit_arm64_MoveEmitter_arm64_h */
