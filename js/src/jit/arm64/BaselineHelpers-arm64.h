/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_arm64_BaselineHelpers_arm64_h
#define jit_arm64_BaselineHelpers_arm64_h

#include "jit/BaselineFrame.h"
#include "jit/BaselineIC.h"
#include "jit/BaselineRegisters.h"
#include "jit/MacroAssembler.h"

namespace js {
namespace jit {

// Distance from sp to the top Value inside an IC stub (no return address on the stack on ARM).
static const size_t ICStackValueOffset = 0;

inline void
EmitRestoreTailCallReg(MacroAssembler &masm)
{
    // No-op on ARM because link register is always holding the return address.
}

inline void
EmitRepushTailCallReg(MacroAssembler &masm)
{
    // No-op on ARM because link register is always holding the return address.
}

inline void
EmitCallIC(CodeOffsetLabel *patchOffset, MacroAssembler &masm)
{
    // Move ICEntry offset into BaselineStubReg
    CodeOffsetLabel offset = masm.movWithPatch(ImmWord(-1), BaselineStubReg);
    *patchOffset = offset;

    // Load stub pointer into BaselineStubReg
    masm.loadPtr(Address(BaselineStubReg, ICEntry::offsetOfFirstStub()), BaselineStubReg);

    // Load stubcode pointer from BaselineStubEntry.
    // R2 won't be active when we call ICs, so we can use r0.
    MOZ_ASSERT(R2 == ValueOperand(r0));
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r0);

    // Call the stubcode via a direct branch-and-link.
    masm.Blr(x0);
}

inline void
EmitEnterTypeMonitorIC(MacroAssembler &masm,
                       size_t monitorStubOffset = ICMonitoredStub::offsetOfFirstMonitorStub())
{
    // This is expected to be called from within an IC, when BaselineStubReg is
    // properly initialized to point to the stub.
    masm.loadPtr(Address(BaselineStubReg, (uint32_t) monitorStubOffset), BaselineStubReg);

    // Load stubcode pointer from BaselineStubEntry.
    // R2 won't be active when we call ICs, so we can use r0.
    MOZ_ASSERT(R2 == ValueOperand(r0));
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r0);

    // Jump to the stubcode.
    masm.Br(x0);
}

inline void
EmitReturnFromIC(MacroAssembler &masm)
{
    masm.ret(); // Defaults to lr.
}

inline void
EmitChangeICReturnAddress(MacroAssembler &masm, Register reg)
{
    masm.movePtr(reg, lr);
}

inline void
EmitTailCallVM(JitCode *target, MacroAssembler &masm, uint32_t argSize)
{
    // We assume that R0 has been pushed, and R2 is unused.
    MOZ_ASSERT(R2 == ValueOperand(r0));

    // Compute frame size into w0. Used below in makeFrameDescriptor().
    masm.Sub(x0, ARMRegister(BaselineFrameReg, 64), masm.GetStackPointer64());
    masm.Add(w0, w0, Operand(BaselineFrame::FramePointerOffset));

    // Store frame size without VMFunction arguments for GC marking.
    masm.Sub(ScratchReg2_32, w0, Operand(argSize));
    masm.store32(ScratchReg2, Address(BaselineFrameReg, BaselineFrame::reverseOffsetOfFrameSize()));

    // Push frame descriptor (minus the return address) and perform the tail call.
    MOZ_ASSERT(BaselineTailCallReg == lr);
    masm.makeFrameDescriptor(r0, JitFrame_BaselineJS);
    masm.MacroAssemblerVIXL::Push(x0);

    // The return address will be pushed by the VM wrapper, for compatibility
    // with direct calls. Refer to the top of generateVMWrapper().
    // BaselineTailCallReg (lr) already contains the return address (as we keep
    // it there through the stub calls).

    masm.branch(target);
}

inline void
EmitCreateStubFrameDescriptor(MacroAssembler &masm, Register reg)
{
    ARMRegister reg64(reg, 64);

    // Compute stub frame size. We have to add two pointers: the stub reg and previous
    // frame pointer pushed by EmitEnterStubFrame.
    masm.Add(reg64, ARMRegister(BaselineFrameReg, 64), Operand(sizeof(void *) * 2));
    masm.Sub(reg64, reg64, masm.GetStackPointer64());

    masm.makeFrameDescriptor(reg, JitFrame_BaselineStub);
}

inline void
EmitCallVM(JitCode *target, MacroAssembler &masm)
{
    EmitCreateStubFrameDescriptor(masm, r0);
    masm.push(r0);
    masm.call(target);
}

// Size of values pushed by EmitEnterStubFrame.
static const uint32_t STUB_FRAME_SIZE = 4 * sizeof(void *);
static const uint32_t STUB_FRAME_SAVED_STUB_OFFSET = sizeof(void *);

inline void
EmitEnterStubFrame(MacroAssembler &masm, Register scratch)
{
    MOZ_ASSERT(scratch != BaselineTailCallReg);

    // Compute frame size.
    masm.movePtr(BaselineFrameReg, scratch);
    masm.addPtr(Imm32(BaselineFrame::FramePointerOffset), scratch);
    masm.Sub(ARMRegister(scratch, 64), ARMRegister(scratch, 64), masm.GetStackPointer64());

    masm.store32(scratch, Address(BaselineFrameReg, BaselineFrame::reverseOffsetOfFrameSize()));

    // Note: when making changes here, don't forget to update STUB_FRAME_SIZE
    // if needed.

    // Push frame descriptor and return address.
    // Save old frame pointer, stack pointer, and stub reg.
    masm.makeFrameDescriptor(scratch, JitFrame_BaselineJS);
    masm.MacroAssemblerVIXL::Push(ARMRegister(scratch, 64),
                                  ARMRegister(BaselineTailCallReg, 64),
                                  ARMRegister(BaselineStubReg, 64),
                                  ARMRegister(BaselineFrameReg, 64));

    // Update the frame register.
    masm.Add(ARMRegister(BaselineFrameReg, 64), masm.GetStackPointer64(), Operand(0));

    // Stack should remain 16-byte aligned.
    masm.checkStackAlignment();
}

inline void
EmitLeaveStubFrame(MacroAssembler &masm, bool calledIntoIon = false)
{
    // Ion frames do not save and restore the frame pointer. If we called
    // into Ion, we have to restore the stack pointer from the frame descriptor.
    // If we performed a VM call, the descriptor has been popped already so
    // in that case we use the frame pointer.
    if (calledIntoIon) {
        masm.pop(ScratchReg);
        masm.Lsr(ScratchReg64, ScratchReg64, FRAMESIZE_SHIFT);
        masm.Add(masm.GetStackPointer64(), masm.GetStackPointer64(), ScratchReg64);
    } else {
        masm.Add(masm.GetStackPointer64(), ARMRegister(BaselineFrameReg, 64), Operand(0));
    }

    // Pop values, discarding the frame descriptor.
    masm.MacroAssemblerVIXL::Pop(ARMRegister(BaselineFrameReg, 64),
                                 ARMRegister(BaselineStubReg, 64),
                                 ARMRegister(BaselineTailCallReg, 64),
                                 ScratchReg64);

    // Stack should remain 16-byte aligned.
    masm.checkStackAlignment();
}

inline void
EmitStowICValues(MacroAssembler &masm, int values)
{
    MOZ_ASSERT(values >= 0 && values <= 2);
    switch (values) {
      case 1:
        // Stow R0.
        masm.pushValue(R0);
        break;
      case 2:
        // Stow R0 and R1.
        masm.pushValue(R0);
        masm.pushValue(R1); // TODO: Use MacroAssemblerVIXL::Push().
        break;
    }
}

inline void
EmitUnstowICValues(MacroAssembler &masm, int values, bool discard = false)
{
    MOZ_ASSERT(values >= 0 && values <= 2);
    switch (values) {
      case 1:
        // Unstow R0.
        if (discard)
            masm.addPtr(Imm32(sizeof(Value)), BaselineStackReg);
        else
            masm.popValue(R0);
        break;
      case 2:
        // Unstow R0 and R1.
        if (discard) {
            masm.addPtr(Imm32(sizeof(Value) * 2), BaselineStackReg);
        } else {
            masm.popValue(R1);
            masm.popValue(R0); // TODO: Use MacroAssemblerVIXL::Pop().
        }
    }
}

inline void
EmitCallTypeUpdateIC(MacroAssembler &masm, JitCode *code, uint32_t objectOffset)
{
    // R0 contains the value that needs to be typechecked.
    // The object we're updating is a boxed Value on the stack, at offset
    // objectOffset from stack top, excluding the return address.
    MOZ_ASSERT(R2 == ValueOperand(r0));

    // Save the current BaselineStubReg to stack, as well as the TailCallReg,
    // since on AArch64, the LR is live.
    masm.MacroAssemblerVIXL::Push(ARMRegister(BaselineStubReg, 64),
                                  ARMRegister(BaselineTailCallReg, 64));

    // This is expected to be called from within an IC, when BaselineStubReg
    // is properly initialized to point to the stub.
    masm.loadPtr(Address(BaselineStubReg, (int32_t)ICUpdatedStub::offsetOfFirstUpdateStub()),
                 BaselineStubReg);

    // Load stubcode pointer from BaselineStubReg into BaselineTailCallReg.
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), BaselineTailCallReg);

    // Call the stubcode.
    masm.Blr(ARMRegister(BaselineTailCallReg, 64));

    // Restore the old stub reg and tailcall reg.
    masm.MacroAssemblerVIXL::Pop(ARMRegister(BaselineTailCallReg, 64),
                                 ARMRegister(BaselineStubReg, 64));

    // The update IC will store 0 or 1 in R1.scratchReg() reflecting if the
    // value in R0 type-checked properly or not.
    Label success;
    masm.cmp32(R1.scratchReg(), Imm32(1));
    masm.j(Assembler::Equal, &success);

    // If the IC failed, then call the update fallback function.
    EmitEnterStubFrame(masm, R1.scratchReg());

    masm.loadValue(Address(BaselineStackReg, STUB_FRAME_SIZE + objectOffset), R1);

    masm.pushValue(R0);
    masm.pushValue(R1);
    masm.push(BaselineStubReg); // TODO: Use MacroAssemblerVIXL::Push().

    // Load previous frame pointer, push BaselineFrame*.
    masm.loadPtr(Address(BaselineFrameReg, 0), R0.scratchReg());
    masm.pushBaselineFramePtr(R0.scratchReg(), R0.scratchReg());

    EmitCallVM(code, masm);
    EmitLeaveStubFrame(masm);

    // Success at end.
    masm.bind(&success);
}

template <typename AddrType>
inline void
EmitPreBarrier(MacroAssembler &masm, const AddrType &addr, MIRType type)
{
    // On AArch64, lr is clobbered by patchableCallPreBarrier. Save it first.
    masm.push(lr);
    masm.patchableCallPreBarrier(addr, type);
    masm.pop(lr);
}

inline void
EmitStubGuardFailure(MacroAssembler &masm)
{
    // NOTE: This routine assumes that the stub guard code left the stack in the
    // same state it was in when it was entered.

    // BaselineStubEntry points to the current stub.

    // Load next stub into BaselineStubReg.
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfNext()), BaselineStubReg);

    // Load stubcode pointer from BaselineStubEntry into scratch register.
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r0);

    // Return address is already loaded, just jump to the next stubcode.
    masm.Br(x0);
}

} // namespace jit
} // namespace js

#endif // jit_arm64_BaselineHelpers_arm64_h
