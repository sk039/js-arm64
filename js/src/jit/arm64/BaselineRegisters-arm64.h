/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_arm64_BaselineRegisters_arm64_h
#define jit_arm64_BaselineRegisters_arm64_h

#ifdef JS_ION

#include "jit/IonMacroAssembler.h"

namespace js {
namespace jit {

static MOZ_CONSTEXPR_VAR Register BaselineFrameReg {Registers::fp};
static MOZ_CONSTEXPR_VAR Register BaselineStackReg {Registers::sp};

// ValueOperands R0, R1, and R2.
// R0 == JSReturnReg, and R2 uses registers not
// preserved across calls.  R1 value should be
// preserved across calls.
static MOZ_CONSTEXPR_VAR Register R0_{Registers::x1};
static MOZ_CONSTEXPR_VAR Register R1_{Registers::x2};
static MOZ_CONSTEXPR_VAR Register R2_{Registers::x0};
static MOZ_CONSTEXPR_VAR ValueOperand R0(R0_);
static MOZ_CONSTEXPR_VAR ValueOperand R1(R1_);
static MOZ_CONSTEXPR_VAR ValueOperand R2(R2_);

// BaselineTailCallReg and BaselineStubReg
// These use registers that are not preserved across
// calls.
static MOZ_CONSTEXPR_VAR Register BaselineTailCallReg {Registers::x30};
static MOZ_CONSTEXPR_VAR Register BaselineStubReg     {Registers::x9};

static MOZ_CONSTEXPR_VAR Register ExtractTemp0        {Registers::x3 };
static MOZ_CONSTEXPR_VAR Register ExtractTemp1        {Registers::x4 };

// Register used internally by MacroAssemblerARM.
static MOZ_CONSTEXPR_VAR Register BaselineSecondScratchReg {Registers::x6};

// R7 - R9 are generally available for use within stubcode.

// Note that BaselineTailCallReg is actually just the link
// register.  In ARM code emission, we do not clobber BaselineTailCallReg
// since we keep the return address for calls there.

// FloatReg0 must be equal to ReturnFloatReg.
static MOZ_CONSTEXPR_VAR FloatRegister FloatReg0   {FloatRegisters::v0};
static MOZ_CONSTEXPR_VAR FloatRegister FloatReg1   {FloatRegisters::v1};

} // namespace jit
} // namespace js

#endif // JS_ION

#endif /* jit_arm64_BaselineRegisters_arm64_h */
