/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/BaselineHelpers.h"
#include "jit/BaselineIC.h"

using namespace js;
using namespace js::jit;

namespace js {
namespace jit {

// ICCompare_Int32

bool
ICCompare_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    MOZ_ASSUME_UNREACHABLE("ICCompare_Int32 generateStubCode");
}

bool
ICCompare_Double::Compiler::generateStubCode(MacroAssembler &masm)
{
    MOZ_ASSUME_UNREACHABLE("ICCompare_Double generateStubCode");
}

// ICBinaryArith_Int32

bool
ICBinaryArith_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    MOZ_ASSUME_UNREACHABLE("ICBinaryArith_Int32 generateStubCode");
}

bool
ICUnaryArith_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    MOZ_ASSUME_UNREACHABLE("ICUnaryArith_Int32 generateStubCode");
}

} // namespace jit
} // namespace js
