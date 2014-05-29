// -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
// vim: set ts=8 sts=2 et sw=2 tw=99:
//
// Copyright 2013, ARM Limited
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//   * Neither the name of ARM Limited nor the names of its contributors may be
//     used to endorse or promote products derived from this software without
//     specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef VIXL_GLOBALS_H
#define VIXL_GLOBALS_H

// Get standard C99 macros for integer types.
#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

#ifndef __STDC_LIMIT_MACROS
#define __STDC_LIMIT_MACROS
#endif

#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif

#include "js/Utility.h"
#include "mozilla/Assertions.h"

#include "jit/arm64/vixl/VIXL-Platform-vixl.h"

#include <stdint.h>
#include <inttypes.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>

typedef uint8_t byte;

const int KBytes = 1024;
const int MBytes = 1024 * KBytes;

#define VIXL_ABORT() printf("in %s, line %i", __FILE__, __LINE__); abort()

#define VIXL_ASSERT(condition) JS_ASSERT(condition)
#define VIXL_CHECK(condition) JS_ASSERT(condition)
#define VIXL_UNIMPLEMENTED() JS_ASSERT(!"VIXL Unimplemented")
#define VIXL_UNREACHABLE() MOZ_ASSUME_UNREACHABLE("VIXL Unreachable")
#define VIXL_STATIC_ASSERT(condition) JS_STATIC_ASSERT(condition)
#define VIXL_ALIGNMENT_EXCEPTION() printf("ALIGNMENT EXCEPTION\t"); VIXL_ABORT()

// Unfortunately, assembler/wtf/Platform.h defines USE() as a
// WTF feature-detection macro already. Renaming to USEARG().
template <typename T> inline void USEARG(T) {}

#endif  // VIXL_GLOBALS_H
