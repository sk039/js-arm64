// -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
// vim: set ts=8 sts=4 et sw=4 tw=99:
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

#ifndef VIXL_A64_DEBUGGER_A64_H_
#define VIXL_A64_DEBUGGER_A64_H_

#include "js-config.h" // For JS_ARM64_SIMULATOR.

#include "jit/arm64/vixl/Constants-vixl.h"
#include "jit/arm64/vixl/Simulator-vixl.h"
#include "jit/arm64/vixl/VIXL-Globals-vixl.h"
#include "jit/arm64/vixl/VIXL-Utils-vixl.h"

#ifdef JS_ARM64_SIMULATOR

namespace js {
namespace jit {

// Debug instructions.
//
// VIXL's macro-assembler and debugger support a few pseudo instructions to
// make debugging easier. These pseudo instructions do not exist on real
// hardware.
//
// Each debug pseudo instruction is represented by a HLT instruction. The HLT
// immediate field is used to identify the type of debug pseudo isntruction.
// Each pseudo instruction use a custom encoding for additional arguments, as
// described below.

// Unreachable
//
// Instruction which should never be executed. This is used as a guard in parts
// of the code that should not be reachable, such as in data encoded inline in
// the instructions.
const Instr kUnreachableOpcode = 0xdeb0;

// Trace
//  - parameter: TraceParameter stored as a uint32_t
//  - command: TraceCommand stored as a uint32_t
//
// Allow for trace management in the generated code. See the corresponding
// enums for more information on permitted actions.
const Instr kTraceOpcode = 0xdeb2;
const unsigned kTraceParamsOffset = 1 * kInstructionSize;
const unsigned kTraceCommandOffset = 2 * kInstructionSize;
const unsigned kTraceLength = 3 * kInstructionSize;

// Log
//  - parameter: TraceParameter stored as a uint32_t
//
// Output the requested information.
const Instr kLogOpcode = 0xdeb3;
const unsigned kLogParamsOffset = 1 * kInstructionSize;
const unsigned kLogLength = 2 * kInstructionSize;

const Instr kInvariantOpcode = 0xdeb4;
const unsigned kInvariantCommandOffset = 1 * kInstructionSize;
const unsigned kInvariantLength = 2 * kInstructionSize;

const Instr kStackCheckOpcode = 0xdeb5;
const unsigned kStackCheckIndexOffset = 1 * kInstructionSize;
const unsigned kStackCheckValueOffset = 2 * kInstructionSize;
const unsigned kStackCheckLength = 3 * kInstructionSize;

const Instr kStackCheckPushPopOpcode = 0xdeb6;
const unsigned kStackCheckPushPopIndexOffset = 1 * kInstructionSize;
const unsigned kStackCheckPushPopValueOffset = 2 * kInstructionSize;
const unsigned kStackCheckPushPopDirOffset = 3 * kInstructionSize;
const unsigned kStackCheckPushPopLength = 4 * kInstructionSize;

// Trace commands.
enum TraceCommand {
    TRACE_ENABLE   = 1,
    TRACE_DISABLE  = 2
};

// Trace parameters.
enum TraceParameters {
    LOG_DISASM     = 1 << 0,  // Log disassembly.
    LOG_REGS       = 1 << 1,  // Log general purpose registers.
    LOG_FP_REGS    = 1 << 2,  // Log floating-point registers.
    LOG_SYS_REGS   = 1 << 3,  // Log the flags and system registers.

    LOG_STATE      = LOG_REGS | LOG_FP_REGS | LOG_SYS_REGS,
    LOG_ALL        = LOG_DISASM | LOG_REGS | LOG_FP_REGS | LOG_SYS_REGS
};

// Debugger parameters
enum DebugParameters {
    DBG_ACTIVE = 1 << 0,  // The debugger is active.
    DBG_BREAK  = 1 << 1   // The debugger is at a breakpoint.
};

// Forward declarations.
class DebugCommand;
class Token;
class FormatToken;

class DebuggerARM64 : public Simulator
{
  public:
    DebuggerARM64(Decoder* decoder, FILE* stream = stdout);
    virtual void Run();
    void VisitException(Instruction* instr);
    void VisitUnallocated(Instruction *instr);
    inline int log_parameters() {
        // The simulator can control disassembly, so make sure that the Debugger's
        // log parameters agree with it.
        if (disasm_trace())
            log_parameters_ |= LOG_DISASM;
        return log_parameters_;
    }
    inline void set_log_parameters(int parameters) {
        set_disasm_trace((parameters & LOG_DISASM) != 0);
        log_parameters_ = parameters;

        update_pending_request();
    }

    inline int debug_parameters() { return debug_parameters_; }
    inline void set_debug_parameters(int parameters) {
        debug_parameters_ = parameters;
        update_pending_request();
    }

    virtual void enable_debugger() {
        set_debug_parameters(DBG_ACTIVE);
    }

    // Numbers of instructions to execute before the debugger shell is given
    // back control.
    inline int steps() { return steps_; }
    inline void set_steps(int value) {
        MOZ_ASSERT(value > 1);
        steps_ = value;
    }

    inline bool IsDebuggerRunning() {
        return (debug_parameters_ & DBG_ACTIVE) != 0;
    }

    inline bool pending_request() { return pending_request_; }
    inline void update_pending_request() {
        const int kLoggingMask = LOG_STATE;
        const bool logging = (log_parameters_ & kLoggingMask) != 0;
        const bool debugging = IsDebuggerRunning();

        pending_request_ = logging || debugging;
    }

    void PrintInstructions(void* address, int64_t count = 1);
    void PrintMemory(const uint8_t* address, const FormatToken* format, int64_t count = 1);
    void PrintRegister(const ARMRegister& target_reg, const char* name, const FormatToken* format);
    void PrintFloatRegister(const ARMFPRegister& target_fpreg, const FormatToken* format);

  private:
    void LogSystemRegisters();
    void LogRegisters();
    void LogFloatRegisters();
    void LogProcessorState();
    char* ReadCommandLine(const char* prompt, char* buffer, int length);
    void RunDebuggerShell();
    void DoBreakpoint(Instruction* instr);
    void DoUnreachable(Instruction* instr);
    void DoTrace(Instruction* instr);
    void DoLog(Instruction* instr);
    void DoStackCheck(Instruction *instr);
    void DoStackCheckPushPop(Instruction *instr);
    static const int numStacks = 5;
    char *stackCheck[numStacks][1024];
    int stackCheckDepth[numStacks];

    int  log_parameters_;
    int  debug_parameters_;
    bool pending_request_;
    int  steps_;
    DebugCommand* last_command_;
    PrintDisassembler* disasm_;
    Decoder* printer_;

    // Length of the biggest command line accepted by the debugger shell.
    static const int kMaxDebugShellLine = 256;
};

} // namespace jit
} // namespace js
static void printInstruction(void *inst, int count) {
    js::jit::Decoder d;
    js::jit::DebuggerARM64 dbg(&d);
    dbg.PrintInstructions(inst, count);
}
#endif  // JS_ARM64_SIMULATOR
#endif  // VIXL_A64_DEBUGGER_A64_H_
