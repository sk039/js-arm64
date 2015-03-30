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

#include "jit/arm64/vixl/Simulator-vixl.h"

#include <string.h>

#include "mozilla/DebugOnly.h"
#include "mozilla/FloatingPoint.h"

#include "jit/arm64/vixl/Assembler-vixl.h"
#include "jit/arm64/vixl/Debugger-vixl.h"
#include "jit/arm64/vixl/VIXL-Platform-vixl.h"

#include "vm/Runtime.h"

using mozilla::DebugOnly;

namespace js {
namespace jit {
// Protects the icache() and redirection() properties of the
// Simulator.
class AutoLockSimulatorCache
{
  public:
    explicit AutoLockSimulatorCache(Simulator *sim) : sim_(sim) {
        PR_Lock(sim_->lock_);
        MOZ_ASSERT(!sim_->lockOwner_);
#ifdef DEBUG
        sim_->lockOwner_ = PR_GetCurrentThread();
#endif
    }

    ~AutoLockSimulatorCache() {
        MOZ_ASSERT(sim_->lockOwner_);
#ifdef DEBUG
        sim_->lockOwner_ = nullptr;
#endif
        PR_Unlock(sim_->lock_);
    }

  private:
    Simulator *const sim_;
};


const Instruction* Simulator::kEndOfSimAddress = nullptr;

void
SimSystemRegister::SetBits(int msb, int lsb, uint32_t bits)
{
    int width = msb - lsb + 1;
    MOZ_ASSERT(is_uintn(width, bits) || is_intn(width, bits));

    bits <<= lsb;
    uint32_t mask = ((1 << width) - 1) << lsb;
    MOZ_ASSERT((mask & write_ignore_mask_) == 0);

    value_ = (value_ & ~mask) | (bits & mask);
}

SimSystemRegister
SimSystemRegister::DefaultValueFor(SystemRegister id)
{
    switch (id) {
      case NZCV:
        return SimSystemRegister(0x00000000, NZCVWriteIgnoreMask);
      case FPCR:
        return SimSystemRegister(0x00000000, FPCRWriteIgnoreMask);
      default:
        VIXL_UNREACHABLE();
        return SimSystemRegister();
    }
}

Simulator::Simulator() : lock_(nullptr), lockOwner_(nullptr)
{
    decoder_ = js_new<Decoder>();
    if (!decoder_) {
        MOZ_ReportAssertionFailure("[unhandlable oom] Decoder", __FILE__, __LINE__);
        MOZ_CRASH();
    }

    // FIXME: This just leaks the Decoder object for now, which is probably OK.
    // FIXME: We should free it at some point.
    // FIXME: Note that it can't be stored in the SimulatorRuntime due to lifetime conflicts.
    this->init(decoder_, stdout);
}

Simulator::Simulator(Decoder* decoder, FILE* stream) : lock_(nullptr), lockOwner_(nullptr)
{
    this->init(decoder, stream);
}

void
Simulator::init(Decoder* decoder, FILE* stream)
{
    // Ensure that shift operations act as the simulator expects.
    MOZ_ASSERT((static_cast<int32_t>(-1) >> 1) == -1);
    MOZ_ASSERT((static_cast<uint32_t>(-1) >> 1) == 0x7FFFFFFF);

    // Set up the decoder.
    decoder_ = decoder;
    decoder_->AppendVisitor(this);

    ResetState();

    stream_ = stdout;

    // Allocate and set up the simulator stack.
    stack_ = reinterpret_cast<byte*>(js_malloc(stack_size_));
    if (!stack_) {
        MOZ_ReportAssertionFailure("[unhandlable oom] Simulator stack", __FILE__, __LINE__);
        MOZ_CRASH();
    }

    stack_limit_ = stack_ + stack_protection_size_;
    // Configure the starting stack pointer.
    //  - Find the top of the stack.
    byte * tos = stack_ + stack_size_;
    //  - There's a protection region at both ends of the stack.
    tos -= stack_protection_size_;
    //  - The stack pointer must be 16-byte aligned.
    tos = AlignDown(tos, 16);
    set_sp(tos);

    void *printDisasmMem = js_malloc(sizeof(PrintDisassembler));
    if (!printDisasmMem) {
        MOZ_ReportAssertionFailure("[unhandlable oom] Simulator PrintDisassembler", __FILE__, __LINE__);
        MOZ_CRASH();
    }
    print_disasm_ = new(printDisasmMem) PrintDisassembler(stream);

    void *instrumentMem = js_malloc(sizeof(Instrument));
    if (!instrumentMem) {
        MOZ_ReportAssertionFailure("[unhandlable oom] Simulator Instrumentation", __FILE__, __LINE__);
        MOZ_CRASH();
    }
    // FIXME: Set the sample period to 10, as the VIXL examples and tests are short.
    instrumentation_ = new(instrumentMem) Instrument("vixl_stats.csv", 10);

    // Print a warning about exclusive-access instructions, but only the first
    // time they are encountered. This warning can be silenced using
    // SilenceExclusiveAccessWarning().
    print_exclusive_access_warning_ = true;

    set_coloured_trace(false);
    disasm_trace_ = false;

    lock_ = PR_NewLock();
    if (!lock_)
        MOZ_CRASH("Could not allocate lock");
    redirection_ = nullptr;
}

Simulator *
Simulator::Current()
{
    return TlsPerThreadData.get()->simulator();
#if 0
    PerThreadData *pt = TlsPerThreadData.get();
    Simulator *sim = pt->simulator();
    if (!sim) {
        // TODO: We should always use the simulator instead of the debugger.
        // TODO: Maybe have a more surfaceable toggle for the shell.
        if (getenv("USE_DEBUGGER") != nullptr) {
            Decoder *decoder = js_new<Decoder>();
            DebuggerARM64 *debugger = js_new<DebuggerARM64>(decoder, stdout);
            debugger->set_log_parameters(LOG_DISASM | LOG_REGS);
            pt->setSimulator(debugger);
            return debugger;
        }

        // FIXME: SimulatorRuntime separation seems wonky. Do we need a new one?
        SimulatorRuntime *srt = CreateSimulatorRuntime();
        if (!srt) {
            MOZ_ReportAssertionFailure("[unhandlable oom] SimulatorRuntime creation", __FILE__, __LINE__);
            MOZ_CRASH();
        }

        sim = js_new<Simulator>(srt);
        pt->setSimulator(sim);
    }

    return sim;
#endif
}

void
Simulator::ResetState()
{
    // Reset the system registers.
    nzcv_ = SimSystemRegister::DefaultValueFor(NZCV);
    fpcr_ = SimSystemRegister::DefaultValueFor(FPCR);

    // Reset registers to 0.
    pc_ = nullptr;
    pc_modified_ = false;

    for (unsigned i = 0; i < kNumberOfRegisters; i++)
        set_xreg(i, 0xbadbeef);

    // Set FP registers to a value that is NaN in both 32-bit and 64-bit FP.
    for (unsigned i = 0; i < kNumberOfFloatRegisters; i++)
        set_dreg(i, kFP64SignallingNaN);
    resume_pc_ = 0;
    // Returning to address 0 exits the Simulator.
    set_lr(kEndOfSimAddress);
}

Simulator::~Simulator()
{
    js_free(stack_);

    // The decoder may outlive the simulator.
    decoder_->RemoveVisitor(print_disasm_);
    js_free(print_disasm_);

    decoder_->RemoveVisitor(instrumentation_);
    js_free(instrumentation_);
}

void
Simulator::Run()
{
    pc_modified_ = false;
    while (pc_ != kEndOfSimAddress) {
        ExecuteInstruction();
    }
}

void
Simulator::RunFrom(Instruction* first)
{
    set_pc(first);
    Run();
}

int64_t
Simulator::call(uint8_t* entry, int argument_count, ...)
{
    va_list parameters;
    va_start(parameters, argument_count);

    // First eight arguments passed in registers.
    MOZ_ASSERT(argument_count <= 8);
    // This code should use the type of the called function
    // (with templates, like the callVM machinery), but since the
    // number of called functions is miniscule, their types have been
    // divined from the number of arguments.
    if (argument_count == 8) {
        // EnterJitData::jitcode.
        set_xreg(0, va_arg(parameters, int64_t));
        // EnterJitData::maxArgc.
        set_xreg(1, va_arg(parameters, unsigned));
        // EnterJitData::maxArgv.
        set_xreg(2, va_arg(parameters, int64_t));
        // EnterJitData::osrFrame.
        set_xreg(3, va_arg(parameters, int64_t));
        // EnterJitData::calleeToken.
        set_xreg(4, va_arg(parameters, int64_t));
        // EnterJitData::scopeChain.
        set_xreg(5, va_arg(parameters, int64_t));
        // EnterJitData::osrNumStackValues.
        set_xreg(6, va_arg(parameters, unsigned));
        // Address of EnterJitData::result.
        set_xreg(7, va_arg(parameters, int64_t));
    } else if (argument_count == 2) {
        // EntryArg *args
        set_xreg(0, va_arg(parameters, int64_t));
        // uint8_t *GlobalData
        set_xreg(1, va_arg(parameters, int64_t));
    } else {
        MOZ_CRASH("Unknown number of arguments");
    }

    va_end(parameters);

    // Call must transition back to native code on exit.
    MOZ_ASSERT(xreg(30) == int64_t(kEndOfSimAddress));

    // Execute the simulation.
    DebugOnly<int64_t> entryStack = xreg(31, Reg31IsStackPointer);
    RunFrom((Instruction *)entry);
    DebugOnly<int64_t> exitStack = xreg(31, Reg31IsStackPointer);
    MOZ_ASSERT(entryStack == exitStack);

    int64_t result = xreg(0);
    if (getenv("USE_DEBUGGER"))
        printf("LEAVE\n");
    return result;
}

// When the generated code calls a VM function (masm.callWithABI) we need to
// call that function instead of trying to execute it with the simulator
// (because it's x64 code instead of AArch64 code). We do that by redirecting the VM
// call to a svc (Supervisor Call) instruction that is handled by the
// simulator. We write the original destination of the jump just at a known
// offset from the svc instruction so the simulator knows what to call.
class Redirection
{
    friend class Simulator;

    Redirection(void *nativeFunction, ABIFunctionType type, Simulator *sim)
      : nativeFunction_(nativeFunction),
      type_(type),
      next_(nullptr)
    {
        next_ = sim->redirection();
        // TODO: Flush ICache?
        sim->setRedirection(this);

        Instruction *instr = (Instruction *)(&svcInstruction_);
        AssemblerVIXL::svc(instr, kCallRtRedirected);
    }

  public:
    void *addressOfSvcInstruction() { return &svcInstruction_; }
    void *nativeFunction() const { return nativeFunction_; }
    ABIFunctionType type() const { return type_; }

    static Redirection *Get(void *nativeFunction, ABIFunctionType type) {
        Simulator *sim = Simulator::Current();
        AutoLockSimulatorCache alsr(sim);

        // TODO: Store srt_ in the simulator for this assertion.
        // MOZ_ASSERT_IF(pt->simulator(), pt->simulator()->srt_ == srt);

        Redirection *current = sim->redirection();
        for (; current != nullptr; current = current->next_) {
            if (current->nativeFunction_ == nativeFunction) {
                MOZ_ASSERT(current->type() == type);
                return current;
            }
        }

        Redirection *redir = (Redirection *)js_malloc(sizeof(Redirection));
        if (!redir) {
            MOZ_ReportAssertionFailure("[unhandlable oom] Simulator redirection", __FILE__, __LINE__);
            MOZ_CRASH();
        }
        new(redir) Redirection(nativeFunction, type, sim);
        return redir;
    }

    static Redirection *FromSvcInstruction(Instruction *svcInstruction) {
        uint8_t *addrOfSvc = reinterpret_cast<uint8_t*>(svcInstruction);
        uint8_t *addrOfRedirection = addrOfSvc - offsetof(Redirection, svcInstruction_);
        return reinterpret_cast<Redirection*>(addrOfRedirection);
    }

  private:
    void *nativeFunction_;
    uint32_t svcInstruction_;
    ABIFunctionType type_;
    Redirection *next_;
};

/* static */
void *
Simulator::RedirectNativeFunction(void *nativeFunction, ABIFunctionType type)
{
    Redirection *redirection = Redirection::Get(nativeFunction, type);
    return redirection->addressOfSvcInstruction();
}

const char* Simulator::xreg_names[] = {
"x0",  "x1",  "x2",  "x3",  "x4",  "x5",  "x6",  "x7",
"x8",  "x9",  "x10", "x11", "x12", "x13", "x14", "x15",
"x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23",
"x24", "x25", "x26", "x27", "x28", "x29", "lr",  "xzr", "sp"};

const char* Simulator::wreg_names[] = {
"w0",  "w1",  "w2",  "w3",  "w4",  "w5",  "w6",  "w7",
"w8",  "w9",  "w10", "w11", "w12", "w13", "w14", "w15",
"w16", "w17", "w18", "w19", "w20", "w21", "w22", "w23",
"w24", "w25", "w26", "w27", "w28", "w29", "w30", "wzr", "wsp"};

const char* Simulator::sreg_names[] = {
"s0",  "s1",  "s2",  "s3",  "s4",  "s5",  "s6",  "s7",
"s8",  "s9",  "s10", "s11", "s12", "s13", "s14", "s15",
"s16", "s17", "s18", "s19", "s20", "s21", "s22", "s23",
"s24", "s25", "s26", "s27", "s28", "s29", "s30", "s31"};

const char* Simulator::dreg_names[] = {
"d0",  "d1",  "d2",  "d3",  "d4",  "d5",  "d6",  "d7",
"d8",  "d9",  "d10", "d11", "d12", "d13", "d14", "d15",
"d16", "d17", "d18", "d19", "d20", "d21", "d22", "d23",
"d24", "d25", "d26", "d27", "d28", "d29", "d30", "d31"};

const char* Simulator::vreg_names[] = {
"v0",  "v1",  "v2",  "v3",  "v4",  "v5",  "v6",  "v7",
"v8",  "v9",  "v10", "v11", "v12", "v13", "v14", "v15",
"v16", "v17", "v18", "v19", "v20", "v21", "v22", "v23",
"v24", "v25", "v26", "v27", "v28", "v29", "v30", "v31"};

const char*
Simulator::WRegNameForCode(unsigned code, Reg31Mode mode)
{
    MOZ_ASSERT(code < kNumberOfRegisters);
    // If the code represents the stack pointer, index the name after zr.
    if ((code == kZeroRegCode) && (mode == Reg31IsStackPointer))
        code = kZeroRegCode + 1;
    return wreg_names[code];
}

const char*
Simulator::XRegNameForCode(unsigned code, Reg31Mode mode)
{
    MOZ_ASSERT(code < kNumberOfRegisters);
    // If the code represents the stack pointer, index the name after zr.
    if ((code == kZeroRegCode) && (mode == Reg31IsStackPointer))
        code = kZeroRegCode + 1;
    return xreg_names[code];
}

const char*
Simulator::SRegNameForCode(unsigned code)
{
    MOZ_ASSERT(code < kNumberOfFloatRegisters);
    return sreg_names[code];
}

const char*
Simulator::DRegNameForCode(unsigned code)
{
    MOZ_ASSERT(code < kNumberOfFloatRegisters);
    return dreg_names[code];
}

const char*
Simulator::VRegNameForCode(unsigned code)
{
    MOZ_ASSERT(code < kNumberOfFloatRegisters);
    return vreg_names[code];
}

#define COLOUR(colour_code)  "\033[0;" colour_code "m"
#define COLOUR_BOLD(colour_code)    "\033[1;" colour_code "m"
#define NORMAL  ""
#define GREY    "30"
#define RED     "31"
#define GREEN   "32"
#define YELLOW  "33"
#define BLUE    "34"
#define MAGENTA "35"
#define CYAN    "36"
#define WHITE   "37"
void
Simulator::set_coloured_trace(bool value)
{
    coloured_trace_ = value;

    clr_normal          = value ? COLOUR(NORMAL)        : "";
    clr_flag_name       = value ? COLOUR_BOLD(GREY)     : "";
    clr_flag_value      = value ? COLOUR_BOLD(WHITE)    : "";
    clr_reg_name        = value ? COLOUR_BOLD(BLUE)     : "";
    clr_reg_value       = value ? COLOUR_BOLD(CYAN)     : "";
    clr_fpreg_name      = value ? COLOUR_BOLD(YELLOW)   : "";
    clr_fpreg_value     = value ? COLOUR_BOLD(MAGENTA)  : "";
    clr_memory_value    = value ? COLOUR_BOLD(GREEN)    : "";
    clr_memory_address  = value ? COLOUR(GREEN)         : "";
    clr_debug_number    = value ? COLOUR_BOLD(YELLOW)   : "";
    clr_debug_message   = value ? COLOUR(YELLOW)        : "";
    clr_warning         = value ? COLOUR_BOLD(RED)      : "";
    clr_warning_message = value ? COLOUR(RED)           : "";
    clr_printf          = value ? COLOUR(GREEN)         : "";
}
#undef COLOUR
#undef BOLD
#undef NORMAL
#undef GREY
#undef GREEN
#undef ORANGE
#undef BLUE
#undef PURPLE
#undef INDIGO
#undef WHITE

// Helpers ---------------------------------------------------------------------
int64_t
Simulator::AddWithCarry(unsigned reg_size, bool set_flags,
                        int64_t src1, int64_t src2, int64_t carry_in)
{
    MOZ_ASSERT((carry_in == 0) || (carry_in == 1));
    MOZ_ASSERT((reg_size == kXRegSize) || (reg_size == kWRegSize));

    uint64_t u1, u2;
    int64_t result;
    int64_t signed_sum = src1 + src2 + carry_in;

    uint32_t N, Z, C, V;

    if (reg_size == kWRegSize) {
        u1 = static_cast<uint64_t>(src1) & kWRegMask;
        u2 = static_cast<uint64_t>(src2) & kWRegMask;

        result = signed_sum & kWRegMask;
        // Compute the C flag by comparing the sum to the max unsigned integer.
        C = ((kWMaxUInt - u1) < (u2 + carry_in)) ||
            ((kWMaxUInt - u1 - carry_in) < u2);
        // Overflow iff the sign bit is the same for the two inputs and different
        // for the result.
        int64_t s_src1 = src1 << (kXRegSize - kWRegSize);
        int64_t s_src2 = src2 << (kXRegSize - kWRegSize);
        int64_t s_result = result << (kXRegSize - kWRegSize);
        V = ((s_src1 ^ s_src2) >= 0) && ((s_src1 ^ s_result) < 0);

    } else {
        u1 = static_cast<uint64_t>(src1);
        u2 = static_cast<uint64_t>(src2);

        result = signed_sum;
        // Compute the C flag by comparing the sum to the max unsigned integer.
        C = ((kXMaxUInt - u1) < (u2 + carry_in)) ||
            ((kXMaxUInt - u1 - carry_in) < u2);
        // Overflow iff the sign bit is the same for the two inputs and different
        // for the result.
        V = ((src1 ^ src2) >= 0) && ((src1 ^ result) < 0);
    }

    N = CalcNFlag(result, reg_size);
    Z = CalcZFlag(result);

    if (set_flags) {
        nzcv().SetN(N);
        nzcv().SetZ(Z);
        nzcv().SetC(C);
        nzcv().SetV(V);
    }
    return result;
}

int64_t
Simulator::ShiftOperand(unsigned reg_size, int64_t value, Shift shift_type, unsigned amount)
{
    if (amount == 0)
      return value;
  
    int64_t mask = reg_size == kXRegSize ? kXRegMask : kWRegMask;
    switch (shift_type) {
      case LSL:
        return (value << amount) & mask;
      case LSR:
        return static_cast<uint64_t>(value) >> amount;
      case ASR: {
        // Shift used to restore the sign.
        unsigned s_shift = kXRegSize - reg_size;
        // Value with its sign restored.
        int64_t s_value = (value << s_shift) >> s_shift;
        return (s_value >> amount) & mask;
      }
      case ROR: {
        if (reg_size == kWRegSize)
          value &= kWRegMask;

        return (static_cast<uint64_t>(value) >> amount) |
               ((value & ((INT64_C(1) << amount) - 1)) <<
                (reg_size - amount));
      }
      default:
        VIXL_UNIMPLEMENTED();
        return 0;
    }
}

int64_t
Simulator::ExtendValue(unsigned reg_size, int64_t value, Extend extend_type, unsigned left_shift)
{
    switch (extend_type) {
      case UXTB:
        value &= kByteMask;
        break;
      case UXTH:
        value &= kHalfWordMask;
        break;
      case UXTW:
        value &= kWordMask;
        break;
      case SXTB:
        value = (value << 56) >> 56;
        break;
      case SXTH:
        value = (value << 48) >> 48;
        break;
      case SXTW:
        value = (value << 32) >> 32;
        break;
      case UXTX:
      case SXTX:
        break;
      default:
        VIXL_UNREACHABLE();
    }
    int64_t mask = (reg_size == kXRegSize) ? kXRegMask : kWRegMask;
    return (value << left_shift) & mask;
}

template<>
double
Simulator::FPDefaultNaN<double>() const
{
    return kFP64DefaultNaN;
}

template<>
float
Simulator::FPDefaultNaN<float>() const
{
    return kFP32DefaultNaN;
}

void
Simulator::FPCompare(double val0, double val1)
{
    AssertSupportedFPCR();

    // TODO: This assumes that the C++ implementation handles comparisons in the
    // way that we expect (as per AssertSupportedFPCR()).
    if ((mozilla::IsNaN(val0) != 0) || (mozilla::IsNaN(val1) != 0))
        nzcv().SetRawValue(FPUnorderedFlag);
    else if (val0 < val1)
        nzcv().SetRawValue(FPLessThanFlag);
    else if (val0 > val1)
        nzcv().SetRawValue(FPGreaterThanFlag);
    else if (val0 == val1)
        nzcv().SetRawValue(FPEqualFlag);
    else
        VIXL_UNREACHABLE();
}

void
Simulator::PrintSystemRegisters(bool print_all)
{
    static bool first_run = true;

    static SimSystemRegister last_nzcv;
    if (print_all || first_run || (last_nzcv.RawValue() != nzcv().RawValue())) {
        fprintf(stream_, "# %sFLAGS: %sN:%d Z:%d C:%d V:%d%s\n",
                clr_flag_name,
                clr_flag_value,
                N(), Z(), C(), V(),
                clr_normal);
    }
    last_nzcv = nzcv();

    static SimSystemRegister last_fpcr;
    if (print_all || first_run || (last_fpcr.RawValue() != fpcr().RawValue())) {
        static const char * rmode[] = {
            "0b00 (Round to Nearest)",
            "0b01 (Round towards Plus Infinity)",
            "0b10 (Round towards Minus Infinity)",
            "0b11 (Round towards Zero)"
        };
        MOZ_ASSERT(fpcr().RMode() <= (sizeof(rmode) / sizeof(rmode[0])));
        fprintf(stream_, "# %sFPCR: %sAHP:%d DN:%d FZ:%d RMode:%s%s\n",
                clr_flag_name,
                clr_flag_value,
                fpcr().AHP(), fpcr().DN(), fpcr().FZ(), rmode[fpcr().RMode()],
                clr_normal);
    }
    last_fpcr = fpcr();

    first_run = false;
}

void
Simulator::PrintRegisters(bool print_all_regs)
{
    static bool first_run = true;
    static int64_t last_regs[kNumberOfRegisters];

    for (unsigned i = 0; i < kNumberOfRegisters; i++) {
        if (print_all_regs || first_run || (last_regs[i] != xreg(i, Reg31IsStackPointer))) {
            fprintf(stream_,
                    "# %s%4s:%s 0x%016" PRIx64 "%s\n",
                    clr_reg_name,
                    XRegNameForCode(i, Reg31IsStackPointer),
                    clr_reg_value,
                    xreg(i, Reg31IsStackPointer),
                    clr_normal);
        }
        // Cache the new register value so the next run can detect any changes.
        last_regs[i] = xreg(i, Reg31IsStackPointer);
    }
    first_run = false;
}

void
Simulator::PrintFloatRegisters(bool print_all_regs)
{
    static bool first_run = true;
    static uint64_t last_regs[kNumberOfFloatRegisters];

    // Print as many rows of registers as necessary, keeping each individual
    // register in the same column each time (to make it easy to visually scan
    // for changes).
    for (unsigned i = 0; i < kNumberOfFloatRegisters; i++) {
        if (print_all_regs || first_run || (last_regs[i] != dreg_bits(i))) {
            fprintf(stream_,
                    "# %s%4s:%s 0x%016" PRIx64 "%s (%s%s:%s %g%s %s:%s %g%s)\n",
                    clr_fpreg_name,
                    VRegNameForCode(i),
                    clr_fpreg_value,
                    dreg_bits(i),
                    clr_normal,
                    clr_fpreg_name,
                    DRegNameForCode(i),
                    clr_fpreg_value,
                    dreg(i),
                    clr_fpreg_name,
                    SRegNameForCode(i),
                    clr_fpreg_value,
                    sreg(i),
                    clr_normal);
        }
        // Cache the new register value so the next run can detect any changes.
        last_regs[i] = dreg_bits(i);
    }
    first_run = false;
}

void
Simulator::PrintProcessorState()
{
    PrintSystemRegisters();
    PrintRegisters();
    PrintFloatRegisters();
}

// Visitors---------------------------------------------------------------------
void
Simulator::VisitUnimplemented(Instruction* instr)
{
    printf("Unimplemented instruction at %p: 0x%08" PRIx32 "\n",
            reinterpret_cast<void*>(instr), instr->InstructionBits());
    VIXL_UNIMPLEMENTED();
}

void
Simulator::VisitUnallocated(Instruction* instr)
{
    printf("Unallocated instruction at %p: 0x%08" PRIx32 "\n",
            reinterpret_cast<void*>(instr), instr->InstructionBits());
    VIXL_UNIMPLEMENTED();
}

void
Simulator::VisitPCRelAddressing(Instruction* instr)
{
    MOZ_ASSERT((instr->Mask(PCRelAddressingMask) == ADR) ||
            (instr->Mask(PCRelAddressingMask) == ADRP));
    set_reg(instr->Rd(), instr->ImmPCOffsetTarget());
}

void
Simulator::VisitUnconditionalBranch(Instruction* instr)
{
    switch (instr->Mask(UnconditionalBranchMask)) {
      case BL:
        set_lr(instr->NextInstruction());
        // Fall through.
      case B:
        set_pc(instr->ImmPCOffsetTarget());
        break;
      default: VIXL_UNREACHABLE();
    }
}

void
Simulator::VisitConditionalBranch(Instruction* instr)
{
    MOZ_ASSERT(instr->Mask(ConditionalBranchMask) == B_cond);
    if (ConditionPassed(instr->ConditionBranch()))
        set_pc(instr->ImmPCOffsetTarget());
}

void
Simulator::VisitUnconditionalBranchToRegister(Instruction* instr)
{
    Instruction* target = Instruction::Cast(xreg(instr->Rn()));

    switch (instr->Mask(UnconditionalBranchToRegisterMask)) {
      case BLR:
        set_lr(instr->NextInstruction());
        // Fall through.
      case BR:
      case RET: set_pc(target); break;
      default: VIXL_UNREACHABLE();
    }
}

void
Simulator::VisitTestBranch(Instruction* instr)
{
    unsigned bit_pos = (instr->ImmTestBranchBit5() << 5) |
        instr->ImmTestBranchBit40();
    bool bit_zero = ((xreg(instr->Rt()) >> bit_pos) & 1) == 0;
    bool take_branch = false;
    switch (instr->Mask(TestBranchMask)) {
      case TBZ: take_branch = bit_zero; break;
      case TBNZ: take_branch = !bit_zero; break;
      default: VIXL_UNIMPLEMENTED();
    }

    if (take_branch)
        set_pc(instr->ImmPCOffsetTarget());
}

void
Simulator::VisitCompareBranch(Instruction* instr)
{
    unsigned rt = instr->Rt();
    bool take_branch = false;
    switch (instr->Mask(CompareBranchMask)) {
      case CBZ_w: take_branch = (wreg(rt) == 0); break;
      case CBZ_x: take_branch = (xreg(rt) == 0); break;
      case CBNZ_w: take_branch = (wreg(rt) != 0); break;
      case CBNZ_x: take_branch = (xreg(rt) != 0); break;
      default: VIXL_UNIMPLEMENTED();
    }

    if (take_branch)
        set_pc(instr->ImmPCOffsetTarget());
}

void
Simulator::AddSubHelper(Instruction* instr, int64_t op2)
{
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    bool set_flags = instr->FlagsUpdate();
    int64_t new_val = 0;
    Instr operation = instr->Mask(AddSubOpMask);

    switch (operation) {
      case ADD:
      case ADDS: {
        new_val = AddWithCarry(reg_size,
                               set_flags,
                               reg(reg_size, instr->Rn(), instr->RnMode()),
                               op2);
        break;
      }
      case SUB:
      case SUBS: {
        new_val = AddWithCarry(reg_size,
                               set_flags,
                               reg(reg_size, instr->Rn(), instr->RnMode()),
                               ~op2,
                               1);
        break;
      }
      default: VIXL_UNREACHABLE();
    }

    set_reg(reg_size, instr->Rd(), new_val, instr->RdMode());
}

void
Simulator::VisitAddSubShifted(Instruction* instr)
{
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    int64_t op2 = ShiftOperand(reg_size,
                               reg(reg_size, instr->Rm()),
                               static_cast<Shift>(instr->ShiftDP()),
                               instr->ImmDPShift());
    AddSubHelper(instr, op2);
}

void
Simulator::VisitAddSubImmediate(Instruction* instr)
{
    int64_t op2 = instr->ImmAddSub() << ((instr->ShiftAddSub() == 1) ? 12 : 0);
    AddSubHelper(instr, op2);
}

void
Simulator::VisitAddSubExtended(Instruction* instr)
{
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    int64_t op2 = ExtendValue(reg_size,
                              reg(reg_size, instr->Rm()),
                              static_cast<Extend>(instr->ExtendMode()),
                              instr->ImmExtendShift());
    AddSubHelper(instr, op2);
}

void
Simulator::VisitAddSubWithCarry(Instruction* instr)
{
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    int64_t op2 = reg(reg_size, instr->Rm());
    int64_t new_val;

    if ((instr->Mask(AddSubOpMask) == SUB) || instr->Mask(AddSubOpMask) == SUBS)
        op2 = ~op2;

    new_val = AddWithCarry(reg_size,
                           instr->FlagsUpdate(),
                           reg(reg_size, instr->Rn()),
                           op2,
                           C());

    set_reg(reg_size, instr->Rd(), new_val);
}

void
Simulator::VisitLogicalShifted(Instruction* instr)
{
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    Shift shift_type = static_cast<Shift>(instr->ShiftDP());
    unsigned shift_amount = instr->ImmDPShift();
    int64_t op2 = ShiftOperand(reg_size, reg(reg_size, instr->Rm()), shift_type,
            shift_amount);
    if (instr->Mask(NOT) == NOT)
        op2 = ~op2;
    LogicalHelper(instr, op2);
}

void
Simulator::VisitLogicalImmediate(Instruction* instr)
{
    LogicalHelper(instr, instr->ImmLogical());
}

void
Simulator::LogicalHelper(Instruction* instr, int64_t op2)
{
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    int64_t op1 = reg(reg_size, instr->Rn());
    int64_t result = 0;
    bool update_flags = false;

    // Switch on the logical operation, stripping out the NOT bit, as it has a
    // different meaning for logical immediate instructions.
    switch (instr->Mask(LogicalOpMask & ~NOT)) {
      case ANDS: update_flags = true;  // Fall through.
      case AND: result = op1 & op2; break;
      case ORR: result = op1 | op2; break;
      case EOR: result = op1 ^ op2; break;
      default: VIXL_UNIMPLEMENTED();
    }

    if (update_flags) {
        nzcv().SetN(CalcNFlag(result, reg_size));
        nzcv().SetZ(CalcZFlag(result));
        nzcv().SetC(0);
        nzcv().SetV(0);
    }

    set_reg(reg_size, instr->Rd(), result, instr->RdMode());
}

void
Simulator::VisitConditionalCompareRegister(Instruction* instr)
{
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    ConditionalCompareHelper(instr, reg(reg_size, instr->Rm()));
}

void
Simulator::VisitConditionalCompareImmediate(Instruction* instr)
{
    ConditionalCompareHelper(instr, instr->ImmCondCmp());
}

void
Simulator::ConditionalCompareHelper(Instruction* instr, int64_t op2)
{
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    int64_t op1 = reg(reg_size, instr->Rn());

    if (ConditionPassed(instr->Condition())) {
        // If the condition passes, set the status flags to the result of comparing
        // the operands.
        if (instr->Mask(ConditionalCompareMask) == CCMP) {
            AddWithCarry(reg_size, true, op1, ~op2, 1);
        } else {
            MOZ_ASSERT(instr->Mask(ConditionalCompareMask) == CCMN);
            AddWithCarry(reg_size, true, op1, op2, 0);
        }
    } else {
        // If the condition fails, set the status flags to the nzcv immediate.
        nzcv().SetFlags(instr->Nzcv());
    }
}

void
Simulator::VisitLoadStoreUnsignedOffset(Instruction* instr)
{
    int offset = instr->ImmLSUnsigned() << instr->SizeLS();
    LoadStoreHelper(instr, offset, Offset);
}

void
Simulator::VisitLoadStoreUnscaledOffset(Instruction* instr)
{
    LoadStoreHelper(instr, instr->ImmLS(), Offset);
}

void
Simulator::VisitLoadStorePreIndex(Instruction* instr)
{
    LoadStoreHelper(instr, instr->ImmLS(), PreIndex);
}

void
Simulator::VisitLoadStorePostIndex(Instruction* instr)
{
    LoadStoreHelper(instr, instr->ImmLS(), PostIndex);
}

void
Simulator::VisitLoadStoreRegisterOffset(Instruction* instr)
{
    Extend ext = static_cast<Extend>(instr->ExtendMode());
    MOZ_ASSERT((ext == UXTW) || (ext == UXTX) || (ext == SXTW) || (ext == SXTX));
    unsigned shift_amount = instr->ImmShiftLS() * instr->SizeLS();

    int64_t offset = ExtendValue(kXRegSize, xreg(instr->Rm()), ext, shift_amount);
    LoadStoreHelper(instr, offset, Offset);
}

void
Simulator::LoadStoreHelper(Instruction* instr, int64_t offset, AddrMode addrmode)
{
    unsigned srcdst = instr->Rt();
    uint8_t* address = AddressModeHelper(instr->Rn(), offset, addrmode);

    LoadStoreOp op = static_cast<LoadStoreOp>(instr->Mask(LoadStoreOpMask));
    switch (op) {
      case LDRB_w:  set_wreg(srcdst, MemoryRead<uint8_t>(address)); break;
      case LDRH_w:  set_wreg(srcdst, MemoryRead<uint16_t>(address)); break;
      case LDR_w:   set_wreg(srcdst, MemoryRead<uint32_t>(address)); break;
      case LDR_x:   set_xreg(srcdst, MemoryRead<uint64_t>(address)); break;
      case LDRSB_w: set_wreg(srcdst, MemoryRead<int8_t>(address)); break;
      case LDRSH_w: set_wreg(srcdst, MemoryRead<int16_t>(address)); break;
      case LDRSB_x: set_xreg(srcdst, MemoryRead<int8_t>(address)); break;
      case LDRSH_x: set_xreg(srcdst, MemoryRead<int16_t>(address)); break;
      case LDRSW_x: set_xreg(srcdst, MemoryRead<int32_t>(address)); break;
      case LDR_s:   set_sreg(srcdst, MemoryRead<float>(address)); break;
      case LDR_d:   set_dreg(srcdst, MemoryRead<double>(address)); break;

      case STRB_w:  MemoryWrite<uint8_t>(address, wreg(srcdst)); break;
      case STRH_w:  MemoryWrite<uint16_t>(address, wreg(srcdst)); break;
      case STR_w:   MemoryWrite<uint32_t>(address, wreg(srcdst)); break;
      case STR_x:   MemoryWrite<uint64_t>(address, xreg(srcdst)); break;
      case STR_s:   MemoryWrite<float>(address, sreg(srcdst)); break;
      case STR_d:   MemoryWrite<double>(address, dreg(srcdst)); break;

      default: VIXL_UNIMPLEMENTED();
    }

    local_monitor_.MaybeClear();
}

void
Simulator::VisitLoadStorePairOffset(Instruction* instr)
{
    LoadStorePairHelper(instr, Offset);
}

void
Simulator::VisitLoadStorePairPreIndex(Instruction* instr)
{
    LoadStorePairHelper(instr, PreIndex);
}

void
Simulator::VisitLoadStorePairPostIndex(Instruction* instr)
{
    LoadStorePairHelper(instr, PostIndex);
}

void
Simulator::VisitLoadStorePairNonTemporal(Instruction* instr)
{
    LoadStorePairHelper(instr, Offset);
}

void
Simulator::LoadStorePairHelper(Instruction* instr, AddrMode addrmode)
{
    unsigned rt = instr->Rt();
    unsigned rt2 = instr->Rt2();
    int offset = instr->ImmLSPair() << instr->SizeLSPair();
    uint8_t* address = AddressModeHelper(instr->Rn(), offset, addrmode);

    LoadStorePairOp op = static_cast<LoadStorePairOp>(instr->Mask(LoadStorePairMask));

    // 'rt' and 'rt2' can only be aliased for stores.
    MOZ_ASSERT(((op & LoadStorePairLBit) == 0) || (rt != rt2));

    switch (op) {
      case LDP_w: {
        set_wreg(rt, MemoryRead<uint32_t>(address));
        set_wreg(rt2, MemoryRead<uint32_t>(address + kWRegSizeInBytes));
        break;
      }
      case LDP_s: {
        set_sreg(rt, MemoryRead<float>(address));
        set_sreg(rt2, MemoryRead<float>(address + kSRegSizeInBytes));
        break;
      }
      case LDP_x: {
        set_xreg(rt, MemoryRead<uint64_t>(address));
        set_xreg(rt2, MemoryRead<uint64_t>(address + kXRegSizeInBytes));
        break;
      }
      case LDP_d: {
        set_dreg(rt, MemoryRead<double>(address));
        set_dreg(rt2, MemoryRead<double>(address + kDRegSizeInBytes));
        break;
      }
      case LDPSW_x: {
        set_xreg(rt, MemoryRead<int32_t>(address));
        set_xreg(rt2, MemoryRead<int32_t>(address + kWRegSizeInBytes));
        break;
      }
      case STP_w: {
        MemoryWrite<uint32_t>(address, wreg(rt));
        MemoryWrite<uint32_t>(address + kWRegSizeInBytes, wreg(rt2));
        break;
      }
      case STP_s: {
        MemoryWrite<float>(address, sreg(rt));
        MemoryWrite<float>(address + kSRegSizeInBytes, sreg(rt2));
        break;
      }
      case STP_x: {
        MemoryWrite<uint64_t>(address, xreg(rt));
        MemoryWrite<uint64_t>(address + kXRegSizeInBytes, xreg(rt2));
        break;
      }
      case STP_d: {
        MemoryWrite<double>(address, dreg(rt));
        MemoryWrite<double>(address + kDRegSizeInBytes, dreg(rt2));
        break;
      }
      default: VIXL_UNREACHABLE();
    }

    local_monitor_.MaybeClear();
}

void
Simulator::PrintExclusiveAccessWarning()
{
    if (print_exclusive_access_warning_) {
        fprintf(
            stderr,
            "%sWARNING:%s VIXL simulator support for load-/store-/clear-exclusive "
            "instructions is limited. Refer to the README for details.%s\n",
            clr_warning, clr_warning_message, clr_normal);
        print_exclusive_access_warning_ = false;
    }
}

void
Simulator::VisitLoadStoreExclusive(Instruction* instr)
{
    PrintExclusiveAccessWarning();

    unsigned rs = instr->Rs();
    unsigned rt = instr->Rt();
    unsigned rt2 = instr->Rt2();
    unsigned rn = instr->Rn();

    LoadStoreExclusive op = static_cast<LoadStoreExclusive>(instr->Mask(LoadStoreExclusiveMask));

    bool is_acquire_release = instr->LdStXAcquireRelease();
    bool is_exclusive = !instr->LdStXNotExclusive();
    bool is_load = instr->LdStXLoad();
    bool is_pair = instr->LdStXPair();

    uint8_t * address = reg<uint8_t *>(rn, Reg31IsStackPointer);
    size_t element_size = 1 << instr->LdStXSizeLog2();
    size_t access_size = is_pair ? element_size * 2 : element_size;

    // Check the alignment of `address`.
    if (AlignDown(address, access_size) != address)
        VIXL_ALIGNMENT_EXCEPTION();

    // The sp must be aligned to 16 bytes when it is accessed.
    if ((rn == 31) && (AlignDown(address, 16) != address))
        VIXL_ALIGNMENT_EXCEPTION();

    if (is_load) {
        if (is_exclusive) {
            local_monitor_.MarkExclusive(address, access_size);
        } else {
            // Any non-exclusive load can clear the local monitor as a side effect. We
            // don't need to do this, but it is useful to stress the simulated code.
            local_monitor_.Clear();
        }

        switch (op) {
          case LDXRB_w:
          case LDAXRB_w:
          case LDARB_w:
            set_wreg(rt, MemoryRead<uint8_t>(address));
            break;
          case LDXRH_w:
          case LDAXRH_w:
          case LDARH_w:
            set_wreg(rt, MemoryRead<uint16_t>(address));
            break;
          case LDXR_w:
          case LDAXR_w:
          case LDAR_w:
            set_wreg(rt, MemoryRead<uint32_t>(address));
            break;
          case LDXR_x:
          case LDAXR_x:
          case LDAR_x:
            set_xreg(rt, MemoryRead<uint64_t>(address));
            break;
          case LDXP_w:
          case LDAXP_w:
            set_wreg(rt, MemoryRead<uint32_t>(address));
            set_wreg(rt2, MemoryRead<uint32_t>(address + element_size));
            break;
          case LDXP_x:
          case LDAXP_x:
            set_xreg(rt, MemoryRead<uint64_t>(address));
            set_xreg(rt2, MemoryRead<uint64_t>(address + element_size));
            break;
          default:
            VIXL_UNREACHABLE();
        }

        if (is_acquire_release) {
            // Approximate load-acquire by issuing a full barrier after the load.
            __sync_synchronize();
        }
    } else {
        if (is_acquire_release) {
            // Approximate store-release by issuing a full barrier before the store.
            __sync_synchronize();
        }

        bool do_store = true;
        if (is_exclusive) {
            do_store = local_monitor_.IsExclusive(address, access_size) &&
                global_monitor_.IsExclusive(address, access_size);
            set_wreg(rs, do_store ? 0 : 1);

            //  - All exclusive stores explicitly clear the local monitor.
            local_monitor_.Clear();
        } else {
            //  - Any other store can clear the local monitor as a side effect.
            local_monitor_.MaybeClear();
        }

        if (do_store) {
            switch (op) {
              case STXRB_w:
              case STLXRB_w:
              case STLRB_w:
                MemoryWrite<uint8_t>(address, wreg(rt));
                break;
              case STXRH_w:
              case STLXRH_w:
              case STLRH_w:
                MemoryWrite<uint16_t>(address, wreg(rt));
                break;
              case STXR_w:
              case STLXR_w:
              case STLR_w:
                MemoryWrite<uint32_t>(address, wreg(rt));
                break;
              case STXR_x:
              case STLXR_x:
              case STLR_x:
                MemoryWrite<uint64_t>(address, xreg(rt));
                break;
              case STXP_w:
              case STLXP_w:
                MemoryWrite<uint32_t>(address, wreg(rt));
                MemoryWrite<uint32_t>(address + element_size, wreg(rt2));
                break;
              case STXP_x:
              case STLXP_x:
                MemoryWrite<uint64_t>(address, xreg(rt));
                MemoryWrite<uint64_t>(address + element_size, xreg(rt2));
                break;
              default:
                VIXL_UNREACHABLE();
            }
        }
    }
}

void
Simulator::VisitLoadLiteral(Instruction* instr)
{
    uint8_t* address = instr->LiteralAddress();
    unsigned rt = instr->Rt();

    switch (instr->Mask(LoadLiteralMask)) {
      case LDR_w_lit: set_wreg(rt, MemoryRead<uint32_t>(address)); break;
      case LDR_x_lit: set_xreg(rt, MemoryRead<uint64_t>(address)); break;
      case LDR_s_lit: set_sreg(rt, MemoryRead<float>(address)); break;
      case LDR_d_lit: set_dreg(rt, MemoryRead<double>(address)); break;
      default: VIXL_UNREACHABLE();
    }

    local_monitor_.MaybeClear();
}

uint8_t *
Simulator::AddressModeHelper(unsigned addr_reg, int64_t offset, AddrMode addrmode)
{
    uint64_t address = xreg(addr_reg, Reg31IsStackPointer);

    if ((addr_reg == 31) && ((address % 16) != 0)) {
        // When the base register is SP the stack pointer is required to be
        // quadword aligned prior to the address calculation and write-backs.
        // Misalignment will cause a stack alignment fault.
        VIXL_ALIGNMENT_EXCEPTION();
    }

    if ((addrmode == PreIndex) || (addrmode == PostIndex)) {
        MOZ_ASSERT(offset != 0);
        set_xreg(addr_reg, address + offset, Reg31IsStackPointer);
    }

    if ((addrmode == Offset) || (addrmode == PreIndex))
        address += offset;

    // Verify that the calculated address is available to the host.
    MOZ_ASSERT(address == static_cast<uintptr_t>(address));

    return reinterpret_cast<uint8_t*>(address);
}

void
Simulator::VisitMoveWideImmediate(Instruction* instr)
{
    MoveWideImmediateOp mov_op =
        static_cast<MoveWideImmediateOp>(instr->Mask(MoveWideImmediateMask));
    int64_t new_xn_val = 0;

    bool is_64_bits = instr->SixtyFourBits() == 1;
    // Shift is limited for W operations.
    MOZ_ASSERT(is_64_bits || (instr->ShiftMoveWide() < 2));

    // Get the shifted immediate.
    int64_t shift = instr->ShiftMoveWide() * 16;
    int64_t shifted_imm16 = instr->ImmMoveWide() << shift;

    // Compute the new value.
    switch (mov_op) {
      case MOVN_w:
      case MOVN_x: {
        new_xn_val = ~shifted_imm16;
        if (!is_64_bits) new_xn_val &= kWRegMask;
        break;
      }
      case MOVK_w:
      case MOVK_x: {
        unsigned reg_code = instr->Rd();
        int64_t prev_xn_val = is_64_bits ? xreg(reg_code) : wreg(reg_code);
        new_xn_val = (prev_xn_val & ~(INT64_C(0xffff) << shift)) | shifted_imm16;
        break;
      }
      case MOVZ_w:
      case MOVZ_x: {
        new_xn_val = shifted_imm16;
        break;
      }
      default:
        VIXL_UNREACHABLE();
    }

    // Update the destination register.
    set_xreg(instr->Rd(), new_xn_val);
}

void
Simulator::VisitConditionalSelect(Instruction* instr)
{
    uint64_t new_val = xreg(instr->Rn());

    if (ConditionFailed(static_cast<Condition>(instr->Condition()))) {
        new_val = xreg(instr->Rm());
        switch (instr->Mask(ConditionalSelectMask)) {
          case CSEL_w:
          case CSEL_x: break;
          case CSINC_w:
          case CSINC_x: new_val++; break;
          case CSINV_w:
          case CSINV_x: new_val = ~new_val; break;
          case CSNEG_w:
          case CSNEG_x: new_val = -new_val; break;
          default: VIXL_UNIMPLEMENTED();
        }
    }
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    set_reg(reg_size, instr->Rd(), new_val);
}

void
Simulator::VisitDataProcessing1Source(Instruction* instr)
{
    unsigned dst = instr->Rd();
    unsigned src = instr->Rn();

    switch (instr->Mask(DataProcessing1SourceMask)) {
      case RBIT_w: set_wreg(dst, ReverseBits(wreg(src), kWRegSize)); break;
      case RBIT_x: set_xreg(dst, ReverseBits(xreg(src), kXRegSize)); break;
      case REV16_w: set_wreg(dst, ReverseBytes(wreg(src), Reverse16)); break;
      case REV16_x: set_xreg(dst, ReverseBytes(xreg(src), Reverse16)); break;
      case REV_w: set_wreg(dst, ReverseBytes(wreg(src), Reverse32)); break;
      case REV32_x: set_xreg(dst, ReverseBytes(xreg(src), Reverse32)); break;
      case REV_x: set_xreg(dst, ReverseBytes(xreg(src), Reverse64)); break;
      case CLZ_w: set_wreg(dst, CountLeadingZeros(wreg(src), kWRegSize)); break;
      case CLZ_x: set_xreg(dst, CountLeadingZeros(xreg(src), kXRegSize)); break;
      case CLS_w: {
        set_wreg(dst, CountLeadingSignBits(wreg(src), kWRegSize));
        break;
      }
      case CLS_x: {
        set_xreg(dst, CountLeadingSignBits(xreg(src), kXRegSize));
        break;
      }
      default: VIXL_UNIMPLEMENTED();
    }
}

uint64_t
Simulator::ReverseBits(uint64_t value, unsigned num_bits)
{
    MOZ_ASSERT((num_bits == kWRegSize) || (num_bits == kXRegSize));
    uint64_t result = 0;
    for (unsigned i = 0; i < num_bits; i++) {
        result = (result << 1) | (value & 1);
        value >>= 1;
    }
    return result;
}

uint64_t
Simulator::ReverseBytes(uint64_t value, ReverseByteMode mode)
{
    // Split the 64-bit value into an 8-bit array, where b[0] is the least
    // significant byte, and b[7] is the most significant.
    uint8_t bytes[8];
    uint64_t mask = 0xff00000000000000;
    for (int i = 7; i >= 0; i--) {
        bytes[i] = (value & mask) >> (i * 8);
        mask >>= 8;
    }

    // Permutation tables for REV instructions.
    //  permute_table[Reverse16] is used by REV16_x, REV16_w
    //  permute_table[Reverse32] is used by REV32_x, REV_w
    //  permute_table[Reverse64] is used by REV_x
    JS_STATIC_ASSERT((Reverse16 == 0) && (Reverse32 == 1) && (Reverse64 == 2));
    static const uint8_t permute_table[3][8] = { {6, 7, 4, 5, 2, 3, 0, 1},
                                                 {4, 5, 6, 7, 0, 1, 2, 3},
                                                 {0, 1, 2, 3, 4, 5, 6, 7} };
    uint64_t result = 0;
    for (int i = 0; i < 8; i++) {
        result <<= 8;
        result |= bytes[permute_table[mode][i]];
    }
    return result;
}

void
Simulator::VisitDataProcessing2Source(Instruction* instr)
{
    Shift shift_op = NO_SHIFT;
    int64_t result = 0;
    switch (instr->Mask(DataProcessing2SourceMask)) {
      case SDIV_w: {
        int32_t rn = wreg(instr->Rn());
        int32_t rm = wreg(instr->Rm());
        if ((rn == kWMinInt) && (rm == -1)) {
            result = kWMinInt;
        } else if (rm == 0) {
            // Division by zero can be trapped, but not on A-class processors.
            result = 0;
        } else {
            result = rn / rm;
        }
        break;
      }
      case SDIV_x: {
        int64_t rn = xreg(instr->Rn());
        int64_t rm = xreg(instr->Rm());
        if ((rn == kXMinInt) && (rm == -1)) {
            result = kXMinInt;
        } else if (rm == 0) {
            // Division by zero can be trapped, but not on A-class processors.
            result = 0;
        } else {
            result = rn / rm;
        }
        break;
      }
      case UDIV_w: {
        uint32_t rn = static_cast<uint32_t>(wreg(instr->Rn()));
        uint32_t rm = static_cast<uint32_t>(wreg(instr->Rm()));
        if (rm == 0) {
            // Division by zero can be trapped, but not on A-class processors.
            result = 0;
        } else {
            result = rn / rm;
        }
        break;
      }
      case UDIV_x: {
        uint64_t rn = static_cast<uint64_t>(xreg(instr->Rn()));
        uint64_t rm = static_cast<uint64_t>(xreg(instr->Rm()));
        if (rm == 0) {
            // Division by zero can be trapped, but not on A-class processors.
            result = 0;
        } else {
            result = rn / rm;
        }
        break;
      }
      case LSLV_w:
      case LSLV_x: shift_op = LSL; break;
      case LSRV_w:
      case LSRV_x: shift_op = LSR; break;
      case ASRV_w:
      case ASRV_x: shift_op = ASR; break;
      case RORV_w:
      case RORV_x: shift_op = ROR; break;
      default: VIXL_UNIMPLEMENTED();
    }

    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    if (shift_op != NO_SHIFT) {
        // Shift distance encoded in the least-significant five/six bits of the
        // register.
        int mask = (instr->SixtyFourBits() == 1) ? 0x3f : 0x1f;
        unsigned shift = wreg(instr->Rm()) & mask;
        result = ShiftOperand(reg_size, reg(reg_size, instr->Rn()), shift_op, shift);
    }
    set_reg(reg_size, instr->Rd(), result);
}

// The algorithm used is adapted from the one described in section 8.2 of
//   Hacker's Delight, by Henry S. Warren, Jr.
// It assumes that a right shift on a signed integer is an arithmetic shift.
static int64_t
MultiplyHighSigned(int64_t u, int64_t v)
{
    uint64_t u0, v0, w0;
    int64_t u1, v1, w1, w2, t;

    u0 = u & 0xffffffff;
    u1 = u >> 32;
    v0 = v & 0xffffffff;
    v1 = v >> 32;

    w0 = u0 * v0;
    t = u1 * v0 + (w0 >> 32);
    w1 = t & 0xffffffff;
    w2 = t >> 32;
    w1 = u0 * v1 + w1;

    return u1 * v1 + w2 + (w1 >> 32);
}

void
Simulator::VisitDataProcessing3Source(Instruction* instr)
{
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;

    int64_t result = 0;
    // Extract and sign- or zero-extend 32-bit arguments for widening operations.
    uint64_t rn_u32 = reg<uint32_t>(instr->Rn());
    uint64_t rm_u32 = reg<uint32_t>(instr->Rm());
    int64_t rn_s32 = reg<int32_t>(instr->Rn());
    int64_t rm_s32 = reg<int32_t>(instr->Rm());
    switch (instr->Mask(DataProcessing3SourceMask)) {
      case MADD_w:
      case MADD_x:
        result = xreg(instr->Ra()) + (xreg(instr->Rn()) * xreg(instr->Rm()));
        break;
      case MSUB_w:
      case MSUB_x:
        result = xreg(instr->Ra()) - (xreg(instr->Rn()) * xreg(instr->Rm()));
        break;
      case SMADDL_x: result = xreg(instr->Ra()) + (rn_s32 * rm_s32); break;
      case SMSUBL_x: result = xreg(instr->Ra()) - (rn_s32 * rm_s32); break;
      case UMADDL_x: result = xreg(instr->Ra()) + (rn_u32 * rm_u32); break;
      case UMSUBL_x: result = xreg(instr->Ra()) - (rn_u32 * rm_u32); break;
      case SMULH_x:
        result = MultiplyHighSigned(xreg(instr->Rn()), xreg(instr->Rm()));
        break;
      default: VIXL_UNIMPLEMENTED();
    }
    set_reg(reg_size, instr->Rd(), result);
}

void
Simulator::VisitBitfield(Instruction* instr)
{
    unsigned reg_size = instr->SixtyFourBits() ? kXRegSize : kWRegSize;
    int64_t reg_mask = instr->SixtyFourBits() ? kXRegMask : kWRegMask;
    int64_t R = instr->ImmR();
    int64_t S = instr->ImmS();
    int64_t diff = S - R;
    int64_t mask;
    if (diff >= 0) {
        mask = (diff < (reg_size - 1)) ? (INT64_C(1) << (diff + 1)) - 1
            : reg_mask;
    } else {
        mask = (INT64_C(1) << (S + 1)) - 1;
        mask = (static_cast<uint64_t>(mask) >> R) | (mask << (reg_size - R));
        diff += reg_size;
    }

    // inzero indicates if the extracted bitfield is inserted into the
    // destination register value or in zero.
    // If extend is true, extend the sign of the extracted bitfield.
    bool inzero = false;
    bool extend = false;
    switch (instr->Mask(BitfieldMask)) {
      case BFM_x:
      case BFM_w:
        break;
      case SBFM_x:
      case SBFM_w:
        inzero = true;
        extend = true;
        break;
      case UBFM_x:
      case UBFM_w:
        inzero = true;
        break;
      default:
        VIXL_UNIMPLEMENTED();
    }

    int64_t dst = inzero ? 0 : reg(reg_size, instr->Rd());
    int64_t src = reg(reg_size, instr->Rn());
    // Rotate source bitfield into place.
    int64_t result = (static_cast<uint64_t>(src) >> R) | (src << (reg_size - R));
    // Determine the sign extension.
    int64_t topbits = ((INT64_C(1) << (reg_size - diff - 1)) - 1) << (diff + 1);
    int64_t signbits = extend && ((src >> S) & 1) ? topbits : 0;

    // Merge sign extension, dest/zero and bitfield.
    result = signbits | (result & mask) | (dst & ~mask);

    set_reg(reg_size, instr->Rd(), result);
}

void
Simulator::VisitExtract(Instruction* instr)
{
    unsigned lsb = instr->ImmS();
    unsigned reg_size = (instr->SixtyFourBits() == 1) ? kXRegSize
        : kWRegSize;
    uint64_t low_res = static_cast<uint64_t>(reg(reg_size, instr->Rm())) >> lsb;
    uint64_t high_res = (lsb == 0) ? 0 : reg(reg_size, instr->Rn()) << (reg_size - lsb);
    set_reg(reg_size, instr->Rd(), low_res | high_res);
}

void
Simulator::VisitFPImmediate(Instruction* instr)
{
    AssertSupportedFPCR();

    unsigned dest = instr->Rd();
    switch (instr->Mask(FPImmediateMask)) {
      case FMOV_s_imm: set_sreg(dest, instr->ImmFP32()); break;
      case FMOV_d_imm: set_dreg(dest, instr->ImmFP64()); break;
      default: VIXL_UNREACHABLE();
    }
}


void
Simulator::VisitFPIntegerConvert(Instruction* instr)
{
    AssertSupportedFPCR();

    unsigned dst = instr->Rd();
    unsigned src = instr->Rn();

    FPRounding round = RMode();

    switch (instr->Mask(FPIntegerConvertMask)) {
      case FCVTAS_ws: set_wreg(dst, FPToInt32(sreg(src), FPTieAway)); break;
      case FCVTAS_xs: set_xreg(dst, FPToInt64(sreg(src), FPTieAway)); break;
      case FCVTAS_wd: set_wreg(dst, FPToInt32(dreg(src), FPTieAway)); break;
      case FCVTAS_xd: set_xreg(dst, FPToInt64(dreg(src), FPTieAway)); break;
      case FCVTAU_ws: set_wreg(dst, FPToUInt32(sreg(src), FPTieAway)); break;
      case FCVTAU_xs: set_xreg(dst, FPToUInt64(sreg(src), FPTieAway)); break;
      case FCVTAU_wd: set_wreg(dst, FPToUInt32(dreg(src), FPTieAway)); break;
      case FCVTAU_xd: set_xreg(dst, FPToUInt64(dreg(src), FPTieAway)); break;
      case FCVTMS_ws:
        set_wreg(dst, FPToInt32(sreg(src), FPNegativeInfinity));
        break;
      case FCVTMS_xs:
        set_xreg(dst, FPToInt64(sreg(src), FPNegativeInfinity));
        break;
      case FCVTMS_wd:
        set_wreg(dst, FPToInt32(dreg(src), FPNegativeInfinity));
        break;
      case FCVTMS_xd:
        set_xreg(dst, FPToInt64(dreg(src), FPNegativeInfinity));
        break;
      case FCVTMU_ws:
        set_wreg(dst, FPToUInt32(sreg(src), FPNegativeInfinity));
        break;
      case FCVTMU_xs:
        set_xreg(dst, FPToUInt64(sreg(src), FPNegativeInfinity));
        break;
      case FCVTMU_wd:
        set_wreg(dst, FPToUInt32(dreg(src), FPNegativeInfinity));
        break;
      case FCVTMU_xd:
        set_xreg(dst, FPToUInt64(dreg(src), FPNegativeInfinity));
        break;

      case FCVTPS_ws:
        set_wreg(dst, FPToInt32(sreg(src), FPPositiveInfinity));
        break;
      case FCVTPS_xs:
        set_xreg(dst, FPToInt64(sreg(src), FPPositiveInfinity));
        break;
      case FCVTPS_wd:
        set_wreg(dst, FPToInt32(dreg(src), FPPositiveInfinity));
        break;
      case FCVTPS_xd:
        set_xreg(dst, FPToInt64(dreg(src), FPPositiveInfinity));
        break;
      case FCVTPU_ws:
        set_wreg(dst, FPToUInt32(sreg(src), FPPositiveInfinity));
        break;
      case FCVTPU_xs:
        set_xreg(dst, FPToUInt64(sreg(src), FPPositiveInfinity));
        break;
      case FCVTPU_wd:
        set_wreg(dst, FPToUInt32(dreg(src), FPPositiveInfinity));
        break;
      case FCVTPU_xd:
        set_xreg(dst, FPToUInt64(dreg(src), FPPositiveInfinity));
        break;

      case FCVTNS_ws: set_wreg(dst, FPToInt32(sreg(src), FPTieEven)); break;
      case FCVTNS_xs: set_xreg(dst, FPToInt64(sreg(src), FPTieEven)); break;
      case FCVTNS_wd: set_wreg(dst, FPToInt32(dreg(src), FPTieEven)); break;
      case FCVTNS_xd: set_xreg(dst, FPToInt64(dreg(src), FPTieEven)); break;
      case FCVTNU_ws: set_wreg(dst, FPToUInt32(sreg(src), FPTieEven)); break;
      case FCVTNU_xs: set_xreg(dst, FPToUInt64(sreg(src), FPTieEven)); break;
      case FCVTNU_wd: set_wreg(dst, FPToUInt32(dreg(src), FPTieEven)); break;
      case FCVTNU_xd: set_xreg(dst, FPToUInt64(dreg(src), FPTieEven)); break;
      case FCVTZS_ws: set_wreg(dst, FPToInt32(sreg(src), FPZero)); break;
      case FCVTZS_xs: set_xreg(dst, FPToInt64(sreg(src), FPZero)); break;
      case FCVTZS_wd: set_wreg(dst, FPToInt32(dreg(src), FPZero)); break;
      case FCVTZS_xd: set_xreg(dst, FPToInt64(dreg(src), FPZero)); break;
      case FCVTZU_ws: set_wreg(dst, FPToUInt32(sreg(src), FPZero)); break;
      case FCVTZU_xs: set_xreg(dst, FPToUInt64(sreg(src), FPZero)); break;
      case FCVTZU_wd: set_wreg(dst, FPToUInt32(dreg(src), FPZero)); break;
      case FCVTZU_xd: set_xreg(dst, FPToUInt64(dreg(src), FPZero)); break;
      case FMOV_ws: set_wreg(dst, sreg_bits(src)); break;
      case FMOV_xd: set_xreg(dst, dreg_bits(src)); break;
      case FMOV_sw: set_sreg_bits(dst, wreg(src)); break;
      case FMOV_dx: set_dreg_bits(dst, xreg(src)); break;

      // A 32-bit input can be handled in the same way as a 64-bit input, since
      // the sign- or zero-extension will not affect the conversion.
      case SCVTF_dx: set_dreg(dst, FixedToDouble(xreg(src), 0, round)); break;
      case SCVTF_dw: set_dreg(dst, FixedToDouble(wreg(src), 0, round)); break;
      case UCVTF_dx: set_dreg(dst, UFixedToDouble(xreg(src), 0, round)); break;
      case UCVTF_dw: {
        set_dreg(dst, UFixedToDouble(static_cast<uint32_t>(wreg(src)), 0, round));
        break;
      }
      case SCVTF_sx: set_sreg(dst, FixedToFloat(xreg(src), 0, round)); break;
      case SCVTF_sw: set_sreg(dst, FixedToFloat(wreg(src), 0, round)); break;
      case UCVTF_sx: set_sreg(dst, UFixedToFloat(xreg(src), 0, round)); break;
      case UCVTF_sw: {
        set_sreg(dst, UFixedToFloat(static_cast<uint32_t>(wreg(src)), 0, round));
        break;
      }

      default: VIXL_UNREACHABLE();
    }
}

void
Simulator::VisitFPFixedPointConvert(Instruction* instr)
{
    AssertSupportedFPCR();

    unsigned dst = instr->Rd();
    unsigned src = instr->Rn();
    int fbits = 64 - instr->FPScale();

    FPRounding round = RMode();

    switch (instr->Mask(FPFixedPointConvertMask)) {
      // A 32-bit input can be handled in the same way as a 64-bit input, since
      // the sign- or zero-extension will not affect the conversion.
      case SCVTF_dx_fixed:
        set_dreg(dst, FixedToDouble(xreg(src), fbits, round));
        break;
      case SCVTF_dw_fixed:
        set_dreg(dst, FixedToDouble(wreg(src), fbits, round));
        break;
      case UCVTF_dx_fixed:
        set_dreg(dst, UFixedToDouble(xreg(src), fbits, round));
        break;
      case UCVTF_dw_fixed: {
        set_dreg(dst, UFixedToDouble(static_cast<uint32_t>(wreg(src)), fbits, round));
        break;
      }
      case SCVTF_sx_fixed:
        set_sreg(dst, FixedToFloat(xreg(src), fbits, round));
        break;
      case SCVTF_sw_fixed:
        set_sreg(dst, FixedToFloat(wreg(src), fbits, round));
        break;
      case UCVTF_sx_fixed:
        set_sreg(dst, UFixedToFloat(xreg(src), fbits, round));
        break;
      case UCVTF_sw_fixed: {
        set_sreg(dst, UFixedToFloat(static_cast<uint32_t>(wreg(src)), fbits, round));
        break;
      }
      default: VIXL_UNREACHABLE();
    }
}

int32_t
Simulator::FPToInt32(double value, FPRounding rmode)
{
    value = FPRoundInt(value, rmode);
    if (value >= kWMaxInt)
        return kWMaxInt;
    if (value < kWMinInt)
        return kWMinInt;
    return mozilla::IsNaN(value) ? 0 : static_cast<int32_t>(value);
}

int64_t
Simulator::FPToInt64(double value, FPRounding rmode)
{
    value = FPRoundInt(value, rmode);
    if (value >= kXMaxInt)
        return kXMaxInt;
    if (value < kXMinInt)
        return kXMinInt;
    return mozilla::IsNaN(value) ? 0 : static_cast<int64_t>(value);
}

uint32_t
Simulator::FPToUInt32(double value, FPRounding rmode)
{
    value = FPRoundInt(value, rmode);
    if (value >= kWMaxUInt)
        return kWMaxUInt;
    if (value < 0.0)
        return 0;
    return mozilla::IsNaN(value) ? 0 : static_cast<uint32_t>(value);
}

uint64_t
Simulator::FPToUInt64(double value, FPRounding rmode)
{
    value = FPRoundInt(value, rmode);
    if (value >= kXMaxUInt)
        return kXMaxUInt;
    if (value < 0.0)
        return 0;
    return mozilla::IsNaN(value) ? 0 : static_cast<uint64_t>(value);
}

void
Simulator::VisitFPCompare(Instruction* instr)
{
    AssertSupportedFPCR();

    switch (instr->Mask(FPCompareMask)) {
      case FCMP_s: FPCompare(sreg(instr->Rn()), sreg(instr->Rm())); break;
      case FCMP_d: FPCompare(dreg(instr->Rn()), dreg(instr->Rm())); break;
      case FCMP_s_zero: FPCompare(sreg(instr->Rn()), 0.0f); break;
      case FCMP_d_zero: FPCompare(dreg(instr->Rn()), 0.0); break;
      default: VIXL_UNIMPLEMENTED();
    }
}

void
Simulator::VisitFPConditionalCompare(Instruction* instr)
{
    AssertSupportedFPCR();

    switch (instr->Mask(FPConditionalCompareMask)) {
      case FCCMP_s:
        if (ConditionPassed(instr->Condition()))
            FPCompare(sreg(instr->Rn()), sreg(instr->Rm()));
        else
            nzcv().SetFlags(instr->Nzcv());
        break;
      case FCCMP_d:
        if (ConditionPassed(instr->Condition()))
            FPCompare(dreg(instr->Rn()), dreg(instr->Rm()));
        else
            nzcv().SetFlags(instr->Nzcv());
        break;
      default: VIXL_UNIMPLEMENTED();
    }
}

void
Simulator::VisitFPConditionalSelect(Instruction* instr)
{
    AssertSupportedFPCR();

    Instr selected;
    if (ConditionPassed(instr->Condition()))
        selected = instr->Rn();
    else
        selected = instr->Rm();

    switch (instr->Mask(FPConditionalSelectMask)) {
      case FCSEL_s: set_sreg(instr->Rd(), sreg(selected)); break;
      case FCSEL_d: set_dreg(instr->Rd(), dreg(selected)); break;
      default: VIXL_UNIMPLEMENTED();
    }
}

void
Simulator::VisitFPDataProcessing1Source(Instruction* instr)
{
    AssertSupportedFPCR();

    unsigned fd = instr->Rd();
    unsigned fn = instr->Rn();

    switch (instr->Mask(FPDataProcessing1SourceMask)) {
      case FMOV_s: set_sreg(fd, sreg(fn)); break;
      case FMOV_d: set_dreg(fd, dreg(fn)); break;
      case FABS_s: set_sreg(fd, fabsf(sreg(fn))); break;
      case FABS_d: set_dreg(fd, fabs(dreg(fn))); break;
      case FNEG_s: set_sreg(fd, -sreg(fn)); break;
      case FNEG_d: set_dreg(fd, -dreg(fn)); break;
      case FSQRT_s: set_sreg(fd, FPSqrt(sreg(fn))); break;
      case FSQRT_d: set_dreg(fd, FPSqrt(dreg(fn))); break;
      case FRINTA_s: set_sreg(fd, FPRoundInt(sreg(fn), FPTieAway)); break;
      case FRINTA_d: set_dreg(fd, FPRoundInt(dreg(fn), FPTieAway)); break;
      case FRINTM_s: set_sreg(fd, FPRoundInt(sreg(fn), FPNegativeInfinity)); break;
      case FRINTM_d: set_dreg(fd, FPRoundInt(dreg(fn), FPNegativeInfinity)); break;
      case FRINTN_s: set_sreg(fd, FPRoundInt(sreg(fn), FPTieEven)); break;
      case FRINTN_d: set_dreg(fd, FPRoundInt(dreg(fn), FPTieEven)); break;
      case FRINTZ_s: set_sreg(fd, FPRoundInt(sreg(fn), FPZero)); break;
      case FRINTZ_d: set_dreg(fd, FPRoundInt(dreg(fn), FPZero)); break;
      case FCVT_ds: set_dreg(fd, FPToDouble(sreg(fn))); break;
      case FCVT_sd: set_sreg(fd, FPToFloat(dreg(fn), FPTieEven)); break;
      default: VIXL_UNIMPLEMENTED();
    }
}

// Assemble the specified IEEE-754 components into the target type and apply
// appropriate rounding.
//  sign:     0 = positive, 1 = negative
//  exponent: Unbiased IEEE-754 exponent.
//  mantissa: The mantissa of the input. The top bit (which is not encoded for
//            normal IEEE-754 values) must not be omitted. This bit has the
//            value 'pow(2, exponent)'.
//
// The input value is assumed to be a normalized value. That is, the input may
// not be infinity or NaN. If the source value is subnormal, it must be
// normalized before calling this function such that the highest set bit in the
// mantissa has the value 'pow(2, exponent)'.
//
// Callers should use FPRoundToFloat or FPRoundToDouble directly, rather than
// calling a templated FPRound.
template <class T, int ebits, int mbits>
static T
FPRound(int64_t sign, int64_t exponent, uint64_t mantissa, FPRounding round_mode)
{
    MOZ_ASSERT((sign == 0) || (sign == 1));

    // Only the FPTieEven rounding mode is implemented.
    MOZ_ASSERT(round_mode == FPTieEven);
    USEARG(round_mode);

    // Rounding can promote subnormals to normals, and normals to infinities. For
    // example, a double with exponent 127 (FLT_MAX_EXP) would appear to be
    // encodable as a float, but rounding based on the low-order mantissa bits
    // could make it overflow. With ties-to-even rounding, this value would become
    // an infinity.

    // ---- Rounding Method ----
    //
    // The exponent is irrelevant in the rounding operation, so we treat the
    // lowest-order bit that will fit into the result ('onebit') as having
    // the value '1'. Similarly, the highest-order bit that won't fit into
    // the result ('halfbit') has the value '0.5'. The 'point' sits between
    // 'onebit' and 'halfbit':
    //
    //            These bits fit into the result.
    //               |---------------------|
    //  mantissa = 0bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    //                                     ||
    //                                    / |
    //                                   /  halfbit
    //                               onebit
    //
    // For subnormal outputs, the range of representable bits is smaller and
    // the position of onebit and halfbit depends on the exponent of the
    // input, but the method is otherwise similar.
    //
    //   onebit(frac)
    //     |
    //     | halfbit(frac)          halfbit(adjusted)
    //     | /                      /
    //     | |                      |
    //  0b00.0 (exact)      -> 0b00.0 (exact)                    -> 0b00
    //  0b00.0...           -> 0b00.0...                         -> 0b00
    //  0b00.1 (exact)      -> 0b00.0111..111                    -> 0b00
    //  0b00.1...           -> 0b00.1...                         -> 0b01
    //  0b01.0 (exact)      -> 0b01.0 (exact)                    -> 0b01
    //  0b01.0...           -> 0b01.0...                         -> 0b01
    //  0b01.1 (exact)      -> 0b01.1 (exact)                    -> 0b10
    //  0b01.1...           -> 0b01.1...                         -> 0b10
    //  0b10.0 (exact)      -> 0b10.0 (exact)                    -> 0b10
    //  0b10.0...           -> 0b10.0...                         -> 0b10
    //  0b10.1 (exact)      -> 0b10.0111..111                    -> 0b10
    //  0b10.1...           -> 0b10.1...                         -> 0b11
    //  0b11.0 (exact)      -> 0b11.0 (exact)                    -> 0b11
    //  ...                   /             |                      /   |
    //                       /              |                     /    |
    //                                                           /     |
    // adjusted = frac - (halfbit(mantissa) & ~onebit(frac));   /      |
    //
    //                   mantissa = (mantissa >> shift) + halfbit(adjusted);

    static const int mantissa_offset = 0;
    static const int exponent_offset = mantissa_offset + mbits;
    static const int sign_offset = exponent_offset + ebits;
    MOZ_ASSERT(sign_offset == (sizeof(T) * 8 - 1));

    // Bail out early for zero inputs.
    if (mantissa == 0)
        return sign << sign_offset;

    // If all bits in the exponent are set, the value is infinite or NaN.
    // This is true for all binary IEEE-754 formats.
    static const int infinite_exponent = (1 << ebits) - 1;
    static const int max_normal_exponent = infinite_exponent - 1;

    // Apply the exponent bias to encode it for the result. Doing this early makes
    // it easy to detect values that will be infinite or subnormal.
    exponent += max_normal_exponent >> 1;

    if (exponent > max_normal_exponent) {
        // Overflow: The input is too large for the result type to represent. The
        // FPTieEven rounding mode handles overflows using infinities.
        exponent = infinite_exponent;
        mantissa = 0;
        return (sign << sign_offset) |
               (exponent << exponent_offset) |
               (mantissa << mantissa_offset);
    }

    // Calculate the shift required to move the top mantissa bit to the proper
    // place in the destination type.
    const int highest_significant_bit = 63 - CountLeadingZeros(mantissa, 64);
    int shift = highest_significant_bit - mbits;

    if (exponent <= 0) {
        // The output will be subnormal (before rounding).

        // For subnormal outputs, the shift must be adjusted by the exponent. The +1
        // is necessary because the exponent of a subnormal value (encoded as 0) is
        // the same as the exponent of the smallest normal value (encoded as 1).
        shift += -exponent + 1;

        // Handle inputs that would produce a zero output.
        //
        // Shifts higher than highest_significant_bit+1 will always produce a zero
        // result. A shift of exactly highest_significant_bit+1 might produce a
        // non-zero result after rounding.
        if (shift > (highest_significant_bit + 1)) {
            // The result will always be +/-0.0.
            return sign << sign_offset;
        }

        // Properly encode the exponent for a subnormal output.
        exponent = 0;
    } else {
        // Clear the topmost mantissa bit, since this is not encoded in IEEE-754
        // normal values.
        mantissa &= ~(UINT64_C(1) << highest_significant_bit);
    }

    if (shift > 0) {
        // We have to shift the mantissa to the right. Some precision is lost, so we
        // need to apply rounding.
        uint64_t onebit_mantissa = (mantissa >> (shift)) & 1;
        uint64_t halfbit_mantissa = (mantissa >> (shift-1)) & 1;
        uint64_t adjusted = mantissa - (halfbit_mantissa & ~onebit_mantissa);
        T halfbit_adjusted = (adjusted >> (shift-1)) & 1;

        T result = (sign << sign_offset) |
                   (exponent << exponent_offset) |
                   ((mantissa >> shift) << mantissa_offset);

        // A very large mantissa can overflow during rounding. If this happens, the
        // exponent should be incremented and the mantissa set to 1.0 (encoded as
        // 0). Applying halfbit_adjusted after assembling the float has the nice
        // side-effect that this case is handled for free.
        //
        // This also handles cases where a very large finite value overflows to
        // infinity, or where a very large subnormal value overflows to become
        // normal.
        return result + halfbit_adjusted;
    }

    // We have to shift the mantissa to the left (or not at all). The input
    // mantissa is exactly representable in the output mantissa, so apply no
    // rounding correction.
    return (sign << sign_offset) |
        (exponent << exponent_offset) |
        ((mantissa << -shift) << mantissa_offset);
}

// See FPRound for a description of this function.
static inline double
FPRoundToDouble(int64_t sign, int64_t exponent, uint64_t mantissa, FPRounding round_mode)
{
    int64_t bits =
        FPRound<int64_t, kDoubleExponentBits, kDoubleMantissaBits>(sign,
                                                                   exponent,
                                                                   mantissa,
                                                                   round_mode);
    return rawbits_to_double(bits);
}

// See FPRound for a description of this function.
static inline float
FPRoundToFloat(int64_t sign, int64_t exponent, uint64_t mantissa, FPRounding round_mode)
{
    int32_t bits =
        FPRound<int32_t, kFloatExponentBits, kFloatMantissaBits>(sign,
                                                                 exponent,
                                                                 mantissa,
                                                                 round_mode);
    return rawbits_to_float(bits);
}

double
Simulator::FixedToDouble(int64_t src, int fbits, FPRounding round)
{
    if (src >= 0)
        return UFixedToDouble(src, fbits, round);

    // This works for all negative values, including INT64_MIN.
    return -UFixedToDouble(-src, fbits, round);
}

double
Simulator::UFixedToDouble(uint64_t src, int fbits, FPRounding round)
{
    // An input of 0 is a special case because the result is effectively
    // subnormal: The exponent is encoded as 0 and there is no implicit 1 bit.
    if (src == 0)
        return 0.0;

    // Calculate the exponent. The highest significant bit will have the value
    // 2^exponent.
    const int highest_significant_bit = 63 - CountLeadingZeros(src, 64);
    const int64_t exponent = highest_significant_bit - fbits;

    return FPRoundToDouble(0, exponent, src, round);
}

float
Simulator::FixedToFloat(int64_t src, int fbits, FPRounding round)
{
    if (src >= 0)
        return UFixedToFloat(src, fbits, round);

    // This works for all negative values, including INT64_MIN.
    return -UFixedToFloat(-src, fbits, round);
}


float
Simulator::UFixedToFloat(uint64_t src, int fbits, FPRounding round)
{
    // An input of 0 is a special case because the result is effectively
    // subnormal: The exponent is encoded as 0 and there is no implicit 1 bit.
    if (src == 0)
        return 0.0f;

    // Calculate the exponent. The highest significant bit will have the value
    // 2^exponent.
    const int highest_significant_bit = 63 - CountLeadingZeros(src, 64);
    const int32_t exponent = highest_significant_bit - fbits;

    return FPRoundToFloat(0, exponent, src, round);
}

double
Simulator::FPRoundInt(double value, FPRounding round_mode)
{
    if ((value == 0.0) || (value == kFP64PositiveInfinity) || (value == kFP64NegativeInfinity))
        return value;

    if (mozilla::IsNaN(value))
        return FPProcessNaN(value);

    double int_result = floor(value);
    double error = value - int_result;
    switch (round_mode) {
      case FPTieAway: {
        // Take care of correctly handling the range ]-0.5, -0.0], which must
        // yield -0.0.
        if ((-0.5 < value) && (value < 0.0)) {
            int_result = -0.0;

        } else if ((error > 0.5) || ((error == 0.5) && (int_result >= 0.0))) {
            // If the error is greater than 0.5, or is equal to 0.5 and the integer
            // result is positive, round up.
            int_result++;
        }
        break;
      }
      case FPTieEven: {
        // Take care of correctly handling the range [-0.5, -0.0], which must
        // yield -0.0.
        if ((-0.5 <= value) && (value < 0.0)) {
            int_result = -0.0;

        // If the error is greater than 0.5, or is equal to 0.5 and the integer
        // result is odd, round up.
        } else if ((error > 0.5) || ((error == 0.5) && (fmod(int_result, 2) != 0))) {
            int_result++;
        }
        break;
      }
      case FPZero: {
        // If value>0 then we take floor(value)
        // otherwise, ceil(value).
        if (value < 0)
            int_result = ceil(value);
        break;
      }
      case FPNegativeInfinity: {
        // We always use floor(value).
        break;
      }
      case FPPositiveInfinity: {
          // We always use ceil(value).
          // Take care of correctly handling the range ]-1.0, -0.0], which must
          // yield -0.0.
          if ((-1.0 < value) && (value < 0.0)) {
              int_result = -0.0;

              // If the error is non-zero, round up.
          } else if (error > 0.0) {
              int_result++;
          }
        break;
      }

      default: VIXL_UNIMPLEMENTED();
    }
    return int_result;
}

double
Simulator::FPToDouble(float value)
{
    switch (std::fpclassify(value)) {
      case FP_NAN: {
        if (DN()) return kFP64DefaultNaN;

        // Convert NaNs as the processor would:
        //  - The sign is propagated.
        //  - The payload (mantissa) is transferred entirely, except that the top
        //    bit is forced to '1', making the result a quiet NaN. The unused
        //    (low-order) payload bits are set to 0.
        uint32_t raw = float_to_rawbits(value);

        uint64_t sign = raw >> 31;
        uint64_t exponent = (1 << 11) - 1;
        uint64_t payload = unsigned_bitextract_64(21, 0, raw);
        payload <<= (52 - 23);  // The unused low-order bits should be 0.
        payload |= (UINT64_C(1) << 51);  // Force a quiet NaN.

        return rawbits_to_double((sign << 63) | (exponent << 52) | payload);
      }

      case FP_ZERO:
      case FP_NORMAL:
      case FP_SUBNORMAL:
      case FP_INFINITE: {
        // All other inputs are preserved in a standard cast, because every value
        // representable using an IEEE-754 float is also representable using an
        // IEEE-754 double.
        return static_cast<double>(value);
      }
    }

    VIXL_UNREACHABLE();
    return static_cast<double>(value);
}

float
Simulator::FPToFloat(double value, FPRounding round_mode)
{
    // Only the FPTieEven rounding mode is implemented.
    MOZ_ASSERT(round_mode == FPTieEven);
    USEARG(round_mode);

    switch (std::fpclassify(value)) {
      case FP_NAN: {
        if (DN()) return kFP32DefaultNaN;

        // Convert NaNs as the processor would:
        //  - The sign is propagated.
        //  - The payload (mantissa) is transferred as much as possible, except
        //    that the top bit is forced to '1', making the result a quiet NaN.
        uint64_t raw = double_to_rawbits(value);

        uint32_t sign = raw >> 63;
        uint32_t exponent = (1 << 8) - 1;
        uint32_t payload = unsigned_bitextract_64(50, 52 - 23, raw);
        payload |= (1 << 22);   // Force a quiet NaN.

        return rawbits_to_float((sign << 31) | (exponent << 23) | payload);
      }

      case FP_ZERO:
      case FP_INFINITE: {
        // In a C++ cast, any value representable in the target type will be
        // unchanged. This is always the case for +/-0.0 and infinities.
        return static_cast<float>(value);
      }

      case FP_NORMAL:
      case FP_SUBNORMAL: {
        // Convert double-to-float as the processor would, assuming that FPCR.FZ
        // (flush-to-zero) is not set.
        uint64_t raw = double_to_rawbits(value);
        // Extract the IEEE-754 double components.
        uint32_t sign = raw >> 63;
        // Extract the exponent and remove the IEEE-754 encoding bias.
        int32_t exponent = unsigned_bitextract_64(62, 52, raw) - 1023;
        // Extract the mantissa and add the implicit '1' bit.
        uint64_t mantissa = unsigned_bitextract_64(51, 0, raw);
        if (std::fpclassify(value) == FP_NORMAL)
            mantissa |= (UINT64_C(1) << 52);
        return FPRoundToFloat(sign, exponent, mantissa, round_mode);
      }
    }

    VIXL_UNREACHABLE();
    return value;
}

void
Simulator::VisitFPDataProcessing2Source(Instruction* instr)
{
    AssertSupportedFPCR();

    unsigned fd = instr->Rd();
    unsigned fn = instr->Rn();
    unsigned fm = instr->Rm();

    // Fmaxnm and Fminnm have special NaN handling.
    switch (instr->Mask(FPDataProcessing2SourceMask)) {
      case FMAXNM_s: set_sreg(fd, FPMaxNM(sreg(fn), sreg(fm))); return;
      case FMAXNM_d: set_dreg(fd, FPMaxNM(dreg(fn), dreg(fm))); return;
      case FMINNM_s: set_sreg(fd, FPMinNM(sreg(fn), sreg(fm))); return;
      case FMINNM_d: set_dreg(fd, FPMinNM(dreg(fn), dreg(fm))); return;
      default:
        break;    // Fall through.
    }

    if (FPProcessNaNs(instr)) return;

    switch (instr->Mask(FPDataProcessing2SourceMask)) {
      case FADD_s: set_sreg(fd, FPAdd(sreg(fn), sreg(fm))); break;
      case FADD_d: set_dreg(fd, FPAdd(dreg(fn), dreg(fm))); break;
      case FSUB_s: set_sreg(fd, FPSub(sreg(fn), sreg(fm))); break;
      case FSUB_d: set_dreg(fd, FPSub(dreg(fn), dreg(fm))); break;
      case FMUL_s: set_sreg(fd, FPMul(sreg(fn), sreg(fm))); break;
      case FMUL_d: set_dreg(fd, FPMul(dreg(fn), dreg(fm))); break;
      case FDIV_s: set_sreg(fd, FPDiv(sreg(fn), sreg(fm))); break;
      case FDIV_d: set_dreg(fd, FPDiv(dreg(fn), dreg(fm))); break;
      case FMAX_s: set_sreg(fd, FPMax(sreg(fn), sreg(fm))); break;
      case FMAX_d: set_dreg(fd, FPMax(dreg(fn), dreg(fm))); break;
      case FMIN_s: set_sreg(fd, FPMin(sreg(fn), sreg(fm))); break;
      case FMIN_d: set_dreg(fd, FPMin(dreg(fn), dreg(fm))); break;
      case FMAXNM_s:
      case FMAXNM_d:
      case FMINNM_s:
      case FMINNM_d:
        // These were handled before the standard FPProcessNaNs() stage.
        VIXL_UNREACHABLE();
      default: VIXL_UNIMPLEMENTED();
    }
}


void
Simulator::VisitFPDataProcessing3Source(Instruction* instr)
{
    AssertSupportedFPCR();

    unsigned fd = instr->Rd();
    unsigned fn = instr->Rn();
    unsigned fm = instr->Rm();
    unsigned fa = instr->Ra();

    switch (instr->Mask(FPDataProcessing3SourceMask)) {
      // fd = fa +/- (fn * fm)
      case FMADD_s: set_sreg(fd, FPMulAdd(sreg(fa), sreg(fn), sreg(fm))); break;
      case FMSUB_s: set_sreg(fd, FPMulAdd(sreg(fa), -sreg(fn), sreg(fm))); break;
      case FMADD_d: set_dreg(fd, FPMulAdd(dreg(fa), dreg(fn), dreg(fm))); break;
      case FMSUB_d: set_dreg(fd, FPMulAdd(dreg(fa), -dreg(fn), dreg(fm))); break;
      // Negated variants of the above.
      case FNMADD_s:
        set_sreg(fd, FPMulAdd(-sreg(fa), -sreg(fn), sreg(fm)));
        break;
      case FNMSUB_s:
        set_sreg(fd, FPMulAdd(-sreg(fa), sreg(fn), sreg(fm)));
        break;
      case FNMADD_d:
        set_dreg(fd, FPMulAdd(-dreg(fa), -dreg(fn), dreg(fm)));
        break;
      case FNMSUB_d:
        set_dreg(fd, FPMulAdd(-dreg(fa), dreg(fn), dreg(fm)));
        break;
      default: VIXL_UNIMPLEMENTED();
    }
}

template <typename T>
T
Simulator::FPAdd(T op1, T op2)
{
    // NaNs should be handled elsewhere.
    MOZ_ASSERT(!mozilla::IsNaN(op1) && !mozilla::IsNaN(op2));

    if (isinf(op1) && isinf(op2) && (op1 != op2)) {
        // inf + -inf returns the default NaN.
        FPProcessException();
        return FPDefaultNaN<T>();
    }

    // Other cases should be handled by standard arithmetic.
    return op1 + op2;
}

template <typename T>
T
Simulator::FPDiv(T op1, T op2)
{
    // NaNs should be handled elsewhere.
    MOZ_ASSERT(!mozilla::IsNaN(op1) && !mozilla::IsNaN(op2));

    if ((isinf(op1) && isinf(op2)) || ((op1 == 0.0) && (op2 == 0.0))) {
        // inf / inf and 0.0 / 0.0 return the default NaN.
        FPProcessException();
        return FPDefaultNaN<T>();
    }

    if (op2 == 0.0) FPProcessException();

    // Other cases should be handled by standard arithmetic.
    return op1 / op2;
}

template <typename T>
T
Simulator::FPMax(T a, T b)
{
    // NaNs should be handled elsewhere.
    MOZ_ASSERT(!mozilla::IsNaN(a) && !mozilla::IsNaN(b));

    if ((a == 0.0) && (b == 0.0) && (copysign(1.0, a) != copysign(1.0, b))) {
        // a and b are zero, and the sign differs: return +0.0.
        return 0.0;
    }

    return (a > b) ? a : b;
}

template <typename T>
T
Simulator::FPMaxNM(T a, T b)
{
    if (IsQuietNaN(a) && !IsQuietNaN(b))
        a = kFP64NegativeInfinity;
    else if (!IsQuietNaN(a) && IsQuietNaN(b))
        b = kFP64NegativeInfinity;

    T result = FPProcessNaNs(a, b);
    return mozilla::IsNaN(result) ? result : FPMax(a, b);
}

template <typename T>
T
Simulator::FPMin(T a, T b)
{
    // NaNs should be handled elsewhere.
    MOZ_ASSERT(!mozilla::IsNaN(a) && !mozilla::IsNaN(b));

    if ((a == 0.0) && (b == 0.0) && (copysign(1.0, a) != copysign(1.0, b))) {
        // a and b are zero, and the sign differs: return -0.0.
        return -0.0;
    }

    return (a < b) ? a : b;
}

template <typename T>
T
Simulator::FPMinNM(T a, T b)
{
    if (IsQuietNaN(a) && !IsQuietNaN(b))
        a = kFP64PositiveInfinity;
    else if (!IsQuietNaN(a) && IsQuietNaN(b))
        b = kFP64PositiveInfinity;

    T result = FPProcessNaNs(a, b);
    return mozilla::IsNaN(result) ? result : FPMin(a, b);
}

template <typename T>
T
Simulator::FPMul(T op1, T op2)
{
    // NaNs should be handled elsewhere.
    MOZ_ASSERT(!mozilla::IsNaN(op1) && !mozilla::IsNaN(op2));

    if ((isinf(op1) && (op2 == 0.0)) || (isinf(op2) && (op1 == 0.0))) {
        // inf * 0.0 returns the default NaN.
        FPProcessException();
        return FPDefaultNaN<T>();
    }

    // Other cases should be handled by standard arithmetic.
    return op1 * op2;
}

template<typename T>
T
Simulator::FPMulAdd(T a, T op1, T op2)
{
    T result = FPProcessNaNs3(a, op1, op2);

    T sign_a = copysign(1.0, a);
    T sign_prod = copysign(1.0, op1) * copysign(1.0, op2);
    bool isinf_prod = isinf(op1) || isinf(op2);
    bool operation_generates_nan =
        (isinf(op1) && (op2 == 0.0)) ||                     // inf * 0.0
        (isinf(op2) && (op1 == 0.0)) ||                     // 0.0 * inf
        (isinf(a) && isinf_prod && (sign_a != sign_prod));  // inf - inf

    if (mozilla::IsNaN(result)) {
        // Generated NaNs override quiet NaNs propagated from a.
        if (operation_generates_nan && IsQuietNaN(a)) {
            FPProcessException();
            return FPDefaultNaN<T>();
        }
        return result;
    }

    // If the operation would produce a NaN, return the default NaN.
    if (operation_generates_nan) {
        FPProcessException();
        return FPDefaultNaN<T>();
    }

    // Work around broken fma implementations for exact zero results: The sign of
    // exact 0.0 results is positive unless both a and op1 * op2 are negative.
    if (((op1 == 0.0) || (op2 == 0.0)) && (a == 0.0))
        return ((sign_a < 0) && (sign_prod < 0)) ? -0.0 : 0.0;

    result = FusedMultiplyAdd(op1, op2, a);
    MOZ_ASSERT(!mozilla::IsNaN(result));

    // Work around broken fma implementations for rounded zero results: If a is
    // 0.0, the sign of the result is the sign of op1 * op2 before rounding.
    if ((a == 0.0) && (result == 0.0))
        return copysign(0.0, sign_prod);

    return result;
}

template <typename T>
T
Simulator::FPSub(T op1, T op2)
{
    // NaNs should be handled elsewhere.
    MOZ_ASSERT(!mozilla::IsNaN(op1) && !mozilla::IsNaN(op2));

    if (isinf(op1) && isinf(op2) && (op1 == op2)) {
        // inf - inf returns the default NaN.
        FPProcessException();
        return FPDefaultNaN<T>();
    }

    // Other cases should be handled by standard arithmetic.
    return op1 - op2;
}

template <typename T>
T
Simulator::FPSqrt(T op)
{
    if (mozilla::IsNaN(op))
        return FPProcessNaN(op);

    if (op < 0.0) {
        FPProcessException();
        return FPDefaultNaN<T>();
    }

    return sqrt(op);
}


template <typename T>
T
Simulator::FPProcessNaN(T op)
{
    MOZ_ASSERT(mozilla::IsNaN(op));
    if (IsSignallingNaN(op))
        FPProcessException();
    return DN() ? FPDefaultNaN<T>() : ToQuietNaN(op);
}

template <typename T>
T
Simulator::FPProcessNaNs(T op1, T op2)
{
    if (IsSignallingNaN(op1))
        return FPProcessNaN(op1);
    if (IsSignallingNaN(op2))
        return FPProcessNaN(op2);

    if (mozilla::IsNaN(op1)) {
        MOZ_ASSERT(IsQuietNaN(op1));
        return FPProcessNaN(op1);
    }

    if (mozilla::IsNaN(op2)) {
        MOZ_ASSERT(IsQuietNaN(op2));
        return FPProcessNaN(op2);
    }

    return 0.0;
}

template <typename T>
T
Simulator::FPProcessNaNs3(T op1, T op2, T op3)
{
    if (IsSignallingNaN(op1))
        return FPProcessNaN(op1);
    if (IsSignallingNaN(op2))
        return FPProcessNaN(op2);
    if (IsSignallingNaN(op3))
        return FPProcessNaN(op3);

    if (mozilla::IsNaN(op1)) {
        MOZ_ASSERT(IsQuietNaN(op1));
        return FPProcessNaN(op1);
    }

    if (mozilla::IsNaN(op2)) {
        MOZ_ASSERT(IsQuietNaN(op2));
        return FPProcessNaN(op2);
    }

    if (mozilla::IsNaN(op3)) {
        MOZ_ASSERT(IsQuietNaN(op3));
        return FPProcessNaN(op3);
    }

    return 0.0;
}

bool
Simulator::FPProcessNaNs(Instruction* instr)
{
    unsigned fd = instr->Rd();
    unsigned fn = instr->Rn();
    unsigned fm = instr->Rm();
    bool done = false;

    if (instr->Mask(FP64) == FP64) {
        double result = FPProcessNaNs(dreg(fn), dreg(fm));
        if (mozilla::IsNaN(result)) {
            set_dreg(fd, result);
            done = true;
        }
    } else {
        float result = FPProcessNaNs(sreg(fn), sreg(fm));
        if (mozilla::IsNaN(result)) {
            set_sreg(fd, result);
            done = true;
        }
    }

    return done;
}

void
Simulator::VisitSystem(Instruction* instr)
{
    // Some system instructions hijack their Op and Cp fields to represent a
    // range of immediates instead of indicating a different instruction. This
    // makes the decoding tricky.
    if (instr->Mask(SystemExclusiveMonitorFMask) == SystemExclusiveMonitorFixed) {
        MOZ_ASSERT(instr->Mask(SystemExclusiveMonitorMask) == CLREX);
        switch (instr->Mask(SystemExclusiveMonitorMask)) {
          case CLREX: {
            PrintExclusiveAccessWarning();
            ClearLocalMonitor();
            break;
          }
        }
    } else if (instr->Mask(SystemSysRegFMask) == SystemSysRegFixed) {
        switch (instr->Mask(SystemSysRegMask)) {
          case MRS: {
            switch (instr->ImmSystemRegister()) {
              case NZCV: set_xreg(instr->Rt(), nzcv().RawValue()); break;
              case FPCR: set_xreg(instr->Rt(), fpcr().RawValue()); break;
              default: VIXL_UNIMPLEMENTED();
            }
            break;
          }
          case MSR: {
            switch (instr->ImmSystemRegister()) {
              case NZCV: nzcv().SetRawValue(xreg(instr->Rt())); break;
              case FPCR: fpcr().SetRawValue(xreg(instr->Rt())); break;
              default: VIXL_UNIMPLEMENTED();
            }
            break;
          }
        }
    } else if (instr->Mask(SystemHintFMask) == SystemHintFixed) {
        MOZ_ASSERT(instr->Mask(SystemHintMask) == HINT);
        switch (instr->ImmHint()) {
          case NOP: break;
          default: VIXL_UNIMPLEMENTED();
        }
    } else if (instr->Mask(MemBarrierFMask) == MemBarrierFixed) {
        __sync_synchronize();
    } else {
        VIXL_UNIMPLEMENTED();
    }
}

void
Simulator::VisitException(Instruction* instr)
{
    switch (instr->Mask(ExceptionMask)) {
      case BRK: {
        int lowbit  = ImmException_offset;
        int highbit = ImmException_offset + ImmException_width - 1;
        HostBreakpoint(instr->Bits(highbit, lowbit));
        break;
      }
      case HLT:
        // The Printf pseudo instruction is so useful, we include it in the
        // default simulator.
        if (instr->ImmException() == kPrintfOpcode)
            DoPrintf(instr);
        else
            HostBreakpoint();
        break;
      case SVC:
        // The SVC instruction is hijacked by the JIT as a pseudo-instruction
        // causing the Simulator to execute host-native code for callWithABI.
        if (instr->ImmException() == kCallRtRedirected) {
            VisitCallRedirection(instr);
        } else if (instr->ImmException() == kMarkStackPointer) {
            spStack_.append(xreg(31, Reg31IsStackPointer));
        } else if (instr->ImmException() == kCheckStackPointer) {
            int64_t current = xreg(31, Reg31IsStackPointer);
            int64_t expected = spStack_.popCopy();
            MOZ_ASSERT(current == expected);
        } else {
            // Other uses of the SVC instruction are not handled by the simulator.
            VIXL_UNIMPLEMENTED();
        }
        break;
      default:
        VIXL_UNIMPLEMENTED();
    }
}

// TODO: Share this across platforms. Duplicated in ARM and MIPS code.
typedef int64_t (*Prototype_General0)();
typedef int64_t (*Prototype_General1)(int64_t arg0);
typedef int64_t (*Prototype_General2)(int64_t arg0, int64_t arg1);
typedef int64_t (*Prototype_General3)(int64_t arg0, int64_t arg1, int64_t arg2);
typedef int64_t (*Prototype_General4)(int64_t arg0, int64_t arg1, int64_t arg2, int64_t arg3);
typedef int64_t (*Prototype_General5)(int64_t arg0, int64_t arg1, int64_t arg2, int64_t arg3,
                                      int64_t arg4);
typedef int64_t (*Prototype_General6)(int64_t arg0, int64_t arg1, int64_t arg2, int64_t arg3,
                                      int64_t arg4, int64_t arg5);
typedef int64_t (*Prototype_General7)(int64_t arg0, int64_t arg1, int64_t arg2, int64_t arg3,
                                      int64_t arg4, int64_t arg5, int64_t arg6);
typedef int64_t (*Prototype_General8)(int64_t arg0, int64_t arg1, int64_t arg2, int64_t arg3,
                                      int64_t arg4, int64_t arg5, int64_t arg6, int64_t arg7);

typedef int64_t (*Prototype_Int_Double)(double arg0);
typedef int64_t (*Prototype_Int_IntDouble)(int32_t arg0, double arg1);

typedef float (*Prototype_Float32_Float32)(float arg0);

typedef double (*Prototype_Double_None)();
typedef double (*Prototype_Double_Double)(double arg0);
typedef double (*Prototype_Double_Int)(int32_t arg0);
typedef double (*Prototype_Double_DoubleInt)(double arg0, int32_t arg1);
typedef double (*Prototype_Double_IntDouble)(int32_t arg0, double arg1);
typedef double (*Prototype_Double_DoubleDouble)(double arg0, double arg1);
typedef double (*Prototype_Double_DoubleDoubleDouble)(double arg0, double arg1, double arg2);
typedef double (*Prototype_Double_DoubleDoubleDoubleDouble)(double arg0, double arg1,
                                                            double arg2, double arg3);

void
Simulator::setGPR32Result(int32_t result)
{
    set_wreg(0, result);
}

void
Simulator::setGPR64Result(int64_t result)
{
    set_xreg(0, result);
}

void
Simulator::setFP32Result(float result)
{
    set_sreg(0, result);
}

void
Simulator::setFP64Result(double result)
{
    set_dreg(0, result);
}

// Simulator support for callWithABI().
void
Simulator::VisitCallRedirection(Instruction *instr)
{
    MOZ_ASSERT(instr->Mask(ExceptionMask) == SVC);
    MOZ_ASSERT(instr->ImmException() == kCallRtRedirected);

    Redirection *redir = Redirection::FromSvcInstruction(instr);
    uintptr_t nativeFn = reinterpret_cast<uintptr_t>(redir->nativeFunction());

    // Stack must be aligned prior to the call.
    // FIXME: It's actually our job to perform the alignment...
    //MOZ_ASSERT((xreg(31, Reg31IsStackPointer) & (StackAlignment - 1)) == 0);

    // Used to assert that callee-saved registers are preserved.
    DebugOnly<int64_t> x19 = xreg(19);
    DebugOnly<int64_t> x20 = xreg(20);
    DebugOnly<int64_t> x21 = xreg(21);
    DebugOnly<int64_t> x22 = xreg(22);
    DebugOnly<int64_t> x23 = xreg(23);
    DebugOnly<int64_t> x24 = xreg(24);
    DebugOnly<int64_t> x25 = xreg(25);
    DebugOnly<int64_t> x26 = xreg(26);
    DebugOnly<int64_t> x27 = xreg(27);
    DebugOnly<int64_t> x28 = xreg(28);
    DebugOnly<int64_t> x29 = xreg(29);
    DebugOnly<int64_t> savedSP = xreg(31, Reg31IsStackPointer);

    // Remember LR for returning from the "call".
    int64_t savedLR = xreg(30);

    // Allow recursive Simulator calls: returning from the call must stop
    // the simulation and transition back to native Simulator code.
    set_xreg(30, int64_t(kEndOfSimAddress));

    // Store argument register values in local variables for ease of use below.
    int64_t x0 = xreg(0);
    int64_t x1 = xreg(1);
    int64_t x2 = xreg(2);
    int64_t x3 = xreg(3);
    int64_t x4 = xreg(4);
    int64_t x5 = xreg(5);
    int64_t x6 = xreg(6);
    int64_t x7 = xreg(7);
    double d0 = dreg(0);
    double d1 = dreg(1);
    double d2 = dreg(2);
    double d3 = dreg(3);
    float s0 = sreg(0);

    // Dispatch the call and set the return value.
    switch (redir->type()) {
      // Cases with int64_t return type.
      case Args_General0: {
        int64_t ret = reinterpret_cast<Prototype_General0>(nativeFn)();
        setGPR64Result(ret);
        break;
      }
      case Args_General1: {
        int64_t ret = reinterpret_cast<Prototype_General1>(nativeFn)(x0);
        setGPR64Result(ret);
        break;
      }
      case Args_General2: {
        int64_t ret = reinterpret_cast<Prototype_General2>(nativeFn)(x0, x1);
        setGPR64Result(ret);
        break;
      }
      case Args_General3: {
        int64_t ret = reinterpret_cast<Prototype_General3>(nativeFn)(x0, x1, x2);
        setGPR64Result(ret);
        break;
      }
      case Args_General4: {
        int64_t ret = reinterpret_cast<Prototype_General4>(nativeFn)(x0, x1, x2, x3);
        setGPR64Result(ret);
        break;
      }
      case Args_General5: {
        int64_t ret = reinterpret_cast<Prototype_General5>(nativeFn)(x0, x1, x2, x3, x4);
        setGPR64Result(ret);
        break;
      }
      case Args_General6: {
        int64_t ret = reinterpret_cast<Prototype_General6>(nativeFn)(x0, x1, x2, x3, x4, x5);
        setGPR64Result(ret);
        break;
      }
      case Args_General7: {
        int64_t ret = reinterpret_cast<Prototype_General7>(nativeFn)(x0, x1, x2, x3, x4, x5, x6);
        setGPR64Result(ret);
        break;
      }
      case Args_General8: {
        int64_t ret = reinterpret_cast<Prototype_General8>(nativeFn)(x0, x1, x2, x3, x4, x5, x6, x7);
        setGPR64Result(ret);
        break;
      }

      // Cases with GPR return type. This can be int32 or int64, but int64 is a safer assumption.
      case Args_Int_Double: {
        int64_t ret = reinterpret_cast<Prototype_Int_Double>(nativeFn)(d0);
        setGPR64Result(ret);
        break;
      }
      case Args_Int_IntDouble: {
        int64_t ret = reinterpret_cast<Prototype_Int_IntDouble>(nativeFn)(x0, d0);
        setGPR64Result(ret);
        break;
      }

      // Cases with float return type.
      case Args_Float32_Float32: {
        float ret = reinterpret_cast<Prototype_Float32_Float32>(nativeFn)(s0);
        setFP32Result(ret);
        break;
      }

      // Cases with double return type.
      case Args_Double_None: {
        double ret = reinterpret_cast<Prototype_Double_None>(nativeFn)();
        setFP64Result(ret);
        break;
      }
      case Args_Double_Double: {
        double ret = reinterpret_cast<Prototype_Double_Double>(nativeFn)(d0);
        setFP64Result(ret);
        break;
      }
      case Args_Double_Int: {
        double ret = reinterpret_cast<Prototype_Double_Int>(nativeFn)(x0);
        setFP64Result(ret);
        break;
      }
      case Args_Double_DoubleInt: {
        double ret = reinterpret_cast<Prototype_Double_DoubleInt>(nativeFn)(d0, x0);
        setFP64Result(ret);
        break;
      }
      case Args_Double_DoubleDouble: {
        double ret = reinterpret_cast<Prototype_Double_DoubleDouble>(nativeFn)(d0, d1);
        setFP64Result(ret);
        break;
      }
      case Args_Double_DoubleDoubleDouble: {
          double ret = reinterpret_cast<Prototype_Double_DoubleDoubleDouble>(nativeFn)(d0, d1, d2);
          setFP64Result(ret);
          break;
      }
      case Args_Double_DoubleDoubleDoubleDouble: {
          double ret = reinterpret_cast<Prototype_Double_DoubleDoubleDoubleDouble>(nativeFn)(d0, d1, d2, d3);
          setFP64Result(ret);
          break;
      }

      case Args_Double_IntDouble: {
        double ret = reinterpret_cast<Prototype_Double_IntDouble>(nativeFn)(x0, d0);
        setFP64Result(ret);
        break;
      }

      default:
        MOZ_CRASH("Unknown function type.");
    }

    // TODO: Nuke the volatile registers.

    // Assert that callee-saved registers are unchanged.
    MOZ_ASSERT(xreg(19) == x19);
    MOZ_ASSERT(xreg(20) == x20);
    MOZ_ASSERT(xreg(21) == x21);
    MOZ_ASSERT(xreg(22) == x22);
    MOZ_ASSERT(xreg(23) == x23);
    MOZ_ASSERT(xreg(24) == x24);
    MOZ_ASSERT(xreg(25) == x25);
    MOZ_ASSERT(xreg(26) == x26);
    MOZ_ASSERT(xreg(27) == x27);
    MOZ_ASSERT(xreg(28) == x28);
    MOZ_ASSERT(xreg(29) == x29);

    // Assert that the stack is unchanged.
    MOZ_ASSERT(savedSP == xreg(31, Reg31IsStackPointer));

    // Simulate a return.
    set_lr(savedLR);
    set_pc((Instruction *)savedLR);
    if (getenv("USE_DEBUGGER"))
        printf("SVCRET\n");
}

void
Simulator::DoPrintf(Instruction* instr)
{
    MOZ_ASSERT((instr->Mask(ExceptionMask) == HLT) && (instr->ImmException() == kPrintfOpcode));

    // Read the arguments encoded inline in the instruction stream.
    uint32_t arg_count;
    uint32_t arg_pattern_list;
    JS_STATIC_ASSERT(sizeof(*instr) == 1);
    memcpy(&arg_count, instr + kPrintfArgCountOffset, sizeof(arg_count));
    memcpy(&arg_pattern_list, instr + kPrintfArgPatternListOffset, sizeof(arg_pattern_list));

    MOZ_ASSERT(arg_count <= kPrintfMaxArgCount);
    MOZ_ASSERT((arg_pattern_list >> (kPrintfArgPatternBits * arg_count)) == 0);

    // We need to call the host printf function with a set of arguments defined by
    // arg_pattern_list. Because we don't know the types and sizes of the
    // arguments, this is very difficult to do in a robust and portable way. To
    // work around the problem, we pick apart the format string, and print one
    // format placeholder at a time.

    // Allocate space for the format string. We take a copy, so we can modify it.
    // Leave enough space for one extra character per expected argument (plus the
    // '\0' termination).
    const char * format_base = reg<const char *>(0);
    MOZ_ASSERT(format_base != nullptr);
    size_t length = strlen(format_base) + 1;
    char * const format = new char[length + arg_count];

    // A list of chunks, each with exactly one format placeholder.
    const char * chunks[kPrintfMaxArgCount];

    // Copy the format string and search for format placeholders.
    uint32_t placeholder_count = 0;
    char * format_scratch = format;
    for (size_t i = 0; i < length; i++) {
        if (format_base[i] != '%') {
            *format_scratch++ = format_base[i];
        } else {
            if (format_base[i + 1] == '%') {
                // Ignore explicit "%%" sequences.
                *format_scratch++ = format_base[i];
                i++;
                // Chunks after the first are passed as format strings to printf, so we
                // need to escape '%' characters in those chunks.
                if (placeholder_count > 0) *format_scratch++ = format_base[i];
            } else {
                MOZ_ASSERT(placeholder_count < arg_count);
                // Insert '\0' before placeholders, and store their locations.
                *format_scratch++ = '\0';
                chunks[placeholder_count++] = format_scratch;
                *format_scratch++ = format_base[i];
            }
        }
    }
    MOZ_ASSERT(placeholder_count == arg_count);

    // Finally, call printf with each chunk, passing the appropriate register
    // argument. Normally, printf returns the number of bytes transmitted, so we
    // can emulate a single printf call by adding the result from each chunk. If
    // any call returns a negative (error) value, though, just return that value.

    printf("%s", clr_printf);

    // Because '\0' is inserted before each placeholder, the first string in
    // 'format' contains no format placeholders and should be printed literally.
    int result = printf("%s", format);
    int pcs_r = 1;      // Start at x1. x0 holds the format string.
    int pcs_f = 0;      // Start at d0.
    if (result >= 0) {
        for (uint32_t i = 0; i < placeholder_count; i++) {
            int part_result = -1;

            uint32_t arg_pattern = arg_pattern_list >> (i * kPrintfArgPatternBits);
            arg_pattern &= (1 << kPrintfArgPatternBits) - 1;
            switch (arg_pattern) {
              case kPrintfArgW: part_result = printf(chunks[i], wreg(pcs_r++)); break;
              case kPrintfArgX: part_result = printf(chunks[i], xreg(pcs_r++)); break;
              case kPrintfArgD: part_result = printf(chunks[i], dreg(pcs_f++)); break;
              default: VIXL_UNREACHABLE();
            }

            if (part_result < 0) {
                // Handle error values.
                result = part_result;
                break;
            }

            result += part_result;
        }
    }

    printf("%s", clr_normal);

    // Printf returns its result in x0 (just like the C library's printf).
    set_xreg(0, result);

    // The printf parameters are inlined in the code, so skip them.
    set_pc(instr->InstructionAtOffset(kPrintfLength));

    // Set LR as if we'd just called a native printf function.
    set_lr(get_pc());

    delete[] format;
}
#if 0
SimulatorRuntime *
CreateSimulatorRuntime()
{
    SimulatorRuntime *srt = js_new<SimulatorRuntime>();
    if (!srt)
        return nullptr;

    if (!srt->init()) {
        js_delete(srt);
        return nullptr;
    }

    // TODO: ARM64_SIM_STOP_AT support and so on.
    return srt;
}

void
DestroySimulatorRuntime(SimulatorRuntime *srt)
{
    js_delete(srt);
}
#endif
} // namespace jit
} // namespace js

// FIXME: All this stuff should probably be shared.

js::jit::Simulator *
js::PerThreadData::simulator() const
{
    return runtime_->simulator();
}
js::jit::Simulator *
JSRuntime::simulator() const
{
    return simulator_;
}

#if 0
void
js::PerThreadData::setSimulator(js::jit::Simulator *sim)
{
    simulator_ = sim;
    simulatorStackLimit_ = sim->stackLimit();
}

js::jit::SimulatorRuntime *
js::PerThreadData::simulatorRuntime() const
{
    return runtime_->simulatorRuntime();
}
#endif
uintptr_t *
JSRuntime::addressOfSimulatorStackLimit()
{
    return simulator_->addressOfStackLimit();
}
#if 0
js::jit::SimulatorRuntime *
JSRuntime::simulatorRuntime() const
{
    return simulatorRuntime_;
}

void
JSRuntime::setSimulatorRuntime(js::jit::SimulatorRuntime *srt)
{
    MOZ_ASSERT(!simulatorRuntime_);
    simulatorRuntime_ = srt;
}
#endif
js::jit::Simulator *
js::jit::Simulator::Create()
{
    Decoder *decoder_ = js_new<Decoder>();
    if (!decoder_) {
        MOZ_ReportAssertionFailure("[unhandlable oom] Decoder", __FILE__, __LINE__);
        MOZ_CRASH();
    }

    // FIXME: This just leaks the Decoder object for now, which is probably OK.
    // FIXME: We should free it at some point.
    // FIXME: Note that it can't be stored in the SimulatorRuntime due to lifetime conflicts.
    if (getenv("USE_DEBUGGER") != nullptr) {
        DebuggerARM64 *debugger = js_new<DebuggerARM64>(decoder_, stdout);
        if (debugger) {
            debugger->set_log_parameters(LOG_DISASM | LOG_REGS | LOG_FP_REGS);
            return debugger;
        }
    }
    Simulator *sim = js_new<Simulator>();
    if (!sim) {
        MOZ_CRASH("NEED SIMULATOR");
        return nullptr;
    }
    sim->init(decoder_, stdout);
    
    return sim;
}

void
js::jit::Simulator::Destroy(js::jit::Simulator * sim)
{
    js_delete(sim);
}

uintptr_t *
js::jit::Simulator::addressOfStackLimit()
{
    return (uintptr_t*)&stack_limit_;
}
