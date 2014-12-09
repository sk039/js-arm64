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

#ifndef VIXL_A64_SIMULATOR_A64_H_
#define VIXL_A64_SIMULATOR_A64_H_

#include "mozilla/Vector.h"

#include "jit/arm64/vixl/Assembler-vixl.h"
#include "jit/arm64/vixl/Disasm-vixl.h"
#include "jit/arm64/vixl/Instructions-vixl.h"
#include "jit/arm64/vixl/Instrument-vixl.h"
#include "jit/arm64/vixl/VIXL-Globals-vixl.h"
#include "jit/arm64/vixl/VIXL-Utils-vixl.h"

#include "jit/IonTypes.h"

#include <unistd.h>

#ifdef JS_CODEGEN_ARM64
#ifdef JS_ARM64_SIMULATOR

namespace js {
namespace jit {

enum ReverseByteMode {
    Reverse16 = 0,
    Reverse32 = 1,
    Reverse64 = 2
};

// Printf. See debugger-a64.h for more information on pseudo instructions.
//  - arg_count: The number of arguments.
//  - arg_pattern: A set of PrintfArgPattern values, packed into two-bit fields.
//
// Simulate a call to printf.
//
// Floating-point and integer arguments are passed in separate sets of registers
// in AAPCS64 (even for varargs functions), so it is not possible to determine
// the type of each argument without some information about the values that were
// passed in. This information could be retrieved from the printf format string,
// but the format string is not trivial to parse so we encode the relevant
// information with the HLT instruction.
//
// The interface is as follows:
//    x0: The format string
// x1-x7: Optional arguments, if type == CPURegister::kRegister
// d0-d7: Optional arguments, if type == CPURegister::kFloatRegister
const Instr kPrintfOpcode = 0xdeb1;
const unsigned kPrintfArgCountOffset = 1 * kInstructionSize;
const unsigned kPrintfArgPatternListOffset = 2 * kInstructionSize;
const unsigned kPrintfLength = 3 * kInstructionSize;

const unsigned kPrintfMaxArgCount = 4;

// The argument pattern is a set of two-bit-fields, each with one of the
// following values:
enum PrintfArgPattern {
    kPrintfArgW = 1,
    kPrintfArgX = 2,
    // There is no kPrintfArgS because floats are always converted to doubles in C
    // varargs calls.
    kPrintfArgD = 3
};
static const unsigned kPrintfArgPatternBits = 2;

// The proper way to initialize a simulated system register (such as NZCV) is as
// follows:
//  SimSystemRegister nzcv = SimSystemRegister::DefaultValueFor(NZCV);
class SimSystemRegister
{
  public:
    // The default constructor represents a register which has no writable bits.
    // It is not possible to set its value to anything other than 0.
    SimSystemRegister()
      : value_(0), write_ignore_mask_(0xffffffff)
    { }

    inline uint32_t RawValue() const {
        return value_;
    }

    inline void SetRawValue(uint32_t new_value) {
        value_ = (value_ & write_ignore_mask_) | (new_value & ~write_ignore_mask_);
    }

    inline uint32_t Bits(int msb, int lsb) const {
        return unsigned_bitextract_32(msb, lsb, value_);
    }

    inline int32_t SignedBits(int msb, int lsb) const {
        return signed_bitextract_32(msb, lsb, value_);
    }

    void SetBits(int msb, int lsb, uint32_t bits);

    // Default system register values.
    static SimSystemRegister DefaultValueFor(SystemRegister id);

    #define DEFINE_GETTER(Name, HighBit, LowBit, Func)                            \
    inline uint32_t Name() const { return Func(HighBit, LowBit); }              \
    inline void Set##Name(uint32_t bits) { SetBits(HighBit, LowBit, bits); }
    #define DEFINE_WRITE_IGNORE_MASK(Name, Mask)                                  \
    static const uint32_t Name##WriteIgnoreMask = ~static_cast<uint32_t>(Mask);

    SYSTEM_REGISTER_FIELDS_LIST(DEFINE_GETTER, DEFINE_WRITE_IGNORE_MASK)

    #undef DEFINE_ZERO_BITS
    #undef DEFINE_GETTER

  protected:
    // Most system registers only implement a few of the bits in the word. Other
    // bits are "read-as-zero, write-ignored". The write_ignore_mask argument
    // describes the bits which are not modifiable.
    SimSystemRegister(uint32_t value, uint32_t write_ignore_mask)
      : value_(value), write_ignore_mask_(write_ignore_mask)
    { }

    uint32_t value_;
    uint32_t write_ignore_mask_;
};

// Represent a register (r0-r31, v0-v31).
template<int kSizeInBytes>
class SimRegisterBase
{
  public:
    // Write the specified value. The value is zero-extended if necessary.
    template<typename T>
    void Set(T new_value) {
        JS_STATIC_ASSERT(sizeof(new_value) <= kSizeInBytes);
        if (sizeof(new_value) < kSizeInBytes) {
            // All AArch64 registers are zero-extending.
            memset(value_ + sizeof(new_value), 0, kSizeInBytes - sizeof(new_value));
        }
        memcpy(value_, &new_value, sizeof(new_value));
    }

    // Read the value as the specified type. The value is truncated if necessary.
    template<typename T>
    T Get() const {
        T result;
        JS_STATIC_ASSERT(sizeof(result) <= kSizeInBytes);
        memcpy(&result, value_, sizeof(T));
        return result;
    }

  protected:
    uint8_t value_[kSizeInBytes];
};
typedef SimRegisterBase<kXRegSizeInBytes> SimRegister;      // r0-r31
typedef SimRegisterBase<kDRegSizeInBytes> SimFloatRegister;    // v0-v31

class SimExclusiveLocalMonitor
{
  public:
    SimExclusiveLocalMonitor()
      : kSkipClearProbability(8), seed_(0x87654321)
    {
        Clear();
    }

    // Clear the exclusive monitor (like clrex).
    void Clear() {
        address_ = 0;
        size_ = 0;
    }

    // Clear the exclusive monitor most of the time.
    void MaybeClear() {
        if ((seed_ % kSkipClearProbability) != 0)
            Clear();

        // Advance seed_ using a simple linear congruential generator.
        seed_ = (seed_ * 48271) % 2147483647;
    }

    // Mark the address range for exclusive access (like load-exclusive).
    template <typename T>
    void MarkExclusive(T address, size_t size) {
        JS_STATIC_ASSERT(sizeof(address) == sizeof(address_));
        address_ = reinterpret_cast<uintptr_t>(address);
        size_ = size;
    }

    // Return true if the address range is marked (like store-exclusive).
    // This helper doesn't implicitly clear the monitor.
    template <typename T>
    bool IsExclusive(T address, size_t size) {
        JS_STATIC_ASSERT(sizeof(address) == sizeof(address_));
        MOZ_ASSERT(size > 0);
        // Be pedantic: Require both the address and the size to match.
        return (size == size_) && (reinterpret_cast<uintptr_t>(address) == address_);
    }

  private:
    uintptr_t address_;
    size_t size_;

    const int kSkipClearProbability;
    uint32_t seed_;
};

// We can't accurately simulate the global monitor since it depends on external
// influences. Instead, this implementation occasionally causes accesses to
// fail, according to kPassProbability.
class SimExclusiveGlobalMonitor
{
  public:
    SimExclusiveGlobalMonitor()
      : kPassProbability(8), seed_(0x87654321)
    { }

    template <typename T>
    bool IsExclusive(T address, size_t size) {
        USEARG(address);
        USEARG(size);

        bool pass = (seed_ % kPassProbability) != 0;
        // Advance seed_ using a simple linear congruential generator.
        seed_ = (seed_ * 48271) % 2147483647;
        return pass;
    }

  private:
    const int kPassProbability;
    uint32_t seed_;
};

class SimulatorRuntime;
class Simulator : public DecoderVisitor
{
  public:
    explicit Simulator(SimulatorRuntime *srt);
    explicit Simulator(Decoder* decoder, FILE* stream = stdout);
    ~Simulator();

#ifdef DEBUG
    // TODO: Get rid of fds.
    int fds[2];
#endif

    void ResetState();

    void init(Decoder* decoder, FILE* stream = stdout);

    // The currently executing Simulator instance.
    // Potentially there can be one for each native thread.
    static Simulator *Current();

    static inline uintptr_t StackLimit() {
        return Simulator::Current()->stackLimit();
    }

    // Run the simulator.
    virtual void Run();
    void RunFrom(Instruction* first);

    // Sets up the simulator state and grabs the result on return.
    int64_t call(uint8_t* entry, int argument_count, ...);

    static void *RedirectNativeFunction(void *nativeFunction, ABIFunctionType type);

    // FIXME: All the simulators should share this logic.
    uintptr_t stackLimit() const {
        // Leave a safety margin of 1MB to prevent overrunning the stack when
        // pushing values (total stack size is 2MB)
        return reinterpret_cast<uintptr_t>(stack_) + 1024 * 1024;
    }

    bool overRecursed(uintptr_t newsp = 0) const {
        if (newsp == 0)
            newsp = xreg(Registers::sp, Reg31IsStackPointer);
        return newsp <= stackLimit();
    }

    bool overRecursedWithExtra(uint32_t extra) const {
        uintptr_t newsp = xreg(Registers::sp, Reg31IsStackPointer) - extra;
        return newsp <= stackLimit();
    }

    // Simulation helpers.
    inline Instruction* get_pc() { return pc_; }
    inline void set_pc(Instruction* new_pc) {
        pc_ = new_pc;
        pc_modified_ = true;
    }

    inline void increment_pc() {
        if (!pc_modified_)
            pc_ = pc_->NextInstruction();
        pc_modified_ = false;
    }

    inline void ExecuteInstruction() {
        // The program counter should always be aligned.
        MOZ_ASSERT(IsWordAligned(pc_));
        decoder_->Decode(pc_);
        increment_pc();
    }

    // Declare all Visitor functions.
    #define DECLARE(A)  void Visit##A(Instruction* instr);
    VISITOR_LIST(DECLARE)
    #undef DECLARE

    // Integer register accessors.

    // Basic accessor: Read the register as the specified type.
    template<typename T>
    inline T reg(unsigned code, Reg31Mode r31mode = Reg31IsZeroRegister) const {
        JS_STATIC_ASSERT((sizeof(T) == kWRegSizeInBytes) || (sizeof(T) == kXRegSizeInBytes));
        MOZ_ASSERT(code < kNumberOfRegisters);

        if ((code == 31) && (r31mode == Reg31IsZeroRegister)) {
            T result;
            memset(&result, 0, sizeof(result));
            return result;
        }
        return registers_[code].Get<T>();
    }

    // Common specialized accessors for the reg() template.
    inline int32_t wreg(unsigned code, Reg31Mode r31mode = Reg31IsZeroRegister) const {
        return reg<int32_t>(code, r31mode);
    }

    inline int64_t xreg(unsigned code, Reg31Mode r31mode = Reg31IsZeroRegister) const {
        return reg<int64_t>(code, r31mode);
    }

    // As above, with parameterized size and return type. The value is
    // either zero-extended or truncated to fit, as required.
    template<typename T>
    inline T reg(unsigned size, unsigned code, Reg31Mode r31mode = Reg31IsZeroRegister) const {
        uint64_t raw;
        switch (size) {
          case kWRegSize: raw = reg<uint32_t>(code, r31mode); break;
          case kXRegSize: raw = reg<uint64_t>(code, r31mode); break;
          default:
            VIXL_UNREACHABLE();
            return 0;
        }

        T result;
        JS_STATIC_ASSERT(sizeof(result) <= sizeof(raw));
        // Copy the result and truncate to fit. This assumes a little-endian host.
        memcpy(&result, &raw, sizeof(result));
        return result;
    }

    // Use int64_t by default if T is not specified.
    inline int64_t reg(unsigned size, unsigned code,
                       Reg31Mode r31mode = Reg31IsZeroRegister) const
    {
        return reg<int64_t>(size, code, r31mode);
    }

    // Basic accessor: Write the specified value.
    template<typename T>
    inline void set_reg(unsigned code, T value, Reg31Mode r31mode = Reg31IsZeroRegister) {
        JS_STATIC_ASSERT((sizeof(T) == kWRegSizeInBytes) || (sizeof(T) == kXRegSizeInBytes));
        MOZ_ASSERT(code < kNumberOfRegisters);

        if ((code == 31) && (r31mode == Reg31IsZeroRegister))
            return;

        registers_[code].Set(value);
    }

    // Common specialized accessors for the set_reg() template.
    inline void set_wreg(unsigned code, int32_t value, Reg31Mode r31mode = Reg31IsZeroRegister) {
        set_reg(code, value, r31mode);
    }

    inline void set_xreg(unsigned code, int64_t value, Reg31Mode r31mode = Reg31IsZeroRegister) {
        set_reg(code, value, r31mode);
    }

    // As above, with parameterized size and type. The value is either
    // zero-extended or truncated to fit, as required.
    template<typename T>
    inline void set_reg(unsigned size, unsigned code, T value,
                        Reg31Mode r31mode = Reg31IsZeroRegister)
    {
        // Zero-extend the input.
        uint64_t raw = 0;
        JS_STATIC_ASSERT(sizeof(value) <= sizeof(raw));
        memcpy(&raw, &value, sizeof(value));

        // Write (and possibly truncate) the value.
        switch (size) {
          case kWRegSize: set_reg<uint32_t>(code, raw, r31mode); break;
          case kXRegSize: set_reg<uint64_t>(code, raw, r31mode); break;
          default:
            VIXL_UNREACHABLE();
            return;
        }
    }

    // Common specialized accessors for the set_reg() template.

    // Commonly-used special cases.
    template<typename T>
    inline void set_lr(T value) {
        set_reg(kLinkRegCode, value);
    }

    template<typename T>
    inline void set_sp(T value) {
        set_reg(31, value, Reg31IsStackPointer);
    }

    // FP register accessors.
    // These are equivalent to the integer register accessors, but for FP
    // registers.

    // Basic accessor: Read the register as the specified type.
    template<typename T>
    inline T fpreg(unsigned code) const {
        JS_STATIC_ASSERT((sizeof(T) == kSRegSizeInBytes) || (sizeof(T) == kDRegSizeInBytes));
        MOZ_ASSERT(code < kNumberOfFloatRegisters);

        return fpregisters_[code].Get<T>();
    }

    // Common specialized accessors for the fpreg() template.
    inline float sreg(unsigned code) const {
        return fpreg<float>(code);
    }

    inline uint32_t sreg_bits(unsigned code) const {
        return fpreg<uint32_t>(code);
    }

    inline double dreg(unsigned code) const {
        return fpreg<double>(code);
    }

    inline uint64_t dreg_bits(unsigned code) const {
        return fpreg<uint64_t>(code);
    }

    // As above, with parameterized size and return type. The value is
    // either zero-extended or truncated to fit, as required.
    template<typename T>
    inline T fpreg(unsigned size, unsigned code) const {
        uint64_t raw;
        switch (size) {
          case kSRegSize: raw = fpreg<uint32_t>(code); break;
          case kDRegSize: raw = fpreg<uint64_t>(code); break;
          default:
            VIXL_UNREACHABLE();
            raw = 0;
            break;
        }

        T result;
        JS_STATIC_ASSERT(sizeof(result) <= sizeof(raw));
        // Copy the result and truncate to fit. This assumes a little-endian host.
        memcpy(&result, &raw, sizeof(result));
        return result;
    }

    // Basic accessor: Write the specified value.
    template<typename T>
    inline void set_fpreg(unsigned code, T value) {
        JS_STATIC_ASSERT((sizeof(value) == kSRegSizeInBytes) ||
                         (sizeof(value) == kDRegSizeInBytes));
        MOZ_ASSERT(code < kNumberOfFloatRegisters);
        fpregisters_[code].Set(value);
    }

    // Common specialized accessors for the set_fpreg() template.
    inline void set_sreg(unsigned code, float value) {
        set_fpreg(code, value);
    }

    inline void set_sreg_bits(unsigned code, uint32_t value) {
        set_fpreg(code, value);
    }

    inline void set_dreg(unsigned code, double value) {
        set_fpreg(code, value);
    }

    inline void set_dreg_bits(unsigned code, uint64_t value) {
        set_fpreg(code, value);
    }

    bool N() { return nzcv_.N() != 0; }
    bool Z() { return nzcv_.Z() != 0; }
    bool C() { return nzcv_.C() != 0; }
    bool V() { return nzcv_.V() != 0; }
    SimSystemRegister& nzcv() { return nzcv_; }

    // TODO(jbramley): Find a way to make the fpcr_ members return the proper
    // types, so these accessors are not necessary.
    FPRounding RMode() { return static_cast<FPRounding>(fpcr_.RMode()); }
    bool DN() { return fpcr_.DN() != 0; }
    SimSystemRegister& fpcr() { return fpcr_; }

    // callWithABI() support helpers.
    void setGPR32Result(int32_t result);
    void setGPR64Result(int64_t result);
    void setFP32Result(float result);
    void setFP64Result(double result);
    void VisitCallRedirection(Instruction *instr);

    // Debug helpers
    void PrintSystemRegisters(bool print_all = false);
    void PrintRegisters(bool print_all_regs = false);
    void PrintFloatRegisters(bool print_all_regs = false);
    void PrintProcessorState();

    static const char* WRegNameForCode(unsigned code, Reg31Mode mode = Reg31IsZeroRegister);
    static const char* XRegNameForCode(unsigned code, Reg31Mode mode = Reg31IsZeroRegister);
    static const char* SRegNameForCode(unsigned code);
    static const char* DRegNameForCode(unsigned code);
    static const char* VRegNameForCode(unsigned code);

    inline bool coloured_trace() { return coloured_trace_; }
    void set_coloured_trace(bool value);

    inline bool disasm_trace() { return disasm_trace_; }
    inline void set_disasm_trace(bool value) {
        if (value != disasm_trace_) {
            if (value)
                decoder_->InsertVisitorBefore(print_disasm_, this);
            else
                decoder_->RemoveVisitor(print_disasm_);
            disasm_trace_ = value;
        }
    }
    inline void set_instruction_stats(bool value) {
        if (value != instruction_stats_) {
            if (value)
                decoder_->AppendVisitor(instrumentation_);
            else
                decoder_->RemoveVisitor(instrumentation_);
            instruction_stats_ = value;
        }
    }

    // Clear the simulated local monitor to force the next store-exclusive
    // instruction to fail.
    inline void ClearLocalMonitor() {
        local_monitor_.Clear();
    }

    inline void SilenceExclusiveAccessWarning() {
        print_exclusive_access_warning_ = false;
    }

    protected:
    const char* clr_normal;
    const char* clr_flag_name;
    const char* clr_flag_value;
    const char* clr_reg_name;
    const char* clr_reg_value;
    const char* clr_fpreg_name;
    const char* clr_fpreg_value;
    const char* clr_memory_value;
    const char* clr_memory_address;
    const char* clr_debug_number;
    const char* clr_debug_message;
    const char* clr_warning;
    const char* clr_warning_message;
    const char* clr_printf;

    // Simulation helpers ------------------------------------
    bool ConditionPassed(Condition cond) {
        switch (cond) {
          case eq:
            return Z();
          case ne:
            return !Z();
          case hs:
            return C();
          case lo:
            return !C();
          case mi:
            return N();
          case pl:
            return !N();
          case vs:
            return V();
          case vc:
            return !V();
          case hi:
            return C() && !Z();
          case ls:
            return !(C() && !Z());
          case ge:
            return N() == V();
          case lt:
            return N() != V();
          case gt:
            return !Z() && (N() == V());
          case le:
            return !(!Z() && (N() == V()));
          case nv:  // Fall through.
          case al:
            return true;
          default:
            VIXL_UNREACHABLE();
            return false;
        }
    }

    bool ConditionPassed(Instr cond) {
        return ConditionPassed(static_cast<Condition>(cond));
    }

    bool ConditionFailed(Condition cond) {
        return !ConditionPassed(cond);
    }

    void AddSubHelper(Instruction* instr, int64_t op2);
    int64_t AddWithCarry(unsigned reg_size, bool set_flags,
                         int64_t src1, int64_t src2, int64_t carry_in = 0);
    void LogicalHelper(Instruction* instr, int64_t op2);
    void ConditionalCompareHelper(Instruction* instr, int64_t op2);
    void LoadStoreHelper(Instruction* instr, int64_t offset, AddrMode addrmode);
    void LoadStorePairHelper(Instruction* instr, AddrMode addrmode);
    uint8_t* AddressModeHelper(unsigned addr_reg, int64_t offset, AddrMode addrmode);

    template <typename T>
    T AddressUntag(T address) {
        uint64_t bits = reinterpret_cast<uint64_t>(address);
        return reinterpret_cast<T>(bits & ~kAddressTagMask);
    }

    template <typename T, typename A>
    T MemoryRead(A address) {
        T value;
        address = AddressUntag(address);
        MOZ_ASSERT((sizeof(value) == 1) || (sizeof(value) == 2) ||
                   (sizeof(value) == 4) || (sizeof(value) == 8));
        memcpy(&value, reinterpret_cast<const char *>(address), sizeof(value));
        return value;
    }

    template <typename T, typename A>
    void MemoryWrite(A address, T value) {
        address = AddressUntag(address);
        MOZ_ASSERT((sizeof(value) == 1) || (sizeof(value) == 2) ||
                   (sizeof(value) == 4) || (sizeof(value) == 8));
        memcpy(reinterpret_cast<char *>(address), &value, sizeof(value));
    }

    int64_t ShiftOperand(unsigned reg_size, int64_t value, Shift shift_type, unsigned amount);
    int64_t Rotate(unsigned reg_width, int64_t value, Shift shift_type, unsigned amount);
    int64_t ExtendValue(unsigned reg_width, int64_t value, Extend extend_type,
                        unsigned left_shift = 0);

    uint64_t ReverseBits(uint64_t value, unsigned num_bits);
    uint64_t ReverseBytes(uint64_t value, ReverseByteMode mode);

    template <typename T>
    T FPDefaultNaN() const;

    void FPCompare(double val0, double val1);
    double FPRoundInt(double value, FPRounding round_mode);
    double FPToDouble(float value);
    float FPToFloat(double value, FPRounding round_mode);
    double FixedToDouble(int64_t src, int fbits, FPRounding round_mode);
    double UFixedToDouble(uint64_t src, int fbits, FPRounding round_mode);
    float FixedToFloat(int64_t src, int fbits, FPRounding round_mode);
    float UFixedToFloat(uint64_t src, int fbits, FPRounding round_mode);
    int32_t FPToInt32(double value, FPRounding rmode);
    int64_t FPToInt64(double value, FPRounding rmode);
    uint32_t FPToUInt32(double value, FPRounding rmode);
    uint64_t FPToUInt64(double value, FPRounding rmode);

    template <typename T>
    T FPAdd(T op1, T op2);

    template <typename T>
    T FPDiv(T op1, T op2);

    template <typename T>
    T FPMax(T a, T b);

    template <typename T>
    T FPMaxNM(T a, T b);

    template <typename T>
    T FPMin(T a, T b);

    template <typename T>
    T FPMinNM(T a, T b);

    template <typename T>
    T FPMul(T op1, T op2);

    template <typename T>
    T FPMulAdd(T a, T op1, T op2);

    template <typename T>
    T FPSqrt(T op);

    template <typename T>
    T FPSub(T op1, T op2);

    // This doesn't do anything at the moment. We'll need it if we want support
    // for cumulative exception bits or floating-point exceptions.
    void FPProcessException() { }

    // Standard NaN processing.
    template <typename T>
    T FPProcessNaN(T op);

    bool FPProcessNaNs(Instruction* instr);

    template <typename T>
    T FPProcessNaNs(T op1, T op2);

    template <typename T>
    T FPProcessNaNs3(T op1, T op2, T op3);

    // Pseudo Printf instruction
    void DoPrintf(Instruction* instr);

    // Processor state ---------------------------------------

    // Simulated monitors for exclusive access instructions.
    SimExclusiveLocalMonitor local_monitor_;
    SimExclusiveGlobalMonitor global_monitor_;

    // Output stream.
    FILE* stream_;
    PrintDisassembler* print_disasm_;

    // Instruction statistics instrumentation.
    Instrument* instrumentation_;

    // General purpose registers. Register 31 is the stack pointer.
    SimRegister registers_[kNumberOfRegisters];

    // Floating point registers
    SimFloatRegister fpregisters_[kNumberOfFloatRegisters];

    // Program Status Register.
    // bits[31, 27]: Condition flags N, Z, C, and V.
    //               (Negative, Zero, Carry, Overflow)
    SimSystemRegister nzcv_;

    // Floating-Point Control Register
    SimSystemRegister fpcr_;

    // Only a subset of FPCR features are supported by the simulator. This helper
    // checks that the FPCR settings are supported.
    //
    // This is checked when floating-point instructions are executed, not when
    // FPCR is set. This allows generated code to modify FPCR for external
    // functions, or to save and restore it when entering and leaving generated
    // code.
    void AssertSupportedFPCR() {
        MOZ_ASSERT(fpcr().FZ() == 0);             // No flush-to-zero support.
        MOZ_ASSERT(fpcr().RMode() == FPTieEven);  // Ties-to-even rounding only.

        // The simulator does not support half-precision operations so fpcr().AHP()
        // is irrelevant, and is not checked here.
    }

    static inline int CalcNFlag(uint64_t result, unsigned reg_size) {
        return (result >> (reg_size - 1)) & 1;
    }

    static inline int CalcZFlag(uint64_t result) {
        return result == 0;
    }

    static const uint32_t kConditionFlagsMask = 0xf0000000;

    // Stack
    byte* stack_;
    static const int stack_protection_size_ = 256;
    // 2 MB stack.
    static const int stack_size_ = 2 * 1024 * 1024 + 2 * stack_protection_size_;
    byte* stack_limit_;

    Decoder* decoder_;
    // Indicates if the pc has been modified by the instruction and should not be
    // automatically incremented.
    bool pc_modified_;
    Instruction* pc_;

    static const char* xreg_names[];
    static const char* wreg_names[];
    static const char* sreg_names[];
    static const char* dreg_names[];
    static const char* vreg_names[];

    static const Instruction* kEndOfSimAddress;

  private:

    // Internal stack for tracking SP consistency.
    // Mutated by SVC instructions of type kMarkStackPointer and kCheckStackPointer.
    // TODO: Remove this before committing?
    Vector<int64_t, 0, SystemAllocPolicy> spStack_;

    bool coloured_trace_;

    // Indicates whether the disassembly trace is active.
    bool disasm_trace_;

    // Indicates whether the instruction instrumentation is active.
    bool instruction_stats_;

    // Indicates whether the exclusive-access warning has been printed.
    bool print_exclusive_access_warning_;
    void PrintExclusiveAccessWarning();

    virtual void enable_debugger() { }
};

class Redirection;

// FIXME: This is MPLv2, and probably shouldn't be here...
// FIXME: It should probably be in jit/shared.
class SimulatorRuntime
{
    friend class AutoLockSimulatorRuntime;

    Redirection *redirection_;

  protected:
    // Synchronize access between main thread and compilation/PJS threads.
    PRLock *lock_;
    mozilla::DebugOnly<PRThread *> lockOwner_;

  public:
    SimulatorRuntime()
      : redirection_(nullptr), lock_(nullptr), lockOwner_(nullptr)
    { }

    ~SimulatorRuntime() {
        if (lock_)
            PR_DestroyLock(lock_);
    }

    bool init() {
        lock_ = PR_NewLock();
        if (!lock_)
            return false;
        return true;
    }
    Redirection *redirection() const {
        MOZ_ASSERT(lockOwner_ == PR_GetCurrentThread());
        return redirection_;
    }
    void setRedirection(js::jit::Redirection *redirection) {
        MOZ_ASSERT(lockOwner_ == PR_GetCurrentThread());
        redirection_ = redirection;
    }
};

// FIXME: This class should definitely be shared.
class AutoLockSimulatorRuntime
{
  protected:
    SimulatorRuntime *srt_;

  public:
    AutoLockSimulatorRuntime(SimulatorRuntime *srt)
      : srt_(srt)
    {
        PR_Lock(srt_->lock_);
        MOZ_ASSERT(!srt_->lockOwner_);
#ifdef DEBUG
        srt_->lockOwner_ = PR_GetCurrentThread();
#endif
    }

    ~AutoLockSimulatorRuntime() {
        MOZ_ASSERT(srt_->lockOwner_ == PR_GetCurrentThread());
        srt_->lockOwner_ = nullptr;
        PR_Unlock(srt_->lock_);
    }
};

SimulatorRuntime *CreateSimulatorRuntime();
void DestroySimulatorRuntime(SimulatorRuntime *srt);

#define JS_CHECK_SIMULATOR_RECURSION_WITH_EXTRA(cx, extra, onerror)             \
    JS_BEGIN_MACRO                                                              \
        if (cx->mainThread().simulator()->overRecursedWithExtra(extra)) {       \
            js_ReportOverRecursed(cx);                                          \
            onerror;                                                            \
        }                                                                       \
    JS_END_MACRO


}  // namespace jit
}  // namespace js

#endif // JS_ARM64_SIMULATOR
#endif // JS_CODEGEN_ARM64

#endif  // VIXL_A64_SIMULATOR_A64_H_
