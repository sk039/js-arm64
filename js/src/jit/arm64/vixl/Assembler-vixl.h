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

#ifndef VIXL_A64_ASSEMBLER_A64_H_
#define VIXL_A64_ASSEMBLER_A64_H_

#include "jit/arm64/vixl/Instructions-vixl.h"
#include "jit/arm64/vixl/VIXL-Globals-vixl.h"
#include "jit/arm64/vixl/VIXL-Utils-vixl.h"

#include "jit/JitSpewer.h"

#include "jit/shared/Assembler-shared.h"
#include "jit/shared/IonAssemblerBufferWithConstantPools.h"

namespace js {
namespace jit {

// Exciting buffer logic, before it gets replaced with the new hotness.
class AssemblerVIXL;
typedef js::jit::AssemblerBufferWithConstantPools<1024, 4, Instruction, AssemblerVIXL> ARMBuffer;

// Registers.

typedef uint64_t RegList;
static const int kRegListSizeInBits = sizeof(RegList) * 8;

// Some CPURegister methods can return ARMRegister and ARMFPRegister types, so we
// need to declare them in advance.
class ARMRegister;
class ARMFPRegister;

class CPURegister
{
  public:
    enum RegisterType {
        // The kInvalid value is used to detect uninitialized static instances,
        // which are always zero-initialized before any constructors are called.
        kInvalid = 0,
        kARMRegister,
        kARMFPRegister,
        kNoRegister
    };

    MOZ_CONSTEXPR CPURegister()
      : code_(0), size_(0), type_(kNoRegister)
    { }

    MOZ_CONSTEXPR CPURegister(unsigned code, unsigned size, RegisterType type)
      : code_(code), size_(size), type_(type)
    { }

    unsigned code() const {
        MOZ_ASSERT(IsValid());
        return code_;
    }

    RegisterType type() const {
        MOZ_ASSERT(IsValidOrNone());
        return type_;
    }

    RegList Bit() const {
        MOZ_ASSERT(code_ < (sizeof(RegList) * 8));
        return IsValid() ? (static_cast<RegList>(1) << code_) : 0;
    }

    unsigned size() const {
        MOZ_ASSERT(IsValid());
        return size_;
    }

    int SizeInBytes() const {
        MOZ_ASSERT(IsValid());
        MOZ_ASSERT(size() % 8 == 0);
        return size_ / 8;
    }

    int SizeInBits() const {
        MOZ_ASSERT(IsValid());
        return size_;
    }

    bool Is32Bits() const {
        MOZ_ASSERT(IsValid());
        return size_ == 32;
    }

    bool Is64Bits() const {
        MOZ_ASSERT(IsValid());
        return size_ == 64;
    }

    bool IsValid() const {
        if (IsValidRegister() || IsValidFPRegister()) {
            MOZ_ASSERT(!IsNone());
            return true;
        }

        MOZ_ASSERT(IsNone());
        return false;
    }

    bool IsValidRegister() const {
        return IsRegister() &&
               ((size_ == kWRegSize) || (size_ == kXRegSize)) &&
               ((code_ < kNumberOfRegisters) || (code_ == kSPRegInternalCode));
    }

    bool IsValidFPRegister() const {
        return IsFPRegister() &&
               ((size_ == kSRegSize) || (size_ == kDRegSize)) &&
               (code_ < kNumberOfFloatRegisters);
    }

    bool IsNone() const {
        // kNoRegister types should always have size 0 and code 0.
        MOZ_ASSERT((type_ != kNoRegister) || (code_ == 0));
        MOZ_ASSERT((type_ != kNoRegister) || (size_ == 0));

        return type_ == kNoRegister;
    }

    bool Aliases(const CPURegister& other) const {
        MOZ_ASSERT(IsValidOrNone() && other.IsValidOrNone());
        return (code_ == other.code_) && (type_ == other.type_);
    }

    bool Is(const CPURegister& other) const {
        MOZ_ASSERT(IsValidOrNone() && other.IsValidOrNone());
        return Aliases(other) && (size_ == other.size_);
    }

    inline bool IsZero() const {
        MOZ_ASSERT(IsValid());
        return IsRegister() && (code_ == kZeroRegCode);
    }

    inline bool IsSP() const {
        MOZ_ASSERT(IsValid());
        return IsRegister() && (code_ == kSPRegInternalCode);
    }

    inline bool IsRegister() const {
        return type_ == kARMRegister;
    }

    inline bool IsFPRegister() const {
        return type_ == kARMFPRegister;
    }

    const ARMRegister& W() const;
    const ARMRegister& X() const;
    const ARMFPRegister& S() const;
    const ARMFPRegister& D() const;

    inline bool IsSameSizeAndType(const CPURegister& other) const {
        return (size_ == other.size_) && (type_ == other.type_);
    }

  protected:
    unsigned code_;
    unsigned size_;
    RegisterType type_;

  private:
    bool IsValidOrNone() const {
        return IsValid() || IsNone();
    }
};
static const CPURegister noReg;

class ARMRegister : public CPURegister
{
  public:
    explicit ARMRegister() : CPURegister() {}

    inline explicit ARMRegister(const CPURegister& other)
      : CPURegister(other.code(), other.size(), other.type())
    {
      MOZ_ASSERT(IsValidRegister());
    }

    MOZ_CONSTEXPR ARMRegister(unsigned code, unsigned size)
      : CPURegister(code, size, kARMRegister)
    { }

    MOZ_CONSTEXPR ARMRegister(Register r, unsigned size)
      : CPURegister(r.code(), size, kARMRegister)
    { }

    bool IsValid() const {
        MOZ_ASSERT(IsRegister() || IsNone());
        return IsValidRegister();
    }

    static const ARMRegister& WRegFromCode(unsigned code);
    static const ARMRegister& XRegFromCode(unsigned code);

    // V8 compatibility.
    static const int kNumRegisters = kNumberOfRegisters;
    static const int kNumAllocatableRegisters = kNumberOfRegisters - 1;

  private:
    static const ARMRegister wregisters[];
    static const ARMRegister xregisters[];
};

class ARMFPRegister : public CPURegister
{
  public:
    inline ARMFPRegister() : CPURegister() {}

    inline explicit ARMFPRegister(const CPURegister& other)
      : CPURegister(other.code(), other.size(), other.type())
    {
      MOZ_ASSERT(IsValidFPRegister());
    }
    MOZ_CONSTEXPR inline ARMFPRegister(FloatRegister r, unsigned size)
        : CPURegister(r.code_, size, kARMFPRegister)
    { }
    MOZ_CONSTEXPR inline ARMFPRegister(FloatRegister r)
        : CPURegister(r.code_, r.size() * 8, kARMFPRegister)
    { }
    MOZ_CONSTEXPR inline ARMFPRegister(unsigned code, unsigned size)
      : CPURegister(code, size, kARMFPRegister)
    { }

    bool IsValid() const {
        MOZ_ASSERT(IsFPRegister() || IsNone());
        return IsValidFPRegister();
    }

    static const ARMFPRegister& SRegFromCode(unsigned code);
    static const ARMFPRegister& DRegFromCode(unsigned code);

    // V8 compatibility.
    static const int kNumRegisters = kNumberOfFloatRegisters;
    static const int kNumAllocatableRegisters = kNumberOfFloatRegisters - 1;

  private:
    static const ARMFPRegister sregisters[];
    static const ARMFPRegister dregisters[];
};

// No*Reg is used to indicate an unused argument, or an error case. Note that
// these all compare equal (using the Is() method). The ARMRegister and ARMFPRegister
// variants are provided for convenience.
const ARMRegister NoReg;
const ARMFPRegister NoFPReg;
const CPURegister NoCPUReg;

#define DEFINE_REGISTERS(N)  \
const ARMRegister w##N(N, kWRegSize);  \
const ARMRegister x##N(N, kXRegSize);
REGISTER_CODE_LIST(DEFINE_REGISTERS)
#undef DEFINE_REGISTERS
const ARMRegister wsp(kSPRegInternalCode, kWRegSize);
const ARMRegister sp(kSPRegInternalCode, kXRegSize);

#define DEFINE_FPREGISTERS(N)  \
const ARMFPRegister s##N(N, kSRegSize);  \
const ARMFPRegister d##N(N, kDRegSize);
REGISTER_CODE_LIST(DEFINE_FPREGISTERS)
#undef DEFINE_FPREGISTERS

// ARMRegisters aliases.
const ARMRegister ip0_64 = x16;
const ARMRegister ip1_64 = x17;
const ARMRegister lr_64 = x30;
const ARMRegister xzr = x31;
const ARMRegister wzr = w31;

// AreAliased returns true if any of the named registers overlap. Arguments
// set to NoReg are ignored. The system stack pointer may be specified.
bool AreAliased(const CPURegister& reg1,
                const CPURegister& reg2,
                const CPURegister& reg3 = NoReg,
                const CPURegister& reg4 = NoReg,
                const CPURegister& reg5 = NoReg,
                const CPURegister& reg6 = NoReg,
                const CPURegister& reg7 = NoReg,
                const CPURegister& reg8 = NoReg);

// AreSameSizeAndType returns true if all of the specified registers have the
// same size, and are of the same type. The system stack pointer may be
// specified. Arguments set to NoReg are ignored, as are any subsequent
// arguments. At least one argument (reg1) must be valid (not NoCPUReg).
bool AreSameSizeAndType(const CPURegister& reg1,
                        const CPURegister& reg2,
                        const CPURegister& reg3 = NoCPUReg,
                        const CPURegister& reg4 = NoCPUReg,
                        const CPURegister& reg5 = NoCPUReg,
                        const CPURegister& reg6 = NoCPUReg,
                        const CPURegister& reg7 = NoCPUReg,
                        const CPURegister& reg8 = NoCPUReg);

// Lists of registers.
class CPURegList
{
  public:
    inline explicit CPURegList(CPURegister reg1,
                               CPURegister reg2 = NoCPUReg,
                               CPURegister reg3 = NoCPUReg,
                               CPURegister reg4 = NoCPUReg)
      : list_(reg1.Bit() | reg2.Bit() | reg3.Bit() | reg4.Bit()),
        size_(reg1.size()), type_(reg1.type())
    {
        MOZ_ASSERT(AreSameSizeAndType(reg1, reg2, reg3, reg4));
        MOZ_ASSERT(IsValid());
    }

    inline CPURegList(CPURegister::RegisterType type, unsigned size, RegList list)
        : list_(list), size_(size), type_(type)
    {
        MOZ_ASSERT(IsValid());
    }

    inline CPURegList(CPURegister::RegisterType type, unsigned size,
                      unsigned first_reg, unsigned last_reg)
      : size_(size), type_(type)
    {
        MOZ_ASSERT(((type == CPURegister::kARMRegister) &&
                     (last_reg < kNumberOfRegisters)) ||
                    ((type == CPURegister::kARMFPRegister) &&
                     (last_reg < kNumberOfFloatRegisters)));
        MOZ_ASSERT(last_reg >= first_reg);
        list_ = (UINT64_C(1) << (last_reg + 1)) - 1;
        list_ &= ~((UINT64_C(1) << first_reg) - 1);
        MOZ_ASSERT(IsValid());
    }

    inline CPURegister::RegisterType type() const {
        MOZ_ASSERT(IsValid());
        return type_;
    }

    // Combine another CPURegList into this one. ARMRegisters that already exist in
    // this list are left unchanged. The type and size of the registers in the
    // 'other' list must match those in this list.
    void Combine(const CPURegList& other) {
        MOZ_ASSERT(IsValid());
        MOZ_ASSERT(other.type() == type_);
        MOZ_ASSERT(other.RegisterSizeInBits() == size_);
        list_ |= other.list();
    }

    // Remove every register in the other CPURegList from this one. ARMRegisters that
    // do not exist in this list are ignored. The type and size of the registers
    // in the 'other' list must match those in this list.
    void Remove(const CPURegList& other) {
        MOZ_ASSERT(IsValid());
        MOZ_ASSERT(other.type() == type_);
        MOZ_ASSERT(other.RegisterSizeInBits() == size_);
        list_ &= ~other.list();
    }

    // Variants of Combine and Remove which take a single register.
    inline void Combine(const CPURegister& other) {
        MOZ_ASSERT(other.type() == type_);
        MOZ_ASSERT(other.size() == size_);
        Combine(other.code());
    }

    inline void Remove(const CPURegister& other) {
        MOZ_ASSERT(other.type() == type_);
        MOZ_ASSERT(other.size() == size_);
        Remove(other.code());
    }

    // Variants of Combine and Remove which take a single register by its code;
    // the type and size of the register is inferred from this list.
    inline void Combine(int code) {
        MOZ_ASSERT(IsValid());
        MOZ_ASSERT(CPURegister(code, size_, type_).IsValid());
        list_ |= (UINT64_C(1) << code);
    }

    inline void Remove(int code) {
        MOZ_ASSERT(IsValid());
        MOZ_ASSERT(CPURegister(code, size_, type_).IsValid());
        list_ &= ~(UINT64_C(1) << code);
    }

    inline RegList list() const {
        MOZ_ASSERT(IsValid());
        return list_;
    }

    inline void set_list(RegList new_list) {
        MOZ_ASSERT(IsValid());
        list_ = new_list;
    }

    // Remove all callee-saved registers from the list. This can be useful when
    // preparing registers for an AAPCS64 function call, for example.
    void RemoveCalleeSaved();

    CPURegister PopLowestIndex();
    CPURegister PopHighestIndex();

    // AAPCS64 callee-saved registers.
    static CPURegList GetCalleeSaved(unsigned size = kXRegSize);
    static CPURegList GetCalleeSavedFP(unsigned size = kDRegSize);

    // AAPCS64 caller-saved registers. Note that this includes lr.
    static CPURegList GetCallerSaved(unsigned size = kXRegSize);
    static CPURegList GetCallerSavedFP(unsigned size = kDRegSize);

    inline bool IsEmpty() const {
        MOZ_ASSERT(IsValid());
        return list_ == 0;
    }

    inline bool IncludesAliasOf(const CPURegister& other) const {
        MOZ_ASSERT(IsValid());
        return (type_ == other.type()) && ((other.Bit() & list_) != 0);
    }

    inline bool IncludesAliasOf(int code) const {
        MOZ_ASSERT(IsValid());
        return ((code & list_) != 0);
    }

    inline int Count() const {
        MOZ_ASSERT(IsValid());
        return CountSetBits(list_, kRegListSizeInBits);
    }

    inline unsigned RegisterSizeInBits() const {
        MOZ_ASSERT(IsValid());
        return size_;
    }

    inline unsigned RegisterSizeInBytes() const {
        int size_in_bits = RegisterSizeInBits();
        MOZ_ASSERT((size_in_bits % 8) == 0);
        return size_in_bits / 8;
    }

    inline unsigned TotalSizeInBytes() const {
        MOZ_ASSERT(IsValid());
        return RegisterSizeInBytes() * Count();
    }

  private:
    RegList list_;
    unsigned size_;
    CPURegister::RegisterType type_;

    bool IsValid() const;
};

// AAPCS64 callee-saved registers.
extern const CPURegList kCalleeSaved;
extern const CPURegList kCalleeSavedFP;

// AAPCS64 caller-saved registers. Note that this includes lr.
extern const CPURegList kCallerSaved;
extern const CPURegList kCallerSavedFP;

// Operand.
class Operand
{
  public:
    // #<immediate>
    // where <immediate> is int64_t.
    // This is allowed to be an implicit constructor because Operand is
    // a wrapper class that doesn't normally perform any type conversion.
    Operand(int64_t immediate);           // NOLINT(runtime/explicit)

    // rm, {<shift> #<shift_amount>}
    // where <shift> is one of {LSL, LSR, ASR, ROR}.
    //       <shift_amount> is uint6_t.
    // This is allowed to be an implicit constructor because Operand is
    // a wrapper class that doesn't normally perform any type conversion.
    Operand(ARMRegister reg,
            Shift shift = LSL,
            unsigned shift_amount = 0);   // NOLINT(runtime/explicit)

    // rm, {<extend> {#<shift_amount>}}
    // where <extend> is one of {UXTB, UXTH, UXTW, UXTX, SXTB, SXTH, SXTW, SXTX}.
    //       <shift_amount> is uint2_t.
    explicit Operand(ARMRegister reg, Extend extend, unsigned shift_amount = 0);

    bool IsImmediate() const;
    bool IsShiftedRegister() const;
    bool IsExtendedRegister() const;
    bool IsZero() const;

    // This returns an LSL shift (<= 4) operand as an equivalent extend operand,
    // which helps in the encoding of instructions that use the stack pointer.
    Operand ToExtendedRegister() const;

    int64_t immediate() const {
        MOZ_ASSERT(IsImmediate());
        return immediate_;
    }

    ARMRegister reg() const {
        MOZ_ASSERT(IsShiftedRegister() || IsExtendedRegister());
        return reg_;
    }

    CPURegister maybeReg() const {
        if (IsShiftedRegister() || IsExtendedRegister())
            return reg_;
        return NoCPUReg;
    }

    Shift shift() const {
        MOZ_ASSERT(IsShiftedRegister());
        return shift_;
    }

    Extend extend() const {
        MOZ_ASSERT(IsExtendedRegister());
        return extend_;
    }

    unsigned shift_amount() const {
        MOZ_ASSERT(IsShiftedRegister() || IsExtendedRegister());
        return shift_amount_;
    }

  private:
    int64_t immediate_;
    ARMRegister reg_;
    Shift shift_;
    Extend extend_;
    unsigned shift_amount_;
};


// MemOperand represents the addressing mode of a load or store instruction.
class MemOperand
{
  public:
    explicit MemOperand(ARMRegister base,
                        ptrdiff_t offset = 0,
                        AddrMode addrmode = Offset);
    explicit MemOperand(ARMRegister base,
                        ARMRegister regoffset,
                        Shift shift = LSL,
                        unsigned shift_amount = 0);
    explicit MemOperand(ARMRegister base,
                        ARMRegister regoffset,
                        Extend extend,
                        unsigned shift_amount = 0);
    explicit MemOperand(ARMRegister base,
                        const Operand& offset,
                        AddrMode addrmode = Offset);
    explicit MemOperand(ptrdiff_t PCoffset);

    // Adapter constructors using C++11 delegating.
    explicit MemOperand(Address addr)
      : MemOperand(ARMRegister(addr.base, 64), (ptrdiff_t)addr.offset)
    { }

    const ARMRegister& base() const { return base_; }
    const ARMRegister& regoffset() const { return regoffset_; }
    ptrdiff_t offset() const { return offset_; }
    AddrMode addrmode() const { return addrmode_; }
    Shift shift() const { return shift_; }
    Extend extend() const { return extend_; }
    unsigned shift_amount() const { return shift_amount_; }
    bool IsImmediateOffset() const;
    bool IsRegisterOffset() const;
    bool IsPreIndex() const;
    bool IsPostIndex() const;

 private:
    ARMRegister base_;
    ARMRegister regoffset_;
    ptrdiff_t offset_;
    AddrMode addrmode_;
    Shift shift_;
    Extend extend_;
    unsigned shift_amount_;
};

#if 0 // Unused: around to preserve interface during porting.
class Label {
 public:
    Label() : is_bound_(false), link_(NULL), target_(NULL) {}
    ~Label() {
      // If the label has been linked to, it needs to be bound to a target.
      MOZ_ASSERT(!IsLinked() || IsBound());
    }

    inline Instruction* link() const { return link_; }
    inline Instruction* target() const { return target_; }

    inline bool IsBound() const { return is_bound_; }
    inline bool IsLinked() const { return link_ != NULL; }

    inline void set_link(Instruction* new_link) { link_ = new_link; }

    static const int kEndOfChain = 0;

 private:
    // Indicates if the label has been bound, ie its location is fixed.
    bool is_bound_;
    // Branches instructions branching to this label form a chained list, with
    // their offset indicating where the next instruction is located.
    // link_ points to the latest branch instruction generated branching to this
    // branch.
    // If link_ is not NULL, the label has been linked to.
    Instruction* link_;
    // The label location.
    Instruction* target_;

    friend class Assembler;
};
#endif

// Control whether or not position-independent code should be emitted.
enum PositionIndependentCodeOption {
    // All code generated will be position-independent; all branches and
    // references to labels generated with the Label class will use PC-relative
    // addressing.
    PositionIndependentCode,
  
    // Allow VIXL to generate code that refers to absolute addresses. With this
    // option, it will not be possible to copy the code buffer and run it from a
    // different address; code must be generated in its final location.
    PositionDependentCode,
  
    // Allow VIXL to assume that the bottom 12 bits of the address will be
    // constant, but that the top 48 bits may change. This allows `adrp` to
    // function in systems which copy code between pages, but otherwise maintain
    // 4KB page alignment.
    PageOffsetDependentCode
};

// Control how scaled- and unscaled-offset loads and stores are generated.
enum LoadStoreScalingOption {
    // Prefer scaled-immediate-offset instructions, but emit unscaled-offset,
    // register-offset, pre-index or post-index instructions if necessary.
    PreferScaledOffset,
  
    // Prefer unscaled-immediate-offset instructions, but emit scaled-offset,
    // register-offset, pre-index or post-index instructions if necessary.
    PreferUnscaledOffset,
  
    // Require scaled-immediate-offset instructions.
    RequireScaledOffset,

    // Require unscaled-immediate-offset instructions.
    RequireUnscaledOffset
};

// Assembler.
class AssemblerVIXL : public AssemblerShared
{
  public:
    AssemblerVIXL()
        // TODO: GetPoolMaxOffset() instead of 1024
        // TODO: GetNopFill() instead of 0x0
        // TODO: Bother to check the rest of the values
        : armbuffer_(1, 1, 8, 1024, 0, BRK | ImmException(0xdead), HINT | ImmHint(0) | Rt(xzr), 0x0), // FIXME: What on earth is this
        pc_(nullptr) // FIXME: Yeah this thing needs some lovin'.
    {
#ifdef DEBUG
        finalized_ = false;
#endif
    }

    // The destructor asserts that one of the following is true:
    //  * The Assembler object has not been used.
    //  * Nothing has been emitted since the last Reset() call.
    //  * Nothing has been emitted since the last FinalizeCode() call.
    ~AssemblerVIXL() {
        // FIXME: Probably useful to assert the above, once we hook up ARMBuffer.
        // MOZ_ASSERT(finalized_ || (pc_ == buffer_));
    }

    // System functions.

    // Helper function for use with the ARMBuffer.
    // We need to wait until an AutoJitContextAlloc is created by the
    // MacroAssembler before allocating any space.
    void initWithAllocator() {
        armbuffer_.initWithAllocator();
    }

    // Start generating code from the beginning of the buffer, discarding any code
    // and data that has already been emitted into the buffer.
    //
    // In order to avoid any accidental transfer of state, Reset ASSERTs that the
    // constant pool is not blocked.
    void Reset();

    // Finalize a code buffer of generated instructions. This function must be
    // called before executing or copying code from the buffer.
    void FinalizeCode();

    // Return the Instruction at a given byte offset.
    Instruction *getInstructionAt(BufferOffset offset) {
        return armbuffer_.getInst(offset);
    }

    // Return the byte offset of a bound label.
    template <typename T>
    inline T GetLabelByteOffset(const Label *label) {
        MOZ_ASSERT(label->bound());
        JS_STATIC_ASSERT(sizeof(T) >= sizeof(uint32_t));
        return reinterpret_cast<T>(label->offset());
    }

    typedef js::jit::Condition Condition;
#define COPYENUM(v) static const Condition v = js::jit::v
#define COPYENUM_(v) static const Condition v = js::jit::v##_
    COPYENUM(Equal);
    COPYENUM(Zero);
    COPYENUM(NotEqual);
    COPYENUM(NonZero);
    COPYENUM(AboveOrEqual);
    COPYENUM(Below);
    COPYENUM(Signed);
    COPYENUM(NotSigned);
    COPYENUM(Overflow);
    COPYENUM(NoOverflow);
    COPYENUM(Above);
    COPYENUM(BelowOrEqual);
    COPYENUM_(GreaterThanOrEqual);
    COPYENUM_(LessThan);
    COPYENUM_(GreaterThan);
    COPYENUM_(LessThanOrEqual);
    COPYENUM(Always);
    COPYENUM(Never);
    // Bit set when a DoubleCondition does not map to a single ARM condition.
    // The MacroAssembler must special-case these conditions, or else
    // ConditionFromDoubleCondition will complain.
    static const int DoubleConditionBitSpecial = 0x100;

    enum DoubleCondition {
        DoubleOrdered                        = Condition::vc,
        DoubleEqual                          = Condition::eq,
        DoubleNotEqual                       = Condition::ne | DoubleConditionBitSpecial,
        DoubleGreaterThan                    = Condition::gt,
        DoubleGreaterThanOrEqual             = Condition::ge,
        DoubleLessThan                       = Condition::lo, // Could also use Condition::mi.
        DoubleLessThanOrEqual                = Condition::ls,

        // If either operand is NaN, these conditions always evaluate to true.
        DoubleUnordered                      = Condition::vs,
        DoubleEqualOrUnordered               = Condition::eq | DoubleConditionBitSpecial,
        DoubleNotEqualOrUnordered            = Condition::ne,
        DoubleGreaterThanOrUnordered         = Condition::hi,
        DoubleGreaterThanOrEqualOrUnordered  = Condition::hs,
        DoubleLessThanOrUnordered            = Condition::lt,
        DoubleLessThanOrEqualOrUnordered     = Condition::le
    };

    static inline Condition InvertCondition(Condition cond) {
        // Conditions al and nv behave identically, as "always true". They can't be
        // inverted, because there is no "always false" condition.
        MOZ_ASSERT((cond != al) && (cond != nv));
        return static_cast<Condition>(cond ^ 1);
    }
    // This is chaging the condition codes for cmp a, b to the same codes for cmp b, a.
    static inline Condition InvertCmpCondition(Condition cond) {
        // Conditions al and nv behave identically, as "always true". They can't be
        // inverted, because there is no "always false" condition.
        switch (cond) {
          case eq:
          case ne:
            return cond;
          case gt:
            return le;
          case le:
            return gt;
          case ge:
            return lt;
          case lt:
            return ge;
          case hi:
            return lo;
          case lo:
            return hi;
          case hs:
            return ls;
          case ls:
            return hs;
          case mi:
            return pl;
          case pl:
            return mi;
          default:
            MOZ_CRASH("TODO: figure this case out.");
        }
        return static_cast<Condition>(cond ^ 1);
    }

    static inline Condition ConditionFromDoubleCondition(DoubleCondition cond) {
        MOZ_ASSERT(!(cond & DoubleConditionBitSpecial));
        return static_cast<Condition>(cond);
    }

    // Instruction set functions.

    // Branch / Jump instructions.
    // Branch to register.

    void br(const ARMRegister& xn);
    static void br(Instruction *at, const ARMRegister& xn);

    // Branch with link to register.
    void blr(const ARMRegister& xn);
    static void blr(Instruction *at, const ARMRegister& xn);
    // Branch to register with return hint.
    void ret(const ARMRegister& xn = lr_64);

    // Unconditional branch to label.
    BufferOffset b(Label* label);

    // Conditional branch to label.
    BufferOffset b(Label* label, Condition cond);

    // Unconditional branch to PC offset.
    BufferOffset b(int imm26);
    static void b(Instruction *at, int imm26);

    // Conditional branch to PC offset.
    BufferOffset b(int imm19, Condition cond);
    static void b(Instruction *at, int imm19, Condition cond);

    // Branch with link to label.
    void bl(Label* label);

    // Branch with link to PC offset.
    void bl(int imm26);
    static void bl(Instruction *at, int imm26);

    // Compare and branch to label if zero.
    void cbz(const ARMRegister& rt, Label* label);

    // Compare and branch to PC offset if zero.
    void cbz(const ARMRegister& rt, int imm19);
    static void cbz(Instruction *at, const ARMRegister& rt, int imm19);

    // Compare and branch to label if not zero.
    void cbnz(const ARMRegister& rt, Label* label);

    // Compare and branch to PC offset if not zero.
    void cbnz(const ARMRegister& rt, int imm19);
    static void cbnz(Instruction *at, const ARMRegister& rt, int imm19);

    // Test bit and branch to label if zero.
    void tbz(const ARMRegister& rt, unsigned bit_pos, Label* label);

    // Test bit and branch to PC offset if zero.
    void tbz(const ARMRegister& rt, unsigned bit_pos, int imm14);
    static void tbz(Instruction *at, const ARMRegister& rt, unsigned bit_pos, int imm14);

    // Test bit and branch to label if not zero.
    void tbnz(const ARMRegister& rt, unsigned bit_pos, Label* label);

    // Test bit and branch to PC offset if not zero.
    void tbnz(const ARMRegister& rt, unsigned bit_pos, int imm14);
    static void tbnz(Instruction *at, const ARMRegister& rt, unsigned bit_pos, int imm14);

    // Address calculation instructions.
    // Calculate a PC-relative address. Unlike for branches the offset in adr is
    // unscaled (i.e. the result can be unaligned).

    // Calculate the address of a label.
    void adr(const ARMRegister& rd, Label* label);

    // Calculate the address of a PC offset.
    void adr(const ARMRegister& rd, int imm21);
    static void adr(Instruction *at, const ARMRegister &rd, int imm21);

    // Calculate the page address of a label.
    void adrp(const ARMRegister& rd, Label* label);

    // Calculate the page address of a PC offset.
    void adrp(const ARMRegister& rd, int imm21);

    // Data Processing instructions.
    // Add.
    void add(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Add and update status flags.
    void adds(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Compare negative.
    void cmn(const ARMRegister& rn, const Operand& operand);

    // Subtract.
    void sub(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Subtract and update status flags.
    void subs(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Compare.
    void cmp(const ARMRegister& rn, const Operand& operand);

    // Negate.
    void neg(const ARMRegister& rd, const Operand& operand);

    // Negate and update status flags.
    void negs(const ARMRegister& rd, const Operand& operand);

    // Add with carry bit.
    void adc(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Add with carry bit and update status flags.
    void adcs(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Subtract with carry bit.
    void sbc(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Subtract with carry bit and update status flags.
    void sbcs(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Negate with carry bit.
    void ngc(const ARMRegister& rd, const Operand& operand);

    // Negate with carry bit and update status flags.
    void ngcs(const ARMRegister& rd, const Operand& operand);

    // Logical instructions.
    // Bitwise and (A & B).
    void and_(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Bitwise and (A & B) and update status flags.
    BufferOffset ands(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Bit test and set flags.
    BufferOffset tst(const ARMRegister& rn, const Operand& operand);

    // Bit clear (A & ~B).
    void bic(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Bit clear (A & ~B) and update status flags.
    void bics(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Bitwise or (A | B).
    void orr(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Bitwise nor (A | ~B).
    void orn(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Bitwise eor/xor (A ^ B).
    void eor(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Bitwise enor/xnor (A ^ ~B).
    void eon(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Logical shift left by variable.
    void lslv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm);

    // Logical shift right by variable.
    void lsrv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm);

    // Arithmetic shift right by variable.
    void asrv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm);

    // Rotate right by variable.
    void rorv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm);

    // Bitfield instructions.
    // Bitfield move.
    void bfm(const ARMRegister& rd, const ARMRegister& rn, unsigned immr, unsigned imms);

    // Signed bitfield move.
    void sbfm(const ARMRegister& rd, const ARMRegister& rn, unsigned immr, unsigned imms);

    // Unsigned bitfield move.
    void ubfm(const ARMRegister& rd, const ARMRegister& rn, unsigned immr, unsigned imms);

    // Bfm aliases.
    // Bitfield insert.
    inline void bfi(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        MOZ_ASSERT(width >= 1);
        MOZ_ASSERT(lsb + width <= rn.size());
        bfm(rd, rn, (rd.size() - lsb) & (rd.size() - 1), width - 1);
    }

    // Bitfield extract and insert low.
    inline void bfxil(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        MOZ_ASSERT(width >= 1);
        MOZ_ASSERT(lsb + width <= rn.size());
        bfm(rd, rn, lsb, lsb + width - 1);
    }

    // Sbfm aliases.
    // Arithmetic shift right.
    inline void asr(const ARMRegister& rd, const ARMRegister& rn, unsigned shift) {
        MOZ_ASSERT(shift < rd.size());
        sbfm(rd, rn, shift, rd.size() - 1);
    }

    // Signed bitfield insert with zero at right.
    inline void sbfiz(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        MOZ_ASSERT(width >= 1);
        MOZ_ASSERT(lsb + width <= rn.size());
        sbfm(rd, rn, (rd.size() - lsb) & (rd.size() - 1), width - 1);
    }

    // Signed bitfield extract.
    inline void sbfx(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        MOZ_ASSERT(width >= 1);
        MOZ_ASSERT(lsb + width <= rn.size());
        sbfm(rd, rn, lsb, lsb + width - 1);
    }

    // Signed extend byte.
    inline void sxtb(const ARMRegister& rd, const ARMRegister& rn) {
        sbfm(rd, rn, 0, 7);
    }

    // Signed extend halfword.
    inline void sxth(const ARMRegister& rd, const ARMRegister& rn) {
        sbfm(rd, rn, 0, 15);
    }

    // Signed extend word.
    inline void sxtw(const ARMRegister& rd, const ARMRegister& rn) {
        sbfm(rd, rn, 0, 31);
    }

    // Ubfm aliases.
    // Logical shift left.
    inline void lsl(const ARMRegister& rd, const ARMRegister& rn, unsigned shift) {
        unsigned reg_size = rd.size();
        MOZ_ASSERT(shift < reg_size);
        ubfm(rd, rn, (reg_size - shift) % reg_size, reg_size - shift - 1);
    }

    // Logical shift right.
    inline void lsr(const ARMRegister& rd, const ARMRegister& rn, unsigned shift) {
        MOZ_ASSERT(shift < rd.size());
        ubfm(rd, rn, shift, rd.size() - 1);
    }

    // Unsigned bitfield insert with zero at right.
    inline void ubfiz(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        MOZ_ASSERT(width >= 1);
        MOZ_ASSERT(lsb + width <= rn.size());
        ubfm(rd, rn, (rd.size() - lsb) & (rd.size() - 1), width - 1);
    }

    // Unsigned bitfield extract.
    inline void ubfx(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        MOZ_ASSERT(width >= 1);
        MOZ_ASSERT(lsb + width <= rn.size());
        ubfm(rd, rn, lsb, lsb + width - 1);
    }

    // Unsigned extend byte.
    inline void uxtb(const ARMRegister& rd, const ARMRegister& rn) {
        ubfm(rd, rn, 0, 7);
    }

    // Unsigned extend halfword.
    inline void uxth(const ARMRegister& rd, const ARMRegister& rn) {
        ubfm(rd, rn, 0, 15);
    }

    // Unsigned extend word.
    inline void uxtw(const ARMRegister& rd, const ARMRegister& rn) {
        ubfm(rd, rn, 0, 31);
    }

    // Extract.
    void extr(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm, unsigned lsb);

    // Conditional select: rd = cond ? rn : rm.
    void csel(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm, Condition cond);

    // Conditional select increment: rd = cond ? rn : rm + 1.
    void csinc(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm, Condition cond);

    // Conditional select inversion: rd = cond ? rn : ~rm.
    void csinv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm, Condition cond);

    // Conditional select negation: rd = cond ? rn : -rm.
    void csneg(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm, Condition cond);

    // Conditional set: rd = cond ? 1 : 0.
    void cset(const ARMRegister& rd, Condition cond);

    // Conditional set mask: rd = cond ? -1 : 0.
    void csetm(const ARMRegister& rd, Condition cond);

    // Conditional increment: rd = cond ? rn + 1 : rn.
    void cinc(const ARMRegister& rd, const ARMRegister& rn, Condition cond);

    // Conditional invert: rd = cond ? ~rn : rn.
    void cinv(const ARMRegister& rd, const ARMRegister& rn, Condition cond);

    // Conditional negate: rd = cond ? -rn : rn.
    void cneg(const ARMRegister& rd, const ARMRegister& rn, Condition cond);

    // Rotate right.
    inline void ror(const ARMRegister& rd, const ARMRegister& rs, unsigned shift) {
        extr(rd, rs, rs, shift);
    }

    // Conditional comparison.
    // Conditional compare negative.
    void ccmn(const ARMRegister& rn, const Operand& operand, StatusFlags nzcv, Condition cond);

    // Conditional compare.
    void ccmp(const ARMRegister& rn, const Operand& operand, StatusFlags nzcv, Condition cond);

    // Multiply.
    void mul(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm);

    // Negated multiply.
    void mneg(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm);

    // Signed long multiply: 32 x 32 -> 64-bit.
    void smull(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm);

    // Signed multiply high: 64 x 64 -> 64-bit <127:64>.
    void smulh(const ARMRegister& xd, const ARMRegister& xn, const ARMRegister& xm);

    // Multiply and accumulate.
    void madd(const ARMRegister& rd, const ARMRegister& rn,
              const ARMRegister& rm, const ARMRegister& ra);

    // Multiply and subtract.
    void msub(const ARMRegister& rd, const ARMRegister& rn,
              const ARMRegister& rm, const ARMRegister& ra);

    // Signed long multiply and accumulate: 32 x 32 + 64 -> 64-bit.
    void smaddl(const ARMRegister& rd, const ARMRegister& rn,
                const ARMRegister& rm, const ARMRegister& ra);

    // Unsigned long multiply and accumulate: 32 x 32 + 64 -> 64-bit.
    void umaddl(const ARMRegister& rd, const ARMRegister& rn,
                const ARMRegister& rm, const ARMRegister& ra);

    // Signed long multiply and subtract: 64 - (32 x 32) -> 64-bit.
    void smsubl(const ARMRegister& rd, const ARMRegister& rn,
                const ARMRegister& rm, const ARMRegister& ra);

    // Unsigned long multiply and subtract: 64 - (32 x 32) -> 64-bit.
    void umsubl(const ARMRegister& rd, const ARMRegister& rn,
                const ARMRegister& rm, const ARMRegister& ra);

    // Signed integer divide.
    void sdiv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm);

    // Unsigned integer divide.
    void udiv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm);

    // Bit reverse.
    void rbit(const ARMRegister& rd, const ARMRegister& rn);

    // Reverse bytes in 16-bit half words.
    void rev16(const ARMRegister& rd, const ARMRegister& rn);

    // Reverse bytes in 32-bit words.
    void rev32(const ARMRegister& rd, const ARMRegister& rn);

    // Reverse bytes.
    void rev(const ARMRegister& rd, const ARMRegister& rn);

    // Count leading zeroes.
    void clz(const ARMRegister& rd, const ARMRegister& rn);

    // Count leading sign bits.
    void cls(const ARMRegister& rd, const ARMRegister& rn);

    // Memory instructions.
    // Load integer or FP register.
    void ldr(const CPURegister& rt, const MemOperand& src,
             LoadStoreScalingOption option = PreferScaledOffset);

    // Store integer or FP register.
    void str(const CPURegister& rt, const MemOperand& dst,
             LoadStoreScalingOption option = PreferScaledOffset);

    // Load word with sign extension.
    void ldrsw(const ARMRegister& rt, const MemOperand& src,
               LoadStoreScalingOption option = PreferScaledOffset);

    // Load byte.
    void ldrb(const ARMRegister& rt, const MemOperand& src,
              LoadStoreScalingOption option = PreferScaledOffset);

    // Store byte.
    void strb(const ARMRegister& rt, const MemOperand& dst,
              LoadStoreScalingOption option = PreferScaledOffset);

    // Load byte with sign extension.
    void ldrsb(const ARMRegister& rt, const MemOperand& src,
               LoadStoreScalingOption option = PreferScaledOffset);

    // Load half-word.
    void ldrh(const ARMRegister& rt, const MemOperand& src,
              LoadStoreScalingOption option = PreferScaledOffset);

    // Store half-word.
    void strh(const ARMRegister& rt, const MemOperand& dst,
              LoadStoreScalingOption option = PreferScaledOffset);

    // Load half-word with sign extension.
    void ldrsh(const ARMRegister& rt, const MemOperand& src,
               LoadStoreScalingOption option = PreferScaledOffset);

    // Load integer or FP register (with unscaled offset).
    void ldur(const CPURegister& rt, const MemOperand& src,
              LoadStoreScalingOption option = PreferUnscaledOffset);
  
    // Store integer or FP register (with unscaled offset).
    void stur(const CPURegister& rt, const MemOperand& src,
              LoadStoreScalingOption option = PreferUnscaledOffset);
  
    // Load word with sign extension.
    void ldursw(const ARMRegister& rt, const MemOperand& src,
                LoadStoreScalingOption option = PreferUnscaledOffset);
  
    // Load byte (with unscaled offset).
    void ldurb(const ARMRegister& rt, const MemOperand& src,
               LoadStoreScalingOption option = PreferUnscaledOffset);
  
    // Store byte (with unscaled offset).
    void sturb(const ARMRegister& rt, const MemOperand& dst,
               LoadStoreScalingOption option = PreferUnscaledOffset);
  
    // Load byte with sign extension (and unscaled offset).
    void ldursb(const ARMRegister& rt, const MemOperand& src,
                LoadStoreScalingOption option = PreferUnscaledOffset);
  
    // Load half-word (with unscaled offset).
    void ldurh(const ARMRegister& rt, const MemOperand& src,
               LoadStoreScalingOption option = PreferUnscaledOffset);
  
    // Store half-word (with unscaled offset).
    void sturh(const ARMRegister& rt, const MemOperand& dst,
               LoadStoreScalingOption option = PreferUnscaledOffset);
  
    // Load half-word with sign extension (and unscaled offset).
    void ldursh(const ARMRegister& rt, const MemOperand& src,
                LoadStoreScalingOption option = PreferUnscaledOffset);

    // Load integer or FP register pair.
    void ldp(const CPURegister& rt, const CPURegister& rt2, const MemOperand& src);

    // Store integer or FP register pair.
    void stp(const CPURegister& rt, const CPURegister& rt2, const MemOperand& dst);

    // Load word pair with sign extension.
    void ldpsw(const ARMRegister& rt, const ARMRegister& rt2, const MemOperand& src);

    // Load integer or FP register pair, non-temporal.
    void ldnp(const CPURegister& rt, const CPURegister& rt2, const MemOperand& src);

    // Store integer or FP register pair, non-temporal.
    void stnp(const CPURegister& rt, const CPURegister& rt2, const MemOperand& dst);

    // Load integer or FP register from pc + imm19 << 2.
    void ldr(const CPURegister& rt, int imm19);
    static void ldr(Instruction *at, const CPURegister& rt, int imm19);

    // Load word with sign extension from pc + imm19 << 2.
    void ldrsw(const ARMRegister &rt, int imm19);

    // Store exclusive byte.
    void stxrb(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst);
  
    // Store exclusive half-word.
    void stxrh(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst);
  
    // Store exclusive register.
    void stxr(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst);
  
    // Load exclusive byte.
    void ldxrb(const ARMRegister& rt, const MemOperand& src);
  
    // Load exclusive half-word.
    void ldxrh(const ARMRegister& rt, const MemOperand& src);
  
    // Load exclusive register.
    void ldxr(const ARMRegister& rt, const MemOperand& src);
  
    // Store exclusive register pair.
    void stxp(const ARMRegister& rs, const ARMRegister& rt,
              const ARMRegister& rt2, const MemOperand& dst);
  
    // Load exclusive register pair.
    void ldxp(const ARMRegister& rt, const ARMRegister& rt2, const MemOperand& src);
  
    // Store-release exclusive byte.
    void stlxrb(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst);
  
    // Store-release exclusive half-word.
    void stlxrh(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst);
  
    // Store-release exclusive register.
    void stlxr(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst);
  
    // Load-acquire exclusive byte.
    void ldaxrb(const ARMRegister& rt, const MemOperand& src);
  
    // Load-acquire exclusive half-word.
    void ldaxrh(const ARMRegister& rt, const MemOperand& src);
  
    // Load-acquire exclusive register.
    void ldaxr(const ARMRegister& rt, const MemOperand& src);
  
    // Store-release exclusive register pair.
    void stlxp(const ARMRegister& rs, const ARMRegister& rt,
               const ARMRegister& rt2, const MemOperand& dst);
  
    // Load-acquire exclusive register pair.
    void ldaxp(const ARMRegister& rt, const ARMRegister& rt2, const MemOperand& src);
  
    // Store-release byte.
    void stlrb(const ARMRegister& rt, const MemOperand& dst);
  
    // Store-release half-word.
    void stlrh(const ARMRegister& rt, const MemOperand& dst);
  
    // Store-release register.
    void stlr(const ARMRegister& rt, const MemOperand& dst);
  
    // Load-acquire byte.
    void ldarb(const ARMRegister& rt, const MemOperand& src);
  
    // Load-acquire half-word.
    void ldarh(const ARMRegister& rt, const MemOperand& src);
  
    // Load-acquire register.
    void ldar(const ARMRegister& rt, const MemOperand& src);

    // Move instructions. The default shift of -1 indicates that the move
    // instruction will calculate an appropriate 16-bit immediate and left shift
    // that is equal to the 64-bit immediate argument. If an explicit left shift
    // is specified (0, 16, 32 or 48), the immediate must be a 16-bit value.
    //
    // For movk, an explicit shift can be used to indicate which half word should
    // be overwritten, eg. movk(x0, 0, 0) will overwrite the least-significant
    // half word with zero, whereas movk(x0, 0, 48) will overwrite the
    // most-significant.

    // Move immediate and keep.
    void movk(const ARMRegister& rd, uint64_t imm, int shift = -1) {
        MoveWide(rd, imm, shift, MOVK);
    }

    // Move inverted immediate.
    void movn(const ARMRegister& rd, uint64_t imm, int shift = -1) {
        MoveWide(rd, imm, shift, MOVN);
    }

    // Move immediate.
    void movz(const ARMRegister& rd, uint64_t imm, int shift = -1) {
        MoveWide(rd, imm, shift, MOVZ);
    }

    // Misc instructions.
    // Monitor debug-mode breakpoint.
    void brk(int code);

    // System exception. Used for simulator directives.
    void svc(int code);
    static void svc(Instruction *at, int code);

    // Halting debug-mode breakpoint.
    void hlt(int code);

    // Move register to register.
    void mov(const ARMRegister& rd, const ARMRegister& rn);

    // Move inverted operand to register.
    void mvn(const ARMRegister& rd, const Operand& operand);

    // System instructions.
    // Move to register from system register.
    void mrs(const ARMRegister& rt, SystemRegister sysreg);

    // Move from register to system register.
    void msr(SystemRegister sysreg, const ARMRegister& rt);

    // System hint.
    void hint(SystemHint code);
    static void hint(Instruction *at, SystemHint code);
    // Clear exclusive monitor.
    void clrex(int imm4 = 0xf);

    // Data memory barrier.
    void dmb(BarrierDomain domain, BarrierType type);

    // Data synchronization barrier.
    void dsb(BarrierDomain domain, BarrierType type);

    // Instruction synchronization barrier.
    void isb();

    // Alias for system instructions.
    // No-op.
    void nop() {
        hint(NOP);
    }
    static void nop(Instruction *at);
    // FP instructions.
    // Move double precision immediate to FP register.
    void fmov(const ARMFPRegister& fd, double imm);

    // Move single precision immediate to FP register.
    void fmov(const ARMFPRegister& fd, float imm);

    // Move FP register to register.
    void fmov(const ARMRegister& rd, const ARMFPRegister& fn);

    // Move register to FP register.
    void fmov(const ARMFPRegister& fd, const ARMRegister& rn);

    // Move FP register to FP register.
    void fmov(const ARMFPRegister& fd, const ARMFPRegister& fn);

    // FP add.
    void fadd(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm);

    // FP subtract.
    void fsub(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm);

    // FP multiply.
    void fmul(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm);

    // FP fused multiply and add.
    void fmadd(const ARMFPRegister& fd, const ARMFPRegister& fn,
               const ARMFPRegister& fm, const ARMFPRegister& fa);

    // FP fused multiply and subtract.
    void fmsub(const ARMFPRegister& fd, const ARMFPRegister& fn,
               const ARMFPRegister& fm, const ARMFPRegister& fa);

    // FP fused multiply, add and negate.
    void fnmadd(const ARMFPRegister& fd, const ARMFPRegister& fn,
                const ARMFPRegister& fm, const ARMFPRegister& fa);

    // FP fused multiply, subtract and negate.
    void fnmsub(const ARMFPRegister& fd, const ARMFPRegister& fn,
                const ARMFPRegister& fm, const ARMFPRegister& fa);

    // FP divide.
    void fdiv(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm);

    // FP maximum.
    void fmax(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm);

    // FP minimum.
    void fmin(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm);

    // FP maximum number.
    void fmaxnm(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm);

    // FP minimum number.
    void fminnm(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm);

    // FP absolute.
    void fabs(const ARMFPRegister& fd, const ARMFPRegister& fn);

    // FP negate.
    void fneg(const ARMFPRegister& fd, const ARMFPRegister& fn);

    // FP square root.
    void fsqrt(const ARMFPRegister& fd, const ARMFPRegister& fn);

    // FP round to integer (nearest with ties to away).
    void frinta(const ARMFPRegister& fd, const ARMFPRegister& fn);

    // FP round to integer (toward minus infinity).
    void frintm(const ARMFPRegister& fd, const ARMFPRegister& fn);

    // FP round to integer (nearest with ties to even).
    void frintn(const ARMFPRegister& fd, const ARMFPRegister& fn);

    // FP round to integer (towards zero).
    void frintz(const ARMFPRegister& fd, const ARMFPRegister& fn);

    // FP compare registers.
    void fcmp(const ARMFPRegister& fn, const ARMFPRegister& fm);

    // FP compare immediate.
    void fcmp(const ARMFPRegister& fn, double value);

    // FP conditional compare.
    void fccmp(const ARMFPRegister& fn, const ARMFPRegister& fm, StatusFlags nzcv, Condition cond);

    // FP conditional select.
    void fcsel(const ARMFPRegister& fd, const ARMFPRegister& fn,
               const ARMFPRegister& fm, Condition cond);

    // Common FP Convert function.
    void FPConvertToInt(const ARMRegister& rd, const ARMFPRegister& fn, FPIntegerConvertOp op);

    // FP convert between single and double precision.
    void fcvt(const ARMFPRegister& fd, const ARMFPRegister& fn);

    // Convert FP to signed integer (nearest with ties to away).
    void fcvtas(const ARMRegister& rd, const ARMFPRegister& fn);

    // Convert FP to unsigned integer (nearest with ties to away).
    void fcvtau(const ARMRegister& rd, const ARMFPRegister& fn);

    // Convert FP to signed integer (round towards -infinity).
    void fcvtms(const ARMRegister& rd, const ARMFPRegister& fn);

    // Convert FP to unsigned integer (round towards -infinity).
    void fcvtmu(const ARMRegister& rd, const ARMFPRegister& fn);

    // Convert FP to signed integer (nearest with ties to even).
    void fcvtns(const ARMRegister& rd, const ARMFPRegister& fn);

    // Convert FP to unsigned integer (nearest with ties to even).
    void fcvtnu(const ARMRegister& rd, const ARMFPRegister& fn);

    // Convert FP to signed integer (round towards zero).
    void fcvtzs(const ARMRegister& rd, const ARMFPRegister& fn);

    // Convert FP to unsigned integer (round towards zero).
    void fcvtzu(const ARMRegister& rd, const ARMFPRegister& fn);

    // Convert signed integer or fixed point to FP.
    void scvtf(const ARMFPRegister& fd, const ARMRegister& rn, unsigned fbits = 0);

    // Convert unsigned integer or fixed point to FP.
    void ucvtf(const ARMFPRegister& fd, const ARMRegister& rn, unsigned fbits = 0);

    // Emit generic instructions.
    // Emit raw instructions into the instruction stream.
    inline void dci(Instr raw_inst) { Emit(raw_inst); }

    // Emit 32 bits of data into the instruction stream.
    inline void dc32(uint32_t data) { EmitData(&data, sizeof(data)); }

    // Emit 64 bits of data into the instruction stream.
    inline void dc64(uint64_t data) { EmitData(&data, sizeof(data)); }

    // Copy a string into the instruction stream, including the terminating NULL
    // character. The instruction pointer (pc_) is then aligned correctly for
    // subsequent instructions.
    void EmitStringData(const char * string) {
        MOZ_ASSERT(string != NULL);

        size_t len = strlen(string) + 1;
        EmitData(string, len);

        // Pad with NULL characters until pc_ is aligned.
        const char pad[] = {'\0', '\0', '\0', '\0'};
        JS_STATIC_ASSERT(sizeof(pad) == kInstructionSize);
        Instruction* next_pc = AlignUp(pc_, kInstructionSize);
        EmitData(&pad, next_pc - pc_);
    }

    // Code generation helpers.

    // ARMRegister encoding.
    static Instr Rd(CPURegister rd) {
        MOZ_ASSERT(rd.code() != kSPRegInternalCode);
        return rd.code() << Rd_offset;
    }

    static Instr Rn(CPURegister rn) {
        MOZ_ASSERT(rn.code() != kSPRegInternalCode);
        return rn.code() << Rn_offset;
    }

    static Instr Rm(CPURegister rm) {
        MOZ_ASSERT(rm.code() != kSPRegInternalCode);
        return rm.code() << Rm_offset;
    }

    static Instr Ra(CPURegister ra) {
        MOZ_ASSERT(ra.code() != kSPRegInternalCode);
        return ra.code() << Ra_offset;
    }

    static Instr Rt(CPURegister rt) {
        MOZ_ASSERT(rt.code() != kSPRegInternalCode);
        return rt.code() << Rt_offset;
    }

    static Instr Rt2(CPURegister rt2) {
        MOZ_ASSERT(rt2.code() != kSPRegInternalCode);
        return rt2.code() << Rt2_offset;
    }

    static Instr Rs(CPURegister rs) {
        MOZ_ASSERT(rs.code() != kSPRegInternalCode);
        return rs.code() << Rs_offset;
    }

    // These encoding functions allow the stack pointer to be encoded, and
    // disallow the zero register.
    static Instr RdSP(ARMRegister rd) {
        MOZ_ASSERT(!rd.IsZero());
        return (rd.code() & kRegCodeMask) << Rd_offset;
    }

    static Instr RnSP(ARMRegister rn) {
        MOZ_ASSERT(!rn.IsZero());
        return (rn.code() & kRegCodeMask) << Rn_offset;
    }

    // Flags encoding.
    static Instr Flags(FlagsUpdate S) {
        if (S == SetFlags)
            return 1 << FlagsUpdate_offset;

        if (S == LeaveFlags)
            return 0 << FlagsUpdate_offset;

        VIXL_UNREACHABLE();
        return 0;
    }

    static Instr Cond(Condition cond) {
        return cond << Condition_offset;
    }

    // PC-relative address encoding.
    static Instr ImmPCRelAddress(int imm21) {
        MOZ_ASSERT(is_int21(imm21));
        Instr imm = static_cast<Instr>(truncate_to_int21(imm21));
        Instr immhi = (imm >> ImmPCRelLo_width) << ImmPCRelHi_offset;
        Instr immlo = imm << ImmPCRelLo_offset;
        return (immhi & ImmPCRelHi_mask) | (immlo & ImmPCRelLo_mask);
    }

    // Branch encoding.
    static Instr ImmUncondBranch(int imm26) {
        MOZ_ASSERT(is_int26(imm26));
        return truncate_to_int26(imm26) << ImmUncondBranch_offset;
    }

    static Instr ImmCondBranch(int imm19) {
        MOZ_ASSERT(is_int19(imm19));
        return truncate_to_int19(imm19) << ImmCondBranch_offset;
    }

    static Instr ImmCmpBranch(int imm19) {
        MOZ_ASSERT(is_int19(imm19));
        return truncate_to_int19(imm19) << ImmCmpBranch_offset;
    }

    static Instr ImmTestBranch(int imm14) {
        MOZ_ASSERT(is_int14(imm14));
        return truncate_to_int14(imm14) << ImmTestBranch_offset;
    }

    static Instr ImmTestBranchBit(unsigned bit_pos) {
        MOZ_ASSERT(is_uint6(bit_pos));
        // Subtract five from the shift offset, as we need bit 5 from bit_pos.
        unsigned b5 = bit_pos << (ImmTestBranchBit5_offset - 5);
        unsigned b40 = bit_pos << ImmTestBranchBit40_offset;
        b5 &= ImmTestBranchBit5_mask;
        b40 &= ImmTestBranchBit40_mask;
        return b5 | b40;
    }

    // Data Processing encoding.
    static Instr SF(ARMRegister rd) {
        return rd.Is64Bits() ? SixtyFourBits : ThirtyTwoBits;
    }

    static Instr ImmAddSub(int64_t imm) {
        MOZ_ASSERT(IsImmAddSub(imm));
        if (is_uint12(imm)) // No shift required.
            return imm << ImmAddSub_offset;
        return ((imm >> 12) << ImmAddSub_offset) | (1 << ShiftAddSub_offset);
    }

    static inline Instr ImmS(unsigned imms, unsigned reg_size) {
        MOZ_ASSERT(((reg_size == kXRegSize) && is_uint6(imms)) ||
                    ((reg_size == kWRegSize) && is_uint5(imms)));
        USEARG(reg_size);
        return imms << ImmS_offset;
    }

    static inline Instr ImmR(unsigned immr, unsigned reg_size) {
        MOZ_ASSERT(((reg_size == kXRegSize) && is_uint6(immr)) ||
                    ((reg_size == kWRegSize) && is_uint5(immr)));
        USEARG(reg_size);
        MOZ_ASSERT(is_uint6(immr));
        return immr << ImmR_offset;
    }

    static inline Instr ImmSetBits(unsigned imms, unsigned reg_size) {
        MOZ_ASSERT((reg_size == kWRegSize) || (reg_size == kXRegSize));
        MOZ_ASSERT(is_uint6(imms));
        MOZ_ASSERT((reg_size == kXRegSize) || is_uint6(imms + 3));
        USEARG(reg_size);
        return imms << ImmSetBits_offset;
    }

    static inline Instr ImmRotate(unsigned immr, unsigned reg_size) {
        MOZ_ASSERT((reg_size == kWRegSize) || (reg_size == kXRegSize));
        MOZ_ASSERT(((reg_size == kXRegSize) && is_uint6(immr)) ||
                    ((reg_size == kWRegSize) && is_uint5(immr)));
        USEARG(reg_size);
        return immr << ImmRotate_offset;
    }

    static inline Instr ImmLLiteral(int imm19) {
        MOZ_ASSERT(is_int19(imm19));
        return truncate_to_int19(imm19) << ImmLLiteral_offset;
    }

    static inline Instr BitN(unsigned bitn, unsigned reg_size) {
        MOZ_ASSERT((reg_size == kWRegSize) || (reg_size == kXRegSize));
        MOZ_ASSERT((reg_size == kXRegSize) || (bitn == 0));
        USEARG(reg_size);
        return bitn << BitN_offset;
    }

    static Instr ShiftDP(Shift shift) {
        MOZ_ASSERT(shift == LSL || shift == LSR || shift == ASR || shift == ROR);
        return shift << ShiftDP_offset;
    }

    static Instr ImmDPShift(unsigned amount) {
        MOZ_ASSERT(is_uint6(amount));
        return amount << ImmDPShift_offset;
    }

    static Instr ExtendMode(Extend extend) {
        return extend << ExtendMode_offset;
    }

    static Instr ImmExtendShift(unsigned left_shift) {
        MOZ_ASSERT(left_shift <= 4);
        return left_shift << ImmExtendShift_offset;
    }

    static Instr ImmCondCmp(unsigned imm) {
        MOZ_ASSERT(is_uint5(imm));
        return imm << ImmCondCmp_offset;
    }

    static Instr Nzcv(StatusFlags nzcv) {
        return ((nzcv >> Flags_offset) & 0xf) << Nzcv_offset;
    }

    // MemOperand offset encoding.
    static Instr ImmLSUnsigned(int imm12) {
        MOZ_ASSERT(is_uint12(imm12));
        return imm12 << ImmLSUnsigned_offset;
    }

    static Instr ImmLS(int imm9) {
        MOZ_ASSERT(is_int9(imm9));
        return truncate_to_int9(imm9) << ImmLS_offset;
    }

    static Instr ImmLSPair(int imm7, LSDataSize size) {
        MOZ_ASSERT(((imm7 >> size) << size) == imm7);
        int scaled_imm7 = imm7 >> size;
        MOZ_ASSERT(is_int7(scaled_imm7));
        return truncate_to_int7(scaled_imm7) << ImmLSPair_offset;
    }

    static Instr ImmShiftLS(unsigned shift_amount) {
        MOZ_ASSERT(is_uint1(shift_amount));
        return shift_amount << ImmShiftLS_offset;
    }

    static Instr ImmException(int imm16) {
        MOZ_ASSERT(is_uint16(imm16));
        return imm16 << ImmException_offset;
    }

    static Instr ImmSystemRegister(int imm15) {
        MOZ_ASSERT(is_uint15(imm15));
        return imm15 << ImmSystemRegister_offset;
    }

    static Instr ImmHint(int imm7) {
        MOZ_ASSERT(is_uint7(imm7));
        return imm7 << ImmHint_offset;
    }

    static Instr CRm(int imm4) {
        MOZ_ASSERT(is_uint4(imm4));
        return imm4 << CRm_offset;
    }

    static Instr ImmBarrierDomain(int imm2) {
        MOZ_ASSERT(is_uint2(imm2));
        return imm2 << ImmBarrierDomain_offset;
    }

    static Instr ImmBarrierType(int imm2) {
        MOZ_ASSERT(is_uint2(imm2));
        return imm2 << ImmBarrierType_offset;
    }

    static LSDataSize CalcLSDataSize(LoadStoreOp op) {
        MOZ_ASSERT((SizeLS_offset + SizeLS_width) == (kInstructionSize * 8));
        return static_cast<LSDataSize>(op >> SizeLS_offset);
    }

    // Move immediates encoding.
    static Instr ImmMoveWide(uint64_t imm) {
        MOZ_ASSERT(is_uint16(imm));
        return imm << ImmMoveWide_offset;
    }

    static Instr ShiftMoveWide(int64_t shift) {
        MOZ_ASSERT(is_uint2(shift));
        return shift << ShiftMoveWide_offset;
    }

    // FP Immediates.
    static Instr ImmFP32(float imm);
    static Instr ImmFP64(double imm);

    // FP register type.
    static Instr FPType(ARMFPRegister fd) {
        return fd.Is64Bits() ? FP64 : FP32;
    }

    static Instr FPScale(unsigned scale) {
        MOZ_ASSERT(is_uint6(scale));
        return scale << FPScale_offset;
    }

    size_t size() const {
        return SizeOfCodeGenerated();
    }

    // Size of the code generated in bytes, including pools.
    size_t SizeOfCodeGenerated() const {
        return armbuffer_.size();
    }

    inline PositionIndependentCodeOption pic() {
        return pic_;
    }
  
    inline bool AllowPageOffsetDependentCode() {
        return (pic() == PageOffsetDependentCode) ||
               (pic() == PositionDependentCode);
    }

  protected:
    inline const ARMRegister& AppropriateZeroRegFor(const CPURegister& reg) const {
        return reg.Is64Bits() ? xzr : wzr;
    }

    BufferOffset LoadStore(const CPURegister& rt, const MemOperand& addr, LoadStoreOp op,
                   LoadStoreScalingOption option = PreferScaledOffset);

    static bool IsImmLSUnscaled(ptrdiff_t offset);
    static bool IsImmLSScaled(ptrdiff_t offset, LSDataSize size);

    BufferOffset Logical(const ARMRegister& rd, const ARMRegister& rn,
                 const Operand& operand, LogicalOp op);
    BufferOffset LogicalImmediate(const ARMRegister& rd, const ARMRegister& rn, unsigned n,
                          unsigned imm_s, unsigned imm_r, LogicalOp op);
    static bool IsImmLogical(uint64_t value, unsigned width, unsigned* n = nullptr,
                             unsigned* imm_s = nullptr, unsigned* imm_r = nullptr);

    void ConditionalCompare(const ARMRegister& rn, const Operand& operand, StatusFlags nzcv,
                            Condition cond, ConditionalCompareOp op);
    static bool IsImmConditionalCompare(int64_t immediate);

    void AddSubWithCarry(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand,
                         FlagsUpdate S, AddSubWithCarryOp op);

    static bool IsImmFP32(float imm);
    static bool IsImmFP64(double imm);

    // Functions for emulating operands not directly supported by the instruction
    // set.
    void EmitShift(const ARMRegister& rd, const ARMRegister& rn, Shift shift, unsigned amount);
    void EmitExtendShift(const ARMRegister& rd, const ARMRegister& rn,
                         Extend extend, unsigned left_shift);

    void AddSub(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand,
                FlagsUpdate S, AddSubOp op);
    static bool IsImmAddSub(int64_t immediate);

    // Find an appropriate LoadStoreOp or LoadStorePairOp for the specified
    // registers. Only simple loads are supported; sign- and zero-extension (such
    // as in LDPSW_x or LDRB_w) are not supported.
    static LoadStoreOp LoadOpFor(const CPURegister& rt);
    static LoadStorePairOp LoadPairOpFor(const CPURegister& rt, const CPURegister& rt2);
    static LoadStoreOp StoreOpFor(const CPURegister& rt);
    static LoadStorePairOp StorePairOpFor(const CPURegister& rt, const CPURegister& rt2);

    static LoadStorePairNonTemporalOp LoadPairNonTemporalOpFor(
      const CPURegister& rt, const CPURegister& rt2);

    static LoadStorePairNonTemporalOp StorePairNonTemporalOpFor(
      const CPURegister& rt, const CPURegister& rt2);

    static LoadLiteralOp LoadLiteralOpFor(const CPURegister& rt);


  private:
    // Instruction helpers.
    void MoveWide(const ARMRegister& rd, uint64_t imm, int shift, MoveWideImmediateOp mov_op);
    BufferOffset DataProcShiftedRegister(const ARMRegister& rd, const ARMRegister& rn,
                                 const Operand& operand, FlagsUpdate S, Instr op);
    void DataProcExtendedRegister(const ARMRegister& rd, const ARMRegister& rn,
                                  const Operand& operand, FlagsUpdate S, Instr op);
    void LoadStorePair(const CPURegister& rt, const CPURegister& rt2,
                       const MemOperand& addr, LoadStorePairOp op);
    void LoadStorePairNonTemporal(const CPURegister& rt, const CPURegister& rt2,
                                  const MemOperand& addr, LoadStorePairNonTemporalOp op);
    void LoadLiteral(const CPURegister& rt, uint64_t imm, LoadLiteralOp op);
    void LoadPCLiteral(const CPURegister& rt, ptrdiff_t PCInsOffset, LoadLiteralOp op);
    void ConditionalSelect(const ARMRegister& rd, const ARMRegister& rn,
                           const ARMRegister& rm, Condition cond, ConditionalSelectOp op);
    void DataProcessing1Source(const ARMRegister& rd, const ARMRegister& rn,
                               DataProcessing1SourceOp op);
    void DataProcessing3Source(const ARMRegister& rd, const ARMRegister& rn,
                               const ARMRegister& rm, const ARMRegister& ra,
                               DataProcessing3SourceOp op);
    void FPDataProcessing1Source(const ARMFPRegister& fd, const ARMFPRegister& fn,
                                 FPDataProcessing1SourceOp op);
    void FPDataProcessing2Source(const ARMFPRegister& fd, const ARMFPRegister& fn,
                                 const ARMFPRegister& fm, FPDataProcessing2SourceOp op);
    void FPDataProcessing3Source(const ARMFPRegister& fd, const ARMFPRegister& fn,
                                 const ARMFPRegister& fm, const ARMFPRegister& fa,
                                 FPDataProcessing3SourceOp op);

    void RecordLiteral(int64_t imm, unsigned size);

    // Link the current (not-yet-emitted) instruction to the specified label, then
    // return an offset to be encoded in the instruction. If the label is not yet
    // bound, an offset of LabelBase::INVALID_OFFSET is returned.
    ptrdiff_t LinkAndGetByteOffsetTo(BufferOffset branch, Label *label);
    ptrdiff_t LinkAndGetInstructionOffsetTo(BufferOffset branch, Label *label);
    ptrdiff_t LinkAndGetPageOffsetTo(BufferOffset branch, Label *label);

    // A common implementation for the LinkAndGet<Type>OffsetTo helpers.
    template <int element_size>
    ptrdiff_t LinkAndGetOffsetTo(BufferOffset branch, Label *label);

    // Emit the instruction, returning its offset.
    BufferOffset Emit(Instr instruction, bool isBranch = false) {
        JS_STATIC_ASSERT(sizeof(*pc_) == 1);
        JS_STATIC_ASSERT(sizeof(instruction) == kInstructionSize);
        // TODO: MOZ_ASSERT((pc_ + sizeof(instruction)) <= (buffer_ + buffer_size_));

#ifdef DEBUG
        finalized_ = false;
#endif

        pc_ += sizeof(instruction);
        return armbuffer_.putInt(*(uint32_t*)(&instruction), isBranch);
    }

    BufferOffset EmitBranch(Instr instruction) {
        BufferOffset ret = Emit(instruction, /* isBranch = */ true);
        return ret;
    }

  // FIXME: This interface should not be public.
  public:
    // Emit the instruction at |at|.
    static void Emit(Instruction *at, Instr instruction) {
        JS_STATIC_ASSERT(sizeof(instruction) == kInstructionSize);
        memcpy(at, &instruction, sizeof(instruction));
    }

    static void EmitBranch(Instruction *at, Instr instruction) {
        // FIXME: Anything special to do here? Probably not, since this is actually
        // just overwriting an instruction. Maybe rename from Emit() to Overwrite()?
        Emit(at, instruction);
    }

  public:
    // Emit data inline in the instruction stream.
    BufferOffset EmitData(void const * data, unsigned size) {
        JS_STATIC_ASSERT(sizeof(*pc_) == 1);
        MOZ_ASSERT(size % 4 == 0);
        pc_ += 1;
        BufferOffset ret = armbuffer_.allocEntry(size / sizeof(uint32_t), 0, (uint8_t*)(data), nullptr);
        // TODO: MOZ_ASSERT((pc_ + size) <= (buffer_ + buffer_size_));

#ifdef DEBUG
        finalized_ = false;
#endif
        return ret;

    }
  private:
    inline void CheckBufferSpace() {
        MOZ_ASSERT(!armbuffer_.oom());
        // TODO: MOZ_ASSERT(pc_ < (buffer_ + buffer_size_));
        // FIXME: Integration with constant pool?
    }

  public:
    // Interface used by IonAssemblerBufferWithConstantPools.
    static void InsertIndexIntoTag(uint8_t *load, uint32_t index);
    static bool PatchConstantPoolLoad(void *loadAddr, void *constPoolAddr);
    static uint32_t PlaceConstantPoolBarrier(int offset) {
        MOZ_CRASH("PlaceConstantPoolBarrier");
    }
    static void WritePoolHeader(uint8_t *start, Pool *p, bool isNatural);
    static void WritePoolFooter(uint8_t *start, Pool *p, bool isNatural);
    static void WritePoolGuard(BufferOffset branch, Instruction *inst, BufferOffset dest);

    // Static interface used by IonAssemblerBufferWithConstantPools.
    static ptrdiff_t GetBranchOffset(const Instruction *i);
    static void RetargetNearBranch(Instruction *i, int offset, Condition cond, bool final = true);
    static void RetargetNearBranch(Instruction *i, int offset, bool final = true);
    static void RetargetFarBranch(Instruction *i, uint8_t **slot, uint8_t *dest, Condition cond);

  protected:
    // Prevent generation of a literal pool for the next |maxInst| instructions.
    // Guarantees instruction linearity.
    class AutoBlockLiteralPool
    {
        ARMBuffer *armbuffer_;

      public:
        AutoBlockLiteralPool(AssemblerVIXL *assembler, size_t maxInst)
          : armbuffer_(&assembler->armbuffer_)
        {
            armbuffer_->enterNoPool(maxInst);
        }
        ~AutoBlockLiteralPool() {
            armbuffer_->leaveNoPool();
        }
    };

  protected:
    // The buffer into which code and relocation info are generated.
    ARMBuffer armbuffer_;

    CompactBufferWriter jumpRelocations_;
    CompactBufferWriter dataRelocations_;
    CompactBufferWriter relocations_;
    CompactBufferWriter preBarriers_;

    // Literal pools.
    mozilla::Array<Pool, 4> pools_;

    PositionIndependentCodeOption pic_;

  protected:
    // Pointer of current instruction (into the ARMBuffer).
    Instruction* pc_;

#ifdef DEBUG
    bool finalized_;
#endif
};

} // namespace jit
} // namespace js

#endif  // VIXL_A64_ASSEMBLER_A64_H_
