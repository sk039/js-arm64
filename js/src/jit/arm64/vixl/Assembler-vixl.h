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
#include "jit/shared/Assembler-shared.h"
#include "jit/shared/IonAssemblerBufferWithConstantPools.h"
#include "jit/IonSpewer.h"

namespace js {
namespace jit {

// Exciting buffer logic, before it gets replaced with the new hotness.
class AssemblerVIXL;
typedef js::jit::AssemblerBufferWithConstantPool<1024, 4, Instruction, AssemblerVIXL, 1> ARMBuffer;

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
        VIXL_ASSERT(IsValid());
        return code_;
    }

    RegisterType type() const {
        VIXL_ASSERT(IsValidOrNone());
        return type_;
    }

    RegList Bit() const {
        VIXL_ASSERT(code_ < (sizeof(RegList) * 8));
        return IsValid() ? (static_cast<RegList>(1) << code_) : 0;
    }

    unsigned size() const {
        VIXL_ASSERT(IsValid());
        return size_;
    }

    int SizeInBytes() const {
        VIXL_ASSERT(IsValid());
        VIXL_ASSERT(size() % 8 == 0);
        return size_ / 8;
    }

    int SizeInBits() const {
        VIXL_ASSERT(IsValid());
        return size_;
    }

    bool Is32Bits() const {
        VIXL_ASSERT(IsValid());
        return size_ == 32;
    }

    bool Is64Bits() const {
        VIXL_ASSERT(IsValid());
        return size_ == 64;
    }

    bool IsValid() const {
        if (IsValidRegister() || IsValidARMFPRegister()) {
            VIXL_ASSERT(!IsNone());
            return true;
        }

        VIXL_ASSERT(IsNone());
        return false;
    }

    bool IsValidRegister() const {
        return IsRegister() &&
               ((size_ == kWRegSize) || (size_ == kXRegSize)) &&
               ((code_ < kNumberOfRegisters) || (code_ == kSPRegInternalCode));
    }

    bool IsValidARMFPRegister() const {
        return IsARMFPRegister() &&
               ((size_ == kSRegSize) || (size_ == kDRegSize)) &&
               (code_ < kNumberOfFloatRegisters);
    }

    bool IsNone() const {
        // kNoRegister types should always have size 0 and code 0.
        VIXL_ASSERT((type_ != kNoRegister) || (code_ == 0));
        VIXL_ASSERT((type_ != kNoRegister) || (size_ == 0));

        return type_ == kNoRegister;
    }

    bool Aliases(const CPURegister& other) const {
        VIXL_ASSERT(IsValidOrNone() && other.IsValidOrNone());
        return (code_ == other.code_) && (type_ == other.type_);
    }

    bool Is(const CPURegister& other) const {
        VIXL_ASSERT(IsValidOrNone() && other.IsValidOrNone());
        return Aliases(other) && (size_ == other.size_);
    }

    inline bool IsZero() const {
        VIXL_ASSERT(IsValid());
        return IsRegister() && (code_ == kZeroRegCode);
    }

    inline bool IsSP() const {
        VIXL_ASSERT(IsValid());
        return IsRegister() && (code_ == kSPRegInternalCode);
    }

    inline bool IsRegister() const {
        return type_ == kARMRegister;
    }

    inline bool IsARMFPRegister() const {
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


class ARMRegister : public CPURegister
{
  public:
    explicit ARMRegister() : CPURegister() {}

    inline explicit ARMRegister(const CPURegister& other)
      : CPURegister(other.code(), other.size(), other.type())
    {
      VIXL_ASSERT(IsValidRegister());
    }

    MOZ_CONSTEXPR ARMRegister(unsigned code, unsigned size)
      : CPURegister(code, size, kARMRegister)
    { }

    MOZ_CONSTEXPR ARMRegister(Register r, unsigned size)
      : CPURegister(r.code(), size, kARMRegister)
    { }

    bool IsValid() const {
        VIXL_ASSERT(IsRegister() || IsNone());
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
      VIXL_ASSERT(IsValidARMFPRegister());
    }
    MOZ_CONSTEXPR inline ARMFPRegister(FloatRegister r, unsigned size)
        : CPURegister(r.code_, size, kARMFPRegister)
    { }
    MOZ_CONSTEXPR inline ARMFPRegister(unsigned code, unsigned size)
      : CPURegister(code, size, kARMFPRegister)
    { }

    bool IsValid() const {
        VIXL_ASSERT(IsARMFPRegister() || IsNone());
        return IsValidARMFPRegister();
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
const ARMRegister ip0 = x16;
const ARMRegister ip1 = x17;
const ARMRegister lr = x30;
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
        VIXL_ASSERT(AreSameSizeAndType(reg1, reg2, reg3, reg4));
        VIXL_ASSERT(IsValid());
    }

    inline CPURegList(CPURegister::RegisterType type, unsigned size, RegList list)
        : list_(list), size_(size), type_(type)
    {
        VIXL_ASSERT(IsValid());
    }

    inline CPURegList(CPURegister::RegisterType type, unsigned size,
                      unsigned first_reg, unsigned last_reg)
      : size_(size), type_(type)
    {
        VIXL_ASSERT(((type == CPURegister::kARMRegister) &&
                     (last_reg < kNumberOfRegisters)) ||
                    ((type == CPURegister::kARMFPRegister) &&
                     (last_reg < kNumberOfFloatRegisters)));
        VIXL_ASSERT(last_reg >= first_reg);
        list_ = (UINT64_C(1) << (last_reg + 1)) - 1;
        list_ &= ~((UINT64_C(1) << first_reg) - 1);
        VIXL_ASSERT(IsValid());
    }

    inline CPURegister::RegisterType type() const {
        VIXL_ASSERT(IsValid());
        return type_;
    }

    // Combine another CPURegList into this one. ARMRegisters that already exist in
    // this list are left unchanged. The type and size of the registers in the
    // 'other' list must match those in this list.
    void Combine(const CPURegList& other) {
        VIXL_ASSERT(IsValid());
        VIXL_ASSERT(other.type() == type_);
        VIXL_ASSERT(other.RegisterSizeInBits() == size_);
        list_ |= other.list();
    }

    // Remove every register in the other CPURegList from this one. ARMRegisters that
    // do not exist in this list are ignored. The type and size of the registers
    // in the 'other' list must match those in this list.
    void Remove(const CPURegList& other) {
        VIXL_ASSERT(IsValid());
        VIXL_ASSERT(other.type() == type_);
        VIXL_ASSERT(other.RegisterSizeInBits() == size_);
        list_ &= ~other.list();
    }

    // Variants of Combine and Remove which take a single register.
    inline void Combine(const CPURegister& other) {
        VIXL_ASSERT(other.type() == type_);
        VIXL_ASSERT(other.size() == size_);
        Combine(other.code());
    }

    inline void Remove(const CPURegister& other) {
        VIXL_ASSERT(other.type() == type_);
        VIXL_ASSERT(other.size() == size_);
        Remove(other.code());
    }

    // Variants of Combine and Remove which take a single register by its code;
    // the type and size of the register is inferred from this list.
    inline void Combine(int code) {
        VIXL_ASSERT(IsValid());
        VIXL_ASSERT(CPURegister(code, size_, type_).IsValid());
        list_ |= (UINT64_C(1) << code);
    }

    inline void Remove(int code) {
        VIXL_ASSERT(IsValid());
        VIXL_ASSERT(CPURegister(code, size_, type_).IsValid());
        list_ &= ~(UINT64_C(1) << code);
    }

    inline RegList list() const {
        VIXL_ASSERT(IsValid());
        return list_;
    }

    inline void set_list(RegList new_list) {
        VIXL_ASSERT(IsValid());
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
        VIXL_ASSERT(IsValid());
        return list_ == 0;
    }

    inline bool IncludesAliasOf(const CPURegister& other) const {
        VIXL_ASSERT(IsValid());
        return (type_ == other.type()) && ((other.Bit() & list_) != 0);
    }

    inline bool IncludesAliasOf(int code) const {
        VIXL_ASSERT(IsValid());
        return ((code & list_) != 0);
    }

    inline int Count() const {
        VIXL_ASSERT(IsValid());
        return CountSetBits(list_, kRegListSizeInBits);
    }

    inline unsigned RegisterSizeInBits() const {
        VIXL_ASSERT(IsValid());
        return size_;
    }

    inline unsigned RegisterSizeInBytes() const {
        int size_in_bits = RegisterSizeInBits();
        VIXL_ASSERT((size_in_bits % 8) == 0);
        return size_in_bits / 8;
    }

    inline unsigned TotalSizeInBytes() const {
        VIXL_ASSERT(IsValid());
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
        VIXL_ASSERT(IsImmediate());
        return immediate_;
    }

    ARMRegister reg() const {
        VIXL_ASSERT(IsShiftedRegister() || IsExtendedRegister());
        return reg_;
    }

    Shift shift() const {
        VIXL_ASSERT(IsShiftedRegister());
        return shift_;
    }

    Extend extend() const {
        VIXL_ASSERT(IsExtendedRegister());
        return extend_;
    }

    unsigned shift_amount() const {
        VIXL_ASSERT(IsShiftedRegister() || IsExtendedRegister());
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
      VIXL_ASSERT(!IsLinked() || IsBound());
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

// Assembler.
class AssemblerVIXL : public AssemblerShared
{
  public:
    AssemblerVIXL()
      : armbuffer_(4, 4, 0, &pools_[0], 8), // FIXME: What on earth is this
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
        // VIXL_ASSERT(finalized_ || (pc_ == buffer_));
    }

    // System functions.

    // Helper function for use with the ARMBuffer.
    // We need to wait until an AutoIonContextAlloc is created by the
    // IonMacroAssembler before allocating any space.
    void initWithAllocator() {
        armbuffer_.initWithAllocator();

        // Set up the backwards double region.
        new (&pools_[2]) Pool(1024, 8, 4, 8, 8, armbuffer_.LifoAlloc_, true);
        // Set up the backwards 32-bit region.
        new (&pools_[3]) Pool(4096, 4, 4, 8, 4, armbuffer_.LifoAlloc_, true);

        // Set up the forwards double region.
        new (&pools_[0]) Pool(1024, 8, 4, 8, 8, armbuffer_.LifoAlloc_, false, false, &pools_[2]);
        // Set up the forwards 32-bit region.
        new (&pools_[1]) Pool(4096, 4, 4, 8, 4, armbuffer_.LifoAlloc_, false, true, &pools_[3]);

        for (int i = 0; i < 4; i++) {
            if (pools_[i].poolData == nullptr)
                armbuffer_.fail_oom();
        }
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

    void trace(JSTracer *trc) {
        JS_ASSERT(0 && "Assembler::trace()");
    }

    // Label.
    // Bind a label to the current PC.
    void bind(Label* label);

    int UpdateAndGetByteOffsetTo(Label* label);
    inline int UpdateAndGetInstructionOffsetTo(Label* label) {
        //VIXL_ASSERT(Label::kEndOfChain == 0);
        return UpdateAndGetByteOffsetTo(label) >> kInstructionSizeLog2;
    }

    // Condition codes.
    enum Condition {
        Equal               =  0, eq =  0,
        Zero                =  0,
        NotEqual            =  1, ne =  1,
        NonZero             =  1,
        AboveOrEqual        =  2, hs =  2,
        Below               =  3, lo =  3,
        Signed              =  4, mi =  4,
        NotSigned           =  5, pl =  5,
        Overflow            =  6, vs =  6,
        NoOverflow          =  7, vc =  7, // AArch64-specific.
        Above               =  8, hi =  8,
        BelowOrEqual        =  9, ls =  9,
        GreaterThanOrEqual  = 10, ge = 10,
        LessThan            = 11, lt = 11,
        GreaterThan         = 12, gt = 12,
        LessThanOrEqual     = 13, le = 13,
        Always              = 14, al = 14,
        Never               = 15, nv = 15  // Behaves as always/al.
    };

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
        VIXL_ASSERT((cond != al) && (cond != nv));
        return static_cast<Condition>(cond ^ 1);
    }

    static inline Condition ConditionFromDoubleCondition(DoubleCondition cond) {
        JS_ASSERT(!(cond & DoubleConditionBitSpecial));
        return static_cast<Condition>(cond);
    }

    // Instruction set functions.

    // Branch / Jump instructions.
    // Branch to register.

    void br(const ARMRegister& xn);

    // Branch with link to register.
    void blr(const ARMRegister& xn);

    // Branch to register with return hint.
    void ret(const ARMRegister& xn = lr);

    // Unconditional branch to label.
    void b(Label* label);

    // Conditional branch to label.
    void b(Label* label, Condition cond);

    // Unconditional branch to PC offset.
    void b(int imm26);
    static void b(Instruction *at, int imm26);

    // Conditional branch to PC offset.
    void b(int imm19, Condition cond);
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

    // Compare and branch to label if not zero.
    void cbnz(const ARMRegister& rt, Label* label);

    // Compare and branch to PC offset if not zero.
    void cbnz(const ARMRegister& rt, int imm19);

    // Test bit and branch to label if zero.
    void tbz(const ARMRegister& rt, unsigned bit_pos, Label* label);

    // Test bit and branch to PC offset if zero.
    void tbz(const ARMRegister& rt, unsigned bit_pos, int imm14);

    // Test bit and branch to label if not zero.
    void tbnz(const ARMRegister& rt, unsigned bit_pos, Label* label);

    // Test bit and branch to PC offset if not zero.
    void tbnz(const ARMRegister& rt, unsigned bit_pos, int imm14);

    // Address calculation instructions.
    // Calculate a PC-relative address. Unlike for branches the offset in adr is
    // unscaled (i.e. the result can be unaligned).

    // Calculate the address of a label.

    void adr(const ARMRegister& rd, Label* label);

    // Calculate the address of a PC offset.
    void adr(const ARMRegister& rd, int imm21);

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
    void ands(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand);

    // Bit test and set flags.
    void tst(const ARMRegister& rn, const Operand& operand);

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
        VIXL_ASSERT(width >= 1);
        VIXL_ASSERT(lsb + width <= rn.size());
        bfm(rd, rn, (rd.size() - lsb) & (rd.size() - 1), width - 1);
    }

    // Bitfield extract and insert low.
    inline void bfxil(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        VIXL_ASSERT(width >= 1);
        VIXL_ASSERT(lsb + width <= rn.size());
        bfm(rd, rn, lsb, lsb + width - 1);
    }

    // Sbfm aliases.
    // Arithmetic shift right.
    inline void asr(const ARMRegister& rd, const ARMRegister& rn, unsigned shift) {
        VIXL_ASSERT(shift < rd.size());
        sbfm(rd, rn, shift, rd.size() - 1);
    }

    // Signed bitfield insert with zero at right.
    inline void sbfiz(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        VIXL_ASSERT(width >= 1);
        VIXL_ASSERT(lsb + width <= rn.size());
        sbfm(rd, rn, (rd.size() - lsb) & (rd.size() - 1), width - 1);
    }

    // Signed bitfield extract.
    inline void sbfx(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        VIXL_ASSERT(width >= 1);
        VIXL_ASSERT(lsb + width <= rn.size());
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
        VIXL_ASSERT(shift < reg_size);
        ubfm(rd, rn, (reg_size - shift) % reg_size, reg_size - shift - 1);
    }

    // Logical shift right.
    inline void lsr(const ARMRegister& rd, const ARMRegister& rn, unsigned shift) {
        VIXL_ASSERT(shift < rd.size());
        ubfm(rd, rn, shift, rd.size() - 1);
    }

    // Unsigned bitfield insert with zero at right.
    inline void ubfiz(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        VIXL_ASSERT(width >= 1);
        VIXL_ASSERT(lsb + width <= rn.size());
        ubfm(rd, rn, (rd.size() - lsb) & (rd.size() - 1), width - 1);
    }

    // Unsigned bitfield extract.
    inline void ubfx(const ARMRegister& rd, const ARMRegister& rn, unsigned lsb, unsigned width) {
        VIXL_ASSERT(width >= 1);
        VIXL_ASSERT(lsb + width <= rn.size());
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
    void ldr(const CPURegister& rt, const MemOperand& src);

    // Store integer or FP register.
    void str(const CPURegister& rt, const MemOperand& dst);

    // Load word with sign extension.
    void ldrsw(const ARMRegister& rt, const MemOperand& src);

    // Load byte.
    void ldrb(const ARMRegister& rt, const MemOperand& src);

    // Store byte.
    void strb(const ARMRegister& rt, const MemOperand& dst);

    // Load byte with sign extension.
    void ldrsb(const ARMRegister& rt, const MemOperand& src);

    // Load half-word.
    void ldrh(const ARMRegister& rt, const MemOperand& src);

    // Store half-word.
    void strh(const ARMRegister& rt, const MemOperand& dst);

    // Load half-word with sign extension.
    void ldrsh(const ARMRegister& rt, const MemOperand& src);

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

    // Load literal to register.
    void ldr(const ARMRegister& rt, uint64_t imm);

    // Load double precision floating point literal to FP register.
    void ldr(const ARMFPRegister& ft, double imm);

    // Load single precision floating point literal to FP register.
    void ldr(const ARMFPRegister& ft, float imm);

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
        VIXL_ASSERT(string != NULL);

        size_t len = strlen(string) + 1;
        EmitData(string, len);

        // Pad with NULL characters until pc_ is aligned.
        const char pad[] = {'\0', '\0', '\0', '\0'};
        VIXL_STATIC_ASSERT(sizeof(pad) == kInstructionSize);
        Instruction* next_pc = AlignUp(pc_, kInstructionSize);
        EmitData(&pad, next_pc - pc_);
    }

    // Code generation helpers.

    // ARMRegister encoding.
    static Instr Rd(CPURegister rd) {
        VIXL_ASSERT(rd.code() != kSPRegInternalCode);
        return rd.code() << Rd_offset;
    }

    static Instr Rn(CPURegister rn) {
        VIXL_ASSERT(rn.code() != kSPRegInternalCode);
        return rn.code() << Rn_offset;
    }

    static Instr Rm(CPURegister rm) {
        VIXL_ASSERT(rm.code() != kSPRegInternalCode);
        return rm.code() << Rm_offset;
    }

    static Instr Ra(CPURegister ra) {
        VIXL_ASSERT(ra.code() != kSPRegInternalCode);
        return ra.code() << Ra_offset;
    }

    static Instr Rt(CPURegister rt) {
        VIXL_ASSERT(rt.code() != kSPRegInternalCode);
        return rt.code() << Rt_offset;
    }

    static Instr Rt2(CPURegister rt2) {
        VIXL_ASSERT(rt2.code() != kSPRegInternalCode);
        return rt2.code() << Rt2_offset;
    }

    // These encoding functions allow the stack pointer to be encoded, and
    // disallow the zero register.
    static Instr RdSP(ARMRegister rd) {
        VIXL_ASSERT(!rd.IsZero());
        return (rd.code() & kRegCodeMask) << Rd_offset;
    }

    static Instr RnSP(ARMRegister rn) {
        VIXL_ASSERT(!rn.IsZero());
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
        VIXL_ASSERT(is_int21(imm21));
        Instr imm = static_cast<Instr>(truncate_to_int21(imm21));
        Instr immhi = (imm >> ImmPCRelLo_width) << ImmPCRelHi_offset;
        Instr immlo = imm << ImmPCRelLo_offset;
        return (immhi & ImmPCRelHi_mask) | (immlo & ImmPCRelLo_mask);
    }

    // Branch encoding.
    static Instr ImmUncondBranch(int imm26) {
        VIXL_ASSERT(is_int26(imm26));
        return truncate_to_int26(imm26) << ImmUncondBranch_offset;
    }

    static Instr ImmCondBranch(int imm19) {
        VIXL_ASSERT(is_int19(imm19));
        return truncate_to_int19(imm19) << ImmCondBranch_offset;
    }

    static Instr ImmCmpBranch(int imm19) {
        VIXL_ASSERT(is_int19(imm19));
        return truncate_to_int19(imm19) << ImmCmpBranch_offset;
    }

    static Instr ImmTestBranch(int imm14) {
        VIXL_ASSERT(is_int14(imm14));
        return truncate_to_int14(imm14) << ImmTestBranch_offset;
    }

    static Instr ImmTestBranchBit(unsigned bit_pos) {
        VIXL_ASSERT(is_uint6(bit_pos));
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
        VIXL_ASSERT(IsImmAddSub(imm));
        if (is_uint12(imm)) // No shift required.
            return imm << ImmAddSub_offset;
        return ((imm >> 12) << ImmAddSub_offset) | (1 << ShiftAddSub_offset);
    }

    static inline Instr ImmS(unsigned imms, unsigned reg_size) {
        VIXL_ASSERT(((reg_size == kXRegSize) && is_uint6(imms)) ||
                    ((reg_size == kWRegSize) && is_uint5(imms)));
        USEARG(reg_size);
        return imms << ImmS_offset;
    }

    static inline Instr ImmR(unsigned immr, unsigned reg_size) {
        VIXL_ASSERT(((reg_size == kXRegSize) && is_uint6(immr)) ||
                    ((reg_size == kWRegSize) && is_uint5(immr)));
        USEARG(reg_size);
        VIXL_ASSERT(is_uint6(immr));
        return immr << ImmR_offset;
    }

    static inline Instr ImmSetBits(unsigned imms, unsigned reg_size) {
        VIXL_ASSERT((reg_size == kWRegSize) || (reg_size == kXRegSize));
        VIXL_ASSERT(is_uint6(imms));
        VIXL_ASSERT((reg_size == kXRegSize) || is_uint6(imms + 3));
        USEARG(reg_size);
        return imms << ImmSetBits_offset;
    }

    static inline Instr ImmRotate(unsigned immr, unsigned reg_size) {
        VIXL_ASSERT((reg_size == kWRegSize) || (reg_size == kXRegSize));
        VIXL_ASSERT(((reg_size == kXRegSize) && is_uint6(immr)) ||
                    ((reg_size == kWRegSize) && is_uint5(immr)));
        USEARG(reg_size);
        return immr << ImmRotate_offset;
    }

    static inline Instr ImmLLiteral(int imm19) {
        VIXL_ASSERT(is_int19(imm19));
        return truncate_to_int19(imm19) << ImmLLiteral_offset;
    }

    static inline Instr BitN(unsigned bitn, unsigned reg_size) {
        VIXL_ASSERT((reg_size == kWRegSize) || (reg_size == kXRegSize));
        VIXL_ASSERT((reg_size == kXRegSize) || (bitn == 0));
        USEARG(reg_size);
        return bitn << BitN_offset;
    }

    static Instr ShiftDP(Shift shift) {
        VIXL_ASSERT(shift == LSL || shift == LSR || shift == ASR || shift == ROR);
        return shift << ShiftDP_offset;
    }

    static Instr ImmDPShift(unsigned amount) {
        VIXL_ASSERT(is_uint6(amount));
        return amount << ImmDPShift_offset;
    }

    static Instr ExtendMode(Extend extend) {
        return extend << ExtendMode_offset;
    }

    static Instr ImmExtendShift(unsigned left_shift) {
        VIXL_ASSERT(left_shift <= 4);
        return left_shift << ImmExtendShift_offset;
    }

    static Instr ImmCondCmp(unsigned imm) {
        VIXL_ASSERT(is_uint5(imm));
        return imm << ImmCondCmp_offset;
    }

    static Instr Nzcv(StatusFlags nzcv) {
        return ((nzcv >> Flags_offset) & 0xf) << Nzcv_offset;
    }

    // MemOperand offset encoding.
    static Instr ImmLSUnsigned(int imm12) {
        VIXL_ASSERT(is_uint12(imm12));
        return imm12 << ImmLSUnsigned_offset;
    }

    static Instr ImmLS(int imm9) {
        VIXL_ASSERT(is_int9(imm9));
        return truncate_to_int9(imm9) << ImmLS_offset;
    }

    static Instr ImmLSPair(int imm7, LSDataSize size) {
        VIXL_ASSERT(((imm7 >> size) << size) == imm7);
        int scaled_imm7 = imm7 >> size;
        VIXL_ASSERT(is_int7(scaled_imm7));
        return truncate_to_int7(scaled_imm7) << ImmLSPair_offset;
    }

    static Instr ImmShiftLS(unsigned shift_amount) {
        VIXL_ASSERT(is_uint1(shift_amount));
        return shift_amount << ImmShiftLS_offset;
    }

    static Instr ImmException(int imm16) {
        VIXL_ASSERT(is_uint16(imm16));
        return imm16 << ImmException_offset;
    }

    static Instr ImmSystemRegister(int imm15) {
        VIXL_ASSERT(is_uint15(imm15));
        return imm15 << ImmSystemRegister_offset;
    }

    static Instr ImmHint(int imm7) {
        VIXL_ASSERT(is_uint7(imm7));
        return imm7 << ImmHint_offset;
    }

    static Instr ImmBarrierDomain(int imm2) {
        VIXL_ASSERT(is_uint2(imm2));
        return imm2 << ImmBarrierDomain_offset;
    }

    static Instr ImmBarrierType(int imm2) {
        VIXL_ASSERT(is_uint2(imm2));
        return imm2 << ImmBarrierType_offset;
    }

    static LSDataSize CalcLSDataSize(LoadStoreOp op) {
        VIXL_ASSERT((SizeLS_offset + SizeLS_width) == (kInstructionSize * 8));
        return static_cast<LSDataSize>(op >> SizeLS_offset);
    }

    // Move immediates encoding.
    static Instr ImmMoveWide(uint64_t imm) {
        VIXL_ASSERT(is_uint16(imm));
        return imm << ImmMoveWide_offset;
    }

    static Instr ShiftMoveWide(int64_t shift) {
        VIXL_ASSERT(is_uint2(shift));
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
        VIXL_ASSERT(is_uint6(scale));
        return scale << FPScale_offset;
    }

    uint64_t size() const {
        return SizeOfCodeGenerated();
    }

    // Size of the code generated in bytes, including pools.
    uint64_t SizeOfCodeGenerated() const {
        return armbuffer_.size();
    }

  protected:
    inline const ARMRegister& AppropriateZeroRegFor(const CPURegister& reg) const {
        return reg.Is64Bits() ? xzr : wzr;
    }

    void LoadStore(const CPURegister& rt, const MemOperand& addr, LoadStoreOp op);

    static bool IsImmLSUnscaled(ptrdiff_t offset);
    static bool IsImmLSScaled(ptrdiff_t offset, LSDataSize size);

    void Logical(const ARMRegister& rd, const ARMRegister& rn,
                 const Operand& operand, LogicalOp op);
    void LogicalImmediate(const ARMRegister& rd, const ARMRegister& rn, unsigned n,
                          unsigned imm_s, unsigned imm_r, LogicalOp op);
    static bool IsImmLogical(uint64_t value, unsigned width, unsigned* n,
                             unsigned* imm_s, unsigned* imm_r);

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


  private:
    // Instruction helpers.
    void MoveWide(const ARMRegister& rd, uint64_t imm, int shift, MoveWideImmediateOp mov_op);
    void DataProcShiftedRegister(const ARMRegister& rd, const ARMRegister& rn,
                                 const Operand& operand, FlagsUpdate S, Instr op);
    void DataProcExtendedRegister(const ARMRegister& rd, const ARMRegister& rn,
                                  const Operand& operand, FlagsUpdate S, Instr op);
    void LoadStorePair(const CPURegister& rt, const CPURegister& rt2,
                       const MemOperand& addr, LoadStorePairOp op);
    void LoadStorePairNonTemporal(const CPURegister& rt, const CPURegister& rt2,
                                  const MemOperand& addr, LoadStorePairNonTemporalOp op);
    void LoadLiteral(const CPURegister& rt, uint64_t imm, LoadLiteralOp op);
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

    // Emit the instruction at pc_.
    void Emit(Instr instruction) {
        VIXL_STATIC_ASSERT(sizeof(*pc_) == 1);
        VIXL_STATIC_ASSERT(sizeof(instruction) == kInstructionSize);
        // TODO: VIXL_ASSERT((pc_ + sizeof(instruction)) <= (buffer_ + buffer_size_));

#ifdef DEBUG
        finalized_ = false;
#endif

        armbuffer_.putInt(*(uint32_t*)(&instruction));
        pc_ += sizeof(instruction);
        CheckBufferSpace();
    }

  // FIXME: This interface should not be public.
  public:
    // Emit the instruction at |at|.
    static void Emit(Instruction *at, Instr instruction) {
        VIXL_STATIC_ASSERT(sizeof(instruction) == kInstructionSize);
        uint32_t *addr = (uint32_t *)at;
        *addr = *(uint32_t *)(&instruction);
    }

  private:
    // Emit data inline in the instruction stream.
    void EmitData(void const * data, unsigned size) {
        JS_ASSERT(0 && "EmitData()");
#if 0
        VIXL_STATIC_ASSERT(sizeof(*pc_) == 1);
        // TODO: VIXL_ASSERT((pc_ + size) <= (buffer_ + buffer_size_));

#ifdef DEBUG
        finalized_ = false;
#endif


        // TODO: Record this 'instruction' as data, so that it can be disassembled
        // correctly.
        memcpy(pc_, data, size);
        pc_ += size;
        CheckBufferSpace();
#endif
    }

    inline void CheckBufferSpace() {
        VIXL_ASSERT(!armbuffer_.oom());
        // TODO: VIXL_ASSERT(pc_ < (buffer_ + buffer_size_));
        // FIXME: Integration with constant pool?
    }

  public:
    // Interface used by IonAssemblerBufferWithConstantPools.
    static void InsertTokenIntoTag(uint32_t instSize, uint8_t *load_, int32_t token) {
        MOZ_ASSUME_UNREACHABLE("InsertTokenIntoTag");
    }
    static bool PatchConstantPoolLoad(void *loadAddr, void *constPoolAddr) {
        MOZ_ASSUME_UNREACHABLE("PatchConstantPoolLoad");
    }
    static uint32_t PlaceConstantPoolBarrier(int offset) {
        MOZ_ASSUME_UNREACHABLE("PlaceConstantPoolBarrier");
    }
    static void WritePoolGuard(BufferOffset branch, Instruction *inst, BufferOffset dest) {
        MOZ_ASSUME_UNREACHABLE("WritePoolGuard");
    }
    static void WritePoolHeader(uint8_t *start, Pool *p, bool isNatural);
    static void WritePoolFooter(uint8_t *start, Pool *p, bool isNatural);

    // Static interface used by IonAssemblerBufferWithConstantPools.
    static ptrdiff_t GetBranchOffset(const Instruction *i);
    static void RetargetNearBranch(Instruction *i, int offset, Condition cond, bool final = true);
    static void RetargetNearBranch(Instruction *i, int offset, bool final = true);
    static void RetargetFarBranch(Instruction *i, uint8_t **slot, uint8_t *dest, Condition cond);

  protected:
    // The buffer into which code and relocation info are generated.
    ARMBuffer armbuffer_;

    CompactBufferWriter jumpRelocations_;
    CompactBufferWriter dataRelocations_;
    CompactBufferWriter relocations_;
    CompactBufferWriter preBarriers_;

    // Literal pools.
    mozilla::Array<Pool, 4> pools_;

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
