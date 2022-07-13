# ifndef CPPAD_CORE_AD_HPP
# define CPPAD_CORE_AD_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// simple AD operations that must be defined for AD as well as base class
# include <cppad/core/ordered.hpp>
# include <cppad/core/identical.hpp>

// define the template classes that are used by the AD template class
# include <cppad/local/op_code_dyn.hpp>
# include <cppad/local/op_code_var.hpp>
# include <cppad/core/ad_type.hpp>
# include <cppad/local/record/recorder.hpp>
# include <cppad/local/play/player.hpp>
# include <cppad/local/ad_tape.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

// tape_manage_enum
typedef enum {
    new_tape_manage,
    delete_tape_manage
}
tape_manage_enum;

template <class Base>
class AD {
private :
    // -----------------------------------------------------------------------
    // Base type value for this object
    Base value_;
    //
    // tape for this object
    tape_id_t tape_id_;
    //
    // tape address for this object
    // (when tape_id is current tape for AD<Base>)
    addr_t taddr_;
    //
    // sub-type for this object
    // (when tape_id is current tape for AD<Base>)
    ad_type_enum ad_type_;
    // -----------------------------------------------------------------------

    // enable use of AD<Base> in parallel mode
    template <class Type>
    friend void parallel_ad(void);

    // template friend functions where template parameter is not bound
    template <class ADVector>
    friend void Independent(
        ADVector&  x              ,
        size_t     abort_op_index ,
        bool       record_compare ,
        ADVector&  dynamic
    );

    // one argument functions
    friend bool Constant  <Base> (const AD<Base>    &u);
    friend bool Constant  <Base> (const VecAD<Base> &u);
    //
    friend bool Dynamic   <Base> (const AD<Base>    &u);
    friend bool Dynamic   <Base> (const VecAD<Base> &u);
    //
    friend bool Parameter <Base> (const AD<Base>    &u);
    friend bool Parameter <Base> (const VecAD<Base> &u);
    //
    friend bool Variable  <Base> (const AD<Base>    &u);
    friend bool Variable  <Base> (const VecAD<Base> &u);
    //
    friend int  Integer   <Base> (const AD<Base>    &u);
    friend AD   Var2Par   <Base> (const AD<Base>    &u);
    //
    friend unsigned short hash_code <Base> (const AD<Base> &u);
    //
    // power function
    friend AD pow <Base>
        (const AD<Base> &x, const AD<Base> &y);

    // azmul function
    friend AD azmul <Base>
        (const AD<Base> &x, const AD<Base> &y);

    // order determining functions, see ordered.hpp
    friend bool GreaterThanZero   <Base> (const AD<Base> &x);
    friend bool GreaterThanOrZero <Base> (const AD<Base> &x);
    friend bool LessThanZero      <Base> (const AD<Base> &x);
    friend bool LessThanOrZero    <Base> (const AD<Base> &x);
    friend bool abs_geq           <Base>
        (const AD<Base>& x, const AD<Base>& y);

    // The identical property functions, see identical.hpp
    friend bool IdenticalCon      <Base> (const AD<Base> &x);
    friend bool IdenticalZero     <Base> (const AD<Base> &x);
    friend bool IdenticalOne      <Base> (const AD<Base> &x);
    friend bool IdenticalEqualCon <Base>
        (const AD<Base> &x, const AD<Base> &y);

    // EqualOpSeq function
    friend bool EqualOpSeq <Base>
        (const AD<Base> &u, const AD<Base> &v);

    // NearEqual function
    friend bool NearEqual <Base> (
    const AD<Base> &x, const AD<Base> &y, const Base &r, const Base &a);

    friend bool NearEqual <Base> (
    const Base &x, const AD<Base> &y, const Base &r, const Base &a);

    friend bool NearEqual <Base> (
    const AD<Base> &x, const Base &y, const Base &r, const Base &a);

    // CondExp function
    friend AD<Base> CondExpOp  <Base> (
        enum CompareOp  cop       ,
        const AD<Base> &left      ,
        const AD<Base> &right     ,
        const AD<Base> &trueCase  ,
        const AD<Base> &falseCase
    );

    // classes
    friend class local::ADTape<Base>;
    friend class local::recorder<Base>;
    friend class ADFun<Base>;
    friend class atomic_base<Base>;
    friend class atomic_three<Base>;
    friend class discrete<Base>;
    friend class VecAD<Base>;
    friend class VecAD_reference<Base>;

    // arithematic binary operators
    friend AD<Base> operator + <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend AD<Base> operator - <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend AD<Base> operator * <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend AD<Base> operator / <Base>
        (const AD<Base> &left, const AD<Base> &right);

    // comparison operators
    friend bool operator < <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend bool operator <= <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend bool operator > <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend bool operator >= <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend bool operator == <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend bool operator != <Base>
        (const AD<Base> &left, const AD<Base> &right);

    // input operator
    friend std::istream& operator >> <Base>
        (std::istream &is, AD<Base> &x);

    // output operations
    friend std::ostream& operator << <Base>
        (std::ostream &os, const AD<Base> &x);
    friend void PrintFor <Base> (
        const AD<Base>&    flag   ,
        const char*        before ,
        const AD<Base>&    var    ,
        const char*        after
    );
public:
    // type of value
    typedef Base value_type;

    // implicit default constructor
    AD(void);

    // destructor
    ~AD(void) { }

    // use default implicit copy constructor
    // AD(const AD &x);

# ifdef CPPAD_FOR_TMB
    // TMB would rather have implicit construction from double,
    // CppAD uses default constructor and assignment to double instead.
    AD(const double &d);
# else
    // implicit construction from base type
    AD(const Base &b);
# endif

    // implicit contructor from VecAD<Base>::reference
    AD(const VecAD_reference<Base> &x);

    // explicit construction from some other type (depricated)
    template <class T> explicit AD(const T &t);

    // conversion from AD to Base type
    friend Base Value <Base> (const AD<Base> &x);

    // use default assignment operator
    // AD& operator=(const AD &x);

    // assingment from base type
    AD& operator=(const Base &b);

    // assignment from VecAD<Base>::reference
    AD& operator=(const VecAD_reference<Base> &x);

    // assignment from some other type
    template <class T> AD& operator=(const T &right);

    // compound assignment operators
    AD& operator += (const AD &right);
    AD& operator -= (const AD &right);
    AD& operator *= (const AD &right);
    AD& operator /= (const AD &right);

    // unary operators
    AD operator +(void) const;
    AD operator -(void) const;

    // interface so these functions need not be friends
    AD abs_me(void) const;
    AD acos_me(void) const;
    AD asin_me(void) const;
    AD atan_me(void) const;
    AD cos_me(void) const;
    AD cosh_me(void) const;
    AD exp_me(void) const;
    AD fabs_me(void) const;
    AD log_me(void) const;
    AD sin_me(void) const;
    AD sign_me(void) const;
    AD sinh_me(void) const;
    AD sqrt_me(void) const;
    AD tan_me(void) const;
    AD tanh_me(void) const;
    AD asinh_me(void) const;
    AD acosh_me(void) const;
    AD atanh_me(void) const;
    AD erf_me(bool complemnet) const;
    AD expm1_me(void) const;
    AD log1p_me(void) const;

    // ----------------------------------------------------------
    // static public member functions

    // abort current AD<Base> recording
    static void        abort_recording(void);

    // set the maximum number of OpenMP threads (deprecated)
    static void        omp_max_thread(size_t number);

    // These functions declared public so can be accessed by user through
    // a macro interface and are not intended for direct use.
    // The macro interface is documented in bool_fun.hpp.
    // Developer documentation for these fucntions is in  bool_fun.hpp
    static bool UnaryBool(
        bool FunName(const Base &x),
        const AD<Base> &x
    );
    static bool BinaryBool(
        bool FunName(const Base &x, const Base &y),
        const AD<Base> &x , const AD<Base> &y
    );

private:
    // -----------------------------------------------------------------
    // Make this parameter a new variable
    void make_variable(tape_id_t id,  addr_t taddr)
    {   CPPAD_ASSERT_UNKNOWN( Parameter(*this) ); // currently a par
        CPPAD_ASSERT_UNKNOWN( taddr > 0 );        // sure valid taddr

        tape_id_ = id;
        taddr_   = taddr;
        ad_type_ = variable_enum;
    }
    // ---------------------------------------------------------------
    // tape linking functions
    //
    // not static
    local::ADTape<Base>* tape_this(void) const;
    //
    // static
    static tape_id_t*            tape_id_ptr(size_t thread);
    static local::ADTape<Base>** tape_handle(size_t thread);
    static local::ADTape<Base>*         tape_manage(tape_manage_enum job);
    static local::ADTape<Base>*  tape_ptr(void);
    static local::ADTape<Base>*  tape_ptr(tape_id_t tape_id);
};
// ---------------------------------------------------------------------------

} // END_CPPAD_NAMESPACE

// tape linking private functions
# include <cppad/core/tape_link.hpp>

// operations that expect the AD template class to be defined


# endif
