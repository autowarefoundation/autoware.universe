# ifndef CPPAD_EXAMPLE_GENERAL_BASE_ALLOC_HPP
# define CPPAD_EXAMPLE_GENERAL_BASE_ALLOC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin base_alloc.hpp$$
$spell
    isnan
    azmul
    expm1
    atanh
    acosh
    asinh
    Rel
    Lt Le Eq Ge Gt
    Cond
    enum
    geq
    cos
    sqrt
    cppad.hpp
    alloc
    op
    const
    ptrdbl
    bool
    CppAD
    sizeof
    inline
    atan
    ostream
    namespace
    exp
    tanh
    acos
    asin
    std
    fabs
    erf
    erfc
    endif
$$
$section Example AD<Base> Where Base Constructor Allocates Memory$$

$head Purpose$$
Demonstrate use of $codei%AD<%Base%>%$$
where memory is allocated for each element of the type $icode Base$$.
In addition, this is a complete example where all the
$cref/required Base/base_require/$$ type
operations are defined (as apposed to other examples where
some of the operations for the Base type are already defined).

$head Include File$$
This file uses some of the definitions in $cref base_require$$
and $cref thread_alloc$$.
$srccode%cpp% */
# include <cppad/base_require.hpp>
# include <cppad/utility/thread_alloc.hpp>
/* %$$

$head Compound Assignment Macro$$
This macro is used for the
$code base_alloc$$ compound assignment operators; to be specific,
used with $icode op $$ equal to
$code +=$$,
$code -=$$,
$code *=$$,
$code /=$$.
$srccode%cpp% */
# define BASE_ALLOC_ASSIGN_OPERATOR(op) \
    void operator op (const base_alloc& x) \
    {   *ptrdbl_ op *x.ptrdbl_; }
/* %$$

$head Binary Operator Macro$$
This macro is used for the
$code base_alloc$$ binary operators (as member functions); to be specific,
used with $icode op $$ equal to
$code +$$,
$code -$$,
$code *$$,
$code /$$.
$srccode%cpp% */
# define BASE_ALLOC_BINARY_OPERATOR(op) const \
    base_alloc operator op (const base_alloc& x) const \
    {   base_alloc result; \
        double   dbl = *ptrdbl_; \
        double x_dbl = *x.ptrdbl_; \
        *result.ptrdbl_ = dbl op x_dbl; \
        return result; \
    }
/* %$$

$head Boolean Operator Macro$$
This macro can be used for the
$code base_alloc$$ binary operators that have a
$code bool$$ result; to be specific,
used with $icode op $$ equal to
$code ==$$,
$code !=$$,
$code <$$,
$code <=$$,
$code >=$$, and
$code >$$,
$srccode%cpp% */
# define BASE_ALLOC_BOOL_OPERATOR(op) const \
    bool operator op (const base_alloc& x) const \
    {   double   dbl = *ptrdbl_; \
        double x_dbl = *x.ptrdbl_; \
        return dbl op x_dbl; \
    }
/* %$$

$head Class Definition$$
The following example class
defines the necessary $cref base_member$$ functions.
It is made more complicated by storing a pointer to a $code double$$
instead of the $code double$$ value itself.
$srccode%cpp% */

class base_alloc {
public:
    double* ptrdbl_;

    base_alloc(void)
    {   size_t cap;
        void* v  = CppAD::thread_alloc::get_memory(sizeof(double), cap);
        ptrdbl_  = static_cast<double*>(v);
    }
    base_alloc(double dbl)
    {   size_t cap;
        void *v  = CppAD::thread_alloc::get_memory(sizeof(double), cap);
        ptrdbl_  = static_cast<double*>(v);
        *ptrdbl_ = dbl;
    }
    base_alloc(const base_alloc& x)
    {   size_t cap;
        void *v  = CppAD::thread_alloc::get_memory(sizeof(double), cap);
        ptrdbl_  = static_cast<double*>(v);
        *ptrdbl_ = *x.ptrdbl_;
    }
    ~base_alloc(void)
    {   void* v  = static_cast<void*>(ptrdbl_);
        CppAD::thread_alloc::return_memory(v);
    }
    base_alloc operator-(void) const
    {   base_alloc result;
        *result.ptrdbl_ = - *ptrdbl_;
        return result;
    }
    base_alloc operator+(void) const
    {   return *this; }
    void operator=(const base_alloc& x)
    {   *ptrdbl_ = *x.ptrdbl_; }
    BASE_ALLOC_ASSIGN_OPERATOR(+=)
    BASE_ALLOC_ASSIGN_OPERATOR(-=)
    BASE_ALLOC_ASSIGN_OPERATOR(*=)
    BASE_ALLOC_ASSIGN_OPERATOR(/=)
    BASE_ALLOC_BINARY_OPERATOR(+)
    BASE_ALLOC_BINARY_OPERATOR(-)
    BASE_ALLOC_BINARY_OPERATOR(*)
    BASE_ALLOC_BINARY_OPERATOR(/)
    BASE_ALLOC_BOOL_OPERATOR(==)
    BASE_ALLOC_BOOL_OPERATOR(!=)
    // The <= operator is not necessary for the base type requirements
    // (needed so we can use NearEqual with base_alloc arguments).
    BASE_ALLOC_BOOL_OPERATOR(<=)
};
/* %$$

$head CondExpOp$$
The type $code base_alloc$$ does not use $cref CondExp$$ operations.
Hence its $code CondExpOp$$ function is defined by
$srccode%cpp% */
namespace CppAD {
    inline base_alloc CondExpOp(
        enum CompareOp     cop          ,
        const base_alloc&       left         ,
        const base_alloc&       right        ,
        const base_alloc&       exp_if_true  ,
        const base_alloc&       exp_if_false )
    {   // not used
        assert(false);

        // to void compiler error
        return base_alloc();
    }
}
/* %$$

$head CondExpRel$$
The $cref/CPPAD_COND_EXP_REL/base_cond_exp/CondExpRel/$$ macro invocation
$srccode%cpp% */
namespace CppAD {
    CPPAD_COND_EXP_REL(base_alloc)
}
/* %$$
uses $code CondExpOp$$ above to
define $codei%CondExp%Rel%$$ for $code base_alloc$$ arguments
and $icode%Rel%$$ equal to
$code Lt$$, $code Le$$, $code Eq$$, $code Ge$$, and $code Gt$$.

$head EqualOpSeq$$
The type $code base_alloc$$ is simple (in this respect) and so we define
$srccode%cpp% */
namespace CppAD {
    inline bool EqualOpSeq(const base_alloc& x, const base_alloc& y)
    {   return *x.ptrdbl_ == *y.ptrdbl_; }
}
/* %$$

$head Identical$$
The type $code base_alloc$$ is simple (in this respect) and so we define
$srccode%cpp% */
namespace CppAD {
    inline bool IdenticalCon(const base_alloc& x)
    {   return true; }
    inline bool IdenticalZero(const base_alloc& x)
    {   return (*x.ptrdbl_ == 0.0); }
    inline bool IdenticalOne(const base_alloc& x)
    {   return (*x.ptrdbl_ == 1.0); }
    inline bool IdenticalEqualCon(const base_alloc& x, const base_alloc& y)
    {   return (*x.ptrdbl_ == *y.ptrdbl_); }
}
/* %$$

$head Output Operator$$
$srccode%cpp% */
namespace CppAD {
    inline std::ostream& operator << (std::ostream &os, const base_alloc& x)
    {   os << *x.ptrdbl_;
        return os;
    }
}
/* %$$

$head Integer$$
$srccode%cpp% */
namespace CppAD {
    inline int Integer(const base_alloc& x)
    {   return static_cast<int>(*x.ptrdbl_); }
}
/* %$$

$head azmul$$
$srccode%cpp% */
namespace CppAD {
    CPPAD_AZMUL( base_alloc )
}
/* %$$

$head Ordered$$
The $code base_alloc$$ type supports ordered comparisons
$srccode%cpp% */
namespace CppAD {
    inline bool GreaterThanZero(const base_alloc& x)
    {   return *x.ptrdbl_ > 0.0; }
    inline bool GreaterThanOrZero(const base_alloc& x)
    {   return *x.ptrdbl_ >= 0.0; }
    inline bool LessThanZero(const base_alloc& x)
    {   return *x.ptrdbl_ < 0.0; }
    inline bool LessThanOrZero(const base_alloc& x)
    {   return *x.ptrdbl_ <= 0.f; }
    inline bool abs_geq(const base_alloc& x, const base_alloc& y)
    {   return std::fabs(*x.ptrdbl_) >= std::fabs(*y.ptrdbl_); }
}
/* %$$

$head Unary Standard Math$$
The macro
$cref/CPPAD_STANDARD_MATH_UNARY/base_std_math/CPPAD_STANDARD_MATH_UNARY/$$
would not work with the type $code base_alloc$$ so we define
a special macro for this type:
$srccode%cpp% */
# define BASE_ALLOC_STD_MATH(fun) \
    inline base_alloc fun (const base_alloc& x) \
    { return   std::fun(*x.ptrdbl_); }
/* %$$
The following invocations of the macro above define the
$cref/unary standard math/base_std_math/Unary Standard Math/$$ functions
(except for $code abs$$):
$srccode%cpp% */
namespace CppAD {
    BASE_ALLOC_STD_MATH(acos)
    BASE_ALLOC_STD_MATH(acosh)
    BASE_ALLOC_STD_MATH(asin)
    BASE_ALLOC_STD_MATH(asinh)
    BASE_ALLOC_STD_MATH(atan)
    BASE_ALLOC_STD_MATH(atanh)
    BASE_ALLOC_STD_MATH(cos)
    BASE_ALLOC_STD_MATH(cosh)
    BASE_ALLOC_STD_MATH(erf)
    BASE_ALLOC_STD_MATH(erfc)
    BASE_ALLOC_STD_MATH(exp)
    BASE_ALLOC_STD_MATH(expm1)
    BASE_ALLOC_STD_MATH(fabs)
    BASE_ALLOC_STD_MATH(log)
    BASE_ALLOC_STD_MATH(log1p)
    BASE_ALLOC_STD_MATH(log10)
    BASE_ALLOC_STD_MATH(sin)
    BASE_ALLOC_STD_MATH(sinh)
    BASE_ALLOC_STD_MATH(sqrt)
    BASE_ALLOC_STD_MATH(tan)
    BASE_ALLOC_STD_MATH(tanh)
}
/* %$$
The absolute value function is special because it $code std$$ name is
$code fabs$$
$srccode%cpp% */
namespace CppAD {
    inline base_alloc abs(const base_alloc& x)
    {   return fabs(*x.ptrdbl_); }
}
/* %$$
The isnan function is special because it returns a bool
$srccode%cpp% */
namespace CppAD {
    inline bool isnan(const base_alloc& x)
    {   return *x.ptrdbl_ != *x.ptrdbl_; }
}
/* %$$

$head sign$$
The following defines the $code CppAD::sign$$ function that
is required to use $code AD<base_alloc>$$:
$srccode%cpp% */
namespace CppAD {
    inline base_alloc sign(const base_alloc& x)
    {   if( *x.ptrdbl_ > 0.0 )
            return 1.0;
        if( *x.ptrdbl_ == 0.0 )
            return 0.0;
        return -1.0;
    }
}
/* %$$

$head pow $$
The following defines a $code CppAD::pow$$ function that
is required to use $code AD<base_alloc>$$:
$srccode%cpp% */
namespace CppAD {
    inline base_alloc pow(const base_alloc& x, const base_alloc& y)
    { return std::pow(*x.ptrdbl_, *y.ptrdbl_); }
}
/* %$$

$head numeric_limits$$
The following defines the CppAD $cref numeric_limits$$
for the type $code base_alloc$$:
$srccode%cpp% */
namespace CppAD {
    CPPAD_NUMERIC_LIMITS(double, base_alloc)
}
/* %$$

$head to_string$$
The following defines the CppAD $cref to_string$$ function
for the type $code base_alloc$$:
$srccode%cpp% */
namespace CppAD {
    CPPAD_TO_STRING(base_alloc)
}
/* %$$

$head hash_code$$
The $cref/default/base_hash/Default/$$ hashing function does
not work well for this case because two different pointers can
have the same value.
$srccode|cpp| */
namespace CppAD {
    inline unsigned short hash_code(const base_alloc& x)
    {   unsigned short code = 0;
        if( *x.ptrdbl_ == 0.0 )
            return code;
        double log_x = log( std::fabs( *x.ptrdbl_ ) );
        // assume log( std::numeric_limits<double>::max() ) is near 700
        code = static_cast<unsigned short>(
            (CPPAD_HASH_TABLE_SIZE / 700 + 1) * log_x
        );
        code = code % CPPAD_HASH_TABLE_SIZE;
        return code;
    }
}
/* |$$
$end
*/
# endif
