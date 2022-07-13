# ifndef CPPAD_CORE_POW_HPP
# define CPPAD_CORE_POW_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin pow$$
$spell
    Vec
    std
    namespace
    CppAD
    const
$$


$section The AD Power Function$$

$head Syntax$$
$icode%z% = pow(%x%, %y%)%$$

$head See Also$$
$cref pow_int$$


$head Purpose$$
Determines the value of the power function which is defined by
$latex \[
    {\rm pow} (x, y) = x^y
\] $$

$subhead If y is a Variable$$
If $icode y$$ is a variable,
the $code pow$$ function may use
logarithms and exponentiation to compute derivatives.
This will not work if $icode x$$ is less than or equal zero.

$subhead If y is a Parameter$$
If $icode y$$ is a parameter, a different method is used to
compute the derivatives; see $cref pow_forward$$.
In the special case where $icode x$$ is zero,
zero is returned as the derivative.
This is correct when $icode y$$ minus the order of the derivative
is greater than zero.
If $icode y$$ minus the order of the derivative is zero,
then $icode y$$ is an integer.
If $icode y$$ minus the order of the derivative is less than zero,
the actual derivative is infinite.

$subhead If y is an Integer$$
If the value of $icode y$$ is an integer,
the $cref pow_int$$ function can be used to compute this value
using only multiplication (and division if $icode y$$ is negative).
This will work even if $icode x$$ is less than or equal zero.


$head x$$
The argument $icode x$$ has one of the following prototypes
$codei%
    const %Base%&                    %x%
    const AD<%Base%>&                %x%
    const VecAD<%Base%>::reference&  %x%
%$$

$head y$$
The argument $icode y$$ has one of the following prototypes
$codei%
    const %Base%&                    %y%
    const AD<%Base%>&                %y%
    const VecAD<%Base%>::reference&  %y%
%$$

$head z$$
If both $icode x$$ and $icode y$$ are $icode Base$$ objects,
the result $icode z$$ is also a $icode Base$$ object.
Otherwise, it has prototype
$codei%
    AD<%Base%> %z%
%$$

$head Operation Sequence$$
This is an AD of $icode Base$$
$cref/atomic operation/glossary/Operation/Atomic/$$
and hence is part of the current
AD of $icode Base$$
$cref/operation sequence/glossary/Operation/Sequence/$$.

$head Example$$
$children%
    example/general/pow.cpp
    %example/general/pow_nan.cpp
%$$
The files
$cref pow.cpp$$ and $cref pow_nan.cpp$$
are examples tests of this function.

$end
-------------------------------------------------------------------------------
*/

//  BEGIN CppAD namespace
namespace CppAD {

// case where x and y are AD<Base> -----------------------------------------
template <class Base> AD<Base>
pow(const AD<Base>& x, const AD<Base>& y)
{
    // compute the Base part
    AD<Base> result;
    result.value_  = pow(x.value_, y.value_);
    CPPAD_ASSERT_UNKNOWN( Parameter(result) );

    // check if there is a recording in progress
    local::ADTape<Base>* tape = AD<Base>::tape_ptr();
    if( tape == nullptr )
        return result;
    tape_id_t tape_id = tape->id_;
    // tape_id cannot match the default value for tape_id_; i.e., 0
    CPPAD_ASSERT_UNKNOWN( tape_id > 0 );

    // check if x and y tapes match
    bool match_x  = x.tape_id_  == tape_id;
    bool match_y  = y.tape_id_  == tape_id;

    // check if x and y are dynamic parameters
    bool dyn_x  = match_x  & (x.ad_type_ == dynamic_enum);
    bool dyn_y  = match_y  & (y.ad_type_ == dynamic_enum);

    // check if x and y are variables
    bool var_x  = match_x  & (x.ad_type_ != dynamic_enum);
    bool var_y  = match_y  & (y.ad_type_ != dynamic_enum);

    CPPAD_ASSERT_KNOWN(
        x.tape_id_ == y.tape_id_ || ! match_x || ! match_y ,
        "pow: AD variables or dynamic parameters on different threads."
    );
    if( var_x )
    {   if( var_y )
        {   // result = variable^variable
            CPPAD_ASSERT_UNKNOWN( local::NumRes(local::PowvvOp) == 3 );
            CPPAD_ASSERT_UNKNOWN( local::NumArg(local::PowvvOp) == 2 );

            // put operand addresses in tape
            tape->Rec_.PutArg(x.taddr_, y.taddr_);

            // put operator in the tape
            result.taddr_ = tape->Rec_.PutOp(local::PowvvOp);

            // make result a variable
            result.tape_id_ = tape_id;
            result.ad_type_ = variable_enum;
        }
        else if( IdenticalZero( y.value_ ) )
        {   // result = variable^0
        }
        else
        {   // result = variable^parameter
            CPPAD_ASSERT_UNKNOWN( local::NumRes(local::PowvpOp) == 1 );
            CPPAD_ASSERT_UNKNOWN( local::NumArg(local::PowvpOp) == 2 );

            // put operand addresses in tape
            addr_t p = y.taddr_;
            if( ! dyn_y )
                p = tape->Rec_.put_con_par(y.value_);
            tape->Rec_.PutArg(x.taddr_, p);

            // put operator in the tape
            result.taddr_ = tape->Rec_.PutOp(local::PowvpOp);

            // make result a variable
            result.tape_id_ = tape_id;
            result.ad_type_ = variable_enum;
        }
    }
    else if( var_y )
    {   if( IdenticalZero(x.value_) )
        {   // result = 0^variable
        }
        else
        {   // result = parameter^variable
            CPPAD_ASSERT_UNKNOWN( local::NumRes(local::PowpvOp) == 3 );
            CPPAD_ASSERT_UNKNOWN( local::NumArg(local::PowpvOp) == 2 );

            // put operand addresses in tape
            addr_t p = x.taddr_;
            if( ! dyn_x )
                p = tape->Rec_.put_con_par(x.value_);
            tape->Rec_.PutArg(p, y.taddr_);

            // put operator in the tape
            result.taddr_ = tape->Rec_.PutOp(local::PowpvOp);

            // make result a variable
            result.tape_id_ = tape_id;
            result.ad_type_ = variable_enum;
        }
    }
    else if( dyn_x | dyn_y )
    {   addr_t arg0 = x.taddr_;
        addr_t arg1 = y.taddr_;
        if( ! dyn_x )
            arg0 = tape->Rec_.put_con_par(x.value_);
        if( ! dyn_y )
            arg1 = tape->Rec_.put_con_par(y.value_);
        //
        // parameters with a dynamic parameter result
        result.taddr_   = tape->Rec_.put_dyn_par(
            result.value_, local::pow_dyn,   arg0, arg1
        );
        result.tape_id_ = tape_id;
        result.ad_type_ = dynamic_enum;
    }
    else
    {   CPPAD_ASSERT_KNOWN( ! (dyn_x | dyn_y) ,
        "pow: one operand is a dynamic parameter and other not a variable"
        );
    }
    return result;
}
// =========================================================================
// Fold operations in same way as CPPAD_FOLD_AD_VALUED_BINARY_OPERATOR(Op)
// -------------------------------------------------------------------------
// Operations with VecAD_reference<Base> and AD<Base> only

template <class Base> AD<Base>
pow(const AD<Base>& x, const VecAD_reference<Base>& y)
{   return pow(x, y.ADBase()); }

template <class Base> AD<Base>
pow(const VecAD_reference<Base>& x, const VecAD_reference<Base>& y)
{   return pow(x.ADBase(), y.ADBase()); }

template <class Base> AD<Base>
pow(const VecAD_reference<Base>& x, const AD<Base>& y)
{   return pow(x.ADBase(), y); }
// -------------------------------------------------------------------------
// Operations with Base

template <class Base> AD<Base>
pow(const Base& x, const AD<Base>& y)
{   return pow(AD<Base>(x), y); }

template <class Base> AD<Base>
pow(const Base& x, const VecAD_reference<Base>& y)
{   return pow(AD<Base>(x), y.ADBase()); }

template <class Base> AD<Base>
pow(const AD<Base>& x, const Base& y)
{   return pow(x, AD<Base>(y)); }

template <class Base> AD<Base>
pow(const VecAD_reference<Base>& x, const Base& y)
{   return pow(x.ADBase(), AD<Base>(y)); }
// -------------------------------------------------------------------------
// Operations with double

template <class Base> AD<Base>
pow(const double& x, const AD<Base>& y)
{   return pow(AD<Base>(x), y); }

template <class Base> AD<Base>
pow(const double& x, const VecAD_reference<Base>& y)
{   return pow(AD<Base>(x), y.ADBase()); }

template <class Base> AD<Base>
pow(const AD<Base>& x, const double& y)
{   return pow(x, AD<Base>(y)); }

template <class Base> AD<Base>
pow(const VecAD_reference<Base>& x, const double& y)
{   return pow(x.ADBase(), AD<Base>(y)); }
// -------------------------------------------------------------------------
// Special case to avoid ambuigity when Base is double

inline AD<double>
pow(const double& x, const AD<double>& y)
{   return pow(AD<double>(x), y); }

inline AD<double>
pow(const double& x, const VecAD_reference<double>& y)
{   return pow(AD<double>(x), y.ADBase()); }

inline AD<double>
pow(const AD<double>& x, const double& y)
{   return pow(x, AD<double>(y)); }

inline AD<double>
pow(const VecAD_reference<double>& x, const double& y)
{   return pow(x.ADBase(), AD<double>(y)); }

// =========================================================================
// Fold operations for the cases where x is an int,
// but let cppad/utility/pow_int.hpp handle the cases where y is an int.
// -------------------------------------------------------------------------
template <class Base> AD<Base> pow
(const int& x, const VecAD_reference<Base>& y)
{   return pow(AD<Base>(x), y.ADBase()); }

template <class Base> AD<Base> pow
(const int& x, const AD<Base>& y)
{   return pow(AD<Base>(x), y); }

} // END CppAD namespace

# endif
