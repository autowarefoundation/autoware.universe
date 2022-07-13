# ifndef CPPAD_CORE_ABS_HPP
# define CPPAD_CORE_ABS_HPP
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
-------------------------------------------------------------------------------
$begin abs$$
$spell
    fabs
    Vec
    std
    faq
    Taylor
    Cpp
    namespace
    const
    abs
$$

$section AD Absolute Value Functions: abs, fabs$$

$head Syntax$$
$icode%y% = abs(%x%)
%$$
$icode%y% = fabs(%x%)%$$

$head x, y$$
See the $cref/possible types/unary_standard_math/Possible Types/$$
for a unary standard math function.

$head Atomic$$
In the case where $icode x$$ is an AD type,
this is an $cref/atomic operation/glossary/Operation/Atomic/$$.

$head Complex Types$$
The functions $code abs$$ and $icode fabs$$
are not defined for the base types
$code std::complex<float>$$ or $code std::complex<double>$$
because the complex $code abs$$ function is not complex differentiable
(see $cref/complex types faq/Faq/Complex Types/$$).

$head Derivative$$
CppAD defines the derivative of the $code abs$$ function is
the $cref sign$$ function; i.e.,
$latex \[
{\rm abs}^{(1)} ( x ) = {\rm sign} (x ) =
\left\{ \begin{array}{rl}
    +1 & {\rm if} \; x > 0 \\
    0  & {\rm if} \; x = 0 \\
    -1 & {\rm if} \; x < 0
\end{array} \right.
\] $$
The result for $icode%x% == 0%$$ used to be a directional derivative.

$head Example$$
$children%
    example/general/fabs.cpp
%$$
The file
$cref fabs.cpp$$
contains an example and test of this function.

$end
-------------------------------------------------------------------------------
*/

//  BEGIN CppAD namespace
namespace CppAD {

template <class Base>
AD<Base> AD<Base>::abs_me (void) const
{
    AD<Base> result;
    result.value_ = abs(value_);
    CPPAD_ASSERT_UNKNOWN( Parameter(result) );

    // check if there is a recording in progress
    local::ADTape<Base>* tape = AD<Base>::tape_ptr();
    if( tape == nullptr )
        return result;

    // check if operand is a constant parameter
    if( tape_id_ != tape->id_ )
        return result;

    if(ad_type_ == dynamic_enum)
    {   // dynamic paramter argument
        result.taddr_   = tape->Rec_.put_dyn_par(
            result.value_, local::abs_dyn, taddr_
        );
        result.tape_id_  = tape_id_;
        result.ad_type_  = dynamic_enum;
    }
    else
    {   // variable argument
        CPPAD_ASSERT_UNKNOWN( local::NumRes(local::AbsOp) == 1 );
        CPPAD_ASSERT_UNKNOWN( local::NumArg(local::AbsOp) == 1 );

        // corresponding operand address
        tape->Rec_.PutArg(taddr_);

        // put operator in the tape
        result.taddr_    = tape->Rec_.PutOp(local::AbsOp);

        // make result a variable
        result.tape_id_  = tape_id_;
        result.ad_type_  = variable_enum;
    }
    return result;
}

template <class Base>
AD<Base> abs(const AD<Base> &x)
{   return x.abs_me(); }

template <class Base>
AD<Base> abs(const VecAD_reference<Base> &x)
{   return x.ADBase().abs_me(); }

} // END CppAD namespace

# endif
