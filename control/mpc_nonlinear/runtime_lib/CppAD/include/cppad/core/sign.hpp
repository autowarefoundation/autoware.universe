# ifndef CPPAD_CORE_SIGN_HPP
# define CPPAD_CORE_SIGN_HPP
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
$begin sign$$
$spell
    CppAD
    Dirac
$$
$section The Sign: sign$$

$head Syntax$$
$icode%y% = sign(%x%)%$$

$head Description$$
Evaluates the $code sign$$ function which is defined by
$latex \[
{\rm sign} (x) =
\left\{ \begin{array}{rl}
    +1 & {\rm if} \; x > 0 \\
    0  & {\rm if} \; x = 0 \\
    -1 & {\rm if} \; x < 0
\end{array} \right.
\] $$

$head x, y$$
See the $cref/possible types/unary_standard_math/Possible Types/$$
for a unary standard math function.

$head Atomic$$
This is an $cref/atomic operation/glossary/Operation/Atomic/$$.

$head Derivative$$
CppAD computes the derivative of the $code sign$$ function as zero for all
argument values $icode x$$.
The correct mathematical derivative is different and
is given by
$latex \[
    {\rm sign}^{(1)} (x) =  2 \delta (x)
\] $$
where $latex \delta (x)$$ is the Dirac Delta function.

$head Example$$
$children%
    example/general/sign.cpp
%$$
The file
$cref sign.cpp$$
contains an example and test of this function.

$end
-------------------------------------------------------------------------------
*/

//  BEGIN CppAD namespace
namespace CppAD {

template <class Base>
AD<Base> AD<Base>::sign_me (void) const
{
    AD<Base> result;
    result.value_ = sign(value_);
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
            result.value_, local::sign_dyn, taddr_
        );
        result.tape_id_  = tape_id_;
        result.ad_type_  = dynamic_enum;
    }
    else
    {   // variable argument
        CPPAD_ASSERT_UNKNOWN( local::NumRes(local::SignOp) == 1 );
        CPPAD_ASSERT_UNKNOWN( local::NumArg(local::SignOp) == 1 );

        // corresponding operand address
        tape->Rec_.PutArg(taddr_);

        // put operator in the tape
        result.taddr_ = tape->Rec_.PutOp(local::SignOp);

        // make result a variable
        result.tape_id_ = tape->id_;
        result.ad_type_ = variable_enum;
    }
    return result;
}

template <class Base>
AD<Base> sign(const AD<Base> &x)
{   return x.sign_me();
}
template <class Base>
AD<Base> sign(const VecAD_reference<Base> &x)
{   return x.ADBase().sign_me(); }

} // END CppAD namespace

# endif
