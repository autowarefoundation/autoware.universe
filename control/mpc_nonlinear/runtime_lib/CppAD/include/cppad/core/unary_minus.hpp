# ifndef CPPAD_CORE_UNARY_MINUS_HPP
# define CPPAD_CORE_UNARY_MINUS_HPP
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
$begin UnaryMinus$$
$spell
    Vec
    const
    inline
$$


$section AD Unary Minus Operator$$

$head Syntax$$

$icode%y% = - %x%$$


$head Purpose$$
Computes the negative of $icode x$$.

$head Base$$
The operation in the syntax above must be supported for the case where
the operand is a $code const$$ $icode Base$$ object.

$head x$$
The operand $icode x$$ has one of the following prototypes
$codei%
    const AD<%Base%>               &%x%
    const VecAD<%Base%>::reference &%x%
%$$

$head y$$
The result $icode y$$ has type
$codei%
    AD<%Base%> %y%
%$$
It is equal to the negative of the operand $icode x$$.

$head Operation Sequence$$
This is an AD of $icode Base$$
$cref/atomic operation/glossary/Operation/Atomic/$$
and hence is part of the current
AD of $icode Base$$
$cref/operation sequence/glossary/Operation/Sequence/$$.

$head Derivative$$
If $latex f$$ is a
$cref/Base function/glossary/Base Function/$$,
$latex \[
    \D{[ - f(x) ]}{x} = - \D{f(x)}{x}
\] $$

$head Example$$
$children%
    example/general/unary_minus.cpp
%$$
The file
$cref unary_minus.cpp$$
contains an example and test of this operation.

$end
-------------------------------------------------------------------------------
*/

//  BEGIN CppAD namespace
namespace CppAD {
//
template <class Base>
AD<Base> AD<Base>::operator - (void) const
{
    // compute the Base part of this AD object
    AD<Base> result;
    result.value_ = - value_;
    CPPAD_ASSERT_UNKNOWN( Parameter(result) );

    // check if there is a recording in progress
    local::ADTape<Base>* tape = AD<Base>::tape_ptr();
    if( tape == nullptr )
        return result;
    // tape_id cannot match the default value for tape_id; i.e., 0
    CPPAD_ASSERT_UNKNOWN( tape->id_ > 0 );
    //
    if( tape->id_ != tape_id_ )
        return result;
    //
    if( ad_type_ == variable_enum )
    {   // result is a variable
        CPPAD_ASSERT_UNKNOWN( local::NumRes(local::NegOp) == 1 );
        CPPAD_ASSERT_UNKNOWN( local::NumRes(local::NegOp) == 1 );
        //
        // put operand address in the tape
        tape->Rec_.PutArg(taddr_);
        // put operator in the tape
        result.taddr_ = tape->Rec_.PutOp(local::NegOp);
        // make result a variable
        result.tape_id_ = tape_id_;
        result.ad_type_ = variable_enum;
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( ad_type_ == dynamic_enum );
        addr_t arg0 = taddr_;
        result.taddr_ = tape->Rec_.put_dyn_par(
            result.value_, local::neg_dyn, arg0
        );
        result.tape_id_  = tape_id_;
        result.ad_type_  = dynamic_enum;
    }
    return result;
}
//
template <class Base>
AD<Base> operator - (const VecAD_reference<Base> &right)
{   return - right.ADBase(); }

}
//  END CppAD namespace


# endif
