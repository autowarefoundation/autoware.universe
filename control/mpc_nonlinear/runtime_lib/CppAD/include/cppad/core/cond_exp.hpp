# ifndef CPPAD_CORE_COND_EXP_HPP
# define CPPAD_CORE_COND_EXP_HPP
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
$begin CondExp$$
$spell
    Atan2
    CondExp
    Taylor
    std
    Cpp
    namespace
    inline
    const
    abs
    Rel
    bool
    Lt
    Le
    Eq
    Ge
    Gt
$$


$section AD Conditional Expressions$$

$head Syntax$$
$icode%result% = CondExp%Rel%(%left%, %right%, %if_true%, %if_false%)%$$


$head Purpose$$
Record,
as part of an AD of $icode Base$$
$cref/operation sequence/glossary/Operation/Sequence/$$,
the conditional result
$codei%
    if( %left% %Cop% %right% )
        %result% = %if_true%
    else
        %result% = %if_false%
%$$
The relational $icode Rel$$ and comparison operator $icode Cop$$
above have the following correspondence:
$codei%
    %Rel%   Lt   Le   Eq   Ge   Gt
    %Cop%    <   <=   ==   >=   >
%$$
If $icode f$$ is the $cref ADFun$$ object corresponding to the
AD operation sequence,
the assignment choice for $icode result$$
in an AD conditional expression is made each time
$cref/f.Forward/Forward/$$ is used to evaluate the zero order Taylor
coefficients with new values for the
$cref/independent variables/glossary/Tape/Independent Variable/$$.
This is in contrast to the $cref/AD comparison operators/Compare/$$
which are boolean valued and not included in the AD operation sequence.

$head Rel$$
In the syntax above, the relation $icode Rel$$ represents one of the following
two characters: $code Lt$$, $code Le$$, $code Eq$$, $code Ge$$, $code Gt$$.
As in the table above,
$icode Rel$$ determines which comparison operator $icode Cop$$ is used
when comparing $icode left$$ and $icode right$$.

$head Type$$
These functions are defined in the CppAD namespace for arguments of
$icode Type$$ is $code float$$ , $code double$$, or any type of the form
$codei%AD<%Base%>%$$.
(Note that all four arguments must have the same type.)

$head left$$
The argument $icode left$$ has prototype
$codei%
    const %Type%& %left%
%$$
It specifies the value for the left side of the comparison operator.

$head right$$
The argument $icode right$$ has prototype
$codei%
    const %Type%& %right%
%$$
It specifies the value for the right side of the comparison operator.

$head if_true$$
The argument $icode if_true$$ has prototype
$codei%
    const %Type%& %if_true%
%$$
It specifies the return value if the result of the comparison is true.

$head if_false$$
The argument $icode if_false$$ has prototype
$codei%
    const %Type%& %if_false%
%$$
It specifies the return value if the result of the comparison is false.

$head result$$
The $icode result$$ has prototype
$codei%
    %Type%& %if_false%
%$$

$head Optimize$$
The $cref optimize$$ method will optimize conditional expressions
in the following way:
During $cref/zero order forward mode/forward_zero/$$,
once the value of the $icode left$$ and $icode right$$ have been determined,
it is known if the true or false case is required.
From this point on, values corresponding to the case that is not required
are not computed.
This optimization is done for the rest of zero order forward mode
as well as forward and reverse derivatives calculations.

$head Deprecate 2005-08-07$$
Previous versions of CppAD used
$codei%
    CondExp(%flag%, %if_true%, %if_false%)
%$$
for the same meaning as
$codei%
    CondExpGt(%flag%, %Type%(0), %if_true%, %if_false%)
%$$
Use of $code CondExp$$ is deprecated, but continues to be supported.

$head Operation Sequence$$
This is an AD of $icode Base$$
$cref/atomic operation/glossary/Operation/Atomic/$$
and hence is part of the current
AD of $icode Base$$
$cref/operation sequence/glossary/Operation/Sequence/$$.


$head Example$$

$head Test$$
$children%
    example/general/cond_exp.cpp
%$$
The file
$cref cond_exp.cpp$$
contains an example and test of this function.

$head Atan2$$
The following implementation of the
AD $cref atan2$$ function is a more complex
example of using conditional expressions:
$srcfile%include/cppad/core/atan2.hpp%0%BEGIN CondExp%// END CondExp%$$


$end
-------------------------------------------------------------------------------
*/
//  BEGIN CppAD namespace
namespace CppAD {

template <class Base>
AD<Base> CondExpOp(
    enum  CompareOp cop       ,
    const AD<Base> &left      ,
    const AD<Base> &right     ,
    const AD<Base> &if_true   ,
    const AD<Base> &if_false  )
{
    AD<Base> result;
    CPPAD_ASSERT_UNKNOWN( Parameter(result) );

    // check first case where do not need to tape
    if( IdenticalCon(left) & IdenticalCon(right) )
    {   switch( cop )
        {
            case CompareLt:
            if( left.value_ < right.value_ )
                result = if_true;
            else
                result = if_false;
            break;

            case CompareLe:
            if( left.value_ <= right.value_ )
                result = if_true;
            else
                result = if_false;
            break;

            case CompareEq:
            if( left.value_ == right.value_ )
                result = if_true;
            else
                result = if_false;
            break;

            case CompareGe:
            if( left.value_ >= right.value_ )
                result = if_true;
            else
                result = if_false;
            break;

            case CompareGt:
            if( left.value_ > right.value_ )
                result = if_true;
            else
                result = if_false;
            break;

            default:
            CPPAD_ASSERT_UNKNOWN(0);
            result = if_true;
        }
        return result;
    }

    // must use CondExp in case Base is an AD type and recording
    result.value_ = CondExpOp(cop,
        left.value_, right.value_, if_true.value_, if_false.value_);

    local::ADTape<Base> *tape = AD<Base>::tape_ptr();

    // add this operation to the tape
    if( tape != nullptr ) tape->Rec_.cond_exp(
            tape->id_, cop, result, left, right, if_true, if_false
    );

    return result;
}

// ------------ CondExpOp(left, right, if_true, if_false) ----------------

# define CPPAD_COND_EXP(Name)                                        \
    template <class Base>                                           \
    CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION                           \
    AD<Base> CondExp##Name(                                         \
        const AD<Base> &left      ,                                \
        const AD<Base> &right     ,                                \
        const AD<Base> &if_true   ,                                \
        const AD<Base> &if_false  )                                \
    {                                                               \
        return CondExpOp(Compare##Name,                            \
            left, right, if_true, if_false);                      \
    }

// AD<Base>
CPPAD_COND_EXP(Lt)
CPPAD_COND_EXP(Le)
CPPAD_COND_EXP(Eq)
CPPAD_COND_EXP(Ge)
CPPAD_COND_EXP(Gt)
template <class Base>
CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
AD<Base> CondExp(
    const AD<Base> &flag      ,
    const AD<Base> &if_true   ,
    const AD<Base> &if_false  )
{
    return CondExpOp(CompareGt, flag, AD<Base>(0), if_true, if_false);
}

# undef CPPAD_COND_EXP
} // END CppAD namespace

# endif
