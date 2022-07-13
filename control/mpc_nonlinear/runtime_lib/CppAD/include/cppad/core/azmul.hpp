# ifndef CPPAD_CORE_AZMUL_HPP
# define CPPAD_CORE_AZMUL_HPP
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
$begin azmul$$
$spell
    azmul
    const
    namespace
    Vec
$$

$section Absolute Zero Multiplication$$

$head Syntax$$
$icode%z% = azmul(%x%, %y%)%$$

$head Purpose$$
Evaluates multiplication with an absolute zero
for any of the possible types listed below.
The result is given by
$latex \[
z = \left\{ \begin{array}{ll}
    0          & {\rm if} \; x = 0 \\
    x \cdot y  & {\rm otherwise}
\end{array} \right.
\] $$
Note if $icode x$$ is zero and $icode y$$ is infinity,
ieee multiplication would result in not a number whereas
$icode z$$ would be zero.

$head Base$$
If $icode Base$$ satisfies the
$cref/base type requirements/base_require/$$
and arguments $icode x$$, $icode y$$ have prototypes
$codei%
    const %Base%& %x%
    const %Base%& %y%
%$$
then the result $icode z$$ has prototype
$codei%
    %Base% %z%
%$$

$head AD<Base>$$
If the arguments $icode x$$, $icode y$$ have prototype
$codei%
    const AD<%Base%>& %x%
    const AD<%Base%>& %y%
%$$
then the result $icode z$$ has prototype
$codei%
    AD<%Base%> %z%
%$$

$head VecAD<Base>$$
If the arguments $icode x$$, $icode y$$ have prototype
$codei%
    const VecAD<%Base%>::reference& %x%
    const VecAD<%Base%>::reference& %y%
%$$
then the result $icode z$$ has prototype
$codei%
    AD<%Base%> %z%
%$$

$head Example$$
$children%
    example/general/azmul.cpp
%$$
The file
$cref azmul.cpp$$
is an examples and tests of this function.

$end
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
// ==========================================================================

// case where x and y are AD<Base> -------------------------------------------
template <class Base> AD<Base>
azmul(const AD<Base>& x, const AD<Base>& y)
{
    // compute the Base part
    AD<Base> result;
    result.value_ = azmul(x.value_, y.value_);

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
        "azmul: AD variables or dynamic parameters on different threads."
    );
    if( var_x )
    {   if( var_y )
        {   // result = azmul(variable, variable)
            CPPAD_ASSERT_UNKNOWN( local::NumRes(local::ZmulvvOp) == 1 );
            CPPAD_ASSERT_UNKNOWN( local::NumArg(local::ZmulvvOp) == 2 );

            // put operand addresses in tape
            tape->Rec_.PutArg(x.taddr_, y.taddr_);

            // put operator in the tape
            result.taddr_ = tape->Rec_.PutOp(local::ZmulvvOp);

            // make result a variable
            result.tape_id_ = tape_id;
            result.ad_type_ = variable_enum;
        }
        else if( ( ! dyn_y ) & IdenticalZero( y.value_ ) )
        {   // result = variable * 0
        }
        else if( ( ! dyn_y ) & IdenticalOne( y.value_ ) )
        {   // result = variable * 1
            result.make_variable(x.tape_id_, x.taddr_);
        }
        else
        {   // result = zmul(variable, parameter)
            CPPAD_ASSERT_UNKNOWN( local::NumRes(local::ZmulvpOp) == 1 );
            CPPAD_ASSERT_UNKNOWN( local::NumArg(local::ZmulvpOp) == 2 );

            // put operand addresses in tape
            addr_t p = y.taddr_;
            if( ! dyn_y )
                p = tape->Rec_.put_con_par(y.value_);
            tape->Rec_.PutArg(x.taddr_, p);

            // put operator in the tape
            result.taddr_ = tape->Rec_.PutOp(local::ZmulvpOp);

            // make result a variable
            result.tape_id_ = tape_id;
            result.ad_type_ = variable_enum;
        }
    }
    else if( var_y )
    {   if( ( ! dyn_x ) & IdenticalZero(x.value_) )
        {   // result = 0 * variable
        }
        else if( ( ! dyn_x ) & IdenticalOne( x.value_ ) )
        {   // result = 1 * variable
            result.make_variable(y.tape_id_, y.taddr_);
        }
        else
        {   // result = zmul(parameter, variable)
            CPPAD_ASSERT_UNKNOWN( local::NumRes(local::ZmulpvOp) == 1 );
            CPPAD_ASSERT_UNKNOWN( local::NumArg(local::ZmulpvOp) == 2 );

            // put operand addresses in tape
            addr_t p = x.taddr_;
            if( ! dyn_x )
                p = tape->Rec_.put_con_par(x.value_);
            tape->Rec_.PutArg(p, y.taddr_);

            // put operator in the tape
            result.taddr_ = tape->Rec_.PutOp(local::ZmulpvOp);

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
            result.value_, local::zmul_dyn,   arg0, arg1
        );
        result.tape_id_ = tape_id;
        result.ad_type_ = dynamic_enum;
    }
    return result;
}
// =========================================================================
// Fold operations into case above
// -------------------------------------------------------------------------
// Operations with VecAD_reference<Base> and AD<Base> only

template <class Base> AD<Base>
azmul(const AD<Base>& x, const VecAD_reference<Base>& y)
{   return azmul(x, y.ADBase()); }

template <class Base> AD<Base>
azmul(const VecAD_reference<Base>& x, const VecAD_reference<Base>& y)
{   return azmul(x.ADBase(), y.ADBase()); }

template <class Base> AD<Base>
azmul(const VecAD_reference<Base>& x, const AD<Base>& y)
{   return azmul(x.ADBase(), y); }
// -------------------------------------------------------------------------
// Operations with Base

template <class Base> AD<Base>
azmul(const Base& x, const AD<Base>& y)
{   return azmul(AD<Base>(x), y); }

template <class Base> AD<Base>
azmul(const Base& x, const VecAD_reference<Base>& y)
{   return azmul(AD<Base>(x), y.ADBase()); }

template <class Base> AD<Base>
azmul(const AD<Base>& x, const Base& y)
{   return azmul(x, AD<Base>(y)); }

template <class Base> AD<Base>
azmul(const VecAD_reference<Base>& x, const Base& y)
{   return azmul(x.ADBase(), AD<Base>(y)); }

// ==========================================================================
} // END_CPPAD_NAMESPACE

# endif
