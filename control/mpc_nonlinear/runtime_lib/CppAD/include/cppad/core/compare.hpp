# ifndef CPPAD_CORE_COMPARE_HPP
# define CPPAD_CORE_COMPARE_HPP
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
$begin Compare$$
$spell
    cos
    Op
    bool
    const
$$



$section AD Binary Comparison Operators$$


$head Syntax$$

$icode%b% = %x% %Op% %y%$$


$head Purpose$$
Compares two operands where one of the operands is an
$codei%AD<%Base%>%$$ object.
The comparison has the same interpretation as for
the $icode Base$$ type.


$head Op$$
The operator $icode Op$$ is one of the following:
$table
$bold Op$$ $pre $$  $cnext $bold Meaning$$                           $rnext
$code <$$   $cnext is $icode x$$ less than $icode y$$              $rnext
$code <=$$  $cnext is $icode x$$ less than or equal $icode y$$     $rnext
$code >$$   $cnext is $icode x$$ greater than $icode y$$           $rnext
$code >=$$  $cnext is $icode x$$ greater than or equal $icode y$$  $rnext
$code ==$$  $cnext is $icode x$$ equal to $icode y$$               $rnext
$code !=$$  $cnext is $icode x$$ not equal to $icode y$$
$tend

$head x$$
The operand $icode x$$ has prototype
$codei%
    const %Type% &%x%
%$$
where $icode Type$$ is $codei%AD<%Base%>%$$, $icode Base$$, or $code int$$.

$head y$$
The operand $icode y$$ has prototype
$codei%
    const %Type% &%y%
%$$
where $icode Type$$ is $codei%AD<%Base%>%$$, $icode Base$$, or $code int$$.

$head b$$
The result $icode b$$ has type
$codei%
    bool %b%
%$$

$head Operation Sequence$$
The result of this operation is a $code bool$$ value
(not an $cref/AD of Base/glossary/AD of Base/$$ object).
Thus it will not be recorded as part of an
AD of $icode Base$$
$cref/operation sequence/glossary/Operation/Sequence/$$.
$pre

$$
For example, suppose
$icode x$$ and $icode y$$ are $codei%AD<%Base%>%$$ objects,
the tape corresponding to $codei%AD<%Base%>%$$ is recording,
$icode b$$ is true,
and the subsequent code is
$codei%
    if( %b% )
        %y% = cos(%x%);
    else
        %y% = sin(%x%);
%$$
only the assignment $icode%y% = cos(%x%)%$$ is recorded on the tape
(if $icode x$$ is a $cref/parameter/glossary/Parameter/$$,
nothing is recorded).
The $cref CompareChange$$ function can yield
some information about changes in comparison operation results.
You can use $cref CondExp$$ to obtain comparison operations
that depends on the
$cref/independent variable/glossary/Tape/Independent Variable/$$
values with out re-taping the AD sequence of operations.

$head Assumptions$$
If one of the $icode Op$$ operators listed above
is used with an $codei%AD<%Base%>%$$ object,
it is assumed that the same operator is supported by the base type
$icode Base$$.

$head Example$$
$children%
    example/general/compare.cpp
%$$
The file
$cref compare.cpp$$
contains an example and test of these operations.

$end
-------------------------------------------------------------------------------
*/
//  BEGIN CppAD namespace
namespace CppAD {
// -------------------------------- < --------------------------
template <class Base>
CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
bool operator < (const AD<Base> &left , const AD<Base> &right)
{   bool result    =  (left.value_ < right.value_);
    //
    // check if we are recording compare operators
    local::ADTape<Base> *tape = AD<Base>::tape_ptr();
    if( tape == nullptr )
        return result;
    if( ! tape->Rec_.get_record_compare() )
        return result;
    tape_id_t tape_id = tape->id_;
    // tape_id cannot match the default value for tape_id_; i.e., 0
    CPPAD_ASSERT_UNKNOWN( tape_id > 0 );

    // check if left and right tapes match
    bool match_left  = left.tape_id_  == tape_id;
    bool match_right = right.tape_id_ == tape_id;

    // check if left and right are dynamic parameters
    bool dyn_left  = match_left  & (left.ad_type_ == dynamic_enum);
    bool dyn_right = match_right & (right.ad_type_ == dynamic_enum);

    // check if left and right are variables
    bool var_left  = match_left  & (left.ad_type_ != dynamic_enum);
    bool var_right = match_right & (right.ad_type_ != dynamic_enum);

    CPPAD_ASSERT_KNOWN(
        left.tape_id_ == right.tape_id_ || ! match_left || ! match_right ,
        "< : AD variables or dynamic parameters on different threads."
    );
    if( var_left )
    {   if( var_right )
        {   // variable < variable
            if( result )
            {   tape->Rec_.PutOp(local::LtvvOp);
                tape->Rec_.PutArg(left.taddr_, right.taddr_);
            }
            else
            {   tape->Rec_.PutOp(local::LevvOp);
                tape->Rec_.PutArg(right.taddr_, left.taddr_);
            }
        }
        else
        {   // variable < parameter
            addr_t p = right.taddr_;
            if( ! dyn_right )
                p = tape->Rec_.put_con_par(right.value_);
            if( result )
            {   tape->Rec_.PutOp(local::LtvpOp);
                tape->Rec_.PutArg(left.taddr_, p);
            }
            else
            {   tape->Rec_.PutOp(local::LepvOp);
                tape->Rec_.PutArg(p, left.taddr_);
            }
        }
    }
    else if ( var_right )
    {   // parameter < variable
        addr_t p = left.taddr_;
        if( ! dyn_left )
            p = tape->Rec_.put_con_par(left.value_);
        if( result )
        {   tape->Rec_.PutOp(local::LtpvOp);
            tape->Rec_.PutArg(p, right.taddr_);
        }
        else
        {   tape->Rec_.PutOp(local::LevpOp);
            tape->Rec_.PutArg(right.taddr_, p);
        }
    }
    else if( dyn_left | dyn_right )
    {   // parameter < parameter
        addr_t arg0 = left.taddr_;
        addr_t arg1 = right.taddr_;
        if( ! dyn_left )
            arg0 = tape->Rec_.put_con_par(left.value_);
        if( ! dyn_right )
            arg1 = tape->Rec_.put_con_par(right.value_);
        //
        if( result )
        {   tape->Rec_.PutOp(local::LtppOp);
            tape->Rec_.PutArg(arg0, arg1);
        }
        else
        {   tape->Rec_.PutOp(local::LeppOp);
            tape->Rec_.PutArg(arg1, arg0);
        }
    }
    return result;
}
// convert other cases into the case above
CPPAD_FOLD_BOOL_VALUED_BINARY_OPERATOR(<)

// -------------------------------- <= --------------------------
template <class Base>
CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
bool operator <= (const AD<Base> &left , const AD<Base> &right)
{   bool result    =  (left.value_ <= right.value_);
    //
    // check if we are recording compare operators
    local::ADTape<Base> *tape = AD<Base>::tape_ptr();
    if( tape == nullptr )
        return result;
    if( ! tape->Rec_.get_record_compare() )
        return result;
    tape_id_t tape_id = tape->id_;
    // tape_id cannot match the default value for tape_id_; i.e., 0
    CPPAD_ASSERT_UNKNOWN( tape_id > 0 );

    // check if left and right tapes match
    bool match_left  = left.tape_id_  == tape_id;
    bool match_right = right.tape_id_ == tape_id;

    // check if left and right are dynamic parameters
    bool dyn_left  = match_left  & (left.ad_type_ == dynamic_enum);
    bool dyn_right = match_right & (right.ad_type_ == dynamic_enum);

    // check if left and right are variables
    bool var_left  = match_left  & (left.ad_type_ != dynamic_enum);
    bool var_right = match_right & (right.ad_type_ != dynamic_enum);

    CPPAD_ASSERT_KNOWN(
        left.tape_id_ == right.tape_id_ || ! match_left || ! match_right ,
        "<= : AD variables or dynamic parameters on different threads."
    );
    if( var_left )
    {   if( var_right )
        {   // variable <= variable
            if( result )
            {   tape->Rec_.PutOp(local::LevvOp);
                tape->Rec_.PutArg(left.taddr_, right.taddr_);
            }
            else
            {   tape->Rec_.PutOp(local::LtvvOp);
                tape->Rec_.PutArg(right.taddr_, left.taddr_);
            }
        }
        else
        {   // variable <= parameter
            addr_t p = right.taddr_;
            if( ! dyn_right )
                p = tape->Rec_.put_con_par(right.value_);
            if( result )
            {   tape->Rec_.PutOp(local::LevpOp);
                tape->Rec_.PutArg(left.taddr_, p);
            }
            else
            {   tape->Rec_.PutOp(local::LtpvOp);
                tape->Rec_.PutArg(p, left.taddr_);
            }
        }
    }
    else if ( var_right )
    {   // parameter <= variable
        addr_t p = left.taddr_;
        if( ! dyn_left )
            p = tape->Rec_.put_con_par(left.value_);
        if( result )
        {   tape->Rec_.PutOp(local::LepvOp);
            tape->Rec_.PutArg(p, right.taddr_);
        }
        else
        {   tape->Rec_.PutOp(local::LtvpOp);
            tape->Rec_.PutArg(right.taddr_, p);
        }
    }
    else if( dyn_left | dyn_right )
    {   // parameter <= parameter
        addr_t arg0 = left.taddr_;
        addr_t arg1 = right.taddr_;
        if( ! dyn_left )
            arg0 = tape->Rec_.put_con_par(left.value_);
        if( ! dyn_right )
            arg1 = tape->Rec_.put_con_par(right.value_);
        //
        if( result )
        {   tape->Rec_.PutOp(local::LeppOp);
            tape->Rec_.PutArg(arg0, arg1);
        }
        else
        {   tape->Rec_.PutOp(local::LtppOp);
            tape->Rec_.PutArg(arg1, arg0);
        }
    }
    return result;
}
// convert other cases into the case above
CPPAD_FOLD_BOOL_VALUED_BINARY_OPERATOR(<=)

// -------------------------------- > --------------------------
template <class Base>
CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
bool operator > (const AD<Base> &left , const AD<Base> &right)
{   bool result    =  (left.value_ > right.value_);
    //
    // check if we are recording compare operators
    local::ADTape<Base> *tape = AD<Base>::tape_ptr();
    if( tape == nullptr )
        return result;
    if( ! tape->Rec_.get_record_compare() )
        return result;
    tape_id_t tape_id = tape->id_;
    // tape_id cannot match the default value for tape_id_; i.e., 0
    CPPAD_ASSERT_UNKNOWN( tape_id > 0 );

    // check if left and right tapes match
    bool match_left  = left.tape_id_  == tape_id;
    bool match_right = right.tape_id_ == tape_id;

    // check if left and right are dynamic parameters
    bool dyn_left  = match_left  & (left.ad_type_ == dynamic_enum);
    bool dyn_right = match_right & (right.ad_type_ == dynamic_enum);

    // check if left and right are variables
    bool var_left  = match_left  & (left.ad_type_ != dynamic_enum);
    bool var_right = match_right & (right.ad_type_ != dynamic_enum);

    CPPAD_ASSERT_KNOWN(
        left.tape_id_ == right.tape_id_ || ! match_left || ! match_right ,
        "> : AD variables or dynamic parameters on different threads."
    );
    if( var_left )
    {   if( var_right )
        {   // variable > variable
            if( result )
            {   tape->Rec_.PutOp(local::LtvvOp);
                tape->Rec_.PutArg(right.taddr_, left.taddr_);
            }
            else
            {   tape->Rec_.PutOp(local::LevvOp);
                tape->Rec_.PutArg(left.taddr_, right.taddr_);
            }
        }
        else
        {   // variable > parameter
            addr_t p = right.taddr_;
            if( ! dyn_right )
                p = tape->Rec_.put_con_par(right.value_);
            if( result )
            {   tape->Rec_.PutOp(local::LtpvOp);
                tape->Rec_.PutArg(p, left.taddr_);
            }
            else
            {   tape->Rec_.PutOp(local::LevpOp);
                tape->Rec_.PutArg(left.taddr_, p);
            }
        }
    }
    else if ( var_right )
    {   // parameter > variable
        addr_t p = left.taddr_;
        if( ! dyn_left )
            p = tape->Rec_.put_con_par(left.value_);
        if( result )
        {   tape->Rec_.PutOp(local::LtvpOp);
            tape->Rec_.PutArg(right.taddr_, p);
        }
        else
        {   tape->Rec_.PutOp(local::LepvOp);
            tape->Rec_.PutArg(p, right.taddr_);
        }
    }
    else if( dyn_left | dyn_right )
    {   // parameter > parameter
        addr_t arg0 = left.taddr_;
        addr_t arg1 = right.taddr_;
        if( ! dyn_left )
            arg0 = tape->Rec_.put_con_par(left.value_);
        if( ! dyn_right )
            arg1 = tape->Rec_.put_con_par(right.value_);
        //
        if( result )
        {   tape->Rec_.PutOp(local::LtppOp);
            tape->Rec_.PutArg(arg1, arg0);
        }
        else
        {   tape->Rec_.PutOp(local::LeppOp);
            tape->Rec_.PutArg(arg0, arg1);
        }
    }
    return result;
}
// convert other cases into the case above
CPPAD_FOLD_BOOL_VALUED_BINARY_OPERATOR(>)

// -------------------------------- >= --------------------------
template <class Base>
CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
bool operator >= (const AD<Base> &left , const AD<Base> &right)
{   bool result    =  (left.value_ >= right.value_);
    //
    // check if we are recording compare operators
    local::ADTape<Base> *tape = AD<Base>::tape_ptr();
    if( tape == nullptr )
        return result;
    if( ! tape->Rec_.get_record_compare() )
        return result;
    tape_id_t tape_id = tape->id_;
    // tape_id cannot match the default value for tape_id_; i.e., 0
    CPPAD_ASSERT_UNKNOWN( tape_id > 0 );

    // check if left and right tapes match
    bool match_left  = left.tape_id_  == tape_id;
    bool match_right = right.tape_id_ == tape_id;

    // check if left and right are dynamic parameters
    bool dyn_left  = match_left  & (left.ad_type_ == dynamic_enum);
    bool dyn_right = match_right & (right.ad_type_ == dynamic_enum);

    // check if left and right are variables
    bool var_left  = match_left  & (left.ad_type_ != dynamic_enum);
    bool var_right = match_right & (right.ad_type_ != dynamic_enum);

    CPPAD_ASSERT_KNOWN(
        left.tape_id_ == right.tape_id_ || ! match_left || ! match_right ,
        ">= : AD variables or dynamic parameters on different threads."
    );
    if( var_left )
    {   if( var_right )
        {   // variable >= variable
            if( result )
            {   tape->Rec_.PutOp(local::LevvOp);
                tape->Rec_.PutArg(right.taddr_, left.taddr_);
            }
            else
            {   tape->Rec_.PutOp(local::LtvvOp);
                tape->Rec_.PutArg(left.taddr_, right.taddr_);
            }
        }
        else
        {   // variable >= parameter
            addr_t p = right.taddr_;
            if( ! dyn_right )
                p = tape->Rec_.put_con_par(right.value_);
            if( result )
            {   tape->Rec_.PutOp(local::LepvOp);
                tape->Rec_.PutArg(p, left.taddr_);
            }
            else
            {   tape->Rec_.PutOp(local::LtvpOp);
                tape->Rec_.PutArg(left.taddr_, p);
            }
        }
    }
    else if ( var_right )
    {   // parameter >= variable
        addr_t p = left.taddr_;
        if( ! dyn_left )
            p = tape->Rec_.put_con_par(left.value_);
        if( result )
        {   tape->Rec_.PutOp(local::LevpOp);
            tape->Rec_.PutArg(right.taddr_, p);
        }
        else
        {   tape->Rec_.PutOp(local::LtpvOp);
            tape->Rec_.PutArg(p, right.taddr_);
        }
    }
    else if( dyn_left | dyn_right )
    {   // parameter >= parameter
        addr_t arg0 = left.taddr_;
        addr_t arg1 = right.taddr_;
        if( ! dyn_left )
            arg0 = tape->Rec_.put_con_par(left.value_);
        if( ! dyn_right )
            arg1 = tape->Rec_.put_con_par(right.value_);
        //
        if( result )
        {   tape->Rec_.PutOp(local::LeppOp);
            tape->Rec_.PutArg(arg1, arg0);
        }
        else
        {   tape->Rec_.PutOp(local::LtppOp);
            tape->Rec_.PutArg(arg0, arg1);
        }
    }
    return result;
}
// convert other cases into the case above
CPPAD_FOLD_BOOL_VALUED_BINARY_OPERATOR(>=)

// -------------------------------- == -------------------------
template <class Base>
CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
bool operator == (const AD<Base> &left , const AD<Base> &right)
{   bool result    =  (left.value_ == right.value_);
    //
    // check if we are recording compare operators
    local::ADTape<Base> *tape = AD<Base>::tape_ptr();
    if( tape == nullptr )
        return result;
    if( ! tape->Rec_.get_record_compare() )
        return result;
    tape_id_t tape_id = tape->id_;
    // tape_id cannot match the default value for tape_id_; i.e., 0
    CPPAD_ASSERT_UNKNOWN( tape_id > 0 );

    // check if left and right tapes match
    bool match_left  = left.tape_id_  == tape_id;
    bool match_right = right.tape_id_ == tape_id;

    // check if left and right are dynamic parameters
    bool dyn_left  = match_left  & (left.ad_type_ == dynamic_enum);
    bool dyn_right = match_right & (right.ad_type_ == dynamic_enum);

    // check if left and right are variables
    bool var_left  = match_left  & (left.ad_type_ != dynamic_enum);
    bool var_right = match_right & (right.ad_type_ != dynamic_enum);

    CPPAD_ASSERT_KNOWN(
        left.tape_id_ == right.tape_id_ || ! match_left || ! match_right ,
        "==: AD variables or dynamic parameters on different threads."
    );
    //
    tape->Rec_.comp_eq(
        var_left, var_right, dyn_left, dyn_right, left, right, result
    );
    //
    return result;
}
// convert other cases into the case above
CPPAD_FOLD_BOOL_VALUED_BINARY_OPERATOR(==)

// -------------------------------- != -------------------------
template <class Base>
CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
bool operator != (const AD<Base> &left , const AD<Base> &right)
{   bool result    =  (left.value_ != right.value_);
    //
    // check if we are recording compare operators
    local::ADTape<Base> *tape = AD<Base>::tape_ptr();
    if( tape == nullptr )
        return result;
    if( ! tape->Rec_.get_record_compare() )
        return result;
    tape_id_t tape_id = tape->id_;
    // tape_id cannot match the default value for tape_id_; i.e., 0
    CPPAD_ASSERT_UNKNOWN( tape_id > 0 );

    // check if left and right tapes match
    bool match_left  = left.tape_id_  == tape_id;
    bool match_right = right.tape_id_ == tape_id;

    // check if left and right are dynamic parameters
    bool dyn_left  = match_left  & (left.ad_type_ == dynamic_enum);
    bool dyn_right = match_right & (right.ad_type_ == dynamic_enum);

    // check if left and right are variables
    bool var_left  = match_left  & (left.ad_type_ != dynamic_enum);
    bool var_right = match_right & (right.ad_type_ != dynamic_enum);

    CPPAD_ASSERT_KNOWN(
        left.tape_id_ == right.tape_id_ || ! match_left || ! match_right ,
        "!=: AD variables or dynamic parameters on different threads."
    );
    if( var_left )
    {   if( var_right )
        {   // variable == variable
            tape->Rec_.PutArg(left.taddr_, right.taddr_);
            if( result )
                tape->Rec_.PutOp(local::NevvOp);
            else
                tape->Rec_.PutOp(local::EqvvOp);
        }
        else
        {   // variable == parameter
            addr_t p = right.taddr_;
            if( ! dyn_right )
                p = tape->Rec_.put_con_par(right.value_);
            tape->Rec_.PutArg(p, left.taddr_);
            if( result )
                tape->Rec_.PutOp(local::NepvOp);
            else
                tape->Rec_.PutOp(local::EqpvOp);
        }
    }
    else if ( var_right )
    {   // parameter == variable
        addr_t p = left.taddr_;
        if( ! dyn_left )
            p = tape->Rec_.put_con_par(left.value_);
        tape->Rec_.PutArg(p, right.taddr_);
        if( result )
            tape->Rec_.PutOp(local::NepvOp);
        else
            tape->Rec_.PutOp(local::EqpvOp);
    }
    else if( dyn_left | dyn_right )
    {   // parameter == parameter
        addr_t arg0 = left.taddr_;
        addr_t arg1 = right.taddr_;
        if( ! dyn_left )
            arg0 = tape->Rec_.put_con_par(left.value_);
        if( ! dyn_right )
            arg1 = tape->Rec_.put_con_par(right.value_);
        //
        tape->Rec_.PutArg(arg0, arg1);
        if( result )
            tape->Rec_.PutOp(local::NeppOp);
        else
            tape->Rec_.PutOp(local::EqppOp);
    }
    return result;
}
// convert other cases into the case above
CPPAD_FOLD_BOOL_VALUED_BINARY_OPERATOR(!=)

} // END CppAD namespace

# endif
