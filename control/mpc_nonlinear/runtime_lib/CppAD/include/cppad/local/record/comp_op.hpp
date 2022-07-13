# ifndef CPPAD_LOCAL_RECORD_COMP_OP_HPP
# define CPPAD_LOCAL_RECORD_COMP_OP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/record/recorder.hpp>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*
$begin recorder_put_comp_op$$
$spell
    rel
    aleft
    eq
    le
    lt
    var
    dyn
    taddr
$$

$section Put Compare Operators in Recording$$

$head Syntax$$
$icode%rec%.put_comp_%rel%( %aleft%, %aright%, %result%)
%$$

$subhead rel$$
The text $icode rel$$ in the function name above
is $code eq$$ (for equals),
$code le$$ (for less than or equal), or
$code lt$$ (for less than).

$head Prototype$$
$srcthisfile%
    0%// BEGIN_COMP_EQ%// END_COMP_EQ%1
%$$
The other prototypes for the functions $code comp_le$$ and $code comp_lt$$
are same except for the function name.

$head var_left$$
is true if the left operand is a variable.

$head var_right$$
is true if the right operand is a variable.

$head dyn_left$$
is true if the left operand is a dynamic parameter.

$head dyn_right$$
is true if the right operand is a dynamic parameter.

$head aleft$$
is the compare operator left operand.

$head aright$$
is the compare operator right operand.

$head taddr_$$
The values $icode%aleft.taddr_%$$ and $icode%aright%.taddr_%$$
are the proper address for dynamic parameters and variables
and does not matter for constants.

$head value_$$
The values $icode%aleft.value_%$$ and $icode%aright%.value_%$$
are the proper address for constants and does not matter
for variables and dynamic parameters.

$head result$$
This is the result for this comparison corresponding to this
recording (sequence of operations).

$end
*/
// BEGIN_COMP_EQ
template <class Base>
void recorder<Base>::comp_eq(
    bool                        var_left     ,
    bool                        var_right    ,
    bool                        dyn_left     ,
    bool                        dyn_right    ,
    const AD<Base>&             aleft        ,
    const AD<Base>&             aright       ,
    bool                        result       )
// END_COMP_EQ
{   if( var_left )
    {   if( var_right )
        {   // variable == variable
            PutArg(aleft.taddr_, aright.taddr_);
            if( result )
                PutOp(EqvvOp);
            else
                PutOp(NevvOp);
        }
        else
        {   // variable == parameter
            addr_t p = aright.taddr_;
            if( ! dyn_right )
                p = put_con_par(aright.value_);
            PutArg(p, aleft.taddr_);
            if( result )
                PutOp(EqpvOp);
            else
                PutOp(NepvOp);
        }
    }
    else if ( var_right )
    {   // parameter == variable
        addr_t p = aleft.taddr_;
        if( ! dyn_left )
            p = put_con_par(aleft.value_);
        PutArg(p, aright.taddr_);
        if( result )
            PutOp(EqpvOp);
        else
            PutOp(NepvOp);
    }
    else if( dyn_left | dyn_right )
    {   // parameter == parameter
        addr_t arg0 = aleft.taddr_;
        addr_t arg1 = aright.taddr_;
        if( ! dyn_left )
            arg0 = put_con_par(aleft.value_);
        if( ! dyn_right )
            arg1 = put_con_par(aright.value_);
        //
        PutArg(arg0, arg1);
        if( result )
            PutOp(EqppOp);
        else
            PutOp(NeppOp);
    }
}
// ---------------------------------------------------------------------------
// comp_le
template <class Base>
void recorder<Base>::comp_le(
    bool                        var_left     ,
    bool                        var_right    ,
    bool                        dyn_left     ,
    bool                        dyn_right    ,
    const AD<Base>&             aleft        ,
    const AD<Base>&             aright       ,
    bool                        result       )
{
    if( var_left )
    {   if( var_right )
        {   // variable <= variable
            if( result )
            {   PutOp(LevvOp);
                PutArg(aleft.taddr_, aright.taddr_);
            }
            else
            {   PutOp(LtvvOp);
                PutArg(aright.taddr_, aleft.taddr_);
            }
        }
        else
        {   // variable <= parameter
            addr_t p = aright.taddr_;
            if( ! dyn_right )
                p = put_con_par(aright.value_);
            if( result )
            {   PutOp(LevpOp);
                PutArg(aleft.taddr_, p);
            }
            else
            {   PutOp(LtpvOp);
                PutArg(p, aleft.taddr_);
            }
        }
    }
    else if ( var_right )
    {   // parameter <= variable
        addr_t p = aleft.taddr_;
        if( ! dyn_left )
            p = put_con_par(aleft.value_);
        if( result )
        {   PutOp(LepvOp);
            PutArg(p, aright.taddr_);
        }
        else
        {   PutOp(LtvpOp);
            PutArg(aright.taddr_, p);
        }
    }
    else if( dyn_left | dyn_right )
    {   // parameter <= parameter
        addr_t arg0 = aleft.taddr_;
        addr_t arg1 = aright.taddr_;
        if( ! dyn_left )
            arg0 = put_con_par(aleft.value_);
        if( ! dyn_right )
            arg1 = put_con_par(aright.value_);
        //
        if( result )
        {   PutOp(LeppOp);
            PutArg(arg0, arg1);
        }
        else
        {   PutOp(LtppOp);
            PutArg(arg1, arg0);
        }
    }
}
// --------------------------------------------------------------------------
// comp_lt
template <class Base>
void recorder<Base>::comp_lt(
    bool                        var_left     ,
    bool                        var_right    ,
    bool                        dyn_left     ,
    bool                        dyn_right    ,
    const AD<Base>&             aleft        ,
    const AD<Base>&             aright       ,
    bool                        result       )
{
    if( var_left )
    {   if( var_right )
        {   // variable < variable
            if( result )
            {   PutOp(LtvvOp);
                PutArg(aleft.taddr_, aright.taddr_);
            }
            else
            {   PutOp(LevvOp);
                PutArg(aright.taddr_, aleft.taddr_);
            }
        }
        else
        {   // variable < parameter
            addr_t p = aright.taddr_;
            if( ! dyn_right )
                p = put_con_par(aright.value_);
            if( result )
            {   PutOp(LtvpOp);
                PutArg(aleft.taddr_, p);
            }
            else
            {   PutOp(LepvOp);
                PutArg(p, aleft.taddr_);
            }
        }
    }
    else if ( var_right )
    {   // parameter < variable
        addr_t p = aleft.taddr_;
        if( ! dyn_left )
            p = put_con_par(aleft.value_);
        if( result )
        {   PutOp(LtpvOp);
            PutArg(p, aright.taddr_);
        }
        else
        {   PutOp(LevpOp);
            PutArg(aright.taddr_, p);
        }
    }
    else if( dyn_left | dyn_right )
    {   // parameter < parameter
        addr_t arg0 = aleft.taddr_;
        addr_t arg1 = aright.taddr_;
        if( ! dyn_left )
            arg0 = put_con_par(aleft.value_);
        if( ! dyn_right )
            arg1 = put_con_par(aright.value_);
        //
        if( result )
        {   PutOp(LtppOp);
            PutArg(arg0, arg1);
        }
        else
        {   PutOp(LeppOp);
            PutArg(arg1, arg0);
        }
    }
}
} } // END_CPPAD_LOCAL_NAMESPACE
# endif
