# ifndef CPPAD_LOCAL_OP_CODE_DYN_HPP
# define CPPAD_LOCAL_OP_CODE_DYN_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
$begin op_code_dyn$$
$spell
    vec
    Op
    dyn
    arg
    hpp
    cond_exp
    ind
    zmul
    Namespace
    enum
    CppAD
$$

$section Dynamic Parameter Op Codes$$

$head Namespace$$
The $code op_code_dyn$$ enum type is in the $code CppAD::local$$ namespace.

$head AD Type$$
All the operators below have no variable arguments,
at least one dynamic parameter argument,
and at most one constant argument; see
$cref ad_type_enum$$.
For example, all the unary operators have one dynamic parameter argument
and one dynamic parameter result.

$head Unary$$
The number of arguments for a unary operator is one
and it is a parameter index.
All the unary operators have one result that is a dynamic parameter.

$head Binary$$
The number of arguments for a binary operator is two
and they are parameter indices.
All the binary operators have one result that is a dynamic parameter.
For binary operators the first argument is the left operand
and the second is the right operand.

$subhead zmul_dyn$$
This binary operator has a non-standard name; see $cref azmul$$ for
its definition.

$head ind_dyn$$
This is an independent dynamic parameter operator.
It has no arguments and one result which is the value of the corresponding
independent dynamic parameter in the call to $cref new_dynamic$$.

$comment ----------------------------------------------------------------- $$
$head atom_dyn$$
This operator is a call to an atomic function.
The number of arguments to this operator is
$icode%arg%[4+%n%+%m%]%$$; see below.

$subhead arg[0]$$
This is the index that identifies this atomic function; see
$code local/atomic_index.hpp$$.

$subhead arg[1]$$
This is the number of arguments to this atomic function.
We use the notation $icode%n% = %arg%[1]%$$ below.

$subhead arg[2]$$
This is the number of results for this atomic function.
We use the notation $icode%m% = %arg%[2]%$$ below.

$subhead arg[3]$$
This is the number of result values that are dynamic parameters
for this function call.

$subhead arg[4+j]$$
For $icode%j% = 0 , %...% , %n%-1%$$,
this is the parameter index for the $th j$$ argument to this atomic
function call.

$subhead arg[4+n+i]$$
For $icode%i% = 0 , %...% , %m%-1%$$,
this is the parameter index for the $th i$$ result to this atomic
function call.

$subhead arg[4+n+m]$$
This is the number of arguments to this operator; i.e.,
$codei%5+%n%+%m%$$.

$head result_dyn$$
This is a place holder for a result of an atomic function call
that is a dynamic parameter.
It has no arguments, no results, and is only there so that the
number of dynamic parameters and the number of dynamic operators are equal.

$comment ----------------------------------------------------------------- $$
$head cond_exp_dyn$$
This is a conditional expression operator and has five arguments
and one result.

$subhead arg[0]$$
This is the
$cref/CompareOp/base_cond_exp/CompareOp/$$ value for this operator.

$subhead arg[1]$$
This is the parameter index for the left operand to the comparison.

$subhead arg[2]$$
This is the parameter index for the right operand to the comparison.

$subhead arg[3]$$
This is the index of the parameter equal to the operator result if
the comparison result is true.

$subhead arg[4]$$
This is the index of the parameter equal to the operator result if
the comparison result is false.

$comment ----------------------------------------------------------------- $$
$head dis_dyn$$
This is a call to a discrete function.
The discrete function has one argument and one result.
This operator has two arguments and one result.
It is not a binary operator because the first argument
is not the index of a parameter.

$subhead arg[0]$$
Is the discrete function index which depends on the $icode Base$$
type used when this function was recorded.

$subhead arg[1]$$
Is the parameter index for the argument to the function.

$comment ----------------------------------------------------------------- $$
$head Source$$
$srcthisfile%
    0%// BEGIN_OP_CODE_DYN%// END_OP_CODE_DYN%1
%$$
$end
*/

// BEGIN_SORT_THIS_LINE_PLUS_3
// BEGIN_OP_CODE_DYN
enum op_code_dyn {
    abs_dyn,       // unary
    acos_dyn,      // unary
    acosh_dyn,     // unary
    add_dyn,       // binary
    asin_dyn,      // unary
    asinh_dyn,     // unary
    atan_dyn,      // unary
    atanh_dyn,     // unary
    atom_dyn,      // ? arguments: atomic function call
    cond_exp_dyn,  // 5 arguments: conditional expression
    cos_dyn,       // unary
    cosh_dyn,      // unary
    dis_dyn,       // 2 arguments: discrete function
    div_dyn,       // binary
    erfc_dyn,      // unary
    erf_dyn,       // unary
    exp_dyn,       // unary
    expm1_dyn,     // unary
    fabs_dyn,      // unary
    ind_dyn,       // 0 arguments: independent parameter
    log1p_dyn,     // unary
    log_dyn,       // unary
    mul_dyn,       // binary
    neg_dyn,       // unary
    pow_dyn,       // binary
    result_dyn,    // 0 arguments: atomic function result
    sign_dyn,      // unary
    sin_dyn,       // unary
    sinh_dyn,      // unary
    sqrt_dyn,      // unary
    sub_dyn,       // binary
    tan_dyn,       // unary
    tanh_dyn,      // unary
    zmul_dyn,      // binary
    number_dyn     // number of operator codes and invalid operator value
};
// END_OP_CODE_DYN
// END_SORT_THIS_LINE_MINUS_4

/*
$begin num_arg_dyn$$
$spell
    num_arg_dyn
    op
    enum
$$

$section Number of Arguments to a Dynamic Parameter Operator$$

$head Syntax$$
$icode%n_arg% = local::num_arg_dyn(%op%)
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_NUM_ARG_DYN_PROTOTYPE%// END_NUM_ARG_DYN_PROTOTYPE%1
%$$

$head Parallel Mode$$
This routine has static data so its first call cannot be in Parallel mode.

$head op$$
is the operator in question.

$head n_arg$$
The return value is the number of arguments as commented in the
$cref/source/op_code_dyn/Source/$$ for $code enum op_code_dyn$$.
There is one exception: if $icode op$$ is $code atom_dyn$$,
$icode n_arg$$ is zero; see $cref/atom_dyn/op_code_dyn/atom_dyn/$$
for the true number of arguments in this case.

$head atom_dyn$$
All of the dynamic parameter operators have a fixed number of arguments
except for the $cref/atom_dyn/op_code_dyn/atom_dyn/$$
operator which calls an atomic functions.
In this special case the return value $icode n_arg$$ is zero
which is not correct.

$end
*/
// BEGIN_NUM_ARG_DYN_PROTOTYPE
inline size_t num_arg_dyn(op_code_dyn op)
// END_NUM_ARG_DYN_PROTOTYPE
{   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;

    // BEGIN_SORT_THIS_LINE_PLUS_2
    static const size_t num_arg_table[] = {
        /* abs_dyn */      1,
        /* acos_dyn */     1,
        /* acosh_dyn */    1,
        /* add_dyn */      2,
        /* asin_dyn */     1,
        /* asinh_dyn */    1,
        /* atan_dyn */     1,
        /* atanh_dyn */    1,
        /* atom_dyn */     0,
        /* cond_exp_dyn */ 5,
        /* cos_dyn */      1,
        /* cosh_dyn */     1,
        /* dis_dyn */      2,
        /* div_dyn */      2,
        /* erfc_dyn */     1,
        /* erf_dyn */      1,
        /* exp_dyn */      1,
        /* expm1_dyn */    1,
        /* fabs_dyn */     1,
        /* ind_dyn */      0,
        /* log1p_dyn */    1,
        /* log_dyn */      1,
        /* mul_dyn */      2,
        /* neg_dyn */      1,
        /* pow_dyn */      2,
        /* result_dyn */   0,
        /* sign_dyn */     1,
        /* sin_dyn */      1,
        /* sinh_dyn */     1,
        /* sqrt_dyn */     1,
        /* sub_dyn */      2,
        /* tan_dyn */      1,
        /* tanh_dyn */     1,
        /* zmul_dyn */     2,
        0  // number_dyn (not used)
    };
    // END_SORT_THIS_LINE_MINUS_3
    //
    static bool first = true;
    if( first )
    {   CPPAD_ASSERT_UNKNOWN(
        size_t(number_dyn)+1 == sizeof(num_arg_table)/sizeof(num_arg_table[0])
        );
        first = false;
    }
    return num_arg_table[op];
}

/*
$begin op_name_dyn$$
$spell
    dyn
    op
    enum
    cond_exp
$$

$section Number of Arguments to a Dynamic Parameter Operator$$

$head Syntax$$
$icode%name% = local::op_name_dyn(%op%)
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_OP_NAME_DYN_PROTOTYPE%// END_OP_NAME_DYN_PROTOTYPE%1
%$$

$head Parallel Mode$$
This routine has static data so its first call cannot be in Parallel mode.

$head op$$
is the operator in question.

$head name$$
The return value $icode name$$ is the same as the operator enum symbol
(see $cref/source/op_code_dyn/Source/$$ for $code enum op_code_dyn$$)
without the $code _dyn$$ at the end. For example,
the name corresponding to the
$cref/cond_exp_dyn/op_code_dyn/cond_exp_dyn/$$ operator is $code cond_exp$$.

$end
*/
// BEGIN_OP_NAME_DYN_PROTOTYPE
inline const char* op_name_dyn(op_code_dyn op)
// END_OP_NAME_DYN_PROTOTYPE
{   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;

    // BEGIN_SORT_THIS_LINE_PLUS_2
    static const char* op_name_table[] = {
        /* abs_dyn */      "abs",
        /* acos_dyn */     "acos",
        /* acosh_dyn */    "acosh",
        /* add_dyn */      "add",
        /* asin_dyn */     "asin",
        /* asinh_dyn */    "asinh",
        /* atan_dyn */     "atan",
        /* atanh_dyn */    "atanh",
        /* atom_dyn */     "call",
        /* cond_exp_dyn */ "cond_exp",
        /* cos_dyn */      "cos",
        /* cosh_dyn */     "cosh",
        /* dis_dyn */      "dis",
        /* div_dyn */      "div",
        /* erfc_dyn */     "erfc",
        /* erf_dyn */      "erf",
        /* exp_dyn */      "exp",
        /* expm1_dyn */    "expm1",
        /* fabs_dyn */     "fabs",
        /* ind_dyn */      "ind",
        /* log1p_dyn */    "log1p",
        /* log_dyn */      "log",
        /* mul_dyn */      "mul",
        /* neg_dyn */      "neg",
        /* pow_dyn */      "pow",
        /* result_dyn */   "result",
        /* sign_dyn */     "sign",
        /* sin_dyn */      "sin",
        /* sinh_dyn */     "sinh",
        /* sqrt_dyn */     "sqrt",
        /* sub_dyn */      "sub",
        /* tan_dyn */      "tan",
        /* tanh_dyn */     "tanh",
        /* zmul_dyn */     "zmul",
        /* number_dyn */   "number"
    };
    // END_SORT_THIS_LINE_MINUS_3
    static bool first = true;
    if( first )
    {   CPPAD_ASSERT_UNKNOWN(
        size_t(number_dyn)+1 == sizeof(op_name_table)/sizeof(op_name_table[0])
        );
        first = false;
    }
    return op_name_table[op];
}

/*
$begin num_non_par_arg_dyn$$
$spell
    arg
    dyn
    op
    num
$$

$section Number Non-Parameter Arguments to a Dynamic Parameters Operator$$

$head Syntax$$
$icode%num% = local::num_non_par_arg_dyn(%op%)
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_NUM_NON_PAR_ARG_DYN%// END_NUM_NON_PAR_ARG_DYN%1
%$$

$head op$$
is the operator in question.

$head num$$
The return value $icode num$$ is the number of arguments,
for this operator $icode op$$, that are not parameters indices.
All of the non-parameter arguments come first
so $icode num$$ is also the offset for the
first argument that is a parameter index.

$head atom_dyn$$
The $cref/atom_dyn/op_code_dyn/atom_dyn/$$ case is special,
$icode num$$ is zero for this case but it is not as documented above; see
$cref/atom_dyn/op_code_dyn/atom_dyn/$$.

$end
*/
// BEGIN_NUM_NON_PAR_ARG_DYN
inline size_t num_non_par_arg_dyn(op_code_dyn op)
// END_NUM_NON_PAR_ARG_DYN
{
    size_t num;
    switch(op)
    {   case atom_dyn:
        num = 4;
        break;

        case cond_exp_dyn:
        case dis_dyn:
        num = 1;
        break;

        default:
        num = 0;
    }
    //
    return num;
}

} } // END_CPPAD_LOCAL_NAMESPACE

# endif
