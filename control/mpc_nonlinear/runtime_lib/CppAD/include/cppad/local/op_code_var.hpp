# ifndef CPPAD_LOCAL_OP_CODE_VAR_HPP
# define CPPAD_LOCAL_OP_CODE_VAR_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <string>
# include <sstream>
# include <iomanip>

# include <cppad/local/atomic_index.hpp>
# include <cppad/local/define.hpp>
# include <cppad/core/cppad_assert.hpp>
# include <cppad/local/pod_vector.hpp>

// needed before one can use CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL
# include <cppad/utility/thread_alloc.hpp>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
$begin op_code_var$$
$spell
    ind
    pos
    Pri
    Ldp
    Ldv
    Vec
    Stpp
    Stvp
    Stpv
    Stvv
    initializes
    Cond
    Rel
    Namespace
    CppAD
    Op
    opcode
    enum
    arg
    addr
    pv
    vp
    vv
    AAddpv
    exp
    Funap
    Funav
    Funrp
    Funrv
    Powpv
    Powvv
$$

$head Namespace$$
All of these definitions are in the $code CppAD::local$$ namespace.

$section Variable Op Codes$$

$head opcode_t$$
This type is used to save space when storing operator enum type in vectors.
$srccode%hpp% */
typedef CPPAD_VEC_ENUM_TYPE opcode_t;
/* %$$

$head OpCode$$
This enum type is used to distinguish different $codei%AD<%Base%>%$$
atomic operations.
Each value in the enum type ends with the characters $code Op$$.
Ignoring the $code Op$$ at the end,
the operators appear in alphabetical order.

$head arg[i]$$
We use the notation $icode%arg[%i%]%$$ below
for the $th i$$ operator argument which is a position integer
represented using the type $code addr_t$$.

$head Unary$$
An operator commented as unary below
has one argument (arg[0]) and it is a variable index.
All of these operators have one result variable.

$head Binary And Compare$$
An operator commented as binary or compare below
has two arguments.
If it is a compare operator it has no result variables
(the result is true or false but not a variable).
Otherwise, it has one result variable.
These operators use the following convention for the operator ending
and the left argument (arg[0]) and right argument (arg[1]):
$table
$icode Ending$$ $pre  $$ $cnext $icode Left$$ $cnext  $icode Right$$ $rnext
$code pvOp$$ $cnext    parameter index  $cnext   variable index   $rnext
$code vpOp$$ $cnext    variable index   $cnext   parameter index  $rnext
$code vvOp$$ $cnext    variable index   $cnext   variable index
$tend
For example, $code AddpvOp$$ represents the addition operator where the left
operand is a parameter and the right operand is a variable.

$subhead Pow$$
The binary $codei%pow(%x%, %y%)%$$ operators PowpvOp, PowvvOp are
special because they have three variable results instead of one.
To be specific, they compute
$codei%log(%x%)%$$,
$codei%log(%x%) * %y%$$,
$codei%exp( log(%x%) * %y%)%$$

$comment ------------------------------------------------------------------ $$
$head AFunOp$$
This operator appears at the start and end of every atomic function call.
This operator has no results variables.

$subhead arg[0]$$
This is the $cref atomic_index$$ for this function.

$subhead arg[1]$$
This is the $cref/id/atomic_one/id/$$ information used by an
old atomic class that has been deprecated

$subhead arg[2]$$
is the number of arguments to this atomic function.
We use the notation $icode%n% = %arg%[2]%$$ below.

$subhead arg[3]$$
is the number of results for this atomic function.
We use the notation $icode%m% = %arg%[3]%$$ below.

$subhead Arguments$$
There are $icode n$$ operators after the first $code AFunOp$$,
one for each argument.
If the $th j$$ argument is a parameter (variable)
the corresponding operator is $code FunapOp$$ ( $code FunavOp$$ ), and
the corresponding operator argument is a parameter index (variable index).
These operators have no result variables.

$subhead Results$$
There are $icode m$$ operators after the last argument operator
one for each result.
If the $th i$$ result is a parameter (variable)
the corresponding operator is $code FunrpOp$$ ( $code FunrvOp$$ ).
In the parameter case, there is one argument and it is the parameter index,
and not result variables.
In the variable case, there are no arguments and one result variable.
The index for the new variable with the next possible index.

$comment ------------------------------------------------------------------ $$
$head BeginOp$$
This operator marks the start of the tape.
It has one parameter index argument that is nan and corresponds
to parameter index zero.
It also has one variable result that has index zero which is used to
indicate that a value is not a variable.
for indicate an parameter.

$comment ------------------------------------------------------------------ $$
$head CExpOp$$
This is a $cref/conditional expression/condexp/$$; i.e., the corresponding
source code is
$codei%
    %result% = CondExp%Rel%(%left%, %right%, %if_true%, %if_false%
%$$
This operator has one variable result.

$subhead arg[0]$$
This is a $cref/CompareOp/base_cond_exp/CompareOp/$$ value corresponding
to $cref/Rel/condexp/Rel/$$ above.  ($icode%Rel% = Ne%$$ is not possible).

$subhead arg[1]$$
The first four bits of this integer are used as flags; see below.

$subhead arg[2]$$
If arg[1] & 1 is true (false),
this is the variable index (parameter index) corresponding to $icode left$$.

$subhead arg[3]$$
If arg[1] & 2 is true (false),
this is the variable index (parameter index) corresponding to $icode right$$.

$subhead arg[4]$$
If arg[1] & 4 is true (false),
this is the variable index (parameter index) corresponding to $icode if_true$$.

$subhead arg[5]$$
If arg[1] & 8 is true (false),
this is the variable index (parameter index) corresponding to $icode if_false$$.

$comment ------------------------------------------------------------------ $$
$head CSkipOp$$
The conditional skip operator (used to skip operations that depend on false
branches to conditional expressions).
This operator has not result variables.

$subhead arg[0]$$
This is a $cref/CompareOp/base_cond_exp/CompareOp/$$ value corresponding
to this conditional skip.

$subhead arg[1]$$
The first two bits of this integer are used as flags; see below.

$subhead arg[2]$$
If arg[1] & 1 is true (false),
this is the variable index (parameter index) corresponding to $icode left$$.

$subhead arg[3]$$
If arg[1] & 2 is true (false),
this is the variable index (parameter index) corresponding to $icode right$$.

$subhead arg[4]$$
is the number of operations to skip if the comparison is true.
We use the notation $icode%n% = %arg%[4]%$$ below.

$subhead arg[5]$$
is the number of operations to skip if the comparison is false.
We use the notation $icode%m% = %arg%[5]%$$ below.

$subhead arg[6+i]$$
For $icode%i% = 0, %...%, %n%-1%$$, this is the index
of an operator that can be skipped if the comparison is true.

$subhead arg[6+n+i]$$
For $icode%i% = 0, %...%, %m%-1%$$, this is the index
of an operator that can be skipped if the comparison is false.

$subhead arg[6+n+m]$$
The is the total number operators that might be skipped; i.e., $icode%n%+%m%$$.

$comment ------------------------------------------------------------------ $$
$head CSumOp$$
Is a cumulative summation operator
which has one result variable.

$subhead arg[0]$$
is the index of the parameter that initializes the summation.

$subhead arg[1]$$
argument index that flags the end of the addition variables,
we use the notation $icode%k% = %arg%[1]%$$ below.

$subhead arg[2]$$
argument index that flags the end of the subtraction variables,
we use the notation $icode%ell% = %arg%[2]%$$ below.

$subhead arg[3]$$
argument index that flags the end of the addition dynamic parameters,
we use the notation $icode%m% = %arg%[3]%$$ below.

$subhead arg[4]$$
argument index that flags the end of the subtraction dynamic parameters,
we use the notation $icode%n% = %arg%[4]%$$ below.

$subhead arg[5+i]$$
for $icode%i% = 0, %...%, %k%-6%$$,
this is the index of the $th i$$ variable to be added in the summation.

$subhead arg[k+i]$$
for $icode%i% = 0, %...%, %ell%-%k%-1%$$,
this is the index of the $th i$$ variable to be subtracted in the summation.

$subhead arg[ell+i]$$
for $icode%i% = 0, %...%, %m%-%ell%-1%$$, this is the index of the
$th i$$ dynamic parameter to be added in the summation.

$subhead arg[m+i]$$
for $icode%i% = 0, %...%, %n%-%m%-1%$$, this is the index of the
$th i$$ dynamic parameter to be subtracted in the summation.

$subhead arg[n]$$
This is equal to $icode n$$.
Note that there are $icode%n%+1%$$ arguments to this operator
and having this value at the end enable reverse model to know how far
to back up to get to the start of this operation.

$comment ------------------------------------------------------------------ $$
$head DisOp$$
Call to a user defined $cref discrete$$ function.
This operator has one result variable.

$subhead arg[0]$$
is the index, in the order of the functions defined by the user,
for this discrete function.

$subhead arg[1]$$
variable index corresponding to the argument for this function call.

$comment ------------------------------------------------------------------ $$
$head Load$$
The load operators create a new variable corresponding to
$icode%vec%[%ind%]%$$ where $icode vec$$ is a $cref VecAD$$ vector
and $icode ind$$ is an $codei%AD<%Base%>%$$.
For these operators either $icode vec$$ or $icode ind$$ is a variable
and there is one variable result.

$subhead LdpOp$$
This load is used for an index $icode ind$$ that is a parameter.

$subhead LdvOp$$
This load is used for an index $icode ind$$ that is a variable.

$subhead arg[0]$$
is the offset of this VecAD vector
relative to the beginning of the single array
that contains all VecAD elements for all the VecAD vectors.
This corresponds to the first element of this vector and not its size
(which comes just before the first element).

$subhead arg[1]$$
is the index in this VecAD vector for this load operation.
For the $code LdpOp$$ ($code LdvOp$$) operator this is the
parameter index (variable index) corresponding to $icode ind$$.

$subhead arg[2]$$
is the index of this VecAD load operation in the set of all
the load operations in this recording.
This includes both dynamic parameter and variable loads.
It is used to map load operations to corresponding
dynamic parameters and variables.

$comment ------------------------------------------------------------------ $$
$head Store$$
The store operators store information corresponding to
$icode%vec%[%ind%]% = %right%$$ where $icode vec$$ is a $cref VecAD$$ vector
and $icode ind$$ is an $codei%AD<%Base%>%$$.
For these operators either $icode vec$$, $icode ind$$, or $icode right$$
is a variable and there is no result.

$subhead StppOp$$
This store is used when $icode ind$$ and $icode right$$ are parameters.

$subhead StpvOp$$
This store is used when $icode ind$$ is a parameter
and $icode right$$ is a variable.

$subhead StvpOp$$
This store is used when $icode ind$$ is a variable
and $icode right$$ is a parameter.

$subhead StvvOp$$
This store is used when $icode index$$ and $icode right$$ are variables.

$subhead arg[0]$$
is the offset of this VecAD vector
relative to the beginning of the single array
that contains all VecAD elements for all the VecAD vectors.
This corresponds to the first element of this vector and not its size
(which comes just before the first element).

$subhead arg[1]$$
is the index in this VecAD vector for this store operation.
For the $code StppOp$$ and $code StpvOp$$ cases
this is the parameter index corresponding to $icode ind$$.
For the $code StvpOp$$ and $code StvvOp$$ cases,
this is the variable index corresponding to $icode ind$$.

$subhead arg[2]$$
For the $code StppOp$$ and $code StvpOp$$ cases,
this is the parameter index corresponding to $icode right$$.
For the $code StpvOp$$ and $code StvvOp$$ cases,
this is the variable index corresponding to $icode right$$.

$comment ------------------------------------------------------------------ $$
$head ParOp$$
This operator has one result that is equal to a parameter
(not that all the derivatives for this result will be zero).

$subhead arg[0]$$
Is the index of the parameter that determines the value of the variable.

$comment ------------------------------------------------------------------ $$
$head PriOp$$
This operator implements the $cref PrintFor$$ command
$codei%
    PrintFor(%pos%, %before%, %value%, %after%)
%$$

$subhead arg[0]$$
The first two bits of this integer are used as flags; see below.

$subhead arg[1]$$
If arg[1] & 1 is true (false),
this is the variable index (parameter index) corresponding to $icode pos$$.

$subhead arg[2]$$
is the text index corresponding to $icode before$$.

$subhead arg[3]$$
If arg[1] & 2 is true (false),
this is the variable index (parameter index) corresponding to $icode value$$.

$subhead arg[4]$$
is the text index corresponding to $icode after$$.

$comment ------------------------------------------------------------------ $$

$head Source$$
$srccode%hpp% */
// BEGIN_SORT_THIS_LINE_PLUS_2
enum OpCode {
    AbsOp,    // unary fabs
    AcosOp,   // unary acos
    AcoshOp,  // unary acosh
    AddpvOp,  // binary +
    AddvvOp,  // ...
    AFunOp,   // see its heading above
    AsinOp,   // unary asin
    AsinhOp,  // unary asinh
    AtanOp,   // unary atan
    AtanhOp,  // unary atanh
    BeginOp,  // see its heading above
    CExpOp,   // ...
    CosOp,    // unary cos
    CoshOp,   // unary cosh
    CSkipOp,  // see its heading above
    CSumOp,   // ...
    DisOp,    // ...
    DivpvOp,  // binary /
    DivvpOp,  // ...
    DivvvOp,  // ...
    EndOp,    // used to mark the end of the tape
    EqppOp,   // compare equal
    EqpvOp,   // ...
    EqvvOp,   // ...
    ErfOp,    // unary erf
    ErfcOp,   // unary erfc
    ExpOp,    // unary exp
    Expm1Op,  // unary expm1
    FunapOp,  // see AFun heading above
    FunavOp,  // ...
    FunrpOp,  // ...
    FunrvOp,  // ...
    InvOp,    // independent variable, no argumements, one result variable
    LdpOp,    // see its heading above
    LdvOp,    // ...
    LeppOp,   // compare <=
    LepvOp,   // ...
    LevpOp,   // ...
    LevvOp,   // ...
    LogOp,    // unary log
    Log1pOp,  // unary log1p
    LtppOp,   // compare <
    LtpvOp,   // ...
    LtvpOp,   // ...
    LtvvOp,   // ...
    MulpvOp,  // binary *
    MulvvOp,  // ...
    NegOp,    // unary negative
    NeppOp,   // compare !=
    NepvOp,   // ...
    NevvOp,   // ...
    ParOp,    // see its heading above
    PowpvOp,  // see its heading above
    PowvpOp,  // binary
    PowvvOp,  // see its heading above
    PriOp,    // see its heading above
    SignOp,   // unary sign
    SinOp,    // unary sin
    SinhOp,   // unary sinh
    SqrtOp,   // unary sqrt
    StppOp,   // see its heading above
    StpvOp,   // ...
    StvpOp,   // ...
    StvvOp,   // ...
    SubpvOp,  // binary -
    SubvpOp,  // ...
    SubvvOp,  // ...
    TanOp,    // unary tan
    TanhOp,   // unary tanh
    ZmulpvOp, // binary azmul
    ZmulvpOp, // ...
    ZmulvvOp, // ...
    NumberOp  // number of operator codes (not an operator)
};
// END_SORT_THIS_LINE_MINUS_3
/* %$$
$end
*/
// Note that bin/check_op_code.sh assumes the pattern NumberOp occurs
// at the end of this list and only at the end of this list.

/*!
Number of arguments for a specified operator.

\return
Number of arguments corresponding to the specified operator.

\param op
Operator for which we are fetching the number of arugments.

\par NumArgTable
this table specifes the number of arguments stored for each
occurance of the operator that is the i-th value in the OpCode enum type.
For example, for the first three OpCode enum values we have
\verbatim
OpCode   j   NumArgTable[j]  Meaning
AbsOp    0                1  index of variable we are taking absolute value of
AcosOp   1                1  index of variable we are taking acos of
AcoshOp  2                1  index of variable we are taking acosh of
\endverbatim
Note that the meaning of the arguments depends on the operator.
*/
inline size_t NumArg( OpCode op)
{   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;

    // agreement with OpCode is checked by bin/check_op_code.sh
    static const size_t NumArgTable[] = {
        1, // AbsOp
        1, // AcosOp
        1, // AcoshOp
        2, // AddpvOp
        2, // AddvvOp
        4, // AFunOp
        1, // AsinOp
        1, // AsinhOp
        1, // AtanOp
        1, // AtanhOp
        1, // BeginOp  offset first real argument to have index 1
        6, // CExpOp
        1, // CosOp
        1, // CoshOp
        0, // CSkipOp  (actually has a variable number of arguments, not zero)
        0, // CSumOp   (actually has a variable number of arguments, not zero)
        2, // DisOp
        2, // DivpvOp
        2, // DivvpOp
        2, // DivvvOp
        0, // EndOp
        2, // EqppOp
        2, // EqpvOp
        2, // EqvvOp
        3, // ErfOp
        3, // ErfcOp
        1, // ExpOp
        1, // Expm1Op
        1, // FunapOp
        1, // FunavOp
        1, // FunrpOp
        0, // FunrvOp
        0, // InvOp
        3, // LdpOp
        3, // LdvOp
        2, // LeppOp
        2, // LepvOp
        2, // LevpOp
        2, // LevvOp
        1, // LogOp
        1, // Log1pOp
        2, // LtppOp
        2, // LtpvOp
        2, // LtvpOp
        2, // LtvvOp
        2, // MulpvOp
        2, // MulvvOp
        1, // NegOp
        2, // NeppOp
        2, // NepvOp
        2, // NevvOp
        1, // ParOp
        2, // PowpvOp
        2, // PowvpOp
        2, // PowvvOp
        5, // PriOp
        1, // SignOp
        1, // SinOp
        1, // SinhOp
        1, // SqrtOp
        3, // StppOp
        3, // StpvOp
        3, // StvpOp
        3, // StvvOp
        2, // SubpvOp
        2, // SubvpOp
        2, // SubvvOp
        1, // TanOp
        1, // TanhOp
        2, // ZmulpvOp
        2, // ZmulvpOp
        2, // ZmulvvOp
        0  // NumberOp not used
    };
# ifndef NDEBUG
    // only do these checks once to save time
    static bool first = true;
    if( first )
    {   first = false;
        // check that NumberOp is last value in op code table
        CPPAD_ASSERT_UNKNOWN(
            size_t(NumberOp) + 1 == sizeof(NumArgTable)/sizeof(NumArgTable[0])
        );
        //Check that the type CPPAD_VEC_ENUM_TYPE as required by define.hpp
        CPPAD_ASSERT_UNKNOWN( is_pod<opcode_t>() );
        size_t number_op_size_t = size_t( NumberOp );
        CPPAD_ASSERT_UNKNOWN(
            number_op_size_t < std::numeric_limits<opcode_t>::max()
        );
    }
    // do this check every time
    CPPAD_ASSERT_UNKNOWN( size_t(op) < size_t(NumberOp) );
# endif

    return NumArgTable[op];
}

/*!
Number of variables resulting from the specified operation.

\param op
Operator for which we are fecching the number of results.

\par NumResTable
table specifes the number of varibles that result for each
occurance of the operator that is the i-th value in the OpCode enum type.
For example, for the first three OpCode enum values we have
\verbatim
OpCode   j   NumResTable[j]  Meaning
AbsOp    0                1  variable that is the result of the absolute value
AcosOp   1                2  acos(x) and sqrt(1-x*x) are required for this op
AcoshOp  2                2  acosh(x) and sqrt(x*x-1) are required for this op
\endverbatim
*/
inline size_t NumRes(OpCode op)
{   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;

    // agreement with OpCode is checked by bin/check_op_code.sh
    static const size_t NumResTable[] = {
        1, // AbsOp
        2, // AcosOp
        2, // AcoshOp
        1, // AddpvOp
        1, // AddvvOp
        0, // AFunOp
        2, // AsinOp
        2, // AsinhOp
        2, // AtanOp
        2, // AtanhOp
        1, // BeginOp  offsets first variable to have index one (not zero)
        1, // CExpOp
        2, // CosOp
        2, // CoshOp
        0, // CSkipOp
        1, // CSumOp
        1, // DisOp
        1, // DivpvOp
        1, // DivvpOp
        1, // DivvvOp
        0, // EndOp
        0, // EqppOp
        0, // EqpvOp
        0, // EqvvOp
        5, // ErfOp
        5, // ErfcOp
        1, // ExpOp
        1, // Expm1Op
        0, // FunapOp
        0, // FunavOp
        0, // FunrpOp
        1, // FunrvOp
        1, // InvOp
        1, // LdpOp
        1, // LdvOp
        0, // LeppOp
        0, // LepvOp
        0, // LevpOp
        0, // LevvOp
        1, // LogOp
        1, // Log1pOp
        0, // LtppOp
        0, // LtpvOp
        0, // LtvpOp
        0, // LtvvOp
        1, // MulpvOp
        1, // MulvvOp
        1, // NegOp
        0, // NeppOp
        0, // NepvOp
        0, // NevvOp
        1, // ParOp
        3, // PowpvOp
        1, // PowvpOp
        3, // PowvvOp
        0, // PriOp
        1, // SignOp
        2, // SinOp
        2, // SinhOp
        1, // SqrtOp
        0, // StppOp
        0, // StpvOp
        0, // StvpOp
        0, // StvvOp
        1, // SubpvOp
        1, // SubvpOp
        1, // SubvvOp
        2, // TanOp
        2, // TanhOp
        1, // ZmulpvOp
        1, // ZmulvpOp
        1, // ZmulvvOp
        0  // NumberOp not used and avoids g++ 4.3.2 warn when pycppad builds
    };
    // check ensuring conversion to size_t is as expected
    CPPAD_ASSERT_UNKNOWN( size_t(NumberOp) + 1 ==
        sizeof(NumResTable) / sizeof(NumResTable[0])
    );
    // this test ensures that all indices are within the table
    CPPAD_ASSERT_UNKNOWN( size_t(op) < size_t(NumberOp) );

    return NumResTable[op];
}


/*!
Fetch the name for a specified operation.

\return
name of the specified operation.

\param op
Operator for which we are fetching the name
*/
inline const char* OpName(OpCode op)
{   // agreement with OpCode is checked by bin/check_op_code.sh
    static const char *OpNameTable[] = {
        "Abs"   ,
        "Acos"  ,
        "Acosh" ,
        "Addpv" ,
        "Addvv" ,
        "AFun"  ,
        "Asin"  ,
        "Asinh" ,
        "Atan"  ,
        "Atanh" ,
        "Begin" ,
        "CExp"  ,
        "Cos"   ,
        "Cosh"  ,
        "CSkip" ,
        "CSum"  ,
        "Dis"   ,
        "Divpv" ,
        "Divvp" ,
        "Divvv" ,
        "End"   ,
        "Eqpp"  ,
        "Eqpv"  ,
        "Eqvv"  ,
        "Erf"   ,
        "Erfc"  ,
        "Exp"   ,
        "Expm1" ,
        "Funap" ,
        "Funav" ,
        "Funrp" ,
        "Funrv" ,
        "Inv"   ,
        "Ldp"   ,
        "Ldv"   ,
        "Lepp"  ,
        "Lepv"  ,
        "Levp"  ,
        "Levv"  ,
        "Log"   ,
        "Log1p" ,
        "Ltpp"  ,
        "Ltpv"  ,
        "Ltvp"  ,
        "Ltvv"  ,
        "Mulpv" ,
        "Mulvv" ,
        "Neg"   ,
        "Nepp"  ,
        "Nepv"  ,
        "Nevv"  ,
        "Par"   ,
        "Powpv" ,
        "Powvp" ,
        "Powvv" ,
        "Pri"   ,
        "Sign"  ,
        "Sin"   ,
        "Sinh"  ,
        "Sqrt"  ,
        "Stpp"  ,
        "Stpv"  ,
        "Stvp"  ,
        "Stvv"  ,
        "Subpv" ,
        "Subvp" ,
        "Subvv" ,
        "Tan"   ,
        "Tanh"  ,
        "Zmulpv",
        "Zmulvp",
        "Zmulvv",
        "Number"  // not used
    };
    // check ensuring conversion to size_t is as expected
    CPPAD_ASSERT_UNKNOWN(
        size_t(NumberOp) + 1 == sizeof(OpNameTable)/sizeof(OpNameTable[0])
    );
    // this test ensures that all indices are within the table
    CPPAD_ASSERT_UNKNOWN( size_t(op) < size_t(NumberOp) );

    return OpNameTable[op];
}

/*!
Prints a single field corresponding to an operator.

A specified leader is printed in front of the value
and then the value is left justified in the following width character.

\tparam Type
is the type of the value we are printing.

\param os
is the stream that we are printing to.

\param leader
are characters printed before the value.

\param value
is the value being printed.

\param width
is the number of character to print the value in.
If the value does not fit in the width, the value is replace
by width '*' characters.
*/
template <class Type>
void printOpField(
    std::ostream      &os ,
    const char *   leader ,
    const Type     &value ,
    size_t          width )
{
    std::ostringstream buffer;
    std::string        str;

    // first print the leader
    os << leader;

    // print the value into an internal buffer
    buffer << std::setw( int(width) ) << value;
    str = buffer.str();

    // length of the string
    size_t len = str.size();
    if( len > width )
    {
        for(size_t i = 0; i < width-1; i++)
            os << str[i];
        os << "*";
        return;
    }

    // count number of spaces at begining
    size_t nspace = 0;
    while(str[nspace] == ' ' && nspace < len)
        nspace++;

    // left justify the string
    size_t i = nspace;
    while( i < len )
        os << str[i++];

    i = width - len + nspace;
    while(i--)
        os << " ";
}

/*!
Prints a single operator and its operands

\tparam Base
Is the base type for these AD< Base > operations.

\param os
is the output stream that the information is printed on.

\param play
Is the entire recording for the tape that this operator is in.

\param i_op
is the index for the operator corresponding to this operation.

\param i_var
is the index for the variable corresponding to the result of this operation
(if NumRes(op) > 0).

\param op
The operator code (OpCode) for this operation.

\param arg
is the vector of argument indices for this operation
(must have NumArg(op) elements).
*/
template <class Base, class RecBase>
void printOp(
    std::ostream&          os     ,
    const local::player<Base>* play,
    size_t                 i_op   ,
    size_t                 i_var  ,
    OpCode                 op     ,
    const addr_t*          arg    )
{
    CPPAD_ASSERT_KNOWN(
        ! thread_alloc::in_parallel() ,
        "cannot print trace of AD operations in parallel mode"
    );
    static const char *CompareOpName[] =
        { "Lt", "Le", "Eq", "Ge", "Gt", "Ne" };

    // print operator
    printOpField(os,  "o=",      i_op,  5);
    if( NumRes(op) > 0 && op != BeginOp )
        printOpField(os,  "v=",      i_var, 5);
    else
        printOpField(os,  "v=",      "",    5);
    if( op == CExpOp || op == CSkipOp )
    {   printOpField(os, "", OpName(op), 5);
        printOpField(os, "", CompareOpName[ arg[0] ], 3);
    }
    else
        printOpField(os, "", OpName(op), 8);

    // print other fields
    size_t ncol = 5;
    switch( op )
    {
        case CSkipOp:
        /*
        arg[0]     = the Rel operator: Lt, Le, Eq, Ge, Gt, or Ne
        arg[1] & 1 = is left a variable
        arg[1] & 2 = is right a variable
        arg[2]     = index correspoding to left
        arg[3]     = index correspoding to right
        arg[4] = number of operations to skip if CExpOp comparison is true
        arg[5] = number of operations to skip if CExpOp comparison is false
        arg[6] -> arg[5+arg[4]]               = skip operations if true
        arg[6+arg[4]] -> arg[5+arg[4]+arg[5]] = skip operations if false
        arg[6+arg[4]+arg[5]] = arg[4] + arg[5]
        */
        CPPAD_ASSERT_UNKNOWN( arg[6+arg[4]+arg[5]] == arg[4]+arg[5] );
        CPPAD_ASSERT_UNKNOWN(arg[1] != 0);
        if( arg[1] & 1 )
            printOpField(os, " vl=", arg[2], ncol);
        else
            printOpField(os, " pl=", play->GetPar(arg[2]), ncol);
        if( arg[1] & 2 )
            printOpField(os, " vr=", arg[3], ncol);
        else
            printOpField(os, " pr=", play->GetPar(arg[3]), ncol);
        if( size_t(arg[4]) < 3 )
        {   for(addr_t i = 0; i < arg[4]; i++)
                printOpField(os, " ot=", arg[6+i], ncol);
        }
        else
        {   printOpField(os, "\n\tot=", arg[6+0], ncol);
            for(addr_t i = 1; i < arg[4]; i++)
                printOpField(os, " ot=", arg[6+i], ncol);
        }
        if( size_t(arg[5]) < 3 )
        {   for(addr_t i = 0; i < arg[5]; i++)
                printOpField(os, " of=", arg[6+arg[4]+i], ncol);
        }
        else
        {   printOpField(os, "\n\tof=", arg[6+arg[4]+0], ncol);
            {   for(addr_t i = 1; i < arg[5]; i++)
                    printOpField(os, " of=", arg[6+arg[4]+i], ncol);
            }
        }
        break;

        case CSumOp:
        /*
        arg[0] = index of parameter that initializes summation
        arg[1] = end in arg of addition variables in summation
        arg[2] = end in arg of subtraction variables in summation
        arg[3] = end in arg of addition dynamic parameters in summation
        arg[4] = end in arg of subtraction dynamic parameters in summation
        arg[5],      ... , arg[arg[1]-1]: indices for addition variables
        arg[arg[1]], ... , arg[arg[2]-1]: indices for subtraction variables
        arg[arg[2]], ... , arg[arg[3]-1]: indices for additon dynamics
        arg[arg[3]], ... , arg[arg[4]-1]: indices for subtraction dynamics
        arg[arg[4]] = arg[4]
        */
        CPPAD_ASSERT_UNKNOWN( arg[arg[4]] == arg[4] );
        printOpField(os, " pr=", play->GetPar(arg[0]), ncol);
        for(addr_t i = 5; i < arg[1]; i++)
             printOpField(os, " +v=", arg[i], ncol);
        for(addr_t i = arg[1]; i < arg[2]; i++)
             printOpField(os, " -v=", arg[i], ncol);
        for(addr_t i = arg[2]; i < arg[3]; i++)
             printOpField(os, " +d=", play->GetPar(arg[i]), ncol);
        for(addr_t i = arg[3]; i < arg[4]; i++)
             printOpField(os, " -d=", play->GetPar(arg[i]), ncol);
        break;

        case LdpOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        printOpField(os, "off=", arg[0], ncol);
        printOpField(os, "  p=", play->GetPar(arg[1]), ncol);
        break;

        case LdvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        printOpField(os, "off=", arg[0], ncol);
        printOpField(os, "  v=", arg[1], ncol);
        break;

        case StppOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        printOpField(os, "off=", arg[0], ncol);
        printOpField(os, " pl=", play->GetPar(arg[1]), ncol);
        printOpField(os, " pr=", play->GetPar(arg[2]), ncol);
        break;

        case StpvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        printOpField(os, "off=", arg[0], ncol);
        printOpField(os, "  p=", play->GetPar(arg[1]), ncol);
        printOpField(os, "  v=", arg[2], ncol);
        break;

        case StvpOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        printOpField(os, "off=", arg[0], ncol);
        printOpField(os, "  v=", arg[1], ncol);
        printOpField(os, "  p=", play->GetPar(arg[2]), ncol);
        break;

        case StvvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        printOpField(os, "off=", arg[0], ncol);
        printOpField(os, " vl=", arg[1], ncol);
        printOpField(os, " vr=", arg[2], ncol);
        break;

        case AddvvOp:
        case DivvvOp:
        case EqvvOp:
        case LevvOp:
        case LtvvOp:
        case NevvOp:
        case MulvvOp:
        case PowvvOp:
        case SubvvOp:
        case ZmulvvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
        printOpField(os, " vl=", arg[0], ncol);
        printOpField(os, " vr=", arg[1], ncol);
        break;

        case AddpvOp:
        case EqpvOp:
        case DivpvOp:
        case LepvOp:
        case LtpvOp:
        case NepvOp:
        case SubpvOp:
        case MulpvOp:
        case PowpvOp:
        case ZmulpvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
        printOpField(os, " pl=", play->GetPar(arg[0]), ncol);
        printOpField(os, " vr=", arg[1], ncol);
        break;

        case DivvpOp:
        case LevpOp:
        case LtvpOp:
        case PowvpOp:
        case SubvpOp:
        case ZmulvpOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
        printOpField(os, " vl=", arg[0], ncol);
        printOpField(os, " pr=", play->GetPar(arg[1]), ncol);
        break;

        case AbsOp:
        case AcosOp:
        case AcoshOp:
        case AsinOp:
        case AsinhOp:
        case AtanOp:
        case AtanhOp:
        case CosOp:
        case CoshOp:
        case ExpOp:
        case Expm1Op:
        case LogOp:
        case Log1pOp:
        case NegOp:
        case SignOp:
        case SinOp:
        case SinhOp:
        case SqrtOp:
        case FunavOp:
        case TanOp:
        case TanhOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
        printOpField(os, "  v=", arg[0], ncol);
        break;

        case ErfOp:
        case ErfcOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        // arg[1] points to the parameter 0
        // arg[2] points to the parameter 2 / sqrt(pi)
        printOpField(os, "  v=", arg[0], ncol);
        break;

        case ParOp:
        case FunapOp:
        case FunrpOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
        printOpField(os, "  p=", play->GetPar(arg[0]), ncol);
        break;

        case AFunOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 4 );
        {
            // get the name of this atomic function
            bool         set_null   = false;
            size_t       atom_index = size_t( arg[0] );
            size_t       type       = 0;          // set to avoid warning
            std::string name;
            void*        v_ptr    = nullptr; // set to avoid warning
            atomic_index<RecBase>(set_null, atom_index, type, &name, v_ptr);
            printOpField(os, " f=",   name.c_str(), ncol);
            printOpField(os, " i=", arg[1], ncol);
            printOpField(os, " n=", arg[2], ncol);
            printOpField(os, " m=", arg[3], ncol);
        }
        break;

        case PriOp:
        CPPAD_ASSERT_NARG_NRES(op, 5, 0);
        if( arg[0] & 1 )
            printOpField(os, " v=", arg[1], ncol);
        else
            printOpField(os, " p=", play->GetPar(arg[1]), ncol);
        os << "before=\"" << play->GetTxt(arg[2]) << "\"";
        if( arg[0] & 2 )
            printOpField(os, " v=", arg[3], ncol);
        else
            printOpField(os, " p=", play->GetPar(arg[3]), ncol);
        os << "after=\"" << play->GetTxt(arg[4]) << "\"";
        break;

        case BeginOp:
        // argument not used (created by independent)
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
        break;

        case EndOp:
        case InvOp:
        case FunrvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 0 );
        break;

        case DisOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
        {   const char* name = discrete<Base>::name(arg[0]);
            printOpField(os, " f=", name, ncol);
            printOpField(os, " x=", arg[1], ncol);
        }
        break;


        case CExpOp:
        CPPAD_ASSERT_UNKNOWN(arg[1] != 0);
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 6 );
        if( arg[1] & 1 )
            printOpField(os, " vl=", arg[2], ncol);
        else
            printOpField(os, " pl=", play->GetPar(arg[2]), ncol);
        if( arg[1] & 2 )
            printOpField(os, " vr=", arg[3], ncol);
        else
            printOpField(os, " pr=", play->GetPar(arg[3]), ncol);
        if( arg[1] & 4 )
            printOpField(os, " vt=", arg[4], ncol);
        else
            printOpField(os, " pt=", play->GetPar(arg[4]), ncol);
        if( arg[1] & 8 )
            printOpField(os, " vf=", arg[5], ncol);
        else
            printOpField(os, " pf=", play->GetPar(arg[5]), ncol);
        break;

        case EqppOp:
        case LeppOp:
        case LtppOp:
        case NeppOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
        printOpField(os, " pl=", play->GetPar(arg[0]), ncol);
        printOpField(os, " pr=", play->GetPar(arg[1]), ncol);
        break;

        default:
        CPPAD_ASSERT_UNKNOWN(0);
    }
}

/*!
Prints the result values correspnding to an operator.

\tparam Base
Is the base type for these AD< Base > operations.

\tparam Value
Determines the type of the values that we are printing.

\param os
is the output stream that the information is printed on.

\param nfz
is the number of forward sweep calculated values of type Value
that correspond to this operation
(ignored if NumRes(op) == 0).

\param fz
points to the first forward calculated value
that correspond to this operation
(ignored if NumRes(op) == 0).

\param nrz
is the number of reverse sweep calculated values of type Value
that correspond to this operation
(ignored if NumRes(op) == 0).

\param rz
points to the first reverse calculated value
that correspond to this operation
(ignored if NumRes(op) == 0).
*/
template <class Value>
void printOpResult(
    std::ostream          &os     ,
    size_t                 nfz    ,
    const  Value          *fz     ,
    size_t                 nrz    ,
    const  Value          *rz     )
{
    size_t k;
    for(k = 0; k < nfz; k++)
        os << "| fz[" << k << "]=" << fz[k];
    for(k = 0; k < nrz; k++)
        os << "| rz[" << k << "]=" << rz[k];
}

/*!
Determines which arguments are variaibles for an operator.

\param op
is the operator. Note that CSkipOp and CSumOp are special cases
because the true number of arguments is not equal to NumArg(op)
and the true number of arguments num_arg can be large.
It may be more efficient to handle these cases separately
(see below).

\param arg
is the argument vector for this operator.

\param is_variable
If the input value of the elements in this vector do not matter.
Upon return, resize has been used to set its size to the true number
of arguments to this operator.
If op != CSkipOp and op != CSumOp, is_variable.size() = NumArg(op).
The j-th argument for this operator is a
variable index if and only if is_variable[j] is true. Note that the variable
index 0, for the BeginOp, does not correspond to a real variable and false
is returned for this case.

\par CSkipOp
In the case of CSkipOp,
\code
        is_variable.size()  = 7 + arg[4] + arg[5];
        is_variable[2]      = (arg[1] & 1) != 0;
        is_variable[3]      = (arg[1] & 2) != 0;
\endcode
and all the other is_variable[j] values are false.

\par CSumOp
In the case of CSumOp,
\code
        is_variable.size() = arg[4]
        for(size_t j = 5; j < arg[2]; ++j)
            is_variable[j] = true;
\endcode
and all the other is_variable values are false.
*/
template <class Addr>
void arg_is_variable(
    OpCode            op          ,
    const Addr*       arg         ,
    pod_vector<bool>& is_variable )
{   size_t num_arg = NumArg(op);
    is_variable.resize( num_arg );
    //
    switch(op)
    {
        // -------------------------------------------------------------------
        // cases where true number of arugments = NumArg(op) == 0

        case EndOp:
        case InvOp:
        case FunrvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 0 );
        break;

        // -------------------------------------------------------------------
        // cases where NumArg(op) == 1
        case AbsOp:
        case AcoshOp:
        case AcosOp:
        case AsinhOp:
        case AsinOp:
        case AtanhOp:
        case AtanOp:
        case CoshOp:
        case CosOp:
        case Expm1Op:
        case ExpOp:
        case Log1pOp:
        case LogOp:
        case NegOp:
        case SignOp:
        case SinhOp:
        case SinOp:
        case SqrtOp:
        case TanhOp:
        case TanOp:
        case FunavOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
        is_variable[0] = true;
        break;

        case BeginOp:
        case ParOp:
        case FunapOp:
        case FunrpOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
        is_variable[0] = false;
        break;


        // -------------------------------------------------------------------
        // cases where NumArg(op) == 2

        case AddpvOp:
        case DisOp:
        case DivpvOp:
        case EqpvOp:
        case LepvOp:
        case LtpvOp:
        case MulpvOp:
        case NepvOp:
        case PowpvOp:
        case SubpvOp:
        case ZmulpvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
        is_variable[0] = false;
        is_variable[1] = true;
        break;

        case DivvpOp:
        case LevpOp:
        case LtvpOp:
        case PowvpOp:
        case SubvpOp:
        case ZmulvpOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
        is_variable[0] = true;
        is_variable[1] = false;
        break;

        case AddvvOp:
        case DivvvOp:
        case EqvvOp:
        case LevvOp:
        case LtvvOp:
        case MulvvOp:
        case NevvOp:
        case PowvvOp:
        case SubvvOp:
        case ZmulvvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
        is_variable[0] = true;
        is_variable[1] = true;
        break;

        case ErfOp:
        case ErfcOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        is_variable[0] = true;
        is_variable[1] = false; // parameter index corresponding to zero
        is_variable[2] = false; // parameter index corresponding to one
        break;

        // --------------------------------------------------------------------
        // cases where NumArg(op) == 3

        case LdpOp:
        case StppOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        is_variable[0] = false;
        is_variable[1] = false;
        is_variable[2] = false;
        break;

        case LdvOp:
        case StvpOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        is_variable[0] = false;
        is_variable[1] = true;
        is_variable[2] = false;
        break;

        case StpvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        is_variable[0] = false;
        is_variable[1] = false;
        is_variable[2] = true;
        break;

        case StvvOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
        is_variable[0] = false;
        is_variable[1] = true;
        is_variable[2] = true;
        break;

        // --------------------------------------------------------------------
        // case where NumArg(op) == 4
        case AFunOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 4 );
        for(size_t i = 0; i < 4; i++)
            is_variable[i] = false;
        break;

        // --------------------------------------------------------------------
        // case where NumArg(op) == 5
        case PriOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 5 );
        is_variable[0] = false;
        is_variable[1] = (arg[0] & 1) != 0;
        is_variable[2] = false;
        is_variable[3] = (arg[0] & 2) != 0;
        is_variable[4] = false;
        break;

        // --------------------------------------------------------------------
        // case where NumArg(op) == 6
        case CExpOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 6 );
        is_variable[0] = false;
        is_variable[1] = false;
        is_variable[2] = (arg[0] & 1) != 0;
        is_variable[3] = (arg[0] & 2) != 0;
        is_variable[4] = (arg[0] & 4) != 0;
        is_variable[5] = (arg[0] & 8) != 0;
        break;

        // -------------------------------------------------------------------
        // CSkipOp:
        case CSkipOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 0 )
        //
        // true number of arguments
        num_arg = size_t(7 + arg[4] + arg[5]);
        is_variable.resize(num_arg);
        is_variable[0] = false;
        is_variable[1] = false;
        is_variable[2] = (arg[1] & 1) != 0;
        is_variable[3] = (arg[1] & 2) != 0;
        for(size_t i = 4; i < num_arg; ++i)
            is_variable[i] = false;
        break;

        // -------------------------------------------------------------------
        // CSumOp:
        case CSumOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 0 )
        //
        // true number of arguments
        num_arg = size_t(arg[4]);
        //
        is_variable.resize( num_arg );
        for(size_t i = 0; i < num_arg; ++i)
            is_variable[i] = (5 <= i) & (i < size_t(arg[2]));
        break;

        case EqppOp:
        case LeppOp:
        case LtppOp:
        case NeppOp:
        CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
        is_variable[0] = false;
        is_variable[1] = false;
        break;

        // --------------------------------------------------------------------
        default:
        CPPAD_ASSERT_UNKNOWN(false);
        break;
    }
    return;
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
