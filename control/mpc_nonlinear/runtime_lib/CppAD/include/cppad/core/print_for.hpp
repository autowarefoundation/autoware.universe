# ifndef CPPAD_CORE_PRINT_FOR_HPP
# define CPPAD_CORE_PRINT_FOR_HPP
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
$begin PrintFor$$
$spell
    notpos
    var
    VecAD
    std
    cout
    const
$$


$section Printing AD Values During Forward Mode$$

$head Syntax$$
$icode%f%.Forward(0, %x%)
%$$
$icode%f%.Forward(0, %x%, %s%)
%$$
$codei%PrintFor(%before%, %value%)
%$$
$codei%PrintFor(%notpos%, %before%, %value%, %after%)
%$$

$head See Also$$
$cref ad_output$$

$head Purpose$$
The $cref/zero order forward/forward_zero/$$ mode command
$codei%
    %f%.Forward(0, %x%)
%$$
sets the
$cref/independent variable/glossary/Tape/Independent Variable/$$ vector
equal to $icode x$$.
It then computes a value for all of the dependent variables in the
$cref/operation sequence/glossary/Operation/Sequence/$$ corresponding
to $icode f$$.
Putting a $code PrintFor$$ in the operation sequence,
prints $icode value$$, corresponding to $icode x$$,
to be printed during zero order forward operations.

$head f.Forward(0, x)$$
The objects $icode f$$, $icode x$$, and the purpose
for this operation, are documented in $cref Forward$$.

$head notpos$$
If present, the argument $icode notpos$$ has one of the following prototypes
$codei%
    const AD<%Base%>&               %notpos%
    const VecAD<%Base%>::reference& %notpos%
%$$
In this case
the text and $icode value$$ will be printed if and only if
$icode notpos$$ is not positive (greater than zero) and a finite number.

$head before$$
The argument $icode before$$ has prototype
$codei%
    const char* %before%
%$$
This text is written to $code std::cout$$ before $icode value$$.

$head value$$
The argument $icode value$$ has one of the following prototypes
$codei%
    const AD<%Base%>&               %value%
    const VecAD<%Base%>::reference& %value%
%$$
The $icode value$$, that corresponds to $icode x$$,
is written to $code std::cout$$ during the execution of
$codei%
    %f%.Forward(0, %x%)
%$$
Note that $icode value$$ may be a
$cref/variable/glossary/Variable/$$ or
$cref/parameter/glossary/Parameter/$$.
If a parameter is
$cref/dynamic/glossary/Parameter/Dynamic/$$ its value
will depend on the previous call to $cref new_dynamic$$.

$head after$$
The argument $icode after$$ has prototype
$codei%
    const char* %after%
%$$
This text is written to $code std::cout$$ after $icode value$$.

$head s$$
You can redirect this output to any standard output stream using the syntax
$codei%
    %f%.Forward(0, %x%, %s%)
%$$
see $cref/s/forward_zero/s/$$ in the zero order forward mode documentation.

$head Discussion$$
This is helpful for understanding why tape evaluations have trouble.
For example, if one of the operations in $icode f$$ is
$codei%log(%value%)%$$ and $icode%value% < 0%$$,
the corresponding result will $cref nan$$.

$head Example$$
$children%
    example/print_for/print_for.cpp%
    example/general/print_for.cpp
%$$
The program
$cref print_for_cout.cpp$$
is an example and test that prints to standard output.
The output of this program
states the conditions for passing and failing the test.
The function
$cref print_for_string.cpp$$
is an example and test that prints to an standard string stream.
This function automatically check for correct output.

$end
------------------------------------------------------------------------------
*/

# include <cstring>

namespace CppAD {
    template <class Base>
    void PrintFor(
        const AD<Base>& notpos        ,
        const char*     before        ,
        const AD<Base>& value         ,
        const char*     after         )
    {   CPPAD_ASSERT_NARG_NRES(local::PriOp, 5, 0);

        // check for case where we are not recording operations
        local::ADTape<Base>* tape = AD<Base>::tape_ptr();
        if( tape == nullptr )
            return;

        CPPAD_ASSERT_KNOWN(
            std::strlen(before) <= 1000 ,
            "PrintFor: length of before is greater than 1000 characters"
        );
        CPPAD_ASSERT_KNOWN(
            std::strlen(after) <= 1000 ,
            "PrintFor: length of after is greater than 1000 characters"
        );
        addr_t arg0, arg1, arg2, arg3, arg4;

        // arg[0] = base 2 representation of [Var(notpos), Var(value)]
        arg0 = 0;

        // arg[1] = address for notpos
        if( Constant(notpos) )
            arg1  = tape->Rec_.put_con_par(notpos.value_);
        else if( Dynamic(notpos) )
            arg1  = notpos.taddr_;
        else
        {   arg0 += 1;
            arg1  = notpos.taddr_;
        }

        // arg[2] = address of before
        arg2 = tape->Rec_.PutTxt(before);

        // arg[3] = address for value
        if( Constant(value) )
            arg3  = tape->Rec_.put_con_par(value.value_);
        else if( Dynamic(value) )
            arg3  = value.taddr_;
        else
        {   arg0 += 2;
            arg3  = value.taddr_;
        }

        // arg[4] = address of after
        arg4 = tape->Rec_.PutTxt(after);

        // put the operator in the tape
        tape->Rec_.PutArg(arg0, arg1, arg2, arg3, arg4);
        tape->Rec_.PutOp(local::PriOp);
    }
    // Fold all other cases into the case above
    template <class Base>
    void PrintFor(const char* before, const AD<Base>& value)
    {   PrintFor(AD<Base>(0), before, value, "" ); }
    //
    template <class Base>
    void PrintFor(const char* before, const VecAD_reference<Base>& value)
    {   PrintFor(AD<Base>(0), before, value.ADBase(), "" ); }
    //
    template <class Base>
    void PrintFor(
        const VecAD_reference<Base>& notpos ,
        const char                  *before ,
        const VecAD_reference<Base>& value  ,
        const char                  *after  )
    {   PrintFor(notpos.ADBase(), before, value.ADBase(), after); }
    //
    template <class Base>
    void PrintFor(
        const VecAD_reference<Base>& notpos ,
        const char                  *before ,
        const AD<Base>&              value  ,
        const char                  *after  )
    {   PrintFor(notpos.ADBase(), before, value, after); }
    //
    template <class Base>
    void PrintFor(
        const AD<Base>&              notpos ,
        const char                  *before ,
        const VecAD_reference<Base>& value  ,
        const char                  *after  )
    {   PrintFor(notpos, before, value.ADBase(), after); }
}

# endif
