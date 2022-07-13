# ifndef CPPAD_CORE_NUM_SKIP_HPP
# define CPPAD_CORE_NUM_SKIP_HPP
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
$begin number_skip$$
$spell
    optimizer
    var
    taylor_
$$


$section Number of Variables that Can be Skipped$$

$head Syntax$$
$icode%n% = %f%.number_skip()%$$

$subhead See Also$$
$cref fun_property$$

$head Purpose$$
The $cref/conditional expressions/CondExp/$$ use either the
$cref/if_true/CondExp/$$ or $cref/if_false/CondExp/$$.
Hence, some terms only need to be evaluated
depending on the value of the comparison in the conditional expression.
The $cref optimize$$ option is capable of detecting some of these
case and determining variables that can be skipped.
This routine returns the number such variables.

$head n$$
The return value $icode n$$ has type $code size_t$$
is the number of variables that the optimizer has determined can be skipped
(given the independent variable values specified by the previous call to
$cref/f.Forward/Forward/$$ for order zero).

$head f$$
The object $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$

$children%
    example/general/number_skip.cpp
%$$
$head Example$$
The file $cref number_skip.cpp$$
contains an example and test of this function.

$end
-----------------------------------------------------------------------------
*/

# include <cppad/local/play/atom_op_info.hpp>

// BEGIN CppAD namespace
namespace CppAD {

// This routine is not const because it runs through the operations sequence
// 2DO: compute this value during zero order forward operations.
template <class Base, class RecBase>
size_t ADFun<Base,RecBase>::number_skip(void)
{   // must pass through operation sequence to map operations to variables

    // information defined by atomic forward
    size_t atom_index=0, atom_old=0, atom_m=0, atom_n=0;

    // number of variables skipped
    size_t num_var_skip = 0;

    // start playback
    local::play::const_sequential_iterator itr = play_.begin();
    local::OpCode op;
    size_t        i_var;
    const addr_t* arg;
    itr.op_info(op, arg, i_var);
    CPPAD_ASSERT_UNKNOWN(op == local::BeginOp)
    while(op != local::EndOp)
    {   // next op
        (++itr).op_info(op, arg, i_var);
        //
        if( op == local::AFunOp )
        {   // skip only appears at front or back AFunOp of atomic function call
            bool skip_call = cskip_op_[ itr.op_index() ];
            local::play::atom_op_info<Base>(
                op, arg, atom_index, atom_old, atom_m, atom_n
            );
            CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
            size_t num_op = atom_m + atom_n + 1;
            for(size_t i = 0; i < num_op; i++)
            {   CPPAD_ASSERT_UNKNOWN(
                    op != local::CSkipOp && op != local::CSumOp
                );
                (++itr).op_info(op, arg, i_var);
                if( skip_call )
                    num_var_skip += NumRes(op);
            }
            CPPAD_ASSERT_UNKNOWN( op == local::AFunOp );
        }
        else
        {   if( cskip_op_[ itr.op_index() ] )
                num_var_skip += NumRes(op);
            //
            if( (op == local::CSkipOp) | (op == local::CSumOp) )
                itr.correct_before_increment();
        }
    }
    return num_var_skip;
}

} // END CppAD namespace


# endif
