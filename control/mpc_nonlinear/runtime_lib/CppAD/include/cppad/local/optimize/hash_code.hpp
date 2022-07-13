# ifndef CPPAD_LOCAL_OPTIMIZE_HASH_CODE_HPP
# define CPPAD_LOCAL_OPTIMIZE_HASH_CODE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*!
\file local/optimize/hash_code.hpp
CppAD hashing utility.
*/


// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize {
/*!
Specialized hash code for a CppAD operator and its arguments
(used during optimization).

\param op
is the operator that we are computing a hash code for.

\param num_arg
number of elements of arg to include in the hash code.

\param arg
is a vector of length num_arg
containing the corresponding argument indices for this operator.

\return
is a hash code that is between zero and CPPAD_HASH_TABLE_SIZE - 1.
*/

inline size_t optimize_hash_code(
    opcode_t      op      ,
    size_t        num_arg ,
    const addr_t* arg     )
{   CPPAD_ASSERT_UNKNOWN( num_arg < 4 );
    size_t prime = 1;
    size_t sum   = prime * size_t(op);
    for(size_t i = 0; i < num_arg; i++)
    {   prime    = prime + 2;  // 3, 5, 7 in that order
        sum += prime * size_t(arg[i]);
    }
    //
    return sum % CPPAD_HASH_TABLE_SIZE;
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
