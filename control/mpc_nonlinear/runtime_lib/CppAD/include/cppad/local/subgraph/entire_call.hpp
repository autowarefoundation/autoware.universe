# ifndef CPPAD_LOCAL_SUBGRAPH_ENTIRE_CALL_HPP
# define CPPAD_LOCAL_SUBGRAPH_ENTIRE_CALL_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/pod_vector.hpp>

// BEGIN_CPPAD_LOCAL_SUBGRAPH_NAMESPACE
namespace CppAD { namespace local { namespace subgraph {
/*!
\file entire_call.hpp
include entire function call in a subgraph
*/
// ===========================================================================
/*!
Convert from just firt AFunOp to entire atomic function call in a subgraph.

\tparam Addr
Type used for indices in the random iterator.

\param random_itr
is a random iterator for this operation sequence.

\param subgraph
It a set of operator indices in this recording.
If the corresponding operator is a AFunOp, it assumed to be the
first one in the corresponding atomic function call.
The other call operators are included in the subgraph.
*/
template <class Addr>
void entire_call(
    const play::const_random_iterator<Addr>& random_itr ,
    pod_vector<addr_t>&                      subgraph   )
{
    // add extra operators corresponding to rest of atomic function calls
    size_t n_sub = subgraph.size();
    for(size_t k = 0; k < n_sub; ++k)
    {   size_t i_op = size_t( subgraph[k] );
        //
        if( random_itr.get_op(i_op) == AFunOp )
        {   // This is the first AFunOp of this atomic function call
            while( random_itr.get_op(++i_op) != AFunOp )
            {   switch(random_itr.get_op(i_op))
                {
                    case FunavOp:
                    case FunrvOp:
                    case FunrpOp:
                    case FunapOp:
                    subgraph.push_back( addr_t(i_op) );
                    break;

                    default:
                    // cannot find second AFunOp in this call
                    CPPAD_ASSERT_UNKNOWN(false);
                    break;
                }
            }
            // THis is the second AFunOp of this atomic function call
            subgraph.push_back( addr_t(i_op) );
        }
    }

}

} } } // END_CPPAD_LOCAL_SUBGRAPH_NAMESPACE

# endif
