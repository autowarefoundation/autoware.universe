# ifndef CPPAD_LOCAL_SUBGRAPH_SPARSITY_HPP
# define CPPAD_LOCAL_SUBGRAPH_SPARSITY_HPP
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
# include <cppad/local/subgraph/arg_variable.hpp>
# include <cppad/local/subgraph/info.hpp>
# include <cppad/local/subgraph/entire_call.hpp>

// BEGIN_CPPAD_LOCAL_SUBGRAPH_NAMESPACE
namespace CppAD { namespace local { namespace subgraph {
/*!
\file sparsity.hpp
Compute dependency sparsity pattern using subgraph technique.
*/
// ===========================================================================
/*!
Compute dependency sparsity pattern for an ADFun<Base> function.

\tparam Addr
type used for indices in random iterator
(must correspond to play->addr_type())

\tparam Base
the operation sequence was recorded using AD<Base>.

\tparam BoolVector
a simple vector class with elements of type bool.

\param play
is the operation sequence corresponding to the ADFun<Base> function.
It is effectively const except that play->setup_random() is called.


\param sub_info
is the subgraph information for this ADFun object.

\param dep_taddr
mapping from user dependent variable index to variable index in play
(must have size sub_info.n_dep()).

\param select_domain
only the selected independent variables will be included in the sparsity
pattern (must have size sub_info.n_ind()).

\param select_range
only the selected dependent variables will be included in the sparsity pattern
(must have size sub_info.n_dep()).

\param row_out
The input size and elements of row_out do not matter.
We use number of non-zeros (nnz) to denote the number of elements
in row_out. For k = 0 , ... , nnz-1, row_out[k] is the row index
of the k-th no-zero element of the dependency sparsitiy pattern for
the function corresponding to the recording.
\code
    0 <= row_out[k] < dep_taddr.size()
    select_range[ row_out[k] ] == true
\endcode

\param col_out
The input size and elements of col_out do not matter.
Upon return is has the same size as row_out; i.e., nnz.
For k = 0 , ... , nnz-1, col_out[k] is the column index
of the k-th no-zero element of the dependency sparsitiy pattern for
the function corresponding to the recording.
\code
    0 <= col_out[k] < sub_info.n_ind()
    select_domain[ col_out[k] ] == true
\endcode

\par AFunOp
All of the inputs and outputs for an atomic function call are considered
to be connected.
2DO: It would be good to use the sparsity patters for atomic function calls
to to make the sparsity pattern more efficient.
*/

template <class Addr, class Base, class BoolVector>
void subgraph_sparsity(
    player<Base>*                              play          ,
    subgraph_info&                             sub_info      ,
    const pod_vector<size_t>&                  dep_taddr     ,
    const BoolVector&                          select_domain ,
    const BoolVector&                          select_range  ,
    pod_vector<size_t>&                        row_out       ,
    pod_vector<size_t>&                        col_out       )
{
    // get random access iterator for this player
    play->template setup_random<Addr>();
    local::play::const_random_iterator<Addr> random_itr =
        play->template get_random<Addr>();

    // check dimension assumptions
    CPPAD_ASSERT_UNKNOWN(
        dep_taddr.size() == sub_info.n_dep()
    );
    CPPAD_ASSERT_UNKNOWN(
        size_t(select_domain.size()) == sub_info.n_ind()
    );
    CPPAD_ASSERT_UNKNOWN(
        size_t(select_range.size()) == sub_info.n_dep()
    );

    // number of dependent variables
    size_t n_dep = dep_taddr.size();
    CPPAD_ASSERT_UNKNOWN( size_t(select_range.size()) == n_dep );

    // start with an empty sparsity pattern
    row_out.resize(0);
    col_out.resize(0);

    // map_user_op
    if( sub_info.map_user_op().size() == 0 )
        sub_info.set_map_user_op(play);
    else
    {   CPPAD_ASSERT_UNKNOWN( sub_info.check_map_user_op(play) );
    }
    CPPAD_ASSERT_UNKNOWN(
            sub_info.map_user_op().size() == play->num_op_rec()
    );

    // subgraph of operators that are are connected to one of the selected
    // dependent variables and depend on the selected independent variables
    pod_vector<addr_t> subgraph;

    // initialize a reverse mode subgraph calculation
    sub_info.init_rev(random_itr, select_domain);
    CPPAD_ASSERT_UNKNOWN(
        sub_info.in_subgraph().size() == play->num_op_rec()
    );
    //
# ifndef NDEBUG
    addr_t depend_yes = addr_t( n_dep );
# endif

    // for each of the selected dependent variables
# ifndef NDEBUG
    addr_t depend_no  = addr_t( n_dep + 1 );
# endif
    CPPAD_ASSERT_UNKNOWN( depend_yes < depend_no );
    CPPAD_ASSERT_UNKNOWN( NumRes(BeginOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(InvOp) == 1 );
    for(size_t i_dep = 0; i_dep < n_dep; ++i_dep) if( select_range[i_dep] )
    {   CPPAD_ASSERT_UNKNOWN( i_dep < size_t( depend_yes ) );
        //
        // subgraph of operators connected to i_dep
        sub_info.get_rev(
            random_itr, dep_taddr, addr_t(i_dep), subgraph
        );
        //
        for(size_t k = 0; k < subgraph.size(); k++)
        {   size_t i_op = size_t( subgraph[k] );
            //
            // operator corresponding to this index
            OpCode op = play->GetOp(i_op);
            //
            // This version of the subgraph only has first AFunOp
            // for each atomic functionc all.
            CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0 || op == AFunOp );
            //
            // independent variable entries correspond to sparsity pattern
            if( op == InvOp )
            {   CPPAD_ASSERT_NARG_NRES(op, 0, 1);
                // i_var is equal i_op becasue BeginOp and InvOp have 1 result
                size_t i_var = i_op;       // tape index for this variable
                size_t i_ind = i_var - 1;  // user index for this variable
                CPPAD_ASSERT_UNKNOWN( random_itr.var2op(i_var) == i_op );
                CPPAD_ASSERT_UNKNOWN( select_domain[i_ind] );
                //
                // put this pair in the sparsity pattern
                row_out.push_back(i_dep);
                col_out.push_back(i_ind);
            }
        }
    }
}

} } } // END_CPPAD_LOCAL_SUBGRAPH_NAMESPACE

# endif
