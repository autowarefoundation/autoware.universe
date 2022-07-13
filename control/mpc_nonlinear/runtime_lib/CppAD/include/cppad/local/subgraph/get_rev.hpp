# ifndef CPPAD_LOCAL_SUBGRAPH_GET_REV_HPP
# define CPPAD_LOCAL_SUBGRAPH_GET_REV_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

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

// BEGIN_CPPAD_LOCAL_SUBGRAPH_NAMESPACE
namespace CppAD { namespace local { namespace subgraph {
/*!
\file get_rev.hpp
Get subgraph corresponding to a dependent variable.
*/

// ===========================================================================
/*!
Get the subgraph corresponding to a dependent variables
(and a selected set of independent variables).

\tparam Addr
Type used for indices in the random iterator.

\param random_itr
is a random iterator for this operation sequence.

\param dep_taddr
is the vector mapping user dependent variable indices
to the correpsonding variable in the recording.

\param i_dep
is the user index for his dependent variable;
that i_dep < n_dep_.

\param subgraph
the input size and contents of this vector do not matter.
Repeated calls with the same subgraph vector should reduce
the amount of memory allocation.
Upon return it contains the operator indices for the subgraph
corresponding to the dependent and the selected independent variables.
Only selected independent variable operators InvOp are included
in the subgraph.
Furthermore the operator indices in subgraph are unique; i.e.,
if i_op != j_op then subgraph[i_op] != subgraph[j_op].

\par map_user_op_
This vector must be set.

\par in_subgraph_
has size equal to the number of operators in play.
If in_subgraph[i_op] <= n_dep_,
the result for this operator depends on the selected independent variables.
In addition, upon input, there is no i_op such that in_subgraph[i_op] == i_dep.
Note that for atomic function call operators i_op,
\code
    n_dep_ < in_subgraph[i_op]
\endcode
except for the first AFunOp in the atomic function call sequence.
For the first AFunOp,
\code
    in_subgraph[i_op] <= n_dep_
\endcode
if any result for the atomic function call
depends on the selected independent variables.
Except for UserOP, only operators with NumRes(op) > 0 are included
in the dependency; e.g., comparison operators are not included.
Upon return, some of the i_op for which in_subgraph[i_op] <= n_dep_,
will be changed to in_subgraph[i_op] = i_dep.

\par process_range_
The value process_range_[i_dep] is checked to make sure it is false.
It is then set to have value true.
*/
template <class Addr>
void subgraph_info::get_rev(
    const play::const_random_iterator<Addr>&   random_itr  ,
    const pod_vector<size_t>&                  dep_taddr   ,
    addr_t                                     i_dep       ,
    pod_vector<addr_t>&                        subgraph    )
{   // check sizes
    CPPAD_ASSERT_UNKNOWN( map_user_op_.size()   == n_op_ );

    // process_range_
    CPPAD_ASSERT_UNKNOWN( process_range_[i_dep] == false );
    process_range_[i_dep] = true;

    // special value; see init_rev_in_subgraph
    addr_t depend_yes = addr_t( n_dep_ );

    // assumption on i_dep
    CPPAD_ASSERT_UNKNOWN( i_dep < depend_yes );

    // start with an empty subgraph for this dependent variable
    subgraph.resize(0);

    // tape index corresponding to this dependent variable
    size_t i_var = dep_taddr[i_dep];

    // operator corresponding to this dependent variable
    size_t i_op = random_itr.var2op(i_var);
    i_op        = size_t( map_user_op_[i_op] );

    // if this variable depends on the selected indepent variables
    // process its subgraph
    CPPAD_ASSERT_UNKNOWN( in_subgraph_[i_op] != i_dep )
    if( in_subgraph_[i_op] <= depend_yes )
    {   subgraph.push_back( addr_t(i_op) );
        in_subgraph_[i_op] = i_dep;
    }

    // space used to return set of arguments that are variables
    pod_vector<size_t> argument_variable;

    // temporary space used by get_argument_variable
    pod_vector<bool> work;

    // scan all the operators in this subgraph
    size_t sub_index = 0;
    while(sub_index < subgraph.size() )
    {   // this operator connected to this dependent and selected independent
        i_op = size_t( subgraph[sub_index] );
        CPPAD_ASSERT_UNKNOWN( in_subgraph_[i_op] == i_dep );
        //
        // There must be a result for this operator
# ifndef NDEBUG
        OpCode op = random_itr.get_op(i_op);
        CPPAD_ASSERT_UNKNOWN(op == AFunOp || NumRes(op) > 0 );
# endif
        //
        // which variables are connected to this operator
        get_argument_variable(random_itr, i_op, argument_variable, work);
        for(size_t j = 0; j < argument_variable.size(); ++j)
        {   // add the corresponding operators to the subgraph
            size_t j_var = argument_variable[j];
            size_t j_op  = random_itr.var2op(j_var);
            j_op         = size_t( map_user_op_[j_op] );
            bool add = in_subgraph_[j_op] <= depend_yes;
            add     &= in_subgraph_[j_op] != i_dep;
            if( random_itr.get_op(j_op) == InvOp )
            {   CPPAD_ASSERT_UNKNOWN( j_op == j_var );
                add &= select_domain_[j_var - 1];
            }
            if( add )
            {   subgraph.push_back( addr_t(j_op) );
                in_subgraph_[j_op] = i_dep;
            }
        }
        // we are done scaning this subgraph operator
        ++sub_index;
    }
}

} } } // END_CPPAD_LOCAL_SUBGRAPH_NAMESPACE

# endif
