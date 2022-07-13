# ifndef CPPAD_LOCAL_SUBGRAPH_INIT_REV_HPP
# define CPPAD_LOCAL_SUBGRAPH_INIT_REV_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/subgraph/info.hpp>

// BEGIN_CPPAD_LOCAL_SUBGRAPH_NAMESPACE
namespace CppAD { namespace local { namespace subgraph {
/*!
\file init_rev.hpp
initialize for a reverse mode subgraph calculation
*/

// -----------------------------------------------------------------------
/*!
Initialize in_subgraph corresponding to a single dependent variable
(and a selected set of independent variables).

\tparam Addr
is the type used for indices in the random iterator.

\param random_itr
Is a random iterator for this operation sequence.

\param select_domain
is a vector with, size equal to the number of independent variables
in the recording. It determines the selected independent variables.

\par in_subgraph_
We use depend_yes (depend_no) for the value n_dep_ (n_dep_ + 1).
The important properties are that depend_yes < depend_no and
for a valid indpendent variable index i_ind < depend_yes.
The input size and elements of in_subgraph_ do not matter.
If in_subgraph_[i_op] == depend_yes (depend_no),
the result for this operator depends (does not depend)
on the selected independent variables.
Note that for atomic function call operators i_op,
in_subgraph[i_op] is depend_no except for the first AFunOp in the
atomic function call sequence. For the first AFunOp,
it is depend_yes (depend_no) if any of the results for the call sequence
depend (do not depend) on the selected independent variables.
Except for UserOP, only operators with NumRes(op) > 0 have in_subgraph_
value depend_yes;
e.g., comparison operators have in_subgraph_ value depend_no.

\par select_domain_
This vector is is set equal to the select_domain argument.

\par process_range_
This vector is to to size n_dep_ and its values are set to false
*/
template <class Addr, class BoolVector>
void subgraph_info::init_rev(
    const local::play::const_random_iterator<Addr>&  random_itr    ,
    const BoolVector&                                select_domain )
{
    // check sizes
    CPPAD_ASSERT_UNKNOWN( map_user_op_.size()   == n_op_ );
    CPPAD_ASSERT_UNKNOWN( random_itr.num_op()   == n_op_ );
    CPPAD_ASSERT_UNKNOWN( size_t( select_domain.size() ) == n_ind_ );

    // depend_yes and depend_no
    addr_t depend_yes = addr_t( n_dep_ );
    addr_t depend_no  = addr_t( n_dep_ + 1 );

    // select_domain_
    select_domain_.resize(n_ind_);
    for(size_t j = 0; j < n_ind_; ++j)
        select_domain_[j]  = select_domain[j];

    // process_range_
    process_range_.resize(n_dep_);
    for(size_t i = 0; i < n_dep_; ++i)
        process_range_[i] = false;

    // set in_subgraph to have proper size
    in_subgraph_.resize(n_op_);

    // space used to return set of arguments that are variables
    pod_vector<size_t> argument_variable;

    // temporary space used by get_argument_variable
    pod_vector<bool> work;

# ifndef NDEBUG
    size_t count_independent = 0;
# endif
    bool begin_atomic_call = false;
    for(size_t i_op = 0; i_op < n_op_; ++i_op)
    {   OpCode op = random_itr.get_op(i_op);
        //
        // default value for this operator
        in_subgraph_[i_op] = depend_no;
        //
        switch(op)
        {   case InvOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0 );
            CPPAD_ASSERT_UNKNOWN( i_op > 0 );
            {   // get user index for this independent variable
                size_t j = i_op - 1;
                CPPAD_ASSERT_UNKNOWN( j < n_ind_ );
                //
                // set in_subgraph_[i_op]
                if( select_domain[j] )
                    in_subgraph_[i_op] = depend_yes;
            }
# ifndef NDEBUG
            ++count_independent;
# endif
            break;

            // only mark both first AFunOp for each call as depending
            // on the selected independent variables
            case AFunOp:
            begin_atomic_call  = ! begin_atomic_call;
            if( begin_atomic_call )
            {   get_argument_variable(random_itr, i_op, argument_variable, work);
                for(size_t j = 0; j < argument_variable.size(); ++j)
                {   size_t j_var = argument_variable[j];
                    size_t j_op  = random_itr.var2op(j_var);
                    j_op         = size_t( map_user_op_[j_op] );
                    CPPAD_ASSERT_UNKNOWN( j_op < i_op );
                    if( in_subgraph_[j_op] == depend_yes )
                        in_subgraph_[i_op] =  depend_yes;
                }
            }
            break;

            // skip FunrvOp (gets mapped to first AFunOp in this call)
            case FunrvOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0 );
            break;

            default:
            // Except for AFunOp, only include when NumRes(op) > 0.
            if( NumRes(op) > 0 )
            {   get_argument_variable(random_itr, i_op, argument_variable, work);
                for(size_t j = 0; j < argument_variable.size(); ++j)
                {   size_t j_var = argument_variable[j];
                    size_t j_op  = random_itr.var2op(j_var);
                    j_op         = size_t( map_user_op_[j_op] );
                    CPPAD_ASSERT_UNKNOWN( j_op < i_op );
                    if( in_subgraph_[j_op] == depend_yes )
                        in_subgraph_[i_op] =  depend_yes;
                }
            }
            break;
        }
    }
    CPPAD_ASSERT_UNKNOWN(
        count_independent == size_t(select_domain.size())
    );
    //
    return;
}
// -----------------------------------------------------------------------
/*!
Initialize in_subgraph corresponding to a single dependent variable
(and a selected set of independent variables).

\tparam Addr
is the type used for indices in the random iterator.

\tparam Base
this recording was made using ADFun<Base>

\param play
is a player for this ADFun<Base> object.

\param select_domain
is a vector with, size equal to the number of independent variables
in the recording. It determines the selected independent variables.

\par in_subgraph_
We use depend_yes (depend_no) for the value n_dep_ (n_dep_ + 1).
The important properties are that depend_yes < depend_no and
for a valid indpendent variable index i_ind < depend_yes.
The input size and elements of in_subgraph_ do not matter.
If in_subgraph_[i_op] == depend_yes (depend_no),
the result for this operator depends (does not depend)
on the selected independent variables.
Note that for atomic function call operators i_op,
in_subgraph[i_op] is depend_no except for the first AFunOp in the
atomic function call sequence. For the first AFunOp,
it is depend_yes (depend_no) if any of the results for the call sequence
depend (do not depend) on the selected independent variables.
Except for UserOP, only operators with NumRes(op) > 0 have in_subgraph_
value depend_yes;
e.g., comparison operators have in_subgraph_ value depend_no.

\par select_domain_
This vector is is set equal to the select_domain argument.

\par process_range_
This vector is to to size n_dep_ and its values are set to false
*/
template <class Addr, class Base, class BoolVector>
void subgraph_info::init_rev(
    player<Base>*       play          ,
    const BoolVector&   select_domain )
{
    // get random access iterator for this player
    play->template setup_random<Addr>();
    local::play::const_random_iterator<Addr> random_itr =
        play->template get_random<Addr>();
    //
    init_rev(random_itr, select_domain);
    //
    return;
}


} } } // END_CPPAD_LOCAL_SUBGRAPH_NAMESPACE

# endif
