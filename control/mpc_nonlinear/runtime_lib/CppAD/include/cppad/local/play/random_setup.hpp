# ifndef CPPAD_LOCAL_PLAY_RANDOM_SETUP_HPP
# define CPPAD_LOCAL_PLAY_RANDOM_SETUP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// BEGIN_CPPAD_LOCAL_PLAY_NAMESPACE
namespace CppAD { namespace local { namespace play {

/*!
\file random_setup.hpp
*/

/*!
Set up random access to a player object.

\tparam Addr
An integer type capable of representing the largest value in the vectors
arg_vec, op2arg_vec, op2var_vec, var2op_vec.

\param num_var
num_var is the number of variables in this operation sequence.

\param op_vec
The mapping
<code>op = OpCode[ op_vec[op_index] ]</code>
maps from operator index op_index to the operator op.

\param arg_vec
is a vector of all the arguments for all the operators.
The mapping op2arg_vec will map from operator indices
to index in this vector.

\param op2arg_vec
On input, op2arg_vec is either the empty vector
(or contains the proper result from a previous call to random_setup).
Upon return it maps each operator index to the index in op_arg_vec of its
first argument for the operator.

\param op2var_vec
On input, op2var_vec is either the empty vector
(or contains the proper result from a previous call to random_setup).
Upon return it maps each operator index to the primary (last)
result for the operator. If there are no results for the operator,
the return value map value is not specified.

\param var2op_vec
On input, var2op_vec is either the empty vector
(or contains the proper result from a previous call to random_setup).
Upon return it maps each primary variable index to the corresponding
operator index. The value of the map is only specifed for primary variable
indices.
*/
template <class Addr>
void random_setup(
    size_t                                    num_var    ,
    const pod_vector<opcode_t>&               op_vec     ,
    const pod_vector<addr_t>&                 arg_vec    ,
    pod_vector<Addr>*                         op2arg_vec ,
    pod_vector<Addr>*                         op2var_vec ,
    pod_vector<Addr>*                         var2op_vec )
{
    if( op2arg_vec->size() != 0 )
    {   CPPAD_ASSERT_UNKNOWN( op2arg_vec->size() == op_vec.size() );
        CPPAD_ASSERT_UNKNOWN( op2var_vec->size() == op_vec.size() );
        CPPAD_ASSERT_UNKNOWN( var2op_vec->size() == num_var        );
        return;
    }
    CPPAD_ASSERT_UNKNOWN( op2var_vec->size() == 0         );
    CPPAD_ASSERT_UNKNOWN( op2var_vec->size() == 0         );
    CPPAD_ASSERT_UNKNOWN( var2op_vec->size() == 0         );
    CPPAD_ASSERT_UNKNOWN( OpCode( op_vec[0] ) == BeginOp );
    CPPAD_ASSERT_NARG_NRES(BeginOp, 1, 1);
    //
    size_t num_op     = op_vec.size();
    size_t  var_index = 0;
    size_t  arg_index = 0;
    //
    op2arg_vec->resize( num_op );
    op2var_vec->resize( num_op );
    var2op_vec->resize( num_var  );
# ifndef NDEBUG
    // value of var2op for auxillary variables is num_op (invalid)
    for(size_t i_var = 0; i_var < num_var; ++i_var)
        (*var2op_vec)[i_var] = Addr( num_op );
    // value of op2var is num_var (invalid) when NumRes(op) = 0
    for(size_t i_op = 0; i_op < num_op; ++i_op)
        (*op2var_vec)[i_op] = Addr( num_var );
# endif
    for(size_t i_op = 0; i_op < num_op; ++i_op)
    {   OpCode  op          = OpCode( op_vec[i_op] );
        //
        // index of first argument for this operator
        (*op2arg_vec)[i_op]   = Addr( arg_index );
        arg_index            += NumArg(op);
        //
        // index of first result for next operator
        var_index  += NumRes(op);
        if( NumRes(op) > 0 )
        {   // index of last (primary) result for this operator
            (*op2var_vec)[i_op] = Addr( var_index - 1 );
            //
            // mapping from primary variable to its operator
            (*var2op_vec)[var_index - 1] = Addr( i_op );
        }
        // CSumOp
        if( op == CSumOp )
        {   CPPAD_ASSERT_UNKNOWN( NumArg(CSumOp) == 0 );
            //
            // pointer to first argument for this operator
            const addr_t* op_arg = arg_vec.data() + arg_index;
            //
            // The actual number of arugments for this operator is
            // op_arg[4] + 1
            // Correct index of first argument for next operator
            arg_index += size_t(op_arg[4] + 1);
        }
        //
        // CSkip
        if( op == CSkipOp )
        {   CPPAD_ASSERT_UNKNOWN( NumArg(CSumOp) == 0 );
            //
            // pointer to first argument for this operator
            const addr_t* op_arg = arg_vec.data() + arg_index;
            //
            // The actual number of arugments for this operator is
            // 7 + op_arg[4] + op_arg[5].
            // Correct index of first argument for next operator.
            arg_index += size_t(7 + op_arg[4] + op_arg[5]);
        }
    }
}

} } } // BEGIN_CPPAD_LOCAL_PLAY_NAMESPACE

# endif
