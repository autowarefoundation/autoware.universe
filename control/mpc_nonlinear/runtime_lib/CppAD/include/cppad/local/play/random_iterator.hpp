# ifndef CPPAD_LOCAL_PLAY_RANDOM_ITERATOR_HPP
# define CPPAD_LOCAL_PLAY_RANDOM_ITERATOR_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

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
\file random_iterator.hpp
*/

/*!
Constant random iterator for a player object.

\tparam Addr
An integer type capable of representing the largest value in the vectors
arg_vec, op2arg_vec, op2var_vec, var2op_vec.
*/
template <class Addr>
class const_random_iterator {
private:
    /// vector of operators on the tape
    const pod_vector<opcode_t>* op_vec_;

    /// vector of arguments for all the operators
    /// (note that this is same type as used in recorder; i.e., addr_t)
    const pod_vector<addr_t>* arg_vec_;

    /// mapping from operator index to index of first argument in arg_vec_
    const pod_vector<Addr>* op2arg_vec_;

    /// mapping from operator index to index of primary (last) result
    const pod_vector<Addr>* op2var_vec_;

    /// mapping from primary variable index to operator index
    /// (only specified for primary variables)
    const pod_vector<Addr>* var2op_vec_;

public:
    /// default constructor
    const_random_iterator(void) :
    op_vec_(nullptr)     ,
    arg_vec_(nullptr)    ,
    op2arg_vec_(nullptr) ,
    op2var_vec_(nullptr) ,
    var2op_vec_(nullptr)
    { }
    /// default assignment operator
    void operator=(const const_random_iterator& rhs)
    {
        op_vec_          = rhs.op_vec_;
        op2arg_vec_      = rhs.op2arg_vec_;
        op2var_vec_      = rhs.op2var_vec_;
        var2op_vec_      = rhs.var2op_vec_;
        return;
    }
    /*!
    Create a random iterator

    \par var2op_vec
    This variable is not needed and can be null if the var2op member
    function is not used.
    */
    const_random_iterator(
        const pod_vector<opcode_t>&           op_vec     , ///< op_vec_
        const pod_vector<addr_t>&             arg_vec    , ///< arg_vec_
        const pod_vector<Addr>*               op2arg_vec , ///< op2ar_vec_
        const pod_vector<Addr>*               op2var_vec , ///< op2var_vec_
        const pod_vector<Addr>*               var2op_vec ) ///< var2op_vec_
    :
    op_vec_          ( &op_vec    )   ,
    arg_vec_         ( &arg_vec   )   ,
    op2arg_vec_      ( op2arg_vec )   ,
    op2var_vec_      ( op2var_vec )   ,
    var2op_vec_      ( var2op_vec )
    { }
    /*!
    \brief
    fetch the information corresponding to an operator

    \param op_index
    index for this operator [in]

    \param op [out]
    op code for this operator.

    \param op_arg [out]
    pointer to the first arguement to this operator.

    \param var_index [out]
    index of the last variable (primary variable) for this operator.
    If there is no primary variable for this operator, i_var not sepcified
    and could have any value.
    */
    void op_info(
        size_t         op_index   ,
        OpCode&        op         ,
        const addr_t*& op_arg     ,
        size_t&        var_index  ) const
    {   op        = OpCode( (*op_vec_)[op_index] );
        op_arg    = (*op2arg_vec_)[op_index] + arg_vec_->data();
        var_index = (*op2var_vec_)[op_index];
        return;
    }
    /*!
    \brief
    map variable index to operator index.

    \param var_index
    must be the index of a primary variable.

    \return
    is the index of the operator corresponding to this primary variable.
    */
    size_t var2op(size_t var_index) const
    {   // check that var2op_vec was not null in constructor
        CPPAD_ASSERT_UNKNOWN( var2op_vec_ != nullptr );
        //
        // operator index
        size_t op_index = size_t( (*var2op_vec_)[var_index] );
        //
        // check that var_index is a primary variable index (see random_setup)
        CPPAD_ASSERT_UNKNOWN( op_index < op_vec_->size() );
        //
        return op_index;
    }
    /// get operator corresponding to operator index
    OpCode get_op(size_t op_index) const
    {   return OpCode( (*op_vec_)[op_index] );
    }
    /// number of operators
    size_t num_op(void) const
    {   return op_vec_->size(); }
    //
    /// number of variables
    size_t num_var(void) const
    {   return var2op_vec_->size(); }
};

} } } // BEGIN_CPPAD_LOCAL_PLAY_NAMESPACE

# endif
