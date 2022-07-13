# ifndef CPPAD_LOCAL_SUBGRAPH_INFO_HPP
# define CPPAD_LOCAL_SUBGRAPH_INFO_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/play/random_iterator.hpp>
# include <cppad/local/pod_vector.hpp>
# include <cppad/local/subgraph/arg_variable.hpp>

// BEGIN_CPPAD_LOCAL_SUBGRAPH_NAMESPACE
namespace CppAD { namespace local { namespace subgraph {
/*!
\file info.hpp
subgraph information attached to a operation sequence
*/

/// class for maintaining subgraph information attached to on ADFun object.
class subgraph_info {
private:
    // -----------------------------------------------------------------------
    // private member data set by constructor, resize, and assign
    // -----------------------------------------------------------------------
    /// number of independent variables for this function
    size_t  n_ind_;

    /// number of dependent variables for this function
    size_t  n_dep_;

    /// number of operatros in operation sequence
    size_t  n_op_;

    /// number of variables in operation sequence
    size_t n_var_;

    // -----------------------------------------------------------------------
    // private member data set by set_map_user_op
    // -----------------------------------------------------------------------

    /// Mapping atomic call operators to AFunOp that begins call sequence,
    /// other operators are not changed by the map.
    /// (size zero after construtor or resize)
    pod_vector<addr_t> map_user_op_;

    // -----------------------------------------------------------------------
    // other private member data
    // -----------------------------------------------------------------------

    /// flags which operatiors are in subgraph
    /// (size zero or n_op_).
    pod_vector<addr_t> in_subgraph_;

    /// flags which dependent variables are selected
    pod_vector<bool> select_domain_;

    /// flags which dependent variables have been processed since
    /// the previous init_rev
    pod_vector<bool> process_range_;

public:
    // -----------------------------------------------------------------------
    // const public functions
    // -----------------------------------------------------------------------
    /// number of independent variables
    size_t n_ind(void) const
    {   return n_ind_; }

    /// number of dependent variables
    size_t n_dep(void) const
    {   return n_dep_; }

    /// number of operators
    size_t n_op(void) const
    {   return n_op_; }

    // number of variables
    size_t n_var(void) const
    {   return n_var_; }

    /// map atomic function calls to first operator in the call
    const pod_vector<addr_t>& map_user_op(void) const
    {   return map_user_op_; }

    /// previous select_domain argument to init_rev
    const pod_vector<bool>& select_domain(void) const
    {   return select_domain_; }

    /// dependent variables that have been processed since previous init_rev
    const pod_vector<bool>& process_range(void) const
    {   return process_range_; }

    /// amount of memory corresonding to this object
    size_t memory(void) const
    {   size_t sum = map_user_op_.size()   * sizeof(addr_t);
        sum       += in_subgraph_.size()   * sizeof(addr_t);
        sum       += select_domain_.size() * sizeof(bool);
        sum       += process_range_.size() * sizeof(bool);
        return sum;
    }

    /// free memory used for calculating subgraph
    void clear(void)
    {   map_user_op_.clear();
        in_subgraph_.clear();
        select_domain_.clear();
        process_range_.clear();
    }
    // -----------------------------------------------------------------------
    /*!
    check that the value of map_user_op is OK for this operation sequence

    \param play
    is the player for this operation sequence.

    \return
    is true, if map_user_op has the correct value for this operation sequence
    (is the same as it would be after a set_map_user_op).
    */
    template <class Base>
    bool check_map_user_op(const player<Base>* play) const
    {   if( map_user_op_.size() != n_op_ )
            return false;
        bool   ok   = true;
        size_t i_op = 0;
        while( i_op < n_op_ )
        {   OpCode op = play->GetOp(i_op);
            ok       &= map_user_op_[i_op] == addr_t( i_op );
            if( op == AFunOp )
            {   addr_t begin = addr_t( i_op );
                op           = play->GetOp(++i_op);
                while( op != AFunOp )
                {   CPPAD_ASSERT_UNKNOWN(
                    op==FunapOp || op==FunavOp || op==FunrpOp || op==FunrvOp
                    );
                    ok  &= map_user_op_[i_op] == begin;
                    op   = play->GetOp(++i_op);
                }
                ok  &= map_user_op_[i_op] == begin;
            }
            ++i_op;
        }
        return ok;
    }
    // -----------------------------------------------------------------------
    // non const public functions
    // -----------------------------------------------------------------------

    /// flag which operators that are in the subgraph
    pod_vector<addr_t>& in_subgraph(void)
    {   return in_subgraph_; }


    /// default constructor (all sizes are zero)
    subgraph_info(void)
    : n_ind_(0), n_dep_(0), n_op_(0), n_var_(0)
    {   CPPAD_ASSERT_UNKNOWN( map_user_op_.size()   == 0 );
        CPPAD_ASSERT_UNKNOWN( in_subgraph_.size()   == 0 );
    }
    // -----------------------------------------------------------------------
    /// assignment operator
    void operator=(const subgraph_info& info)
    {   n_ind_            = info.n_ind_;
        n_dep_            = info.n_dep_;
        n_op_             = info.n_op_;
        n_var_            = info.n_var_;
        map_user_op_      = info.map_user_op_;
        in_subgraph_      = info.in_subgraph_;
        select_domain_    = info.select_domain_;
        process_range_    = info.process_range_;
        return;
    }
    // -----------------------------------------------------------------------
    /// swap
    /// (used for move semantics version of ADFun assignment)
    void swap(subgraph_info& info)
    {   // size_t objects
        std::swap(n_ind_ , info.n_ind_);
        std::swap(n_dep_ , info.n_dep_);
        std::swap(n_op_  , info.n_op_);
        std::swap(n_var_ , info.n_var_);
        //
        // pod_vectors
        map_user_op_.swap(   info.map_user_op_);
        in_subgraph_.swap(   info.in_subgraph_);
        select_domain_.swap( info.select_domain_);
        process_range_.swap( info.process_range_);
        //
        return;
    }
    // -----------------------------------------------------------------------
    /*!
    set sizes for this object (the default sizes are zero)

    \param n_ind
    number of indepent variables.

    \param n_dep
    number of dependent variables.

    \param n_op
    number of operators.

    \param n_var
    number of variables.

    \par map_user_op_
    is resized to zero.

    \par in_subgraph_
    is resized to zero.
    */
    void resize(size_t n_ind, size_t n_dep, size_t n_op, size_t n_var)
    {   CPPAD_ASSERT_UNKNOWN(
            n_op <= size_t( std::numeric_limits<addr_t>::max() )
        );
        // n_ind_
        n_ind_ = n_ind;
        // n_dep_
        n_dep_ = n_dep;
        // n_op_
        n_op_  = n_op;
        // n_var_
        n_var_ = n_var;

        //
        // map_user_op_
        map_user_op_.resize(0);
        //
        // in_subgraph_
        in_subgraph_.resize(0);
        //
        return;
    }
    // -----------------------------------------------------------------------
    /*!
    set the value of map_user_op for this operation sequence

    \param play
    is the player for this operation sequence. It must same number of
    operators and variables as this subgraph_info object.

    \par map_user_op_
    This size of map_user_op_ must be zero when this function is called
    (which is true after a resize operation).
    This function sets its size to the number of operations in play.
    We use the term user OpCocde for the any one of the following:
    AFunOp, FunapOp, FunavOp, FunrpOp, or FunrvOp. Suppose
    \code
        OpCodce op_i = play->GetOp(i_op);
        size_t  j_op = map_user_op[i_op];
        OpCode  op_j = play->GetOP(j_op);
    \endcode
    If op is a user OpCode, j_op is the index of the first operator
    in the corresponding atomic function call and op_j == AFunOp.
    Otherwise j_op == i_op;

    */
    template <class Base>
    void set_map_user_op(const player<Base>* play)
    {   CPPAD_ASSERT_UNKNOWN( map_user_op_.size()   == 0 );
        //
        CPPAD_ASSERT_UNKNOWN( n_op_  == play->num_op_rec() );
        CPPAD_ASSERT_UNKNOWN( n_var_ == play->num_var_rec() );
        //
        // resize map_user_op_
        map_user_op_.resize(n_op_);
        //
        // set map_user_op for each operator
        for(size_t i_op = 0; i_op < n_op_; ++i_op)
        {   // this operator
            OpCode op = play->GetOp(i_op);
            //
            // value of map_user_op when op is not in atomic function call)
            map_user_op_[i_op] = addr_t( i_op );
            //
            if( op == AFunOp )
            {   // first AFunOp in an atomic function call sequence
                //
                // All operators in this atomic call sequence will be
                // mapped to the AFunOp that begins this call.
                addr_t begin = addr_t( i_op );
                op           = play->GetOp(++i_op);
                while( op != AFunOp )
                {   CPPAD_ASSERT_UNKNOWN(
                    op==FunapOp || op==FunavOp || op==FunrpOp || op==FunrvOp
                    );
                    // map this operator to the beginning of the call
                    map_user_op_[i_op] = begin;
                    op                 = play->GetOp(++i_op);
                }
                // map the second AFunOp to the beginning of the call
                map_user_op_[i_op] = begin;
            }
        }
        return;
    }
    // -----------------------------------------------------------------------
    // see init_rev.hpp
    template <class Addr, class BoolVector>
    void init_rev(
        const play::const_random_iterator<Addr>& random_itr ,
        const BoolVector&                        select_domain
    );
    template <class Addr, class Base, class BoolVector>
    void init_rev(
        player<Base>*        play          ,
        const BoolVector&    select_domain
    );
    // -----------------------------------------------------------------------
    // see get_rev.hpp
    template <class Addr>
    void get_rev(
        const play::const_random_iterator<Addr>&   random_itr   ,
        const pod_vector<size_t>&                  dep_taddr    ,
        addr_t                                     i_dep        ,
        pod_vector<addr_t>&                        subgraph
    );
};

} } } // END_CPPAD_LOCAL_SUBGRAPH_NAMESPACE

// routines that operate on in_subgraph
# include <cppad/local/subgraph/init_rev.hpp>
# include <cppad/local/subgraph/get_rev.hpp>

# endif
