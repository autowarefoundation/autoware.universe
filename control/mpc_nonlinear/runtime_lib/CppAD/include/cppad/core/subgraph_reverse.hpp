# ifndef CPPAD_CORE_SUBGRAPH_REVERSE_HPP
# define CPPAD_CORE_SUBGRAPH_REVERSE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin subgraph_reverse$$
$spell
    resize
    subgraph
    Subgraphs
    dw
    Taylor
    Bool
    const
$$

$section Reverse Mode Using Subgraphs$$

$head Syntax$$
$icode%f%.subgraph_reverse(%select_domain%)
%$$
$icode%f%.subgraph_reverse(%q%, %ell%, %col%, %dw%)
%$$
$icode%f%.clear_subgraph()
%$$

$head Purpose$$
We use $latex F : \B{R}^n \rightarrow \B{R}^m$$ to denote the
$cref/AD function/glossary/AD Function/$$ corresponding to $icode f$$.
Reverse mode computes the derivative of the $cref Forward$$ mode
$cref/Taylor coefficients/glossary/Taylor Coefficient/$$
with respect to the domain variable $latex x$$.

$head Notation$$
We use the reverse mode
$cref/notation/reverse_any/Notation/$$ with the following change:
the vector
$cref/w^(k)/reverse_any/Notation/w^(k)/$$ is defined
$latex \[
w_i^{(k)} = \left\{ \begin{array}{ll}
    1 & {\rm if} \; k = q-1 \; \R{and} \; i = \ell
    \\
    0       & {\rm otherwise}
\end{array} \right.
\] $$

$head BaseVector$$
The type $icode BaseVector$$ must be a $cref SimpleVector$$ class with
$cref/elements of type/SimpleVector/Elements of Specified Type/$$
$icode Base$$.
The routine $cref CheckSimpleVector$$ will generate an error message
if this is not the case.

$head BoolVector$$
The type $icode BoolVector$$ is a $cref SimpleVector$$ class with
$cref/elements of type/SimpleVector/Elements of Specified Type/$$
$code bool$$.

$head SizeVector$$
The type $icode SizeVector$$ is a $cref SimpleVector$$ class with
$cref/elements of type/SimpleVector/Elements of Specified Type/$$
$code size_t$$.

$head select_domain$$
The argument $icode select_domain$$ has prototype
$codei%
    const %BoolVector%& %select_domain%
%$$
It has size $latex n$$ and specifies which independent variables
to include in future $code subgraph_reverse$$ calculations.
If $icode%select_domain%[%j%]%$$ is false,
it is assumed that $latex u^{(k)}_j = 0$$ for $latex k > 0$$; i.e.,
the $th j$$ component of the Taylor coefficient for $latex x$$,
with order greater that zero, are zero; see
$cref/u^(k)/reverse_any/Notation/u^(k)/$$.

$head q$$
The argument $icode q$$ has prototype
$codei%
    size_t %q%
%$$
and specifies the number of Taylor coefficient orders to be differentiated.

$head ell$$
The argument $icode ell$$ has prototype
$codei%
    size_t %ell%
%$$
and specifies the dependent variable index that we are computing
the derivatives for; i.e. $latex \ell$$.
This index can only be used once per, and after, a call that selects
the independent variables using $icode select_domain$$.

$head col$$
This argument $icode col$$ has prototype
$codei%
    %SizeVector% %col%
%$$
The input size and value of its elements do not matter.
The $icode%col%.resize%$$ member function is used to change its size
to the number the number of possible non-zero derivative components.
For each $icode c$$,
$codei%
    %select_domain%[ %col%[%c%] ] == true
    %col%[%c%+1] >= %col%[%c%]
%$$
and the derivative with respect to the $th j$$ independent
variable is possibly non-zero where
$icode%j% = %col%[%c%]%$$.

$head dw$$
The argument $icode dw$$ has prototype
$codei%
    %Vector% %dw%
%$$
Its input size and value does not matter.
Upon return,
it is a vector with size $latex n \times q$$.
For $latex c = 0 , \ldots , %col%.size()-1$$,
and $latex k = 0, \ldots , q-1$$,
$latex \[
    dw[ j * q + k ] = W^{(1)} ( x )_{j,k}
\] $$
is the derivative of the specified Taylor coefficients w.r.t the $th j$$
independent variable where $icode%j% = %col%[%c%]%$$.
Note that this corresponds to the $cref reverse_any$$ convention when
$cref/w/reverse_any/w/$$ has size $icode%m% * %q%$$.

$head clear_subgraph$$
Calling this routine will free memory that holds
information between calls to subgraph calculations so that
it does not need to be recalculated.
(This memory is automatically freed when $icode f$$ is deleted.)
You cannot free this memory between calls that select the domain
and corresponding calls that compute reverse mode derivatives.
Some of this information is also used by $cref subgraph_sparsity$$.

$head Example$$
$children%
    example/sparse/subgraph_reverse.cpp
%$$
The file
$cref subgraph_reverse.cpp$$
contains an example and test of this operation.

$end
*/
namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file subgraph_reverse.hpp
Compute derivatvies using reverse mode and subgraphs.
*/

/// clear all subgraph information
template <class Base, class RecBase>
void ADFun<Base,RecBase>::clear_subgraph(void)
{   play_.clear_random();
    subgraph_info_.clear();
    subgraph_partial_.clear();
}

/*!
Initialize reverse mode derivative computation on subgraphs.

\param select_domain
is a vector with size equal to the dimension of the domain for this function.
Only derivatives w.r.t. the components that are true will be computed.

\par subgraph_info_.map_user_op()
If the input size of this vector is zero,
its value for this player (play_) is computed.

\par subgraph_info.in_subgraph_
This vector is initialized for a reverse mode computation on subgraphs.

\par subgraph_info.select_domain()
This vector is set equal to the select_domain argument.

\par subgraph_info.process_range()
This vector is initialized to have size Range() and its elements are false.
*/

template <class Base, class RecBase>
template <class BoolVector>
void ADFun<Base,RecBase>::subgraph_reverse( const BoolVector& select_domain )
{   using local::pod_vector;
    //
    CPPAD_ASSERT_UNKNOWN(
        dep_taddr_.size() == subgraph_info_.n_dep()
    );
    CPPAD_ASSERT_UNKNOWN(
        size_t( select_domain.size() ) == subgraph_info_.n_ind()
    );

    // map_user_op
    if( subgraph_info_.map_user_op().size() == 0 )
        subgraph_info_.set_map_user_op(&play_);
    else
    {   CPPAD_ASSERT_UNKNOWN( subgraph_info_.check_map_user_op(&play_) );
    }
    CPPAD_ASSERT_UNKNOWN(
        subgraph_info_.map_user_op().size() == play_.num_op_rec()
    );

    // initialize for reverse mode subgraph computations
    switch( play_.address_type() )
    {
        case local::play::unsigned_short_enum:
        subgraph_info_.init_rev<unsigned short>(&play_, select_domain);
        break;

        case local::play::unsigned_int_enum:
        subgraph_info_.init_rev<unsigned int>(&play_, select_domain);
        break;

        case local::play::size_t_enum:
        subgraph_info_.init_rev<size_t>(&play_, select_domain);
        break;

        default:
        CPPAD_ASSERT_UNKNOWN(false);
    }
    CPPAD_ASSERT_UNKNOWN(
        subgraph_info_.in_subgraph().size() == play_.num_op_rec()
    );

    return;
}


/*!
Use reverse mode to compute derivative of Taylor coefficients on a subgraph.

The function
\f$ X : {\bf R} \times {\bf R}^{n \times q} \rightarrow {\bf R} \f$
is defined by
\f[
X(t , u) = \sum_{k=0}^{q-1} u^{(k)} t^k
\f]
The function
\f$ Y : {\bf R} \times {\bf R}^{n \times q} \rightarrow {\bf R} \f$
is defined by
\f[
Y(t , u) = F[ X(t, u) ]
\f]
The function
\f$ W : {\bf R}^{n \times q} \rightarrow {\bf R} \f$ is defined by
\f[
W(u) = \sum_{k=0}^{q-1} ( w^{(k)} )^{\rm T}
\frac{1}{k !} \frac{ \partial^k } { t^k } Y(0, u)
\f]

\param q
is the number of Taylor coefficient we are differentiating.

\param ell
is the component of the range that is selected for differentiation.

\param col
is the set of indices j = col[c] where the return value is defined.
If an index j is not in col, then either its derivative is zero,
or it is not in select_domain.

\param dw
Is a vector \f$ dw \f$ such that
for j = col[c],
\f$ k = 0 , \ldots , q-1 \f$
\f[
    dw[ j * q + k ] = W^{(1)} ( x )_{j,k}
\f]
where the matrix \f$ x \f$ is the value for \f$ u \f$
that corresponding to the forward mode Taylor coefficients
for the independent variables as specified by previous calls to Forward.

\par subgraph_info.process_range()
The element process_range[ell] is set to true by this operation.

\par subgraph_info.in_subgraph_
some of the elements of this vector are set to have value ell
(so it can not longer be used to determine the subgraph corresponding to
the ell-th dependent variable).
*/
template <class Base, class RecBase>
template <class Addr, class BaseVector, class SizeVector>
void ADFun<Base,RecBase>::subgraph_reverse_helper(
    size_t      q   ,
    size_t      ell ,
    SizeVector& col ,
    BaseVector& dw  )
{   using local::pod_vector;
    // used to identify the RecBase type in calls to sweeps
    RecBase not_used_rec_base(0.0);
    //
    // get a random iterator for this player
    play_.template setup_random<Addr>();
    typename local::play::const_random_iterator<Addr> random_itr =
        play_.template get_random<Addr>();

    // check BaseVector is Simple Vector class with Base type elements
    CheckSimpleVector<Base, BaseVector>();
    CPPAD_ASSERT_KNOWN(
        q > 0,
        "The second argument to Reverse must be greater than zero."
    );
    CPPAD_ASSERT_KNOWN(
        num_order_taylor_ >= q,
        "Less than q Taylor coefficients are currently stored"
        " in this ADFun object."
    );
    CPPAD_ASSERT_KNOWN(
        num_direction_taylor_ == 1,
        "reverse mode for Forward(q, r, xq) with more than one direction"
        "\n(r > 1) is not yet supported."
    );
    CPPAD_ASSERT_KNOWN(
        ell < dep_taddr_.size(),
        "dependent variable index in to large for this function"
    );
    CPPAD_ASSERT_KNOWN(
        subgraph_info_.process_range()[ell] == false,
        "This dependent variable index has already been processed\n"
        "after the previous subgraph_reverse(select_domain)."
    );

    // subgraph of operators connected to dependent variable ell
    pod_vector<addr_t> subgraph;
    subgraph_info_.get_rev(
        random_itr, dep_taddr_, addr_t(ell), subgraph
    );

    // Add all the atomic function call operators
    // for calls that have first operator in the subgraph
    local::subgraph::entire_call(random_itr, subgraph);

    // First add the BeginOp and EndOp to the subgraph and then sort it
    // sort the subgraph
    addr_t i_op_begin_op = 0;
    addr_t i_op_end_op   = addr_t( play_.num_op_rec() - 1);
    subgraph.push_back(i_op_begin_op);
    subgraph.push_back(i_op_end_op);
    std::sort( subgraph.data(), subgraph.data() + subgraph.size() );
    CPPAD_ASSERT_UNKNOWN( subgraph[0] == i_op_begin_op );
    CPPAD_ASSERT_UNKNOWN( subgraph[subgraph.size()-1] == i_op_end_op );
    /*
    // Use this printout for debugging
    std::cout << "{ ";
    for(size_t k = 0; k < subgraph.size(); k++)
    {   if( k > 0 )
            std::cout << ", ";
        std::cout << subgraph[k];
    }
    std::cout << "}\n";
    */

    // initialize subgraph_partial_ matrix to zero on subgraph
    Base zero(0);
    subgraph_partial_.resize(num_var_tape_ * q);
    for(size_t k = 0; k < subgraph.size(); ++k)
    {
        size_t               i_op = size_t( subgraph[k] );
        local::OpCode        op;
        const addr_t*        arg;
        size_t               i_var;
        random_itr.op_info(i_op, op, arg, i_var);
        if( NumRes(op) == 0 )
        {   CPPAD_ASSERT_UNKNOWN(
                op == local::AFunOp  ||
                op == local::FunapOp ||
                op == local::FunavOp ||
                op == local::FunrpOp ||
                op == local::EndOp
            );
        }
        else if( op != local::BeginOp )
        {   CPPAD_ASSERT_UNKNOWN( i_var >= NumRes(op) );
            size_t j_var = i_var + 1 - NumRes(op);
            for(size_t i = j_var; i <= i_var; ++i)
            {   for(size_t j = 0; j < q; ++j)
                    subgraph_partial_[i * q + j] = zero;
            }
        }
    }

    // set partial to one for component we are differentiating
    subgraph_partial_[ dep_taddr_[ell] * q + q - 1] = Base(1);

    // evaluate the derivatives
    CPPAD_ASSERT_UNKNOWN( cskip_op_.size() == play_.num_op_rec() );
    CPPAD_ASSERT_UNKNOWN( load_op2var_.size()  == play_.num_var_load_rec() );
    size_t n = Domain();
    //
    local::play::const_subgraph_iterator<Addr> subgraph_itr =
        play_.end_subgraph(random_itr, &subgraph);
    //
    local::sweep::reverse(
        q - 1,
        n,
        num_var_tape_,
        &play_,
        cap_order_taylor_,
        taylor_.data(),
        q,
        subgraph_partial_.data(),
        cskip_op_.data(),
        load_op2var_,
        subgraph_itr,
        not_used_rec_base
    );

    // number of non-zero in return value
    size_t col_size       = 0;
    size_t subgraph_index = 0;
    CPPAD_ASSERT_UNKNOWN( subgraph[subgraph_index] == 0 );
    // Skip BeginOp
    ++subgraph_index;
    while( subgraph_index < subgraph.size() )
    {   // check for InvOp
        if( subgraph[subgraph_index] > addr_t(n) )
            subgraph_index = subgraph.size();
        else
        {   ++col_size;
            ++subgraph_index;
        }
    }
    col.resize(col_size);

    // return the derivative values
    dw.resize(n * q);
    for(size_t c = 0; c < col_size; ++c)
    {   size_t i_op = size_t( subgraph[c + 1] );
        CPPAD_ASSERT_UNKNOWN( play_.GetOp(i_op) == local::InvOp );
        //
        size_t j = i_op - 1;
        CPPAD_ASSERT_UNKNOWN( i_op == random_itr.var2op( ind_taddr_[j] ) );
        //
        // return paritial for this independent variable
        col[c] = j;
        for(size_t k = 0; k < q; k++)
            dw[j * q + k ] = subgraph_partial_[ind_taddr_[j] * q + k];
    }
    //
    CPPAD_ASSERT_KNOWN( ! ( hasnan(dw) && check_for_nan_ ) ,
        "f.subgraph_reverse(dw, q, ell): dw has a nan,\n"
        "but none of f's Taylor coefficents are nan."
    );
    //
    return;
}
/*!
\copydoc subgraph_reverse_helper

*/
template <class Base, class RecBase>
template <class BaseVector, class SizeVector>
void ADFun<Base,RecBase>::subgraph_reverse(
    size_t      q   ,
    size_t      ell ,
    SizeVector& col ,
    BaseVector& dw  )
{   using local::pod_vector;
    //
    // call proper version of helper function
    switch( play_.address_type() )
    {
        case local::play::unsigned_short_enum:
        subgraph_reverse_helper<unsigned short>(q, ell, col, dw);
        break;

        case local::play::unsigned_int_enum:
        subgraph_reverse_helper<unsigned int>(q, ell, col, dw);
        break;

        case local::play::size_t_enum:
        subgraph_reverse_helper<size_t>(q, ell, col, dw);
        break;

        default:
        CPPAD_ASSERT_UNKNOWN(false);
    }
    //
    return;
}

} // END_CPPAD_NAMESPACE
# endif
