# ifndef CPPAD_CORE_SUBGRAPH_SPARSITY_HPP
# define CPPAD_CORE_SUBGRAPH_SPARSITY_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin subgraph_sparsity$$
$spell
    const
    subgraph
    subgraphs
    rc
    Jacobian
    bool
$$

$section Subgraph Dependency Sparsity Patterns$$

$head Syntax$$
$icode%f%.subgraph_sparsity(
    %select_domain%, %select_range%, %transpose%, %pattern_out%
)%$$

$head See Also$$
$cref/clear_subgraph/subgraph_reverse/clear_subgraph/$$.

$head Notation$$
We use $latex F : \B{R}^n \rightarrow \B{R}^m$$ to denote the
$cref/AD function/glossary/AD Function/$$ corresponding to
the operation sequence stored in $icode f$$.


$head Method$$
This routine uses a subgraph technique. To be specific,
for each dependent variable,
it creates a subgraph of the operation sequence
containing the variables that affect the dependent variable.
This avoids the overhead of performing set operations
that is inherent in other methods for computing sparsity patterns.

$head Atomic Function$$
The sparsity calculation for
$cref/atomic functions/atomic_two_afun/$$ in the $icode f$$ operation sequence
are not efficient. To be specific, each atomic function is treated as if
all of its outputs depend on all of its inputs.
This may be improved upon in the future; see the
$cref/subgraph sparsity/wish_list/Subgraph/Sparsity/$$
wish list item.

$head BoolVector$$
The type $icode BoolVector$$ is a $cref SimpleVector$$ class with
$cref/elements of type/SimpleVector/Elements of Specified Type/$$
$code bool$$.

$head SizeVector$$
The type $icode SizeVector$$ is a $cref SimpleVector$$ class with
$cref/elements of type/SimpleVector/Elements of Specified Type/$$
$code size_t$$.

$head f$$
The object $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$

$head select_domain$$
The argument $icode select_domain$$ has prototype
$codei%
    const %BoolVector%& %select_domain%
%$$
It has size $latex n$$ and specifies which independent variables
to include in the calculation.
If not all the independent variables are included in the calculation,
a forward pass on the operation sequence is used to determine which
nodes may be in the subgraphs.

$head select_range$$
The argument $icode select_range$$ has prototype
$codei%
    const %BoolVector%& %select_range%
%$$
It has size $latex m$$ and specifies which components of the range
to include in the calculation.
A subgraph is built for each dependent variable
and the selected set of independent variables.

$head transpose$$
This argument has prototype
$codei%
    bool %transpose%
%$$
If $icode transpose$$ it is false (true),
upon return $icode pattern_out$$ is a sparsity pattern for
$latex J(x)$$ ($latex J(x)^\R{T}$$) defined below.

$head pattern_out$$
This argument has prototype
$codei%
    sparse_rc<%SizeVector%>& %pattern_out%
%$$
This input value of $icode pattern_out$$ does not matter.
Upon return $icode pattern_out$$ is a
$cref/dependency pattern/dependency.cpp/Dependency Pattern/$$
for $latex F(x)$$.
The pattern has $latex m$$ rows, $latex n$$ columns.
If $icode%select_domain%[%j%]%$$ is true,
$icode%select_range%[%i%]%$$ is true, and
$latex F_i (x)$$ depends on $latex x_j$$,
then the pair $latex (i, j)$$ is in $icode pattern_out$$.
Not that this is also a sparsity pattern for the Jacobian
$latex \[
    J(x) = R F^{(1)} (x) D
\] $$
where $latex D$$ ($latex R$$) is the diagonal matrix corresponding
to $icode select_domain$$ ($icode select_range$$).

$head Example$$
$children%
    example/sparse/subgraph_sparsity.cpp
%$$
The file
$cref subgraph_sparsity.cpp$$
contains an example and test of this operation.

$end
-----------------------------------------------------------------------------
*/
# include <cppad/core/ad_fun.hpp>
# include <cppad/local/subgraph/sparsity.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

/*!
Subgraph sparsity patterns.

\tparam Base
is the base type for this recording.

\tparam SizeVector
is the simple vector with elements of type size_t that is used for
row, column index sparsity patterns.

\param select_domain
sparsity pattern for the diagonal of the square matrix D.

\param select_range
sparsity pattern for the diagnoal of the square matrix R

\param transpose
If true, the return is a dependency sparsity pattern for
\f$ D F^{(1)} (x)^T R \f$

\param pattern_out
The input value does not matter.
The return value is a dependency sparsity pattern for  \f$ R F^{(1)} (x) D \f$
where F is the function corresponding to the operation sequence
and x is any argument value.
is the sparsity pattern transposed.
*/
template <class Base, class RecBase>
template <class BoolVector, class SizeVector>
void ADFun<Base,RecBase>::subgraph_sparsity(
    const BoolVector&            select_domain    ,
    const BoolVector&            select_range     ,
    bool                         transpose        ,
    sparse_rc<SizeVector>&       pattern_out      )
{
    // compute the sparsity pattern in row, col
    local::pod_vector<size_t> row;
    local::pod_vector<size_t> col;

    // create the optimized recording
    switch( play_.address_type() )
    {
        case local::play::unsigned_short_enum:
        local::subgraph::subgraph_sparsity<unsigned short>(
            &play_,
            subgraph_info_,
            dep_taddr_,
            select_domain,
            select_range,
            row,
            col
        );
        break;

        case local::play::unsigned_int_enum:
        local::subgraph::subgraph_sparsity<unsigned int>(
            &play_,
            subgraph_info_,
            dep_taddr_,
            select_domain,
            select_range,
            row,
            col
        );
        break;

        case local::play::size_t_enum:
        local::subgraph::subgraph_sparsity<size_t>(
            &play_,
            subgraph_info_,
            dep_taddr_,
            select_domain,
            select_range,
            row,
            col
        );
        break;

        default:
        CPPAD_ASSERT_UNKNOWN(false);
    }

    CPPAD_ASSERT_UNKNOWN( row.size() == col.size() );

    // return the sparsity pattern
    size_t nr  = dep_taddr_.size();
    size_t nc  = ind_taddr_.size();
    size_t nnz = row.size();
    if( transpose )
    {   pattern_out.resize(nc, nr, nnz);
        for(size_t k = 0; k < nnz; k++)
            pattern_out.set(k, col[k], row[k]);
    }
    else
    {   pattern_out.resize(nr, nc, nnz);
        for(size_t k = 0; k < nnz; k++)
            pattern_out.set(k, row[k], col[k]);
    }
    return;
}
} // END_CPPAD_NAMESPACE
# endif
