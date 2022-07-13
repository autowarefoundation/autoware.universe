# ifndef CPPAD_CORE_FOR_HES_SPARSITY_HPP
# define CPPAD_CORE_FOR_HES_SPARSITY_HPP
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
$begin for_hes_sparsity$$
$spell
    Andrea Walther
    Jacobian
    Hessian
    jac
    hes
    bool
    const
    rc
    cpp
$$

$section Forward Mode Hessian Sparsity Patterns$$

$head Syntax$$
$icode%f%.for_hes_sparsity(
    %select_domain%, %select_range%, %internal_bool%, %pattern_out%
)%$$

$head Purpose$$
We use $latex F : \B{R}^n \rightarrow \B{R}^m$$ to denote the
$cref/AD function/glossary/AD Function/$$ corresponding to
the operation sequence stored in $icode f$$.
Fix a diagonal matrix $latex D \in \B{R}^{n \times n}$$,
a vector $latex s \in \B{R}^m$$ and define the function
$latex \[
    H(x) = D ( s^\R{T} F )^{(2)} ( x ) D
\] $$
Given  the sparsity for $latex D$$ and $latex s$$,
$code for_hes_sparsity$$ computes a sparsity pattern for $latex H(x)$$.

$head x$$
Note that the sparsity pattern $latex H(x)$$ corresponds to the
operation sequence stored in $icode f$$ and does not depend on
the argument $icode x$$.

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
It has size $latex n$$ and specifies which components of the diagonal of
$latex D$$ are non-zero; i.e., $icode%select_domain%[%j%]%$$ is true
if and only if $latex D_{j,j}$$ is possibly non-zero.


$head select_range$$
The argument $icode select_range$$ has prototype
$codei%
    const %BoolVector%& %select_range%
%$$
It has size $latex m$$ and specifies which components of the vector
$latex s$$ are non-zero; i.e., $icode%select_range%[%i%]%$$ is true
if and only if $latex s_i$$ is possibly non-zero.

$head internal_bool$$
If this is true, calculations are done with sets represented by a vector
of boolean values. Otherwise, a vector of sets of integers is used.

$head pattern_out$$
This argument has prototype
$codei%
    sparse_rc<%SizeVector%>& %pattern_out%
%$$
This input value of $icode pattern_out$$ does not matter.
Upon return $icode pattern_out$$ is a sparsity pattern for $latex H(x)$$.

$head Sparsity for Entire Hessian$$
Suppose that $latex R$$ is the $latex n \times n$$ identity matrix.
In this case, $icode pattern_out$$ is a sparsity pattern for
$latex (s^\R{T} F) F^{(2)} ( x )$$.

$head Algorithm$$
See Algorithm II in
$italic Computing sparse Hessians with automatic differentiation$$
by Andrea Walther.
Note that $icode s$$ provides the information so that
'dead ends' are not included in the sparsity pattern.

$head Example$$
$children%
    example/sparse/for_hes_sparsity.cpp
%$$
The file $cref for_hes_sparsity.cpp$$
contains an example and test of this operation.

$end
-----------------------------------------------------------------------------
*/
# include <cppad/core/ad_fun.hpp>
# include <cppad/local/sparse/internal.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

/*!
Forward Hessian sparsity patterns.

\tparam Base
is the base type for this recording.

\tparam BoolVector
is the simple vector with elements of type bool that is used for
sparsity for the vector s.

\tparam SizeVector
is the simple vector with elements of type size_t that is used for
row, column index sparsity patterns.

\param select_domain
is a sparsity pattern for for the diagonal of D.

\param select_range
is a sparsity pattern for for s.

\param internal_bool
If this is true, calculations are done with sets represented by a vector
of boolean values. Otherwise, a vector of standard sets is used.

\param pattern_out
The return value is a sparsity pattern for H(x) where
\f[
    H(x) = D * F^{(1)} (x) * D
\f]
Here F is the function corresponding to the operation sequence
and x is any argument value.
*/
template <class Base, class RecBase>
template <class BoolVector, class SizeVector>
void ADFun<Base,RecBase>::for_hes_sparsity(
    const BoolVector&            select_domain    ,
    const BoolVector&            select_range     ,
    bool                         internal_bool    ,
    sparse_rc<SizeVector>&       pattern_out      )
{
    // used to identify the RecBase type in calls to sweeps
    RecBase not_used_rec_base(0.0);
    //
    size_t n  = Domain();
    size_t m  = Range();
    //
    CPPAD_ASSERT_KNOWN(
        size_t( select_domain.size() ) == n,
        "for_hes_sparsity: size of select_domain is not equal to "
        "number of independent variables"
    );
    CPPAD_ASSERT_KNOWN(
        size_t( select_range.size() ) == m,
        "for_hes_sparsity: size of select_range is not equal to "
        "number of dependent variables"
    );
    // do not need transpose or depenency
    bool transpose  = false;
    bool dependency = false;
    //
    local::pod_vector<bool> select_domain_pod_vector(n);
    for(size_t j = 0; j < n; ++j)
        select_domain_pod_vector[j] = select_domain[j];
    //
    sparse_rc<SizeVector> pattern_tmp;
    if( internal_bool )
    {
        // reverse Jacobian sparsity pattern for select_range
        local::sparse::pack_setvec internal_rev_jac;
        internal_rev_jac.resize(num_var_tape_, 1);
        for(size_t i = 0; i < m; i++) if( select_range[i] )
        {   CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_ );
            // Not using post_element because only adding one element per set
            internal_rev_jac.add_element( dep_taddr_[i] , 0 );
        }
        // reverse Jacobian sparsity for all variables on tape
        local::sweep::rev_jac<addr_t>(
            &play_,
            dependency,
            n,
            num_var_tape_,
            internal_rev_jac,
            not_used_rec_base
        );
        // internal vector of sets that will hold Hessian
        local::sparse::pack_setvec internal_for_hes;
        internal_for_hes.resize(n + 1 + num_var_tape_, n + 1);
        //
        // compute forward Hessian sparsity pattern
        local::sweep::for_hes<addr_t>(
            &play_,
            n,
            num_var_tape_,
            select_domain_pod_vector,
            internal_rev_jac,
            internal_for_hes,
            not_used_rec_base
        );
        //
        // put the result in pattern_tmp
        local::sparse::get_internal_pattern(
            transpose, ind_taddr_, internal_for_hes, pattern_tmp
        );
    }
    else
    {
        // reverse Jacobian sparsity pattern for select_range
        // (corresponds to s)
        local::sparse::list_setvec internal_rev_jac;
        internal_rev_jac.resize(num_var_tape_, 1);
        for(size_t i = 0; i < m; i++) if( select_range[i] )
        {   CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_ );
            // Not using post_element because only adding one element per set
            internal_rev_jac.add_element( dep_taddr_[i] , 0 );
        }
        // reverse Jacobian sparsity for all variables on tape
        local::sweep::rev_jac<addr_t>(
            &play_,
            dependency,
            n,
            num_var_tape_,
            internal_rev_jac,
            not_used_rec_base

        );
        // internal vector of sets that will hold Hessian
        local::sparse::list_setvec internal_for_hes;
        internal_for_hes.resize(n + 1 + num_var_tape_, n + 1);
        //
        // compute forward Hessian sparsity pattern
        local::sweep::for_hes<addr_t>(
            &play_,
            n,
            num_var_tape_,
            select_domain_pod_vector,
            internal_rev_jac,
            internal_for_hes,
            not_used_rec_base
        );
        //
        // put the result in pattern_tmp
        local::sparse::get_internal_pattern(
            transpose, ind_taddr_, internal_for_hes, pattern_tmp
        );
    }
    // subtract 1 from all column values
    CPPAD_ASSERT_UNKNOWN( pattern_tmp.nr() == n );
    CPPAD_ASSERT_UNKNOWN( pattern_tmp.nc() == n + 1 );
    const SizeVector& row( pattern_tmp.row() );
    const SizeVector& col( pattern_tmp.col() );
    size_t nr   = n;
    size_t nc   = n;
    size_t nnz  = pattern_tmp.nnz();
    pattern_out.resize(nr, nc, nnz);
    for(size_t k = 0; k < nnz; k++)
    {   CPPAD_ASSERT_UNKNOWN( 0 < col[k] );
        pattern_out.set(k, row[k], col[k] - 1);
    }
    return;
}
} // END_CPPAD_NAMESPACE
# endif
