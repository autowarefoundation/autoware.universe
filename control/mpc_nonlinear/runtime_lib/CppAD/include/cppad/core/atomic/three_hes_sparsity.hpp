# ifndef CPPAD_CORE_ATOMIC_THREE_HES_SPARSITY_HPP
# define CPPAD_CORE_ATOMIC_THREE_HES_SPARSITY_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin atomic_three_hes_sparsity$$
$spell
    Hessian
    afun
    hes
$$

$section Atomic Function Hessian Sparsity Patterns$$

$head Syntax$$
$icode%ok% = %afun%.hes_sparsity(
    %parameter_x%, %type_x%, %select_x%, %select_y%, %pattern_out%
)%$$

$head Prototype$$
$srcthisfile%0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
%$$

$head Implementation$$
This function must be defined if
$cref/afun/atomic_three_ctor/atomic_user/afun/$$ is
used to define an $cref ADFun$$ object $icode f$$,
and Hessian sparsity patterns are computed for $icode f$$.

$head Base$$
See $cref/Base/atomic_three_afun/Base/$$.

$head parameter_x$$
See $cref/parameter_x/atomic_three/parameter_x/$$.

$head type_x$$
See $cref/type_x/atomic_three/type_x/$$.

$head select_x$$
This argument has size equal to the number of arguments to this
atomic function; i.e. the size of $icode ax$$.
It specifies which domain components are included in
the calculation of $icode pattern_out$$.
If $icode%select_x%[%j%]%$$ is false, then there will be no indices
$icode k$$ such that either of the following hold:
$codei%
    %pattern_out%.row()[%k%] == %j%
    %pattern_out%.col()[%k%] == %j%
%$$.

$head select_y$$
This argument has size equal to the number of results to this
atomic function; i.e. the size of $icode ay$$.
It specifies which range component functions $latex g_i (x)$$ are included in
of $icode pattern_out$$.

$head pattern_out$$
This input value of $icode pattern_out$$ does not matter.
Upon return it is the union,
with respect to $icode i$$ such that $icode%select_y%[%i%]%$$ is true,
of the sparsity pattern for Hessian of $latex g_i (x)$$.
To be specific, there are non-negative indices
$icode i$$, $icode r$$, $icode c$$, and $icode k$$ such that
$codei%
    %pattern_out%.row()[%k%] == %r%
    %pattern_out%.col()[%k%] == %c%
%$$
if and only if
$icode%select_y%[%i%]%$$ is true,
$icode%select_x%[%r%]%$$ is true,
$icode%select_x%[%c%]%$$ is true,
and
$latex \[
    \partial_{x(r)} \partial_{x(c)} g_i(x)
\] $$
is possibly non-zero.
Note that the sparsity pattern should be symmetric.

$head ok$$
If this calculation succeeded, $icode ok$$ is true.
Otherwise it is false.

$children%
    example/atomic_three/hes_sparsity.cpp
%$$
$head Examples$$
The file $cref atomic_three_hes_sparsity.cpp$$ contains an example and test
that uses this routine.
$end
-----------------------------------------------------------------------------
*/
namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/three_hes_sparsity.hpp
Third generation atomic Hessian dependency and sparsity patterns.
*/
/*!
atomic_three to Hessian dependency and sparsity calculations.

\param parameter_x [in]
contains the values for arguments that are parameters.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.

\param select_x [in]
which domain components to include in the dependency or sparsity pattern.

\param select_y [in]
which range components to include in the dependency or sparsity pattern.

\param pattern_out [out]
is the sparsity pattern for Hessian.
*/
// BEGIN_PROTOTYPE
template <class Base>
bool atomic_three<Base>::hes_sparsity(
    const vector<Base>&                     parameter_x  ,
    const vector<ad_type_enum>&             type_x       ,
    const vector<bool>&                     select_x     ,
    const vector<bool>&                     select_y     ,
    sparse_rc< vector<size_t> >&            pattern_out  )
// END_PROTOTYPE
{   return false; }
/*!
Link from forward Hessian sweep to atomic_three.
2DO: move this functiton outside this file so can change
developer documentation to omhelp formating.

\tparam InternalSparsity
Is the used internaly for sparsity calculations; i.e.,
sparse_pack or sparse_list.

\param parameter_x
is parameter arguments to the function, other components are nan.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.

\param x_index
is the variable index, on the tape, for the arguments to this function.
This size of x_index is n, the number of arguments to this function.
The index zero is used for parameters.

\param y_index
is the variable index, on the tape, for the results for this function.
This size of y_index is m, the number of results for this function.
The index zero is used for parameters.

\param for_jac_sparsity
On input, for j = 0, ... , n-1, the sparsity pattern with index x_index[j],
is the forward Jacobian sparsity for the j-th argument to this atomic function.

\param rev_jac_pattern
On input, for i = 0, ... , m-1, the sparsity pattern with index y_index[i],
is the reverse Jacobian sparsity for the i-th result to this atomic function.
This shows which components of the result affect the function we are
computing the Hessian of.

\param hes_sparsity_for
This is the sparsity pattern for the Hessian. On input, the non-linear
terms in the atomic fuction have not been included. Upon return, they
have been included.
*/
template <class Base>
template <class InternalSparsity>
bool atomic_three<Base>::for_hes_sparsity(
    const vector<Base>&              parameter_x      ,
    const vector<ad_type_enum>&      type_x           ,
    const local::pod_vector<size_t>& x_index          ,
    const local::pod_vector<size_t>& y_index          ,
    size_t                           np1              ,
    size_t                           numvar           ,
    const InternalSparsity&          rev_jac_pattern  ,
    InternalSparsity&                for_sparsity     )
{   typedef typename InternalSparsity::const_iterator const_iterator;
    //
    CPPAD_ASSERT_UNKNOWN( rev_jac_pattern.end() == 1 );
    CPPAD_ASSERT_UNKNOWN( for_sparsity.end() == np1 );
    CPPAD_ASSERT_UNKNOWN( for_sparsity.n_set() == np1 + numvar );
    size_t n      = x_index.size();
    size_t m      = y_index.size();
    //
    // select_x
    vector<bool> select_x(n);
    for(size_t j = 0; j < n; j++)
    {   // check if should compute pattern w.r.t x[j]
        select_x[j] = for_sparsity.number_elements(np1 + x_index[j]) > 0;
    }
    //
    // bool select_y
    vector<bool> select_y(m);
    for(size_t i = 0; i < m; i++)
    {   // check if we should include y[i]
        select_y[i] = rev_jac_pattern.number_elements(y_index[i]) > 0;
    }
    // ------------------------------------------------------------------------
    // call user's version of atomic function for Jacobian
    sparse_rc< vector<size_t> > pattern_out;
    bool dependency = false;
    bool ok = jac_sparsity(
        parameter_x, type_x, dependency, select_x, select_y, pattern_out
    );
    if( ! ok )
        return false;
    //
    // transfer sparsity patterns from pattern_out to var_sparsity
    size_t                nnz = pattern_out.nnz();
    const vector<size_t>& row( pattern_out.row() );
    const vector<size_t>& col( pattern_out.col() );
    for(size_t k = 0; k < nnz; ++k)
    {   size_t i = row[k];
        size_t j = col[k];
        CPPAD_ASSERT_KNOWN(
            select_y[i] & select_x[j],
            "atomic: jac_sparsity: pattern_out not in "
            "select_x or select_y range"
        );
        const_iterator itr(for_sparsity, np1 + x_index[j]);
        size_t ell = *itr;
        while( ell < np1 )
        {   for_sparsity.post_element(np1 + y_index[i], ell );
            ell = *(++itr);
        }
    }
    for(size_t i = 0; i < m; ++i)
        for_sparsity.process_post( np1 + y_index[i] );
    // ------------------------------------------------------------------------
    // call user's version of atomic function for Hessian
    ok = hes_sparsity(
        parameter_x, type_x, select_x, select_y, pattern_out
    );
    if( ! ok )
        return ok;
    //
    // add new elements to Hessian sparisty in calling routine
    nnz = pattern_out.nnz();
    for(size_t k = 0; k < nnz; ++k)
    {   size_t r = row[k];
        size_t c = col[k];
        CPPAD_ASSERT_KNOWN(
            select_x[r] & select_x[c],
            "atomic: hes_sparsity: pattern_out not in select_x range"
        );
        const_iterator itr_1(for_sparsity, np1 + x_index[r]);
        size_t v1 = *itr_1;
        while( v1 < np1 )
        {   for_sparsity.binary_union(
                v1, v1, np1 + x_index[c], for_sparsity
             );
             v1 = *(++itr_1);
        }
        // no need to add same elements twice
        if( c != r )
        {   const_iterator itr_2(for_sparsity, np1 + x_index[c]);
            size_t v2 = *itr_2;
            while( v2 < np1 )
            {   for_sparsity.binary_union(
                    v2, v2, np1 + x_index[r], for_sparsity
                );
                v2 = *(++itr_2);
            }
        }
    }
    return ok;
}
/*!
Link from for_reverse Hessian sweep to atomic_three.

\tparam InternalSparsity
Is the used internaly for sparsity calculations; i.e.,
sparse_pack or sparse_list.

\param parameter_x
is parameter arguments to the function, other components are nan.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.

\param x_index
is the variable index, on the tape, for the arguments to this function.
This size of x_index is n, the number of arguments to this function.
The index zero is used for parameters.

\param y_index
is the variable index, on the tape, for the results for this function.
This size of y_index is m, the number of results for this function.
The index zero is used for parameters.

\param for_jac_pattern
On input, for j = 0, ... , n-1, the sparsity pattern with index x_index[j],
is the forward Jacobian pattern for the j-th argument to this atomic function.

\param rev_jac_flag
On input, for i = 0, ... , m-1, rev_jac_flag[ y_index[i] ] is true
if the function we are computing the Hessian of has possibly non-zero Jacobian
w.r.t varialbe y_index[i].
On output, for j = 0, ... , n, rev_jac_flag[ x_index[j] ] is set to true
if the varialbe with index x_index[j] has possible non-zero Jacobian
with repect to one of the true y_index[i] cases.
Otherwise, rev_jac_flag [ x_inde[j] ] is not changed.

\param hes_sparsity_rev
Is the reverse mode sparsity pattern for the Hessian. On input, the non-linear
terms in the atomic fuction have not been included. Upon return, they
have been included.
*/
template <class Base>
template <class InternalSparsity>
bool atomic_three<Base>::rev_hes_sparsity(
    const vector<Base>&              parameter_x      ,
    const vector<ad_type_enum>&      type_x           ,
    const local::pod_vector<size_t>& x_index          ,
    const local::pod_vector<size_t>& y_index          ,
    const InternalSparsity&          for_jac_pattern  ,
    bool*                            rev_jac_flag     ,
    InternalSparsity&                hes_sparsity_rev )
{   typedef typename InternalSparsity::const_iterator const_iterator;
    size_t n      = x_index.size();
    size_t m      = y_index.size();
    //
    // select_x
    vector<bool> select_x(n);
    for(size_t j = 0; j < n; j++)
    {   // check if should compute pattern w.r.t x[j]
        const_iterator itr(for_jac_pattern, x_index[j]);
        size_t i = *itr;
        select_x[j] = i < for_jac_pattern.end();
        CPPAD_ASSERT_UNKNOWN( x_index[j] > 0 || ! select_x[j] );
    }
    //
    // bool select_y
    vector<bool> select_y(m);
    for(size_t i = 0; i < m; i++)
    {   // check if we should include y[i]
        select_y[i] = rev_jac_flag[ y_index[i] ];
        CPPAD_ASSERT_UNKNOWN( y_index[i] > 0 || ! select_y[i] );
    }
    //
    // call atomic function for Jacobain sparsity
    bool dependency = false;
    sparse_rc< vector<size_t> > pattern_jac;
    bool ok = jac_sparsity(
        parameter_x, type_x, dependency, select_x, select_y, pattern_jac
    );
    const vector<size_t>& row_jac( pattern_jac.row() );
    const vector<size_t>& col_jac( pattern_jac.col() );
    size_t nnz_jac = pattern_jac.nnz();
    if( ! ok )
        return ok;
    //
    // call atomic function for Hessian sparsity
    sparse_rc< vector<size_t> > pattern_hes;
    ok = hes_sparsity(parameter_x, type_x, select_x, select_y, pattern_hes);
    const vector<size_t>& row_hes( pattern_hes.row() );
    const vector<size_t>& col_hes( pattern_hes.col() );
    size_t nnz_hes = pattern_hes.nnz();
    if( ! ok )
        return ok;
    //
    // propagate Hessian sparsity through the Jacobian
    for(size_t k = 0; k < nnz_jac; ++k)
    {   size_t i = row_jac[k];
        size_t j = col_jac[k];
        CPPAD_ASSERT_KNOWN(
            select_y[i] & select_x[j] ,
            "atomic: jac_sparsity: pattern_out not in "
            "select_x or select_y range"
        );
        // from y_index[i] to x_index[j]
        hes_sparsity_rev.binary_union(
            x_index[j], x_index[j], y_index[i], hes_sparsity_rev
        );
    }
    //
    // propagate rev_jac_flag through the Jacobian
    // (seems OK to exclude variables with zero forward jacobian)
    for(size_t k = 0; k < nnz_jac; ++k)
    {   size_t j = col_jac[k];
        rev_jac_flag[ x_index[j] ] = true;
    }
    //
    // new hessian sparsity terms between y and x
    for(size_t k = 0; k < nnz_hes; ++k)
    {   size_t r = row_hes[k];
        size_t c = col_hes[k];
        CPPAD_ASSERT_KNOWN(
            select_x[r] & select_x[c] ,
            "atomic: hes_sparsity: pattern_out not in select_x range"
        );
        hes_sparsity_rev.binary_union(
            x_index[r], x_index[r], x_index[c], for_jac_pattern
        );
        hes_sparsity_rev.binary_union(
            x_index[c], x_index[c], x_index[r], for_jac_pattern
        );
    }
    return ok;
}

} // END_CPPAD_NAMESPACE

# endif
