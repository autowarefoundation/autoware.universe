/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include "cppad_ipopt_nlp.hpp"
# include "vec_fun_pattern.hpp"
// ---------------------------------------------------------------------------
namespace cppad_ipopt {
// ---------------------------------------------------------------------------
/*!
\{
\file vec_fun_pattern.cpp
\brief Determine a sparsity pattern for a vector of AD function objects.
*/

/*!
Determine a sparsity patterns for each function in a vector of functions.

\param K
is the number of functions that we are computing the sparsity pattern for.

\param p
is a vector with size K.
For <tt>k = 0 , ... , K-1, p[k]</tt>
is dimension of the range space for \f$ r_k (u) \f$; i.e.,
\f$ r_k (u) \in {\bf R}^{p(k)} \f$.

\param q
is a vector with size K.
For <tt>k = 0 , ... , K-1, q[k]</tt>
is dimension of the domain space for \f$ r_k (u) \f$; i.e.,
\f$ u \in {\bf R}^{q(k)} \f$.

\param retape
is a vector with size K.
For <tt>k = 0 , ... , K-1</tt>,
if <tt>retape[k]</tt> is true,
the function object <tt>r[k]</tt> is a valid representation
for \f$ r_k (u) \f$ for all \f$ u \in {\bf R}^{q(k)} \f$.
Otherwise, the function object must be retaped for each
value of \f$ u \f$.

\param r_fun
is the vector of AD function objects which has size size K.
For <tt>k = 0 , ... , K-1</tt>,
if <tt>retape[k]</tt> is true, <tt>r_fun[k]</tt> is not used.
If <tt>retape[k]</tt> is false, <tt>r_fun[k]</tt> is not used.
is a CppAD function object correspopnding to the function
\f$ r_k : {\bf R}^{q[k]} \rightarrow {\bf R}^{p[k]} \f$.
The following non-constant member functions will be called:
\verbatim
    r_fun[k].ForSparseJac(q[k], pattern_domain)
    r_fun[k].RevSparseHes(p[k], pattern_range)
\endverbatim
The following const member functions <tt>r_fun[k].Range()</tt>
and <tt>r_fun[k].Domain()</tt> may also be called.

\param pattern_jac_r
is a vector with size K.
On input, For <tt>k = 0 , ... , K-1, pattern_jac_r[k]</tt>
is a vector of length p[k] * q[k]
and the value of its elements does not matter.
On output it is a CppAD sparsity pattern for the Jacobian of
\f$ r_k (u) \f$.

\param pattern_hes_r
is a vector with size K.
On input, For <tt>k = 0 , ... , K-1, pattern_hes_r[k]</tt>
is a vector of length q[k] * q[k]
and the value of its elements does not matter.
On output it is a CppAD sparsity pattern for the Hessian of
\f$ R : {\bf R}^{q[k]} \rightarrow {\bf R} \f$ which is defined by
\f[
    R(u) = \sum_{i=0}^{p[k]-1} r_k (u)_i
\f]
*/
void vec_fun_pattern(
    size_t                                          K              ,
    const CppAD::vector<size_t>&                    p              ,
    const CppAD::vector<size_t>&                    q              ,
    const CppAD::vectorBool&                        retape         ,
    CppAD::vector< CppAD::ADFun<Ipopt::Number> >&   r_fun          ,
    CppAD::vector<CppAD::vectorBool>&               pattern_jac_r  ,
    CppAD::vector<CppAD::vectorBool>&               pattern_hes_r  )
{   // check some assumptions
    CPPAD_ASSERT_UNKNOWN( K == p.size() );
    CPPAD_ASSERT_UNKNOWN( K == q.size() );
    CPPAD_ASSERT_UNKNOWN( K == retape.size() );
    CPPAD_ASSERT_UNKNOWN( K == r_fun.size() );
    CPPAD_ASSERT_UNKNOWN( K == pattern_jac_r.size() );
    CPPAD_ASSERT_UNKNOWN( K == pattern_hes_r.size() );

    using CppAD::vectorBool;
    size_t i, j, k;

    for(k = 0; k < K; k++)
    {   // check some k specific assumptions
        CPPAD_ASSERT_UNKNOWN( pattern_jac_r[k].size() == p[k] * q[k] );
        CPPAD_ASSERT_UNKNOWN( pattern_hes_r[k].size() == q[k] * q[k] );

        if( retape[k] )
        {   for(i = 0; i < p[k]; i++)
            {   for(j = 0; j < q[k]; j++)
                    pattern_jac_r[k][i*q[k] + j] = true;
            }
            for(i = 0; i < q[k]; i++)
            {   for(j = 0; j < q[k]; j++)
                    pattern_hes_r[k][i*q[k] + j] = true;
            }
        }
        else
        {   // check assumptions about r_k
            CPPAD_ASSERT_UNKNOWN( r_fun[k].Range() == p[k] );
            CPPAD_ASSERT_UNKNOWN( r_fun[k].Domain() == q[k] );

            // pattern for the identity matrix
            CppAD::vectorBool pattern_domain(q[k] * q[k]);
            for(i = 0; i < q[k]; i++)
            {   for(j = 0; j < q[k]; j++)
                    pattern_domain[i*q[k] + j] = (i == j);
            }
            // use forward mode to compute Jacobian sparsity
            pattern_jac_r[k] =
                r_fun[k].ForSparseJac(q[k], pattern_domain);
            // user reverse mode to compute Hessian sparsity
            CppAD::vectorBool pattern_ones(p[k]);
            for(i = 0; i < p[k]; i++)
                pattern_ones[i] = true;
            pattern_hes_r[k] =
                r_fun[k].RevSparseHes(q[k], pattern_ones);
        }
    }
}
// ---------------------------------------------------------------------------
} // end namespace cppad_ipopt
// ---------------------------------------------------------------------------
