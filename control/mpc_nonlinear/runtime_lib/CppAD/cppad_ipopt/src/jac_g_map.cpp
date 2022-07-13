/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include "cppad_ipopt_nlp.hpp"
# include "jac_g_map.hpp"
// ---------------------------------------------------------------------------
namespace cppad_ipopt {
// ---------------------------------------------------------------------------
/*!
\{
\file jac_g_map.cpp
\brief Creates a mapping between two representations for Jacobian of g.
*/


/*!
Create mapping from CppAD to Ipopt sparse representations of Jacobian of g.

The functions
\f$ f : {\bf R}^n \rightarrow {\bf R} \f$ and
\f$ g : {\bf R}^n \rightarrow {\bf R}^m \f$ are defined by
the \ref Users_Representation.

\param fg_info
For <tt>k = 0 , ... , K-1</tt>,
for <tt>ell = 0 , ... , L[k]</tt>,
the function call
\verbatim
    fg_info->index(k, ell, I, J);
\endverbatim
is made by jac_g_map.
The values k and ell are inputs.
The input size of I ( J )
is greater than or equal <tt>p[k] ( q[k] )</tt>
and this size is not changed.
The input values of the elements of I and J are not specified.
The output value of the elements of I define
\f[
I_{k, \ell} = ( {\rm I[0]} , \cdots , {\rm I[p[k]-1]} )
\f]
The output value of the elements of J define
\f[
J_{k, \ell} = ( {\rm J[0]} , \cdots , {\rm J[q[k]-1]} )
\f]

\param m
is the dimension of the range space for \f$ g(x) \f$; i.e.,
\f$ g(x) \in {\bf R}^m \f$.

\param n
is the dimension of the domain space for \f$ f(x) \f$ and \f$ g(x) \f$;
i.e., \f$ x \in {\bf R}^n \f$.

\param K
is the number of functions \f$ r_k ( u ) \f$ used for the representation of
\f$ f(x) \f$ and \f$ g(x) \f$.

\param L
is a vector with size K.
For <tt>k = 0 , ... , K-1, L[k]</tt>
is the number of terms that use \f$ r_k (u) \f$
in the representation of \f$ f(x) \f$ and \f$ g(x) \f$.

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

\param pattern_jac_r
is a vector with size K.
For <tt>k = 0 , ... , K-1, pattern_jac_r[k]</tt>
is a CppAD sparsity pattern for the Jacobian of the function
\f$ r_k : {\bf R}^{q(k)} \rightarrow {\bf R}^{p(k)} \f$.
As such, <tt>pattern_jac_r[k].size() == p[k] * q[k]</tt>.

\param I
is a work vector of length greater than or equal <tt>p[k]</tt> for all k.
The input and output value of its elements are unspecified.
The size of I is not changed.

\param J
is a work vector of length greater than or equal <tt>q[k]</tt> for all k.
The input and output value of its elements are unspecified.
The size of J is not changed.

\param index_jac_g:
On input, this is empty; i.e., <tt>index_jac_g.size() == 0</tt>.
On output, it is the index mapping from \f$ (i, j) \f$ in the Jacobian of
\f$ g(x) \f$ to the corresponding index value used by Ipopt to represent
the Jacobian.
Furthermore, if <tt>index_jac_g[i].find(j) == index_jac_g[i].end()</tt>,
then the \f$ (i, j)\f$ entry in the Jacobian of \f$ g(x) \f$ is always zero.
*/
void jac_g_map(
    cppad_ipopt_fg_info*  fg_info                                  ,
    size_t                                          m              ,
    size_t                                          n              ,
    size_t                                          K              ,
    const CppAD::vector<size_t>&                    L              ,
    const CppAD::vector<size_t>&                    p              ,
    const CppAD::vector<size_t>&                    q              ,
    const CppAD::vector<CppAD::vectorBool>&         pattern_jac_r  ,
    CppAD::vector<size_t>&                          I              ,
    CppAD::vector<size_t>&                          J              ,
    CppAD::vector< std::map<size_t,size_t> >&       index_jac_g    )
{
    using CppAD::vectorBool;
    size_t i, j, ij, k, ell;

    CPPAD_ASSERT_UNKNOWN( K == L.size() );
    CPPAD_ASSERT_UNKNOWN( K == p.size() );
    CPPAD_ASSERT_UNKNOWN( K == q.size() );
    CPPAD_ASSERT_UNKNOWN( K == pattern_jac_r.size() );
# ifndef NDEBUG
    for(k = 0; k < K; k++)
    {   CPPAD_ASSERT_UNKNOWN( p[k] <= I.size() );
        CPPAD_ASSERT_UNKNOWN( q[k] <= J.size() );
        CPPAD_ASSERT_UNKNOWN( p[k]*q[k] == pattern_jac_r[k].size() );
    }
# endif
    // Now compute pattern for g
    // (use standard set representation because can be huge).
    CppAD::vector< std::set<size_t> > pattern_jac_g(m);
    for(k = 0; k < K; k++) for(ell = 0; ell < L[k]; ell++)
    {   fg_info->index(k, ell, I, J);
        for(i = 0; i < p[k]; i++) if( I[i] != 0 )
        {   for(j = 0; j < q[k]; j++)
            {   ij  = i * q[k] + j;
                if( pattern_jac_r[k][ij] )
                    pattern_jac_g[I[i]-1].insert(J[j]);
            }
        }
    }

    // Now compute the mapping from (i, j) in the Jacobian of g to the
    // corresponding index value used by Ipopt to represent the Jacobian.
    CPPAD_ASSERT_UNKNOWN( index_jac_g.size() == 0 );
    index_jac_g.resize(m);
    std::set<size_t>::const_iterator itr;
    ell = 0;
    for(i = 0; i < m; i++)
    {   for(itr = pattern_jac_g[i].begin();
            itr != pattern_jac_g[i].end();
            itr++)
        {
            index_jac_g[i][*itr] = ell++;
        }
    }
    return;
}

// ---------------------------------------------------------------------------
} // end namespace cppad_ipopt
// ---------------------------------------------------------------------------
