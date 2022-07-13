# ifndef CPPAD_CPPAD_IPOPT_SRC_FUN_RECORD_HPP
# define CPPAD_CPPAD_IPOPT_SRC_FUN_RECORD_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include "cppad_ipopt_nlp.hpp"

// ---------------------------------------------------------------------------
namespace cppad_ipopt {
// ---------------------------------------------------------------------------
/*!
\{
\file fun_record.hpp
\brief Records operation sequence for r_k (u)
*/

/*!
Records operation sequence for \f$ r_k (u) \f$ at \f$u = [ J \circ n ] (x)\f$.

\tparam NumVector
is the type of the argumen x. It can either be
<tt>Ipopt::Number*</tt> or
<tt>CppAD::vector<Ipopt::Number></tt>; i.e., <tt>NumberVector</tt>.

\param fg_info
Given a value \f$ u \in {\bf R}^{q[k]} \f$,
 fg_info returns the value \f$ r_k (u) \in {\bf R}^{p[k]} \f$.
using the syntax
\verbatim
    fg_info->eval_r(k, u);
\endverbatim
No other use is made of fg_info.

\param k
is a value less that K specifying
the index value for k in the evaluation <tt>eval_r</tt>.

\param p
<tt>p[k]</tt> is dimension of the range space for \f$ r_k (u) \f$; i.e.,
\f$ r_k (u) \in {\bf R}^{p(k)} \f$.

\param q
<tt>q[k]</tt> is dimension of the domain space for \f$ r_k (u) \f$; i.e.,
\f$ u \in {\bf R}^{q(k)} \f$.

\param n
is the length of the vector x.

\param x
the length of x is equal to n and the point
\f[
    u = [ J \circ n ] (x)
\f]
is the point at which the operation sequence for \f$ r_k \f$ is recorded.

\param J
is a vector with length <tt>q[k]</tt> that projects from \f$ {\bf R}^n \f$
to \f$ {\bf R}^{q[k]} \f$
by selecting an ordered subset of the possible indices
\f$ \{ 0 , \ldots , n-1 \} \f$.
Hence, <tt>0 <= J[j] < n</tt> for <tt>j = 0 , ... , q[k]-1</tt>.

\param r_fun
is the vector of AD function objects which has size size greater than k.
Only the function object <tt>r_fun[k]</tt> is referenced.
The input value of this function object does not matter.
On output it is a recording of the function \f$ r_k (u) \f$
at the value of \f$ u \f$ specified by x and J.
*/

template <class NumVector>
void fun_record(
    cppad_ipopt_fg_info*                              fg_info ,
    size_t                                            k       ,
    const SizeVector&                                 p       ,
    const SizeVector&                                 q       ,
    size_t                                            n       ,
    const NumVector&                                  x       ,
    const SizeVector&                                 J       ,
    CppAD::vector< CppAD::ADFun<Ipopt::Number> >&     r_fun   )
{   size_t j;

    // extract u from x
    ADVector u(q[k]);
    for(j = 0; j < q[k]; j++)
    {   // when NDEBUG is not defined, this error should be caught
        // during the cppad_ipopt_nlp constructor.
        CPPAD_ASSERT_UNKNOWN( J[j] < n );
        u[j] = x[ J[j] ];
    }

    // start the recording
    CppAD::Independent(u);

    // record the evaulation of r_k (u)
    ADVector r_k = fg_info->eval_r(k, u);
    CPPAD_ASSERT_KNOWN( r_k.size() == p[k] ,
    "cppad_ipopt_nlp: eval_r return value size not equal to p[k]."
    );

    // stop the recording and store operation sequence in
    r_fun[k].Dependent(u, r_k);
}
// ---------------------------------------------------------------------------
} // end namespace cppad_ipopt
// ---------------------------------------------------------------------------
# endif
