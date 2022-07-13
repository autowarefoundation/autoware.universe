/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <limits>

# include "cppad_ipopt_nlp.hpp"
# include "sparse_map2vec.hpp"
# include "jac_g_map.hpp"
# include "hes_fg_map.hpp"
# include "vec_fun_pattern.hpp"
# include "fun_record.hpp"


/// If 0 tracing is off, otherwise tracing is on.
# define  CPPAD_IPOPT_NLP_TRACE 0

# if CPPAD_IPOPT_NLP_TRACE
# include <cstdio>
# endif

// ---------------------------------------------------------------------------
namespace cppad_ipopt {
// ---------------------------------------------------------------------------
/*!
\{
\file cppad_ipopt_nlp.cpp
\brief Member functions for the cppad_ipopt_nlp class.
*/


/*!
Constructor for the \ref Nonlinear_Programming_Problem.

\param n
dimension of the domain space for f(x) and g(x).

\param m
dimension of the range space for g(x)

\param x_i
initial value of x during the optimization procedure (size n).

\param x_l
lower limit for x (size n).

\param x_u
upper limit for x (size n).

\param g_l
lower limit for g(x) (size m).

\param g_u
upper limit for g(x) (size m).

\param fg_info
pointer to base class version of derived class object used to get
information about the user's representation for f(x) and g(x).
(The object pointed to must not be deleted before this cppad_ipopt_nlp object).

\param solution
pointer to object where final results are stored.
(The object pointed to must not be deleted before this cppad_ipopt_nlp object).

\par Constants
The following values are set by the constructor and are const
or effectively const; i.e., they are set by the constructor and should
not be changed:
\verbatim
    n_, m_, x_i_, x_l_, x_u_, g_l_, g_u_, K_, L_, p_, q_, retape_,
    pattern_jac_r_, pattern_hes_r_, index_jac_g_, index_hes_fg_,
    nnz_jac_g_, iRow_jac_g_, jCol_jac_g_,
    nnz_h_lag_, iRow_h_lag_, jCol_h_lag_,
\endverbatim
In addition, the function calls <tt>fg_info->set_n(n)</tt>
and <tt>fg_info->set_m(m)</tt> are used to set the values of n
and m in fg_info.

\par Variables
The following arrays have fixed size which is set during this constructor:

\li tape_ok_ has size K_. It is initialized as true for indices
 k such that <tt>retape[k]</tt> is false.

\li r_fun_ has size K_. It is initilaize with the default
 ADFun constructor. Then, for indices k such that
<tt>retape[k]</tt> is false, the operation sequence corresponding
to \f$ r_k (u) \f$ is stored in <tt>r_fun_[k]</tt>.

\li I_ has size equal to the maximum of <tt>p[k]</tt> w.r.t k.

\li J_ has size equal to the maximum of <tt>q[k]</tt> w.r.t k.

\par NDEBUG
If the preprocessor symbol NEBUG is not defined,
certain of the assumptions about the function calls of the form
\verbatim
    fg_info->index(k, ell, I, J)
\endverbatim
are checked to make sure they hold.
*/
cppad_ipopt_nlp::cppad_ipopt_nlp(
    size_t n                         ,
    size_t m                         ,
    const NumberVector    &x_i       ,
    const NumberVector    &x_l       ,
    const NumberVector    &x_u       ,
    const NumberVector    &g_l       ,
    const NumberVector    &g_u       ,
    cppad_ipopt_fg_info*   fg_info   ,
    cppad_ipopt_solution*  solution )
    : n_ ( n ),
      m_ ( m ),
      x_i_ ( x_i ),
      x_l_ ( x_l ),
      x_u_ ( x_u ),
      g_l_ ( g_l ),
      g_u_ ( g_u ),
      fg_info_ ( fg_info ) ,
      solution_ (solution) ,
      infinity_ ( std::numeric_limits<Number>::infinity() )
{   size_t k;

    // set information needed in cppad_ipopt_fg_info
    fg_info_->set_n(n);
    fg_info_->set_m(m);

    // get information from derived class version of fg_info
    K_ = fg_info_->number_functions();
    L_.resize(K_);
    p_.resize(K_);
    q_.resize(K_);
    r_fun_.resize(K_);
    retape_.resize(K_);
    tape_ok_.resize(K_);
    pattern_jac_r_.resize(K_);
    pattern_hes_r_.resize(K_);
    size_t max_p      = 0;
    size_t max_q      = 0;
    for(k = 0; k < K_; k++)
    {   L_[k]       = fg_info_->number_terms(k);
        p_[k]       = fg_info_->range_size(k);
        q_[k]       = fg_info_->domain_size(k);
        retape_[k]  = fg_info_->retape(k);
        max_p       = std::max(max_p, p_[k]);
        max_q       = std::max(max_q, q_[k]);
        pattern_jac_r_[k].resize( p_[k] * q_[k] );
        pattern_hes_r_[k].resize( q_[k] * q_[k] );
    }
    I_.resize(max_p);
    J_.resize(max_q);
# ifndef NDEBUG
    size_t i, j, ell;
    // check for valid range and domain indices
    for(k = 0; k < K_; k++) for(ell = 0; ell < L_[k]; ell++)
    {
        for( i = 0; i < p_[k]; i++)
            I_[i] = m+1; // an invalid range index
        for( j = 0; j < q_[k]; j++)
            J_[j] = n; // an invalid domain index
        fg_info_->index(k, ell, I_, J_);
        for( i = 0; i < p_[k]; i++) if( I_[i] > m )
        {   std::cerr << "k=" << k << ", ell=" << ell
            << ", I[" << i << "]=" << I_[i] << std::endl;
            CPPAD_ASSERT_KNOWN( I_[i] <= m,
            "cppad_ipopt_nlp: invalid value in index vector I"
            );
        }
        for( j = 0; j < q_[k]; j++) if( J_[j] >= n )
        {   std::cerr << "k=" << k << ", ell=" << ell
            << ", J[" << j << "]=" << J_[j] << std::endl;
            CPPAD_ASSERT_KNOWN( J_[j] < n,
            "cppad_ipopt_nlp: invalid value in index vector J"
            );
        }
    }
# endif
    // record r[k] for functions that do not need retaping
    for(k = 0; k < K_; k++)
    {   tape_ok_[k] = false;
        if( ! retape_[k] )
        {   // Operation sequence does not depend on value
            // of u so record it once here in the constructor.
            fg_info_->index(k, 0, I_, J_);
            fun_record(
                fg_info_        ,   // inputs
                k               ,
                p_              ,
                q_              ,
                n_              ,
                x_i_            ,
                J_              ,
                r_fun_              // output
            );
            // take time to optimize because only recording once
            r_fun_[k].optimize();
            // ok and will stay that way
            tape_ok_[k] = true;
        }
    }

    // compute a sparsity patterns for each r_k (u)
    vec_fun_pattern(
        K_, p_, q_, retape_, r_fun_,      // inputs
        pattern_jac_r_, pattern_hes_r_    // outputs
    );

    // mapping from (i,j) to Ipopt sparsity index for Jacobian of g
    jac_g_map(
        fg_info_, m_, n_, K_, L_, p_, q_, pattern_jac_r_,   // inputs
        I_, J_,                                             // work
        index_jac_g_                                        // outputs
    );

    // mapping from (i,j) to Ipopt sparsity index for Hessian of Lagragian
    hes_fg_map(
        fg_info_, m_, n_, K_, L_, p_, q_, pattern_hes_r_,   // inputs
        I_, J_,                                             // work
        index_hes_fg_                                       // outputs
    );

    // Compute Ipopt sparsity structure for Jacobian of g
    sparse_map2vec(
        index_jac_g_,                         // inputs
        nnz_jac_g_, iRow_jac_g_, jCol_jac_g_  // outputs
    );

    // Compute Ipopt sparsity structure for Hessian of Lagragian
    sparse_map2vec(
        index_hes_fg_,                        // inputs
        nnz_h_lag_, iRow_h_lag_, jCol_h_lag_  // outputs
    );
}

/// The destructor takes no special action.
cppad_ipopt_nlp::~cppad_ipopt_nlp()
{}

/*!
Return dimension information about optimization problem.

\param[out] n
is set to the value n_.

\param[out] m
is set to the value m_.

\param[out] nnz_jac_g
is set to the value of nnz_jac_g_.

\param[out] nnz_h_lag
is set to the vlaue of nnz_h_lag_.

\param[out] index_style
is set to C_STYLE; i.e., zeoro based indexing is used in the
information passed to Ipopt.
*/
bool cppad_ipopt_nlp::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n = static_cast<Index>( n_ );
    m = static_cast<Index>( m_ );
    nnz_jac_g = static_cast<Index>( nnz_jac_g_ );
    nnz_h_lag = static_cast<Index>( nnz_h_lag_ );

    // use the fortran index style for row/col entries
    index_style = C_STYLE;

    return true;
}

/*!
Return bound information about optimization problem.

\param[in] n
is the dimension of the domain space for f(x) and g(x); i.e.,
it must be equal to n_.

\param[out] x_l
is a vector of size n.
The input value of its elements does not matter.
On output, it is a copy of the lower bound for \f$ x \f$; i.e.,
 x_l_.

\param[out] x_u
is a vector of size n.
The input value of its elements does not matter.
On output, it is a copy of the upper bound for \f$ x \f$; i.e.,
 x_u_.

\param[in] m
is the dimension of the range space for g(x). i.e.,
it must be equal to m_.

\param[out] g_l
is a vector of size m.
The input value of its elements does not matter.
On output, it is a copy of the lower bound for \f$ g(x) \f$; i.e., g_l_.

\param[out] g_u
is a vector of size m.
The input value of its elements does not matter.
On output, it is a copy of the upper bound for \f$ g(x) \f$; i.e, g_u_.
*/
bool cppad_ipopt_nlp::get_bounds_info(Index n, Number* x_l, Number* x_u,
                            Index m, Number* g_l, Number* g_u)
{   size_t i, j;
    // here, the n and m we gave IPOPT in get_nlp_info are passed back
    CPPAD_ASSERT_UNKNOWN(size_t(n) == n_);
    CPPAD_ASSERT_UNKNOWN(size_t(m) == m_);

    // pass back bounds
    for(j = 0; j < n_; j++)
    {   x_l[j] = x_l_[j];
        x_u[j] = x_u_[j];
    }
    for(i = 0; i < m_; i++)
    {   g_l[i] = g_l_[i];
        g_u[i] = g_u_[i];
    }

    return true;
}

/*!
Return initial x value where optimiation is started.

\param[in] n
must be equal to the domain dimension for f(x) and g(x); i.e.,
it must be equal to n_.

\param[in] init_x
must be equal to true.

\param[out] x
is a vector of size n.
The input value of its elements does not matter.
On output, it is a copy of the initial value for \f$ x \f$; i.e. x_i_.

\param[in] init_z
must be equal to false.

\param z_L
is not used.

\param z_U
is not used.

\param[in] m
must be equal to the range dimension for g(x); i.e.,
it must be equal to m_.

\param init_lambda
must be equal to false.

\param lambda
is not used.
*/
bool cppad_ipopt_nlp::get_starting_point(Index n, bool init_x, Number* x,
                               bool init_z, Number* z_L, Number* z_U,
                               Index m, bool init_lambda,
                               Number* lambda)
{   size_t j;

    CPPAD_ASSERT_UNKNOWN(size_t(n) == n_ );
    CPPAD_ASSERT_UNKNOWN(size_t(m) == m_ );
    CPPAD_ASSERT_UNKNOWN(init_x == true);
    CPPAD_ASSERT_UNKNOWN(init_z == false);
    CPPAD_ASSERT_UNKNOWN(init_lambda == false);

    for(j = 0; j < n_; j++)
        x[j] = x_i_[j];

    return true;
}

/*!
Evaluate the objective fucntion f(x).

\param[in] n
is the dimension of the argument space for f(x); i.e., must be equal n_.

\param[in] x
is a vector of size n containing the point at which to evaluate
the function f(x).

\param[in] new_x
is false if the previous call to any one of the
\ref Deprecated_Evaluation_Methods used the same value for x.

\param[out] obj_value
is the value of the objective f(x) at this value of x.

\return
The return value is always true; see \ref Deprecated_Evaluation_Methods.

\par Efficiency
This routine could be more efficient
(for certain when when L[k] > 1 and retape[k] is true)
if the users also provided a version
of the function <tt>fg_info->eval_r(k, u)</tt> where u was of type
 NumberVector.
*/
bool cppad_ipopt_nlp::eval_f(
    Index n, const Number* x, bool new_x, Number& obj_value
)
{
    CPPAD_ASSERT_UNKNOWN(size_t(n) == n_ );

    size_t iobj, j, k, ell;

    // initialize summation
    obj_value = 0.;

    // update tape_ok_ flag
    for(k = 0; k < K_; k++)
    {   if( retape_[k] && (new_x || L_[k] > 1) )
            tape_ok_[k] = false;
    }

    for(k = 0; k < K_; k++) for(ell = 0; ell < L_[k]; ell++)
    {   fg_info_->index(k, ell, I_, J_);
        for(iobj = 0; iobj < p_[k]; iobj++) if( I_[iobj] == 0 )
        {   if( ! tape_ok_[k] )
            {   // Record r_k for value of u corresponding to x
                fun_record(
                    fg_info_        ,   // inputs
                    k               ,
                    p_              ,
                    q_              ,
                    n_              ,
                    x               ,
                    J_              ,
                    r_fun_             // output
                );
                tape_ok_[k] = ! (retape_[k] || L_[k] > 1);
            }
            NumberVector u(q_[k]);
            NumberVector r(p_[k]);
            for(j = 0; j < q_[k]; j++)
            {   CPPAD_ASSERT_UNKNOWN( J_[j] < n_ );
                u[j]   = x[ J_[j] ];
            }
            r          = r_fun_[k].Forward(0, u);
            obj_value += r[iobj];
        }
    }
# if CPPAD_IPOPT_NLP_TRACE
    using std::printf;
    for(j = 0; j < n_; j++)
        printf("cppad_ipopt_nlp::eval_f::x[%d] = %20.14g\n", j, x[j]);
    printf("cppad_ipopt_nlp::eval_f::obj_value = %20.14g\n", obj_value);
# endif
# ifndef NDEBUG
    CPPAD_ASSERT_KNOWN(
        (-infinity_ < obj_value) && (obj_value < infinity_),
        "cppad_ipopt_nlp::eval_f:: objective value is not finite"
    );
# endif
    return true;
}

/*!
Evaluate the gradient of f(x).

\param[in] n
is the dimension of the argument space for f(x); i.e., must be equal n_.

\param[in] x
has a vector of size n containing the point at which to evaluate
the gradient of f(x).

\param[in] new_x
is false if the previous call to any one of the
\ref Deprecated_Evaluation_Methods used the same value for x.

\param[out] grad_f
is a vector of size n.
The input value of its elements does not matter.
The output value of its elements is the gradient of f(x)
at this value of.

\return
The return value is always true; see \ref Deprecated_Evaluation_Methods.
*/
bool cppad_ipopt_nlp::eval_grad_f(
    Index n, const Number* x, bool new_x, Number* grad_f
)
{   CPPAD_ASSERT_UNKNOWN(size_t(n) == n_ );

    size_t iobj, i, j, k, ell;

    // initialize summation
    for(j = 0; j < n_; j++)
        grad_f[j] = 0.;

    // update tape_ok_ flag
    for(k = 0; k < K_; k++)
    {   if( retape_[k] && (new_x || L_[k] > 1) )
            tape_ok_[k] = false;
    }

    for(k = 0; k < K_; k++) for(ell = 0; ell < L_[k]; ell++)
    {   fg_info_->index(k, ell, I_, J_);
        for(iobj = 0; iobj < p_[k]; iobj++) if( I_[iobj] == 0 )
        {   if( ! tape_ok_[k] )
            {   // Record r_k for value of u corresponding to x
                fun_record(
                    fg_info_        ,   // inputs
                    k               ,
                    p_              ,
                    q_              ,
                    n_              ,
                    x               ,
                    J_              ,
                    r_fun_              // output
                );
                tape_ok_[k] = ! (retape_[k] || L_[k] > 1);
            }
            NumberVector u(q_[k]);
            NumberVector w(p_[k]);
            NumberVector r_grad(q_[k]);
            for(j = 0; j < q_[k]; j++)
            {   CPPAD_ASSERT_UNKNOWN( J_[j] < n_ );
                u[j]   = x[ J_[j] ];
            }
            r_fun_[k].Forward(0, u);
            for(i = 0; i < p_[k]; i++)
                w[i] = 0.;
            w[iobj]    = 1.;
            r_grad     = r_fun_[k].Reverse(1, w);
            for(j = 0; j < q_[k]; j++)
            {   CPPAD_ASSERT_UNKNOWN( J_[j] < n_ );
                grad_f[ J_[j] ]  += r_grad[j];
            }
        }
    }
# if CPPAD_IPOPT_NLP_TRACE
    using std::printf;
    for(j = 0; j < n_; j++) printf(
    "cppad_ipopt_nlp::eval_grad_f::x[%d] = %20.14g\n", j, x[j]
    );
    for(j = 0; j < n_; j++) printf(
    "cppad_ipopt_nlp::eval_grad_f::grad_f[%d] = %20.14g\n", j, grad_f[j]
    );
# endif
# ifndef NDEBUG
    for(j = 0; j < n_; j++) CPPAD_ASSERT_KNOWN(
        (-infinity_ < grad_f[j]) && (grad_f[j] < infinity_),
        "cppad_ipopt_nlp::grad_f:: gradient of objective is not finite"
    );
# endif
    return true;
}

/*!
Evaluate the function g(x).

\param[in] n
is the dimension of the argument space for g(x); i.e., must be equal n_.

\param[in] x
has a vector of size n containing the point at which to evaluate
the constraint function g(x).

\param[in] new_x
is false if the previous call to any one of the
\ref Deprecated_Evaluation_Methods used the same value for x.

\param[in] m
is the dimension of the range space for g(x); i.e., must be equal to m_.

\param[out] g
is a vector of size m.
The input value of its elements does not matter.
The output value of its elements is
the value of the function g(x) at this value of x.

\return
The return value is always true; see \ref Deprecated_Evaluation_Methods.
*/
bool cppad_ipopt_nlp::eval_g(
    Index n, const Number* x, bool new_x, Index m, Number* g
)
{   CPPAD_ASSERT_UNKNOWN(size_t(n) == n_ );

    size_t i, j, k, ell;

    // initialize summation
    for(i = 0; i < m_; i++)
        g[i] = 0.;

    // update tape_ok_ flag
    for(k = 0; k < K_; k++)
    {   if( retape_[k] && (new_x || L_[k] > 1) )
            tape_ok_[k] = false;
    }

    for(k = 0; k < K_; k++) for(ell = 0; ell < L_[k]; ell++)
    {   fg_info_->index(k, ell, I_, J_);
        if( ! tape_ok_[k] )
        {   // Record r_k for value of u corresponding to x
            fun_record(
                fg_info_        ,   // inputs
                k               ,
                p_              ,
                q_              ,
                n_              ,
                x               ,
                J_              ,
                r_fun_              // output
            );
        }
        tape_ok_[k] = ! (retape_[k] || L_[k] > 1);
        NumberVector u(q_[k]);
        NumberVector r(p_[k]);
        for(j = 0; j < q_[k]; j++)
        {   CPPAD_ASSERT_UNKNOWN( J_[j] < n_ );
            u[j]   = x[ J_[j] ];
        }
        r   = r_fun_[k].Forward(0, u);
        for(i = 0; i < p_[k]; i++)
        {   CPPAD_ASSERT_UNKNOWN( I_[i] <= m_ );
            if( I_[i] >= 1 )
                g[ I_[i] - 1 ] += r[i];
        }
    }
# if CPPAD_IPOPT_NLP_TRACE
    using std::printf;
    for(j = 0; j < n_; j++)
        printf("cppad_ipopt_nlp::eval_g::x[%d] = %20.14g\n", j, x[j]);
    for(i = 0; i < m_; i++)
        printf("cppad_ipopt_nlp::eval_g::g[%d] = %20.14g\n", i, g[i]);
# endif
# ifndef NDEBUG
    for(i = 0; i < m_; i++) CPPAD_ASSERT_KNOWN(
        (-infinity_ < g[i]) && (g[i] < infinity_),
        "cppad_ipopt_nlp::eval_g:: not all constraints are not finite"
    );
# endif
    return true;
}

/*!
Evaluate the Jacobian of g(x).

\param[in] n
is the dimension of the argument space for g(x); i.e., must be equal n_.

\param x
if values is not NULL,
 x is a vector of size n containing the point at which to evaluate
the gradient of g(x).

\param[in] new_x
is false if the previous call to any one of the
\ref Deprecated_Evaluation_Methods used the same value for x.

\param[in] m
is the dimension of the range space for g(x); i.e., must be equal to m_.

\param[in] nele_jac
is the number of possibly non-zero elements in the Jacobian of g(x);
i.e., must be equal to nnz_jac_g_.

\param iRow
if values is not NULL, iRow is not defined.
if values is NULL, iRow
is a vector with size nele_jac.
The input value of its elements does not matter.
On output,
For <tt>k = 0 , ... , nele_jac-1, iRow[k]</tt> is the
base zero row index for the
k-th possibly non-zero entry in the Jacobian of g(x).

\param jCol
if values is not NULL, jCol is not defined.
if values is NULL, jCol
is a vector with size nele_jac.
The input value of its elements does not matter.
On output,
For <tt>k = 0 , ... , nele_jac-1, jCol[k]</tt> is the
base zero column index for the
k-th possibly non-zero entry in the Jacobian of g(x).

\param values
if values is not NULL, values
is a vector with size nele_jac.
The input value of its elements does not matter.
On output,
For <tt>k = 0 , ... , nele_jac-1, values[k]</tt> is the
value for the
k-th possibly non-zero entry in the Jacobian of g(x).

\return
The return value is always true; see \ref Deprecated_Evaluation_Methods.
*/
bool cppad_ipopt_nlp::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{   CPPAD_ASSERT_UNKNOWN(size_t(m)          == m_ );
    CPPAD_ASSERT_UNKNOWN(size_t(n)          == n_ );
    CPPAD_ASSERT_UNKNOWN( size_t(nele_jac)  == nnz_jac_g_ );

    size_t i, j, k, ell, l;
    std::map<size_t,size_t>::iterator index_ij;


    if (values == NULL)
    {   for(k = 0; k < nnz_jac_g_; k++)
        {   iRow[k] = static_cast<Index>( iRow_jac_g_[k] );
            jCol[k] = static_cast<Index>( jCol_jac_g_[k] );
        }
        return true;
    }

    // initialize summation
    l = nnz_jac_g_;
    while(l--)
        values[l] = 0.;

    // update tape_ok_ flag
    for(k = 0; k < K_; k++)
    {   if( retape_[k] && (new_x || L_[k] > 1) )
            tape_ok_[k] = false;
    }

    for(k = 0; k < K_; k++) for(ell = 0; ell < L_[k]; ell++)
    {   fg_info_->index(k, ell, I_, J_);
        if( ! tape_ok_[k] )
        {   // Record r_k for value of u corresponding to x
            fun_record(
                fg_info_        ,   // inputs
                k               ,
                p_              ,
                q_              ,
                n_              ,
                x               ,
                J_              ,
                r_fun_              // output
            );
        }
        tape_ok_[k] = ! (retape_[k] || L_[k] > 1);
        NumberVector u(q_[k]);
        NumberVector jac_r(p_[k] * q_[k]);
        for(j = 0; j < q_[k]; j++)
        {   CPPAD_ASSERT_UNKNOWN( J_[j] < n_ );
            u[j]   = x[ J_[j] ];
        }
        if( retape_[k] )
            jac_r = r_fun_[k].Jacobian(u);
        else
            jac_r = r_fun_[k].SparseJacobian(u, pattern_jac_r_[k]);
        for(i = 0; i < p_[k]; i++) if( I_[i] != 0 )
        {   CPPAD_ASSERT_UNKNOWN( I_[i] <= m_ );
            for(j = 0; j < q_[k]; j++)
            {   index_ij = index_jac_g_[I_[i]-1].find(J_[j]);
                if( index_ij != index_jac_g_[I_[i]-1].end() )
                {   l          = index_ij->second;
                    values[l] += jac_r[i * q_[k] + j];
                }
                else
                    CPPAD_ASSERT_UNKNOWN(
                    jac_r[i * q_[k] + j] == 0.
                );
            }
        }
    }
# ifndef NDEBUG
    for(l = 0; l < nnz_jac_g_; l++) CPPAD_ASSERT_KNOWN(
        (-infinity_ < values[l]) && (values[l] < infinity_),
        "cppad_ipopt_nlp::eval_jac_g:: a component of "
        "gradient of g is not finite"
    );
# endif

    return true;
}

/*!
Evaluate the Hessian of the Lagragian

\section Deprecated_Hessian_of_the_Lagragian The Hessian of the Lagragian
The Hessian of the Lagragian is defined as
\f[
H(x, \sigma, \lambda )
=
\sigma \nabla^2 f(x) + \sum_{i=0}^{m-1} \lambda_i \nabla^2 g(x)_i
\f]

\param[in] n
is the dimension of the argument space for g(x); i.e., must be equal n_.

\param x
if values is not NULL, x
is a vector of size n containing the point at which to evaluate
the Hessian of the Lagrangian.

\param[in] new_x
is false if the previous call to any one of the
\ref Deprecated_Evaluation_Methods used the same value for x.

\param[in] obj_factor
the value \f$ \sigma \f$ multiplying the Hessian of
f(x) in the expression for \ref Deprecated_Hessian_of_the_Lagragian.

\param[in] m
is the dimension of the range space for g(x); i.e., must be equal to m_.

\param[in] lambda
if values is not NULL, lambda
is a vector of size m specifing the value of \f$ \lambda \f$
in the expression for \ref Deprecated_Hessian_of_the_Lagragian.

\param[in] new_lambda
is true if the previous call to eval_h had the same value for
 lambda and false otherwise.
(Not currently used.)

\param[in] nele_hess
is the number of possibly non-zero elements in the Hessian of the Lagragian;
i.e., must be equal to nnz_h_lag_.

\param iRow
if values is not NULL, iRow is not defined.
if values is NULL, iRow
is a vector with size nele_jac.
The input value of its elements does not matter.
On output,
For <tt>k = 0 , ... , nele_jac-1, iRow[k]</tt> is the
base zero row index for the
k-th possibly non-zero entry in the Hessian of the Lagragian.

\param jCol
if values is not NULL, jCol is not defined.
if values is NULL, jCol
is a vector with size nele_jac.
The input value of its elements does not matter.
On output,
For <tt>k = 0 , ... , nele_jac-1, jCol[k]</tt> is the
base zero column index for the
k-th possibly non-zero entry in the Hessian of the Lagragian.

\param values
if values is not NULL, it
is a vector with size nele_jac.
The input value of its elements does not matter.
On output,
For <tt>k = 0 , ... , nele_jac-1, values[k]</tt> is the
value for the
k-th possibly non-zero entry in the Hessian of the Lagragian.

\return
The return value is always true; see \ref Deprecated_Evaluation_Methods.
*/
bool cppad_ipopt_nlp::eval_h(Index n, const Number* x, bool new_x,
                   Number obj_factor, Index m, const Number* lambda,
                   bool new_lambda, Index nele_hess, Index* iRow,
                   Index* jCol, Number* values)
{   CPPAD_ASSERT_UNKNOWN(size_t(m) == m_ );
    CPPAD_ASSERT_UNKNOWN(size_t(n) == n_ );

    size_t i, j, k, ell, l;
    std::map<size_t,size_t>::iterator index_ij;

    if (values == NULL)
    {   for(k = 0; k < nnz_h_lag_; k++)
        {   iRow[k] = static_cast<Index>( iRow_h_lag_[k] );
            jCol[k] = static_cast<Index>( jCol_h_lag_[k] );
        }
        return true;
    }

    // initialize summation
    l = nnz_h_lag_;
    while(l--)
        values[l] = 0.;

    // update tape_ok_ flag
    for(k = 0; k < K_; k++)
    {   if( retape_[k] && (new_x || L_[k] > 1) )
            tape_ok_[k] = false;
    }

    for(k = 0; k < K_; k++) for(ell = 0; ell < L_[k]; ell++)
    {   fg_info_->index(k, ell, I_, J_);
        bool in_use = false;
        for(i = 0; i < p_[k]; i++)
        {   if( I_[i] == 0 )
                in_use |= obj_factor > 0.;
            else
                in_use |= lambda[ I_[i] - 1 ] > 0;
        }
        if( in_use )
        {
            if( ! tape_ok_[k]  )
            {   // Record r_k for value of u corresponding to x
                fun_record(
                    fg_info_        ,   // inputs
                    k               ,
                    p_              ,
                    q_              ,
                    n_              ,
                    x               ,
                    J_              ,
                    r_fun_              // output
                );
                tape_ok_[k] = ! (retape_[k] || L_[k] > 1);
            }
            NumberVector w(p_[k]);
            NumberVector r_hes(q_[k] * q_[k]);
            NumberVector u(q_[k]);
            for(j = 0; j < q_[k]; j++)
            {   CPPAD_ASSERT_UNKNOWN( J_[j] < n_ );
                u[j]   = x[ J_[j] ];
            }
            for(i = 0; i < p_[k]; i++)
            {   CPPAD_ASSERT_UNKNOWN( I_[i] <= m_ );
                if( I_[i] == 0 )
                    w[i] = obj_factor;
                else
                    w[i] = lambda[ I_[i] - 1 ];
            }
            if( retape_[k] )
                r_hes = r_fun_[k].Hessian(u, w);
            else
                r_hes = r_fun_[k].SparseHessian(
                    u, w, pattern_hes_r_[k]
            );
            for(i = 0; i < q_[k]; i++) for(j = 0; j < q_[k]; j++)
            if( J_[j] <= J_[i] )
            {   index_ij = index_hes_fg_[J_[i]].find(J_[j]);
                if( index_ij != index_hes_fg_[J_[i]].end() )
                {   l          = index_ij->second;
                    values[l] += r_hes[i * q_[k] + j];
                }
                else
                    CPPAD_ASSERT_UNKNOWN(
                    r_hes[i * q_[k] + j] == 0.
                );
            }
        }
    }
# ifndef NDEBUG
    for(l = 0; l < nnz_h_lag_; l++) CPPAD_ASSERT_KNOWN(
        (-infinity_ < values[l]) && (values[l] < infinity_),
        "cppad_ipopt_nlp::eval_h:: a component of "
        "Hessian of Lagragian is not finite"
    );
# endif
    return true;
}

/*!
Pass solution information from Ipopt to users solution structure.

\param[in] status
is value that the Ipopt solution status
which gets mapped to a correponding value for
\n
<tt>solution_->status</tt>.

\param[in] n
is the dimension of the domain space for f(x) and g(x); i.e.,
it must be equal to n_.

\param[in] x
is a vector with size n specifing the final solution.
\n
<tt>solution_->x</tt> is set to be a vector with size n
and to have the same element values.

\param[in] z_L
is a vector with size n specifing the Lagragian multipliers for the
constraint \f$ x^l \leq x \f$.
\n
<tt>solution_->z_l</tt> is set to be a vector with size n
and to have the same element values.

\param[in] z_U
is a vector with size n specifing the Lagragian multipliers for the
constraint \f$ x \leq x^u \f$.
\n
<tt>solution_->z_u</tt> is set to be a vector with size n
and to have the same element values.

\param[in] m
is the dimension of the range space for g(x). i.e.,
it must be equal to m_.

\param[in] g
is a vector with size m containing the value of the constraint function
g(x) at the final solution for x.
\n
<tt>solution_->g</tt> is set to be a vector with size m
and to have the same element values.

\param[in] lambda
is a vector with size m specifing the Lagragian multipliers for the
constraints \f$ g^l \leq g(x) \leq g^u \f$.
\n
<tt>solution_->lambda</tt> is set to be a vector with size m
and to have the same element values.

\param[in] obj_value
is the value of the objective function f(x) at the final solution for x.
\n
<tt>solution_->obj_value</tt> is set to have the same value.

\param[in] ip_data
is unspecified (by Ipopt) and hence not used.

\param[in] ip_cq
is unspecified (by Ipopt) and hence not used.

\par solution_[out]
the pointer solution_ , which is equal to the pointer solution
in the constructor for cppad_ipopt_nlp,
is used to set output values (see documentation above).
*/
void cppad_ipopt_nlp::finalize_solution(
    Ipopt::SolverReturn               status    ,
    Index                             n         ,
    const Number*                     x         ,
    const Number*                     z_L       ,
    const Number*                     z_U       ,
    Index                             m         ,
    const Number*                     g         ,
    const Number*                     lambda    ,
    Number                            obj_value ,
    const Ipopt::IpoptData*           ip_data   ,
    Ipopt::IpoptCalculatedQuantities* ip_cq
)
{   size_t i, j;

    CPPAD_ASSERT_UNKNOWN(size_t(n) == n_ );
    CPPAD_ASSERT_UNKNOWN(size_t(m) == m_ );

    switch(status)
    {   // convert status from Ipopt enum to cppad_ipopt_solution enum
        case Ipopt::SUCCESS:
        solution_->status =
            cppad_ipopt_solution::success;
        break;

        case Ipopt::MAXITER_EXCEEDED:
        solution_->status =
            cppad_ipopt_solution::maxiter_exceeded;
        break;

        case Ipopt::STOP_AT_TINY_STEP:
        solution_->status =
            cppad_ipopt_solution::stop_at_tiny_step;
        break;

        case Ipopt::STOP_AT_ACCEPTABLE_POINT:
        solution_->status =
            cppad_ipopt_solution::stop_at_acceptable_point;
        break;

        case Ipopt::LOCAL_INFEASIBILITY:
        solution_->status =
            cppad_ipopt_solution::local_infeasibility;
        break;

        case Ipopt::USER_REQUESTED_STOP:
        solution_->status =
            cppad_ipopt_solution::user_requested_stop;
        break;

        case Ipopt::DIVERGING_ITERATES:
        solution_->status =
            cppad_ipopt_solution::diverging_iterates;
        break;

        case Ipopt::RESTORATION_FAILURE:
        solution_->status =
            cppad_ipopt_solution::restoration_failure;
        break;

        case Ipopt::ERROR_IN_STEP_COMPUTATION:
        solution_->status =
            cppad_ipopt_solution::error_in_step_computation;
        break;

        case Ipopt::INVALID_NUMBER_DETECTED:
        solution_->status =
            cppad_ipopt_solution::invalid_number_detected;
        break;

        case Ipopt::INTERNAL_ERROR:
        solution_->status =
            cppad_ipopt_solution::internal_error;
        break;

        default:
        solution_->status =
            cppad_ipopt_solution::unknown;
    }

    solution_->x.resize(n_);
    solution_->z_l.resize(n_);
    solution_->z_u.resize(n_);
    for(j = 0; j < n_; j++)
    {   solution_->x[j]    = x[j];
        solution_->z_l[j]  = z_L[j];
        solution_->z_u[j]  = z_U[j];
    }
    solution_->g.resize(m_);
    solution_->lambda.resize(m_);
    for(i = 0; i < m_; i++)
    {   solution_->g[i]      = g[i];
        solution_->lambda[i] = lambda[i];
    }
    solution_->obj_value = obj_value;
    return;
}

// This routine is defined, but not yet used
// (trying to figure out a problem with Ipopt-3.9.1 and dismod4).
bool cppad_ipopt_nlp::intermediate_callback(
    Ipopt::AlgorithmMode              mode,
    Index                             iter,
    Number                            obj_value,
    Number                            inf_pr,
    Number                            inf_du,
    Number                            mu,
    Number                            d_norm,
    Number                            regularization_size,
    Number                            alpha_du,
    Number                            alpha_pr,
    Index                             ls_trials,
    const Ipopt::IpoptData*           ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq
)
{
    // std::cout << "intermediate_callback called" << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
} // end namespace cppad_ipopt
// ---------------------------------------------------------------------------
