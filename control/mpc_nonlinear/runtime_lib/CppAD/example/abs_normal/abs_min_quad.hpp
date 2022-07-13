# ifndef CPPAD_EXAMPLE_ABS_NORMAL_ABS_MIN_QUAD_HPP
# define CPPAD_EXAMPLE_ABS_NORMAL_ABS_MIN_QUAD_HPP
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
$begin abs_min_quad$$
$spell
    hpp
    qp
    jac
    Jacobian
    maxitr
$$
$section abs_normal: Minimize a Linear Abs-normal Approximation$$

$head Syntax$$
$icode%ok% = abs_min_quad(
    %level%, %n%, %m%, %s%,
    %g_hat%, %g_jac%, %hessian%, %bound%, %epsilon%, %maxitr%, %delta_x%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%
1%$$

$head Source$$
This following is a link to the source code for this example:
$cref/abs_min_quad.hpp/abs_min_quad.hpp/$$.

$head Purpose$$
We are given a point $latex \hat{x} \in \B{R}^n$$ and
use the notation $latex \tilde{f} (x)$$ for the abs-normal
$cref/approximation for f(x)
    /abs_normal_fun
    /Abs-normal Approximation
    /Approximating f(x)
/$$
near $latex \hat{x}$$.
We are also given a vector $latex b \in \B{R}_+^n$$
and a positive definite matrix $latex H \in \B{R}^{n \times n}$$.
This routine solves the problem
$latex \[
\begin{array}{lll}
\R{minimize} &
    \Delta x^T H \Delta x / 2 + \tilde{f}( \hat{x} + \Delta x ) &
    \R{w.r.t} \; \Delta x \in \B{R}^n
\\
\R{subject \; to} & | \Delta x_j | \leq b_j & j = 0 , \ldots , n-1
\end{array}
\] $$

$head DblVector$$
is a $cref SimpleVector$$ class with elements of type $code double$$.

$head SizeVector$$
is a $cref SimpleVector$$ class with elements of type $code size_t$$.

$head f$$
We use the notation $icode f$$ for the original function; see
$cref/f/abs_normal_fun/f/$$.

$head level$$
This value is less that or equal 3.
If $icode%level% == 0%$$,
no tracing of the optimization is printed.
If $icode%level% >= 1%$$,
a trace of each iteration of $code abs_min_quad$$ is printed.
If $icode%level% >= 2%$$,
a trace of the $cref qp_box$$ sub-problem is printed.
If $icode%level% >= 3%$$,
a trace of the $cref qp_interior$$ sub-problem is printed.

$head n$$
This is the dimension of the domain space for $icode f$$; see
$cref/n/abs_normal_fun/f/n/$$.

$head m$$
This is the dimension of the range space for $icode f$$; see
$cref/m/abs_normal_fun/f/m/$$. This must be one so that $latex f$$
is an objective function.

$head s$$
This is the number of absolute value terms in $icode f$$; see
$cref/s/abs_normal_fun/f/s/$$.

$head g$$
We use the notation $icode g$$ for the abs-normal representation of $icode f$$;
see $cref/g/abs_normal_fun/g/$$.

$head g_hat$$
This vector has size $icode%m% + %s%$$ and is the value of
$icode g(x, u)$$ at $latex x = \hat{x}$$ and $latex u = a( \hat{x} )$$.

$head g_jac$$
This vector has size $codei%(%m% + %s%) * (%n% + %s%)%$$ and is the Jacobian of
$latex g(x, u)$$ at $latex x = \hat{x}$$ and $latex u = a( \hat{x} )$$.

$head hessian$$
This vector has size $icode%n% * %n%$$.
It is a $cref/row-major/glossary/Row-major Representation/$$ representation
of the matrix $latex H \in \B{R}^{n \times n}$$.

$head bound$$
This vector has size $icode n$$ and is the vector $latex b \in \B{R}^n$$.
The trust region is defined as the set of $latex \Delta x$$ such that
$latex \[
    | \Delta x | \leq b_j
\]$$
for $latex j = 0 , \ldots , n-1$$.

$head epsilon$$
The value $icode%epsilon%[0]%$$ is convergence criteria in terms
of the infinity norm of the difference of $icode delta_x$$
between iterations.
The value $icode%epsilon%[1]%$$ is convergence criteria in terms
of the derivative of the objective; i.e.
$latex \[
    \Delta x^T H \Delta x / 2 + \tilde{f}( \hat{x} + \Delta x)
\] $$

$head maxitr$$
This is a vector with size 2.
The value $icode%maxitr%[0]%$$ is the maximum number of
$code abs_min_quad$$ iterations to try before giving up on convergence.
The value $icode%maxitr%[1]%$$ is the maximum number of iterations in
the $cref/qp_interior/qp_interior/maxitr/$$ sub-problems.

$head delta_x$$
This vector $latex \Delta x$$ has size $icode n$$.
The input value of its elements does not matter.
Upon return,
the approximate minimizer of the objective with respect to the trust region.

$head Method$$

$subhead sigma$$
We use the notation
$latex \[
    \sigma (x) = \R{sign} ( z[ x , a(x) ] )
\] $$
where
$cref/a(x)/abs_normal_fun/a/a(x)/$$ and
$cref/z(x, u)/abs_normal_fun/g/z(x, u)/$$
are as defined in the abs-normal representation of $latex f(x)$$.

$subhead Cutting Planes$$
At each iteration,
we are given affine functions $latex p_k (x)$$
such that $latex p_k ( x_k ) = \tilde{f}( x_k )$$  and
$latex p_k^{(1)} ( x_k )$$ is the derivative $latex \tilde{f}^{(1)} ( x_k )$$
corresponding to $latex \sigma ( x_k )$$.

$subhead Iteration$$
At iteration $latex k$$, we solve the problem
$latex \[
\begin{array}{lll}
\R{minimize}
& \Delta x^T H \Delta x / 2 +
    \max \{ p_k ( \hat{x} + \Delta x) \W{:} k = 0 , \ldots , K-1 \}
& \R{w.r.t} \; \Delta x
\\
\R{subject \; to} & - b \leq \Delta x \leq + b
\end{array}
\] $$
The solution is the new point $latex x_K$$
at which the new affine approximation
$latex p_K (x)$$ is constructed.
This process is iterated until the difference
$latex x_K - x_{K-1}$$ is small enough.


$children%example/abs_normal/abs_min_quad.cpp
    %example/abs_normal/abs_min_quad.omh
%$$
$head Example$$
The file $cref abs_min_quad.cpp$$ contains an example and test of
$code abs_min_quad$$.

$end
-----------------------------------------------------------------------------
*/
# include <cppad/cppad.hpp>
# include "qp_box.hpp"
# include "abs_eval.hpp"

// BEGIN C++
namespace CppAD { // BEGIN_CPPAD_NAMESPACE

// BEGIN PROTOTYPE
template <class DblVector, class SizeVector>
bool abs_min_quad(
    size_t            level   ,
    size_t            n       ,
    size_t            m       ,
    size_t            s       ,
    const DblVector&  g_hat   ,
    const DblVector&  g_jac   ,
    const DblVector&  hessian ,
    const DblVector&  bound   ,
    const DblVector&  epsilon ,
    const SizeVector& maxitr  ,
    DblVector&        delta_x )
// END PROTOTYPE
{   using std::fabs;
    bool ok    = true;
    double inf = std::numeric_limits<double>::infinity();
    //
    CPPAD_ASSERT_KNOWN(
        level <= 4,
        "abs_min_quad: level is not less that or equal 3"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(epsilon.size()) == 2,
        "abs_min_quad: size of epsilon not equal to 2"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(maxitr.size()) == 2,
        "abs_min_quad: size of maxitr not equal to 2"
    );
    CPPAD_ASSERT_KNOWN(
        m == 1,
        "abs_min_quad: m is not equal to 1"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(delta_x.size()) == n,
        "abs_min_quad: size of delta_x not equal to n"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(bound.size()) == n,
        "abs_min_quad: size of bound not equal to n"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(g_hat.size()) == m + s,
        "abs_min_quad: size of g_hat not equal to m + s"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(g_jac.size()) == (m + s) * (n + s),
        "abs_min_quad: size of g_jac not equal to (m + s)*(n + s)"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(hessian.size()) == n * n,
        "abs_min_quad: size of hessian not equal to n * n"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(bound.size()) == n,
        "abs_min_quad: size of bound is not equal to n"
    );
    if( level > 0 )
    {   std::cout << "start abs_min_quad\n";
        CppAD::abs_print_mat("g_hat", m + s, 1, g_hat);
        CppAD::abs_print_mat("g_jac", m + s, n + s, g_jac);
        CppAD::abs_print_mat("hessian", n, n, hessian);
        CppAD::abs_print_mat("bound", n, 1, bound);
    }
    // partial y(x, u) w.r.t x (J in reference)
    DblVector py_px(n);
    for(size_t j = 0; j < n; j++)
        py_px[ j ] = g_jac[ j ];
    //
    // partial y(x, u) w.r.t u (Y in reference)
    DblVector py_pu(s);
    for(size_t j = 0; j < s; j++)
        py_pu[ j ] = g_jac[ n + j ];
    //
    // partial z(x, u) w.r.t x (Z in reference)
    DblVector pz_px(s * n);
    for(size_t i = 0; i < s; i++)
    {   for(size_t j = 0; j < n; j++)
        {   pz_px[ i * n + j ] = g_jac[ (n + s) * (i + m) + j ];
        }
    }
    // partial z(x, u) w.r.t u (L in reference)
    DblVector pz_pu(s * s);
    for(size_t i = 0; i < s; i++)
    {   for(size_t j = 0; j < s; j++)
        {   pz_pu[ i * s + j ] = g_jac[ (n + s) * (i + m) + n + j ];
        }
    }
    // initailize delta_x
    for(size_t j = 0; j < n; j++)
        delta_x[j] = 0.0;
    //
    // current set of cutting planes
    DblVector C(maxitr[0] * n), c(maxitr[0]);
    //
    // value of abs-normal approximation at x_hat + delta_x
    DblVector g_tilde = CppAD::abs_eval(n, m, s, g_hat, g_jac, delta_x);
    //
    // value of sigma at delta_x = 0; i.e., sign( z(x, u) )
    CppAD::vector<double> sigma(s);
    for(size_t i = 0; i < s; i++)
        sigma[i] = CppAD::sign( g_tilde[m + i] );
    //
    // initial value of the objective
    double obj_cur =  g_tilde[0];
    //
    // initial number of cutting planes
    size_t n_plane = 0;
    //
    if( level > 0 )
    {   std::cout << "obj = " << obj_cur << "\n";
        CppAD::abs_print_mat("delta_x", n, 1, delta_x);
    }
    for(size_t itr = 0; itr < maxitr[0]; itr++)
    {
        // Equation (5), Propostion 3.1 of reference
        // dy_dx = py_px + py_pu * Sigma * (I - pz_pu * Sigma)^-1 * pz_px
        //
        // tmp_ss = I - pz_pu * Sigma
        DblVector tmp_ss(s * s);
        for(size_t i = 0; i < s; i++)
        {   for(size_t j = 0; j < s; j++)
                tmp_ss[i * s + j] = - pz_pu[i * s + j] * sigma[j];
            tmp_ss[i * s + i] += 1.0;
        }
        // tmp_sn = (I - pz_pu * Sigma)^-1 * pz_px
        double logdet;
        DblVector tmp_sn(s * n);
        LuSolve(s, n, tmp_ss, pz_px, tmp_sn, logdet);
        //
        // tmp_sn = Sigma * (I - pz_pu * Sigma)^-1 * pz_px
        for(size_t i = 0; i < s; i++)
        {   for(size_t j = 0; j < n; j++)
                tmp_sn[i * n + j] *= sigma[i];
        }
        // dy_dx = py_px + py_pu * Sigma * (I - pz_pu * Sigma)^-1 * pz_px
        DblVector dy_dx(n);
        for(size_t j = 0; j < n; j++)
        {   dy_dx[j] = py_px[j];
            for(size_t k = 0; k < s; k++)
                dy_dx[j] += py_pu[k] * tmp_sn[ k * n + j];
        }
        //
        // compute derivative of the quadratic term
        DblVector dq_dx(n);
        for(size_t j = 0; j < n; j++)
        {   dq_dx[j] = 0.0;
            for(size_t i = 0; i < n; i++)
                dq_dx[j] += delta_x[i] * hessian[i * n + j];
        }
        //
        // check for case where derivative of objective is zero
        // (in convex case, this is the minimizer)
        bool near_zero = true;
        for(size_t j = 0; j < n; j++)
            near_zero &= std::fabs( dq_dx[j] + dy_dx[j] ) < epsilon[1];
        if( near_zero )
        {   if( level > 0 )
                std::cout << "end abs_min_quad: local derivative near zero\n";
            return true;
        }
        // value of hyperplane at delta_x
        double plane_at_zero = g_tilde[0];
        //
        // value of hyperplane at 0
        for(size_t j = 0; j < n; j++)
            plane_at_zero -= dy_dx[j] * delta_x[j];
        //
        // add a cutting plane with value g_tilde[0] at delta_x
        // and derivative dy_dx
        c[n_plane] = plane_at_zero;
        for(size_t j = 0; j < n; j++)
            C[n_plane * n + j] = dy_dx[j];
        ++n_plane;
        //
        // variables for cutting plane problem are (dx, w)
        // c[i] + C[i,:] * dx <= w
        DblVector c_box(n_plane), C_box(n_plane * (n + 1));
        for(size_t i = 0; i < n_plane; i++)
        {   c_box[i] = c[i];
            for(size_t j = 0; j < n; j++)
                C_box[i * (n+1) + j] = C[i * n + j];
            C_box[i * (n+1) + n] = -1.0;
        }
        //
        // w is the objective
        DblVector g_box(n + 1);
        for(size_t i = 0; i < size_t(c_box.size()); i++)
            g_box[i] = 0.0;
        g_box[n] = 1.0;
        //
        // a_box, b_box
        DblVector a_box(n+1), b_box(n+1);
        for(size_t j = 0; j < n; j++)
        {   a_box[j] = - bound[j];
            b_box[j] = + bound[j];
        }
        a_box[n] = - inf;
        b_box[n] = + inf;
        //
        // initial delta_x in qp_box is zero
        DblVector xin_box(n + 1);
        for(size_t j = 0; j < n; j++)
            xin_box[j] = 0.0;
        // initial w in qp_box is 1 + max_i c[i]
        xin_box[n] = 1.0 + c_box[0];
        for(size_t i = 1; i < n_plane; i++)
            xin_box[n] = std::max( xin_box[n], 1.0 + c_box[i] );
        //
        DblVector hessian_box( (n+1) * (n+1) );
        for(size_t i = 0; i < n+1; i++)
        {   for(size_t j = 0; j < n+1; j++)
            {   if( i == n || j == n )
                    hessian_box[i * (n+1) + j] = 0.0;
                else
                    hessian_box[i * (n+1) + j] = hessian[i * n + j];
            }
        }
        //
        // solve the cutting plane problem
        DblVector xout_box(n + 1);
        size_t level_box = 0;
        if( level > 0 )
            level_box = level - 1;
        ok &= CppAD::qp_box(
            level_box,
            a_box,
            b_box,
            c_box,
            C_box,
            g_box,
            hessian_box,
            epsilon[1],
            maxitr[1],
            xin_box,
            xout_box
        );
        if( ! ok )
        {   if( level > 0 )
            {   CppAD::abs_print_mat("delta_x", n, 1, delta_x);
                std::cout << "end abs_min_quad: qp_box failed\n";
            }
            return false;
        }
        DblVector delta_new(n);
        for(size_t j = 0; j < n; j++)
            delta_new[j] = xout_box[j];
        //
        // check for convergence
        double max_diff = 0.0;
        for(size_t j = 0; j < n; j++)
        {   double diff = delta_x[j] - delta_new[j];
            max_diff    = std::max( max_diff, std::fabs(diff) );
        }
        //
        // new value of the objective
        DblVector g_new   = CppAD::abs_eval(n, m, s, g_hat, g_jac, delta_new);
        double    obj_new = g_new[0];
        for(size_t i = 0; i < n; i++)
        {   for(size_t j = 0; j < n; j++)
                obj_new += delta_new[i] * hessian[i * n + j] * delta_new[j];
        }
        g_tilde = g_new;
        obj_cur = obj_new;
        delta_x = delta_new;
        //
        if( level > 0 )
        {   std::cout << "itr = " << itr << ", max_diff = " << max_diff
                << ", obj_cur = " << obj_cur << "\n";
            CppAD::abs_print_mat("delta_x", n, 1, delta_x);
        }
        //
        // value of sigma at new delta_x; i.e., sign( z(x, u) )
        for(size_t i = 0; i < s; i++)
            sigma[i] = CppAD::sign( g_tilde[m + i] );
        //
        if( max_diff < epsilon[0] )
        {   if( level > 0 )
                std::cout << "end abs_min_quad: change in delta_x near zero\n";
            return true;
        }
    }
    if( level > 0 )
        std::cout << "end abs_min_quad: maximum number of iterations exceeded\n";
    return false;
}
} // END_CPPAD_NAMESPACE
// END C++

# endif
