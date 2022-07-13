# ifndef CPPAD_EXAMPLE_ABS_NORMAL_ABS_MIN_LINEAR_HPP
# define CPPAD_EXAMPLE_ABS_NORMAL_ABS_MIN_LINEAR_HPP
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
$begin abs_min_linear$$
$spell
    hpp
    jac
    Jacobian
    maxitr
$$
$section abs_normal: Minimize a Linear Abs-normal Approximation$$

$head Syntax$$
$icode%ok% = abs_min_linear(
    %level%, %n%, %m%, %s%,
    %g_hat%, %g_jac%, %bound%, %epsilon%, %maxitr%, %delta_x%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%
1%$$

$head Source$$
This following is a link to the source code for this example:
$cref/abs_min_linear.hpp/abs_min_linear.hpp/$$.

$head Purpose$$
We are given a point $latex \hat{x} \in \B{R}^n$$ and
use the notation $latex \tilde{f} (x)$$ for the abs-normal
$cref/approximation for f(x)
    /abs_normal_fun
    /Abs-normal Approximation
    /Approximating f(x)
/$$
near $latex \hat{x}$$.
We are also given a vector $latex b \in \B{R}_+^n$$.
This routine solves the problem
$latex \[
\begin{array}{lll}
\R{minimize} & \tilde{f}(x) & \R{w.r.t} \; x \in \B{R}^n
\\
\R{subject \; to} & | x_j - \hat{x}_j | \leq b_j & j = 0 , \ldots , n-1
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
This value is less that or equal 4.
If $icode%level% == 0%$$,
no tracing of the optimization is printed.
If $icode%level% >= 1%$$,
a trace of each iteration of $code abs_min_linear$$ is printed.
If $icode%level% >= 2%$$,
a trace of the $cref lp_box$$ sub-problem is printed.
If $icode%level% >= 3%$$,
a trace of the objective and primal variables $latex x$$ are printed
at each $cref simplex_method$$ iteration.
If $icode%level% == 4%$$,
the simplex tableau is printed at each simplex iteration.

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

$head bound$$
This vector has size $icode n$$
and we denote its value by $latex b \in \B{R}^n$$.
The trust region is defined as the set of $latex x$$ such that
$latex \[
    | x_j - \hat{x}_j | \leq b_j
\]$$
for $latex j = 0 , \ldots , n-1$$,
where $latex x$$ is the point that we are approximating $latex f(x)$$.


$head epsilon$$
The value $icode%epsilon%[0]%$$ is convergence criteria in terms
of the infinity norm of the difference of $icode delta_x$$
between iterations.
The value $icode%epsilon%[1]%$$ is convergence criteria in terms
of the derivative of the objective; i.e., $latex \tilde{f}(x)$$.

$head maxitr$$
This is a vector with size 2.
The value $icode%maxitr%[0]%$$ is the maximum number of
$code abs_min_linear$$ iterations to try before giving up on convergence.
The value $icode%maxitr%[1]%$$ is the maximum number of iterations in
the $cref/simplex_method/simplex_method/maxitr/$$ sub-problems.

$head delta_x$$
This vector $latex \Delta x$$ has size $icode n$$.
The input value of its elements does not matter.
Upon return,
the approximate minimizer of $latex \tilde{f}(x)$$
with respect to the trust region
is $latex x = \hat{x} + \Delta x$$.

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
$latex p_k^{(1)} ( x_k )$$ is the derivative
$latex \tilde{f}^{(1)} ( x_k )$$
corresponding to $latex \sigma ( x_k )$$.

$subhead Iteration$$
At iteration $latex k$$, we solve the problem
$latex \[
\begin{array}{lll}
\R{minimize}
    & \max \{ p_k (x) \W{:} k = 0 , \ldots , K-1 \}
    & \R{w.r.t} \; x
\\
\R{subject \; to} & - b \leq x \leq + b
\end{array}
\] $$
The solution is the new point $latex x_K$$
at which the new affine approximation
$latex p_K (x)$$ is constructed.
This process is iterated until the difference
$latex x_K - x_{K-1}$$ is small enough.


$children%example/abs_normal/abs_min_linear.cpp
    %example/abs_normal/abs_min_linear.omh
%$$
$head Example$$
The file $cref abs_min_linear.cpp$$ contains an example and test of
$code abs_min_linear$$.

$end
-----------------------------------------------------------------------------
*/
# include <cppad/cppad.hpp>
# include "lp_box.hpp"
# include "abs_eval.hpp"

// BEGIN C++
namespace CppAD { // BEGIN_CPPAD_NAMESPACE

// BEGIN PROTOTYPE
template <class DblVector, class SizeVector>
bool abs_min_linear(
    size_t            level   ,
    size_t            n       ,
    size_t            m       ,
    size_t            s       ,
    const DblVector&  g_hat   ,
    const DblVector&  g_jac   ,
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
        "abs_min_linear: level is not less that or equal 4"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(epsilon.size()) == 2,
        "abs_min_linear: size of epsilon not equal to 2"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(maxitr.size()) == 2,
        "abs_min_linear: size of maxitr not equal to 2"
    );
    CPPAD_ASSERT_KNOWN(
        m == 1,
        "abs_min_linear: m is not equal to 1"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(delta_x.size()) == n,
        "abs_min_linear: size of delta_x not equal to n"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(bound.size()) == n,
        "abs_min_linear: size of bound not equal to n"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(g_hat.size()) == m + s,
        "abs_min_linear: size of g_hat not equal to m + s"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(g_jac.size()) == (m + s) * (n + s),
        "abs_min_linear: size of g_jac not equal to (m + s)*(n + s)"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(bound.size()) == n,
        "abs_min_linear: size of bound is not equal to n"
    );
    if( level > 0 )
    {   std::cout << "start abs_min_linear\n";
        CppAD::abs_print_mat("bound", n, 1, bound);
        CppAD::abs_print_mat("g_hat", m + s, 1, g_hat);
        CppAD::abs_print_mat("g_jac", m + s, n + s, g_jac);

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
    // value of approximation for g(x, u) at current delta_x
    DblVector g_tilde = CppAD::abs_eval(n, m, s, g_hat, g_jac, delta_x);
    //
    // value of sigma at delta_x = 0; i.e., sign( z(x, u) )
    CppAD::vector<double> sigma(s);
    for(size_t i = 0; i < s; i++)
        sigma[i] = CppAD::sign( g_tilde[m + i] );
    //
    // current set of cutting planes
    DblVector C(maxitr[0] * n), c(maxitr[0]);
    //
    //
    size_t n_plane = 0;
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
        // check for case where derivative of hyperplane is zero
        // (in convex case, this is the minimizer)
        bool near_zero = true;
        for(size_t j = 0; j < n; j++)
            near_zero &= std::fabs( dy_dx[j] ) < epsilon[1];
        if( near_zero )
        {   if( level > 0 )
                std::cout << "end abs_min_linear: local derivative near zero\n";
            return true;
        }

        // value of hyperplane at delta_x
        double plane_at_zero = g_tilde[0];
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
        // c[i] + C[i,:]*dx <= w
        DblVector b_box(n_plane), A_box(n_plane * (n + 1));
        for(size_t i = 0; i < n_plane; i++)
        {   b_box[i] = c[i];
            for(size_t j = 0; j < n; j++)
                A_box[i * (n+1) + j] = C[i * n + j];
            A_box[i *(n+1) + n] = -1.0;
        }
        // w is the objective
        DblVector c_box(n + 1);
        for(size_t i = 0; i < size_t(c_box.size()); i++)
            c_box[i] = 0.0;
        c_box[n] = 1.0;
        //
        // d_box
        DblVector d_box(n+1);
        for(size_t j = 0; j < n; j++)
            d_box[j] = bound[j];
        d_box[n] = inf;
        //
        // solve the cutting plane problem
        DblVector xout_box(n + 1);
        size_t level_box = 0;
        if( level > 0 )
            level_box = level - 1;
        ok &= CppAD::lp_box(
            level_box,
            A_box,
            b_box,
            c_box,
            d_box,
            maxitr[1],
            xout_box
        );
        if( ! ok )
        {   if( level > 0 )
            {   CppAD::abs_print_mat("delta_x", n, 1, delta_x);
                std::cout << "end abs_min_linear: lp_box failed\n";
            }
            return false;
        }
        //
        // check for convergence
        double max_diff = 0.0;
        for(size_t j = 0; j < n; j++)
        {   double diff = delta_x[j] - xout_box[j];
            max_diff    = std::max( max_diff, std::fabs(diff) );
        }
        //
        // check for descent in value of approximation objective
        DblVector delta_new(n);
        for(size_t j = 0; j < n; j++)
            delta_new[j] = xout_box[j];
        DblVector g_new = CppAD::abs_eval(n, m, s, g_hat, g_jac, delta_new);
        if( level > 0 )
        {   std::cout << "itr = " << itr << ", max_diff = " << max_diff
                << ", y_cur = " << g_tilde[0] << ", y_new = " << g_new[0]
                << "\n";
            CppAD::abs_print_mat("delta_new", n, 1, delta_new);
        }
        //
        g_tilde = g_new;
        delta_x = delta_new;
        //
        // value of sigma at new delta_x; i.e., sign( z(x, u) )
        for(size_t i = 0; i < s; i++)
            sigma[i] = CppAD::sign( g_tilde[m + i] );
        //
        if( max_diff < epsilon[0] )
        {   if( level > 0 )
                std::cout << "end abs_min_linear: change in delta_x near zero\n";
            return true;
        }
    }
    if( level > 0 )
        std::cout << "end abs_min_linear: maximum number of iterations exceeded\n";
    return false;
}
} // END_CPPAD_NAMESPACE
// END C++

# endif
