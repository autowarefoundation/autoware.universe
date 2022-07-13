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
$begin atomic_two_eigen_cholesky.cpp$$
$spell
    Eigen
    Cholesky
$$

$section  Atomic Eigen Cholesky Factorization: Example and Test$$

$head Description$$
The $cref ADFun$$ function object $icode f$$ for this example is
$latex \[
f(x)
=
\R{chol} \left( \begin{array}{cc}
    x_0   & x_1 \\
    x_1   & x_2
\end{array} \right)
=
\frac{1}{ \sqrt{x_0} }
\left( \begin{array}{cc}
    x_0 & 0 \\
    x_1 & \sqrt{ x_0 x_2 - x_1 x_1 }
\end{array} \right)
\] $$
where the matrix is positive definite; i.e.,
$latex x_0 > 0$$, $latex x_2 > 0$$ and
$latex x_0 x_2 - x_1 x_1 > 0$$.

$childtable%omh/theory/cholesky.omh
    %include/cppad/example/atomic_two/eigen_cholesky.hpp
%$$

$head Use Atomic Function$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
# include <cppad/example/atomic_two/eigen_cholesky.hpp>


bool eigen_cholesky(void)
{
    typedef double scalar;
    typedef atomic_eigen_cholesky<scalar>::ad_scalar ad_scalar;
    typedef atomic_eigen_cholesky<scalar>::ad_matrix ad_matrix;
    //
    bool ok    = true;
    scalar eps = 10. * std::numeric_limits<scalar>::epsilon();
    using CppAD::NearEqual;
    //
/* %$$
$subhead Constructor$$
$srccode%cpp% */
    // -------------------------------------------------------------------
    // object that computes cholesky factor of a matrix
    atomic_eigen_cholesky<scalar> cholesky;
    // -------------------------------------------------------------------
    // declare independent variable vector x
    size_t n = 3;
    CPPAD_TESTVECTOR(ad_scalar) ad_x(n);
    ad_x[0] = 2.0;
    ad_x[1] = 0.5;
    ad_x[2] = 3.0;
    CppAD::Independent(ad_x);
    // -------------------------------------------------------------------
    // A = [ x[0]  x[1] ]
    //     [ x[1]  x[2] ]
    size_t nr  = 2;
    ad_matrix ad_A(nr, nr);
    ad_A(0, 0) = ad_x[0];
    ad_A(1, 0) = ad_x[1];
    ad_A(0, 1) = ad_x[1];
    ad_A(1, 1) = ad_x[2];
    // -------------------------------------------------------------------
    // use atomic operation to L such that A = L * L^T
    ad_matrix ad_L = cholesky.op(ad_A);
    // -------------------------------------------------------------------
    // declare the dependent variable vector y
    size_t m = 3;
    CPPAD_TESTVECTOR(ad_scalar) ad_y(m);
    ad_y[0] = ad_L(0, 0);
    ad_y[1] = ad_L(1, 0);
    ad_y[2] = ad_L(1, 1);
    CppAD::ADFun<scalar> f(ad_x, ad_y);
    // -------------------------------------------------------------------
    // check zero order forward mode
    CPPAD_TESTVECTOR(scalar) x(n), y(m);
    x[0] = 2.0;
    x[1] = 0.5;
    x[2] = 5.0;
    y   = f.Forward(0, x);
    scalar check;
    check = std::sqrt( x[0] );
    ok   &= NearEqual(y[0], check, eps, eps);
    check = x[1] / std::sqrt( x[0] );
    ok   &= NearEqual(y[1], check, eps, eps);
    check = std::sqrt( x[2] - x[1] * x[1] / x[0] );
    ok   &= NearEqual(y[2], check, eps, eps);
    // -------------------------------------------------------------------
    // check first order forward mode
    CPPAD_TESTVECTOR(scalar) x1(n), y1(m);
    //
    // partial w.r.t. x[0]
    x1[0] = 1.0;
    x1[1] = 0.0;
    x1[2] = 0.0;
    //
    y1    = f.Forward(1, x1);
    check = 1.0 / (2.0 * std::sqrt( x[0] ) );
    ok   &= NearEqual(y1[0], check, eps, eps);
    //
    check = - x[1] / (2.0 * x[0] * std::sqrt( x[0] ) );
    ok   &= NearEqual(y1[1], check, eps, eps);
    //
    check = std::sqrt( x[2] - x[1] * x[1] / x[0] );
    check = x[1] * x[1] / (x[0] * x[0] * 2.0 * check);
    ok   &= NearEqual(y1[2], check, eps, eps);
    //
    // partial w.r.t. x[1]
    x1[0] = 0.0;
    x1[1] = 1.0;
    x1[2] = 0.0;
    //
    y1    = f.Forward(1, x1);
    ok   &= NearEqual(y1[0], 0.0, eps, eps);
    //
    check = 1.0 / std::sqrt( x[0] );
    ok   &= NearEqual(y1[1], check, eps, eps);
    //
    check = std::sqrt( x[2] - x[1] * x[1] / x[0] );
    check = - 2.0 * x[1] / (2.0 * check * x[0] );
    ok   &= NearEqual(y1[2], check, eps, eps);
    //
    // partial w.r.t. x[2]
    x1[0] = 0.0;
    x1[1] = 0.0;
    x1[2] = 1.0;
    //
    y1    = f.Forward(1, x1);
    ok   &= NearEqual(y1[0], 0.0, eps, eps);
    ok   &= NearEqual(y1[1], 0.0, eps, eps);
    //
    check = std::sqrt( x[2] - x[1] * x[1] / x[0] );
    check = 1.0 / (2.0 * check);
    ok   &= NearEqual(y1[2], check, eps, eps);
    // -------------------------------------------------------------------
    // check second order forward mode
    CPPAD_TESTVECTOR(scalar) x2(n), y2(m);
    //
    // second partial w.r.t x[2]
    x2[0] = 0.0;
    x2[1] = 0.0;
    x2[2] = 0.0;
    y2    = f.Forward(2, x2);
    ok   &= NearEqual(y2[0], 0.0, eps, eps);
    ok   &= NearEqual(y2[1], 0.0, eps, eps);
    //
    check = std::sqrt( x[2] - x[1] * x[1] / x[0] );  // funciton value
    check = - 1.0 / ( 4.0 * check * check * check ); // second derivative
    check = 0.5 * check;                             // taylor coefficient
    ok   &= NearEqual(y2[2], check, eps, eps);
    // -------------------------------------------------------------------
    // check first order reverse mode
    CPPAD_TESTVECTOR(scalar) w(m), d1w(n);
    w[0] = 0.0;
    w[1] = 0.0;
    w[2] = 1.0;
    d1w  = f.Reverse(1, w);
    //
    // partial of f[2] w.r.t x[0]
    scalar f2    = std::sqrt( x[2] - x[1] * x[1] / x[0] );
    scalar f2_x0 = x[1] * x[1] / (2.0 * f2 * x[0] * x[0] );
    ok          &= NearEqual(d1w[0], f2_x0, eps, eps);
    //
    // partial of f[2] w.r.t x[1]
    scalar f2_x1 = - x[1] / (f2 * x[0] );
    ok          &= NearEqual(d1w[1], f2_x1, eps, eps);
    //
    // partial of f[2] w.r.t x[2]
    scalar f2_x2 = 1.0 / (2.0 * f2 );
    ok          &= NearEqual(d1w[2], f2_x2, eps, eps);
    // -------------------------------------------------------------------
    // check second order reverse mode
    CPPAD_TESTVECTOR(scalar) d2w(2 * n);
    d2w  = f.Reverse(2, w);
    //
    // check first order results
    ok &= NearEqual(d2w[0 * 2 + 0], f2_x0, eps, eps);
    ok &= NearEqual(d2w[1 * 2 + 0], f2_x1, eps, eps);
    ok &= NearEqual(d2w[2 * 2 + 0], f2_x2, eps, eps);
    //
    // check second order results
    scalar f2_x2_x0 = - 0.5 * f2_x0 / (f2 * f2 );
    ok             &= NearEqual(d2w[0 * 2 + 1], f2_x2_x0, eps, eps);
    scalar f2_x2_x1 = - 0.5 * f2_x1 / (f2 * f2 );
    ok             &= NearEqual(d2w[1 * 2 + 1], f2_x2_x1, eps, eps);
    scalar f2_x2_x2 = - 0.5 * f2_x2 / (f2 * f2 );
    ok             &= NearEqual(d2w[2 * 2 + 1], f2_x2_x2, eps, eps);
    // -------------------------------------------------------------------
    // check third order reverse mode
    CPPAD_TESTVECTOR(scalar) d3w(3 * n);
    d3w  = f.Reverse(3, w);
    //
    // check first order results
    ok &= NearEqual(d3w[0 * 3 + 0], f2_x0, eps, eps);
    ok &= NearEqual(d3w[1 * 3 + 0], f2_x1, eps, eps);
    ok &= NearEqual(d3w[2 * 3 + 0], f2_x2, eps, eps);
    //
    // check second order results
    ok             &= NearEqual(d3w[0 * 3 + 1], f2_x2_x0, eps, eps);
    ok             &= NearEqual(d3w[1 * 3 + 1], f2_x2_x1, eps, eps);
    ok             &= NearEqual(d3w[2 * 3 + 1], f2_x2_x2, eps, eps);
    // -------------------------------------------------------------------
    scalar f2_x2_x2_x0 = - 0.5 * f2_x2_x0 / (f2 * f2);
    f2_x2_x2_x0 += f2_x2 * f2_x0 / (f2 * f2 * f2);
    ok          &= NearEqual(d3w[0 * 3 + 2], 0.5 * f2_x2_x2_x0, eps, eps);
    scalar f2_x2_x2_x1 = - 0.5 * f2_x2_x1 / (f2 * f2);
    f2_x2_x2_x1 += f2_x2 * f2_x1 / (f2 * f2 * f2);
    ok          &= NearEqual(d3w[1 * 3 + 2], 0.5 * f2_x2_x2_x1, eps, eps);
    scalar f2_x2_x2_x2 = - 0.5 * f2_x2_x2 / (f2 * f2);
    f2_x2_x2_x2 += f2_x2 * f2_x2 / (f2 * f2 * f2);
    ok          &= NearEqual(d3w[2 * 3 + 2], 0.5 * f2_x2_x2_x2, eps, eps);
    return ok;
}
/* %$$
$end
*/
