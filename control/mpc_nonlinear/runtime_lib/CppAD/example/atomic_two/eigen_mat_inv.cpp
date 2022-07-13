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
$begin atomic_two_eigen_mat_inv.cpp$$
$spell
    mul
    Eigen
$$

$section  Atomic Eigen Matrix Inverse: Example and Test$$

$head Description$$
The $cref ADFun$$ function object $icode f$$ for this example is
$latex \[
f(x) =
\left( \begin{array}{cc}
    x_0   & 0 \\
    0   & x_1
\end{array} \right)^{-1}
\left( \begin{array}{c}
    0   \\
    x_2
\end{array} \right)
=
\left( \begin{array}{c}
    0 \\
    x_2 / x_1 )
\end{array} \right)
\] $$

$children%
    include/cppad/example/atomic_two/eigen_mat_inv.hpp
%$$

$head Class Definition$$
This example uses the file $cref atomic_two_eigen_mat_inv.hpp$$
which defines matrix multiply as a $cref atomic_two$$ operation.

$nospell

$head Use Atomic Function$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
# include <cppad/example/atomic_two/eigen_mat_inv.hpp>
# include <cppad/example/atomic_two/eigen_mat_mul.hpp>


bool eigen_mat_inv(void)
{
    typedef double                                   scalar;
    typedef CppAD::AD<scalar>                        ad_scalar;
    typedef atomic_eigen_mat_inv<scalar>::ad_matrix  ad_matrix;
    //
    bool ok    = true;
    scalar eps = 10. * std::numeric_limits<scalar>::epsilon();
    using CppAD::NearEqual;
    //
/* %$$
$subhead Constructor$$
$srccode%cpp% */
    // -------------------------------------------------------------------
    // object that multiplies matrices
    atomic_eigen_mat_mul<scalar> mat_mul;
    // -------------------------------------------------------------------
    // object that computes inverse of a square matrix
    atomic_eigen_mat_inv<scalar> mat_inv;
    // -------------------------------------------------------------------
    // declare independent variable vector x
    size_t n = 3;
    CPPAD_TESTVECTOR(ad_scalar) ad_x(n);
    for(size_t j = 0; j < n; j++)
        ad_x[j] = ad_scalar(j + 1);
    CppAD::Independent(ad_x);
    // -------------------------------------------------------------------
    // left = [ x[0]  0    ]
    //        [ 0     x[1] ]
    size_t nr_left  = 2;
    ad_matrix ad_left(nr_left, nr_left);
    ad_left(0, 0) = ad_x[0];
    ad_left(0, 1) = ad_scalar(0.0);
    ad_left(1, 0) = ad_scalar(0.0);
    ad_left(1, 1) = ad_x[1];
    // -------------------------------------------------------------------
    // right = [ 0 , x[2] ]^T
    size_t nc_right = 1;
    ad_matrix ad_right(nr_left, nc_right);
    ad_right(0, 0) = ad_scalar(0.0);
    ad_right(1, 0) = ad_x[2];
    // -------------------------------------------------------------------
    // use atomic operation to compute left^{-1}
    ad_matrix ad_left_inv = mat_inv.op(ad_left);
    // use atomic operation to multiply left^{-1} * right
    ad_matrix ad_result   = mat_mul.op(ad_left_inv, ad_right);
    // -------------------------------------------------------------------
    // declare the dependent variable vector y
    size_t m = 2;
    CPPAD_TESTVECTOR(ad_scalar) ad_y(2);
    for(size_t i = 0; i < m; i++)
        ad_y[i] = ad_result( long(i), 0);
    CppAD::ADFun<scalar> f(ad_x, ad_y);
    // -------------------------------------------------------------------
    // check zero order forward mode
    CPPAD_TESTVECTOR(scalar) x(n), y(m);
    for(size_t i = 0; i < n; i++)
        x[i] = scalar(i + 2);
    y   = f.Forward(0, x);
    ok &= NearEqual(y[0], 0.0,          eps, eps);
    ok &= NearEqual(y[1], x[2] / x[1],  eps, eps);
    // -------------------------------------------------------------------
    // check first order forward mode
    CPPAD_TESTVECTOR(scalar) x1(n), y1(m);
    x1[0] = 1.0;
    x1[1] = 0.0;
    x1[2] = 0.0;
    y1    = f.Forward(1, x1);
    ok   &= NearEqual(y1[0], 0.0,        eps, eps);
    ok   &= NearEqual(y1[1], 0.0,        eps, eps);
    x1[0] = 0.0;
    x1[1] = 0.0;
    x1[2] = 1.0;
    y1    = f.Forward(1, x1);
    ok   &= NearEqual(y1[0], 0.0,        eps, eps);
    ok   &= NearEqual(y1[1], 1.0 / x[1], eps, eps);
    x1[0] = 0.0;
    x1[1] = 1.0;
    x1[2] = 0.0;
    y1    = f.Forward(1, x1);
    ok   &= NearEqual(y1[0], 0.0,                  eps, eps);
    ok   &= NearEqual(y1[1], - x[2] / (x[1]*x[1]), eps, eps);
    // -------------------------------------------------------------------
    // check second order forward mode
    CPPAD_TESTVECTOR(scalar) x2(n), y2(m);
    x2[0] = 0.0;
    x2[1] = 0.0;
    x2[2] = 0.0;
    scalar  f1_x1_x1 = 2.0 * x[2] / (x[1] * x[1] * x[1] );
    y2    = f.Forward(2, x2);
    ok   &= NearEqual(y2[0], 0.0,            eps, eps);
    ok   &= NearEqual(y2[1], f1_x1_x1 / 2.0, eps, eps);
    // -------------------------------------------------------------------
    // check first order reverse
    CPPAD_TESTVECTOR(scalar) w(m), d1w(n);
    w[0] = 1.0;
    w[1] = 0.0;
    d1w  = f.Reverse(1, w);
    ok  &= NearEqual(d1w[0], 0.0, eps, eps);
    ok  &= NearEqual(d1w[1], 0.0, eps, eps);
    ok  &= NearEqual(d1w[2], 0.0, eps, eps);
    w[0] = 0.0;
    w[1] = 1.0;
    d1w  = f.Reverse(1, w);
    ok  &= NearEqual(d1w[0], 0.0,                  eps, eps);
    ok  &= NearEqual(d1w[1], - x[2] / (x[1]*x[1]), eps, eps);
    ok  &= NearEqual(d1w[2], 1.0 / x[1],           eps, eps);
    // -------------------------------------------------------------------
    // check second order reverse
    CPPAD_TESTVECTOR(scalar) d2w(2 * n);
    d2w  = f.Reverse(2, w);
    // partial f_1 w.r.t x_0
    ok  &= NearEqual(d2w[0 * 2 + 0], 0.0,                  eps, eps);
    // partial f_1 w.r.t x_1
    ok  &= NearEqual(d2w[1 * 2 + 0], - x[2] / (x[1]*x[1]), eps, eps);
    // partial f_1 w.r.t x_2
    ok  &= NearEqual(d2w[2 * 2 + 0], 1.0 / x[1],           eps, eps);
    // partial f_1 w.r.t x_1, x_0
    ok  &= NearEqual(d2w[0 * 2 + 1], 0.0,                  eps, eps);
    // partial f_1 w.r.t x_1, x_1
    ok  &= NearEqual(d2w[1 * 2 + 1], f1_x1_x1,             eps, eps);
    // partial f_1 w.r.t x_1, x_2
    ok  &= NearEqual(d2w[2 * 2 + 1], - 1.0 / (x[1]*x[1]),  eps, eps);
    // -------------------------------------------------------------------
    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
