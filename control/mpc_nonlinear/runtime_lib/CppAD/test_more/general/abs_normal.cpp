/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/cppad.hpp>

namespace { // BEGIN_EMPTY_NAMESPACE
// join
CPPAD_TESTVECTOR(double) join(
    const CPPAD_TESTVECTOR(double)& x ,
    const CPPAD_TESTVECTOR(double)& u )
{   size_t n = x.size();
    size_t s = u.size();
    CPPAD_TESTVECTOR(double) xu(n + s);
    for(size_t j = 0; j < n; j++)
        xu[j] = x[j];
    for(size_t j = 0; j < s; j++)
        xu[n + j] = u[j];
    return xu;
}
// test_pow
bool test_pow(void)
{   bool ok = true;
    //
    using CppAD::AD;
    using CppAD::ADFun;
    //
    typedef CPPAD_TESTVECTOR(double)       d_vector;
    typedef CPPAD_TESTVECTOR( AD<double> ) ad_vector;
    //
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    size_t n = 2; // size of x
    size_t m = 1; // size of y
    size_t s = 1; // number of absolute value terms
    //
    // record the function f(x)
    ad_vector ad_x(n), ad_y(m);
    for(size_t j = 0; j < n; j++)
        ad_x[j] = double(j + 1);
    Independent( ad_x );
    //
    // for this example, we the function is
    // f(x) = pow( |x_0|, x_1) + pow( |x_0| , 2) + pow(2, |x_0|)
    AD<double> abs_x0 = abs( ad_x[0] );
    AD<double> pow_vv = pow( abs_x0 , ad_x[1] );
    AD<double> pow_vp = pow( abs_x0 , 2.0 );
    AD<double> pow_pv = pow( 2.0 , abs_x0 );
    ad_y[0] = pow_vv + pow_vp + pow_pv;
    ADFun<double> f(ad_x, ad_y);

    // create its abs_normal representation in g, a
    ADFun<double> g, a;
    f.abs_normal_fun(g, a);

    // check dimension of domain and range space for g
    ok &= g.Domain() == n + s;
    ok &= g.Range()  == m + s;

    // check dimension of domain and range space for a
    ok &= a.Domain() == n;
    ok &= a.Range()  == s;

    // --------------------------------------------------------------------
    // Choose a point x_hat
    d_vector x_hat(n);
    x_hat[0] = -2.0;
    x_hat[1] = 2.0;

    // value of a_hat = a(x_hat)
    d_vector a_hat = a.Forward(0, x_hat);

    // (x_hat, a_hat)
    d_vector xu_hat = join(x_hat, a_hat);

    // value of g[ x_hat, a_hat ]
    d_vector g_hat = g.Forward(0, xu_hat);

    // Jacobian of g[ x_hat, a_hat ]
    d_vector g_jac = g.Jacobian(xu_hat);

    // value of delta_x
    d_vector delta_x(n);
    delta_x[0] =  4.0;
    delta_x[1] =  1.0;

    // value of x
    d_vector x(n);
    for(size_t j = 0; j < n; j++)
        x[j] = x_hat[j] + delta_x[j];

    // value of f(x)
    d_vector y = f.Forward(0, x);

    // check
    double check = std::pow( std::fabs(x[0]) , x[1]);
    check       += std::pow( std::fabs(x[0]) , 2.0 );
    check       += std::pow( 2.0, std::fabs(x[0]) );
    ok          &= CppAD::NearEqual(y[0], check, eps99, eps99);

    return ok;
}
} // END_EMPYT_NAMESPACE

bool abs_normal(void)
{   bool ok = true;
    ok     &= test_pow();
    return ok;
}
