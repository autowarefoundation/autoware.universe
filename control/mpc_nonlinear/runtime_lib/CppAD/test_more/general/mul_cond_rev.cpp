/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
Test of multi-level conditional expressions reverse mode
*/

# include <cppad/cppad.hpp>

bool mul_cond_rev(void)
{
    bool ok = true;
    using CppAD::vector;
    using CppAD::NearEqual;
    double eps = 10. * std::numeric_limits<double>::epsilon();
    //
    typedef CppAD::AD<double>   a1double;
    typedef CppAD::AD<a1double> a2double;
    //
    a1double a1zero = 0.0;
    a2double a2zero = a1zero;
    a1double a1one  = 1.0;
    a2double a2one  = a1one;
    //
    // --------------------------------------------------------------------
    // create a1f = f(x)
    size_t n = 1;
    size_t m = 25;
    //
    vector<a2double> a2x(n), a2y(m);
    a2x[0] = a2double( 5.0 );
    Independent(a2x);
    //
    size_t i = 0;
    // variable that is greater than one when x[0] is zero
    // and less than one when x[0] is 1.0 or greater
    a2double a2switch  = a2one / (a2x[0] + a2double(0.5));
    // variable that is infinity when x[0] is zero
    // and a normal number when x[0] is 1.0 or greater
    a2double a2inf_var = a2one / a2x[0];
    // variable that is nan when x[0] is zero
    // and a normal number when x[0] is 1.0 or greater
    a2double a2nan_var = ( a2one / a2inf_var ) / a2x[0];
    // variable that is one when x[0] is zero
    // and less then one when x[0] is 1.0 or greater
    a2double a2one_var = a2one / ( a2one + a2x[0] );
    // div
    a2y[i++]  = CondExpGt(a2x[0], a2zero, a2nan_var, a2zero);
    // abs
    a2y[i++]  = CondExpGt(a2x[0], a2zero, fabs( a2y[0] ), a2zero);
    // add
    a2y[i++]  = CondExpGt(a2x[0], a2zero, a2nan_var + a2nan_var, a2zero);
    // acos
    a2y[i++]  = CondExpGt(a2x[0], a2zero, acos(a2switch), a2zero);
    // asin
    a2y[i++]  = CondExpGt(a2x[0], a2zero, asin(a2switch), a2zero);
    // atan
    a2y[i++]  = CondExpGt(a2x[0], a2zero, atan(a2nan_var), a2zero);
    // cos
    a2y[i++]  = CondExpGt(a2x[0], a2zero, cos(a2nan_var), a2zero);
    // cosh
    a2y[i++]  = CondExpGt(a2x[0], a2zero, cosh(a2nan_var), a2zero);
    // exp
    a2y[i++]  = CondExpGt(a2x[0], a2zero, exp(a2nan_var), a2zero);
    // log
    a2y[i++]  = CondExpGt(a2x[0], a2zero, log(a2x[0]), a2zero);
    // mul
    a2y[i++]  = CondExpGt(a2x[0], a2zero, a2x[0] * a2inf_var, a2zero);
    // pow
    a2y[i++]  = CondExpGt(a2x[0], a2zero, pow(a2inf_var, a2x[0]), a2zero);
    // sin
    a2y[i++]  = CondExpGt(a2x[0], a2zero, sin(a2nan_var), a2zero);
    // sinh
    a2y[i++]  = CondExpGt(a2x[0], a2zero, sinh(a2nan_var), a2zero);
    // sqrt
    a2y[i++]  = CondExpGt(a2x[0], a2zero, sqrt(a2x[0]), a2zero);
    // sub
    a2y[i++]  = CondExpGt(a2x[0], a2zero, a2inf_var - a2nan_var, a2zero);
    // tan
    a2y[i++]  = CondExpGt(a2x[0], a2zero, tan(a2nan_var), a2zero);
    // tanh
    a2y[i++]  = CondExpGt(a2x[0], a2zero, tanh(a2nan_var), a2zero);
    // azmul
    a2y[i++]  = CondExpGt(a2x[0], a2zero, azmul(a2x[0], a2inf_var), a2zero);
    //
    // Operations that are C+11 atomic
    //
    // acosh
    a2y[i++]  = CondExpGt(a2x[0], a2zero, acosh( a2x[0] ), a2zero);
    // asinh
    a2y[i++]  = CondExpGt(a2x[0], a2zero, asinh( a2nan_var ), a2zero);
    // atanh
    a2y[i++]  = CondExpGt(a2x[0], a2zero, atanh( a2one_var ), a2zero);
    // erf
    a2y[i++]  = CondExpGt(a2x[0], a2zero, erf( a2nan_var ), a2zero);
    // expm1
    a2y[i++]  = CondExpGt(a2x[0], a2zero, expm1(a2nan_var), a2zero);
    // log1p
    a2y[i++]  = CondExpGt(a2x[0], a2zero, log1p(- a2one_var ), a2zero);
    //
    ok &= i == m;
    CppAD::ADFun<a1double> a1f;
    a1f.Dependent(a2x, a2y);
    // --------------------------------------------------------------------
    // create h = f(x)
    vector<a1double> a1x(n), a1y(m);
    a1x[0] = 5.0;
    //
    Independent(a1x);
    i = 0;
    a1double a1switch  = a1one / (a1x[0] + a1double(0.5));
    a1double a1inf_var = a1one / a1x[0];
    a1double a1nan_var = ( a1one / a1inf_var ) / a1x[0];
    a1double a1one_var = a1one / ( a1one + a1x[0] );
    // div
    a1y[i++]  = CondExpGt(a1x[0], a1zero, a1nan_var, a1zero);
    // abs
    a1y[i++]  = CondExpGt(a1x[0], a1zero, fabs( a1y[0] ), a1zero);
    // add
    a1y[i++]  = CondExpGt(a1x[0], a1zero, a1nan_var + a1nan_var, a1zero);
    // acos
    a1y[i++]  = CondExpGt(a1x[0], a1zero, acos(a1switch), a1zero);
    // asin
    a1y[i++]  = CondExpGt(a1x[0], a1zero, asin(a1switch), a1zero);
    // atan
    a1y[i++]  = CondExpGt(a1x[0], a1zero, atan(a1nan_var), a1zero);
    // cos
    a1y[i++]  = CondExpGt(a1x[0], a1zero, cos(a1nan_var), a1zero);
    // cosh
    a1y[i++]  = CondExpGt(a1x[0], a1zero, cosh(a1nan_var), a1zero);
    // exp
    a1y[i++]  = CondExpGt(a1x[0], a1zero, exp(a1nan_var), a1zero);
    // log
    a1y[i++]  = CondExpGt(a1x[0], a1zero, log(a1x[0]), a1zero);
    // mul
    a1y[i++]  = CondExpGt(a1x[0], a1zero, a1x[0] * a1inf_var, a1zero);
    // pow
    a1y[i++]  = CondExpGt(a1x[0], a1zero, pow(a1inf_var, a1x[0]), a1zero);
    // sin
    a1y[i++]  = CondExpGt(a1x[0], a1zero, sin(a1nan_var), a1zero);
    // sinh
    a1y[i++]  = CondExpGt(a1x[0], a1zero, sinh(a1nan_var), a1zero);
    // sqrt
    a1y[i++]  = CondExpGt(a1x[0], a1zero, sqrt(a1x[0]), a1zero);
    // sub
    a1y[i++]  = CondExpGt(a1x[0], a1zero, a1inf_var - a1nan_var, a1zero);
    // tan
    a1y[i++]  = CondExpGt(a1x[0], a1zero, tan(a1nan_var), a1zero);
    // tanh
    a1y[i++]  = CondExpGt(a1x[0], a1zero, tanh(a1nan_var), a1zero);
    // azmul
    a1y[i++]  = CondExpGt(a1x[0], a1zero, azmul(a1x[0], a1inf_var), a1zero);
    //
    // Operations that are C+11 atomic
    //
    // acosh
    a1y[i++]  = CondExpGt(a1x[0], a1zero, acosh( a1x[0] ), a1zero);
    // asinh
    a1y[i++]  = CondExpGt(a1x[0], a1zero, asinh( a1nan_var ), a1zero);
    // atanh
    a1y[i++]  = CondExpGt(a1x[0], a1zero, atanh( a1one_var ), a1zero);
    // erf
    a1y[i++]  = CondExpGt(a1x[0], a1zero, erf( a1nan_var ), a1zero);
    // expm1
    a1y[i++]  = CondExpGt(a1x[0], a1zero, expm1(a1nan_var), a1zero);
    // log1p
    a1y[i++]  = CondExpGt(a1x[0], a1zero, log1p(- a1one_var ), a1zero);
    //
    ok &= i == m;
    CppAD::ADFun<double> h;
    h.Dependent(a1x, a1y);
    // --------------------------------------------------------------------
    // create g = f'(x)
    vector<a1double> a1dy(m), a1w(m);
    a1x[0] = 2.0;
    for(i = 0; i < m; i++)
        a1w[i] = 0.0;
    //
    Independent(a1x);
    a1f.Forward(0, a1x);
    //
    for(i = 0; i < m; i++)
    {   a1w[i] = 1.0;
        vector<a1double> dyi_dx = a1f.Reverse(1, a1w);
        a1dy[i] = dyi_dx[0];
        a1w[i] = 0.0;
    }
    CppAD::ADFun<double> g; // g uses reverse mode derivatives
    g.Dependent(a1x, a1dy);
    // --------------------------------------------------------------------
    // check case where x[0] > 0
    vector<double> x(1), dx(1), dg(m), dh(m);
    x[0]  = 2.0;
    dx[0] = 1.0;
    h.Forward(0, x);
    dh   = h.Forward(1, dx); // dh uses forward mode derivatives
    dg   = g.Forward(0, x);
    for(i = 0; i < m; i++)
        ok  &= NearEqual(dg[i], dh[i], eps, eps);
    // --------------------------------------------------------------------
    // check case where x[0] = 0
    x[0] = 0.0;
    dg   = g.Forward(0, x);
    h.Forward(0, x);
    dh   = h.Forward(1, dx);
    for(i = 0; i < m; i++)
    {   ok  &= dg[i] == 0.0;
        ok  &= dh[i] == 0.0;
    }
    // --------------------------------------------------------------------
    return ok;
}
