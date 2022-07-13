/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
// Check that multilevel reverse with conditial expressions works properly
// when using AD< AD<zdouble> >.

# include <cppad/cppad.hpp>
namespace {
    using CppAD::AD;
    using CppAD::zdouble;
    using CppAD::ADFun;
    using CppAD::vector;
    typedef AD<zdouble> a1type;
    typedef AD<a1type>  a2type;
    typedef vector<a2type> (*a2fun)(const vector<a2type>& a2x);
    //
    zdouble eps    = 10. * std::numeric_limits<double>::epsilon();
    size_t  n_     = 2;
    size_t  m_     = 1;
    //
    void record(a2fun fun, ADFun<zdouble>& g)
    {   vector<zdouble>  x(n_);
        vector<a1type> a1x(n_), a1w(m_), a1z(m_ * n_);
        vector<a2type> a2x(n_), a2y(m_);
        //
        for(size_t j = 0; j < n_; j++)
        {   x[j]   = 0.0;
            a1x[j] = a1type( x[j] );
            a2x[j] = a2type( a1x[j] );
        }
        Independent(a2x);
        // f(x) = x[0] / x[1] if x[1] > 0.0 else 0.0
        a2y = fun(a2x);
        ADFun<a1type> a1f;
        a1f.Dependent(a2x, a2y);
        // use reverse mode to calculate g(x) = f'(x)
        a1w[0] = a1type(1.0);
        Independent(a1x);
        a1f.Forward(0, a1x);
        a1z    = a1f.Reverse(1, a1w);
        g.Dependent(a1x, a1z);
        //
        return;
    }
    // ----------------------------------------------------------------------
    vector<a2type> div(const vector<a2type>& a2x)
    {   vector<a2type> a2y(m_);
        a2type a2zero = a2type(0.0);
        a2type a2four = a2type(4.0);
        a2y[0]  = CondExpGt(a2x[1], a2zero, a2x[0] / a2x[1], a2zero);
        a2y[0] += CondExpGt(a2x[1], a2zero, a2four / a2x[1], a2zero);
        return a2y;
    }
    bool check_div(void)
    {   bool ok = true;
        // record division operations
        ADFun<zdouble> g;
        record(div, g);
        vector<zdouble>  x(n_), z(n_);
        // check result where x[1] <= 0.0 (would be nan without absoute zero)
        x[0] = 0.0;
        x[1] = 0.0;
        z   = g.Forward(0, x);
        z = g.Forward(0, x);
        ok &= z[0] == 0.0;
        ok &= z[1] == 0.0;
        // check result where x[1] > 0.0
        x[0] = 2.0;
        x[1] = 3.0;
        z   = g.Forward(0, x);
        ok &= CppAD::NearEqual(z[0], 1.0/x[1], eps, eps);
        ok &= CppAD::NearEqual(z[1], - (x[0]+4.0)/(x[1]*x[1]), eps, eps);
        //
        return ok;
    }
    // ----------------------------------------------------------------------
    vector<a2type> mul(const vector<a2type>& a2x)
    {   vector<a2type> a2y(m_);
        a2type a2zero = a2type(0.0);
        a2type a2four = a2type(4.0);
        a2y[0]  = CondExpLt(a2x[0], a2four, a2x[0] * a2x[1], a2zero);
        a2y[0] += CondExpLt(a2x[0], a2four, a2four * a2x[1], a2zero);
        a2y[0] += CondExpLt(a2x[0], a2four, a2x[1] * a2four, a2zero);
        return a2y;
    }
    bool check_mul(void)
    {   bool ok = true;
        // record multiplication operations
        ADFun<zdouble> g;
        record(mul, g);
        vector<zdouble>  x(n_), z(n_);
        // check result where x[0] > 4 (would be nan without absoute zero)
        ok &= std::numeric_limits<double>::has_infinity;
        x[0] = std::numeric_limits<double>::infinity();
        x[1] = 0.0;
        z = g.Forward(0, x);
        ok &= z[0] == 0.0;
        ok &= z[1] == 0.0;
        // check result where x[0] < 4
        x[0] = 2.0;
        x[1] = 3.0;
        z   = g.Forward(0, x);
        ok &= CppAD::NearEqual(z[0], x[1], eps, eps);
        ok &= CppAD::NearEqual(z[1], x[0]+8.0, eps, eps);
        //
        return ok;
    }
    // ----------------------------------------------------------------------
    bool check_numeric_limits(void)
    {   bool ok = true;
        //
        double   double_eps  = std::numeric_limits<double>::epsilon();
        zdouble  zdouble_eps = CppAD::numeric_limits<zdouble>::epsilon();
        ok &= double_eps == zdouble_eps;
        //
        double   double_min  = std::numeric_limits<double>::min();
        zdouble  zdouble_min = CppAD::numeric_limits<zdouble>::min();
        ok &= double_min == zdouble_min;
        //
        double   double_max  = std::numeric_limits<double>::max();
        zdouble  zdouble_max = CppAD::numeric_limits<zdouble>::max();
        ok &= double_max == zdouble_max;
        //
        return ok;
    }

}

bool mul_zdouble(void)
{   bool ok = true;

    ok &= check_div();
    ok &= check_mul();
    ok &= check_numeric_limits();

    return ok;
}
