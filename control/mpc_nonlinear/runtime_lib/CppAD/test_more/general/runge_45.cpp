/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
Old OdeRunge example now used just for valiadation testing of Runge45
*/

# include <cppad/cppad.hpp>
# include <iostream>
# include <cassert>

namespace { // BEGIN Empty namespace
    class TestFun {
    public:
        TestFun(const CPPAD_TESTVECTOR(CppAD::AD<double>) &w_)
        {   w.resize( w_.size() );
            w = w_;
        }
        void Ode(
            const CppAD::AD<double>                      &t,
            const CPPAD_TESTVECTOR(CppAD::AD<double>) &x,
            CPPAD_TESTVECTOR(CppAD::AD<double>)       &f)
        {
            using CppAD::exp;

            size_t n = x.size();

            size_t i;
            f[0]  = 0.;
            for(i = 1; i < n-1; i++)
                f[i] = w[i] * x[i-1];

            f[n-1] = x[0] * x[1];
        }
    private:
        CPPAD_TESTVECTOR(CppAD::AD<double>) w;
    };
} // END Empty namespace

bool Runge45(void)
{   bool ok = true;

    using namespace CppAD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    size_t i;
    size_t j;
    size_t k;

    size_t n = 6;
    size_t m = n - 1;

    CPPAD_TESTVECTOR(AD<double>) x(n);
    AD<double>                t0    = 0.;
    AD<double>                t1    = 2.;
    size_t                    nstep = 2;

    // vector of independent variables
    CPPAD_TESTVECTOR(AD<double>) w(m);
    for(i = 0; i < m; i++)
        w[i] = double(i);
    Independent(w);

    // construct function object using independent variables
    TestFun fun(w);

    // initial value of x
    CPPAD_TESTVECTOR(AD<double>) x0(n);
    for(i = 0; i < n; i++)
        x0[i] = 0.;
    x0[0] = exp( w[0] );

    // solve the differential equation
    x = Runge45(fun, nstep, t0, t1, x0);

    // create f : w -> x and vectors for evaluating derivatives
    ADFun<double> f(w, x);
    CPPAD_TESTVECTOR(double) q( f.Domain() );
    CPPAD_TESTVECTOR(double) r( f.Range() );

    // for i < n-1,
    // x[i](2) = exp( w[0] ) * (w[1] / 1) * ... * (w[i] / i) * 2^i
    AD<double> xi2 = exp(w[0]);
    for(i = 0; i < n-1; i++)
    {   ok &= NearEqual(x[i],  xi2, eps99, eps99);
        if( i < n-2 )
            xi2 *= w[i+1] * 2. / double(i+1);
    }

    // x[n-1](2) = exp(2 * w[0]) * w[1] * 2^2 / 2
    xi2 = exp(2. * w[0]) * w[1] * 2.;
    ok &= NearEqual(x[n-1], xi2, eps99, eps99);

    // the partial of x[i](2) with respect to w[j] is
    //    x[i](2) / w[j] if 0 < j <= i < n-1
    //    x[i](2)        if j == 0 and i < n-1
    //    2*x[i](2)      if j == 0 and i = n-1
    //    x[i](2) / w[j] if j == 1 and i = n-1
    //    zero           otherwise

    for(i = 0; i < n-1; i++)
    {   // compute partials of x[i]
        for(k = 0; k < n; k++)
            r[k] = 0.;
        r[i] = 1.;
        q    = f.Reverse(1,r);

        for(j = 0; j < m; j++)
        {   // check partial of x[i] w.r.t w[j]
            if (j == 0 )
                ok &= NearEqual(q[j], x[i], eps99, eps99);
            else if( j <= i  )
                ok &= NearEqual(
                    q[j], x[i]/w[j], 1e-14, 1e-14);
            else
                ok &= NearEqual(q[j], 0., eps99, eps99);
        }
    }

    // compute partials of x[n-1]
    i = n-1;
    for(k = 0; k < n; k++)
        r[k] = 0.;
    r[i] = 1.;
    q    = f.Reverse(1,r);

    for(j = 0; j < m; j++)
    {   // check partial of x[n-1] w.r.t w[j]
        if (j == 0 )
            ok &= NearEqual(q[j], 2.*x[i], eps99, eps99);
        else if( j == 1  )
            ok &= NearEqual(
                q[j], x[i]/w[1], 1e-14, 1e-14);
        else
            ok &= NearEqual(q[j], 0., eps99, eps99);
    }

    return ok;
}
