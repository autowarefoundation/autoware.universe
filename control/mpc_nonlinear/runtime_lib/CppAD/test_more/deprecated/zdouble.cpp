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
$begin zdouble.cpp$$
$spell
    zdouble
$$

$section zdouble: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace {
    template <class Base> bool test_one(void)
    {   bool ok = true;
        Base eps99 = 99. * std::numeric_limits<double>::epsilon();

        typedef CppAD::AD<Base>   a1type;
        typedef CppAD::AD<a1type> a2type;

        // value during taping
        size_t n = 2;
        CPPAD_TESTVECTOR(Base) x(n);
        x[0] = 0.0;
        x[1] = 0.0;

        // declare independent variable
        CPPAD_TESTVECTOR(a2type) a2x(n);
        for (size_t j = 0; j < n; j++)
            a2x[j] = a2type( a1type(x[j]) );
        Independent(a2x);

        // zero and one as a2type values
        a2type a2zero = a2type(0.0);
        a2type a2one  = a2type(1.0);

        // h(x) = x[0] / x[1] if x[1] > x[0] else 1.0
        a2type h_x = CondExpGt(a2x[1], a2x[0], a2x[0] / a2x[1], a2one);

        // f(x) = h(x) if x[0] > 0.0 else 0.0
        //      = x[0] / x[1] if x[1] > x[0]  and x[0] > 0.0
        //      = 1.0         if x[0] >= x[1] and x[0] > 0.0
        //      = 0.0         if x[0] <= 0.0
        a2type f_x = CondExpGt(a2x[0], a2zero, h_x, a2one);

        // define the function f(x)
        size_t m = 1;
        CPPAD_TESTVECTOR(a2type) a2y(m);
        a2y[0] = f_x;
        CppAD::ADFun<a1type> af1;
        af1.Dependent(a2x, a2y);

        // Define function g(x) = gradient of f(x)
        CPPAD_TESTVECTOR(a1type) a1x(n), a1z(n), a1w(m);
        for (size_t j = 0; j < n; j++)
            a1x[j] = a1type(x[j]);
        a1w[0] = a1type(1.0);
        Independent(a1x);
        af1.Forward(0, a1x);
        a1z = af1.Reverse(1, a1w);
        CppAD::ADFun<Base> g;
        g.Dependent(a1x, a1z);

        // check result for a case where f(x) = 0.0;
        CPPAD_TESTVECTOR(Base) z(2);
        x[0] = 0.0;
        x[1] = 0.0;
        z    = g.Forward(0, x);
        ok &= z[0] == 0.0;
        ok &= z[1] == 0.0;

        // check result for a case where f(x) = 1.0;
        x[0] = 1.0;
        x[1] = 0.5;
        z    = g.Forward(0, x);
        ok &= z[0] == 0.0;
        ok &= z[1] == 0.0;

        // check result for a case where f(x) = x[0] / x[1];
        x[0] = 1.0;
        x[1] = 2.0;
        z    = g.Forward(0, x);
        ok &= CppAD::NearEqual(z[0], 1.0/x[1], eps99, eps99);
        ok &= CppAD::NearEqual(z[1], - x[0]/(x[1]*x[1]), eps99, eps99);

        return ok;
    }
    bool test_two(void)
    {   bool ok = true;
        using CppAD::zdouble;
        //
        zdouble eps = CppAD::numeric_limits<zdouble>::epsilon();
        ok          &= eps == std::numeric_limits<double>::epsilon();
        //
        zdouble min = CppAD::numeric_limits<zdouble>::min();
        ok          &= min == std::numeric_limits<double>::min();
        //
        zdouble max = CppAD::numeric_limits<zdouble>::max();
        ok          &= max == std::numeric_limits<double>::max();
        //
        zdouble nan = CppAD::numeric_limits<zdouble>::quiet_NaN();
        ok          &= nan != nan;
        //
        int digits10 = CppAD::numeric_limits<zdouble>::digits10;
        ok          &= digits10 == std::numeric_limits<double>::digits10;
        //
        return ok;
    }
}

bool zdouble(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    using CppAD::zdouble;
    //
    ok &= test_one<zdouble>();
    ok &= test_one<double>();
    //
    ok &= test_two();
    //
    return ok;
}
// END C++
