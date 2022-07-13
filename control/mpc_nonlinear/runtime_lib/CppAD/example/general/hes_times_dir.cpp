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
$begin hes_times_dir.cpp$$
$spell
$$

$section Hessian Times Direction: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
// Example and test of computing the Hessian times a direction; i.e.,
// given F : R^n -> R and a direction dx in R^n, we compute F''(x) * dx

# include <cppad/cppad.hpp>

namespace { // put this function in the empty namespace
    // F(x) = |x|^2 = x[0]^2 + ... + x[n-1]^2
    template <class Type>
    Type F(CPPAD_TESTVECTOR(Type) &x)
    {   Type sum = 0;
        size_t i = x.size();
        while(i--)
            sum += x[i] * x[i];
        return sum;
    }
}

bool HesTimesDir(void)
{   bool ok = true;                   // initialize test result
    size_t j;                         // a domain variable variable

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 5;
    CPPAD_TESTVECTOR(AD<double>)  X(n);
    for(j = 0; j < n; j++)
        X[j] = AD<double>(j);

    // declare independent variables and start recording
    CppAD::Independent(X);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y[0] = F(X);

    // create f : X -> Y and stop recording
    CppAD::ADFun<double> f(X, Y);

    // choose a direction dx and compute dy(x) = F'(x) * dx
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    for(j = 0; j < n; j++)
        dx[j] = double(n - j);
    dy = f.Forward(1, dx);

    // compute ddw = F''(x) * dx
    CPPAD_TESTVECTOR(double) w(m);
    CPPAD_TESTVECTOR(double) ddw(2 * n);
    w[0] = 1.;
    ddw  = f.Reverse(2, w);

    // F(x)        = x[0]^2 + x[1]^2 + ... + x[n-1]^2
    // F''(x)      = 2 * Identity_Matrix
    // F''(x) * dx = 2 * dx
    for(j = 0; j < n; j++)
        ok &= NearEqual(ddw[j * 2 + 1], 2.*dx[j], eps99, eps99);

    return ok;
}
// END C++
