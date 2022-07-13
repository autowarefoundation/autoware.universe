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
$begin interp_retape.cpp$$
$spell
    Retaping
    retape
$$

$section Interpolation With Retaping: Example and Test$$


$head See Also$$
$cref interp_onetape.cpp$$

$pre

$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
# include <cassert>
# include <cmath>

namespace {
    double ArgumentValue[] = {
        .0 ,
        .2 ,
        .4 ,
        .8 ,
        1.
    };
    double FunctionValue[] = {
        std::sin( ArgumentValue[0] ) ,
        std::sin( ArgumentValue[1] ) ,
        std::sin( ArgumentValue[2] ) ,
        std::sin( ArgumentValue[3] ) ,
        std::sin( ArgumentValue[4] )
    };
    size_t TableLength = 5;

    size_t Index(const CppAD::AD<double> &x)
    {   // determine the index j such that x is between
        // ArgumentValue[j] and ArgumentValue[j+1]
        static size_t j = 0;
        while ( x < ArgumentValue[j] && j > 0 )
            j--;
        while ( x > ArgumentValue[j+1] && j < TableLength - 2)
            j++;
        // assert conditions that must be true given logic above
        assert( j >= 0 && j < TableLength - 1 );
        return j;
    }
    double Argument(const CppAD::AD<double> &x)
    {   size_t j = Index(x);
        return ArgumentValue[j];
    }
    double Function(const CppAD::AD<double> &x)
    {   size_t j = Index(x);
        return FunctionValue[j];
    }
    double Slope(const CppAD::AD<double> &x)
    {   size_t j  = Index(x);
        double dx = ArgumentValue[j+1] - ArgumentValue[j];
        double dy = FunctionValue[j+1] - FunctionValue[j];
        return dy / dx;
    }
}

bool interp_retape(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 1;
    CPPAD_TESTVECTOR(AD<double>) X(n);

    // loop over argument values
    size_t k;
    for(k = 0; k < TableLength - 1; k++)
    {
        X[0] = .4 * ArgumentValue[k] + .6 * ArgumentValue[k+1];

        // declare independent variables and start tape recording
        // (use a different tape for each argument value)
        CppAD::Independent(X);

        // evaluate piecewise linear interpolant at X[0]
        AD<double> A = Argument(X[0]);
        AD<double> F = Function(X[0]);
        AD<double> S = Slope(X[0]);
        AD<double> I = F + (X[0] - A) * S;

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) Y(m);
        Y[0] = I;

        // create f: X -> Y and stop tape recording
        CppAD::ADFun<double> f(X, Y);

        // vectors for arguments to the function object f
        CPPAD_TESTVECTOR(double) x(n);   // argument values
        CPPAD_TESTVECTOR(double) y(m);   // function values
        CPPAD_TESTVECTOR(double) dx(n);  // differentials in x space
        CPPAD_TESTVECTOR(double) dy(m);  // differentials in y space

        // to check function value we use the fact that X[0] is between
        // ArgumentValue[k] and ArgumentValue[k+1]
        double delta, check;
        x[0]   = Value(X[0]);
        delta  = ArgumentValue[k+1] - ArgumentValue[k];
        check  = FunctionValue[k+1] * (x[0]-ArgumentValue[k]) / delta
                   + FunctionValue[k] * (ArgumentValue[k+1]-x[0]) / delta;
        ok    &= NearEqual(Y[0], check, eps99, eps99);

        // evaluate partials w.r.t. x[0]
        dx[0] = 1.;
        dy    = f.Forward(1, dx);

        // check that the derivative is the slope
        check = (FunctionValue[k+1] - FunctionValue[k])
              / (ArgumentValue[k+1] - ArgumentValue[k]);
        ok   &= NearEqual(dy[0], check, eps99, eps99);
    }
    return ok;
}

// END C++
