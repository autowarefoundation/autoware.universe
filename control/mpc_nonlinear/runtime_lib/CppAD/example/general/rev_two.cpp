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
$begin rev_two.cpp$$
$spell
    Cpp
$$

$section Second Partials Reverse Driver: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
namespace { // -----------------------------------------------------
// define the template function in empty namespace
// bool RevTwoCases<BaseVector, SizeVector_t>(void)
template <class BaseVector, class SizeVector_t>
bool RevTwoCases()
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    using CppAD::exp;
    using CppAD::sin;
    using CppAD::cos;

    // domain space vector
    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>)  X(n);
    X[0] = 1.;
    X[1] = 2.;

    // declare independent variables and starting recording
    CppAD::Independent(X);

    // a calculation between the domain and range values
    AD<double> Square = X[0] * X[0];

    // range space vector
    size_t m = 3;
    CPPAD_TESTVECTOR(AD<double>)  Y(m);
    Y[0] = Square * exp( X[1] );
    Y[1] = Square * sin( X[1] );
    Y[2] = Square * cos( X[1] );

    // create f: X -> Y and stop tape recording
    CppAD::ADFun<double> f(X, Y);

    // new value for the independent variable vector
    BaseVector x(n);
    x[0] = 2.;
    x[1] = 1.;

    // set i and j to compute specific second partials of y
    size_t p = 2;
    SizeVector_t i(p);
    SizeVector_t j(p);
    i[0] = 0; j[0] = 0; // for partials y[0] w.r.t x[0] and x[k]
    i[1] = 1; j[1] = 1; // for partials y[1] w.r.t x[1] and x[k]

    // compute the second partials
    BaseVector ddw(n * p);
    ddw = f.RevTwo(x, i, j);

    // partials of y[0] w.r.t x[0] is 2 * x[0] * exp(x[1])
    // check partials of y[0] w.r.t x[0] and x[k] for k = 0, 1
    ok &=  NearEqual(      2.*exp(x[1]), ddw[0*p+0], eps99, eps99);
    ok &=  NearEqual( 2.*x[0]*exp(x[1]), ddw[1*p+0], eps99, eps99);

    // partials of y[1] w.r.t x[1] is x[0] * x[0] * cos(x[1])
    // check partials of F_1 w.r.t x[1] and x[k] for k = 0, 1
    ok &=  NearEqual(    2.*x[0]*cos(x[1]), ddw[0*p+1], eps99, eps99);
    ok &=  NearEqual( -x[0]*x[0]*sin(x[1]), ddw[1*p+1], eps99, eps99);

    return ok;
}
} // End empty namespace
# include <vector>
# include <valarray>
bool RevTwo(void)
{   bool ok = true;
        // Run with BaseVector equal to three different cases
        // all of which are Simple Vectors with elements of type double.
    ok &= RevTwoCases< CppAD::vector <double>, std::vector<size_t> >();
    ok &= RevTwoCases< std::vector   <double>, std::vector<size_t> >();
    ok &= RevTwoCases< std::valarray <double>, std::vector<size_t> >();

        // Run with SizeVector_t equal to two other cases
        // which are Simple Vectors with elements of type size_t.
    ok &= RevTwoCases< std::vector <double>, CppAD::vector<size_t> >();
    ok &= RevTwoCases< std::vector <double>, std::valarray<size_t> >();

    return ok;
}
// END C++
