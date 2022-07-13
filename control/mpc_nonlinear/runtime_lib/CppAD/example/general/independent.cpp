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
WARNING: This file is used an an example by omh/FunConstruct.omh.

$begin independent.cpp$$

$comment ! NOTE the title states that this example is used two places !$$
$section Independent and ADFun Constructor: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace { // --------------------------------------------------------
// define the template function Test<ADVector>(void) in empty namespace
template <class ADVector>
bool Test(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t  n  = 2;
    ADVector X(n);  // ADVector is the template parameter in call to Test
    X[0] = 0.;
    X[1] = 1.;

    // declare independent variables and start recording
    // use the template parameter ADVector for the vector type
    CppAD::Independent(X);

    AD<double> a = X[0] + X[1];      // first AD operation
    AD<double> b = X[0] * X[1];      // second AD operation

    // range space vector
    size_t m = 2;
    ADVector Y(m);  // ADVector is the template paraemter in call to Test
    Y[0] = a;
    Y[1] = b;

    // create f: X -> Y and stop tape recording
    // use the template parameter ADVector for the vector type
    CppAD::ADFun<double> f(X, Y);

    // check value
    ok &= NearEqual(Y[0] , 1.,  eps99 , eps99);
    ok &= NearEqual(Y[1] , 0.,  eps99 , eps99);

    // compute f(1, 2)
    CPPAD_TESTVECTOR(double) x(n);
    CPPAD_TESTVECTOR(double) y(m);
    x[0] = 1.;
    x[1] = 2.;
    y    = f.Forward(0, x);
    ok &= NearEqual(y[0] , 3.,  eps99 , eps99);
    ok &= NearEqual(y[1] , 2.,  eps99 , eps99);

    // compute partial of f w.r.t x[0] at (1, 2)
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 1.;
    dx[1] = 0.;
    dy    = f.Forward(1, dx);
    ok &= NearEqual(dy[0] ,   1.,  eps99 , eps99);
    ok &= NearEqual(dy[1] , x[1],  eps99 , eps99);

    // compute partial of f w.r.t x[1] at (1, 2)
    dx[0] = 0.;
    dx[1] = 1.;
    dy    = f.Forward(1, dx);
    ok &= NearEqual(dy[0] ,   1.,  eps99 , eps99);
    ok &= NearEqual(dy[1] , x[0],  eps99 , eps99);

    return ok;
}
} // End of empty namespace -------------------------------------------

# include <vector>
# include <valarray>
bool Independent(void)
{   bool ok = true;
    typedef CppAD::AD<double> ADdouble;
    // Run with ADVector equal to three different cases
    // all of which are Simple Vectors with elements of type AD<double>.
    ok &= Test< CppAD::vector  <ADdouble> >();
    ok &= Test< std::vector    <ADdouble> >();
    ok &= Test< std::valarray  <ADdouble> >();
    return ok;
}

// END C++
