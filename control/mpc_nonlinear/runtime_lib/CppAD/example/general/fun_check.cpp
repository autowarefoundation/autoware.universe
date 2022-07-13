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
! WARNING: This file is used as an example by FunConstruct and Dependent

$begin fun_check.cpp$$
$spell
    abs
$$

$section ADFun Check and Re-Tape: Example and Test$$




$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace { // -----------------------------------------------------------
// define the template function object Fun<Type,Vector> in empty namespace
template <class Type, class Vector>
class Fun {
private:
    size_t n;
public:
    // function constructor
    Fun(size_t n_) : n(n_)
    { }
    // function evaluator
    Vector operator() (const Vector &x)
    {   Vector y(n);
        size_t i;
        for(i = 0; i < n; i++)
        {   // This operaiton sequence depends on x
            if( x[i] >= 0 )
                y[i] = exp(x[i]);
            else
                y[i] = exp(-x[i]);
        }
        return y;
    }
};
// template function FunCheckCases<Vector, ADVector> in empty namespace
template <class Vector, class ADVector>
bool FunCheckCases(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::ADFun;
    using CppAD::Independent;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // use the ADFun default constructor
    ADFun<double> f;

    // domain space vector
    size_t n = 2;
    ADVector X(n);
    X[0] = -1.;
    X[1] = 1.;

    // declare independent variables and starting recording
    Independent(X);

    // create function object to use with AD<double>
    Fun< AD<double>, ADVector > G(n);

    // range space vector
    size_t m = n;
    ADVector Y(m);
    Y = G(X);

    // stop tape and store operation sequence in f : X -> Y
    f.Dependent(X, Y);
    ok &= (f.size_order() == 0);  // no implicit forward operation

    // create function object to use with double
    Fun<double, Vector> g(n);

    // function values should agree when the independent variable
    // values are the same as during recording
    Vector x(n);
    size_t j;
    for(j = 0; j < n; j++)
        x[j] = Value(X[j]);
    double r = eps99;
    double a = eps99;
    ok      &= FunCheck(f, g, x, a, r);

    // function values should not agree when the independent variable
    // values are the negative of values during recording
    for(j = 0; j < n; j++)
        x[j] = - Value(X[j]);
    ok      &= ! FunCheck(f, g, x, a, r);

    // re-tape to obtain the new AD of double operation sequence
    for(j = 0; j < n; j++)
        X[j] = x[j];
    Independent(X);
    Y = G(X);

    // stop tape and store operation sequence in f : X -> Y
    f.Dependent(X, Y);
    ok &= (f.size_order() == 0);  // no implicit forward with this x

    // function values should agree now
    ok      &= FunCheck(f, g, x, a, r);

    return ok;
}
} // End empty namespace
# include <vector>
# include <valarray>
bool FunCheck(void)
{   bool ok = true;
    typedef CppAD::vector<double>                Vector1;
    typedef CppAD::vector< CppAD::AD<double> > ADVector1;
    typedef   std::vector<double>                Vector2;
    typedef   std::vector< CppAD::AD<double> > ADVector2;
    typedef std::valarray<double>                Vector3;
    typedef std::valarray< CppAD::AD<double> > ADVector3;
    // Run with Vector and ADVector equal to three different cases
    // all of which are Simple Vectors with elements of type
    // double and AD<double> respectively.
    ok &= FunCheckCases< Vector1, ADVector2 >();
    ok &= FunCheckCases< Vector2, ADVector3 >();
    ok &= FunCheckCases< Vector3, ADVector1 >();
    return ok;
}
// END C++
