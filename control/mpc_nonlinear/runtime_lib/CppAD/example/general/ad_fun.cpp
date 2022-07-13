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
$begin ad_fun.cpp$$
$spell
$$

$section Creating Your Own Interface to an ADFun Object$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace {

    // This class is an example of a different interface to an AD function object
    template <class Base>
    class my_ad_fun {

    private:
        CppAD::ADFun<Base> f;

    public:
        // default constructor
        my_ad_fun(void)
        { }

        // destructor
        ~ my_ad_fun(void)
        { }

        // Construct an my_ad_fun object with an operation sequence.
        // This is the same as for ADFun<Base> except that no zero
        // order forward sweep is done. Note Hessian and Jacobian do
        // their own zero order forward mode sweep.
        template <class ADvector>
        my_ad_fun(const ADvector& x, const ADvector& y)
        {   f.Dependent(x, y); }

        // same as ADFun<Base>::Jacobian
        template <class BaseVector>
        BaseVector jacobian(const BaseVector& x)
        {   return f.Jacobian(x); }

        // same as ADFun<Base>::Hessian
            template <class BaseVector>
        BaseVector hessian(const BaseVector &x, const BaseVector &w)
        {   return f.Hessian(x, w); }
    };

} // End empty namespace

bool ad_fun(void)
{   // This example is similar to example/jacobian.cpp, except that it
    // uses my_ad_fun instead of ADFun.

    bool ok = true;
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
    my_ad_fun<double> f(X, Y);

    // new value for the independent variable vector
    CPPAD_TESTVECTOR(double) x(n);
    x[0] = 2.;
    x[1] = 1.;

    // compute the derivative at this x
    CPPAD_TESTVECTOR(double) jac( m * n );
    jac = f.jacobian(x);

    /*
    F'(x) = [ 2 * x[0] * exp(x[1]) ,  x[0] * x[0] * exp(x[1]) ]
            [ 2 * x[0] * sin(x[1]) ,  x[0] * x[0] * cos(x[1]) ]
            [ 2 * x[0] * cos(x[1]) , -x[0] * x[0] * sin(x[i]) ]
    */
    ok &=  NearEqual( 2.*x[0]*exp(x[1]), jac[0*n+0], eps99, eps99);
    ok &=  NearEqual( 2.*x[0]*sin(x[1]), jac[1*n+0], eps99, eps99);
    ok &=  NearEqual( 2.*x[0]*cos(x[1]), jac[2*n+0], eps99, eps99);

    ok &=  NearEqual( x[0] * x[0] *exp(x[1]), jac[0*n+1], eps99, eps99);
    ok &=  NearEqual( x[0] * x[0] *cos(x[1]), jac[1*n+1], eps99, eps99);
    ok &=  NearEqual(-x[0] * x[0] *sin(x[1]), jac[2*n+1], eps99, eps99);

    return ok;
}


// END C++
