/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
Old examples now only used for validation testing
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cmath>


namespace { // Begin empty namespace

void myhandler(
    bool known       ,
    int  line        ,
    const char *file ,
    const char *exp  ,
    const char *msg  )
{   // error handler must not return, so throw an exception
    throw msg;
}

bool test_comp_assign(void)
{   bool ok = true;

    // replace the default CppAD error handler
    CppAD::ErrorHandler info(myhandler);


    // create a VecAD vector
    CppAD::VecAD<double> v(1);

    // assign its element a value
    v[0] = 1.0;

# ifndef NDEBUG
    // use try / catch because error haandler throws an exception
    try {
        // set ok to false unless catch block is executed
        ok = false;

        // attempt to use a compound assignment operator
        // with a reference to a VecAD object
        CppAD::AD<double> x = 0.0;
        v[x] += 1.0;
    }
    catch (const char* msg)
    {   std::string check =
            "Can't use VecAD<Base>::reference on left side of +=";
        ok = msg == check;
    }
# endif

    return ok;
}

bool VecADTestOne(void)
{   bool ok = true;
    using namespace CppAD;
    using CppAD::sin;
    using CppAD::cos;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();


    size_t      n = 3;
    AD<double>  N(n);

    AD<double>  a;
    size_t      i;

    // create the array
    CppAD::VecAD<double> V(n);

    // check assignment from double (while not taping)
    for(i = 0; i < n; i++)
        V[i] = double(n - i);

    // check assignment from an AD<double> (while not taping)
    for(i = 0; i < n; i++)
        V[i] = 2. * V[i];

    // check array values (while not taping)
    for(i = 0; i < n; i++)
        ok &= ( V[i] == 2. * double(n - i) );

    // independent variable
    CPPAD_TESTVECTOR(AD<double>) X(1);
    X[0] = double(n - 1);
    Independent(X);

    // check assignment from double during taping
    a = -1.;
    for(i = 0; i < n; i++)
    {   a += 1.;
        V[a] = double(n - i);
    }

    // check assignment from AD<double> during taping
    a = -1.;
    for(i = 0; i < n; i++)
    {   a += 1.;
        V[a] = sin( X[0] ) * V[a];
    }

    // dependent variable
    CPPAD_TESTVECTOR(AD<double>) Z(1);
    Z[0] = V[ X[0] ];

    // create f: X -> Z
    ADFun<double> f(X, Z);
    CPPAD_TESTVECTOR(double)  x( f.Domain() );
    CPPAD_TESTVECTOR(double) dx( f.Domain() );
    CPPAD_TESTVECTOR(double)  z( f.Range() );
    CPPAD_TESTVECTOR(double) dz( f.Range() );

    double vx;
    for(i = 0; i < n; i++)
    {   // check that the indexing operation was taped
        x[0] = double(i);
        z    = f.Forward(0, x);
        vx   = double(n - i);
        ok  &= NearEqual(z[0], sin(x[0]) * vx, eps99, eps99);

        // note that derivative of v[x] w.r.t. x is zero
        dx[0] = 1.;
        dz    = f.Forward(1, dx);
        ok   &= NearEqual(dz[0], cos(x[0]) * vx, eps99, eps99);

        // reverse mode calculation of same value
        dz[0] = 1.;
        dx    = f.Reverse(1, dz);
        ok   &= NearEqual(dx[0], cos(x[0]) * vx, eps99, eps99);
    }


    return ok;
}

// create the discrete function AD<double> Floor(const AD<double> &X)
double Floor(const double &x)
{   return std::floor(x); }
CPPAD_DISCRETE_FUNCTION(double, Floor)

bool VecADTestTwo(void)
{   bool ok = true;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    double pi    = 4. * CppAD::atan(1.);
    size_t nx    = 10;                             // number of x grid point
    double xLow  = 0;                              // minimum value for x
    double xUp   = 2 * pi;                         // maximum value for x
    double xStep = (xUp - xLow) / double(nx - 1);  // step size in x
    double xCur;                                   // current value for x

    // fill in the data vector on a uniform grid
    VecAD<double> Data(nx);
    size_t i;
    for(i = 0; i < nx; i++)
    {   xCur = xLow + double(i) * xStep;
        // use size_t indexing of Data while not taping
        Data[i] = CppAD::sin(xCur);
    }

    // declare independent variable
    CPPAD_TESTVECTOR(AD<double>) X(1);
    X[0] = 2.;
    Independent(X);

    // compute index corresponding to current value of X[0]
    AD<double> I = X[0] / xStep;
    AD<double> Ifloor = Floor(I);

    // make sure Ifloor >= 0  (during taping)
    AD<double> Zero(0);
    Ifloor = CondExpLt(Ifloor, Zero, Zero, Ifloor);

    // make sure Ifloor <= nx - 2 (during taping)
    AD<double> Nxminus2(nx - 2);
    Ifloor = CondExpGt(Ifloor, Nxminus2, Nxminus2, Ifloor);

    // Iceil is Ifloor + 1
    AD<double> Iceil  = Ifloor + 1.;

    // linear interpolate Data
    CPPAD_TESTVECTOR(AD<double>) Y(1);
    Y[0] = Data[Ifloor] + (I - Ifloor) * (Data[Iceil] - Data[Ifloor]);

    // create f: X -> Y that linearly interpolates the data vector
    ADFun<double> f(X, Y);

    // evaluate the linear interpolant at the mid point for first interval
    CPPAD_TESTVECTOR(double)  x(1);
    CPPAD_TESTVECTOR(double)  y(1);
    x[0] = xStep / 2.;
    y    = f.Forward(0, x);
    ok  &= NearEqual(y[0], (Data[0] + Data[1])/2., eps99, eps99);

    // evalute the derivative with respect to x
    CPPAD_TESTVECTOR(double) dx(1);
    CPPAD_TESTVECTOR(double) dy(1);
    dx[0] = 1.;
    dy    = f.Forward(1, dx);
    ok   &= NearEqual(dy[0], (Data[1] - Data[0]) / xStep, eps99, eps99);

    return ok;
}

# include <limits>
bool SecondOrderReverse(void)
{   // Bradley M. Bell 2009-07-06
    // Reverse mode for LdpOp was only modifying the highest order partial
    // This test demonstrated the bug
    bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * std::numeric_limits<double>::epsilon();

    size_t n = 1;
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 2.;
    CppAD::Independent(X);

    size_t m = 2;
    CPPAD_TESTVECTOR(AD<double>) Y(m);

    // The LdpOp instruction corresponds to operations with VecAD vectors.
    CppAD::VecAD<double> Z(2);
    AD<double> zero = 0;
    Z[zero] = X[0] + 1;

    // The LdvOp instruction corresponds to the index being a variable.
    AD<double> one = X[0] - 1; // one in a variable here
    Z[one]  = X[0] + 1.;


    // Compute a function where the second order partial for y
    // depends on the first order partials for z
    // This will use the LdpOp instruction because the index
    // access to z is the parameter zero.
    Y[0] = Z[zero] * Z[zero];
    Y[1] = Z[one]  * Z[one];

    CppAD::ADFun<double> f(X, Y);

    // first order forward
    CPPAD_TESTVECTOR(double) dx(n);
    size_t p = 1;
    dx[0]    = 1.;
    f.Forward(p, dx);

    // Test LdpOp
    // second order reverse (test exp_if_true case)
    CPPAD_TESTVECTOR(double) w(m), dw(2 * n);
    w[0] = 1.;
    w[1] = 0.;
    p    = 2;
    dw = f.Reverse(p, w);

    // check first derivative in dw
    double check = 2. * (Value( X[0] ) + 1.);
    ok &= NearEqual(dw[0], check, eps, eps);

    // check second derivative in dw
    check = 2.;
    ok &= NearEqual(dw[1], check, eps, eps);

    // Test LdvOp
    // second order reverse (test exp_if_true case)
    w[0] = 0.;
    w[1] = 1.;
    p    = 2;
    dw = f.Reverse(p, w);

    // check first derivative in dw
    check = 2. * (Value( X[0] ) + 1.);
    ok &= NearEqual(dw[0], check, eps, eps);

    // check second derivative in dw
    check = 2.;
    ok &= NearEqual(dw[1], check, eps, eps);

    return ok;
}

} // END empty namespace

bool VecAD(void)
{   bool ok = true;
    ok &= test_comp_assign();
    ok &= VecADTestOne();
    ok &= VecADTestTwo();
    ok &= SecondOrderReverse();
    return ok;
}
