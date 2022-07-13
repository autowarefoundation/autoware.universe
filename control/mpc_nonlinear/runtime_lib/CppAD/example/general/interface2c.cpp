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
$begin interface2c.cpp$$
$spell

$$

$section Interfacing to C: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>  // CppAD utilities
# include <cassert>        // assert macro

namespace { // Begin empty namespace
/*
Compute the value of a sum of Gaussians defined by a and evaluated at x
    y = sum_{i=1}^n a[3*i] exp( (x - a[3*i+1])^2 / a[3*i+2])^2 )
where the floating point type is a template parameter
*/
template <class Float>
Float sumGauss(const Float &x, const CppAD::vector<Float> &a)
{
    // number of components in a
    size_t na = a.size();

    // number of Gaussians
    size_t n = na / 3;

    // check the restricitons on na
    assert( na == n * 3 );

    // declare temporaries used inside of loop
    Float ex, arg;

    // initialize sum
    Float y = 0.;

    // loop with respect to Gaussians
    size_t i;
    for(i = 0; i < n; i++)
    {
        arg =   (x - a[3*i+1]) / a[3*i+2];
        ex  =   exp(-arg * arg);
        y  +=   a[3*i] * ex;
    }
    return y;
}
/*
Create a C function interface that computes both
    y = sum_{i=1}^n a[3*i] exp( (x - a[3*i+1])^2 / a[3*i+2])^2 )
and its derivative with respect to the parameter vector a.
*/
extern "C"
void sumGauss(float x, float a[], float *y, float dyda[], size_t na)
{   // Note that any simple vector could replace CppAD::vector;
    // for example, std::vector, std::valarray

    // check the restrictions on na
    assert( na % 3 == 0 );  // mod(na, 3) = 0

    // use the shorthand ADfloat for the type CppAD::AD<float>
    typedef CppAD::AD<float> ADfloat;

    // vector for indpendent variables
    CppAD::vector<ADfloat> A(na);      // used with template function above
    CppAD::vector<float>   acopy(na);  // used for derivative calculations

    // vector for the dependent variables (there is only one)
    CppAD::vector<ADfloat> Y(1);

    // copy the independent variables from C vector to CppAD vectors
    size_t i;
    for(i = 0; i < na; i++)
        A[i] = acopy[i] = a[i];

    // declare that A is the independent variable vector
    CppAD::Independent(A);

    // value of x as an ADfloat object
    ADfloat X = x;

    // Evaluate template version of sumGauss with ADfloat as the template
    // parameter. Set the independent variable to the resulting value
    Y[0] = sumGauss(X, A);

    // create the AD function object F : A -> Y
    CppAD::ADFun<float> F(A, Y);

    // use Value to convert Y[0] to float and return y = F(a)
    *y = CppAD::Value(Y[0]);

    // evaluate the derivative F'(a)
    CppAD::vector<float> J(na);
    J = F.Jacobian(acopy);

    // return the value of dyda = F'(a) as a C vector
    for(i = 0; i < na; i++)
        dyda[i] = J[i];

    return;
}
/*
Link CppAD::NearEqual so do not have to use namespace notation in Interface2C
*/
bool NearEqual(float x, float y, float r, float a)
{   return CppAD::NearEqual(x, y, r, a);
}

} // End empty namespace

bool Interface2C(void)
{   // This routine is intentionally coded as if it were a C routine
    // except for the fact that it uses the predefined type bool.
    bool ok = true;

    // declare variables
    float x, a[6], y, dyda[6], tmp[6];
    size_t na, i;

    // number of parameters (3 for each Gaussian)
    na = 6;

    // number of Gaussians: n  = na / 3;

    // value of x
    x = 1.;

    // value of the parameter vector a
    for(i = 0; i < na; i++)
        a[i] = (float) (i+1);

    // evaulate function and derivative
    sumGauss(x, a, &y, dyda, na);

    // compare dyda to central difference approximation for deriative
    for(i = 0; i < na; i++)
    {   // local variables
        float eps, ai, yp, ym, dy_da;

        // We assume that the type float has at least 7 digits of
        // precision, so we choose eps to be about pow(10., -7./2.).
        eps  = (float) 3e-4;

        // value of this component of a
        ai    = a[i];

        // evaluate F( a + eps * ei )
        a[i]  = ai + eps;
        sumGauss(x, a, &yp, tmp, na);

        // evaluate F( a - eps * ei )
        a[i]  = ai - eps;
        sumGauss(x, a, &ym, tmp, na);

        // evaluate central difference approximates for partial
        dy_da = (yp - ym) / (2 * eps);

        // restore this component of a
        a[i]  = ai;

        ok   &= NearEqual(dyda[i], dy_da, eps, eps);
    }
    return ok;
}
// END C++
