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
$begin bool_fun.cpp$$
$spell
    bool
    Geq
    Cpp
$$

$section AD Boolean Functions: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <complex>


// define abbreviation for double precision complex
typedef std::complex<double> Complex;

namespace {
    // a unary bool function with Complex argument
    static bool IsReal(const Complex &x)
    {   return x.imag() == 0.; }

    // a binary bool function with Complex arguments
    static bool AbsGeq(const Complex &x, const Complex &y)
    {   double axsq = x.real() * x.real() + x.imag() * x.imag();
        double aysq = y.real() * y.real() + y.imag() * y.imag();

        return axsq >= aysq;
    }

    // Create version of IsReal with AD<Complex> argument
    // inside of namespace and outside of any other function.
    CPPAD_BOOL_UNARY(Complex, IsReal)

    // Create version of AbsGeq with AD<Complex> arguments
    // inside of namespace and outside of any other function.
    CPPAD_BOOL_BINARY(Complex, AbsGeq)

}
bool BoolFun(void)
{   bool ok = true;

    CppAD::AD<Complex> x = Complex(1.,  0.);
    CppAD::AD<Complex> y = Complex(1.,  1.);

    ok &= IsReal(x);
    ok &= ! AbsGeq(x, y);

    return ok;
}

// END C++
