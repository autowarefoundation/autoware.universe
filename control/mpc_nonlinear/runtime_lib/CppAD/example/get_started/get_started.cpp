/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin get_started.cpp$$
$spell
    cppad.hpp
    http://www.coin-or.org/CppAD/
    namespace
    iostream
    const
    std
    Jacobian
    jac
    endl
    cout
    cmake
$$

$section Getting Started Using CppAD to Compute Derivatives$$

$head Purpose$$
Demonstrate the use of CppAD by computing the derivative
of a simple example function.

$head Function$$
The example function $latex f : \B{R} \rightarrow \B{R}$$ is defined by
$latex \[
    f(x) = a_0 + a_1 * x^1 + \cdots + a_{k-1} * x^{k-1}
\] $$
where $icode a$$ is a fixed vector of length $icode k$$.

$head Derivative$$
The derivative of $latex f(x)$$ is given by
$latex \[
    f' (x) = a_1 + 2 * a_2 * x +  \cdots + (k-1) * a_{k-1} * x^{k-2}
\] $$

$head Value$$
For the particular case in this example,
$latex k$$ is equal to 5,
$latex a = (1, 1, 1, 1, 1)$$, and
$latex x = 3$$.
If follows that
$latex \[
    f' ( 3 ) = 1 + 2 * 3 + 3 * 3^2 + 4 * 3^3 = 142
\] $$

$head Include File$$
The following command, in the program below, includes the CppAD package:
$codei%
    # include <cppad/cppad.hpp>
%$$

$head Poly$$
The routine $code Poly$$, defined below, evaluates a polynomial.
A general purpose polynomial evaluation routine is documented and
distributed with CppAD; see $cref Poly$$.

$head CppAD Namespace$$
All of the functions and objects defined by CppAD are in the
$code CppAD$$ namespace. In the example below,
$codei%
    using CppAD::AD;
%$$
enables one to abbreviate $code CppAD::AD$$ using just $code AD$$.

$head CppAD Preprocessor Symbols$$
All the $cref preprocessor$$ symbols defined by CppAD begin with
$code CPPAD_$$ (some deprecated symbols begin with $code CppAD_$$).
The preprocessor symbol $cref/CPPAD_TESTVECTOR/testvector/$$
is used in the example below.

$head Program$$
$srccode%cpp% */
# include <iostream>        // standard input/output
# include <vector>          // standard vector
# include <cppad/cppad.hpp> // the CppAD package

namespace { // begin the empty namespace
    // define the function Poly(a, x) = a[0] + a[1]*x[1] + ... + a[k-1]*x[k-1]
    template <class Type>
    Type Poly(const CPPAD_TESTVECTOR(double) &a, const Type &x)
    {   size_t k  = a.size();
        Type y   = 0.;  // initialize summation
        Type x_i = 1.;  // initialize x^i
        for(size_t i = 0; i < k; i++)
        {   y   += a[i] * x_i;  // y   = y + a_i * x^i
            x_i *= x;           // x_i = x_i * x
        }
        return y;
    }
}
// main program
int main(void)
{   using CppAD::AD;   // use AD as abbreviation for CppAD::AD
    using std::vector; // use vector as abbreviation for std::vector

    // vector of polynomial coefficients
    size_t k = 5;                  // number of polynomial coefficients
    CPPAD_TESTVECTOR(double) a(k); // vector of polynomial coefficients
    for(size_t i = 0; i < k; i++)
        a[i] = 1.;                 // value of polynomial coefficients

    // domain space vector
    size_t n = 1;               // number of domain space variables
    vector< AD<double> > ax(n); // vector of domain space variables
    ax[0] = 3.;                 // value at which function is recorded

    // declare independent variables and start recording operation sequence
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;               // number of ranges space variables
    vector< AD<double> > ay(m); // vector of ranges space variables
    ay[0] = Poly(a, ax[0]);     // record operations that compute ay[0]

    // store operation sequence in f: X -> Y and stop recording
    CppAD::ADFun<double> f(ax, ay);

    // compute derivative using operation sequence stored in f
    vector<double> jac(m * n); // Jacobian of f (m by n matrix)
    vector<double> x(n);       // domain space vector
    x[0] = 3.;                 // argument value for computing derivative
    jac  = f.Jacobian(x);      // Jacobian for operation sequence

    // print the results
    std::cout << "f'(3) computed by CppAD = " << jac[0] << std::endl;

    // check if the derivative is correct
    int error_code;
    if( jac[0] == 142. )
        error_code = 0;      // return code for correct case
    else  error_code = 1;    // return code for incorrect case

    return error_code;
}
/* %$$
$head Output$$
Executing the program above will generate the following output:
$codep
    f'(3) computed by CppAD = 142
$$

$head Running$$
After you configure your system using the $cref cmake$$ command,
you compile and run this example by executing the command
$codei%
    make check_example_get_started
%$$
in the build directory; i.e., the directory where the cmake command
was executed.

$head Exercises$$
Modify the program above to accomplish the following tasks
using CppAD:
$list number$$
Compute and print the derivative of $latex f(x) = 1 + x + x^2 + x^3 + x^4$$
at the point $latex x = 2$$.
$lnext
Compute and print the derivative of $latex f(x) = 1 + x + x^2 / 2$$
at the point $latex x = .5$$.
$lnext
Compute and print the derivative of $latex f(x) = \exp (x) - 1 - x - x^2 / 2$$
at the point $latex x = .5$$.
$lend

$end
*/
