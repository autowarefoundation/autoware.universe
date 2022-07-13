/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/utility/vector.hpp>
# include <cppad/utility/poly.hpp>
# include <cppad/utility/near_equal.hpp>
# include <cppad/utility/time_test.hpp>
// BEGIN PROTOTYPE
extern bool link_poly(
    size_t                     size     ,
    size_t                     repeat   ,
    CppAD::vector<double>&     a        ,
    CppAD::vector<double>&     z        ,
    CppAD::vector<double>&     ddp
);
// END PROTOTYPE
/*
-------------------------------------------------------------------------------
$begin link_poly$$
$spell
    poly
    bool
    CppAD
    ddp
$$


$section Speed Testing Second Derivative of a Polynomial$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%0
%$$

$head Purpose$$
Each $cref/package/speed_main/package/$$
must define a version of this routine as specified below.
This is used by the $cref speed_main$$ program
to run the corresponding speed and correctness tests.

$head Method$$
The same template routine $cref Poly$$ is used
by the different AD packages.

$head Return Value$$
If this speed test is not yet
supported by a particular $icode package$$,
the corresponding return value for $code link_poly$$
should be $code false$$.

$head size$$
The argument $icode size$$ is the order of the polynomial
(the number of coefficients in the polynomial).

$head repeat$$
The argument $icode repeat$$ is the number of different argument values
that the second derivative (or just the polynomial) will be computed at.

$head a$$
The argument $icode a$$ is a vector with $icode%size%$$ elements.
The input value of its elements does not matter.
The output value of its elements is the coefficients of the
polynomial that is differentiated
($th i$$ element is coefficient of order $icode i$$).

$head z$$
The argument $icode z$$ is a vector with one element.
The input value of the element does not matter.
The output of its element is the polynomial argument value
were the last second derivative (or polynomial value) was computed.

$head ddp$$
The argument $icode ddp$$ is a vector with one element.
The input value of its element does not matter.
The output value of its element is the
second derivative of the polynomial with respect to it's argument value.

$subhead double$$
In the case where $icode package$$ is $code double$$,
the output value of the element of $icode ddp$$
is the polynomial value (the second derivative is not computed).

$end
-----------------------------------------------------------------------------
*/
// ---------------------------------------------------------------------------
// The routines below are documented in dev_link.omh
// ---------------------------------------------------------------------------
namespace {
void time_poly_callback(size_t size, size_t repeat)
    {   // free statically allocated memory
        if( size == 0 && repeat == 0 )
            return;
        //
        CppAD::vector<double>  a(size), z(1), ddp(1);

        link_poly(size, repeat, a, z, ddp);
        return;
    }
}
// ---------------------------------------------------------------------------
bool available_poly(void)
{   size_t size   = 10;
    size_t repeat = 1;
    CppAD::vector<double>  a(size), z(1), ddp(1);

    return link_poly(size, repeat, a, z, ddp);
}
// ---------------------------------------------------------------------------
bool correct_poly(bool is_package_double)
{   size_t size   = 10;
    size_t repeat = 1;
    CppAD::vector<double>  a(size), z(1), ddp(1);
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    link_poly(size, repeat, a, z, ddp);

    size_t k;
    double check;
    if( is_package_double )
        k = 0;
    else
        k = 2;
    check = CppAD::Poly(k, a, z[0]);

    bool ok = CppAD::NearEqual(check, ddp[0], eps99, eps99);
    return ok;
}
// ---------------------------------------------------------------------------
double time_poly(double time_min, size_t size)
{   return CppAD::time_test(time_poly_callback, time_min, size);
}
// ---------------------------------------------------------------------------
