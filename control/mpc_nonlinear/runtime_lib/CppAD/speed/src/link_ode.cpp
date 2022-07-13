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
# include <cppad/speed/ode_evaluate.hpp>
# include <cppad/utility/near_equal.hpp>
# include <cppad/utility/near_equal.hpp>
# include <cppad/utility/time_test.hpp>
// BEGIN PROTOTYPE
extern bool link_ode(
    size_t                     size       ,
    size_t                     repeat     ,
    CppAD::vector<double>&     x          ,
    CppAD::vector<double>&     jacobian
);
// END PROTOTYPE
/*
-------------------------------------------------------------------------------
$begin link_ode$$
$spell
    Jacobian
    fp
    bool
    CppAD
$$


$section Speed Testing the Jacobian of Ode Solution$$

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
The same template routine $cref ode_evaluate$$ is used
by th different AD packages.

$head f$$
The function
$latex f : \B{R}^n \rightarrow \B{R}^n$$ that is defined and computed by
evaluating $cref ode_evaluate$$ with a call of the form
$codei%
    ode_evaluate(%x%, %p%, %fp%)
%$$
with $icode p$$ equal to zero.
Calls with the value $icode p$$ equal to one are used to check
the derivative values.

$head Return Value$$
If this speed test is not yet
supported by a particular $icode package$$,
the corresponding return value for $code link_ode$$
should be $code false$$.

$head size$$
The argument $icode size$$
is the number of variables in the ordinary differential equations
which is also equal to $latex n$$.

$head repeat$$
The argument $icode repeat$$ is the number of times the
Jacobian is computed.

$head x$$
The argument $icode x$$ is a vector with $latex n$$ elements.
The input value of the elements of $icode x$$ does not matter.
On output, it has been set to the
argument value for which the function,
or its derivative, is being evaluated.
The value of this vector must change with each repetition.

$head jacobian$$
The argument $icode jacobian$$ is a vector with $latex n^2$$ elements.
The input value of its elements does not matter.
The output value of its elements is the Jacobian of the function $latex f(x)$$
that corresponds to output value of $icode x$$.
To be more specific, for
$latex i = 0 , \ldots , n-1$$ and $latex j = 0 , \ldots , n-1$$,
$latex \[
    \D{f[i]}{x[j]} (x) = jacobian [ i \cdot n + j ]
\] $$

$subhead double$$
In the case where $icode package$$ is $code double$$,
only the first $latex n$$ element of $icode jacobian$$
are modified and they are to the function value
$latex f(x)$$ corresponding to the output value of $icode x$$.

$end
-----------------------------------------------------------------------------
*/
// ---------------------------------------------------------------------------
// The routines below are documented in dev_link.omh
// ---------------------------------------------------------------------------
namespace {
    void time_ode_callback(size_t n, size_t repeat)
    {   // free statically allocated memory
        if( n == 0 && repeat == 0 )
            return;
        //
        CppAD::vector<double> x(n);
        CppAD::vector<double> jacobian(n * n);
        link_ode(n, repeat, x, jacobian);
        return;
    }
}
// ---------------------------------------------------------------------------
bool available_ode(void)
{   size_t n      = 1;
    size_t repeat = 1;
    CppAD::vector<double> x(n);
    CppAD::vector<double> jacobian(n * n);

    return link_ode(n, repeat, x, jacobian);
}
// ----------------------------------------------------------------------------
bool correct_ode(bool is_package_double)
{   bool ok       = true;

    size_t n      = 5;
    size_t repeat = 1;
    CppAD::vector<double> x(n);
    CppAD::vector<double> jacobian(n * n);

    link_ode(n, repeat, x, jacobian);

    size_t size = n * n;
    size_t p = 1;
    if( is_package_double )
    {   p    = 0;  // check function value
        size = n;
    }
    CppAD::vector<double> check(size);
    CppAD::ode_evaluate(x, p, check);
    size_t k;
    for(k = 0; k < size; k++)
        ok &= CppAD::NearEqual(check[k], jacobian[k], 1e-6, 1e-6);

    return ok;
}
double time_ode(double time_min, size_t size)
{   return CppAD::time_test(time_ode_callback, time_min, size);
}
// ----------------------------------------------------------------------------
