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
$begin optimize_cumulative_sum.cpp$$

$section Optimize Cumulative Sum Operations: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace {
    struct tape_size { size_t n_var; size_t n_op; };

    template <class Vector> void fun(
        const Vector& x, Vector& y, tape_size& before, tape_size& after
    )
    {   typedef typename Vector::value_type scalar;

        // phantom variable with index 0 and independent variables
        // begin operator, independent variable operators and end operator
        before.n_var = 1 + x.size(); before.n_op  = 2 + x.size();
        after.n_var  = 1 + x.size(); after.n_op   = 2 + x.size();

        // operators that are identical, and that will be made part of the
        // cumulative summation. Make sure do not replace second variable
        // using the first and then remove the first as part of the
        // cumulative summation.
        scalar first  = x[0] + x[1];
        scalar second = x[0] + x[1];
        before.n_var += 2; before.n_op  += 2;
        after.n_var  += 0; after.n_op   += 0;

        // test that subtractions are also included in cumulative summations
        scalar third = x[1] - 2.0;
        before.n_var += 1; before.n_op  += 1;
        after.n_var  += 0; after.n_op   += 0;

        // the finial summation is converted to a cumulative summation
        // the other is removed.
        scalar csum = first + second + third;
        before.n_var += 2; before.n_op  += 2;
        after.n_var  += 1; after.n_op   += 1;

        // results for this operation sequence
        y[0] = csum;
        before.n_var += 0; before.n_op  += 0;
        after.n_var  += 0; after.n_op   += 0;
    }
}
bool cumulative_sum(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps10 = 10.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 2;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.5;
    ax[1] = 1.5;

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    tape_size before, after;
    fun(ax, ay, before, after);

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);
    ok &= f.size_order() == 1; // this constructor does 0 order forward
    ok &= f.size_var() == before.n_var;
    ok &= f.size_op()  == before.n_op;

    // Optimize the operation sequence
    f.optimize();
    ok &= f.size_order() == 0; // 0 order forward not present
    ok &= f.size_var() == after.n_var;
    ok &= f.size_op()  == after.n_op;

    // Check result for a zero order calculation for a different x,
    CPPAD_TESTVECTOR(double) x(n), y(m), check(m);
    x[0] = 0.75;
    x[1] = 2.25;
    y    = f.Forward(0, x);
    fun(x, check, before, after);
    ok  &= CppAD::NearEqual(y[0], check[0], eps10, eps10);

    return ok;
}
// END C++
