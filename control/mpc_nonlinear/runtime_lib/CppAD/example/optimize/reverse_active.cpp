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
$begin optimize_reverse_active.cpp$$

$section Optimize Reverse Activity Analysis: Example and Test$$

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

        // initilized product of even and odd variables
        scalar prod_even = x[0];
        scalar prod_odd  = x[1];
        before.n_var += 0; before.n_op  += 0;
        after.n_var  += 0; after.n_op   += 0;
        //
        // compute product of even and odd variables
        for(size_t i = 2; i < size_t( x.size() ); i++)
        {   if( i % 2 == 0 )
            {   // prod_even will affect dependent variable
                prod_even = prod_even * x[i];
                before.n_var += 1; before.n_op += 1;
                after.n_var  += 1; after.n_op  += 1;
            }
            else
            {   // prod_odd will not affect dependent variable
                prod_odd  = prod_odd * x[i];
                before.n_var += 1; before.n_op += 1;
                after.n_var  += 0; after.n_op  += 0;
            }
        }

        // dependent variable for this operation sequence
        y[0] = prod_even;
        before.n_var += 0; before.n_op  += 0;
        after.n_var  += 0; after.n_op   += 0;
    }
}

bool reverse_active(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps10 = 10.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 6;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    for(size_t i = 0; i < n; i++)
        ax[i] = AD<double>(i + 1);

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

    // check zero order forward with different argument value
    CPPAD_TESTVECTOR(double) x(n), y(m), check(m);
    for(size_t i = 0; i < n; i++)
        x[i] = double(i + 2);
    y    = f.Forward(0, x);
    fun(x, check, before, after);
    ok &= NearEqual(y[0], check[0], eps10, eps10);

    return ok;
}

// END C++
