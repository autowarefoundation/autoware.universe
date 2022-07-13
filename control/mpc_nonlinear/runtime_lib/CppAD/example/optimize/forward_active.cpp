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
$begin optimize_forward_active.cpp$$

$section Optimize Forward Activity Analysis: Example and Test$$

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

        // adding the constant zero does not take any operations
        scalar zero   = 0.0 + x[0];
        before.n_var += 0; before.n_op  += 0;
        after.n_var  += 0; after.n_op   += 0;

        // multiplication by the constant one does not take any operations
        scalar one    = 1.0 * x[1];
        before.n_var += 0; before.n_op  += 0;
        after.n_var  += 0; after.n_op   += 0;

        // multiplication by the constant zero does not take any operations
        // and results in the constant zero.
        scalar two    = 0.0 * x[0];

        // operations that only involve constants do not take any operations
        scalar three  = (1.0 + two) * 3.0;
        before.n_var += 0; before.n_op  += 0;
        after.n_var  += 0; after.n_op   += 0;

        // The optimizer will reconize that zero + one = one + zero
        // for all values of x.
        scalar four   = zero + one;
        scalar five   = one  + zero;
        before.n_var += 2; before.n_op  += 2;
        after.n_var  += 1; after.n_op   += 1;

        // The optimizer will reconize that sin(x[3]) = sin(x[3])
        // for all values of x. Note that, for computation of derivatives,
        // sin(x[3]) and cos(x[3]) are stored on the tape as a pair.
        scalar six    = sin(x[2]);
        scalar seven  = sin(x[2]);
        before.n_var += 4; before.n_op  += 2;
        after.n_var  += 2; after.n_op   += 1;

        // If we used addition here, five + seven = zero + one + seven
        // which would get converted to a cumulative summation operator.
        scalar eight = five * seven;
        before.n_var += 1; before.n_op  += 1;
        after.n_var  += 1; after.n_op   += 1;

        // Use two, three, four and six in order to avoid a compiler warning
        // Note that addition of two and three does not take any operations.
        // Also note that optimizer reconizes four * six == five * seven.
        scalar nine  = eight + four * six * (two + three);
        before.n_var += 3; before.n_op  += 3;
        after.n_var  += 2; after.n_op   += 2;

        // results for this operation sequence
        y[0] = nine;
        before.n_var += 0; before.n_op  += 0;
        after.n_var  += 0; after.n_op   += 0;
    }
}

bool forward_active(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps10 = 10.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 3;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.5;
    ax[1] = 1.5;
    ax[2] = 2.0;

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
    // Note that, for this case, all the optimization was done during
    // the recording and there is no benifit to the optimization.
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
