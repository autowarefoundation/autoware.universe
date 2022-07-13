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
$begin optimize_conditional_skip.cpp$$

$section Optimize Conditional Expressions: Example and Test$$

$head See Also$$
$cref cond_exp.cpp$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
namespace {
    struct tape_size { size_t n_var; size_t n_op; };

    template <class Vector> void fun(
        const std::string& options ,
        const Vector& x, Vector& y, tape_size& before, tape_size& after
    )
    {   typedef typename Vector::value_type scalar;


        // phantom variable with index 0 and independent variables
        // begin operator, independent variable operators and end operator
        before.n_var = 1 + x.size(); before.n_op  = 2 + x.size();
        after.n_var  = 1 + x.size(); after.n_op   = 2 + x.size();

        // Create a variable that is is only used as left operand
        // in the comparison operation
        scalar left = 1. / x[0];
        before.n_var += 1; before.n_op += 1;
        after.n_var  += 1; after.n_op  += 1;

        // right operand in comparison operation
        scalar right  = x[0];
        before.n_var += 0; before.n_op  += 0;
        after.n_var  += 0; after.n_op   += 0;

        // Note that the left and right operand in the CondExpLt comparison
        // are determined at this point. Hence the conditional skip operator
        // will be inserted here so that the operations mentioned below can
        // also be skipped during zero order foward mode.
        if( options.find("no_conditional_skip") == std::string::npos )
            after.n_op += 1; // for conditional skip operation

        // Create a variable that is only used when comparison result is true
        // (can be skipped when the comparison result is false)
        scalar if_true = x[0] * 5.0;
        before.n_var += 1; before.n_op += 1;
        after.n_var  += 1; after.n_op  += 1;

        // Create two variables only used when the comparison result is false
        // (can be skipped when the comparison result is true)
        scalar temp      = 5.0 + x[0];
        scalar if_false  = temp * 3.0;
        before.n_var += 2; before.n_op += 2;
        after.n_var  += 2; after.n_op  += 2;

        // conditional comparison is 1 / x[0] < x[0]
        scalar value = CppAD::CondExpLt(left, right, if_true, if_false);
        before.n_var += 1; before.n_op += 1;
        after.n_var  += 1; after.n_op  += 1;

        // results for this operation sequence
        y[0] = value;
        before.n_var += 0; before.n_op  += 0;
        after.n_var  += 0; after.n_op   += 0;
    }
}

bool conditional_skip(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps10 = 10.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 1;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.5;

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) ay(m);

    for(size_t k = 0; k < 2; k++)
    {   // optimization options
        std::string options = "";
        if( k == 0 )
            options = "no_conditional_skip";

        // declare independent variables and start tape recording
        CppAD::Independent(ax);

        // compute function computation
        tape_size before, after;
        fun(options, ax, ay, before, after);

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);
        ok &= f.size_order() == 1; // this constructor does 0 order forward
        ok &= f.size_var() == before.n_var;
        ok &= f.size_op()  == before.n_op;

        // Optimize the operation sequence
        f.optimize(options);
        ok &= f.size_order() == 0; // 0 order forward not present
        ok &= f.size_var() == after.n_var;
        ok &= f.size_op()  == after.n_op;

        // Check case where result of the comparison is true (x[0] > 1.0).
        CPPAD_TESTVECTOR(double) x(n), y(m), check(m);
        x[0] = 1.75;
        y    = f.Forward(0, x);
        fun(options, x, check, before, after);
        ok &= NearEqual(y[0], check[0], eps10, eps10);
        if( options == "" )
            ok  &= f.number_skip() == 2;
        else
            ok &= f.number_skip() == 0;

        // Check case where result of the comparison is false (x[0] <= 1.0)
        x[0] = 0.5;
        y    = f.Forward(0, x);
        fun(options, x, check, before, after);
        ok &= NearEqual(y[0], check[0], eps10, eps10);
        if( options == "" )
            ok  &= f.number_skip() == 1;
        else
            ok &= f.number_skip() == 0;
    }
    return ok;
}

// END C++
