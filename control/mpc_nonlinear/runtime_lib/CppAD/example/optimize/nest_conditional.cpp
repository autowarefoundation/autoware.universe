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
$begin optimize_nest_conditional.cpp$$

$section Optimize Nested Conditional Expressions: Example and Test$$

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

        // Create a variable that is is only used in the second comparison
        scalar two = 1. + x[0];
        before.n_var += 1; before.n_op += 1;
        after.n_var  += 1; after.n_op  += 1;

        // Conditional skip for second comparison will be inserted here.
        if( options.find("no_conditional_skip") == std::string::npos )
            after.n_op += 1; // for conditional skip operation

        // Create a variable that is is only used in the first comparison
        // (can be skipped when second comparison result is false)
        scalar one = 1. / x[0];
        before.n_var += 1; before.n_op += 1;
        after.n_var  += 1; after.n_op  += 1;

        // Conditional skip for first comparison will be inserted here.
        if( options.find("no_conditional_skip") == std::string::npos )
            after.n_op += 1; // for conditional skip operation

        // value when first comparison if false
        scalar one_false = 5.0;

        // Create a variable that is only used when second comparison is true
        // (can be skipped when it is false)
        scalar one_true = x[0] / 5.0;
        before.n_var += 1; before.n_op += 1;
        after.n_var  += 1; after.n_op  += 1;

        // value when second comparison is false
        scalar two_false = 3.0;

        // First conditional compaison is 1 / x[0] < x[0]
        // is only used when second conditional expression is true
        // (can be skipped when it is false)
        scalar two_true  = CppAD::CondExpLt(one, x[0], one_true, one_false);
        before.n_var += 1; before.n_op += 1;
        after.n_var  += 1; after.n_op  += 1;

        // Second conditional compaison is 1 + x[0] < x[1]
        scalar two_value = CppAD::CondExpLt(two, x[1], two_true, two_false);
        before.n_var += 1; before.n_op += 1;
        after.n_var  += 1; after.n_op  += 1;

        // results for this operation sequence
        y[0] = two_value;
        before.n_var += 0; before.n_op  += 0;
        after.n_var  += 0; after.n_op   += 0;
    }
}

bool nest_conditional(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps10 = 10.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 2;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 0.5;
    ax[1] = 0.5;

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

        // Check case where result of the second comparison is true
        // and first comparison is true
        CPPAD_TESTVECTOR(double) x(n), y(m), check(m);
        x[0] = 1.75;
        x[1] = 4.0;
        y    = f.Forward(0, x);
        fun(options, x, check, before, after);
        ok &= NearEqual(y[0], check[0], eps10, eps10);
        ok  &= f.number_skip() == 0;

        // Check case where result of the second comparison is true
        // and first comparison is false
        x[0] = 0.5;
        x[1] = 4.0;
        y    = f.Forward(0, x);
        fun(options, x, check, before, after);
        ok &= NearEqual(y[0], check[0], eps10, eps10);
        if( options == "" )
            ok  &= f.number_skip() == 1;
        else
            ok &= f.number_skip() == 0;

        // Check case where result of the second comparison is false
        // and first comparison is true
        x[0] = 1.75;
        x[1] = 0.0;
        y    = f.Forward(0, x);
        fun(options, x, check, before, after);
        ok &= NearEqual(y[0], check[0], eps10, eps10);
        if( options == "" )
            ok  &= f.number_skip() == 3;
        else
            ok &= f.number_skip() == 0;

        // Check case where result of the second comparison is false
        // and first comparison is false
        x[0] = 0.5;
        x[1] = 0.0;
        y    = f.Forward(0, x);
        fun(options, x, check, before, after);
        ok &= NearEqual(y[0], check[0], eps10, eps10);
        if( options == "" )
            ok  &= f.number_skip() == 3;
        else
            ok &= f.number_skip() == 0;
    }
    return ok;
}

// END C++
