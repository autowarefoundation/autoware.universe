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
$begin code_gen_fun_jac_as_fun.cpp$$
$spell
    jacobian
$$

$section Pass Jacobian as Code Gen Function: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/example/code_gen_fun.hpp>

bool jac_as_fun(void)
{   bool ok = true;
    //
    typedef CppAD::cg::CG<double>     c_double;
    typedef CppAD::AD<c_double>      ac_double;
    //
    typedef CppAD::vector<double>     d_vector;
    typedef CppAD::vector<ac_double> ac_vector;
    //
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 2;
    ac_vector ac_x(n);
    for(size_t j = 0; j < n; ++j)
        ac_x[j] = 1.0 / double(j + 1);

    // declare independent variables and start tape recording
    CppAD::Independent(ac_x);

    // range space vector
    size_t m = 3;
    ac_vector ac_y(m);
    for(size_t i = 0; i < m; ++i)
        ac_y[i] = double(i + 1) * sin( ac_x[i % n] );

    // create f: x -> y and stop tape recording
    CppAD::ADFun<c_double> c_f(ac_x, ac_y);

    // create a version of f that evalutes using ac_double
    CppAD::ADFun<ac_double, c_double> ac_f = c_f.base2ad();

    // Independent varialbes while evaluating Jacobian
    CppAD::Independent(ac_x);

    // Evaluate the Jacobian using any CppAD method
    // (for this example we just use the simpliest thing)
    ac_vector ac_J = ac_f.Jacobian(ac_x);

    // create g: x -> f'(x)
    CppAD::ADFun<c_double> c_g(ac_x, ac_J);

    // create compiled version of c_g
    std::string file_name = "example_lib";
    code_gen_fun g(file_name, c_g);

    // evaluate the compiled jacobian
    d_vector x(n), J(m * n);
    for(size_t j = 0; j < n; ++j)
        x[j] = 1.0 / double(j + 2);
    J = g(x);

    // check Jaociban values
    for(size_t i = 0; i < m; ++i)
    {   for(size_t j = 0; j < n; ++j)
        {   double check = 0.0;
            if( j == i % n )
                check = double(i + 1) * cos( x[i % n] );
            ok &= CppAD::NearEqual(J[i * n + j] , check, eps99, eps99);
        }
    }
    return ok;
}
// END C++
