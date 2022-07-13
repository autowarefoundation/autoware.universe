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
$begin json_pow_op.cpp$$
$spell
    pow
    Json
$$

$section Json pow Operator: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool pow_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : p[1]
    // node_3 : x[0]
    // node_4 : pow(p[0], p[1])
    // node_5 : pow(p[0], x[0])
    // node_6 = pow(p[0], p[1]) + pow(p[0], x[0])
    // y[0]   = pow(p[0], p[1]) + pow(p[0], x[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'pow_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'add', 'n_arg':2 } ,\n"
        "       { 'op_code':2, 'name':'pow', 'n_arg':2 } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 2,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 3, [\n"
        "       [ 2, 1, 2 ] ,\n" // pow(p[0], p[1])
        "       [ 2, 1, 3 ] ,\n" // pow(p[0], x[0])
        "       [ 1, 4, 5 ] ]\n" // pow(p[0], p[1]) + pow(p[0], x[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [6] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = pow(p_0, p_1) + pow(p_0, x_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    //
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 2;
    //
    // set independent variables and parameters
    vector<double> p(2), x(1);
    p[0] = 2.0;
    p[1] = 3.0;
    x[0] = 4.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check_y0 = std::pow(p[0], p[1]) + std::pow(p[0], x[0]);
    ok &= NearEqual(y[0], check_y0, eps99, eps99);
    // -----------------------------------------------------------------------
    // Convert to Json graph and back again
    json = f.to_json();
    // std::cout << "json = " << json;
    f.from_json(json);
    // -----------------------------------------------------------------------
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 2;
    //
    // set independent variables and parameters
    p[0] = 2.0;
    p[1] = 3.0;
    x[0] = 4.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= NearEqual(y[0], check_y0, eps99, eps99);
    //
    return ok;
}
// END C++
