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
$begin json_sum_op.cpp$$
$spell
    sum
    Json
$$

$section Json sum Operator: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool sum_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : p[1]
    // node_3 : p[2]
    // node_4 : x[0]
    // node_5 : p[0] + p[1] + p[2]
    // node_6 : x[0] + p[0] + p[1] + p[2]
    // y[0]   = x[0] + p[0] + p[1] + p[2]
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'sum_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'add', 'n_arg':2 } ,\n"
        "       { 'op_code':2, 'name':'sum' } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 3,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 2, [\n"
        "       [ 2, 1, 3, [1, 2, 3] ] ,\n" // p[0] + p[1] + p[2]
        "       [ 2, 1, 2, [4, 5 ]   ] ]\n" // x[0] + p[0] + p[1] + p[2]
        "   ],\n"
        "   'dependent_vec' : [ 1, [6] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = x_0 + p_0 + p_1 + p_2
    CppAD::ADFun<double> f;
    f.from_json(json);
    //
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 3;
    //
    // set independent variables and parameters
    vector<double> p(3), x(1);
    for(size_t j = 0; j < 3; ++j)
        p[j] = double(j + 1);
    x[0] = 5.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    ok &= y[0] == x[0] + p[0] + p[1] + p[2];
    // -----------------------------------------------------------------------
    // Convert to Json graph and back again
    json = f.to_json();
    // std::cout << "json = " << json;
    f.from_json(json);
    // -----------------------------------------------------------------------
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 3;
    //
    // set independent variables and parameters
    for(size_t j = 0; j < 3; ++j)
        p[j] = double(j + 1);
    x[0] = 5.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= y[0] == x[0] + p[0] + p[1] + p[2];
    //
    return ok;
}
// END C++
