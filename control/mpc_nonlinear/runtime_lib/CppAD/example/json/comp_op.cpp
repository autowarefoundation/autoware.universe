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
$begin json_comp_op.cpp$$
$spell
    abs
    Json
$$

$section Json Comparison Operators: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool comp_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    //        : x[0] < p[0]
    // node_3 : p[0] - x[0]
    // node_4 : log( p[0] - x[0] )
    // y[0]   = log( p[0] - x[0] )
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'comp_op example',\n"
        "   'op_define_vec'  : [ 3, [\n"
        "       { 'op_code':1, 'name':'comp_lt'            } ,\n"
        "       { 'op_code':2, 'name':'sub',     'n_arg':2 } ,\n"
        "       { 'op_code':3, 'name':'log',     'n_arg':1 } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 3, [\n"
        "       [ 1, 0, 2, [2, 1 ] ] ,\n" // x[0] < p[0]
        "       [ 2, 1, 2          ] ,\n" // p[0] - x[0]
        "       [ 3, 3             ] ]\n" // log( p[0] - x[0] )
        "   ],\n"
        "   'dependent_vec' : [ 1, [4] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = log( p[0] - x[0] )
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.3;
    x[0] = 0.2;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    //  x[0] < p[0] so comparison should not have changed
    ok &= f.compare_change_number() == 0;
    //
    // check result
    double check = std::log( p[0] - x[0] );
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // case where comparison is false
    f.check_for_nan(false); // suppress checking for nan for this test
    x[0] = 0.4;
    y = f.Forward(0, x);
    ok &= f.compare_change_number() == 1;
    //
    // -----------------------------------------------------------------------
    // Convert to Json graph and back again
    json = f.to_json();
    // std::cout << json;
    f.from_json(json);
    // -----------------------------------------------------------------------
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    x[0] = 0.2;
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // case where comparison is false
    x[0] = 0.4;
    y = f.Forward(0, x);
    ok &= f.compare_change_number() == 1;
    //
    return ok;
}
// END C++
