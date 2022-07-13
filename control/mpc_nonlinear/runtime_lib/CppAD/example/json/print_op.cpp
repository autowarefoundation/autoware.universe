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
$begin json_print_op.cpp$$
$spell
    Json
$$

$section Json AD Graph print Operator: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool print_op(void)
{   bool ok = true;
    using std::string;
    std::stringstream stream_out;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    //        : print(p[0], "p[0] = ", p[0], "\n")
    //        : print(x[0], "x[0] = ", x[0], "\n")
    // node_3 : log(p[0])
    // node_4 : log(x[0])
    // node_5 = log(p[0]) + log(x[0])
    // y[0]   = log(p[0]) + log(x[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'print_op example',\n"
        "   'op_define_vec'  : [ 3, [\n"
        "       { 'op_code':1, 'name':'print'              } ,\n"
        "       { 'op_code':2, 'name':'log',     'n_arg':1 } ,\n"
        "       { 'op_code':3, 'name':'add',     'n_arg':2 } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 5, [\n"
        "       [ 1, 'p[0] = ', '\n', 0, 2, [1, 1 ] ] ,\n" // first print
        "       [ 1, 'x[0] = ', '\n', 0, 2, [2, 2 ] ] ,\n" // second print
        "       [ 2, 1    ] ,\n" // log(p[0])
        "       [ 2, 2    ] ,\n" // log(x[0])
        "       [ 3, 3, 4 ] ]\n" // log(p[0]) + log(x[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [5] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = log(p[0]) + log(x[0])
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // set independent variables and parameters
    CPPAD_TESTVECTOR(double) p(1), x(1);
    p[0] = 1.0;
    x[0] = 2.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y = f.Forward(0, x, stream_out);
    //
    // check result
    ok &= stream_out.str() == "";
    double check = std::log(p[0]) + std::log(x[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // -----------------------------------------------------------------------
    // Convert function to json and back again
    json = f.to_json();
    // std::cout << json;
    f.from_json(json);
    // -----------------------------------------------------------------------
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // set independent variables and parameters
    p[0] = 1.0;
    x[0] = -2.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    f.check_for_nan(false);
    y = f.Forward(0, x, stream_out);
    //
    // check result
    ok &= stream_out.str() == "x[0] = -2\n";
    ok &= std::isnan(y[0]);
    //
    return ok;
}
// END C++
