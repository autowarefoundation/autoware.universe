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
$begin to_json.cpp$$
$spell
    Json
$$

$section Convert an ADFun Object to a Json AD Graph: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool to_json(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    //
    // An AD graph example
    // node_1 : x[0]
    // node_2 : x[1]
    // node_3 : x[0] + x[1]
    // node_4 : (x[0] + x[1]) * x[1]
    // y[0]   = (x[0] + x[1]) * x[1]
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'to_json example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'add', 'n_arg':2 } ,\n"
        "       { 'op_code':2, 'name':'mul', 'n_arg':2 } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 0,\n"
        "   'n_variable_ind' : 2,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 2, [\n"
        "       [ 1, 1, 2 ] ,\n"
        "       [ 2, 3, 2 ] ]\n"
        "   ],\n"
        "   'dependent_vec' : [ 1, [4] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x) = (x_0 + x_1) * x_1
    CppAD::ADFun< AD<double> > af;
    af.from_json(json);
    ok &= af.Domain() == 2;
    ok &= af.Range() == 1;
    //
    // Declare independent variables for a new recording
    vector< AD<double> > ax(2);
    ax[0] = 1.0;
    ax[1] = 2.0;
    CppAD::Independent(ax);
    //
    // Compute f(x)
    af.Forward(0, ax);
    //
    // Compute z = f'(x)
    vector< AD<double> > aw(1), az(2);
    aw[0] = 1.0;
    az    = af.Reverse(1, aw);
    //
    // define g(x) = f'(x)
    CppAD::ADFun<double> g(ax, az);
    // ------------------------------------------------------------------------
    // Convert to Json graph and back
    json = g.to_json();
    // std::cout << json;
    g.from_json(json);
    // ------------------------------------------------------------------------
    //
    // Evaluate function corresponding to g
    vector<double> x(2), z(2);
    x[0] = 3.0;
    x[1] = 4.0;
    z = g.Forward(0, x);
    //
    // should be derivative of f
    ok &= z[0] == x[1];
    ok &= z[1] == x[0] + 2.0 * x[1];
    //
    return ok;
}
// END C++
