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
$begin json_discrete_op.cpp$$
$spell
    add
    Json
$$

$section Json add Operator: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace {
    double heaviside(const double &x)
    {   if( x < 0.0 )
            return 0.0;
        if( x == 0.0 )
            return 0.5;
        return 1.0;
    }
    CPPAD_DISCRETE_FUNCTION(double, heaviside)
}

bool discrete_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    //
    // The discrete function does not exist until its AD version is called
    heaviside( CppAD::AD<double>( 0.0 ) );
    //
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : heaviside(p[0])
    // node_4 : heaviside(p[1])
    // node_5 = heaviside(p[0]) + heaviside(p[1])
    // y[0]   = heaviside(p[0]) + heaviside(p[1])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'discrete_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'add',      'n_arg':2 } ,\n"
        "       { 'op_code':2, 'name':'discrete' }            ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 3, [\n"
        "       [ 2, 'heaviside', 1, 1, [ 1 ] ] ,\n" // heaviside(p[0])
        "       [ 2, 'heaviside', 1, 1, [ 2 ] ] ,\n" // heaviside(x[0])
        "       [ 1, 3, 4 ] ]\n"  // heaviside(p[0]) + heaviside(x[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [5] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = x_0 + ( p_0 + p_1 )
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // set independent variables and parameters
    CPPAD_TESTVECTOR(double) p(1), x(1);
    p[0] = 0.0;
    x[0] = 2.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y = f.Forward(0, x);
    //
    // check result
    ok &= y[0] == heaviside(p[0]) + heaviside(x[0]);
    // -----------------------------------------------------------------------
    // Convert to Json graph and back again
    json = f.to_json();
    // std::cout << "json = " << json;
    f.from_json(json);
    // -----------------------------------------------------------------------
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // set independent variables and parameters
    p[0] = -2.0;
    x[0] = 2.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= y[0] == heaviside(p[0]) + heaviside(x[0]);
    //
    return ok;
}
// END C++
