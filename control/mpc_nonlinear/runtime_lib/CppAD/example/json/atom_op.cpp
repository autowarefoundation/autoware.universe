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
$begin json_atom_op.cpp$$
$spell
    atom
    Json
$$

$section Json Atomic Function Operator: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool atom_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    // use single quote in graphs to avoid having to escape double quote
    //
    // -----------------------------------------------------------------------
    // Define f_0 (x_0, x_1; p) = x_1 + p_0 * x_0
    //
    // This function does not have an atomic function operator
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : x[1]
    // node_4 : p[0] * x[0]
    // node_5 : x[1] + p[0] * x[0]
    // y[0]   = x[1] + p[0] * x[0]
    std::string json =
        "{\n"
        "   'function_name'  : 'f(x; p)',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'mul', 'n_arg':2 } ,\n"
        "       { 'op_code':2, 'name':'add', 'n_arg':2 } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"         // p[0]
        "   'n_variable_ind' : 2,\n"         // x[0], x[1]
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 2, [\n"
        "       [ 1, 1, 2 ] ,\n" // p[0] * x[0]
        "       [ 2, 3, 4 ] ]\n" // x[1] + p[0] * x[0]
        "   ],\n"
        "   'dependent_vec' : [ 1, [5] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 2;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // A ckhpoint_two function with name f(x; p) is derived from
    // an atomic_three fucntion with the same name.
    bool internal_bool    = false;
    bool use_hes_sparsity = false;
    bool use_base2ad      = false;
    bool use_in_parallel  = false;
    CppAD::chkpoint_two<double> chk_f(f, "f(x; p)",
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );
    // -----------------------------------------------------------------------
    // g (u_0, u_1; p, q) = f(u_0 + q_0, u_1 + q_1, p)
    //                    = u_1 + q_1 + p_0 * ( u_0 + q_0 )
    //
    // This function has an atomic function operator with name f(x; p)
    // node_1 : q[0]
    // node_2 : q[1]
    // node_3 : u[0]
    // node_4 : u[1]
    // node_5 : u[0] + q[0]
    // node_6 : u[1] + q[1]
    // node_7 : f( u[0] + q[0], u[1] + q[1]; p)
    // y[0]   = u[1] + q[1] + p[0] * (u[0]  + q[0])
    json =
        "{\n"
        "   'function_name'  : 'g(u; p, q)',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'atom'           } ,\n"
        "       { 'op_code':2, 'name':'add', 'n_arg':2 } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 2,\n"              // q[0], q[1]
        "   'n_variable_ind' : 2,\n"              // u[0], u[1]
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 3, [\n"
        "       [ 2, 3, 1 ]                      ,\n" // u[0] + q[0]
        "       [ 2, 4, 2 ]                      ,\n" // u[1] + q[1]
        // f(u_0 + q_0, u_1 + q_1; p) =  u[1] + q[1] + p[0] * (u[0]  + q[0])
        "       [ 1, 'f(x; p)', 1, 2, [ 5, 6 ] ] ]\n"
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    // ------------------------------------------------------------------------
    CppAD::ADFun<double> g;
    g.from_json(json);
    // ------------------------------------------------------------------------
    ok &= g.Domain() == 2;
    ok &= g.Range() == 1;
    ok &= g.size_dyn_ind() == 2;
    //
    // set p in g(u; p, q)
    vector<double> p(1);
    p[0] = 2.0;
    chk_f.new_dynamic(p);
    //
    // set q in g(u; p, q)
    vector<double> q(2);
    q[0] = 3.0;
    q[1] = 4.0;
    g.new_dynamic(q);
    //
    // evalute g(u; p, q)
    vector<double> u(2), y(1);
    u[0] = 5.0;
    u[1] = 6.0;
    y    = g.Forward(0, u);
    //
    // check value
    ok &= y[0] == u[1] + q[1] + p[0] * (u[0]  + q[0]);
    // ------------------------------------------------------------------------
    json = g.to_json();
    // std::cout << json;
    g.from_json(json);
    // ------------------------------------------------------------------------
    ok &= g.Domain() == 2;
    ok &= g.Range() == 1;
    ok &= g.size_dyn_ind() == 2;
    //
    // set p in g(u; p, q)
    p[0] = 3.0;
    chk_f.new_dynamic(p);
    //
    // set q in g(u; p, q)
    q[0] = 4.0;
    q[1] = 5.0;
    g.new_dynamic(q);
    //
    // evalute g(u; p, q)
    u[0] = 6.0;
    u[1] = 7.0;
    y    = g.Forward(0, u);
    //
    // check value
    ok &= y[0] == u[1] + q[1] + p[0] * (u[0]  + q[0]);
    // ------------------------------------------------------------------------
    return ok;
}
// END C++
