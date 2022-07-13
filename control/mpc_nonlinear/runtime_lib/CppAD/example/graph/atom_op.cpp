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
$begin graph_atom_op.cpp$$
$spell
    atom
    Json
$$

$section C++ AD Graph Atomic Functions: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool atom_op(void)
{   bool ok = true;
    using std::string;
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
    //
    // C++ graph object
    CppAD::cpp_graph graph_obj;
    //
    // operator being used
    CppAD::graph::graph_op_enum op_enum;
    //
    // set scalars
    graph_obj.function_name_set("f(x; p)");
    size_t n_dynamic_ind = 1;
    graph_obj.n_dynamic_ind_set(n_dynamic_ind);
    size_t n_variable_ind = 2;
    graph_obj.n_variable_ind_set(n_variable_ind);
    //
    // node_4 : p[0] * x[0]
    op_enum = CppAD::graph::mul_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(1);
    graph_obj.operator_arg_push_back(2);
    //
    // node_5 : x[1] + p[0] * x[0]
    op_enum = CppAD::graph::add_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(3);
    graph_obj.operator_arg_push_back(4);
    //
    // y[0]   = x[1] + p[0] * x[0]
    graph_obj.dependent_vec_push_back(5);
    //
    // f(x, p) = x_1 + p_0 * x_0
    CppAD::ADFun<float> f;
    f.from_graph(graph_obj);
    //
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
    CppAD::chkpoint_two<float> chk_f(f, "f(x; p)",
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
    //
    graph_obj.initialize();
    //
    graph_obj.function_name_set("g(u; p, q)");
    n_dynamic_ind = 2;
    graph_obj.n_dynamic_ind_set(n_dynamic_ind);
    n_variable_ind = 2;
    graph_obj.n_variable_ind_set(n_variable_ind);
    //
    // node_5 : u[0] + q[0]
    op_enum = CppAD::graph::add_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(3);
    graph_obj.operator_arg_push_back(1);
    //
    // node_6 : u[1] + q[1]
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(4);
    graph_obj.operator_arg_push_back(2);
    //
    // node_7 : f( u[0] + q[0], u[1] + q[1]; p)
    //
    // name_index, n_result, n_arg come before first_node
    size_t name_index = graph_obj.atomic_name_vec_size();
    graph_obj.atomic_name_vec_push_back("f(x; p)");
    //
    op_enum = CppAD::graph::atom_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(name_index);  // name_index
    graph_obj.operator_arg_push_back(1);           // n_result
    graph_obj.operator_arg_push_back(2);           // n_node_arg
    graph_obj.operator_arg_push_back(5);           // first node arg
    graph_obj.operator_arg_push_back(6);           // second node arg
    //
    // y[0]   = u[1] + q[1] + p[0] * (u[0]  + q[0])
    graph_obj.dependent_vec_push_back(7);
    // ------------------------------------------------------------------------
    CppAD::ADFun<float> g;
    g.from_graph(graph_obj);
    // ------------------------------------------------------------------------
    ok &= g.Domain() == 2;
    ok &= g.Range() == 1;
    ok &= g.size_dyn_ind() == 2;
    //
    // set p in g(u; p, q)
    CPPAD_TESTVECTOR(float) p(1);
    p[0] = 2.0;
    chk_f.new_dynamic(p);
    //
    // set q in g(u; p, q)
    CPPAD_TESTVECTOR(float) q(2);
    q[0] = 3.0;
    q[1] = 4.0;
    g.new_dynamic(q);
    //
    // evalute g(u; p, q)
    CPPAD_TESTVECTOR(float) u(2), y(1);
    u[0] = 5.0;
    u[1] = 6.0;
    y    = g.Forward(0, u);
    //
    // check value
    ok &= y[0] == u[1] + q[1] + p[0] * (u[0]  + q[0]);
    // ------------------------------------------------------------------------
    g.to_graph(graph_obj);
    g.from_graph(graph_obj);
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
