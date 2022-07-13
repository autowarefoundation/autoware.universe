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
$begin graph_sum_op.cpp$$
$spell
    sum
$$

$section C++ AD Graph sum Operator: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool sum_op(void)
{   bool ok = true;
    using std::string;
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : p[1]
    // node_3 : p[2]
    // node_4 : x[0]
    // node_5 : p[0] + p[1] + p[2]
    // node_6 : x[0] + p[0] + p[1] + p[2]
    // y[0]   = x[0] + p[0] + p[1] + p[2]
    //
    //
    // C++ graph object
    CppAD::cpp_graph graph_obj;
    //
    // operator being used
    CppAD::graph::graph_op_enum op_enum;
    //
    // set scalars
    graph_obj.function_name_set("sum_op example");
    size_t n_dynamic_ind = 3;
    graph_obj.n_dynamic_ind_set(n_dynamic_ind);
    size_t n_variable_ind = 1;
    graph_obj.n_variable_ind_set(n_variable_ind);
    //
    // node_5 : p[0] + p[1] + p[2]
    //
    op_enum = CppAD::graph::sum_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(3);  // n_node_arg
    graph_obj.operator_arg_push_back(1);  // first node arg
    graph_obj.operator_arg_push_back(2);  // second node arg
    graph_obj.operator_arg_push_back(3);  // third node are
    //
    // node_6 : x[0] + p[0] + p[1] + p[2]
    //
    // n_arg comes before first_node
    graph_obj.operator_arg_push_back(2);
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(4);
    graph_obj.operator_arg_push_back(5);
    //
    // y[0]   = x[0] + p[0] + p[1] + p[2]
    graph_obj.dependent_vec_push_back(6);
    //
    // f(x, p) = x_0 + p_0 + p_1 + p_2
    CppAD::ADFun<double> f;
    f.from_graph(graph_obj);
    //
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 3;
    //
    // set independent variables and parameters
    CPPAD_TESTVECTOR(double) p(3), x(1);
    for(size_t j = 0; j < 3; ++j)
        p[j] = double(j + 1);
    x[0] = 5.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y = f.Forward(0, x);
    //
    // check result
    ok &= y[0] == x[0] + p[0] + p[1] + p[2];
    // -----------------------------------------------------------------------
    // Convert to Graph graph and back again
    f.to_graph(graph_obj);
    // std::cout << "json = " << json;
    f.from_graph(graph_obj);
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
