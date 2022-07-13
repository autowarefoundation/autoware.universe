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
$begin graph_comp_op.cpp$$
$spell
    abs
$$

$section C++ AD Graph Comparison Operators: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool comp_op(void)
{   bool ok = true;
    using std::string;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    //        : x[0] < p[0]
    // node_3 : p[0] - x[0]
    // node_4 : log( p[0] - x[0] )
    // y[0]   = log( p[0] - x[0] )
    //
    // C++ graph object
    CppAD::cpp_graph graph_obj;
    //
    // operator being used
    CppAD::graph::graph_op_enum op_enum;
    //
    // set scalars
    graph_obj.function_name_set("comp_op example");
    size_t n_dynamic_ind = 1;
    graph_obj.n_dynamic_ind_set(n_dynamic_ind);
    size_t n_variable_ind = 1;
    graph_obj.n_variable_ind_set(n_variable_ind);
    //
    // x[0] < p[0]
    op_enum = CppAD::graph::comp_lt_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(2);
    graph_obj.operator_arg_push_back(1);
    //
    // node_3 : p[0] - x[0]
    op_enum = CppAD::graph::sub_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(1);
    graph_obj.operator_arg_push_back(2);
    //
    // node_4 : log( p[0] - x[0] )
    op_enum = CppAD::graph::log_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(3);
    //
    // y[0]   = log( p[0] - x[0] )
    graph_obj.dependent_vec_push_back(4);
    //
    // f(x, p) = log( p[0] - x[0] )
    CppAD::ADFun<double> f;
    f.from_graph(graph_obj);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // set independent variables and parameters
    CPPAD_TESTVECTOR(double) p(1), x(1);
    p[0] = 0.3;
    x[0] = 0.2;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y = f.Forward(0, x);
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
    // Convert to Graph graph and back again
    f.to_graph(graph_obj);
    // std::cout << graph;
    f.from_graph(graph_obj);
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
