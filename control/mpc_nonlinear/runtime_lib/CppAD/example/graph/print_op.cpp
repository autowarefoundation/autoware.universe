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
$begin graph_print_op.cpp$$
$spell
    Json
$$

$section C++ AD Graph print Operator: Example and Test$$

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
    //
    // C++ graph object
    CppAD::cpp_graph graph_obj;
    //
    // operator being used
    CppAD::graph::graph_op_enum op_enum;
    //
    // set scalars
    graph_obj.function_name_set("print_op example");
    size_t n_dynamic_ind = 1;
    graph_obj.n_dynamic_ind_set(n_dynamic_ind);
    size_t n_variable_ind = 1;
    graph_obj.n_variable_ind_set(n_variable_ind);
    //
    // if p[0] <= 0: print "p[0] = ", p[0], "\n"
    op_enum  = CppAD::graph::print_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    size_t before = graph_obj.print_text_vec_size();
    graph_obj.print_text_vec_push_back("p[0] = ");
    graph_obj.operator_arg_push_back(before);
    size_t after = graph_obj.print_text_vec_size();
    graph_obj.print_text_vec_push_back("\n");
    graph_obj.operator_arg_push_back(after);
    graph_obj.operator_arg_push_back(1);
    graph_obj.operator_arg_push_back(1);
    //
    // if x[0] <= 0: print "x[0] = ", x[0], "\n"
    graph_obj.operator_vec_push_back(op_enum);
    before = graph_obj.print_text_vec_size();
    graph_obj.print_text_vec_push_back("x[0] = ");
    graph_obj.operator_arg_push_back(before);
    graph_obj.operator_arg_push_back(after);
    graph_obj.operator_arg_push_back(2);
    graph_obj.operator_arg_push_back(2);
    //
    // node_3 : log(p[0])
    op_enum = CppAD::graph::log_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(1);
    //
    // node_4 : log(x[0])
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(2);
    //
    // node_5: log(p[0]) + log(x[0])
    op_enum = CppAD::graph::add_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(3);
    graph_obj.operator_arg_push_back(4);
    //
    // y[0] = log(p[0]) + log(x[0])
    graph_obj.dependent_vec_push_back(5);
    //
    // f(x, p) = log(x) + log(p)
    CppAD::ADFun<double> f;
    f.from_graph(graph_obj);
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
    // Convert function to graph and back again
    f.to_graph(graph_obj);
    f.from_graph(graph_obj);
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
