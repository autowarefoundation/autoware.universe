/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin print_graph.cpp$$
$spell
    Json
$$

$section Print a C++ AD Graph: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool print_graph(void)
{   bool ok = true;
    using std::string;
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : p[1]
    // node_3 : x[0]
    // node_4 : p[0] + p[1]
    // node_5 : x[0] + ( p[0] + p[1] )
    // y[0]   = x[0] + ( p[0] + p[1] )
    //
    // C++ graph object
    CppAD::cpp_graph graph_obj;
    //
    // operator being used
    CppAD::graph::graph_op_enum op_enum;
    //
    // set scalars
    graph_obj.function_name_set("print_graph example");
    size_t n_dynamic_ind = 2;
    graph_obj.n_dynamic_ind_set(n_dynamic_ind);
    size_t n_variable_ind = 1;
    graph_obj.n_variable_ind_set(n_variable_ind);
    //
    // node_4 : p[0] + p[1]
    op_enum = CppAD::graph::add_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(1);
    graph_obj.operator_arg_push_back(2);
    //
    // node_5 : x[0] + ( p[0] + p[1] )
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(3);
    graph_obj.operator_arg_push_back(4);
    //
    // y[0]   = x[0] + ( p[0] + p[1] )
    graph_obj.dependent_vec_push_back(5);
    //
    // get output of print command
    std::stringstream os;
    graph_obj.print(os);
    //
    std::string check =
        "print_graph example\n"
        "          1      p[0]\n"
        "          2      p[1]\n"
        "          3      x[0]\n"
        "          4       add    1    2\n"
        "          5       add    3    4\n"
        "y nodes = 5\n"
    ;
    std::string str = os.str();
    ok &= str == check;
    //
    return ok;
}
// END C++
