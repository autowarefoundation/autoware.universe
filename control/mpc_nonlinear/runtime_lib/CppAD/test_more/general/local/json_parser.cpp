/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/graph/json_parser.hpp>

bool json_parser(void)
{   bool ok = true;
    using CppAD::cpp_graph;
    using CppAD::vector;
    //
    // An AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : x[1]
    // node_4 : -2.0
    // node_5 : p[0] + x[0] + x[1]
    // node_6 : (p[0] + x[0] + x[1]) * (p[0] + x[0] + x[1])
    // y[0]   = (p[0] + x[0] + x[1]) * (p[0] + x[0] + x[1])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name' : 'json_parser test',\n"
        "   'op_define_vec'  : [ 3, [\n"
        "       { 'op_code':1, 'name':'add', 'n_arg':2 } ,\n"
        "       { 'op_code':2, 'name':'mul', 'n_arg':2 } ,\n"
        "       { 'op_code':3, 'name':'sum'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 2,\n"
        "   'constant_vec'   : [ 1, [ -2.0 ] ],\n"
        "   'op_usage_vec'   : [ 2, [\n"
        "       [ 3, 1, 3, [1, 2, 3 ] ] ,\n"
        "       [ 2, 5, 5             ] ] \n"
        "   ],\n"
        "   'dependent_vec'   : [ 1, [6] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // C++ graph object
    cpp_graph graph_obj;
    //
    const std::string& function_name(    graph_obj.function_name_get() );
    const size_t&      n_dynamic_ind(    graph_obj.n_dynamic_ind_get() );
    const size_t&      n_variable_ind(    graph_obj.n_variable_ind_get() );
    //
    // call parser
    CppAD::local::graph::json_parser( json, graph_obj );
    //
    ok &= function_name == "json_parser test";
    ok &= n_dynamic_ind == 1;
    ok &= n_variable_ind == 2;
    ok &= graph_obj.atomic_name_vec_size() == 0;
    //
    ok &= graph_obj.constant_vec_size() == 1;
    ok &= graph_obj.constant_vec_get(0) == -2.0;
    //
    ok &= graph_obj.operator_vec_size() == 2;
    //
    //
    vector<size_t> arg_node;
    cpp_graph::const_iterator graph_itr             = graph_obj.begin();
    cpp_graph::const_iterator::value_type itr_value = *graph_itr;
    arg_node = *(itr_value.arg_node_ptr);
    ok &= itr_value.op_enum == CppAD::graph::sum_graph_op;
    ok &= arg_node.size() == 3;
    ok &= arg_node[0] == 1;
    ok &= arg_node[1] == 2;
    ok &= arg_node[2] == 3;
    //
    itr_value = *++graph_itr;
    ok &= itr_value.op_enum == CppAD::graph::mul_graph_op;
    arg_node.resize(0); // to avoid CppAD::vector assignment error
    arg_node = *(itr_value.arg_node_ptr);
    ok &= arg_node.size() == 2;
    ok &= arg_node[0] == 5;
    ok &= arg_node[1] == 5;
    //
    ok &= graph_obj.dependent_vec_size() == 1;
    ok &= graph_obj.dependent_vec_get(0) == 6;
    // -----------------------------------------------------------------------
    //
    return ok;
}
