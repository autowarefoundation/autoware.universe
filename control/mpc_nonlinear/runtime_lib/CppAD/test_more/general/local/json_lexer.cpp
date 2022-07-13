/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/cppad.hpp>

bool json_lexer(void)
{   bool ok = true;
    typedef CppAD::graph::graph_op_enum graph_op_enum;
    using CppAD::local::graph::op_name2enum;
    //
    // match_any_string
    std::string match_any_string = "";
    //
    // An AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : x[1]
    // node_4 : -2.0
    // node_5 : p[0] + x[0] + x[1]
    // node_6 : (p[0] + x[0] + x[1]) * (p[0] + x[0] + x[1])
    // y[0] = (p[0] + x[0] + x[1]) * (p[0] + x[0] + x[1])
    // use single quote to avoid having to escape double quote
    std::string graph =
        "{\n"
        "   'op_define_vec'  : [ 3, [\n"
        "       { 'op_code':1, 'name':'add', 'n_arg':2 } ,\n"
        "       { 'op_code':2, 'name':'mul', 'n_arg':2 } ,\n"
        "       { 'op_code':3, 'name':'sum'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 2,\n"
        "   'constant_vec'   : [ 1, [ -2.0 ] ],\n"
        "   'op_usage_vec'   : [ 2, [\n"
        "       [ 3, 1, 3, [1, 2, 3] ] ,\n"
        "       [ 2, 5, 5 ] ] \n"
        "   ],\n"
        "   'dependent_vec'   : [ 1, [6] ]\n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < graph.size(); ++i)
        if( graph[i] == '\'' ) graph[i] = '"';
    //
    // json_lexer constructor checks for { at beginning
    CppAD::local::graph::json_lexer json_lexer(graph);
    // -----------------------------------------------------------------------
    // op_define_vec
    json_lexer.check_next_string("op_define_vec");
    json_lexer.check_next_char(':');
    json_lexer.check_next_char('[');
    //
    // n_define
    json_lexer.next_non_neg_int();
    size_t n_define = json_lexer.token2size_t();
    json_lexer.check_next_char(',');
    json_lexer.check_next_char('[');
    CppAD::vector<graph_op_enum> op_code2enum(1);
    for(size_t i = 0; i < n_define; ++i)
    {   json_lexer.check_next_char('{');
        //
        // op_code
        json_lexer.check_next_string("op_code");
        json_lexer.check_next_char(':');
        json_lexer.next_non_neg_int();
# ifndef NDEBUG
        size_t op_code = json_lexer.token2size_t();
        assert( op_code == op_code2enum.size() );
# endif
        json_lexer.check_next_char(',');
        //
        // name
        json_lexer.check_next_string("name");
        json_lexer.check_next_char(':');
        json_lexer.check_next_string(match_any_string);
        std::string   name   = json_lexer.token();
        graph_op_enum op_enum = op_name2enum[name];
        //
        // op_code2enum
        op_code2enum.push_back(op_enum);
        //
        if( op_enum != CppAD::graph::sum_graph_op )
        {   json_lexer.check_next_char(',');
            //
            // n_arg
            json_lexer.check_next_string("n_arg");
            json_lexer.check_next_char(':');
            json_lexer.next_non_neg_int();
            ok &= json_lexer.token2size_t() == 2;
        }
        //
        json_lexer.check_next_char('}');
        if( i + 1 == n_define )
            json_lexer.check_next_char(']');
        else
            json_lexer.check_next_char(',');
    }
    json_lexer.check_next_char(']');
    json_lexer.check_next_char(',');
    // -----------------------------------------------------------------------
    // n_dynamic_ind
    json_lexer.check_next_string("n_dynamic_ind");
    json_lexer.check_next_char(':');
    json_lexer.next_non_neg_int();
    size_t n_dynamic_ind = json_lexer.token2size_t();
    json_lexer.check_next_char(',');
    //
    ok &= n_dynamic_ind == 1;
    // -----------------------------------------------------------------------
    // n_variable_ind
    json_lexer.check_next_string("n_variable_ind");
    json_lexer.check_next_char(':');
    json_lexer.next_non_neg_int();
    size_t n_variable_ind = json_lexer.token2size_t();
    json_lexer.check_next_char(',');
    //
    ok &= n_variable_ind == 2;
    // -----------------------------------------------------------------------
    // constant_vec
    json_lexer.check_next_string("constant_vec");
    json_lexer.check_next_char(':');
    json_lexer.check_next_char('[');
    json_lexer.next_non_neg_int();
    size_t n_constant = json_lexer.token2size_t();
    CppAD::vector<double> constant_vec(n_constant);
    json_lexer.check_next_char(',');
    //
    // [ first_constant, ... , last_constant ]
    json_lexer.check_next_char('[');
    for(size_t i = 0; i < n_constant; ++i)
    {   json_lexer.next_float();
        constant_vec[i] = json_lexer.token2double();
        //
        if( i + 1 == n_constant )
            json_lexer.check_next_char(']');
        else
            json_lexer.check_next_char(',');
    }
    //
    json_lexer.check_next_char(']');
    json_lexer.check_next_char(',');
    //
    ok &= constant_vec.size() == 1;
    ok &= constant_vec[0] == -2.0;
    // -----------------------------------------------------------------------
    // op_usage_vec
    json_lexer.check_next_string("op_usage_vec");
    //
    json_lexer.check_next_char(':');
    json_lexer.check_next_char('[');
    //
    json_lexer.next_non_neg_int();
    size_t n_usage = json_lexer.token2size_t();
    CppAD::vector<graph_op_enum> operator_vec(n_usage);
    //
    json_lexer.check_next_char(',');
    //
    // [ first_operator, ... , last_operator ]
    json_lexer.check_next_char('[');
    for(size_t i = 0; i < n_usage; ++i)
    {   // start next operator
        json_lexer.check_next_char('[');
        graph_op_enum op_usage;
        //
        // op_code
        json_lexer.next_non_neg_int();
        size_t op_code   = json_lexer.token2size_t();
        //
        // op_enum
        op_usage = op_code2enum[op_code];
        json_lexer.check_next_char(',');
        //
        size_t n_result, n_arg;
        if( op_usage != CppAD::graph::sum_graph_op )
        {   n_result = 1;
            n_arg    = 2;
        }
        else
        {   // n_result
            json_lexer.next_non_neg_int();
            n_result = json_lexer.token2size_t();
            json_lexer.check_next_char(',');
            ok &= n_result == 1;
            //
            // n_arg
            json_lexer.next_non_neg_int();
            n_arg = json_lexer.token2size_t();
            json_lexer.check_next_char(',');
            json_lexer.check_next_char('[');
            ok &= n_arg == 3;
            //
        }
        ok &= n_result == 1;
        CppAD::vector<size_t> arg_node(0);
        // first_arg_node, ... , last_arg_node
        for(size_t j = 0; j < n_arg; ++j)
        {   // next argument node
            json_lexer.next_non_neg_int();
            size_t argument_node = json_lexer.token2size_t();
            arg_node.push_back( argument_node );
            //
            if( j + 1 == n_arg )
                json_lexer.check_next_char(']');
            else
                json_lexer.check_next_char(',');
        }
        if( op_usage == CppAD::graph::sum_graph_op )
        {
            json_lexer.check_next_char(']');
            ok &= arg_node.size() == 3;
            ok &= arg_node[0] == 1;
            ok &= arg_node[1] == 2;
            ok &= arg_node[2] == 3;
        }
        else
        {   ok &= arg_node.size() == 2;
            ok &= arg_node[0] == 5;
            ok &= arg_node[1] == 5;
        }
        //
        // end of this operator
        operator_vec[i] = op_usage;
        //
        if( i + 1 == n_usage )
            json_lexer.check_next_char(']');
        else
            json_lexer.check_next_char(',');

    }
    json_lexer.check_next_char(']');
    //
    json_lexer.check_next_char(',');
    //
    ok &= operator_vec.size() == 2;
    //
    graph_op_enum op_enum = operator_vec[0];
    ok &= op_enum == CppAD::graph::sum_graph_op;
    //
    op_enum   = operator_vec[1];
    ok &= op_enum == CppAD::graph::mul_graph_op;
    // -----------------------------------------------------------------------
    // dependent_vec
    json_lexer.check_next_string("dependent_vec");
    json_lexer.check_next_char(':');
    json_lexer.check_next_char('[');
    json_lexer.next_non_neg_int();
    size_t n_dependent = json_lexer.token2size_t();
    CppAD::vector<size_t> dependent_vec(n_dependent);
    //
    json_lexer.check_next_char(',');
    //
    // [ first_dependent, ... , last_dependent ]
    json_lexer.check_next_char('[');
    for(size_t i = 0; i < n_dependent; ++i)
    {   json_lexer.next_float();
        dependent_vec[i] = json_lexer.token2size_t();
        //
        if( i + 1 == n_dependent )
            json_lexer.check_next_char(']');
        else
            json_lexer.check_next_char(',');
    }
    //
    json_lexer.check_next_char(']');
    //
    ok &= dependent_vec.size() == 1;
    ok &= dependent_vec[0] == 6;
    // -----------------------------------------------------------------------
    // }
    json_lexer.check_next_char('}');
    //
    return ok;
}
