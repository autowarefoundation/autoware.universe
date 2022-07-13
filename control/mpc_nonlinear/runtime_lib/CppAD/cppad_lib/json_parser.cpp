/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */


# include <cppad/core/graph/cpp_graph.hpp>
# include <cppad/local/graph/json_lexer.hpp>
# include <cppad/local/define.hpp>
# include <cppad/local/atomic_index.hpp>
# include <cppad/utility/to_string.hpp>

// documentation for this routine is in the file below
# include <cppad/local/graph/json_parser.hpp>

void CppAD::local::graph::json_parser(
    const std::string& json      ,
    cpp_graph&         graph_obj )
{   using std::string;
    //
    //
    // match_any_string
    const string match_any_string = "";
    //
    // initilize atomic_name_vec
    graph_obj.initialize();
    //
    // The values in this vector will be set while parsing op_define_vec.
    // Note that the values in op_code2enum[0] are not used.
    CppAD::vector<graph_op_enum> op_code2enum(1);
    //
    // -----------------------------------------------------------------------
    // json_lexer constructor checks for { at beginning
    CppAD::local::graph::json_lexer json_lexer(json);
    //
    // "function_name" : function_name
    json_lexer.check_next_string("function_name");
    json_lexer.check_next_char(':');
    json_lexer.check_next_string(match_any_string);
    std::string function_name = json_lexer.token();
    graph_obj.function_name_set(function_name);
    json_lexer.set_function_name(function_name);
    json_lexer.check_next_char(',');
    //
    // -----------------------------------------------------------------------
    // "op_define_vec" : [ n_define, [
    json_lexer.check_next_string("op_define_vec");
    json_lexer.check_next_char(':');
    json_lexer.check_next_char('[');
    //
    json_lexer.next_non_neg_int();
    size_t n_define = json_lexer.token2size_t();
    json_lexer.check_next_char(',');
    //
    json_lexer.check_next_char('[');
    for(size_t i = 0; i < n_define; ++i)
    {   // {
        json_lexer.check_next_char('{');
        //
        // "op_code" : op_code,
        json_lexer.check_next_string("op_code");
        json_lexer.check_next_char(':');
        json_lexer.next_non_neg_int();
# ifndef NDEBUG
        size_t op_code = json_lexer.token2size_t();
        CPPAD_ASSERT_UNKNOWN( op_code == op_code2enum.size() );
# endif
        json_lexer.check_next_char(',');
        //
        // "name" : name
        json_lexer.check_next_string("name");
        json_lexer.check_next_char(':');
        json_lexer.check_next_string(match_any_string);
        string name = json_lexer.token();
        graph_op_enum op_enum = op_name2enum[name];
        //
        // op_code2enum for this op_code
        op_code2enum.push_back(op_enum);
        //
        size_t n_arg = op_enum2fixed_n_arg[op_enum];
        if( n_arg > 0 )
        {   // , "narg" : n_arg
            json_lexer.check_next_char(',');
            json_lexer.check_next_string("n_arg");
            json_lexer.check_next_char(':');
            json_lexer.next_non_neg_int();
            if( n_arg != json_lexer.token2size_t() )
            {   string expected = CppAD::to_string(n_arg);
                string found    = json_lexer.token();
                json_lexer.report_error(expected, found);
            }
        }
        json_lexer.check_next_char('}');
        //
        // , (if not last entry)
        if( i + 1 < n_define )
            json_lexer.check_next_char(',');
    }
    json_lexer.check_next_char(']');
    // ],
    json_lexer.check_next_char(']');
    json_lexer.check_next_char(',');
    //
    // -----------------------------------------------------------------------
    // "n_dynamic_ind" : n_dynamic_ind ,
    json_lexer.check_next_string("n_dynamic_ind");
    json_lexer.check_next_char(':');
    //
    json_lexer.next_non_neg_int();
    size_t n_dynamic_ind = json_lexer.token2size_t();
    graph_obj.n_dynamic_ind_set(n_dynamic_ind);
    //
    json_lexer.check_next_char(',');
    // -----------------------------------------------------------------------
    // "n_variable_ind" : n_variable_ind ,
    json_lexer.check_next_string("n_variable_ind");
    json_lexer.check_next_char(':');
    //
    json_lexer.next_non_neg_int();
    size_t n_variable_ind = json_lexer.token2size_t();
    graph_obj.n_variable_ind_set(n_variable_ind);
    //
    json_lexer.check_next_char(',');
    // -----------------------------------------------------------------------
    // "constant_vec": [ n_constant, [ first_constant, ..., last_constant ] ],
    json_lexer.check_next_string("constant_vec");
    json_lexer.check_next_char(':');
    json_lexer.check_next_char('[');
    //
    json_lexer.next_non_neg_int();
    size_t n_constant = json_lexer.token2size_t();
    //
    json_lexer.check_next_char(',');
    //
    // [ first_constant, ... , last_constant ]
    json_lexer.check_next_char('[');
    for(size_t i = 0; i < n_constant; ++i)
    {   json_lexer.next_float();
        graph_obj.constant_vec_push_back( json_lexer.token2double() );
        //
        if( i + 1 < n_constant )
            json_lexer.check_next_char(',');
    }
    json_lexer.check_next_char(']');
    json_lexer.check_next_char(']');
    json_lexer.check_next_char(',');
    // -----------------------------------------------------------------------
    // "op_usage_vec": [ n_usage, [ first_op_usage, ..., last_op_usage ] ],
    json_lexer.check_next_string("op_usage_vec");
    json_lexer.check_next_char(':');
    json_lexer.check_next_char('[');
    //
    json_lexer.next_non_neg_int();
    size_t n_usage = json_lexer.token2size_t();
    //
    json_lexer.check_next_char(',');
    json_lexer.check_next_char('[');
    //
    // index for strings in current operator
    CppAD::vector<size_t> str_index;
    for(size_t i = 0; i < n_usage; ++i)
    {   str_index.resize(0);
        //
        // [ op_code,
        json_lexer.check_next_char('[');
        //
        // op_enum
        json_lexer.next_non_neg_int();
        graph_op_enum op_enum    = op_code2enum[ json_lexer.token2size_t() ];
        json_lexer.check_next_char(',');
        //
        size_t n_result = 1;
        size_t n_arg    = op_enum2fixed_n_arg[op_enum];
        //
        // check if number of arguments is fixed
        bool fixed        = n_arg > 0;
        if( ! fixed )
        {   if( op_enum == discrete_graph_op )
            {   // name,
                json_lexer.check_next_string(match_any_string);
                string name = json_lexer.token();
                json_lexer.check_next_char(',');
                //
                size_t name_index = graph_obj.discrete_name_vec_find(name);
                if( name_index == graph_obj.discrete_name_vec_size() )
                    graph_obj.discrete_name_vec_push_back( name );
                str_index.push_back(name_index);
            }
            else if( op_enum == atom_graph_op )
            {   // name
                json_lexer.check_next_string(match_any_string);
                string name = json_lexer.token();
                json_lexer.check_next_char(',');
                //
                size_t name_index = graph_obj.atomic_name_vec_find(name);
                if( name_index == graph_obj.atomic_name_vec_size() )
                    graph_obj.atomic_name_vec_push_back( name );
                str_index.push_back(name_index);
            }
            else if( op_enum == print_graph_op )
            {   // before
                json_lexer.check_next_string(match_any_string);
                string before = json_lexer.token();
                json_lexer.check_next_char(',');
                //
                size_t before_index = graph_obj.print_text_vec_find(before);
                if( before_index == graph_obj.print_text_vec_size() )
                    graph_obj.print_text_vec_push_back(before);
                str_index.push_back(before_index);
                //
                // aftere
                json_lexer.check_next_string(match_any_string);
                string after = json_lexer.token();
                json_lexer.check_next_char(',');
                //
                size_t after_index = graph_obj.print_text_vec_find(after);
                if( after_index == graph_obj.print_text_vec_size() )
                    graph_obj.print_text_vec_push_back(after);
                str_index.push_back(after_index);
            }
            else CPPAD_ASSERT_UNKNOWN(
                op_enum == comp_eq_graph_op ||
                op_enum == comp_le_graph_op ||
                op_enum == comp_lt_graph_op ||
                op_enum == comp_ne_graph_op ||
                op_enum == sum_graph_op
            );
            // n_result,
            json_lexer.next_non_neg_int();
            n_result = json_lexer.token2size_t();
            json_lexer.check_next_char(',');
            //
            // n_arg, [
            json_lexer.next_non_neg_int();
            n_arg = json_lexer.token2size_t();
            json_lexer.check_next_char(',');
            json_lexer.check_next_char('[');
        }
        //
        // atom_graph_op: name_index, n_result, n_arg
        // come before first argument
        if( op_enum == atom_graph_op )
        {   // name_index, n_result, n_arg come before first_node
            size_t name_index = str_index[0];
            CPPAD_ASSERT_UNKNOWN(
                name_index < graph_obj.atomic_name_vec_size()
            );
            graph_obj.operator_arg_push_back( name_index );
            graph_obj.operator_arg_push_back( n_result );
            graph_obj.operator_arg_push_back( n_arg );
        }
        // discrete_op: name_index comes before first argument
        if( op_enum == discrete_graph_op )
        {   size_t name_index = str_index[0];
            graph_obj.operator_arg_push_back( name_index );
        }
        // print_op: before_index, after_index come before first argument
        if( op_enum == print_graph_op )
        {   size_t before_index = str_index[0];
            size_t after_index  = str_index[1];
            graph_obj.operator_arg_push_back( before_index );
            graph_obj.operator_arg_push_back( after_index );
        }
        //
        // sum_graph_op: n_arg comes before first argument
        if( op_enum == sum_graph_op )
            graph_obj.operator_arg_push_back( n_arg );
        //
        // operator_vec
        graph_op_enum op_usage;
        op_usage = op_enum;
        graph_obj.operator_vec_push_back( op_usage );
        for(size_t j = 0; j < n_arg; ++j)
        {   // next_arg
            json_lexer.next_non_neg_int();
            size_t argument_node = json_lexer.token2size_t();
            graph_obj.operator_arg_push_back( argument_node );
            //
            // , (if not last entry)
            if( j + 1 < n_arg )
                json_lexer.check_next_char(',');
        }
        json_lexer.check_next_char(']');
        if( ! fixed )
            json_lexer.check_next_char(']');
        //
        // , (if not last entry)
        if( i + 1 < n_usage )
            json_lexer.check_next_char(',');
    }
    json_lexer.check_next_char(']');
    json_lexer.check_next_char(']');
    json_lexer.check_next_char(',');
    // -----------------------------------------------------------------------
    // "dependent_vec": [ n_dependent, [first_dependent, ..., last_dependent] ]
    json_lexer.check_next_string("dependent_vec");
    json_lexer.check_next_char(':');
    json_lexer.check_next_char('[');
    //
    json_lexer.next_non_neg_int();
    size_t n_dependent = json_lexer.token2size_t();
    json_lexer.check_next_char(',');
    //
    json_lexer.check_next_char('[');
    for(size_t i = 0; i < n_dependent; ++i)
    {   json_lexer.next_float();
        graph_obj.dependent_vec_push_back( json_lexer.token2size_t() );
        //
        if( i + 1 < n_dependent )
            json_lexer.check_next_char(',');
    }
    json_lexer.check_next_char(']');
    json_lexer.check_next_char(']');
    // -----------------------------------------------------------------------
    // end of Json object
    json_lexer.check_next_char('}');
    //
    return;
}
