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
$begin graph_discrete_op.cpp$$
$spell
    add
    Json
$$

$section C++ AD Graph add Operator: Example and Test$$

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
    using std::string;
    //
    // The discrete function does not exist until its AD version is called
    heaviside( CppAD::AD<double>( 0.0 ) );
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : heaviside(p[0])
    // node_4 : heaviside(p[1])
    // node_5 = heaviside(p[0]) + heaviside(p[1])
    // y[0]   = heaviside(p[0]) + heaviside(p[1])
    //
    // C++ graph object
    CppAD::cpp_graph graph_obj;
    //
    // operator being used
    CppAD::graph::graph_op_enum op_enum;
    //
    // set scalars
    graph_obj.function_name_set("discrete_op example");
    size_t n_dynamic_ind = 1;
    graph_obj.n_dynamic_ind_set(n_dynamic_ind);
    size_t n_variable_ind = 1;
    graph_obj.n_variable_ind_set(n_variable_ind);
    //
    // name_index corresponding to heaviside function
    size_t name_index = graph_obj.discrete_name_vec_size();
    graph_obj.discrete_name_vec_push_back("heaviside");
    //
    // heaviside(p[0])
    op_enum = CppAD::graph::discrete_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(name_index);  // name_index
    graph_obj.operator_arg_push_back(1);           // node arg
    //
    // heaviside(x[0])
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(name_index);  // name_index
    graph_obj.operator_arg_push_back(2);           // node arg
    //
    // heaviside(p[0]) + heaviside(x[0])
    op_enum = CppAD::graph::add_graph_op;
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(3);           // first node arg
    graph_obj.operator_arg_push_back(4);           // second node arg
    //
    // y[0] = heaviside(p[0]) + heaviside(x[0])
    graph_obj.dependent_vec_push_back(5);
    //
    // f(x, p) = heaviside(p[0]) + heaviside(x[0])
    CppAD::ADFun<double> f;
    f.from_graph(graph_obj);
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
    // Convert function to graph and back again
    f.to_graph(graph_obj);
    f.from_graph(graph_obj);
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
