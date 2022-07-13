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
$begin graph_unary_op.cpp$$
$spell
    sin
$$

$section Graph Unary Operator: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace { // BEGIN_EMPTY_NAMESPACE

typedef double (*unary_fun_t)(double);

bool test_unary_fun(unary_fun_t fun, CppAD::graph::graph_op_enum op_enum)
{   bool ok = true;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // C++ graph object
    CppAD::cpp_graph graph_obj;
    //
    // value of constant in function
    CPPAD_TESTVECTOR(double) p(1), x(1), c(1);
    c[0] = 0.1;
    p[0] = 0.2;
    x[0] = 0.3;
    if( std::isnan( fun( c[0] ) ) )
        c[0] = 1.0;
    if( std::isnan( fun( p[0] ) ) )
        p[0] = 2.0;
    if( std::isnan( fun( x[0] ) ) )
        x[0] = 3.0;
    //
    // set scalars
    graph_obj.function_name_set("unary_op example");
    size_t n_dynamic_ind = 1;
    graph_obj.n_dynamic_ind_set(n_dynamic_ind);
    size_t n_variable_ind = 1;
    graph_obj.n_variable_ind_set(n_variable_ind);
    graph_obj.constant_vec_push_back( c[0] );
    //
    // node_4 : sin(p[0])
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(1);
    //
    // node_5 : sin(x[0])
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(2);
    //
    // node_6 : sin(c[0])
    graph_obj.operator_vec_push_back(op_enum);
    graph_obj.operator_arg_push_back(3);
    //
    // y[0]   = sin(p[0])
    graph_obj.dependent_vec_push_back(4);
    // y[1]   = sin(x[0])
    graph_obj.dependent_vec_push_back(5);
    // y[2]   = sin(c[0])
    graph_obj.dependent_vec_push_back(6);
    //
    // f(p, x) = y
    CppAD::ADFun<double> f;
    f.from_graph(graph_obj);
    ok &= f.Domain() == 1;
    ok &= f.size_dyn_ind() == 1;
    ok &= f.Range() == 3;
    //
    //
    // compute y = f(p, x)
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], fun(p[0]), eps99, eps99);
    ok &= CppAD::NearEqual(y[1], fun(x[0]), eps99, eps99);
    ok &= CppAD::NearEqual(y[2], fun(c[0]), eps99, eps99);
    // ------------------------------------------------------------------
    // Convert to Graph graph and back again
    f.to_graph(graph_obj);
    f.from_graph(graph_obj);
    // -------------------------------------------------------------------
    //
    // compute y = f(p, x)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], fun(p[0]), eps99, eps99);
    ok &= CppAD::NearEqual(y[1], fun(x[0]), eps99, eps99);
    ok &= CppAD::NearEqual(y[2], fun(c[0]), eps99, eps99);
    //
    return ok;
}

double sign(double x)
{   return CppAD::sign(x);
}
double neg(double x)
{   return - x;
}

} // END_EMPTY_NAMESPACE

bool cpp_graph(void)
{   bool ok = true;
    ok     &= test_unary_fun(std::fabs,   CppAD::graph::abs_graph_op);
    ok     &= test_unary_fun(std::acos,   CppAD::graph::acos_graph_op);
    ok     &= test_unary_fun(std::acosh,  CppAD::graph::acosh_graph_op);
    ok     &= test_unary_fun(std::asinh,  CppAD::graph::asinh_graph_op);
    ok     &= test_unary_fun(std::atanh,  CppAD::graph::atanh_graph_op);
    ok     &= test_unary_fun(std::erf,    CppAD::graph::erf_graph_op);
    ok     &= test_unary_fun(std::erfc,   CppAD::graph::erfc_graph_op);
    ok     &= test_unary_fun(std::expm1,  CppAD::graph::expm1_graph_op);
    ok     &= test_unary_fun(std::log1p,  CppAD::graph::log1p_graph_op);
    ok     &= test_unary_fun(neg,         CppAD::graph::neg_graph_op);
    ok     &= test_unary_fun(sign,        CppAD::graph::sign_graph_op);
    ok     &= test_unary_fun(std::sinh,   CppAD::graph::sinh_graph_op);
    ok     &= test_unary_fun(std::sin,    CppAD::graph::sin_graph_op);
    ok     &= test_unary_fun(std::sqrt,   CppAD::graph::sqrt_graph_op);
    ok     &= test_unary_fun(std::tanh,   CppAD::graph::tanh_graph_op);
    ok     &= test_unary_fun(std::tan,    CppAD::graph::tan_graph_op);
    ok     &= test_unary_fun(std::acosh,  CppAD::graph::acosh_graph_op);
    ok     &= test_unary_fun(std::acosh,  CppAD::graph::acosh_graph_op);
    ok     &= test_unary_fun(std::sin,    CppAD::graph::sin_graph_op);
    //
    return ok;
}
