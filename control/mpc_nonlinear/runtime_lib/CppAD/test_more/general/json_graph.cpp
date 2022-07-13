/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/cppad.hpp>

namespace { // BEGIN_EMPTY_NAMESPACE
// ---------------------------------------------------------------------------
bool comp_op_dyn_dyn(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    //
    // AD graph example
    // node[0:4] : p[0], p[1], p[2], p[3], p[4]
    // node_5    : x[0]
    //           : p[0] == p[1]
    //           : p[0] <= p[2]
    //           : p[0] <  p[3]
    //           : p[0] != p[4]
    // y[0]      = p[0]
    std::string json =
        "{\n"
        "   'function_name'  : 'comp_op example',\n"
        "   'op_define_vec'  : [ 4, [\n"
        "       { 'op_code':1, 'name':'comp_eq'            } ,\n"
        "       { 'op_code':2, 'name':'comp_le'            } ,\n"
        "       { 'op_code':3, 'name':'comp_lt'            } ,\n"
        "       { 'op_code':4, 'name':'comp_ne'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 5,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 0, 2, [1, 2 ] ] ,\n" // p[0] == p[1]
        "       [ 2, 0, 2, [1, 3 ] ] ,\n" // p[0] <= p[2]
        "       [ 3, 0, 2, [1, 4 ] ] ,\n" // p[0] <  p[3]
        "       [ 4, 0, 2, [1, 5 ] ] ]\n" // p[0] != p[4]
        "   ],\n"
        "   'dependent_vec' : [ 1, [1] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = p[0]
    CppAD::ADFun<double> f;
    f.from_json(json);
    //
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 5;
    //
    // set independent variables and parameters so all comparisons true
    vector<double> x(1), p(5);
    x[0] = 1.0;
    p[0] = 3.0;
    p[1] = p[0];
    p[2] = p[0] + 1.0;
    p[3] = p[0] + 1.0;
    p[4] = p[0] + 1.0;
    //
    // compute y = f(x; p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check that all comparisons are true
    ok &= f.compare_change_number() == 0;
    //
    // case where all the comparisons are false
    p[1] = p[0] - 1.0;
    p[2] = p[0] - 1.0;
    p[3] = p[0] - 1.0;
    p[4] = p[0];
    f.new_dynamic(p);
    y = f.Forward(0, x);
    y = f.Forward(0, x);
    ok &= f.compare_change_number() == 4;
    //
    // -----------------------------------------------------------------------
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    // -----------------------------------------------------------------------
    //
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 5;
    //
    // set independent variables and parameters so all comparisons true
    p[0] = 3.0;
    p[1] = p[0];
    p[2] = p[0] + 1.0;
    p[3] = p[0] + 1.0;
    p[4] = p[0] + 1.0;
    //
    // compute y = f(x)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check that all comparisons are true
    ok &= f.compare_change_number() == 0;
    //
    // case where all the comparisons are false
    p[1] = p[0] - 1.0;
    p[2] = p[0] - 1.0;
    p[3] = p[0] - 1.0;
    p[4] = p[0];
    f.new_dynamic(p);
    y = f.Forward(0, x);
    ok &= f.compare_change_number() == 4;
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool comp_op_var_var(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    //
    // AD graph example
    // node[1:5] : x[0], x[1], x[2], x[3], x[4]
    //           : x[0] == x[1]
    //           : x[0] <= x[2]
    //           : x[0] <  x[3]
    //           : x[0] != x[4]
    // y[0]      = x[0]
    std::string json =
        "{\n"
        "   'function_name'  : 'comp_op example',\n"
        "   'op_define_vec'  : [ 4, [\n"
        "       { 'op_code':1, 'name':'comp_eq'            } ,\n"
        "       { 'op_code':2, 'name':'comp_le'            } ,\n"
        "       { 'op_code':3, 'name':'comp_lt'            } ,\n"
        "       { 'op_code':4, 'name':'comp_ne'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 0,\n"
        "   'n_variable_ind' : 5,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 0, 2, [1, 2 ] ] ,\n" // x[0] == x[1]
        "       [ 2, 0, 2, [1, 3 ] ] ,\n" // x[0] <= x[2]
        "       [ 3, 0, 2, [1, 4 ] ] ,\n" // x[0] <  x[3]
        "       [ 4, 0, 2, [1, 5 ] ] ]\n" // x[0] != x[4]
        "   ],\n"
        "   'dependent_vec' : [ 1, [1] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = p[0]
    CppAD::ADFun<double> f;
    f.from_json(json);
    //
    ok &= f.Domain() == 5;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 0;
    //
    // set independent variables and parameters so all comparisons true
    vector<double> x(5);
    x[0] = 3.0;
    x[1] = x[0];
    x[2] = x[0] + 1.0;
    x[3] = x[0] + 1.0;
    x[4] = x[0] + 1.0;
    //
    // compute y = f(x)
    vector<double> y = f.Forward(0, x);
    //
    // check that all comparisons are true
    ok &= f.compare_change_number() == 0;
    //
    // case where all the comparisons are false
    x[1] = x[0] - 1.0;
    x[2] = x[0] - 1.0;
    x[3] = x[0] - 1.0;
    x[4] = x[0];
    y = f.Forward(0, x);
    ok &= f.compare_change_number() == 4;
    //
    // -----------------------------------------------------------------------
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    // -----------------------------------------------------------------------
    //
    ok &= f.Domain() == 5;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 0;
    //
    // set independent variables and parameters so all comparisons true
    x[0] = 3.0;
    x[1] = x[0];
    x[2] = x[0] + 1.0;
    x[3] = x[0] + 1.0;
    x[4] = x[0] + 1.0;
    //
    // compute y = f(x)
    y = f.Forward(0, x);
    //
    // check that all comparisons are true
    ok &= f.compare_change_number() == 0;
    //
    // case where all the comparisons are false
    x[1] = x[0] - 1.0;
    x[2] = x[0] - 1.0;
    x[3] = x[0] - 1.0;
    x[4] = x[0];
    y = f.Forward(0, x);
    ok &= f.compare_change_number() == 4;
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool comp_op_dyn_var(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    //
    // AD graph example
    // node_1    : p[0]
    // node[2:5] : x[0], x[1], x[2], x[3]
    //           : p[0] == x[0]
    //           : p[0] <= x[1]
    //           : p[0] <  x[2]
    //           : p[0] != x[3]
    // y[0]      = p[0]
    std::string json =
        "{\n"
        "   'function_name'  : 'comp_op example',\n"
        "   'op_define_vec'  : [ 4, [\n"
        "       { 'op_code':1, 'name':'comp_eq'            } ,\n"
        "       { 'op_code':2, 'name':'comp_le'            } ,\n"
        "       { 'op_code':3, 'name':'comp_lt'            } ,\n"
        "       { 'op_code':4, 'name':'comp_ne'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 4,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 0, 2, [1, 2 ] ] ,\n" // p[0] == x[0]
        "       [ 2, 0, 2, [1, 3 ] ] ,\n" // p[0] <= x[1]
        "       [ 3, 0, 2, [1, 4 ] ] ,\n" // p[0] <  x[2]
        "       [ 4, 0, 2, [1, 5 ] ] ]\n" // p[0] != x[3]
        "   ],\n"
        "   'dependent_vec' : [ 1, [1] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = p[0]
    CppAD::ADFun<double> f;
    f.from_json(json);
    //
    ok &= f.Domain() == 4;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // set independent variables and parameters so all comparisons true
    vector<double> x(4), p(1);
    p[0] = 3.0;
    x[0] = p[0];
    x[1] = p[0] + 1.0;
    x[2] = p[0] + 1.0;
    x[3] = p[0] + 1.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check that all comparisons are true
    ok &= f.compare_change_number() == 0;
    //
    // case where all the comparisons are false
    x[0] = p[0] - 1.0;
    x[1] = p[0] - 1.0;
    x[2] = p[0] - 1.0;
    x[3] = p[0];
    f.new_dynamic(p);
    y = f.Forward(0, x);
    ok &= f.compare_change_number() == 4;
    //
    // -----------------------------------------------------------------------
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    // -----------------------------------------------------------------------
    //
    ok &= f.Domain() == 4;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // set independent variables and parameters so all comparisons true
    p[0] = 3.0;
    x[0] = p[0];
    x[1] = p[0] + 1.0;
    x[2] = p[0] + 1.0;
    x[3] = p[0] + 1.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check that all comparisons are true
    ok &= f.compare_change_number() == 0;
    //
    // case where all the comparisons are false
    x[0] = p[0] - 1.0;
    x[1] = p[0] - 1.0;
    x[2] = p[0] - 1.0;
    x[3] = p[0];
    f.new_dynamic(p);
    y = f.Forward(0, x);
    ok &= f.compare_change_number() == 4;
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool comp_op_var_dyn(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    //
    // AD graph example
    // node[1:4] : p[0], p[1], p[2], p[3]
    // node_5    : x[0]
    //           : x[0] == p[0]
    //           : x[0] <= p[1]
    //           : x[0] <  p[2]
    //           : x[0] != p[3]
    // y[0]      = p[0]
    std::string json =
        "{\n"
        "   'function_name'  : 'comp_op example',\n"
        "   'op_define_vec'  : [ 4, [\n"
        "       { 'op_code':1, 'name':'comp_eq'            } ,\n"
        "       { 'op_code':2, 'name':'comp_le'            } ,\n"
        "       { 'op_code':3, 'name':'comp_lt'            } ,\n"
        "       { 'op_code':4, 'name':'comp_ne'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 4,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 0, 2, [5, 1 ] ] ,\n" // x[0] == p[0]
        "       [ 2, 0, 2, [5, 2 ] ] ,\n" // x[0] <= p[1]
        "       [ 3, 0, 2, [5, 3 ] ] ,\n" // x[0] <  p[2]
        "       [ 4, 0, 2, [5, 4 ] ] ]\n" // x[0] != p[3]
        "   ],\n"
        "   'dependent_vec' : [ 1, [1] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = p[0]
    CppAD::ADFun<double> f;
    f.from_json(json);
    //
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 4;
    //
    // set independent variables and parameters so all comparisons true
    vector<double> x(1), p(4);
    x[0] = 3.0;
    p[0] = x[0];
    p[1] = x[0] + 1.0;
    p[2] = x[0] + 1.0;
    p[3] = x[0] + 1.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check that all comparisons are true
    ok &= f.compare_change_number() == 0;
    //
    // case where all the comparisons are false
    p[0] = x[0] - 1.0;
    p[1] = x[0] - 1.0;
    p[2] = x[0] - 1.0;
    p[3] = x[0];
    f.new_dynamic(p);
    y = f.Forward(0, x);
    ok &= f.compare_change_number() == 4;
    //
    // -----------------------------------------------------------------------
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    // -----------------------------------------------------------------------
    //
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 4;
    //
    // set independent variables and parameters so all comparisons true
    x[0] = 3.0;
    p[0] = x[0];
    p[1] = x[0] + 1.0;
    p[2] = x[0] + 1.0;
    p[3] = x[0] + 1.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check that all comparisons are true
    ok &= f.compare_change_number() == 0;
    //
    // case where all the comparisons are false
    p[0] = x[0] - 1.0;
    p[1] = x[0] - 1.0;
    p[2] = x[0] - 1.0;
    p[3] = x[0];
    f.new_dynamic(p);
    y = f.Forward(0, x);
    ok &= f.compare_change_number() == 4;
    //
    return ok;
}
// ===========================================================================
bool acosh_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : acosh(p[0])
    // node_5 : acosh(x[0])
    // node_6 : acosh(c[0])
    // node_7 : acosh(p[0]) + acosh(x[0]) + acosh(c[0])
    // y[0]   = acosh(p[0]) + acosh(x[0]) + acosh(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'acosh_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'acosh', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'              } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ 1.3 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // acosh(p0)
        "       [ 1, 2]                ,\n" // acosh(x0)
        "       [ 1, 3]                ,\n" // acosh(c0)
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // acosh(p0)+acosh(x0)+acosh(c0)
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = acosh(p0) + acosh(x0) + acosh(c0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = 1.3;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 1.1;
    x[0] = 1.2;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = CppAD::acosh(p[0]) + CppAD::acosh(x[0]) + CppAD::acosh(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool log1p_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : log1p(p[0])
    // node_5 : log1p(x[0])
    // node_6 : log1p(c[0])
    // node_7 : log1p(p[0]) + log1p(x[0]) + log1p(c[0])
    // y[0]   = log1p(p[0]) + log1p(x[0]) + log1p(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'log1p_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'log1p', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'               } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ 0.3 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // log1p(p0)
        "       [ 1, 2]                ,\n" // log1p(x0)
        "       [ 1, 3]                ,\n" // log1p(c0)
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // log1p(p0)+log1p(x0)+log1p(c0)
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = log1p(p0) + log1p(x0) + log1p(c0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = 0.3;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = -0.1;
    x[0] = 0.2;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = CppAD::log1p(p[0]) + CppAD::log1p(x[0]) + CppAD::log1p(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool expm1_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : expm1(p[0])
    // node_5 : expm1(x[0])
    // node_6 : expm1(c[0])
    // node_7 : expm1(p[0]) + expm1(x[0]) + expm1(c[0])
    // y[0]   = expm1(p[0]) + expm1(x[0]) + expm1(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'expm1_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'expm1', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'              } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ 0.3 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // expm1(p0)
        "       [ 1, 2]                ,\n" // expm1(x0)
        "       [ 1, 3]                ,\n" // expm1(c0)
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // expm1(p0)+expm1(x0)+expm1(c0)
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = expm1(p0) + expm1(x0) + expm1(c0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = 0.3;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = -0.1;
    x[0] = 0.2;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = CppAD::expm1(p[0]) + CppAD::expm1(x[0]) + CppAD::expm1(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool erfc_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : erfc(p[0])
    // node_5 : erfc(x[0])
    // node_6 : erfc(c[0])
    // node_7 : erfc(p[0]) + erfc(x[0]) + erfc(c[0])
    // y[0]   = erfc(p[0]) + erfc(x[0]) + erfc(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'erfc_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'erfc', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'             } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ 0.3 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // erfc(p0)
        "       [ 1, 2]                ,\n" // erfc(x0)
        "       [ 1, 3]                ,\n" // erfc(c0)
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // erfc(p0)+erfc(x0)+erfc(c0)
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = erfc(p0) + erfc(x0) + erfc(c0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = 0.3;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = -0.1;
    x[0] = 0.2;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = CppAD::erfc(p[0]) + CppAD::erfc(x[0]) + CppAD::erfc(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool erf_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : erf(p[0])
    // node_5 : erf(x[0])
    // node_6 : erf(c[0])
    // node_7 : erf(p[0]) + erf(x[0]) + erf(c[0])
    // y[0]   = erf(p[0]) + erf(x[0]) + erf(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'erf_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'erf', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ 0.3 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // erf(p0)
        "       [ 1, 2]                ,\n" // erf(x0)
        "       [ 1, 3]                ,\n" // erf(c0)
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // erf(p0)+erf(x0)+erf(c0)
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = erf(p0) + erf(x0) + erf(c0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = 0.3;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = -0.1;
    x[0] = 0.2;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = CppAD::erf(p[0]) + CppAD::erf(x[0]) + CppAD::erf(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool atanh_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : atanh(p[0])
    // node_5 : atanh(x[0])
    // node_6 : atanh(c[0])
    // node_7 : atanh(p[0]) + atanh(x[0]) + atanh(c[0])
    // y[0]   = atanh(p[0]) + atanh(x[0]) + atanh(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'atanh_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'atanh', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'              } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ 0.3 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // atanh(p0)
        "       [ 1, 2]                ,\n" // atanh(x0)
        "       [ 1, 3]                ,\n" // atanh(c0)
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // atanh(p0)+atanh(x0)+atanh(c0)
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = atanh(p0) + atanh(x0) + atanh(c0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = 0.3;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = -0.1;
    x[0] = 0.2;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = CppAD::atanh(p[0]) + CppAD::atanh(x[0]) + CppAD::atanh(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool asinh_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : asinh(p[0])
    // node_5 : asinh(x[0])
    // node_6 : asinh(c[0])
    // node_7 : asinh(p[0]) + asinh(x[0]) + asinh(c[0])
    // y[0]   = asinh(p[0]) + asinh(x[0]) + asinh(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'asinh_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'asinh', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'              } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ 0.3 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // asinh(p0)
        "       [ 1, 2]                ,\n" // asinh(x0)
        "       [ 1, 3]                ,\n" // asinh(c0)
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // asinh(p0)+asinh(x0)+asinh(c0)
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = asinh(p0) + asinh(x0) + asinh(c0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = 0.3;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = -0.1;
    x[0] = 0.2;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = CppAD::asinh(p[0]) + CppAD::asinh(x[0]) + CppAD::asinh(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ===========================================================================
bool tan_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : tan(p[0])
    // node_5 : tan(x[0])
    // node_6 : tan(c[0])
    // node_7 : tan(p[0]) + tan(x[0]) + tan(c[0])
    // y[0]   = tan(p[0]) + tan(x[0]) + tan(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'tan_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'tan', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // tan(p[0])
        "       [ 1, 2]                ,\n" // tan(x[0])
        "       [ 1, 3]                ,\n" // tan(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // tan(p[0])+tan(x[0])+tan(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = tan(p_0) + tan(x_0) + tan(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant that is in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::tan(p[0]) + std::tan(x[0]) + std::tan(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool tanh_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : tanh(p[0])
    // node_5 : tanh(x[0])
    // node_6 : tanh(c[0])
    // node_7 : tanh(p[0]) + tanh(x[0]) + tanh(c[0])
    // y[0]   = tanh(p[0]) + tanh(x[0]) + tanh(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'tanh_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'tanh', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'             } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // tanh(p[0])
        "       [ 1, 2]                ,\n" // tanh(x[0])
        "       [ 1, 3]                ,\n" // tanh(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // tanh(p[0])+tanh(x[0])+tanh(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = tanh(p_0) + tanh(x_0) + tanh(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::tanh(p[0]) + std::tanh(x[0]) + std::tanh(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool sqrt_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : sqrt(p[0])
    // node_5 : sqrt(x[0])
    // node_6 : sqrt(c[0])
    // node_7 : sqrt(p[0]) + sqrt(x[0]) + sqrt(c[0])
    // y[0]   = sqrt(p[0]) + sqrt(x[0]) + sqrt(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'sqrt_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'sqrt', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'             } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ +0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // sqrt(p[0])
        "       [ 1, 2]                ,\n" // sqrt(x[0])
        "       [ 1, 3]                ,\n" // sqrt(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // sqrt(p[0])+sqrt(x[0])+sqrt(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = sqrt(p_0) + sqrt(x_0) + sqrt(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = +0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::sqrt(p[0]) + std::sqrt(x[0]) + std::sqrt(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool sin_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : sin(p[0])
    // node_5 : sin(x[0])
    // node_6 : sin(c[0])
    // node_7 : sin(p[0]) + sin(x[0]) + sin(c[0])
    // y[0]   = sin(p[0]) + sin(x[0]) + sin(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'sin_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'sin', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // sin(p[0])
        "       [ 1, 2]                ,\n" // sin(x[0])
        "       [ 1, 3]                ,\n" // sin(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // sin(p[0])+sin(x[0])+sin(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = sin(p_0) + sin(x_0) + sin(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::sin(p[0]) + std::sin(x[0]) + std::sin(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool sinh_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : sinh(p[0])
    // node_5 : sinh(x[0])
    // node_6 : sinh(c[0])
    // node_7 : sinh(p[0]) + sinh(x[0]) + sinh(c[0])
    // y[0]   = sinh(p[0]) + sinh(x[0]) + sinh(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'sinh_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'sinh', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'             } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // sinh(p[0])
        "       [ 1, 2]                ,\n" // sinh(x[0])
        "       [ 1, 3]                ,\n" // sinh(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // sinh(p[0])+sinh(x[0])+sinh(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = sinh(p_0) + sinh(x_0) + sinh(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::sinh(p[0]) + std::sinh(x[0]) + std::sinh(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool sign_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : sign(p[0])
    // node_5 : sign(x[0])
    // node_6 : sign(c[0])
    // node_7 : sign(p[0]) + sign(x[0]) + sign(c[0])
    // y[0]   = sign(p[0]) + sign(x[0]) + sign(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'sign_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'sign', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'             } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // sign(p[0])
        "       [ 1, 2]                ,\n" // sign(x[0])
        "       [ 1, 3]                ,\n" // sign(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // sign(p[0])+sign(x[0])+sign(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = sign(p_0) + sign(x_0) + sign(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = CppAD::sign(p[0]) + CppAD::sign(x[0]) + CppAD::sign(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool log_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : log(p[0])
    // node_5 : log(x[0])
    // node_6 : log(c[0])
    // node_7 : log(p[0]) + log(x[0]) + log(c[0])
    // y[0]   = log(p[0]) + log(x[0]) + log(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'log_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'log', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ +0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // log(p[0])
        "       [ 1, 2]                ,\n" // log(x[0])
        "       [ 1, 3]                ,\n" // log(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // log(p[0])+log(x[0])+log(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = log(p_0) + log(x_0) + log(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = +0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::log(p[0]) + std::log(x[0]) + std::log(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool exp_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : exp(p[0])
    // node_5 : exp(x[0])
    // node_6 : exp(c[0])
    // node_7 : exp(p[0]) + exp(x[0]) + exp(c[0])
    // y[0]   = exp(p[0]) + exp(x[0]) + exp(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'exp_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'exp', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // exp(p[0])
        "       [ 1, 2]                ,\n" // exp(x[0])
        "       [ 1, 3]                ,\n" // exp(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // exp(p[0])+exp(x[0])+exp(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = exp(p_0) + exp(x_0) + exp(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::exp(p[0]) + std::exp(x[0]) + std::exp(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool cos_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : cos(p[0])
    // node_5 : cos(x[0])
    // node_6 : cos(c[0])
    // node_7 : cos(p[0]) + cos(x[0]) + cos(c[0])
    // y[0]   = cos(p[0]) + cos(x[0]) + cos(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'cos_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'cos', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // cos(p[0])
        "       [ 1, 2]                ,\n" // cos(x[0])
        "       [ 1, 3]                ,\n" // cos(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // cos(p[0])+cos(x[0])+cos(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = cos(p_0) + cos(x_0) + cos(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::cos(p[0]) + std::cos(x[0]) + std::cos(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool cosh_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : cosh(p[0])
    // node_5 : cosh(x[0])
    // node_6 : cosh(c[0])
    // node_7 : cosh(p[0]) + cosh(x[0]) + cosh(c[0])
    // y[0]   = cosh(p[0]) + cosh(x[0]) + cosh(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'cosh_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'cosh', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'             } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // cosh(p[0])
        "       [ 1, 2]                ,\n" // cosh(x[0])
        "       [ 1, 3]                ,\n" // cosh(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // cosh(p[0])+cosh(x[0])+cosh(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = cosh(p_0) + cosh(x_0) + cosh(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::cosh(p[0]) + std::cosh(x[0]) + std::cosh(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool atan_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : atan(p[0])
    // node_5 : atan(x[0])
    // node_6 : atan(c[0])
    // node_7 : atan(p[0]) + atan(x[0]) + atan(c[0])
    // y[0]   = atan(p[0]) + atan(x[0]) + atan(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'atan_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'atan', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'             } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // atan(p[0])
        "       [ 1, 2]                ,\n" // atan(x[0])
        "       [ 1, 3]                ,\n" // atan(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // atan(p[0])+atan(x[0])+atan(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = atan(p_0) + atan(x_0) + atan(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::atan(p[0]) + std::atan(x[0]) + std::atan(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool asin_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : asin(p[0])
    // node_5 : asin(x[0])
    // node_6 : asin(c[0])
    // node_7 : asin(p[0]) + asin(x[0]) + asin(c[0])
    // y[0]   = asin(p[0]) + asin(x[0]) + asin(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'asin_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'asin', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'             } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // asin(p[0])
        "       [ 1, 2]                ,\n" // asin(x[0])
        "       [ 1, 3]                ,\n" // asin(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // asin(p[0])+asin(x[0])+asin(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = asin(p_0) + asin(x_0) + asin(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::asin(p[0]) + std::asin(x[0]) + std::asin(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool acos_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : acos(p[0])
    // node_5 : acos(x[0])
    // node_6 : acos(c[0])
    // node_7 : acos(p[0]) + acos(x[0]) + acos(c[0])
    // y[0]   = acos(p[0]) + acos(x[0]) + acos(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'acos_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'acos', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'             } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // acos(p[0])
        "       [ 1, 2]                ,\n" // acos(x[0])
        "       [ 1, 3]                ,\n" // acos(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // acos(p[0])+acos(x[0])+acos(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = acos(p_0) + acos(x_0) + acos(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::acos(p[0]) + std::acos(x[0]) + std::acos(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
bool abs_op(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // AD graph example
    // node_1 : p[0]
    // node_2 : x[0]
    // node_3 : c[0]
    // node_4 : abs(p[0])
    // node_5 : abs(x[0])
    // node_6 : abs(c[0])
    // node_7 : abs(p[0]) + abs(x[0]) + abs(c[0])
    // y[0]   = abs(p[0]) + abs(x[0]) + abs(c[0])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'abs_op example',\n"
        "   'op_define_vec'  : [ 2, [\n"
        "       { 'op_code':1, 'name':'abs', 'n_arg':1 } ,\n"
        "       { 'op_code':2, 'name':'sum'            } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 1,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
        "   'op_usage_vec'   : [ 4, [\n"
        "       [ 1, 1]                ,\n" // abs(p[0])
        "       [ 1, 2]                ,\n" // abs(x[0])
        "       [ 1, 3]                ,\n" // abs(c[0])
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // abs(p[0])+abs(x[0])+abs(c[0])
        "   ],\n"
        "   'dependent_vec' : [ 1, [7] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    // f(x, p) = abs(p_0) + abs(x_0) + abs(c_0)
    CppAD::ADFun<double> f;
    f.from_json(json);
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 1;
    //
    // value of constant in function
    vector<double> c(1);
    c[0] = -0.1;
    //
    // set independent variables and parameters
    vector<double> p(1), x(1);
    p[0] = 0.2;
    x[0] = 0.3;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    vector<double> y = f.Forward(0, x);
    //
    // check result
    double check = std::fabs(p[0]) + std::fabs(x[0]) + std::fabs(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    // Convert to Json graph and back again
    json = f.to_json();
    // std::cout << graph;
    f.from_json(json);
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);
    //
    return ok;
}
// ---------------------------------------------------------------------------
// Test conditonal expression
bool cexp_lt_variable(void)
{   bool ok = true;
    using CppAD::vector;
    //
    // An AD graph example
    // node_1 : x[0]
    // node_2 : x[1]
    // node_3 : cexp_lt(x[0], x[1], x[1], x[0])
    // y[0]   = max(x[0], x[1])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'cexp_lt test',\n"
        "   'op_define_vec'  : [ 1, [\n"
        "       { 'op_code':1, 'name':'cexp_lt', 'n_arg':4 } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 0,\n"
        "   'n_variable_ind' : 2,\n"
        "   'constant_vec'   : [ 0, [] ],\n"
        "   'op_usage_vec'   : [ 1, [\n"
        "       [ 1, 1, 2, 2, 1 ] ] \n"
        "   ],\n"
        "   'dependent_vec'   : [ 1, [3] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    CppAD::ADFun<double> f;
    f.from_json(json);
    // ---------------------------------------------------------------------
    ok &= f.Domain() == 2;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 0;
    //
    vector<double> x(2), y(1);
    x[0] = 2.0;
    x[1] = 3.0;
    y = f.Forward(0, x);
    ok &= y[0] == std::max(x[0], x[1]);
    //
    x[0] = 3.0;
    x[1] = 2.0;
    y = f.Forward(0, x);
    ok &= y[0] == std::max(x[0], x[1]);
    // ---------------------------------------------------------------------
    json = f.to_json();
    f.from_json(json);
    // ---------------------------------------------------------------------
    ok &= f.Domain() == 2;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 0;
    //
    x[0] = 2.0;
    x[1] = 3.0;
    y = f.Forward(0, x);
    ok &= y[0] == std::max(x[0], x[1]);
    //
    x[0] = 3.0;
    x[1] = 2.0;
    y = f.Forward(0, x);
    ok &= y[0] == std::max(x[0], x[1]);
    //
    // std::cout << graph;
    return ok;
}
// Test conditonal expression
bool cexp_lt_constant(void)
{   bool ok = true;
    using CppAD::vector;
    //
    // An AD graph example
    // node_1 : x[0]
    // node_2 : c[0]
    // node_3 : c[1]
    // node_4 : cexp_lt(c[0], c[1], c[1], c[0])
    // y[0]   = max(c[0], c[1])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'cexp_lt test',\n"
        "   'op_define_vec'  : [ 1, [\n"
        "       { 'op_code':1, 'name':'cexp_lt', 'n_arg':4 } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 0,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 2, [ 5.0, -5.0 ] ],\n"
        "   'op_usage_vec'   : [ 1, [\n"
        "       [ 1, 2, 3,  3, 2 ] ] \n"
        "   ],\n"
        "   'dependent_vec'   : [ 1, [4] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    CppAD::ADFun<double> f;
    f.from_json(json);
    // ---------------------------------------------------------------------
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 0;
    //
    vector<double> c(2), x(1), y(1);
    c[0] = 5.0;
    c[1] = -5.0;
    y = f.Forward(0, x);
    ok &= y[0] == std::max(c[0], c[1]);
    // ---------------------------------------------------------------------
    json = f.to_json();
    f.from_json(json);
    // ---------------------------------------------------------------------
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 0;
    //
    ok &= y[0] == std::max(c[0], c[1]);
    // ---------------------------------------------------------------------
    // std::cout << graph;
    return ok;
}
bool cexp_lt_dynamic(void)
{   bool ok = true;
    using CppAD::vector;
    //
    // An AD graph example
    // node_1 : p[0]
    // node_2 : p[1]
    // node_3 : x[0]
    // node_4 : cexp_lt(p[0], p[1], p[1], p[0])
    // y[0]   = max(p[0], p[1])
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'cexp_lt test',\n"
        "   'op_define_vec'  : [ 1, [\n"
        "       { 'op_code':1, 'name':'cexp_lt', 'n_arg':4 } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 2,\n"
        "   'n_variable_ind' : 1,\n"
        "   'constant_vec'   : [ 0, [ ] ],\n"
        "   'op_usage_vec'   : [ 1, [\n"
        "       [ 1, 1, 2, 2, 1 ] ] \n"
        "   ],\n"
        "   'dependent_vec'   : [ 1, [4] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    CppAD::ADFun<double> f;
    f.from_json(json);
    // ---------------------------------------------------------------------
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 2;
    //
    vector<double> p(2), x(1), y(1);
    p[0] = 3.0;
    p[1] = 2.0;
    f.new_dynamic(p);
    y = f.Forward(0, x);
    ok &= y[0] == std::max(p[0], p[1]);
    // ---------------------------------------------------------------------
    json = f.to_json();
    // std::cout << graph;
    f.from_json(json);
    // ---------------------------------------------------------------------
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 2;
    //
    p[0] = 2.0;
    p[1] = 3.0;
    f.new_dynamic(p);
    y = f.Forward(0, x);
    ok &= y[0] == std::max(p[0], p[1]);
    // ---------------------------------------------------------------------
    // std::cout << graph;
    return ok;
}
// ---------------------------------------------------------------------------
// Test atomic function that gets passed both variables and dynamic parameters
bool atomic_both(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    //
    // y[0] = p[0] * x[0]
    vector< AD<double> > ap(1), ax(1), ay(1);
    ap[0] = 2.0;
    ax[0] = 3.0;
    size_t abort_op_index = 0;
    bool   record_compare = false;
    CppAD::Independent(ax, abort_op_index, record_compare, ap);
    ay[0] = ap[0] * ax[0];
    CppAD::ADFun<double> f(ax, ay);
    //
    // Create a ckhpoint_two with name f(x; p).
    // (This also creates an atomic_three fucntion with same name.)
    bool internal_bool    = false;
    bool use_hes_sparsity = false;
    bool use_base2ad      = false;
    bool use_in_parallel  = false;
    CppAD::chkpoint_two<double> chk_f(f, "f(x; p)",
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );
    // -----------------------------------------------------------------------
    // g(u; p)
    vector< AD<double> > au(2), av(1);
    au[0] = 5.0;
    au[1] = 6.0;
    CppAD::Independent(au);
    ax[0] = au[0];
    chk_f(ax, av);          // v[0] = p[0] * u[0]
    ay[0] = au[1] + av[0];  // y[0] = u[1] + p[0] * u[0]
    CppAD::ADFun<double> g(au, ay);
    // ---------------------------------------------------------------------
    ok &= g.Domain() == 2;
    ok &= g.Range() == 1;
    ok &= g.size_dyn_ind() == 0;
    //
    // evalute g(u; p)
    vector<double> p(1), u(2), y(1);
    p[0] = 3.0;
    u[0] = 4.0;
    u[1] = 5.0;
    chk_f.new_dynamic(p);
    y    = g.Forward(0, u);
    //
    // check value
    double check = u[1] + p[0] * u[0];
    ok &= y[0] == check;
    // ---------------------------------------------------------------------
    std::string json = g.to_json();
    // std::cout << json;
    g.from_json(json);
    // ---------------------------------------------------------------------
    ok &= g.Domain() == 2;
    ok &= g.Range() == 1;
    ok &= g.size_dyn_ind() == 0;
    //
    // evalute g(u; p)
    p[0] = 4.0;
    u[0] = 5.0;
    u[1] = 6.0;
    chk_f.new_dynamic(p);
    y    = g.Forward(0, u);
    //
    // check value
    check = u[1] + p[0] * u[0];
    ok &= y[0] == check;

    // -----------------------------------------------------------------------
    // std::cout << graph;
    return ok;
}
// ---------------------------------------------------------------------------
// Test atomic function that only gets passed dynamic parameters
bool atomic_dynamic(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    //
    // y[0] = x[0] * x[1]
    vector< AD<double> > ax(2), ay(1);
    ax[0] = 2.0;
    ax[1] = 3.0;
    CppAD::Independent(ax);
    ay[0] = ax[0] * ax[1];
    CppAD::ADFun<double> f(ax, ay);
    //
    // Create a ckhpoint_two with name f(x).
    // (This also creates an atomic_three fucntion with same name.)
    bool internal_bool    = false;
    bool use_hes_sparsity = false;
    bool use_base2ad      = false;
    bool use_in_parallel  = false;
    CppAD::chkpoint_two<double> chk_f(f, "f(x)",
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );
    // -----------------------------------------------------------------------
    vector< AD<double> > au(1), aq(2), av(1);
    aq[0] = 4.0;
    aq[1] = 5.0;
    au[0] = 6.0;
    size_t abort_op_index = 0;
    bool   record_compare = false;
    CppAD::Independent(au, abort_op_index, record_compare, aq);
    chk_f(aq, av);          // v[0] = q[0] * q[1]
    ay[0] = au[0] + av[0];  // y[0] = u[0] + q[0] * q[1]
    CppAD::ADFun<double> g(au, ay);
    //
    // ---------------------------------------------------------------------
    ok &= g.Domain() == 1;
    ok &= g.Range() == 1;
    ok &= g.size_dyn_ind() == 2;
    //
    // set q in g(u; q)
    vector<double> q(2);
    q[0] = 2.0;
    q[1] = 3.0;
    g.new_dynamic(q);
    //
    // evalute g(u; q)
    vector<double> u(1), y(1);
    u[0] = 4.0;
    y    = g.Forward(0, u);
    //
    // check value
    double check = u[0] + q[0] * q[1];
    ok &= y[0] == check;
    // ---------------------------------------------------------------------
    std::string json = g.to_json();
    // std::cout << graph;
    g.from_json(json);
    //
    ok &= g.Domain() == 1;
    ok &= g.Range() == 1;
    ok &= g.size_dyn_ind() == 2;
    //
    // set q in g(u; q)
    q[0] = 3.0;
    q[1] = 4.0;
    g.new_dynamic(q);
    //
    // evalute g(u; q)
    u[0] = 5.0;
    y    = g.Forward(0, u);
    //
    // check value
    check = u[0] + q[0] * q[1];
    ok &= y[0] == check;
    // ----------------------------------------------------------------------
    // std::cout << graph;
    return ok;
}
// ---------------------------------------------------------------------------
// Test transforming to Json and back to a function
bool to_json_and_back(void)
{   bool ok = true;
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
        "   'function_name'  : 'to_json_and_back test',\n"
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
    CppAD::ADFun<double> fun;
    fun.from_json(json);
    json = fun.to_json();
    // For debugging
    // std::cout << "json = " << json;
    fun.from_json(json);
    //
    // Compute function value
    vector<double> p(1), x(2);
    p[0] = 1.0;
    x[0] = 2.0;
    x[1] = 3.0;
    fun.new_dynamic(p);
    vector<double> y = fun.Forward(0, x);
    ok  &= y[0] ==  (p[0] + x[0] + x[1]) * (p[0] + x[0] + x[1]);
    //
    // Conpute derivative value
    vector<double> jac = fun.Jacobian(x);
    ok &= jac[0] == 2.0 * (p[0] + x[0] + x[1]);
    ok &= jac[1] == 2.0 * (p[0] + x[0] + x[1]);
    //
    // Uncomment statement below to see the graph
    // std::cout << graph;
    //
    return ok;
}
// ---------------------------------------------------------------------------
// Test binary operators that should be implemented
bool binary_operators(void)
{   bool ok   = true;
    using CppAD::AD;
    //
    size_t np = 1;
    size_t nx = 2;
    size_t ny = 10;
    CPPAD_TESTVECTOR(double)       p(np),   x(nx);
    CPPAD_TESTVECTOR( AD<double> ) ap(np), ax(nx), ay(ny);
    for(size_t i = 0; i < np; ++i)
    {   ap[i] = 0.5;
        p[i]  = double(i + 1);
    }
    for(size_t i = 0; i < nx; ++i)
    {   ax[i] = 0.25;
        x[i]  = double(2 * i + 1);
    }
    CppAD::Independent(ax, ap);
    //
    size_t j = 0;
    ay[j++] = ap[0] + 2.0;    // dynamic + constant (and ParOp)
    ay[j++] = 2.0 + ap[0];    // constant + dynamic (and ParOp)
    ay[j++] = ax[0] + ap[0];  // variable + dynamic
    ay[j++] = ap[0] + ax[0];  // dynamic + variable
    ay[j++] = ax[0] + ax[1];  // variable + variable
    //
    ay[j++] = ap[0] * 2.0;    // dynamic * constant (and ParOp)
    ay[j++] = 2.0 * ap[0];    // constant * dynamic (and ParOp)
    ay[j++] = ax[0] * ap[0];  // variable * dynamic
    ay[j++] = ap[0] * ax[0];  // dynamic * variable
    ay[j++] = ax[0] * ax[1];  // variable * variable
    //
    ok &= j == ny;
    //
    // Create function
    CppAD::ADFun<double> f(ax, ay);
    //
    // Evaluate function at x before
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y_before = f.Forward(0, x);
    //
    // Convert to Json and back again
    std::string json = f.to_json();
    // std::cout << graph;
    f.from_json(json);
    //
    // Evaluate function at x after
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y_after = f.Forward(0, x);
    //
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    for(size_t i = 0; i < ny; ++i)
        ok &= CppAD::NearEqual( y_before[i], y_after[i], eps99, eps99 );
    //
    // Uncomment statement below to see the graph
    // std::cout << graph;
    return ok;
}
// ---------------------------------------------------------------------------
// Test cumulative sum operator
bool cumulative_sum(void)
{   bool ok   = true;
    using CppAD::AD;
    //
    size_t np = 2;
    size_t nx = 2;
    size_t ny = 1;
    CPPAD_TESTVECTOR(double)       p(np),   x(nx);
    CPPAD_TESTVECTOR( AD<double> ) ap(np), ax(nx), ay(ny);
    for(size_t i = 0; i < np; ++i)
    {   ap[i] = 0.5;
        p[i]  = double(i + 1);
    }
    for(size_t i = 0; i < nx; ++i)
    {   ax[i] = 0.25;
        x[i]  = double(2 * i + 1);
    }
    CppAD::Independent(ax, ap);
    //
    AD<double> asum = 0.0;
    asum +=  1.0 + ap[0];
    asum += ap[1] + 1.0;
    asum -= ap[1] + ap[0];
    //
    asum +=  1.0 + ax[0];
    asum += ax[1] + 1.0;
    asum -= ax[1] + ax[0];
    //
    asum += ap[0] + ax[0];
    asum += ax[1] + ap[1];
    //
    ay[0] = asum;
    //
    // Create function
    CppAD::ADFun<double> f(ax, ay);
    f.optimize();
    //
    // Evaluate function at x before
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y_before = f.Forward(0, x);
    //
    // Convert to Json and back again
    std::string json = f.to_json();
    // std::cout << graph;
    f.from_json(json);
    //
    // Evaluate function at x after
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y_after = f.Forward(0, x);
    //
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    for(size_t i = 0; i < ny; ++i)
        ok &= CppAD::NearEqual( y_before[i], y_after[i], eps99, eps99 );
    //
    // Uncomment statement below to see the graph
    // std::cout << graph;
    return ok;
}
// ---------------------------------------------------------------------------
// Test unary operators
bool unary(bool p_first)
{   bool ok   = true;
    using CppAD::AD;
    //
    size_t np = 11;
    size_t nx = 11;
    size_t ny = np + nx;
    CPPAD_TESTVECTOR(double)       p(np),  x(nx), y(ny);
    CPPAD_TESTVECTOR( AD<double> ) ap(np), ax(nx), ay(ny);
    for(size_t i = 0; i < np; ++i)
    {   p[i]  = double(i + 1) / double( np + nx + 1 );
        ap[i] = p[i] / 2.0;
    }
    for(size_t i = 0; i < nx; ++i)
    {   x[i]  = double(i + 1 + np) / double( np + nx + 1);
        ax[i] = x[i] / 2.0;
    }
    CppAD::Independent(ax, ap);
    //
    CPPAD_TESTVECTOR( AD<double> ) atmp(np + nx);
    if( p_first )
    {   for(size_t i = 0; i < np; ++i)
            atmp[i] = ap[i];
        for(size_t i = 0; i < nx; ++i)
            atmp[i + np] = ax[i];
    }
    else
    {   for(size_t i = 0; i < np; ++i)
            atmp[i + nx] = ap[i];
        for(size_t i = 0; i < nx; ++i)
            atmp[i] = ax[i];
    }
    //
    ay[0]  = fabs(atmp[0]);
    ay[1]  = acos(atmp[1]);
    ay[2]  = acosh(atmp[2] + 1.0);
    ay[3]  = asin(atmp[3]);
    ay[4]  = asinh(atmp[4]);
    ay[5]  = atan(atmp[5]);
    ay[6]  = atanh(atmp[6]);
    ay[7]  = cos(atmp[7]);
    ay[8]  = cosh(atmp[8]);
    ay[9]  = erf(atmp[9]);
    ay[10] = erfc(atmp[10]);
    ay[11] = exp(atmp[11]);
    ay[12] = expm1(atmp[12]);
    ay[13] = log(atmp[13]);
    ay[14] = log1p(atmp[14]);
    ay[15] = - atmp[15];
    ay[16] = sign(atmp[16]);
    ay[17] = sin(atmp[17]);
    ay[18] = sinh(atmp[18]);
    ay[19] = sqrt(atmp[19]);
    ay[20] = tan(atmp[20]);
    ay[21] = sin(atmp[21]);
    // Create function
    CppAD::ADFun<double> f(ax, ay);
    f.optimize();
    //
    // Evaluate function at x before
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y_before = f.Forward(0, x);
    //
    // Convert to Json and back again
    std::string json = f.to_json();
    // std::cout << graph;
    f.from_json(json);
    //
    // Evaluate function at x after
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y_after = f.Forward(0, x);
    //
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    for(size_t i = 0; i < ny; ++i)
        ok &= CppAD::NearEqual( y_before[i], y_after[i], eps99, eps99 );
    //
    return ok;
}

// ---------------------------------------------------------------------------
} // END_EMPTY_NAMESPACE


bool json_graph(void)
{   bool ok = true;
    ok     &= comp_op_dyn_dyn();
    ok     &= comp_op_var_var();
    ok     &= comp_op_dyn_var();
    ok     &= comp_op_var_dyn();
    ok     &= acosh_op();
    ok     &= log1p_op();
    ok     &= expm1_op();
    ok     &= erfc_op();
    ok     &= erf_op();
    ok     &= atanh_op();
    ok     &= asinh_op();
    ok     &= tan_op();
    ok     &= tanh_op();
    ok     &= sqrt_op();
    ok     &= sin_op();
    ok     &= sinh_op();
    ok     &= sign_op();
    ok     &= log_op();
    ok     &= exp_op();
    ok     &= cos_op();
    ok     &= cosh_op();
    ok     &= atan_op();
    ok     &= asin_op();
    ok     &= acos_op();
    ok     &= abs_op();
    ok     &= cexp_lt_variable();
    ok     &= cexp_lt_constant();
    ok     &= cexp_lt_dynamic();
    ok     &= atomic_both();
    ok     &= atomic_dynamic();
    ok     &= to_json_and_back();
    ok     &= binary_operators();
    ok     &= cumulative_sum();
    ok     &= unary(true);
    ok     &= unary(false);
    //
    return ok;
}
