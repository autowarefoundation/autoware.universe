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
$begin json_get_started.cpp$$
$spell
    Json
    CppAD
    op
    vec
    ind
    arg
    var
$$

$section Json Get Started: Example and Test$$

$head Notation$$
$table
Notation
    $cnext Description                     $cnext size $rnext
$cref/p/json_ad_graph/Node Indices/p/$$
    $cnext vector of dynamic parameters    $cnext 1    $rnext
$cref/x/json_ad_graph/Node Indices/x/$$
    $cnext vector of independent variables $cnext 1    $rnext
$cref/c/json_ad_graph/Node Indices/c/$$
    $cnext vector of constants             $cnext 1    $rnext
y   $cnext vector of dependent variables   $cnext 1
$tend

$head Node Table$$
$table
index $cnext  value  $rnext
1     $cnext  p[0]   $rnext
2     $cnext  x[0]   $rnext
3     $cnext  c[0]   $rnext
4     $cnext  sin(p[0])   $rnext
5     $cnext  sin(x[0])   $rnext
6     $cnext  sin(c[0])   $rnext
7     $cnext  sin(p[0]) + sin(x[0]) + sin(c[0])   $rnext
y[0]  $cnext  sin(p[0]) + sin(x[0]) + sin(c[0])
$tend

$head Include$$
Include the CppAD core functions:
$srccode%cpp% */
# include <cppad/cppad.hpp>
/* %$$

$head Syntax$$
$icode%ok% = get_started()%$$
$srccode%cpp% */
bool get_started(void)
{
/* %$$

$head Setup$$
$srccode%cpp% */
    bool ok = true;
    using CppAD::vector;
    using CppAD::AD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
/* %$$
$head Function$$

$subhead Begin Function$$
See $cref/function/json_ad_graph/AD Graph/function/$$:
$srccode%cpp% */
    std::string json =
        "{\n"
        "   'function_name'  : 'get_started example',\n"
/* %$$
$subhead Begin op_define_vec$$
see $cref/op_define_vec/json_ad_graph/op_define_vec/$$:
$srccode%cpp% */
        "   'op_define_vec'  : [ 2, [\n"
/* %$$
$subhead Define Unary$$
see $cref/unary operators/json_graph_op/Unary Operators/$$:
$srccode%cpp% */
        "       { 'op_code':1, 'name':'sin', 'n_arg':1 } ,\n"
/* %$$
$subhead Define Sum$$
see $cref/sum/json_graph_op/sum/$$:
$srccode%cpp% */
        "       { 'op_code':2, 'name':'sum'            } ]\n"
/* %$$
$subhead End op_define_vec$$
$srccode%cpp% */
        "   ],\n"
/* %$$
$subhead n_dynamic_ind$$
see $cref/n_dynamic_ind/json_ad_graph/dynamic_ind_vec/n_dynamic_ind/$$:
$srccode%cpp% */
        "   'n_dynamic_ind'  : 1,\n"
/* %$$
$subhead n_variable_ind$$
see $cref/n_variable_ind/json_ad_graph/variable_ind_vec/n_variable_ind/$$:
$srccode%cpp% */
        "   'n_variable_ind' : 1,\n"
/* %$$
$subhead constant_vec$$
see $cref/constant_vec/json_ad_graph/constant_vec/$$:
$srccode%cpp% */
        "   'constant_vec'   : [ 1, [ -0.1 ] ],\n" // c[0]
/* %$$
$subhead Begin op_usage_vec$$
see $cref/op_usage_vec/json_ad_graph/op_usage_vec/$$:
$srccode%cpp% */
        "   'op_usage_vec'   : [ 4, [\n"
/* %$$
$subhead op_usage$$
see op_usage with
$cref/n_arg in definition/json_ad_graph/op_usage/n_arg In Definition/$$:
$srccode%cpp% */
        "       [ 1, 1]                ,\n" // sin(p[0])
        "       [ 1, 2]                ,\n" // sin(x[0])
        "       [ 1, 3]                ,\n" // sin(c[0])
/* %$$
see op_usage with
$cref/n_arg not in definition
    /json_ad_graph
    /op_usage
    /n_arg Not In Definition
/$$:

$srccode%cpp% */
        "       [ 2, 1, 3, [4, 5, 6] ] ]\n" // sin(p[0])+sin(x[0])+sin(c[0])
/* %$$
$subhead End op_usage_vec$$
$srccode%cpp% */
        "   ],\n"
/* %$$
$subhead dependent_vec$$
see $cref/dependent_var/json_ad_graph/dependent_vec/$$
$srccode%cpp% */
        "   'dependent_vec' : [ 1, [7] ] \n"
/* %$$
$subhead End Function$$
$srccode%cpp% */
        "}\n";
/* %$$
$head Convert Single to Double Quotes$$
$srccode%cpp% */
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
/* %$$
$head double f(x, p)$$
$srccode%cpp% */
    CppAD::ADFun<double> f;
    f.from_json(json);
/* %$$
$head Check f(x, p)$$
$srccode%cpp% */
    vector<double> c(1), p(1), x(1), y(1);
    c[0] = -0.1; // must match value in graph
    p[0] = 0.2;  // can be any value
    x[0] = 0.3;  // can be any value
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // f(x, p) = sin(p_0) + sin(x_0) + sin(c_0)
    double check = std::sin(p[0]) + std::sin(x[0]) + std::sin(c[0]);
    ok &= CppAD::NearEqual(y[0], check, eps99, eps99);

/* %$$
$head AD<double> f(x, p)$$
$srccode%cpp% */
    CppAD::ADFun< AD<double>, double > af( f.base2ad() );
/* %$$
$head Evaluate Derivative$$
$srccode%cpp% */
    // set independent variables and parameters
    vector< AD<double> > ap(1), ax(1);
    ap[0] = 0.2;
    ax[0] = 0.3;
    //
    // record computation of z = d/dx f(x, p)
    CppAD::Independent(ax, ap);
    af.new_dynamic(ap);
    vector< AD<double> > az = af.Jacobian(ax);
/* %$$
$head double g(x, p) = d/dx f(x, p)$$
$srccode%cpp% */
    CppAD::ADFun<double> g(ax, az);
/* %$$
$head Convert to Json and Back$$
$srccode%cpp% */
    json = g.to_json();
    g.from_json(json);
/* %$$
$head Check g(x, p)$$
$srccode%cpp% */
    c[0] = -0.1; // must match value in graph
    p[0] = 0.3;  // can be any value
    x[0] = 0.4;  // can be any value
    //
    // compute z = g(x, p)
    g.new_dynamic(p);
    vector<double> z = g.Forward(0, x);
    //
    // g(x, p) = d/dx f(x, p) = d/dx sin(x) = cos(x)
    check = std::cos(x[0]);
    ok &= CppAD::NearEqual(z[0], check, eps99, eps99);
    //
    return ok;
}
/* %$$
$end
*/
