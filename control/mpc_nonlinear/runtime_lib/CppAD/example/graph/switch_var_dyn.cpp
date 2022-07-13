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
$begin switch_var_dyn.cpp$$
$spell
    add
    Json
    dyn2var
    cpp
$$

$section Switching Between Variables and Dynamic Parameters: Example and Test$$

$head Function$$
For each $cref ADFun$$ object there is a corresponding function
$latex f(x, p)$$ where
$cref/x/independent/x/$$ is the vector of independent variables
and $icode p$$ is the vector of
independent $cref/dynamic/Independent/dynamic/$$ parameters.

$head Convert a Function to a Graph$$
The $cref to_graph$$ routine can be used to convert a $code ADFun$$
to a graph representation; see $cref cpp_ad_graph$$.

$head Convert a Graph to a Function$$
The $cref from_graph$$ routine can be used to convert a graph back
to a function. During this conversion, it is possible to change
dynamic parameters to variables and variables to dynamic parameters;
see $cref/dyn2var/from_graph/dyn2var/$$ and $icode var2dyn$$ in the
$code from_graph$$ documentation.
Note that many such conversions can be done
using the same $code cpp_ad_graph$$ object.

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool switch_var_dyn(void)
{   bool ok = true;
    using std::string;
    //
    // f(x_0, x_1, x_2) = y_0 = x_2 * ( x_0 + x_1 );
    CPPAD_TESTVECTOR( CppAD::AD<double> ) ax(3), ay(1);
    for(size_t j = 0; j < 3; ++j)
        ax[j] = CppAD::AD<double>(j);
    Independent(ax);
    ay[0] = ax[2] * ( ax[0] + ax[1] );
    CppAD::ADFun<double> f(ax, ay);
    ok &= f.Domain() == 3;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 0;
    //
    // set independent variables and parameters
    CPPAD_TESTVECTOR(double) p(0), x(3);
    x[0] = 2.0;
    x[1] = 3.0;
    x[2] = 4.0;
    //
    // compute y = f(x)
    f.new_dynamic(p);
    CPPAD_TESTVECTOR(double) y = f.Forward(0, x);
    //
    // check result
    ok &= y[0] == x[2] * ( x[0] + x[1] );
    // -----------------------------------------------------------------------
    //
    // C++ graph object
    CppAD::cpp_graph graph_obj;
    f.to_graph(graph_obj);
    //
    // change x[0]->p[0], x[1]->p[1], x[2]->x[0]
    CppAD::vector<bool> dyn2var(0), var2dyn(3);
    var2dyn[0] = true;
    var2dyn[1] = true;
    var2dyn[2] = false;
    f.from_graph(graph_obj, dyn2var, var2dyn);
    p.resize(2);
    x.resize(1);
    //
    ok &= f.Domain() == 1;
    ok &= f.Range() == 1;
    ok &= f.size_dyn_ind() == 2;
    //
    // set independent variables and parameters
    p[0] = 1.0;
    p[1] = 2.0;
    x[0] = 3.0;
    //
    // compute y = f(x, p)
    f.new_dynamic(p);
    y = f.Forward(0, x);
    //
    // check result
    ok &= y[0] == x[0] * ( p[0] + p[1] );
    //
    return ok;
}
// END C++
