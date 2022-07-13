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
$begin atomic_three_dynamic.cpp$$

$section Atomic Functions with Dynamic Parameters: Example and Test$$

$head Purpose$$
This example demonstrates using dynamic parameters with an
$cref atomic_three$$ function.

$head Function$$
For this example, the atomic function
$latex g : \B{R}^3 \rightarrow \B{R}^3$$ is defined by
$latex g_0 (x) = x_0 * x_ 0$$,
$latex g_1 (x) = x_0 * x_ 1$$,
$latex g_2 (x) = x_1 * x_ 2$$.

$nospell

$head Start Class Definition$$
$srccode%cpp% */
# include <cppad/cppad.hpp>  // CppAD include file
namespace {                  // start empty namespace
using CppAD::vector;         // abbreviate CppAD::vector using vector
// start definition of atomic derived class using atomic_three interface
class atomic_dynamic : public CppAD::atomic_three<double> {
/* %$$
$head Constructor$$
$srccode%cpp% */
public:
    // can use const char* name when calling this constructor
    atomic_dynamic(const std::string& name) : // can have more arguments
    CppAD::atomic_three<double>(name)             // inform base class of name
    { }

private:
/* %$$
$head for_type$$
$srccode%cpp% */
    // calculate type_y
    bool for_type(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        vector<CppAD::ad_type_enum>&        type_y      ) override
    {   assert( parameter_x.size() == type_x.size() );
        bool ok = type_x.size() == 3; // n
        ok     &= type_y.size() == 3; // m
        if( ! ok )
            return false;
        type_y[0] = type_x[0];
        type_y[1] = std::max( type_x[0], type_x[1] );
        type_y[2] = std::max( type_x[1], type_x[2] );
        return true;
    }
/* %$$
$head forward$$
$srccode%cpp% */
    // forward mode routine called by CppAD
    bool forward(
        const vector<double>&               parameter_x  ,
        const vector<CppAD::ad_type_enum>&  type_x       ,
        size_t                              need_y       ,
        size_t                              order_low    ,
        size_t                              order_up     ,
        const vector<double>&               taylor_x     ,
        vector<double>&                     taylor_y     ) override
    {
# ifndef NDEBUG
        size_t n = taylor_x.size() / (order_up + 1);
        size_t m = taylor_y.size() / (order_up + 1);
# endif
        assert( n == 3 );
        assert( m == 3 );
        assert( order_low <= order_up );

        // return flag
        bool ok = order_up == 0;
        if( ! ok )
            return ok;

        // Order zero forward mode.
        // This case must always be implemented
        if( need_y > size_t(CppAD::variable_enum) )
        {   // g_0 = x_0 * x_0
            taylor_y[0] = taylor_x[0] * taylor_x[0];
            // g_1 = x_0 * x_1
            taylor_y[1] = taylor_x[0] * taylor_x[1];
            // g_2 = x_1 * x_2
            taylor_y[2] = taylor_x[1] * taylor_x[2];
        }
        else
        {   // This uses need_y to reduce amount of computation.
            // It is probably faster, for this case, to ignore need_y.
            vector<CppAD::ad_type_enum> type_y( taylor_y.size() );
            for_type(taylor_x, type_x, type_y);
            // g_0 = x_0 * x_0
            if( size_t(type_y[0]) == need_y )
                taylor_y[0] = taylor_x[0] * taylor_x[0];
            // g_1 = x_0 * x_1
            if( size_t(type_y[1]) == need_y )
                taylor_y[1] = taylor_x[0] * taylor_x[1];
            // g_2 = x_1 * x_2
            if( size_t(type_y[2]) == need_y )
                taylor_y[2] = taylor_x[1] * taylor_x[2];
        }

        return ok;
    }
/* %$$
$head End Class Definition$$
$srccode%cpp% */
}; // End of atomic_dynamic class
}  // End empty namespace

/* %$$
$head Use Atomic Function$$
$srccode%cpp% */
bool dynamic(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
/* %$$
$subhead Constructor$$
$srccode%cpp% */
    // Create the atomic dynamic object corresponding to g(x)
    atomic_dynamic afun("atomic_dynamic");
/* %$$
$subhead Recording$$
$srccode%cpp% */
    // Create the function f(u) = g(c, p, u) for this example.
    //
    // constant parameter
    double c_0 = 2.0;
    //
    // indepndent dynamic parameter vector
    size_t np = 1;
    CPPAD_TESTVECTOR(double) p(np);
    CPPAD_TESTVECTOR( AD<double> ) ap(np);
    ap[0] = p[0] = 3.0;
    //
    // independent variable vector
    size_t  nu  = 1;
    double  u_0 = 0.5;
    CPPAD_TESTVECTOR( AD<double> ) au(nu);
    au[0] = u_0;

    // declare independent variables and start tape recording
    CppAD::Independent(au, ap);

    // range space vector
    size_t ny = 3;
    CPPAD_TESTVECTOR( AD<double> ) ay(ny);

    // call atomic function and store result in ay
    // y = ( c * c, c * p, p * x )
    CPPAD_TESTVECTOR( AD<double> ) ax(3);
    ax[0] = c_0;   // x_0
    ax[1] = ap[0]; // x_1
    ax[2] = au[0]; // x_2
    afun(ax, ay);

    // check type of result
    ok &= Constant( ay[0] );
    ok &= Dynamic(  ay[1] );
    ok &= Variable( ay[2] );

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (au, ay);  // f(u) = (c * c, c * p, p * u)
/* %$$
$subhead forward$$
$srccode%cpp% */
    // check function value
    double check = c_0 * c_0;
    ok &= NearEqual( Value(ay[0]) , check,  eps, eps);
    check = c_0 * p[0];
    ok &= NearEqual( Value(ay[1]) , check,  eps, eps);
    check = p[0] * u_0;
    ok &= NearEqual( Value(ay[2]) , check,  eps, eps);

    // check zero order forward mode
    size_t q;
    CPPAD_TESTVECTOR( double ) u_q(nu), y_q(ny);
    q      = 0;
    u_q[0] = u_0;
    y_q    = f.Forward(q, u_q);
    check = c_0 * c_0;
    ok    &= NearEqual(y_q[0] , check,  eps, eps);
    check = c_0 * p[0];
    ok    &= NearEqual(y_q[1] , check,  eps, eps);
    check = p[0] * u_0;
    ok    &= NearEqual(y_q[2] , check,  eps, eps);

    // set new value for dynamic parameters
    p[0]   = 2.0 * p[0];
    f.new_dynamic(p);
    y_q    = f.Forward(q, u_q);
    check = c_0 * c_0;
    ok    &= NearEqual(y_q[0] , check,  eps, eps);
    check = c_0 * p[0];
    ok    &= NearEqual(y_q[1] , check,  eps, eps);
    check = p[0] * u_0;
    ok    &= NearEqual(y_q[2] , check,  eps, eps);

/* %$$
$subhead Return Test Result$$
$srccode%cpp% */
    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
