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
$begin atomic_three_get_started.cpp$$

$section Getting Started with Atomic Functions: Example and Test$$

$head Purpose$$
This example demonstrates the minimal amount of information
necessary for a $cref atomic_three$$ function.

$nospell

$head Start Class Definition$$
$srccode%cpp% */
# include <cppad/cppad.hpp>  // CppAD include file
namespace {                  // start empty namespace
using CppAD::vector;         // abbreviate CppAD::vector using vector
// start definition of atomic derived class using atomic_three interface
class atomic_get_started : public CppAD::atomic_three<double> {
/* %$$
$head Constructor$$
$srccode%cpp% */
public:
    // can use const char* name when calling this constructor
    atomic_get_started(const std::string& name) : // can have more arguments
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
        bool ok = type_x.size() == 1; // n
        ok     &= type_y.size() == 1; // m
        if( ! ok )
            return false;
        type_y[0] = type_x[0];
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
        assert( n == 1 );
        assert( m == 1 );
        assert( order_low <= order_up );

        // return flag
        bool ok = order_up == 0;
        if( ! ok )
            return ok;

        // Order zero forward mode.
        // This case must always be implemented
        // y^0 = g( x^0 ) = 1 / x^0
        taylor_y[0] = 1. / taylor_x[0];
        //
        return ok;
    }
/* %$$
$head End Class Definition$$
$srccode%cpp% */
}; // End of atomic_get_started class
}  // End empty namespace

/* %$$
$head Use Atomic Function$$
$srccode%cpp% */
bool get_started(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
/* %$$
$subhead Constructor$$
$srccode%cpp% */
    // Create the atomic get_started object corresponding to g(x)
    atomic_get_started afun("atomic_get_started");
/* %$$
$subhead Recording$$
$srccode%cpp% */
    // Create the function f(x) which is eqaul to g(x) for this example.
    //
    // domain space vector
    size_t  n  = 1;
    double  x0 = 0.5;
    CPPAD_TESTVECTOR( AD<double> ) ax(n);
    ax[0]     = x0;

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR( AD<double> ) ay(m);

    // call atomic function and store result in au[0]
    // u = 1 / x
    CPPAD_TESTVECTOR( AD<double> ) au(m);
    afun(ax, au);

    // now use AD division to invert to invert the operation
    ay[0] = 1.0 / au[0]; // y = 1 / u = x

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (ax, ay);  // f(x) = x
/* %$$
$subhead forward$$
$srccode%cpp% */
    // check function value
    double check = x0;
    ok &= NearEqual( Value(ay[0]) , check,  eps, eps);

    // check zero order forward mode
    size_t q;
    CPPAD_TESTVECTOR( double ) x_q(n), y_q(m);
    q      = 0;
    x_q[0] = x0;
    y_q    = f.Forward(q, x_q);
    ok    &= NearEqual(y_q[0] , check,  eps, eps);

/* %$$
$subhead Return Test Result$$
$srccode%cpp% */
    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
