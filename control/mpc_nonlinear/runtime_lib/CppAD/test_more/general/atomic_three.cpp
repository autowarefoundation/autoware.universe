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
g_0 (x) = x_0 * x_0
g_1 (x) = x_0 * x_1
g_2 (x) = x_1 * x_2
g_3 (x) = x_2 * x_2

*/
# include <cppad/cppad.hpp>  // CppAD include file
namespace {                  // start empty namespace
using CppAD::vector;         // abbreviate CppAD::vector using vector

// ============================================================================
// Testing dynamic parameters in atomic_three fucntions.
// ============================================================================
class dynamic_optimize : public CppAD::atomic_three<double> {
public:
    // can use const char* name when calling this constructor
    dynamic_optimize(const std::string& name) : // can have more arguments
    CppAD::atomic_three<double>(name)          // inform base class of name
    { }

private:
    // calculate type_y
    virtual bool for_type(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        vector<CppAD::ad_type_enum>&        type_y      )
    {   assert( parameter_x.size() == type_x.size() );
        bool ok = type_x.size() == 3; // n
        ok     &= type_y.size() == 4; // m
        if( ! ok )
            return false;
        type_y[0] = type_x[0];
        type_y[1] = std::max( type_x[0], type_x[1] );
        type_y[2] = std::max( type_x[1], type_x[2] );
        type_y[3] = type_x[2];
        return true;
    }
    // calculate depend_x
    virtual bool rev_depend(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        vector<bool>&                       depend_x    ,
        const vector<bool>&                 depend_y    )
    {   assert( parameter_x.size() == depend_x.size() );
        bool ok = depend_x.size() == 3; // n
        ok     &= depend_y.size() == 4; // m
        if( ! ok )
            return false;
        depend_x[0] = depend_y[0] | depend_y[1];
        depend_x[1] = depend_y[1] | depend_y[2];
        depend_x[2] = depend_y[2] | depend_y[3];
        return true;
    }
    virtual bool forward(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        size_t                              need_y    ,
        size_t                              order_low ,
        size_t                              order_up  ,
        const vector<double>&               taylor_x  ,
        vector<double>&                     taylor_y
    )
    {
# ifndef NDEBUG
        size_t n = taylor_x.size() / (order_up + 1);
        size_t m = taylor_y.size() / (order_up + 1);
# endif
        assert( n == 3 );
        assert( m == 4 );
        assert( order_low <= order_up );

        // return flag
        bool ok = order_up == 0;
        if( ! ok )
            return ok;

        // g_0 = x_0 * x_0
        taylor_y[0] = taylor_x[0] * taylor_x[0];
        // g_1 = x_0 * x_1
        taylor_y[1] = taylor_x[0] * taylor_x[1];
        // g_2 = x_1 * x_2
        taylor_y[2] = taylor_x[1] * taylor_x[2];
        // g_3 = x_2 * x_2
        taylor_y[3] = taylor_x[2] * taylor_x[2];

        return ok;
    }
}; // End of dynamic_optimize class

// ---------------------------------------------------------------------------
bool optimize_dynamic_one(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    dynamic_optimize afun("dynamic_optimize");
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

    // create a dynamic parameter that is not used
    AD<double> ar = 2.0 * ap[0];

    // call atomic function and store result in ay
    CPPAD_TESTVECTOR( AD<double> ) ax(3), av(4);
    ax[0] = c_0;   // x_0 = c
    ax[1] = ap[0]; // x_1 = p
    ax[2] = au[0]; // x_2 = u
    afun(ax, av);

    // check type of result
    ok &= Constant( av[0] ); // v_0 = c * c
    ok &= Dynamic(  av[1] ); // v_1 = c * p
    ok &= Variable( av[2] ); // v_2 = p * u
    ok &= Variable( av[3] ); // v_3 = u * u

    // range space vector
    size_t ny = 3;
    CPPAD_TESTVECTOR( AD<double> ) ay(ny);
    for(size_t i = 0; i < ny; ++i)
        ay[i] = av[i];

    // create f: u -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (au, ay);  // f(u) = (c * c, c * p, p * u)

    // sequence properties
    ok &= f.size_dyn_ind() == 1; // p
    ok &= f.size_dyn_par() == 3; // p, r, c * p
    // Three constant parameters: nan, c, c * c
    ok &= f.size_par() == 6;
    // Normal variables: u, p * u, u * u
    // Extra variables: phanton at index 0, y[0], y[1]
    ok &= f.size_var() == 6;

    // optimize
    f.optimize();

    // sequence properties
    ok &= f.size_dyn_ind() == 1; // p
    ok &= f.size_dyn_par() == 2; // c * p

    // Four constant parameters: nan, zero, c, c * c
    ok &= f.size_par() == 6;

    // Normal variables: u, p * u
    // Extra variables: phanton at index 0, y[0], y[1]
    ok &= f.size_var() == 5;

    // check
    double check;

    // check zero order forward mode
    size_t q;
    CPPAD_TESTVECTOR( double ) u_q(nu), y_q(ny);
    q      = 0;
    u_q[0] = u_0;
    y_q    = f.Forward(q, u_q);
    check  = c_0 * c_0;
    ok    &= NearEqual(y_q[0] , check,  eps, eps);
    check = c_0 * p[0];
    ok    &= NearEqual(y_q[1] , check,  eps, eps);
    check = p[0] * u_0;
    ok    &= NearEqual(y_q[2] , check,  eps, eps);

    // set new value for dynamic parameters
    p[0]  = 2.0 * p[0];
    f.new_dynamic(p);
    y_q   = f.Forward(q, u_q);
    check = c_0 * c_0;
    ok    &= NearEqual(y_q[0] , check,  eps, eps);
    check = c_0 * p[0];
    ok    &= NearEqual(y_q[1] , check,  eps, eps);
    check = p[0] * u_0;
    ok    &= NearEqual(y_q[2] , check,  eps, eps);

    return ok;
}
// ---------------------------------------------------------------------------
bool optimize_dynamic_two(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    dynamic_optimize afun("dynamic_optimize");
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

    // create a dynamic parameter that is used by atomic function
    // but not needed to compute f(u)
    AD<double> ar = 2.0 * ap[0];

    // call atomic function and store result in ay
    CPPAD_TESTVECTOR( AD<double> ) ax(3), av(4);
    ax[0] = au[0]; // x_0 = u
    ax[1] = ap[0]; // x_1 = p
    ax[2] = ar;    // x_2 = r
    afun(ax, av);

    // check type of result
    ok &= Variable( av[0] );  // v_0 = u * u , used
    ok &= Variable( av[1] );  // v_1 = u * p , used
    ok &= Dynamic( av[2] );   // v_2 = r * p , not used
    ok &= Dynamic( av[3] );   // v_3 = r * r , not used

    // range space vector
    size_t ny = 2;
    CPPAD_TESTVECTOR( AD<double> ) ay(ny);
    for(size_t i = 0; i < ny; ++i)
        ay[i] = av[i];

    // create f: u -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (au, ay);  // f(u) = (u * u, u * p)

    // sequence properties
    ok &= f.size_dyn_ind() == 1; // p
    ok &= f.size_dyn_par() == 4; // p, r, r * p, r * r
    // Two constant parameters: nan, 2.0 in computation of r
    ok &= f.size_par() == 6;


    // optimize
    f.optimize();

    // sequence properties
    ok &= f.size_dyn_ind() == 1; // p
    ok &= f.size_dyn_par() == 1; // p
    // Two constant paramete: nan, zero
    ok &= f.size_par() == 3;

    // check
    double check;

    // check zero order forward mode
    size_t q;
    CPPAD_TESTVECTOR( double ) u_q(nu), y_q(ny);
    q      = 0;
    u_q[0] = u_0;
    y_q    = f.Forward(q, u_q);
    check  = u_0 * u_0;
    ok    &= NearEqual(y_q[0] , check,  eps, eps);
    check = u_0 * p[0];
    ok    &= NearEqual(y_q[1] , check,  eps, eps);

    // set new value for dynamic parameters
    p[0]  = 2.0 * p[0];
    f.new_dynamic(p);
    y_q   = f.Forward(q, u_q);
    check = u_0 * u_0;
    ok    &= NearEqual(y_q[0] , check,  eps, eps);
    check = u_0 * p[0];
    ok    &= NearEqual(y_q[1] , check,  eps, eps);

    return ok;
}
// ---------------------------------------------------------------------------
bool optimize_dynamic_three(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    dynamic_optimize afun("dynamic_optimize");
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

    // create a dynamic parameter that is used by atomic function
    // but not needed to compute f(u)
    AD<double> ar = 2.0 * ap[0];

    // call atomic function and store result in ay
    CPPAD_TESTVECTOR( AD<double> ) ax(3), av(4);
    ax[0] = au[0]; // x_0 = u
    ax[1] = ar;    // x_1 = r
    ax[2] = ap[0]; // x_2 = p
    afun(ax, av);

    // check type of result
    ok &= Variable( av[0] );  // v_0 = u * u , used
    ok &= Variable( av[1] );  // v_1 = u * r , used
    ok &= Dynamic( av[2] );   // v_2 = r * p , not used
    ok &= Dynamic( av[3] );   // v_3 = p * p , not used

    // range space vector
    size_t ny = 2;
    CPPAD_TESTVECTOR( AD<double> ) ay(ny);
    for(size_t i = 0; i < ny; ++i)
        ay[i] = av[i];

    // create f: u -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (au, ay);  // f(u) = (u * u, u * p)

    // sequence properties
    ok &= f.size_dyn_ind() == 1; // p
    ok &= f.size_dyn_par() == 4; // p, r, r * p, p * p
    // Two constant parameters: nan, 2.0 in computation of r
    ok &= f.size_par() == 6;


    // optimize
    f.optimize();

    // sequence properties
    ok &= f.size_dyn_ind() == 1; // p
    ok &= f.size_dyn_par() == 2; // p, r
    // Three constant parameters: nan, zero, 2.0 in computation of r
    ok &= f.size_par() == 5;

    // check
    double check;

    // check zero order forward mode
    double r = 2.0 * p[0];
    size_t q;
    CPPAD_TESTVECTOR( double ) u_q(nu), y_q(ny);
    q      = 0;
    u_q[0] = u_0;
    y_q    = f.Forward(q, u_q);
    check  = u_0 * u_0;
    ok    &= NearEqual(y_q[0] , check,  eps, eps);
    check = u_0 * r;
    ok    &= NearEqual(y_q[1] , check,  eps, eps);

    // set new value for dynamic parameters
    p[0]  = 2.0 * p[0];
    r     = 2.0 * p[0];
    f.new_dynamic(p);
    y_q   = f.Forward(q, u_q);
    check = u_0 * u_0;
    ok    &= NearEqual(y_q[0] , check,  eps, eps);
    check = u_0 * r;
    ok    &= NearEqual(y_q[1] , check,  eps, eps);

    return ok;
}
// ============================================================================
// Testing Varialbes that get removed
// ============================================================================
class variable_optimize : public CppAD::atomic_three<double> {
public:
    // can use const char* name when calling this constructor
    variable_optimize(const std::string& name) : // can have more arguments
    CppAD::atomic_three<double>(name)          // inform base class of name
    { }

private:
    // calculate type_y
    virtual bool for_type(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        vector<CppAD::ad_type_enum>&        type_y      )
    {   assert( parameter_x.size() == type_x.size() );
        bool ok = type_x.size() == 2; // n
        ok     &= type_y.size() == 2; // m
        if( ! ok )
            return false;
        type_y[0] = type_x[0];
        type_y[1] = std::max( type_x[0], type_x[1] );
        return true;
    }
    // calculate depend_x
    virtual bool rev_depend(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        vector<bool>&                       depend_x    ,
        const vector<bool>&                 depend_y    )
    {   assert( parameter_x.size() == depend_x.size() );
        bool ok = depend_x.size() == 2; // n
        ok     &= depend_y.size() == 2; // m
        if( ! ok )
            return false;
        depend_x[0] = depend_y[0] | depend_y[1];
        depend_x[1] = depend_y[1];
        return true;
    }
    virtual bool forward(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        size_t                              need_y    ,
        size_t                              order_low ,
        size_t                              order_up  ,
        const vector<double>&               taylor_x  ,
        vector<double>&                     taylor_y
    )
    {
# ifndef NDEBUG
        size_t n = taylor_x.size() / (order_up + 1);
        size_t m = taylor_y.size() / (order_up + 1);
# endif
        assert( n == 2 );
        assert( m == 2 );
        assert( order_low <= order_up );

        // return flag
        bool ok = order_up == 0;
        if( ! ok )
            return ok;

        // g_0 = exp( x_0 )
        taylor_y[0] = std::exp( taylor_x[0] );
        // g_1 = exp( x_0 * x_1 )
        taylor_y[1] = std::exp( taylor_x[0] * taylor_x[1] );

        return ok;
    }
    virtual bool reverse(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        size_t                              order_up    ,
        const vector<double>&               taylor_x    ,
        const vector<double>&               taylor_y    ,
        vector<double>&                     partial_x   ,
        const vector<double>&               partial_y
    )
    {
        size_t q1 = order_up + 1;
        size_t n  = taylor_x.size() / q1;
# ifndef NDEBUG
        size_t m  = taylor_y.size() / q1;
# endif
        assert( n == 2 );
        assert( m == 2 );

        // return flag
        bool ok = order_up == 0;
        if( ! ok )
            return ok;

        // initialize summation as zero
        for(size_t j = 0; j < n; ++j)
            partial_x[j] = 0.0;

        // g_0  = exp( x_0 )
        partial_x[0] += partial_y[0] * taylor_y[0];
        // g_1 = exp( x_0 * x_1 )
        partial_x[0] += partial_y[1] * taylor_y[1] * taylor_x[1];
        partial_x[1] += partial_y[1] * taylor_y[1] * taylor_x[0];
        //
        return ok;
    }
}; // End of variable_optimize class
// ---------------------------------------------------------------------------
bool optimize_variable_one(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    variable_optimize afun("variable_optimize");
    //
    // independent variable vector
    size_t  nu  = 2;
    CPPAD_TESTVECTOR( AD<double> ) au(nu);
    for(size_t j = 0; j < nu; ++j)
        au[j] = double(j + 1);

    // declare independent variables and start tape recording
    CppAD::Independent(au);

    // call atomic function and store result in ay
    CPPAD_TESTVECTOR( AD<double> ) ax(nu), av(nu);
    for(size_t j = 0; j < nu; ++j)
        ax[j] = au[j] / 2.0; // x = u / 2
    afun(ax, av);

    // check type of result
    for(size_t j = 0; j < nu; ++j)
        ok &= Variable( av[j] );

    // range space vector
    size_t ny = 1;
    CPPAD_TESTVECTOR( AD<double> ) ay(ny);
    //
    // only the first component of av affects the function value
    ay[0] = av[0];

    // create f: u -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (au, ay);  // f(u) = exp( u[0] / 2 )

    // optimize
    f.optimize();

    // check
    double check;

    // check zero order forward mode
    CPPAD_TESTVECTOR( double ) u(nu), y(ny);
    for(size_t j = 0; j < nu; ++j)
        u[j] = double(j + 1) / double(nu);
    y    = f.Forward(0, u);
    check  = std::exp( u[0] / 2.0 );
    ok    &= NearEqual(y[0] , check,  eps, eps);

    // Check first order reverse mode. This test would vaile when
    // nan was used for argument and function values that were optimized out
    // because they were not used.
    CPPAD_TESTVECTOR( double ) w(ny), dw(nu);
    w[0] = 1.0;
    dw  = f.Reverse(1, w);
    check  = std::exp( u[0] / 2.0 ) / 2.0;
    ok    &= NearEqual(dw[0] , check,  eps, eps);
    check  = 0.0;
    ok    &= NearEqual(dw[1] , check,  eps, eps);
    //
    return ok;
}
}  // End empty namespace

bool atomic_three(void)
{   bool ok = true;
    ok     &= optimize_dynamic_one();
    ok     &= optimize_dynamic_two();
    ok     &= optimize_dynamic_three();
    ok     &= optimize_variable_one();
    return ok;
}
