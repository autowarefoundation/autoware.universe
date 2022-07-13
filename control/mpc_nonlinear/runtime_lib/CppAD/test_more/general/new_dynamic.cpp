/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <limits>
# include <cppad/cppad.hpp>

namespace { // BEGIN_EMPTY_NAMESPACE
// ----------------------------------------------------------------------------
bool vecad(void)
{   bool ok = true;
    using CppAD::AD;

    // dynamic parameter vector
    size_t np = 2;
    size_t nx = 1;
    size_t ny = 2;
    CPPAD_TESTVECTOR(AD<double>) ap(np), ax(nx), ay(ny);
    ap[0] = 0.0;
    ap[1] = 1.0;
    ax[0] = 2.0;

    // declare independent variables, dynamic parameters, start recording
    CppAD::Independent(ax, ap);

    // Create a VecAD object
    size_t nv = 2;
    CppAD::VecAD<double> av(nv);
    // ap[0] is either 0 or 1
    av[ ap[0] ]       = ap[1];
    av[ 1.0 - ap[0] ] = 2.0 * ap[1];

    // create f: x -> y and stop tape recording
    CppAD::AD<double> zero(0.0), one(1.0);
    ay[0] = av[zero];
    ay[1] = av[one];
    CppAD::ADFun<double> f(ax, ay);

    // zero order forward mode with p[0] = 0
    CPPAD_TESTVECTOR(double) p(np), x(nx), y(ny);
    p[0] = 0.0;
    p[1] = 1.0;
    x[0] = 2.0;
    f.new_dynamic(p);
    y   = f.Forward(0, x);
    ok &= y[0] == p[1];
    ok &= y[1] == 2.0 * p[1];

    // zero order forward mode with p[0] = 1
    p[0] = 1.0;
    p[1] = 2.0;
    x[0] = 3.0;
    f.new_dynamic(p);
    y   = f.Forward(0, x);
    ok &= y[0] == 2.0 * p[1];
    ok &= y[1] == p[1];
    CPPAD_TESTVECTOR(double) dx(nx), dy(ny);
    dx[0] = 1.0;
    dy    = f.Forward(1, dx);
    ok   &= dy[0] == 0.0;
    ok   &= dy[1] == 0.0;

    return ok;
}
// ----------------------------------------------------------------------------
bool operator_with_variable(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    using CppAD::azmul;
    using CppAD::CondExpLt;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    size_t n   = 11;

    // dynamic parameter vector
    CPPAD_TESTVECTOR(AD<double>) adynamic(n);
    for(size_t j = 0; j < n; ++j)
        adynamic[j] = 2.0;

    // domain space vector
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    for(size_t j = 0; j < n; ++j)
        ax[j] = AD<double>(j + 1);

    // declare independent variables, dynamic parammeters, starting recording
    CppAD::Independent(ax, adynamic);

    // range space vector
    CPPAD_TESTVECTOR(AD<double>) ay(n);
    size_t k = 0;
    ay[k] = -1.0*double(k+1)*(adynamic[k] + ax[k]) * (ax[k] + adynamic[k]);
    ++k;
    ay[k] = -1.0*double(k+1)*(adynamic[k] - ax[k]) * (ax[k] - adynamic[k]);
    ++k;
    ay[k] = -1.0*double(k+1)*(adynamic[k] * ax[k]) + (ax[k] * adynamic[k]);
    ++k;
    ay[k] = -1.0*double(k+1)*(adynamic[k] / ax[k]) + (ax[k] / adynamic[k]);
    ++k;
    ay[k]  = ax[k];
    ay[k] += adynamic[k];
    ++k;
    ay[k]  = adynamic[k];
    ay[k] -= ax[k];
    ++k;
    ay[k]  = ax[k];
    ay[k] *= adynamic[k];
    ++k;
    ay[k]  = adynamic[k];
    ay[k] /= ax[k];
    ++k;
    ay[k]  = pow(ax[k], adynamic[k]) + pow(adynamic[k], ax[k]);
    ++k;
    ay[k]  = azmul(ax[k], adynamic[k]) + azmul(adynamic[k], ax[k]);
    ++k;
    ay[k]  = CondExpLt(ax[k], adynamic[k], ax[k], adynamic[k]);
    ++k;
    ok &= size_t(k) == n;

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // zero order forward mode
    CPPAD_TESTVECTOR(double) x(n), y(n);
    for(size_t j = 0; j < n; ++j)
        x[j] = double(j + 1);
    y    = f.Forward(0, x);
    double check;
    k = 0;
    check  = ( Value(adynamic[k]) + x[k] ) * ( x[k] + Value(adynamic[k]) );
    check *= -1.0*double(k+1);
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = ( Value(adynamic[k]) - x[k] ) * ( x[k] - Value(adynamic[k]) );
    check *= -1.0*double(k+1);
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = -1.0*double(k+1)*( Value(adynamic[k]) * x[k] );
    check += ( x[k] * Value(adynamic[k]) );
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = -1.0*double(k+1)*( Value(adynamic[k]) / x[k] );
    check += ( x[k] / Value(adynamic[k]) );
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = x[k] + Value(adynamic[k]);
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = Value(adynamic[k]) - x[k];
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = x[k] * Value(adynamic[k]);
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = Value(adynamic[k]) / x[k];
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = std::pow(x[k], Value(adynamic[k]));
    check += std::pow(Value(adynamic[k]), x[k]);
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = azmul(x[k], Value(adynamic[k])) + azmul(Value(adynamic[k]), x[k]);
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = CondExpLt( x[k], Value(adynamic[k]), x[k], Value(adynamic[k]) );
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    ok &= size_t(k) == n;

    // change the dynamic parameter values
    CPPAD_TESTVECTOR(double) dynamic(n);
    for(size_t j = 0; j < n; j++)
        dynamic[j] = double(2 * j + 1);
    f.new_dynamic(dynamic);
    //
    y = f.Forward(0, x);
    k     = 0;
    check = -1.0*double(k+1)*( dynamic[k] + x[k] ) * ( x[k] + dynamic[k] );
    ok   &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check = -1.0*double(k+1)*( dynamic[k] - x[k] ) * ( x[k] - dynamic[k] );
    ok   &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check = -1.0*double(k+1)*( dynamic[k] * x[k] ) + ( x[k] * dynamic[k] );
    ok   &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check = -1.0*double(k+1)*( dynamic[k] / x[k] ) + ( x[k] / dynamic[k] );
    ok   &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check = x[k] + dynamic[k];
    ok   &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check = dynamic[k] - x[k];
    ok   &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check = x[k] * dynamic[k];
    ok   &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check = dynamic[k] / x[k];
    ok   &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = std::pow(x[k], dynamic[k]);
    check += std::pow(dynamic[k], x[k]);
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = azmul(x[k], dynamic[k]) + CppAD::azmul(dynamic[k], x[k]);
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    check  = CondExpLt(x[k], dynamic[k], x[k], dynamic[k]);
    ok    &= NearEqual(y[k] , check, eps99, eps99);
    ++k;
    ok &= size_t(k) == n;
    //
    return ok;
}
// ----------------------------------------------------------------------------
bool dynamic_operator(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    using CppAD::azmul;
    using CppAD::sign;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent dynamic parameter vector
    size_t nd  = 1;
    CPPAD_TESTVECTOR(AD<double>) adynamic(nd);
    adynamic[0] = 0.5;

    // domain space vector
    size_t nx = 1;
    CPPAD_TESTVECTOR(AD<double>) ax(nx);
    ax[0] = 0.25;

    // declare independent variables, dynamic parammeters, starting recording
    CppAD::Independent(ax, adynamic);

    // range space vector
    size_t ny = 27;
    ny += 6;
    CPPAD_TESTVECTOR(AD<double>) ay(ny);
    size_t k = 0;
    // ----------------------------------------------------------
    // 98 standard math
    ay[k] = acos(adynamic[0]);
    ++k;
    ay[k] = asin(adynamic[0]);
    ++k;
    ay[k] = atan(adynamic[0]);
    ++k;
    ay[k] = cos(adynamic[0]);
    ++k;
    ay[k] = cosh(adynamic[0]);
    ++k;
    ay[k] = exp(adynamic[0]);
    ++k;
    ay[k] = acos(adynamic[0]);
    ++k;
    ay[k] = log(adynamic[0]);
    ++k;
    ay[k] = sin(adynamic[0]);
    ++k;
    ay[k] = sinh(adynamic[0]);
    ++k;
    ay[k] = sqrt(adynamic[0]);
    ++k;
    ay[k] = tan(adynamic[0]);
    ++k;
    ay[k] = tanh(adynamic[0]);
    ++k;
    // ----------------------------------------------------------
    // 2011 standard math
    ay[k] = asinh(adynamic[0]);
    ++k;
    ay[k] = acosh(adynamic[0] + 1.0);
    ++k;
    ay[k] = atanh(adynamic[0]);
    ++k;
    ay[k] = expm1(adynamic[0]);
    ++k;
    ay[k] = erf(adynamic[0]);
    ++k;
    ay[k] = log1p(adynamic[0]);
    ++k;
    // ----------------------------------------------------------
    // binary
    ay[k]  = 2.0 + adynamic[0];
    ++k;
    ay[k]  = 2.0;
    ay[k] += adynamic[0];  // constant += dynamic
    ++k;
    ay[k]  = adynamic[0] / 2.0;
    ++k;
    ay[k]  = adynamic[0];
    ay[k] /= 2.0;          // dynamic /= constant
    ++k;
    ay[k]  = 2.0 * adynamic[0];
    ++k;
    ay[k]  = 2.0;
    ay[k] *= adynamic[0];  // constant *= dynamic
    ++k;
    ay[k]  = adynamic[0] - 2.0;
    ++k;
    ay[k]  = adynamic[0];
    ay[k] -= 2.0;          // dynamic -= constant
    ++k;
    // ----------------------------------------------------------
    // other
    ay[k] = - adynamic[0];
    ++k;
    ay[k] = fabs(adynamic[0]);
    ++k;
    ay[k] = pow(adynamic[0], 2.0);
    ++k;
    ay[k] = sign(adynamic[0]);
    ++k;
    ay[k] = azmul(2.0, adynamic[0]);
    ++k;
    // if dynamic[0] == 0.5 then ay[k] = 1.0 else ay[k] = -1.0
    ay[k] = CppAD::CondExpEq(
        adynamic[0], AD<double>(0.5), AD<double>(1.0), AD<double>(-1.0)
    );
    ++k;
    // ----------------------------------------------------------
    ok &= size_t(k) == ny;

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // change the dynamic parameter values
    CPPAD_TESTVECTOR(double) dynamic(nd);
    dynamic[0] = 0.75;
    f.new_dynamic(dynamic);
    //
    CPPAD_TESTVECTOR(double) x(nx), y(ny);
    y = f.Forward(0, x);
    k = 0;
    // ----------------------------------------------------------
    // 98 standard math
    double check = std::acos(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::asin(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::atan(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::cos(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::cosh(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::exp(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::acos(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::log(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::sin(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::sinh(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::sqrt(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::tan(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::tanh(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    // ----------------------------------------------------------
    // 2011 standard math
    check = asinh(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::acosh(dynamic[0] + 1.0);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::atanh(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::expm1(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::erf(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::log1p(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    // ----------------------------------------------------------
    // binary
    check = 2.0 + dynamic[0];
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = dynamic[0] / 2.0;
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = 2.0 * dynamic[0];
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = dynamic[0] - 2.0;
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    // ----------------------------------------------------------
    // other
    check = - dynamic[0];
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::fabs(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = std::pow(dynamic[0], 2.0);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = CppAD::sign(dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    check = azmul(2.0, dynamic[0]);
    ok   &= NearEqual(y[k], check, eps99, eps99);
    ++k;
    // if dynamic[0] == 0.5 then ay[k] = 1.0 else ay[k] = -1.0
    check = CppAD::CondExpEq(dynamic[0], 0.5, 1.0, -1.0);
    ++k;
    // ----------------------------------------------------------
    ok &= k == ny;
    //
    return ok;
}
// ----------------------------------------------------------------------------
bool dynamic_compare(void)
{   bool ok = true;
    using CppAD::AD;

    // independent dynamic parameter vector
    size_t nd  = 1;
    CPPAD_TESTVECTOR(AD<double>) adynamic(nd);

    // domain space vector
    size_t nx = 1;
    CPPAD_TESTVECTOR(AD<double>) ax(nx);
    ax[0] = 0.0; // value does not matter for this example;

    // range space vector
    size_t ny = 2;
    CPPAD_TESTVECTOR(AD<double>) ay(ny);

    // not using abort_op_index, are recording compare operators
    size_t abort_op_index = 0;
    bool   record_compare = true;

    // Function object
    CppAD::ADFun<double> f;

    // vectors used for new_dynamic and Forward.
    CPPAD_TESTVECTOR(double) dynamic(nd), x(nx), y(ny);

    // ----------------------------------------------------------
    // operators: ==, !=
    adynamic[0] = 0.5;
    CppAD::Independent(ax, abort_op_index, record_compare, adynamic);
    //
    // ==
    if( adynamic[0] == 0.5 )
        ay[0] = 1.0;
    else
        ay[0] = 0.0;
    //
    // !=
    if( adynamic[0] != 0.5 )
        ay[1] = 1.0;
    else
        ay[1] = 0.0;

    // create f: x -> y and stop tape recording
    f.Dependent(ax, ay);
    //
    dynamic[0] = Value( adynamic[0] );
    f.new_dynamic(dynamic);
    y  = f.Forward(0, x);
    ok = f.compare_change_number() == 0;
    //
    dynamic[0] = 1.0;
    f.new_dynamic(dynamic);
    y  = f.Forward(0, x);
    ok = f.compare_change_number() == 2;

    // ----------------------------------------------------------
    // operators: <, <=
    adynamic[0] = 0.5;
    CppAD::Independent(ax, abort_op_index, record_compare, adynamic);
    //
    // <
    if( adynamic[0] < 0.5 )
        ay[0] = 0.0;
    else
        ay[0] = 1.0;
    //
    // <=
    if( adynamic[0] <= 0.5 )
        ay[1] = 1.0;
    else
        ay[1] = 0.0;

    // create f: x -> y and stop tape recording
    f.Dependent(ax, ay);
    //
    dynamic[0] = Value( adynamic[0] );
    f.new_dynamic(dynamic);
    y  = f.Forward(0, x);
    ok = f.compare_change_number() == 0;
    //
    dynamic[0] = 1.0;
    f.new_dynamic(dynamic);
    y  = f.Forward(0, x);
    ok = f.compare_change_number() == 1;

    // ----------------------------------------------------------
    // operators: >, >=
    adynamic[0] = 0.5;
    CppAD::Independent(ax, abort_op_index, record_compare, adynamic);
    //
    // >
    if( adynamic[0] > 0.4 )
        ay[0] = 1.0;
    else
        ay[0] = 0.0;
    //
    // >=
    if( adynamic[0] >= 0.4 )
        ay[1] = 1.0;
    else
        ay[1] = 0.0;

    // create f: x -> y and stop tape recording
    f.Dependent(ax, ay);
    //
    dynamic[0] = Value( adynamic[0] );
    f.new_dynamic(dynamic);
    y  = f.Forward(0, x);
    ok = f.compare_change_number() == 0;
    //
    dynamic[0] = 0.3;
    f.new_dynamic(dynamic);
    y  = f.Forward(0, x);
    ok = f.compare_change_number() == 2;
    //
    // ----------------------------------------------------------
    return ok;
}
// ----------------------------------------------------------------------------
bool optimize_csum(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent dynamic parameter vector
    size_t num_dyn_ind  = 2;
    CPPAD_TESTVECTOR(AD<double>) adynamic(num_dyn_ind);

    // domain space vector
    size_t nx = 1;
    CPPAD_TESTVECTOR(AD<double>) ax(nx);
    ax[0] = 0.0; // value does not matter for this example;

    // range space vector
    size_t ny = 2;
    CPPAD_TESTVECTOR(AD<double>) ay(ny);

    // not using abort_op_index, are recording compare operators
    size_t abort_op_index = 0;
    bool   record_compare = true;

    // Function object
    CppAD::ADFun<double> f;
    // ----------------------------------------------------------
    // record f
    adynamic[0] = 1.0;
    adynamic[1] = 2.0;
    CppAD::Independent(ax, abort_op_index, record_compare, adynamic);
    //
    // create three constant parameters that are only used in a cumulative
    // summation and will be optimized to just one constant parameter
    ay[0] = 3.0 - ax[0];
    ay[0] = ay[0] - 4.0;
    ay[0] = ay[0] + 5.0;
    //
    // create a dependent dynamic parameter that is not used
    AD<double> adp1 = adynamic[0] + adynamic[1];
    //
    // create a dependent dynamic parameter that is used
    AD<double> adp2 = adynamic[1] - adynamic[0];
    ay[1] = ax[0] + adp2;
    //
    // create f: x -> y and stop tape recording
    f.Dependent(ax, ay);
    ok &= f.size_dyn_ind() == num_dyn_ind;
    ok &= f.size_dyn_par() == num_dyn_ind + 2;
    // constant parameters: nan, 3.0, 4.0, 5.0
    ok &= f.size_par()     == num_dyn_ind + 2 + 4;
    // -------------------------------------------------------------
    // vectors used for new_dynamic and Forward.
    CPPAD_TESTVECTOR(double) dynamic(num_dyn_ind), x(nx), y(ny);
    dynamic[0] = 3.0;
    dynamic[1] = 4.0;
    f.new_dynamic(dynamic);
    x[0] = 5.0;
    y    = f.Forward(0, x);
    double check = 3.0 - x[0] - 4.0 + 5.0;
    ok   &= NearEqual(y[0], check, eps99, eps99);
    check = x[0] + dynamic[1] - dynamic[0];
    ok   &= NearEqual(y[1], check, eps99, eps99);
    // -------------------------------------------------------------
    // optimize and re-test
    f.optimize(); // add the zero parameter
    ok &= f.size_dyn_ind() == num_dyn_ind;
    ok &= f.size_dyn_par() == num_dyn_ind + 1;
    // constant parameters: nan, zero, (3.0 - 4.0 + 5.0)
    ok &= f.size_par()     == num_dyn_ind + 1 + 3;
    dynamic[0] = 0.3;
    dynamic[1] = 0.4;
    f.new_dynamic(dynamic);
    x[0] = 0.5;
    y    = f.Forward(0, x);
    check = 3.0 - x[0] - 4.0 + 5.0;
    ok   &= NearEqual(y[0], check, eps99, eps99);
    check = x[0] + dynamic[1] - dynamic[0];
    ok   &= NearEqual(y[1], check, eps99, eps99);
    //
    return ok;
}

// ----------------------------------------------------------------------------
typedef CPPAD_TESTVECTOR( CppAD::AD<double> )  ADvector;
void g_algo(const ADvector& ax, ADvector& ay)
{   ay[0] = ax[0] * ax[1];
}
bool dynamic_atomic(void)
{   bool ok = true;

    // checkpoint version of g(x) = x[0] * x[1];
    ADvector ax(2), ay(1);
    ax[0] = 2.0;
    ax[1] = 3.0;
    CppAD::checkpoint<double> atom_g("g_algo", g_algo, ax, ay);

    // record the function f(x) = dynamic[0] * dynamic[1]
    // using atom_g to compute the product
    ADvector adynamic(2);
    adynamic[0] = 1.0;
    adynamic[1] = 2.0;
    size_t abort_op_index = 0;
    bool   record_compare = true;
    Independent(ax, abort_op_index, record_compare, adynamic);
    atom_g(adynamic, ay);
    CppAD::ADFun<double> f(ax, ay);

    // change the dynamic parameter values
    CPPAD_TESTVECTOR(double) x(2), y(1), dynamic(2);
    dynamic[0] = 4.0;
    dynamic[1] = 5.0;
    f.new_dynamic(dynamic);

    // check zero order forward
    x[0] = std::numeric_limits<double>::quiet_NaN();
    x[1] = std::numeric_limits<double>::quiet_NaN();
    y    = f.Forward(0, x);
    ok  &= y[0] == dynamic[0] * dynamic[1];


    return ok;
}

// ----------------------------------------------------------------------------
double n_digits(const double& x)
{   double logx = std::log10(x);
    double ret  = floor(logx) + 1.0;
    return ret;
}
CPPAD_DISCRETE_FUNCTION(double, n_digits)
//
bool dynamic_discrete(void)
{   bool ok = true;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // record the function f(x) = x[0] * n_digits( dynamic[0] )
    ADvector ax(1);
    ax[0] = 2.0;
    ADvector adynamic(1);
    adynamic[0] = 3.0;
    size_t abort_op_index = 0;
    bool   record_compare = true;
    Independent(ax, abort_op_index, record_compare, adynamic);
    ADvector ay(1);
    ay[0] = ax[0] * n_digits( adynamic[0] );
    CppAD::ADFun<double> f(ax, ay);

    // change the dynamic parameter value to 14
    CPPAD_TESTVECTOR(double) x(1), y(1), dynamic(1);
    dynamic[0] = 14.0;
    f.new_dynamic(dynamic);

    // check zero order forward
    x[0] = 3.0;
    y    = f.Forward(0, x);
    ok  &= CppAD::NearEqual(y[0], x[0] * 2.0, eps99, eps99);

    return ok;
}
// ----------------------------------------------------------------------------
bool dynamic_optimize(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent dynamic parameter vector
    size_t nd  = 1;
    CPPAD_TESTVECTOR(AD<double>) adynamic(nd);

    // domain space vector
    size_t nx = 1;
    CPPAD_TESTVECTOR(AD<double>) ax(nx);
    ax[0] = 0.0; // value does not matter for this example;

    // range space vector
    size_t ny = 2;
    CPPAD_TESTVECTOR(AD<double>) ay(ny);

    // not using abort_op_index, are recording compare operators
    size_t abort_op_index = 0;
    bool   record_compare = true;

    // Function object
    CppAD::ADFun<double> f;
    // ----------------------------------------------------------
    // record f
    adynamic[0] = 1.0;
    CppAD::Independent(ax, abort_op_index, record_compare, adynamic);
    //
    // Avoid cumulative summations because it adds constant parameters.
    // Use one constant parameter, namely 3.0.
    // Create five dynamic parameters where the second, fourth, and fifth,
    // should be optimized out.
    //
    // first extra dynamic parameter (also is in optimized tape)
    AD<double> first  = 3.0 + adynamic[0];
    //
    // second that is the same as first because addition is communative
    AD<double> second = adynamic[0] + 3.0;
    //
    // third extra dynamic parameter (also in optimized tape)
    AD<double> third  = first * first;
    //
    // fourth is same as third and uses second which gets optimized out
    AD<double> fourth = second * second;
    //
    // fifth is not used and so should be optimized out
    AD<double> fifth  = first / 3.0;
    //
    ay[0] = ax[0] + third;
    ay[1] = ax[0] * fourth;
    //
    // create f: x -> y and stop tape recording
    f.Dependent(ax, ay);
    ok &= f.size_dyn_ind() == nd;
    ok &= f.size_dyn_par() == nd + 5;
    // constant parameters: nan, 3.0
    ok &= f.size_par()     == nd + 5 + 2;
    // -------------------------------------------------------------
    // vectors used for new_dynamic and Forward.
    CPPAD_TESTVECTOR(double) dynamic(nd), x(nx), y(ny);
    dynamic[0] = 4.0;
    f.new_dynamic(dynamic);
    x[0] = 5.0;
    y    = f.Forward(0, x);
    double check = x[0] + (3.0 + dynamic[0] ) * (3.0 + dynamic[0]);
    ok   &= NearEqual(y[0], check, eps99, eps99);
    check = x[0] * (3.0 + dynamic[0] ) * (3.0 + dynamic[0]);
    ok   &= NearEqual(y[1], check, eps99, eps99);
    // -------------------------------------------------------------
    // optimize and re-test
    f.optimize(); // adds the zero parameter
    ok &= f.size_dyn_ind() == nd;
    ok &= f.size_dyn_par() == nd + 2;
    // constant parameters: nan, zero, 3.0
    ok &= f.size_par()     == nd + 2 + 3;
    dynamic[0] = 6.0;
    f.new_dynamic(dynamic);
    y     = f.Forward(0, x);
    check = x[0] + (3.0 + dynamic[0]) * (3.0 + dynamic[0]);
    ok   &= NearEqual(y[0], check, eps99, eps99);
    check = x[0] * (3.0 + dynamic[0]) * (3.0 + dynamic[0]);
    ok   &= NearEqual(y[1], check, eps99, eps99);
    //
    return ok;
}


} // END_EMPTY_NAMESPACE

// ----------------------------------------------------------------------------
bool new_dynamic(void)
{   bool ok = true;
    ok     &= vecad();
    ok     &= operator_with_variable();
    ok     &= dynamic_operator();
    ok     &= dynamic_compare();
    ok     &= optimize_csum();
    ok     &= dynamic_atomic();
    ok     &= dynamic_discrete();
    ok     &= dynamic_optimize();
    //
    return ok;
}
