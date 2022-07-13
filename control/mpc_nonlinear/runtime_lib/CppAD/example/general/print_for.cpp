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
$begin print_for_string.cpp$$
$spell
    Cpp
$$

$section Print During Zero Order Forward Mode: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace {
    using std::endl;
    using CppAD::AD;

    // use of PrintFor to check for invalid function arguments
    AD<double> check_log(const AD<double>& u, std::ostream& s_out)
    {   // check AD<double> value during recording
        if( u <= 0 )
            s_out << "check_log: u = " << u << " which is <= 0\n";

        // check double value during zero order forward calculation
        PrintFor(u, "check_log: u = ", u , " which is <= 0\n");

        return log(u);
    }
}

bool print_for(void)
{   bool ok = true;
    using CppAD::PrintFor;
    std::stringstream stream_out;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector
    size_t np = 1;
    size_t nx = 1;
    size_t ny = 1;
    CPPAD_TESTVECTOR(AD<double>) ap(np), ax(nx), ay(ny);
    ap[0] = 1.0;
    ax[0] = 2.0;
    Independent(ax, ap);
    //
    // define f(x, p) = log(p) + log(x)
    ay[0] = check_log(ap[0], stream_out) + check_log(ax[0], stream_out);
    CppAD::ADFun<double> f(ax, ay);
    //
    // zero order forward
    // both x and p are positive so no output generated
    CPPAD_TESTVECTOR(double) p(np), x(nx), y(ny);
    p[0] = 1.0;
    x[0] = 2.0;
    f.new_dynamic(p);
    f.check_for_nan(false);
    y = f.Forward(0, x, stream_out);
    ok &= stream_out.str() == "";
    ok &= CppAD::NearEqual(y[0], std::log(p[0]) + std::log(x[0]), eps99, eps99);
    //
    // zero order forward
    // p is negative so output generated
    p[0] = -1.0;
    x[0] = 2.0;
    f.new_dynamic(p);
    f.check_for_nan(false);
    y = f.Forward(0, x, stream_out);
    ok &= stream_out.str() == "check_log: u = -1 which is <= 0\n";
    ok &= std::isnan(y[0]);
    //
    return ok;
}
// END C++
