/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// Test that reverse mode handles conditional expressions properly
// in that infinity and nans do not propagate thouh un-used case.

# include <cppad/cppad.hpp>

bool cond_exp_rev(void)
{   bool ok = true;
    using CppAD::vector;
    using CppAD::AD;

    AD<double> anan  = std::numeric_limits<double>::quiet_NaN();
    AD<double> azero = 0.0;


    size_t n = 2;
    vector< AD<double> > ax(n), ay;
    ax[0] = 1.0;
    ax[1] = anan;
    Independent(ax);

    // AbsOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], fabs(ax[1]) ));

    // AcosOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], acos(ax[1]) ));

    // AddvvOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], ax[0] + ax[1] ));

    // AddpvOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], 1.0 + ax[1] ));

    // AsinOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], asin(ax[1]) ));

    // AtanOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], atan(ax[1]) ));

    // CosOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], cos(ax[1]) ));

    // CoshOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], cosh(ax[1]) ));

    // DivvvOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], ax[0] / ax[1] ));

    // DivpvOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], 1.0 / ax[1] ));

    // DivvpOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], ax[1] / 2.0 ));

    // ErfOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], erf(ax[1]) ));

    // ExpOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], exp(ax[1]) ));

    // LogOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], log(ax[1]) ));

    // MulvvOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], ax[0] * ax[1] ));

    // MulpvOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], 2.0 * ax[1] ));

    // PowvvOP
    // uses check in log, mul, and exp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], pow(ax[1], ax[1]) ));

    // PowvpOP
    // uses check in log, mul, and exp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], pow(ax[1], 2.0) ));

    // PowpvOP
    // uses check in log, mul, and exp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], pow(2.0, ax[1]) ));

    // SignOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], sign(ax[1]) ));

    // SinOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], sin(ax[1]) ));

    // SinhOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], sinh(ax[1]) ));

    // SqrtOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], sqrt(ax[1]) ));

    // SubvvOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], ax[0] - ax[1] ));

    // SubpvOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], 1.0 - ax[1] ));

    // SubvpOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], ax[1] - 1.0 ));

    // TanOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], tan(ax[1]) ));

    // TanhOp
    ay.push_back( CondExpGt(ax[0], azero, ax[0], tanh(ax[1]) ));

    // create f : x -> y
    size_t m = ay.size();
    CppAD::ADFun<double> f(ax, ay);

    // weighting vector and reverse mode derivative
    vector<double> w(m), dw(n);
    for(size_t i = 0; i < m; i++)
        w[i] = 0.0;

    // check DivvOp
    for(size_t i = 0; i < m; i++)
    {   w[i] = 1.0;
        dw = f.Reverse(1, w);
        ok &= dw[0] == 1.0;
        ok &= dw[1] == 0.0;
        w[i] = 0.0;
    }

    return ok;
}
