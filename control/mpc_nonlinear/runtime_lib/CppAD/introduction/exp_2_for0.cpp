/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin exp_2_for0.cpp$$
$spell
    std
    exp_2_seq
    cmath
    bool
    fabs
$$

$section exp_2: Verify Zero Order Forward Sweep$$



$srccode%cpp% */
# include <cmath>            // for fabs function
bool exp_2_for0(double *v0)  // double v0[6]
{   bool  ok = true;
    double x = .5;

    v0[1] = x;                                  // v1 = x
    ok  &= std::fabs( v0[1] - 0.5) < 1e-10;

    v0[2] = 1. + v0[1];                         // v2 = 1 + v1
    ok  &= std::fabs( v0[2] - 1.5) < 1e-10;

    v0[3] = v0[1] * v0[1];                      // v3 = v1 * v1
    ok  &= std::fabs( v0[3] - 0.25) < 1e-10;

    v0[4] = v0[3] / 2.;                         // v4 = v3 / 2
    ok  &= std::fabs( v0[4] - 0.125) < 1e-10;

    v0[5] = v0[2] + v0[4];                      // v5  = v2 + v4
    ok  &= std::fabs( v0[5] - 1.625) < 1e-10;

    return ok;
}
bool exp_2_for0(void)
{   double v0[6];
    return exp_2_for0(v0);
}
/* %$$
$end
*/
