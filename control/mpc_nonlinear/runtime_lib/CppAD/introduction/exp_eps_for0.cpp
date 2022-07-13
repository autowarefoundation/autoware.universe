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
$begin exp_eps_for0.cpp$$
$spell
    std
    exp_eps_seq
    cmath
    bool
    fabs
$$


$section exp_eps: Verify Zero Order Forward Sweep$$


$srccode%cpp% */
# include <cmath>                // for fabs function
bool exp_eps_for0(double *v0)    // double v0[8]
{   bool  ok = true;
    double x = .5;

    v0[1] = x;                                  // abs_x = x;
    ok  &= std::fabs( v0[1] - 0.5) < 1e-10;

    v0[2] = 1. * v0[1];                         // temp = term * abs_x;
    ok  &= std::fabs( v0[2] - 0.5) < 1e-10;

    v0[3] = v0[2] / 1.;                         // term = temp / Type(k);
    ok  &= std::fabs( v0[3] - 0.5) < 1e-10;

    v0[4] = 1. + v0[3];                         // sum = sum + term;
    ok  &= std::fabs( v0[4] - 1.5) < 1e-10;

    v0[5] = v0[3] * v0[1];                      // temp = term * abs_x;
    ok  &= std::fabs( v0[5] - 0.25) < 1e-10;

    v0[6] = v0[5] / 2.;                         // term = temp / Type(k);
    ok  &= std::fabs( v0[6] - 0.125) < 1e-10;

    v0[7] = v0[4] + v0[6];                      // sum = sum + term;
    ok  &= std::fabs( v0[7] - 1.625) < 1e-10;

    return ok;
}
bool exp_eps_for0(void)
{   double v0[8];
    return exp_eps_for0(v0);
}
/* %$$
$end
*/
