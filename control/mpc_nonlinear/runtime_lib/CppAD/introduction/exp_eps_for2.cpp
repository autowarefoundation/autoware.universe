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
$begin exp_eps_for2.cpp$$
$spell
    std
    vars
    exp_eps_for
    cmath
    fabs
    bool
$$

$section exp_eps: Verify Second Order Forward Sweep$$


$srccode%cpp% */
# include <cmath>                     // for fabs function
extern bool exp_eps_for0(double *v0); // computes zero order forward sweep
extern bool exp_eps_for1(double *v1); // computes first order forward sweep
bool exp_eps_for2(void)
{   bool ok = true;
    double v0[8], v1[8], v2[8];

    // set the value of v0[j], v1[j] for j = 1 , ... , 7
    ok &= exp_eps_for0(v0);
    ok &= exp_eps_for1(v1);

    v2[1] = 0.;                                      // v1 = x
    ok    &= std::fabs( v2[1] - 0. ) <= 1e-10;

    v2[2] = 1. * v2[1];                              // v2 = 1 * v1
    ok    &= std::fabs( v2[2] - 0. ) <= 1e-10;

    v2[3] = v2[2] / 1.;                              // v3 = v2 / 1
    ok    &= std::fabs( v2[3] - 0. ) <= 1e-10;

    v2[4] = v2[3];                                   // v4 = 1 + v3
    ok    &= std::fabs( v2[4] - 0. ) <= 1e-10;

    v2[5] = v2[3] * v0[1] + 2. * v1[3] * v1[1]       // v5 = v3 * v1
          + v0[3] * v2[1];
    ok    &= std::fabs( v2[5] - 2. ) <= 1e-10;

    v2[6] = v2[5] / 2.;                              // v6 = v5 / 2
    ok    &= std::fabs( v2[6] - 1. ) <= 1e-10;

    v2[7] = v2[4] + v2[6];                           // v7 = v4 + v6
    ok    &= std::fabs( v2[7] - 1. ) <= 1e-10;

    return ok;
}
/* %$$
$end
*/
