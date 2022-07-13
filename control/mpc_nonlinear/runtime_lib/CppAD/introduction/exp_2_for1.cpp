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
$begin exp_2_for1.cpp$$
$spell
    std
    exp_2_for
    cmath
    fabs
    bool
$$

$section exp_2: Verify First Order Forward Sweep$$


$srccode%cpp% */
# include <cmath>                   // prototype for fabs
extern bool exp_2_for0(double *v0); // computes zero order forward sweep
bool exp_2_for1(double *v1)         // double v1[6]
{   bool ok = true;
    double v0[6];

    // set the value of v0[j] for j = 1 , ... , 5
    ok &= exp_2_for0(v0);

    v1[1] = 1.;                                     // v1 = x
    ok    &= std::fabs( v1[1] - 1. ) <= 1e-10;

    v1[2] = v1[1];                                  // v2 = 1 + v1
    ok    &= std::fabs( v1[2] - 1. ) <= 1e-10;

    v1[3] = v1[1] * v0[1] + v0[1] * v1[1];          // v3 = v1 * v1
    ok    &= std::fabs( v1[3] - 1. ) <= 1e-10;

    v1[4] = v1[3] / 2.;                             // v4 = v3 / 2
    ok    &= std::fabs( v1[4] - 0.5) <= 1e-10;

    v1[5] = v1[2] + v1[4];                          // v5 = v2 + v4
    ok    &= std::fabs( v1[5] - 1.5) <= 1e-10;

    return ok;
}
bool exp_2_for1(void)
{   double v1[6];
    return exp_2_for1(v1);
}
/* %$$
$end
*/
