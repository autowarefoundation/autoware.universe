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
$begin exp_2_rev1.cpp$$
$spell
    cstddef
    cmath
    exp_2_rev
    bool
    std
    fabs
$$

$section exp_2: Verify First Order Reverse Sweep$$



$srccode%cpp% */
# include <cstddef>                 // define size_t
# include <cmath>                   // prototype for fabs
extern bool exp_2_for0(double *v0); // computes zero order forward sweep
bool exp_2_rev1(void)
{   bool ok = true;

    // set the value of v0[j] for j = 1 , ... , 5
    double v0[6];
    ok &= exp_2_for0(v0);

    // initial all partial derivatives as zero
    double f_v[6];
    size_t j;
    for(j = 0; j < 6; j++)
        f_v[j] = 0.;

    // set partial derivative for f5
    f_v[5] = 1.;
    ok &= std::fabs( f_v[5] - 1. ) <= 1e-10; // f5_v5

    // f4 = f5( v1 , v2 , v3 , v4 , v2 + v4 )
    f_v[2] += f_v[5] * 1.;
    f_v[4] += f_v[5] * 1.;
    ok &= std::fabs( f_v[2] - 1. ) <= 1e-10; // f4_v2
    ok &= std::fabs( f_v[4] - 1. ) <= 1e-10; // f4_v4

    // f3 = f4( v1 , v2 , v3 , v3 / 2 )
    f_v[3] += f_v[4] / 2.;
    ok &= std::fabs( f_v[3] - 0.5) <= 1e-10; // f3_v3

    // f2 = f3( v1 , v2 , v1 * v1 )
    f_v[1] += f_v[3] * 2. * v0[1];
    ok &= std::fabs( f_v[1] - 0.5) <= 1e-10; // f2_v1

    // f1 = f2( v1 , 1 + v1 )
    f_v[1] += f_v[2] * 1.;
    ok &= std::fabs( f_v[1] - 1.5) <= 1e-10; // f1_v1

    return ok;
}
/* %$$
$end
*/
