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
$begin exp_eps_rev1.cpp$$
$spell
    cstddef
    cmath
    vars
    ir
    iq
    ir
    ia
    df
    exp_eps_rev
    bool
    std
    fabs
$$

$section exp_eps: Verify First Order Reverse Sweep$$


$srccode%cpp% */
# include <cstddef>                     // define size_t
# include <cmath>                       // for fabs function
extern bool exp_eps_for0(double *v0);   // computes zero order forward sweep
bool exp_eps_rev1(void)
{   bool ok = true;

    // set the value of v0[j] for j = 1 , ... , 7
    double v0[8];
    ok &= exp_eps_for0(v0);

    // initial all partial derivatives as zero
    double f_v[8];
    size_t j;
    for(j = 0; j < 8; j++)
        f_v[j] = 0.;

    // set partial derivative for f7
    f_v[7] = 1.;
    ok    &= std::fabs( f_v[7] - 1. ) <= 1e-10;     // f7_v7

    // f6( v1 , v2 , v3 , v4 , v5 , v6 )
    f_v[4] += f_v[7] * 1.;
    f_v[6] += f_v[7] * 1.;
    ok     &= std::fabs( f_v[4] - 1.  ) <= 1e-10;   // f6_v4
    ok     &= std::fabs( f_v[6] - 1.  ) <= 1e-10;   // f6_v6

    // f5( v1 , v2 , v3 , v4 , v5 )
    f_v[5] += f_v[6] / 2.;
    ok     &= std::fabs( f_v[5] - 0.5 ) <= 1e-10;   // f5_v5

    // f4( v1 , v2 , v3 , v4 )
    f_v[1] += f_v[5] * v0[3];
    f_v[3] += f_v[5] * v0[1];
    ok     &= std::fabs( f_v[1] - 0.25) <= 1e-10;   // f4_v1
    ok     &= std::fabs( f_v[3] - 0.25) <= 1e-10;   // f4_v3

    // f3( v1 , v2 , v3 )
    f_v[3] += f_v[4] * 1.;
    ok     &= std::fabs( f_v[3] - 1.25) <= 1e-10;   // f3_v3

    // f2( v1 , v2 )
    f_v[2] += f_v[3] / 1.;
    ok     &= std::fabs( f_v[2] - 1.25) <= 1e-10;   // f2_v2

    // f1( v1 )
    f_v[1] += f_v[2] * 1.;
    ok     &= std::fabs( f_v[1] - 1.5 ) <= 1e-10;   // f1_v2

    return ok;
}
/* %$$
$end
*/
