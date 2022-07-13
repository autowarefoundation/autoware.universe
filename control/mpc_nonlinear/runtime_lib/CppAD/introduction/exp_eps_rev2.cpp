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
$begin exp_eps_rev2.cpp$$
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

$section exp_eps: Verify Second Order Reverse Sweep$$


$srccode%cpp% */
# include <cstddef>                     // define size_t
# include <cmath>                       // for fabs function
extern bool exp_eps_for0(double *v0);   // computes zero order forward sweep
extern bool exp_eps_for1(double *v1);   // computes first order forward sweep
bool exp_eps_rev2(void)
{   bool ok = true;

    // set the value of v0[j], v1[j] for j = 1 , ... , 7
    double v0[8], v1[8];
    ok &= exp_eps_for0(v0);
    ok &= exp_eps_for1(v1);

    // initial all partial derivatives as zero
    double f_v0[8], f_v1[8];
    size_t j;
    for(j = 0; j < 8; j++)
    {   f_v0[j] = 0.;
        f_v1[j] = 0.;
    }

    // set partial derivative for f_7
    f_v1[7] = 1.;
    ok &= std::fabs( f_v1[7] - 1.  ) <= 1e-10; // partial f_7 w.r.t. v_7^1

    // f_6 = f_7( v_1^0 , ... , v_6^1 , v_4^0 + v_6^0, v_4^1 , v_6^1 )
    f_v0[4] += f_v0[7];
    f_v0[6] += f_v0[7];
    f_v1[4] += f_v1[7];
    f_v1[6] += f_v1[7];
    ok &= std::fabs( f_v0[4] - 0.  ) <= 1e-10; // partial f_6 w.r.t. v_4^0
    ok &= std::fabs( f_v0[6] - 0.  ) <= 1e-10; // partial f_6 w.r.t. v_6^0
    ok &= std::fabs( f_v1[4] - 1.  ) <= 1e-10; // partial f_6 w.r.t. v_4^1
    ok &= std::fabs( f_v1[6] - 1.  ) <= 1e-10; // partial f_6 w.r.t. v_6^1

    // f_5 = f_6( v_1^0 , ... , v_5^1 , v_5^0 / 2 , v_5^1 / 2 )
    f_v0[5] += f_v0[6] / 2.;
    f_v1[5] += f_v1[6] / 2.;
    ok &= std::fabs( f_v0[5] - 0.  ) <= 1e-10; // partial f_5 w.r.t. v_5^0
    ok &= std::fabs( f_v1[5] - 0.5 ) <= 1e-10; // partial f_5 w.r.t. v_5^1

    // f_4 = f_5( v_1^0 , ... , v_4^1 , v_3^0 * v_1^0 ,
    //            v_3^1 * v_1^0 + v_3^0 * v_1^1 )
    f_v0[1] += f_v0[5] * v0[3] + f_v1[5] * v1[3];
    f_v0[3] += f_v0[5] * v0[1] + f_v1[5] * v1[1];
    f_v1[1] += f_v1[5] * v0[3];
    f_v1[3] += f_v1[5] * v0[1];
    ok &= std::fabs( f_v0[1] - 0.5  ) <= 1e-10; // partial f_4 w.r.t. v_1^0
    ok &= std::fabs( f_v0[3] - 0.5  ) <= 1e-10; // partial f_4 w.r.t. v_3^0
    ok &= std::fabs( f_v1[1] - 0.25 ) <= 1e-10; // partial f_4 w.r.t. v_1^1
    ok &= std::fabs( f_v1[3] - 0.25 ) <= 1e-10; // partial f_4 w.r.t. v_3^1

    // f_3 = f_4(  v_1^0 , ... , v_3^1 , 1 + v_3^0 , v_3^1 )
    f_v0[3] += f_v0[4];
    f_v1[3] += f_v1[4];
    ok &= std::fabs( f_v0[3] - 0.5 ) <= 1e-10;  // partial f_3 w.r.t. v_3^0
    ok &= std::fabs( f_v1[3] - 1.25) <= 1e-10;  // partial f_3 w.r.t. v_3^1

    // f_2 = f_3( v_1^0 , ... , v_2^1 , v_2^0 , v_2^1 )
    f_v0[2] += f_v0[3];
    f_v1[2] += f_v1[3];
    ok &= std::fabs( f_v0[2] - 0.5 ) <= 1e-10;  // partial f_2 w.r.t. v_2^0
    ok &= std::fabs( f_v1[2] - 1.25) <= 1e-10;  // partial f_2 w.r.t. v_2^1

    // f_1 = f_2 ( v_1^0 , v_2^0 , v_1^0 , v_2^0 )
    f_v0[1] += f_v0[2];
    f_v1[1] += f_v1[2];
    ok &= std::fabs( f_v0[1] - 1.  ) <= 1e-10;  // partial f_1 w.r.t. v_1^0
    ok &= std::fabs( f_v1[1] - 1.5 ) <= 1e-10;  // partial f_1 w.r.t. v_1^1

    return ok;
}
/* %$$
$end
*/
