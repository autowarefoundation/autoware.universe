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
$begin ipopt_nlp_ode_check.cpp$$

$section Correctness Check for Both Simple and Fast Representations$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include "ode_run.hpp"

bool ode_check(const SizeVector& N, const NumberVector& x)
{   bool ok = true;
    size_t i, j;

    // number of components of x corresponding to values for y
    size_t ny_inx = x.size() - Na;

    // compute the partial sums of the number of grid points
    // and the maximum step size for the trapezoidal approximation
    SizeVector S(Nz+1);
    S[0] = 0;
    Number max_step = 0.;
    for(i = 1; i <= Nz; i++)
    {   S[i] = S[i-1] + N[i];
        max_step = std::max(max_step, Number(s[i] - s[i-1]) / Number(N[i]) );
    }

    // split out return values
    NumberVector a(Na), y_0(Ny), y_1(Ny), y_2(Ny);
    for(j = 0; j < Na; j++)
        a[j] = x[ny_inx+j];
    for(j = 0; j < Ny; j++)
    {   y_0[j] = x[j];
        y_1[j] = x[Ny + j];
        y_2[j] = x[2 * Ny + j];
    }

    // Check some of the optimal a value
    Number rel_tol = max_step * max_step;
    Number abs_tol = rel_tol;
    Number check_a[] = {a0, a1, a2}; // see the y_one function
    for(j = 0; j < Na; j++)
    {
        ok &= CppAD::NearEqual(
            check_a[j], a[j], rel_tol, abs_tol
        );
    }

    // check accuarcy of constraint equations
    rel_tol = 1e-9;
    abs_tol = 1e-9;

    // check the initial value constraint
    NumberVector F = eval_F(a);
    for(j = 0; j < Ny; j++)
        ok &= CppAD::NearEqual(F[j], y_0[j], rel_tol, abs_tol);

    // check the first trapezoidal equation
    NumberVector G_0 = eval_G(y_0, a);
    NumberVector G_1 = eval_G(y_1, a);
    Number dt = (s[1] - s[0]) / Number(N[1]);
    Number check;
    for(j = 0; j < Ny; j++)
    {   check = y_1[j] - y_0[j] - (G_1[j]+G_0[j])*dt/2;
        ok &= CppAD::NearEqual( check, 0., rel_tol, abs_tol);
    }
    //
    // check the second trapezoidal equation
    NumberVector G_2 = eval_G(y_2, a);
    if( N[1] == 1 )
        dt = (s[2] - s[1]) / Number(N[2]);
    for(j = 0; j < Ny; j++)
    {   check = y_2[j] - y_1[j] - (G_2[j]+G_1[j])*dt/2;
        ok &= CppAD::NearEqual( check, 0., rel_tol, abs_tol);
    }
    //
    // check the objective function (specialized to this case)
    check = 0.;
    NumberVector y_i(Ny);
    for(size_t k = 0; k < Nz; k++)
    {   for(j = 0; j < Ny; j++)
            y_i[j] =  x[S[k+1] * Ny + j];
        check += eval_H<Number>(k + 1, y_i, a);
    }
    Number obj_value = 0.; // optimal object (no noise in simulation)
    ok &= CppAD::NearEqual(check, obj_value, rel_tol, abs_tol);

    // Use this empty namespace function to avoid warning that it is not used
    static size_t ode_check_count = 0;
    ode_check_count++;
    ok &= count_eval_r() == ode_check_count;

    return ok;
}
// END C++
