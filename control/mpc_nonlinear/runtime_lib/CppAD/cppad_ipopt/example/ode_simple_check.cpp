/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include "ode_run.hpp"
# include "ode_simple.hpp"
# include "ode_check.hpp"

bool ode_simple_check(void)
{   bool ok = true;
    bool retape;
    size_t i;

    // solution vector
    NumberVector x;

    // number of time grid intervals between measurement values
    SizeVector N(Nz + 1);
    N[0] = 0;
    for(i = 1; i <= Nz; i++)
        N[i] = 4;

    for(i = 0; i < 2; i++)
    {   retape = bool(i);
        ipopt_ode_case<FG_simple>(retape, N, x);
        ok &= ode_check(N, x);
    }

    return ok;
}
