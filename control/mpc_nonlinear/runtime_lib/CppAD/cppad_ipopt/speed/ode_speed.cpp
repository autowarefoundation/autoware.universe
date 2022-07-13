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
$begin ipopt_ode_speed.cpp$$

$section Speed Test for Both Simple and Fast Representations$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/

// BEGIN C++
# include "../example/ode_run.hpp"
# include "../example/ode_simple.hpp"
# include "../example/ode_fast.hpp"
# include <cassert>
# include <cstring>

# if CPPAD_HAS_GETTIMEOFDAY & CPPAD_NO_MICROSOFT
# include <sys/time.h>
# else
# include <ctime>
# endif

namespace {
    double current_second(void)
    {
# if CPPAD_HAS_GETTIMEOFDAY & CPPAD_NOT_MICOROSOFT
        struct timeval value;
        gettimeofday(&value, 0);
        return double(value.tv_sec) + double(value.tv_usec) * 1e-6;
# else
        return (double) clock() / (double) CLOCKS_PER_SEC;
# endif
    }
}

double ode_speed(const char* name, size_t& count)
{
    // determine simple and retape flags
    bool simple = true, retape = true;
    if( std::strcmp(name, "simple_retape_no") == 0 )
    {   simple = true; retape = false; }
    else if( std::strcmp(name, "simple_retape_yes") == 0 )
    {   simple = true; retape = true; }
    else if( std::strcmp(name, "fast_retape_no") == 0 )
    {   simple = false; retape = false; }
    else if( std::strcmp(name, "fast_retape_yes") == 0 )
    {   simple = false; retape = true; }
    else
        assert(false);

    size_t i;
    double s0, s1;
    size_t  c0, c1;

    // solution vector
    NumberVector x;

    // number of time grid intervals between measurement values
    SizeVector N(Nz + 1);
    N[0] = 0;
    for(i = 1; i <= Nz; i++)
    {   N[i] = 10;
        // n   += N[i] * Ny;
    }
    // n += Na;

    s0              = current_second();
    c0              = count_eval_r();
    if( simple )
        ipopt_ode_case<FG_simple>(retape, N, x);
    else
        ipopt_ode_case<FG_fast>(retape, N, x);
    s1              = current_second();
    c1              = count_eval_r();
    count           = c1 - c0 - 1;
    return s1 - s0;
}
// END C++
