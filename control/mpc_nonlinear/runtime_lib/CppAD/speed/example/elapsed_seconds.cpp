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
$begin elapsed_seconds.cpp$$
$spell
    Cpp
    Lu
$$

$section Elapsed Seconds: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/utility/elapsed_seconds.hpp>

# include <iostream>
# include <algorithm>
# include <cmath>

# define CPPAD_DEBUG_ELAPSED_SECONDS 0

bool elapsed_seconds(void)
{   bool ok = true;

    double max_diff = 0.;
    double s0 = CppAD::elapsed_seconds();
    double s1 = CppAD::elapsed_seconds();
    double s2 = CppAD::elapsed_seconds();
    while(s2 - s0 < 1.)
    {   max_diff = std::max(s2 - s1, max_diff);
        s1 = s2;
        s2 = CppAD::elapsed_seconds();

    }
# if CPPAD_DEBUG_ELAPSED_SECONDS
    std::cout << "max_diff = " << max_diff << std::endl;
# endif
    ok &= 0. < max_diff && max_diff < .04;
    return ok;
}

// END C++
