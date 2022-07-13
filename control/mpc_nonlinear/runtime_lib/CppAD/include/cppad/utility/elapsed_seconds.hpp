# ifndef CPPAD_UTILITY_ELAPSED_SECONDS_HPP
# define CPPAD_UTILITY_ELAPSED_SECONDS_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin elapsed_seconds$$
$spell
    cppad.hpp
    std::chrono
$$

$section Returns Elapsed Number of Seconds$$


$head Syntax$$
$codei%# include <cppad/utility/elapsed_seconds.hpp>
%$$
$icode%s% = elapsed_seconds()%$$

$head Accuracy$$
This routine uses $code std::chrono::steady_clock$$ to do its timing.

$head s$$
is a $code double$$ equal to the
number of seconds since the first call to $code elapsed_seconds$$.

$children%
    speed/example/elapsed_seconds.cpp
%$$
$head Example$$
The routine $cref elapsed_seconds.cpp$$ is
an example and test of this routine.


$end
-----------------------------------------------------------------------
*/

# include <cppad/core/cppad_assert.hpp>

// needed before one can use CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL
# include <cppad/utility/thread_alloc.hpp>

// c++11 time function
# include <chrono>



namespace CppAD { // BEGIN_CPPAD_NAMESPACE

inline double elapsed_seconds(void)
// --------------------------------------------------------------------------
{   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;
    static bool first_ = true;
    static std::chrono::time_point<std::chrono::steady_clock> start_;
    if( first_ )
    {   start_ = std::chrono::steady_clock::now();
        first_ = false;
        return 0.0;
    }
    std::chrono::time_point<std::chrono::steady_clock> now;
    now   = std::chrono::steady_clock::now();
    std::chrono::duration<double> difference = now - start_;
    return difference.count();
}
// --------------------------------------------------------------------------
} // END_CPPAD_NAMESPACE

# endif
