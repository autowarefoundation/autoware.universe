/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// system include files used for I/O
# include <iostream>

// C style asserts
# include <cassert>

// CppAD include file
# include <cppad/cppad.hpp>

// test runner
# include <cppad/utility/test_boolofvoid.hpp>

// external complied tests
extern bool ipopt_get_started(void);
extern bool ode_simple_check(void);
extern bool ode_fast_check(void);

// main program that runs all the tests
int main(void)
{   std::string group = "cppad_ipopt/example";
    size_t      width = 20;
    CppAD::test_boolofvoid Run(group, width);

    // external compiled tests
    Run( ipopt_get_started,   "ipopt_get_started"  );
    Run( ode_simple_check,    "ode_simple_check"   );
    Run( ode_fast_check,      "ode_fast_check"     );
    //
    // check for memory leak
    bool memory_ok = CppAD::thread_alloc::free_all();
    // print summary at end
    bool ok = Run.summary(memory_ok);
    //
    return static_cast<int>( ! ok );
}
