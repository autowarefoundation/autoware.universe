/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// CPPAD_HAS_* defines
# include <cppad/configure.hpp>

// system include files used for I/O
# include <iostream>

// C style asserts
# include <cassert>

// for thread_alloc
# include <cppad/utility/thread_alloc.hpp>

// test runner
# include <cppad/utility/test_boolofvoid.hpp>

// BEGIN_SORT_THIS_LINE_PLUS_2
// external compiled tests
extern bool file(void);
extern bool function(void);
extern bool jacobian(void);
extern bool jac_as_fun(void);
extern bool sparse_jacobian(void);
extern bool sparse_jac_as_fun(void);
// END_SORT_THIS_LINE_MINUS_1

// main program that runs all the tests
int main(void)
{   std::string group = "test_more/code_gen_fun";
    size_t      width = 20;
    CppAD::test_boolofvoid Run(group, width);

    // This line is used by test_one.sh

    // BEGIN_SORT_THIS_LINE_PLUS_2
    // external compiled tests
    Run( file,                 "file"              );
    Run( function,             "function"          );
    Run( jacobian,             "jacobian"          );
    Run( jac_as_fun,           "jac_as_fun"        );
    Run( sparse_jacobian,      "sparse_jacobian"   );
    Run( sparse_jac_as_fun,    "sparse_jac_as_fun" );
    // END_SORT_THIS_LINE_MINUS_1

    // check for memory leak
    bool memory_ok = CppAD::thread_alloc::free_all();
    // print summary at end
    bool ok = Run.summary(memory_ok);
    //
    return static_cast<int>( ! ok );
}
