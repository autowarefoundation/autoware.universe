/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

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

// external complied tests
extern bool abs_eval(void);
extern bool abs_min_linear(void);
extern bool abs_min_quad(void);
extern bool get_started(void);
extern bool lp_box(void);
extern bool min_nso_linear(void);
extern bool min_nso_quad(void);
extern bool qp_box(void);
extern bool qp_interior(void);
extern bool simplex_method(void);

// main program that runs all the tests
int main(void)
{   std::string group = "example/abs_norml";
    size_t      width = 20;
    CppAD::test_boolofvoid Run(group, width);

    // This line is used by test_one.sh

    // external compiled tests
    Run( abs_eval,            "abs_eval"           );
    Run( abs_min_linear,      "abs_min_linear"     );
    Run( abs_min_quad,        "abs_min_quad"       );
    Run( get_started,         "get_started"        );
    Run( lp_box,              "lp_box"             );
    Run( min_nso_linear,      "min_nso_linear"     );
    Run( min_nso_quad,         "min_nso_quad"      );
    Run( qp_box,              "qp_box"             );
    Run( qp_interior,         "qp_interior"        );
    Run( simplex_method,      "simplex_method"     );

    // check for memory leak
    bool memory_ok = CppAD::thread_alloc::free_all();
    // print summary at end
    bool ok = Run.summary(memory_ok);
    //
    return static_cast<int>( ! ok );
}
