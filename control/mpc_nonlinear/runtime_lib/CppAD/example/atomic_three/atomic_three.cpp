/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

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

// BEGIN_SORT_THIS_LINE_PLUS_1
extern bool base2ad(void);
extern bool dynamic(void);
extern bool forward(void);
extern bool get_started(void);
extern bool hes_sparsity(void);
extern bool jac_sparsity(void);
extern bool mat_mul(void);
extern bool norm_sq(void);
extern bool reciprocal(void);
extern bool rev_depend(void);
extern bool reverse(void);
extern bool tangent(void);
extern bool vector_op(void);
// END_SORT_THIS_LINE_MINUS_1

// main program that runs all the tests
int main(void)
{   std::string group = "example/atomic";
    size_t      width = 20;
    CppAD::test_boolofvoid Run(group, width);

    // This line is used by test_one.sh

    // BEGIN_SORT_THIS_LINE_PLUS_1
    Run( base2ad,             "base2ad"        );
    Run( dynamic,             "dynamic"        );
    Run( forward,             "forward"        );
    Run( get_started,         "get_started"    );
    Run( hes_sparsity,        "hes_sparsity"   );
    Run( jac_sparsity,        "jac_sparsity"   );
    Run( mat_mul,             "mat_mul"        );
    Run( norm_sq,             "norm_sq"        );
    Run( reciprocal,          "reciprocal"     );
    Run( rev_depend,          "rev_depend"     );
    Run( reverse,             "reverse"        );
    Run( tangent,             "tangent"        );
    Run( vector_op,           "vector_op"      );
    // END_SORT_THIS_LINE_MINUS_1

    // check for memory leak
    bool memory_ok = CppAD::thread_alloc::free_all();
    // print summary at end
    bool ok = Run.summary(memory_ok);
    //
    return static_cast<int>( ! ok );
}
