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

// BEGIN_SORT_THIS_LINE_PLUS_2
// external compiled tests
extern bool add_op(void);
extern bool atom_op(void);
extern bool azmul_op(void);
extern bool cexp_op(void);
extern bool comp_op(void);
extern bool discrete_op(void);
extern bool div_op(void);
extern bool mul_op(void);
extern bool pow_op(void);
extern bool print_graph(void);
extern bool print_op(void);
extern bool sub_op(void);
extern bool sum_op(void);
extern bool switch_var_dyn(void);
extern bool unary_op(void);
// END_SORT_THIS_LINE_MINUS_1

// main program that runs all the tests
int main(void)
{   std::string group = "example/graph";
    size_t      width = 20;
    CppAD::test_boolofvoid Run(group, width);

    // This line is used by test_one.sh

    // BEGIN_SORT_THIS_LINE_PLUS_2
    // external compiled tests
    Run( add_op,               "add_op"          );
    Run( atom_op,              "atom_op"         );
    Run( azmul_op,             "azmul_op"        );
    Run( cexp_op,              "cexp_op"         );
    Run( comp_op,              "comp_op"         );
    Run( div_op,               "div_op"          );
    Run( discrete_op,          "discrete_op"     );
    Run( mul_op,               "mul_op"          );
    Run( pow_op,               "pow_op"          );
    Run( print_graph,          "print_graph"     );
    Run( print_op,             "print_op"        );
    Run( sub_op,               "sub_op"          );
    Run( sum_op,               "sum_op"          );
    Run( switch_var_dyn,       "switch_var_dyn"  );
    Run( unary_op,             "unary_op"        );
    // END_SORT_THIS_LINE_MINUS_1

    // check for memory leak
    bool memory_ok = CppAD::thread_alloc::free_all();
    // print summary at end
    bool ok = Run.summary(memory_ok);
    //
    return static_cast<int>( ! ok );
}
