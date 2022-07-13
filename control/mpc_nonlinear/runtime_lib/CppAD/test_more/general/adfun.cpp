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
Test that ADFun copy constructor generates an error message.
*/

# include <cppad/cppad.hpp>
# include <string>

namespace { // BEGIN_EMPTY_NAMESPACE


bool adfun_empty(void)
{   bool ok = true;
    size_t thread  = CppAD::thread_alloc::thread_num();
    //
    // Allocate this memory first incase we are using
    // CppAD::vector which uses thread_alloc.
    CPPAD_TESTVECTOR( CppAD::AD<double> ) ax(1), ay(1);
    //
    // check that an empty function does not hold memory
    size_t inuse_1  = CppAD::thread_alloc::inuse(thread);
    CppAD::ADFun<double> f;
    size_t inuse_2  = CppAD::thread_alloc::inuse(thread);
    ok &= inuse_1 == inuse_2;
    //
    // check that creating an adfun uses memory
    CppAD::Independent(ax);
    ay = ax;
    CppAD::ADFun<double> g(ax, ay);
    size_t inuse_3  = CppAD::thread_alloc::inuse(thread);
    ok &= inuse_1 < inuse_3;
    //
    // assigning to an empty function uses assignment to pod_vectors
    // which just changes their lenght to zero
    g = f;
    size_t inuse_4  = CppAD::thread_alloc::inuse(thread);
    ok &= inuse_3 == inuse_4;
    //
    // assigning to a temporary empty function to g
    // uses move semantics (hence frees all memory in g)
    g = CppAD::ADFun<double>();
    size_t inuse_5  = CppAD::thread_alloc::inuse(thread);
    ok &= inuse_1 == inuse_5;
    //
    return ok;
}

bool adfun_swap(void)
{   bool ok = true;
    //
    // Independent variables
    CPPAD_TESTVECTOR( CppAD::AD<double> ) ax(1);
    CppAD::Independent(ax);
    //
    // Dependent variables
    CPPAD_TESTVECTOR( CppAD::AD<double> ) ay(2);
    ay[0] = ax[0];
    ay[1] = ax[0];
    //
    // Nonempty ADFun
    CppAD::ADFun<double> f(ax, ay);
    ok &= f.size_var() != 0;
    //
    // Empry ADFun
    CppAD::ADFun<double> g;
    ok &= g.size_var() == 0;
    //
    // swap
    f.swap(g);
    ok &= f.size_var() == 0;
    ok &= g.size_var() != 0;
    //
    return ok;
}

} // END_EMPTY_NAMESPACE

bool adfun(void)
{   bool ok = true;
    ok     &= adfun_empty();
    ok     &= adfun_swap();
    return ok;
}
