/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/cppad.hpp>

// checks that d_ptr points to a double with value 5.0 and then
// frees the double using thread_alloc::return_memory
bool debug_thread_alloc(double *d_ptr)
{   bool ok = true;
    ok &= ( *d_ptr == 5.0);
    void* v_ptr = reinterpret_cast<void*>(d_ptr);
    CppAD::thread_alloc::return_memory(v_ptr);
    return ok;
}

// just use ADFun<double> constructor
void debug_adfun_ctor(void)
{
    CppAD::vector< CppAD::AD<double> > ax(1), ay(1);
    ax[0] = 0.;
    CppAD::Independent(ax);
    ay[0] = fabs(ax[0]);
    CppAD::ADFun<double> f(ax, ay);
    return;
}
