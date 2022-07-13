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

// for free_all
# include <cppad/utility/thread_alloc.hpp>

namespace {
    size_t n_ok    = 0;
    size_t n_error = 0;
    void print_test(bool ok, const char* name)
    {
        std::cout.width(20);
        std::cout.setf( std::ios_base::left );
        std::cout << name;
        //
        if( ok )
        {   std::cout << "OK\n";
            n_ok++;
        }
        else
        {   std::cout << "Error\n";
            n_error++;
        }
    }
}

// thread_alloc
double *release_thread_alloc(void);
bool      debug_thread_alloc(double* d_ptr);

// adfun_ctor
void debug_adfun_ctor(void);
void release_adfun_ctor(void);

// main program that runs all the tests
int main(void)
{   using std::cout;
    cout << "Begin test_more/debug_rel\n";
    //
    // thread_alloc
    {   double* d_ptr = release_thread_alloc();
        bool ok       = debug_thread_alloc(d_ptr);
        print_test(ok, "thead_alloc");
    }
    // adfun_ctor
    {   // this test would fail with an assert during release_adfun_ctor
        release_adfun_ctor();
        debug_adfun_ctor();
        bool ok = true;
        print_test(ok, "adfun_ctor");
    }
    // memory
    {   bool ok = CppAD::thread_alloc::free_all();
        print_test(ok, "memory");
    }
    if( n_error == 0 )
        std::cout << "All " << n_ok << " tests passed." << std::endl;
    else
        std::cout << n_error << " tests failed." << std::endl;
    //
    if( n_error == 0 )
        return 0;
    return 1;
}
