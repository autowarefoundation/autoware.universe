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
$begin simple_ad_openmp.cpp$$
$spell
    openmp
    CppAD
$$

$section A Simple OpenMP AD: Example and Test$$


$head Purpose$$
This example demonstrates how CppAD can be used in a
OpenMP multi-threading environment.

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
------------------------------------------------------------------------------
*/
// BEGIN C++
# include <cppad/cppad.hpp>
# include <omp.h>
# define NUMBER_THREADS  4

namespace {
    // structure with problem specific information
    typedef struct {
        // function argument (worker input)
        double          x;
        // This structure would also have return information in it,
        // but this example only returns the ok flag
    } problem_specific;
    // =====================================================================
    // General purpose code you can copy to your application
    // =====================================================================
    using CppAD::thread_alloc;
    // ------------------------------------------------------------------
    // used to inform CppAD when we are in parallel execution mode
    bool in_parallel(void)
    {   return omp_in_parallel() != 0; }
    // ------------------------------------------------------------------
    // used to inform CppAD of the current thread number
    size_t thread_number(void)
    {   return static_cast<size_t>( omp_get_thread_num() ); }
    // ------------------------------------------------------------------
    // structure with information for one thread
    typedef struct {
        // false if an error occurs, true otherwise (worker output)
        bool               ok;
    } thread_one_t;
    // vector with information for all threads
    thread_one_t thread_all_[NUMBER_THREADS];
    // ------------------------------------------------------------------
    // function that calls all the workers
    bool worker(problem_specific* info);
    bool run_all_workers(size_t num_threads, problem_specific* info_all[])
    {   bool ok = true;

        // initialize thread_all_
        int thread_num, int_num_threads = int(num_threads);
        for(thread_num = 0; thread_num < int_num_threads; thread_num++)
        {   // initialize as false to make sure gets called for all threads
            thread_all_[thread_num].ok         = false;
        }

        // turn off dynamic thread adjustment
        omp_set_dynamic(0);

        // set the number of OpenMP threads
        omp_set_num_threads( int_num_threads );

        // setup for using CppAD::AD<double> in parallel
        thread_alloc::parallel_setup(
            num_threads, in_parallel, thread_number
        );
        thread_alloc::hold_memory(true);
        CppAD::parallel_ad<double>();

        // execute worker in parallel
# pragma omp parallel for
        for(thread_num = 0; thread_num < int_num_threads; thread_num++)
            thread_all_[thread_num].ok = worker(info_all[thread_num]);
// end omp parallel for

        // set the number of OpenMP threads to one
        omp_set_num_threads(1);

        // now inform CppAD that there is only one thread
        thread_alloc::parallel_setup(1, nullptr, nullptr);
        thread_alloc::hold_memory(false);
        CppAD::parallel_ad<double>();

        // check to ok flag returned by during calls to work by other threads
        for(thread_num = 1; thread_num < int_num_threads; thread_num++)
            ok &= thread_all_[thread_num].ok;

        return ok;
    }
    // =====================================================================
    // End of General purpose code
    // =====================================================================
    // function that does the work for one thread
    bool worker(problem_specific* info)
    {   using CppAD::NearEqual;
        using CppAD::AD;
        bool ok = true;

        // CppAD::vector uses the CppAD fast multi-threading allocator
        CppAD::vector< AD<double> > ax(1), ay(1);
        ax[0] = info->x;
        Independent(ax);
        ay[0] = sqrt( ax[0] * ax[0] );
        CppAD::ADFun<double> f(ax, ay);

        // Check function value corresponds to the identity
        double eps = 10. * CppAD::numeric_limits<double>::epsilon();
        ok        &= NearEqual(ay[0], ax[0], eps, eps);

        // Check derivative value corresponds to the identity.
        CppAD::vector<double> d_x(1), d_y(1);
        d_x[0] = 1.;
        d_y    = f.Forward(1, d_x);
        ok    &= NearEqual(d_x[0], 1., eps, eps);

        return ok;
    }
}
bool simple_ad(void)
{   bool ok = true;
    size_t num_threads = NUMBER_THREADS;

    // Check that no memory is in use or avialable at start
    // (using thread_alloc in sequential mode)
    size_t thread_num;
    for(thread_num = 0; thread_num < num_threads; thread_num++)
    {   ok &= thread_alloc::inuse(thread_num) == 0;
        ok &= thread_alloc::available(thread_num) == 0;
    }

    // initialize info_all
    problem_specific *info, *info_all[NUMBER_THREADS];
    for(thread_num = 0; thread_num < num_threads; thread_num++)
    {   // problem specific information
        size_t min_bytes(sizeof(info)), cap_bytes;
        void*  v_ptr = thread_alloc::get_memory(min_bytes, cap_bytes);
        info         = static_cast<problem_specific*>(v_ptr);
        info->x      = double(thread_num) + 1.;
        info_all[thread_num] = info;
    }

    ok &= run_all_workers(num_threads, info_all);

    // go down so that free memory for other threads before memory for master
    thread_num = num_threads;
    while(thread_num--)
    {   // delete problem specific information
        void* v_ptr = static_cast<void*>( info_all[thread_num] );
        thread_alloc::return_memory( v_ptr );
        // check that there is no longer any memory inuse by this thread
        ok &= thread_alloc::inuse(thread_num) == 0;
        // return all memory being held for future use by this thread
        thread_alloc::free_available(thread_num);
    }

    return ok;
}
// END C++
