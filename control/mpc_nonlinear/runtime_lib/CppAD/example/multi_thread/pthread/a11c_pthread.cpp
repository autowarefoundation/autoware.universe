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
$begin a11c_pthread.cpp$$
$spell
    pthread
    pthreads
    CppAD
    const
$$

$section A Simple Parallel Pthread Example and Test$$

$head Purpose$$
This example just demonstrates pthreads and does not use CppAD at all.

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$
$end
----------------------------------------------------------------------------
*/
// BEGIN C++
# include <pthread.h>
# include <limits>
# include <cmath>
# include <cassert>
// for size_t
# include <cstddef>

# define NUMBER_THREADS 4

# ifdef NDEBUG
# define CHECK_ZERO(expression) expression
# else
# define CHECK_ZERO(expression) assert( expression == 0 );
# endif
namespace {
    // Beginning of Example A.1.1.1c of OpenMP 2.5 standard document ---------
    void a1(int n, float *a, float *b)
    {   int i;
        // for some reason this function is missing on some systems
        // assert( pthread_is_multithreaded_np() > 0 );
        for(i = 1; i < n; i++)
            b[i] = (a[i] + a[i-1]) / 2.0f;
        return;
    }
    // End of Example A.1.1.1c of OpenMP 2.5 standard document ---------------
    struct start_arg { int  n; float* a; float* b; };
    void* start_routine(void* arg_vptr)
    {   start_arg* arg = static_cast<start_arg*>( arg_vptr );
        a1(arg->n, arg->a, arg->b);

        void* no_status = nullptr;
        pthread_exit(no_status);

        return no_status;
    }
}

bool a11c(void)
{   bool ok = true;

    // Test setup
    int i, j, n_total = 10;
    float *a = new float[size_t(n_total)];
    float *b = new float[size_t(n_total)];
    for(i = 0; i < n_total; i++)
        a[i] = float(i);

    // number of threads
    int n_thread = NUMBER_THREADS;
    // the threads
    pthread_t thread[NUMBER_THREADS];
    // arguments to start_routine
    struct start_arg arg[NUMBER_THREADS];
    // attr
    pthread_attr_t attr;
    CHECK_ZERO( pthread_attr_init( &attr ) );
    CHECK_ZERO( pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE) );
    //
    // Break the work up into sub work for each thread
    int n = n_total / n_thread;
    arg[0].n = n;
    arg[0].a = a;
    arg[0].b = b;
    for(j = 1; j < n_thread; j++)
    {   arg[j].n = n + 1;
        arg[j].a = arg[j-1].a + n - 1;
        arg[j].b = arg[j-1].b + n - 1;
        if( j == (n_thread - 1) )
            arg[j].n = n_total - j * n + 1;
    }
    for(j = 0; j < n_thread; j++)
    {   // inform each thread of which block it is working on
        void* arg_vptr = static_cast<void*>( &arg[j] );
        CHECK_ZERO( pthread_create(
            &thread[j], &attr, start_routine, arg_vptr
        ) );
    }
    for(j = 0; j < n_thread; j++)
    {   void* no_status = nullptr;
        CHECK_ZERO( pthread_join(thread[j], &no_status) );
    }

    // check the result
    float eps = 100.0f * std::numeric_limits<float>::epsilon();
    for(i = 1; i < n ; i++)
        ok &= std::fabs( (2. * b[i] - a[i] - a[i-1]) / b[i] ) <= eps;

    delete [] a;
    delete [] b;

    return ok;
}
// END C++
