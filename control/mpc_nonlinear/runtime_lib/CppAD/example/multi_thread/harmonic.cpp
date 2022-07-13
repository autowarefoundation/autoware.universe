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
$begin harmonic_common$$
$spell
    inv
    num
$$

$section Common Variables Used by Multi-threading Sum of 1/i$$

$head Purpose$$
This source code defines the common
include files, defines, and variables that are used by
the summation that defines the harmonic series
$latex \[
    1 + 1/2 + 1/3 + ... + 1/n
\] $$

$head Source$$
$srcthisfile%0
    %// BEGIN COMMON C++%// END COMMON C++%
1%$$


$end
*/
// BEGIN COMMON C++


# include <cppad/cppad.hpp>
# include "harmonic.hpp"
# include "team_thread.hpp"
# define MAX_NUMBER_THREADS 48

namespace {
    using CppAD::thread_alloc; // fast multi-threadeding memory allocator

    // Number of threads, set by previous call to harmonic_time
    // (zero means one thread with no multi-threading setup)
    size_t num_threads_ = 0;

    // value of mega_sum, set by previous call to harmonic_time.
    size_t mega_sum_;

    // structure with information for one thread
    typedef struct {
        // index to start summation at (worker input)
        // set by previous call to harmonic_setup
        size_t start;
        // index to end summation at (worker input)
        // set by previous call to harmonic_setup
        size_t stop;
        // summation for this thread
        // set by worker
        double sum;
        // false if an error occurs, true otherwise
        // set by worker
        bool   ok;
    } work_one_t;

    // vector with information for all threads
    // (use pointers instead of values to avoid false sharing)
    work_one_t* work_all_[MAX_NUMBER_THREADS];
}
// END COMMON C++
/*
------------------------------------------------------------------------------
$begin harmonic_setup$$
$spell
    inv
    num
$$

$section Set Up Multi-threading Sum of 1/i$$

$head Syntax$$
$icode%ok% = harmonic_setup(%num_sum%)
%$$

$head Purpose$$
This routine does the setup for splitting
the summation that defines the harmonic series
$latex \[
    1 + 1/2 + 1/3 + ... + 1/n
\] $$
into separate parts for each thread.

$head Thread$$
It is assumed that this function is called by thread zero,
and all the other threads are blocked (waiting).

$head num_sum$$
The argument $icode num_sum$$ has prototype
$codei%
    size_t %num_sum%
%$$
It specifies the value of $latex n$$ in the summation.

$head Source$$
$srcthisfile%0
    %// BEGIN SETUP C++%// END SETUP C++%
1%$$

$end
*/
// BEGIN SETUP C++
namespace {
bool harmonic_setup(size_t num_sum)
{   // sum = 1/num_sum + 1/(num_sum-1) + ... + 1
    size_t num_threads  = std::max(num_threads_, size_t(1));
    bool ok             = num_threads == thread_alloc::num_threads();
    ok                 &= thread_alloc::thread_num() == 0;
    ok                 &= num_sum >= num_threads;
    //
    for(size_t thread_num = 0; thread_num < num_threads; thread_num++)
    {   // allocate separate memory for this thread to avoid false sharing
        size_t min_bytes(sizeof(work_one_t)), cap_bytes;
        void* v_ptr = thread_alloc::get_memory(min_bytes, cap_bytes);
        work_all_[thread_num] = static_cast<work_one_t*>(v_ptr);
        //
        // in case this thread's worker does not get called
        work_all_[thread_num]->ok = false;
        // parameters that define the work for this and previous thread
        if( thread_num == 0 )
            work_all_[0]->start = 1;
        else
        {   size_t index  = (num_sum * thread_num) / num_threads;
            work_all_[thread_num-1]->stop = index;
            work_all_[thread_num]->start  = index;
        }
    }
    work_all_[num_threads-1]->stop = num_sum + 1;
    return ok;
}
}
// END SETUP C++
/*
-------------------------------------------------------------------------------
$begin harmonic_worker$$
$spell
    inv
    num
$$

$section Do One Thread's Work for Sum of 1/i$$


$head Syntax$$
$codei%harmonic_worker()
%$$

$head Purpose$$
This routines computes the sum
the summation that defines the harmonic series
$codei%
    1/%start% + 1/(%start%+1) + ... + 1/(%end%-1)
%$$

$head start$$
This is the value of the $cref harmonic_common$$ information
$codei%
    %start% = work_all_[%thread_num%]->start
%$$

$head end$$
This is the value of the $cref harmonic_common$$ information
$codei%
    %end% = work_all_[%thread_num%]->end
%$$

$head thread_num$$
This is the number for the current thread; see
$cref/thread_num/ta_thread_num/$$.

$head Source$$
$srcthisfile%0
    %// BEGIN WORKER C++%// END WORKER C++%
1%$$

$end
*/
// BEGIN WORKER C++
namespace {
void harmonic_worker(void)
{   // sum =  1/(stop-1) + 1/(stop-2) + ... + 1/start
    size_t thread_num  = thread_alloc::thread_num();
    size_t num_threads = std::max(num_threads_, size_t(1));
    bool   ok          = thread_num < num_threads;
    size_t start       = work_all_[thread_num]->start;
    size_t stop        = work_all_[thread_num]->stop;
    double sum         = 0.;

    ok &= stop > start;
    size_t i = stop;
    while( i > start )
    {   i--;
        sum += 1. / double(i);
    }

    work_all_[thread_num]->sum = sum;
    work_all_[thread_num]->ok  = ok;
}
}
// END WORKER C++
/*
-------------------------------------------------------------------------------
$begin harmonic_takedown$$
$spell
    inv
    num
$$

$section Take Down Multi-threading Sum of 1/i$$

$head Syntax$$
$icode%ok% = harmonic_takedown(%sum%)
%$$

$head Purpose$$
This routine does the takedown for splitting
the summation that defines the harmonic series
$latex \[
    s = 1 + 1/2 + 1/3 + ... + 1/n
\] $$
into separate parts for each thread; see $cref harmonic_setup$$.

$head Thread$$
It is assumed that this function is called by thread zero,
and all the other threads have completed their work and are blocked (waiting).

$head sum$$
This argument has prototype
$codei%
    double& %sum%
%$$
The input value of the argument does not matter.
Upon return it is the value of the summation; i.e. $latex s$$.

$head Source$$
$srcthisfile%0
    %// BEGIN TAKEDOWN C++%// END TAKEDOWN C++%
1%$$

$end
*/
// BEGIN TAKEDOWN C++
namespace {
bool harmonic_takedown(double& sum)
{   // sum = 1/num_sum + 1/(num_sum-1) + ... + 1
    bool ok            = true;
    ok                &= thread_alloc::thread_num() == 0;
    size_t num_threads = std::max(num_threads_, size_t(1));
    sum                = 0.;
    //
    // go down so that free memory for other threads before memory for master
    size_t thread_num = num_threads;
    while(thread_num--)
    {   // check that this tread was ok with the work it did
        ok  &= work_all_[thread_num]->ok;
        //
        // add this threads contribution to the sum
        sum += work_all_[thread_num]->sum;
        //
        // delete problem specific information
        void* v_ptr = static_cast<void*>( work_all_[thread_num] );
        thread_alloc::return_memory( v_ptr );
        //
        // check that there is no longer any memory inuse by this thread
        // (for general applications, the master might still be using memory)
        ok &= thread_alloc::inuse(thread_num) == 0;
        //
        // return all memory being held for future use by this thread
        thread_alloc::free_available(thread_num);
    }
    return ok;
}
}
// END TAKEDOWN C++
/*
------------------------------------------------------------------------------
$begin harmonic_sum$$
$spell
    inv
    num
    bool
$$


$section Multi-Threaded Implementation of Summation of 1/i$$

$head Syntax$$
$icode%ok% = harmonic_sum(%sum%, %num_sum%)%$$

$head Purpose$$
Multi-threaded computation of the summation that defines the harmonic series
$latex \[
    s = 1 + 1/2 + 1/3 + ... + 1/n
\] $$

$head Thread$$
It is assumed that this function is called by thread zero,
and all the other threads are blocked (waiting).

$head ok$$
This return value has prototype
$codei%
    bool %ok%
%$$
If this return value is false, an error occurred during $code harmonic$$.

$head sum$$
This argument has prototype
$codei%
    double& %sum%
%$$
The input value of the argument does not matter.
Upon return it is the value of the summation; i.e. $latex s$$.

$head num_sum$$
This argument has prototype
$codei%
    size_t %num_sum%
%$$
It specifies the number of terms in the summation; i.e. $latex n$$.

$head Source$$
$srcthisfile%0
    %// BEGIN SUM C++%// END SUM C++%
1%$$

$end
*/
// BEGIN SUM C++
namespace {
bool harmonic_sum(double& sum, size_t num_sum)
{   // sum = 1/num_sum + 1/(num_sum-1) + ... + 1
    bool ok = true;
    ok     &= thread_alloc::thread_num() == 0;

    // setup the work for multi-threading
    ok &= harmonic_setup(num_sum);

    // now do the work for each thread
    if( num_threads_ > 0 )
        team_work( harmonic_worker );
    else
        harmonic_worker();

    // combine the result for each thread and takedown the multi-threading.
    ok &= harmonic_takedown(sum);

    return ok;
}
}
// END SUM C++
/*
-------------------------------------------------------------------------------
$begin harmonic_time$$
$spell
    openmp
    pthreads
    alloc
    num
    bool
    mega
    inv
    CppAD
    parallelize
$$


$section Timing Test of Multi-Threaded Summation of 1/i$$

$head Syntax$$
$icode%ok% = harmonic_time(
    %time_out%, %test_time%, %num_threads%, %mega_sum%
)%$$

$head Purpose$$
Runs a correctness and timing test for a multi-threaded
computation of the summation that defines the harmonic series
$latex \[
    1 + 1/2 + 1/3 + ... + 1/n
\] $$

$head Thread$$
It is assumed that this function is called by thread zero in sequential
mode; i.e., not $cref/in_parallel/ta_in_parallel/$$.

$head ok$$
This return value has prototype
$codei%
    bool %ok%
%$$
If it is true,
$code harmonic_time$$ passed the correctness test.
Otherwise it is false.

$head time_out$$
This argument has prototype
$codei%
    double& %time_out%
%$$
The input value of the argument does not matter.
Upon return it is the number of wall clock seconds required for
to compute the summation.

$head test_time$$
Is the minimum amount of wall clock time that the test should take.
The number of repeats for the test will be increased until this time
is reached.
The reported $icode time_out$$ is the total wall clock time divided by the
number of repeats.

$head num_threads$$
This argument has prototype
$codei%
    size_t %num_threads%
%$$
It specifies the number of threads that are available for this test.
If it is zero, the test is run without the multi-threading environment and
$codei%
    1 == thread_alloc::num_threads()
%$$
when $code harmonic_time$$ is called.
If it is non-zero, the test is run with the multi-threading and
$codei%
    %num_threads% = thread_alloc::num_threads()
%$$
when $code harmonic_time$$ is called.

$head mega_sum$$
This argument has prototype
$codei%
    size_t& %mega_sum%
%$$
and is greater than zero.
The value $latex n$$ in the summation
is equal to $latex 10^6$$ times $icode mega_sum$$.

$head Source$$
$srcthisfile%0
    %// BEGIN TIME C++%// END TIME C++%
1%$$

$end
*/
// BEGIN TIME C++
# include <cstring>
# include <limits>
# include <iostream>
# include <cstdlib>
# include <algorithm>

// Note there is no mention of parallel mode in the documentation for
// speed_test (so it is safe to use without special consideration).
# include <cppad/utility/time_test.hpp>

namespace {
    // value of sum resulting from most recent call to test_once
    double sum_ = 0.;
    //
    void test_once(void)
    {   if( mega_sum_ < 1 )
        {   std::cerr << "harmonic_time: mega_sum < 1" << std::endl;
            exit(1);
        }
        size_t num_sum = mega_sum_ * 1000000;
        bool ok = harmonic_sum(sum_, num_sum);
        if( ! ok )
        {   std::cerr << "harmonic: error" << std::endl;
            exit(1);
        }
        return;
    }
    //
    void test_repeat(size_t repeat)
    {   size_t i;
        for(i = 0; i < repeat; i++)
            test_once();
        return;
    }
}

// This is the only routine that is accessible outside of this file
bool harmonic_time(
    double& time_out, double test_time, size_t num_threads, size_t mega_sum)
{   bool ok  = true;
    ok      &= thread_alloc::thread_num() == 0;

    // arguments passed to harmonic_sum
    num_threads_ = num_threads;
    mega_sum_    = mega_sum;

    // create team of threads
    ok &= thread_alloc::in_parallel() == false;
    if( num_threads > 0 )
    {   team_create(num_threads);
        ok &= num_threads == thread_alloc::num_threads();
    }
    else
    {   ok &= 1 == thread_alloc::num_threads();
    }

    // run the test case and set the time return value
    time_out = CppAD::time_test(test_repeat, test_time);

    // destroy team of threads
    if( num_threads > 0 )
        team_destroy();
    ok &= thread_alloc::in_parallel() == false;

    // Correctness check
    double eps1000 =
        double(mega_sum_) * 1e3 * std::numeric_limits<double>::epsilon();
    size_t i       = mega_sum_ * 1000000;
    double check = 0.;
    while(i)
        check += 1. / double(i--);
    ok &= std::fabs(sum_ - check) <= eps1000;

    return ok;
}

// END TIME C++
