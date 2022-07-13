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
$begin team_pthread.cpp$$
$spell
    Cygwin
    pthread
$$


$section Pthread Implementation of a Team of AD Threads$$
See $cref team_thread.hpp$$ for this routines specifications.

$head Bug in Cygwin$$

There is a bug in $code pthread_exit$$,
using cygwin 5.1 and g++ version 4.3.4,
whereby calling $code pthread_exit$$ is not the same as returning from
the corresponding routine.
To be specific, destructors for the vectors are not called
and a memory leaks result.
Set the following preprocessor symbol to 1 to demonstrate this bug:
$srccode%cpp% */
# define DEMONSTRATE_BUG_IN_CYGWIN 0
/* %$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <pthread.h>
# include <cppad/cppad.hpp>
# include "../team_thread.hpp"
# define MAX_NUMBER_THREADS 48

// It seems that when a barrier is passed, its counter is automatically reset
// to its original value and it can be used again, but where is this
// stated in the pthreads speicifcations ?
namespace {
    using CppAD::thread_alloc;

    // number of threads in the team
    size_t num_threads_ = 1;

    // key for accessing thread specific information
    pthread_key_t thread_specific_key_;

    // no need to destroy thread specific information
    void thread_specific_destructor(void* thread_num_vptr)
    {   return; }

    // type of the job currently being done by each thread
    enum thread_job_t { init_enum, work_enum, join_enum } thread_job_;

    // barrier used to wait for other threads to finish work
    pthread_barrier_t wait_for_work_;

    // barrier used to wait for master thread to set next job
    pthread_barrier_t wait_for_job_;

    // Are we in sequential mode; i.e., other threads are waiting for
    // master thread to set up next job ?
    bool sequential_execution_ = true;

    // structure with information for one thread
    typedef struct {
        // cppad unique identifier for thread that uses this struct
        size_t          thread_num;
        // pthread unique identifier for thread that uses this struct
        pthread_t       pthread_id;
        // true if no error for this thread, false otherwise.
        bool            ok;
    } thread_one_t;

    // vector with information for all threads
    thread_one_t thread_all_[MAX_NUMBER_THREADS];

    // pointer to function that does the work for one thread
    void (* worker_)(void) = nullptr;

    // ---------------------------------------------------------------------
    // in_parallel()
    bool in_parallel(void)
    {   return ! sequential_execution_; }

    // ---------------------------------------------------------------------
    // thread_number()
    size_t thread_number(void)
    {   // get thread specific information
        void*   thread_num_vptr = pthread_getspecific(thread_specific_key_);
        size_t* thread_num_ptr  = static_cast<size_t*>(thread_num_vptr);
        size_t  thread_num      = *thread_num_ptr;
        if( thread_num >= num_threads_ )
        {   std::cerr << "thread_number: program error" << std::endl;
            exit(1);
        }
        return thread_num;
    }
    // --------------------------------------------------------------------
    // function that gets called by pthread_create
    void* thread_work(void* thread_num_vptr)
    {   int rc;
        bool ok = true;

        // Set thread specific data where other routines can access it
        rc = pthread_setspecific(thread_specific_key_, thread_num_vptr);
        ok &= rc == 0;

        // thread_num to problem specific information for this thread
        size_t thread_num = *static_cast<size_t*>(thread_num_vptr);

        // master thread does not use this routine
        ok &= thread_num > 0;

        while( true )
        {
            // Use wait_for_job_ to give master time in sequential mode
            // (so it can change global infromation like thread_job_)
            rc = pthread_barrier_wait(&wait_for_job_);
            ok &= (rc == 0 || rc == PTHREAD_BARRIER_SERIAL_THREAD);

            // case where we are terminating this thread (no more work)
            if( thread_job_ == join_enum )
                break;

            // only other case once wait_for_job_ barrier is passed (so far)
            ok &= thread_job_ == work_enum;
            worker_();

            // Use wait_for_work_ to inform master that our work is done and
            // that this thread will not use global information until
            // passing its barrier wait_for_job_ above.
            rc = pthread_barrier_wait(&wait_for_work_);
            ok &= (rc == 0 || rc == PTHREAD_BARRIER_SERIAL_THREAD);
        }
        thread_all_[thread_num].ok = ok;
# if DEMONSTRATE_BUG_IN_CYGWIN
        // Terminate this thread
        void* no_status = nullptr;
        pthread_exit(no_status);
# endif
        return nullptr;
    }
}

bool team_create(size_t num_threads)
{   bool ok = true;;
    int rc;

    if( num_threads > MAX_NUMBER_THREADS )
    {   std::cerr << "team_create: num_threads greater than ";
        std::cerr << MAX_NUMBER_THREADS << std::endl;
        exit(1);
    }
    // check that we currently do not have multiple threads running
    ok  = num_threads_ == 1;
    ok &= sequential_execution_;

    size_t thread_num;
    for(thread_num = 0; thread_num < num_threads; thread_num++)
    {   // Each thread gets a pointer to its version of this thread_num
        // so it knows which section of thread_all_ it is working with
        thread_all_[thread_num].thread_num = thread_num;

        // initialize
        thread_all_[thread_num].ok         = true;
    }
    // Finish setup of thread_all_ for this thread
    thread_all_[0].pthread_id = pthread_self();

    // create a key for thread specific information
    rc = pthread_key_create(&thread_specific_key_,thread_specific_destructor);
    ok &= (rc == 0);

    // set thread specific information for this (master thread)
    void* thread_num_vptr = static_cast<void*>(&(thread_all_[0].thread_num));
    rc = pthread_setspecific(thread_specific_key_, thread_num_vptr);
    ok &= (rc == 0);

    // Now that thread_number() has necessary information for this thread
    // (number zero), and while still in sequential mode,
    // call setup for using CppAD::AD<double> in parallel mode.
    thread_alloc::parallel_setup(num_threads, in_parallel, thread_number);
    thread_alloc::hold_memory(true);
    CppAD::parallel_ad<double>();

    // Now change num_threads_ to its final value. Waiting till now allows
    // calls to thread_number during parallel_setup to check thread_num == 0.
    num_threads_ = num_threads;

    // initialize two barriers, one for work done, one for new job ready
    pthread_barrierattr_t* no_barrierattr = nullptr;
    rc = pthread_barrier_init(
        &wait_for_work_, no_barrierattr, (unsigned int) num_threads
    );
    ok &= (rc == 0);
    rc  = pthread_barrier_init(
        &wait_for_job_, no_barrierattr, (unsigned int) num_threads
    );
    ok &= (rc == 0);

    // structure used to create the threads
    pthread_t       pthread_id;
    // default for pthread_attr_setdetachstate is PTHREAD_CREATE_JOINABLE
    pthread_attr_t* no_attr= nullptr;

    // initial job for the threads
    thread_job_           = init_enum;
    if( num_threads > 1 )
        sequential_execution_ = false;

    // This master thread is already running, we need to create
    // num_threads - 1 more threads
    for(thread_num = 1; thread_num < num_threads; thread_num++)
    {
        // Create the thread with thread number equal to thread_num
        thread_num_vptr = static_cast<void*> (
            &(thread_all_[thread_num].thread_num)
        );
        rc = pthread_create(
                &pthread_id ,
                no_attr     ,
                thread_work ,
                thread_num_vptr
        );
        thread_all_[thread_num].pthread_id = pthread_id;
        ok &= (rc == 0);
    }

    // Current state is other threads are at wait_for_job_.
    // This master thread (thread zero) has not completed wait_for_job_
    sequential_execution_ = true;
    return ok;
}

bool team_work(void worker(void))
{   int rc;

    // Current state is other threads are at wait_for_job_.
    // This master thread (thread zero) has not completed wait_for_job_
    bool ok = sequential_execution_;
    ok     &= thread_number() == 0;

    // set global version of this work routine
    worker_ = worker;


    // set the new job that other threads are waiting for
    thread_job_ = work_enum;

    // enter parallel execution soon as master thread completes wait_for_job_
    if( num_threads_ > 1 )
        sequential_execution_ = false;

    // wait until all threads have completed wait_for_job_
    rc  = pthread_barrier_wait(&wait_for_job_);
    ok &= (rc == 0 || rc == PTHREAD_BARRIER_SERIAL_THREAD);

    // Now do the work in this thread and then wait
    // until all threads have completed wait_for_work_
    worker();
    rc = pthread_barrier_wait(&wait_for_work_);
    ok &= (rc == 0 || rc == PTHREAD_BARRIER_SERIAL_THREAD);

    // Current state is other threads are at wait_for_job_.
    // This master thread (thread zero) has not completed wait_for_job_
    sequential_execution_ = true;

    size_t thread_num;
    for(thread_num = 0; thread_num < num_threads_; thread_num++)
        ok &= thread_all_[thread_num].ok;
    return ok;
}

bool team_destroy(void)
{   int rc;

    // Current state is other threads are at wait_for_job_.
    // This master thread (thread zero) has not completed wait_for_job_
    bool ok = sequential_execution_;
    ok     &= thread_number() == 0;

    // set the new job that other threads are waiting for
    thread_job_ = join_enum;

    // Enter parallel exectuion soon as master thread completes wait_for_job_
    if( num_threads_ > 1 )
            sequential_execution_ = false;
    rc  = pthread_barrier_wait(&wait_for_job_);
    ok &= (rc == 0 || rc == PTHREAD_BARRIER_SERIAL_THREAD);

    // now wait for the other threads to exit
    size_t thread_num;
    for(thread_num = 1; thread_num < num_threads_; thread_num++)
    {   void* no_status = nullptr;
        rc      = pthread_join(
            thread_all_[thread_num].pthread_id, &no_status
        );
        ok &= (rc == 0);
    }

    // now we are down to just the master thread (thread zero)
    sequential_execution_ = true;

    // destroy the key for thread specific data
    pthread_key_delete(thread_specific_key_);

    // destroy wait_for_work_
    rc  = pthread_barrier_destroy(&wait_for_work_);
    ok &= (rc == 0);

    // destroy wait_for_job_
    rc  = pthread_barrier_destroy(&wait_for_job_);
    ok &= (rc == 0);

    // check ok before changing num_threads_
    for(thread_num = 0; thread_num < num_threads_; thread_num++)
        ok &= thread_all_[thread_num].ok;

    // now inform CppAD that there is only one thread
    num_threads_ = 1;
    thread_alloc::parallel_setup(num_threads_, nullptr, nullptr);
    thread_alloc::hold_memory(false);
    CppAD::parallel_ad<double>();

    return ok;
}

const char* team_name(void)
{   return "pthread"; }
// END C++
