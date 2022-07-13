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
$begin team_bthread.cpp$$
$spell
    bthread
$$


$section Boost Thread Implementation of a Team of AD Threads$$
See $cref team_thread.hpp$$ for this routines specifications.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <boost/thread.hpp>
# include <cppad/cppad.hpp>
# include "../team_thread.hpp"
# define MAX_NUMBER_THREADS 48

namespace {
    using CppAD::thread_alloc;

    // number of threads in the team
    size_t num_threads_ = 1;

    // no need to cleanup up thread specific data
    void cleanup(size_t*)
    {   return; }

    // thread specific pointer the thread number (initialize as null)
    boost::thread_specific_ptr<size_t> thread_num_ptr_(cleanup);

    // type of the job currently being done by each thread
    enum thread_job_t { init_enum, work_enum, join_enum } thread_job_;

    // barrier used to wait for other threads to finish work
    boost::barrier* wait_for_work_ = nullptr;

    // barrier used to wait for master thread to set next job
    boost::barrier* wait_for_job_ = nullptr;

    // Are we in sequential mode; i.e., other threads are waiting for
    // master thread to set up next job ?
    bool sequential_execution_ = true;

    // structure with information for one thread
    typedef struct {
        // The thread
        boost::thread*       bthread;
        // CppAD thread number as global (pointed to by thread_num_ptr_)
        size_t               thread_num;
        // true if no error for this thread, false otherwise.
        bool                 ok;
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
    {   // return thread_all_[thread_num].thread_num
        return *thread_num_ptr_.get();
    }
    // --------------------------------------------------------------------
    // function that gets called by boost thread constructor
    void thread_work(size_t thread_num)
    {   bool ok = wait_for_work_ != nullptr;
        ok     &= wait_for_job_  != nullptr;
        ok     &= thread_num     != 0;

        // thread specific storage of thread number for this thread
        thread_num_ptr_.reset(& thread_all_[thread_num].thread_num );

        while( true )
        {
            // Use wait_for_jog_ to give master time in sequential mode
            // (so it can change global information like thread_job_)
            wait_for_job_->wait();

            // case where we are terminating this thread (no more work)
            if( thread_job_ == join_enum)
                break;

            // only other case once wait_for_job_ has been completed (so far)
            ok &= thread_job_ == work_enum;
            worker_();

            // Use wait_for_work_ to inform master that our work is done and
            // that this thread will not use global infromation until
            // passing its barrier wait_for_job_ above.
            wait_for_work_->wait();

        }
        thread_all_[thread_num].ok &= ok;
        return;
    }
}

bool team_create(size_t num_threads)
{   bool ok = true;;

    if( num_threads > MAX_NUMBER_THREADS )
    {   std::cerr << "team_create: num_threads greater than ";
        std::cerr << MAX_NUMBER_THREADS << std::endl;
        exit(1);
    }
    // check that we currently do not have multiple threads running
    ok  = num_threads_ == 1;
    ok &= wait_for_work_ == nullptr;
    ok &= wait_for_job_  == nullptr;
    ok &= sequential_execution_;

    size_t thread_num;
    for(thread_num = 0; thread_num < num_threads; thread_num++)
    {   // Each thread gets a pointer to its version of this thread_num
        // so it knows which section of thread_all it is working with
        thread_all_[thread_num].thread_num = thread_num;

        // initialize
        thread_all_[thread_num].ok = true;
        thread_all_[0].bthread     = nullptr;
    }
    // Finish setup of thread_all_ for this thread
    thread_num_ptr_.reset(& thread_all_[0].thread_num);

    // Now that thread_number() has necessary information for the case
    // num_threads_ == 1, and while still in sequential mode,
    // call setup for using CppAD::AD<double> in parallel mode.
    thread_alloc::parallel_setup(num_threads, in_parallel, thread_number);
    thread_alloc::hold_memory(true);
    CppAD::parallel_ad<double>();

    // now change num_threads_ to its final value.
    num_threads_ = num_threads;

    // initialize two barriers, one for work done, one for new job ready
    wait_for_work_ = new boost::barrier( (unsigned int) num_threads );
    wait_for_job_  = new boost::barrier( (unsigned int) num_threads );

    // initial job for the threads
    thread_job_           = init_enum;
    if( num_threads > 1 )
        sequential_execution_ = false;

    // This master thread is already running, we need to create
    // num_threads - 1 more threads
    for(thread_num = 1; thread_num < num_threads; thread_num++)
    {   // Create the thread with thread number equal to thread_num
        thread_all_[thread_num].bthread =
            new boost::thread(thread_work, thread_num);
    }

    // Current state is other threads are at wait_for_job_.
    // This master thread (thread zero) has not completed wait_for_job_
    sequential_execution_ = true;
    return ok;
}

bool team_work(void worker(void))
{
    // Current state is other threads are at wait_for_job_.
    // This master thread (thread zero) has not completed wait_for_job_
    bool ok = sequential_execution_;
    ok     &= thread_number() == 0;
    ok     &= wait_for_work_  != nullptr;
    ok     &= wait_for_job_   != nullptr;

    // set global version of this work routine
    worker_ = worker;

    // set the new job that other threads are waiting for
    thread_job_ = work_enum;

    // Enter parallel exectuion when master thread calls wait_for_job_
    if( num_threads_ > 1 )
        sequential_execution_ = false;
    wait_for_job_->wait();

    // Now do the work in this thread and then wait
    // until all threads have completed wait_for_work_
    worker();
    wait_for_work_->wait();

    // Current state is other threads are at wait_for_job_.
    // This master thread (thread zero) has not completed wait_for_job_
    sequential_execution_ = true;

    size_t thread_num;
    for(thread_num = 0; thread_num < num_threads_; thread_num++)
        ok &= thread_all_[thread_num].ok;
    return ok;
}

bool team_destroy(void)
{   // Current state is other threads are at wait_for_job_.
    // This master thread (thread zero) has not completed wait_for_job_
    bool ok = sequential_execution_;
    ok     &= thread_number() == 0;
    ok     &= wait_for_work_ != nullptr;
    ok     &= wait_for_job_  != nullptr;

    // set the new job that other threads are waiting for
    thread_job_ = join_enum;

    // enter parallel exectuion soon as master thread completes wait_for_job_
    if( num_threads_ > 1 )
            sequential_execution_ = false;
    wait_for_job_->wait();

    // now wait for the other threads to be destroyed
    size_t thread_num;
    ok &= thread_all_[0].bthread == nullptr;
    for(thread_num = 1; thread_num < num_threads_; thread_num++)
    {   thread_all_[thread_num].bthread->join();
        delete thread_all_[thread_num].bthread;
        thread_all_[thread_num].bthread = nullptr;
    }
    // now we are down to just the master thread (thread zero)
    sequential_execution_ = true;

    // destroy wait_for_work_
    delete wait_for_work_;
    wait_for_work_ = nullptr;

    // destroy wait_for_job_
    delete wait_for_job_;
    wait_for_job_ = nullptr;

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
{   return "bthread"; }
// END C++
