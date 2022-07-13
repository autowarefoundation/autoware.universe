/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/cppad.hpp>
# include <omp.h>

namespace {

    typedef CPPAD_TESTVECTOR(double)               d_vector;
    typedef CPPAD_TESTVECTOR( CppAD::AD<double> ) ad_vector;


    // algorithm that we are checkpoingint
    const size_t length_of_sum_ = 5000;
    void long_sum_algo(const ad_vector& ax, ad_vector& ay)
    {   ay[0] = 0.0;
        for(size_t i = 0; i < length_of_sum_; ++i)
            ay[0] += ax[0];
        return;
    }
    // inform CppAD if we are in parallel mode
    bool in_parallel(void)
    {   return omp_in_parallel() != 0; }
    //
    // inform CppAD of the current thread number
    size_t thread_num(void)
    {   return static_cast<size_t>( omp_get_thread_num() ); }

}

// multi_thread_chkpoint_two
bool multi_chkpoint_two(void)
{   bool ok = true;

    // OpenMP setup
    size_t num_threads = 4;      // number of threads
    omp_set_dynamic(0);          // turn off dynamic thread adjustment
    omp_set_num_threads( int(num_threads) );  // set number of OMP threads

    // check that multi-threading is possible on this machine
    if( omp_get_max_threads() < 2 )
    {   std::cout << "This machine does not support multi-threading: ";
    }

    // create ADFun corresponding to long_sum_algo
    size_t n(1), m(1);
    ad_vector ax(n), ay(m);
    ax[0] = 2.0;
    CppAD::Independent(ax);
    long_sum_algo(ax, ay);
    CppAD::ADFun<double> fun(ax, ay);

    // create chkpoint_two version of algorithm
    const char* name      = "long_sum";
    bool internal_bool    = false;
    bool use_hes_sparsity = false;
    bool use_base2ad      = false;
    bool use_in_parallel  = true;
    CppAD::chkpoint_two<double> chk_fun( fun, name,
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );
    // setup for using CppAD in paralle mode
    CppAD::thread_alloc::parallel_setup(num_threads, in_parallel, thread_num);
    CppAD::thread_alloc::hold_memory(true);
    CppAD::parallel_ad<double>();

    // place to hold result for each thread
    d_vector y(num_threads);
    for(size_t thread = 0; thread < num_threads; thread++)
        y[thread] = 0.0;

    # pragma omp parallel for
    for(int thread = 0; thread < int(num_threads); thread++)
    {   ad_vector au(n), av(m);
        au[0] = 1.0;
        CppAD::Independent(au);
        chk_fun(au, av);
        CppAD::ADFun<double> f(au, av);
        //
        d_vector x(n), v(m);
        x[0]      = double( thread + 1 );
        v         = f.Forward(0, x);
        //
        // this assigment has false sharing; i.e., will case cache resets
        // (conversion avoids boost vector conversion warning)
        y[size_t(thread)] = v[0];
    }

    // check the results
    for(size_t thread = 0; thread < num_threads; thread++)
    {   double check = double( length_of_sum_ * (thread + 1) );
        ok          &= check == y[thread];
    }
    return ok;
}
