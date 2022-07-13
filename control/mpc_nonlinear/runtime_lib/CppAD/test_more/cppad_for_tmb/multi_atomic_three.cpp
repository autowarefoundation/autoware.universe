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
    using CppAD::vector;
    typedef CPPAD_TESTVECTOR(double)               d_vector;
    typedef CPPAD_TESTVECTOR( CppAD::AD<double> ) ad_vector;
    typedef CppAD::ad_type_enum                   ad_type_enum;


    // --------------------------------------------------------------------
    class long_sum_atomic : public CppAD::atomic_three<double> {
    private:
        const size_t length_of_sum_;
    public:
        // constructor
        long_sum_atomic(const std::string& name, size_t length_of_sum)
        :
        CppAD::atomic_three<double>(name) ,
        length_of_sum_(length_of_sum)
        { }
        // for_type
        virtual bool for_type(
            const vector<double>&       parameter_x ,
            const vector<ad_type_enum>& type_x      ,
            vector<ad_type_enum>&       type_y      )
        {   bool ok = parameter_x.size() == 1;
            ok     &= type_x.size() == 1;
            ok     &= type_y.size() == 1;
            if( ! ok )
                return false;
            type_y[0] = type_x[0];
            return true;
        }
        // forward
        virtual bool forward(
            const vector<double>&       parameter_x ,
            const vector<ad_type_enum>& type_x      ,
            size_t                      need_y      ,
            size_t                      order_low   ,
            size_t                      order_up    ,
            const vector<double>&       taylor_x    ,
            vector<double>&             taylor_y    )
        {
            // check for errors in usage
            bool ok = order_low == 0 && order_up == 0;
            ok     &= taylor_x.size() == 1;
            ok     &= taylor_y.size() == 1;
            ok     &= type_x.size() <= 1;
            if( ! ok )
                return false;

            // value information
            taylor_y[0] = 0.0;
            for(size_t i = 0; i < length_of_sum_; ++i)
                taylor_y[0] += taylor_x[0];

            return true;
        }
    };
    // --------------------------------------------------------------------

    // inform CppAD if we are in parallel mode
    bool in_parallel(void)
    {   return omp_in_parallel() != 0; }
    //
    // inform CppAD of the current thread number
    size_t thread_num(void)
    {   return static_cast<size_t>( omp_get_thread_num() ); }

}

// multi_thread_checkpoint
bool multi_atomic_three(void)
{   bool ok = true;

    // OpenMP setup
    size_t num_threads = 4;      // number of threads
    omp_set_dynamic(0);          // turn off dynamic thread adjustment
    omp_set_num_threads( int(num_threads) );  // set number of OMP threads

    // check that multi-threading is possible on this machine
    if( omp_get_max_threads() < 2 )
    {   std::cout << "This machine does not support multi-threading: ";
    }

    // create checkpoint version of algorithm
    size_t n(1), m(1);
    ad_vector ax(n), ay(m);
    ax[0] = 2.0;
    size_t length_of_sum = 5000;
    long_sum_atomic atom_fun("long_sum", length_of_sum);

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
        atom_fun(au, av);
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
    {   double check = double( length_of_sum * (thread + 1) );
        ok          &= check == y[thread];
    }
    return ok;
}
