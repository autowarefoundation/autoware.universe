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


    // --------------------------------------------------------------------
    class long_sum_atomic : public CppAD::atomic_base<double> {
    private:
        const size_t length_of_sum_;
    public:
        // constructor
        long_sum_atomic(const std::string& name, size_t length_of_sum)
        :
        CppAD::atomic_base<double>(name) ,
        length_of_sum_(length_of_sum)
        { }
        // forward (only implement zero order)
        virtual bool forward(
            size_t                p  ,
            size_t                q  ,
            const vector<bool>&   vx ,
            vector<bool>&         vy ,
            const vector<double>& tx ,
            vector<double>&       ty )
        {
            // check for errors in usage
            bool ok = p == 0 && q == 0;
            ok     &= tx.size() == 1;
            ok     &= ty.size() == 1;
            ok     &= vx.size() <= 1;
            if( ! ok )
                return false;

            // variable information
            if( vx.size() > 0 )
                vy[0] = vx[0];

            // value information
            ty[0] = 0.0;
            for(size_t i = 0; i < length_of_sum_; ++i)
                ty[0] += tx[0];

            return ok;
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
bool multi_atomic_two(void)
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
