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

namespace {
    using CppAD::AD;
    using CppAD::vector;
    using CppAD::chkpoint_two;
    // -----------------------------------------------------------------------
    template <class Algo>
    chkpoint_two<double> checkpoint_two(
        std::string name ,
        Algo&      algo  ,
        vector< AD<double> >& ax ,
        vector< AD<double> >& ay )
    {   CppAD::Independent(ax);
        algo(ax, ay);
        CppAD::ADFun<double> g_fun(ax, ay);
        bool internal_bool = false;
        bool use_hes_sparsity = true;
        bool use_base2ad      = true;
        bool use_in_parallel  = false;
        return chkpoint_two<double>(g_fun, name,
            internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
        );
    }
    // ----------------------------------------------------------------
    // Test for bug where checkpoint function did not depend on
    // the operands in the logical comparison because of the CondExp
    // sparsity pattern.
    void j_algo(
        const CppAD::vector< CppAD::AD<double> >& ax ,
              CppAD::vector< CppAD::AD<double> >& ay )
    {   ay[0] = CondExpGt(ax[0], ax[1], ax[2], ax[3]); }

    bool test_one(void)
    {   bool ok = true;
        using CppAD::NearEqual;
        double eps10 = 10.0 * std::numeric_limits<double>::epsilon();

        // Create a checkpoint version of the function g
        vector< AD<double> > au(4), av(1);
        for(size_t i = 0; i < 4; i++)
            au[i] = AD<double>(i);
        chkpoint_two<double> j_check =
            checkpoint_two("j_check", j_algo, au, av);

        // independent variable vector
        vector< AD<double> > ax(2), ay(1);
        ax[0] = 1.;
        ax[1] = 1.;
        Independent(ax);

        // call atomic function that does not get used
        for(size_t i = 0; i < 4; i++)
            au[i] = ax[0] + AD<double>(i + 1) * ax[1];
        j_check(au, ay);

        // create function object f : ax -> ay
        CppAD::ADFun<double> f(ax, ay);


        // now optimize the operation sequence
        f.optimize();

        // check result where true case is used; i.e., au[0] > au[1]
        vector<double> x(2), y(1);
        x[0] = 1.;
        x[1] = -1;
        y    = f.Forward(0, x);
        ok &= NearEqual(y[0], x[0] + double(3) * x[1], eps10, eps10);


        // check result where false case is used; i.e., au[0] <= au[1]
        x[0] = 1.;
        x[1] = 1;
        y    = f.Forward(0, x);
        ok &= NearEqual(y[0], x[0] + double(4) * x[1], eps10, eps10);

        return ok;
    }
    // -------------------------------------------------------------------
    // Test conditional optimizing out call to an atomic function call
    void k_algo(
        const CppAD::vector< CppAD::AD<double> >& x ,
              CppAD::vector< CppAD::AD<double> >& y )
    {   y[0] = x[0] + x[1]; }

    void h_algo(
        const CppAD::vector< CppAD::AD<double> >& x ,
              CppAD::vector< CppAD::AD<double> >& y )
    {   y[0] = x[0] - x[1]; }

    bool test_two(void)
    {   bool ok = true;
        typedef vector< AD<double> > ADVector;
        using CppAD::NearEqual;
        double eps10 = 10.0 * std::numeric_limits<double>::epsilon();

        // Create a checkpoint version of the function g
        ADVector ax(2), ag(1), ah(1), ay(1);
        ax[0] = 0.;
        ax[1] = 1.;
        chkpoint_two<double> k_check =
            checkpoint_two("k_check", k_algo, ax, ag);
        chkpoint_two<double> h_check =
            checkpoint_two("h_check", h_algo, ax, ah);

        // independent variable vector
        Independent(ax);

        // atomic function calls that get conditionally used
        k_check(ax, ag);
        h_check(ax, ah);

        // conditional expression
        ay[0] = CondExpLt(ax[0], ax[1], ag[0], ah[0]);

        // create function object f : ax -> ay
        CppAD::ADFun<double> f;
        f.Dependent(ax, ay);

        // use zero order to evaluate when condition is true
        CppAD::vector<double>  x(2), dx(2);
        CppAD::vector<double>  y(1), dy(1), w(1);
        x[0] = 3.;
        x[1] = 4.;
        y    = f.Forward(0, x);
        ok &= NearEqual(y[0], x[0] + x[1], eps10, eps10);

        // before optimize
        ok  &= f.number_skip() == 0;

        // now optimize the operation sequence
        f.optimize();

        // optimized zero order forward when condition is false
        x[0] = 4.;
        x[1] = 3.;
        y    = f.Forward(0, x);
        ok &= NearEqual(y[0], x[0] - x[1], eps10, eps10);

        // after optimize can skip either call to g or call to h
        ok  &= f.number_skip() == 1;

        // optimized first order forward
        dx[0] = 2.;
        dx[1] = 1.;
        dy    = f.Forward(1, dx);
        ok &= NearEqual(dy[0], dx[0] - dx[1], eps10, eps10);

        // optimized first order reverse
        w[0]  = 1.;
        dx    = f.Reverse(1, w);
        ok &= NearEqual(dx[0], 1., eps10, eps10);
        ok &= NearEqual(dx[1], -1., eps10, eps10);

        return ok;
    }
    // -------------------------------------------------------------------
    // Test of optimizing out arguments to an atomic function
    void g_algo(
        const CppAD::vector< CppAD::AD<double> >& ax ,
              CppAD::vector< CppAD::AD<double> >& ay )
    {   ay = ax; }

    bool test_three(void)
    {   bool ok = true;
        using CppAD::NearEqual;
        double eps10 = 10.0 * std::numeric_limits<double>::epsilon();

        // Create a checkpoint version of the function g
        vector< AD<double> > ax(2), ay(2), az(1);
        ax[0] = 0.;
        ax[1] = 1.;
        chkpoint_two<double> g_check =
            checkpoint_two("g_check", g_algo, ax, ay);

        // independent variable vector
        Independent(ax);

        // call atomic function that does not get used
        g_check(ax, ay);

        // conditional expression
        az[0] = CondExpLt(ax[0], ax[1], ax[0] + ax[1], ax[0] - ax[1]);

        // create function object f : ax -> az
        CppAD::ADFun<double> f(ax, az);

        // number of variables before optimization
        // (include ay[0] and ay[1])
        size_t n_before = f.size_var();

        // now optimize the operation sequence
        f.optimize();

        // number of variables after optimization
        // (does not include ay[0] and ay[1])
        size_t n_after = f.size_var();
        ok            &= n_after + 2 == n_before;

        // check optimization works ok
        vector<double> x(2), z(1);
        x[0] = 4.;
        x[1] = 3.;
        z    = f.Forward(0, x);
        ok &= NearEqual(z[0], x[0] - x[1], eps10, eps10);

        return ok;
    }
    bool test_four(void)
    {   bool ok = true;
        using CppAD::NearEqual;
        double eps10 = 10.0 * std::numeric_limits<double>::epsilon();
        vector< AD<double> > au(2), aw(2), ax(2), ay(1);

        // create atomic function corresponding to g_algo
        au[0] = 1.0;
        au[1] = 2.0;
        chkpoint_two<double> g_check =
            checkpoint_two("g_algo", g_algo, au, ax);

        // start recording a new function
        CppAD::Independent(ax);

        // now use g_check during the recording
        au[0] = ax[0] + ax[1]; // this argument requires a new variable
        au[1] = ax[0] - ax[1]; // this argument also requires a new variable
        g_check(au, aw);

        // now create f(x) = x_0 + x_1
        ay[0] = aw[0];
        CppAD::ADFun<double> f(ax, ay);

        // number of variables before optimization
        // ax[0], ax[1], ax[0] + ax[1], ax[0] - ax[1], g[0], g[1]
        // and phantom variable at index 0
        size_t n_before = f.size_var();
        ok &= n_before == 7;

        // now optimize f so that the calculation of au[1] is removed
        f.optimize();

        // number of varialbes after optimization
        // ax[0], ax[1], ax[0] + ax[1], g[0]
        // and phantom variable at index 0
        size_t n_after = f.size_var();
        ok &= n_after == 5;

        // now compute and check a forward mode calculation
        vector<double> x(2), y(1);
        x[0] = 5.0;
        x[1] = 6.0;
        y    = f.Forward(0, x);
        ok &= NearEqual(y[0], x[0] + x[1], eps10, eps10);

        return ok;
    }
    // -----------------------------------------------------------------------
    void i_algo(
        const CppAD::vector< CppAD::AD<double> >& ax ,
              CppAD::vector< CppAD::AD<double> >& ay )
    {   ay[0] = 1.0 / ax[0]; }
    //
    // Test bug where atomic functions were not properly conditionally skipped.
    bool test_five(bool conditional_skip)
    {   bool ok = true;
        using CppAD::AD;
        using CppAD::NearEqual;
        double eps10 = 10.0 * std::numeric_limits<double>::epsilon();
        using CppAD::vector;

        // Create a checkpoint version of the function i_algo
        vector< AD<double> > au(1), av(1), aw(1);
        au[0] = 1.0;
        chkpoint_two<double> i_check =
            checkpoint_two("i_check", i_algo, au, av);

        // independent variable vector
        vector< AD<double> > ax(2), ay(1);
        ax[0] = 1.0;
        ax[1] = 2.0;
        Independent(ax);

        // call atomic function that does not get used
        au[0] = ax[0];
        i_check(au, av);
        au[0] = ax[1];
        i_check(au, aw);
        AD<double> zero = 0.0;
        ay[0] = CondExpGt(av[0], zero, av[0], aw[0]);

        // create function object f : ax -> ay
        CppAD::ADFun<double> f(ax, ay);

        // run case that skips the second call to afun
        // (can use trace in forward0sweep.hpp to see this).
        vector<double> x(2), y_before(1), y_after(1);
        x[0]      = 1.0;
        x[1]      = 2.0;
        y_before  = f.Forward(0, x);
        if( conditional_skip )
            f.optimize();
        else
            f.optimize("no_conditional_skip");
        y_after   = f.Forward(0, x);

        ok &= NearEqual(y_before[0], y_after[0], eps10, eps10);

        return ok;
    }
    // -----------------------------------------------------------------------
    void m_algo(
        const CppAD::vector< CppAD::AD<double> >& ax ,
              CppAD::vector< CppAD::AD<double> >& ay )
    {   ay[0] = 0.0;
        for(size_t j = 0; j < ax.size(); ++j)
            ay[0] += ax[j] * ax[j];
    }
    //
    // Test bug where select_y[i] should be select_x[i]
    bool test_six(void)
    {   bool ok = true;
        using CppAD::AD;
        using CppAD::NearEqual;
        using CppAD::vector;

        // Create a checkpoint version of the function m_algo
        size_t n = 3, m = 1;
        vector< AD<double> > ax(n), ay(m);
        for(size_t j = 0; j < n; ++j)
            ax[j] = 1.0;
        chkpoint_two<double> m_check =
            checkpoint_two("m_check", m_algo, ax, ay);

        // independent variable vector
        Independent(ax);

        // call atomic function that does not get used
        m_check(ax, ay);

        // create function object f : ax -> ay
        CppAD::ADFun<double> f(ax, ay);

        // Evaluate Hessian sparsity
        vector<bool> select_domain(n), select_range(m);
        select_range[0] = true;
        for(size_t j = 0; j < n; ++j)
            select_domain[j] = true;
        bool internal_bool = true;
        CppAD::sparse_rc< vector<size_t> > pattern_out;
        //
        f.for_hes_sparsity(
            select_domain, select_range, internal_bool, pattern_out
        );
        size_t nnz = pattern_out.nnz();
        const vector<size_t>& row( pattern_out.row() );
        const vector<size_t>& col( pattern_out.col() );
        vector<size_t> row_major = pattern_out.row_major();
        //
        ok &= nnz == n;
        for(size_t k = 0; k < nnz; ++k)
        {   ok &= row[ row_major[k] ] == k;
            ok &= col[ row_major[k] ] == k;
        }
        return ok;
    }

}
bool chkpoint_two(void)
{   bool ok = true;
    //
    ok  &= test_one();
    ok  &= test_two();
    ok  &= test_three();
    ok  &= test_four();
    ok  &= test_five(true);
    ok  &= test_five(false);
    ok  &= test_six();
    //
    return ok;
}
// END C++
