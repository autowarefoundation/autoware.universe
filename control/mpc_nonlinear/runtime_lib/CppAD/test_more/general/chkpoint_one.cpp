/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

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
    typedef CPPAD_TESTVECTOR(AD<double>) ADVector;

    // ------------------------------------------------------------------------
    bool f_algo(const ADVector& x, ADVector& y)
    {   size_t m = y.size();
        assert( size_t(x.size()) == m + 1);
        for(size_t i = 0; i < m; i++)
            y[i] = x[i] * x[i+1];
        return true;
    }
    bool g_algo(const ADVector& y, ADVector& z)
    {   size_t m = z.size();
        assert( size_t(y.size()) + 1 == m );
        z[0] = 0.0;
        for(size_t i = 1; i < m; i++)
        {   z[0] += y[i-1];
            z[i]  = y[i-1];
        }
        return true;
    }
    bool test_one(void)
    {   bool ok = true;
        using CppAD::checkpoint;
        using CppAD::ADFun;
        using CppAD::NearEqual;
        size_t i, j, k, n = 4, ell = n-1 , m = ell + 1;
        double eps = 10. * std::numeric_limits<double>::epsilon();

        // checkpoint version of the function F(x)
        ADVector ax(n), ay(ell), az(m);
        for(j = 0; j < n; j++)
            ax[j] = double(j);
        checkpoint<double> f_check("f_check", f_algo, ax, ay);
        checkpoint<double> g_check("g_check", g_algo, ay, az);

        // Record a version of z = g[f(x)] without checkpointing
        Independent(ax);
        f_algo(ax, ay);
        g_algo(ay, az);
        ADFun<double> check_not(ax, az);

        // Record a version of z = g[f(x)] with checkpointing
        Independent(ax);
        f_check(ax, ay);
        g_check(ay, az);
        ADFun<double> check_yes(ax, az);

        // compare forward mode results for orders 0, 1, 2
        size_t p = 2;
        CPPAD_TESTVECTOR(double) x_p(n*(p+1)), z_not(m*(p+1)), z_yes(m*(p+1));
        for(j = 0; j < n; j++)
        {   for(k = 0; k <= p; k++)
                x_p[ j * (p+1) + k ] = 1.0 / double(p + 1 - k);
        }
        z_not = check_not.Forward(p, x_p);
        z_yes = check_yes.Forward(p, x_p);
        for(i = 0; i < m; i++)
        {   for(k = 0; k <= p; k++)
            {   double zik_not = z_not[ i * (p+1) + k];
                double zik_yes = z_yes[ i * (p+1) + k];
                ok &= NearEqual(zik_not, zik_yes, eps, eps);
            }
        }

        // compare reverse mode results
        CPPAD_TESTVECTOR(double) w(m*(p+1)), dw_not(n*(p+1)), dw_yes(n*(p+1));
        for(i = 0; i < m; i++)
        {   for(k = 0; k <= p; k++)
                w[ i * (p+1) + k ] = 2.0 / double(p + 1 - k );
        }
        dw_not = check_not.Reverse(p+1, w);
        dw_yes = check_yes.Reverse(p+1, w);
        for(j = 0; j < n; j++)
        {   for(k = 0; k <= p; k++)
            {   double dwjk_not = dw_not[ j * (p+1) + k];
                double dwjk_yes = dw_yes[ j * (p+1) + k];
                ok &= NearEqual(dwjk_not, dwjk_yes, eps, eps);
            }
        }

        // mix sparsity so test both cases
        f_check.option( CppAD::atomic_base<double>::bool_sparsity_enum );
        g_check.option( CppAD::atomic_base<double>::set_sparsity_enum );

        // compare forward mode Jacobian sparsity patterns
        size_t q = n - 1;
        CppAD::vector< std::set<size_t> > r(n), s_not(m), s_yes(m);
        for(j = 0; j < n; j++)
        {   if( j < q )
                r[j].insert(j);
            else
            {   r[j].insert(0);
                r[j].insert(1);
            }
        }
        s_not = check_not.ForSparseJac(q, r);
        s_yes = check_yes.ForSparseJac(q, r);
        for(i = 0; i < m; i++)
            ok &= s_not[i] == s_yes[i];

        // compare reverse mode Jacobian sparsity patterns
        CppAD::vector< std::set<size_t> > s(m), r_not(m), r_yes(m);
        for(i = 0; i < m; i++)
            s[i].insert(i);
        r_not = check_not.RevSparseJac(m, s);
        r_yes = check_yes.RevSparseJac(m, s);
        for(i = 0; i < m; i++)
            ok &= s_not[i] == s_yes[i];


        // compare reverse mode Hessian sparsity patterns
        CppAD::vector< std::set<size_t> > s_one(1), h_not(q), h_yes(q);
        for(i = 0; i < m; i++)
            s_one[0].insert(i);
        h_not = check_not.RevSparseHes(q, s_one);
        h_yes = check_yes.RevSparseHes(q, s_one);
        for(i = 0; i < q; i++)
            ok &= h_not[i] == h_yes[i];

        checkpoint<double>::clear();
        return ok;
    }
    // ------------------------------------------------------------------------
    bool h_algo(const ADVector& ax, ADVector& ay)
    {   ay[0] = ax[0];
        ay[1] = ax[1] + ax[2];
        return true;
    }
    bool test_two(void)
    {   bool ok = true;
        using CppAD::checkpoint;
        using CppAD::ADFun;
        using CppAD::NearEqual;

        // checkpoint version of H(x)
        size_t m = 2;
        size_t n = 3;
        ADVector ax(n), ay(m);
        for(size_t j = 0; j < n; j++)
            ax[j] = double(j);
        checkpoint<double> h_check("h_check", h_algo, ax, ay);

        // record function using h_check
        Independent(ax);
        h_check(ax, ay);
        ADFun<double> h(ax, ay);
        //
        // --------------------------------------------------------------------
        // sparsity pattern
        // --------------------------------------------------------------------
        for(size_t k = 0; k < 3; k++)
        {   if( k == 0 )
                h_check.option(CppAD::atomic_base<double>::pack_sparsity_enum);
            if( k == 1 )
                h_check.option(CppAD::atomic_base<double>::bool_sparsity_enum);
            if( k == 2 )
                h_check.option(CppAD::atomic_base<double>::set_sparsity_enum);

            // compute sparsity pattern h_1(x) = x[1] + x[2]
            CppAD::vector< std::set<size_t> > r(1), s(1);
            r[0].insert(1);
            s = h.RevSparseJac(1, r);

            // check result
            std::set<size_t> check;
            check.insert(1);
            check.insert(2);
            ok &= s[0] == check;
        }
        // --------------------------------------------------------------------
        // base2ad
        // --------------------------------------------------------------------
        ADFun< AD<double> , double > ah;
        ah = h.base2ad();
        //
        // forward mode
        ADVector au(n), av(m);
        for(size_t j = 0; j < n; j++)
            ax[j] = au[j] = double(j + 1);
        av = ah.Forward(0, au);
        h_algo(ax, ay);
        for(size_t i = 0; i < m; ++i)
            ok &= av[i] == ay[i];
        //
        // reverse mode
        ADVector adw(n), aw(m);
        for(size_t i = 0; i < m; ++i)
            aw[i] = 1.0;
        adw = ah.Reverse(1, aw);
        ok &= Value( adw[0] ) == 1.0;
        ok &= Value( adw[1] ) == 1.0;
        ok &= Value( adw[2] ) == 1.0;
        //
        return ok;
    }
}

bool chkpoint_one(void)
{   bool ok = true;
    ok  &= test_one();
    ok  &= test_two();
    return ok;
}
// END C++
