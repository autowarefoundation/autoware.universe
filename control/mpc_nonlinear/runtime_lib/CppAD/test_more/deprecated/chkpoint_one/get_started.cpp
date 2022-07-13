/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin chkpoint_one_get_started.cpp$$
$spell
    checkpointing
    Taylor
$$

$section Simple Checkpointing: Example and Test$$

$head Purpose$$
Break a large computation into pieces and only store values at the
interface of the pieces.
In actual applications, there may be many functions, but
for this example there are only two.
The functions
$latex F : \B{R}^2 \rightarrow \B{R}^2$$
and
$latex G : \B{R}^2 \rightarrow \B{R}^2$$
defined by
$latex \[
    F(y) = \left( \begin{array}{c}
        y_0 + y_0 + y_0
        \\
        y_1 + y_1 + y_1
    \end{array} \right)
    \; , \;
    G(x) = \left( \begin{array}{c}
        x_0 \cdot x_0 \cdot x_0
        \\
        x_1 \cdot x_1 \cdot x_1
    \end{array} \right)
\] $$

$comment %example/chkpoint_one/get_started.cpp%0%// BEGIN C++%// END C++%1%$$


$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace {
    using CppAD::AD;
    typedef CPPAD_TESTVECTOR(AD<double>)            ADVector;
    typedef CppAD::atomic_base<double>::option_enum option_enum;

    void f_algo(const ADVector& y, ADVector& z)
    {   z[0] = 0.0;
        z[1] = 0.0;
        for(size_t k = 0; k < 3; k++)
        {   z[0] += y[0];
            z[1] += y[1];
        }
        return;
    }
    void g_algo(const ADVector& x, ADVector& y)
    {   y[0] = 1.0;
        y[1] = 1.0;
        for(size_t k = 0; k < 3; k++)
        {   y[0] *= x[0];
            y[1] *= x[1];
        }
        return;
    }
    bool test_case(
        option_enum f_sparsity, option_enum g_sparsity, bool optimize )
    {   bool ok = true;
        using CppAD::checkpoint;
        using CppAD::ADFun;
        using CppAD::NearEqual;
        size_t i, j, k, n = 2, m = n;
        double eps = 10. * std::numeric_limits<double>::epsilon();

        // checkpoint version of the function F(x)
        ADVector ax(n), ay(n), az(m);
        for(j = 0; j < n; j++)
            ax[j] = double(j + 1);
        // could also use bool_sparsity_enum or set_sparsity_enum
        checkpoint<double> atom_f("atom_f", f_algo, ax, ay, f_sparsity);
        checkpoint<double> atom_g("atom_g", g_algo, ay, az, g_sparsity);

        // Record a version of z = g[f(x)] without checkpointing
        Independent(ax);
        f_algo(ax, ay);
        g_algo(ay, az);
        ADFun<double> check_not(ax, az);

        // Record a version of z = g[f(x)] with checkpointing
        Independent(ax);
        atom_f(ax, ay);
        atom_g(ay, az);
        ADFun<double> check_yes(ax, az);

        // checkpointing should use fewer operations
        ok &= check_yes.size_var() < check_not.size_var();

        // this does not really save space because f and g are only used once
        ok &= check_not.size_var() <=
            check_yes.size_var() + atom_f.size_var() + atom_g.size_var();

        // compare forward mode results for orders 0, 1, 2
        size_t q = 2;
        CPPAD_TESTVECTOR(double) x_q(n*(q+1)), z_not(m*(q+1)), z_yes(m*(q+1));
        for(j = 0; j < n; j++)
        {   for(k = 0; k <= q; k++)
                x_q[ j * (q+1) + k ] = 1.0 / double(q + 1 - k);
        }
        z_not = check_not.Forward(q, x_q);
        z_yes = check_yes.Forward(q, x_q);
        for(i = 0; i < m; i++)
        {   for(k = 0; k <= q; k++)
            {   double zik_not = z_not[ i * (q+1) + k];
                double zik_yes = z_yes[ i * (q+1) + k];
                ok &= NearEqual(zik_not, zik_yes, eps, eps);
            }
        }

        // compare reverse mode results
        CPPAD_TESTVECTOR(double) w(m*(q+1)), dw_not(n*(q+1)), dw_yes(n*(q+1));
        for(i = 0; i < m * (q + 1); i++)
            w[i] = 1.0 / double(i + 1);
        dw_not = check_not.Reverse(q+1, w);
        dw_yes = check_yes.Reverse(q+1, w);
        for(j = 0; j < n; j++)
        {   for(k = 0; k <= q; k++)
            {   double dwjk_not = dw_not[ j * (q+1) + k];
                double dwjk_yes = dw_yes[ j * (q+1) + k];
                ok &= NearEqual(dwjk_not, dwjk_yes, eps, eps);
            }
        }

        // compare forward mode Jacobian sparsity patterns
        CppAD::vector< std::set<size_t> > r(n), s_not(m), s_yes(m);
        for(j = 0; j < n; j++)
            r[j].insert(j);
        s_not = check_not.ForSparseJac(n, r);
        s_yes = check_yes.ForSparseJac(n, r);
        for(i = 0; i < m; i++)
            ok &= s_not[i] == s_yes[i];

        // compare reverse mode Jacobian sparsity patterns
        CppAD::vector< std::set<size_t> > s(m), r_not(m), r_yes(m);
        for(i = 0; i < m; i++)
            s[i].insert(i);
        r_not = check_not.RevSparseJac(m, s);
        r_yes = check_yes.RevSparseJac(m, s);
        for(i = 0; i < m; i++)
            ok &= r_not[i] == r_yes[i];


        // compare reverse mode Hessian sparsity patterns
        CppAD::vector< std::set<size_t> > s_one(1), h_not(n), h_yes(n);
        for(i = 0; i < m; i++)
            s_one[0].insert(i);
        h_not = check_not.RevSparseHes(n, s_one);
        h_yes = check_yes.RevSparseHes(n, s_one);
        for(i = 0; i < n; i++)
            ok &= h_not[i] == h_yes[i];

        return ok;
    }
}

bool get_started(void)
{   bool ok = true;

    // different types of sparsity
    option_enum pack_sparsity = CppAD::atomic_base<double>::pack_sparsity_enum;
    option_enum bool_sparsity = CppAD::atomic_base<double>::bool_sparsity_enum;
    option_enum set_sparsity  = CppAD::atomic_base<double>::set_sparsity_enum;

    // test some different cases
    ok &= test_case(pack_sparsity, pack_sparsity, true);
    ok &= test_case(pack_sparsity, bool_sparsity, false);
    ok &= test_case(bool_sparsity, set_sparsity,  true);
    ok &= test_case(set_sparsity,  set_sparsity,  false);

    return ok;
}
// END C++
