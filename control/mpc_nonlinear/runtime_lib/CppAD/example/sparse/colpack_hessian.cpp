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
$begin colpack_hessian.cpp$$
$spell
    colpack_hessian
    jacobian
$$

$section ColPack: Sparse Hessian Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
bool colpack_hessian(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    typedef CPPAD_TESTVECTOR(AD<double>) a_vector;
    typedef CPPAD_TESTVECTOR(double)     d_vector;
    typedef CppAD::vector<size_t>        i_vector;
    size_t i, j, k, ell;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 5;
    a_vector  a_x(n);
    for(j = 0; j < n; j++)
        a_x[j] = AD<double> (0);

    // declare independent variables and starting recording
    CppAD::Independent(a_x);

    // colpack example case where hessian is a spear head
    // i.e, H(i, j) non zero implies i = 0, j = 0, or i = j
    AD<double> sum = 0.0;
    // partial_0 partial_j = x[j]
    // partial_j partial_j = x[0]
    for(j = 1; j < n; j++)
        sum += a_x[0] * a_x[j] * a_x[j] / 2.0;
    //
    // partial_i partial_i = 2 * x[i]
    for(i = 0; i < n; i++)
        sum += a_x[i] * a_x[i] * a_x[i] / 3.0;

    // declare dependent variables
    size_t m = 1;
    a_vector  a_y(m);
    a_y[0] = sum;

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(a_x, a_y);

    // new value for the independent variable vector
    d_vector x(n);
    for(j = 0; j < n; j++)
        x[j] = double(j + 1);

    /*
          [ 2  2  3  4  5 ]
    hes = [ 2  5  0  0  0 ]
          [ 3  0  7  0  0 ]
          [ 4  0  0  9  0 ]
          [ 5  0  0  0 11 ]
    */
    d_vector check(n * n);
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
        {   size_t index = i * n + j;
            check[index] = 0.0;
            if( i == 0 && 1 <= j )
                check[index] += x[j];
            if( 1 <= i && j == 0 )
                check[index] += x[i];
            if( i == j )
            {   check[index] += 2.0 * x[i];
                if( i != 0 )
                    check[index] += x[0];
            }
        }
    }
    // Normally one would use f.RevSparseHes to compute
    // sparsity pattern, but for this example we extract it from check.
    std::vector< std::set<size_t> >  p(n);
    i_vector row, col;
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
        {   ell = i * n + j;
            if( check[ell] != 0. )
            {   // insert this non-zero entry in sparsity pattern
                p[i].insert(j);

                // the Hessian is symmetric, so only lower triangle
                if( j <= i )
                {   row.push_back(i);
                    col.push_back(j);
                }
            }
        }
    }
    size_t K = row.size();
    d_vector hes(K);

    // default coloring method is cppad.symmetric
    CppAD::sparse_hessian_work work;
    ok &= work.color_method == "cppad.symmetric";

    // contrast and check results for both CppAD and Colpack
    for(size_t i_method = 0; i_method < 4; i_method++)
    {   // empty work structure
        switch(i_method)
        {   case 0:
            work.color_method = "cppad.symmetric";
            break;

            case 1:
            work.color_method = "cppad.general";
            break;

            case 2:
            work.color_method = "colpack.symmetric";
            break;

            case 3:
            work.color_method = "colpack.general";
            break;
        }

        // compute Hessian
        d_vector w(m);
        w[0] = 1.0;
        size_t n_sweep = f.SparseHessian(x, w, p, row, col, hes, work);
        //
        // check result
        for(k = 0; k < K; k++)
        {   ell = row[k] * n + col[k];
            ok &= NearEqual(check[ell], hes[k], eps, eps);
        }
        if(
            work.color_method == "cppad.symmetric"
        ||  work.color_method == "colpack.symmetric"
        )
            ok &= n_sweep == 2;
        else
            ok &= n_sweep == 5;
        //
        // check that clear resets color_method to cppad.symmetric
        work.clear();
        ok &= work.color_method == "cppad.symmetric";
    }

    return ok;
}
// END C++
