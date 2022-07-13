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
$begin colpack_hes.cpp$$
$spell
    colpack_hes
    jacobian
$$

$section ColPack: Sparse Hessian Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
bool colpack_hes(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    typedef CPPAD_TESTVECTOR(AD<double>)            a_vector;
    typedef CPPAD_TESTVECTOR(double)                d_vector;
    typedef CppAD::vector<size_t>                   i_vector;
    typedef CppAD::sparse_rc<i_vector>              sparsity;
    typedef CppAD::sparse_rcv<i_vector, d_vector>   sparse_matrix;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    //
    // domain space vector
    size_t n = 5;
    a_vector  a_x(n);
    for(size_t j = 0; j < n; j++)
        a_x[j] = AD<double> (0);
    //
    // declare independent variables and starting recording
    CppAD::Independent(a_x);

    // colpack example case where hessian is a spear head
    // i.e, H(i, j) non zero implies i = 0, j = 0, or i = j
    AD<double> sum = 0.0;
    // partial_0 partial_j = x[j]
    // partial_j partial_j = x[0]
    for(size_t j = 1; j < n; j++)
        sum += a_x[0] * a_x[j] * a_x[j] / 2.0;
    //
    // partial_i partial_i = 2 * x[i]
    for(size_t i = 0; i < n; i++)
        sum += a_x[i] * a_x[i] * a_x[i] / 3.0;

    // declare dependent variables
    size_t m = 1;
    a_vector  a_y(m);
    a_y[0] = sum;

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(a_x, a_y);

    // new value for the independent variable vector
    d_vector x(n);
    for(size_t j = 0; j < n; j++)
        x[j] = double(j + 1);

    /*
          [ 2  2  3  4  5 ]
    hes = [ 2  5  0  0  0 ]
          [ 3  0  7  0  0 ]
          [ 4  0  0  9  0 ]
          [ 5  0  0  0 11 ]
    */
    // Normally one would use CppAD to compute sparsity pattern, but for this
    // example we set it directly
    size_t nr  = n;
    size_t nc  = n;
    size_t nnz = n + 2 * (n - 1);
    sparsity pattern(nr, nc, nnz);
    for(size_t k = 0; k < n; k++)
    {   size_t r = k;
        size_t c = k;
        pattern.set(k, r, c);
    }
    for(size_t i = 1; i < n; i++)
    {   size_t k = n + 2 * (i - 1);
        size_t r = i;
        size_t c = 0;
        pattern.set(k,   r, c);
        pattern.set(k+1, c, r);
    }

    // subset of elements to compute
    // (only compute lower traingle)
    nnz = n + (n - 1);
    sparsity lower_triangle(nr, nc, nnz);
    d_vector check(nnz);
    for(size_t k = 0; k < n; k++)
    {   size_t r = k;
        size_t c = k;
        lower_triangle.set(k, r, c);
        check[k] = 2.0 * x[k];
        if( k > 0 )
            check[k] += x[0];
    }
    for(size_t j = 1; j < n; j++)
    {   size_t k = n + (j - 1);
        size_t r = 0;
        size_t c = j;
        lower_triangle.set(k, r, c);
        check[k] = x[c];
    }
    sparse_matrix subset( lower_triangle );

    // check results for both CppAD and Colpack
    for(size_t i_method = 0; i_method < 4; i_method++)
    {   // coloring method
        std::string coloring;
        switch(i_method)
        {   case 0:
            coloring = "cppad.symmetric";
            break;

            case 1:
            coloring = "cppad.general";
            break;

            case 2:
            coloring = "colpack.symmetric";
            break;

            case 3:
            coloring = "colpack.general";
            break;
        }
        //
        // compute Hessian
        CppAD::sparse_hes_work work;
        d_vector w(m);
        w[0] = 1.0;
        size_t n_sweep = f.sparse_hes(
            x, w, subset, pattern, coloring, work
        );
        //
        // check result
        const d_vector& hes( subset.val() );
        for(size_t k = 0; k < nnz; k++)
            ok &= NearEqual(check[k], hes[k], eps, eps);
        if(
            coloring == "cppad.symmetric"
        ||  coloring == "colpack.symmetric"
        )
            ok &= n_sweep == 2;
        else
            ok &= n_sweep == 5;
    }

    return ok;
}
// END C++
