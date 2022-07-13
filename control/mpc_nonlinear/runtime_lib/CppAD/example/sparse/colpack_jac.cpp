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
$begin colpack_jac.cpp$$
$spell
    colpack_jac
    jacobian
$$

$section ColPack: Sparse Jacobian Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
bool colpack_jac(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    typedef CPPAD_TESTVECTOR(AD<double>)            a_vector;
    typedef CPPAD_TESTVECTOR(double)                d_vector;
    typedef CppAD::vector<size_t>                   i_vector;
    typedef CppAD::sparse_rc<i_vector>              sparsity;
    typedef CppAD::sparse_rcv<i_vector, d_vector>   sparse_matrix;

    // domain space vector
    size_t n = 4;
    a_vector  a_x(n);
    for(size_t j = 0; j < n; j++)
        a_x[j] = AD<double> (0);

    // declare independent variables and starting recording
    CppAD::Independent(a_x);

    size_t m = 3;
    a_vector  a_y(m);
    a_y[0] = a_x[0] + a_x[1];
    a_y[1] = a_x[2] + a_x[3];
    a_y[2] = a_x[0] + a_x[1] + a_x[2] + a_x[3] * a_x[3] / 2.;

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(a_x, a_y);

    // new value for the independent variable vector
    d_vector x(n);
    for(size_t j = 0; j < n; j++)
        x[j] = double(j);

    /*
          [ 1 1 0 0  ]
    jac = [ 0 0 1 1  ]
          [ 1 1 1 x_3]
    */
    // Normally one would use CppAD to compute sparsity pattern, but for this
    // example we set it directly
    size_t nr  = m;
    size_t nc  = n;
    size_t nnz = 8;
    sparsity pattern(nr, nc, nnz);
    d_vector check(nnz);
    for(size_t k = 0; k < nnz; k++)
    {   size_t r, c;
        if( k < 2 )
        {   r = 0;
            c = k;
        }
        else if( k < 4 )
        {   r = 1;
            c = k;
        }
        else
        {   r = 2;
            c = k - 4;
        }
        pattern.set(k, r, c);
        if( k == nnz - 1 )
            check[k] = x[3];
        else
            check[k] = 1.0;
    }

    // using row and column indices to compute non-zero in rows 1 and 2
    sparse_matrix subset( pattern );

    // check results for both CppAD and Colpack
    for(size_t i_method = 0; i_method < 4; i_method++)
    {   // coloring method
        std::string coloring;
        if( i_method % 2 == 0 )
            coloring = "cppad";
        else
            coloring = "colpack";
        //
        CppAD::sparse_jac_work work;
        size_t group_max = 1;
        if( i_method / 2 == 0 )
        {   size_t n_sweep = f.sparse_jac_for(
                group_max, x, subset, pattern, coloring, work
            );
            ok &= n_sweep == 4;
        }
        else
        {   size_t n_sweep = f.sparse_jac_rev(
                x, subset, pattern, coloring, work
            );
            ok &= n_sweep == 2;
        }
        const d_vector& hes( subset.val() );
        for(size_t k = 0; k < nnz; k++)
            ok &= check[k] == hes[k];
    }
    return ok;
}
// END C++
