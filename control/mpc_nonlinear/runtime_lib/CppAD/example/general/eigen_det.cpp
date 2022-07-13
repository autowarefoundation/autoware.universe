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
$begin eigen_det.cpp$$
$spell
    Eigen
$$

$section Using Eigen To Compute Determinant: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/example/cppad_eigen.hpp>
# include <cppad/speed/det_by_minor.hpp>
# include <Eigen/Dense>

bool eigen_det(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    using Eigen::Matrix;
    using Eigen::Dynamic;
    using Eigen::Index;
    //
    typedef Matrix< double     , Dynamic, Dynamic > matrix;
    typedef Matrix< AD<double> , Dynamic, Dynamic > a_matrix;
    //
    typedef CppAD::eigen_vector<double>          vector;
    typedef CppAD::eigen_vector< AD<double> >    a_vector;
    //

    // domain and range space vectors
    size_t size = 3, n  = size * size, m = 1;
    a_vector a_x(n), a_y(m);
    vector x(n);

    // set and declare independent variables and start tape recording
    for(size_t i = 0; i < size; i++)
    {   for(size_t j = 0; j < size; j++)
        {   // lower triangular matrix
            a_x[i * size + j] = x[i * size + j] = 0.0;
            if( j <= i )
                a_x[i * size + j] = x[i * size + j] = double(1 + i + j);
        }
    }
    CppAD::Independent(a_x);

    // copy independent variable vector to a matrix
    Index Size = Index(size);
    a_matrix a_X(Size, Size);
    matrix     X(Size, Size);
    for(size_t i = 0; i < size; i++)
    {   for(size_t j = 0; j < size; j++)
        {   Index I = Index(i);
            Index J = Index(j);
            X(I ,J)   = x[i * size + j];
            // If we used a_X(i, j) = X(i, j), a_X would not depend on a_x.
            a_X(I, J) = a_x[i * size + j];
        }
    }

    // Compute the log of determinant of X
    a_y[0] = log( a_X.determinant() );

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(a_x, a_y);

    // check function value
    double eps = 100. * CppAD::numeric_limits<double>::epsilon();
    CppAD::det_by_minor<double> det(size);
    ok &= NearEqual(Value(a_y[0]) , log(det(x)), eps, eps);

    // compute the derivative of y w.r.t x using CppAD
    vector jac = f.Jacobian(x);

    // check the derivative using the formula
    // d/dX log(det(X)) = transpose( inv(X) )
    matrix inv_X = X.inverse();
    for(size_t i = 0; i < size; i++)
    {   for(size_t j = 0; j < size; j++)
        {   Index I = Index(i);
            Index J = Index(j);
            ok &= NearEqual(jac[i * size + j], inv_X(J, I), eps, eps);
        }
    }

    return ok;
}

// END C++
