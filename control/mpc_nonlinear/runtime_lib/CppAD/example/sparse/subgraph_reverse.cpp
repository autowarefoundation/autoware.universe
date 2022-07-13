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
$begin subgraph_reverse.cpp$$
$spell
    Subgraphs
$$

$section Computing Reverse Mode on Subgraphs: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
bool subgraph_reverse(void)
{   bool ok = true;
    //
    using CppAD::AD;
    using CppAD::NearEqual;
    using CppAD::sparse_rc;
    using CppAD::sparse_rcv;
    //
    typedef CPPAD_TESTVECTOR(AD<double>) a_vector;
    typedef CPPAD_TESTVECTOR(double)     d_vector;
    typedef CPPAD_TESTVECTOR(bool)       b_vector;
    typedef CPPAD_TESTVECTOR(size_t)     s_vector;
    //
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    // domain space vector
    size_t n = 4;
    a_vector  a_x(n);
    for(size_t j = 0; j < n; j++)
        a_x[j] = AD<double> (0);
    //
    // declare independent variables and starting recording
    CppAD::Independent(a_x);
    //
    size_t m = 3;
    a_vector  a_y(m);
    a_y[0] = a_x[0] + a_x[1];
    a_y[1] = a_x[2] + a_x[3];
    a_y[2] = a_x[0] + a_x[1] + a_x[2] + a_x[3] * a_x[3] / 2.;
    //
    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(a_x, a_y);
    ok &= f.size_random() == 0;
    //
    // new value for the independent variable vector
    d_vector x(n);
    for(size_t j = 0; j < n; j++)
        x[j] = double(j);
    f.Forward(0, x);
    /*
           [ 1 1 0 0  ]
    J(x) = [ 0 0 1 1  ]
           [ 1 1 1 x_3]
    */
    double J[] = {
        1.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 0.0
    };
    J[11] = x[3];
    //
    // exclude x[0] from the calculations
    b_vector select_domain(n);
    select_domain[0] = false;
    for(size_t j = 1; j < n; j++)
        select_domain[j] = true;
    //
    // initilaize for reverse mode derivatives computation on subgraphs
    f.subgraph_reverse(select_domain);
    //
    // compute the derivative for each range component
    for(size_t i = 0; i < m; i++)
    {   d_vector dw;
        s_vector col;
        size_t   q = 1; // derivative of one Taylor coefficient (zero order)
        f.subgraph_reverse(q, i, col, dw);
        //
        // check order in col
        for(size_t c = 1; c < size_t( col.size() ); c++)
            ok &= col[c] > col[c-1];
        //
        // check that x[0] has been excluded by select_domain
        if( size_t( col.size() ) > 0 )
            ok &= col[0] != 0;
        //
        // check derivatives for i-th row of J(x)
        // note that dw is only specified for j in col
        size_t c = 0;
        for(size_t j = 1; j < n; j++)
        {   while( c < size_t( col.size() ) && col[c] < j )
                ++c;
            if( c < size_t( col.size() ) && col[c] == j )
                ok &= NearEqual(dw[j], J[i * n + j], eps99, eps99);
            else
                ok &= NearEqual(0.0, J[i * n + j], eps99, eps99);
        }
    }
    ok &= f.size_random() > 0;
    f.clear_subgraph();
    ok &= f.size_random() == 0;
    return ok;
}
// END C++
