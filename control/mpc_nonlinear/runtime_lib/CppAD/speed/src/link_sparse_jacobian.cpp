/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/utility/near_equal.hpp>
# include <cppad/speed/sparse_jac_fun.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/utility/index_sort.hpp>
# include <cppad/utility/time_test.hpp>

# include "link_sparse_jacobian.hpp"

namespace { // BEGIN_EMPTY_NAMESPACE
    static CppAD::vector<size_t> info_size_vec_;
    static CppAD::vector<size_t> info_n_color_vec_;
    //
    static size_t                callback_size_    = 0;
    static size_t                callback_n_color_ = 0;
    static CppAD::vector<size_t> callback_row_;
    static CppAD::vector<size_t> callback_col_;
/*
------------------------------------------------------------------------------
$begin sparse_jacobian_choose_row_col$$
$spell
    Jacobian
    CppAD
$$
$section Randomly Choose Row and Column Indices for Sparse Jacobian$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_CHOOSE_ROW_COL%// END_CHOOSE_ROW_COL%1
%$$

$head n$$
is the dimension of the domain space for the function f(x).

$head m$$
is the dimension of the range space for the function f(x).

$head row$$
The input size and elements of $icode row$$ do not matter.
Upon return it is the chosen row indices.

$head col$$
The input size and elements of $icode col$$ do not matter.
Upon return it is the chosen column indices.

$head Row Major$$
The result is in row major order.

$end
*/
// BEGIN_CHOOSE_ROW_COL
void choose_row_col(
    size_t                 n    ,
    size_t                 m    ,
    CppAD::vector<size_t>& row  ,
    CppAD::vector<size_t>& col  )
// END_CHOOSE_ROW_COL
{   using CppAD::vector;
    //
    // get random numbers for row and column indices
    size_t K = 5 * std::max(m, n);
    vector<double>  random(2 * K);
    CppAD::uniform_01(2 * K, random);
    //
    // sort the temporary row and colunn choices
    vector<size_t> key(K);
    vector<size_t> ind(K);
    for(size_t k = 0; k < K; k++)
    {   // convert from [0,1] to row index
        // avoid warning when converting double to size_t
        size_t r = size_t( float( double(m) * random[k] ) );
        r        = std::min(m-1, r);
        // convert from [0,1] to column index
        size_t c = size_t( float( double(n) * random[k + K] ) );
        c        = std::min(n-1, c);
        //
        // key in row major order
        key[k] = c + n * r;
    }
    CppAD::index_sort(key, ind);
    //
    // remove duplicates and set row, col in row major order
    row.resize(0);
    col.resize(0);
    size_t k          = ind[0];
    size_t c_previous = key[k] % n;
    size_t r_previous = key[k] / n;
    CPPAD_ASSERT_UNKNOWN( r_previous < m && c_previous < n );
    CPPAD_ASSERT_UNKNOWN( key[k] == c_previous + n * r_previous );
    row.push_back(r_previous);
    col.push_back(c_previous);
    for(size_t ell = 1; ell < K; ell++)
    {   k        = ind[ell];
        size_t c = key[k] % n;
        size_t r = key[k] / n;
        CPPAD_ASSERT_UNKNOWN( key[k] == c + n * r );
        CPPAD_ASSERT_UNKNOWN( r < m && c < n );
        if( r != r_previous || c != c_previous)
        {   row.push_back(r);
            col.push_back(c);
        }
        r_previous = r;
        c_previous = c;
    }
# ifndef NDEBUG
    size_t nnz = row.size();
    CPPAD_ASSERT_UNKNOWN( nnz > 0 );
    r_previous    = row[0];
    c_previous    = col[0];
    for(k = 1; k < nnz; ++k)
    {   CPPAD_ASSERT_UNKNOWN( r_previous <= row[k] );
        if( r_previous == row[k] )
            CPPAD_ASSERT_UNKNOWN( c_previous <= col[k] );
        r_previous = row[k];
        c_previous = col[k];
    }
# endif
}
} // END_EMPTY_NAMESPACE
/*
------------------------------------------------------------------------------
$begin info_sparse_jacobian$$
$spell
    Jacobian
    Namespace
    CppAD
    vec
$$

$section Sparse Jacobian Speed Test Information$$

$head Namespace$$
This function is in the global namespace, not the CppAD namespace.

$head Prototype$$
$srcthisfile%
    0%// BEGIN_INFO_SPARSE_JACOBIAN%// END_INFO_SPARSE_JACOBIAN%1
%$$

$head size_vec$$
The input value of $icode size_vec$$ does not matter.
Upon return, it contains the values (in order) of
$cref/size/speed_time/size/$$ in the previous call to
$code time_sparse_jacobian$$.
Only calls since the previous call to $code info_sparse_jacobian$$
are included.

$head n_color$$
The input value of $icode n_color_vec$$ does not matter.
Upon return, it has the same size as $icode size_vec$$.
The value $icode%n_color%[%j%]%$$ is the value of
$cref/n_color/link_sparse_jacobian/n_color/$$
returned by a call to $code link_sparse_jacobian$$
with size equal to $icode%size_vec%[%j%]%$$.

$end
*/
// BEGIN_INFO_SPARSE_JACOBIAN
void info_sparse_jacobian(
    CppAD::vector<size_t>& size_vec    ,
    CppAD::vector<size_t>& n_color_vec )
// END_INFO_SPARSE_JACOBIAN
{   using CppAD::vector;
    //
    size_vec.clear();
    n_color_vec.clear();
    //
    size_vec.swap( info_size_vec_ );
    n_color_vec.swap( info_n_color_vec_ );
    //
    return;
}
// ---------------------------------------------------------------------------
// The routines below are documented in dev_link.omh
// ---------------------------------------------------------------------------
namespace {
    void time_sparse_jacobian_callback(size_t size, size_t repeat)
    {   using CppAD::vector;
        CPPAD_ASSERT_UNKNOWN( callback_size_ == size );
        //
        size_t          n = size;
        size_t          m = 2 * n;
        vector<double>  x(n);
        size_t          nnz = callback_row_.size();
        vector<double>  jacobian(nnz);
        //
        std::string    job = "run";
        link_sparse_jacobian(
            job,
            n,
            repeat,
            m,
            callback_row_,
            callback_col_,
            x,
            jacobian,
            callback_n_color_
        );
        return;
    }
}
// ---------------------------------------------------------------------------
bool available_sparse_jacobian(void)
{   using CppAD::vector;
    //
    size_t n      = 3;
    size_t m      = 2 * n;
    vector<size_t> row, col;
    choose_row_col(n, m, row, col);
    //
    size_t          repeat = 1;
    vector<double>  x(n);
    size_t          nnz = row.size();
    vector<double>  jacobian(nnz);
    size_t          n_color;
    //
    std::string job = "setup";
    bool result = link_sparse_jacobian(
        job, n, repeat, m, row, col, x, jacobian, n_color
    );
    //
    job = "teardown";
    link_sparse_jacobian(
        job, n, repeat, m, row, col, x, jacobian, n_color
    );
    //
    return result;
}
// ----------------------------------------------------------------------------
bool correct_sparse_jacobian(bool is_package_double)
{   using CppAD::vector;
    bool ok       = true;
    double eps    = 10. * CppAD::numeric_limits<double>::epsilon();
    //
    size_t n      = 10;
    size_t m      = 2 * n;
    vector<size_t> row, col;
    choose_row_col(n, m, row, col);
    //
    // The double package assumes jacobian.size() >= m
    size_t nnz = row.size();
    CPPAD_ASSERT_UNKNOWN( nnz >= m );
    //
    size_t         repeat  = 1;
    vector<double> x(n);
    vector<double> jacobian(nnz);
    size_t         n_color;
    // ----------------------------------------------------------------------
    // compute jacobian
    std::string job = "setup";
    link_sparse_jacobian(
            job, n, repeat, m, row, col, x, jacobian, n_color
    );
    //
    job = "run";
    link_sparse_jacobian(
            job, n, repeat, m, row, col, x, jacobian, n_color
    );
    // ----------------------------------------------------------------------
    // check result
    if( is_package_double)
    {   // check f(x)
        size_t order = 0;
        vector<double> check(m);
        CppAD::sparse_jac_fun<double>(m, n, x, row, col, order, check);
        for(size_t i = 0; i < m; i++)
            ok &= CppAD::NearEqual(check[i], jacobian[i], eps, eps);

    }
    else
    {   // check f'(x)
        size_t order = 1;
        vector<double> check(nnz);
        CppAD::sparse_jac_fun<double>(m, n, x, row, col, order, check);
        for(size_t k = 0; k < nnz; k++)
            ok &= CppAD::NearEqual(check[k], jacobian[k], eps, eps);
    }
    // -----------------------------------------------------------------------
    job = "teardown";
    link_sparse_jacobian(
            job, n, repeat, m, row, col, x, jacobian, n_color
    );
    return ok;
}
// ----------------------------------------------------------------------------
double time_sparse_jacobian(double time_min, size_t size)
{   CPPAD_ASSERT_UNKNOWN( size != 0 );
    using CppAD::vector;
    //
    // set callback_row_, callback_col_, callback_size_
    callback_size_   = size;
    size_t n         = size;
    size_t m         = 2 * n;
    choose_row_col(n, m, callback_row_, callback_col_);
    //
    size_t          repeat = 1;
    vector<double>  x(n);
    size_t          nnz = callback_row_.size();
    vector<double>  jacobian(nnz);
    size_t          n_color;
    //
    std::string  job = "setup";
    link_sparse_jacobian(
        job, n, repeat, m, callback_row_, callback_col_, x, jacobian, n_color
    );
    //
    // job is run in time_sparse_jacoabian_callback
    double time = CppAD::time_test(
        time_sparse_jacobian_callback, time_min, size
    );
    if( callback_n_color_ == 0 )
        callback_n_color_ = n_color;

    job = "teardown";
    link_sparse_jacobian(
        job, n, repeat, m, callback_row_, callback_col_, x, jacobian, n_color
    );
    if( callback_n_color_ == 0 )
        callback_n_color_ = n_color;
    //
    // memory allocated for callback_row_, callback_col_
    callback_size_ = 0;
    callback_row_.clear();
    callback_col_.clear();
    //
    info_size_vec_.push_back(size);
    info_n_color_vec_.push_back(callback_n_color_);
    callback_n_color_ = 0;
    //
    return time;
}
