# ifndef CPPAD_EXAMPLE_ATOMIC_THREE_MAT_MUL_HPP
# define CPPAD_EXAMPLE_ATOMIC_THREE_MAT_MUL_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin atomic_three_mat_mul.hpp$$
$spell
    Taylor
    ty
    px
    CppAD
    jac
    hes
    nr
    nc
    afun
    mul
$$

$section Matrix Multiply as an Atomic Operation$$

$head See Also$$
$cref atomic_two_eigen_mat_mul.hpp$$

$head Purpose$$
Use scalar $code double$$ operations in an $cref atomic_three$$ operation
that computes the matrix product for $code AD<double$$ operations.

$subhead parameter_x$$
This example demonstrates the use of the
$cref/parameter_x/atomic_three/parameter_x/$$
argument to the $cref atomic_three$$ virtual functions.

$subhead type_x$$
This example also demonstrates the use of the
$cref/type_x/atomic_three/type_x/$$
argument to the $cref atomic_three$$ virtual functions.

$head Matrix Dimensions$$
The first three components of the argument vector $icode ax$$
in the call $icode%afun%(%ax%, %ay%)%$$
are parameters and contain the matrix dimensions.
This enables them to be different for each use of the same atomic
function $icode afun$$.
These dimensions are:
$table
$icode%ax%[0]%$$  $pre  $$
    $cnext $icode nr_left$$ $pre  $$
    $cnext number of rows in the left matrix and result matrix
$rend
$icode%ax%[1]%$$  $pre  $$
    $cnext $icode n_middle$$ $pre  $$
    $cnext columns in the left matrix and rows in right matrix
$rend
$icode%ax%[2]%$$  $pre  $$
    $cnext $icode nc_right$$ $pre  $$
    $cnext number of columns in the right matrix and result matrix
$tend


$head Left Matrix$$
The number of elements in the left matrix is
$codei%
    %n_left% = %nr_left% * %n_middle%
%$$
The elements are in
$icode%ax%[3]%$$ through $icode%ax%[2+%n_left%]%$$ in row major order.

$head Right Matrix$$
The number of elements in the right matrix is
$codei%
    %n_right% = %n_middle% * %nc_right%
%$$
The elements are in
$icode%ax%[3+%n_left%]%$$ through
$icode%ax%[2+%n_left%+%n_right%]%$$ in row major order.

$head Result Matrix$$
The number of elements in the result matrix is
$codei%
    %n_result% = %nr_left% * %nc_right%
%$$
The elements are in
$icode%ay%[0]%$$ through $icode%ay%[%n_result%-1]%$$ in row major order.

$head Start Class Definition$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
namespace { // Begin empty namespace
using CppAD::vector;
//
// matrix result = left * right
class atomic_mat_mul : public CppAD::atomic_three<double> {
/* %$$
$head Constructor$$
$srccode%cpp% */
public:
    // ---------------------------------------------------------------------
    // constructor
    atomic_mat_mul(void) : CppAD::atomic_three<double>("mat_mul")
    { }
private:
/* %$$
$head Left Operand Element Index$$
Index in the Taylor coefficient matrix $icode tx$$ of a left matrix element.
$srccode%cpp% */
    size_t left(
        size_t i        , // left matrix row index
        size_t j        , // left matrix column index
        size_t k        , // Taylor coeffocient order
        size_t nk       , // number of Taylor coefficients in tx
        size_t nr_left  , // rows in left matrix
        size_t n_middle , // rows in left and columns in right
        size_t nc_right ) // columns in right matrix
    {   assert( i < nr_left );
        assert( j < n_middle );
        return (3 + i * n_middle + j) * nk + k;
    }
/* %$$
$head Right Operand Element Index$$
Index in the Taylor coefficient matrix $icode tx$$ of a right matrix element.
$srccode%cpp% */
    size_t right(
        size_t i        , // right matrix row index
        size_t j        , // right matrix column index
        size_t k        , // Taylor coeffocient order
        size_t nk       , // number of Taylor coefficients in tx
        size_t nr_left  , // rows in left matrix
        size_t n_middle , // rows in left and columns in right
        size_t nc_right ) // columns in right matrix
    {   assert( i < n_middle );
        assert( j < nc_right );
        size_t offset = 3 + nr_left * n_middle;
        return (offset + i * nc_right + j) * nk + k;
    }
/* %$$
$head Result Element Index$$
Index in the Taylor coefficient matrix $icode ty$$ of a result matrix element.
$srccode%cpp% */
    size_t result(
        size_t i        , // result matrix row index
        size_t j        , // result matrix column index
        size_t k        , // Taylor coeffocient order
        size_t nk       , // number of Taylor coefficients in ty
        size_t nr_left  , // rows in left matrix
        size_t n_middle , // rows in left and columns in right
        size_t nc_right ) // columns in right matrix
    {   assert( i < nr_left  );
        assert( j < nc_right );
        return (i * nc_right + j) * nk + k;
    }
/* %$$
$head Forward Matrix Multiply$$
Forward mode multiply Taylor coefficients in $icode tx$$ and sum into
$icode ty$$ (for one pair of left and right orders)
$srccode%cpp% */
    void forward_multiply(
        size_t                 k_left   , // order for left coefficients
        size_t                 k_right  , // order for right coefficients
        const vector<double>&  tx       , // domain space Taylor coefficients
              vector<double>&  ty       , // range space Taylor coefficients
        size_t                 nr_left  , // rows in left matrix
        size_t                 n_middle , // rows in left and columns in right
        size_t                 nc_right ) // columns in right matrix
    {
        size_t nx       = 3 + (nr_left + nc_right) * n_middle;
        size_t nk       = tx.size() / nx;
# ifndef NDEBUG
        size_t ny       = nr_left * nc_right;
        assert( nk == ty.size() / ny );
# endif
        //
        size_t k_result = k_left + k_right;
        assert( k_result < nk );
        //
        for(size_t i = 0; i < nr_left; i++)
        {   for(size_t j = 0; j < nc_right; j++)
            {   double sum = 0.0;
                for(size_t ell = 0; ell < n_middle; ell++)
                {   size_t i_left  = left(
                        i, ell, k_left, nk, nr_left, n_middle, nc_right
                    );
                    size_t i_right = right(
                        ell, j,  k_right, nk, nr_left, n_middle, nc_right
                    );
                    sum           += tx[i_left] * tx[i_right];
                }
                size_t i_result = result(
                    i, j, k_result, nk, nr_left, n_middle, nc_right
                );
                ty[i_result]   += sum;
            }
        }
    }
/* %$$
$head Reverse Matrix Multiply$$
Reverse mode partials of Taylor coefficients and sum into $icode px$$
(for one pair of left and right orders)
$srccode%cpp% */
    void reverse_multiply(
        size_t                 k_left  , // order for left coefficients
        size_t                 k_right , // order for right coefficients
        const vector<double>&  tx      , // domain space Taylor coefficients
        const vector<double>&  ty      , // range space Taylor coefficients
              vector<double>&  px      , // partials w.r.t. tx
        const vector<double>&  py      , // partials w.r.t. ty
        size_t                 nr_left  , // rows in left matrix
        size_t                 n_middle , // rows in left and columns in right
        size_t                 nc_right ) // columns in right matrix
    {
        size_t nx       = 3 + (nr_left + nc_right) * n_middle;
        size_t nk       = tx.size() / nx;
# ifndef NDEBUG
        size_t ny       = nr_left * nc_right;
        assert( nk == ty.size() / ny );
# endif
        assert( tx.size() == px.size() );
        assert( ty.size() == py.size() );
        //
        size_t k_result = k_left + k_right;
        assert( k_result < nk );
        //
        for(size_t i = 0; i < nr_left; i++)
        {   for(size_t j = 0; j < nc_right; j++)
            {   size_t i_result = result(
                    i, j, k_result, nk, nr_left, n_middle, nc_right
                );
                for(size_t ell = 0; ell < n_middle; ell++)
                {   size_t i_left  = left(
                        i, ell, k_left, nk, nr_left, n_middle, nc_right
                    );
                    size_t i_right = right(
                        ell, j,  k_right, nk, nr_left, n_middle, nc_right
                    );
                    // sum        += tx[i_left] * tx[i_right];
                    px[i_left]    += tx[i_right] * py[i_result];
                    px[i_right]   += tx[i_left]  * py[i_result];
                }
            }
        }
        return;
    }
/* %$$
$head for_type$$
Routine called by CppAD during $cref/afun(ax, ay)/atomic_three_afun/$$.
$srccode%cpp% */
    // calculate type_y
    virtual bool for_type(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        vector<CppAD::ad_type_enum>&        type_y      )
    {   assert( parameter_x.size() == type_x.size() );
        bool ok = true;
        ok &= type_x[0] == CppAD::constant_enum;
        ok &= type_x[1] == CppAD::constant_enum;
        ok &= type_x[2] == CppAD::constant_enum;
        if( ! ok )
            return false;
        //
        size_t nr_left  = size_t( parameter_x[0] );
        size_t n_middle = size_t( parameter_x[1] );
        size_t nc_right = size_t( parameter_x[2] );
        //
        ok &= type_x.size() == 3 + (nr_left + nc_right) * n_middle;
        ok &= type_y.size() == n_middle * nc_right;
        if( ! ok )
            return false;
        //
        // commpute type_y
        size_t nk = 1; // number of orders
        size_t k  = 0; // order
        for(size_t i = 0; i < nr_left; ++i)
        {   for(size_t j = 0; j < nc_right; ++j)
            {   // compute type for result[i, j]
                CppAD::ad_type_enum type_yij = CppAD::constant_enum;
                for(size_t ell = 0; ell < n_middle; ++ell)
                {   // index for left(i, ell)
                    size_t i_left = left(
                        i, ell, k, nk, nr_left, n_middle, nc_right
                    );
                    // indx for right(ell, j)
                    size_t i_right = right(
                        ell, j, k, nk, nr_left, n_middle, nc_right
                    );
                    // multiplication on left or right by the constant zero
                    // always results in a constant
                    bool zero_left  = type_x[i_left] == CppAD::constant_enum;
                    zero_left      &= parameter_x[i_left] == 0.0;
                    bool zero_right = type_x[i_right] == CppAD::constant_enum;
                    zero_right     &= parameter_x[i_right] == 0.0;
                    if( ! (zero_left | zero_right) )
                    {   type_yij = std::max(type_yij, type_x[i_left] );
                        type_yij = std::max(type_yij, type_x[i_right] );
                    }
                }
                size_t i_result = result(
                    i, j, k, nk, nr_left, n_middle, nc_right
                );
                type_y[i_result] = type_yij;
            }
        }
        return true;
    }
/* %$$
$head forward$$
Routine called by CppAD during $cref Forward$$ mode.
$srccode%cpp% */
    virtual bool forward(
        const vector<double>&              parameter_x ,
        const vector<CppAD::ad_type_enum>& type_x ,
        size_t                             need_y ,
        size_t                             q      ,
        size_t                             p      ,
        const vector<double>&              tx     ,
        vector<double>&                    ty     )
    {   size_t n_order  = p + 1;
        size_t nr_left  = size_t( tx[ 0 * n_order + 0 ] );
        size_t n_middle = size_t( tx[ 1 * n_order + 0 ] );
        size_t nc_right = size_t( tx[ 2 * n_order + 0 ] );
# ifndef NDEBUG
        size_t nx       = 3 + (nr_left + nc_right) * n_middle;
        size_t ny       = nr_left * nc_right;
# endif
        assert( nx * n_order == tx.size() );
        assert( ny * n_order == ty.size() );
        size_t i, j, ell;

        // initialize result as zero
        size_t k;
        for(i = 0; i < nr_left; i++)
        {   for(j = 0; j < nc_right; j++)
            {   for(k = q; k <= p; k++)
                {   size_t i_result = result(
                        i, j, k, n_order, nr_left, n_middle, nc_right
                    );
                    ty[i_result] = 0.0;
                }
            }
        }
        for(k = q; k <= p; k++)
        {   // sum the produces that result in order k
            for(ell = 0; ell <= k; ell++)
                forward_multiply(
                    ell, k - ell, tx, ty, nr_left, n_middle, nc_right
                );
        }

        // all orders are implemented, so always return true
        return true;
    }
/* %$$
$head reverse$$
Routine called by CppAD during $cref Reverse$$ mode.
$srccode%cpp% */
    virtual bool reverse(
        const vector<double>&              parameter_x ,
        const vector<CppAD::ad_type_enum>& type_x      ,
        size_t                             p           ,
        const vector<double>&              tx          ,
        const vector<double>&              ty          ,
        vector<double>&                    px          ,
        const vector<double>&              py          )
    {   size_t n_order  = p + 1;
        size_t nr_left  = size_t( tx[ 0 * n_order + 0 ] );
        size_t n_middle = size_t( tx[ 1 * n_order + 0 ] );
        size_t nc_right = size_t( tx[ 2 * n_order + 0 ] );
# ifndef NDEBUG
        size_t nx       = 3 + (nr_left + nc_right) * n_middle;
        size_t ny       = nr_left * nc_right;
# endif
        assert( nx * n_order == tx.size() );
        assert( ny * n_order == ty.size() );
        assert( px.size() == tx.size() );
        assert( py.size() == ty.size() );

        // initialize summation
        for(size_t i = 0; i < px.size(); i++)
            px[i] = 0.0;

        // number of orders to differentiate
        size_t k = n_order;
        while(k--)
        {   // differentiate the produces that result in order k
            for(size_t ell = 0; ell <= k; ell++)
                reverse_multiply(
                    ell, k - ell, tx, ty, px, py, nr_left, n_middle, nc_right
                );
        }

        // all orders are implented, so always return true
        return true;
    }
/* %$$
$head jac_sparsity$$
$srccode%cpp% */
    // Jacobian sparsity routine called by CppAD
    virtual bool jac_sparsity(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        bool                                dependency  ,
        const vector<bool>&                 select_x    ,
        const vector<bool>&                 select_y    ,
        CppAD::sparse_rc< vector<size_t> >& pattern_out )
    {
        size_t n = select_x.size();
        size_t m = select_y.size();
        assert( parameter_x.size() == n );
        assert( type_x.size() == n );
        //
        size_t nr_left  = size_t( parameter_x[0] );
        size_t n_middle = size_t( parameter_x[1] );
        size_t nc_right = size_t( parameter_x[2] );
        size_t nk       = 1; // only one order
        size_t k        = 0; // order zero
        //
        // count number of non-zeros in sparsity pattern
        size_t nnz = 0;
        for(size_t i = 0; i < nr_left; ++i)
        {   for(size_t j = 0; j < nc_right; ++j)
            {   size_t i_result = result(
                    i, j, k, nk, nr_left, n_middle, nc_right
                );
                if( select_y[i_result] )
                {   for(size_t ell = 0; ell < n_middle; ++ell)
                    {   size_t i_left = left(
                            i, ell, k, nk, nr_left, n_middle, nc_right
                        );
                        size_t i_right = right(
                            ell, j, k, nk, nr_left, n_middle, nc_right
                        );
                        bool zero_left  =
                            type_x[i_left] == CppAD::constant_enum;
                        zero_left      &= parameter_x[i_left] == 0.0;
                        bool zero_right =
                            type_x[i_right] == CppAD::constant_enum;
                        zero_right     &= parameter_x[i_right] == 0.0;
                        if( ! (zero_left | zero_right ) )
                        {   bool var_left  =
                                type_x[i_left] == CppAD::variable_enum;
                            bool var_right =
                                type_x[i_right] == CppAD::variable_enum;
                            if( select_x[i_left] & var_left )
                                ++nnz;
                            if( select_x[i_right] & var_right )
                                ++nnz;
                        }
                    }
                }
            }
        }
        //
        // fill in the sparsity pattern
        pattern_out.resize(m, n, nnz);
        size_t idx = 0;
        for(size_t i = 0; i < nr_left; ++i)
        {   for(size_t j = 0; j < nc_right; ++j)
            {   size_t i_result = result(
                    i, j, k, nk, nr_left, n_middle, nc_right
                );
                if( select_y[i_result] )
                {   for(size_t ell = 0; ell < n_middle; ++ell)
                    {   size_t i_left = left(
                            i, ell, k, nk, nr_left, n_middle, nc_right
                        );
                        size_t i_right = right(
                            ell, j, k, nk, nr_left, n_middle, nc_right
                        );
                        bool zero_left  =
                            type_x[i_left] == CppAD::constant_enum;
                        zero_left      &= parameter_x[i_left] == 0.0;
                        bool zero_right =
                            type_x[i_right] == CppAD::constant_enum;
                        zero_right     &= parameter_x[i_right] == 0.0;
                        if( ! (zero_left | zero_right ) )
                        {   bool var_left  =
                                type_x[i_left] == CppAD::variable_enum;
                            bool var_right =
                                type_x[i_right] == CppAD::variable_enum;
                            if( select_x[i_left] & var_left )
                                pattern_out.set(idx++, i_result, i_left);
                            if( select_x[i_right] & var_right )
                                pattern_out.set(idx++, i_result, i_right);
                        }
                    }
                }
            }
        }
        assert( idx == nnz );
        //
        return true;
    }
/* %$$
$head hes_sparsity$$
$srccode%cpp% */
    // Jacobian sparsity routine called by CppAD
    virtual bool hes_sparsity(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        const vector<bool>&                 select_x    ,
        const vector<bool>&                 select_y    ,
        CppAD::sparse_rc< vector<size_t> >& pattern_out )
    {
        size_t n = select_x.size();
        assert( parameter_x.size() == n );
        assert( type_x.size() == n );
        //
        size_t nr_left  = size_t( parameter_x[0] );
        size_t n_middle = size_t( parameter_x[1] );
        size_t nc_right = size_t( parameter_x[2] );
        size_t nk       = 1; // only one order
        size_t k        = 0; // order zero
        //
        // count number of non-zeros in sparsity pattern
        size_t nnz = 0;
        for(size_t i = 0; i < nr_left; ++i)
        {   for(size_t j = 0; j < nc_right; ++j)
            {   size_t i_result = result(
                    i, j, k, nk, nr_left, n_middle, nc_right
                );
                if( select_y[i_result] )
                {   for(size_t ell = 0; ell < n_middle; ++ell)
                    {   // i_left depends on i, ell
                        size_t i_left = left(
                            i, ell, k, nk, nr_left, n_middle, nc_right
                        );
                        // i_right depens on ell, j
                        size_t i_right = right(
                            ell, j, k, nk, nr_left, n_middle, nc_right
                        );
                        bool var_left   = select_x[i_left] &
                            (type_x[i_left] == CppAD::variable_enum);
                        bool var_right  = select_x[i_right] &
                            (type_x[i_right] == CppAD::variable_enum);
                        if( var_left & var_right )
                                nnz += 2;
                    }
                }
            }
        }
        //
        // fill in the sparsity pattern
        pattern_out.resize(n, n, nnz);
        size_t idx = 0;
        for(size_t i = 0; i < nr_left; ++i)
        {   for(size_t j = 0; j < nc_right; ++j)
            {   size_t i_result = result(
                    i, j, k, nk, nr_left, n_middle, nc_right
                );
                if( select_y[i_result] )
                {   for(size_t ell = 0; ell < n_middle; ++ell)
                    {   size_t i_left = left(
                            i, ell, k, nk, nr_left, n_middle, nc_right
                        );
                        size_t i_right = right(
                            ell, j, k, nk, nr_left, n_middle, nc_right
                        );
                        bool var_left   = select_x[i_left] &
                            (type_x[i_left] == CppAD::variable_enum);
                        bool var_right  = select_x[i_right] &
                            (type_x[i_right] == CppAD::variable_enum);
                        if( var_left & var_right )
                        {   // Cannot possibly set the same (i_left, i_right)
                            // pair twice.
                            assert( i_left != i_right );
                            pattern_out.set(idx++, i_left, i_right);
                            pattern_out.set(idx++, i_right, i_left);
                        }
                    }
                }
            }
        }
        assert( idx == nnz );
        //
        return true;
    }
/* %$$
$head rev_depend$$
Routine called when a function using $code mat_mul$$ is optimized.
$srccode%cpp% */
    // calculate depend_x
    virtual bool rev_depend(
        const vector<double>&              parameter_x ,
        const vector<CppAD::ad_type_enum>& type_x      ,
        vector<bool>&                      depend_x    ,
        const vector<bool>&                depend_y    )
    {   assert( parameter_x.size() == depend_x.size() );
        assert( parameter_x.size() == type_x.size() );
        bool ok = true;
        //
        size_t nr_left  = size_t( parameter_x[0] );
        size_t n_middle = size_t( parameter_x[1] );
        size_t nc_right = size_t( parameter_x[2] );
        //
        ok &= depend_x.size() == 3 + (nr_left + nc_right) * n_middle;
        ok &= depend_y.size() == n_middle * nc_right;
        if( ! ok )
            return false;
        //
        // initialize depend_x
        for(size_t ell = 0; ell < 3; ++ell)
            depend_x[ell] = true; // always need these parameters
        for(size_t ell = 3; ell < depend_x.size(); ++ell)
            depend_x[ell] = false; // initialize as false
        //
        // commpute depend_x
        size_t nk = 1; // number of orders
        size_t k  = 0; // order
        for(size_t i = 0; i < nr_left; ++i)
        {   for(size_t j = 0; j < nc_right; ++j)
            {   // check depend for result[i, j]
                size_t i_result = result(
                    i, j, k, nk, nr_left, n_middle, nc_right
                );
                if( depend_y[i_result] )
                {   for(size_t ell = 0; ell < n_middle; ++ell)
                    {   // index for left(i, ell)
                        size_t i_left = left(
                            i, ell, k, nk, nr_left, n_middle, nc_right
                        );
                        // indx for right(ell, j)
                        size_t i_right = right(
                            ell, j, k, nk, nr_left, n_middle, nc_right
                        );
                        bool zero_left  =
                            type_x[i_left] == CppAD::constant_enum;
                        zero_left      &= parameter_x[i_left] == 0.0;
                        bool zero_right =
                            type_x[i_right] == CppAD::constant_enum;
                        zero_right     &= parameter_x[i_right] == 0.0;
                        if( ! zero_right )
                            depend_x[i_left]  = true;
                        if( ! zero_left )
                            depend_x[i_right] = true;
                    }
                }
            }
        }
        return true;
    }
/* %$$
$head End Class Definition$$
$srccode%cpp% */
}; // End of mat_mul class
}  // End empty namespace
/* %$$
$comment end nospell$$
$end
*/


# endif
