# ifndef CPPAD_TEST_MORE_DEPRECATED_OLD_MAT_MUL_HPP
# define CPPAD_TEST_MORE_DEPRECATED_OLD_MAT_MUL_HPP
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
$begin old_mat_mul.hpp$$
$spell
    old_mat_mul.hpp
    cppad
    CppAD
    namespace
    struct
    nr
    nc
    bool
    vx
    const
    im
    mj
    ij
    px
    py
    std
    tx
    ty
    resize
    nz
    var
    jac
    Jacobian
    hes
    vy
$$


$section Define Matrix Multiply as a User Atomic Operation$$


$head Syntax$$
This file is located in the $code example$$ directory.
It can be copied to the current working directory and included
with the syntax
$codei%
    # include "old_mat_mul.hpp"
%$$

$head Example$$
The file old_mat_mul.cpp contains an example use of
$code old_mat_mul.hpp$$.

$head Begin Source$$
$srccode%cpp% */
# include <cppad/cppad.hpp>      // Include CppAD definitions
namespace {                      // Begin empty namespace
    using CppAD::vector;        // Let vector denote CppAD::vector
/* %$$

$head Extra Call Information$$
$srccode%cpp% */
    // Information we will attach to each mat_mul call
    struct call_info {
        size_t nr_result;
        size_t n_middle;
        size_t nc_result;
        vector<bool>  vx;
    };
    vector<call_info> info_; // vector of call information

    // number of orders for this operation (k + 1)
    size_t n_order_ = 0;
    // number of rows in the result matrix
    size_t nr_result_ = 0;
    // number of columns in left matrix and number of rows in right matrix
    size_t n_middle_ = 0;
    // number of columns in the result matrix
    size_t nc_result_ = 0;
    // which components of x are variables
    vector<bool>* vx_ = nullptr;

    // get the information corresponding to this call
    void get_info(size_t id, size_t k, size_t n, size_t m)
    {   n_order_   = k + 1;
        nr_result_ = info_[id].nr_result;
        n_middle_  = info_[id].n_middle;
        nc_result_ = info_[id].nc_result;
        vx_        = &(info_[id].vx);

        assert(n == nr_result_ * n_middle_ + n_middle_ * nc_result_);
        assert(m ==  nr_result_ * nc_result_);
    }

/* %$$
$head Matrix Indexing$$
$srccode%cpp% */
    // Convert left matrix index pair and order to a single argument index
    size_t left(size_t i, size_t j, size_t ell)
    {   assert( i < nr_result_ );
        assert( j < n_middle_ );
        return (i * n_middle_ + j) * n_order_ + ell;
    }
    // Convert right matrix index pair and order to a single argument index
    size_t right(size_t i, size_t j, size_t ell)
    {   assert( i < n_middle_ );
        assert( j < nc_result_ );
        size_t offset = nr_result_ * n_middle_;
        return (offset + i * nc_result_ + j) * n_order_ + ell;
    }
    // Convert result matrix index pair and order to a single result index
    size_t result(size_t i, size_t j, size_t ell)
    {   assert( i < nr_result_ );
        assert( j < nc_result_ );
        return (i * nc_result_ + j) * n_order_ + ell;
    }
/* %$$

$head One Matrix Multiply$$
Forward mode matrix multiply left times right and sum into result:
$srccode%cpp% */
    void multiply_and_sum(
        size_t                order_left ,
        size_t                order_right,
        const vector<double>&         tx ,
        vector<double>&               ty )
    {   size_t i, j;
        size_t order_result = order_left + order_right;
        for(i = 0; i < nr_result_; i++)
        {   for(j = 0; j < nc_result_; j++)
            {   double sum = 0.;
                size_t middle, im_left, mj_right, ij_result;
                for(middle = 0; middle < n_middle_; middle++)
                {   im_left  = left(i, middle, order_left);
                    mj_right = right(middle, j, order_right);
                    sum     += tx[im_left] * tx[mj_right];
                }
                ij_result = result(i, j, order_result);
                ty[ ij_result ] += sum;
            }
        }
        return;
    }
/* %$$

$head Reverse Partials One Order$$
Compute reverse mode partials for one order and sum into px:
$srccode%cpp% */
    void reverse_multiply(
        size_t                order_left ,
        size_t                order_right,
        const vector<double>&         tx ,
        const vector<double>&         ty ,
        vector<double>&               px ,
        const vector<double>&         py )
    {   size_t i, j;
        size_t order_result = order_left + order_right;
        for(i = 0; i < nr_result_; i++)
        {   for(j = 0; j < nc_result_; j++)
            {   size_t middle, im_left, mj_right, ij_result;
                for(middle = 0; middle < n_middle_; middle++)
                {   ij_result = result(i, j, order_result);
                    im_left   = left(i, middle, order_left);
                    mj_right  = right(middle, j, order_right);
                    // sum       += tx[im_left]  * tx[mj_right];
                    px[im_left]  += tx[mj_right] * py[ij_result];
                    px[mj_right] += tx[im_left]  * py[ij_result];
                }
            }
        }
        return;
    }
/* %$$
$head Set Union$$
$srccode%cpp% */
    using CppAD::set_union;
/* %$$

$head CppAD User Atomic Callback Functions$$
$srccode%cpp% */
    // ----------------------------------------------------------------------
    // forward mode routine called by CppAD
    bool mat_mul_forward(
        size_t                   id ,
        size_t                    k ,
        size_t                    n ,
        size_t                    m ,
        const vector<bool>&      vx ,
        vector<bool>&            vy ,
        const vector<double>&    tx ,
        vector<double>&          ty
    )
    {   size_t i, j, ell;
        get_info(id, k, n, m);

        // check if this is during the call to mat_mul(id, ax, ay)
        if( vx.size() > 0 )
        {   assert( k == 0 && vx.size() > 0 );

            // store the vx information in info_
            assert( vx_->size() == 0 );
            info_[id].vx.resize(n);
            for(j = 0; j < n; j++)
                info_[id].vx[j] = vx[j];
            assert( vx_->size() == n );

            // now compute vy
            for(i = 0; i < nr_result_; i++)
            {   for(j = 0; j < nc_result_; j++)
                {   // compute vy[ result(i, j, 0) ]
                    bool   var = false;
                    bool   nz_left, nz_right;
                    size_t middle, im_left, mj_right, ij_result;
                    for(middle = 0; middle < n_middle_; middle++)
                    {   im_left  = left(i, middle, k);
                        mj_right = right(middle, j, k);
                        nz_left  = vx[im_left]  | (tx[im_left] != 0.);
                        nz_right = vx[mj_right] | (tx[mj_right]!= 0.);
                        // if not multiplying by the constant zero
                        if( nz_left & nz_right )
                            var |= (vx[im_left] | vx[mj_right]);
                    }
                    ij_result     = result(i, j, k);
                    vy[ij_result] = var;
                }
            }
        }

        // initialize result as zero
        for(i = 0; i < nr_result_; i++)
        {   for(j = 0; j < nc_result_; j++)
                ty[ result(i, j, k) ] = 0.;
        }
        // sum the product of proper orders
        for(ell = 0; ell <=k; ell++)
            multiply_and_sum(ell, k-ell, tx, ty);

        // All orders are implemented and there are no possible error
        // conditions, so always return true.
        return true;
    }
    // ----------------------------------------------------------------------
    // reverse mode routine called by CppAD
    bool mat_mul_reverse(
        size_t                   id ,
        size_t                    k ,
        size_t                    n ,
        size_t                    m ,
        const vector<double>&    tx ,
        const vector<double>&    ty ,
        vector<double>&          px ,
        const vector<double>&    py
    )
    {   get_info(id, k, n, m);

        size_t ell = n * n_order_;
        while(ell--)
            px[ell] = 0.;

        size_t order = n_order_;
        while(order--)
        {   // reverse sum the products for specified order
            for(ell = 0; ell <=order; ell++)
                reverse_multiply(ell, order-ell, tx, ty, px, py);
        }

        // All orders are implemented and there are no possible error
        // conditions, so always return true.
        return true;
    }

    // ----------------------------------------------------------------------
    // forward Jacobian sparsity routine called by CppAD
    bool mat_mul_for_jac_sparse(
        size_t                               id ,
        size_t                                n ,
        size_t                                m ,
        size_t                                p ,
        const vector< std::set<size_t> >&     r ,
        vector< std::set<size_t> >&           s )
    {   size_t i, j, k, im_left, middle, mj_right, ij_result;
        k = 0;
        get_info(id, k, n, m);

        for(i = 0; i < nr_result_; i++)
        {   for(j = 0; j < nc_result_; j++)
            {   ij_result = result(i, j, k);
                s[ij_result].clear();
                for(middle = 0; middle < n_middle_; middle++)
                {   im_left   = left(i, middle, k);
                    mj_right  = right(middle, j, k);

                    // s[ij_result] = union( s[ij_result], r[im_left] )
                    s[ij_result] = set_union(s[ij_result], r[im_left]);

                    // s[ij_result] = union( s[ij_result], r[mj_right] )
                    s[ij_result] = set_union(s[ij_result], r[mj_right]);
                }
            }
        }
        return true;
    }
    // ----------------------------------------------------------------------
    // reverse Jacobian sparsity routine called by CppAD
    bool mat_mul_rev_jac_sparse(
        size_t                               id ,
        size_t                                n ,
        size_t                                m ,
        size_t                                p ,
        vector< std::set<size_t> >&           r ,
        const vector< std::set<size_t> >&     s )
    {   size_t i, j, k, im_left, middle, mj_right, ij_result;
        k = 0;
        get_info(id, k, n, m);

        for(j = 0; j < n; j++)
            r[j].clear();

        for(i = 0; i < nr_result_; i++)
        {   for(j = 0; j < nc_result_; j++)
            {   ij_result = result(i, j, k);
                for(middle = 0; middle < n_middle_; middle++)
                {   im_left   = left(i, middle, k);
                    mj_right  = right(middle, j, k);

                    // r[im_left] = union( r[im_left], s[ij_result] )
                    r[im_left] = set_union(r[im_left], s[ij_result]);

                    // r[mj_right] = union( r[mj_right], s[ij_result] )
                    r[mj_right] = set_union(r[mj_right], s[ij_result]);
                }
            }
        }
        return true;
    }
    // ----------------------------------------------------------------------
    // reverse Hessian sparsity routine called by CppAD
    bool mat_mul_rev_hes_sparse(
        size_t                               id ,
        size_t                                n ,
        size_t                                m ,
        size_t                                p ,
        const vector< std::set<size_t> >&     r ,
        const vector<bool>&                   s ,
        vector<bool>&                         t ,
        const vector< std::set<size_t> >&     u ,
        vector< std::set<size_t> >&           v )
    {   size_t i, j, k, im_left, middle, mj_right, ij_result;
        k = 0;
        get_info(id, k, n, m);

        for(j = 0; j < n; j++)
        {   t[j] = false;
            v[j].clear();
        }

        assert( vx_->size() == n );
        for(i = 0; i < nr_result_; i++)
        {   for(j = 0; j < nc_result_; j++)
            {   ij_result = result(i, j, k);
                for(middle = 0; middle < n_middle_; middle++)
                {   im_left   = left(i, middle, k);
                    mj_right  = right(middle, j, k);

                    // back propagate Jacobian sparsity
                    t[im_left]   = (t[im_left] | s[ij_result]);
                    t[mj_right]  = (t[mj_right] | s[ij_result]);
                    // Visual Studio C++ 2008 warns unsafe mix of int and
                    // bool if we use the following code directly above:
                    // t[im_left]  |= s[ij_result];
                    // t[mj_right] |= s[ij_result];

                    // back propagate Hessian sparsity
                    // v[im_left]  = union( v[im_left],  u[ij_result] )
                    // v[mj_right] = union( v[mj_right], u[ij_result] )
                    v[im_left] = set_union(v[im_left],  u[ij_result] );
                    v[mj_right] = set_union(v[mj_right], u[ij_result] );

                    // Check for case where the (i,j) result element
                    // is in reverse Jacobian and both left and right
                    // operands in multiplication are variables
                    if(s[ij_result] & (*vx_)[im_left] & (*vx_)[mj_right])
                    {   // v[im_left] = union( v[im_left], r[mj_right] )
                        v[im_left] = set_union(v[im_left], r[mj_right] );
                        // v[mj_right] = union( v[mj_right], r[im_left] )
                        v[mj_right] = set_union(v[mj_right], r[im_left] );
                    }
                }
            }
        }
        return true;
    }
/* %$$

$head Declare mat_mul Function$$
Declare the $code AD<double>$$ routine $codei%mat_mul(%id%, %ax%, %ay%)%$$
and end empty namespace
(we could use any $cref/simple vector template class/SimpleVector/$$
instead of $code CppAD::vector$$):
$srccode%cpp% */
    CPPAD_USER_ATOMIC(
        mat_mul                 ,
        CppAD::vector           ,
        double                  ,
        mat_mul_forward         ,
        mat_mul_reverse         ,
        mat_mul_for_jac_sparse  ,
        mat_mul_rev_jac_sparse  ,
        mat_mul_rev_hes_sparse
    )
} // End empty namespace
/* %$$
$end
*/

# endif
