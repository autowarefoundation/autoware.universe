# ifndef CPPAD_LOCAL_OP_LOG1P_OP_HPP
# define CPPAD_LOCAL_OP_LOG1P_OP_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE

template <class Base>
void forward_log1p_op(
    size_t p           ,
    size_t q           ,
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{
    size_t k;

    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(Log1pOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(Log1pOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( p <= q );

    // Taylor coefficients corresponding to argument and result
    Base* x = taylor + i_x * cap_order;
    Base* z = taylor + i_z * cap_order;

    if( p == 0 )
    {   z[0] = log1p( x[0] );
        p++;
        if( q == 0 )
            return;
    }
    if ( p == 1 )
    {   z[1] = x[1] / (Base(1.0) + x[0]);
        p++;
    }
    for(size_t j = p; j <= q; j++)
    {
        z[j] = -z[1] * x[j-1];
        for(k = 2; k < j; k++)
            z[j] -= Base(double(k)) * z[k] * x[j-k];
        z[j] /= Base(double(j));
        z[j] += x[j];
        z[j] /= (Base(1.0) + x[0]);
    }
}

template <class Base>
void forward_log1p_op_dir(
    size_t q           ,
    size_t r           ,
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{

    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(Log1pOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(Log1pOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < q );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );

    // Taylor coefficients corresponding to argument and result
    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    Base* x = taylor + i_x * num_taylor_per_var;
    Base* z = taylor + i_z * num_taylor_per_var;

    size_t m = (q-1) * r + 1;
    for(size_t ell = 0; ell < r; ell++)
    {   z[m+ell] = Base(double(q)) * x[m+ell];
        for(size_t k = 1; k < q; k++)
            z[m+ell] -= Base(double(k)) * z[(k-1)*r+1+ell] * x[(q-k-1)*r+1+ell];
        z[m+ell] /= (Base(double(q)) + Base(q) * x[0]);
    }
}

template <class Base>
void forward_log1p_op_0(
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{

    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(Log1pOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(Log1pOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < cap_order );

    // Taylor coefficients corresponding to argument and result
    Base* x = taylor + i_x * cap_order;
    Base* z = taylor + i_z * cap_order;

    z[0] = log1p( x[0] );
}


template <class Base>
void reverse_log1p_op(
    size_t      d            ,
    size_t      i_z          ,
    size_t      i_x          ,
    size_t      cap_order    ,
    const Base* taylor       ,
    size_t      nc_partial   ,
    Base*       partial      )
{   size_t j, k;

    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(Log1pOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(Log1pOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( d < cap_order );
    CPPAD_ASSERT_UNKNOWN( d < nc_partial );

    // Taylor coefficients and partials corresponding to argument
    const Base* x  = taylor  + i_x * cap_order;
    Base* px       = partial + i_x * nc_partial;

    // Taylor coefficients and partials corresponding to result
    const Base* z  = taylor  + i_z * cap_order;
    Base* pz       = partial + i_z * nc_partial;

    Base inv_1px0 = Base(1.0) / (Base(1) + x[0]);

    j = d;
    while(j)
    {   // scale partial w.r.t z[j]
        pz[j]   = azmul(pz[j]   , inv_1px0);

        px[0]   -= azmul(pz[j], z[j]);
        px[j]   += pz[j];

        // further scale partial w.r.t. z[j]
        pz[j]   /= Base(double(j));

        for(k = 1; k < j; k++)
        {   pz[k]   -= Base(double(k)) * azmul(pz[j], x[j-k]);
            px[j-k] -= Base(double(k)) * azmul(pz[j], z[k]);
        }
        --j;
    }
    px[0] += azmul(pz[0], inv_1px0);
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
