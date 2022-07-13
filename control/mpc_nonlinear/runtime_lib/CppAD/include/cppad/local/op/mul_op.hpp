# ifndef CPPAD_LOCAL_OP_MUL_OP_HPP
# define CPPAD_LOCAL_OP_MUL_OP_HPP
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

// --------------------------- Mulvv -----------------------------------------

// See dev documentation: forward_binary_op
template <class Base>
void forward_mulvv_op(
    size_t        p           ,
    size_t        q           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(MulvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(MulvvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( p <= q );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    size_t k;
    for(size_t d = p; d <= q; d++)
    {   z[d] = Base(0.0);
        for(k = 0; k <= d; k++)
            z[d] += x[d-k] * y[k];
    }
}

// See dev documentation: forward_binary_op
template <class Base>
void forward_mulvv_op_dir(
    size_t        q           ,
    size_t        r           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(MulvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(MulvvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < q );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );

    // Taylor coefficients corresponding to arguments and result
    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    Base* x = taylor + size_t(arg[0]) * num_taylor_per_var;
    Base* y = taylor + size_t(arg[1]) * num_taylor_per_var;
    Base* z = taylor +    i_z * num_taylor_per_var;

    size_t k, ell, m;
    for(ell = 0; ell < r; ell++)
    {   m = (q-1)*r + ell + 1;
        z[m] = x[0] * y[m] + x[m] * y[0];
        for(k = 1; k < q; k++)
            z[m] += x[(q-k-1)*r + ell + 1] * y[(k-1)*r + ell + 1];
    }
}


// See dev documentation: forward_binary_op
template <class Base>
void forward_mulvv_op_0(
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(MulvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(MulvvOp) == 1 );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    z[0] = x[0] * y[0];
}


// See dev documentation: reverse_binary_op
template <class Base>
void reverse_mulvv_op(
    size_t        d           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    const Base*   taylor      ,
    size_t        nc_partial  ,
    Base*         partial     )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(MulvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(MulvvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( d < cap_order );
    CPPAD_ASSERT_UNKNOWN( d < nc_partial );

    // Arguments
    const Base* x  = taylor + size_t(arg[0]) * cap_order;
    const Base* y  = taylor + size_t(arg[1]) * cap_order;

    // Partial derivatives corresponding to arguments and result
    Base* px = partial + size_t(arg[0]) * nc_partial;
    Base* py = partial + size_t(arg[1]) * nc_partial;
    Base* pz = partial + i_z    * nc_partial;


    // number of indices to access
    size_t j = d + 1;
    size_t k;
    while(j)
    {   --j;
        for(k = 0; k <= j; k++)
        {
            px[j-k] += azmul(pz[j], y[k]);
            py[k]   += azmul(pz[j], x[j-k]);
        }
    }
}
// --------------------------- Mulpv -----------------------------------------

// See dev documentation: forward_binary_op
template <class Base>
void forward_mulpv_op(
    size_t        p           ,
    size_t        q           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(MulpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(MulpvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( p <= q );

    // Taylor coefficients corresponding to arguments and result
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    // Paraemter value
    Base x = parameter[ arg[0] ];

    for(size_t d = p; d <= q; d++)
        z[d] = x * y[d];
}

// See dev documentation: forward_binary_op
template <class Base>
void forward_mulpv_op_dir(
    size_t        q           ,
    size_t        r           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(MulpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(MulpvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < q );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );

    // Taylor coefficients corresponding to arguments and result
    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    size_t m                  = (q-1) * r + 1;
    Base* y = taylor + size_t(arg[1]) * num_taylor_per_var + m;
    Base* z = taylor + i_z    * num_taylor_per_var + m;

    // Paraemter value
    Base x = parameter[ arg[0] ];

    for(size_t ell = 0; ell < r; ell++)
        z[ell] = x * y[ell];
}

// See dev documentation: forward_binary_op
template <class Base>
void forward_mulpv_op_0(
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(MulpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(MulpvOp) == 1 );

    // Paraemter value
    Base x = parameter[ arg[0] ];

    // Taylor coefficients corresponding to arguments and result
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    z[0] = x * y[0];
}


// See dev documentation: reverse_binary_op
template <class Base>
void reverse_mulpv_op(
    size_t        d           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    const Base*   taylor      ,
    size_t        nc_partial  ,
    Base*         partial     )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(MulpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(MulpvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( d < cap_order );
    CPPAD_ASSERT_UNKNOWN( d < nc_partial );

    // Arguments
    Base x  = parameter[ arg[0] ];

    // Partial derivatives corresponding to arguments and result
    Base* py = partial + size_t(arg[1]) * nc_partial;
    Base* pz = partial + i_z    * nc_partial;

    // number of indices to access
    size_t j = d + 1;
    while(j)
    {   --j;
        py[j] += azmul(pz[j], x);
    }
}


} } // END_CPPAD_LOCAL_NAMESPACE
# endif
