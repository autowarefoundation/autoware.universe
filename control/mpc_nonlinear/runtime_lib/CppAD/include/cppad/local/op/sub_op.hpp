# ifndef CPPAD_LOCAL_OP_SUB_OP_HPP
# define CPPAD_LOCAL_OP_SUB_OP_HPP
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

// --------------------------- Subvv -----------------------------------------

// See dev documentation: forward_binary_op
template <class Base>
void forward_subvv_op(
    size_t        p           ,
    size_t        q           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SubvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubvvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( p <= q );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    for(size_t d = p; d <= q; d++)
        z[d] = x[d] - y[d];
}

// See dev documentation: forward_binary_op
template <class Base>
void forward_subvv_op_dir(
    size_t        q           ,
    size_t        r           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SubvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubvvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < q );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );

    // Taylor coefficients corresponding to arguments and result
    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    size_t m                  = (q-1) * r + 1;
    Base* x = taylor + size_t(arg[0]) * num_taylor_per_var + m;
    Base* y = taylor + size_t(arg[1]) * num_taylor_per_var + m;
    Base* z = taylor + i_z    * num_taylor_per_var + m;

    for(size_t ell = 0; ell < r; ell++)
        z[ell] = x[ell] - y[ell];
}


// See dev documentation: forward_binary_op
template <class Base>
void forward_subvv_op_0(
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SubvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubvvOp) == 1 );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    z[0] = x[0] - y[0];
}


// See dev documentation: reverse_binary_op
template <class Base>
void reverse_subvv_op(
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
    CPPAD_ASSERT_UNKNOWN( NumArg(SubvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubvvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( d < cap_order );
    CPPAD_ASSERT_UNKNOWN( d < nc_partial );

    // Partial derivatives corresponding to arguments and result
    Base* px = partial + size_t(arg[0]) * nc_partial;
    Base* py = partial + size_t(arg[1]) * nc_partial;
    Base* pz = partial + i_z    * nc_partial;

    // number of indices to access
    size_t i = d + 1;
    while(i)
    {   --i;
        px[i] += pz[i];
        py[i] -= pz[i];
    }
}

// --------------------------- Subpv -----------------------------------------

// See dev documentation: forward_binary_op
template <class Base>
void forward_subpv_op(
    size_t        p           ,
    size_t        q           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SubpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubpvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( p <= q );

    // Taylor coefficients corresponding to arguments and result
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    // Paraemter value
    Base x = parameter[ arg[0] ];
    if( p == 0 )
    {   z[0] = x - y[0];
        p++;
    }
    for(size_t d = p; d <= q; d++)
        z[d] = - y[d];
}

// See dev documentation: forward_binary_op
template <class Base>
void forward_subpv_op_dir(
    size_t        q           ,
    size_t        r           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SubpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubpvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < q );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );

    // Taylor coefficients corresponding to arguments and result
    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    size_t m                  = (q-1) * r + 1;
    Base* y = taylor + size_t(arg[1]) * num_taylor_per_var + m;
    Base* z = taylor + i_z    * num_taylor_per_var + m;

    // Paraemter value
    for(size_t ell = 0; ell < r; ell++)
        z[ell] = - y[ell];
}

// See dev documentation: forward_binary_op
template <class Base>
void forward_subpv_op_0(
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SubpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubpvOp) == 1 );

    // Paraemter value
    Base x = parameter[ arg[0] ];

    // Taylor coefficients corresponding to arguments and result
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    z[0] = x - y[0];
}


// See dev documentation: reverse_binary_op
template <class Base>
void reverse_subpv_op(
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
    CPPAD_ASSERT_UNKNOWN( NumArg(SubvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubvvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( d < cap_order );
    CPPAD_ASSERT_UNKNOWN( d < nc_partial );

    // Partial derivatives corresponding to arguments and result
    Base* py = partial + size_t(arg[1]) * nc_partial;
    Base* pz = partial + i_z    * nc_partial;

    // number of indices to access
    size_t i = d + 1;
    while(i)
    {   --i;
        py[i] -= pz[i];
    }
}

// --------------------------- Subvp -----------------------------------------

// See dev documentation: forward_binary_op
template <class Base>
void forward_subvp_op(
    size_t        p           ,
    size_t        q           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SubvpOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubvpOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( p <= q );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    // Parameter value
    Base y = parameter[ arg[1] ];
    if( p == 0 )
    {   z[0] = x[0] - y;
        p++;
    }
    for(size_t d = p; d <= q; d++)
        z[d] = x[d];
}

// See dev documentation: forward_binary_op
template <class Base>
void forward_subvp_op_dir(
    size_t        q           ,
    size_t        r           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SubvpOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubvpOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < q );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );

    // Taylor coefficients corresponding to arguments and result
    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    Base* x = taylor + size_t(arg[0]) * num_taylor_per_var;
    Base* z = taylor + i_z    * num_taylor_per_var;

    // Parameter value
    size_t m = (q-1) * r + 1;
    for(size_t ell = 0; ell < r; ell++)
        z[m+ell] = x[m+ell];
}


// See dev documentation: forward_binary_op
template <class Base>
void forward_subvp_op_0(
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SubvpOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubvpOp) == 1 );

    // Parameter value
    Base y = parameter[ arg[1] ];

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    z[0] = x[0] - y;
}


// See dev documentation: reverse_binary_op
template <class Base>
void reverse_subvp_op(
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
    CPPAD_ASSERT_UNKNOWN( NumArg(SubvpOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SubvpOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( d < cap_order );
    CPPAD_ASSERT_UNKNOWN( d < nc_partial );

    // Partial derivatives corresponding to arguments and result
    Base* px = partial + size_t(arg[0]) * nc_partial;
    Base* pz = partial + i_z    * nc_partial;

    // number of indices to access
    size_t i = d + 1;
    while(i)
    {   --i;
        px[i] += pz[i];
    }
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
