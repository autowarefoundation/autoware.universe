# ifndef CPPAD_LOCAL_OP_ADD_OP_HPP
# define CPPAD_LOCAL_OP_ADD_OP_HPP
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

// --------------------------- Addvv -----------------------------------------

// See dev documentation: forward_unary_op
// See dev documentation: forward_binary_op
template <class Base>
void forward_addvv_op(
    size_t        p           ,
    size_t        q           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(AddvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(AddvvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( p <= q  );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    for(size_t j = p; j <= q; j++)
        z[j] = x[j] + y[j];
}

// See dev documentation: forward_unary_op
// See dev documentation: forward_binary_op
template <class Base>
void forward_addvv_op_dir(
    size_t        q           ,
    size_t        r           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(AddvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(AddvvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( 0 < q  );

    // Taylor coefficients corresponding to arguments and result
    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    Base* x = taylor + size_t(arg[0]) * num_taylor_per_var;
    Base* y = taylor + size_t(arg[1]) * num_taylor_per_var;
    Base* z = taylor +    i_z * num_taylor_per_var;

    size_t m = (q-1)*r + 1 ;
    for(size_t ell = 0; ell < r; ell++)
        z[m+ell] = x[m+ell] + y[m+ell];
}


// See dev documentation: forward_unary_op
// See dev documentation: forward_binary_op
template <class Base>
void forward_addvv_op_0(
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(AddvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(AddvvOp) == 1 );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    z[0] = x[0] + y[0];
}


// See dev documentation: reverse_unary_op
// See dev documentation: reverse_binary_op
template <class Base>
void reverse_addvv_op(
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
    CPPAD_ASSERT_UNKNOWN( NumArg(AddvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(AddvvOp) == 1 );
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
        py[i] += pz[i];
    }
}

// --------------------------- Addpv -----------------------------------------
// See dev documentation: forward_unary_op
// See dev documentation: forward_binary_op
template <class Base>
void forward_addpv_op(
    size_t        p           ,
    size_t        q           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(AddpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(AddpvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( p <= q );

    // Taylor coefficients corresponding to arguments and result
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    if( p == 0 )
    {   // Paraemter value
        Base x = parameter[ arg[0] ];
        z[0] = x + y[0];
        p++;
    }
    for(size_t j = p; j <= q; j++)
        z[j] = y[j];
}
// See dev documentation: forward_unary_op
// See dev documentation: forward_binary_op
template <class Base>
void forward_addpv_op_dir(
    size_t        q           ,
    size_t        r           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(AddpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(AddpvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < q );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );

    // Taylor coefficients corresponding to arguments and result
    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    size_t m                  = (q-1) * r + 1;
    Base* y = taylor + size_t(arg[1]) * num_taylor_per_var + m;
    Base* z = taylor + i_z    * num_taylor_per_var + m;

    for(size_t ell = 0; ell < r; ell++)
        z[ell] = y[ell];
}

// See dev documentation: forward_unary_op
// See dev documentation: forward_binary_op
template <class Base>
void forward_addpv_op_0(
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(AddpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(AddpvOp) == 1 );

    // Paraemter value
    Base x = parameter[ arg[0] ];

    // Taylor coefficients corresponding to arguments and result
    Base* y = taylor + size_t(arg[1]) * cap_order;
    Base* z = taylor + i_z    * cap_order;

    z[0] = x + y[0];
}


// See dev documentation: reverse_unary_op
// See dev documentation: reverse_binary_op
template <class Base>
void reverse_addpv_op(
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
    CPPAD_ASSERT_UNKNOWN( NumArg(AddvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(AddvvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( d < cap_order );
    CPPAD_ASSERT_UNKNOWN( d < nc_partial );

    // Partial derivatives corresponding to arguments and result
    Base* py = partial + size_t(arg[1]) * nc_partial;
    Base* pz = partial + i_z    * nc_partial;

    // number of indices to access
    size_t i = d + 1;
    while(i)
    {   --i;
        py[i] += pz[i];
    }
}


} } // END_CPPAD_LOCAL_NAMESPACE
# endif
