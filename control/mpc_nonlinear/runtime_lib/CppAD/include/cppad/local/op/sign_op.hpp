# ifndef CPPAD_LOCAL_OP_SIGN_OP_HPP
# define CPPAD_LOCAL_OP_SIGN_OP_HPP
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

// See dev documentation: forward_unary_op
template <class Base>
void forward_sign_op(
    size_t p           ,
    size_t q           ,
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SignOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SignOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( p <= q );

    // Taylor coefficients corresponding to argument and result
    Base* x = taylor + i_x * cap_order;
    Base* z = taylor + i_z * cap_order;

    if( p == 0 )
    {   z[0] = sign(x[0]);
        p++;
    }
    for(size_t j = p; j <= q; j++)
        z[j] = Base(0.);
}
// See dev documentation: forward_unary_op
template <class Base>
void forward_sign_op_dir(
    size_t q           ,
    size_t r           ,
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SignOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SignOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < q );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );

    // Taylor coefficients corresponding to argument and result
    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    size_t m = (q - 1) * r + 1;
    Base* z = taylor + i_z * num_taylor_per_var;

    for(size_t ell = 0; ell < r; ell++)
        z[m+ell] = Base(0.);
}

// See dev documentation: forward_unary_op
template <class Base>
void forward_sign_op_0(
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{

    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SignOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SignOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < cap_order );

    // Taylor coefficients corresponding to argument and result
    Base x0 = *(taylor + i_x * cap_order);
    Base* z = taylor + i_z * cap_order;

    z[0] = sign(x0);
}

// See dev documentation: reverse_unary_op
template <class Base>
void reverse_sign_op(
    size_t      d            ,
    size_t      i_z          ,
    size_t      i_x          ,
    size_t      cap_order    ,
    const Base* taylor       ,
    size_t      nc_partial   ,
    Base*       partial      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(SignOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(SignOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( d < cap_order );
    CPPAD_ASSERT_UNKNOWN( d < nc_partial );

    // nothing to do because partials of sign are zero
    return;
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
