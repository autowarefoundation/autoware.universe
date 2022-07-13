# ifndef CPPAD_LOCAL_OP_PARAMETER_OP_HPP
# define CPPAD_LOCAL_OP_PARAMETER_OP_HPP
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
void forward_par_op_0(
    size_t        i_z         ,
    const addr_t* arg         ,
    size_t        num_par     ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(ParOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(ParOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
    CPPAD_ASSERT_UNKNOWN( 0 < cap_order );

    Base* z = taylor + i_z * cap_order;

    z[0]  = parameter[ arg[0] ];
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
