# ifndef CPPAD_LOCAL_OP_CSUM_OP_HPP
# define CPPAD_LOCAL_OP_CSUM_OP_HPP
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
/*!
\file csum_op.hpp
Forward, reverse and sparsity calculations for cummulative summation.
*/

/*!
Compute forward mode Taylor coefficients for result of op = CsumOp.

This operation is
\verbatim
    z = s + x(0) + ... + x(n1-1) - y(0) - ... - y(n2-1)
          + p(0) + ... + p(n3-1) - q(0) - ... - q(n4-1).
\endverbatim

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
Base.

\param p
lowest order of the Taylor coefficient that we are computing.

\param q
highest order of the Taylor coefficient that we are computing.

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in taylor corresponding to z.

\param arg
-- arg[0]
parameter[arg[0]] is the parameter value s in this cummunative summation.

-- arg[1]
end in arg of addition variables in summation.
arg[5] , ... , arg[arg[1]-1] correspond to x(0), ... , x(n1-1)

-- arg[2]
end in arg of subtraction variables in summation.
arg[arg[1]] , ... , arg[arg[2]-1] correspond to y(0), ... , y(n2-1)

-- arg[3]
end in arg of addition dynamic parameters in summation.
arg[arg[2]] , ... , arg[arg[3]-1] correspond to p(0), ... , p(n3-1)

-- arg[4]
end in arg of subtraction dynamic parameters in summation.
arg[arg[3]] , ... , arg[arg[4]-1] correspond to q(0), ... , q(n4-1)

\param num_par
is the number of parameters in parameter.

\param parameter
is the parameter vector for this operation sequence.

\param cap_order
number of colums in the matrix containing all the Taylor coefficients.

\param taylor
\b Input: taylor [ arg[5+i] * cap_order + k ]
for i = 0, ..., n1-1
and k = 0 , ... , q
is the k-th order Taylor coefficient corresponding to x(i)
\n
\b Input: taylor [ arg[arg[1]+1] * cap_order + k ]
for i = 0, ..., n2-1
and k = 0 , ... , q
is the k-th order Taylor coefficient corresponding to y(i)
\n
\b Input: taylor [ i_z * cap_order + k ]
for k = 0 , ... , p,
is the k-th order Taylor coefficient corresponding to z.
\n
\b Output: taylor [ i_z * cap_order + k ]
for k = p , ... , q,
is the k-th order Taylor coefficient corresponding to z.
*/
template <class Base>
void forward_csum_op(
    size_t        p           ,
    size_t        q           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    size_t        num_par     ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{   Base zero(0);

    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumRes(CSumOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( p <= q );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
    CPPAD_ASSERT_UNKNOWN(
        arg[arg[4]] == arg[4]
    );

    // Taylor coefficients corresponding to result
    Base* z = taylor + i_z    * cap_order;
    for(size_t k = p; k <= q; k++)
        z[k] = zero;
    if( p == 0 )
    {   // normal parameters in the sum
        z[p] = parameter[ arg[0] ];
        // addition dynamic parameters
        for(size_t i = size_t(arg[2]); i < size_t(arg[3]); ++i)
            z[p] += parameter[ arg[i] ];
        // subtraction dynamic parameters
        for(size_t i = size_t(arg[3]); i < size_t(arg[4]); ++i)
            z[p] -= parameter[ arg[i] ];
    }
    Base* x;
    for(size_t i = 5; i < size_t(arg[1]); ++i)
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[i]) < i_z );
        x     = taylor + size_t(arg[i]) * cap_order;
        for(size_t k = p; k <= q; k++)
            z[k] += x[k];
    }
    for(size_t i = size_t(arg[1]); i < size_t(arg[2]); ++i)
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[i]) < i_z );
        x     = taylor + size_t(arg[i]) * cap_order;
        for(size_t k = p; k <= q; k++)
            z[k] -= x[k];
    }
}

/*!
Multiple direction forward mode Taylor coefficients for op = CsumOp.

This operation is
\verbatim
    z = s + x(0) + ... + x(m-1) - y(0) - ... - y(n-1).
\endverbatim

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD<Base> and computations by this routine are done using type
Base.

\param q
order ot the Taylor coefficients that we are computing.

\param r
number of directions for Taylor coefficients that we are computing.

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in taylor corresponding to z.

\param arg
-- arg[0]
parameter[arg[0]] is the parameter value s in this cummunative summation.

-- arg[1]
end in arg of addition variables in summation.
arg[5] , ... , arg[arg[1]-1] correspond to x(0), ... , x(m-1)

-- arg[2]
end in arg of subtraction variables in summation.
arg[arg[1]] , ... , arg[arg[2]-1] correspond to y(0), ... , y(n-1)

-- arg[3]
end in arg of addition dynamic parameters in summation.

-- arg[4]
end in arg of subtraction dynamic parameters in summation.

\param num_par
is the number of parameters in parameter.

\param parameter
is the parameter vector for this operation sequence.

\param cap_order
number of colums in the matrix containing all the Taylor coefficients.

\param taylor
\b Input: taylor [ arg[5+i]*((cap_order-1)*r + 1) + 0 ]
for i = 0, ..., m-1
is the 0-th order Taylor coefficient corresponding to x(i) and
taylor [ arg[5+i]*((cap_order-1)*r + 1) + (q-1)*r + ell + 1 ]
for i = 0, ..., m-1,
ell = 0 , ... , r-1
is the q-th order Taylor coefficient corresponding to x(i)
and direction ell.
\n
\b Input: taylor [ arg[arg[1]+1]*((cap_order-1)*r + 1) + 0 ]
for i = 0, ..., n-1
is the 0-th order Taylor coefficient corresponding to y(i) and
taylor [ arg[arg[1]+1]*((cap_order-1)*r + 1) + (q-1)*r + ell + 1 ]
for i = 0, ..., n-1,
ell = 0 , ... , r-1
is the q-th order Taylor coefficient corresponding to y(i)
and direction ell.
\n
\b Output: taylor [ i_z*((cap_order-1)*r+1) + (q-1)*r + ell + 1 ]
is the q-th order Taylor coefficient corresponding to z
for direction ell = 0 , ... , r-1.
*/
template <class Base>
void forward_csum_op_dir(
    size_t        q           ,
    size_t        r           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    size_t        num_par     ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{   Base zero(0);

    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumRes(CSumOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( 0 < q );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
    CPPAD_ASSERT_UNKNOWN(
        arg[arg[4]] == arg[4]
    );

    // Taylor coefficients corresponding to result
    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    size_t m                  = (q-1)*r + 1;
    Base* z = taylor + i_z * num_taylor_per_var + m;
    for(size_t ell = 0; ell < r; ell++)
        z[ell] = zero;
    Base* x;
    for(size_t i = 5; i < size_t(arg[1]); ++i)
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[i]) < i_z );
        x = taylor + size_t(arg[i]) * num_taylor_per_var + m;
        for(size_t ell = 0; ell < r; ell++)
            z[ell] += x[ell];
    }
    for(size_t i = size_t(arg[1]); i < size_t(arg[2]); ++i)
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[i]) < i_z );
        x = taylor + size_t(arg[i]) * num_taylor_per_var + m;
        for(size_t ell = 0; ell < r; ell++)
            z[ell] -= x[ell];
    }
}

/*!
Compute reverse mode Taylor coefficients for result of op = CsumOp.

This operation is
\verbatim
    z = q + x(0) + ... + x(m-1) - y(0) - ... - y(n-1).
    H(y, x, w, ...) = G[ z(x, y), y, x, w, ... ]
\endverbatim

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
Base.

\param d
order the highest order Taylor coefficient that we are computing
the partial derivatives with respect to.

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in taylor corresponding to z.

\param arg
-- arg[0]
parameter[arg[0]] is the parameter value s in this cummunative summation.

-- arg[1]
end in arg of addition variables in summation.
arg[5] , ... , arg[arg[1]-1] correspond to x(0), ... , x(m-1)

-- arg[2]
end in arg of subtraction variables in summation.
arg[arg[1]] , ... , arg[arg[2]-1] correspond to y(0), ... , y(n-1)

-- arg[3]
end in arg of addition dynamic parameters in summation.

-- arg[4]
end in arg of subtraction dynamic parameters in summation.

\param nc_partial
number of colums in the matrix containing all the partial derivatives.

\param partial
\b Input: partial [ arg[5+i] * nc_partial + k ]
for i = 0, ..., m-1
and k = 0 , ... , d
is the partial derivative of G(z, y, x, w, ...) with respect to the
k-th order Taylor coefficient corresponding to x(i)
\n
\b Input: partial [ arg[arg[1]+1] * nc_partial + k ]
for i = 0, ..., n-1
and k = 0 , ... , d
is the partial derivative of G(z, y, x, w, ...) with respect to the
k-th order Taylor coefficient corresponding to y(i)
\n
\b Input: partial [ i_z * nc_partial + k ]
for i = 0, ..., n-1
and k = 0 , ... , d
is the partial derivative of G(z, y, x, w, ...) with respect to the
k-th order Taylor coefficient corresponding to z.
\n
\b Output: partial [ arg[5+i] * nc_partial + k ]
for i = 0, ..., m-1
and k = 0 , ... , d
is the partial derivative of H(y, x, w, ...) with respect to the
k-th order Taylor coefficient corresponding to x(i)
\n
\b Output: partial [ arg[arg[1]+1] * nc_partial + k ]
for i = 0, ..., n-1
and k = 0 , ... , d
is the partial derivative of H(y, x, w, ...) with respect to the
k-th order Taylor coefficient corresponding to y(i)
*/

template <class Base>
void reverse_csum_op(
    size_t        d           ,
    size_t        i_z         ,
    const addr_t* arg         ,
    size_t        nc_partial  ,
    Base*         partial     )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumRes(CSumOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( d < nc_partial );

    // Taylor coefficients and partial derivative corresponding to result
    Base* pz = partial + i_z * nc_partial;
    Base* px;
    size_t d1 = d + 1;
    for(size_t i = 5; i < size_t(arg[1]); ++i)
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[i]) < i_z );
        px    = partial + size_t(arg[i]) * nc_partial;
        size_t k = d1;
        while(k--)
            px[k] += pz[k];
    }
    for(size_t i = size_t(arg[1]); i < size_t(arg[2]); ++i)
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[i]) < i_z );
        px    = partial + size_t(arg[i]) * nc_partial;
        size_t k = d1;
        while(k--)
            px[k] -= pz[k];
    }
}


/*!
Forward mode Jacobian sparsity pattern for CSumOp operator.

This operation is
\verbatim
    z = q + x(0) + ... + x(m-1) - y(0) - ... - y(n-1).
\endverbatim

\tparam Vector_set
is the type used for vectors of sets. It can be either
sparse::pack_setvec or sparse::list_setvec.

\param i_z
variable index corresponding to the result for this operation;
i.e. the index in sparsity corresponding to z.

\param arg
-- arg[0]
parameter[arg[0]] is the parameter value s in this cummunative summation.

-- arg[1]
end in arg of addition variables in summation.
arg[5] , ... , arg[arg[1]-1] correspond to x(0), ... , x(m-1)

-- arg[2]
end in arg of subtraction variables in summation.
arg[arg[1]] , ... , arg[arg[2]-1] correspond to y(0), ... , y(n-1)

-- arg[3]
end in arg of addition dynamic parameters in summation.

-- arg[4]
end in arg of subtraction dynamic parameters in summation.

\param sparsity
\b Input:
For i = 0, ..., m-1,
the set with index arg[5+i] in sparsity
is the sparsity bit pattern for x(i).
This identifies which of the independent variables the variable
x(i) depends on.
\n
\b Input:
For i = 0, ..., n-1,
the set with index arg[2+arg[0]+i] in sparsity
is the sparsity bit pattern for x(i).
This identifies which of the independent variables the variable
y(i) depends on.
\n
\b Output:
The set with index i_z in sparsity
is the sparsity bit pattern for z.
This identifies which of the independent variables the variable z
depends on.
*/

template <class Vector_set>
void forward_sparse_jacobian_csum_op(
    size_t           i_z         ,
    const addr_t*    arg         ,
    Vector_set&      sparsity    )
{   sparsity.clear(i_z);

    for(size_t i = 5; i < size_t(arg[2]); ++i)
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[i]) < i_z );
        sparsity.binary_union(
            i_z        , // index in sparsity for result
            i_z        , // index in sparsity for left operand
            size_t(arg[i]), // index for right operand
            sparsity     // sparsity vector for right operand
        );
    }
}

/*!
Reverse mode Jacobian sparsity pattern for CSumOp operator.

This operation is
\verbatim
    z = q + x(0) + ... + x(m-1) - y(0) - ... - y(n-1).
    H(y, x, w, ...) = G[ z(x, y), y, x, w, ... ]
\endverbatim

\tparam Vector_set
is the type used for vectors of sets. It can be either
sparse::pack_setvec or sparse::list_setvec.

\param i_z
variable index corresponding to the result for this operation;
i.e. the index in sparsity corresponding to z.

\param arg
-- arg[0]
parameter[arg[0]] is the parameter value s in this cummunative summation.

-- arg[1]
end in arg of addition variables in summation.
arg[5] , ... , arg[arg[1]-1] correspond to x(0), ... , x(m-1)

-- arg[2]
end in arg of subtraction variables in summation.
arg[arg[1]] , ... , arg[arg[2]-1] correspond to y(0), ... , y(n-1)

-- arg[3]
end in arg of addition dynamic parameters in summation.

-- arg[4]
end in arg of subtraction dynamic parameters in summation.

\param sparsity
For i = 0, ..., m-1,
the set with index arg[5+i] in sparsity
is the sparsity bit pattern for x(i).
This identifies which of the dependent variables depend on x(i).
On input, the sparsity patter corresponds to G,
and on ouput it corresponds to H.
\n
For i = 0, ..., m-1,
the set with index arg[2+arg[0]+i] in sparsity
is the sparsity bit pattern for y(i).
This identifies which of the dependent variables depend on y(i).
On input, the sparsity patter corresponds to G,
and on ouput it corresponds to H.
\n
\b Input:
The set with index i_z in sparsity
is the sparsity bit pattern for z.
On input it corresponds to G and on output it is undefined.
*/

template <class Vector_set>
void reverse_sparse_jacobian_csum_op(
    size_t           i_z         ,
    const addr_t*    arg         ,
    Vector_set&      sparsity    )
{
    for(size_t i = 5; i < size_t(arg[2]); ++i)
    {
        CPPAD_ASSERT_UNKNOWN( size_t(arg[i]) < i_z );
        sparsity.binary_union(
            size_t(arg[i]), // index in sparsity for result
            size_t(arg[i]), // index in sparsity for left operand
            i_z        , // index for right operand
            sparsity     // sparsity vector for right operand
        );
    }
}
/*!
Reverse mode Hessian sparsity pattern for CSumOp operator.

This operation is
\verbatim
    z = q + x(0) + ... + x(m-1) - y(0) - ... - y(n-1).
    H(y, x, w, ...) = G[ z(x, y), y, x, w, ... ]
\endverbatim

\tparam Vector_set
is the type used for vectors of sets. It can be either
sparse::pack_setvec or sparse::list_setvec.

\param i_z
variable index corresponding to the result for this operation;
i.e. the index in sparsity corresponding to z.

\param arg
-- arg[0]
parameter[arg[0]] is the parameter value s in this cummunative summation.

-- arg[1]
end in arg of addition variables in summation.
arg[5] , ... , arg[arg[1]-1] correspond to x(0), ... , x(m-1)

-- arg[2]
end in arg of subtraction variables in summation.
arg[arg[1]] , ... , arg[arg[2]-1] correspond to y(0), ... , y(n-1)

-- arg[3]
end in arg of addition dynamic parameters in summation.

-- arg[4]
end in arg of subtraction dynamic parameters in summation.

\param rev_jacobian
rev_jacobian[i_z]
is all false (true) if the Jabobian of G with respect to z must be zero
(may be non-zero).
\n
\n
For i = 0, ..., m-1
rev_jacobian[ arg[5+i] ]
is all false (true) if the Jacobian with respect to x(i)
is zero (may be non-zero).
On input, it corresponds to the function G,
and on output it corresponds to the function H.
\n
\n
For i = 0, ..., n-1
rev_jacobian[ arg[2+arg[0]+i] ]
is all false (true) if the Jacobian with respect to y(i)
is zero (may be non-zero).
On input, it corresponds to the function G,
and on output it corresponds to the function H.

\param rev_hes_sparsity
The set with index i_z in in rev_hes_sparsity
is the Hessian sparsity pattern for the fucntion G
where one of the partials derivative is with respect to z.
\n
\n
For i = 0, ..., m-1
The set with index arg[5+i] in rev_hes_sparsity
is the Hessian sparsity pattern
where one of the partials derivative is with respect to x(i).
On input, it corresponds to the function G,
and on output it corresponds to the function H.
\n
\n
For i = 0, ..., n-1
The set with index arg[2+arg[0]+i] in rev_hes_sparsity
is the Hessian sparsity pattern
where one of the partials derivative is with respect to y(i).
On input, it corresponds to the function G,
and on output it corresponds to the function H.
*/

template <class Vector_set>
void reverse_sparse_hessian_csum_op(
    size_t           i_z                 ,
    const addr_t*    arg                 ,
    bool*            rev_jacobian        ,
    Vector_set&      rev_hes_sparsity    )
{
    for(size_t i = 5; i < size_t(arg[2]); ++i)
    {
        CPPAD_ASSERT_UNKNOWN( size_t(arg[i]) < i_z );
        rev_hes_sparsity.binary_union(
        size_t(arg[i]), // index in sparsity for result
        size_t(arg[i]), // index in sparsity for left operand
        i_z                , // index for right operand
        rev_hes_sparsity     // sparsity vector for right operand
        );
        rev_jacobian[arg[i]] |= rev_jacobian[i_z];
    }
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
