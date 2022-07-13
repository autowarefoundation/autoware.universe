# ifndef CPPAD_LOCAL_OP_PROTOTYPE_OP_HPP
# define CPPAD_LOCAL_OP_PROTOTYPE_OP_HPP
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
\file prototype_op.hpp
Documentation for generic cases (these generic cases are never used).
*/

// ==================== Unary operators with one result ====================


/*!
Prototype for forward mode unary operator with one result (not used).

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

\param i_x
variable index corresponding to the argument for this operator;
i.e. the row index in taylor corresponding to x.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
\b Input: <code>taylor [ i_x * cap_order + k ]</code>,
for k = 0 , ... , q,
is the k-th order Taylor coefficient corresponding to x.
\n
\b Input: <code>taylor [ i_z * cap_order + k ]</code>,
for k = 0 , ... , p-1,
is the k-th order Taylor coefficient corresponding to z.
\n
\b Output: <code>taylor [ i_z * cap_order + k ]</code>,
for k = p , ... , q,
is the k-th order Taylor coefficient corresponding to z.

\par Checked Assertions
\li NumArg(op) == 1
\li NumRes(op) == 1
\li q < cap_order
\li p <= q
*/
template <class Base>
void forward_unary1_op(
    size_t p           ,
    size_t q           ,
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

/*!
Prototype for multiple direction forward mode unary operator with one result
(not used).

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param q
order of the Taylor coefficients that we are computing.

\param r
number of directions for Taylor coefficients that we are computing.

\param i_z
variable index corresponding to the last (primary) result for this operation;
i.e. the row index in taylor corresponding to z.

\param i_x
variable index corresponding to the argument for this operator;
i.e. the row index in taylor corresponding to x.

\param cap_order
maximum number of orders that will fit in the taylor array.

\par tpv
We use the notation
<code>tpv = (cap_order-1) * r + 1</code>
which is the number of Taylor coefficients per variable

\param taylor
\b Input: If x is a variable,
<code>taylor [ arg[0] * tpv + 0 ]</code>,
is the zero order Taylor coefficient for all directions and
<code>taylor [ arg[0] * tpv + (k-1)*r + ell + 1 ]</code>,
for k = 1 , ... , q,
ell = 0, ..., r-1,
is the k-th order Taylor coefficient
corresponding to x and the ell-th direction.
\n
\b Input: <code>taylor [ i_z * tpv + 0 ]</code>,
is the zero order Taylor coefficient for all directions and
<code>taylor [ i_z * tpv + (k-1)*r + ell + 1 ]</code>,
for k = 1 , ... , q-1,
ell = 0, ..., r-1,
is the k-th order Taylor coefficient
corresponding to z and the ell-th direction.
\n
\b Output:
<code>taylor [ i_z * tpv + (q-1)*r + ell + 1]</code>,
ell = 0, ..., r-1,
is the q-th order Taylor coefficient
corresponding to z and the ell-th direction.

\par Checked Assertions
\li NumArg(op) == 1
\li NumRes(op) == 2
\li i_x < i_z
\li 0 < q
\li q < cap_order
*/
template <class Base>
void forward_unary1_op_dir(
    size_t q           ,
    size_t r           ,
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

/*!
Prototype for zero order forward mode unary operator with one result (not used).
\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base .

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in taylor corresponding to z.

\param i_x
variable index corresponding to the argument for this operator;
i.e. the row index in taylor corresponding to x.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
\b Input: taylor [ i_x * cap_order + 0 ]
is the zero order Taylor coefficient corresponding to x.
\n
\b Output: taylor [ i_z * cap_order + 0 ]
is the zero order Taylor coefficient corresponding to z.

\par Checked Assertions
\li NumArg(op) == 1
\li NumRes(op) == 1
\li i_x < i_z
\li 0 < cap_order
*/
template <class Base>
void forward_unary1_op_0(
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

/*!
Prototype for reverse mode unary operator with one result (not used).

This routine is given the partial derivatives of a function
G(z , x , w, u ... )
and it uses them to compute the partial derivatives of
\verbatim
    H( x , w , u , ... ) = G[ z(x) , x , w , u , ... ]
\endverbatim

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base .

\param d
highest order Taylor coefficient that
we are computing the partial derivatives with respect to.

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in taylor to z.

\param i_x
variable index corresponding to the argument for this operation;
i.e. the row index in taylor corresponding to x.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
 taylor [ i_x * cap_order + k ]
for k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to x.
\n
 taylor [ i_z * cap_order + k ]
for k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to z.

\param nc_partial
number of colums in the matrix containing all the partial derivatives.

\param partial
\b Input: partial [ i_x * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of G( z , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for x.
\n
\b Input: partial [ i_z * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of G( z , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for z.
\n
\b Output: partial [ i_x * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of H( x , w , u , ... ) with respect to
the k-th order Taylor coefficient for x.
\n
\b Output: partial [ i_z * nc_partial + k ]
for k = 0 , ... , d
may be used as work space; i.e., may change in an unspecified manner.


\par Checked Assumptions
\li NumArg(op) == 1
\li NumRes(op) == 1
\li i_x < i_z
\li d < cap_order
\li d < nc_partial
*/
template <class Base>
void reverse_unary1_op(
    size_t      d            ,
    size_t      i_z          ,
    size_t      i_x          ,
    size_t      cap_order    ,
    const Base* taylor       ,
    size_t      nc_partial   ,
    Base*       partial      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

// ==================== Unary operators with two results ====================

/*!
Prototype for forward mode unary operator with two results (not used).

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param p
lowest order of the Taylor coefficients that we are computing.

\param q
highest order of the Taylor coefficients that we are computing.

\param i_z
variable index corresponding to the last (primary) result for this operation;
i.e. the row index in taylor corresponding to z.
The auxillary result is called y has index i_z - 1.

\param i_x
variable index corresponding to the argument for this operator;
i.e. the row index in taylor corresponding to x.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
\b Input: <code>taylor [ i_x * cap_order + k ]</code>
for k = 0 , ... , q,
is the k-th order Taylor coefficient corresponding to x.
\n
\b Input: <code>taylor [ i_z * cap_order + k ]</code>
for k = 0 , ... , p - 1,
is the k-th order Taylor coefficient corresponding to z.
\n
\b Input: <code>taylor [ ( i_z - 1) * cap_order + k ]</code>
for k = 0 , ... , p-1,
is the k-th order Taylor coefficient corresponding to the auxillary result y.
\n
\b Output: <code>taylor [ i_z * cap_order + k ]</code>,
for k = p , ... , q,
is the k-th order Taylor coefficient corresponding to z.
\n
\b Output: <code>taylor [ ( i_z - 1 ) * cap_order + k ]</code>,
for k = p , ... , q,
is the k-th order Taylor coefficient corresponding to
the autillary result y.

\par Checked Assertions
\li NumArg(op) == 1
\li NumRes(op) == 2
\li i_x + 1 < i_z
\li q < cap_order
\li p <= q
*/
template <class Base>
void forward_unary2_op(
    size_t p           ,
    size_t q           ,
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

/*!
Prototype for multiple direction forward mode unary operator with two results
(not used).

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param q
order of the Taylor coefficients that we are computing.

\param r
number of directions for Taylor coefficients that we are computing.

\param i_z
variable index corresponding to the last (primary) result for this operation;
i.e. the row index in taylor corresponding to z.
The auxillary result is called y has index i_z - 1.

\param i_x
variable index corresponding to the argument for this operator;
i.e. the row index in taylor corresponding to x.

\param cap_order
maximum number of orders that will fit in the taylor array.

\par tpv
We use the notation
<code>tpv = (cap_order-1) * r + 1</code>
which is the number of Taylor coefficients per variable

\param taylor
\b Input: <code>taylor [ i_x * tpv + 0 ]</code>
is the zero order Taylor coefficient for all directions and
<code>taylor [ i_x * tpv + (k-1)*r + ell + 1</code>
for k = 1 , ... , q,
ell = 0 , ..., r-1,
is the k-th order Taylor coefficient
corresponding to x and the ell-th direction.
\n
\b Input: <code>taylor [ i_z * tpv + 0 ]</code>,
is the zero order Taylor coefficient for all directions and
<code>taylor [ i_z * tpv + (k-1)*r + ell + 1 ]</code>,
for k = 1 , ... , q-1,
ell = 0, ..., r-1,
is the k-th order Taylor coefficient
corresponding to z and the ell-th direction.
\n
\b Input: <code>taylor [ (i_z-1) * tpv + 0 ]</code>,
is the zero order Taylor coefficient for all directions and
<code>taylor [ (i_z-1) * tpv + (k-1)*r + ell + 1 ]</code>,
for k = 1 , ... , q-1,
ell = 0, ..., r-1,
is the k-th order Taylor coefficient
corresponding to the auxillary result y and the ell-th direction.
\n
\b Output:
<code>taylor [ i_z * tpv + (q-1)*r + ell + 1]</code>,
ell = 0, ..., r-1,
is the q-th order Taylor coefficient
corresponding to z and the ell-th direction.

\par Checked Assertions
\li NumArg(op) == 1
\li NumRes(op) == 2
\li i_x + 1 < i_z
\li 0 < q
\li q < cap_order
*/
template <class Base>
void forward_unary2_op_dir(
    size_t q           ,
    size_t r           ,
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

/*!
Prototype for zero order forward mode unary operator with two results (not used).
\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base .

\param i_z
variable index corresponding to the last (primary) result for this operation;
i.e. the row index in taylor corresponding to z.
The auxillary result is called y and has index i_z - 1.

\param i_x
variable index corresponding to the argument for this operator;
i.e. the row index in taylor corresponding to x.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
\b Input: taylor [ i_x * cap_order + 0 ]
is the zero order Taylor coefficient corresponding to x.
\n
\b Output: taylor [ i_z * cap_order + 0 ]
is the zero order Taylor coefficient corresponding to z.
\n
\b Output: taylor [ ( i_z - 1 ) * cap_order + j ]
is the j-th order Taylor coefficient corresponding to
the autillary result y.

\par Checked Assertions
\li NumArg(op) == 1
\li NumRes(op) == 2
\li i_x + 1 < i_z
\li j < cap_order
*/
template <class Base>
void forward_unary2_op_0(
    size_t i_z         ,
    size_t i_x         ,
    size_t cap_order   ,
    Base*  taylor      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

/*!
Prototype for reverse mode unary operator with two results (not used).

This routine is given the partial derivatives of a function
G( z , y , x , w , ... )
and it uses them to compute the partial derivatives of
\verbatim
    H( x , w , u , ... ) = G[ z(x) , y(x), x , w , u , ... ]
\endverbatim

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base .

\param d
highest order Taylor coefficient that
we are computing the partial derivatives with respect to.

\param i_z
variable index corresponding to the last (primary) result for this operation;
i.e. the row index in taylor to z.
The auxillary result is called y and has index i_z - 1.

\param i_x
variable index corresponding to the argument for this operation;
i.e. the row index in taylor corresponding to x.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
 taylor [ i_x * cap_order + k ]
for k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to x.
\n
 taylor [ i_z * cap_order + k ]
for k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to z.
\n
 taylor [ ( i_z - 1) * cap_order + k ]
for k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to
the auxillary variable y.

\param nc_partial
number of colums in the matrix containing all the partial derivatives.

\param partial
\b Input: partial [ i_x * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of
G( z , y , x , w , u , ... )
with respect to the k-th order Taylor coefficient for x.
\n
\b Input: partial [ i_z * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of G( z , y , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for z.
\n
\b Input: partial [ ( i_z - 1) * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of G( z , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for the auxillary variable y.
\n
\b Output: partial [ i_x * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of H( x , w , u , ... ) with respect to
the k-th order Taylor coefficient for x.
\n
\b Output: partial [ ( i_z - j ) * nc_partial + k ]
for j = 0 , 1 , and for k = 0 , ... , d
may be used as work space; i.e., may change in an unspecified manner.


\par Checked Assumptions
\li NumArg(op) == 1
\li NumRes(op) == 2
\li i_x + 1 < i_z
\li d < cap_order
\li d < nc_partial
*/
template <class Base>
void reverse_unary2_op(
    size_t      d            ,
    size_t      i_z          ,
    size_t      i_x          ,
    size_t      cap_order    ,
    const Base* taylor       ,
    size_t      nc_partial   ,
    Base*       partial      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}
// =================== Binary operators with one result ====================

/*!
Prototype forward mode x op y (not used)

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
 arg[0]
index corresponding to the left operand for this operator;
i.e. the index corresponding to x.
\n
 arg[1]
index corresponding to the right operand for this operator;
i.e. the index corresponding to y.

\param parameter
If x is a parameter, parameter [ arg[0] ]
is the value corresponding to x.
\n
If y is a parameter, parameter [ arg[1] ]
is the value corresponding to y.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
\b Input: If x is a variable,
<code>taylor [ size_t(arg[0]) * cap_order + k ]</code>,
for k = 0 , ... , q,
is the k-th order Taylor coefficient corresponding to x.
\n
\b Input: If y is a variable,
<code>taylor [ size_t(arg[1]) * cap_order + k ]</code>,
for k = 0 , ... , q,
is the k-th order Taylor coefficient corresponding to y.
\n
\b Input: <code>taylor [ i_z * cap_order + k ]</code>,
for k = 0 , ... , p-1,
is the k-th order Taylor coefficient corresponding to z.
\n
\b Output: <code>taylor [ i_z * cap_order + k ]</code>,
for k = p, ... , q,
is the k-th order Taylor coefficient corresponding to z.

\par Checked Assertions
\li NumArg(op) == 2
\li NumRes(op) == 1
\li q <  cap_order
\li p <=  q
*/
template <class Base>
void forward_binary_op(
    size_t        p          ,
    size_t        q          ,
    size_t        i_z        ,
    const addr_t* arg        ,
    const Base*   parameter  ,
    size_t        cap_order  ,
    Base*         taylor     )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

/*!
Prototype multiple direction forward mode x op y (not used)

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param q
is the order of the Taylor coefficients that we are computing.

\param r
number of directions for Taylor coefficients that we are computing

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in taylor corresponding to z.

\param arg
 arg[0]
index corresponding to the left operand for this operator;
i.e. the index corresponding to x.
\n
 arg[1]
index corresponding to the right operand for this operator;
i.e. the index corresponding to y.

\param parameter
If x is a parameter, parameter [ arg[0] ]
is the value corresponding to x.
\n
If y is a parameter, parameter [ arg[1] ]
is the value corresponding to y.

\param cap_order
maximum number of orders that will fit in the taylor array.

\par tpv
We use the notation
<code>tpv = (cap_order-1) * r + 1</code>
which is the number of Taylor coefficients per variable

\param taylor
\b Input: If x is a variable,
<code>taylor [ arg[0] * tpv + 0 ]</code>,
is the zero order Taylor coefficient for all directions and
<code>taylor [ arg[0] * tpv + (k-1)*r + ell + 1 ]</code>,
for k = 1 , ... , q,
ell = 0, ..., r-1,
is the k-th order Taylor coefficient
corresponding to x and the ell-th direction.
\n
\b Input: If y is a variable,
<code>taylor [ arg[1] * tpv + 0 ]</code>,
is the zero order Taylor coefficient for all directions and
<code>taylor [ arg[1] * tpv + (k-1)*r + ell + 1 ]</code>,
for k = 1 , ... , q,
ell = 0, ..., r-1,
is the k-th order Taylor coefficient
corresponding to y and the ell-th direction.
\n
\b Input: <code>taylor [ i_z * tpv + 0 ]</code>,
is the zero order Taylor coefficient for all directions and
<code>taylor [ i_z * tpv + (k-1)*r + ell + 1 ]</code>,
for k = 1 , ... , q-1,
ell = 0, ..., r-1,
is the k-th order Taylor coefficient
corresponding to z and the ell-th direction.
\n
\b Output:
<code>taylor [ i_z * tpv + (q-1)*r + ell + 1]</code>,
ell = 0, ..., r-1,
is the q-th order Taylor coefficient
corresponding to z and the ell-th direction.

\par Checked Assertions
\li NumArg(op) == 2
\li NumRes(op) == 1
\li 0 < q <  cap_order
*/
template <class Base>
void forward_binary_op_dir(
    size_t        q          ,
    size_t        r          ,
    size_t        i_z        ,
    const addr_t* arg        ,
    const Base*   parameter  ,
    size_t        cap_order  ,
    Base*         taylor     )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}


/*!
Prototype zero order forward mode x op y (not used)

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in taylor corresponding to z.

\param arg
 arg[0]
index corresponding to the left operand for this operator;
i.e. the index corresponding to x.
\n
 arg[1]
index corresponding to the right operand for this operator;
i.e. the index corresponding to y.

\param parameter
If x is a parameter, parameter [ arg[0] ]
is the value corresponding to x.
\n
If y is a parameter, parameter [ arg[1] ]
is the value corresponding to y.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
\b Input: If x is a variable, taylor [ arg[0] * cap_order + 0 ]
is the zero order Taylor coefficient corresponding to x.
\n
\b Input: If y is a variable, taylor [ arg[1] * cap_order + 0 ]
is the zero order Taylor coefficient corresponding to y.
\n
\b Output: taylor [ i_z * cap_order + 0 ]
is the zero order Taylor coefficient corresponding to z.

\par Checked Assertions
\li NumArg(op) == 2
\li NumRes(op) == 1
*/
template <class Base>
void forward_binary_op_0(
    size_t        i_z         ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

/*!
Prototype for reverse mode binary operator x op y (not used).

This routine is given the partial derivatives of a function
G( z , y , x , w , ... )
and it uses them to compute the partial derivatives of
\verbatim
    H( y , x , w , u , ... ) = G[ z(x , y) , y , x , w , u , ... ]
\endverbatim

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base .

\param d
highest order Taylor coefficient that
we are computing the partial derivatives with respect to.

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in taylor corresponding to z.

\param arg
 arg[0]
index corresponding to the left operand for this operator;
i.e. the index corresponding to x.
\n
 arg[1]
index corresponding to the right operand for this operator;
i.e. the index corresponding to y.

\param parameter
If x is a parameter, parameter [ arg[0] ]
is the value corresponding to x.
\n
If y is a parameter, parameter [ arg[1] ]
is the value corresponding to y.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
 taylor [ i_z * cap_order + k ]
for k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to z.
\n
If x is a variable, taylor [ arg[0] * cap_order + k ]
for k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to x.
\n
If y is a variable, taylor [ arg[1] * cap_order + k ]
for k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to y.

\param nc_partial
number of colums in the matrix containing all the partial derivatives.

\param partial
\b Input: partial [ i_z * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of
G( z , y , x , w , u , ... )
with respect to the k-th order Taylor coefficient for z.
\n
\b Input: If x is a variable, partial [ arg[0] * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of G( z , y , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for x.
\n
\b Input: If y is a variable, partial [ arg[1] * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of G( z , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for the auxillary variable y.
\n
\b Output: If x is a variable, partial [ arg[0] * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of H( y , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for x.
\n
\b Output: If y is a variable, partial [ arg[1] * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of H( y , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for y.
\n
\b Output: partial [ i_z * nc_partial + k ]
for k = 0 , ... , d
may be used as work space; i.e., may change in an unspecified manner.

\par Checked Assumptions
\li NumArg(op) == 2
\li NumRes(op) == 1
\li If x is a variable, arg[0] < i_z
\li If y is a variable, arg[1] < i_z
\li d < cap_order
\li d < nc_partial
*/
template <class Base>
void reverse_binary_op(
    size_t      d            ,
    size_t      i_z          ,
    addr_t*     arg          ,
    const Base* parameter    ,
    size_t      cap_order    ,
    const Base* taylor       ,
    size_t      nc_partial   ,
    Base*       partial      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}
// ======================= Pow Function ===================================
/*!
Prototype for forward mode z = pow(x, y) (not used).

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param p
lowest order of the Taylor coefficient that we are computing.

\param q
highest order of the Taylor coefficient that we are computing.

\param i_z
variable index corresponding to the last (primary) result for this operation;
i.e. the row index in taylor corresponding to z.
Note that there are three results for this operation,
below they are referred to as z_0, z_1, z_2 and correspond to
\verbatim
    z_0 = log(x)
    z_1 = z0 * y
    z_2 = exp(z1)
\endverbatim
It follows that the final result is equal to z; i.e., z = z_2 = pow(x, y).

\param arg
 arg[0]
index corresponding to the left operand for this operator;
i.e. the index corresponding to x.
\n
 arg[1]
index corresponding to the right operand for this operator;
i.e. the index corresponding to y.

\param parameter
If x is a parameter, parameter [ arg[0] ]
is the value corresponding to x.
\n
If y is a parameter, parameter [ arg[1] ]
is the value corresponding to y.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
\b Input: If x is a variable,
<code>taylor [ size_t(arg[0]) * cap_order + k ]</code>
for k = 0 , ... , q,
is the k-th order Taylor coefficient corresponding to x.
\n
\b Input: If y is a variable,
<code>taylor [ size_t(arg[1]) * cap_order + k ]</code>
for k = 0 , ... , q
is the k-th order Taylor coefficient corresponding to y.
\n
\b Input: <code>taylor [ (i_z-2+j) * cap_order + k ]</code>,
for j = 0, 1, 2 , for k = 0 , ... , p-1,
is the k-th order Taylor coefficient corresponding to z_j.
\n
\b Output: <code>taylor [ (i_z-2+j) * cap_order + k ]</code>,
is the k-th order Taylor coefficient corresponding to z_j.

\par Checked Assertions
\li NumArg(op) == 2
\li NumRes(op) == 3
\li If x is a variable, arg[0] < i_z - 2
\li If y is a variable, arg[1] < i_z - 2
\li q < cap_order
\li p <= q
*/
template <class Base>
void forward_pow_op(
    size_t        p          ,
    size_t        q          ,
    size_t        i_z        ,
    const addr_t* arg        ,
    const Base*   parameter  ,
    size_t        cap_order  ,
    Base*         taylor     )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}
/*!
Prototype for multiple direction forward mode z = pow(x, y) (not used).

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param q
order of the Taylor coefficient that we are computing.

\param r
is the number of Taylor coefficient directions that we are computing

\param i_z
variable index corresponding to the last (primary) result for this operation;
i.e. the row index in taylor corresponding to z.
Note that there are three results for this operation,
below they are referred to as z_0, z_1, z_2 and correspond to
\verbatim
    z_0 = log(x)
    z_1 = z0 * y
    z_2 = exp(z1)
\endverbatim
It follows that the final result is equal to z; i.e., z = z_2 = pow(x, y).

\param arg
 arg[0]
index corresponding to the left operand for this operator;
i.e. the index corresponding to x.
\n
 arg[1]
index corresponding to the right operand for this operator;
i.e. the index corresponding to y.

\param parameter
If x is a parameter, parameter [ arg[0] ]
is the value corresponding to x.
\n
If y is a parameter, parameter [ arg[1] ]
is the value corresponding to y.

\param cap_order
maximum number of orders that will fit in the taylor array.

\par tpv
We use the notation
<code>tpv = (cap_order-1) * r + 1</code>
which is the number of Taylor coefficients per variable

\param taylor
\b Input: If x is a variable,
<code>taylor [ arg[0] * tpv + 0 ]</code>
is the zero order coefficient corresponding to x and
<code>taylor [ arg[0] * tpv + (k-1)*r+1+ell ]</code>
for k = 1 , ... , q,
ell = 0 , ... , r-1,
is the k-th order Taylor coefficient corresponding to x
for the ell-th direction.
\n
\n
\b Input: If y is a variable,
<code>taylor [ arg[1] * tpv + 0 ]</code>
is the zero order coefficient corresponding to y and
<code>taylor [ arg[1] * tpv + (k-1)*r+1+ell ]</code>
for k = 1 , ... , q,
ell = 0 , ... , r-1,
is the k-th order Taylor coefficient corresponding to y
for the ell-th direction.
\n
\n
\b Input:
<code>taylor [ (i_z-2+j) * tpv + 0 ]</code>,
is the zero order coefficient corresponding to z_j and
<code>taylor [ (i_z-2+j) * tpv + (k-1)*r+1+ell ]</code>,
for j = 0, 1, 2 , k = 0 , ... , q-1, ell = 0, ... , r-1,
is the k-th order Taylor coefficient corresponding to z_j
for the ell-th direction.
\n
\n
\b Output:
<code>taylor [ (i_z-2+j) * tpv + (q-1)*r+1+ell ]</code>,
for j = 0, 1, 2 , ell = 0, ... , r-1,
is the q-th order Taylor coefficient corresponding to z_j
for the ell-th direction.

\par Checked Assertions
\li NumArg(op) == 2
\li NumRes(op) == 3
\li If x is a variable, arg[0] < i_z - 2
\li If y is a variable, arg[1] < i_z - 2
\li 0 < q
\li q < cap_order
*/
template <class Base>
void forward_pow_op_dir(
    size_t        q          ,
    size_t        r          ,
    size_t        i_z        ,
    const addr_t* arg        ,
    const Base*   parameter  ,
    size_t        cap_order  ,
    Base*         taylor     )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}
/*!
Prototype for zero order forward mode z = pow(x, y) (not used).

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param i_z
variable index corresponding to the last (primary) result for this operation;
i.e. the row index in taylor corresponding to z.
Note that there are three results for this operation,
below they are referred to as z_0, z_1, z_2 and correspond to
\verbatim
    z_0 = log(x)
    z_1 = z0 * y
    z_2 = exp(z1)
\endverbatim
It follows that the final result is equal to z; i.e., z = z_2 = pow(x, y).

\param arg
 arg[0]
index corresponding to the left operand for this operator;
i.e. the index corresponding to x.
\n
 arg[1]
index corresponding to the right operand for this operator;
i.e. the index corresponding to y.

\param parameter
If x is a parameter, parameter [ arg[0] ]
is the value corresponding to x.
\n
If y is a parameter, parameter [ arg[1] ]
is the value corresponding to y.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
\b Input: If x is a variable, taylor [ arg[0] * cap_order + 0 ]
is the zero order Taylor coefficient corresponding to x.
\n
\b Input: If y is a variable, taylor [ arg[1] * cap_order + 0 ]
is the k-th order Taylor coefficient corresponding to y.
\n
\b Output: taylor [ (i_z - 2 + j) * cap_order + 0 ]
is the zero order Taylor coefficient corresponding to z_j.

\par Checked Assertions
\li NumArg(op) == 2
\li NumRes(op) == 3
\li If x is a variable, arg[0] < i_z - 2
\li If y is a variable, arg[1] < i_z - 2
*/
template <class Base>
void forward_pow_op_0(
    size_t        i_z        ,
    const addr_t* arg        ,
    const Base*   parameter  ,
    size_t        cap_order  ,
    Base*         taylor     )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}
/*!
Prototype for reverse mode z = pow(x, y) (not used).

This routine is given the partial derivatives of a function
G( z , y , x , w , ... )
and it uses them to compute the partial derivatives of
\verbatim
    H( y , x , w , u , ... ) = G[ pow(x , y) , y , x , w , u , ... ]
\endverbatim

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base .

\param d
highest order Taylor coefficient that
we are computing the partial derivatives with respect to.

\param i_z
variable index corresponding to the last (primary) result for this operation;
i.e. the row index in taylor corresponding to z.
Note that there are three results for this operation,
below they are referred to as z_0, z_1, z_2 and correspond to
\verbatim
    z_0 = log(x)
    z_1 = z0 * y
    z_2 = exp(z1)
\endverbatim
It follows that the final result is equal to z; i.e., z = z_2 = pow(x, y).

\param arg
 arg[0]
index corresponding to the left operand for this operator;
i.e. the index corresponding to x.
\n
 arg[1]
index corresponding to the right operand for this operator;
i.e. the index corresponding to y.

\param parameter
If x is a parameter, parameter [ arg[0] ]
is the value corresponding to x.
\n
If y is a parameter, parameter [ arg[1] ]
is the value corresponding to y.

\param cap_order
maximum number of orders that will fit in the taylor array.

\param taylor
 taylor [ (i_z - 2 + j) * cap_order + k ]
for j = 0, 1, 2 and k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to z_j.
\n
If x is a variable, taylor [ arg[0] * cap_order + k ]
for k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to x.
\n
If y is a variable, taylor [ arg[1] * cap_order + k ]
for k = 0 , ... , d
is the k-th order Taylor coefficient corresponding to y.

\param nc_partial
number of colums in the matrix containing all the partial derivatives.

\param partial
\b Input: partial [ (i_z - 2 + j) * nc_partial + k ]
for j = 0, 1, 2, and k = 0 , ... , d
is the partial derivative of
G( z , y , x , w , u , ... )
with respect to the k-th order Taylor coefficient for z_j.
\n
\b Input: If x is a variable, partial [ arg[0] * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of G( z , y , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for x.
\n
\b Input: If y is a variable, partial [ arg[1] * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of G( z , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for the auxillary variable y.
\n
\b Output: If x is a variable, partial [ arg[0] * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of H( y , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for x.
\n
\b Output: If y is a variable, partial [ arg[1] * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of H( y , x , w , u , ... ) with respect to
the k-th order Taylor coefficient for y.
\n
\b Output: partial [ ( i_z - j ) * nc_partial + k ]
for j = 0 , 1 , 2 and for k = 0 , ... , d
may be used as work space; i.e., may change in an unspecified manner.

\par Checked Assumptions
\li NumArg(op) == 2
\li NumRes(op) == 3
\li If x is a variable, arg[0] < i_z - 2
\li If y is a variable, arg[1] < i_z - 2
\li d < cap_order
\li d < nc_partial
*/
template <class Base>
void reverse_pow_op(
    size_t      d            ,
    size_t      i_z          ,
    addr_t*     arg          ,
    const Base* parameter    ,
    size_t      cap_order    ,
    const Base* taylor       ,
    size_t      nc_partial   ,
    Base*       partial      )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

// ==================== Sparsity Calculations ==============================
/*!
Prototype for reverse mode Hessian sparsity unary operators.

This routine is given the forward mode Jacobian sparsity patterns for x.
It is also given the reverse mode dependence of G on z.
In addition, it is given the revese mode Hessian sparsity
for the quanity of interest G(z , y , ... )
and it uses them to compute the sparsity patterns for
\verbatim
    H( x , w , u , ... ) = G[ z(x) , x , w , u , ... ]
\endverbatim

\tparam Vector_set
is the type used for vectors of sets. It can be either
sparse::pack_setvec or sparse::list_setvec.

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in sparsity corresponding to z.

\param i_x
variable index corresponding to the argument for this operator;
i.e. the row index in sparsity corresponding to x.

\param rev_jacobian
 rev_jacobian[i_z]
is all false (true) if the Jacobian of G with respect to z must be zero
(may be non-zero).
\n
\n
 rev_jacobian[i_x]
is all false (true) if the Jacobian with respect to x must be zero
(may be non-zero).
On input, it corresponds to the function G,
and on output it corresponds to the function H.

\param for_jac_sparsity
The set with index i_x in for_jac_sparsity
is the forward mode Jacobian sparsity pattern for the variable x.

\param rev_hes_sparsity
The set with index i_z in in rev_hes_sparsity
is the Hessian sparsity pattern for the fucntion G
where one of the partials derivative is with respect to z.
\n
\n
The set with index i_x in rev_hes_sparsity
is the Hessian sparsity pattern
where one of the partials derivative is with respect to x.
On input, it corresponds to the function G,
and on output it corresponds to the function H.

\par Checked Assertions:
\li i_x < i_z
*/

template <class Vector_set>
void reverse_sparse_hessian_unary_op(
    size_t              i_z               ,
    size_t              i_x               ,
    bool*               rev_jacobian      ,
    Vector_set&         for_jac_sparsity  ,
    Vector_set&         rev_hes_sparsity  )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}

/*!
Prototype for reverse mode Hessian sparsity binary operators.

This routine is given the sparsity patterns the Hessian
of a function G(z, y, x, ... )
and it uses them to compute the sparsity patterns for the Hessian of
\verbatim
    H( y, x, w , u , ... ) = G[ z(x,y) , y , x , w , u , ... ]
\endverbatim

\tparam Vector_set
is the type used for vectors of sets. It can be either
sparse::pack_setvec or sparse::list_setvec.

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in sparsity corresponding to z.

\param arg
 arg[0]
variable index corresponding to the left operand for this operator;
i.e. the set with index arg[0] in var_sparsity
is the spasity pattern correspoding to x.
\n
\n arg[1]
variable index corresponding to the right operand for this operator;
i.e. the row index in sparsity patterns corresponding to y.

\param jac_reverse
 jac_reverse[i_z]
is false (true) if the Jacobian of G with respect to z is always zero
(may be non-zero).
\n
\n
 jac_reverse[ arg[0] ]
is false (true) if the Jacobian with respect to x is always zero
(may be non-zero).
On input, it corresponds to the function G,
and on output it corresponds to the function H.
\n
\n
 jac_reverse[ arg[1] ]
is false (true) if the Jacobian with respect to y is always zero
(may be non-zero).
On input, it corresponds to the function G,
and on output it corresponds to the function H.

\param for_jac_sparsity
The set with index arg[0] in for_jac_sparsity for the
is the forward Jacobian sparsity pattern for x.
\n
\n
The set with index arg[1] in for_jac_sparsity
is the forward sparsity pattern for y.

\param rev_hes_sparsity
The set wiht index i_x in rev_hes_sparsity
is the Hessian sparsity pattern for the function G
where one of the partial derivatives is with respect to z.
\n
\n
The set with index arg[0] in  rev_hes_sparsity
is the Hessian sparsity pattern where one of the
partial derivatives is with respect to x.
On input, it corresponds to the function G,
and on output it correspondst to H.
\n
\n
The set with index arg[1] in rev_hes_sparsity
is the Hessian sparsity pattern where one of the
partial derivatives is with respect to y.
On input, it corresponds to the function G,
and on output it correspondst to H.

\par Checked Assertions:
\li arg[0] < i_z
\li arg[1] < i_z
*/
template <class Vector_set>
void reverse_sparse_hessian_binary_op(
    size_t            i_z                ,
    const addr_t*     arg                ,
    bool*             jac_reverse        ,
    Vector_set&       for_jac_sparsity   ,
    Vector_set&       rev_hes_sparsity   )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}


} } // END_CPPAD_LOCAL_NAMESPACE
# endif
