# ifndef CPPAD_LOCAL_OP_CSKIP_OP_HPP
# define CPPAD_LOCAL_OP_CSKIP_OP_HPP
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
\file cskip_op.hpp
Zero order forward mode set which operations to skip.
*/

/*!
Zero order forward mode execution of op = CSkipOp.

\par Parameters and Variables
The terms parameter and variable depend on if we are referring to its
AD<Base> or Base value.
We use Base parameter and Base variable to refer to the
correspond Base value.
We use AD<Base> parameter and AD<Base> variable to refer to the
correspond AD<Base> value.

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD<Base> and computations by this routine are done using type Base.

\param i_z
variable index corresponding to the result of the previous operation.
This is used for error checking. To be specific,
the left and right operands for the CExpOp operation must have indexes
less than or equal this value.

\param arg [in]
\n
 arg[0]
is static cast to size_t from the enum type
\verbatim
    enum CompareOp {
        CompareLt,
        CompareLe,
        CompareEq,
        CompareGe,
        CompareGt,
        CompareNe
    }
\endverbatim
for this operation.
Note that arg[0] cannot be equal to CompareNe.
\n
\n
 arg[1] & 1
\n
If this is zero, left is an AD<Base> parameter.
Otherwise it is an AD<Base> variable.
\n
\n
 arg[1] & 2
\n
If this is zero, right is an AD<Base> parameter.
Otherwise it is an AD<Base> variable.
\n
 arg[2]
is the index corresponding to left in comparison.
\n
 arg[3]
is the index corresponding to right in comparison.
\n
 arg[4]
is the number of operations to skip if the comparison result is true.
\n
 arg[5]
is the number of operations to skip if the comparison result is false.
\n
<tt>arg[5+i]</tt>
for <tt>i = 1 , ... , arg[4]</tt> are the operations to skip if the
comparison result is true and both left and right are
identically Base parameters.
\n
<tt>arg[5+arg[4]+i]</tt>
for <tt>i = 1 , ... , arg[5]</tt> are the operations to skip if the
comparison result is false and both left and right are
identically Base parameters.

\param num_par [in]
is the total number of values in the vector parameter.

\param parameter [in]
If left is an AD<Base> parameter,
<code>parameter [ arg[2] ]</code> is its value.
If right is an AD<Base> parameter,
<code>parameter [ arg[3] ]</code> is its value.

\param cap_order [in]
number of columns in the matrix containing the Taylor coefficients.

\param taylor [in]
If left is an AD<Base> variable,
<code>taylor [ size_t(arg[2]) * cap_order + 0 ]</code>
is the zeroth order Taylor coefficient corresponding to left.
If right is an AD<Base> variable,
<code>taylor [ size_t(arg[3]) * cap_order + 0 ]</code>
is the zeroth order Taylor coefficient corresponding to right.

\param cskip_op [in,out]
is vector specifying which operations are at this point are know to be
unecessary and can be skipped.
This is both an input and an output.
*/
template <class Base>
void forward_cskip_op_0(
    size_t               i_z            ,
    const addr_t*        arg            ,
    size_t               num_par        ,
    const Base*          parameter      ,
    size_t               cap_order      ,
    Base*                taylor         ,
    bool*                cskip_op       )
{
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < size_t(CompareNe) );
    CPPAD_ASSERT_UNKNOWN( arg[1] != 0 );

    Base left, right;
    if( arg[1] & 1 )
    {   // If variable arg[2] <= i_z, it has already been computed,
        // but it will be skipped for higher orders.
        CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) <= i_z );
        left = taylor[ size_t(arg[2]) * cap_order + 0 ];
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) < num_par );
        left = parameter[ arg[2] ];
    }
    if( arg[1] & 2 )
    {   // If variable arg[3] <= i_z, it has already been computed,
        // but it will be skipped for higher orders.
        CPPAD_ASSERT_UNKNOWN( size_t(arg[3]) <= i_z );
        right = taylor[ size_t(arg[3]) * cap_order + 0 ];
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[3]) < num_par );
        right = parameter[ arg[3] ];
    }
    bool ok_to_skip = IdenticalCon(left) & IdenticalCon(right);
    if( ! ok_to_skip )
        return;

    // initialize to avoid compiler warning
    bool true_case = false;
    Base diff      = left - right;
    switch( CompareOp( arg[0] ) )
    {
        case CompareLt:
        true_case = LessThanZero(diff);
        break;

        case CompareLe:
        true_case = LessThanOrZero(diff);
        break;

        case CompareEq:
        true_case = IdenticalZero(diff);
        break;

        case CompareGe:
        true_case = GreaterThanOrZero(diff);
        break;

        case CompareGt:
        true_case = GreaterThanZero(diff);
        break;

        case CompareNe:
        true_case = ! IdenticalZero(diff);
        break;

        default:
        CPPAD_ASSERT_UNKNOWN(false);
    }
    if( true_case )
    {   for(addr_t i = 0; i < arg[4]; i++)
            cskip_op[ arg[6+i] ] = true;
    }
    else
    {   for(addr_t i = 0; i < arg[5]; i++)
            cskip_op[ arg[6+arg[4]+i] ] = true;
    }
    return;
}
} } // END_CPPAD_LOCAL_NAMESPACE
# endif
