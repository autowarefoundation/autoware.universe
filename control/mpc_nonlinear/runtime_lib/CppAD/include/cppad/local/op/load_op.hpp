# ifndef CPPAD_LOCAL_OP_LOAD_OP_HPP
# define CPPAD_LOCAL_OP_LOAD_OP_HPP
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
/*
 ------------------------------------------------------------------------------
$begin load_op_var$$
$spell
    pv
    Vec
    op
    var
    isvar
    ind
    Taylor
    arg
    num
    Addr
    vecad
$$
$section Accessing an Element in a Variable VecAD Vector$$

$head See Also$$
$cref/op_code_var load/op_code_var/Load/$$.

$head Syntax$$
$codei%forward_load_%I%_op_0(
    %play%,
    %i_z%,
    %arg%,
    %parameter%,
    %cap_order%,
    %taylor%,
    %vec_ad2isvar%,
    %vec_ad2index%,
    %load_op2var%
)
%$$
where the index type $icode I$$ is $code p$$ (for parameter)
or $code v$$ (for variable).

$head Prototype$$
$srcthisfile%
    0%// BEGIN_FORWARD_LOAD_P_OP_0%// END_FORWARD_LOAD_P_OP_0%1
%$$
The prototype for $code forward_load_v_op_0$$ is the same
except for the function name.

$head Notation$$

$subhead v$$
We use $icode v$$ to denote the $cref VecAD$$ vector for this operation.

$subhead x$$
We use $icode x$$ to denote the $codei%AD%<%Base%>%$$
index for this operation.

$subhead i_vec$$
We use $icode i_vec$$ to denote the $code size_t$$ value
corresponding to $icode x$$.

$subhead n_load$$
This is the number of load instructions in this recording; i.e.,
$icode%play%->num_var_load_rec()%$$.

$subhead n_all$$
This is the number of values in the single array that includes
all the vectors together with the size of each vector; i.e.,
$icode%play%->num_var_vecad_ind_rec()%$$.

$head Addr$$
Is the type used for address on this tape.

$head Base$$
base type for the operator; i.e., this operation was recorded
using AD<Base> and computations by this routine are done using type Base.

$head play$$
is the tape that this operation appears in.
This is for error detection and not used when NDEBUG is defined.

$head i_z$$
is the AD variable index corresponding to the result of this load operation.

$head arg$$

$subhead arg[0]$$
is the offset of this VecAD vector relative to the beginning
of the $icode vec_ad2isvar$$ and $icode vec_ad2index$$ arrays.

$subhead arg[1]$$
If this is
$code forward_load_p_op_0$$ ($code forward_load_v_op_0$$)
$icode%arg%[%1%]%$$ is the parameter index (variable index)
corresponding to $cref/i_vec/load_op_var/Notation/i_vec/$$.

$subhead arg[2]$$
Is the index of this VecAD load instruction in the
$icode load_op2var$$ array.

$head parameter$$
This is the vector of parameters for this recording which has size
$icode%play%->num_par_rec()%$$.

$head cap_order$$
number of columns in the matrix containing the Taylor coefficients.

$head taylor$$
Is the matrix of Taylor coefficients.

$subhead Input$$
In the $code forward_load_v_op_0$$ case,
$codei%
    size_t( %taylor%[ %arg%[1]% * %cap_order% + 0 ] )
%$$
is the index in this VecAD vector.

$subhead Output$$
$icode%taylor%[ %i_z% * %cap_order% + 0 ]%$$
is set to the zero order Taylor coefficient for the result of this operator.

$head vec_ad2isvar$$
This vector has size $icode n_all$$.
If $icode%vec_ad2isvar%[ %arg%[%0%] + %i_vec% ]%$$ is false (true),
the vector element is parameter (variable).

$subhead i_pv$$
If this element is a parameter (variable),
$codei%
    %i_pv% = %vec_ad2index%[ %arg%[%0%] + %i_vec% ]
%$$
is the corresponding parameter (variable) index;

$head vec_ad2index$$
This array has size $icode n_all$$
The value $icode%vec_ad2index%[ %arg%[0] - 1 ]%$$
is the number of elements in the user vector containing this load.
$icode%vec_ad2index%[%i_pv%]%$$ is the variable or
parameter index for this element,

$head load_op2var$$
is a vector with size $icode n_load$$.
The input value of its elements does not matter.
If the result of this load is a variable,
$codei%
    %load_op2var%[%arg%[2]] = %i_pv%
%$$
Otherwise,
$codei%
    %load_op2var%[%arg%[2]] = 0
%$$

$end
*/
// BEGIN_FORWARD_LOAD_P_OP_0
template <class Addr, class Base>
void forward_load_p_op_0(
    const local::player<Base>* play ,
    size_t         i_z              ,
    const Addr*    arg              ,
    const Base*    parameter        ,
    size_t         cap_order        ,
    Base*          taylor           ,
    const bool*    vec_ad2isvar     ,
    const size_t*  vec_ad2index     ,
    Addr*          load_op2var   )
// END_FORWARD_LOAD_P_OP_0
{   CPPAD_ASSERT_UNKNOWN( NumArg(LdpOp) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(LdpOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < play->num_par_rec() );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) < play->num_var_load_rec() );
    CPPAD_ASSERT_UNKNOWN(
        size_t( std::numeric_limits<addr_t>::max() ) >= i_z
    );

    addr_t i_vec = addr_t( Integer( parameter[ arg[1] ] ) );
    CPPAD_ASSERT_KNOWN(
        size_t(i_vec) < vec_ad2index[ arg[0] - 1 ] ,
        "VecAD: dynamic parmaeter index out or range during zero order forward"
    );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0] + i_vec) < play->num_var_vecad_ind_rec() );

    size_t i_pv   = vec_ad2index[ arg[0] + i_vec ];
    Base* z       = taylor + i_z * cap_order;
    if( vec_ad2isvar[ arg[0] + i_vec ]  )
    {   CPPAD_ASSERT_UNKNOWN( i_pv < i_z );
        load_op2var[ arg[2] ] = addr_t( i_pv );
        Base* v_x = taylor + i_pv * cap_order;
        z[0]      = v_x[0];
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( i_pv < play->num_par_rec()  );
        load_op2var[ arg[2] ] = 0;
        Base v_x  = parameter[i_pv];
        z[0]      = v_x;
    }
}
template <class Addr, class Base>
void forward_load_v_op_0(
    const local::player<Base>* play ,
    size_t         i_z              ,
    const Addr*    arg              ,
    const Base*    parameter        ,
    size_t         cap_order        ,
    Base*          taylor           ,
    const bool*    vec_ad2isvar     ,
    const size_t*  vec_ad2index     ,
    Addr*          load_op2var   )
{   CPPAD_ASSERT_UNKNOWN( NumArg(LdvOp) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(LdvOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < i_z );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) < play->num_var_load_rec() );
    CPPAD_ASSERT_UNKNOWN(
        size_t( std::numeric_limits<addr_t>::max() ) >= i_z
    );

    addr_t i_vec = addr_t(Integer(taylor[ size_t(arg[1]) * cap_order + 0 ] ));
    CPPAD_ASSERT_KNOWN(
        size_t(i_vec) < vec_ad2index[ arg[0] - 1 ] ,
        "VecAD: variable index out or range during zero order forward"
    );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0] + i_vec) < play->num_var_vecad_ind_rec() );

    size_t i_pv   = vec_ad2index[ arg[0] + i_vec ];
    Base* z       = taylor + i_z * cap_order;
    if( vec_ad2isvar[ arg[0] + i_vec ]  )
    {   CPPAD_ASSERT_UNKNOWN( i_pv < i_z );
        load_op2var[ arg[2] ] = addr_t( i_pv );
        Base* v_x = taylor + i_pv * cap_order;
        z[0]      = v_x[0];
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( i_pv < play->num_par_rec() );
        load_op2var[ arg[2] ] = 0;
        Base v_x  = parameter[i_pv];
        z[0]      = v_x;
    }
}
/*!
------------------------------------------------------------------------------
Shared documentation for sparsity operations corresponding to
op = LdpOp or LdvOp (not called).

<!-- replace preamble -->
The C++ source code corresponding to this operation is
\verbatim
    v[x] = y
\endverbatim
where v is a VecAD<Base> vector, x is an AD<Base> object,
and y is AD<Base> or Base objects.
We define the index corresponding to v[x] by
\verbatim
    i_pv = vec_ad2index[ arg[0] + i_vec ]
\endverbatim
where i_vec is defined under the heading arg[1] below:
<!-- end preamble -->

\tparam Vector_set
is the type used for vectors of sets. It can be either
sparse::pack_setvec or sparse::list_setvec.

\param op
is the code corresponding to this operator;
i.e., LdpOp or LdvOp.

\param i_z
is the AD variable index corresponding to the variable z; i.e.,
the set with index i_z in var_sparsity is the sparsity pattern
corresponding to z.

\param arg
\n
 arg[0]
is the offset corresponding to this VecAD vector in the VecAD combined array.

\param num_combined
is the total number of elements in the VecAD combinded array.

\param combined
is the VecAD combined array.
\n
\n
 combined[ arg[0] - 1 ]
is the index of the set corresponding to the vector v  in vecad_sparsity.
We use the notation i_v for this value; i.e.,
\verbatim
    i_v = combined[ arg[0] - 1 ]
\endverbatim

\param var_sparsity
The set with index i_z in var_sparsity is the sparsity pattern for z.
This is an output for forward mode operations,
and an input for reverse mode operations.

\param vecad_sparsity
The set with index i_v is the sparsity pattern for the vector v.
This is an input for forward mode operations.
For reverse mode operations,
the sparsity pattern for z is added to the sparsity pattern for v.

\par Checked Assertions
\li NumArg(op) == 3
\li NumRes(op) == 1
\li 0         <  arg[0]
\li arg[0] < num_combined
\li i_v       < vecad_sparsity.n_set()
*/
template <class Vector_set, class Addr>
void sparse_load_op(
    OpCode              op             ,
    size_t              i_z            ,
    const Addr*          arg           ,
    size_t              num_combined   ,
    const size_t*       combined       ,
    Vector_set&         var_sparsity   ,
    Vector_set&         vecad_sparsity )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}



/*!
Forward mode, except for zero order, for op = LdpOp or op = LdvOp


<!-- replace preamble -->
The C++ source code corresponding to this operation is
\verbatim
    v[x] = y
\endverbatim
where v is a VecAD<Base> vector, x is an AD<Base> object,
and y is AD<Base> or Base objects.
We define the index corresponding to v[x] by
\verbatim
    i_pv = vec_ad2index[ arg[0] + i_vec ]
\endverbatim
where i_vec is defined under the heading arg[1] below:
<!-- end preamble -->

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD<Base> and computations by this routine are done using type Base.

\param play
is the tape that this operation appears in.
This is for error detection and not used when NDEBUG is defined.

\param op
is the code corresponding to this operator; i.e., LdpOp or LdvOp
(only used for error checking).

\param p
is the lowest order of the Taylor coefficient that we are computing.

\param q
is the highest order of the Taylor coefficient that we are computing.

\param r
is the number of directions for the Taylor coefficients that we
are computing.

\param cap_order
number of columns in the matrix containing the Taylor coefficients.

\par tpv
We use the notation
<code>tpv = (cap_order-1) * r + 1</code>
which is the number of Taylor coefficients per variable

\param i_z
is the AD variable index corresponding to the variable z.

\param arg
arg[2]
Is the index of this vecad load instruction in the load_op2var array.

\param load_op2var
is a vector with size play->num_var_load_rec().
It contains the variable index corresponding to each load instruction.
In the case where the index is zero,
the instruction corresponds to a parameter (not variable).

\par i_var
We use the notation
\verbatim
    i_var = size_t( load_op2var[ arg[2] ] )
\endverbatim

\param taylor
\n
Input
\n
If <code>i_var > 0</code>, v[x] is a variable and
for k = 1 , ... , q
<code>taylor[ i_var * tpv + (k-1)*r+1+ell ]</code>
is the k-th order coefficient for v[x] in the ell-th direction,
\n
\n
Output
\n
for k = p , ... , q,
<code>taylor[ i_z * tpv + (k-1)*r+1+ell ]</code>
is set to the k-order Taylor coefficient for z in the ell-th direction.
*/
template <class Addr, class Base>
void forward_load_op(
    const local::player<Base>* play,
    OpCode               op                   ,
    size_t               p                    ,
    size_t               q                    ,
    size_t               r                    ,
    size_t               cap_order            ,
    size_t               i_z                  ,
    const Addr*          arg                  ,
    const Addr*          load_op2var       ,
          Base*          taylor               )
{
    CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(op) == 1 );
    CPPAD_ASSERT_UNKNOWN( q < cap_order );
    CPPAD_ASSERT_UNKNOWN( 0 < r);
    CPPAD_ASSERT_UNKNOWN( 0 < p);
    CPPAD_ASSERT_UNKNOWN( p <= q );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) < play->num_var_load_rec() );

    size_t i_var = size_t( load_op2var[ arg[2] ] );
    CPPAD_ASSERT_UNKNOWN( i_var < i_z );

    size_t num_taylor_per_var = (cap_order-1) * r + 1;
    Base* z  = taylor + i_z * num_taylor_per_var;
    if( i_var > 0 )
    {   Base* v_x = taylor + i_var * num_taylor_per_var;
        for(size_t ell = 0; ell < r; ell++)
        {   for(size_t k = p; k <= q; k++)
            {   size_t m = (k-1) * r + 1 + ell;
                z[m]     = v_x[m];
            }
        }
    }
    else
    {   for(size_t ell = 0; ell < r; ell++)
        {   for(size_t k = p; k <= q; k++)
            {   size_t m = (k-1) * r + 1 + ell;
                z[m]     = Base(0.0);
            }
        }
    }
}

/*!
Reverse mode for op = LdpOp or LdvOp.

<!-- replace preamble -->
The C++ source code corresponding to this operation is
\verbatim
    v[x] = y
\endverbatim
where v is a VecAD<Base> vector, x is an AD<Base> object,
and y is AD<Base> or Base objects.
We define the index corresponding to v[x] by
\verbatim
    i_pv = vec_ad2index[ arg[0] + i_vec ]
\endverbatim
where i_vec is defined under the heading arg[1] below:
<!-- end preamble -->

This routine is given the partial derivatives of a function
G(z , y[x] , w , u ... )
and it uses them to compute the partial derivatives of
\verbatim
    H( y[x] , w , u , ... ) = G[ z( y[x] ) , y[x] , w , u , ... ]
\endverbatim

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param op
is the code corresponding to this operator; i.e., LdpOp or LdvOp
(only used for error checking).

\param d
highest order the Taylor coefficient that we are computing the partial
derivative with respect to.

\param i_z
is the AD variable index corresponding to the variable z.

\param arg
 arg[2]
Is the index of this vecad load instruction in the
load_op2var array.

\param cap_order
number of columns in the matrix containing the Taylor coefficients
(not used).

\param taylor
matrix of Taylor coefficients (not used).

\param nc_partial
number of colums in the matrix containing all the partial derivatives
(not used if arg[2] is zero).

\param partial
If arg[2] is zero, y[x] is a parameter
and no values need to be modified; i.e., partial is not used.
Otherwise, y[x] is a variable and:
\n
\n
 partial [ i_z * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative of G
with respect to the k-th order Taylor coefficient for z.
\n
\n
If arg[2] is not zero,
 partial [ arg[2] * nc_partial + k ]
for k = 0 , ... , d
is the partial derivative with respect to
the k-th order Taylor coefficient for x.
On input, it corresponds to the function G,
and on output it corresponds to the the function H.

\param load_op2var
is a vector with size play->num_var_load_rec().
It contains the variable index corresponding to each load instruction.
In the case where the index is zero,
the instruction corresponds to a parameter (not variable).

\par Checked Assertions
\li NumArg(op) == 3
\li NumRes(op) == 1
\li d < cap_order
\li size_t(arg[2]) < i_z
*/
template <class Addr, class Base>
void reverse_load_op(
    OpCode         op          ,
    size_t         d           ,
    size_t         i_z         ,
    const Addr*    arg         ,
    size_t         cap_order   ,
    const Base*    taylor      ,
    size_t         nc_partial  ,
    Base*          partial     ,
    const Addr*          load_op2var )
{   size_t i_load = size_t( load_op2var[ arg[2] ] );

    CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(op) == 1 );
    CPPAD_ASSERT_UNKNOWN( d < cap_order );
    CPPAD_ASSERT_UNKNOWN( i_load < i_z );

    if( i_load > 0 )
    {
        Base* pz   = partial + i_z    * nc_partial;
        Base* py_x = partial + i_load * nc_partial;
        size_t j = d + 1;
        while(j--)
            py_x[j]   += pz[j];
    }
}


/*!
Forward mode sparsity operations for LdpOp and LdvOp

\param dependency
is this a dependency (or sparsity) calculation.

\copydetails CppAD::local::sparse_load_op
*/
template <class Vector_set, class Addr>
void forward_sparse_load_op(
    bool               dependency     ,
    OpCode             op             ,
    size_t             i_z            ,
    const Addr*        arg            ,
    size_t             num_combined   ,
    const size_t*      combined       ,
    Vector_set&        var_sparsity   ,
    Vector_set&        vecad_sparsity )
{
    CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(op) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_combined );
    size_t i_v = combined[ arg[0] - 1 ];
    CPPAD_ASSERT_UNKNOWN( i_v < vecad_sparsity.n_set() );

    var_sparsity.assignment(i_z, i_v, vecad_sparsity);
    if( dependency & (op == LdvOp) )
        var_sparsity.binary_union(i_z, i_z, size_t(arg[1]), var_sparsity);

    return;
}


/*!
Reverse mode Jacobian sparsity operations for LdpOp and LdvOp

\param dependency
is this a dependency (or sparsity) calculation.

\copydetails CppAD::local::sparse_load_op
*/
template <class Vector_set, class Addr>
void reverse_sparse_jacobian_load_op(
    bool               dependency     ,
    OpCode             op             ,
    size_t             i_z            ,
    const Addr*        arg            ,
    size_t             num_combined   ,
    const size_t*      combined       ,
    Vector_set&        var_sparsity   ,
    Vector_set&        vecad_sparsity )
{
    CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(op) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_combined );
    size_t i_v = combined[ arg[0] - 1 ];
    CPPAD_ASSERT_UNKNOWN( i_v < vecad_sparsity.n_set() );

    vecad_sparsity.binary_union(i_v, i_v, i_z, var_sparsity);
    if( dependency & (op == LdvOp) )
        var_sparsity.binary_union( size_t(arg[1]), size_t(arg[1]), i_z, var_sparsity);

    return;
}


/*!
Reverse mode Hessian sparsity operations for LdpOp and LdvOp

\copydetails CppAD::local::sparse_load_op

\param var_jacobian
 var_jacobian[i_z]
is false (true) if the Jacobian of G with respect to z is always zero
(many be non-zero).

\param vecad_jacobian
 vecad_jacobian[i_v]
is false (true) if the Jacobian with respect to x is always zero
(may be non-zero).
On input, it corresponds to the function G,
and on output it corresponds to the function H.

*/
template <class Vector_set, class Addr>
void reverse_sparse_hessian_load_op(
    OpCode             op             ,
    size_t             i_z            ,
    const Addr*        arg            ,
    size_t             num_combined   ,
    const size_t*      combined       ,
    Vector_set&        var_sparsity   ,
    Vector_set&        vecad_sparsity ,
    bool*              var_jacobian   ,
    bool*              vecad_jacobian )
{
    CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(op) == 1 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_combined );
    size_t i_v = combined[ arg[0] - 1 ];
    CPPAD_ASSERT_UNKNOWN( i_v < vecad_sparsity.n_set() );

    vecad_sparsity.binary_union(i_v, i_v, i_z, var_sparsity);

    vecad_jacobian[i_v] |= var_jacobian[i_z];

    return;
}


} } // END_CPPAD_LOCAL_NAMESPACE
# endif
