# ifndef CPPAD_LOCAL_SWEEP_REVERSE_HPP
# define CPPAD_LOCAL_SWEEP_REVERSE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */


# include <cppad/local/play/atom_op_info.hpp>

// BEGIN_CPPAD_LOCAL_SWEEP_NAMESPACE
namespace CppAD { namespace local { namespace sweep {
/*!
\file sweep/reverse.hpp
Compute derivatives of arbitrary order Taylor coefficients.
*/

/*!
\def CPPAD_REVERSE_TRACE
This value is either zero or one.
Zero is the normal operational value.
If it is one, a trace of every reverse_sweep computation is printed.
*/
# define CPPAD_REVERSE_TRACE 0

/*!
Compute derivative of arbitrary order forward mode Taylor coefficients.

\tparam Base
this operation sequence was recorded using AD<Base>
and computations by this routine are done using type Base.

\param d
is the highest order Taylor coefficients that
we are computing the derivative of.

\param n
is the number of independent variables on the tape.

\param numvar
is the total number of variables on the tape.
This is also equal to the number of rows in the matrix Taylor; i.e.,
play->num_var_rec().

\param play
The information stored in play
is a recording of the operations corresponding to the function
\f[
    F : {\bf R}^n \rightarrow {\bf R}^m
\f]
where \f$ n \f$ is the number of independent variables and
\f$ m \f$ is the number of dependent variables.
We define \f$ u^{(k)} \f$ as the value of <code>x_k</code> in the previous call
of the form
<code>
    f.Forward(k, x_k)
</code>
We define
\f$ X : {\bf R}^{n \times d} \rightarrow {\bf R}^n \f$ by
\f[
    X(t, u) =  u^{(0)} + u^{(1)} t + \cdots + u^{(d)} t^d
\f]
We define
\f$ Y : {\bf R}^{n \times d} \rightarrow {\bf R}^m \f$ by
\f[
    Y(t, u) =  F[ X(t, u) ]
\f]
We define the function
\f$ W : {\bf R}^{n \times d} \rightarrow {\bf R} \f$ by
\f[
W(u)
=
\sum_{k=0}^{d} ( w^{(k)} )^{\rm T}
    \frac{1}{k !} \frac{\partial^k}{\partial t^k} Y(0, u)
\f]
(The matrix \f$ w \in {\bf R}^m \f$,
is defined below under the heading Partial.)
Note that the scale factor  1 / k  converts
the k-th partial derivative to the k-th order Taylor coefficient.
This routine computes the derivative of \f$ W(u) \f$
with respect to all the Taylor coefficients
\f$ u^{(k)} \f$ for \f$ k = 0 , ... , d \f$.

\param J
Is the number of columns in the coefficient matrix Taylor.
This must be greater than or equal d + 1.

\param Taylor
For i = 1 , ... , numvar, and for k = 0 , ... , d,
 Taylor [ i * J + k ]
is the k-th order Taylor coefficient corresponding to
variable with index i on the tape.
The value \f$ u \in {\bf R}^{n \times d} \f$,
at which the derivative is computed,
is defined by
\f$ u_j^{(k)} \f$ = Taylor [ j * J + k ]
for j = 1 , ... , n, and for k = 0 , ... , d.

\param K
Is the number of columns in the partial derivative matrix Partial.
It must be greater than or equal d + 1.

\param Partial
\b Input:
The last \f$ m \f$ rows of Partial are inputs.
The matrix \f$ w \f$, used to define \f$ W(u) \f$,
is specified by these rows.
For i = 0 , ... , m - 1,
for k = 0 , ... , d,
<code>Partial [ (numvar - m + i ) * K + k ] = w[i,k]</code>.
\n
\n
\b Temporary:
For i = n+1 , ... , numvar - 1 and for k = 0 , ... , d,
the value of Partial [ i * K + k ] is used for temporary work space
and its output value is not defined.
\n
\n
\b Output:
For j = 1 , ... , n and for k = 0 , ... , d,
 Partial [ j * K + k ]
is the partial derivative of \f$ W( u ) \f$ with
respect to \f$ u_j^{(k)} \f$.

\param cskip_op
Is a vector with size play->num_op_rec().
If cskip_op[i] is true, the operator index i in the recording
does not affect any of the dependent variable (given the value
of the independent variables).
Note that all the operators in an atomic function call are skipped as a block,
so only the last AFunOp fore each call needs to have cskip_op[i] true.

\param load_op2var
is a vector with size play->num_var_load_rec().
It contains the variable index corresponding to each load instruction.
In the case where the index is zero,
the instruction corresponds to a parameter (not variable).

\tparam Iterator
This is either player::const_iteratoror player::const_subgraph_iterator.

\param play_itr
On input this is either play->end(), for the entire graph,
or play->end(subgraph), for a subgraph.
This routine mode will use --play_itr to iterate over the graph or subgraph.
It is assumes that the iterator starts just past the EndOp and it will
continue until it reaches the BeginOp.
If i_var is a variable index, and the corresponding operator
is not in the subgraph,
then the partials with respect to i_var are not modified and need to be
initialized as zero. Note that this means the partial for the independent
varaibles, that are not in the subgraph are not calculated.
If part of an atomic function call is in the subgraph,
the entire atomic function call must be in the subgraph.

\param not_used_rec_base
Specifies RecBase for this call.

\par Assumptions
The first operator on the tape is a BeginOp,
and the next n operators are InvOp operations for the
corresponding independent variables; see play->check_inv_op(n_ind).
*/
template <class Addr, class Base, class Iterator, class RecBase>
void reverse(
    size_t                      d,
    size_t                      n,
    size_t                      numvar,
    const local::player<Base>*  play,
    size_t                      J,
    const Base*                 Taylor,
    size_t                      K,
    Base*                       Partial,
    bool*                       cskip_op,
    const pod_vector<Addr>&     load_op2var,
    Iterator&                   play_itr,
    const RecBase&              not_used_rec_base
)
{
    // check numvar argument
    CPPAD_ASSERT_UNKNOWN( play->num_var_rec() == numvar );
    CPPAD_ASSERT_UNKNOWN( numvar > 0 );

    // length of the parameter vector (used by CppAD assert macros)
    const size_t num_par = play->num_par_rec();

    // pointer to the beginning of the parameter vector
    CPPAD_ASSERT_UNKNOWN( num_par > 0 )
    const Base* parameter = play->GetPar();

    // work space used by AFunOp.
    const size_t         atom_k  = d;   // highest order we are differentiating
    const size_t         atom_k1 = d+1; // number orders for this calculation
    vector<Base>         atom_par_x;    // argument parameter values
    vector<ad_type_enum> atom_type_x;   // argument type
    vector<size_t>       atom_ix;       // variable indices for argument vector
    vector<Base>         atom_tx;       // argument vector Taylor coefficients
    vector<Base>         atom_ty;       // result vector Taylor coefficients
    vector<Base>         atom_px;       // partials w.r.t argument vector
    vector<Base>         atom_py;       // partials w.r.t. result vector
    //
    // information defined by atomic forward
    size_t atom_index=0, atom_old=0, atom_m=0, atom_n=0, atom_i=0, atom_j=0;
    enum_atom_state atom_state = end_atom; // proper initialization

    // A vector with unspecified contents declared here so that operator
    // routines do not need to re-allocate it
    vector<Base> work;

    // temporary indices
    size_t j, ell;

    // Initialize
# if CPPAD_REVERSE_TRACE
    std::cout << std::endl;
# endif
    OpCode        op;
    const Addr*   arg;
    size_t        i_var;
    play_itr.op_info(op, arg, i_var);
    CPPAD_ASSERT_UNKNOWN( op == EndOp );
    while(op != BeginOp )
    {   bool flag; // temporary for use in switch cases
        //
        // next op
        (--play_itr).op_info(op, arg, i_var);

        // check if we are skipping this operation
        size_t i_op = play_itr.op_index();
        while( cskip_op[i_op] )
        {   switch(op)
            {
                case AFunOp:
                {   // get information for this atomic function call
                    CPPAD_ASSERT_UNKNOWN( atom_state == end_atom );
                    play::atom_op_info<Base>(
                        op, arg, atom_index, atom_old, atom_m, atom_n
                    );
                    //
                    // skip to the first AFunOp
                    for(size_t i = 0; i < atom_m + atom_n + 1; ++i)
                        --play_itr;
                    play_itr.op_info(op, arg, i_var);
                    CPPAD_ASSERT_UNKNOWN( op == AFunOp );
                }
                break;

                default:
                break;
            }
            (--play_itr).op_info(op, arg, i_var);
            i_op = play_itr.op_index();
        }
# if CPPAD_REVERSE_TRACE
        size_t       i_tmp  = i_var;
        const Base*  Z_tmp  = Taylor + i_var * J;
        const Base*  pZ_tmp = Partial + i_var * K;
        printOp<Base, RecBase>(
            std::cout,
            play,
            i_op,
            i_tmp,
            op,
            arg
        );
        if( NumRes(op) > 0 && op != BeginOp ) printOpResult(
            std::cout,
            d + 1,
            Z_tmp,
            d + 1,
            pZ_tmp
        );
        std::cout << std::endl;
# endif
        switch( op )
        {
            case AbsOp:
            reverse_abs_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case AcosOp:
            // sqrt(1 - x * x), acos(x)
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_acos_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case AcoshOp:
            // sqrt(x * x - 1), acosh(x)
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_acosh_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case AddvvOp:
            reverse_addvv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case AddpvOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
            reverse_addpv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case AsinOp:
            // sqrt(1 - x * x), asin(x)
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_asin_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case AsinhOp:
            // sqrt(1 + x * x), asinh(x)
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_asinh_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case AtanOp:
            // 1 + x * x, atan(x)
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_atan_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // -------------------------------------------------

            case AtanhOp:
            // 1 - x * x, atanh(x)
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_atanh_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // -------------------------------------------------

            case BeginOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            CPPAD_ASSERT_UNKNOWN( i_op == 0 );
            break;
            // --------------------------------------------------

            case CSkipOp:
            // CSkipOp has a zero order forward action.
            play_itr.correct_after_decrement(arg);
            break;
            // -------------------------------------------------

            case CSumOp:
            play_itr.correct_after_decrement(arg);
            reverse_csum_op(
                d, i_var, arg, K, Partial
            );
            // end of a cumulative summation
            break;
            // -------------------------------------------------

            case CExpOp:
            reverse_cond_op(
                d,
                i_var,
                arg,
                num_par,
                parameter,
                J,
                Taylor,
                K,
                Partial
            );
            break;
            // --------------------------------------------------

            case CosOp:
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_cos_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case CoshOp:
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_cosh_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case DisOp:
            // Derivative of discrete operation is zero so no
            // contribution passes through this operation.
            break;
            // --------------------------------------------------

            case DivvvOp:
            reverse_divvv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case DivpvOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
            reverse_divpv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case DivvpOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < num_par );
            reverse_divvp_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------
            case EndOp:
            CPPAD_ASSERT_UNKNOWN(
                i_op == play->num_op_rec() - 1
            );
            break;

            // --------------------------------------------------

            case ErfOp:
            case ErfcOp:
            reverse_erf_op(
                op, d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case ExpOp:
            reverse_exp_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case Expm1Op:
            reverse_expm1_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case InvOp:
            break;
            // --------------------------------------------------

            case LdpOp:
            reverse_load_op(
            op, d, i_var, arg, J, Taylor, K, Partial, load_op2var.data()
            );
            break;
            // -------------------------------------------------

            case LdvOp:
            reverse_load_op(
            op, d, i_var, arg, J, Taylor, K, Partial, load_op2var.data()
            );
            break;
            // --------------------------------------------------

            case EqppOp:
            case EqpvOp:
            case EqvvOp:
            case LtppOp:
            case LtpvOp:
            case LtvpOp:
            case LtvvOp:
            case LeppOp:
            case LepvOp:
            case LevpOp:
            case LevvOp:
            case NeppOp:
            case NepvOp:
            case NevvOp:
            break;
            // -------------------------------------------------

            case LogOp:
            reverse_log_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case Log1pOp:
            reverse_log1p_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case MulpvOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
            reverse_mulpv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case MulvvOp:
            reverse_mulvv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // -------------------------------------------------

            case NegOp:
            reverse_neg_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case ParOp:
            break;
            // --------------------------------------------------

            case PowvpOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < num_par );
            reverse_powvp_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial, work
            );
            break;
            // -------------------------------------------------

            case PowpvOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
            reverse_powpv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // -------------------------------------------------

            case PowvvOp:
            reverse_powvv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case PriOp:
            // no result so nothing to do
            break;
            // --------------------------------------------------

            case SignOp:
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_sign_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // -------------------------------------------------

            case SinOp:
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_sin_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // -------------------------------------------------

            case SinhOp:
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_sinh_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case SqrtOp:
            reverse_sqrt_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case StppOp:
            break;
            // --------------------------------------------------

            case StpvOp:
            break;
            // -------------------------------------------------

            case StvpOp:
            break;
            // -------------------------------------------------

            case StvvOp:
            break;
            // --------------------------------------------------

            case SubvvOp:
            reverse_subvv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case SubpvOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
            reverse_subpv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case SubvpOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < num_par );
            reverse_subvp_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // -------------------------------------------------

            case TanOp:
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_tan_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // -------------------------------------------------

            case TanhOp:
            CPPAD_ASSERT_UNKNOWN( i_var < numvar );
            reverse_tanh_op(
                d, i_var, size_t(arg[0]), J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case AFunOp:
            // start or end an atomic function call
            flag = atom_state == end_atom;
            play::atom_op_info<RecBase>(
                op, arg, atom_index, atom_old, atom_m, atom_n
            );
            if( flag )
            {   atom_state = ret_atom;
                atom_i     = atom_m;
                atom_j     = atom_n;
                //
                atom_ix.resize(atom_n);
                atom_par_x.resize(atom_n);
                atom_type_x.resize(atom_n);
                atom_tx.resize(atom_n * atom_k1);
                atom_px.resize(atom_n * atom_k1);
                atom_ty.resize(atom_m * atom_k1);
                atom_py.resize(atom_m * atom_k1);
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( atom_i == 0 );
                CPPAD_ASSERT_UNKNOWN( atom_j == 0  );
                atom_state = end_atom;
                //
                // call atomic function for this operation
                call_atomic_reverse<Base, RecBase>(
                    atom_par_x,
                    atom_type_x,
                    atom_k,
                    atom_index,
                    atom_old,
                    atom_tx,
                    atom_ty,
                    atom_px,
                    atom_py
                );
                for(j = 0; j < atom_n; j++) if( atom_ix[j] > 0 )
                {   for(ell = 0; ell < atom_k1; ell++)
                        Partial[atom_ix[j] * K + ell] +=
                            atom_px[j * atom_k1 + ell];
                }
            }
            break;

            case FunapOp:
            // parameter argument in an atomic operation sequence
            CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
            CPPAD_ASSERT_UNKNOWN( atom_state == arg_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i == 0 );
            CPPAD_ASSERT_UNKNOWN( atom_j <= atom_n );
            CPPAD_ASSERT_UNKNOWN( size_t( arg[0] ) < num_par );
            //
            --atom_j;
            atom_ix[atom_j]               = 0;
            if( play->dyn_par_is()[ arg[0] ] )
                atom_type_x[atom_j]       = dynamic_enum;
            else
                atom_type_x[atom_j]       = constant_enum;
            atom_par_x[atom_j]            = parameter[ arg[0] ];
            atom_tx[atom_j * atom_k1 + 0] = parameter[ arg[0] ];
            for(ell = 1; ell < atom_k1; ell++)
                atom_tx[atom_j * atom_k1 + ell] = Base(0.);
            //
            if( atom_j == 0 )
                atom_state = start_atom;
            break;

            case FunavOp:
            // variable argument in an atomic operation sequence
            CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
            CPPAD_ASSERT_UNKNOWN( atom_state == arg_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i == 0 );
            CPPAD_ASSERT_UNKNOWN( atom_j <= atom_n );
            //
            --atom_j;
            atom_ix[atom_j]     = size_t( arg[0] );
            atom_type_x[atom_j] = variable_enum;
            atom_par_x[atom_j] = CppAD::numeric_limits<Base>::quiet_NaN();
            for(ell = 0; ell < atom_k1; ell++)
                atom_tx[atom_j*atom_k1 + ell] =
                    Taylor[ size_t(arg[0]) * J + ell];
            //
            if( atom_j == 0 )
                atom_state = start_atom;
            break;

            case FunrpOp:
            // parameter result for a atomic function
            CPPAD_ASSERT_NARG_NRES(op, 1, 0);
            CPPAD_ASSERT_UNKNOWN( atom_state == ret_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i <= atom_m );
            CPPAD_ASSERT_UNKNOWN( atom_j == atom_n );
            CPPAD_ASSERT_UNKNOWN( size_t( arg[0] ) < num_par );
            //
            --atom_i;
            for(ell = 0; ell < atom_k1; ell++)
            {   atom_py[atom_i * atom_k1 + ell] = Base(0.);
                atom_ty[atom_i * atom_k1 + ell] = Base(0.);
            }
            atom_ty[atom_i * atom_k1 + 0] = parameter[ arg[0] ];
            //
            if( atom_i == 0 )
                atom_state = arg_atom;
            break;

            case FunrvOp:
            // variable result for a atomic function
            CPPAD_ASSERT_NARG_NRES(op, 0, 1);
            CPPAD_ASSERT_UNKNOWN( atom_state == ret_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i <= atom_m );
            CPPAD_ASSERT_UNKNOWN( atom_j == atom_n );
            //
            --atom_i;
            for(ell = 0; ell < atom_k1; ell++)
            {   atom_py[atom_i * atom_k1 + ell] =
                        Partial[i_var * K + ell];
                atom_ty[atom_i * atom_k1 + ell] =
                        Taylor[i_var * J + ell];
            }
            if( atom_i == 0 )
                atom_state = arg_atom;
            break;
            // ------------------------------------------------------------

            case ZmulpvOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
            reverse_zmulpv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case ZmulvpOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < num_par );
            reverse_zmulvp_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            case ZmulvvOp:
            reverse_zmulvv_op(
                d, i_var, arg, parameter, J, Taylor, K, Partial
            );
            break;
            // --------------------------------------------------

            default:
            CPPAD_ASSERT_UNKNOWN(false);
        }
    }
# if CPPAD_REVERSE_TRACE
    std::cout << std::endl;
# endif
}

} } } // END_CPPAD_LOCAL_SWEEP_NAMESPACE

// preprocessor symbols that are local to this file
# undef CPPAD_REVERSE_TRACE

# endif
