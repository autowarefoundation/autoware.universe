# ifndef CPPAD_LOCAL_SWEEP_REV_HES_HPP
# define CPPAD_LOCAL_SWEEP_REV_HES_HPP
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
# include <cppad/local/sweep/call_atomic.hpp>

// BEGIN_CPPAD_LOCAL_SWEEP_NAMESPACE
namespace CppAD { namespace local { namespace sweep {
/*!
\file sweep/rev_hes.hpp
Compute Reverse mode Hessian sparsity patterns.
*/

/*!
\def CPPAD_REV_HES_TRACE
This value is either zero or one.
Zero is the normal operational value.
If it is one, a trace of every rev_hes_sweep computation is printed.
*/
# define CPPAD_REV_HES_TRACE 0

/*!
Given the forward Jacobian sparsity pattern for all the variables,
and the reverse Jacobian sparsity pattern for the dependent variables,
RevHesSweep computes the Hessian sparsity pattern for all the independent
variables.

\tparam Base
this operation sequence was recorded using AD<Base>.

\tparam Vector_set
is the type used for vectors of sets. It can be either
sparse::pack_setvec or sparse::list_setvec.

\param n
is the number of independent variables on the tape.

\param numvar
is the total number of variables on the tape; i.e.,
 play->num_var_rec().
This is also the number of rows in the entire sparsity pattern
 rev_hes_sparse.

\param play
The information stored in play
is a recording of the operations corresponding to a function
\f[
    F : {\bf R}^n \rightarrow {\bf R}^m
\f]
where \f$ n \f$ is the number of independent variables
and \f$ m \f$ is the number of dependent variables.

\param for_jac_sparse
For i = 0 , ... , numvar - 1,
(for all the variables on the tape),
the forward Jacobian sparsity pattern for the variable with index i
corresponds to the set with index i in for_jac_sparse.

\param RevJac
\b Input:
For i = 0, ... , numvar - 1
the if the variable with index i on the tape is an dependent variable and
included in the Hessian, RevJac[ i ] is equal to true,
otherwise it is equal to false.
\n
\n
\b Output: The values in RevJac upon return are not specified; i.e.,
it is used for temporary work space.

\param rev_hes_sparse
The reverse Hessian sparsity pattern for the variable with index i
corresponds to the set with index i in rev_hes_sparse.
\n
\n
\b Input: For i = 0 , ... , numvar - 1
the reverse Hessian sparsity pattern for the variable with index i is empty.
\n
\n
\b Output: For j = 1 , ... , n,
the reverse Hessian sparsity pattern for the independent dependent variable
with index (j-1) is given by the set with index j
in rev_hes_sparse.
The values in the rest of rev_hes_sparse are not specified; i.e.,
they are used for temporary work space.

\param not_used_rec_base
Specifies RecBase for this call.
*/

template <class Addr, class Base, class Vector_set, class RecBase>
void rev_hes(
    const local::player<Base>* play,
    size_t                     n,
    size_t                     numvar,
    const Vector_set&          for_jac_sparse,
    bool*                      RevJac,
    Vector_set&                rev_hes_sparse,
    const RecBase&             not_used_rec_base
)
{
    // length of the parameter vector (used by CppAD assert macros)
    const size_t num_par = play->num_par_rec();

    size_t             i, j, k;

    // check numvar argument
    CPPAD_ASSERT_UNKNOWN( play->num_var_rec()    == numvar );
    CPPAD_ASSERT_UNKNOWN( for_jac_sparse.n_set() == numvar );
    CPPAD_ASSERT_UNKNOWN( rev_hes_sparse.n_set() == numvar );
    CPPAD_ASSERT_UNKNOWN( numvar > 0 );

    // upper limit exclusive for set elements
    size_t limit   = rev_hes_sparse.end();
    CPPAD_ASSERT_UNKNOWN( for_jac_sparse.end() == limit );

    // check number of sets match
    CPPAD_ASSERT_UNKNOWN(
        for_jac_sparse.n_set() == rev_hes_sparse.n_set()
    );

    // vecad_sparsity contains a sparsity pattern for each VecAD object.
    // vecad_ind maps a VecAD index (beginning of the VecAD object)
    // to the index for the corresponding set in vecad_sparsity.
    size_t num_vecad_ind   = play->num_var_vecad_ind_rec();
    size_t num_vecad_vec   = play->num_var_vecad_rec();
    Vector_set vecad_sparse;
    pod_vector<size_t> vecad_ind;
    pod_vector<bool>   vecad_jac;
    if( num_vecad_vec > 0 )
    {   size_t length;
        vecad_sparse.resize(num_vecad_vec, limit);
        vecad_ind.extend(num_vecad_ind);
        vecad_jac.extend(num_vecad_vec);
        j             = 0;
        for(i = 0; i < num_vecad_vec; i++)
        {   // length of this VecAD
            length   = play->GetVecInd(j);
            // set vecad_ind to proper index for this VecAD
            vecad_ind[j] = i;
            // make all other values for this vector invalid
            for(k = 1; k <= length; k++)
                vecad_ind[j+k] = num_vecad_vec;
            // start of next VecAD
            j       += length + 1;
            // initialize this vector's reverse jacobian value
            vecad_jac[i] = false;
        }
        CPPAD_ASSERT_UNKNOWN( j == play->num_var_vecad_ind_rec() );
    }

    // ----------------------------------------------------------------------
    // work space used by AFunOp.
    vector<Base>         atom_x;  // value of parameter arguments to function
    vector<ad_type_enum> type_x;  // argument types
    pod_vector<size_t>   atom_ix; // variable indices for argument vector
    pod_vector<size_t>   atom_iy; // variable indices for result vector
    //
    // information set by atomic forward (initialization to avoid warnings)
    size_t atom_index=0, atom_old=0, atom_m=0, atom_n=0, atom_i=0, atom_j=0;
    // information set by atomic forward (necessary initialization)
    enum_atom_state atom_state = end_atom; // proper initialization
    // ----------------------------------------------------------------------
    //
    // pointer to the beginning of the parameter vector
    // (used by atomic functions
    CPPAD_ASSERT_UNKNOWN( num_par > 0 )
    const Base* parameter = play->GetPar();
    //
    // which parametes are dynamic
    const pod_vector<bool>& dyn_par_is( play->dyn_par_is() );
    //
    // skip the EndOp at the end of the recording
    play::const_sequential_iterator itr = play->end();
    // op_info
    OpCode op;
    size_t i_var;
    const Addr*   arg;
    itr.op_info(op, arg, i_var);
    CPPAD_ASSERT_UNKNOWN( op == EndOp );
# if CPPAD_REV_HES_TRACE
    std::cout << std::endl;
    CppAD::vectorBool zf_value(limit);
    CppAD::vectorBool zh_value(limit);
# endif
    bool more_operators = true;
    while(more_operators)
    {   bool flag; // temporary for use in switch cases
        //
        // next op
        (--itr).op_info(op, arg, i_var);

        // rest of information depends on the case
        switch( op )
        {
            case AbsOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1)
            sparse::rev_hes_lin_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case AddvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_addsub_op(
            i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case AddpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_lin_unary_op(
            i_var, size_t(arg[1]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case AcosOp:
            // sqrt(1 - x * x), acos(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case AcoshOp:
            // sqrt(x * x - 1), acosh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case AsinOp:
            // sqrt(1 - x * x), asin(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case AsinhOp:
            // sqrt(1 + x * x), asinh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case AtanOp:
            // 1 + x * x, atan(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case AtanhOp:
            // 1 - x * x, atanh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case BeginOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1)
            more_operators = false;
            break;
            // -------------------------------------------------

            case CSkipOp:
            itr.correct_after_decrement(arg);
            break;
            // -------------------------------------------------

            case CSumOp:
            itr.correct_after_decrement(arg);
            reverse_sparse_hessian_csum_op(
                i_var, arg, RevJac, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case CExpOp:
            reverse_sparse_hessian_cond_op(
                i_var, arg, num_par, RevJac, rev_hes_sparse
            );
            break;
            // ---------------------------------------------------

            case CosOp:
            // sin(x), cos(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // ---------------------------------------------------

            case CoshOp:
            // sinh(x), cosh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case DisOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            // derivativve is identically zero
            break;
            // -------------------------------------------------

            case DivvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_div_op(
            i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case DivpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[1]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case DivvpOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_lin_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case ErfOp:
            case ErfcOp:
            // arg[1] is always the parameter 0
            // arg[2] is always the parameter 2 / sqrt(pi)
            CPPAD_ASSERT_NARG_NRES(op, 3, 5);
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case ExpOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case Expm1Op:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case InvOp:
            CPPAD_ASSERT_NARG_NRES(op, 0, 1)
            // Z is already defined
            break;
            // -------------------------------------------------

            case LdpOp:
            reverse_sparse_hessian_load_op(
                op,
                i_var,
                arg,
                num_vecad_ind,
                vecad_ind.data(),
                rev_hes_sparse,
                vecad_sparse,
                RevJac,
                vecad_jac.data()
            );
            break;
            // -------------------------------------------------

            case LdvOp:
            reverse_sparse_hessian_load_op(
                op,
                i_var,
                arg,
                num_vecad_ind,
                vecad_ind.data(),
                rev_hes_sparse,
                vecad_sparse,
                RevJac,
                vecad_jac.data()
            );
            break;
            // -------------------------------------------------

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
            CPPAD_ASSERT_NARG_NRES(op, 2, 0);
            break;
            // -------------------------------------------------

            case LogOp:
            case NegOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case Log1pOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case MulpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_lin_unary_op(
            i_var, size_t(arg[1]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case MulvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_mul_op(
            i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case ParOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1)

            break;
            // -------------------------------------------------

            case PowpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 3)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[1]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case PowvpOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case PowvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 3)
            sparse::rev_hes_pow_op(
            i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case PriOp:
            CPPAD_ASSERT_NARG_NRES(op, 5, 0);
            break;
            // -------------------------------------------------

            case SignOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            // Derivative is identiaclly zero
            break;
            // -------------------------------------------------

            case SinOp:
            // cos(x), sin(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case SinhOp:
            // cosh(x), sinh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case SqrtOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case StppOp:
            // sparsity cannot propagate through a parameter
            CPPAD_ASSERT_NARG_NRES(op, 3, 0)
            break;
            // -------------------------------------------------

            case StpvOp:
            reverse_sparse_hessian_store_op(
                op,
                arg,
                num_vecad_ind,
                vecad_ind.data(),
                rev_hes_sparse,
                vecad_sparse,
                RevJac,
                vecad_jac.data()
            );
            break;
            // -------------------------------------------------

            case StvpOp:
            // sparsity cannot propagate through a parameter
            CPPAD_ASSERT_NARG_NRES(op, 3, 0)
            break;
            // -------------------------------------------------

            case StvvOp:
            reverse_sparse_hessian_store_op(
                op,
                arg,
                num_vecad_ind,
                vecad_ind.data(),
                rev_hes_sparse,
                vecad_sparse,
                RevJac,
                vecad_jac.data()
            );
            break;
            // -------------------------------------------------

            case SubvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_addsub_op(
            i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case SubpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_lin_unary_op(
            i_var, size_t(arg[1]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case SubvpOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_lin_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case TanOp:
            // tan(x)^2, tan(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case TanhOp:
            // tanh(x)^2, tanh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2)
            sparse::rev_hes_nl_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case AFunOp:
            // start or end an atomic function call
            CPPAD_ASSERT_UNKNOWN(
                atom_state == start_atom || atom_state == end_atom
            );
            flag = atom_state == end_atom;
            play::atom_op_info<RecBase>(
                op, arg, atom_index, atom_old, atom_m, atom_n
            );
            if( flag )
            {   atom_state = ret_atom;
                atom_i     = atom_m;
                atom_j     = atom_n;
                //
                atom_x.resize(atom_n);
                type_x.resize(atom_n);
                atom_ix.resize(atom_n);
                atom_iy.resize(atom_m);
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( atom_i == 0 );
                CPPAD_ASSERT_UNKNOWN( atom_j == 0 );
                atom_state = end_atom;
                //
                // call atomic function for this operation
                call_atomic_rev_hes_sparsity<Base,RecBase>(
                    atom_index, atom_old, atom_x, type_x, atom_ix, atom_iy,
                    for_jac_sparse, RevJac, rev_hes_sparse
                );
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
            // argument parameter value
            atom_x[atom_j] = parameter[arg[0]];
            // argument type
            if( dyn_par_is[arg[0]] )
                type_x[atom_j] = dynamic_enum;
            else
                type_x[atom_j] = constant_enum;
            // special variable index used for parameters
            atom_ix[atom_j] = 0;
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
            // argument variables not available during sparsity calculations
            atom_x[atom_j] = CppAD::numeric_limits<Base>::quiet_NaN();
            type_x[atom_j] = variable_enum;
            // variable index for this argument
            atom_ix[atom_j] = size_t(arg[0]);
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
            atom_iy[atom_i] = 0; // special variable used for parameters
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
            atom_iy[atom_i] = i_var; // variable for this result
            //
            if( atom_i == 0 )
                atom_state = arg_atom;
            break;
            // -------------------------------------------------

            case ZmulpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_lin_unary_op(
            i_var, size_t(arg[1]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case ZmulvpOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_lin_unary_op(
            i_var, size_t(arg[0]), RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;
            // -------------------------------------------------

            case ZmulvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::rev_hes_mul_op(
            i_var, arg, RevJac, for_jac_sparse, rev_hes_sparse
            );
            break;

            // -------------------------------------------------

            default:
            CPPAD_ASSERT_UNKNOWN(0);
        }
# if CPPAD_REV_HES_TRACE
        for(j = 0; j < limit; j++)
        {   zf_value[j] = false;
            zh_value[j] = false;
        }
        typename Vector_set::const_iterator itr_jac(for_jac_sparse, i_var);
        j = *itr_jac;
        while( j < limit )
        {   zf_value[j] = true;
            j = *(++itr_jac);
        }
        typename Vector_set::const_iterator itr_hes(rev_hes_sparse, i_var);
        j = *itr_hes;
        while( j < limit )
        {   zh_value[j] = true;
            j = *(++itr_hes);
        }
        printOp<Base, RecBase>(
            std::cout,
            play,
            itr.op_index(),
            i_var,
            op,
            arg
        );
        // should also print RevJac[i_var], but printOpResult does not
        // yet allow for this
        if( NumRes(op) > 0 && op != BeginOp ) printOpResult(
            std::cout,
            1,
            &zf_value,
            1,
            &zh_value
        );
        std::cout << std::endl;
    }
    std::cout << std::endl;
# else
    }
# endif
    // values corresponding to BeginOp
    CPPAD_ASSERT_UNKNOWN( itr.op_index() == 0 );
    CPPAD_ASSERT_UNKNOWN( i_var == 0 );

    return;
}
} } } // END_CPPAD_LOCAL_SWEEP_NAMESPACE

// preprocessor symbols that are local to this file
# undef CPPAD_REV_HES_TRACE

# endif
