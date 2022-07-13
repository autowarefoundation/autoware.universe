# ifndef CPPAD_LOCAL_SWEEP_FOR_JAC_HPP
# define CPPAD_LOCAL_SWEEP_FOR_JAC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <set>
# include <cppad/local/pod_vector.hpp>
# include <cppad/local/play/atom_op_info.hpp>
# include <cppad/local/sweep/call_atomic.hpp>

// BEGIN_CPPAD_LOCAL_SWEEP_NAMESPACE
namespace CppAD { namespace local { namespace sweep {
/*!
\file sweep/for_jac.hpp
Compute Forward mode Jacobian sparsity patterns.
*/

/*!
\def CPPAD_FOR_JAC_TRACE
This value is either zero or one.
Zero is the normal operational value.
If it is one, a trace of every for_jac_sweep computation is printed.
*/
# define CPPAD_FOR_JAC_TRACE 0

/*!
Given the sparsity pattern for the independent variables,
ForJacSweep computes the sparsity pattern for all the other variables.

\tparam Base
this operation sequence was recorded using AD<Base>.

\tparam Vector_set
is the type used for vectors of sets. It can be either
sparse::pack_setvec or sparse::list_setvec.

\param dependency
Are the derivatives with respect to left and right of the expression below
considered to be non-zero:
\code
    CondExpRel(left, right, if_true, if_false)
\endcode
This is used by the optimizer to obtain the correct dependency relations.

\param n
is the number of independent variables on the tape.

\param numvar
is the total number of variables on the tape; i.e.,
 play->num_var_rec().

\param play
The information stored in play
is a recording of the operations corresponding to a function
\f[
    F : {\bf R}^n \rightarrow {\bf R}^m
\f]
where \f$ n \f$ is the number of independent variables
and \f$ m \f$ is the number of dependent variables.

\param var_sparsity
\b Input: For j = 1 , ... , n,
the sparsity pattern for the independent variable with index (j-1)
corresponds to the set with index j in var_sparsity.
\n
\n
\b Output: For i = n + 1 , ... , numvar - 1,
the sparsity pattern for the variable with index i on the tape
corresponds to the set with index i in var_sparsity.

\par Checked Assertions:
\li numvar == var_sparsity.n_set()
\li numvar == play->num_var_rec()

\param not_used_rec_base
Specifies RecBase for this call.
*/

template <class Addr, class Base, class Vector_set, class RecBase>
void for_jac(
    const local::player<Base>* play,
    bool                       dependency        ,
    size_t                     n                 ,
    size_t                     numvar            ,
    Vector_set&                var_sparsity,
    const RecBase&             not_used_rec_base
)
{
    size_t            i, j, k;

    // check numvar argument
    CPPAD_ASSERT_UNKNOWN( play->num_var_rec()  == numvar );
    CPPAD_ASSERT_UNKNOWN( var_sparsity.n_set() == numvar );

    // length of the parameter vector (used by CppAD assert macros)
    const size_t num_par = play->num_par_rec();

    // cum_sparsity accumulates sparsity pattern a cumulative sum
    size_t limit = var_sparsity.end();

    // vecad_sparsity contains a sparsity pattern from each VecAD object
    // to all the other variables.
    // vecad_ind maps a VecAD index (the beginning of the
    // VecAD object) to its from index in vecad_sparsity
    size_t num_vecad_ind   = play->num_var_vecad_ind_rec();
    size_t num_vecad_vec   = play->num_var_vecad_rec();
    Vector_set  vecad_sparsity;
    pod_vector<size_t> vecad_ind;
    if( num_vecad_vec > 0 )
    {   size_t length;
        vecad_sparsity.resize(num_vecad_vec, limit);
        vecad_ind.extend(num_vecad_ind);
        j             = 0;
        for(i = 0; i < num_vecad_vec; i++)
        {   // length of this VecAD
            length   = play->GetVecInd(j);
            // set to proper index for this VecAD
            vecad_ind[j] = i;
            for(k = 1; k <= length; k++)
                vecad_ind[j+k] = num_vecad_vec; // invalid index
            // start of next VecAD
            j       += length + 1;
        }
        CPPAD_ASSERT_UNKNOWN( j == play->num_var_vecad_ind_rec() );
    }

    // --------------------------------------------------------------
    // work space used by AFunOp.
    vector<Base>         atom_x;  //// value of parameter arguments to function
    vector<ad_type_enum> type_x;  // argument types
    pod_vector<size_t>   atom_ix; // variable index (on tape) for each argument
    pod_vector<size_t>   atom_iy; // variable index (on tape) for each result
    //
    // information set by atomic forward (initialization to avoid warnings)
    size_t atom_index=0, atom_old=0, atom_m=0, atom_n=0, atom_i=0, atom_j=0;
    // information set by atomic forward (necessary initialization)
    enum_atom_state atom_state = start_atom;
    // --------------------------------------------------------------
    //
    // pointer to the beginning of the parameter vector
    // (used by atomic functions)
    CPPAD_ASSERT_UNKNOWN( num_par > 0 )
    const Base* parameter = play->GetPar();
    //
    // which parametes are dynamic
    const pod_vector<bool>& dyn_par_is( play->dyn_par_is() );
    //
# if CPPAD_FOR_JAC_TRACE
    vector<size_t>    atom_funrp; // parameter index for FunrpOp operators
    std::cout << std::endl;
    CppAD::vectorBool z_value(limit);
# endif

    // skip the BeginOp at the beginning of the recording
    play::const_sequential_iterator itr = play->begin();
    // op_info
    OpCode op;
    size_t i_var;
    const Addr*   arg;
    itr.op_info(op, arg, i_var);
    CPPAD_ASSERT_UNKNOWN( op == BeginOp );
    //
    bool more_operators = true;
    while(more_operators)
    {   bool flag; // temporary for use in switch cases.

        // this op
        (++itr).op_info(op, arg, i_var);

        // rest of information depends on the case
        switch( op )
        {
            case AbsOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case AddvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_binary_op(
                i_var, arg, var_sparsity
            );
            break;
            // -------------------------------------------------

            case AddpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[1]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case AcosOp:
            // sqrt(1 - x * x), acos(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case AcoshOp:
            // sqrt(x * x - 1), acosh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case AsinOp:
            // sqrt(1 - x * x), asin(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case AsinhOp:
            // sqrt(1 + x * x), asinh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case AtanOp:
            // 1 + x * x, atan(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case AtanhOp:
            // 1 - x * x, atanh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case CSkipOp:
            itr.correct_before_increment();
            break;
            // -------------------------------------------------

            case CSumOp:
            forward_sparse_jacobian_csum_op(
                i_var, arg, var_sparsity
            );
            itr.correct_before_increment();
            break;
            // -------------------------------------------------

            case CExpOp:
            forward_sparse_jacobian_cond_op(
                dependency, i_var, arg, num_par, var_sparsity
            );
            break;
            // --------------------------------------------------

            case CosOp:
            // sin(x), cos(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // ---------------------------------------------------

            case CoshOp:
            // sinh(x), cosh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case DisOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            // derivative is identically zero but dependency is not
            if( dependency ) sparse::for_jac_unary_op(
                i_var, size_t(arg[1]), var_sparsity
            );
            else
                var_sparsity.clear(i_var);
            break;
            // -------------------------------------------------

            case DivvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_binary_op(
                i_var, arg, var_sparsity
            );
            break;
            // -------------------------------------------------

            case DivpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[1]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case DivvpOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case EndOp:
            CPPAD_ASSERT_NARG_NRES(op, 0, 0);
            more_operators = false;
            break;
            // -------------------------------------------------

            case ErfOp:
            case ErfcOp:
            // arg[1] is always the parameter 0
            // arg[0] is always the parameter 2 / sqrt(pi)
            CPPAD_ASSERT_NARG_NRES(op, 3, 5);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case ExpOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case Expm1Op:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case InvOp:
            CPPAD_ASSERT_NARG_NRES(op, 0, 1);
            // sparsity pattern is already defined
            break;
            // -------------------------------------------------

            case LdpOp:
            forward_sparse_load_op(
                dependency,
                op,
                i_var,
                arg,
                num_vecad_ind,
                vecad_ind.data(),
                var_sparsity,
                vecad_sparsity
            );
            break;
            // -------------------------------------------------

            case LdvOp:
            forward_sparse_load_op(
                dependency,
                op,
                i_var,
                arg,
                num_vecad_ind,
                vecad_ind.data(),
                var_sparsity,
                vecad_sparsity
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
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case Log1pOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case MulpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[1]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case MulvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_binary_op(
                i_var, arg, var_sparsity
            );
            break;
            // -------------------------------------------------

            case ParOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            var_sparsity.clear(i_var);
            break;
            // -------------------------------------------------

            case PowvpOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case PowpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 3);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[1]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case PowvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 3);
            sparse::for_jac_binary_op(
                i_var, arg, var_sparsity
            );
            break;
            // -------------------------------------------------

            case PriOp:
            CPPAD_ASSERT_NARG_NRES(op, 5, 0);
            break;
            // -------------------------------------------------

            case SignOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            // derivative is identically zero but dependency is not
            if( dependency ) sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            else
                var_sparsity.clear(i_var);
            break;
            // -------------------------------------------------

            case SinOp:
            // cos(x), sin(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case SinhOp:
            // cosh(x), sinh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case SqrtOp:
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case StppOp:
            CPPAD_ASSERT_NARG_NRES(op, 3, 0);
            // if both arguments are parameters does not affect sparsity
            // or dependency
            break;
            // -------------------------------------------------

            case StpvOp:
            forward_sparse_store_op(
                dependency,
                op,
                arg,
                num_vecad_ind,
                vecad_ind.data(),
                var_sparsity,
                vecad_sparsity
            );
            break;
            // -------------------------------------------------

            case StvpOp:
            CPPAD_ASSERT_NARG_NRES(op, 3, 0);
            forward_sparse_store_op(
                dependency,
                op,
                arg,
                num_vecad_ind,
                vecad_ind.data(),
                var_sparsity,
                vecad_sparsity
            );
            break;
            // -------------------------------------------------

            case StvvOp:
            forward_sparse_store_op(
                dependency,
                op,
                arg,
                num_vecad_ind,
                vecad_ind.data(),
                var_sparsity,
                vecad_sparsity
            );
            break;
            // -------------------------------------------------

            case SubvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_binary_op(
                i_var, arg, var_sparsity
            );
            break;
            // -------------------------------------------------

            case SubpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[1]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case SubvpOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case TanOp:
            // tan(x)^2, tan(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case TanhOp:
            // tanh(x)^2, tanh(x)
            CPPAD_ASSERT_NARG_NRES(op, 1, 2);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case AFunOp:
            // start or end an atomic function call
            CPPAD_ASSERT_UNKNOWN(
                atom_state == start_atom || atom_state == end_atom
            );
            flag = atom_state == start_atom;
            play::atom_op_info<RecBase>(
                op, arg, atom_index, atom_old, atom_m, atom_n
            );
            if( flag )
            {   atom_state = arg_atom;
                atom_i     = 0;
                atom_j     = 0;
                //
                atom_x.resize( atom_n );
                type_x.resize( atom_n );
                atom_ix.resize( atom_n );
                atom_iy.resize( atom_m );
# if CPPAD_FOR_JAC_TRACE
                atom_funrp.resize( atom_m );
# endif
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( atom_i == atom_m );
                CPPAD_ASSERT_UNKNOWN( atom_j == atom_n );
                atom_state = start_atom;
                //
                call_atomic_for_jac_sparsity<Base,RecBase>(
                    atom_index,
                    atom_old,
                    dependency,
                    atom_x,
                    type_x,
                    atom_ix,
                    atom_iy,
                    var_sparsity
                );
            }
            break;

            case FunapOp:
            // parameter argument for a atomic function
            CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
            CPPAD_ASSERT_UNKNOWN( atom_state == arg_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i == 0 );
            CPPAD_ASSERT_UNKNOWN( atom_j < atom_n );
            CPPAD_ASSERT_UNKNOWN( size_t( arg[0] ) < num_par );
            //
            atom_x[atom_j]  = parameter[arg[0]];
            // argument type
            if( dyn_par_is[arg[0]] )
                type_x[atom_j] = dynamic_enum;
            else
                type_x[atom_j] = constant_enum;
            atom_ix[atom_j] = 0; // special variable used for parameters
            //
            ++atom_j;
            if( atom_j == atom_n )
                atom_state = ret_atom;
            break;

            case FunavOp:
            // variable argument for a atomic function
            CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
            CPPAD_ASSERT_UNKNOWN( atom_state == arg_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i == 0 );
            CPPAD_ASSERT_UNKNOWN( atom_j < atom_n );
            //
            // argument variables not avaiable during sparsity calculations
            atom_x[atom_j]  = CppAD::numeric_limits<Base>::quiet_NaN();
            type_x[atom_j]  = variable_enum;
            atom_ix[atom_j] = size_t(arg[0]); // variable for this argument
            //
            ++atom_j;
            if( atom_j == atom_n )
                atom_state = ret_atom;
            break;

            case FunrpOp:
            // parameter result for a atomic function
            CPPAD_ASSERT_NARG_NRES(op, 1, 0);
            CPPAD_ASSERT_UNKNOWN( atom_state == ret_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i < atom_m );
            CPPAD_ASSERT_UNKNOWN( atom_j == atom_n );
            CPPAD_ASSERT_UNKNOWN( size_t( arg[0] ) < num_par );
            //
            atom_iy[atom_i] = 0; // special value for parameters
# if CPPAD_FOR_JAC_TRACE
            // remember argument for delayed tracing
            atom_funrp[atom_i] = arg[0];
# endif
            ++atom_i;
            if( atom_i == atom_m )
                atom_state = end_atom;
            break;

            case FunrvOp:
            // variable result for a atomic function
            CPPAD_ASSERT_NARG_NRES(op, 0, 1);
            CPPAD_ASSERT_UNKNOWN( atom_state == ret_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i < atom_m );
            CPPAD_ASSERT_UNKNOWN( atom_j == atom_n );
            //
            atom_iy[atom_i] = i_var; // variable index for this result
            //
            ++atom_i;
            if( atom_i == atom_m )
                atom_state = end_atom;
            break;
            // -------------------------------------------------

            case ZmulpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[1]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case ZmulvpOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_unary_op(
                i_var, size_t(arg[0]), var_sparsity
            );
            break;
            // -------------------------------------------------

            case ZmulvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            sparse::for_jac_binary_op(
                i_var, arg, var_sparsity
            );
            break;
            // -------------------------------------------------

            default:
            CPPAD_ASSERT_UNKNOWN(0);
        }
# if CPPAD_FOR_JAC_TRACE
        if( op == AFunOp && atom_state == start_atom )
        {   // print operators that have been delayed
            CPPAD_ASSERT_UNKNOWN( atom_m == atom_iy.size() );
            CPPAD_ASSERT_UNKNOWN( itr.op_index() > atom_m );
            CPPAD_ASSERT_NARG_NRES(FunrpOp, 1, 0);
            CPPAD_ASSERT_NARG_NRES(FunrvOp, 0, 1);
            addr_t arg_tmp[1];
            for(i = 0; i < atom_m; i++)
            {   size_t j_var = atom_iy[i];
                // value for this variable
                for(j = 0; j < limit; j++)
                    z_value[j] = false;
                typename Vector_set::const_iterator itr(var_sparsity, j_var);
                j = *itr;
                while( j < limit )
                {   z_value[j] = true;
                    j = *(++itr);
                }
                OpCode op_tmp = FunrvOp;
                if( j_var == 0 )
                {   op_tmp     = FunrpOp;
                    arg_tmp[0] = atom_funrp[i];
                }
                // j_var is zero when there is no result.
                printOp<Base, RecBase>(
                    std::cout,
                    play,
                    itr.op_index() - atom_m + i,
                    j_var,
                    op_tmp,
                    arg_tmp
                );
                if( j_var > 0 ) printOpResult(
                    std::cout,
                    1,
                    &z_value,
                    0,
                    (CppAD::vectorBool *) nullptr
                );
                std::cout << std::endl;
            }
        }
        // value for this variable
        for(j = 0; j < limit; j++)
            z_value[j] = false;
        typename Vector_set::const_iterator itr(var_sparsity, i_var);
        j = *itr;
        while( j < limit )
        {   z_value[j] = true;
            j = *(++itr);
        }
        // must delay print for these cases till after atomic function call
        bool delay_print = op == FunrpOp;
        delay_print     |= op == FunrvOp;
        if( ! delay_print )
        {    printOp<Base, RecBase>(
                std::cout,
                play,
                itr.op_index(),
                i_var,
                op,
                arg
            );
            if( NumRes(op) > 0 && (! delay_print) ) printOpResult(
                std::cout,
                1,
                &z_value,
                0,
                (CppAD::vectorBool *) nullptr
            );
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;
# else
    }
# endif

    return;
}

} } } // END_CPPAD_LOCAL_SWEEP_NAMESPACE

// preprocessor symbols that are local to this file
# undef CPPAD_FOR_JAC_TRACE

# endif
