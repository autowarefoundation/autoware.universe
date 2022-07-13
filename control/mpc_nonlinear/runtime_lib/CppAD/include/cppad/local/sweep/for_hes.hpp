# ifndef CPPAD_LOCAL_SWEEP_FOR_HES_HPP
# define CPPAD_LOCAL_SWEEP_FOR_HES_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-22 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/play/atom_op_info.hpp>

/*
$begin local_sweep_for_hes$$
$spell
    hes
    numvar
    jac
    Jacobian
    num_var
    Addr
    InvOp
    setvec
$$

$section Forward Mode Hessian Sparsity Patterns$$

$head Syntax$$
$codei%local::sweep::for_hes(
    %play%              ,
    %n%                 ,
    %numvar%            ,
    %select_domain%     ,
    %rev_jac_sparse%    ,
    %for_hes_sparse%    ,
    %not_used_rec_base
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%1
%$$


$head Purpose$$
Given the forward Jacobian sparsity pattern for all the variables,
and the reverse Jacobian sparsity pattern for the dependent variables,
$code for_hes$$ computes the Hessian sparsity pattern for all the independent
variables.

$head Tracing$$
This value is either zero or one.  Zero is the normal operational value.
If it is one, a trace of Jacobian and Hessian sparsity result for every
operation for every $code for_hes$$ sweep is printed.
The sparsity patterns are printed as binary numbers with 1 (0) meaning that
the corresponding index is (is not) in the set.
$codep */
# define CPPAD_FOR_HES_TRACE 0
/* $$

$head Addr$$
Is the type used to record address on this tape
This is allows for smaller tapes when address are smaller.

$head Base$$
The operation sequence in $icode play$$ was recorded using
$codei%AD<%Base%>%$$.

$head RecBase$$
Is the base type when this function was recorded.
This is different from $icode Base$$ if
this function object was created by $cref base2ad$$.

$head SetVector$$
This is a $cref SetVector$$ type.

$head play$$
The information stored in play
is a recording of the operations corresponding to a function
$latex F : \B{R}^n \rightarrow \B{R}^m$$
where $icode m$$ is the number of dependent variables.

$head n$$
is the number of independent variables in the tape.

$head numvar$$
is the total number of variables in the tape; i.e.,
$icode%play%->num_var_rec()%$$.
This is also the number of sets in all the sparsity patterns.

$head select_domain$$
is a vector with size $icode n$$ that specifies
which components of the domain to include in the Hessian sparsity pattern.
For $icode%j%= 0, ..., %n%-1%$$, the $th j$$ independent variable
will be included if and only if $icode%select_domain%[%j%]%$$ is true.
This assumes that the order of the independent variables is the same
as the order of the InvOp operators.

$head rev_jac_sparse$$
Is a sparsity pattern with size $icode numvar$$ by one.
For $icode%i%=1, %...%, %numvar%-1%$$,
the if the function we are computing the Hessian for has a non-zero
derivative w.r.t. variable with index $icode i$$,
the set with index $icode i$$ has the element zero.
Otherwise it has no elements.

$head for_hes_sparse$$
Is a sparsity pattern with size $icode%n%+1+%numvar%$$ by $icode%n%+1%$$.
The set with index zero and the element zero are not used.
The sets with index greater than $icode n$$
are used for forward Jacobian sparsity.
The forward Hessian sparsity pattern for the variable with index $icode i$$
corresponds to the set with index $icode i$$ in $icode for_hes_sparse$$.
The number of sets in this sparsity pattern is $icode%n%+1%$$ and the set
with index zero is not used.

$subhead On Input$$
For $icode%j%=1, %...%, %n%$$,
the forward Hessian sparsity pattern for the variable with index
$icode i$$ is empty.

$subhead On Output$$
For $icode%j%=1, %...%, %n%$$,
the forward Hessian sparsity pattern for the independent dependent variable
with index $icode%j%-1%$$ is given by the set with index $icode j$$
in $icode for_hes_sparse$$.

$head not_used_rec_base$$
This argument is only used to specify the type $icode RecBase$$ for this call.

$end
*/

// BEGIN_CPPAD_LOCAL_SWEEP_NAMESPACE
namespace CppAD { namespace local { namespace sweep {

// BEGIN PROTOTYPE
template <class Addr, class Base, class SetVector, class RecBase>
void for_hes(
    const local::player<Base>* play                ,
    size_t                     n                   ,
    size_t                     numvar              ,
    const pod_vector<bool>&    select_domain       ,
    const SetVector&           rev_jac_sparse      ,
    SetVector&                 for_hes_sparse      ,
    const RecBase&             not_used_rec_base   )
// END PROTOTYPE
{
    // length of the parameter vector (used by CppAD assert macros)
# ifndef NDEBUG
    const size_t num_par = play->num_par_rec();
# endif

    // check arguments
    size_t np1 = n+1;
    CPPAD_ASSERT_UNKNOWN( select_domain.size()   == n );
    CPPAD_ASSERT_UNKNOWN( play->num_var_rec()    == numvar );
    CPPAD_ASSERT_UNKNOWN( rev_jac_sparse.n_set() == numvar );
    CPPAD_ASSERT_UNKNOWN( for_hes_sparse.n_set() == np1+numvar );
    //
    CPPAD_ASSERT_UNKNOWN( rev_jac_sparse.end()   == 1   );
    CPPAD_ASSERT_UNKNOWN( for_hes_sparse.end()   == np1 );
    //
    CPPAD_ASSERT_UNKNOWN( numvar > 0 );
    //
    // vecad_sparsity contains a sparsity pattern for each VecAD object.
    // vecad_ind maps a VecAD index (beginning of the VecAD object)
    // to the index for the corresponding set in vecad_sparsity.
    size_t num_vecad_ind   = play->num_var_vecad_ind_rec();
    size_t num_vecad_vec   = play->num_var_vecad_rec();
    SetVector vecad_sparse;
    pod_vector<size_t> vecad_ind;
    pod_vector<bool>   vecad_jac;
    if( num_vecad_vec > 0 )
    {   size_t length;
        vecad_sparse.resize(num_vecad_vec, np1);
        vecad_ind.extend(num_vecad_ind);
        vecad_jac.extend(num_vecad_vec);
        size_t j  = 0;
        for(size_t i = 0; i < num_vecad_vec; i++)
        {   // length of this VecAD
            length   = play->GetVecInd(j);
            // set vecad_ind to proper index for this VecAD
            vecad_ind[j] = i;
            // make all other values for this vector invalid
            for(size_t k = 1; k <= length; k++)
                vecad_ind[j+k] = num_vecad_vec;
            // start of next VecAD
            j       += length + 1;
            // initialize this vector's reverse jacobian value
            vecad_jac[i] = false;
        }
        CPPAD_ASSERT_UNKNOWN( j == play->num_var_vecad_ind_rec() );
    }
    // ------------------------------------------------------------------------
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
    // -------------------------------------------------------------------------
    //
    // pointer to the beginning of the parameter vector
    // (used by atomic functions)
    CPPAD_ASSERT_UNKNOWN( num_par > 0 )
    const Base* parameter = play->GetPar();
    //
    // which parametes are dynamic
    const pod_vector<bool>& dyn_par_is( play->dyn_par_is() );
    //
    // skip the BeginOp at the beginning of the recording
    play::const_sequential_iterator itr = play->begin();
    // op_info
    OpCode op;
    size_t i_var;
    const Addr*   arg;
    itr.op_info(op, arg, i_var);
    CPPAD_ASSERT_UNKNOWN( op == BeginOp );
# if CPPAD_FOR_HES_TRACE
    vector<Addr> atom_funrp; // parameter index for FunrpOp operators
    std::cout << std::endl;
    CppAD::vectorBool zf_value(np1);
    CppAD::vectorBool zh_value(np1 * np1);
# endif
    bool   flag; // temporary for use in switch cases below
    bool   more_operators = true;
    size_t count_independent = 0;
    while(more_operators)
    {
        // next op
        (++itr).op_info(op, arg, i_var);

        // does the Hessian in question have a non-zero derivative
        // with respect to this variable
        bool include = NumRes(op) > 0;
        if( include )
            include = rev_jac_sparse.is_element(i_var, 0);
        //
        // operators to include even if derivative is zero
        include |= op == EndOp;
        include |= op == CSkipOp;
        include |= op == CSumOp;
        include |= op == AFunOp;
        include |= op == FunapOp;
        include |= op == FunavOp;
        include |= op == FunrpOp;
        include |= op == FunrvOp;
        //
        if( ! include )
        {   if( op == InvOp )
                ++count_independent;
        }
        else switch( op )
        {   // operators that should not occurr
            // case BeginOp

            // operators that do not affect Jacobian or Hessian
            // and where with a fixed number of arguments and results
            case CExpOp:
            case DisOp:
            case LdpOp:
            case LdvOp:
            case ParOp:
            case PriOp:
            case SignOp:
            case StppOp:
            case StpvOp:
            case StvpOp:
            case StvvOp:
            break;
            // -------------------------------------------------

            // independent variable operator: set J(i_var) = { i_var }
            case InvOp:
            CPPAD_ASSERT_UNKNOWN( for_hes_sparse.number_elements(i_var) == 0 );
            if( select_domain[count_independent] )
            {   // Not using post_element becasue only adding one element
                // per set
                for_hes_sparse.add_element(np1 + i_var, i_var);
            }
            ++count_independent;
            break;

            // -------------------------------------------------
            // linear operators where arg[0] is the only variable
            // only assign Jacobian term J(i_var)
            case AbsOp:
            case DivvpOp:
            case SubvpOp:
            case ZmulvpOp:
            for_hes_sparse.assignment(
                np1 + i_var, np1 + size_t(arg[0]), for_hes_sparse
            );
            break;

            // -------------------------------------------------
            // linear operators where arg[1] is the only variable
            // only assign Jacobian term J(i_var)
            case AddpvOp:
            case MulpvOp:
            case SubpvOp:
            for_hes_sparse.assignment(
                np1 + i_var, np1 + size_t(arg[1]), for_hes_sparse
            );
            break;

            // -------------------------------------------------
            // linear operators where arg[0] and arg[1] are variables
            // only assign Jacobian term J(i_var)
            case AddvvOp:
            case SubvvOp:
            for_hes_sparse.binary_union(
                np1 + i_var          ,
                np1 + size_t(arg[0]) ,
                np1 + size_t(arg[1]) ,
                for_hes_sparse
            );
            break;

            // nonlinear unary operators
            case AcosOp:
            case AsinOp:
            case AtanOp:
            case CosOp:
            case CoshOp:
            case ExpOp:
            case LogOp:
            case NegOp:
            case SinOp:
            case SinhOp:
            case SqrtOp:
            case TanOp:
            case TanhOp:
            case AcoshOp:
            case AsinhOp:
            case AtanhOp:
            case Expm1Op:
            case Log1pOp:
            CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 )
            sparse::for_hes_nl_unary_op(
                np1, numvar, i_var, size_t(arg[0]), for_hes_sparse
            );
            break;
            // -------------------------------------------------

            case CSkipOp:
            itr.correct_before_increment();
            break;
            // -------------------------------------------------

            case CSumOp:
            itr.correct_before_increment();
            break;
            // -------------------------------------------------

            case DivvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::for_hes_div_op(
                np1, numvar, i_var, arg, for_hes_sparse
            );
            break;
            // -------------------------------------------------

            case DivpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::for_hes_nl_unary_op(
                np1, numvar, i_var, size_t(arg[1]), for_hes_sparse
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
            // arg[2] is always the parameter 2 / sqrt(pi)
            CPPAD_ASSERT_NARG_NRES(op, 3, 5);
            sparse::for_hes_nl_unary_op(
                np1, numvar, i_var, size_t(arg[0]), for_hes_sparse
            );
            break;
            // -------------------------------------------------

            // -------------------------------------------------
            // logical comparison operators
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
            case NepvOp:
            case NeppOp:
            case NevvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 0);
            break;
            // -------------------------------------------------

            case MulvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::for_hes_mul_op(
                np1, numvar, i_var, arg, for_hes_sparse
            );
            break;
            // -------------------------------------------------

            case PowpvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 3)
            sparse::for_hes_nl_unary_op(
                np1, numvar, i_var, size_t(arg[1]), for_hes_sparse
            );
            break;
            // -------------------------------------------------

            case PowvpOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::for_hes_nl_unary_op(
                np1, numvar, i_var, size_t(arg[0]), for_hes_sparse
            );
            break;
            // -------------------------------------------------

            case PowvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 3)
            sparse::for_hes_pow_op(
                np1, numvar, i_var, arg, for_hes_sparse
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
# if CPPAD_FOR_HES_TRACE
                atom_funrp.resize( atom_m );
# endif
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( atom_i == atom_m );
                CPPAD_ASSERT_UNKNOWN( atom_j == atom_n );
                atom_state = start_atom;
                //
                call_atomic_for_hes_sparsity<Base,RecBase>(
                    atom_index, atom_old, atom_x, type_x, atom_ix, atom_iy,
                    np1, numvar, rev_jac_sparse, for_hes_sparse
                );
            }
            break;

            case FunapOp:
            // parameter argument for an atomic function
            CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
            CPPAD_ASSERT_UNKNOWN( atom_state == arg_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i == 0 );
            CPPAD_ASSERT_UNKNOWN( atom_j < atom_n );
            CPPAD_ASSERT_UNKNOWN( size_t( arg[0] ) < num_par );
            //
            atom_x[atom_j] = parameter[arg[0]];
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
            // variable argument for an atomic function
            CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
            CPPAD_ASSERT_UNKNOWN( atom_state == arg_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i == 0 );
            CPPAD_ASSERT_UNKNOWN( atom_j < atom_n );
            CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
            CPPAD_ASSERT_UNKNOWN( size_t( arg[0] ) < numvar );
            //
            // arguemnt variables not avaialbe during sparisty calculations
            atom_x[atom_j] = CppAD::numeric_limits<Base>::quiet_NaN();
            type_x[atom_j] = variable_enum;
            atom_ix[atom_j] = size_t(arg[0]); // variable for this argument
            //
            ++atom_j;
            if( atom_j == atom_n )
                atom_state = ret_atom;
            break;

            case FunrpOp:
            // parameter result for an atomic function
            CPPAD_ASSERT_NARG_NRES(op, 1, 0);
            CPPAD_ASSERT_UNKNOWN( atom_state == ret_atom );
            CPPAD_ASSERT_UNKNOWN( atom_i < atom_m );
            CPPAD_ASSERT_UNKNOWN( atom_j == atom_n );
            CPPAD_ASSERT_UNKNOWN( size_t( arg[0] ) < num_par );
            //
            atom_iy[atom_i] = 0; // special variable used for parameters
# if CPPAD_FOR_HES_TRACE
            // remember argument for delayed tracing
            atom_funrp[atom_i] = arg[0];
# endif
            ++atom_i;
            if( atom_i == atom_m )
                atom_state = end_atom;
            break;

            case FunrvOp:
            // variable result for an atomic function
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

            case ZmulvvOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1)
            sparse::for_hes_mul_op(
                np1, numvar, i_var, arg, for_hes_sparse
            );
            break;

            // -------------------------------------------------

            default:
            CPPAD_ASSERT_UNKNOWN(0);
        }
# if CPPAD_FOR_HES_TRACE
        typedef typename SetVector::const_iterator const_iterator;
        if( op == AFunOp && atom_state == start_atom )
        {   // print operators that have been delayed
            CPPAD_ASSERT_UNKNOWN( atom_m == atom_iy.size() );
            CPPAD_ASSERT_UNKNOWN( itr.op_index() > atom_m );
            CPPAD_ASSERT_NARG_NRES(FunrpOp, 1, 0);
            CPPAD_ASSERT_NARG_NRES(FunrvOp, 0, 1);
            addr_t arg_tmp[1];
            for(size_t k = 0; k < atom_m; k++)
            {   size_t k_var = atom_iy[k];
                // value for this variable
                for(size_t i = 0; i < np1; i++)
                {   zf_value[i] = false;
                    for(size_t j = 0; j < np1; j++)
                        zh_value[i * np1 + j] = false;
                }
                const_iterator itr_1(for_hes_sparse, np1 + i_var);
                size_t j = *itr_1;
                while( j < np1 )
                {   zf_value[j] = true;
                    j = *(++itr_1);
                }
                for(size_t i = 0; i < np1; i++)
                {   const_iterator itr_2(for_hes_sparse, i);
                    j = *itr_2;
                    while( j < np1 )
                    {   zh_value[i * np1 + j] = true;
                        j = *(++itr_2);
                    }
                }
                OpCode op_tmp = FunrvOp;
                if( k_var == 0 )
                {   op_tmp     = FunrpOp;
                    arg_tmp[0] = atom_funrp[k];
                }
                // k_var is zero when there is no result
                printOp<Base, RecBase>(
                    std::cout,
                    play,
                    itr.op_index() - atom_m + k,
                    k_var,
                    op_tmp,
                    arg_tmp
                );
                if( k_var > 0 ) printOpResult(
                    std::cout,
                    1,
                    &zf_value,
                    1,
                    &zh_value
                );
                std::cout << std::endl;
            }
        }
        for(size_t i = 0; i < np1; i++)
        {   zf_value[i] = false;
            for(size_t j = 0; j < np1; j++)
                zh_value[i * np1 + j] = false;
        }
        const_iterator itr_1(for_hes_sparse, np1 + i_var);
        size_t j = *itr_1;
        while( j < np1 )
        {   zf_value[j] = true;
            j = *(++itr_1);
        }
        for(size_t i = 0; i < np1; i++)
        {   const_iterator itr_2(for_hes_sparse, i);
            j = *itr_2;
            while( j < np1 )
            {   zh_value[i * np1 + j] = true;
                j = *(++itr_2);
            }
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
                &zf_value,
                1,
                &zh_value
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
# undef CPPAD_FOR_HES_TRACE

# endif
