# ifndef CPPAD_LOCAL_OPTIMIZE_GET_PAR_USAGE_HPP
# define CPPAD_LOCAL_OPTIMIZE_GET_PAR_USAGE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*!
\file get_cexp_info.hpp
Create operator information tables
*/
# include <cppad/local/optimize/get_op_usage.hpp>

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize {

/*!
$begin optimize_get_par_usage$$
$spell
    Addr
    iterator
    itr
    op
    num
    var
    vecad
    Vec
    Ind
$$

$section Use Reverse Activity Analysis to Get Usage for Each Parameter$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_GET_PAR_USAGE%// END_PROTOTYPE%1
%$$

$head Base$$
Base type for the operator; i.e., this operation was recorded
using $codei%AD<%Base%>%$$
and computations by this routine are done using type $icode Base$$.

$head Addr$$
Type used by random iterator for the player.

$head play$$
This is the operation sequence.

$head random_itr$$
This is a random iterator for the operation sequence.

$head op_usage$$
This argument has size equal to the number of operators
in the operation sequence; i.e., num_op = play->nun_var_rec().
The value $icode%op_usage%[%i%]%$$ have been set to the usage for
the i-th operator in the operation sequence.

$head vecad_used$$
This argument has size equal to the number of VecAD vectors
in the operations sequences; i.e., play->num_var_vecad_rec().
The VecAD vectors are indexed in the order that their indices appear
in the one large play->GetVecInd that holds all the VecAD vectors.

$head par_usage$$
The input size of this vector must be zero.
Upon return it has size equal to the number of parameters
in the operation sequence; i.e., play->num_par_rec();
The value $icode%par_usage%[%i%]%$$ is true if an only if
the i-th parameter is used to compute a dependent variable
or parameter.
The nan at the beginning of the parameter vector
and the independent dynamic parameters are always used.

$end
*/

// BEGIN_GET_PAR_USAGE
template <class Addr, class Base>
void get_par_usage(
    const player<Base>*                         play                ,
    const play::const_random_iterator<Addr>&    random_itr          ,
    const pod_vector<usage_t>&                  op_usage            ,
    pod_vector<bool>&                           vecad_used          ,
    pod_vector<bool>&                           par_usage           )
// END_PROTOTYPE
{
    CPPAD_ASSERT_UNKNOWN( op_usage.size()   == play->num_op_rec() );
    CPPAD_ASSERT_UNKNOWN( par_usage.size()  == 0 );
    //
    // number of operators in the tape
    const size_t num_op = play->num_op_rec();
    //
    // number of parameters in the tape
    const size_t num_par = play->num_par_rec();
    //
    // number of dynamic parameters
    const size_t num_dynamic_par = play->num_dynamic_par();
    //
    // number of independent dynamic parameters
    size_t num_dynamic_ind = play->num_dynamic_ind();
    //
    // number of VecAD vectors
    size_t num_vecad_vec = play->num_var_vecad_rec();
    //
    // dynamic parameter information
    const pod_vector<bool>&        dyn_par_is( play->dyn_par_is() );
    const pod_vector<opcode_t>&    dyn_par_op( play->dyn_par_op() );
    const pod_vector<addr_t>&      dyn_par_arg( play->dyn_par_arg() );
    const pod_vector<addr_t>&      dyn_ind2par_ind( play->dyn_ind2par_ind() );
    const pod_vector_maybe<Base>&  all_par_vec( play->all_par_vec() );
    // -----------------------------------------------------------------------
    // initialize par_usage
    par_usage.resize(num_par);
    par_usage[0] = true; // true for nan at beginning of parameter vector
    for(size_t i_par = 1; i_par <= num_dynamic_ind; ++i_par)
        par_usage[i_par] = true;  // true for independent dynamic parameters
    for(size_t i_par = num_dynamic_ind+1; i_par < num_par; ++i_par)
        par_usage[i_par] = false; // initialize as false for other parameters
    //
    // -----------------------------------------------------------------------
    // set usage to true for VecAD parameters that get used
    size_t start_this_vector = 0;
    for(size_t i_vec = 0; i_vec < num_vecad_vec; ++i_vec)
    {   // length of this vector (note length is not a parameter)
        size_t length = play->GetVecInd(start_this_vector);
        //
        if( vecad_used[i_vec] )
        {   // this vector gets used
            for(size_t k = 1; k <= length; ++k)
            {   // index of parameter used by this VecAD vector
                size_t i_par = play->GetVecInd(start_this_vector + k);
                // must not be a dynamic parameter
                CPPAD_ASSERT_UNKNOWN( ! dyn_par_is[i_par] );
                // set usage for this parameter
                par_usage[i_par] = true;
            }
        }
        start_this_vector += length + 1;
    }
    CPPAD_ASSERT_UNKNOWN( start_this_vector == play->num_var_vecad_ind_rec() );
    //
    // -----------------------------------------------------------------------
    // forward pass to mark which parameters are used by necessary operators
    // -----------------------------------------------------------------------
    //
    // information about atomic function calls
    size_t atom_index=0, atom_old=0, atom_m=0, atom_n=0, atom_i=0, atom_j=0;
    enum_atom_state atom_state = start_atom;
    //
    // work space used by user atomic functions
    vector<Base>         parameter_x; // value of parameters in x
    vector<ad_type_enum> type_x;      // type for each component of z
    vector<size_t>       atom_ix;     // variables indices for argument vector
    vector<bool>         depend_y;    // results that are used
    vector<bool>         depend_x;    // arguments that are used
    //
    for(size_t i_op = 0; i_op < num_op; ++i_op)
    {
        // information about current operator
        OpCode        op;     // operator
        const addr_t* arg;    // arguments
        size_t        i_var;  // variable index of first result
        random_itr.op_info(i_op, op, arg, i_var);
        //
        bool skip = op_usage[i_op] == usage_t(no_usage);
        skip     &= atom_state == start_atom;
        if( ! skip ) switch( op )
        {
            // add or subtract with left a parameter and right a variable
            case AddpvOp:
            case SubpvOp:
            if( dyn_par_is[ arg[0] ] )
                par_usage[ arg[0] ] = true;
            else
            {   // determine if this parameter will be absorbed by csum
                if( ! (op_usage[i_op] == csum_usage) )
                {   // determine operator corresponding to variable
                    size_t j_op = random_itr.var2op(size_t(arg[1]));
                    CPPAD_ASSERT_UNKNOWN( op_usage[j_op] != no_usage );
                    if( op_usage[j_op] != csum_usage )
                        par_usage[ arg[0] ] = true;
                }
            }
            break;

            // subtract with left a variable and right a parameter
            case SubvpOp:
            if( dyn_par_is[ arg[1] ] )
                par_usage[ arg[1] ] = true;
            else
            {   // determine if this parameter will be absorbed by csum
                if( ! (op_usage[i_op] == csum_usage) )
                {   // determine operator corresponding to variable
                    size_t j_op = random_itr.var2op(size_t(arg[0]));
                    CPPAD_ASSERT_UNKNOWN( op_usage[j_op] != no_usage );
                    if( op_usage[j_op] != csum_usage )
                        par_usage[ arg[1] ] = true;
                }
            }
            break;



            // cases with no parameter arguments
            case AbsOp:
            case AcosOp:
            case AcoshOp:
            case AddvvOp:
            case AsinOp:
            case AsinhOp:
            case AtanOp:
            case AtanhOp:
            case BeginOp:
            case CosOp:
            case CoshOp:
            case CSkipOp:
            case DisOp:
            case DivvvOp:
            case EndOp:
            case EqvvOp:
            case ExpOp:
            case Expm1Op:
            case InvOp:
            case LdvOp:
            case LevvOp:
            case LogOp:
            case Log1pOp:
            case LtvvOp:
            case MulvvOp:
            case NegOp:
            case NevvOp:
            case PowvvOp:
            case SignOp:
            case SinOp:
            case SinhOp:
            case SqrtOp:
            case StvvOp:
            case SubvvOp:
            case TanOp:
            case TanhOp:
            case ZmulvvOp:
            break;

            // cases where first and second arguments are parameters
            case EqppOp:
            case LeppOp:
            case LtppOp:
            case NeppOp:
            CPPAD_ASSERT_UNKNOWN( 2 <= NumArg(op) )
            par_usage[arg[0]] = true;
            par_usage[arg[1]] = true;
            break;


            // cases where only first argument is a parameter
            case CSumOp:
            case EqpvOp:
            case DivpvOp:
            case LepvOp:
            case LtpvOp:
            case MulpvOp:
            case NepvOp:
            case ParOp:
            case PowpvOp:
            case ZmulpvOp:
            CPPAD_ASSERT_UNKNOWN( 1 <= NumArg(op) || op == CSumOp )
            par_usage[arg[0]] = true;
            break;

            // cases where only second argument is a parameter
            case DivvpOp:
            case LevpOp:
            case LdpOp:
            case LtvpOp:
            case PowvpOp:
            case StpvOp:
            case ZmulvpOp:
            CPPAD_ASSERT_UNKNOWN( 2 <= NumArg(op) )
            par_usage[arg[1]] = true;
            break;

            // cases where second and thrid arguments are parameters
            case ErfOp:
            case ErfcOp:
            case StppOp:
            CPPAD_ASSERT_UNKNOWN( 3 <= NumArg(op) )
            par_usage[arg[1]] = true;
            par_usage[arg[2]] = true;
            break;

            // cases where only third argument is a parameter
            case StvpOp:
            CPPAD_ASSERT_UNKNOWN( 3 <= NumArg(op) )
            par_usage[arg[2]] = true;
            break;

            // conditional expression operator
            case CExpOp:
            CPPAD_ASSERT_UNKNOWN( 6 == NumArg(op) )
            if( (arg[1] & 1) == 0 )
                par_usage[arg[2]] = true;
            if( (arg[1] & 2) == 0 )
                par_usage[arg[3]] = true;
            if( (arg[1] & 4) == 0 )
                par_usage[arg[4]] = true;
            if( (arg[1] & 8) == 0 )
                par_usage[arg[5]] = true;
            break;

            // print function
            case PriOp:
            if( (arg[0] & 1) == 0 )
                par_usage[arg[1]] = true;
            if( (arg[0] & 2) == 0 )
                par_usage[arg[3]] = true;
            CPPAD_ASSERT_UNKNOWN( 5 == NumArg(op) )
            break;

            // --------------------------------------------------------------
            // atomic function calls
            case AFunOp:
            if( atom_state == start_atom )
            {   atom_index        = size_t(arg[0]);
                atom_old          = size_t(arg[1]);
                atom_n            = size_t(arg[2]);
                atom_m            = size_t(arg[3]);
                atom_j            = 0;
                atom_i            = 0;
                atom_state        = arg_atom;
                // -------------------------------------------------------
                parameter_x.resize(  atom_n );
                type_x.resize( atom_n );
                atom_ix.resize( atom_n );
                //
                depend_y.resize( atom_m );
                depend_x.resize( atom_n );
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( atom_state == end_atom );
                CPPAD_ASSERT_UNKNOWN( atom_n == size_t(arg[2]) );
                CPPAD_ASSERT_UNKNOWN( atom_m == size_t(arg[3]) );
                CPPAD_ASSERT_UNKNOWN( atom_j == atom_n );
                CPPAD_ASSERT_UNKNOWN( atom_i == atom_m );
                atom_state = start_atom;
                //
                // call atomic function for this operation
                sweep::call_atomic_rev_depend<Base, Base>(
                atom_index, atom_old, parameter_x, type_x, depend_x, depend_y
                );
                for(size_t j = 0; j < atom_n; j++)
                if( depend_x[j] && type_x[j] != variable_enum )
                {   // This user argument is a parameter that is needed

                       CPPAD_ASSERT_UNKNOWN( atom_ix[j] > 0 );
                    par_usage[ atom_ix[j] ] = true;
                }
            }
            break;

            case FunavOp:
            // this argument is a variable
            CPPAD_ASSERT_UNKNOWN( atom_state == arg_atom );
            atom_ix[atom_j]     = 0;
            parameter_x[atom_j] = all_par_vec[0]; // variables get value nan
            type_x[atom_j]      = variable_enum;
            ++atom_j;
            if( atom_j == atom_n )
                atom_state = ret_atom;
            break;

            case FunapOp:
            // this argument is a parameter
            CPPAD_ASSERT_UNKNOWN( atom_state == arg_atom );
            atom_ix[atom_j]     = size_t( arg[0] );
            parameter_x[atom_j] = all_par_vec[arg[0]]; // parameter value
            if( dyn_par_is[arg[0]] )
                    type_x[atom_j] = dynamic_enum;
            else
                    type_x[atom_j] = dynamic_enum;
            ++atom_j;
            if( atom_j == atom_n )
                atom_state = ret_atom;
            break;

            case FunrpOp:
            // this result is a parameter
            CPPAD_ASSERT_UNKNOWN( atom_state == ret_atom );
            depend_y[atom_i] = op_usage[i_op] != usage_t(no_usage);
            ++atom_i;
            if( atom_i == atom_m )
                atom_state = end_atom;
            break;

            case FunrvOp:
            // this result is a variable
            CPPAD_ASSERT_UNKNOWN( atom_state == ret_atom );
            depend_y[atom_i] = op_usage[i_op] != usage_t(no_usage);
            ++atom_i;
            if( atom_i == atom_m )
                atom_state = end_atom;
            break;
            // --------------------------------------------------------------


            default:
            CPPAD_ASSERT_UNKNOWN(false);
        }
    }
    // -----------------------------------------------------------------------
    // reverse pass to determine which dynamic parameters are necessary
    // -----------------------------------------------------------------------
    size_t i_arg = dyn_par_arg.size(); // index in dyn_par_arg
    size_t i_dyn = num_dynamic_par;    // index in dyn_ind2par_ind
    while(i_dyn)
    {   // next dynamic parameter in reverse order
        --i_dyn;
        op_code_dyn op = op_code_dyn( dyn_par_op[i_dyn] );
        while( op == result_dyn )
        {   --i_dyn;
            op = op_code_dyn( dyn_par_op[i_dyn] );
            CPPAD_ASSERT_UNKNOWN( op == result_dyn || op == atom_dyn );
        }
        if( op == atom_dyn )
        {   // number of arguments for this operator
            size_t n_arg = size_t( dyn_par_arg[i_arg - 1] );
            //
            // index of first argument for this operation
            i_arg -= n_arg;
            //
            atom_index = size_t( dyn_par_arg[i_arg + 0] );
            size_t n          = size_t( dyn_par_arg[i_arg + 1] );
            size_t m          = size_t( dyn_par_arg[i_arg + 2] );
            CPPAD_ASSERT_UNKNOWN( n_arg == 5 + n + m );
            //
            // parameter_x, type_x
            parameter_x.resize(n);
            type_x.resize(n);
            for(size_t j = 0; j < n; ++j)
            {   // parameter index zero is used for variable
                CPPAD_ASSERT_UNKNOWN( isnan( all_par_vec[0] ) );
                addr_t arg_j = dyn_par_arg[i_arg + 4 + j];
                parameter_x[j] = all_par_vec[arg_j];
                if( arg_j == 0 )
                    type_x[j] = variable_enum;
                else if( dyn_par_is[arg_j] )
                    type_x[j] = dynamic_enum;
                else
                    type_x[j] = constant_enum;
            }
            //
            // depend_y
            depend_y.resize(m);
            for(size_t i = 0; i < m; ++i)
            {   // a constant prameter cannot depend on a dynamic parameter
                // so do not worry about constant parameters in depend_y
                size_t i_par = size_t( dyn_par_arg[i_arg + 4 + n + i] );
                depend_y[i]  = par_usage[i_par];
            }
            //
            // call back to atomic function for this operation
            depend_x.resize(n);
            atom_old = 0; // not used with dynamic parameters
            sweep::call_atomic_rev_depend<Base, Base>(
                atom_index, atom_old, parameter_x, type_x, depend_x, depend_y
            );
            //
            // transfer depend_x to par_usage
            for(size_t j = 0; j < n; ++j)
            {   size_t i_par = size_t( dyn_par_arg[i_arg + 4 + j] );
                par_usage[i_par] = par_usage[i_par] | depend_x[j];
            }
        }
        else
        {   // corresponding parameter index
            size_t i_par = size_t( dyn_ind2par_ind[i_dyn] );
            CPPAD_ASSERT_UNKNOWN( dyn_par_is[i_par] );
            //
            // number of argumens to this operator
            size_t n_arg = num_arg_dyn(op);
            //
            // index of first argument for this operator
            CPPAD_ASSERT_UNKNOWN( op != atom_dyn );
            i_arg -= n_arg;
            //
            // if this dynamic parameter is needed
            if( par_usage[i_par] )
            {   // need dynamic parameters that are used to generate this one
                size_t offset = num_non_par_arg_dyn(op);
                for(size_t i = offset; i < n_arg; ++i)
                    par_usage[ dyn_par_arg[i_arg + i] ] = true;
            }
        }
    }
    CPPAD_ASSERT_UNKNOWN( i_arg == 0 );
    //
    return;
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
