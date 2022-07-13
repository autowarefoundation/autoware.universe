# ifndef CPPAD_LOCAL_OPTIMIZE_GET_OP_USAGE_HPP
# define CPPAD_LOCAL_OPTIMIZE_GET_OP_USAGE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/optimize/cexp_info.hpp>
# include <cppad/local/optimize/usage.hpp>
# include <cppad/local/sweep/call_atomic.hpp>

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize {

/// Is this an addition or subtraction operator
inline bool op_add_or_sub(
    OpCode op ///< operator we are checking
)
{   bool result;
    switch(op)
    {
        case AddpvOp:
        case AddvvOp:
        case SubpvOp:
        case SubvpOp:
        case SubvvOp:
        result = true;
        break;

        default:
        result = false;
        break;
    }
    return result;
}

/*!
$begin optimize_op_inc_arg_usage$$
$spell
    cexp
    op
    arg
    csum
    optimizer
$$

$section
Increase Argument Usage and Propagate cexp_set From Result to Argument
$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_OP_INC_ARG_USAGE%// END_PROTOTYPE%1
%$$


$head play$$
is the player for the old operation sequence.

$head check_csum$$
is result an addition or subtraction operator,
and the optimizer is allowed to generate cumulative sum operators.

$head i_result$$
is the operator index for the result operator.
There are no posting waiting to be processed for the corresponding cexp_set.

$head i_arg$$
is the operator index for the argument to the result operator.
There may be postings waiting to be processed for the corresponding cexp_set.

$head op_usage$$
structure that holds the information for each of the operators.
The output value of op_usage[i_arg] is increased; to be specific,
If check_csum is true and the input value of op_usage[i_arg]
is usage_t(no_usage), its output value is usage_t(csum_usage).
Otherwise, the output value of op_usage[i_arg] is usage_t(yes_usage).

$head cexp_set$$
This is a vector of sets with one set for each operator.
We denote the i-th set by set[i].
These are the conditional expression conditions that must be
satisfied for this argument to be used.

$list number$$
In the special case where cexp_set.n_set() is zero,
cexp_set is not changed.
$lnext
If cexp_set.n_set() != 0 and op_usage[i_arg] == usage_t(no_usage),
the input value of set[i_arg] must be empty.
In this case the output value if set[i_arg] is equal to set[i_result]
(which may also be empty).
$lnext
If cexp_set.n_set() != 0 and op_usage[i_arg] != usage_t(no_usage),
the output value of set[i_arg] is the intersection of
its input value and set[i_result].
$lend

$end
*/
// BEGIN_OP_INC_ARG_USAGE
template <class Base>
void op_inc_arg_usage(
    const player<Base>*         play           ,
    bool                        check_csum     ,
    size_t                      i_result       ,
    size_t                      i_arg          ,
    pod_vector<usage_t>&        op_usage       ,
    sparse::list_setvec&        cexp_set       )
// END_PROTOTYPE
{   // value of argument input on input to this routine
    enum_usage arg_usage = enum_usage( op_usage[i_arg] );
    //
    // new value for usage
    op_usage[i_arg] = usage_t(yes_usage);
    if( check_csum )
    {   if( arg_usage == no_usage )
        {   OpCode op_a = play->GetOp(i_arg);
            if( op_add_or_sub( op_a ) )
            {   op_usage[i_arg] = usage_t(csum_usage);
            }
        }
    }
    //
    // cexp_set
    if( cexp_set.n_set() == 0 )
        return;
    //
    if( arg_usage == no_usage )
    {   // set[i_arg] = set[i_result]
        // not necessary to process posts for set i_result
        cexp_set.assignment(i_arg, i_result, cexp_set);
    }
    else
    {   // set[i_arg] = set[i_arg] intersect set[i_result]
        // is necessary to process postts for set i_arg
        cexp_set.process_post(i_arg);
        cexp_set.binary_intersection(i_arg, i_arg, i_result, cexp_set);
    }
    //
    return;
}

/*!
$begin optimize_get_op_usage$$
$spell
    Ind
    var
    itr
    dep_taddr
    cexp
    vecad
    Addr
    iterator
    NumRes
    PriOp
    Exp
    bool
    Vec
$$

$section
Use Reverse Activity Analysis to Get Usage Information for Each Operator
$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_GET_OP_USAGE%// END_PROTOTYPE%1
%$$

$head Base$$
Base type for the operator; i.e., this operation was recorded
using $codei%AD<%Base%>%$$ and computations by this routine are done
using type $icode Base$$.

$head Addr$$
Type used by random iterator for the player.

$head cumulative_sum_op$$
If this is true (false), cumulative summation operator are allowed
(not allowed) to be generated by the optimization.

$head compare_op$$
if this is true, arguments are considered used if they appear in compare
operators. This is a side effect because compare operators have boolean
results (and the result is not in the tape; i.e. NumRes(op) is zero
for these operators. (This is an example of a side effect.)

$head print_for_op$$
if this is true, arguments are considered used if they appear in
print forward operators; i.e., PriOp.
This is also a side effect; i.e. NumRes(PriOp) is zero.

$head conditional_skip$$
If this is true,
the conditional expression information cexp_info will be calculated.
This may be time intensive and may not have much benefit in the optimized
recording.


$head play$$
This is the operation sequence.

$head random_itr$$
This is a random iterator for the operation sequence.

$head dep_taddr$$
is a vector of indices for the dependent variables
(where the reverse activity analysis starts).

$head cexp2op$$
The input size of this vector must be zero.
Upon return it has size equal to the number of conditional expressions,
CExpOp operators. The value $icode%cexp2op[%j%]%$$ is the operator
index corresponding to the $th j$$ conditional expressions.

$head cexp_set$$
This is a vector of sets that is empty on input.
If $icode conditional_skip$$ is false, $icode cexp_usage$$ is not modified.
Otherwise, set[i] is a set of elements for the i-th operator.
Suppose that e is an element of set[i], j = e / 2, k = e % 2.
If the comparison for the j-th conditional expression is equal to bool(k),
the i-th operator can be skipped (is not used by any of the results).
Note that j indexes the CExpOp operators in the operation sequence.

$head vecad_used$$
The input size of this vector must be zero.
Upon return it has size equal to the number of VecAD vectors
in the operations sequences; i.e., play->num_var_vecad_rec().
The VecAD vectors are indexed in the order that their indices appear
in the one large play->GetVecInd that holds all the VecAD vectors.

$head op_usage$$
The input size of this vector must be zero.
Upon return it has size equal to the number of operators
in the operation sequence; i.e., num_op = play->nun_var_rec().
The value $icode%op_usage%[%i%]%$$ has been set to the usage for
the i-th operator in the operation sequence.
Atomic function calls are a special case,
the first and second AFunOp have usage corresponding to the entire call.
The arguments have the usage for particular parameter or variable.
This usage is only for creating variables, not for creating
dynamic parameters.

$end
*/
// BEGIN_GET_OP_USAGE
template <class Addr, class Base>
void get_op_usage(
    bool                                        conditional_skip    ,
    bool                                        compare_op          ,
    bool                                        print_for_op        ,
    bool                                        cumulative_sum_op   ,
    const player<Base>*                         play                ,
    const play::const_random_iterator<Addr>&    random_itr          ,
    const pod_vector<size_t>&                   dep_taddr           ,
    pod_vector<addr_t>&                         cexp2op             ,
    sparse::list_setvec&                        cexp_set            ,
    pod_vector<bool>&                           vecad_used          ,
    pod_vector<usage_t>&                        op_usage            )
// END_PROTOTYPE
{
    CPPAD_ASSERT_UNKNOWN( cexp_set.n_set()  == 0 );
    CPPAD_ASSERT_UNKNOWN( vecad_used.size() == 0 );
    CPPAD_ASSERT_UNKNOWN( op_usage.size()   == 0 );

    // number of operators in the tape
    const size_t num_op = play->num_op_rec();
    //
    // initialize mapping from variable index to operator index
    CPPAD_ASSERT_UNKNOWN(
        size_t( (std::numeric_limits<addr_t>::max)() ) >= num_op
    );
    // -----------------------------------------------------------------------
    // information about current operator
    OpCode        op;     // operator
    const addr_t* arg;    // arguments
    size_t        i_op;   // operator index
    size_t        i_var;  // variable index of first result
    // -----------------------------------------------------------------------
    // information about atomic function calls
    size_t atom_index=0, atom_old=0, atom_m=0, atom_n=0, atom_i=0, atom_j=0;
    enum_atom_state atom_state;
    //
    // work space used by user atomic functions
    vector<Base>         atom_x;    // value of parameters in x
    vector<ad_type_enum> type_x;    // type for each argument
    vector<size_t>       atom_ix;   // variables indices for argument vector
    vector<bool>         depend_y;  // results that are used
    vector<bool>         depend_x;  // arguments that are used
    //
    // parameter information (used by atomic function calls)
# ifndef NDEBUG
    size_t num_par = play->num_par_rec();
# endif
    CPPAD_ASSERT_UNKNOWN( num_par > 0 )
    const Base* parameter = play->GetPar();
    // -----------------------------------------------------------------------
    // vecad information
    size_t num_vecad      = play->num_var_vecad_rec();
    size_t num_vecad_ind  = play->num_var_vecad_ind_rec();
    //
    vecad_used.resize(num_vecad);
    for(size_t i = 0; i < num_vecad; i++)
        vecad_used[i] = false;
    //
    vector<size_t> arg2vecad(num_vecad_ind);
    for(size_t i = 0; i < num_vecad_ind; i++)
        arg2vecad[i] = num_vecad; // invalid value
    size_t arg_0 = 1; // value of arg[0] for theh first vecad
    for(size_t i = 0; i < num_vecad; i++)
    {
        // mapping from arg[0] value to index for this vecad object.
        arg2vecad[arg_0] = i;
        //
        // length of this vecad object
        size_t length = play->GetVecInd(arg_0 - 1);
        //
        // set to proper index in GetVecInd for next VecAD arg[0] value
        arg_0        += length + 1;
    }
    CPPAD_ASSERT_UNKNOWN( arg_0 == num_vecad_ind + 1 );
    // -----------------------------------------------------------------------
    // conditional expression information
    //
    size_t num_cexp_op = 0;
    if( conditional_skip )
    {   for(i_op = 0; i_op < num_op; ++i_op)
        {   if( random_itr.get_op(i_op) == CExpOp )
            {   // count the number of conditional expressions.
                ++num_cexp_op;
            }
        }
    }
    //
    cexp2op.resize( num_cexp_op );
    //
    // number of sets
    size_t num_set = 0;
    if( conditional_skip && num_cexp_op > 0)
        num_set = num_op;
    //
    // conditional expression index   = element / 2
    // conditional expression compare = bool ( element % 2)
    size_t end_set = 2 * num_cexp_op;
    //
    if( num_set > 0 )
        cexp_set.resize(num_set, end_set);
    // -----------------------------------------------------------------------
    // initilaize operator usage for reverse dependency analysis.
    op_usage.resize( num_op );
    for(i_op = 0; i_op < num_op; ++i_op)
        op_usage[i_op] = usage_t(no_usage);
    for(size_t i = 0; i < dep_taddr.size(); i++)
    {   i_op           = random_itr.var2op(dep_taddr[i]);
        op_usage[i_op] = usage_t(yes_usage);    // dependent variables
    }
    // ----------------------------------------------------------------------
    // Reverse pass to compute usage and cexp_set for each operator
    // ----------------------------------------------------------------------
    //
    // Initialize reverse pass
    size_t last_atom_i_op = 0;
    size_t cexp_index     = num_cexp_op;
    atom_state            = end_atom;
    i_op = num_op;
    while(i_op != 0 )
    {   --i_op;
        if( num_set > 0 )
        {   // no more elements will be added to this set
            cexp_set.process_post(i_op);
        }
        //
        // this operator information
        random_itr.op_info(i_op, op, arg, i_var);
        //
        // Is the result of this operation used.
        // (This only makes sense when NumRes(op) > 0.)
        usage_t use_result = op_usage[i_op];
        //
        bool check_csum = false;
        switch( op )
        {
            // =============================================================
            // normal operators
            // =============================================================

            // Only one variable with index arg[0]
            case SubvpOp:
            check_csum = cumulative_sum_op;
            //
            case AbsOp:
            case AcosOp:
            case AcoshOp:
            case AsinOp:
            case AsinhOp:
            case AtanOp:
            case AtanhOp:
            case CosOp:
            case CoshOp:
            case DivvpOp:
            case ErfOp:
            case ErfcOp:
            case ExpOp:
            case Expm1Op:
            case LogOp:
            case Log1pOp:
            case NegOp:
            case PowvpOp:
            case SignOp:
            case SinOp:
            case SinhOp:
            case SqrtOp:
            case TanOp:
            case TanhOp:
            case ZmulvpOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0 );
            if( use_result != usage_t(no_usage) )
            {   size_t j_op = random_itr.var2op(size_t(arg[0]));
                op_inc_arg_usage(
                    play, check_csum, i_op, j_op, op_usage, cexp_set
                );
            }
            break; // --------------------------------------------

            // Only one variable with index arg[1]
            case AddpvOp:
            case SubpvOp:
            check_csum = cumulative_sum_op;
            //
            case DisOp:
            case DivpvOp:
            case MulpvOp:
            case PowpvOp:
            case ZmulpvOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0 );
            if( use_result != usage_t(no_usage) )
            {   size_t j_op = random_itr.var2op(size_t(arg[1]));
                op_inc_arg_usage(
                    play, check_csum, i_op, j_op, op_usage, cexp_set
                );
            }
            break; // --------------------------------------------

            // arg[0] and arg[1] are the only variables
            case AddvvOp:
            case SubvvOp:
            check_csum = cumulative_sum_op;
            //
            case DivvvOp:
            case MulvvOp:
            case PowvvOp:
            case ZmulvvOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0 );
            if( use_result != usage_t(no_usage) )
            {   for(size_t i = 0; i < 2; i++)
                {   size_t j_op = random_itr.var2op(size_t(arg[i]));
                    op_inc_arg_usage(
                        play, check_csum, i_op, j_op, op_usage, cexp_set
                    );
                }
            }
            break; // --------------------------------------------

            // Conditional expression operators
            // arg[2], arg[3], arg[4], arg[5] are parameters or variables
            case CExpOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0 );
            if( conditional_skip )
            {   --cexp_index;
                cexp2op[ cexp_index ] = addr_t(i_op);
            }
            if( use_result != usage_t(no_usage) )
            {   CPPAD_ASSERT_UNKNOWN( NumArg(CExpOp) == 6 );
                // propgate from result to left argument
                if( arg[1] & 1 )
                {   size_t j_op = random_itr.var2op(size_t(arg[2]));
                    op_inc_arg_usage(
                        play, check_csum, i_op, j_op, op_usage, cexp_set
                    );
                }
                // propgate from result to right argument
                if( arg[1] & 2 )
                {   size_t j_op = random_itr.var2op(size_t(arg[3]));
                    op_inc_arg_usage(
                            play, check_csum, i_op, j_op, op_usage, cexp_set
                    );
                }
                // are if_true and if_false cases the same variable
                bool same_variable = (arg[1] & 4) != 0;
                same_variable     &= (arg[1] & 8) != 0;
                same_variable     &= arg[4] == arg[5];
                //
                // if_true
                if( arg[1] & 4 )
                {   size_t j_op = random_itr.var2op(size_t(arg[4]));
                    bool can_skip = conditional_skip & (! same_variable);
                    can_skip     &= op_usage[j_op] == usage_t(no_usage);
                    op_inc_arg_usage(
                        play, check_csum, i_op, j_op, op_usage, cexp_set
                    );
                    if( can_skip )
                    {   // j_op corresponds to the value used when the
                        // comparison result is true. It can be skipped when
                        // the comparison is false (0).
                        size_t element = 2 * cexp_index + 0;
                        cexp_set.post_element(j_op, element);
                        //
                        op_usage[j_op] = usage_t(yes_usage);
                    }
                }
                //
                // if_false
                if( arg[1] & 8 )
                {   size_t j_op = random_itr.var2op(size_t(arg[5]));
                    bool can_skip = conditional_skip & (! same_variable);
                    can_skip     &= op_usage[j_op] == usage_t(no_usage);
                    op_inc_arg_usage(
                        play, check_csum, i_op, j_op, op_usage, cexp_set
                    );
                    if( can_skip )
                    {   // j_op corresponds to the value used when the
                        // comparison result is false. It can be skipped when
                        // the comparison is true (0).
                        size_t element = 2 * cexp_index + 1;
                        cexp_set.post_element(j_op, element);
                        //
                        op_usage[j_op] = usage_t(yes_usage);
                    }
                }
            }
            break;  // --------------------------------------------

            // Operations that are never used
            // (new CSkip options are generated if conditional_skip is true)
            case CSkipOp:
            case ParOp:
            break;

            // Operators that are always used
            case InvOp:
            case BeginOp:
            case EndOp:
            op_usage[i_op] = usage_t(yes_usage);
            break;  // -----------------------------------------------

            // The print forward operator
            case PriOp:
            CPPAD_ASSERT_NARG_NRES(op, 5, 0);
            if( print_for_op )
            {   op_usage[i_op] = usage_t(yes_usage);
                if( arg[0] & 1 )
                {   // arg[1] is a variable
                    size_t j_op = random_itr.var2op(size_t(arg[1]));
                    op_inc_arg_usage(
                        play, check_csum, i_op, j_op, op_usage, cexp_set
                    );
                }
                if( arg[0] & 2 )
                {   // arg[3] is a variable
                    size_t j_op = random_itr.var2op(size_t(arg[3]));
                    op_inc_arg_usage(
                        play, check_csum, i_op, j_op, op_usage, cexp_set
                    );
                }
            }
            break; // -----------------------------------------------------

            // =============================================================
            // Comparison operators
            // =============================================================

            // Compare operators where arg[1] is only variable
            case LepvOp:
            case LtpvOp:
            case EqpvOp:
            case NepvOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
            if( compare_op )
            {   op_usage[i_op] = usage_t(yes_usage);
                //
                size_t j_op = random_itr.var2op(size_t(arg[1]));
                op_inc_arg_usage(
                    play, check_csum, i_op, j_op, op_usage, cexp_set
                );
            }
            break; // ----------------------------------------------

            // Compare operators where arg[0] is only variable
            case LevpOp:
            case LtvpOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
            if( compare_op )
            {   op_usage[i_op] = usage_t(yes_usage);
                //
                size_t j_op = random_itr.var2op(size_t(arg[0]));
                op_inc_arg_usage(
                    play, check_csum, i_op, j_op, op_usage, cexp_set
                );
            }
            break; // ----------------------------------------------

            // Compare operators where arg[0] and arg[1] are variables
            case LevvOp:
            case LtvvOp:
            case EqvvOp:
            case NevvOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
            if( compare_op )
            {   op_usage[i_op] = usage_t(yes_usage);
                //
                for(size_t i = 0; i < 2; i++)
                {   size_t j_op = random_itr.var2op(size_t(arg[i]));
                    op_inc_arg_usage(
                        play, check_csum, i_op, j_op, op_usage, cexp_set
                    );
                }
            }
            break; // ----------------------------------------------

            // =============================================================
            // VecAD operators
            // =============================================================

            // load operator using a parameter index
            case LdpOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0 );
            if( use_result != usage_t(no_usage) )
            {   size_t i_vec = arg2vecad[ arg[0] ];
                vecad_used[i_vec] = true;
            }
            break; // --------------------------------------------

            // load operator using a variable index
            case LdvOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) > 0 );
            if( use_result != usage_t(no_usage) )
            {   size_t i_vec = arg2vecad[ arg[0] ];
                vecad_used[i_vec] = true;
                //
                size_t j_op = random_itr.var2op(size_t(arg[1]));
                op_usage[j_op] = usage_t(yes_usage);
            }
            break; // --------------------------------------------

            // Store a variable using a parameter index
            case StpvOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
            if( vecad_used[ arg2vecad[ arg[0] ] ] )
            {   op_usage[i_op] = usage_t(yes_usage);
                //
                size_t j_op = random_itr.var2op(size_t(arg[2]));
                op_usage[j_op] = usage_t(yes_usage);
            }
            break; // --------------------------------------------

            // Store a variable using a variable index
            case StvvOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
            if( vecad_used[ arg2vecad[ arg[0] ] ] )
            {   op_usage[i_op] = usage_t(yes_usage);
                //
                size_t j_op = random_itr.var2op(size_t(arg[1]));
                op_usage[j_op] = usage_t(yes_usage);
                size_t k_op = random_itr.var2op(size_t(arg[2]));
                op_usage[k_op] = usage_t(yes_usage);
            }
            break; // -----------------------------------------------------

            // =============================================================
            // cumulative summation operator
            // ============================================================
            case CSumOp:
            CPPAD_ASSERT_UNKNOWN( NumRes(op) == 1 );
            {
                for(size_t i = 5; i < size_t(arg[2]); i++)
                {   size_t j_op = random_itr.var2op(size_t(arg[i]));
                    op_inc_arg_usage(
                        play, check_csum, i_op, j_op, op_usage, cexp_set
                    );
                }
            }
            break;

            // =============================================================
            // user defined atomic operators
            // ============================================================

            case AFunOp:
            // start or end atomic operation sequence
            if( atom_state == end_atom )
            {   // reverse_user using random_itr instead of play
                atom_index        = size_t(arg[0]);
                atom_old          = size_t(arg[1]);
                atom_n            = size_t(arg[2]);
                atom_m            = size_t(arg[3]);
                atom_j            = atom_n;
                atom_i            = atom_m;
                atom_state        = ret_atom;
                // -------------------------------------------------------
                last_atom_i_op = i_op;
                CPPAD_ASSERT_UNKNOWN( i_op > atom_n + atom_m + 1 );
                CPPAD_ASSERT_UNKNOWN(
                    op_usage[last_atom_i_op] == usage_t(no_usage)
                );
# ifndef NDEBUG
                if( cexp_set.n_set() > 0 )
                {   cexp_set.process_post(last_atom_i_op);
                    CPPAD_ASSERT_UNKNOWN(
                        cexp_set.number_elements(last_atom_i_op) == 0
                    );
                }
# endif
                //
                atom_x.resize(  atom_n );
                type_x.resize( atom_n );
                atom_ix.resize( atom_n );
                //
                depend_y.resize( atom_m );
                depend_x.resize( atom_n );
                for(size_t i = 0; i < atom_m; i++)
                    depend_y[ i ] = false;
            }
            else
            {   // reverse_user using random_itr instead of play
                CPPAD_ASSERT_UNKNOWN( atom_state == start_atom );
                CPPAD_ASSERT_UNKNOWN( atom_n == size_t(arg[2]) );
                CPPAD_ASSERT_UNKNOWN( atom_m == size_t(arg[3]) );
                CPPAD_ASSERT_UNKNOWN( atom_j == 0 );
                CPPAD_ASSERT_UNKNOWN( atom_i == 0 );
                atom_state = end_atom;
                // -------------------------------------------------------
                CPPAD_ASSERT_UNKNOWN(
                    i_op + atom_n + atom_m + 1 == last_atom_i_op
                );
                if( op_usage[last_atom_i_op] != usage_t(no_usage) )
                {   // call atomic function for this operation
                    sweep::call_atomic_rev_depend<Base, Base>(
                    atom_index, atom_old, atom_x, type_x, depend_x, depend_y
                    );
                    for(size_t j = 0; j < atom_n; j++)
                    if( depend_x[j] )
                    {   // The parameter or variable correspnding to the j-th
                        // argument gets used
                        op_usage[i_op + 1 + j] = true;
                        if( type_x[j] == variable_enum )
                        {   CPPAD_ASSERT_UNKNOWN( atom_ix[j] > 0 );
                            if( depend_x[j] )
                            {   size_t j_op = random_itr.var2op(atom_ix[j]);
                                op_inc_arg_usage(play, check_csum,
                                    last_atom_i_op, j_op, op_usage, cexp_set
                                );
                            }
                        }
                    }
                }
                // copy set infomation from last to first
                if( cexp_set.n_set() > 0 )
                {   cexp_set.process_post(last_atom_i_op);
                    cexp_set.assignment(i_op, last_atom_i_op, cexp_set);
                }
                // copy usage information from last to first
                op_usage[i_op] = op_usage[last_atom_i_op];
            }
            break; // -------------------------------------------------------

            case FunapOp:
            // parameter argument in an atomic operation sequence
            CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
            //
            // reverse_user using random_itr instead of play
            CPPAD_ASSERT_NARG_NRES(op, 1, 0);
            CPPAD_ASSERT_UNKNOWN( 0 < atom_j && atom_j <= atom_n );
            --atom_j;
            if( atom_j == 0 )
                atom_state = start_atom;
            // -------------------------------------------------------------
            atom_ix[atom_j] = 0;
            //
            // parameter arguments
            atom_x[atom_j] = parameter[arg[0]];
            if( play->dyn_par_is()[arg[0]] )
                type_x[atom_j] = dynamic_enum;
            else
                type_x[atom_j] = constant_enum;
            //
            break;

            case FunavOp:
            // variable argument in an atomic operation sequence
            CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
            //
            // reverse_user using random_itr instead of play
            CPPAD_ASSERT_NARG_NRES(op, 1, 0);
            CPPAD_ASSERT_UNKNOWN( 0 < atom_j && atom_j <= atom_n );
            --atom_j;
            if( atom_j == 0 )
                atom_state = start_atom;
            // -------------------------------------------------------------
            atom_ix[atom_j] = size_t(arg[0]);
            //
            // variable arguments as parameters
            atom_x[atom_j] = CppAD::numeric_limits<Base>::quiet_NaN();
            type_x[atom_j] = variable_enum;
            //
            break;

            case FunrvOp:
            // variable result in an atomic operation sequence
            //
            // reverse_user using random_itr instead of play
            CPPAD_ASSERT_NARG_NRES(op, 0, 1);
            CPPAD_ASSERT_UNKNOWN( 0 < atom_i && atom_i <= atom_m );
            --atom_i;
            if( atom_i == 0 )
                atom_state = arg_atom;
            // -------------------------------------------------------------
            if( use_result )
            {   depend_y[atom_i] = true;
                op_inc_arg_usage(
                    play, check_csum, i_op, last_atom_i_op, op_usage, cexp_set
                );
            }
            break; // --------------------------------------------------------

            case FunrpOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
            //
            // reverse_user using random_itr instead of play
            CPPAD_ASSERT_NARG_NRES(op, 1, 0);
            CPPAD_ASSERT_UNKNOWN( 0 < atom_i && atom_i <= atom_m );
            --atom_i;
            if( atom_i == 0 )
                atom_state = arg_atom;
            break;
            // ============================================================

            // all cases should be handled above
            default:
            CPPAD_ASSERT_UNKNOWN(0);
        }
    }
    return;
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
