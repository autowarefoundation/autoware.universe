# ifndef CPPAD_LOCAL_OPTIMIZE_GET_OP_PREVIOUS_HPP
# define CPPAD_LOCAL_OPTIMIZE_GET_OP_PREVIOUS_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/optimize/match_op.hpp>
# include <cppad/local/optimize/usage.hpp>

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize {
/*
$begin optimize_get_op_previous$$
$spell
    itr
    iterator
    bool
    Exp
    num
    var
    Op
    cexp
    Arg
    Res
$$

$section Get Mapping From Op to Previous Op That is Equivalent$$

$head Syntax$$
$icode%exceed_collision_limit% = get_op_previous(
    %collision_limit%,
    %play%,
    %random_itr%,
    %cexp_set%,
    %op_previous%,
    %op_usage%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
%$$

$head Base$$
base type for the operator; i.e., this operation was recorded
using AD<Base> and computations by this routine are done using type Base.

$head collision_limit$$
is the maximum number of collisions (matches)
allowed in the hash expression has table.

$head play$$
is the old operation sequence.

$head random_itr$$
is a random iterator for the old operation sequence.

$head cexp_set$$
set[i] is a set of elements for the i-th operator.
Suppose that e is an element of set[i], j = e / 2, k = e % 2.
If the comparison for the j-th conditional expression is equal to bool(k),
the i-th operator can be skipped (is not used by any of the results).
Note the j indexes the CExpOp operators in the operation sequence.
On input, cexp_set is does not count previous optimization.
On output, it does count previous optimization.

$head op_previous$$
The input size of this vector must be zero.
Upon return it has size equal to the number of operators
in the operation sequence; i.e., num_op = play->nun_var_rec().
Let j = op_previous[i]. If j = 0, no replacement was found for i-th operator.
If j != 0:
$list number$$
j < i
$lnext
op_previous[j] == 0
$lnext
op_usage[j] == usage_t(yes_usage)
$lnext
i-th operator has NumArg(op) <= 3
$lnext
i-th operator has 0 < NumRes(op)
$lnext
i-th operator is not one of the following:
$nospell PriOp, ParOp, InvOp, EndOp, CexpOp, BeginOp.$$
$lnext
i-th operator is not one of the load store operator:
$nospell LtpvOp, LtvpOp, LtvvOp, StppOp, StpvOp, StvpOp, StvvOp.$$
$lnext
i-th operator is not a atomic function operator:
$nospell AFunOp, FunapOp, FunavOp, FunrpOp, FunrvOp.$$
$lend

$head op_usage$$
The size of this vector is the number of operators in the
old operation sequence.i.e., play->nun_var_rec().
On input, op_usage[i] is the usage for
the i-th operator in the operation sequence not counting previous
optimization.
On output, it is the usage counting previous operator optimization.

$head exceed_collision_limit$$
If the $icode collision_limit$$ is exceeded (is not exceeded),
the return value is true (false).

$end
*/

// BEGIN_PROTOTYPE
template <class Addr, class Base>
bool get_op_previous(
    size_t                                      collision_limit     ,
    const player<Base>*                         play                ,
    const play::const_random_iterator<Addr>&    random_itr          ,
    sparse::list_setvec&                        cexp_set            ,
    pod_vector<addr_t>&                         op_previous         ,
    pod_vector<usage_t>&                        op_usage            )
// END_PROTOTYPE
{   bool exceed_collision_limit = false;
    //
    // number of operators in the tape
    const size_t num_op = random_itr.num_op();
    CPPAD_ASSERT_UNKNOWN( op_previous.size() == 0 );
    CPPAD_ASSERT_UNKNOWN( op_usage.size() == num_op );
    op_previous.resize( num_op );
    //
    // number of conditional expressions in the tape
    //
    // initialize mapping from variable index to operator index
    CPPAD_ASSERT_UNKNOWN(
        size_t( (std::numeric_limits<addr_t>::max)() ) >= num_op
    );
    // ----------------------------------------------------------------------
    // compute op_previous
    // ----------------------------------------------------------------------
    sparse::list_setvec  hash_table_op;
    hash_table_op.resize(CPPAD_HASH_TABLE_SIZE, num_op);
    //
    pod_vector<bool> work_bool;
    pod_vector<addr_t> work_addr_t;
    for(size_t i_op = 0; i_op < num_op; ++i_op)
    {   op_previous[i_op] = 0;

        if( op_usage[i_op] == usage_t(yes_usage) )
        switch( random_itr.get_op(i_op) )
        {
            // ----------------------------------------------------------------
            // these operators never match pevious operators
            case BeginOp:
            case CExpOp:
            case CSkipOp:
            case CSumOp:
            case EndOp:
            case InvOp:
            case LdpOp:
            case LdvOp:
            case ParOp:
            case PriOp:
            case StppOp:
            case StpvOp:
            case StvpOp:
            case StvvOp:
            case AFunOp:
            case FunapOp:
            case FunavOp:
            case FunrpOp:
            case FunrvOp:
            break;

            // ----------------------------------------------------------------
            // check for a previous match
            case AbsOp:
            case AcosOp:
            case AcoshOp:
            case AddpvOp:
            case AddvvOp:
            case AsinOp:
            case AsinhOp:
            case AtanOp:
            case AtanhOp:
            case CosOp:
            case CoshOp:
            case DisOp:
            case DivpvOp:
            case DivvpOp:
            case DivvvOp:
            case EqpvOp:
            case EqvvOp:
            case ErfOp:
            case ErfcOp:
            case ExpOp:
            case Expm1Op:
            case LepvOp:
            case LevpOp:
            case LevvOp:
            case LogOp:
            case Log1pOp:
            case LtpvOp:
            case LtvpOp:
            case LtvvOp:
            case MulpvOp:
            case MulvvOp:
            case NegOp:
            case NepvOp:
            case NevvOp:
            case PowpvOp:
            case PowvpOp:
            case PowvvOp:
            case SignOp:
            case SinOp:
            case SinhOp:
            case SqrtOp:
            case SubpvOp:
            case SubvpOp:
            case SubvvOp:
            case TanOp:
            case TanhOp:
            case ZmulpvOp:
            case ZmulvpOp:
            case ZmulvvOp:
            exceed_collision_limit |= match_op(
                collision_limit,
                random_itr,
                op_previous,
                i_op,
                hash_table_op,
                work_bool,
                work_addr_t
            );
            if( op_previous[i_op] != 0 )
            {   // like a unary operator that assigns i_op equal to previous.
                size_t previous = size_t( op_previous[i_op] );
                bool sum_op = false;
                CPPAD_ASSERT_UNKNOWN( previous < i_op );
                op_inc_arg_usage(
                    play, sum_op, i_op, previous, op_usage, cexp_set
                );
            }
            break;

            // ----------------------------------------------------------------
            default:
            CPPAD_ASSERT_UNKNOWN(false);
            break;
        }
    }
    /* ---------------------------------------------------------------------
    // Print out hash code usage summary
    CppAD::vector<size_t> count(collision_limit + 1);
    for(size_t i = 0; i <= collision_limit; ++i)
        count[i] = 0;
    for(size_t code = 0; code < CPPAD_HASH_TABLE_SIZE; ++code)
    {   size_t size = hash_table_op.number_elements(code);
        ++count[size];
    }
    std::cout << "count = " << count << "\n";
    --------------------------------------------------------------------- */
    return exceed_collision_limit;
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
