# ifndef CPPAD_LOCAL_OPTIMIZE_GET_CEXP_INFO_HPP
# define CPPAD_LOCAL_OPTIMIZE_GET_CEXP_INFO_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/optimize/match_op.hpp>
# include <cppad/local/optimize/cexp_info.hpp>
# include <cppad/local/optimize/usage.hpp>

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize {

/*!
$begin optimize_get_cexp_info.hpp$$
$spell
    cexp
    itr
    op
    iterator
    bool
    Exp
    deallocate
    Funap
    Funav
    Funrp
    Funrv
    num
    var
$$

$section Information for Each Conditional Expression$$

$head Syntax$$
$codei%get_cexp_info(
    %play%,
    %random_itr%,
    %op_previous%,
    %op_usage%,
    %cexp2op%,
    %cexp_set%,
    %cexp_info%,
    %skip_op_true%,
    %skip_op_false%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
%$$


$head Restrictions$$
Do not call this routine unless you are optimizing conditional expressions
and there are conditional expressions in the operation sequence.

$head Base$$
base type for the operator; i.e., this operation was recorded
using AD<Base> and computations by this routine are done using type Base.

$head play$$
This is the old operation sequence.

$head random_itr$$
This is a random iterator for the old operation sequence.

$head cexp2op$$
This is the number of conditional expressions in the operation sequence
and must be non-zero.

$head cexp_set$$
This is a vector of sets, the i-th set, set[i],
is a set of elements for the i-th operator.
If e is an element of set[i], let j = e / 2 and k = e % 2.
If the comparison for the j-th conditional expression is equal to bool(k),
the i-th operator can be skipped (is not used by any of the results).
Note that j indexes the subset of operators that are conditional expressions
in the old operation sequence.

$head cexp_info$$
The input size of this vector must be zero.
Upon return cexp_info has size equal to the number of conditional expressions
in the operation sequence; i.e., the number of CExpOp operators.
The value cexp_info[j] is the information corresponding to the j-th
conditional expression in the operation sequence.
This vector is in the same order as the operation sequence; i.e.
if j1 > j2, cexp_info[j1].i_op > cexp_info[j2].i_op.
Note that skip_op_true and skip_op_false could be part of this structure,
but then we would allocate and deallocate two vectors for each conditional
expression in the operation sequence.

$head skip_op_true$$
This vector of sets is empty on input.
Upon return, the j-th set is the operators that are not used when
comparison result for cexp_info[j] is true.
Note that FunapOp, FunavOp, FunrpOp, and FunrvOp, are not in this
set and should be skipped when the corresponding AFunOp are skipped.

$head skip_op_false$$
This vector of sets is empty on input.
Upon return, the j-th set is the operators that are not used when
comparison result for cexp_info[j] is false.
Note that FunapOp, FunavOp, FunrpOp, and FunrvOp, are not in this
set and should be skipped when the corresponding AFunOp are skipped.

$head op_previous$$
This argument has size equal to the number of operators
in the operation sequence; i.e., num_op = play->nun_var_rec().
If op_previous[i] == 0, no replacement was found for the i-th operator.
If op_previous[i] != 0, op_usage[ op_previous[i] ] == usage_t(yes_usage).

$head op_usage$$
This argument has size equal to the number of operators
in the operation sequence; i.e., num_op = play->nun_var_rec().
The value op_usage[i] is the usage for
the i-th operator in the operation sequence.

$end
*/

// BEGIN_PROTOTYPE
template <class Addr, class Base>
void get_cexp_info(
    const player<Base>*                         play                ,
    const play::const_random_iterator<Addr>&    random_itr          ,
    const pod_vector<addr_t>&                   op_previous         ,
    const pod_vector<usage_t>&                  op_usage            ,
    const pod_vector<addr_t>&                   cexp2op             ,
    const sparse::list_setvec&                  cexp_set            ,
    vector<struct_cexp_info>&                   cexp_info           ,
    sparse::list_setvec&                        skip_op_true        ,
    sparse::list_setvec&                        skip_op_false       )
// END_PROTOTYPE
{
    CPPAD_ASSERT_UNKNOWN( cexp_set.n_set() > 0  );
    CPPAD_ASSERT_UNKNOWN( cexp_info.size() == 0 );

    // number of operators in the tape
    const size_t num_op = play->num_op_rec();
    CPPAD_ASSERT_UNKNOWN( op_usage.size() == num_op );
    CPPAD_ASSERT_UNKNOWN( op_previous.size() == num_op );
    //
    // number of conditional expressions in the tape
    size_t num_cexp_op = cexp2op.size();
    //
    // initialize mapping from variable index to operator index
    CPPAD_ASSERT_UNKNOWN(
        size_t( (std::numeric_limits<addr_t>::max)() ) >= num_op
    );
    // ----------------------------------------------------------------------
    // compute cexp_info
    // ----------------------------------------------------------------------
    //
    // initialize information for each conditional expression
    cexp_info.resize(num_cexp_op);
    skip_op_true.resize(num_cexp_op, num_op);
    skip_op_false.resize(num_cexp_op, num_op);
    //
    for(size_t i = 0; i < num_cexp_op; i++)
    {   size_t i_op = size_t( cexp2op[i] );
        CPPAD_ASSERT_UNKNOWN(
            op_previous[i_op] == 0 || op_usage[i_op] == usage_t(yes_usage)
        );
        OpCode        op;     // operator
        const addr_t* arg;    // arguments
        size_t        i_var;  // variable index of first result
        random_itr.op_info(i_op, op, arg, i_var);
        CPPAD_ASSERT_UNKNOWN( op == CExpOp );
        //
        struct_cexp_info info;
        info.i_op       = addr_t(i_op);
        info.cop        = CompareOp( arg[0] );
        info.flag       = static_cast<unsigned char>(arg[1]);
        info.left       = arg[2];
        info.right      = arg[3];
        //
        // max_left_right
        addr_t index    = 0;
        if( arg[1] & 1 )
            index = std::max<addr_t>(index, info.left);
        if( arg[1] & 2 )
            index = std::max<addr_t>(index, info.right);
        info.max_left_right = index;
        //
        cexp_info[i] = info;
    };
    // Determine which operators can be conditionally skipped
    size_t i_op = 0;
    while(i_op < num_op)
    {   size_t j_op = i_op;
        bool keep = op_usage[i_op] != usage_t(no_usage);
        keep     &= op_usage[i_op] != usage_t(csum_usage);
        keep     &= op_previous[i_op] == 0;
        if( keep )
        {   sparse::list_setvec_const_iterator itr(cexp_set, i_op);
            if( *itr != cexp_set.end() )
            {   if( play->GetOp(i_op) == AFunOp )
                {   // i_op is the first operations in this atomic function call.
                    // Find the last operation in this call.
                    ++j_op;
                    while( play->GetOp(j_op) != AFunOp )
                    {   switch( play->GetOp(j_op) )
                        {   case FunapOp:
                            case FunavOp:
                            case FunrpOp:
                            case FunrvOp:
                            break;

                            default:
                            CPPAD_ASSERT_UNKNOWN(false);
                        }
                        ++j_op;
                    }
                }
            }
            while( *itr != cexp_set.end() )
            {   size_t element = *itr;
                size_t index   = element / 2;
                bool   compare = bool( element % 2 );
                if( compare == false )
                {   // cexp_info[index].skip_op_false.push_back(i_op);
                    skip_op_false.post_element(index, i_op);
                    if( j_op != i_op )
                    {   // cexp_info[index].skip_op_false.push_back(j_op);
                        skip_op_false.post_element(index, j_op);
                    }
                }
                else
                {   // cexp_info[index].skip_op_true.push_back(i_op);
                    skip_op_true.post_element(index, i_op);
                    if( j_op != i_op )
                    {   // cexp_info[index].skip_op_true.push_back(j_op);
                        skip_op_true.post_element(index, j_op);
                    }
                }
                ++itr;
            }
        }
        CPPAD_ASSERT_UNKNOWN( i_op <= j_op );
        i_op += (1 + j_op) - i_op;
    }
    // process postings for skip_op_false, skip_op_true
    for(size_t i = 0; i < num_cexp_op; ++i)
    {   skip_op_false.process_post(i);
        skip_op_true.process_post(i);
    }
    return;
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
