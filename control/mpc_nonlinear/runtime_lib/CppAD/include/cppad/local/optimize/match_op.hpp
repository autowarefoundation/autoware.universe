# ifndef CPPAD_LOCAL_OPTIMIZE_MATCH_OP_HPP
# define CPPAD_LOCAL_OPTIMIZE_MATCH_OP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/optimize/hash_code.hpp>
// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize  {
/*
$begin optimize_match_op$$

$section Search for a Previous Operator that Matches Current Operator$$
$spell
    op
    itr
    bool
    addr
    erf
    erfc
    iterator
$$

$head Syntax$$
$codei%exceed_collision_limit% = match_op(
    %collision_limit%,
    %random_itr%,
    %op_previous%,
    %current%,
    %hash_tape_op%,
    %work_bool%,
    %work_addr_t%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
%$$

$head Operator Arguments$$
If an argument for the current operator is a variable,
and the argument has previous match,
the previous match for the argument is used when checking for a match
for the current operator.

$head collision_limit$$
is the maximum number of collisions (matches) allowed for one
expression hash code value.

$head random_itr$$
is a random iterator for the old operation sequence.

$head op_previous$$
Mapping from operator index to previous operator that can replace this one.
The input value of
$codei%
    %previous% = %op_previous%[%current%]
%$$
is assumed to be zero.  If a match if found, the output value of
$icode previous$$ is set to the matching operator index,
otherwise it is left as is.  Note that $icode%previous% < %current%$$
and $icode%op_previous[%previous%]%$$ is zero.

$head current$$
is the index of the current operator which cannot be any of the
operators in the list below:
$srcthisfile%
    0%// BEGIN_INVALID_OP%// END_INVALID_OP%1
%$$
After this initialization, the value of $icode current$$
increases with each call to match_op.

$subhead erf$$
The operators $code ErfOp$$ and $code ErfcOp$$ have
three arguments, but only one true argument (the others are always the same).


$head hash_table_op$$
is assumed to be initialized as a vector of empty sets before the
first call to match_op (for a pass of the operation sequence).
$codei%
    %hash_table_op%.n_set() == CPPAD_HASH_TABLE_SIZE
    %hash_table_op%.end()   == %op_previous%.size()
%$$
If $icode i_op$$ is an element of the j-th set,
then the operation $icode%op_previous%[%i_op%]%$$ has hash code j,
and does not match any other element of the j-th set.
An entry to j-th set for the current operator is added each time
match_op is called and a match for the current operator is not found.

$head work_bool$$
work space that is used by match_op between calls to increase speed.
Should be empty on first call for this forward pass of the operation
sequence and not modified until forward pass is done

$head work_addr_t$$
work space that is used by match_op between calls to increase speed.
Should be empty on first call for this forward pass of the operation
sequence and not modified until forward pass is done

$head exceed_collision_limit$$
If the $icode collision_limit$$ is exceeded (is not exceeded),
the return value is true (false).

$end
*/
// BEGIN_PROTOTYPE
template <class Addr>
bool match_op(
    size_t                                      collision_limit ,
    const play::const_random_iterator<Addr>&    random_itr      ,
    pod_vector<addr_t>&                         op_previous     ,
    size_t                                      current         ,
    sparse::list_setvec&                        hash_table_op   ,
    pod_vector<bool>&                           work_bool       ,
    pod_vector<addr_t>&                         work_addr_t     )
// END_PROTOTYPE
{
# ifndef NDEBUG
    switch( random_itr.get_op(current) )
    {
        // BEGIN_INVALID_OP
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
        // END_INVALID_OP
        CPPAD_ASSERT_UNKNOWN(false);
        break;

        default:
        break;
    }
# endif
    // initialize return value
    bool exceed_collision_limit = false;
    // num_op
    size_t num_op = random_itr.num_op();
    //
    // num_var
    size_t num_var = random_itr.num_var();
    //
    // variable is a reference to, and better name for, work_bool
    pod_vector<bool>&  variable(work_bool);
    //
    // var2previous_var is a reference to, and better name for, work_addr_t
    pod_vector<addr_t>&  var2previous_var(work_addr_t);
    if( var2previous_var.size() == 0 )
    {   var2previous_var.resize(num_var);
        for(size_t i = 0; i < num_var; ++i)
            var2previous_var[i] = addr_t(i);
    }
    //
    CPPAD_ASSERT_UNKNOWN( var2previous_var.size() == num_var );
    CPPAD_ASSERT_UNKNOWN( num_op == op_previous.size() );
    CPPAD_ASSERT_UNKNOWN( op_previous[current] == 0 );
    CPPAD_ASSERT_UNKNOWN(
        hash_table_op.n_set() == CPPAD_HASH_TABLE_SIZE
    );
    CPPAD_ASSERT_UNKNOWN( hash_table_op.end() == num_op );
    CPPAD_ASSERT_UNKNOWN( current < num_op );
    //
    // op, arg, i_var
    OpCode        op;
    const addr_t* arg;
    size_t        i_var;
    random_itr.op_info(current, op, arg, i_var);
    //
    // num_arg
    size_t num_arg = NumArg(op);
    CPPAD_ASSERT_UNKNOWN( 0 < num_arg );
    CPPAD_ASSERT_UNKNOWN(
        (num_arg < 3) | ( (num_arg == 3) & (op == ErfOp || op == ErfcOp) )
    );
    //
    arg_is_variable(op, arg, variable);
    CPPAD_ASSERT_UNKNOWN( variable.size() == num_arg );
    //
    // If j-th argument to this operator is a variable, and a previous
    // variable will be used in its place, use the previous variable for
    // hash coding and matching.
    addr_t arg_match[] = {
        // Invalid value that will not be used. This initialization avoid
        // a wraning on some compilers
        std::numeric_limits<addr_t>::max(),
        std::numeric_limits<addr_t>::max(),
        std::numeric_limits<addr_t>::max()
    };
    if( (op == AddvvOp) | (op == MulvvOp ) )
    {   // in special case where operator is commutative and operands are variables,
        // put lower index first so hash code does not depend on operator order
        CPPAD_ASSERT_UNKNOWN( num_arg == 2 );
        arg_match[0] = var2previous_var[ arg[0] ];
        arg_match[1] = var2previous_var[ arg[1] ];
        if( arg_match[1] < arg_match[0] )
            std::swap( arg_match[0], arg_match[1] );
    }
    else for(size_t j = 0; j < num_arg; ++j)
    {   arg_match[j] = arg[j];
        if( variable[j] )
            arg_match[j] = var2previous_var[ arg[j] ];
    }

    //
    size_t code = optimize_hash_code(opcode_t(op), num_arg, arg_match);
    //
    // iterator for the set with this hash code
    sparse::list_setvec_const_iterator itr(hash_table_op, code);
    //
    // check for a match
    size_t count = 0;
    while( *itr != num_op )
    {   ++count;
        //
        // candidate previous for current operator
        size_t  candidate  = *itr;
        CPPAD_ASSERT_UNKNOWN( candidate < current );
        CPPAD_ASSERT_UNKNOWN( op_previous[candidate] == 0 );
        //
        OpCode        op_c;
        const addr_t* arg_c;
        size_t        i_var_c;
        random_itr.op_info(candidate, op_c, arg_c, i_var_c);
        //
        // check for a match
        bool match = op == op_c;
        size_t j   = 0;
        while( match & (j < num_arg) )
        {   if( variable[j] )
                match &= arg_match[j] == var2previous_var[ arg_c[j] ];
            else
                match &= arg_match[j] == arg_c[j];
            ++j;
        }
        if( (! match) & ( (op == AddvvOp) | (op == MulvvOp) ) )
        {   // communative so check for reverse order match
            match  = op == op_c;
            match &= arg_match[0] == var2previous_var[ arg_c[1] ];
            match &= arg_match[1] == var2previous_var[ arg_c[0] ];
        }
        if( match )
        {   op_previous[current] = static_cast<addr_t>( candidate );
            if( NumRes(op) > 0 )
            {   CPPAD_ASSERT_UNKNOWN( i_var_c < i_var );
                var2previous_var[i_var] = addr_t( i_var_c );
            }
            return exceed_collision_limit;
        }
        ++itr;
    }

    // see print (that is commented out) at bottom of get_op_previous.hpp
    CPPAD_ASSERT_UNKNOWN( count <= collision_limit );
    if( count == collision_limit )
    {   // restart the list
        hash_table_op.clear(code);
        // limit has been exceeded
        exceed_collision_limit = true;
    }
    // No match was found. Add this operator to the set for this hash code
    // Not using post_element becasue we need to iterate for
    // this code before adding another element for this code.
    hash_table_op.add_element(code, current);
    //
    return exceed_collision_limit;
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
