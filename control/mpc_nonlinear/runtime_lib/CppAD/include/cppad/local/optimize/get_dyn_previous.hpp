# ifndef CPPAD_LOCAL_OPTIMIZE_GET_DYN_PREVIOUS_HPP
# define CPPAD_LOCAL_OPTIMIZE_GET_DYN_PREVIOUS_HPP
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

# include <cppad/local/optimize/match_op.hpp>
# include <cppad/local/optimize/usage.hpp>
# include <cppad/local/optimize/hash_code.hpp>

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize {

/*!
mapping from a dynamic parameter index to its arguments

\param i_dyn
is the dynamic parameter index

\param dyn_ind2par_ind
is the mapping from dynamic parameter index to parameter index
(size is number of dynamic parameters).

\param dyn_par_is
i-th element is true (false) if i-th parameter is (is not) dynamic
(size is number of parameters).

\param dyn_arg_offset
j-th element is the offset in dyn_par_arg of the first argument for j-th
dynamic parameter's operator (size is number of dynamic parameters).
This is only defined for dynamic parameters indices less than or equal i_dyn.

\param dyn_par_arg
it the vector of arguments for all the dynamic parameter operators.
This is only defined for dynamic parameters indices less than or equal i_dyn.

\param par_ind2dyn_ind
is the mapping from parameter index to dynamic parameter index
(size is number of parameters). This is only defined for parameter
indices less than or equal the parameter index corresponding to i_dyn.

\param dyn_previous
is the mapping from dynamic parameter index to previous dynamic parameter
that can be used as a replacement (size is number of dynamic parameters).
This is only defined for dynamic parameters indices less than or equal i_dyn.

\param arg_match
Size of this vector must be number of arguments for operator for i_dyn.
The input value of its elements does not matter.
Upn return it containts the parameter indices for the arguments
to use when matching this operator
Arguments that are dynamic prarameters, and have previous matches,
have been replaced by their previous matches.
*/
inline void dyn_arg_match(
    size_t                    i_dyn           ,
    const pod_vector<addr_t>& dyn_ind2par_ind ,
    const pod_vector<bool>  & dyn_par_is      ,
    const pod_vector<addr_t>& dyn_arg_offset  ,
    const pod_vector<addr_t>& dyn_par_arg     ,
    const pod_vector<addr_t>& par_ind2dyn_ind ,
    const pod_vector<addr_t>& dyn_previous    ,
    pod_vector<addr_t>&       arg_match       )
{
    // number of dynamic parameters
    addr_t num_dynamic_par = addr_t( dyn_ind2par_ind.size() );
    //
    // check some assumptions
    CPPAD_ASSERT_UNKNOWN( size_t( num_dynamic_par ) == dyn_arg_offset.size() );
    CPPAD_ASSERT_UNKNOWN( size_t( num_dynamic_par ) == dyn_previous.size() );
    CPPAD_ASSERT_UNKNOWN( dyn_par_is.size() == par_ind2dyn_ind.size() );
    //
    // number of arguments for this operator
    addr_t n_arg = addr_t( arg_match.size() );
    //
    // index in dyn_par_arg of first argument for this operator
    addr_t i_arg = dyn_arg_offset[i_dyn];
    //
    // loop over arguments for this operator
    for(addr_t j = 0; j < n_arg; ++j)
    {   // parameter index for this argument
        addr_t j_par = dyn_par_arg[i_arg + j];
        CPPAD_ASSERT_UNKNOWN( j_par < dyn_ind2par_ind[i_dyn] );
        //
        // map dynamic parameters arguments to previous matches
        if( dyn_par_is[j_par] )
        {   addr_t j_dyn = par_ind2dyn_ind[j_par];
            if( dyn_previous[j_dyn] != num_dynamic_par )
            {   CPPAD_ASSERT_UNKNOWN( dyn_previous[j_dyn] < j_dyn );
                // previous dynamic parameter
                j_dyn = dyn_previous[j_dyn];
                // correspoding parameter
                j_par = dyn_ind2par_ind[j_dyn];
            }
        }
        arg_match[j] = j_par;
    }
    return;
}

/*!
Get mapping from each dynamic parameter to a previous dynamic parameter
that can be used to replace it (if one exists).

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param play
This is the old operation sequence.

\param random_itr
This is a random iterator for the old operation sequence.

\param par_usage
The size of this vector is the number of parameters in the
operation sequence.i.e., play->nun_var_rec().
It is the usage counting previous operator optimization of operators.

\param dyn_previous
The input size of this vector must be zero.
Upon return it has size equal to the number of dynamic parameters in the
operation sequence; i.e., num_dyn = play->num_dynamic_par().
Let k = dyn_parvious[j]. If k == num_dyn, no replacement was found for the
j-th dynamic parameter. If k != num_dyn, the k-th dynamic parameter can be
used in place of the j-th dynamic parameter, k < j, dyn_previous[k] != num_dyn,
par_usage[dyn_ind2par_ind[k]] == true.
*/

template <class Addr, class Base>
void get_dyn_previous(
    const player<Base>*                         play                ,
    const play::const_random_iterator<Addr>&    random_itr          ,
    pod_vector<bool>&                           par_usage           ,
    pod_vector<addr_t>&                         dyn_previous        )
{
    // number of parameters in the recording
    size_t num_par = play->num_par_rec();

    // number of dynamic parameters in the recording
    size_t num_dynamic_par = play->num_dynamic_par();

    // number of independent dynamic parameters in the recording
    size_t num_dynamic_ind  = play->num_dynamic_ind();

    // check some assumptions
    CPPAD_ASSERT_UNKNOWN( dyn_previous.size() == 0 );
    CPPAD_ASSERT_UNKNOWN( par_usage.size() == num_par );
    CPPAD_ASSERT_UNKNOWN( num_dynamic_par <= num_par );
    CPPAD_ASSERT_UNKNOWN( num_dynamic_ind <= num_dynamic_par );
    CPPAD_ASSERT_UNKNOWN( num_arg_dyn( ind_dyn ) == 0 );

    // dynamic parameter information
    dyn_previous.resize( num_dynamic_par );
    const pod_vector<addr_t>&   dyn_ind2par_ind( play->dyn_ind2par_ind() );
    const pod_vector<bool>&     dyn_par_is( play->dyn_par_is() );
    const pod_vector<opcode_t>& dyn_par_op( play->dyn_par_op() );
    const pod_vector<addr_t>&   dyn_par_arg( play->dyn_par_arg() );

    // mapping from parameter index to dynamic parameter index
    // only defined when dyn_par_is is true
    pod_vector<addr_t> par_ind2dyn_ind(num_par);

    // mapping from dynamic parameter index to first argument index
    pod_vector<addr_t> dyn_arg_offset(num_dynamic_par);

    // ----------------------------------------------------------------------
    // compute dyn_previous
    // ----------------------------------------------------------------------
    sparse::list_setvec  hash_table_dyn;
    hash_table_dyn.resize(CPPAD_HASH_TABLE_SIZE, num_dynamic_par);
    //
    // Initialize in dyn_par_arg
    // (independent dynamic parameters do not have any arguments)
    size_t i_arg = 0;
    //
    // independent dynamic parameters
    for(size_t i_dyn = 0; i_dyn < num_dynamic_ind; ++i_dyn)
    {   // parameter index
        size_t i_par = size_t( dyn_ind2par_ind[i_dyn] );
        // dynamic parameter index is one greater because phantom parameter
        // at index 0 is not dynamic
        CPPAD_ASSERT_UNKNOWN( i_par == i_dyn + 1 );
        // mapping from parameter index to dynamic parameter index
        par_ind2dyn_ind[i_par] = addr_t( i_dyn );
        // never get optimized out
        dyn_previous[i_dyn]    = addr_t( num_dynamic_par );
    }
    //
    // other dynamic parameters
    for(size_t i_dyn = num_dynamic_ind; i_dyn < num_dynamic_par; ++i_dyn)
    {   // Initialize previous for this dynamic parameter. This is only
        // defined for dynamic parameter indices less than or equal i_dyn
        dyn_previous[i_dyn] = addr_t( num_dynamic_par );
        //
        // mapping from dynamic parameter index to argument offset
        // is only defined for j_dyn <= i_dyn
        dyn_arg_offset[i_dyn] = addr_t( i_arg );
        //
        // parameter index for this dynamic parameter
        size_t i_par = size_t( dyn_ind2par_ind[i_dyn] );
        //
        // mapping from parameter indices to dynamic parameter indices
        // is only defined when dyn_par_is[i_par] is true and for parameter
        // indices less than or equal i_par
        CPPAD_ASSERT_UNKNOWN( dyn_par_is[i_par] );
        par_ind2dyn_ind[i_par] = addr_t( i_dyn );
        //
        // operator for this dynamic parameter
        op_code_dyn op = op_code_dyn( dyn_par_op[i_dyn] );
        //
        // temporary used below and decaled here to reduce memory allocation
        pod_vector<addr_t> arg_match;
        //
        // temporaries used below and decaled here to reduce indentation level
        bool   match;
        size_t code;
        size_t count;
        //
        // check for a previous match for i_dyn
        if( par_usage[i_par] ) switch( op )
        {
            // ---------------------------------------------------------------
            // unary operators
            case abs_dyn:
            case acos_dyn:
            case acosh_dyn:
            case asin_dyn:
            case asinh_dyn:
            case atan_dyn:
            case atanh_dyn:
            case cos_dyn:
            case cosh_dyn:
            case erf_dyn:
            case erfc_dyn:
            case exp_dyn:
            case expm1_dyn:
            case fabs_dyn:
            case log_dyn:
            case log1p_dyn:
            case neg_dyn:
            case sign_dyn:
            case sin_dyn:
            case sinh_dyn:
            case sqrt_dyn:
            case tan_dyn:
            case tanh_dyn:
            CPPAD_ASSERT_UNKNOWN( num_arg_dyn(op) == 1);
            CPPAD_ASSERT_UNKNOWN( dyn_par_is[i_par] );
            {   size_t num_arg = 1;
                arg_match.resize(num_arg);
                dyn_arg_match(
                    i_dyn,
                    dyn_ind2par_ind,
                    dyn_par_is,
                    dyn_arg_offset,
                    dyn_par_arg,
                    par_ind2dyn_ind,
                    dyn_previous,
                    arg_match
                );
                opcode_t op_t  = opcode_t(op);
                code           = optimize_hash_code(
                    op_t, num_arg, arg_match.data()
                );
                //
                // iterator for the set with this hash code
                sparse::list_setvec_const_iterator itr(hash_table_dyn, code);
                //
                // check for a match
                count = 0;
                match = false;
                while( ! match && *itr != num_dynamic_par )
                {   ++count;
                    //
                    // candidate for current dynamic parameter
                    size_t  k_dyn  = *itr;
                    CPPAD_ASSERT_UNKNOWN( k_dyn < i_dyn );
                    //
                    // argument offset for the candidate
                    addr_t k_arg   = dyn_arg_offset[k_dyn];
                    //
                    match  = op_t == dyn_par_op[k_dyn];
                    match &= arg_match[0] == dyn_par_arg[k_arg + 0];
                    if( ! match )
                        ++itr;
                }
                if( match )
                {   size_t  k_dyn  = *itr;
                    CPPAD_ASSERT_UNKNOWN( k_dyn < i_dyn );
                    dyn_previous[i_dyn] = addr_t( k_dyn );
                }
                else
                {   CPPAD_ASSERT_UNKNOWN( count < 11 );
                    if( count == 10 )
                    {   // restart list for this hash code
                        hash_table_dyn.clear(code);
                    }
                    // Add this entry to hash table.
                    // Not using post_element becasue we need to iterate for
                    // this code before adding another element for this code.
                    hash_table_dyn.add_element(code, i_dyn);
                }
            }
            break;

            // ---------------------------------------------------------------
            // binary operators
            case add_dyn:
            case div_dyn:
            case mul_dyn:
            case pow_dyn:
            case sub_dyn:
            case zmul_dyn:
            CPPAD_ASSERT_UNKNOWN( num_arg_dyn(op) == 2);
            CPPAD_ASSERT_UNKNOWN( dyn_par_is[i_par] );
            match = false;
            {   size_t num_arg = 2;
                arg_match.resize(num_arg);
                dyn_arg_match(
                    i_dyn,
                    dyn_ind2par_ind,
                    dyn_par_is,
                    dyn_arg_offset,
                    dyn_par_arg   ,
                    par_ind2dyn_ind,
                    dyn_previous,
                    arg_match
                );
                opcode_t op_t  = opcode_t(op);
                code           = optimize_hash_code(
                    op_t, num_arg, arg_match.data()
                );
                //
                // iterator for the set with this hash code
                sparse::list_setvec_const_iterator itr(hash_table_dyn, code);
                //
                // check for a match
                count = 0;
                while( ! match && *itr != num_dynamic_par )
                {   ++count;
                    //
                    // candidate for current dynamic parameter
                    size_t  k_dyn  = *itr;
                    CPPAD_ASSERT_UNKNOWN( k_dyn < i_dyn );
                    //
                    // argument offset for the candidate
                    addr_t k_arg   = dyn_arg_offset[k_dyn];
                    //
                    match  = op_t == dyn_par_op[k_dyn];
                    match &= arg_match[0] == dyn_par_arg[k_arg + 0];
                    match &= arg_match[1] == dyn_par_arg[k_arg + 1];
                    if( ! match )
                        ++itr;
                }
                if( match )
                {   size_t  k_dyn  = *itr;
                    CPPAD_ASSERT_UNKNOWN( k_dyn < i_dyn );
                    dyn_previous[i_dyn] = addr_t( k_dyn );
                }
            }
            if( (! match) & ( (op == add_dyn) | (op == mul_dyn) ) )
            {   size_t num_arg = 2;
                std::swap( arg_match[0], arg_match[1] );
                opcode_t op_t    = opcode_t(op);
                size_t code_swp  = optimize_hash_code(
                    op_t, num_arg, arg_match.data()
                );
                //
                // iterator for the set with this hash code
                sparse::list_setvec_const_iterator itr(hash_table_dyn, code_swp);
                //
                // check for a match
                while( ! match && *itr != num_dynamic_par )
                {   //
                    // candidate for current dynamic parameter
                    size_t  k_dyn  = *itr;
                    CPPAD_ASSERT_UNKNOWN( k_dyn < i_dyn );
                    //
                    // argument offset for the candidate
                    addr_t k_arg   = dyn_arg_offset[k_dyn];
                    //
                    match  = op_t == dyn_par_op[k_dyn];
                    match &= arg_match[0] == dyn_par_arg[k_arg + 0];
                    match &= arg_match[1] == dyn_par_arg[k_arg + 1];
                    if( ! match )
                        ++itr;
                }
                if( match )
                {   size_t  k_dyn  = *itr;
                    CPPAD_ASSERT_UNKNOWN( k_dyn < i_dyn );
                    dyn_previous[i_dyn] = addr_t( k_dyn );
                }
            }
            if( ! match )
            {   CPPAD_ASSERT_UNKNOWN( count < 11 );
                if( count == 10 )
                {   // restart list for this hash code
                    hash_table_dyn.clear(code);
                }
                // Add the entry to hash table
                // Not using post_element becasue we need to iterate for
                // this code before adding another element for this code.
                hash_table_dyn.add_element(code, i_dyn);
            }

            // --------------------------------------------------------------
            // skipping these cases for now
            case dis_dyn:
            case cond_exp_dyn:
            case atom_dyn:
            case result_dyn:
            break;


            // --------------------------------------------------------------
            // should be no other cases; e.g., no ind_dyn or number_dyn.
            default:
            CPPAD_ASSERT_UNKNOWN(false);
            break;
        }
        i_arg += num_arg_dyn(op);
        if( op == atom_dyn )
        {   CPPAD_ASSERT_UNKNOWN( num_arg_dyn(op) == 0 );
            size_t n     = size_t( dyn_par_arg[i_arg + 1] );
            size_t m     = size_t( dyn_par_arg[i_arg + 2] );
            size_t n_arg = 5 + n + m;
            i_arg += n_arg;
        }
    }
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
