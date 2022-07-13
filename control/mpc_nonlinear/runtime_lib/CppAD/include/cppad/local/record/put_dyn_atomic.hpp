# ifndef CPPAD_LOCAL_RECORD_PUT_DYN_ATOMIC_HPP
# define CPPAD_LOCAL_RECORD_PUT_DYN_ATOMIC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/record/recorder.hpp>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*
$begin recorder_put_dyn_atomic$$
$spell
    ptr
    dyn
    enum
    taddr
$$

$section Put a Dynamic Parameter Atomic Call Operator in Recording$$

$head Syntax$$
$icode%rec%.put_dyn_atomic(
    %tape_id%, %atomic_index%, %type_x%, %type_y%, %ax%, %ay%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PUT_DYN_ATOMIC%// END_PROTOTYPE%1
%$$

$head tape_id$$
identifies the tape that this recording corresponds to.
This is zero if and only if there is no tape for this recording; i.e.
$codei%AD<%Base%>.tape_ptr()%$$ is null.

$head atomic_index$$
is the $cref atomic_index$$ for this atomic function.

$head type_x$$
is the $cref ad_type_enum$$ for each of the atomic function arguments.

$head type_y$$
is the $code ad_type_enum$$ for each of the atomic function results.

$head ax$$
is the atomic function argument vector for this call.

$subhead value_$$
The value $icode%ax%[%j%].value_%$$ is the proper value for
parameters and does not matter for variables.

$subhead taddr_$$
The value $icode%ax%[%j%].taddr_%$$ is the proper address for
dynamic parameters and does not matter for constants or variables.

$head ay$$
is the atomic function result vector for this call.

$subhead Input$$
On input, $icode%ay%[%j%].value_%$$ has the proper value for parameters
(result for the atomic function).

$subhead Output$$
Upon return, if the $th i$$ result is a dynamic parameter,
$codei%
    %ay%[%i%].ad_type_ = dynamic_enum
    %ay%[%i%].tape_id_ = %tape_id%
    %ay%[%i%].taddr_   = %p_index%
%$$
where $icode p_index$$ is the index of this dynamic parameter
in the vector of all parameters.

$end
*/

// BEGIN_PUT_DYN_ATOMIC
template <class Base> template <class VectorAD>
void recorder<Base>::put_dyn_atomic(
    tape_id_t                   tape_id      ,
    size_t                      atomic_index ,
    const vector<ad_type_enum>& type_x       ,
    const vector<ad_type_enum>& type_y       ,
    const VectorAD&             ax           ,
    VectorAD&                   ay           )
// END_PROTOTYPE
{   CPPAD_ASSERT_UNKNOWN(
        (tape_id == 0) == (AD<Base>::tape_ptr() == nullptr)
    );
    CPPAD_ASSERT_UNKNOWN( ax.size() == type_x.size() );
    CPPAD_ASSERT_UNKNOWN( ay.size() == type_y.size() );
    size_t n       = ax.size();
    size_t m       = ay.size();
    size_t num_dyn = 0;
    for(size_t i = 0; i < m; ++i)
        if( type_y[i] == dynamic_enum )
            ++num_dyn;
    CPPAD_ASSERT_UNKNOWN( num_dyn > 0 );
    //
    dyn_par_arg_.push_back( addr_t(atomic_index )); // arg[0] = atomic_index
    dyn_par_arg_.push_back( addr_t( n ) );          // arg[1] = n
    dyn_par_arg_.push_back( addr_t( m ) );          // arg[2] = m
    dyn_par_arg_.push_back( addr_t( num_dyn ) );    // arg[3] = num_dyn
    // arg[4 + j] for j = 0, ... , n-1
    for(size_t j = 0; j < n; ++j)
    {   addr_t arg = 0;
        switch( type_x[j] )
        {   case constant_enum:
            arg = put_con_par( ax[j].value_ );
            break;

            case dynamic_enum:
            arg = ax[j].taddr_;
            break;

            case variable_enum:
            arg = 0; // phantom parameter index
            CPPAD_ASSERT_UNKNOWN( isnan( all_par_vec_[arg] ) )
            break;

            default:
            arg = 0;
            CPPAD_ASSERT_UNKNOWN( false );
        }
        dyn_par_arg_.push_back( arg ); // arg[4 + j]
    }
    // arg[4 + n + i] for i = 0, ... , m-1
    bool first_dynamic_result = true;
    for(size_t i = 0; i < m; ++i)
    {   addr_t arg;
        switch( type_y[i] )
        {   case constant_enum:
            arg = 0; // phantom parameter index
            break;

            case dynamic_enum:
            // one operator for each dynamic parameter result
            // so number of operators is equal number of dynamic parameters
            if( first_dynamic_result )
                arg = put_dyn_par(ay[i].value_, atom_dyn );    // atom_dyn
            else
                arg = put_dyn_par(ay[i].value_, result_dyn );  // result_dyn
            ay[i].ad_type_ = dynamic_enum;
            ay[i].taddr_   = arg;
            ay[i].tape_id_ = tape_id;
            first_dynamic_result = false;
            break;

            case variable_enum:
            arg = 0; // phantom parameter (has value nan)
            break;

            default:
            arg = 0;
            CPPAD_ASSERT_UNKNOWN( false );
        }
        dyn_par_arg_.push_back( arg ); // arg[4 + n + i]
    }
    dyn_par_arg_.push_back( addr_t(5 + n + m) ); // arg[4 + n + m]
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
