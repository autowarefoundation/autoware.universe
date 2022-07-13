# ifndef CPPAD_LOCAL_RECORD_PUT_VAR_ATOMIC_HPP
# define CPPAD_LOCAL_RECORD_PUT_VAR_ATOMIC_HPP
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
$begin recorder_put_var_atomic$$
$spell
    ptr
    var
    enum
    taddr
$$

$section Put a Variable Atomic Call Operator in Recording$$

$head Syntax$$
$icode%rec%.put_var_atomic(
    %tape_id%, %atomic_index%, %type_x%, %type_y%, %ax%, %ay%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PUT_VAR_ATOMIC%// END_PROTOTYPE%1
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
The value $icode%ax%[%j%].value_%$$ is the proper value for all arguments.

$subhead taddr_$$
The value $icode%ax%[%j%].taddr_%$$ is the proper address
for dynamic parameters and variables and  does not matter for constants.

$head ay$$
is the atomic function result vector for this call.

$subhead Input$$
On input, $icode%ay%[%i%]%$$ has all the correct values for
parameters and does not matter for variables.

$subhead Output$$
Upon return, if the $th i$$ result is a variable,
$codei%
    %ay%[%i%].ad_type_ = dynamic_enum
    %ay%[%i%].tape_id_ = %tape_id%
    %ay%[%i%].taddr_   = %v_index%
%$$
where $icode v_index$$ is the index of this variable
in the arrays containing all the variables.

$end
*/
// BEGIN_PUT_VAR_ATOMIC
template <class Base> template <class VectorAD>
void recorder<Base>::put_var_atomic(
    tape_id_t                   tape_id      ,
    size_t                      atomic_index ,
    const vector<ad_type_enum>& type_x       ,
    const vector<ad_type_enum>& type_y       ,
    const VectorAD&             ax           ,
    VectorAD&                   ay           )
// END_PROTOTYPE
{   CPPAD_ASSERT_KNOWN(
        size_t( std::numeric_limits<addr_t>::max() ) >=
            std::max( std::max(atomic_index, ax.size() ), ay.size() ),
        "atomic_three: cppad_tape_addr_type maximum not large enough"
    );
    CPPAD_ASSERT_UNKNOWN(
        (tape_id == 0) == (AD<Base>::tape_ptr() == nullptr)
    );
    // Operator that marks beginning of this atomic operation
    CPPAD_ASSERT_NARG_NRES(local::AFunOp, 4, 0 );
    addr_t old_id = 0; // used by atomic_two to implement atomic_one interface
    size_t n = ax.size();
    size_t m = ay.size();
    PutArg(addr_t(atomic_index), old_id, addr_t(n), addr_t(m));
    PutOp(local::AFunOp);

    // Now put n operators, one for each element of argument vector
    CPPAD_ASSERT_NARG_NRES(local::FunavOp, 1, 0 );
    CPPAD_ASSERT_NARG_NRES(local::FunapOp, 1, 0 );
    for(size_t j = 0; j < n; j++)
    {   if( type_x[j] == variable_enum )
        {   // information for an argument that is a variable
            PutArg(ax[j].taddr_);
            PutOp(local::FunavOp);
        }
        else
        {   // information for an argument that is parameter
            addr_t par = ax[j].taddr_;
            if( type_x[j] == constant_enum )
                par = put_con_par(ax[j].value_);
            PutArg(par);
            PutOp(local::FunapOp);
        }
    }

    // Now put m operators, one for each element of result vector
    CPPAD_ASSERT_NARG_NRES(local::FunrvOp, 0, 1);
    CPPAD_ASSERT_NARG_NRES(local::FunrpOp, 1, 0);
    for(size_t i = 0; i < m; i++)
    {   if( type_y[i] == variable_enum )
        {   ay[i].taddr_    = PutOp(local::FunrvOp);
            ay[i].tape_id_  = tape_id;
            ay[i].ad_type_  = variable_enum;
        }
        else
        {   addr_t par = ay[i].taddr_;
            if( type_y[i] == constant_enum )
                par = put_con_par( ay[i].value_ );
            PutArg(par);
            PutOp(local::FunrpOp);
        }
    }

    // Put a duplicate AFunOp at end of AFunOp sequence
    PutArg(addr_t(atomic_index), old_id, addr_t(n), addr_t(m));
    PutOp(local::AFunOp);
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
