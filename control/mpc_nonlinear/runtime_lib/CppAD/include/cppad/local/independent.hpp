# ifndef CPPAD_LOCAL_INDEPENDENT_HPP
# define CPPAD_LOCAL_INDEPENDENT_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
namespace CppAD { namespace local { //  BEGIN_CPPAD_LOCAL_NAMESPACE
/*
\file local/independent.hpp
Start recording AD<Base> operations.
*/

/*!
Start recording AD<Base> operations: Implementation in local namespace.

\tparam ADVector
This is simple vector type with elements of type AD<Base>.

\param x
Vector of the independent variables.

\param abort_op_index
operator index at which execution will be aborted (during  the recording

\param record_compare
should comparison operators be recorded.
of operations). The value zero corresponds to not aborting (will not match).

\param dynamic
Vector of dynamic parameters.
*/
template <class Base>
template <class ADVector>
void ADTape<Base>::Independent(
    ADVector&    x               ,
    size_t       abort_op_index  ,
    bool         record_compare  ,
    ADVector&    dynamic
) {
    // check ADVector is Simple Vector class with AD<Base> elements
    CheckSimpleVector< AD<Base>, ADVector>();

    // dimension of the domain space
    size_t n = x.size();
    CPPAD_ASSERT_KNOWN(
        n > 0,
        "Indepdendent: the argument vector x has zero size"
    );
    CPPAD_ASSERT_UNKNOWN( Rec_.num_var_rec() == 0 );
    CPPAD_ASSERT_UNKNOWN( Rec_.get_abort_op_index() == 0 );
    CPPAD_ASSERT_UNKNOWN( Rec_.get_record_compare() == true );
    CPPAD_ASSERT_UNKNOWN( Rec_.get_num_dynamic_ind()    == 0 );

    // set record_compare and abort_op_index before doing anything else
    Rec_.set_record_compare(record_compare);
    Rec_.set_abort_op_index(abort_op_index);
    Rec_.set_num_dynamic_ind( dynamic.size() );

    // mark the beginning of the tape and skip the first variable index
    // (zero) because parameters use taddr zero
    CPPAD_ASSERT_NARG_NRES(BeginOp, 1, 1);
    Rec_.PutOp(BeginOp);
    Rec_.PutArg(0);

    // place each of the independent variables in the tape
    CPPAD_ASSERT_NARG_NRES(InvOp, 0, 1);
    for(size_t j = 0; j < n; j++)
    {   // tape address for this independent variable
        CPPAD_ASSERT_UNKNOWN( ! Variable(x[j] ) );
        x[j].taddr_     = Rec_.PutOp(InvOp);
        x[j].tape_id_   = id_;
        x[j].ad_type_   = variable_enum;
        CPPAD_ASSERT_UNKNOWN( size_t(x[j].taddr_) == j+1 );
        CPPAD_ASSERT_UNKNOWN( Variable(x[j] ) );
    }

    // done specifying all of the independent variables
    size_independent_ = n;

    // parameter index zero is used by dynamic parameter tape
    // to indicate that an argument is a variable
    Base nan = CppAD::numeric_limits<Base>::quiet_NaN();
# ifndef NDEBUG
    CPPAD_ASSERT_UNKNOWN( Rec_.put_con_par(nan) == 0 );
# else
    Rec_.put_con_par(nan);
# endif

    // Place independent dynamic parameters at beginning of parameter vector,
    // just after the nan at index zero.
    for(size_t j = 0; j < Rec_.get_num_dynamic_ind(); ++j)
    {   CPPAD_ASSERT_UNKNOWN( ! Dynamic( dynamic[j] ) );
        CPPAD_ASSERT_UNKNOWN( Parameter( dynamic[j] ) );
        //
        // dynamic parameters are placed at the end, so i == j
# ifndef NDEBUG
        addr_t i = Rec_.put_dyn_par(dynamic[j].value_ , ind_dyn);
        CPPAD_ASSERT_UNKNOWN( size_t(i) == j+1 );
# else
        Rec_.put_dyn_par(dynamic[j].value_ , ind_dyn);
# endif
        //
        // make this parameter dynamic
        dynamic[j].taddr_   = static_cast<addr_t>(j+1);
        dynamic[j].tape_id_ = id_;
        dynamic[j].ad_type_ = dynamic_enum;
        CPPAD_ASSERT_UNKNOWN( Dynamic( dynamic[j] ) );
    }
}
} } // END_CPPAD_LOCAL_NAMESPACE

# endif
