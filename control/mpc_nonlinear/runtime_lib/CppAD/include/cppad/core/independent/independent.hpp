# ifndef CPPAD_CORE_INDEPENDENT_INDEPENDENT_HPP
# define CPPAD_CORE_INDEPENDENT_INDEPENDENT_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/independent.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

/*
$begin independent_all$$
$spell
    op
$$

$section Independent: All Arguments Present$$

$head Purpose$$
This implements $cref Independent$$ with all the possible arguments present.

$head Syntax$$
$codei%Independent(%x%, %abort_op_index%, %record_compare%, %dynamic%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_ALL_ARGUMENT%// END_ALL_ARGUMENT%1
%$$

$head Base$$
The base type the recording started by this operation.

$head ADVector$$
is simple vector type with elements of type $codei%AD<%Base%>%$$.

$head x$$
is the vector of the independent variables.

$head abort_op_index$$
operator index at which execution will be aborted (during  the recording
of operations). The value zero corresponds to not aborting (will not match).

$head record_compare$$
should comparison operators be recorded.

$head dynamic$$
is the independent dynamic parameter vector.

$end
*/
// BEGIN_ALL_ARGUMENT
template <class ADVector>
void Independent(
    ADVector&  x              ,
    size_t     abort_op_index ,
    bool       record_compare ,
    ADVector&  dynamic        )
// END_ALL_ARGUMENT
{   CPPAD_ASSERT_KNOWN(
        abort_op_index == 0 || record_compare,
        "Independent: abort_op_index is non-zero and record_compare is false."
    );
    typedef typename ADVector::value_type ADBase;
    typedef typename ADBase::value_type   Base;
    CPPAD_ASSERT_KNOWN(
        ADBase::tape_ptr() == nullptr,
        "Independent: cannot create a new tape because\n"
        "a previous tape is still active (for this thread).\n"
        "AD<Base>::abort_recording() would abort this previous recording."
    );
    local::ADTape<Base>* tape = ADBase::tape_manage(new_tape_manage);
    tape->Independent(x, abort_op_index, record_compare, dynamic);
}
/*
----------------------------------------------------------------------------
$begin independent_x_abort_record$$
$spell
    op
$$

$section Independent: Default For dynamic$$

$head Purpose$$
This implements $cref Independent$$ using
the default for the dynamic argument.

$head Syntax$$
$codei%Independent(%x%, %abort_op_index%, %record_compare%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_THREE_ARGUMENT%// END_THREE_ARGUMENT%1
%$$

$head Base$$
The base type the recording started by this operation.

$head ADVector$$
is simple vector type with elements of type $codei%AD<%Base%>%$$.

$head x$$
is the vector of the independent variables.

$head abort_op_index$$
operator index at which execution will be aborted (during  the recording
of operations). The value zero corresponds to not aborting (will not match).

$head record_compare$$
should comparison operators be recorded.

$end
*/
// BEGIN_THREE_ARGUMENT
template <class ADVector>
void Independent(ADVector &x, size_t abort_op_index, bool record_compare)
// END_THREE_ARGUMENT
{   ADVector dynamic(0); // empty vector
    Independent(x, abort_op_index, record_compare, dynamic);
}
/*
------------------------------------------------------------------------------
$begin independent_x_abort_op_index$$
$spell
    op
$$

$section Independent: Default For record_compare, dynamic$$

$head Purpose$$
This implements $cref Independent$$ using
the default for the record_compare and dynamic arguments.

$head Syntax$$
$codei%Independent(%x%, %abort_op_index%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_X_ABORT_OP_INDEX%// END_X_ABORT_OP_INDEX%1
%$$

$head Base$$
The base type the recording started by this operation.

$head ADVector$$
is simple vector type with elements of type $codei%AD<%Base%>%$$.

$head x$$
is the vector of the independent variables.

$head abort_op_index$$
operator index at which execution will be aborted (during  the recording
of operations). The value zero corresponds to not aborting (will not match).

$end
*/
// BEGIN_X_ABORT_OP_INDEX
template <class ADVector>
void Independent(ADVector &x, size_t abort_op_index)
// END_X_ABORT_OP_INDEX
{   bool     record_compare = true;
    ADVector dynamic(0); // empty vector
    Independent(x, abort_op_index, record_compare, dynamic);
}
/*
------------------------------------------------------------------------------
$begin independent_x_dynamic$$
$spell
    op
$$

$section Independent: Default For abort_op_index, record_compare$$

$head Purpose$$
This implements $cref Independent$$ using
the default for the abort_op_index and record_compare.

$head Syntax$$
$codei%Independent(%x%, %dynamic%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_X_DYNAMIC%// END_X_DYNAMIC%1
%$$

$head Base$$
The base type the recording started by this operation.

$head ADVector$$
is simple vector type with elements of type $codei%AD<%Base%>%$$.

$head x$$
is the vector of the independent variables.

$head dynamic$$
is the independent dynamic parameter vector.

$end
*/
// BEGIN_X_DYNAMIC
template <class ADVector>
void Independent(ADVector& x, ADVector& dynamic)
// END_X_DYNAMIC
{   size_t   abort_op_index = 0;
    bool     record_compare = true;
    Independent(x, abort_op_index, record_compare, dynamic);
}
/*
------------------------------------------------------------------------------
$begin independent_x$$
$spell
    op
$$

$section Independent: Default For abort_op_index, record_compare, dynamic$$

$head Purpose$$
This implements $cref Independent$$ using
the default for the abort_op_index, record_compare and dynamic arguments.

$head Syntax$$
$codei%Independent(%x%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_ONE_ARGUMENT%// END_ONE_ARGUMENT%1
%$$

$head Base$$
The base type the recording started by this operation.

$head ADVector$$
is simple vector type with elements of type $codei%AD<%Base%>%$$.

$head x$$
is the vector of the independent variables.

$end
*/
// BEGIN_ONE_ARGUMENT
template <class ADVector>
void Independent(ADVector &x)
// END_ONE_ARGUMENT
{   size_t   abort_op_index = 0;
    bool     record_compare = true;
    ADVector dynamic(0); // empty vector
    Independent(x, abort_op_index, record_compare, dynamic);
}

} // END_CPPAD_NAMESPACE
# endif
