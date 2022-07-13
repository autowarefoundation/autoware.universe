# ifndef CPPAD_LOCAL_RECORD_PUT_VAR_VECAD_HPP
# define CPPAD_LOCAL_RECORD_PUT_VAR_VECAD_HPP
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
------------------------------------------------------------------------------
$begin put_var_vecad_ind$$
$spell
    Vec
    var
    vecad
    ind
    taddr
$$

$section Add One Index to End of Combined Variable VecAD Vector$$

$head Syntax$$
$icode%offset% = %rec%.put_var_vecad_ind(%vec_ind%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PUT_VAR_VECAD_IND%// END_PUT_VAR_VECAD_IND%1
%$$

$head Purpose$$
For each variable VecAD vector, this routine is used to store the length
of the vector followed by the parameter index corresponding to initial
value in the vector; i.e., the values just before it changed from a parameter
to a variable.

$head vec_ind$$
is the index to be placed at the end of the combined vector of VecAD indices.

$head offset$$
is the index in the combined variable VecAD vector
where the value $icode vec_ind$$ is stored.
This index starts at zero after the recorder default constructor and
increments by one for each call to put_var_vecad_ind.

$end
*/
// BEGIN_PUT_VAR_VECAD_IND
template <class Base>
addr_t recorder<Base>::put_var_vecad_ind(addr_t vec_ind)
// END_PUT_VAR_VECAD_IND
{   size_t offset = all_var_vecad_ind_.size();
    all_var_vecad_ind_.push_back( vec_ind );
    CPPAD_ASSERT_KNOWN(
        size_t( addr_t( offset ) ) == offset,
        "cppad_tape_addr_type cannot support needed index range"
    );
    return static_cast<addr_t>( offset );
}
/*
------------------------------------------------------------------------------
$begin recorder_put_var_vecad$$
$spell
    Vec
    var
    vecad
    taddr
$$
$section Tape Initialization for a Variable VecAD Object$$

$head Syntax$$
$icode%offset% = %rec%.put_var_vecad(%length%, %taddr%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PUT_VAR_VECAD_VEC%// END_PUT_VAR_VECAD_VEC%1
%$$

$head Usage$$
This routine should be called once for each variable VecAD object just
before it changes from a parameter to a variable.

$head length$$
is the size of the VecAD object.

$head taddr$$
vector of parameter indices corresponding to the value of this VecAD vector
just before it becomes a variable.

$head offset$$
index of the start of this VecAD vector in the combined variable VecAD vector.
The value corresponding to $icode offset$$ is the length of this VecAD vector.
There are $icode length$$ more indices following the length.
These values are the parameter indices.

$end
*/
// BEGIN_PUT_VAR_VECAD_VEC
template <class Base>
addr_t recorder<Base>::put_var_vecad(
    size_t                        length   ,
    const pod_vector<addr_t>&     taddr    )
// END_PUT_VAR_VECAD_VEC
{   CPPAD_ASSERT_UNKNOWN( length > 0 );
    CPPAD_ASSERT_UNKNOWN( length == taddr.size() );
    CPPAD_ASSERT_KNOWN(
        size_t( std::numeric_limits<addr_t>::max() ) >= length,
        "A VecAD vector length is too large fur cppad_tape_addr_type"
    );

    // store the length in VecInd
    addr_t start = put_var_vecad_ind( addr_t(length) );

    // store indices of the values in VecInd
    for(size_t i = 0; i < length; i++)
        put_var_vecad_ind( taddr[i] );

    // return the taddr of the length (where the vector starts)
    return start;
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
