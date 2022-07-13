# ifndef CPPAD_CORE_ABORT_RECORDING_HPP
# define CPPAD_CORE_ABORT_RECORDING_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin abort_recording$$
$spell
$$

$section Abort Recording of an Operation Sequence$$


$head Syntax$$
$codei%AD<%Base%>::abort_recording()%$$

$head Purpose$$
Sometimes it is necessary to abort the recording of an operation sequence
that started with a call of the form
$codei%
    Independent(%x%)
%$$
If such a recording is currently in progress,
$code abort_recording$$ will stop the recording and delete the
corresponding information.
Otherwise, $code abort_recording$$ has no effect.

$children%
    example/general/abort_recording.cpp
%$$
$head Example$$
The file
$cref abort_recording.cpp$$
contains an example and test of this operation.

$end
----------------------------------------------------------------------------
*/


namespace CppAD {
    template <class Base>
    void AD<Base>::abort_recording(void)
    {   local::ADTape<Base>* tape = AD<Base>::tape_ptr();
        if( tape != nullptr )
            AD<Base>::tape_manage(delete_tape_manage);
    }
}

# endif
