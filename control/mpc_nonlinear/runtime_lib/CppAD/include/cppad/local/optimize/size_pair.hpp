# ifndef CPPAD_LOCAL_OPTIMIZE_SIZE_PAIR_HPP
# define CPPAD_LOCAL_OPTIMIZE_SIZE_PAIR_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*!
\file size_pair.hpp
Information for one variable and one operation sequence.
*/
// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize  {

/*!
\file size_pair.hpp
Information for one variable in one operation sequence.
*/
struct struct_size_pair {
    size_t i_op;  /// operator index for this variable
    size_t i_var; /// variable index for this variable
};

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
