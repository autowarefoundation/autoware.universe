# ifndef CPPAD_LOCAL_OPTIMIZE_USAGE_HPP
# define CPPAD_LOCAL_OPTIMIZE_USAGE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/define.hpp>

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize {

typedef CPPAD_VEC_ENUM_TYPE usage_t;

enum enum_usage {
    /// This operator is not used.
    no_usage,

    /// This operator is used one or more times.
    yes_usage,

    /*!
    This operator is only used once, it is a summation operator,
    and its parrent is a summation operator. Furthermore, its result is not
    a dependent variable. Hence case it can be removed as part of a
    cumulative summation starting at its parent or above.
    */
    csum_usage
};


} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
# endif
