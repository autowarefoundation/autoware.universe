# ifndef CPPAD_LOCAL_PLAY_ADDR_ENUM_HPP
# define CPPAD_LOCAL_PLAY_ADDR_ENUM_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// BEGIN_CPPAD_LOCAL_PLAY_NAMESPACE
namespace CppAD { namespace local { namespace play {

/*!
\file addr_enum.hpp
*/
/// enum corresponding to type used for addressing iterators for a player
enum addr_enum {
    unsigned_short_enum  ,
    unsigned_int_enum    ,
    size_t_enum
};

} } } // BEGIN_CPPAD_LOCAL_PLAY_NAMESPACE

# endif
