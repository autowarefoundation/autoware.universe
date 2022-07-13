/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/cppad.hpp>
# include "extern_value.hpp"

# define INSTANTIATE(Type) template class extern_value< Type >

template <class Type>
extern_value<Type>::extern_value(Type value)
{   value_ = value; }

template <class Type>
void extern_value<Type>::set(Type value)
{   value_ = value; }

template <class Type>
Type extern_value<Type>::get(void)
{   return value_; }

INSTANTIATE( float );
INSTANTIATE( double );
INSTANTIATE( std::complex<float> );
INSTANTIATE( std::complex<double> );
//
INSTANTIATE( CppAD::AD< float > );
INSTANTIATE( CppAD::AD< double > );
INSTANTIATE( CppAD::AD< std::complex<float> > );
INSTANTIATE( CppAD::AD< std::complex<double> > );
