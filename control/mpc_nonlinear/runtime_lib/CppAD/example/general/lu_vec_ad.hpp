# ifndef CPPAD_EXAMPLE_GENERAL_LU_VEC_AD_HPP
# define CPPAD_EXAMPLE_GENERAL_LU_VEC_AD_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/cppad.hpp>

namespace CppAD {
    extern CppAD::AD<double> lu_vec_ad(
        size_t                           n,
        size_t                           m,
        CppAD::VecAD<double>             &Matrix,
        CppAD::VecAD<double>             &Rhs,
        CppAD::VecAD<double>             &Result,
        CppAD::AD<double>                &logdet
    );
}

# endif
