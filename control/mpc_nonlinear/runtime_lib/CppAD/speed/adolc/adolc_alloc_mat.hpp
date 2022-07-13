# ifndef CPPAD_SPEED_ADOLC_ADOLC_ALLOC_MAT_HPP
# define CPPAD_SPEED_ADOLC_ADOLC_ALLOC_MAT_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
double** adolc_alloc_mat(size_t m, size_t n);
void adolc_free_mat(double** mat);

# endif
