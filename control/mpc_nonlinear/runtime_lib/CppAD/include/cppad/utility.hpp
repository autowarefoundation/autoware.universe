# ifndef CPPAD_UTILITY_HPP
# define CPPAD_UTILITY_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
// BEGIN_SORT_THIS_LINE_PLUS_1
# include <cppad/utility/check_numeric_type.hpp>
# include <cppad/utility/check_simple_vector.hpp>
# include <cppad/utility/elapsed_seconds.hpp>
# include <cppad/utility/error_handler.hpp>
# include <cppad/utility/index_sort.hpp>
# include <cppad/utility/lu_factor.hpp>
# include <cppad/utility/lu_invert.hpp>
# include <cppad/utility/lu_solve.hpp>
# include <cppad/utility/memory_leak.hpp>
# include <cppad/utility/nan.hpp>
# include <cppad/utility/near_equal.hpp>
# include <cppad/utility/ode_err_control.hpp>
# include <cppad/utility/ode_gear_control.hpp>
# include <cppad/utility/ode_gear.hpp>
# include <cppad/utility/omp_alloc.hpp>
# include <cppad/utility/poly.hpp>
# include <cppad/utility/pow_int.hpp>
# include <cppad/utility/romberg_mul.hpp>
# include <cppad/utility/romberg_one.hpp>
# include <cppad/utility/rosen_34.hpp>
# include <cppad/utility/runge_45.hpp>
# include <cppad/utility/set_union.hpp>
# include <cppad/utility/sparse_rc.hpp>
# include <cppad/utility/sparse_rcv.hpp>
# include <cppad/utility/speed_test.hpp>
# include <cppad/utility/test_boolofvoid.hpp>
# include <cppad/utility/thread_alloc.hpp>
# include <cppad/utility/time_test.hpp>
# include <cppad/utility/to_string.hpp>
# include <cppad/utility/track_new_del.hpp>
# include <cppad/utility/vector.hpp>
// END_SORT_THIS_LINE_MINUS_1

# if CPPAD_HAS_EIGEN
# include <cppad/utility/sparse2eigen.hpp>
# endif

# endif
