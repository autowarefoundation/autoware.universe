# ifndef CPPAD_CPPAD_HPP
# define CPPAD_CPPAD_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*!
\file cppad.hpp
\brief includes the entire CppAD package in the necessary order.

\namespace CppAD
\brief contains all the variables and functions defined by the CppAD package.
*/

# include <cppad/base_require.hpp> // all base type requirements
// ---------------------------------------------------------------------------
// CppAD general purpose library routines (can be included separately)
# include <cppad/utility.hpp>
// --------------------------------------------------------------------------
// System routines that can be used by rest of CppAD with out including

# include <cstddef>
# include <iostream>
# include <complex>
# include <cmath>

// ---------------------------------------------------------------------------
// definitions needed by rest of includes

// definitions that come from the installation
# include <cppad/configure.hpp>

// definitions that are local to the CppAD include files
# include <cppad/local/define.hpp>

// vectors used with CppAD
# include <cppad/core/testvector.hpp>

// deprecated vectors used with CppAD
# include <cppad/core/test_vector.hpp>

// Declare classes and fucntions that are used before defined
# include <cppad/local/declare_ad.hpp>

// ---------------------------------------------------------------------------
// declare the AD<Base> template class

# include <cppad/core/ad.hpp>

// ---------------------------------------------------------------------------

# include <cppad/core/user_ad.hpp>  // AD class methods available to the user
// tape that tape for AD<Base> acts as a user of Base operations
// so user_ad.hpp must come before op.hpp
# include <cppad/local/op.hpp>       // executes taped operations
# include <cppad/core/ad_fun.hpp>   // ADFun objects

// ---------------------------------------------------------------------------
// library routines that require the rest of CppAD
# include <cppad/core/lu_ratio.hpp>
# include <cppad/core/bender_quad.hpp>
# include <cppad/core/opt_val_hes.hpp>
# include <cppad/local/graph/cpp_graph_op.hpp>
# include <cppad/local/graph/json_lexer.hpp>
# if CPPAD_HAS_IPOPT
# include <cppad/ipopt/solve.hpp>
# endif

// undo definitions in Define.h
# include <cppad/core/undef.hpp>

# endif
