# ifndef CPPAD_CORE_UNDEF_HPP
# define CPPAD_CORE_UNDEF_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
----------------------------------------------------------------------------
Preprecessor definitions that presist after cppad/cppad.hpp is included.
These are part of the user API with some exceptions that are used
by the CppAD examples and tests.

# undef CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL used by CPPAD_USER_ATOMIC
# undef CPPAD_ASSERT_KNOWN                   used by cppad_ipopt
# undef CPPAD_ASSERT_UNKNOWN                 used by cppad_ipopt
# undef CPPAD_HASH_TABLE_SIZE                used by test_more/optimize.cpp
# undef EIGEN_MATRIXBASE_PLUGIN              example use of Eigen with CppAD
# undef CPPAD_HAS_COLPACK                    used by speed/cppad/sparse_*.cpp

# undef CPPAD_BOOL_BINARY         in user api
# undef CPPAD_BOOL_UNARY          in user api
# undef CPPAD_DEBUG_AND_RELEASE   in user api
# undef CPPAD_DISCRETE_FUNCTION   in user api
# undef CPPAD_EIGENVECTOR         in user api
# undef CPPAD_MAX_NUM_THREADS     in user api
# undef CPPAD_NUMERIC_LIMITS      in user api
# undef CPPAD_NULL                in user api
# undef CPPAD_PACKAGE_STRING      in user api
# undef CPPAD_STANDARD_MATH_UNARY in user api
# undef CPPAD_TAPE_ADDR_TYPE      in user api
# undef CPPAD_TAPE_ID_TYPE        in user api
# undef CPPAD_TESTVECTOR          in user api
# undef CPPAD_TO_STRING           in user api
# undef CPPAD_USE_CPLUSPLUS_2011  in user api

# undef CPPAD_TRACK_COUNT    in deprecated api
# undef CPPAD_TRACK_DEL_VEC  in deprecated api
# undef CPPAD_TRACK_EXTEND   in deprecated api
# undef CPPAD_TRACK_NEW_VEC  in deprecated api
# undef CPPAD_USER_ATOMIC    in deprecated api

# undef CPPAD_TEST_VECTOR     deprecated verssion of CPPAD_TESTVECTOR
# undef CppADCreateBinaryBool deprecated version of CPPAD_BOOL_BINARY
# undef CppADCreateDiscrete   deprecated version of CPPAD_DISCRETE_FUNCTION
# undef CppADCreateUnaryBool  deprecated version of CPPAD_BOOL_UNARY
# undef CppADTrackCount       deprecated version of CPPAD_TRACK_COUNT
# undef CppADTrackDelVec      deprecated version of CPPAD_TRACK_DEL_VEC
# undef CppADTrackExtend      deprecated version of CPPAD_TRACK_EXTEND
# undef CppADTrackNewVec      deprecated version of CPPAD_TRACK_NEW_VEC
# undef CppADvector           deprecated version of CPPAD_TEST_VECTOR

// for conditional testing when implicit conversion is not present
# undef CPPAD_DEPRECATED
-----------------------------------------------------------------------------
*/
// Preprecessor definitions that do not presist. None of these are in the
// user API.
# undef CPPAD_ASSERT_NARG_NRES
# undef CPPAD_AZMUL
# undef CPPAD_BOOSTVECTOR
# undef CPPAD_COMPILER_HAS_CONVERSION_WARN
# undef CPPAD_COND_EXP
# undef CPPAD_COND_EXP_BASE_REL
# undef CPPAD_COND_EXP_REL
# undef CPPAD_CPPADVECTOR
# undef CPPAD_FOLD_AD_VALUED_BINARY_OPERATOR
# undef CPPAD_FOLD_ASSIGNMENT_OPERATOR
# undef CPPAD_FOLD_BOOL_VALUED_BINARY_OPERATOR
# undef CPPAD_HAS_ADOLC
# undef CPPAD_HAS_EIGEN
# undef CPPAD_HAS_GETTIMEOFDAY
# undef CPPAD_HAS_IPOPT
# undef CPPAD_HAS_MKSTEMP
# undef CPPAD_HAS_TMPNAM_S
# undef CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
# undef CPPAD_LIB_EXPORT
# undef CPPAD_MAX_NUM_CAPACITY
# undef CPPAD_MIN_DOUBLE_CAPACITY
# undef CPPAD_NDEBUG_NOEXCEPT
# undef CPPAD_NOEXCEPT
# undef CPPAD_STANDARD_MATH_UNARY_AD
# undef CPPAD_STDVECTOR
# undef CPPAD_TRACE_CAPACITY
# undef CPPAD_TRACE_THREAD
# undef CPPAD_TRACK_DEBUG
# undef CPPAD_USER_MACRO
# undef CPPAD_USER_MACRO_ONE
# undef CPPAD_USER_MACRO_TWO
# undef CPPAD_VEC_AD_COMP_ASSIGN
# undef CPPAD_VEC_ENUM_TYPE

# endif
