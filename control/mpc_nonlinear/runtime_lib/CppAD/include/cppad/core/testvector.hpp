# ifndef CPPAD_CORE_TESTVECTOR_HPP
# define CPPAD_CORE_TESTVECTOR_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin testvector$$
$spell
    CppAD
    cmake
    testvector
    cppad
    Eigen
    ifdef
    hpp
    std
    endif
    ublas
$$


$section Using The CppAD Test Vector Template Class$$

$head Syntax$$
$codei%CPPAD_TESTVECTOR(%Scalar%)
%$$

$head Choice$$
The user can choose, during the install procedure,
which template class to use in the examples and tests; see below.
This shows that any
$cref/simple vector/SimpleVector/$$ class can be used in place of
$codei%
    CPPAD_TESTVECTOR(%Type%)
%$$
When writing their own code,
users can choose a specific simple vector they prefer; for example,
$codei%
    CppAD::vector<%Type%>
%$$


$head CppAD::vector$$
If in the $cref/cmake command/cmake/CMake Command/$$
you specify $cref cppad_testvector$$ to be $code cppad$$,
$code CPPAD_CPPADVECTOR$$ will be true.
In this case,
$code CPPAD_TESTVECTOR$$ is defined by the following source code:
$srccode%cpp% */
# if CPPAD_CPPADVECTOR
# define CPPAD_TESTVECTOR(Scalar) CppAD::vector< Scalar >
# endif
/* %$$
In this case CppAD will use its own vector for
many of its examples and tests.

$head std::vector$$
If in the cmake command
you specify $icode cppad_testvector$$ to be $code std$$,
$code CPPAD_STDVECTOR$$ will be true.
In this case,
$code CPPAD_TESTVECTOR$$ is defined by the following source code:
$srccode%cpp% */
# if CPPAD_STDVECTOR
# include <vector>
# define CPPAD_TESTVECTOR(Scalar) std::vector< Scalar >
# endif
/* %$$
In this case CppAD will use standard vector for
many of its examples and tests.

$head boost::numeric::ublas::vector$$
If in the cmake command
you specify $icode cppad_testvector$$ to be $code boost$$,
$code CPPAD_BOOSTVECTOR$$ will be true.
In this case,
$code CPPAD_TESTVECTOR$$ is defined by the following source code:
$srccode%cpp% */
# if CPPAD_BOOSTVECTOR
# include <boost/numeric/ublas/vector.hpp>
# define CPPAD_TESTVECTOR(Scalar) boost::numeric::ublas::vector< Scalar >
# endif
/* %$$
In this case CppAD will use this boost vector for
many of its examples and tests.

$head CppAD::eigen_vector$$
If in the cmake command
you specify $icode cppad_testvector$$ to be $code eigen$$,
$code CPPAD_EIGENVECTOR$$ will be true.
In this case,
$code CPPAD_TESTVECTOR$$ is defined by the following source code:
$srccode%cpp% */
# if CPPAD_EIGENVECTOR
# include <cppad/example/cppad_eigen.hpp>
# define CPPAD_TESTVECTOR(Scalar) CppAD::eigen_vector< Scalar >
# endif
/* %$$
see $cref/eigen_vector/cppad_eigen.hpp/eigen_vector/$$.
In this case CppAD will use the Eigen vector
for many of its examples and tests.

$end
------------------------------------------------------------------------
*/

# endif
