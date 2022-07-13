# ifndef CPPAD_UTILITY_SPARSE2EIGEN_HPP
# define CPPAD_UTILITY_SPARSE2EIGEN_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin sparse2eigen$$
$spell
    CppAD
    Eigen
    cppad.hpp
    const
    Ptr
    nnz
    cmake
    namespace
$$

$section Convert A CppAD Sparse Matrix to an Eigen Sparse Matrix$$

$head Syntax$$
$codei%# include <cppad/utility/sparse2eigen.hpp>
%$$
$codei%sparse2eigen(%source%, %destination%)%$$

$head Prototype$$
$srcthisfile%0
    %// BEGIN_PROTOTYPE%// END_PROTOTYPE%
1%$$

$head Include$$
If $cref/include_eigen/cmake/include_eigen/$$ is specified on the cmake command line,
the file $code cppad/utility/sparse2eigen.hpp$$
is included by $code cppad/cppad.hpp$$.
In any case,
it can also be included separately with out the rest of
the $code CppAD$$ routines.
Including this file defines
this version of the $code sparse2eigen$$ within the $code CppAD$$ namespace.

$head SizeVector$$
We use $cref/SizeVector/sparse_rc/SizeVector/$$ to denote a
$cref SimpleVector$$ class with elements of $code size_t$$.

$head ValueVector$$
We use $icode ValueVector$$ to denote a
$cref SimpleVector$$ class with elements of type $icode value_type$$.

$head Options$$
We use $icode Options$$ to denote either
$code Eigen::RowMajor$$ of $code Eigen::ColMajor$$.

$head value_type$$
The type of elements of elements in $icode source$$ and $icode destination$$
must be the same. We use $icode value_type$$ to denote this type.

$head source$$
This is the CppAD sparse matrix that is being converted to eigen format.

$head destination$$
This is the Eigen sparse matrix that is the result of the conversion.

$head Compressed$$
The result matrix $icode destination$$
is in compressed format. For example, let
$codei%
    size_t %%           %nnz%       = %source%.nnz();
    const %s_vector%&   %s_value%   = %source%.val();
    const %value_type%* %d_value%   = %destination%.valuePtr();
    const %s_vector%&   %row_major% = %source%.row_major();
    const %s_vector%&   %col_major% = %source%.col_major();
%$$
It follows that, for $icode%k% = 0 , %...%, %nnz%$$:
If $icode Options$$ is $code Eigen::RowMajor$$,
$codei%
    %d_value%[%k%] == %s_value%[ %row_major%[%k%] ]
%$$
If $icode Options$$ is $code Eigen::ColMajor$$,
$codei%
    %d_value%[%k%] == %s_value%[ %col_major%[%k%] ]
%$$

$children%example/sparse/sparse2eigen.cpp
%$$

$head Example$$
The file $cref sparse2eigen.cpp$$ contains an example and test
of $code sparse2eigen.cpp$$ It return true if the test passes
and false otherwise.

$end
*/
# include <cppad/configure.hpp>
# include <Eigen/Sparse>
# include <cppad/utility/sparse_rcv.hpp>
# include <cppad/utility/vector.hpp>

namespace CppAD { // BEGIN CPPAD_NAMESPACE

// BEGIN_PROTOTYPE
template <class SizeVector, class ValueVector, int Options>
void sparse2eigen(
const CppAD::sparse_rcv<SizeVector, ValueVector>&               source       ,
Eigen::SparseMatrix<typename ValueVector::value_type, Options>& destination  )
// END_PROTOTYPE
{   using Eigen::Index;
    typedef typename ValueVector::value_type value_type;
    typedef Eigen::Triplet<value_type>       triplet;
    std::vector<triplet> vec( source.nnz() );
    //
    const SizeVector&  row = source.row();
    const SizeVector&  col = source.col();
    const ValueVector& val = source.val();
    //
    for(size_t k = 0; k < source.nnz(); k++)
        vec[k] = triplet( int(row[k]), int(col[k]), val[k] );
    //
    size_t nr = source.nr();
    size_t nc = source.nc();
    destination.resize( Index(nr), Index(nc) );
    destination.setFromTriplets(vec.begin(), vec.end());
    //
    CPPAD_ASSERT_UNKNOWN( destination.isCompressed() );
    //
    return;
}

} // END_CPPAD_NAMESPACE
# endif
