/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin adolc_alloc_mat$$
$spell
    adolc
    alloc
$$

$section Adolc Test Utility: Allocate and Free Memory For a Matrix$$

$head Syntax$$
$codei%mat% = adolc_alloc_mat(%m%, %n%)
%$$
$codei%adolc_free_mat(%mat%)
%$$

$head Purpose$$
Use the $cref thread_alloc$$ memory allocator to allocate and free
memory that can be used as a matrix with the Adolc package.

$head m$$
Is the number of rows in the matrix.

$head n$$
Is the number of columns in the matrix.

$head mat$$
Is the matrix.
To be specific,
between a call to $code adolc_alloc_mat$$,
and the corresponding call to $code adolc_free_mat$$,
for $icode%i% = 0 , %...% , %m%-1%$$
and $icode%j% = 0 , %...% , %n%-1%$$,
$icode%mat%[%i%][%j%]%$$ is the element in row $icode i$$
and column $icode j$$.

$end
*/
# include <cppad/utility/thread_alloc.hpp>

double** adolc_alloc_mat(size_t m, size_t n)
{   using CppAD::thread_alloc;
    size_t size_min = m * n, size_out;
    double*  vec = thread_alloc::create_array<double>(size_min, size_out);
    double** mat = thread_alloc::create_array<double*>(size_min, size_out);

    for(size_t i = 0; i < m; i++)
        mat[i] = vec + i * n;

    return mat;
}
void adolc_free_mat(double** mat)
{   using CppAD::thread_alloc;
    thread_alloc::delete_array(mat[0]);
    thread_alloc::delete_array(mat);
    return;
}
