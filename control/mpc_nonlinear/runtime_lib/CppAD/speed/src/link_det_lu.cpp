/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/utility/vector.hpp>
# include <cppad/speed/det_grad_33.hpp>
# include <cppad/speed/det_33.hpp>
# include <cppad/utility/time_test.hpp>

// BEGIN PROTOTYPE
extern bool link_det_lu(
    size_t                     size      ,
    size_t                     repeat    ,
    CppAD::vector<double>&     matrix    ,
    CppAD::vector<double>&     gradient
);
// END PROTOTYPE
/*
-------------------------------------------------------------------------------
$begin link_det_lu$$
$spell
    det_lu
    bool
    CppAD
$$


$section Speed Testing Gradient of Determinant Using Lu Factorization$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%0
%$$

$head Purpose$$
Each $cref/package/speed_main/package/$$
must define a version of this routine as specified below.
This is used by the $cref speed_main$$ program
to run the corresponding speed and correctness tests.

$head Method$$
The same template routine $cref det_by_lu$$ is used
by the different AD packages.

$head Return Value$$
If this speed test is not yet
supported by a particular $icode package$$,
the corresponding return value for $code link_det_lu$$
should be $code false$$.

$head size$$
The argument $icode size$$
is the number of rows and columns in the matrix.

$head repeat$$
The argument $icode repeat$$ is the number of different matrices
that the gradient (or determinant) is computed for.

$head matrix$$
The argument $icode matrix$$ is a vector with $icode%size%*%size%$$ elements.
The input value of its elements does not matter.
The output value of its elements is the last matrix that the
gradient (or determinant) is computed for.

$head gradient$$
The argument $icode gradient$$ is a vector with $icode%size%*%size%$$ elements.
The input value of its elements does not matter.
The output value of its elements is the gradient of the
determinant of $icode matrix$$ with respect to its elements.

$subhead double$$
In the case where $icode package$$ is $code double$$,
only the first element of $icode gradient$$ is used and it is actually
the determinant value (the gradient value is not computed).

$end
-----------------------------------------------------------------------------
*/
// ---------------------------------------------------------------------------
// The routines below are documented in dev_link.omh
// ---------------------------------------------------------------------------
namespace {
    void time_det_lu_callback(size_t size, size_t repeat)
    {   // free statically allocated memory
        if( size == 0 && repeat == 0 )
            return;
        //
        CppAD::vector<double> matrix(size * size);
        CppAD::vector<double> gradient(size * size);

        link_det_lu(size, repeat, matrix, gradient);
        return;
    }
}
// ---------------------------------------------------------------------------
bool available_det_lu(void)
{   size_t size   = 3;
    size_t repeat = 1;
    CppAD::vector<double> matrix(size * size);
    CppAD::vector<double> gradient(size * size);

    return link_det_lu(size, repeat, matrix, gradient);
}
// ---------------------------------------------------------------------------
bool correct_det_lu(bool is_package_double)
{   size_t size   = 3;
    size_t repeat = 1;
    CppAD::vector<double> matrix(size * size);
    CppAD::vector<double> gradient(size * size);

    link_det_lu(size, repeat, matrix, gradient);
    bool ok;
    if( is_package_double )
        ok = CppAD::det_33(matrix, gradient);
    else
        ok = CppAD::det_grad_33(matrix, gradient);
    return ok;
}
// ---------------------------------------------------------------------------
double time_det_lu(double time_min, size_t size)
{   return CppAD::time_test(time_det_lu_callback, time_min, size);
}
