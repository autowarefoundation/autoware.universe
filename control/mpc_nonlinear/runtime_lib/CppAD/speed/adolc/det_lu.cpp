/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin adolc_det_lu.cpp$$
$spell
    onetape
    thread_alloc
    cppad
    fos
    adouble
    CppAD
    typedef
    adolc
    Lu
    Adolc
    det
    hpp
    const
    bool
    srand
$$

$section Adolc Speed: Gradient of Determinant Using Lu Factorization$$


$head Specifications$$
See $cref link_det_lu$$.

$head Implementation$$
$srccode%cpp% */
// suppress conversion warnings before other includes
# include <cppad/wno_conversion.hpp>
//
# include <adolc/adolc.h>

# include <cppad/speed/det_by_lu.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/utility/track_new_del.hpp>

// list of possible options
# include <map>
extern std::map<std::string, bool> global_option;

bool link_det_lu(
    size_t                     size     ,
    size_t                     repeat   ,
    CppAD::vector<double>     &matrix   ,
    CppAD::vector<double>     &gradient )
{
    // speed test global option values
    if( global_option["onetape"] || global_option["atomic"] )
        return false;
    if( global_option["memory"] || global_option["optimize"] )
        return false;
    // -----------------------------------------------------
    // setup
    int tag  = 0;         // tape identifier
    int keep = 1;         // keep forward mode results in buffer
    int m    = 1;         // number of dependent variables
    int n    = size*size; // number of independent variables
    double f;             // function value
    int j;                // temporary index

    // set up for thread_alloc memory allocator (fast and checks for leaks)
    using CppAD::thread_alloc; // the allocator
    size_t size_min;           // requested number of elements
    size_t size_out;           // capacity of an allocation

    // object for computing determinant
    typedef adouble            ADScalar;
    typedef ADScalar*          ADVector;
    CppAD::det_by_lu<ADScalar> Det(size);

    // AD value of determinant
    ADScalar   detA;

    // AD version of matrix
    size_min    = n;
    ADVector A  = thread_alloc::create_array<ADScalar>(size_min, size_out);

    // vectors of reverse mode weights
    size_min    = m;
    double* u   = thread_alloc::create_array<double>(size_min, size_out);
    u[0] = 1.;

    // vector with matrix value
    size_min     = n;
    double* mat  = thread_alloc::create_array<double>(size_min, size_out);

    // vector to receive gradient result
    size_min     = n;
    double* grad = thread_alloc::create_array<double>(size_min, size_out);
    // ------------------------------------------------------
    while(repeat--)
    {   // get the next matrix
        CppAD::uniform_01(n, mat);

        // declare independent variables
        trace_on(tag, keep);
        for(j = 0; j < n; j++)
            A[j] <<= mat[j];

        // AD computation of the determinant
        detA = Det(A);

        // create function object f : A -> detA
        detA >>= f;
        trace_off();

        // evaluate and return gradient using reverse mode
        fos_reverse(tag, m, n, u, grad);
    }
    // ------------------------------------------------------

    // return matrix and gradient
    for(j = 0; j < n; j++)
    {   matrix[j] = mat[j];
        gradient[j] = grad[j];
    }
    // tear down
    thread_alloc::delete_array(grad);
    thread_alloc::delete_array(mat);
    thread_alloc::delete_array(u);
    thread_alloc::delete_array(A);

    return true;
}
/* %$$
$end
*/
