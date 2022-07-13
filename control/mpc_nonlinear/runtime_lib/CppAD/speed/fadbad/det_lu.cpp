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
$begin fadbad_det_lu.cpp$$
$spell
    onetape
    cppad
    std
    Lu
    Fadbad
    det
    badiff.hpp
    const
    CppAD
    typedef
    diff
    bool
    srand
$$

$section Fadbad Speed: Gradient of Determinant Using Lu Factorization$$


$head Specifications$$
See $cref link_det_lu$$.

$head Implementation$$
$srccode%cpp% */
// suppress conversion warnings before other includes
# include <cppad/wno_conversion.hpp>
//
# include <FADBAD++/badiff.h>
# include <cppad/speed/det_by_lu.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/utility/vector.hpp>

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
    //
    // object for computing determinant
    typedef fadbad::B<double>       ADScalar;
    typedef CppAD::vector<ADScalar> ADVector;
    CppAD::det_by_lu<ADScalar>      Det(size);

    size_t i;                // temporary index
    size_t m = 1;            // number of dependent variables
    size_t n = size * size;  // number of independent variables
    ADScalar   detA;         // AD value of the determinant
    ADVector   A(n);         // AD version of matrix

    // ------------------------------------------------------
    while(repeat--)
    {   // get the next matrix
        CppAD::uniform_01(n, matrix);

        // set independent variable values
        for(i = 0; i < n; i++)
            A[i] = matrix[i];

        // compute the determinant
        detA = Det(A);

        // create function object f : A -> detA
        detA.diff(0, (unsigned int) m);  // index 0 of m dependent variables

        // evaluate and return gradient using reverse mode
        for(i =0; i < n; i++)
            gradient[i] = A[i].d(0); // partial detA w.r.t A[i]
    }
    // ---------------------------------------------------------
    return true;
}
/* %$$
$end
*/
