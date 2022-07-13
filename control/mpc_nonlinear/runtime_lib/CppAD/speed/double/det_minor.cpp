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
$begin double_det_minor.cpp$$
$spell
    onetape
    retape
    bool
    cppad
    det
    CppAD
    hpp
$$

$section Double Speed: Determinant by Minor Expansion$$


$head Specifications$$
See $cref link_det_minor$$.

$head Implementation$$

$srccode%cpp% */
# include <cppad/utility/vector.hpp>
# include <cppad/speed/det_by_minor.hpp>
# include <cppad/speed/uniform_01.hpp>

// Note that CppAD uses global_option["memory"] at the main program level
# include <map>
extern std::map<std::string, bool> global_option;

bool link_det_minor(
    const std::string&         job      ,
    size_t                     size     ,
    size_t                     repeat   ,
    CppAD::vector<double>     &matrix   ,
    CppAD::vector<double>     &det      )
{   // --------------------------------------------------------------------------
    // ignore global_option
    // --------------------------------------------------------------------------
    if( job == "setup" || job == "teardown" )
        return true;
    CPPAD_ASSERT_UNKNOWN( job == "run" );
    //
    // setup
    CppAD::det_by_minor<double>   Det(size);
    //
    // number of independent variables
    size_t n = size * size;
    // -------------------------------------------------------------------------
    while(repeat--)
    {   // get the next matrix
        CppAD::uniform_01(n, matrix);

        // computation of the determinant
        det[0] = Det(matrix);
    }
    return true;
}
/* %$$
$end
*/
