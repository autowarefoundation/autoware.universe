/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin double_poly.cpp$$
$spell
    onetape
    retape
    bool
    cppad
    CppAD
    hpp
$$

$section Double Speed: Evaluate a Polynomial$$



$head Specifications$$
See $cref link_poly$$.

$head Implementation$$

$srccode%cpp% */
# include <cppad/cppad.hpp>
# include <cppad/speed/uniform_01.hpp>

// Note that CppAD uses global_option["memory"] at the main program level
# include <map>
extern std::map<std::string, bool> global_option;

bool link_poly(
    size_t                     size     ,
    size_t                     repeat   ,
    CppAD::vector<double>     &a        ,  // coefficients of polynomial
    CppAD::vector<double>     &z        ,  // polynomial argument value
    CppAD::vector<double>     &p        )  // second derivative w.r.t z
{
    if(global_option["onetape"]||global_option["atomic"]||global_option["optimize"])
        return false;
    // -----------------------------------------------------
    // setup

    // ------------------------------------------------------
    while(repeat--)
    {   // get the next argument value
        CppAD::uniform_01(1, z);

        // evaluate the polynomial at the new argument value
        p[0] = CppAD::Poly(0, a, z[0]);
    }
    return true;
}
/* %$$
$end
*/
