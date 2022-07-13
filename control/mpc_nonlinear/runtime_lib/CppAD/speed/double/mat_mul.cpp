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
$begin double_mat_mul.cpp$$
$spell
    onetape
    CppAD
    cppad
    mul_mat
    hpp
    sq
    bool
    dz
    typedef
$$

$section Double Speed: Matrix Multiplication$$


$head Specifications$$
See $cref link_mat_mul$$.

$head Implementation$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
# include <cppad/speed/mat_sum_sq.hpp>
# include <cppad/speed/uniform_01.hpp>

// Note that CppAD uses global_option["memory"] at the main program level
# include <map>
extern std::map<std::string, bool> global_option;

bool link_mat_mul(
    size_t                           size     ,
    size_t                           repeat   ,
    CppAD::vector<double>&           x        ,
    CppAD::vector<double>&           z        ,
    CppAD::vector<double>&           dz
)
{
    if(global_option["onetape"]||global_option["atomic"]||global_option["optimize"])
        return false;
    // -----------------------------------------------------
    size_t n = size * size; // number of independent variables
    CppAD::vector<double> y(n);

    while(repeat--)
    {   // get the next matrix
        CppAD::uniform_01(n, x);

        // do computation
        mat_sum_sq(size, x, y, z);

    }
    return true;
}
/* %$$
$end
*/
