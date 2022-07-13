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

/*
$begin fadbad_sparse_hessian.cpp$$
$spell
    const
    Fadbad
    bool
    CppAD
$$

$section Fadbad Speed: Sparse Hessian$$

$srccode%cpp% */
// A fadbad version of this test is not yet available
bool link_sparse_hessian(
    size_t                            size      ,
    size_t                            repeat    ,
    const CppAD::vector<size_t>&      row       ,
    const CppAD::vector<size_t>&      col       ,
    CppAD::vector<double>&            x         ,
    CppAD::vector<double>&            hessian   ,
    size_t&                           n_color
)
{
    return false;
}
/* %$$
$end
*/
