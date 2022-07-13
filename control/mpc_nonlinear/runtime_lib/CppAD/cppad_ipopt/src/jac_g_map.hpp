# ifndef CPPAD_CPPAD_IPOPT_SRC_JAC_G_MAP_HPP
# define CPPAD_CPPAD_IPOPT_SRC_JAC_G_MAP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include "cppad_ipopt_nlp.hpp"
/*!
\file jac_g_map.hpp
\brief Create a mapping between two representations for Jacobian of g.

\ingroup jac_g_map_cpp
*/
// ---------------------------------------------------------------------------
namespace cppad_ipopt {
// ---------------------------------------------------------------------------


extern void jac_g_map(
    cppad_ipopt_fg_info*  fg_info                                  ,
    size_t                                          m              ,
    size_t                                          n              ,
    size_t                                          K              ,
    const CppAD::vector<size_t>&                    L              ,
    const CppAD::vector<size_t>&                    p              ,
    const CppAD::vector<size_t>&                    q              ,
    const CppAD::vector<CppAD::vectorBool>&         pattern_jac_r  ,
    CppAD::vector<size_t>&                          I              ,
    CppAD::vector<size_t>&                          J              ,
    CppAD::vector< std::map<size_t,size_t> >&       index_jac_g
);

// ---------------------------------------------------------------------------
} // end namespace cppad_ipopt
// ---------------------------------------------------------------------------

# endif
