# ifndef CPPAD_LOCAL_OPTIMIZE_CSUM_OP_INFO_HPP
# define CPPAD_LOCAL_OPTIMIZE_CSUM_OP_INFO_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/op_code_var.hpp>
# include <cppad/local/declare_ad.hpp> // defines addr_t

/*!
\file csum_op_info.hpp
Information about one old variable that is part of a new CSumOp operation.
*/

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize  {
/*!
Information about one old variable that is part of a new CSumOp operation.
*/
struct struct_csum_op_info {
    /// Pointer to first argument (child) for this old operator.
    /// Set by the reverse sweep at beginning of optimization.
    const addr_t*       arg;

    /// Was this old variable added to the summation
    /// (if not it was subtracted)
    bool                add;

    /// Operator for which this old variable is the result, NumRes(op) > 0.
    OpCode              op;
};

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
