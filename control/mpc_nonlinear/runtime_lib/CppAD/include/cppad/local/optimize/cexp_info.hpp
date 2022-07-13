# ifndef CPPAD_LOCAL_OPTIMIZE_CEXP_INFO_HPP
# define CPPAD_LOCAL_OPTIMIZE_CEXP_INFO_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/declare_ad.hpp> // defines CompareOp
# include <cppad/utility/vector.hpp>

/*!
$begin optimize_cexp_info$$
$spell
    struct
    cexp
    op
    Funap
    Funav
    Funrp
    Funrv
    cskip
    arg
$$

$section Optimization Information About Conditional Expressions$$

$head struct_cexp_info$$
information about a conditional expression
in the old operation sequence (before optimization).
$srcthisfile%
    0%// BEGIN_STRUCT_CEXP_INFO%// END_STRUCT_CEXP_INFO%1
%$$

$subhead i_op$$
is the operator index for this conditional expression.

$subhead left$$
is the variable or parameter index (depending on flag)
for left operand in the comparison.

$subhead right$$
is the variable or parameter index (depending on flag)
for right operand in the comparison.

$subhead max_left_right$$
is the maximum of the left and right variable indices.
This is a variable index, so parameters correspond to index zero.

$subhead cop$$
is the comparison operator for this conditional expression.

$subhead flag$$
$list number$$
(flag & 1) is true if and only if left is a variable
$lnext
(flag & 2) is true if and only if right is a variable
$lend

$head struct_cskip_new$$
information about a conditional expression
in thew new operation sequence (after optimization).
$srcthisfile%
    0%// BEGIN_STRUCT_CSKIP_NEW%// END_STRUCT_CSKIP_NEW%1
%$$

$subhead left$$
is the variable or parameter index (depending on flag)
for left operand in the comparison.

$subhead right$$
is the variable or parameter index (depending on flag)
for right operand in the comparison.

$subhead max_left_right$$
is the maximum of the left and right variable indices.
This is a variable index, so parameters correspond to index zero.

$subhead i_arg$$
index where this conditional skips arguments start
(in the vector or arguments for all operators).

$end
*/

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize  {
/*!
Information about one conditional expression.
*/
// BEGIN_STRUCT_CEXP_INFO
struct struct_cexp_info {
    addr_t                i_op;
    addr_t                left;
    addr_t                right;
    addr_t                max_left_right;
    CompareOp             cop;
    unsigned char         flag;
};
// END_STRUCT_CEXP_INFO

// BEGIN_STRUCT_CSKIP_NEW
struct struct_cskip_new {
    size_t left;
    size_t right;
    size_t max_left_right;
    size_t i_arg;
};
// END_STRUCT_CSKIP_NEW

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

namespace CppAD { namespace local {
    template <> inline bool is_pod<optimize::struct_cskip_new>(void)
    { return true; }
} }

# endif
