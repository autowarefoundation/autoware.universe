# ifndef CPPAD_LOCAL_GRAPH_CPP_GRAPH_OP_HPP
# define CPPAD_LOCAL_GRAPH_CPP_GRAPH_OP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

  CppAD is distributed under the terms of the
               Eclipse Public License Version 2.0.

  This Source Code may also be made available under the following
  Secondary License when the conditions for such availability set forth
  in the Eclipse Public License, Version 2.0 are satisfied:
        GNU General Public License, Version 2.0 or later.
-------------------------------------------------------------------------- */

# include <cstddef>
# include <string>
# include <map>

# include <cppad/utility/vector.hpp>
# include <cppad/configure.hpp>
# include <cppad/core/graph/graph_op_enum.hpp>

namespace CppAD { namespace local { namespace graph {
/*
$begin cpp_graph_op$$
$spell
    vec
    asinh
    acosh
    atanh
    erf
    erfc
    expm
    namespace
    enum
    struct
    op
    arg
    CppAD
    addr_t
$$

$section C++ AD Graph Operators$$

$head Namespace$$
All of these definitions
are in the $code CppAD::local::graph$$ namespace.

$head CppAD::graph$$
$srccode%hpp% */
    using namespace CppAD::graph;
/* %$$

$head addr_t$$
$srccode%hpp% */
    typedef CPPAD_TAPE_ADDR_TYPE addr_t;
/* %$$

$head op_name2enum$$
This is a mapping from the operator name to its enum value.
The name is the operator enum without the $code _operator$$ at the end.
$srccode%hpp% */
    extern CPPAD_LIB_EXPORT std::map< std::string, graph_op_enum > op_name2enum;
/* %$$

$head op_enum2fixed_n_arg$$
This is the number of arguments for the operators that have
a fixed number of arguments and one result.
For other operators, this value is zero.
$srccode%hpp% */
    extern CPPAD_LIB_EXPORT size_t op_enum2fixed_n_arg[];
/* %$$

$head op_enum2name$$
This is mapping from operator enum value to its name.
In the $code local::graph$$ namespace:
$srccode%hpp% */
    extern CPPAD_LIB_EXPORT const char* op_enum2name[];
/* %$$

$head set_operator_info$$
This routine sets the values in
$code op_enum2fixed_n_arg$$,
$code op_enum2name$$, and
$code op_name2enum$$.
$srccode%hpp% */
    extern CPPAD_LIB_EXPORT void set_operator_info(void);
/* %$$
$end
*/

} } } // END_CPPAD_LOCAL_GRAPH_NAMESPACE

# endif
