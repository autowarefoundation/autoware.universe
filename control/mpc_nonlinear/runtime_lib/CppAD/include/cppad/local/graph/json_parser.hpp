# ifndef CPPAD_LOCAL_GRAPH_JSON_PARSER_HPP
# define CPPAD_LOCAL_GRAPH_JSON_PARSER_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

  CppAD is distributed under the terms of the
               Eclipse Public License Version 2.0.

  This Source Code may also be made available under the following
  Secondary License when the conditions for such availability set forth
  in the Eclipse Public License, Version 2.0 are satisfied:
        GNU General Public License, Version 2.0 or later.
-------------------------------------------------------------------------- */

# include <string>
# include <cppad/utility/vector.hpp>
# include <cppad/local/graph/cpp_graph_op.hpp>
# include <cppad/core/graph/cpp_graph.hpp>

/*
$begin json_parser$$
$spell
    Json
    CppAD
    obj
$$

$section Json AD Graph Parser$$

$head Syntax$$
$codei%json_parser(%json%, %graph_obj%)%$$

$head json$$
The $cref json_ad_graph$$.

$head graph_obj$$
This is a $code cpp_graph$$ object.
The input value of the object does not matter.
Upon return it is a $cref cpp_ad_graph$$ representation of this function.

$head Prototype$$
$srccode%hpp% */
namespace CppAD { namespace local { namespace graph {
    CPPAD_LIB_EXPORT void json_parser(
        const std::string&  json      ,
        cpp_graph&          graph_obj
    );
} } }
/* %$$
$end
*/


# endif
