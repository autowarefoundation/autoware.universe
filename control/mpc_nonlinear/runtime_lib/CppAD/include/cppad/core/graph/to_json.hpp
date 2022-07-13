# ifndef CPPAD_CORE_GRAPH_TO_JSON_HPP
# define CPPAD_CORE_GRAPH_TO_JSON_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/core/ad_fun.hpp>
# include <cppad/local/op_code_dyn.hpp>
# include <cppad/local/graph/cpp_graph_op.hpp>
# include <cppad/core/graph/cpp_graph.hpp>
# include <cppad/local/graph/json_writer.hpp>

/*
------------------------------------------------------------------------------
$begin to_json$$
$spell
    Json
    cpp
$$

$section Json AD Graph Corresponding to an ADFun Object$$

$head Syntax$$
$codei%
    %json% = %fun%.to_json()
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
%$$

$head fun$$
is the $cref/ADFun/adfun/$$ object.

$head json$$
The return value of $icode json$$ is a
$cref json_ad_graph$$ representation of the corresponding function.

$head Base$$
is the type corresponding to this $cref/ADFun/adfun/$$ object;
i.e., its calculations are done using the type $icode Base$$.

$head RecBase$$
in the prototype above, $icode RecBase$$ is the same type as $icode Base$$.

$head Restrictions$$
The $code to_json$$ routine is not yet implement for some
possible $cref ADFun$$ operators; see
$cref/missing operators/graph_op_enum/Missing Operators/$$.

$children%
    example/json/to_json.cpp
%$$
$head Example$$
The file $cref to_json.cpp$$ is an example and test of this operation.

$end
*/
// BEGIN_PROTOTYPE
template <class Base, class RecBase>
std::string CppAD::ADFun<Base,RecBase>::to_json(void)
// END_PROTOTYPE
{   using local::pod_vector;
    using local::opcode_t;
    //
    // to_graph return values
    cpp_graph graph_obj;
    //
    // graph corresponding to this function
    to_graph(graph_obj);
    //
    // convert to json
    std::string json;
    local::graph::json_writer(json, graph_obj);
    //
    return json;
}

# endif
