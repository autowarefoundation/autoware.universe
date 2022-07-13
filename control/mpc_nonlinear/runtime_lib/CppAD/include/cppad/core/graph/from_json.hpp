# ifndef CPPAD_CORE_GRAPH_FROM_JSON_HPP
# define CPPAD_CORE_GRAPH_FROM_JSON_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/core/ad_fun.hpp>
# include <cppad/core/ad_type.hpp>
# include <cppad/local/graph/json_parser.hpp>

/*
$begin from_json$$
$spell
    Json
$$

$section ADFun Object Corresponding to a Json AD Graph$$

$head Syntax$$
$codei%
    ADFun<%Base%> %fun%
    %fun%.from_json(%json%)
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
%$$

$head json$$
is a $cref json_ad_graph$$.

$head Base$$
is the type corresponding to this $cref/ADFun/adfun/$$ object;
i.e., its calculations are done using the type $icode Base$$.

$head RecBase$$
in the prototype above, $icode RecBase$$ is the same type as $icode Base$$.

$children%
    example/json/from_json.cpp
%$$
$head Example$$
The file $cref from_json.cpp$$ is an example and test of this operation.

$end
*/
// BEGIN_PROTOTYPE
template <class Base, class RecBase>
void CppAD::ADFun<Base,RecBase>::from_json(const std::string& json)
// END_PROTOTYPE
{
    using CppAD::isnan;
    //
    //
    // C++ graph object
    cpp_graph graph_obj;
    //
    // convert json to graph representation
    local::graph::json_parser(json, graph_obj);
    //
    // convert the graph representation to a function
    from_graph(graph_obj);
    //
    return;
}

# endif
