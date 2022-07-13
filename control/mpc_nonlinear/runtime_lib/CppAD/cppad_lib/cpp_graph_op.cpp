/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

  CppAD is distributed under the terms of the
               Eclipse Public License Version 2.0.

  This Source Code may also be made available under the following
  Secondary License when the conditions for such availability set forth
  in the Eclipse Public License, Version 2.0 are satisfied:
        GNU General Public License, Version 2.0 or later.
-------------------------------------------------------------------------- */
# include <cppad/core/cppad_assert.hpp>
# include <utility>

// documentations for this routintes are in the file below
# include <cppad/local/graph/cpp_graph_op.hpp>

// BEGIN_CPPAD_LOCAL_GRAPH_NAMESPACE
namespace CppAD { namespace local { namespace graph {

// mapping from operator name to graph_op_enum value
std::map<std::string, graph_op_enum> op_name2enum;

// map from operator enum to name
const char* op_enum2name[n_graph_op];

// map from operator enum to n_arg (when fixed number of arguments)
size_t op_enum2fixed_n_arg[n_graph_op];

// This routine is called by the first use of the cpp_graph constructor
// see cpp_grpah.hpp.
void set_operator_info(void)
{   // This routine cannot be called in parallel mode
    CPPAD_ASSERT_UNKNOWN( ! ( CppAD::thread_alloc::in_parallel() ));
    //
    typedef std::pair<std::string, graph_op_enum> pair;
    struct op_info {
        graph_op_enum code;
        const char*  name;
        size_t       n_arg;
    };
    /*
    If n_arg is zero in the table below, the table contains a comment as to
    the heading in graph_op_enum.hpp below which you can find the
    specifications for n_arg, n_result for the corresponding operator.
    */
    // BEGIN_SORT_THIS_LINE_PLUS_2
    op_info op_info_vec[] = {
        { abs_graph_op,      "abs",      1 }, // 1 result
        { acos_graph_op,     "acos",     1 }, // 1 result
        { acosh_graph_op,    "acosh",    1 }, // 1 result
        { add_graph_op,      "add",      2 }, // 1 result
        { asin_graph_op,     "asin",     1 }, // 1 result
        { asinh_graph_op,    "asinh",    1 }, // 1 result
        { atan_graph_op,     "atan",     1 }, // 1 result
        { atanh_graph_op,    "atanh",    1 }, // 1 result
        { atom_graph_op,     "atom",     0 }, // See Atomic Function
        { azmul_graph_op,    "azmul",    2 }, // 1 result
        { cexp_eq_graph_op,  "cexp_eq",  4 }, // 1 result
        { cexp_le_graph_op,  "cexp_le",  4 }, // 1 result
        { cexp_lt_graph_op,  "cexp_lt",  4 }, // 1 result
        { comp_eq_graph_op,  "comp_eq",  0 }, // See Comparisons
        { comp_le_graph_op,  "comp_le",  0 }, // ...
        { comp_lt_graph_op,  "comp_lt",  0 }, // ...
        { comp_ne_graph_op,  "comp_ne",  0 }, // ...
        { cos_graph_op,      "cos",      1 }, // 1 result
        { cosh_graph_op,     "cosh",     1 }, // 1 result
        { discrete_graph_op, "discrete", 0 }, // See Discrete Function
        { div_graph_op,      "div",      2 }, // 1 result
        { erf_graph_op,      "erf",      1 }, // 1 result
        { erfc_graph_op,     "erfc",     1 }, // 1 result
        { exp_graph_op,      "exp",      1 }, // 1 result
        { expm1_graph_op,    "expm1",    1 }, // 1 result
        { log1p_graph_op,    "log1p",    1 }, // 1 result
        { log_graph_op,      "log",      1 }, // 1 result
        { neg_graph_op,      "neg",      1 }, // 1 result
        { mul_graph_op,      "mul",      2 }, // 1 result
        { pow_graph_op,      "pow",      2 }, // 1 result
        { print_graph_op,    "print",    0 }, // See Print
        { sign_graph_op,     "sign",     1 }, // 1 result
        { sin_graph_op,      "sin",      1 }, // 1 result
        { sinh_graph_op,     "sinh",     1 }, // 1 result
        { sqrt_graph_op,     "sqrt",     1 }, // 1 result
        { sub_graph_op,      "sub",      2 }, // 1 result
        { sum_graph_op,      "sum",      0 }, // See Summation
        { tan_graph_op,      "tan",      1 }, // 1 result
        { tanh_graph_op,     "tanh",     1 }  // 1 result
    };
    // END_SORT_THIS_LINE_MINUS_2
    CPPAD_ASSERT_UNKNOWN(
        size_t(n_graph_op) == sizeof(op_info_vec) / sizeof(op_info_vec[0])
    );
    for(size_t i = 0; i < size_t(n_graph_op); ++i)
    {   graph_op_enum code              = op_info_vec[i].code;
        const char*  name              = op_info_vec[i].name;
        size_t       n_arg             = op_info_vec[i].n_arg;
        CPPAD_ASSERT_UNKNOWN( size_t(code) == i );
        //
        op_enum2name[code]        = name;
        op_enum2fixed_n_arg[code] = n_arg;
        op_name2enum.insert( pair(name, code) );
    }
}

} } } // END_CPPAD_LOCAL_GRAPH_NAMESPACE
