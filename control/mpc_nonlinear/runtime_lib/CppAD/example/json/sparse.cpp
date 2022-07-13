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
$begin json_sparse.cpp$$
$spell
    CppAD
    Json
$$

$section Json Representation of a Sparse Matrix: Example and Test$$

$head Discussion$$
The example using a CppAD Json to represent the sparse matrix
$latex \[
\partial_x f(x, p) = \left( \begin{array}{ccc}
    p_0 & 0    & 0 \\
    0   & p_1  & 0 \\
    0   & 0    & c_0
\end{array} \right)
\] $$
where $latex c_0$$ is the constant 3.

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

bool sparse(void)
{   bool ok = true;
    using CppAD::vector;
    typedef vector<size_t> s_vector;
    typedef vector<double> d_vector;
    //
    // An AD graph example
    // node_1 : p[0]
    // node_2 : p[1]
    // node_3 : x[0]
    // node_4 : x[1]
    // node_5 : x[2]
    // node_6 : c[0]
    // node_7 : p[0] * x[0]
    // node_8 : p[1] * x[1]
    // node_9 : c[0] * x[2]
    // y[0]   = p[0] * x[0]
    // y[1]   = p[1] * x[1]
    // y[2]   = c[0] * x[0]
    // use single quote to avoid having to escape double quote
    std::string json =
        "{\n"
        "   'function_name'  : 'sparse example',\n"
        "   'op_define_vec'  : [ 1, [\n"
        "       { 'op_code':1, 'name':'mul', 'n_arg':2 } ]\n"
        "   ],\n"
        "   'n_dynamic_ind'  : 2,\n"
        "   'n_variable_ind' : 3,\n"
        "   'constant_vec'   : [ 1, [ 3.0 ] ],\n"
        "   'op_usage_vec'   : [ 3, [\n"
        "       [ 1, 1, 3 ] , \n"
        "       [ 1, 2, 4 ] , \n"
        "       [ 1, 6, 5 ] ] \n"
        "   ],\n"
        "   'dependent_vec'   : [ 3, [7, 8, 9] ] \n"
        "}\n";
    // Convert the single quote to double quote
    for(size_t i = 0; i < json.size(); ++i)
        if( json[i] == '\'' ) json[i] = '"';
    //
    CppAD::ADFun<double> fun;
    fun.from_json(json);
    ok &= fun.Domain() == 3;
    ok &= fun.Range()  == 3;
    ok &= fun.size_dyn_ind() == 2;
    //
    // Point at which we are going to evaluate the Jacobian of f(x, p).
    // The Jacobian is constant w.r.t. x, so the value of x does not matter.
    vector<double> p(2), x(3);
    p[0] = 1.0;
    p[1] = 2.0;
    for(size_t j = 0; j < x.size(); ++j)
        x[j] = 0.0;
    //
    // set the dynamic parameters
    fun.new_dynamic(p);
    //
    // compute the sparsity pattern for f_x(x, p)
    vector<bool> select_domain(3);
    vector<bool> select_range(3);
    for(size_t i = 0; i < 3; ++i)
    {   select_domain[i] = true;
        select_range[i]  = true;
    }
    bool transpose = false;
    CppAD::sparse_rc<s_vector> pattern;
    fun.subgraph_sparsity(select_domain, select_range, transpose, pattern);
    //
    // compute the entire Jacobian
    CppAD::sparse_rcv<s_vector, d_vector> subset(pattern);
    fun.subgraph_jac_rev(x, subset);
    //
    // information in the sparse Jacobian
    const s_vector& row( subset.row() );
    const s_vector& col( subset.col() );
    const d_vector& val( subset.val() );
    size_t   nnz       = subset.nnz();
    s_vector row_major = subset.row_major();
    //
    // check number of non-zero elements in sparse matrix
    ok      &= nnz == 3;
    //
    // check first element of matrix (in row major order)
    size_t k = row_major[0];
    ok      &= row[k] == 0;
    ok      &= col[k] == 0;
    ok      &= val[k] == p[0];
    //
    // check second element of matrix
    k        = row_major[1];
    ok      &= row[k] == 1;
    ok      &= col[k] == 1;
    ok      &= val[k] == p[1];
    //
    // check third element of matrix
    k        = row_major[2];
    ok      &= row[k] == 2;
    ok      &= col[k] == 2;
    ok      &= val[k] == 3.0;
    //
    return ok;
}
// END C++
