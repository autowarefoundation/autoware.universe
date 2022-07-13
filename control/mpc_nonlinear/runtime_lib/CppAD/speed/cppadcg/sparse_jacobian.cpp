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
$begin cppadcg_sparse_jacobian.cpp$$
$spell
    cppadcg
    jacobian
    Cpp
$$

$section Cppadcg Speed: Sparse Jacobian$$

$head Specifications$$
See $cref link_sparse_jacobian$$.

$head PASS_SPARSE_JACOBIAN_TO_CODE_GEN$$
If this is one, the sparse Jacobian is the is the function passed
to CppADCodeGen, In this case, the $code code_gen_fun$$
$cref/function/code_gen_fun/Syntax/function/$$ is used to calculate
the sparse Jacobian.
Otherwise, this flag is zero and the original function passed
to CppADCodeGen. In this case, the $code code_gen_fun$$
$cref/sparse_jacobian/code_gen_fun/Syntax/sparse_jacobian/$$
is used to calculate the sparse Jacobian.
$srccode%cpp% */
# define PASS_SPARSE_JACOBIAN_TO_CODE_GEN 1
/* %$$


$head Implementation$$
$srccode%cpp% */
# include <cppad/speed/uniform_01.hpp>
# include <cppad/utility/vector.hpp>
# include <cppad/speed/sparse_jac_fun.hpp>
# include <cppad/example/code_gen_fun.hpp>

# include <map>
extern std::map<std::string, bool> global_option;

namespace {
    // -----------------------------------------------------------------------
    // typedefs
    typedef CppAD::cg::CG<double>       c_double;
    typedef CppAD::AD<c_double>        ac_double;
    typedef CppAD::vector<bool>         b_vector;
    typedef CppAD::vector<size_t>       s_vector;
    typedef CppAD::vector<double>       d_vector;
    typedef CppAD::vector<ac_double>   ac_vector;
    typedef CppAD::sparse_rc<s_vector> sparsity;
    // ------------------------------------------------------------------------
# if PASS_SPARSE_JACOBIAN_TO_CODE_GEN
    // calc_sparsity
    void calc_sparsity(
        CppAD::sparse_rc<s_vector>& pattern ,
        CppAD::ADFun<c_double>&     f       )
    {   bool reverse       = global_option["revsparsity"];
        bool transpose     = false;
        bool internal_bool = global_option["boolsparsity"];
        bool dependency    = false;
        bool subgraph      = global_option["subsparsity"];
        size_t n = f.Domain();
        size_t m = f.Range();
        if( subgraph )
        {   b_vector select_domain(n), select_range(m);
            for(size_t j = 0; j < n; ++j)
                select_domain[j] = true;
            for(size_t i = 0; i < m; ++i)
                select_range[i] = true;
            f.subgraph_sparsity(
                select_domain, select_range, transpose, pattern
            );
        }
        else
        {   size_t q = n;
            if( reverse )
                q = m;
            //
            CppAD::sparse_rc<s_vector> identity;
            identity.resize(q, q, q);
            for(size_t k = 0; k < q; k++)
                identity.set(k, k, k);
            //
            if( reverse )
            {   f.rev_jac_sparsity(
                    identity, transpose, dependency, internal_bool, pattern
                );
            }
            else
            {   f.for_jac_sparsity(
                    identity, transpose, dependency, internal_bool, pattern
                );
            }
        }
    }
# endif // PASS_SPARSE_JACOBIAN_TO_CODE_GEN
    // -------------------------------------------------------------------------
    // setup
    void setup(
            // inputs
            size_t          size     ,
            const s_vector& row      ,
            const s_vector& col      ,
            // outputs
            size_t&         n_color  ,
            code_gen_fun&   fun      ,
            s_vector&  row_major     )
    {   // optimization options
        std::string optimize_options =
            "no_conditional_skip no_compare_op no_print_for_op";
        //
        // independent variable vector
        size_t nc = size;
        ac_vector ac_x(nc);
        //
        // dependent variable vector
        size_t nr = 2 * nc;
        ac_vector ac_y(nr);
        //
        // values of independent variables do not matter
        for(size_t j = 0; j < nc; j++)
            ac_x[j] = ac_double( double(j) / double(nc) );
        //
        // declare independent variables
        size_t abort_op_index = 0;
        bool record_compare   = false;
        CppAD::Independent(ac_x, abort_op_index, record_compare);
        //
        // AD computation of f(x) (order zero derivative is function value)
        size_t order = 0;
        CppAD::sparse_jac_fun<ac_double>(nr, nc, ac_x, row, col, order, ac_y);
        //
        // create function object f : x -> y
        CppAD::ADFun<c_double>            c_f;
        CppAD::ADFun<ac_double, c_double> ac_f;
        c_f.Dependent(ac_x, ac_y);
        if( global_option["optimize"] )
            c_f.optimize(optimize_options);
        //
        // number of non-zeros in sparsity pattern for Jacobian
# if ! PASS_SPARSE_JACOBIAN_TO_CODE_GEN
        // set fun
        code_gen_fun::evaluation_enum eval_jac = code_gen_fun::sparse_enum;
        code_gen_fun f_tmp("sparse_jacobian", c_f, eval_jac);
        fun.swap(f_tmp);
        //
        // set row_major
        d_vector x(nc);
        CppAD::uniform_01(nc, x);
        CppAD::sparse_rcv<s_vector, d_vector> Jrcv = fun.sparse_jacobian(x);
        row_major = Jrcv.row_major();
# ifndef NDEBUG
        size_t nnz = row.size();
        CPPAD_ASSERT_UNKNOWN( row_major.size() == nnz );
        for(size_t k = 0; k < nnz; ++k)
        {   size_t ell = row_major[k];
            CPPAD_ASSERT_UNKNOWN(
                Jrcv.row()[ell] == row[k] && Jrcv.col()[ell] == col[k]
            );
        }
# endif
        //
# else  // PASS_SPARSE_JACOBIAN_TO_CODE_GEN
        //
        // sparsity patttern  for subset of Jacobian pattern that is evaluated
        size_t nnz = row.size();
        sparsity subset_pattern(nr, nc, nnz);
        for(size_t k = 0; k < nnz; ++k)
            subset_pattern.set(k, row[k], col[k]);
        //
        // spoarse matrix for subset of Jacobian that is evaluated
        CppAD::sparse_rcv<s_vector, ac_vector> ac_subset( subset_pattern );
        //
        // coloring method
        std::string coloring = "cppad";
# if CPPAD_HAS_COLPACK
        if( global_option["colpack"] )
            coloring = "colpack";
# endif
        //
        // maximum number of colors at once
        size_t group_max = 1;
        ac_f = c_f.base2ad();
        //
        // declare independent variables for jacobian computation
        CppAD::Independent(ac_x, abort_op_index, record_compare);
        //
        if( global_option["subgraph"] )
        {   // use reverse mode because forward not yet implemented
            ac_f.subgraph_jac_rev(ac_x, ac_subset);
            n_color = 0;
        }
        else
        {   // calculate the Jacobian sparsity pattern for this function
            sparsity pattern;
            calc_sparsity(pattern, c_f);
            //
           // use forward mode to compute Jacobian
            CppAD::sparse_jac_work work;
            n_color = ac_f.sparse_jac_for(
                group_max, ac_x, ac_subset, pattern, coloring, work
            );
        }
        const ac_vector ac_val ( ac_subset.val() );
        //
        // create function g : x -> f'(x)
        CppAD::ADFun<c_double> c_g;
        c_g.Dependent(ac_x, ac_val);
        if( global_option["optimize"] )
            c_g.optimize(optimize_options);
        code_gen_fun g_tmp("sparse_jacobian", c_g);
        //
        // set reture value
        fun.swap(g_tmp);
# endif // PASS_SPARSE_JACOBIAN_TO_CODE_GEN
        return;
    }
}

bool link_sparse_jacobian(
    const std::string&               job      ,
    size_t                           size     ,
    size_t                           repeat   ,
    size_t                           m        ,
    const CppAD::vector<size_t>&     row      ,
    const CppAD::vector<size_t>&     col      ,
    CppAD::vector<double>&           x        ,
    CppAD::vector<double>&           jacobian ,
    size_t&                          n_color  )
{   assert( x.size() == size );
    assert( jacobian.size() == row.size() );
    // --------------------------------------------------------------------
    // check global options
    const char* valid[] = {
        "memory", "onetape", "optimize", "subgraph",
        "boolsparsity", "revsparsity", "subsparsity"
# if CPPAD_HAS_COLPACK
        , "colpack"
# endif
    };
    size_t n_valid = sizeof(valid) / sizeof(valid[0]);
    typedef std::map<std::string, bool>::iterator iterator;
    //
    for(iterator itr=global_option.begin(); itr!=global_option.end(); ++itr)
    {   if( itr->second )
        {   bool ok = false;
            for(size_t i = 0; i < n_valid; i++)
                ok |= itr->first == valid[i];
            if( ! ok )
                return false;
        }
    }
    if( global_option["subsparsity"] )
    {   if( global_option["boolsparisty"]
        ||  global_option["revsparsity"]
        ||  global_option["colpack"]  )
            return false;
    }
    // -----------------------------------------------------
    // size corresponding to static_fun
    static size_t static_size = 0;
    //
    // function object mapping x to f'(x)
    static code_gen_fun static_fun;
    //
    // row_major order for Jrcv
    static s_vector static_row_major;
    //
# if ! PASS_SPARSE_JACOBIAN_TO_CODE_GEN
    // code gen value for sparse jacobian
    CppAD::sparse_rcv<s_vector, d_vector> Jrcv;
# endif
    //
    // number of independent variables
    size_t nx = size;
    //
    bool onetape = global_option["onetape"];
    //
    // default return value
    n_color = 0;
    // -----------------------------------------------------
    if( job == "setup" )
    {   if( onetape )
        {   // sets n_color when ontape is true
            setup(size, row, col, n_color, static_fun, static_row_major);
            static_size = size;
        }
        else
        {   static_size = 0;
        }
        return true;
    }
    if( job == "teardown" )
    {   code_gen_fun f_tmp;
        static_fun.swap(f_tmp);
        static_row_major.clear();
        //
        static_size    = 0;
        return true;
    }
    // -----------------------------------------------------
    CPPAD_ASSERT_UNKNOWN( job == "run" )
    if( onetape ) while(repeat--)
    {   // use if before assert to vaoid warning that static_size is not used
        if( size != static_size )
        {   CPPAD_ASSERT_UNKNOWN( size == static_size );
        }

        // get next x
        CppAD::uniform_01(nx, x);

        // evaluate the jacobian
# if PASS_SPARSE_JACOBIAN_TO_CODE_GEN
        jacobian = static_fun(x);
# else
        Jrcv = static_fun.sparse_jacobian(x);
        CPPAD_ASSERT_UNKNOWN( Jrcv.nnz() == jacobian.size() );
        for(size_t k = 0; k < row.size(); ++k)
            jacobian[k] = Jrcv.val()[ static_row_major[k] ];
# endif
    }
    else while(repeat--)
    {   // sets n_color when ontape is false
        setup(size, row, col, n_color, static_fun, static_row_major);
        static_size = size;

        // get next x
        CppAD::uniform_01(nx, x);

        // evaluate the jacobian
# if PASS_SPARSE_JACOBIAN_TO_CODE_GEN
        jacobian = static_fun(x);
# else
        Jrcv = static_fun.sparse_jacobian(x);
        CPPAD_ASSERT_UNKNOWN( Jrcv.nnz() == jacobian.size() );
        for(size_t k = 0; k < row.size(); ++k)
            jacobian[k] = Jrcv.val()[ static_row_major[k] ];
# endif
    }
    return true;
}
/* %$$
$end
*/
