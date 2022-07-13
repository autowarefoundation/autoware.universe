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
$begin cppad_sparse_jacobian.cpp$$
$spell
    const
    colpack
    boolsparsity
    namespace
    onetape
    work work
    jac
    CppAD
    cppad
    hpp
    bool
    typedef
    endif
    std
    Jacobian
$$

$section Cppad Speed: Sparse Jacobian$$


$head Specifications$$
See $cref link_sparse_jacobian$$.

$head Implementation$$

$srccode%cpp% */
# include <cppad/cppad.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/speed/sparse_jac_fun.hpp>

// Note that CppAD uses global_option["memory"] at the main program level
# include <map>
extern std::map<std::string, bool> global_option;
// see comments in main program for this external
extern size_t global_cppad_thread_alloc_inuse;

namespace {
    using CppAD::vector;
    typedef CppAD::AD<double>                     a_double;
    typedef vector<size_t>                        s_vector;
    typedef vector<bool>                          b_vector;
    typedef vector<double>                        d_vector;
    typedef vector<a_double>                      a_vector;
    typedef CppAD::sparse_rc<s_vector>            sparsity;
    typedef CppAD::sparse_rcv<s_vector, d_vector> sparse_matrix;


    void calc_sparsity(
        CppAD::sparse_rc<s_vector>& pattern ,
        CppAD::ADFun<double>&       f        )
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
    // --------------------------------------------------------------------
    void setup(
        // inputs
        size_t                  size    ,
        size_t                  m       ,
        const s_vector&         row     ,
        const s_vector&         col     ,
        // outputs
        size_t&                 n_color ,
        CppAD::ADFun<double>&   f       ,
        sparse_matrix&          subset  ,
        CppAD::sparse_jac_work& work    )
    {   // optimization options
        std::string optimize_options =
            "no_conditional_skip no_compare_op no_print_for_op";
        //
        // default value for n_color
        n_color = 0;
        //
        // independent variable vector
        size_t nc = size;
        a_vector a_x(nc);
        d_vector x(nc);
        //
        // dependent variable vector
        size_t nr = m;
        a_vector a_y(nr);
        //
        // choose a value for independent variable vector
        CppAD::uniform_01(nc, x);
        for(size_t j = 0; j < nc; j++)
            a_x[j] = x[j];
        //
        // declare independent variables
        size_t abort_op_index = 0;
        bool record_compare   = false;
        CppAD::Independent(a_x, abort_op_index, record_compare);
        //
        // AD computation of f(x)
        size_t order = 0;
        CppAD::sparse_jac_fun<a_double>(nr, nc, a_x, row, col, order, a_y);
        //
        // create function object f : x -> y
        f.Dependent(a_x, a_y);
        //
        if( global_option["optimize"] )
            f.optimize(optimize_options);
        //
        // coloring method
        std::string coloring = "cppad";
# if CPPAD_HAS_COLPACK
        if( global_option["colpack"] )
            coloring = "colpack";
# else
        CPPAD_ASSERT_UNKNOWN( ! global_option["colpack"] );
# endif
        //
        // sparsity pattern for subset of Jacobian that is evaluated
        size_t nnz = row.size();
        sparsity subset_pattern(nr, nc, nnz);
        for(size_t k = 0; k < nnz; ++k)
            subset_pattern.set(k, row[k], col[k]);
        //
        // sparse matrix for subset of Jacobian that is evaluated
        subset = sparse_matrix( subset_pattern );
        //
        // maximum number of colors at once
        size_t group_max = 25;
        //
        if( global_option["subgraph"] )
        {   // This would cache some information in f,  but would it enough ?
            // The time it takes to compute derivatives that are not used
            // slows down the test when onetape is false.
            // f.subgraph_jac_rev(x, ac_subset);
        }
        else
        {   // need full sparsity pattern
            // (could use subset_sparsity, but pretend we do not konw that)
            sparsity pattern;
            calc_sparsity(pattern, f);
            //
            // Use forward mode to compute the Jacobian
            // (this caches informaiton in work),
            work.clear();
            n_color = f.sparse_jac_for(
                group_max, x, subset, pattern, coloring, work
            );
        }
    }
}

bool link_sparse_jacobian(
    const std::string&               job      ,
    size_t                           size     ,
    size_t                           repeat   ,
    size_t                           m        ,
    const CppAD::vector<size_t>&     row      ,
    const CppAD::vector<size_t>&     col      ,
          CppAD::vector<double>&     x        ,
          CppAD::vector<double>&     jacobian ,
          size_t&                    n_color  )
{   global_cppad_thread_alloc_inuse = 0;

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
    // size corresponding to static_f
    static size_t static_size = 0;
    //
    // function object corresponding to f(x)
    static CppAD::ADFun<double> static_f;
    //
    // subset of Jacobian that we are using
    static sparse_matrix static_subset;
    //
    // information used by for_sparse_jac_for
    static CppAD::sparse_jac_work static_work;
    //
    // sparsity pattern not used because work is non-empty
    sparsity empty_pattern;
    // -----------------------------------------------------------------------
    //
    // default value for n_color
    n_color = 0;
    //
    bool onetape = global_option["onetape"];
    //
    if( job == "setup" )
    {   if( onetape )
        {   setup(size, m, row, col,
                n_color, static_f, static_subset, static_work
            );
            static_size = size;
        }
        else
        {   static_size = 0;
        }
        return true;
    }
    if( job == "teardown" )
    {   static_f      = CppAD::ADFun<double>();
        sparse_matrix empty_matrix;
        static_subset.swap( empty_matrix );
        static_work.clear();
        static_size = 0;
        return true;
    }
    // ------------------------------------------------------------------------
    CPPAD_ASSERT_UNKNOWN( job == "run" );
    //
    // number of independent variables
    static size_t n = size;
    //
    // maximum number of colors at once
    size_t group_max = 25;
    //
    // coloring method
    std::string coloring = "cppad";
    if( global_option["colpack"] )
        coloring = "colpack";
    // ------------------------------------------------------
    while(repeat--)
    {   if( onetape )
        {   if( size != static_size )
                CPPAD_ASSERT_UNKNOWN( size == static_size );
        }
        else
        {   setup(size, m, row, col,
                n_color, static_f, static_subset, static_work
            );
        }
        // choose a value for x
        CppAD::uniform_01(n, x);

        if( global_option["subgraph"] )
        {   // user reverse mode becasue forward not yet implemented
            static_f.subgraph_jac_rev(x, static_subset);
        }
        else
        {   // Use forward mode because m > n (is this sufficient reason ?)
            n_color = static_f.sparse_jac_for(group_max, x,
                static_subset, empty_pattern, coloring, static_work
            );
        }
        jacobian = static_subset.val();
    }
    size_t thread                   = CppAD::thread_alloc::thread_num();
    global_cppad_thread_alloc_inuse = CppAD::thread_alloc::inuse(thread);
    return true;
}
/* %$$
$end
*/
