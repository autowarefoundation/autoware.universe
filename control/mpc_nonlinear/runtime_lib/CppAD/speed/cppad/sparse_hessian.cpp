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
$begin cppad_sparse_hessian.cpp$$
$spell
    ifdef
    ifndef
    colpack
    boolsparsity
    namespace
    Jac
    onetape
    const
    hes
    CppAD
    cppad
    hpp
    bool
    typedef
    endif
    tmp
    std
    var
    cout
    endl
$$

$section Cppad Speed: Sparse Hessian$$


$head Specifications$$
See $cref link_sparse_hessian$$.

$head Implementation$$

$srccode%cpp% */
# include <cppad/cppad.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/speed/sparse_hes_fun.hpp>

// Note that CppAD uses global_option["memory"] at the main program level
# include <map>
extern std::map<std::string, bool> global_option;
// see comments in main program for this external
extern size_t global_cppad_thread_alloc_inuse;

namespace {
    // typedefs
    using CppAD::vector;
    typedef CppAD::AD<double>                     a1double;
    typedef CppAD::AD<a1double>                   a2double;
    typedef vector<bool>                          b_vector;
    typedef vector<size_t>                        s_vector;
    typedef vector<double>                        d_vector;
    typedef vector<a1double>                      a1vector;
    typedef vector<a2double>                      a2vector;
    typedef CppAD::sparse_rc<s_vector>            sparsity_pattern;
    typedef CppAD::sparse_rcv<s_vector, d_vector> sparse_matrix;
    // ------------------------------------------------------------------------
    void create_fun(
        const d_vector&             x        ,
        const s_vector&             row      ,
        const s_vector&             col      ,
        CppAD::ADFun<double>&       fun      )
    {
        // initialize a1double version of independent variables
        size_t n = x.size();
        a1vector a1x(n);
        for(size_t j = 0; j < n; j++)
            a1x[j] = x[j];
        //
        // optimization options
        std::string optimize_options =
            "no_conditional_skip no_compare_op no_print_for_op";
        //
        // order of derivative in sparse_hes_fun
        size_t order = 0;
        //
        // do not even record comparison operators
        size_t abort_op_index = 0;
        bool record_compare   = false;
        //
        if( ! global_option["hes2jac"] )
        {
            // declare independent variables
            Independent(a1x, abort_op_index, record_compare);
            //
            // AD computation of y
            a1vector a1y(1);
            CppAD::sparse_hes_fun<a1double>(n, a1x, row, col, order, a1y);
            //
            // create function object f : X -> Y
            fun.Dependent(a1x, a1y);
            //
            if( global_option["optimize"] )
                fun.optimize(optimize_options);
            //
            // skip comparison operators
            fun.compare_change_count(0);
            //
            // fun corresonds to f(x)
            return;
        }
        // declare independent variables for f(x)
        a2vector a2x(n);
        for(size_t j = 0; j < n; j++)
            a2x[j] = a1x[j];
        Independent(a2x, abort_op_index, record_compare);
        //
        // a2double computation of y
        a2vector a2y(1);
        CppAD::sparse_hes_fun<a2double>(n, a2x, row, col, order, a2y);
        //
        // create function object corresponding to y = f(x)
        CppAD::ADFun<a1double> a1f;
        a1f.Dependent(a2x, a2y);
        //
        // declare independent variables for g(x)
        Independent(a1x, abort_op_index, record_compare);
        //
        // a1double computation of z
        a1vector a1w(1), a1z(n);
        a1w[0] = 1.0;
        a1f.Forward(0, a1x);
        a1z = a1f.Reverse(1, a1w);
        //
        // create function object z = g(x) = f'(x)
        fun.Dependent(a1x, a1z);
        //
        if( global_option["optimize"] )
            fun.optimize(optimize_options);
        //
        // skip comparison operators
        fun.compare_change_count(0);
        //
        // fun corresonds to g(x)
        return;
    }
    // ------------------------------------------------------------------------
    void calc_sparsity(
        sparsity_pattern&      sparsity ,
        CppAD::ADFun<double>&  fun      )
    {
        size_t n = fun.Domain();
        size_t m = fun.Range();
        //
        bool transpose     = false;
        //
        if( global_option["subsparsity"] )
        {   CPPAD_ASSERT_UNKNOWN( global_option["hes2jac"] )
            CPPAD_ASSERT_UNKNOWN( n == m );
            b_vector select_domain(n), select_range(m);
            for(size_t j = 0; j < n; ++j)
                select_domain[j] = true;
            for(size_t i = 0; i < m; ++i)
                select_range[i] = true;
            //
            // fun corresponds to g(x)
            fun.subgraph_sparsity(
                select_domain, select_range, transpose, sparsity
            );
            return;
        }
        bool dependency    = false;
        bool reverse       = global_option["revsparsity"];
        bool internal_bool = global_option["boolsparsity"];
        //
        if( ! global_option["hes2jac"] )
        {   // fun corresponds to f(x)
            //
            CPPAD_ASSERT_UNKNOWN( m == 1 );
            //
            b_vector select_range(m);
            select_range[0] = true;
            //
            if( reverse )
            {   sparsity_pattern identity;
                identity.resize(n, n, n);
                for(size_t k = 0; k < n; k++)
                    identity.set(k, k, k);
                fun.for_jac_sparsity(
                    identity, transpose, dependency, internal_bool, sparsity
                );
                fun.rev_hes_sparsity(
                    select_range, transpose, internal_bool, sparsity
                );
            }
            else
            {   b_vector select_domain(n);
                for(size_t j = 0; j < n; j++)
                    select_domain[j] = true;
                fun.for_hes_sparsity(
                    select_domain, select_range, internal_bool, sparsity
                );
            }
            return;
        }
        // fun correspnds to g(x)
        CPPAD_ASSERT_UNKNOWN( m == n );
        //
        // sparsity pattern for identity matrix
        sparsity_pattern eye;
        eye.resize(n, n, n);
        for(size_t k = 0; k < n; k++)
            eye.set(k, k, k);
        //
        if( reverse )
        {   fun.rev_jac_sparsity(
                eye, transpose, dependency, internal_bool, sparsity
            );
        }
        else
        {   fun.for_jac_sparsity(
                eye, transpose, dependency, internal_bool, sparsity
            );
        }
        return;
    }
    // ------------------------------------------------------------------------
    size_t calc_hessian(
        d_vector&               hessian  ,
        const d_vector&         x        ,
        sparse_matrix&          subset   ,
        const sparsity_pattern& sparsity ,
        CppAD::sparse_jac_work& jac_work ,
        CppAD::sparse_hes_work& hes_work ,
        CppAD::ADFun<double>&   fun      )
    {   size_t n_color;
        //
        if( ! global_option["hes2jac"] )
        {   // fun corresponds to f(x)
            //
            // coloring method
            std::string coloring = "cppad";
            if( global_option["colpack"] )
                coloring = "colpack";
            if( global_option["symmetric"] )
                coloring += ".symmetric";
            else
                coloring += ".general";
            //
            // only one function component
            d_vector w(1);
            w[0] = 1.0;
            //
            // compute hessian
            n_color = fun.sparse_hes(
                x, w, subset, sparsity, coloring, hes_work
            );
        }
        else
        {   // fun corresponds to g(x)
            //
            if( global_option["subgraph"] )
            {   fun.subgraph_jac_rev(x, subset);
                n_color = 0;
            }
            else
            {
                //
                // coloring method
                std::string coloring = "cppad";
# if CPPAD_HAS_COLPACK
                if( global_option["colpack"] )
                    coloring = "colpack";
# endif
                size_t group_max = 1;
                n_color = fun.sparse_jac_for(
                    group_max, x, subset, sparsity, coloring, jac_work
                );
            }
        }
        // return result
        const d_vector& val( subset.val() );
        size_t nnz = subset.nnz();
        for(size_t k = 0; k < nnz; k++)
            hessian[k] = val[k];
        //
        return n_color;
    }
}

bool link_sparse_hessian(
    size_t                           size     ,
    size_t                           repeat   ,
    const CppAD::vector<size_t>&     row      ,
    const CppAD::vector<size_t>&     col      ,
    CppAD::vector<double>&           x        ,
    CppAD::vector<double>&           hessian  ,
    size_t&                          n_color  )
{   global_cppad_thread_alloc_inuse = 0;

    // --------------------------------------------------------------------
    // check global options
    const char* valid[] = {
        "memory", "onetape", "optimize", "hes2jac", "subgraph",
        "boolsparsity", "revsparsity", "symmetric"
# if CPPAD_HAS_COLPACK
        , "colpack"
# else
        , "subsparsity"
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
    {   if( global_option["boolsparsity"] || global_option["revsparsity"] )
            return false;
        if( ! global_option["hes2jac"] )
            return false;
    }
    if( global_option["subgraph"] )
    {   if( ! global_option["hes2jac"] )
            return false;
    }
# if ! CPPAD_HAS_COLPACK
    if( global_option["colpack"] )
        return false;
# endif
    // -----------------------------------------------------------------------
    // setup
    size_t n = size;          // number of independent variables
    CppAD::ADFun<double> fun; // AD function object used to calculate Hessian
    //
    // declare sparsity pattern
    sparsity_pattern sparsity;
    //
    // declare subset where Hessian is evaluated
    sparsity_pattern subset_pattern;
    size_t nr  = n;
    size_t nc  = n;
    size_t nnz = row.size();
    subset_pattern.resize(nr, nc, nnz);
    for(size_t k = 0; k < nnz; k++)
        subset_pattern.set(k, row[k], col[k]);
    sparse_matrix subset( subset_pattern );
    //
    // structures that holds some of the work done by sparse_jac, sparse_hes
    CppAD::sparse_jac_work jac_work;
    CppAD::sparse_hes_work hes_work;

    // -----------------------------------------------------------------------
    if( ! global_option["onetape"] ) while(repeat--)
    {   // choose a value for x
        CppAD::uniform_01(n, x);
        //
        // create f(x)
        create_fun(x, row, col, fun);
        //
        // calculate the sparsity pattern for Hessian of f(x)
        calc_sparsity(sparsity, fun);
        //
        // calculate the Hessian at this x
        jac_work.clear(); // wihtout work from previous calculation
        hes_work.clear();
        n_color = calc_hessian(
            hessian, x, subset, sparsity, jac_work, hes_work, fun
        );
    }
    else
    {   // choose a value for x
        CppAD::uniform_01(n, x);
        //
        // create f(x)
        create_fun(x, row, col, fun);
        //
        // calculate the sparsity pattern for Hessian of f(x)
        calc_sparsity(sparsity, fun);
        //
        while(repeat--)
        {   // choose a value for x
            CppAD::uniform_01(n, x);
            //
            // calculate this Hessian at this x
            n_color = calc_hessian(
                hessian, x, subset, sparsity, jac_work, hes_work, fun
            );
        }
    }
    size_t thread                   = CppAD::thread_alloc::thread_num();
    global_cppad_thread_alloc_inuse = CppAD::thread_alloc::inuse(thread);
    return true;
}
/* %$$
$end
*/
