/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cstring>
# include <cppad/utility/vector.hpp>

/*
$begin adolc_sparse_jacobian.cpp$$
$spell
    const
    sparsedrivers.cpp
    colpack
    boolsparsity
    adouble
    int int_n
    cppad.hpp
    onetape
    typedef
    alloc
    jac
    nnz
    cind
    bool
    CppAD
    adolc
    sparse_jacobian
$$

$section Adolc Speed: Sparse Jacobian$$


$head Specifications$$
See $cref link_sparse_jacobian$$.

$head Implementation$$

$srccode%cpp% */
// suppress conversion warnings before other includes
# include <cppad/wno_conversion.hpp>
//
# include <adolc/adolc.h>
# include <adolc/adolc_sparse.h>
# include <cppad/utility/vector.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/speed/sparse_jac_fun.hpp>

// list of possible options
# include <map>
extern std::map<std::string, bool> global_option;

namespace {
    using CppAD::vector;
    typedef vector<size_t>    s_vector;
    typedef vector<double>    d_vector;
    typedef vector<adouble>   a_vector;
    void setup(
        // inputs
        int             tag     ,
        size_t          size    ,
        size_t          m       ,
        const s_vector& row     ,
        const s_vector& col     ,
        const d_vector& x       ,
        int*            options , // const but adolc want non-const arg
        // outupts
        size_t&         n_color ,
        int&            nnz     ,
        unsigned int*&  rind    ,
        unsigned int*&  cind    ,
        double*&        values  )
    {   // independent variables
        CPPAD_ASSERT_UNKNOWN( size = x.size() );
        int keep = 0; // keep forward mode results
        trace_on(tag, keep);
        size_t n = size;
        a_vector a_x(n);
        for(size_t j = 0; j < n; ++j)
            a_x[j] <<= x[j];
        //
        // dependent variables
        a_vector a_y(m);
        //
        // AD computation of f(x)
        size_t order = 0;
        CppAD::sparse_jac_fun<adouble>(m, n, a_x, row, col, order, a_y);
        //
        // create function object f : x -> y
        double yi;
        for(size_t i = 0; i < m; i++)
            a_y[i] >>= yi;
        trace_off();
        //
        // null pointers for recalculation of sparsity pattern
        free(rind);
        free(cind);
        free(values);
        rind   = nullptr;
        cind   = nullptr;
        values = nullptr;
        //
        // Retrieve n_color using undocumented feature of sparsedrivers.cpp
        int same_pattern = 0;
        n_color = sparse_jac(tag, int(m), int(n),
            same_pattern, x.data(), &nnz, &rind, &cind, &values, options
        );
    }

}

bool link_sparse_jacobian(
    const std::string&               job      ,
    size_t                           size     ,
    size_t                           repeat   ,
    size_t                           m        ,
    const CppAD::vector<size_t>&     row      ,
    const CppAD::vector<size_t>&     col      ,
          CppAD::vector<double>&     x_return ,
          CppAD::vector<double>&     jacobian ,
          size_t&                    n_color  )
{
    // --------------------------------------------------------------------
    // check global options
    // Allow colpack true even though it is not used below because it is
    // true durng the adolc correctness tests.
    const char* valid[] = { "onetape", "optimize", "colpack"};
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
    // -----------------------------------------------------
    static size_t  static_size     = 0;
    static int     static_nnz      = 0;
    unsigned int*  static_rind     = nullptr;
    unsigned int*  static_cind     = nullptr;
    double*        static_values   = nullptr;
    // -----------------------------------------------------
    int tag  = 0;
    //
    // options that control sparse_jac
    int        options[4];
    if( global_option["boolsparsity"] )
        options[0] = 1;  // sparsity by propagation of bit pattern
    else
        options[0] = 0;  // sparsity pattern by index domains
    options[1] = 0;  // 0 = safe mode, 1 = tight mode
    options[2] = 0;  // 0 = autodetect, 1 = forward, 2 = reverse
    options[3] = 0;  // 0 = column compression, 1 = row compression
    //
    // independent variiables
    size_t n = size;
    d_vector x(n);
    //
    // default value for n_color
    n_color = 0;
    //
    bool onetape = global_option["onetape"];
    // -----------------------------------------------------
    if( job == "setup" )
    {   // get a value for x
        CppAD::uniform_01(n, x);
        //
        // record the tape and run coloring problem
        options[2] = -1;
        setup(tag, size, m, row, col, x, options,
            n_color, static_nnz, static_rind, static_cind, static_values
        );
        static_size = size;
        //
        return true;
    }
    if( job == "teardown" )
    {   free(static_rind);
        free(static_cind);
        free(static_values);
        static_rind   = nullptr;
        static_cind   = nullptr;
        static_values = nullptr;
        return true;
    }
    // -----------------------------------------------------
    CPPAD_ASSERT_UNKNOWN( job == "run" );
    //
    while (repeat--)
    {   // choose a value for x
        CppAD::uniform_01(n, x);
        //
        if ( ! onetape )
        {   // retape and calculate jacobian
            options[2] = -1; // stop at sparsity pattern, return n_color
            setup(tag, size, m, row, col, x, options,
                n_color, static_nnz, static_rind, static_cind, static_values
            );
            options[2] = 0;
        }
        else
        {   if( size != static_size )
                CPPAD_ASSERT_UNKNOWN( size == static_size );
        }
        // calculate the jacobian at this x
        int same_pattern = 1;
        sparse_jac(
            tag, int(m), int(n), same_pattern, x.data(),
            &static_nnz, &static_rind, &static_cind, &static_values,
            options
        );
    }
    // --------------------------------------------------------------------
    // jacobian
    CPPAD_ASSERT_UNKNOWN( size_t(static_nnz) == row.size() );
    for(int ell = 0; ell < static_nnz; ell++)
    {   CPPAD_ASSERT_UNKNOWN( row[ell] == size_t(static_rind[ell]) );
        CPPAD_ASSERT_UNKNOWN( col[ell] == size_t(static_cind[ell]) );
        jacobian[ell] = static_values[ell];
    }
    // x_return
    for(size_t j = 0; j < n; j++)
        x_return[j] = x[j];
    //
    return true;
}
/* %$$
$end
*/
