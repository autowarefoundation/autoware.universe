/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin atomic_three_norm_sq.cpp$$
$spell
    sq
    bool
    enum
$$

$section Atomic Euclidean Norm Squared: Example and Test$$

$head Function$$
This example demonstrates using $cref atomic_three$$
to define the operation
$latex g : \B{R}^n \rightarrow \B{R}^m$$ where
$latex n = 2$$, $latex m = 1$$, where
$latex \[
    g(x) =  x_0^2 + x_1^2
\] $$

$nospell

$head Start Class Definition$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
namespace {           // isolate items below to this file
using CppAD::vector;  // abbreivate CppAD::vector as vector
//
class atomic_norm_sq : public CppAD::atomic_three<double> {
/* %$$
$head Constructor $$
$srccode%cpp% */
public:
    atomic_norm_sq(const std::string& name) :
    CppAD::atomic_three<double>(name)
    { }
private:
/* %$$
$head for_type$$
$srccode%cpp% */
    // calculate type_y
    bool for_type(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        vector<CppAD::ad_type_enum>&        type_y      ) override
    {   assert( parameter_x.size() == type_x.size() );
        bool ok = type_x.size() == 2; // n
        ok     &= type_y.size() == 1; // m
        if( ! ok )
            return false;
        type_y[0] = std::max(type_x[0], type_x[1]);
        return true;
    }
/* %$$
$head forward$$
$srccode%cpp% */
    // forward mode routine called by CppAD
    bool forward(
        const vector<double>&              parameter_x ,
        const vector<CppAD::ad_type_enum>& type_x      ,
        size_t                             need_y      ,
        size_t                             p           ,
        size_t                             q           ,
        const vector<double>&              tx          ,
        vector<double>&                    ty          ) override
    {
# ifndef NDEBUG
        size_t n = tx.size() / (q+1);
        size_t m = ty.size() / (q+1);
# endif
        assert( type_x.size() == n );
        assert( n == 2 );
        assert( m == 1 );
        assert( p <= q );

        // return flag
        bool ok = q <= 1;

        // Order zero forward mode must always be implemented.
        // y^0 = g( x^0 )
        double x_00 = tx[ 0*(q+1) + 0];        // x_0^0
        double x_10 = tx[ 1*(q+1) + 0];        // x_10
        double g = x_00 * x_00 + x_10 * x_10;  // g( x^0 )
        if( p <= 0 )
            ty[0] = g;   // y_0^0
        if( q <= 0 )
            return ok;

        // Order one forward mode.
        // This case needed if first order forward mode is used.
        // y^1 = g'( x^0 ) x^1
        double x_01 = tx[ 0*(q+1) + 1];   // x_0^1
        double x_11 = tx[ 1*(q+1) + 1];   // x_1^1
        double gp_0 = 2.0 * x_00;         // partial f w.r.t x_0^0
        double gp_1 = 2.0 * x_10;         // partial f w.r.t x_1^0
        if( p <= 1 )
            ty[1] = gp_0 * x_01 + gp_1 * x_11; // g'( x^0 ) * x^1
        if( q <= 1 )
            return ok;

        // Assume we are not using forward mode with order > 1
        assert( ! ok );
        return ok;
    }
/* %$$
$head reverse$$
$srccode%cpp% */
    // reverse mode routine called by CppAD
    bool reverse(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        size_t                              q           ,
        const vector<double>&               tx          ,
        const vector<double>&               ty          ,
        vector<double>&                     px          ,
        const vector<double>&               py          ) override
    {
# ifndef NDEBUG
        size_t n = tx.size() / (q+1);
        size_t m = ty.size() / (q+1);
# endif
        assert( px.size() == tx.size() );
        assert( py.size() == ty.size() );
        assert( n == 2 );
        assert( m == 1 );
        bool ok = q <= 1;

        double gp_0, gp_1;
        switch(q)
        {   case 0:
            // This case needed if first order reverse mode is used
            // F ( {x} ) = g( x^0 ) = y^0
            gp_0  =  2.0 * tx[0];  // partial F w.r.t. x_0^0
            gp_1  =  2.0 * tx[1];  // partial F w.r.t. x_0^1
            px[0] = py[0] * gp_0;; // partial G w.r.t. x_0^0
            px[1] = py[0] * gp_1;; // partial G w.r.t. x_0^1
            assert(ok);
            break;

            default:
            // Assume we are not using reverse with order > 1 (q > 0)
            assert(!ok);
        }
        return ok;
    }
/* %$$
$head jac_sparsity$$
$srccode%cpp% */
    // Jacobian sparsity routine called by CppAD
    bool jac_sparsity(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        bool                                dependency  ,
        const vector<bool>&                 select_x    ,
        const vector<bool>&                 select_y    ,
        CppAD::sparse_rc< vector<size_t> >& pattern_out ) override
    {   size_t n = select_x.size();
        size_t m = select_y.size();
        assert( n == 2 );
        assert( m == 1 );
        assert( parameter_x.size() == select_x.size() );
        //
        // count number non-zeros
        size_t nnz = 0;
        if( select_y[0] )
        {   if( select_x[0] )
                ++nnz;
            if( select_x[1] )
                ++nnz;
        }
        // sparsity pattern
        pattern_out.resize(m, n, nnz);
        size_t k = 0;
        if( select_y[0] )
        {   if( select_x[0] )
                pattern_out.set(k++, 0, 0);
            if( select_x[1] )
                pattern_out.set(k++, 0, 1);
        }
        return true;
    }
/* %$$
$head hes_sparsity$$
$srccode%cpp% */
    // Hessian sparsity routine called by CppAD
    bool hes_sparsity(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        const vector<bool>&                 select_x    ,
        const vector<bool>&                 select_y    ,
        CppAD::sparse_rc< vector<size_t> >& pattern_out ) override
    {   size_t n = select_x.size();
        assert( n == 2 );
        assert( select_y.size() == 1 ); // m
        assert( parameter_x.size() == select_x.size() );
        //
        // count number non-zeros
        size_t nnz = 0;
        if( select_y[0] )
        {   if( select_x[0] )
                ++nnz;
            if( select_x[1] )
                ++nnz;
        }
        // sparsity pattern
        pattern_out.resize(n, n, nnz);
        size_t k = 0;
        if( select_y[0] )
        {   if( select_x[0] )
                pattern_out.set(k++, 0, 0);
            if( select_x[1] )
                pattern_out.set(k++, 1, 1);
        }
        return true;
    }
/* %$$
$head End Class Definition$$
$srccode%cpp% */
}; // End of atomic_norm_sq class
}  // End empty namespace

/* %$$
$head Use Atomic Function$$
$srccode%cpp% */
bool norm_sq(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
/* %$$
$subhead Constructor$$
$srccode%cpp% */
    // --------------------------------------------------------------------
    // Create the atomic reciprocal object
    atomic_norm_sq afun("atomic_norm_sq");
/* %$$
$subhead Recording$$
$srccode%cpp% */
    // Create the function f(x) = g(x)
    //
    // domain space vector
    size_t  n  = 2;
    double  x0 = 0.25;
    double  x1 = 0.75;
    vector< AD<double> > ax(n);
    ax[0]      = x0;
    ax[1]      = x1;

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    vector< AD<double> > ay(m);

    // call atomic function and store norm_sq(x) in au[0]
    afun(ax, ay);        // y_0 = x_0 * x_0 + x_1 * x_1

    // create g: x -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (ax, ay);
/* %$$
$subhead forward$$
$srccode%cpp% */
    // check function value
    double check = x0 * x0 + x1 * x1;
    ok &= NearEqual( Value(ay[0]) , check,  eps, eps);

    // check zero order forward mode
    size_t q;
    vector<double> x_q(n), y_q(m);
    q      = 0;
    x_q[0] = x0;
    x_q[1] = x1;
    y_q    = f.Forward(q, x_q);
    ok &= NearEqual(y_q[0] , check,  eps, eps);

    // check first order forward mode
    q      = 1;
    x_q[0] = 0.3;
    x_q[1] = 0.7;
    y_q    = f.Forward(q, x_q);
    check  = 2.0 * x0 * x_q[0] + 2.0 * x1 * x_q[1];
    ok &= NearEqual(y_q[0] , check,  eps, eps);

/* %$$
$subhead reverse$$
$srccode%cpp% */
    // first order reverse mode
    q     = 1;
    vector<double> w(m), dw(n * q);
    w[0]  = 1.;
    dw    = f.Reverse(q, w);
    check = 2.0 * x0;
    ok &= NearEqual(dw[0] , check,  eps, eps);
    check = 2.0 * x1;
    ok &= NearEqual(dw[1] , check,  eps, eps);
/* %$$
$subhead rev_jac_sparsity$$
$srccode%cpp% */
    // reverse mode Jacobian sparstiy pattern
    CppAD::sparse_rc< CPPAD_TESTVECTOR(size_t) > pattern_in, pattern_out;
    pattern_in.resize(m, m, m);
    for(size_t i = 0; i < m; ++i)
        pattern_in.set(i, i, i);
    bool transpose     = false;
    bool dependency    = false;
    bool internal_bool = false;
    f.rev_jac_sparsity(
        pattern_in, transpose, dependency, internal_bool, pattern_out
    );
    CPPAD_TESTVECTOR(size_t) row_major  = pattern_out.row_major();
    //
    // first element in row major order is (0, 0)
    size_t k = 0;
    size_t r = pattern_out.row()[ row_major[k] ];
    size_t c = pattern_out.col()[ row_major[k] ];
    ok      &= r == 0 && c == 0;
    //
    // second element in row major order is (0, 1)
    ++k;
    r        = pattern_out.row()[ row_major[k] ];
    c        = pattern_out.col()[ row_major[k] ];
    ok      &= r == 0 && c == 1;
    //
    // k + 1 should be number of values in sparsity pattern
    ok      &= k + 1 == pattern_out.nnz();
/* %$$
$subhead for_hes_sparsity$$
$srccode%cpp% */
    // forward mode Hessian sparsity pattern
    CPPAD_TESTVECTOR(bool) select_x(n), select_y(m);
    for(size_t j = 0; j < n; ++j)
        select_x[j] = true;
    for(size_t i = 0; i < m; ++i)
        select_y[i] = true;
    f.for_hes_sparsity(
        select_x, select_y, internal_bool, pattern_out
    );
    CPPAD_TESTVECTOR(size_t) order  = pattern_out.row_major();
    //
    // first element in row major order is (0, 0)
    k   = 0;
    r   = pattern_out.row()[ order[k] ];
    c   = pattern_out.col()[ order[k] ];
    ok &= r == 0 && c == 0;
    //
    // second element in row major order is (1, 1)
    ++k;
    r   = pattern_out.row()[ order[k] ];
    c   = pattern_out.col()[ order[k] ];
    ok &= r == 1 && c == 1;
    //
    // k + 1 should be number of values in sparsity pattern
    ok &= k + 1 == pattern_out.nnz();
    //
    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
