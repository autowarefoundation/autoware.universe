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
$begin atomic_three_tangent.cpp$$
$spell
    Tanh
    bool
$$

$section Tan and Tanh as User Atomic Operations: Example and Test$$

$head Discussion$$
The code below uses the $cref tan_forward$$ and $cref tan_reverse$$
to implement the tangent and hyperbolic tangent
functions as atomic function operations.
It also uses $code AD<float>$$,
while most atomic examples use $code AD<double>$$.


$nospell

$head Start Class Definition$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
namespace { // Begin empty namespace
using CppAD::vector;
//
class atomic_tangent : public CppAD::atomic_three<float> {
/* %$$
$head Constructor $$
$srccode%cpp% */
private:
    const bool hyperbolic_; // is this hyperbolic tangent
public:
    // constructor
    atomic_tangent(const char* name, bool hyperbolic)
    : CppAD::atomic_three<float>(name),
    hyperbolic_(hyperbolic)
    { }
private:
/* %$$
$head for_type$$
$srccode%cpp% */
    // calculate type_y
    bool for_type(
        const vector<float>&                parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        vector<CppAD::ad_type_enum>&        type_y      ) override
    {   assert( parameter_x.size() == type_x.size() );
        bool ok = type_x.size() == 1; // n
        ok     &= type_y.size() == 2; // m
        if( ! ok )
            return false;
        type_y[0] = type_x[0];
        type_y[1] = type_x[0];
        return true;
    }
/* %$$
$head forward$$
$srccode%cpp% */
    // forward mode routine called by CppAD
    bool forward(
        const vector<float>&               parameter_x ,
        const vector<CppAD::ad_type_enum>& type_x      ,
        size_t                             need_y      ,
        size_t                             p           ,
        size_t                             q           ,
        const vector<float>&               tx          ,
        vector<float>&                     tzy         ) override
    {   size_t q1 = q + 1;
# ifndef NDEBUG
        size_t n  = tx.size()  / q1;
        size_t m  = tzy.size() / q1;
# endif
        assert( type_x.size() == n );
        assert( n == 1 );
        assert( m == 2 );
        assert( p <= q );
        size_t j, k;

        if( p == 0 )
        {   // z^{(0)} = tan( x^{(0)} ) or tanh( x^{(0)} )
            if( hyperbolic_ )
                tzy[0] = float( tanh( tx[0] ) );
            else
                tzy[0] = float( tan( tx[0] ) );

            // y^{(0)} = z^{(0)} * z^{(0)}
            tzy[q1 + 0] = tzy[0] * tzy[0];

            p++;
        }
        for(j = p; j <= q; j++)
        {   float j_inv = 1.f / float(j);
            if( hyperbolic_ )
                j_inv = - j_inv;

            // z^{(j)} = x^{(j)} +- sum_{k=1}^j k x^{(k)} y^{(j-k)} / j
            tzy[j] = tx[j];
            for(k = 1; k <= j; k++)
                tzy[j] += tx[k] * tzy[q1 + j-k] * float(k) * j_inv;

            // y^{(j)} = sum_{k=0}^j z^{(k)} z^{(j-k)}
            tzy[q1 + j] = 0.;
            for(k = 0; k <= j; k++)
                tzy[q1 + j] += tzy[k] * tzy[j-k];
        }

        // All orders are implemented and there are no possible errors
        return true;
    }
/* %$$
$head reverse$$
$srccode%cpp% */
    // reverse mode routine called by CppAD
    bool reverse(
        const vector<float>&               parameter_x ,
        const vector<CppAD::ad_type_enum>& type_x      ,
        size_t                             q           ,
        const vector<float>&               tx          ,
        const vector<float>&               tzy         ,
        vector<float>&                     px          ,
        const vector<float>&               pzy         ) override
    {   size_t q1 = q + 1;
# ifndef NDEBUG
        size_t n  = tx.size()  / q1;
        size_t m  = tzy.size() / q1;
# endif
        assert( px.size()  == n * q1 );
        assert( pzy.size() == m * q1 );
        assert( n == 1 );
        assert( m == 2 );

        size_t j, k;

        // copy because partials w.r.t. y and z need to change
        vector<float> qzy = pzy;

        // initialize accumultion of reverse mode partials
        for(k = 0; k < q1; k++)
            px[k] = 0.;

        // eliminate positive orders
        for(j = q; j > 0; j--)
        {   float j_inv = 1.f / float(j);
            if( hyperbolic_ )
                j_inv = - j_inv;

            // H_{x^{(k)}} += delta(j-k) +- H_{z^{(j)} y^{(j-k)} * k / j
            px[j] += qzy[j];
            for(k = 1; k <= j; k++)
                px[k] += qzy[j] * tzy[q1 + j-k] * float(k) * j_inv;

            // H_{y^{j-k)} += +- H_{z^{(j)} x^{(k)} * k / j
            for(k = 1; k <= j; k++)
                qzy[q1 + j-k] += qzy[j] * tx[k] * float(k) * j_inv;

            // H_{z^{(k)}} += H_{y^{(j-1)}} * z^{(j-k-1)} * 2.
            for(k = 0; k < j; k++)
                qzy[k] += qzy[q1 + j-1] * tzy[j-k-1] * 2.f;
        }

        // eliminate order zero
        if( hyperbolic_ )
            px[0] += qzy[0] * (1.f - tzy[q1 + 0]);
        else
            px[0] += qzy[0] * (1.f + tzy[q1 + 0]);

        return true;
    }
/* %$$
$head jac_sparsity$$
$srccode%cpp% */
    // Jacobian sparsity routine called by CppAD
    bool jac_sparsity(
        const vector<float>&                parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        bool                                dependency  ,
        const vector<bool>&                 select_x    ,
        const vector<bool>&                 select_y    ,
        CppAD::sparse_rc< vector<size_t> >& pattern_out ) override
    {
        size_t n = select_x.size();
        size_t m = select_y.size();
        assert( parameter_x.size() == n );
        assert( n == 1 );
        assert( m == 2 );

        // number of non-zeros in sparsity pattern
        size_t nnz = 0;
        if( select_x[0] )
        {   if( select_y[0] )
                ++nnz;
            if( select_y[1] )
                ++nnz;
        }

        // sparsity pattern
        pattern_out.resize(m, n, nnz);
        size_t k = 0;
        if( select_x[0] )
        {   if( select_y[0] )
                pattern_out.set(k++, 0, 0);
            if( select_y[1] )
                pattern_out.set(k++, 1, 0);
        }
        assert( k == nnz );

        return true;
    }
/* %$$
$head hes_sparsity$$
$srccode%cpp% */
    // Hessian sparsity routine called by CppAD
    bool hes_sparsity(
        const vector<float>&                parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        const vector<bool>&                 select_x    ,
        const vector<bool>&                 select_y    ,
        CppAD::sparse_rc< vector<size_t> >& pattern_out ) override
    {
        assert( parameter_x.size() == select_x.size() );
        assert( select_y.size() == 2 );
        size_t n = select_x.size();
        assert( n == 1 );

        // number of non-zeros in sparsity pattern
        size_t nnz = 0;
        if( select_x[0] & (select_y[0] | select_y[1]) )
            nnz = 1;
        // sparsity pattern
        pattern_out.resize(n, n, nnz);
        if( select_x[0] & (select_y[0] | select_y[1]) )
            pattern_out.set(0, 0, 0);

        return true;
    }
/* %$$
$head End Class Definition$$
$srccode%cpp% */
}; // End of atomic_tangent class
}  // End empty namespace

/* %$$
$head Use Atomic Function$$
$srccode%cpp% */
bool tangent(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    float eps = 10.f * CppAD::numeric_limits<float>::epsilon();
/* %$$
$subhead Constructor$$
$srccode%cpp% */
    // --------------------------------------------------------------------
    // Creater a tan and tanh object
    atomic_tangent my_tan("my_tan", false), my_tanh("my_tanh", true);
/* %$$
$subhead Recording$$
$srccode%cpp% */
    // domain space vector
    size_t n  = 1;
    float  x0 = 0.5;
    CppAD::vector< AD<float> > ax(n);
    ax[0]     = x0;

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 3;
    CppAD::vector< AD<float> > av(m);

    // temporary vector for computations
    // (my_tan and my_tanh computes tan or tanh and its square)
    CppAD::vector< AD<float> > au(2);

    // call atomic tan function and store tan(x) in f[0], ignore tan(x)^2
    my_tan(ax, au);
    av[0] = au[0];

    // call atomic tanh function and store tanh(x) in f[1], ignore tanh(x)^2
    my_tanh(ax, au);
    av[1] = au[0];

    // put a constant in f[2] = tanh(1.),  for sparsity pattern testing
    CppAD::vector< AD<float> > one(1);
    one[0] = 1.;
    my_tanh(one, au);
    av[2] = au[0];

    // create f: x -> v and stop tape recording
    CppAD::ADFun<float> f;
    f.Dependent(ax, av);
/* %$$
$subhead forward$$
$srccode%cpp% */
    // check function value
    float tan = std::tan(x0);
    ok &= NearEqual(av[0] , tan,  eps, eps);
    float tanh = std::tanh(x0);
    ok &= NearEqual(av[1] , tanh,  eps, eps);

    // check zero order forward
    CppAD::vector<float> x(n), v(m);
    x[0] = x0;
    v    = f.Forward(0, x);
    ok &= NearEqual(v[0] , tan,  eps, eps);
    ok &= NearEqual(v[1] , tanh,  eps, eps);

    // tan'(x)   = 1 + tan(x)  * tan(x)
    // tanh'(x)  = 1 - tanh(x) * tanh(x)
    float tanp  = 1.f + tan * tan;
    float tanhp = 1.f - tanh * tanh;

    // compute first partial of f w.r.t. x[0] using forward mode
    CppAD::vector<float> dx(n), dv(m);
    dx[0] = 1.;
    dv    = f.Forward(1, dx);
    ok   &= NearEqual(dv[0] , tanp,   eps, eps);
    ok   &= NearEqual(dv[1] , tanhp,  eps, eps);
    ok   &= NearEqual(dv[2] , 0.f,    eps, eps);

    // tan''(x)   = 2 *  tan(x) * tan'(x)
    // tanh''(x)  = - 2 * tanh(x) * tanh'(x)
    // Note that second order Taylor coefficient for u half the
    // corresponding second derivative.
    float two    = 2;
    float tanpp  =   two * tan * tanp;
    float tanhpp = - two * tanh * tanhp;

    // compute second partial of f w.r.t. x[0] using forward mode
    CppAD::vector<float> ddx(n), ddv(m);
    ddx[0] = 0.;
    ddv    = f.Forward(2, ddx);
    ok   &= NearEqual(two * ddv[0], tanpp, eps, eps);
    ok   &= NearEqual(two * ddv[1], tanhpp, eps, eps);
    ok   &= NearEqual(two * ddv[2], 0.f, eps, eps);

/* %$$
$subhead reverse$$
$srccode%cpp% */
    // compute derivative of tan - tanh using reverse mode
    CppAD::vector<float> w(m), dw(n);
    w[0]  = 1.;
    w[1]  = 1.;
    w[2]  = 0.;
    dw    = f.Reverse(1, w);
    ok   &= NearEqual(dw[0], w[0]*tanp + w[1]*tanhp, eps, eps);

    // compute second derivative of tan - tanh using reverse mode
    CppAD::vector<float> ddw(2);
    ddw   = f.Reverse(2, w);
    ok   &= NearEqual(ddw[0], w[0]*tanp  + w[1]*tanhp , eps, eps);
    ok   &= NearEqual(ddw[1], w[0]*tanpp + w[1]*tanhpp, eps, eps);
/* %$$
$subhead for_jac_sparsity$$
$srccode%cpp% */
    // forward mode Jacobian sparstiy pattern
    CppAD::sparse_rc< CPPAD_TESTVECTOR(size_t) > pattern_in, pattern_out;
    pattern_in.resize(1, 1, 1);
    pattern_in.set(0, 0, 0);
    bool transpose     = false;
    bool dependency    = false;
    bool internal_bool = false;
    f.for_jac_sparsity(
        pattern_in, transpose, dependency, internal_bool, pattern_out
    );
    // (0, 0) and (1, 0) are in sparsity pattern
    ok &= pattern_out.nnz() == 2;
    ok &= pattern_out.row()[0] == 0;
    ok &= pattern_out.col()[0] == 0;
    ok &= pattern_out.row()[1] == 1;
    ok &= pattern_out.col()[1] == 0;
/* %$$
$subhead rev_sparse_hes$$
$srccode%cpp% */
    // Hesian sparsity (using previous for_jac_sparsity call)
    CPPAD_TESTVECTOR(bool) select_y(m);
    select_y[0] = true;
    select_y[1] = false;
    select_y[2] = false;
    f.rev_hes_sparsity(
        select_y, transpose, internal_bool, pattern_out
    );
    ok &= pattern_out.nnz() == 1;
    ok &= pattern_out.row()[0] == 0;
    ok &= pattern_out.col()[0] == 0;
/* %$$
$subhead Large x Values$$
$srccode%cpp% */
    // check tanh results for a large value of x
    x[0]  = std::numeric_limits<float>::max() / two;
    v     = f.Forward(0, x);
    tanh  = 1.;
    ok   &= NearEqual(v[1], tanh, eps, eps);
    dv    = f.Forward(1, dx);
    tanhp = 0.;
    ok   &= NearEqual(dv[1], tanhp, eps, eps);

    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
