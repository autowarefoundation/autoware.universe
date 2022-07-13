/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin atomic_two_tangent.cpp$$
$spell
    Tanh
    bool
$$

$section Tan and Tanh as User Atomic Operations: Example and Test$$

$head Theory$$
The code below uses the $cref tan_forward$$ and $cref tan_reverse$$
to implement the tangent and hyperbolic tangent
functions as atomic function operations.

$head sparsity$$
This atomic operation can use both set and bool sparsity patterns.

$nospell

$head Start Class Definition$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
namespace { // Begin empty namespace
using CppAD::vector;
//
// a utility to compute the union of two sets.
using CppAD::set_union;
//
class atomic_tangent : public CppAD::atomic_base<float> {
/* %$$
$head Constructor $$
$srccode%cpp% */
private:
    const bool hyperbolic_; // is this hyperbolic tangent
public:
    // constructor
    atomic_tangent(const char* name, bool hyperbolic)
    : CppAD::atomic_base<float>(name),
    hyperbolic_(hyperbolic)
    { }
private:
/* %$$
$head forward$$
$srccode%cpp% */
    // forward mode routine called by CppAD
    bool forward(
        size_t                    p ,
        size_t                    q ,
        const vector<bool>&      vx ,
              vector<bool>&     vzy ,
        const vector<float>&     tx ,
              vector<float>&    tzy
    )
    {   size_t q1 = q + 1;
# ifndef NDEBUG
        size_t n  = tx.size()  / q1;
        size_t m  = tzy.size() / q1;
# endif
        assert( n == 1 );
        assert( m == 2 );
        assert( p <= q );
        size_t j, k;

        // check if this is during the call to old_tan(id, ax, ay)
        if( vx.size() > 0 )
        {   // set variable flag for both y an z
            vzy[0] = vx[0];
            vzy[1] = vx[0];
        }

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
    virtual bool reverse(
        size_t                    q ,
        const vector<float>&     tx ,
        const vector<float>&    tzy ,
              vector<float>&     px ,
        const vector<float>&    pzy
    )
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
$head for_sparse_jac$$
$srccode%cpp% */
    // forward Jacobian sparsity routine called by CppAD
    virtual bool for_sparse_jac(
        size_t                                p ,
        const vector<bool>&                   r ,
              vector<bool>&                   s ,
        const vector<float>&                  x )
    {
# ifndef NDEBUG
        size_t n = r.size() / p;
        size_t m = s.size() / p;
# endif
        assert( n == x.size() );
        assert( n == 1 );
        assert( m == 2 );

        // sparsity for S(x) = f'(x) * R
        for(size_t j = 0; j < p; j++)
        {   s[0 * p + j] = r[j];
            s[1 * p + j] = r[j];
        }

        return true;
    }
    // forward Jacobian sparsity routine called by CppAD
    virtual bool for_sparse_jac(
        size_t                                p ,
        const vector< std::set<size_t> >&     r ,
              vector< std::set<size_t> >&     s ,
        const vector<float>&                  x )
    {
# ifndef NDEBUG
        size_t n = r.size();
        size_t m = s.size();
# endif
        assert( n == x.size() );
        assert( n == 1 );
        assert( m == 2 );

        // sparsity for S(x) = f'(x) * R
        s[0] = r[0];
        s[1] = r[0];

        return true;
    }
/* %$$
$head rev_sparse_jac$$
$srccode%cpp% */
    // reverse Jacobian sparsity routine called by CppAD
    virtual bool rev_sparse_jac(
        size_t                                p ,
        const vector<bool>&                  rt ,
              vector<bool>&                  st ,
        const vector<float>&                  x )
    {
# ifndef NDEBUG
        size_t n = st.size() / p;
        size_t m = rt.size() / p;
# endif
        assert( n == 1 );
        assert( m == 2 );
        assert( n == x.size() );

        // sparsity for S(x)^T = f'(x)^T * R^T
        for(size_t j = 0; j < p; j++)
            st[j] = rt[0 * p + j] | rt[1 * p + j];

        return true;
    }
    // reverse Jacobian sparsity routine called by CppAD
    virtual bool rev_sparse_jac(
        size_t                                p ,
        const vector< std::set<size_t> >&    rt ,
              vector< std::set<size_t> >&    st ,
        const vector<float>&                  x )
    {
# ifndef NDEBUG
        size_t n = st.size();
        size_t m = rt.size();
# endif
        assert( n == 1 );
        assert( m == 2 );
        assert( n == x.size() );

        // sparsity for S(x)^T = f'(x)^T * R^T
        st[0] = set_union(rt[0], rt[1]);
        return true;
    }
/* %$$
$head rev_sparse_hes$$
$srccode%cpp% */
    // reverse Hessian sparsity routine called by CppAD
    virtual bool rev_sparse_hes(
        const vector<bool>&                   vx,
        const vector<bool>&                   s ,
              vector<bool>&                   t ,
        size_t                                p ,
        const vector<bool>&                   r ,
        const vector<bool>&                   u ,
              vector<bool>&                   v ,
        const vector<float>&                  x )
    {
# ifndef NDEBUG
        size_t m = s.size();
        size_t n = t.size();
# endif
        assert( x.size() == n );
        assert( r.size() == n * p );
        assert( u.size() == m * p );
        assert( v.size() == n * p );
        assert( n == 1 );
        assert( m == 2 );

        // There are no cross term second derivatives for this case,
        // so it is not necessary to vx.

        // sparsity for T(x) = S(x) * f'(x)
        t[0] =  s[0] | s[1];

        // V(x) = f'(x)^T * g''(y) * f'(x) * R  +  g'(y) * f''(x) * R
        // U(x) = g''(y) * f'(x) * R
        // S(x) = g'(y)

        // back propagate the sparsity for U, note both components
        // of f'(x) may be non-zero;
        size_t j;
        for(j = 0; j < p; j++)
            v[j] = u[ 0 * p + j ] | u[ 1 * p + j ];

        // include forward Jacobian sparsity in Hessian sparsity
        // (note sparsty for f''(x) * R same as for R)
        if( s[0] | s[1] )
        {   for(j = 0; j < p; j++)
            {   // Visual Studio 2013 generates warning without bool below
                v[j] |= bool( r[j] );
            }
        }

        return true;
    }
    // reverse Hessian sparsity routine called by CppAD
    virtual bool rev_sparse_hes(
        const vector<bool>&                   vx,
        const vector<bool>&                   s ,
              vector<bool>&                   t ,
        size_t                                p ,
        const vector< std::set<size_t> >&     r ,
        const vector< std::set<size_t> >&     u ,
              vector< std::set<size_t> >&     v ,
        const vector<float>&                  x )
    {
# ifndef NDEBUG
        size_t m = s.size();
        size_t n = t.size();
# endif
        assert( x.size() == n );
        assert( r.size() == n );
        assert( u.size() == m );
        assert( v.size() == n );
        assert( n == 1 );
        assert( m == 2 );

        // There are no cross term second derivatives for this case,
        // so it is not necessary to vx.

        // sparsity for T(x) = S(x) * f'(x)
        t[0] =  s[0] | s[1];

        // V(x) = f'(x)^T * g''(y) * f'(x) * R  +  g'(y) * f''(x) * R
        // U(x) = g''(y) * f'(x) * R
        // S(x) = g'(y)

        // back propagate the sparsity for U, note both components
        // of f'(x) may be non-zero;
        v[0] = set_union(u[0], u[1]);

        // include forward Jacobian sparsity in Hessian sparsity
        // (note sparsty for f''(x) * R same as for R)
        if( s[0] | s[1] )
            v[0] = set_union(v[0], r[0]);

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
    CppAD::vector< AD<float> > af(m);

    // temporary vector for computations
    // (my_tan and my_tanh computes tan or tanh and its square)
    CppAD::vector< AD<float> > az(2);

    // call atomic tan function and store tan(x) in f[0] (ignore tan(x)^2)
    my_tan(ax, az);
    af[0] = az[0];

    // call atomic tanh function and store tanh(x) in f[1] (ignore tanh(x)^2)
    my_tanh(ax, az);
    af[1] = az[0];

    // put a constant in f[2] = tanh(1.) (for sparsity pattern testing)
    CppAD::vector< AD<float> > one(1);
    one[0] = 1.;
    my_tanh(one, az);
    af[2] = az[0];

    // create f: x -> f and stop tape recording
    CppAD::ADFun<float> F;
    F.Dependent(ax, af);
/* %$$
$subhead forward$$
$srccode%cpp% */
    // check function value
    float tan = std::tan(x0);
    ok &= NearEqual(af[0] , tan,  eps, eps);
    float tanh = std::tanh(x0);
    ok &= NearEqual(af[1] , tanh,  eps, eps);

    // check zero order forward
    CppAD::vector<float> x(n), f(m);
    x[0] = x0;
    f    = F.Forward(0, x);
    ok &= NearEqual(f[0] , tan,  eps, eps);
    ok &= NearEqual(f[1] , tanh,  eps, eps);

    // compute first partial of f w.r.t. x[0] using forward mode
    CppAD::vector<float> dx(n), df(m);
    dx[0] = 1.;
    df    = F.Forward(1, dx);
/* %$$
$subhead reverse$$
$srccode%cpp% */
    // compute derivative of tan - tanh using reverse mode
    CppAD::vector<float> w(m), dw(n);
    w[0]  = 1.;
    w[1]  = 1.;
    w[2]  = 0.;
    dw    = F.Reverse(1, w);

    // tan'(x)   = 1 + tan(x)  * tan(x)
    // tanh'(x)  = 1 - tanh(x) * tanh(x)
    float tanp  = 1.f + tan * tan;
    float tanhp = 1.f - tanh * tanh;
    ok   &= NearEqual(df[0], tanp, eps, eps);
    ok   &= NearEqual(df[1], tanhp, eps, eps);
    ok   &= NearEqual(dw[0], w[0]*tanp + w[1]*tanhp, eps, eps);

    // compute second partial of f w.r.t. x[0] using forward mode
    CppAD::vector<float> ddx(n), ddf(m);
    ddx[0] = 0.;
    ddf    = F.Forward(2, ddx);

    // compute second derivative of tan - tanh using reverse mode
    CppAD::vector<float> ddw(2);
    ddw   = F.Reverse(2, w);

    // tan''(x)   = 2 *  tan(x) * tan'(x)
    // tanh''(x)  = - 2 * tanh(x) * tanh'(x)
    // Note that second order Taylor coefficient for u half the
    // corresponding second derivative.
    float two    = 2;
    float tanpp  =   two * tan * tanp;
    float tanhpp = - two * tanh * tanhp;
    ok   &= NearEqual(two * ddf[0], tanpp, eps, eps);
    ok   &= NearEqual(two * ddf[1], tanhpp, eps, eps);
    ok   &= NearEqual(ddw[0], w[0]*tanp  + w[1]*tanhp , eps, eps);
    ok   &= NearEqual(ddw[1], w[0]*tanpp + w[1]*tanhpp, eps, eps);
/* %$$
$subhead for_sparse_jac$$
$srccode%cpp% */
    // Forward mode computation of sparsity pattern for F.
    size_t p = n;
    // user vectorBool because m and n are small
    CppAD::vectorBool r1(p), s1(m * p);
    r1[0] = true;            // propagate sparsity for x[0]
    s1    = F.ForSparseJac(p, r1);
    ok  &= (s1[0] == true);  // f[0] depends on x[0]
    ok  &= (s1[1] == true);  // f[1] depends on x[0]
    ok  &= (s1[2] == false); // f[2] does not depend on x[0]
/* %$$
$subhead rev_sparse_jac$$
$srccode%cpp% */
    // Reverse mode computation of sparsity pattern for F.
    size_t q = m;
    CppAD::vectorBool s2(q * m), r2(q * n);
    // Sparsity pattern for identity matrix
    size_t i, j;
    for(i = 0; i < q; i++)
    {   for(j = 0; j < m; j++)
            s2[i * q + j] = (i == j);
    }
    r2   = F.RevSparseJac(q, s2);
    ok  &= (r2[0] == true);  // f[0] depends on x[0]
    ok  &= (r2[1] == true);  // f[1] depends on x[0]
    ok  &= (r2[2] == false); // f[2] does not depend on x[0]
/* %$$
$subhead rev_sparse_hes$$
$srccode%cpp% */
    // Hessian sparsity for f[0]
    CppAD::vectorBool s3(m), h(p * n);
    s3[0] = true;
    s3[1] = false;
    s3[2] = false;
    h    = F.RevSparseHes(p, s3);
    ok  &= (h[0] == true);  // Hessian is non-zero

    // Hessian sparsity for f[2]
    s3[0] = false;
    s3[2] = true;
    h    = F.RevSparseHes(p, s3);
    ok  &= (h[0] == false);  // Hessian is zero
/* %$$
$subhead Large x Values$$
$srccode%cpp% */
    // check tanh results for a large value of x
    x[0]  = std::numeric_limits<float>::max() / two;
    f     = F.Forward(0, x);
    tanh  = 1.;
    ok   &= NearEqual(f[1], tanh, eps, eps);
    df    = F.Forward(1, dx);
    tanhp = 0.;
    ok   &= NearEqual(df[1], tanhp, eps, eps);

    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
