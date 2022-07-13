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
$begin old_reciprocal.cpp$$
$section Old Atomic Operation Reciprocal: Example and Test$$

$head Deprecated 2013-05-27$$
This example has been deprecated;
see $cref atomic_two_reciprocal.cpp$$ instead.

$head Theory$$
The example below defines the atomic function
$latex f : \B{R}^n \rightarrow \B{R}^m$$ where
$latex n = 1$$, $latex m = 1$$, and $latex f(x) = 1 / x$$.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace { // Begin empty namespace
    using CppAD::vector;
    // ----------------------------------------------------------------------
    // a utility to compute the union of two sets.
    using CppAD::set_union;

    // ----------------------------------------------------------------------
    // forward mode routine called by CppAD
    bool reciprocal_forward(
        size_t                   id ,
        size_t                    k ,
        size_t                    n ,
        size_t                    m ,
        const vector<bool>&      vx ,
        vector<bool>&            vy ,
        const vector<double>&    tx ,
        vector<double>&          ty
    )
    {   assert( id == 0 );
        assert( n == 1 );
        assert( m == 1 );
        assert( k == 0 || vx.size() == 0 );
        bool ok = false;
        double f, fp, fpp;

        // Must always define the case k = 0.
        // Do not need case k if not using f.Forward(q, xp) for q >= k.
        switch(k)
        {   case 0:
            // this case must  be implemented
            if( vx.size() > 0 )
                vy[0] = vx[0];
            // y^0 = f( x^0 ) = 1 / x^0
            ty[0] = 1. / tx[0];
            ok    = true;
            break;

            case 1:
            // needed if first order forward mode is used
            assert( vx.size() == 0 );
            // y^1 = f'( x^0 ) x^1
            f     = ty[0];
            fp    = - f / tx[0];
            ty[1] = fp * tx[1];
            ok    = true;
            break;

            case 2:
            // needed if second order forward mode is used
            assert( vx.size() == 0 );
            // Y''(t) = X'(t)^\R{T} f''[X(t)] X'(t) + f'[X(t)] X''(t)
            // 2 y^2  = x^1 * f''( x^0 ) x^1 + 2 f'( x^0 ) x^2
            f     = ty[0];
            fp    = - f / tx[0];
            fpp   = - 2.0 * fp / tx[0];
            ty[2] = tx[1] * fpp * tx[1] / 2.0 + fp * tx[2];
            ok    = true;
            break;
        }
        return ok;
    }
    // ----------------------------------------------------------------------
    // reverse mode routine called by CppAD
    bool reciprocal_reverse(
        size_t                   id ,
        size_t                    k ,
        size_t                    n ,
        size_t                    m ,
        const vector<double>&    tx ,
        const vector<double>&    ty ,
        vector<double>&          px ,
        const vector<double>&    py
    )
    {   // Do not need case k if not using f.Reverse(k+1, w).
        assert( id == 0 );
        assert( n == 1 );
        assert( m == 1 );
        bool ok = false;

        double f, fp, fpp, fppp;
        switch(k)
        {   case 0:
            // needed if first order reverse mode is used
            // reverse: F^0 ( tx ) = y^0 = f( x^0 )
            f     = ty[0];
            fp    = - f / tx[0];
            px[0] = py[0] * fp;;
            ok    = true;
            break;

            case 1:
            // needed if second order reverse mode is used
            // reverse: F^1 ( tx ) = y^1 = f'( x^0 ) x^1
            f      = ty[0];
            fp     = - f / tx[0];
            fpp    = - 2.0 * fp / tx[0];
            px[1]  = py[1] * fp;
            px[0]  = py[1] * fpp * tx[1];
            // reverse: F^0 ( tx ) = y^0 = f( x^0 );
            px[0] += py[0] * fp;

            ok     = true;
            break;

            case 2:
            // needed if third order reverse mode is used
            // reverse: F^2 ( tx ) = y^2 =
            //            = x^1 * f''( x^0 ) x^1 / 2 + f'( x^0 ) x^2
            f      = ty[0];
            fp     = - f / tx[0];
            fpp    = - 2.0 * fp / tx[0];
            fppp   = - 3.0 * fpp / tx[0];
            px[2]  = py[2] * fp;
            px[1]  = py[2] * fpp * tx[1];
            px[0]  = py[2] * tx[1] * fppp * tx[1] / 2.0 + fpp * tx[2];
            // reverse: F^1 ( tx ) = y^1 = f'( x^0 ) x^1
            px[1] += py[1] * fp;
            px[0] += py[1] * fpp * tx[1];
            // reverse: F^0 ( tx ) = y^0 = f( x^0 );
            px[0] += py[0] * fp;

            ok = true;
            break;
        }
        return ok;
    }
    // ----------------------------------------------------------------------
    // forward Jacobian sparsity routine called by CppAD
    bool reciprocal_for_jac_sparse(
        size_t                               id ,
        size_t                                n ,
        size_t                                m ,
        size_t                                p ,
        const vector< std::set<size_t> >&     r ,
        vector< std::set<size_t> >&           s )
    {   // Can just return false if not using f.ForSparseJac
        assert( id == 0 );
        assert( n == 1 );
        assert( m == 1 );

        // sparsity for S(x) = f'(x) * R is same as sparsity for R
        s[0] = r[0];

        return true;
    }
    // ----------------------------------------------------------------------
    // reverse Jacobian sparsity routine called by CppAD
    bool reciprocal_rev_jac_sparse(
        size_t                               id ,
        size_t                                n ,
        size_t                                m ,
        size_t                                p ,
        vector< std::set<size_t> >&           r ,
        const vector< std::set<size_t> >&     s )
    {   // Can just return false if not using RevSparseJac.
        assert( id == 0 );
        assert( n == 1 );
        assert( m == 1 );

        // sparsity for R(x) = S * f'(x) is same as sparsity for S
        for(size_t q = 0; q < p; q++)
            r[q] = s[q];

        return true;
    }
    // ----------------------------------------------------------------------
    // reverse Hessian sparsity routine called by CppAD
    bool reciprocal_rev_hes_sparse(
        size_t                               id ,
        size_t                                n ,
        size_t                                m ,
        size_t                                p ,
        const vector< std::set<size_t> >&     r ,
        const vector<bool>&                   s ,
              vector<bool>&                   t ,
        const vector< std::set<size_t> >&     u ,
              vector< std::set<size_t> >&     v )
    {   // Can just return false if not use RevSparseHes.
        assert( id == 0 );
        assert( n == 1 );
        assert( m == 1 );

        // sparsity for T(x) = S(x) * f'(x) is same as sparsity for S
        t[0] = s[0];

        // V(x) = [ f'(x)^T * g''(y) * f'(x) + g'(y) * f''(x) ] * R
        // U(x) = g''(y) * f'(x) * R
        // S(x) = g'(y)

        // back propagate the sparsity for U because derivative of
        // reciprocal may be non-zero
        v[0] = u[0];

        // convert forward Jacobian sparsity to Hessian sparsity
        // because second derivative of reciprocal may be non-zero
        if( s[0] )
            v[0] = set_union(v[0], r[0] );


        return true;
    }
    // ---------------------------------------------------------------------
    // Declare the AD<double> routine reciprocal(id, ax, ay)
    CPPAD_USER_ATOMIC(
        reciprocal                 ,
        CppAD::vector              ,
        double                     ,
        reciprocal_forward         ,
        reciprocal_reverse         ,
        reciprocal_for_jac_sparse  ,
        reciprocal_rev_jac_sparse  ,
        reciprocal_rev_hes_sparse
    )
} // End empty namespace

bool old_reciprocal(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();

    // --------------------------------------------------------------------
    // Create the function f(x)
    //
    // domain space vector
    size_t n  = 1;
    double  x0 = 0.5;
    vector< AD<double> > ax(n);
    ax[0]     = x0;

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    vector< AD<double> > ay(m);

    // call atomic function and store reciprocal(x) in au[0]
    vector< AD<double> > au(m);
    size_t id = 0;           // not used
    reciprocal(id, ax, au);  // u = 1 / x

    // call atomic function and store reciprocal(u) in ay[0]
    reciprocal(id, au, ay);  // y = 1 / u = x

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (ax, ay);    // f(x) = x

    // --------------------------------------------------------------------
    // Check forward mode results
    //
    // check function value
    double check = x0;
    ok &= NearEqual( Value(ay[0]) , check,  eps, eps);

    // check zero order forward mode
    size_t q;
    vector<double> x_q(n), y_q(m);
    q      = 0;
    x_q[0] = x0;
    y_q    = f.Forward(q, x_q);
    ok &= NearEqual(y_q[0] , check,  eps, eps);

    // check first order forward mode
    q      = 1;
    x_q[0] = 1;
    y_q    = f.Forward(q, x_q);
    check  = 1.;
    ok &= NearEqual(y_q[0] , check,  eps, eps);

    // check second order forward mode
    q      = 2;
    x_q[0] = 0;
    y_q    = f.Forward(q, x_q);
    check  = 0.;
    ok &= NearEqual(y_q[0] , check,  eps, eps);

    // --------------------------------------------------------------------
    // Check reverse mode results
    //
    // third order reverse mode
    q     = 3;
    vector<double> w(m), dw(n * q);
    w[0]  = 1.;
    dw    = f.Reverse(q, w);
    check = 1.;
    ok &= NearEqual(dw[0] , check,  eps, eps);
    check = 0.;
    ok &= NearEqual(dw[1] , check,  eps, eps);
    ok &= NearEqual(dw[2] , check,  eps, eps);

    // --------------------------------------------------------------------
    // forward mode sparstiy pattern
    size_t p = n;
    CppAD::vectorBool r1(n * p), s1(m * p);
    r1[0] = true;          // compute sparsity pattern for x[0]
    s1    = f.ForSparseJac(p, r1);
    ok  &= s1[0] == true;  // f[0] depends on x[0]

    // --------------------------------------------------------------------
    // reverse mode sparstiy pattern
    q = m;
    CppAD::vectorBool s2(q * m), r2(q * n);
    s2[0] = true;          // compute sparsity pattern for f[0]
    r2    = f.RevSparseJac(q, s2);
    ok  &= r2[0] == true;  // f[0] depends on x[0]

    // --------------------------------------------------------------------
    // Hessian sparsity (using previous ForSparseJac call)
    CppAD::vectorBool s3(m), h(p * n);
    s3[0] = true;        // compute sparsity pattern for f[0]
    h     = f.RevSparseHes(p, s3);
    ok  &= h[0] == true; // second partial of f[0] w.r.t. x[0] may be non-zero

    // -----------------------------------------------------------------
    // Free all temporary work space associated with atomic_one objects.
    // (If there are future calls to atomic functions, they will
    // create new temporary work space.)
    CppAD::user_atomic<double>::clear();

    return ok;
}
// END C++
