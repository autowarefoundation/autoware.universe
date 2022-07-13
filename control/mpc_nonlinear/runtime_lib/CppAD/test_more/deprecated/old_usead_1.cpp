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
$begin old_usead_1.cpp$$
$spell
    checkpoint
    var
$$

$section Using AD to Compute Atomic Function Derivatives$$

$head Deprecated 2013-05-27$$
This example has been deprecated because it is easier to use the
$cref/checkpoint/chkpoint_one/$$ class instead.

$head Purpose$$
Consider the case where an inner function is used repeatedly in the
definition of an outer function.
In this case, it may reduce the number of variables
$cref/size_var/fun_property/size_var/$$,
and hence the required memory.

$head Simple Case$$
This example is the same as old_reciprocal.cpp, except that it
uses AD to compute the
derivatives needed by an atomic function.
This is a simple example of an inner function, and hence not really
useful for the purpose above;
see old_usead_2.cpp for a more complete example.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>

namespace { // Begin empty namespace
    using CppAD::AD;
    using CppAD::ADFun;
    using CppAD::vector;

    // ----------------------------------------------------------------------
    // function that computes reciprocal
    ADFun<double>* r_ptr_;
    void create_r(void)
    {   vector< AD<double> > ax(1), ay(1);
        ax[0]  = 1;
        CppAD::Independent(ax);
        ay[0]  = 1.0 / ax[0];
        r_ptr_ = new ADFun<double>(ax, ay);
    }
    void destroy_r(void)
    {   delete r_ptr_;
        r_ptr_ = nullptr;
    }

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
        bool ok = true;
        vector<double> x_q(1), y_q(1);

        // check for special case
        if( vx.size() > 0 )
            vy[0] = vx[0];

        // make sure r_ has proper lower order Taylor coefficients stored
        // then compute ty[k]
        for(size_t q = 0; q <= k; q++)
        {   x_q[0] = tx[q];
            y_q    = r_ptr_->Forward(q, x_q);
            if( q == k )
                ty[k] = y_q[0];
            assert( q == k || ty[q] == y_q[0] );
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
    {   assert( id == 0 );
        assert( n == 1 );
        assert( m == 1 );
        bool ok = true;
        vector<double> x_q(1), w(k+1), dw(k+1);

        // make sure r_ has proper forward mode coefficients
        size_t q;
        for(q = 0; q <= k; q++)
        {   x_q[0] = tx[q];
# ifdef NDEBUG
            r_ptr_->Forward(q, x_q);
# else
            vector<double> y_q(1);
            y_q    = r_ptr_->Forward(q, x_q);
            assert( ty[q] == y_q[0] );
# endif
        }
        for(q = 0; q <=k; q++)
            w[q] = py[q];
        dw = r_ptr_->Reverse(k+1, w);
        for(q = 0; q <=k; q++)
            px[q] = dw[q];

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
    {   assert( id == 0 );
        assert( n == 1 );
        assert( m == 1 );
        bool ok = true;

        vector< std::set<size_t> > R(1), S(1);
        R[0] = r[0];
        S = r_ptr_->ForSparseJac(p, R);
        s[0] = S[0];

        return ok;
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
    {
        assert( id == 0 );
        assert( n == 1 );
        assert( m == 1 );
        bool ok = true;

        vector< std::set<size_t> > R(p), S(p);
        size_t q;
        for(q = 0; q < p; q++)
            S[q] = s[q];
        R = r_ptr_->RevSparseJac(p, S);
        for(q = 0; q < p; q++)
            r[q] = R[q];

        return ok;
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
        vector<bool>&                         t ,
        const vector< std::set<size_t> >&     u ,
        vector< std::set<size_t> >&           v )
    {   // Can just return false if not use RevSparseHes.
        assert( id == 0 );
        assert( n == 1 );
        assert( m == 1 );
        bool ok = true;

        // compute sparsity pattern for T(x) = S(x) * f'(x)
        vector<bool> T(1), S(1);
        S[0]   = s[0];
        T      = r_ptr_->RevSparseJac(1, S);
        t[0]   = T[0];

        // compute sparsity pattern for A(x) = U(x)^T * f'(x)
        vector<bool> Ut(p), A(p);
        size_t q;
        for(q = 0; q < p; q++)
            Ut[q] = false;
        std::set<size_t>::const_iterator itr;
        for(itr = u[0].begin(); itr != u[0].end(); itr++)
            Ut[*itr] = true;
        A = r_ptr_-> RevSparseJac(p, Ut);

        // compute sparsity pattern for H(x) = R^T * (S * F)''(x)
        vector<bool> H(p), R(n);
        for(q = 0; q < p; q++)
            R[q] = false;
        for(itr = r[0].begin(); itr != r[0].end(); itr++)
            R[*itr] = true;
        r_ptr_->ForSparseJac(p, R);
        H = r_ptr_->RevSparseHes(p, S);

        // compute sparsity pattern for V(x) = A(x)^T + H(x)^T
        v[0].clear();
        for(q = 0; q < p; q++)
            if( A[q] | H[q] )
                v[0].insert(q);

        return ok;
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

bool old_usead_1(void)
{   bool ok = true;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();

    // --------------------------------------------------------------------
    // Create the ADFun<doulbe> r_
    create_r();

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
    ADFun<double> f;
    f.Dependent(ax, ay);     // f(x) = x

    // --------------------------------------------------------------------
    // Check function value results
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
    h     = f.RevSparseJac(p, s3);
    ok  &= h[0] == true; // second partial of f[0] w.r.t. x[0] may be non-zero

    // -----------------------------------------------------------------
    // Free all memory associated with the object r_ptr
    destroy_r();

    // -----------------------------------------------------------------
    // Free all temporary work space associated with atomic_one objects.
    // (If there are future calls to atomic functions, they will
    // create new temporary work space.)
    CppAD::user_atomic<double>::clear();

    return ok;
}
// END C++
