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
Atomic function
g( x ) = [ x_2, x_0 * x_1 ]
\] $$
*/
# include <cppad/cppad.hpp>
namespace {   // isolate items below to this file
using   CppAD::vector;                          // vector
typedef vector< std::set<size_t> > set_vector;  // atomic_sparsity
//
// a utility to compute the union of two sets.
using CppAD::set_union;
//
class atomic_set_sparsity : public CppAD::atomic_base<double> {
public:
    // constructor
    atomic_set_sparsity(const std::string& name) :
    // this exampel only uses set sparsity patterns
    CppAD::atomic_base<double>(name, set_sparsity_enum )
    { }
private:
    // forward
    virtual bool forward(
        size_t                    p ,
        size_t                    q ,
        const vector<bool>&      vx ,
        vector<bool>&            vy ,
        const vector<double>&    tx ,
        vector<double>&          ty
    )
    {
        size_t n = tx.size() / (q + 1);
# ifndef NDEBUG
        size_t m = ty.size() / (q + 1);
# endif
        assert( n == 3 );
        assert( m == 2 );

        // only order zero
        bool ok = q == 0;
        if( ! ok )
            return ok;

        // check for defining variable information
        if( vx.size() > 0 )
        {   ok   &= vx.size() == n;
            vy[0] = vx[2];
            vy[1] = vx[0] || vx[1];
        }

        // Order zero forward mode.
        // y[0] = x[2], y[1] = x[0] * x[1]
        if( p <= 0 )
        {   ty[0] = tx[2];
            ty[1] = tx[0] * tx[1];
        }
        return ok;
    }
    // for_sparse_jac
    virtual bool for_sparse_jac(
        size_t                          p ,
        const set_vector&               r ,
        set_vector&                     s ,
        const vector<double>&           x )
    {   // This function needed if using f.ForSparseJac
# ifndef NDEBUG
        size_t n = r.size();
        size_t m = s.size();
# endif
        assert( n == x.size() );
        assert( n == 3 );
        assert( m == 2 );

        // sparsity for S(x) = f'(x) * R  = [ 0,   0, 1 ] * R
        s[0] = r[2];
        // s[1] = union(r[0], r[1])
        s[1] = set_union(r[0], r[1]);
        //
        return true;
    }
    virtual bool rev_sparse_jac(
        size_t                                p  ,
        const set_vector&                     rt ,
        set_vector&                           st ,
        const vector<double>&                 x  )
    {   // This function needed if using RevSparseJac or optimize
# ifndef NDEBUG
        size_t n = st.size();
        size_t m = rt.size();
# endif
        assert( n == x.size() );
        assert( n == 3 );
        assert( m == 2 );

        //                                       [ 0, x1 ]
        // sparsity for S(x)^T = f'(x)^T * R^T = [ 0, x0 ] * R^T
        //                                       [ 1, 0  ]
        st[0] = rt[1];
        st[1] = rt[1];
        st[2] = rt[0];
        return true;
    }
    virtual bool for_sparse_hes(
        const vector<bool>&                   vx,
        const vector<bool>&                   r ,
        const vector<bool>&                   s ,
        set_vector&                           h ,
        const vector<double>&                 x )
    {
        size_t n = r.size();
# ifndef NDEBUG
        size_t m = s.size();
# endif
        assert( x.size() == n );
        assert( h.size() == n );
        assert( n == 3 );
        assert( m == 2 );

        // initialize h as empty
        for(size_t i = 0; i < n; i++)
            h[i].clear();

        // only f_1 has a non-zero hessian
        if( ! s[1] )
            return true;

        // only the cross term between x[0] and x[1] is non-zero
        if( ! ( r[0] & r[1] ) )
            return true;

        // set the possibly non-zero terms in the hessian
        h[0].insert(1);
        h[1].insert(0);

        return true;
    }
    virtual bool rev_sparse_hes(
        const vector<bool>&                   vx,
        const vector<bool>&                   s ,
        vector<bool>&                         t ,
        size_t                                p ,
        const set_vector&                     r ,
        const set_vector&                     u ,
        set_vector&                           v ,
        const vector<double>&                 x )
    {   // This function needed if using RevSparseHes
# ifndef NDEBUG
        size_t m = s.size();
        size_t n = t.size();
# endif
        assert( x.size() == n );
        assert( r.size() == n );
        assert( u.size() == m );
        assert( v.size() == n );
        assert( n == 3 );
        assert( m == 2 );

        // sparsity for T(x) = S(x) * f'(x) = S(x) * [  0,  0,  1 ]
        //                                           [ x1, x0,  0 ]
        t[0] = s[1];
        t[1] = s[1];
        t[2] = s[0];

        // V(x) = f'(x)^T * g''(y) * f'(x) * R  +  g'(y) * f''(x) * R
        // U(x) = g''(y) * f'(x) * R
        // S(x) = g'(y)

        //                                      [ 0, x1 ]
        // sparsity for W(x) = f'(x)^T * U(x) = [ 0, x0 ] * U(x)
        //                                      [ 1, 0  ]
        v[0] = u[1];
        v[1] = u[1];
        v[2] = u[0];
        //
        //                                      [ 0, 1, 0 ]
        // sparsity for V(x) = W(x) + S_1 (x) * [ 1, 0, 0 ] * R
        //                                      [ 0, 0, 0 ]
        if( s[1] )
        {   // v[0] = union( v[0], r[1] )
            v[0] = set_union(v[0], r[1]);
            // v[1] = union( v[1], r[0] )
            v[1] = set_union(v[1], r[0]);
        }
        return true;
    }
}; // End of atomic_set_sparsity class


// f(u) = g( u_0 + u_1 , u_0 + u_1 , u_2 )
//      = [ u_2 , (u_0 + u_1)^2 ]
bool test_one(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * std::numeric_limits<double>::epsilon();
    // Create the atomic get_started object
    atomic_set_sparsity afun("atomic_set_sparsity");
    size_t n = 3;
    size_t m = 2;
    vector< AD<double> > au(n), ay(m);
    for(size_t j = 0; j < n; j++)
        au[j] = double(j + 1);

    // declare independent variables and start tape recording
    CppAD::Independent(au);

    // ax
    vector< AD<double> > ax(n);
    ax[0] = au[0] + au[1];
    ax[1] = au[0] + au[1];
    ax[2] = au[2];

    // call atomic function
    afun(ax, ay);

    // create f: u -> y and stop tape recording
    CppAD::ADFun<double> f(au, ay);

    // check function value
    ok &= NearEqual(ay[0] , au[2],  eps, eps);
    ok &= NearEqual(ay[1] , (au[0] + au[1]) * (au[0] + au[1]),  eps, eps);

    // correct Jacobian result
    set_vector check_s(m);
    check_s[0].insert(2);
    check_s[1].insert(0);
    check_s[1].insert(1);
    // compute and test forward mode
    {   set_vector r(n), s(m);
        for(size_t i = 0; i < n; i++)
            r[i].insert(i);
        s = f.ForSparseJac(n, r);
        for(size_t i = 0; i < m; i++)
            ok &= s[i] == check_s[i];
    }
    // compute and test reverse mode
    {   set_vector r(m), s(m);
        for(size_t i = 0; i < m; i++)
            r[i].insert(i);
        s = f.RevSparseJac(m, r);
        for(size_t i = 0; i < m; i++)
            ok &= s[i] == check_s[i];
    }
    // correct Hessian result for w_0 * f_0 (u) + w_1 * f_1(u)
    set_vector check_h(n);
    check_h[0].insert(0);
    check_h[0].insert(1);
    check_h[1].insert(0);
    check_h[1].insert(1);
    // compute and test forward mode
    {   set_vector r(1), s(1), h(n);
        for(size_t i = 0; i < m; i++)
            s[0].insert(i);
        for(size_t j = 0; j < n; j++)
            r[0].insert(j);
        h = f.ForSparseHes(r, s);
        for(size_t i = 0; i < n; i++)
            ok &= h[i] == check_h[i];
    }
    // compute and test reverse mode
    {   set_vector s(1), h(n);
        for(size_t i = 0; i < m; i++)
            s[0].insert(i);
        h = f.RevSparseHes(n, s);
        for(size_t i = 0; i < n; i++)
            ok &= h[i] == check_h[i];
    }
    return ok;
}

// f(u) = g( u_0 + u_1 , u_1 + u_2 , u_2 + u_0 )
//      = [ u_2 + u_0 , (u_0 + u_1)*(u_1 + u_2) ]
bool test_two(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * std::numeric_limits<double>::epsilon();
    // Create the atomic get_started object
    atomic_set_sparsity afun("atomic_set_sparsity");
    size_t n = 3;
    size_t m = 2;
    vector< AD<double> > au(n), ay(m);
    for(size_t j = 0; j < n; j++)
        au[j] = double(j + 1);

    // declare independent variables and start tape recording
    CppAD::Independent(au);

    // ax
    vector< AD<double> > ax(n);
    ax[0] = au[0] + au[1];
    ax[1] = au[1] + au[2];
    ax[2] = au[2] + au[0];

    // call atomic function
    afun(ax, ay);

    // create f: u -> y and stop tape recording
    CppAD::ADFun<double> f(au, ay);

    // check function value
    ok &= NearEqual(ay[0] , au[2] + au[0],  eps, eps);
    ok &= NearEqual(ay[1] , (au[0] + au[1]) * (au[1] + au[2]),  eps, eps);

    // correct Jacobian result
    set_vector check_s(m);
    check_s[0].insert(2);
    check_s[0].insert(0);
    check_s[1].insert(0);
    check_s[1].insert(1);
    check_s[1].insert(2);
    // compute and test forward mode
    {   set_vector r(n), s(m);
        for(size_t i = 0; i < n; i++)
            r[i].insert(i);
        s = f.ForSparseJac(n, r);
        for(size_t i = 0; i < m; i++)
            ok &= s[i] == check_s[i];
    }
    // compute and test reverse mode
    {   set_vector r(m), s(m);
        for(size_t i = 0; i < m; i++)
            r[i].insert(i);
        s = f.RevSparseJac(m, r);
        for(size_t i = 0; i < m; i++)
            ok &= s[i] == check_s[i];
    }
    // ----------------------------------------------------------------------
    // correct Hessian result for f_0 (u)
    set_vector check_h(n), s(1);
    s[0].insert(0);
    // compute and test forward mode
    {   set_vector r(1), h(n);
        for(size_t j = 0; j < n; j++)
            r[0].insert(j);
        h = f.ForSparseHes(r, s);
        for(size_t i = 0; i < n; i++)
            ok &= h[i] == check_h[i];
    }
    // compute and test reverse mode
    {   set_vector h(n);
        h = f.RevSparseHes(n, s);
        for(size_t i = 0; i < n; i++)
            ok &= h[i] == check_h[i];
    }
    // ----------------------------------------------------------------------
    // correct Hessian result for f_1 (u)
    s[0].clear();
    s[0].insert(1);
    check_h[0].insert(1);
    check_h[0].insert(2);
    check_h[1].insert(0);
    check_h[1].insert(1);
    check_h[1].insert(2);
    check_h[2].insert(0);
    check_h[2].insert(1);
    // compute and test forward mode
    {   set_vector r(1), h(n);
        for(size_t j = 0; j < n; j++)
            r[0].insert(j);
        h = f.ForSparseHes(r, s);
        for(size_t i = 0; i < n; i++)
            ok &= h[i] == check_h[i];
    }
    // compute and test reverse mode
    {   set_vector h(n);
        h = f.RevSparseHes(n, s);
        for(size_t i = 0; i < n; i++)
            ok &= h[i] == check_h[i];
    }
    return ok;
}

}  // End empty namespace

bool atomic_sparsity(void)
{   bool ok = true;
    ok     &= test_one();
    ok     &= test_two();
    return ok;
}
