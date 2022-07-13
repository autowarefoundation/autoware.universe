/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availabilitaylor_y set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin atomic_three_jac_sparsity.cpp$$
$spell
    Jacobian
$$

$section Atomic Function Jacobian Sparsity: Example and Test$$

$head Purpose$$
This example demonstrates calculation of a Jacobian sparsity pattern
using an atomic operation.

$head Function$$
For this example, the atomic function
$latex g : \B{R}^3 \rightarrow \B{R}^2$$ is defined by
$latex \[
g(x) = \left( \begin{array}{c}
    x_2 * x_2 \\
    x_0 * x_1
\end{array} \right)
\] $$

$head Jacobian$$
The corresponding Jacobian is
$latex \[
g^{(1)} (x) = \left( \begin{array}{ccc}
  0  &   0 & 2 x_2 \\
x_1  & x_0 & 0
\end{array} \right)
\] $$

$nospell

$head Start  Class Definition$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
namespace {          // begin empty namespace
using CppAD::vector; // abbreviate CppAD::vector as vector
//
class atomic_jac_sparsity : public CppAD::atomic_three<double> {
/* %$$
$head Constructor $$
$srccode%cpp% */
public:
    atomic_jac_sparsity(const std::string& name) :
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
        bool ok = type_x.size() == 3; // n
        ok     &= type_y.size() == 2; // m
        if( ! ok )
            return false;

        type_y[0]  = type_x[2];
        type_y[1] = std::max(type_x[0], type_x[1]);
        return true;
    }
/* %$$
$head forward$$
$srccode%cpp% */
    // forward mode routine called by CppAD
    bool forward(
        const vector<double>&              parameter_x  ,
        const vector<CppAD::ad_type_enum>& type_x       ,
        size_t                             need_y       ,
        size_t                             order_low    ,
        size_t                             order_up     ,
        const vector<double>&              taylor_x     ,
        vector<double>&                    taylor_y     ) override
    {
# ifndef NDEBUG
        size_t n = taylor_x.size() / (order_up + 1);
        size_t m = taylor_y.size() / (order_up + 1);
# endif
        assert( n == 3 );
        assert( m == 2 );
        assert( order_low <= order_up );

        // return flag
        bool ok = order_up == 0;
        if( ! ok )
            return ok;

        // Order zero forward mode must always be implemented
        taylor_y[0] = taylor_x[2] * taylor_x[2];
        taylor_y[1] = taylor_x[0] * taylor_x[1];

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
    {
        size_t n = select_x.size();
        size_t m = select_y.size();
        assert( parameter_x.size() == n );
        assert( n == 3 );
        assert( m == 2 );

        // count number of non-zeros in sparsity pattern
        size_t nnz = 0;
        // row 0
        if( select_y[0] & select_x[2] )
            ++nnz;
        // row 1
        if( select_y[1] )
        {   // column 0
            if( select_x[0] )
                ++nnz;
            // column 1
            if( select_x[1] )
                ++nnz;
        }

        // size of pattern_out
        size_t nr = m;
        size_t nc = n;
        pattern_out.resize(nr, nc, nnz);

        // set the values in pattern_out using index k
        size_t k = 0;

        // y_0 depends and has possibly non-zeron partial w.r.t x_2
        if( select_y[0] & select_x[2] )
            pattern_out.set(k++, 0, 2);
        if( select_y[1] )
        {   // y_1 depends and has possibly non-zero partial w.r.t x_0
            if( select_x[0] )
                pattern_out.set(k++, 1, 0);
            // y_1 depends and has possibly non-zero partial w.r.t x_1
            if( select_x[1] )
                pattern_out.set(k++, 1, 1);
        }
        assert( k == nnz );
        //
        return true;
    }
}; // End of atomic_three_jac_sparsity class

/* %$$
$head Use Atomic Function$$
$srccode%cpp% */
bool use_jac_sparsity(bool x_1_variable, bool forward)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    //
    // Create the atomic_jac_sparsity object correspnding to g(x)
    atomic_jac_sparsity afun("atomic_jac_sparsity");
    //
    // Create the function f(u) = g(u) for this example.
    //
    // domain space vector
    size_t n  = 3;
    double u_0 = 1.00;
    double u_1 = 2.00;
    double u_2 = 3.00;
    vector< AD<double> > au(n);
    au[0] = u_0;
    au[1] = u_1;
    au[2] = u_2;

    // declare independent variables and start tape recording
    CppAD::Independent(au);

    // range space vector
    size_t m = 2;
    vector< AD<double> > ay(m);

    // call atomic function
    vector< AD<double> > ax(n);
    ax[0] = au[0];
    ax[2] = au[2];
    if( x_1_variable )
    {   ok   &= Variable( au[1] );
        ax[1] = au[1];
    }
    else
    {   AD<double> ap = u_1;
        ok   &= Parameter(ap);
        ok   &= ap == au[1];
        ax[1] = u_1;
    }
    // x_1_variable true:  y = [ u_2 * u_2 ,  u_0 * u_1 ]^T
    // x_1_variable false: y = [ u_2 * u_2 ,  u_0 * p   ]^T
    afun(ax, ay);

    // create f: u -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (au, ay);  // f(u) = y
    //
    // check function value
    double check = u_2 * u_2;
    ok &= NearEqual( Value(ay[0]) , check,  eps, eps);
    check = u_0 * u_1;
    ok &= NearEqual( Value(ay[1]) , check,  eps, eps);

    // check zero order forward mode
    size_t q;
    vector<double> xq(n), yq(m);
    q     = 0;
    xq[0] = u_0;
    xq[1] = u_1;
    xq[2] = u_2;
    yq    = f.Forward(q, xq);
    check = u_2 * u_2;
    ok &= NearEqual(yq[0] , check,  eps, eps);
    check = u_0 * u_1;
    ok &= NearEqual(yq[1] , check,  eps, eps);

    // sparsity pattern for identity matrix
    size_t nnz;
    if( forward )
        nnz = n;
    else
        nnz = m;
    CppAD::sparse_rc< CPPAD_TESTVECTOR(size_t) > pattern_in(nnz, nnz, nnz);
    for(size_t k = 0; k < nnz; ++k)
        pattern_in.set(k, k, k);

    // Jacobian sparsity for f(u)
    bool transpose     = false;
    bool dependency    = false;
    bool internal_bool = false;
    CppAD::sparse_rc< CPPAD_TESTVECTOR(size_t) > pattern_out;
    if( forward )
    {   f.for_jac_sparsity(
            pattern_in, transpose, dependency, internal_bool, pattern_out
        );
    }
    else
    {   f.rev_jac_sparsity(
            pattern_in, transpose, dependency, internal_bool, pattern_out
        );
    }
    const CPPAD_TESTVECTOR(size_t)& row = pattern_out.row();
    const CPPAD_TESTVECTOR(size_t)& col = pattern_out.col();
    CPPAD_TESTVECTOR(size_t) row_major  = pattern_out.row_major();
    //
    // first element in row major order has index (0, 2)
    size_t k = 0;
    size_t r = row[ row_major[k] ];
    size_t c = col[ row_major[k] ];
    ok      &= r == 0 && c == 2;
    //
    // second element in row major order has index (1, 0)
    ++k;
    r   = row[ row_major[k] ];
    c   = col[ row_major[k] ];
    ok &= r == 1 && c == 0;
    //
    if( x_1_variable )
    {   // third element in row major order has index (1, 1)
        ++k;
        r   = row[ row_major[k] ];
        c   = col[ row_major[k] ];
        ok &= r == 1 && c == 1;
    }
    // k + 1 should be the number of values in sparsity pattern
    ok &= k + 1 == pattern_out.nnz();
    //
    return ok;
}
}  // End empty namespace
/* %$$
$head Test with u_1 Both a Variable and a Parameter$$
$srccode%cpp% */
bool jac_sparsity(void)
{   bool ok           = true;
    //
    bool u_1_variable = true;
    bool forward      = true;
    ok               &= use_jac_sparsity(u_1_variable, forward);
    //
    u_1_variable      = true;
    forward           = false;
    ok               &= use_jac_sparsity(u_1_variable, forward);
    //
    u_1_variable      = false;
    forward           = true;
    ok               &= use_jac_sparsity(u_1_variable, forward);
    //
    u_1_variable      = false;
    forward           = false;
    ok               &= use_jac_sparsity(u_1_variable, forward);
    //
    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
