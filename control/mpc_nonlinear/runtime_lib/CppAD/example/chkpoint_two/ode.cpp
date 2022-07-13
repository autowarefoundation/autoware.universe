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
$begin chkpoint_two_ode.cpp$$
$spell
    Checkpointing
    Runge-Kutta
$$

$section Checkpointing an ODE Solver: Example and Test$$

$head Purpose$$
In this example we $cref/checkpoint/chkpoint_two/$$ one step of an ODE solver.

$head Problem$$
We consider the initial value problem with parameter $latex x$$ defined by,
$latex z(0, x) = z_0 (x)$$,
$latex \[
    \partial_t z(t, x ) = h [ x , z(t, x) ]
\]$$
Note that if $latex t$$ needs to be in the equation, one can define
the first component of $latex z(t, x)$$ to be equal to $latex t$$.

$head ODE Solver$$
For this example, we consider the Fourth order Runge-Kutta ODE solver.
Given an approximation solution at time $latex t_k$$ denoted by
$latex \tilde{z}_k (x)$$, and $latex \Delta t = t_{k+1} - t_k$$,
it defines the approximation solution $latex \tilde{z}_{k+1} (x)$$
at time $latex t_{k+1}$$ by
$latex \[
\begin{array}{rcl}
h_1 & =  & h [ x , \tilde{z}_k (x) ]
\\
h_2 & =  & h [ x , \tilde{z}_k (x) + \Delta t \; h_1 / 2 ]
\\
h_3 & =  & h [ x , \tilde{z}_k (x) + \Delta t \; h_2 / 2 ]
\\
h_4 & =  & h [ x , \tilde{z}_k (x) + \Delta t \; h_3 ]
\\
\tilde{z}_{k+1} (x) & = &
    \tilde{z}_k (x) + \Delta t \; ( h_1 +  2 h_2 + 2 h_3 + h_4 ) / 6
\end{array}
\] $$
If $latex \tilde{z}_k (x) = z_k (x)$$,
$latex \tilde{z}_{k+1} (x) = z_{k+1} (x) + O( \Delta t^5 )$$.
Other ODE solvers can use a similar method to the one used below.

$head ODE$$
For this example the ODE is defined by
$latex z(0, x) = 0$$ and
$latex \[
    h[ x, z(t, x) ] =
    \left( \begin{array}{c}
            x_0                     \\
            x_1 z_0 (t, x)          \\
            \vdots                  \\
            x_{n-1} z_{n-2} (t, x)
    \end{array} \right)
    =
    \left( \begin{array}{c}
            \partial_t z_0 (t , x)      \\
            \partial_t z_1 (t , x)      \\
            \vdots                      \\
            \partial_t z_{n-1} (t , x)
    \end{array} \right)
\] $$

$head Solution$$
The solution of the ODE for this example,
which is used to check the results,
can be calculated by
starting with the first row and then using the solution
for the first row to solve the second and so on.
Doing this we obtain
$latex \[
    z(t, x) =
    \left( \begin{array}{c}
        x_0 t                  \\
        x_1 x_0 t^2 / 2        \\
        \vdots                 \\
        x_{n-1} x_{n-2} \ldots x_0 t^n / n !
    \end{array} \right)
\] $$


$head Source$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace {
    using CppAD::AD;
    typedef AD<double>                     a1double;
    typedef AD<a1double>                   a2double;
    //
    typedef CPPAD_TESTVECTOR(   double )   a0vector;
    typedef CPPAD_TESTVECTOR( a1double )   a1vector;
    typedef CPPAD_TESTVECTOR( a2double )   a2vector;
    //
    // set once by main and kept that way
    double delta_t_ = std::numeric_limits<double>::quiet_NaN();
    size_t n_       = 0;
    //
    // The function h( x , y)
    template <class FloatVector>
    FloatVector h(const FloatVector& x, const FloatVector& y)
    {   assert( size_t( x.size() ) == n_ );
        assert( size_t( y.size() ) == n_ );
        FloatVector result(n_);
        result[0] = x[0];
        for(size_t i = 1; i < n_; i++)
            result[i] = x[i] * y[i-1];
        return result;
    }

    // The 4-th Order Runge-Kutta Step
    template <class FloatVector>
    FloatVector Runge4(const FloatVector& x, const FloatVector& z0
    )
    {   assert( size_t( x.size() ) == n_ );
        assert( size_t( z0.size() ) == n_ );
        //
        typedef typename FloatVector::value_type Float;
        //
        Float  dt = Float(delta_t_);
        size_t m  = z0.size();
        //
        FloatVector h1(m), h2(m), h3(m), h4(m), result(m);
        h1 = h( x, z0 );
        //
        for(size_t i = 0; i < m; i++)
            h2[i] = z0[i] + dt * h1[i] / 2.0;
        h2 = h( x, h2 );
        //
        for(size_t i = 0; i < m; i++)
            h3[i] = z0[i] + dt * h2[i] / 2.0;
        h3 = h( x, h3 );
        //
        for(size_t i = 0; i < m; i++)
            h4[i] = z0[i] + dt * h3[i];
        h4 = h( x, h4 );
        //
        for(size_t i = 0; i < m; i++)
        {   Float dz = dt * ( h1[i] + 2.0*h2[i] + 2.0*h3[i] + h4[i] ) / 6.0;
            result[i] = z0[i] + dz;
        }
        return result;
    }

    // pack x and z into an axz vector
    template <class FloatVector>
    void pack(
        FloatVector&         axz      ,
        const FloatVector&   x        ,
        const FloatVector&   z        )
    {   assert( size_t( axz.size() ) == n_ + n_ );
        assert( size_t( x.size()        ) == n_      );
        assert( size_t( z.size()        ) == n_      );
        //
        size_t offset = 0;
        for(size_t i = 0; i < n_; i++)
            axz[offset + i] = x[i];
        offset += n_;
        for(size_t i = 0; i < n_; i++)
            axz[offset + i] = z[i];
    }

    // unpack an axz vector
    template <class FloatVector>
    void unpack(
        const FloatVector&         axz      ,
        FloatVector&               x        ,
        FloatVector&               z        )
    {   assert( size_t( axz.size() ) == n_ + n_ );
        assert( size_t( x.size()        ) == n_      );
        assert( size_t( z.size()        ) == n_      );
        //
        size_t offset = 0;
        for(size_t i = 0; i < n_; i++)
            x[i] = axz[offset + i];
        offset += n_;
        for(size_t i = 0; i < n_; i++)
            z[i] = axz[offset + i];
    }

    // Algorithm that z(t, x)
    void ode_algo(const a1vector& axz_in, a1vector& axz_out)
    {   assert( size_t( axz_in.size()  ) == n_ + n_ );
        assert( size_t( axz_out.size() ) == n_ + n_ );
        //
        // initial ode information
        a1vector x(n_), z0(n_);
        unpack(axz_in, x, z0);
        //
        // advance z(t, x)
        a1vector z1 = Runge4(x, z0);
        //
        // final ode information
        pack(axz_out, x, z1);
        //
        return;
    }
}
//
bool ode(void)
{   bool ok = true;
    using CppAD::NearEqual;
    double eps = std::numeric_limits<double>::epsilon();
    //
    // number of terms in the differential equation
    n_ = 6;
    //
    // step size for the differentiail equation
    size_t n_step = 10;
    double T      = 1.0;
    delta_t_ = T / double(n_step);
    //
    // set parameter value and initial value of the ode
    a1vector ax(n_), az0(n_);
    for(size_t i = 0; i < n_; i++)
    {   ax[i]  = a1double(i + 1);
        az0[i] = a1double(0);
    }
    //
    // pack ode information input vector
    //
    // function corresponding to one step of the ode Algorithm
    a1vector axz_in(2 * n_), axz_out(2 * n_);
    pack(axz_in, ax, az0);
    CppAD::Independent(axz_in);
    ode_algo(axz_in, axz_out);
    CppAD::ADFun<double> ode_fun(axz_in, axz_out);
    //
    // create checkpoint version of the algorithm
    bool internal_bool    = false;
    bool use_hes_sparsity = false;
    bool use_base2ad      = false;
    bool use_in_parallel  = false;
    CppAD::chkpoint_two<double> ode_check(ode_fun, "ode",
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );
    //
    // set the independent variables for recording
    CppAD::Independent( ax );
    //
    // repack to get dependence on ax
    pack(axz_in, ax, az0);
    //
    // Now run the checkpoint algorithm n_step times
    for(size_t k = 0; k < n_step; k++)
    {   ode_check(axz_in, axz_out);
        axz_in = axz_out;
    }
    //
    // Unpack the results (must use ax1 so do not overwrite ax)
    a1vector ax1(n_), az1(n_);
    unpack(axz_out, ax1, az1);
    //
    // We could record a complicated funciton of x and z(T, x) in f,
    // but make this example simpler we record x -> z(T, x).
    CppAD::ADFun<double> f(ax, az1);
    //
    // check function values
    a0vector x(n_), z1(n_);
    for(size_t j = 0; j < n_; j++)
        x[j] = double(j + 1);
    z1 = f.Forward(0, x);
    //
    // separate calculation of z(t, x)
    a0vector check_z1(n_);
    check_z1[0] = x[0] * T;
    for(size_t i = 1; i < n_; i++)
        check_z1[i] = x[i] * T * check_z1[i-1] / double(i+1);
    //
    // expected accuracy for each component of of z(t, x)
    a0vector acc(n_);
    for(size_t i = 0; i < n_; i++)
    {   if( i < 4 )
        {   // Runge-Kutta methos is exact for this case
            acc[i] = 10. * eps;
        }
        else
        {   acc[i] = 1.0;
            for(size_t k = 0; k < 5; k++)
                    acc[i] *= x[k] * delta_t_;
        }
    }
    // check z1(T, x)
    for(size_t i = 0; i < n_; i++)
        ok &= NearEqual(z1[i] , check_z1[i], acc[i], acc[i]);
    //
    // Now use f to compute a derivative. For this 'simple' example it is
    // the derivative of z_{n-1} (T, x) respect to x of the
    a0vector w(n_), dw(n_);
    for(size_t i = 0; i < n_; i++)
    {   w[i] = 0.0;
        if( i == n_ - 1 )
            w[i] = 1.0;
    }
    dw = f.Reverse(1, w);
    for(size_t j = 0; j < n_; j++)
    {   double check = z1[n_ - 1] / x[j];
        ok &= NearEqual(dw[j] , check, 100.*eps, 100.*eps);
    }
    //
    return ok;
}
// END C++
