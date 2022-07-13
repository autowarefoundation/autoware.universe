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
$begin chkpoint_one_extended_ode.cpp$$
$spell
    Checkpointing
    Runge-Kutta
    mul
$$

$section Checkpointing an Extended ODE Solver: Example and Test$$
$index mul_level, checkpoint$$

$head See Also$$
$cref chkpoint_one_ode.cpp$$,
$cref chkpoint_one_mul_level.cpp$$.


$head Discussion$$
Suppose that we wish to extend an ODE to include derivatives with respect
to some parameter in the ODE. In addition, suppose we wish to
differentiate a function that depends on these derivatives.
Applying checkpointing to at the second level of AD would not work;
see $cref chkpoint_one_mul_level.cpp$$
In this example we show how one can do this by
checkpointing an extended ODE solver.

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
If $latex \tilde{z}_k (x) = z_k (x)$$, then
$latex \tilde{z}_{k+1} (x) = z_{k+1} (x) + O( \Delta t^5 )$$, then
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


$comment %example/chkpoint_one/extended_ode.cpp%0%// BEGIN C++%// END C++%1%$$

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

    // Derivative of 4-th Order Runge-Kutta Step w.r.t x
    a1vector Runge4_x(const a1vector& x, const a1vector& z0)
    {   assert( size_t( x.size() ) == n_ );
        assert( size_t( z0.size() ) == n_ );
        //
        a2vector ax(n_);
        for(size_t j = 0; j < n_; j++)
            ax[j] = x[j];
        //
        a2vector az0(n_);
        for(size_t i = 0; i < n_; i++)
            az0[i] = z0[i];
        //
        CppAD::Independent(ax);
        a2vector az(n_);
        az = Runge4(ax, az0);
        CppAD::ADFun<a1double> f(ax, az);
        //
        a1vector result =  f.Jacobian(x);
        //
        return result;
    }

    // Derivative of 4-th Order Runge-Kutta Step w.r.t z0
    a1vector Runge4_z0(const a1vector& x, const a1vector& z0)
    {   assert( size_t( x.size()  ) == n_ );
        assert( size_t( z0.size() ) == n_ );
        //
        a2vector ax(n_);
        for(size_t j = 0; j < n_; j++)
            ax[j] = x[j];
        //
        a2vector az0(n_);
        for(size_t i = 0; i < n_; i++)
            az0[i] = z0[i];
        //
        CppAD::Independent(az0);
        a2vector az(n_);
        az = Runge4(ax, az0);
        CppAD::ADFun<a1double> f(az0, az);
        //
        a1vector result =  f.Jacobian(z0);
        //
        return result;
    }

    // pack an extended ode vector
    template <class FloatVector>
    void pack(
        FloatVector&         extended_ode ,
        const FloatVector&   x            ,
        const FloatVector&   z            ,
        const FloatVector&   z_x          )
    {   assert( size_t( extended_ode.size() ) == n_ + n_ + n_ * n_ );
        assert( size_t( x.size()            ) == n_                );
        assert( size_t( z.size()            ) == n_                );
        assert( size_t( z_x.size()          ) == n_ * n_           );
        //
        size_t offset = 0;
        for(size_t i = 0; i < n_; i++)
            extended_ode[offset + i] = x[i];
        offset += n_;
        for(size_t i = 0; i < n_; i++)
            extended_ode[offset + i] = z[i];
        offset += n_;
        for(size_t i = 0; i < n_; i++)
        {   for(size_t j = 0; j < n_; j++)
            {   // partial of z_i (t , x ) w.r.t x_j
                extended_ode[offset + i * n_ + j] = z_x[i * n_ + j];
            }
        }
    }

    // unpack an extended ode vector
    template <class FloatVector>
    void unpack(
        const FloatVector&         extended_ode ,
        FloatVector&               x            ,
        FloatVector&               z            ,
        FloatVector&               z_x          )
    {   assert( size_t( extended_ode.size() ) == n_ + n_ + n_ * n_ );
        assert( size_t( x.size()            ) == n_                );
        assert( size_t( z.size()            ) == n_                );
        assert( size_t( z_x.size()          ) == n_ * n_           );
        //
        size_t offset = 0;
        for(size_t i = 0; i < n_; i++)
            x[i] = extended_ode[offset + i];
        offset += n_;
        for(size_t i = 0; i < n_; i++)
            z[i] = extended_ode[offset + i];
        offset += n_;
        for(size_t i = 0; i < n_; i++)
        {   for(size_t j = 0; j < n_; j++)
            {   // partial of z_i (t , x ) w.r.t x_j
                z_x[i * n_ + j] = extended_ode[offset + i * n_ + j];
            }
        }
    }

    // Algorithm that advances the partial of z(t, x) w.r.t x
    void ext_ode_algo(const a1vector& ext_ode_in, a1vector& ext_ode_out)
    {   assert( size_t( ext_ode_in.size()  ) == n_ + n_ + n_ * n_ );
        assert( size_t( ext_ode_out.size() ) == n_ + n_ + n_ * n_ );
        //
        // initial extended ode information
        a1vector x(n_), z0(n_), z0_x(n_ * n_);
        unpack(ext_ode_in, x, z0, z0_x);
        //
        // advance z(t, x)
        a1vector z1 = Runge4(x, z0);
        //
        // partial of z1 w.r.t. x
        a1vector z1_x = Runge4_x(x, z0);
        //
        // partial of z1 w.r.t. z0
        a1vector z1_z0 = Runge4_z0(x, z0);
        //
        // total derivative of z1 w.r.t x
        for(size_t i = 0; i < n_; i++)
        {   for(size_t j = 0; j < n_; j++)
            {   a1double sum = 0.0;
                for(size_t k = 0; k < n_; k++)
                    sum += z1_z0 [ i * n_ + k ] * z0_x [ k * n_ + j ];
                z1_x[ i * n_ + j] += sum;
            }
        }
        //
        // final extended ode information
        pack(ext_ode_out, x, z1, z1_x);
        //
        return;
    }
}
//
bool extended_ode(void)
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
    // set parameter value and initial value of the extended ode
    a1vector ax(n_), az0(n_), az0_x(n_ * n_);
    for(size_t i = 0; i < n_; i++)
    {   ax[i]  = a1double(i + 1);
        az0[i] = a1double(0);
        for(size_t j = 0; j < n_; j++)
            az0_x[ i * n_ + j ] = 0.0;
    }
    //
    // pack into extended ode information input vector
    size_t n_ext = n_ + n_ + n_ * n_;
    a1vector aext_ode_in(n_ext);
    pack(aext_ode_in, ax, az0, az0_x);
    //
    // create checkpoint version of the algorithm
    a1vector aext_ode_out(n_ext);
    CppAD::checkpoint<double> ext_ode_check(
        "ext_ode", ext_ode_algo, aext_ode_in, aext_ode_out
    );
    //
    // set the independent variables for recording
    CppAD::Independent( ax );
    //
    // repack to get dependence on ax
    pack(aext_ode_in, ax, az0, az0_x);
    //
    // Now run the checkpoint algorithm n_step times
    for(size_t k = 0; k < n_step; k++)
    {   ext_ode_check(aext_ode_in, aext_ode_out);
        aext_ode_in = aext_ode_out;
    }
    //
    // Unpack the results (must use ax1 so do not overwrite ax)
    a1vector ax1(n_), az1(n_), az1_x(n_ * n_);
    unpack(aext_ode_out, ax1, az1, az1_x);
    //
    // We could record a complicated funciton of x and z_x(T, x) in f,
    // but make this example simpler we record x -> z_x(T, x).
    CppAD::ADFun<double> f(ax, az1_x);
    //
    // check function values
    a0vector x(n_), z1(n_), z1_x(n_ * n_);
    for(size_t j = 0; j < n_; j++)
        x[j] = double(j + 1);
    z1_x = f.Forward(0, x);
    //
    // use z(t, x) for checking solution
    z1[0] = x[0] * T;
    for(size_t i = 1; i < n_; i++)
        z1[i] = x[i] * T * z1[i-1] / double(i+1);
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
    {   for(size_t j = 0; j < n_; j++)
        {   // check partial of z1_i w.r.t x_j
            double check = 0.0;
            if( j <= i )
                check = z1[i] / x[j];
            ok &= NearEqual(z1_x[ i * n_ + j ] , check, acc[i], acc[i]);
        }
    }
    //
    // Now use f to compute a derivative. For this 'simple' example it is
    // the derivative with respect to x of the
    // parital with respect to x[n-1] of z_{n-1} (t , x)
    a0vector w(n_ * n_), dw(n_);
    for(size_t i = 0; i < n_; i++)
    {   for(size_t j = 0; j < n_; j++)
        {   w[ i * n_ + j ] = 0.0;
            if( i == n_ - 1 && j == n_ - 1 )
                w[ i * n_ + j ] = 1.0;
        }
    }
    dw = f.Reverse(1, w);
    for(size_t j = 0; j < n_; j++)
    {   double check = 0.0;
        if( j < n_ - 1 )
            check = z1[n_ - 1] / ( x[n_ - 1] * x[j] );
        ok &= NearEqual(dw[j] , check, acc[n_-1], acc[n_-1]);
    }
    //
    return ok;
}
// END C++
