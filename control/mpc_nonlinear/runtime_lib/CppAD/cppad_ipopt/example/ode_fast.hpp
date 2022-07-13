# ifndef CPPAD_CPPAD_IPOPT_EXAMPLE_ODE_FAST_HPP
# define CPPAD_CPPAD_IPOPT_EXAMPLE_ODE_FAST_HPP
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
$begin ipopt_nlp_ode_fast.hpp$$
$spell
    cppad_ipopt_nlp
$$

$section ODE Fitting Using Fast Representation$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/

// BEGIN C++
# include "ode_problem.hpp"

namespace {
    using namespace cppad_ipopt;

    class FG_fast : public cppad_ipopt_fg_info
    {
    private:
        bool       retape_;
        SizeVector N_;
        SizeVector S_;
    public:
        // derived class part of constructor
        FG_fast(bool retape_in, const SizeVector& N)
        : retape_ (retape_in), N_(N)
        {   assert( N_[0] == 0 );
            S_.resize( N_.size() );
            S_[0] = 0;
            for(size_t i = 1; i < N_.size(); i++)
                S_[i] = S_[i-1] + N_[i];
        }
        // r^k for k = 0, 1, ..., Nz-1 used for measurements
        // r^k for k = Nz              use for initial condition
        // r^k for k = Nz+1, ..., 2*Nz used for trapezoidal approx
        size_t number_functions(void)
        {   return Nz + 1 + Nz; }
        ADVector eval_r(size_t k, const ADVector &u)
        {   count_eval_r();

            size_t j;
            ADVector y(Ny), a(Na);
            // objective function --------------------------------
            if( k < Nz )
            {   // used for measurement with index k+1
                ADVector r(1); // return value is a scalar
                // u is [y( s[k+1] ) , a]
                for(j = 0; j < Ny; j++)
                    y[j] = u[j];
                for(j = 0; j < Na; j++)
                    a[j] = u[Ny + j];
                r[0] = eval_H<ADNumber>(k+1, y, a);
                return r;
            }
            // initial condition ---------------------------------
            if( k == Nz )
            {   ADVector r(Ny), F(Ny);
                // u is [y(t), a] at t = 0
                for(j = 0; j < Ny; j++)
                    y[j] = u[j];
                for(j = 0; j < Na; j++)
                    a[j] = u[Ny + j];
                F    = eval_F(a);
                for(j = 0; j < Ny; j++)
                    r[j]   = y[j] - F[j];
                return  r;
            }
            // trapezoidal approximation -------------------------
            ADVector ym(Ny), G(Ny), Gm(Ny), r(Ny);
            // r^k for k = Nz+1, ... , 2*Nz
            // interval between data samples
            Number T = s[k-Nz] - s[k-Nz-1];
            // integration step size
            Number dt = T / Number( N_[k-Nz] );
            // u = [ y(t[i-1], a) , y(t[i], a), a )
            for(j = 0; j < Ny; j++)
            {   ym[j] = u[j];
                y[j]  = u[Ny + j];
            }
            for(j = 0; j < Na; j++)
                a[j] = u[2 * Ny + j];
            Gm  = eval_G(ym, a);
            G   = eval_G(y,  a);
            for(j = 0; j < Ny; j++)
                r[j] = y[j] - ym[j] - (G[j] + Gm[j]) * dt / 2.;
            return r;
        }
        // The operations sequence for r_eval does not depend on u,
        // hence retape = false should work and be faster.
        bool retape(size_t k)
        {   return retape_; }
        // size of the vector u in eval_r
        size_t domain_size(size_t k)
        {   if( k < Nz )
                return Ny + Na;   // objective function
            if( k == Nz )
                return Ny + Na;  // initial value constraint
            return 2 * Ny + Na;      // trapezodial constraints
        }
        // size of the return value from eval_r
        size_t range_size(size_t k)
        {   if( k < Nz )
                return 1;
            return Ny;
        }
        // number of terms that use this value of k
        size_t number_terms(size_t k)
        {   if( k <= Nz )
                return 1;  // r^k used once for k <= Nz
            // r^k used N_[k-Nz] times for k > Nz
            return N_[k-Nz];
        }
        void index(size_t k, size_t ell, SizeVector& I, SizeVector& J)
        {   size_t i, j;
            // # of components of x corresponding to values for y
            size_t ny_inx = (S_[Nz] + 1) * Ny;
            // objective function -------------------------------
            if( k < Nz )
            {   // index in fg corresponding to objective
                I[0] = 0;
                // u = [ y(t, a) , a ]
                // The first Ny components of u is y(t) at
                //    t = s[k+1] = t[S_[k+1]]
                // x indices corresponding to this value of y
                for(j = 0; j < Ny; j++)
                    J[j] = S_[k + 1] * Ny + j;
                // components of x correspondig to a
                for(j = 0; j < Na; j++)
                    J[Ny + j] = ny_inx + j;
                return;
            }
            // initial conditions --------------------------------
            if( k == Nz )
            {   // index in fg for inidial condition constraint
                for(j = 0; j < Ny; j++)
                    I[j] = 1 + j;
                // u = [ y(t, a) , a ] where t = 0
                // x indices corresponding to this value of y
                for(j = 0; j < Ny; j++)
                    J[j] = j;
                // following that, u contains the vector a
                for(j = 0; j < Na; j++)
                    J[Ny + j] = ny_inx + j;
                return;
            }
            // trapoziodal approximation -------------------------
            // index of first grid point in this approximation
            i = S_[k - Nz - 1]  + ell;
            // There are Ny difference equations for each time
            // point.  Add one for the objective function, and Ny
            // for the initial value constraints.
            for(j = 0; j < Ny; j++)
                I[j] = 1 + Ny + i * Ny + j;
            // u = [ y(t, a) , y(t+dt, a) , a ] at t = t[i]
            for(j = 0; j < Ny; j++)
            {   J[j]      = i * Ny  + j; // y^i indices
                J[Ny + j] = J[j] + Ny;   // y^{i+1} indices
            }
            for(j = 0; j < Na; j++)
                J[2 * Ny + j] = ny_inx + j; // a indices
        }
    };

}
// END C++
# endif
