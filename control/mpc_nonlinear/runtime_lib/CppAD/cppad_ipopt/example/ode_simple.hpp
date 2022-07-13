# ifndef CPPAD_CPPAD_IPOPT_EXAMPLE_ODE_SIMPLE_HPP
# define CPPAD_CPPAD_IPOPT_EXAMPLE_ODE_SIMPLE_HPP
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
$begin ipopt_nlp_ode_simple.hpp$$
$spell
    cppad_ipopt_nlp
    Nz
    Ny
    Na
$$

$section ODE Fitting Using Simple Representation$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/

// BEGIN C++
# include "ode_problem.hpp"

// define in the empty namespace
namespace {
    using namespace cppad_ipopt;

    class FG_simple : public cppad_ipopt_fg_info
    {
    private:
        bool       retape_;
        SizeVector N_;
        SizeVector S_;
    public:
        // derived class part of constructor
        FG_simple(bool retape_in, const SizeVector& N)
        : retape_ (retape_in), N_(N)
        {   assert( N_[0] == 0 );
            S_.resize( N.size() );
            S_[0] = 0;
            for(size_t i = 1; i < N_.size(); i++)
                S_[i] = S_[i-1] + N_[i];
        }
        // Evaluation of the objective f(x), and constraints g(x)
        // using an Algorithmic Differentiation (AD) class.
        ADVector eval_r(size_t not_used, const ADVector&  x)
        {   count_eval_r();

            // temporary indices
            size_t i, j, k;
            // # of components of x corresponding to values for y
            size_t ny_inx = (S_[Nz] + 1) * Ny;
            // # of constraints (range dimension of g)
            size_t m = ny_inx;
            // # of components in x (domain dimension for f and g)
            assert ( x.size() == ny_inx + Na );
            // vector for return value
            ADVector fg(m + 1);
            // vector of parameters
            ADVector a(Na);
            for(j = 0; j < Na; j++)
                a[j] = x[ny_inx + j];
            // vector for value of y(t)
            ADVector y(Ny);
            // objective function -------------------------------
            fg[0] = 0.;
            for(k = 0; k < Nz; k++)
            {   for(j = 0; j < Ny; j++)
                    y[j] = x[Ny*S_[k+1] + j];
                fg[0] += eval_H<ADNumber>(k+1, y, a);
            }
            // initial condition ---------------------------------
            ADVector F = eval_F(a);
            for(j = 0; j < Ny; j++)
            {   y[j]    = x[j];
                fg[1+j] = y[j] - F[j];
            }
            // trapezoidal approximation --------------------------
            ADVector ym(Ny), G(Ny), Gm(Ny);
            G = eval_G(y, a);
            ADNumber dy;
            for(k = 0; k < Nz; k++)
            {   // interval between data points
                Number T  = s[k+1] - s[k];
                // integration step size
                Number dt = T / Number( N_[k+1] );
                for(j = 0; j < N_[k+1]; j++)
                {   size_t Index = (j + S_[k]) * Ny;
                    // y(t) at end of last step
                    ym = y;
                    // G(y, a) at end of last step
                    Gm = G;
                    // value of y(t) at end of this step
                    for(i = 0; i < Ny; i++)
                        y[i] = x[Ny + Index + i];
                    // G(y, a) at end of this step
                    G = eval_G(y, a);
                    // trapezoidal approximation residual
                    for(i = 0; i < Ny; i++)
                    {   dy = (G[i] + Gm[i]) * dt / 2;
                        fg[1+Ny+Index+i] =
                            y[i] - ym[i] - dy;
                    }
                }
            }
            return fg;
        }
        // The operations sequence for r_eval does not depend on u,
        // hence retape = false should work and be faster.
        bool retape(size_t k)
        {   return retape_; }
    };

}
// END C++
# endif
