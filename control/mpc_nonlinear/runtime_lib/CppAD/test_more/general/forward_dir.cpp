/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
// test multiple directions operators
// MulvvOp is tested by example/forward_dir.cpp

# include <limits>
# include <cmath>
# include <cppad/cppad.hpp>

namespace {
    using CppAD::AD;
    using CppAD::NearEqual;
    // ---------------------------------------------------------------------
    // Used the check that fun is an idenity funciton
    typedef AD<double> (*adfun)(const AD<double>&);
    bool check_identity(adfun identity, double argument)
    {
        bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;


        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = argument;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = identity(ax[0]);

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0  (t)    = F[X_0(t)] = X_0(t)
        //             =  0.5 + 1t + 2t^2
        double y_1_0   = 1.0;
        double y_2_0   = 2.0;
        //
        // Y_1  (t)    = F[X_1(t)] = X_1(t)
        //             =  0.5 + 2t + 3t^2
        double y_1_1   = 2.0;
        double y_2_1   = 3.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // AbsOp
    bool abs_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 2;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;
        ax[1] = -1.0;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = fabs( ax[0] ) + fabs( 2.0 * ax[1] );

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0  (t)    = F[X_0(t)]
        //             = fabs(0.5 + 1t + 2t^2) + fabs( 2*(-1.0 + 2t + 3t^2 ) )
        double y_1_0   = -3.0;
        double y_2_0   = -4.0;
        //
        // Y_1  (t)    = F[X_1(t)]
        //             = fabs(0.5 + 2t + 3t^2) + fabs( 2*(-1.0 + 3t + 4t^2 ) )
        double y_1_1   = -4.0;
        double y_2_1   = -5.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // AddpvOp
    bool addpv_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = 2.0 + ax[0];

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 3);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0 (t)     = F[X_0(t)]
        //             =  2.0 + (0.5 + 1t + 3t^2)
        double y_1_0   = 1.0;
        double y_2_0   = 3.0;
        //
        // Y_1 (t)     = F[X_1(t)]
        //             =  2.0 + (0.5 + 2t + 4t^2)
        double y_1_1   = 2.0;
        double y_2_1   = 4.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // AddvvOp
    bool addvv_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 2;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;
        ax[1] = 2.0;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = ax[0] + ax[1];

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0 (t)     = F[X_0(t)]
        //             =  (0.5 + 1t + 2t^2) + (2.0 + 2t + 3t^2)
        double y_1_0   = 1.0 + 2.0;
        double y_2_0   = 2.0 + 3.0;
        //
        // Y_1 (t)     = F[X_1(t)]
        //             =  (2.0 + 2t + 3t^2) + (2.0 + 3t + 4t^2)
        double y_1_1   = 2.0 + 3.0;
        double y_2_1   = 3.0 + 4.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // CosOp
    bool cos_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;


        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = cos( ax[0] );

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0  (t)    = F[X_0(t)]
        //             =  cos( 0.5 + 1t + 2t^2 )
        // Y_0' (t)    = -sin( 0.5 + 1t + 2t^2) * (1 + 4t)
        double y_1_0   = - sin(0.5);
        double y_2_0   = - ( cos(0.5) + 4.0 * sin(0.5) ) / 2.0;
        //
        // Y_1  (t)    = F[X_1(t)]
        //             =  cos( 0.5 + 2t + 3t^2 )
        // Y_1' (t)    = -sin( 0.5 + 2t + 3t^2) * (2 + 6t)
        double y_1_1   = - sin(0.5) * 2.0;
        double y_2_1   = - ( cos(0.5) * 4.0 + 6.0 * sin(0.5) ) / 2.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // CoshOp
    bool cosh_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;


        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = cosh( ax[0] );

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0  (t)    = F[X_0(t)]
        //             = cosh( 0.5 + 1t + 2t^2 )
        // Y_0' (t)    = sinh( 0.5 + 1t + 2t^2) * (1 + 4t)
        double y_1_0   = sinh(0.5);
        double y_2_0   = ( sinh(0.5) * 4.0 + cosh(0.5) ) / 2.0;
        //
        // Y_1  (t)    = F[X_1(t)]
        //             = cosh( 0.5 + 2t + 3t^2 )
        // Y_1' (t)    = sinh( 0.5 + 2t + 3t^2) * (2 + 6t)
        double y_1_1   = sinh(0.5) * 2.0;
        double y_2_1   = ( sinh(0.5) * 6.0 + cosh(0.5) * 4.0 ) / 2.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // CExpOp
    bool cexp_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 4;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        for(j = 0; j < n; j++)
            ax[j] = double(j);

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 2;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = CondExpLt(ax[0], ax[1], ax[2], ax[3]);
        ay[1] = CondExpGt(ax[0], ax[1], ax[2], ax[3]);

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y0_0 (t)     = X2_0(t)
        //             =  2.0 + 3t + 4t^2
        double y0_1_0  = 3.0;
        double y0_2_0  = 4.0;
        //
        // Y1_0 (t)     = X3_0(t)
        //             =  3.0 + 4t + 5t^2
        double y1_1_0  = 4.0;
        double y1_2_0  = 5.0;
        //
        // Y0_1 (t)     = X2_1(t)
        //             =  2.0 + 4t + 5t^2
        double y0_1_1  = 4.0;
        double y0_2_1  = 5.0;
        //
        // Y1_1 (t)     = X3_0(t)
        //             =  3.0 + 5t + 6t^2
        double y1_1_1  = 5.0;
        double y1_2_1  = 6.0;
        //
        ok  &= NearEqual(y1[0*r+0] , y0_1_0, eps, eps);
        ok  &= NearEqual(y1[1*r+0] , y1_1_0, eps, eps);
        ok  &= NearEqual(y1[0*r+1] , y0_1_1, eps, eps);
        ok  &= NearEqual(y1[1*r+1] , y1_1_1, eps, eps);
        //
        ok  &= NearEqual(y2[0*r+0] , y0_2_0, eps, eps);
        ok  &= NearEqual(y2[1*r+0] , y1_2_0, eps, eps);
        ok  &= NearEqual(y2[0*r+1] , y0_2_1, eps, eps);
        ok  &= NearEqual(y2[1*r+1] , y1_2_1, eps, eps);
        //
        return ok;
    }

    // ---------------------------------------------------------------------
    // CSumOp
    bool csum_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 3;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        for(j = 0; j < n; j++)
            ax[j] = double(j);

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = 0.0;
        for(j = 0; j < n; j++)
            ay[0] += ax[j];

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // optmize the tape so converts summation to on CSumOp operator
        f.optimize();

        // zero order
        CPPAD_TESTVECTOR(double) x0(n);
        for(j = 0; j < n; j++)
            x0[j] = double(j);
        f.Forward(0, x0);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 3);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        double check = 0.0;
        for(j = 0; j < n; j++)
            check += x1[ r * j + 0];
        ok  &= NearEqual(y1[0] , check, eps, eps);
        //
        check = 0.0;
        for(j = 0; j < n; j++)
            check += x1[ r * j + 1];
        ok  &= NearEqual(y1[1] , check, eps, eps);
        //
        check = 0.0;
        for(j = 0; j < n; j++)
            check += x2[ r * j + 0];
        ok  &= NearEqual(y2[0] , check, eps, eps);
        //
        check = 0.0;
        for(j = 0; j < n; j++)
            check += x2[ r * j + 1];
        ok  &= NearEqual(y2[1] , check, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // DisOp (test assuming that AddvvOp is correct)
    double round_off(const double& x)
    {   // std::round(x); is C++11, so we avoid using it
        return std::floor( x + 0.5 );
    }
    CPPAD_DISCRETE_FUNCTION(double, round_off)
    bool dis_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = round_off( ax[0] ) + ax[0];

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // zero order
        CPPAD_TESTVECTOR(double) x0(n), y0;
        x0[0] = 2.2;
        y0  = f.Forward(0, x0);
        ok &= size_t( y0.size() ) == m;
        ok &= NearEqual(y0[0], round_off(x0[0]) + x0[0], eps, eps);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        //
        // Y_0 (t)     = F[X_0(t)]
        //             =  2.0 + (2.2 + 1t + 2t^2)
        double y_1_0   = 1.0;
        double y_2_0   = 2.0;
        //
        // Y_1 (t)     = F[X_1(t)]
        //             =  2.0 + (2.2 + 2t + 3t^2)
        double y_1_1   = 2.0;
        double y_2_1   = 3.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // DivpvOp (testing assumping MulpvOp is correct)
    bool divpv_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = (2.0 / ax[0]) * (ax[0] * ax[0]);

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0 (t)     = F[X_0(t)]
        //             = 2.0 * (0.5 + 1t + 2t^2)
        double y_1_0   = 2.0;
        double y_2_0   = 4.0;
        //
        // Y_1 (t)     = F[X_1(t)]
        //             = 2.0 * (0.5 + 2t + 3t^2)/2.0
        double y_1_1   = 4.0;
        double y_2_1   = 6.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // DivvpOp
    bool divvp_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = ax[0] / 2.0;

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 3);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0 (t)     = F[X_0(t)]
        //             =  (0.5 + 1t + 3t^2)/2.0
        double y_1_0   = 1.0 / 2.0;
        double y_2_0   = 3.0 / 2.0;
        //
        // Y_1 (t)     = F[X_1(t)]
        //             =  (0.5 + 2t + 4t^2)/2.0
        double y_1_1   = 2.0 / 2.0;
        double y_2_1   = 4.0 / 2.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // ExpOp
    bool exp_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = exp( ax[0] );

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0  (t)    = F[X_0(t)]
        //             =  exp(0.5 + 1t + 2t^2)
        // Y_0' (t)    =  exp(0.5 + 1t + 2t^2)*(1 + 4t)
        double y_1_0   = exp(0.5);
        double y_2_0   = ( exp(0.5)*4.0 + exp(0.5) ) / 2.0;
        //
        // Y_1  (t)    = F[X_1(t)]
        //             =  exp(0.5 + 2t + 3t^2)
        // Y_1' (t)    =  exp(0.5 + 2t + 3t^2)*(2 + 6t)
        double y_1_1   = exp(0.5)*2.0;
        double y_2_1   = ( exp(0.5)*6.0 + exp(0.5)*2.0*2.0 ) / 2.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // LdpOp and LdvOp (test assuming AdvvOp is correct)
    bool load_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 2;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.0;
        ax[1] = 1.0;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // Store operations
        CppAD::VecAD<double> avec(3);
        avec[ AD<double>(0) ]    = ax[0];  // store a variable
        avec[ AD<double>(1) ]    = ax[1];  // store a variable
        avec[ AD<double>(2) ]    = 5.0;    // store a parameter

        // range space vector
        size_t m = 2;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = avec[ AD<double>(0) ];     // load using parameter index
        ay[1] = avec[ ax[0] ];             // load using variable index

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // zero order Taylor coefficients
        CPPAD_TESTVECTOR(double) x0(n), y0;
        x0[0] = 2;
        x0[1] = 3;
        y0  = f.Forward(0, x0);
        ok &= size_t( y0.size() ) == m;
        // y[0] = avec[0] = x[0]
        ok &= y0[0] == x0[0];
        // y[1] = avec[ x[0] ] = avec[2] = 5.0
        ok &= y0[1] == 5.0;

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y0_0 (t)    = 2.0 + 1t + 2t^2
        double y0_1_0  = 1.0;
        double y0_2_0  = 2.0;
        //
        // Y1_0 (t)    = 5.0
        double y1_1_0  = 0.0;
        double y1_2_0  = 0.0;
        //
        // Y0_1 (t)    = 2.0 + 2t + 3t^2
        double y0_1_1  = 2.0;
        double y0_2_1  = 3.0;
        //
        // Y1_1 (t)    = 5.0
        double y1_1_1  = 0.0;
        double y1_2_1  = 0.0;
        //
        ok  &= NearEqual(y1[0*r+0] , y0_1_0, eps, eps);
        ok  &= NearEqual(y1[1*r+0] , y1_1_0, eps, eps);
        ok  &= NearEqual(y1[0*r+1] , y0_1_1, eps, eps);
        ok  &= NearEqual(y1[1*r+1] , y1_1_1, eps, eps);
        //
        ok  &= NearEqual(y2[0*r+0] , y0_2_0, eps, eps);
        ok  &= NearEqual(y2[1*r+0] , y1_2_0, eps, eps);
        ok  &= NearEqual(y2[0*r+1] , y0_2_1, eps, eps);
        ok  &= NearEqual(y2[1*r+1] , y1_2_1, eps, eps);
        //
        return ok;
    }

    // ---------------------------------------------------------------------
    // MulpvOp
    bool mulpv_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = 2.0 * ax[0];

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 3);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0 (t)     = F[X_0(t)]
        //             =  2.0 * (0.5 + 1t + 3t^2)
        double y_1_0   = 2.0 * 1.0;
        double y_2_0   = 2.0 * 3.0;
        //
        // Y_1 (t)     = F[X_1(t)]
        //             =  2.0 * (0.5 + 2t + 4t^2)
        double y_1_1   = 2.0 * 2.0;
        double y_2_1   = 2.0 * 4.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // ParOp
    bool par_op(void)
    {   bool ok = true;
        size_t j;

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = 0.0 * ax[0];

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0  (t)    = 0.0
        for(ell = 0; ell < r; ell++)
        {   ok &= y1[ell] == 0.0;
            ok &= y2[ell] == 0.0;
        }
        return ok;
    }
    // ---------------------------------------------------------------------
    // PowvvOp  (test assuming LogOp, ExpOp and DivvvOp are correct)
    bool powvv_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 2;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;
        ax[1] = 2.0;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 2;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = log( pow( exp(ax[0]) , ax[1] ) ) / ax[1] ;
        ay[1] = log( pow( exp(ax[0]) , ax[1] ) ) / ax[0] ;

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y0_0 (t)    = 0.5 + 1t + 2t^2
        double y0_1_0  = 1.0;
        double y0_2_0  = 2.0;
        //
        // Y0_1 (t)    = 0.5 + 2t + 3t^2
        double y0_1_1  = 2.0;
        double y0_2_1  = 3.0;
        //
        // Y1_0 (t)    = 2.0 + 2t + 3t^2
        double y1_1_0  = 2.0;
        double y1_2_0  = 3.0;
        //
        // Y1_1 (t)    = 2.0 + 3t + 4t^2
        double y1_1_1  = 3.0;
        double y1_2_1  = 4.0;
        //
        ok  &= NearEqual(y1[0*r+0] , y0_1_0, eps, eps);
        ok  &= NearEqual(y1[1*r+0] , y1_1_0, eps, eps);
        ok  &= NearEqual(y1[0*r+1] , y0_1_1, eps, eps);
        ok  &= NearEqual(y1[1*r+1] , y1_1_1, eps, eps);
        //
        ok  &= NearEqual(y2[0*r+0] , y0_2_0, eps, eps);
        ok  &= NearEqual(y2[1*r+0] , y1_2_0, eps, eps);
        ok  &= NearEqual(y2[0*r+1] , y0_2_1, eps, eps);
        ok  &= NearEqual(y2[1*r+1] , y1_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // SignOp (test assuming that MulvvOp is correct)
    bool sign_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = sign( ax[0] ) * ax[0];

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // zero order
        CPPAD_TESTVECTOR(double) x0(n), y0;
        x0[0] = -3.0;
        y0  = f.Forward(0, x0);
        ok &= size_t( y0.size() ) == m;
        ok &= NearEqual(y0[0], fabs(x0[0]), eps, eps);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        //
        // Y_0 (t)     = F[X_0(t)]
        //             =  -(-3.0 + 1t + 2t^2)
        double y_1_0   = -1.0;
        double y_2_0   = -2.0;
        //
        // Y_1 (t)     = F[X_1(t)]
        //             =  -(-3.0 + 2t + 3t^2)
        double y_1_1   = -2.0;
        double y_2_1   = -3.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }

    // ---------------------------------------------------------------------
    // SinOp
    bool sin_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;


        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = sin( ax[0] );

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0  (t)    = F[X_0(t)]
        //             = sin( 0.5 + 1t + 2t^2 )
        // Y_0' (t)    = cos( 0.5 + 1t + 2t^2) * (1 + 4t)
        double y_1_0   = cos(0.5);
        double y_2_0   = ( cos(0.5) * 4.0 - sin(0.5) ) / 2.0;
        //
        // Y_1  (t)    = F[X_1(t)]
        //             = sin( 0.5 + 2t + 3t^2 )
        // Y_1' (t)    = cos( 0.5 + 2t + 3t^2) * (2 + 6t)
        double y_1_1   = cos(0.5) * 2.0;
        double y_2_1   = ( cos(0.5) * 6.0 - sin(0.5) * 4.0 ) / 2.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // SinhOp
    bool sinh_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;


        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = sinh( ax[0] );

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0  (t)    = F[X_0(t)]
        //             = sinh( 0.5 + 1t + 2t^2 )
        // Y_0' (t)    = cosh( 0.5 + 1t + 2t^2) * (1 + 4t)
        double y_1_0   = cosh(0.5);
        double y_2_0   = ( cosh(0.5) * 4.0 + sinh(0.5) ) / 2.0;
        //
        // Y_1  (t)    = F[X_1(t)]
        //             = sinh( 0.5 + 2t + 3t^2 )
        // Y_1' (t)    = cosh( 0.5 + 2t + 3t^2) * (2 + 6t)
        double y_1_1   = cosh(0.5) * 2.0;
        double y_2_1   = ( cosh(0.5) * 6.0 + sinh(0.5) * 4.0 ) / 2.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // SubpvOp
    bool subpv_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = 2.0 - ax[0];

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 3);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0 (t)     = F[X_0(t)]
        //             =  2.0 - (0.5 + 1t + 3t^2)/2.0
        double y_1_0   = - 1.0;
        double y_2_0   = - 3.0;
        //
        // Y_1 (t)     = F[X_1(t)]
        //             =  3.0 - (0.5 + 2t + 4t^2)/2.0
        double y_1_1   = - 2.0;
        double y_2_1   = - 4.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // SubvvOp
    bool subvv_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 2;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;
        ax[1] = 2.0;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = ax[0] - 2.0 * ax[1];

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0 (t)     = F[X_0(t)]
        //             =  (0.5 + 1t + 2t^2) - 2.0 * (2.0 + 2t + 3t^2)
        double y_1_0   = 1.0 - 4.0;
        double y_2_0   = 2.0 - 6.0;
        //
        // Y_1 (t)     = F[X_1(t)]
        //             =  (2.0 + 2t + 3t^2) - 2.0 * (2.0 + 3t + 4t^2)
        double y_1_1   = 2.0 - 6.0;
        double y_2_1   = 3.0 - 8.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // TanOp
    bool tan_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // domain space vector
        size_t n = 1;
        CPPAD_TESTVECTOR(AD<double>) ax(n);
        ax[0] = 0.5;

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // range space vector
        size_t m = 1;
        CPPAD_TESTVECTOR(AD<double>) ay(m);
        ay[0] = tan( ax[0] );

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y_0  (t)    = F[X_0(t)]
        //             =  tan(0.5 + 1t + 2t^2)
        // Y_0' (t)    =  cos(0.5 + 1t + 2t^2)^(-2)*(1 + 4t)
        // Y_0''(0)    = 2*cos(0.5)^(-3)*sin(0.5) + 4*cos(0.5)^(-2)
        double sec_sq  = 1.0 / ( cos(0.5) * cos(0.5) );
        double y_1_0   = sec_sq;
        double y_2_0   = (2.0 * tan(0.5) + 4.0) * sec_sq / 2.0;
        //
        // Y_1  (t)    = F[X_1(t)]
        //             = tan(0.5 + 2t + 3t^2)
        // Y_1' (t)    = cos(0.5 + 2t + 3t^2)^(-2)*(2 + 6t)
        // Y_1''(0)    = 2*cos(0.5)^(-3)*sin(0.5)*2*2 + 6*cos(0.5)^(-2)
        double y_1_1   = 2.0 * sec_sq;
        double y_2_1   = (8.0 * tan(0.5) + 6.0) * sec_sq / 2.0;
        //
        ok  &= NearEqual(y1[0] , y_1_0, eps, eps);
        ok  &= NearEqual(y1[1] , y_1_1, eps, eps);
        ok  &= NearEqual(y2[0] , y_2_0, eps, eps);
        ok  &= NearEqual(y2[1] , y_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // Usr*Op
    typedef CPPAD_TESTVECTOR(AD<double>) avector;
    void usr_algo(const avector& x, avector& z)
    {   z[0] = ( x[0] + x[1] ) / 2.0;
        z[1] = x[0] * x[1];
        z[2] = ( x[0] - x[1] ) / 2.0;
        return;
    }
    bool usr_op(void)
    {   bool ok = true;
        double eps = 10. * std::numeric_limits<double>::epsilon();
        size_t j;

        // define checkpoint function
        size_t n = 2;
        avector ax(n), az(3);
        ax[0] = 0.5;
        ax[1] = 2.0;
        CppAD::checkpoint<double> usr_check("usr_check", usr_algo, ax, az);

        // declare independent variables and starting recording
        CppAD::Independent(ax);

        // record checkpoint function
        usr_check(ax, az);

        // range space vector
        size_t m = 2;
        avector ay(m);
        ay[0] = az[0] + az[2]; // = ax[0]
        ay[1] = az[0] - az[2]; // = ax[1]

        // create f: x -> y and stop tape recording
        CppAD::ADFun<double> f(ax, ay);

        // first order Taylor coefficients
        size_t r = 2, ell;
        CPPAD_TESTVECTOR(double) x1(r*n), y1;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x1[ r * j + ell ] = double(j + ell + 1);
        }
        y1  = f.Forward(1, r, x1);
        ok &= size_t( y1.size() ) == r*m;

        // secondorder Taylor coefficients
        CPPAD_TESTVECTOR(double) x2(r*n), y2;
        for(ell = 0; ell < r; ell++)
        {   for(j = 0; j < n; j++)
                x2[ r * j + ell ] = double(j + ell + 2);
        }
        y2  = f.Forward(2, r, x2);
        ok &= size_t( y2.size() ) == r*m;
        //
        // Y0_0 (t)    = 0.5 + 1t + 2t^2
        double y0_1_0  = 1.0;
        double y0_2_0  = 2.0;
        //
        // Y0_1 (t)    = 0.5 + 2t + 3t^2
        double y0_1_1  = 2.0;
        double y0_2_1  = 3.0;
        //
        // Y1_0 (t)    = 2.0 + 2t + 3t^2
        double y1_1_0  = 2.0;
        double y1_2_0  = 3.0;
        //
        // Y1_1 (t)    = 2.0 + 3t + 4t^2
        double y1_1_1  = 3.0;
        double y1_2_1  = 4.0;
        //
        ok  &= NearEqual(y1[0*r+0] , y0_1_0, eps, eps);
        ok  &= NearEqual(y1[1*r+0] , y1_1_0, eps, eps);
        ok  &= NearEqual(y1[0*r+1] , y0_1_1, eps, eps);
        ok  &= NearEqual(y1[1*r+1] , y1_1_1, eps, eps);
        //
        ok  &= NearEqual(y2[0*r+0] , y0_2_0, eps, eps);
        ok  &= NearEqual(y2[1*r+0] , y1_2_0, eps, eps);
        ok  &= NearEqual(y2[0*r+1] , y0_2_1, eps, eps);
        ok  &= NearEqual(y2[1*r+1] , y1_2_1, eps, eps);
        //
        return ok;
    }
    // ---------------------------------------------------------------------
    // Inverse functions assume the following already tested:
    // CosOp, SinOp, TanOp, ExpOp, MulvvOp, DivvpOp, AddpvOp
    //
    // AcosOp
    AD<double> acos_fun(const AD<double>& x)
    {   return acos( cos(x) ); }
    bool acos_op(void)
    {   return check_identity(acos_fun, 0.5); }
    //
    // AcoshOp
    AD<double> acosh_fun(const AD<double>& x)
    {   return acosh( cosh(x) ); }
    bool acosh_op(void)
    {   return check_identity(acosh_fun, 0.5); }
    //
    // AsinOp
    AD<double> asin_fun(const AD<double>& x)
    {   return asin( sin(x) ); }
    bool asin_op(void)
    {   return check_identity(asin_fun, 0.5); }
    //
    // AsinhOp
    AD<double> asinh_fun(const AD<double>& x)
    {   return asinh( sinh(x) ); }
    bool asinh_op(void)
    {   return check_identity(asinh_fun, 0.5); }
    //
    // AtanOp
    AD<double> atan_fun(const AD<double>& x)
    {   return atan( tan(x) ); }
    bool atan_op(void)
    {   return check_identity(atan_fun, 0.5); }
    //
    // AtanhOp
    AD<double> atanh_fun(const AD<double>& x)
    {   return atanh( tanh(x) ); }
    bool atanh_op(void)
    {   return check_identity(atanh_fun, 0.5); }
    //
    // LogOp
    AD<double> log_fun(const AD<double>& x)
    {   return log( exp(x) ); }
    bool log_op(void)
    {   return check_identity(log_fun, 0.5); }
    //
    // DivvvOp
    AD<double> divvv_fun(const AD<double>& x)
    {   return (x * x) / x; }
    bool divvv_op(void)
    {   return check_identity(divvv_fun, 0.5); }
    //
    // PowpvOp
    AD<double> powpv_fun(const AD<double>& x )
    {   return log( pow( exp(3.0) , x ) ) / 3.0; }
    bool powpv_op(void)
    {   return check_identity(powpv_fun, 0.5); }
    //
    // PowvpOp
    AD<double> powvp_fun(const AD<double>& x )
    {   return log( pow( exp(x) , 3.0 ) ) / 3.0; }
    bool powvp_op(void)
    {   return check_identity(powvp_fun, 0.5); }
    //
    // SqrtOp
    AD<double> sqrt_fun(const AD<double>& x )
    {   return sqrt( x * x ); }
    bool sqrt_op(void)
    {   return check_identity(sqrt_fun, 0.5); }
    //
    // SubvpOp
    AD<double> subvp_fun(const AD<double>& x )
    {   return 3.0 + ( x - 3.0 ); }
    bool subvp_op(void)
    {   return check_identity(subvp_fun, 0.5); }
    //
    // TanhOp
    AD<double> tanh_fun(const AD<double>& x )
    {   AD<double> z = tanh(x);
        return log( (1.0 + z) / (1.0 - z) ) / 2.0;
    }
    bool tanh_op(void)
    {   return check_identity(tanh_fun, 0.5); }
}

bool forward_dir(void)
{   bool ok = true;
    //
    ok     &= abs_op();
    ok     &= acos_op();
    ok     &= acosh_op();
    ok     &= asin_op();
    ok     &= asinh_op();
    ok     &= atan_op();
    ok     &= atanh_op();
    ok     &= addpv_op();
    ok     &= addvv_op();
    ok     &= cexp_op();
    ok     &= cosh_op();
    ok     &= cos_op();
    ok     &= csum_op();
    ok     &= dis_op();
    ok     &= divpv_op();
    ok     &= divvp_op();
    ok     &= divvv_op();
    ok     &= exp_op();
    ok     &= load_op();
    ok     &= log_op();
    ok     &= mulpv_op();
    ok     &= par_op();
    ok     &= powpv_op();
    ok     &= powvp_op();
    ok     &= powvv_op();
    ok     &= sign_op();
    ok     &= sin_op();
    ok     &= sinh_op();
    ok     &= subpv_op();
    ok     &= subvp_op();
    ok     &= subvv_op();
    ok     &= sqrt_op();
    ok     &= tan_op();
    ok     &= tanh_op();
    ok     &= usr_op();
    //
    return ok;
}
