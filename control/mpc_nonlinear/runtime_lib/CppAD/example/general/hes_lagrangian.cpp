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
$begin hes_lagrangian.cpp$$
$spell
    Cpp
    HesLagrangian
$$

$comment ! NOTE the title states that this example is used two places !$$
$section Hessian of Lagrangian and ADFun Default Constructor: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cassert>

namespace {
    CppAD::AD<double> Lagragian(
        const CppAD::vector< CppAD::AD<double> > &xyz )
    {   using CppAD::AD;
        assert( xyz.size() == 6 );

        AD<double> x0 = xyz[0];
        AD<double> x1 = xyz[1];
        AD<double> x2 = xyz[2];
        AD<double> y0 = xyz[3];
        AD<double> y1 = xyz[4];
        AD<double> z  = xyz[5];

        // compute objective function
        AD<double> f = x0 * x0;
        // compute constraint functions
        AD<double> g0 = 1. + 2.*x1 + 3.*x2;
        AD<double> g1 = log( x0 * x2 );
        // compute the Lagragian
        AD<double> L = y0 * g0 + y1 * g1 + z * f;

        return L;

    }
    CppAD::vector< CppAD::AD<double> > fg(
        const CppAD::vector< CppAD::AD<double> > &x )
    {   using CppAD::AD;
        using CppAD::vector;
        assert( x.size() == 3 );

        vector< AD<double> > fg(3);
        fg[0] = x[0] * x[0];
        fg[1] = 1. + 2. * x[1] + 3. * x[2];
        fg[2] = log( x[0] * x[2] );

        return fg;
    }
    bool CheckHessian(
    CppAD::vector<double> H ,
    double x0, double x1, double x2, double y0, double y1, double z )
    {   using CppAD::NearEqual;
        double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
        bool ok  = true;
        size_t n = 3;
        assert( H.size() == n * n );
        /*
        L   =    z*x0*x0 + y0*(1 + 2*x1 + 3*x2) + y1*log(x0*x2)

        L_0 = 2 * z * x0 + y1 / x0
        L_1 = y0 * 2
        L_2 = y0 * 3 + y1 / x2
        */
        // L_00 = 2 * z - y1 / ( x0 * x0 )
        double check = 2. * z - y1 / (x0 * x0);
        ok &= NearEqual(H[0 * n + 0], check, eps99, eps99);
        // L_01 = L_10 = 0
        ok &= NearEqual(H[0 * n + 1], 0., eps99, eps99);
        ok &= NearEqual(H[1 * n + 0], 0., eps99, eps99);
        // L_02 = L_20 = 0
        ok &= NearEqual(H[0 * n + 2], 0., eps99, eps99);
        ok &= NearEqual(H[2 * n + 0], 0., eps99, eps99);
        // L_11 = 0
        ok &= NearEqual(H[1 * n + 1], 0., eps99, eps99);
        // L_12 = L_21 = 0
        ok &= NearEqual(H[1 * n + 2], 0., eps99, eps99);
        ok &= NearEqual(H[2 * n + 1], 0., eps99, eps99);
        // L_22 = - y1 / (x2 * x2)
        check = - y1 / (x2 * x2);
        ok &= NearEqual(H[2 * n + 2], check, eps99, eps99);

        return ok;
    }
    bool UseL()
    {   using CppAD::AD;
        using CppAD::vector;

        // double values corresponding to x, y, and z vectors
        double x0(.5), x1(1e3), x2(1), y0(2.), y1(3.), z(4.);

        // domain space vector
        size_t n = 3;
        vector< AD<double> >  a_x(n);
        a_x[0] = x0;
        a_x[1] = x1;
        a_x[2] = x2;

        // declare a_x as independent variable vector and start recording
        CppAD::Independent(a_x);

        // vector including x, y, and z
        vector< AD<double> > a_xyz(n + 2 + 1);
        a_xyz[0] = a_x[0];
        a_xyz[1] = a_x[1];
        a_xyz[2] = a_x[2];
        a_xyz[3] = y0;
        a_xyz[4] = y1;
        a_xyz[5] = z;

        // range space vector
        size_t m = 1;
        vector< AD<double> >  a_L(m);
        a_L[0] = Lagragian(a_xyz);

        // create K: x -> L and stop tape recording.
        // Use default ADFun construction for example purposes.
        CppAD::ADFun<double> K;
        K.Dependent(a_x, a_L);

        // Operation sequence corresponding to K depends on
        // value of y0, y1, and z. Must redo calculations above when
        // y0, y1, or z changes.

        // declare independent variable vector and Hessian
        vector<double> x(n);
        vector<double> H( n * n );

        // point at which we are computing the Hessian
        // (must redo calculations below each time x changes)
        x[0] = x0;
        x[1] = x1;
        x[2] = x2;
        H = K.Hessian(x, 0);

        // check this Hessian calculation
        return CheckHessian(H, x0, x1, x2, y0, y1, z);
    }
    bool Usefg()
    {   using CppAD::AD;
        using CppAD::vector;

        // parameters defining problem
        double x0(.5), x1(1e3), x2(1), y0(2.), y1(3.), z(4.);

        // domain space vector
        size_t n = 3;
        vector< AD<double> >  a_x(n);
        a_x[0] = x0;
        a_x[1] = x1;
        a_x[2] = x2;

        // declare a_x as independent variable vector and start recording
        CppAD::Independent(a_x);

        // range space vector
        size_t m = 3;
        vector< AD<double> >  a_fg(m);
        a_fg = fg(a_x);

        // create K: x -> fg and stop tape recording
        CppAD::ADFun<double> K;
        K.Dependent(a_x, a_fg);

        // Operation sequence corresponding to K does not depend on
        // value of x0, x1, x2, y0, y1, or z.

        // forward and reverse mode arguments and results
        vector<double> x(n);
        vector<double> H( n * n );
        vector<double>  dx(n);
        vector<double>   w(m);
        vector<double>  dw(2*n);

        // compute Hessian at this value of x
        // (must redo calculations below each time x changes)
        x[0] = x0;
        x[1] = x1;
        x[2] = x2;
        K.Forward(0, x);

        // set weights to Lagrange multiplier values
        // (must redo calculations below each time y0, y1, or z changes)
        w[0] = z;
        w[1] = y0;
        w[2] = y1;

        // initialize dx as zero
        size_t i, j;
        for(i = 0; i < n; i++)
            dx[i] = 0.;
        // loop over components of x
        for(i = 0; i < n; i++)
        {   dx[i] = 1.;             // dx is i-th elementary vector
            K.Forward(1, dx);       // partial w.r.t dx
            dw = K.Reverse(2, w);   // deritavtive of partial
            for(j = 0; j < n; j++)
                H[ i * n + j ] = dw[ j * 2 + 1 ];
            dx[i] = 0.;             // dx is zero vector
        }

        // check this Hessian calculation
        return CheckHessian(H, x0, x1, x2, y0, y1, z);
    }
}

bool HesLagrangian(void)
{   bool ok = true;

    // UseL is simpler, but must retape every time that y of z changes
    ok     &= UseL();

    // Usefg does not need to retape unless operation sequence changes
    ok     &= Usefg();
    return ok;
}

// END C++
