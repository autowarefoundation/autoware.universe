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
$begin Rombergmul.cpp$$
$spell
    Romberg
$$

$section One Dimensional Romberg Integration: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/utility/romberg_mul.hpp>
# include <cppad/utility/vector.hpp>
# include <cppad/utility/near_equal.hpp>


namespace {

    class TestFun {
    private:
        const CppAD::vector<size_t> deg;
    public:
        // constructor
        TestFun(const CppAD::vector<size_t> deg_)
        : deg(deg_)
        { }

        // function F(x) = x[0]^deg[0] * x[1]^deg[1]
        double operator () (const CppAD::vector<double> &x)
        {   size_t i;
            double   f = 1;
            for(i = 0; i < deg[0]; i++)
                f *= x[0];
            for(i = 0; i < deg[1]; i++)
                f *= x[1];
            return f;
        }
    };

}

bool RombergMul(void)
{   bool ok = true;
    size_t i;
    size_t k;

    CppAD::vector<size_t> deg(2);
    deg[0] = 5;
    deg[1] = 3;
    TestFun F(deg);

    CppAD::RombergMul<
        TestFun              ,
        CppAD::vector<size_t>,
        CppAD::vector<double>,
        2                    > RombergMulTest;

    // arugments to RombergMul
    CppAD::vector<double> a(2);
    CppAD::vector<double> b(2);
    CppAD::vector<size_t> n(2);
    CppAD::vector<size_t> p(2);
    for(i = 0; i < 2; i++)
    {   a[i] = 0.;
        b[i] = 1.;
    }
    n[0] = 4;
    n[1] = 3;
    double r, e;

    // int_a1^b1 dx1 int_a0^b0 F(x0,x1) dx0
    //    = [ b0^(deg[0]+1) - a0^(deg[0]+1) ] / (deg[0]+1)
    //    * [ b1^(deg[1]+1) - a1^(deg[1]+1) ] / (deg[1]+1)
    double bpow = 1.;
    double apow = 1.;
    for(i = 0; i <= deg[0]; i++)
    {   bpow *= b[0];
        apow *= a[0];
    }
    double check = (bpow - apow) / double(deg[0]+1);
    bpow = 1.;
    apow = 1.;
    for(i = 0; i <= deg[1]; i++)
    {   bpow *= b[1];
        apow *= a[1];
    }
    check *= (bpow - apow) / double(deg[1]+1);

    double step = (b[1] - a[1]) / exp(log(2.)*double(n[1]-1));
    double spow = 1;
    for(k = 0; k <= n[1]; k++)
    {   spow = spow * step * step;
        double bnd = 3 * double(deg[1] + 1) * spow;

        for(i = 0; i < 2; i++)
            p[i] = k;
        r    = RombergMulTest(F, a, b, n, p, e);

        ok  &= e < bnd;
        ok  &= CppAD::NearEqual(check, r, 0., e);

    }

    return ok;
}

// END C++
