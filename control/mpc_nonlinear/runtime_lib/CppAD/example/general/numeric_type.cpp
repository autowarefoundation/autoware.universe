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
$begin numeric_type.cpp$$

$section The NumericType: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace { // Empty namespace

    // -------------------------------------------------------------------
    class MyType {
    private:
        double d;
    public:
        // constructor from void
        MyType(void) : d(0.)
        { }
        // constructor from an int
        MyType(int d_) : d(d_)
        { }
        // copy constructor
        MyType(const MyType &x)
        {   d = x.d; }
        // assignment operator
        void operator = (const MyType &x)
        {   d = x.d; }
        // member function that converts to double
        double Double(void) const
        {   return d; }
        // unary plus
        MyType operator + (void) const
        {   MyType x;
            x.d =  d;
            return x;
        }
        // unary plus
        MyType operator - (void) const
        {   MyType x;
            x.d = - d;
            return x;
        }
        // binary addition
        MyType operator + (const MyType &x) const
        {   MyType y;
            y.d = d + x.d ;
            return y;
        }
        // binary subtraction
        MyType operator - (const MyType &x) const
        {   MyType y;
            y.d = d - x.d ;
            return y;
        }
        // binary multiplication
        MyType operator * (const MyType &x) const
        {   MyType y;
            y.d = d * x.d ;
            return y;
        }
        // binary division
        MyType operator / (const MyType &x) const
        {   MyType y;
            y.d = d / x.d ;
            return y;
        }
        // compound assignment addition
        void operator += (const MyType &x)
        {   d += x.d; }
        // compound assignment subtraction
        void operator -= (const MyType &x)
        {   d -= x.d; }
        // compound assignment multiplication
        void operator *= (const MyType &x)
        {   d *= x.d; }
        // compound assignment division
        void operator /= (const MyType &x)
        {   d /= x.d; }
    };
}
bool NumericType(void)
{   bool ok  = true;
    using CppAD::AD;
    using CppAD::CheckNumericType;

    CheckNumericType<MyType>            ();

    CheckNumericType<int>               ();
    CheckNumericType<double>            ();
    CheckNumericType< AD<double> >      ();
    CheckNumericType< AD< AD<double> > >();

    return ok;
}

// END C++
