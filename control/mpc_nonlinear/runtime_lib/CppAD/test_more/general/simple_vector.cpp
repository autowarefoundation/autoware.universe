/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */


/*
Old simple vector example now used just for validation testing
*/
// BEGIN C++

// need this to define size_t
# include <cstddef>

// # include <vector>
// # define MySimpleVector std::vector

// # include <valarray>
// # define MySimpleVector std::valarray

// Assuming CppAD (http://www.seanet.com/~bradbell/cppad) is installed
# include <cppad/utility/vector.hpp>
# define MySimpleVector CppAD::vector

// Assuming Boost (http://www.boost.org) is installed
// # include <boost/numeric/ublas/vector.hpp>
// # define MySimpleVector boost::numeric::ublas::vector

// A special type of element that counts assignment operations
class MyInt {
public:
    // constructors
    MyInt(void)       : count(0), value(0)
    { }
    MyInt(int _value) : count(0), value(_value)
    { }
    // assignment operator
    MyInt& operator=(const MyInt &x)
    {   value = x.value;
        count++;
        return *this;
    }
    // equality operator
    bool operator==(const MyInt &x) const
    {   return value == x.value; }

    int Value(void) const
    {   return value; }
    size_t Count(void) const
    {   return count; }
private:
    size_t count;  // count number of assignments
    int    value;  // value of this object
};

// Test of a Simple Vector template class
bool SimpleVector(void)
{   bool ok = true;
    typedef MySimpleVector<MyInt> vector;    // class we are testing
    typedef vector::value_type myInt;        // type of elements

    vector x;                // default constructor
    ok &= (x.size() == 0);

    x.resize(2);             // resize and set element assignment
    ok &= (x.size() == 2);
    x[0] = myInt(0);
    x[1] = myInt(1);

    vector y(2);             // sizing constructor
    ok &= (y.size() == 2);

    const vector z(x);       // copy constructor and const element access
    ok &= (z.size() == 2);
    ok &= ( (z[0] == myInt(0)) && (z[1] == myInt(1)) );

    // check that vector assignment
    x[0] = 2;                      // modify so that assignment changes x
    size_t x0count = x[0].Count(); // store initial counts
    size_t y1count = y[1].Count();
    size_t z0count = z[0].Count();
    x = y = z;                     // vector assignment

    // check resulting values
    ok &= ( (x[0] == myInt(0)) && (x[1] == myInt(1)) );
    ok &= ( (y[0] == myInt(0)) && (y[1] == myInt(1)) );
    ok &= ( (z[0] == myInt(0)) && (z[1] == myInt(1)) );

    // check that MyInt assignment was called (not raw memory copy)
    ok &= (x[0].Count() == x0count + 1);
    ok &= (y[1].Count() == y1count + 1);
    ok &= (z[0].Count() == z0count);

    return ok;
}

// END C++
