/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// CppAD::vector tests
# include <cppad/utility/vector.hpp>
# include <vector>
# include <algorithm>

namespace { // BEGIN_EMPTY_NAMESPACE

bool test_find(void)
{   // find requires an input iterator
    bool ok = true;
    typedef CppAD::vector<char>::const_iterator const_iterator;
    //
    size_t n = 10;
    CppAD::vector<char> vec(n);
    for(size_t i = 0; i < n; ++i)
        vec[i] = static_cast<char>( '0' + i);
    const_iterator itr;
    //
    itr = std::find(vec.begin(), vec.end(), '3');
    ok &= itr != vec.end();
    ok &= *itr == '3';
    //
    itr = std::find(vec.begin(), vec.end(), 'a');
    ok &= itr == vec.end();
    //
    return ok;
}

bool test_copy(void)
{   // copy requires a forward iterator
    bool ok = true;
    //
    size_t n = 10;
    CppAD::vector<char> vec(n), des(n);
    for(size_t i = 0; i < n; ++i)
        vec[i] = static_cast<char>( '0' + i);
    const CppAD::vector<char> src(vec);
    //
    // src.begin(), src.end() are const_iterator
    // des.begin() is an iterator
    std::copy(src.begin(), src.end(), des.begin());
    //
    for(size_t i = 0; i < n; ++i)
        ok &= src[i] == des[i];
    //
    return ok;
}

bool test_reverse(void)
{   // copy requires a bidirectional iterator
    bool ok = true;
    //
    size_t n = 10;
    CppAD::vector<char> src(n), des(n);
    for(size_t i = 0; i < n; ++i)
        src[i] = static_cast<char>( '0' + i);
    //
    std::reverse(src.begin(), src.end());
    //
    for(size_t i = 0; i < n; ++i)
        ok &= src[i] == static_cast<char>( '0' + (n - i - 1) );
    //
    return ok;
}

# ifndef  _MSC_VER
bool test_sort(void)
{   // copy requires a random access iterator
    bool ok = true;
    //
    size_t n = 10;
    CppAD::vector<char> vec(n);
    for(size_t i = 0; i < n; ++i)
        vec[i] = static_cast<char>( '0' + (n - i - 1) );
    //
    std::sort(vec.begin(), vec.end());
    //
    for(size_t i = 0; i < n; ++i)
        ok &= vec[i] == static_cast<char>( '0' + i );
    //
    return ok;
}
# endif


} // END_EMPTY_NAMESPACE

bool cppad_vector(void)
{   bool ok = true;
    //
    ok &= test_find();
    ok &= test_copy();
    ok &= test_reverse();
// 2DO: Determine out why this test fails with Visual Studio 2019
# ifndef  _MSC_VER
    ok &= test_sort();
# endif
    //
    return ok;
}
