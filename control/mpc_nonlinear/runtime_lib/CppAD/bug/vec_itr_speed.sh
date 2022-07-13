#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
name=`echo $0 | sed -e 's|^bug/||' -e 's|\.sh$||'`
error='none'
if [ "$0" != "bug/$name.sh" ]
then
    echo "program name is not bug/$name.sh"
    ok='no'
fi
compiler="$1"
if [ "$compiler" != 'g++' ] && [ "$compiler" != 'clang++' ]
then
    if [ "$compiler" != '' ]
    then
        echo 'complier is not g++ or clang++'
    fi
    ok='no'
fi
debug="$2"
if [ "$debug" != 'yes' ] && [ "$debug" != 'no' ]
then
    if [ "$debug" != '' ]
    then
        echo 'debug is not yes or no'
    fi
    ok='no'
fi
opt_level="$3"
if [[ "$opt_level" =~ "[^0-3]" ]]
then
    if [ "$opt_level" != '' ]
    then
        echo 'opt_leve is not 0, 1, 2, or 3'
    fi
    ok='no'
fi
if [ "$ok" == 'no' ]
then
    echo
    echo "usage: bug/$name.sh compiler debug opt_level"
    echo 'complier is:  g++ or clang++'
    echo 'debug is:     yes or no'
    echo 'opt_level is: 0, 1, 2, or 3.'
    exit 1
fi
# -----------------------------------------------------------------------------
if [ -e build/bug ]
then
    rm -r build/bug
fi
mkdir -p build/bug
cd build/bug
# cmake ../..
# -----------------------------------------------------------------------------
cat << EOF
This is speed test (not a bug report) comparing the speed using CppAD vector
iterators and raw pointer with the algorithms std::sort and std::reverse.
EOF
cat << EOF > $name.cpp
# include <cppad/utility/vector.hpp>
# include <cppad/utility/time_test.hpp>
namespace {
    // declared here so setup does not include allocation
    CppAD::vector<size_t> vec;
    // ----------------------------------------------------------------------
    // sort test functions
    void sort_itr(size_t size, size_t repeat)
    {   // size and vec.size() are equal
        size_t* data = vec.data();
        while( repeat-- )
        {   // sort a vector that is not in order
            for(size_t i = 0; i < size; ++i)
                data[i] = (size - i) % 21;
            std::sort(vec.begin(), vec.end());
        }
    }
    void sort_ptr(size_t size, size_t repeat)
    {   // size and vec.size() are equal
        size_t* data = vec.data();
        while( repeat-- )
        {   // sort same vector as in sort_itr
            for(size_t i = 0; i < size; ++i)
                data[i] = (size - i) % 21;
            std::sort(vec.data(), vec.data() + vec.size());
        }
    }
    // ----------------------------------------------------------------------
    // reverse test functions
    void reverse_itr(size_t size, size_t repeat)
    {   // size and vec.size() are equal
        size_t* data = vec.data();
        while( repeat-- )
        {   // reverse a vector that is not in order
            for(size_t i = 0; i < size; ++i)
                data[i] = i;
            std::reverse(vec.begin(), vec.end());
        }
    }
    void reverse_ptr(size_t size, size_t repeat)
    {   // size and vec.size() are equal
        size_t* data = vec.data();
        while( repeat-- )
        {   // reverse same vector as in reverse_itr
            for(size_t i = 0; i < size; ++i)
                data[i] = i;
            std::reverse(vec.data(), vec.data() + vec.size());
        }
    }
}
int main(void)
{   bool ok = true;
    using CppAD::time_test;
    using std::cout;
    //
    size_t test_size = 100000; // size of vector in test
    double time_min  = 1.0;    // minimum time in seconds for each test
    vec.resize(test_size);     // allocate memory outsize of test
    size_t repeat;             // output by time_test function
    // -----------------------------------------------------------------------
    // sort tests
    //
    // iterator
    double sort_itr_sec  = time_test(sort_itr, time_min, test_size, repeat);
    for(size_t i = 1; i < test_size; ++i)
        ok &= vec[i-1] <= vec[i];
    cout << "sort_itr_sec=" << sort_itr_sec << ", repeat=" << repeat << "\n";
    //
    // pointer
    double sort_ptr_sec  = time_test(sort_ptr, time_min, test_size, repeat);
    for(size_t i = 1; i < test_size; ++i)
        ok &= vec[i-1] <= vec[i];
    cout << "sort_ptr_sec=" << sort_ptr_sec << ", repeat=" << repeat << "\n";
    // -----------------------------------------------------------------------
    // reverse tests
    //
    // iterator
    double rev_itr_sec  = time_test(reverse_itr, time_min, test_size, repeat);
    for(size_t i = 1; i < test_size; ++i)
        ok &= vec[i] == test_size - 1 - i;
    cout << "rev_itr_sec=" << rev_itr_sec << ", repeat=" << repeat << "\n";
    //
    // pointer
    double rev_ptr_sec  = time_test(reverse_ptr, time_min, test_size, repeat);
    for(size_t i = 1; i < test_size; ++i)
        ok &= vec[i] == test_size - 1 - i;
    cout << "rev_ptr_sec=" << rev_ptr_sec << ", repeat=" << repeat;
    // -----------------------------------------------------------------------
    if( ok )
        return 0;
    return 1;
}
EOF
cxx_flags="-Wall -std=c++11 -Wshadow -Wconversion -O$opt_level"
if [ "$debug" == 'no' ]
then
    cxx_flags="$cxx_flags -DNDEBUG"
fi
echo "$compiler -I../../include $cxx_flags $name.cpp -o $name"
$compiler -I../../include $cxx_flags $name.cpp -o $name
#
echo "build/bug/$name"
if ! ./$name
then
    echo
    echo "build/bug/$name: Error"
    exit 1
fi
echo
# -----------------------------------------------------------------------------
echo "bug/$name.sh: OK"
exit 0
