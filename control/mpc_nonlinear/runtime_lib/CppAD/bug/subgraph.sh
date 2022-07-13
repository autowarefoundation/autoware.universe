#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell
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
if [ "$0" != "bug/$name.sh" ]
then
    echo 'usage: bug/alloc_global.sh'
    exit 1
fi
# -----------------------------------------------------------------------------
if [ -e build/bug ]
then
    rm -r build/bug
fi
mkdir -p build/bug
cd build/bug
cmake ../..
# -----------------------------------------------------------------------------
cat << EOF
Description: Test for bug in subgraph_jac_rev(x, subset).
EOF
cat << EOF > $name.cpp
# include <cppad/cppad.hpp>
int main(void)
{   bool ok = true;
    using std::cout;
    using CppAD::AD;
    using CppAD::vector;
    typedef vector<double> d_vector;
    typedef vector<size_t> s_vector;
    //
    size_t n = 4;
    d_vector x(n);
    vector< AD<double> > ax(n), ay(n);
    for(size_t j = 0; j < n; ++j)
        ax[j] = x[j] = double(j);
    CppAD::Independent(ax);
    for(size_t i = 0; i < n; ++i)
    {   ay[i] = 0.0;
        for(size_t j = 0; j < n; ++j)
            ay[i] += double(i + j + 1) * ax[j];
    }
    CppAD::ADFun<double> f(ax, ay);
    //
    size_t nnz = (n * (n + 1)) / 2;
    CppAD::sparse_rc<s_vector> upper_triangle(n, n, nnz);
    size_t k = 0;
    for(size_t i = 0; i < n; ++i)
    {   for(size_t j = i; j < n; ++j)
            upper_triangle.set(k++, i, j);
    }
    ok &= k == nnz;
    CppAD::sparse_rcv<s_vector, d_vector> subset( upper_triangle );
    //
    f.subgraph_jac_rev(x, subset);
    const d_vector& val = subset.val();
    k = 0;
    for(size_t i = 0; i < n; ++i)
    {   for(size_t j = i; j < n; ++j)
            ok &= val[k++] == double(i + j + 1);
    }
    ok &= k == nnz;
    //
    if( ok )
        return 0;
    return 1;
}
EOF
cxx_flags='-Wall -pedantic-errors -std=c++11 -Wshadow -Wconversion -g -O0'
echo "g++ -I../../include $cxx_flags $name.cpp -o $name"
g++ -I../../include $cxx_flags $name.cpp -o $name
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
