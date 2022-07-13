#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
cat << EOF
This is not a bug but rather a test of installing with
    cppad_prefix=$HOME/prefix
EOF
cat << EOF > bug.$$
# include <cppad/cppad.hpp>
int main(void)
{   bool ok = true;
    using std::cout;
    using CppAD::AD;
    //
    CPPAD_TESTVECTOR( AD<double> ) ax(1), ay(1);
    ax[0] = 1.0;
    CppAD::Independent(ax);
    ay[0] = sin( ax[0] );
    CppAD::ADFun<double> f(ax, ay);
    //
    std::vector< std::set<size_t> > p(1);
    p[0].insert(0);
    CppAD::vector< size_t > row(1), col(1);
    row[0] = 0;
    col[0] = 0;
    CppAD::sparse_jacobian_work work;
    work.color_method = "colpack";
    CPPAD_TESTVECTOR(double) x(1), jac(1);
    x[0] = 2.0;
    f.SparseJacobianForward(x, p, row, col, jac, work);
    //
    ok  &= jac[0] == std::cos( x[0] );
    //
    if( ok )
        return 0;
    return 1;
}
EOF
# -----------------------------------------------------------------------------
if [ ! -e build ]
then
    mkdir build
fi
cd build
echo "$0"
name=`echo $0 | sed -e 's|.*/||' -e 's|\..*||'`
mv ../bug.$$ $name.cpp
cmd="g++ -I $HOME/prefix/cppad/include --std=c++11 -g $name.cpp -o $name"
cmd="$cmd -L $HOME/prefix/cppad/lib64 -lcppad_lib"
cmd="$cmd -L $HOME/prefix/colpack/lib64 -lColPack"
echo "$cmd"
eval $cmd
#
echo "./$name"
if ! ./$name
then
    echo
    echo "$name.sh: Error"
    exit 1
fi
echo
echo "$name.sh: OK"
exit 0
