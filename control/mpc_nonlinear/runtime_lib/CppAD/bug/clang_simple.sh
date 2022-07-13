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
name='clang_simple'
if [ "$0" != "bug/$name.sh" ]
then
    echo "usage: bug/$name.sh"
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
cat << EOF > $name.cpp
# include <cppad/cppad.hpp>
int main(void)
{   bool ok = true;
    using std::cout;
    using CppAD::AD;
    //
    std::cout << "Test for issue 31\n";
    CppAD::CheckSimpleVector<double, CppAD::vector<double> >();
    //
    if( ok )
        return 0;
    return 1;
}
EOF
cxx_flags='-g -O0'
eigen_dir="$HOME/prefix/eigen/include"
echo "clang++ -I../../include -isystem $eigen_dir $cxx_flags $name.cpp -o $name"
clang++ -I../../include -isystem $eigen_dir $cxx_flags $name.cpp -o $name
#
if ! ./$name
then
    echo
    echo "build/bug/$name: Error"
    exit 1
fi
echo
# ------------------------------------------------------------------------------
echo "./$name.sh: OK"
exit 0
