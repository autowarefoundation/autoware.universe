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
gigabytes='1.0'   # memory limit in gigabytes
# -----------------------------------------------------------------------------
kilobytes=`echo "($gigabytes * 10^9) / 1024" | bc`
ulimit -Sv $kilobytes
cat << EOF
This is testing the idea that
    vector< std::set<size_t> > sparsity;
is inefficient compared to
    vector<size_t> row, col;
The results on one system for this script are:
    ./sparsity set 1e7 elapsed_seconds = 3.62206
    ./sparsity set 2e7 std::bad_alloc
    ./sparsity vec 2e7 elapsed_seconds = 1.34243
    ./sparsity vec 4e7 std::bad_alloc
EOF
cat << EOF > bug.$$
# include <cppad/cppad.hpp>

int main(int argc, char *argv[])
{   using CppAD::elapsed_seconds;
    if( argc != 3 )
    {   std::cerr << "usage: $0 (set|vec) n" << std::endl;
        return 1;
    }
    bool set  = std::strcmp(argv[1], "set") == 0;
    bool vec  = std::strcmp(argv[1], "vec") == 0;
    bool ok   = vec || set;
    if( ! ok )
    {   std::cerr << "usage: $0 (set|vec) n" << std::endl;
        return 1;
    }
    size_t n = size_t( std::atof( argv[2] ) );
    const char* label;
    elapsed_seconds();
    try
    {
        if( set )
        {   std::vector< std::set<size_t> > my_set(n);
            for(size_t i = 0; i < n; i++)
                my_set[i].insert(i);
        }
        else
        {   std::vector<size_t> row;
            std::vector<size_t> col;
            for(size_t i = 0; i < n; i++)
            {   row.push_back(i);
                col.push_back(i);
            }
        }
        for(int i = 0; i < argc; i++)
            std::cout << argv[i] << " ";
        std::cout << "elapsed_seconds = " << elapsed_seconds() << std::endl;
    }
    catch( std::bad_alloc& ba )
    {
        for(int i = 0; i < argc; i++)
            std::cout << argv[i] << " ";
        std::cout << ba.what() << std::endl;
    }
    return 0;
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
echo "g++ -I../.. -DNDEBUG --std=c++11 $name.cpp -o $name"
g++ -I../.. -DNDEBUG --std=c++11 $name.cpp -o $name
#
./$name set 1e7
./$name set 2e7
./$name vec 2e7
./$name vec 4e7
