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
if [ ! -e "bin/check_example.sh" ]
then
    echo "bin/check_example.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
echo "Checking that all examples are in omh/example_list.omh"
echo "-------------------------------------------------------"
file_list=`git ls-files | sed -n \
    -e '/^example\/deprecated\//d' \
    -e '/^example\//p'`
#
sed < omh/example_list.omh > check_example.$$ \
    -n -e '/\$begin ListAllExamples\$\$/,/\$end/p'
#
# Make sure all example names are of the form *.cpp or *.hpp
check=`sed -n -e '/$rref [0-9a-zA-Z_]*\.[hc]pp/d' \
    -e '/$rref/p' check_example.$$`
if [ "$check" != '' ]
then
    echo $check
    echo 'Not all examples are for *.hpp or *.cpp files'
    exit 1
fi
ok="yes"
for file in $file_list
do
    # only files in example directory with $begin *.cpp or *.hpp
    # e.g., example/multi_thread/harmonic.omh has $begin harmonic.cpp$$ in it
    name=`grep '$begin *[0-9a-zA-Z_]*\.[hc]pp' $file |
        sed -e 's/.*$begin *//' -e 's/ *$$.*//'`
    if [ "$name" != "" ]
    then
        if ! grep "$name" check_example.$$ > /dev/null
        then
            echo "$name is missing from omh/example_list.omh"
            ok="no"
        fi
    fi
done
rm check_example.$$
echo "-------------------------------------------------------"
if [ "$ok" != "yes" ]
then
    echo "Error: nothing should be between the two dashed lines above"
    exit 1
fi
#
# fix sort order; see
# unix.stackexchange.com/questions/87745/what-does-lc-all-c-do/87763#87763
export LC_ALL='C'
#
dir_list='
    example/graph
    example/json
    example/general
'
for dir in $dir_list
do
    echo "Checking $dir file versus example names"
    name=`echo $dir | sed -e 's|.*/||'`
    #
    ls $dir/*.cpp | sed  \
        -e "s|$dir/||" -e "/^$name\\.cpp/d" -e 's|\.cpp||' > check_example.1.$$
    if [ "$name" == 'general' ]
    then
        list=`ls $dir/*.hpp \
            | sed -e "s|$dir/||" -e 's|\.hpp$||' -e '/^general$/d'`
        for file in $list
        do
            sed -i check_example.1.$$ -e "/^$file\$/d"
        done
    fi
    sed -n -e '/^extern bool [a-z0-9A-Z_]*(void);/p' $dir/$name.cpp \
        | sed -e 's/extern bool \([a-z0-9A-Z_]*\)(void);/\1/' \
        | sed -e 's/\([a-z]\)\([A-Z]\)/\1_\2/g' \
        | tr '[A-Z]' '[a-z]' \
        | sort > check_example.2.$$
    if ! diff check_example.1.$$ check_example.2.$$
    then
        rm check_example.1.$$ check_example.2.$$
        echo "$dir: file and function names do not agree"
        echo 'see output above.'
        exit 1
    fi
done
rm check_example.1.$$ check_example.2.$$
#
echo 'bin/check_example.sh: OK'
exit 0
