#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ ! -e "bin/check_include_file.sh" ]
then
    msg="must be executed from its parent directory"
    echo "bin/check_include_file.sh: $msg"
    exit 1
fi
# -----------------------------------------------------------------------------
#
echo "Checking difference between C++ include directives and file names."
echo "-------------------------------------------------------------------"
if [ -e check_include_file.1.$$ ]
then
    echo "bin/check_include_file.sh: unexpected check_include_file.1.$$"
    exit 1
fi
list=`git ls-files | sed -n \
    -e '/\.cpp$/p' \
    -e '/\.hpp$/p'`
for file in $list
do
    sed -n $file \
        -e '/^# *include *<cppad\/cg\//d' \
        -e '/^# *include *<cppad\//p' \
        >> check_include_file.1.$$
done
#
cat check_include_file.1.$$ | \
    sed -e 's%[^<]*<%%'  -e 's%>.*$%%' | \
    sort -u > check_include_file.2.$$
#
# The following files should never be included:
#   cppad/local/prototype_op.hpp
#   cppad/local/optimize/define_prototype.hpp
# All other files should.
#
# The files cppad/configure.hpp and cppad/local/is_pod.hpp
# are not in git repository (they are built during configuration)
git ls-files | sed -n -e '/include\/cppad\/.*\.hpp$/p' | \
    sed \
        -e '1,1s|^|include/cppad/configure.hpp\n|' \
        -e '1,1s|^|include/cppad/local/is_pod.hpp\n|' \
        -e '/include\/cppad\/local\/op\/prototype_op.hpp/d' \
        -e '/include\/cppad\/example\/eigen_plugin.hpp/d' | \
    sed -e 's|^include/||' | \
    sort -u > check_include_file.3.$$
#
different='no'
if ! diff check_include_file.2.$$ check_include_file.3.$$ > /dev/null
then
    found='no'
    different='yes'
    for file in `cat check_include_file.2.$$`
    do
        if ! grep "$file" check_include_file.3.$$ > /dev/null
        then
            found='yes'
            echo "The file include/$file is unknown to git."
            echo 'Perhaps it needs to be added with the command'
            echo "    git add include/$file"
        fi
    done
    for file in `cat check_include_file.3.$$`
    do
        if ! grep "$file" check_include_file.2.$$ > /dev/null
        then
            found='yes'
            echo "The included $file is no longer included."
            echo 'Perhaps it needs to be git deleted ?'
        fi
    done
    if [ "$found" == 'no' ]
    then
        echo 'bin/check_include_file.sh: Cannot find reason for difference'
        echo 'Improve this script.'
        exit 1
    fi
fi
for index in 1 2 3
do
    rm check_include_file.$index.$$
done
#
echo "-------------------------------------------------------------------"
if [ $different = "yes" ]
then
    echo "Error: nothing should be between the two dashed lines above"
    exit 1
else
    echo "Ok: nothing is between the two dashed lines above"
    exit 0
fi
