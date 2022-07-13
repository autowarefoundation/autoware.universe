#! /bin/bash -eu
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
if [ "$0" != "bin/valgrind.sh" ]
then
    echo "bin/valgrind.sh: must be executed from its parent directory"
    exit 1
fi
if [ "$#" != '0' ]
then
    echo 'usage: bin/valgrind.sh.sh'
    exit 1
fi
# -----------------------------------------------------------------------------
list=`find build -perm -700 -type f | \
    sed -e '/^build\/cppad-*/d'  -e '/\/CMakeFiles\//d'`
for program in $list
do
    arguments=''
    if echo "$program" | grep '\/speed\/' > /dev/null
    then
        arguments='correct none'
    fi
    echo "valgrind $program $arguments 2> valgrind.log"
    valgrind $program $arguments 2> valgrind.log
    if ! grep '^==[0-9]*== ERROR SUMMARY: 0' valgrind.log
    then
        grep 'ERROR SUMMARY' valgrind.log
        exit 1
    fi
    if ! grep '^==[0-9]*== *definitely lost: 0' valgrind.log
    then
        if grep '^==[0-9]*== *definitely lost:' valgrind.log
        then
            exit 1
        fi
    fi
    if ! grep '^==[0-9]*== *indirectly lost: 0' valgrind.log
    then
        if grep '^==[0-9]*== *indirectly lost:' valgrind.log
        then
            exit 1
        fi
    fi
    if ! grep '^==[0-9]*== *possibly lost: 0' valgrind.log
    then
        if grep '^==[0-9]*== *possibly lost:' valgrind.log
        then
            exit 1
        fi
    fi
    if ! grep '^==[0-9]*== *suppressed: 0' valgrind.log
    then
        if grep '^==[0-9]*== *suppressed:' valgrind.log
        then
            exit 1
        fi
    fi
done
# -----------------------------------------------------------------------------
echo 'bin/valgrind.sh: OK'
exit 0
