#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ $0 != "bin/travis.sh" ]
then
    echo 'bin/travis.sh: must be executed from its parent directory'
    exit 1
fi
if [ "$1" != 'make' ] && [ "$1" != 'test_one' ]
then
    echo 'usage: bin/travis.sh (make|test_one) target1 target2 ...'
    echo 'target: if make specified, is one of the available make commands'
    echo          if test_one, specified, is the path to a test file.
    exit 1
fi
cmd="$1"
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
if [ -e 'build' ]
then
    echo_eval rm -r build
fi
echo_eval mkdir build
echo_eval cd build
echo_eval cmake \
    -D cppad_prefix=$(pwd)/prefix \
    -D cppad_cxx_flags='-std=c++11' \
    ..
# -----------------------------------------------------------------------------
if [ "$cmd" == 'make' ]
then
    shift
    while [ "$1" != '' ]
    do
        echo_eval make "$1"
        shift
    done
else
    echo_eval cd ..
    shift
    while [ "$1" != '' ]
    do
        echo_eval bin/test_one.sh "$1"
        shift
    done
fi
# -----------------------------------------------------------------------------
echo 'bin/travis.sh: OK'
exit 0
