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
program="bin/speed_new.sh"
if [ "$0" != "$program" ]
then
    echo "$program: must be executed from its parent directory"
    exit 1
fi
if [ "$1" == '' ]
then
cat << EOF
usage:
$program test_name [option_1] [option_2] ...

possible test are:
all, det_lu, det_minor, mat_mul ode, poly, sparse_hessian, sparse_jacobian

possible options are:
atomic, boolsparsity, colpack, memory, onetape, optimize, revsparsity
EOF
    exit 1
fi
if [ ! -d new ]
then
    echo "$program: the directory ./new does not exist."
    echo 'It should contain the new source code.'
    exit 1
fi
test_name="$1"
shift
option_list="$test_name"
for option in $*
do
    option_list="${option_list}_$option"
done
if [ "$test_name" == 'all' ]
then
    test_name='speed'
fi
# ----------------------------------------------------------------------------
build_dir='build/speed/cppad'
if [ ! -e $build_dir ]
then
    echo_eval mkdir -p $build_dir
fi
# ----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# ----------------------------------------------------------------------------
echo "bin/run_cmake.sh --debug_none > /dev/null"
bin/run_cmake.sh --debug_none > /dev/null
#
for name in cur new
do
    if [ "$name" == 'cur' ]
    then
        # revert cppad source code to the current version
        echo_eval git reset --hard --quiet
    else
        echo_eval git_new.sh from
    fi
    out_file="$name.$option_list.out"
    if [ -e "$build_dir/$out_file" ]
    then
        echo "Using existing $build_dir/$out_file"
    else
        # change into cppad speed directory
        echo_eval cd $build_dir
        #
        # compile the speed test
        echo "make check_speed_cppad > $build_dir/$name.log"
        make check_speed_cppad > $name.log
        #
        # run speed test for the current version
        echo "./speed_cppad $test_name 123 $* > $build_dir/$out_file"
        ./speed_cppad $test_name 123 $* > $out_file
        #
        echo_eval cd ../../..
    fi
done
# compare the results
echo "    one=cur, two=new"
bin/speed_diff.sh \
    $build_dir/cur.$option_list.out $build_dir/new.$option_list.out
# ----------------------------------------------------------------------------
echo "$0: OK"
exit 0
