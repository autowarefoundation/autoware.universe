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
if [ "$0" != "bin/check_deprecated.sh" ]
then
    echo "bin/check_deprecated.sh: must be executed from its parent directory"
    exit 1
fi
if [ "$#" != '0' ]
then
    echo 'usage: bin/check_deprecated.sh.sh'
    exit 1
fi
# -----------------------------------------------------------------------------
file_list=`git ls-files example speed | sed -e '/example\/deprecated\//d'`
# -----------------------------------------------------------------------------
# deprecated functions with not arugments
list_just_name='
    CPPAD_TRACK_
    CPPAD_TEST_VECTOR
    CppADCreateUnaryBool
    CppADCreateDiscrete
    zdouble
    colpack.star
'
list_namespace='
    omp_alloc
    cppad_ipopt
'
template_name='
    epsilon
'
list_no_argument='
    Order
    Memory
    Size
    taylor_size
    use_VecAD
    size_taylor
    capacity_taylor
    CompareChange
    memory_leak
'
list_one_argument='
    nan
    Dependent
    omp_max_thread
    memory_leak
'
list_two_argument='
'
list_three_argument='
'
for file in $file_list
do
    for name in $list_just_name
    do
        if grep "$name" $file > /dev/null
        then
            echo "$name is deprecated and appreas in $file"
            exit 1
        fi
    done
    for name in $list_namespace
    do
        if grep "[^a-zA-Z_]$name::" $file > /dev/null
        then
            echo "$name:: is deprecated and appreas in $file"
            exit 1
        fi
    done
    for name in $list_namespace
    do
        if grep "using *$name[^a-zA-Z_]" $file > /dev/null
        then
            echo "using $name is deprecated and appreas in $file"
            exit 1
        fi
    done
    for name in $template_name
    do
        if grep "[^a-zA-Z_]$name *< *[a-zA-Z_][a-zA-Z_]* *>" $file > /dev/null
        then
            echo "$name<arg> is deprecated and appreas in $file"
            exit 1
        fi
    done
    for fun in $list_no_argument
    do
        if grep "[^a-zA-Z_]$fun *( *)" $file > /dev/null
        then
            echo "$fun() is deprecated and appreas in $file"
            exit 1
        fi
    done
    for fun in $list_one_argument
    do
        if sed -e "s|bool *$fun(void)||" $file | \
            grep "[^a-zA-Z_]$fun *( *[a-zA-Z_0-9.][a-zA-Z_0-9.]* *)" > /dev/null
        then
            echo "$fun(arg1) is deprecated and appreas in $file"
            exit 1
        fi
    done
    for fun in $list_two_argument
    do
        if grep "[^a-zA-Z_]$fun *([^,)]*,[^,)]*)" $file > /dev/null
        then
            echo "$fun(arg1,arg2) is deprecated and appreas in $file"
            exit 1
        fi
    done
    for fun in $list_three_argument
    do
        if grep "[^a-zA-Z_]$fun *([^,)]*,[^,)]*,[^,)]*)" $file > /dev/null
        then
            echo "$fun(arg1,arg2,arg3) is deprecated and appreas in $file"
            exit 1
        fi
    done
done
# -----------------------------------------------------------------------------
echo 'bin/check_deprecated.sh: OK'
exit 0
