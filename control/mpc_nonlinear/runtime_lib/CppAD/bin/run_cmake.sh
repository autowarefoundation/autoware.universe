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
if [ ! -e "bin/run_cmake.sh" ]
then
    echo "bin/run_cmake.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
# prefix
eval `grep '^prefix=' bin/get_optional.sh`
if [[ "$prefix" =~ ^[^/] ]]
then
    prefix="$(pwd)/$prefix"
fi
echo "prefix=$prefix"
# -----------------------------------------------------------------------------
# PKG_CONFIG_PATH
PKG_CONFIG_PATH="$prefix/lib64/pkgconfig:$prefix/lib/pkgconfig"
PKG_CONFIG_PATH="$prefix/share/pkgconfig:$PKG_CONFIG_PATH"
export PKG_CONFIG_PATH
# -----------------------------------------------------------------------------
addr_t_size_t='no'
verbose='no'
standard='c++17'
profile_speed='no'
clang='no'
yes_adolc='yes'
yes_colpack='yes'
yes_eigen='yes'
yes_ipopt='yes'
yes_fadbad='yes'
yes_cppadcg='yes'
yes_sacado='yes'
yes_documentation='yes'
testvector='boost'
debug_which='debug_all'
while [ "$1" != "" ]
do
    if [ "$1" == '--help' ]
    then
        cat << EOF
usage: bin/run_cmake.sh: \\
    [--help] \\
    [--addr_t_size_t] \\
    [--verbose] \\
    [--c++11] \\
    [--profile_speed] \\
    [--callgrind] \\
    [--clang ] \\
    [--no_adolc] \\
    [--no_colpack] \\
    [--no_eigen] \\
    [--no_ipopt] \\
    [--no_fadbad] \\
    [--no_cppadcg] \\
    [--no_sacado] \\
    [--no_optional] \\
    [--no_documentation] \\
    [--<package>_vector] \\
    [--debug_<which>]
The --help option just prints this message and exits.
The value <package> above must be one of: cppad, boost, or eigen.
The value <which> must be one of: odd, even, all, none.

EOF
        exit 0
    fi
    case "$1" in

        --addr_t_size_t)
        addr_t_size_t='yes'
        ;;

        --verbose)
        verbose='yes'
        ;;

        --c++11)
        standard='c++11'
        ;;

        --profile_speed)
        profile_speed='yes'
        ;;

        --callgrind)
        callgrind='yes'
        ;;

        --clang)
        clang='yes'
        ;;

        --no_adolc)
        yes_adolc='no'
        ;;

        --no_colpack)
        yes_colpack='no'
        ;;

        --no_eigen)
        yes_eigen='no'
        ;;

        --no_ipopt)
        yes_ipopt='no'
        ;;

        --no_cppadcg)
        yes_cppadcg='no'
        ;;

        --no_fadbad)
        yes_fadbad='no'
        ;;

        --no_sacado)
        yes_sacado='no'
        ;;

        --no_optional)
        yes_adolc='no'
        yes_colpack='no'
        yes_eigen='no'
        yes_ipopt='no'
        yes_cppadcg='no'
        yes_fadbad='no'
        yes_sacado='no'
        testvector='cppad'
        ;;

        --no_documentation)
        yes_documentation='no'
        ;;

        --cppad_vector)
        testvector='cppad'
        ;;

        --boost_vector)
        testvector='boost'
        ;;

        --eigen_vector)
        testvector='eigen'
        ;;

        --std_vector)
        testvector='std'
        ;;

        --debug_odd)
        debug_which='debug_odd'
        ;;

        --debug_even)
        debug_which='debug_even'
        ;;

        --debug_all)
        debug_which='debug_all'
        ;;

        --debug_none)
        debug_which='debug_none'
        ;;

        *)
        echo "$1 is an invalid option, try bin/run_cmake.sh --help"
        exit 1
    esac
    shift
done
# ---------------------------------------------------------------------------
if [ ! -e build ]
then
    echo_eval mkdir build
fi
echo_eval cd build
if [ -e CMakeCache.txt ]
then
    echo_eval rm CMakeCache.txt
fi
if [ -e CMakeFiles ]
then
    echo_eval rm -r CMakeFiles
fi
# ---------------------------------------------------------------------------
# clean all variables in cmake cache
cmake_args='-U .+'
#
if [ "$verbose" == 'yes' ]
then
    # echo each command that make executes
    cmake_args="$cmake_args  -D CMAKE_VERBOSE_MAKEFILE=YES"
fi
# -----------------------------------------------------------------------------
# cppad_prefix
cmake_args="$cmake_args  -D cppad_prefix=$prefix"
#
# cmake_install_includedirs
if [ -d '/usr/include' ]
then
    cmake_args="$cmake_args -D cmake_install_includedirs=include"
fi
#
# cmake_install_datadir
if [ -d '/usr/share' ]
then
    cmake_args="$cmake_args -D cmake_install_datadir=share"
fi
#
# cmake_install_docdir
if [ -d '/usr/share' ] && [ "$yes_documentation" == 'yes' ]
then
    cmake_args="$cmake_args -D cmake_install_docdir=share/doc"
fi
#
# cmake_install_libdirs
if [ -d '/usr/lib64' ]
then
    cmake_args="$cmake_args -D cmake_install_libdirs='lib64;lib'"
elif [ -d '/usr/lib' ]
then
    cmake_args="$cmake_args -D cmake_install_libdirs='lib;lib64'"
fi
#
# {package}_prefix
prefix_list=''
include_list=''
if [ "$yes_cppadcg" == 'yes' ]
then
    if [ ! -e "$prefix/include/cppad/cg/cg.hpp" ]
    then
        echo "Cannot find $prefix/include/cppad/cg/cg.hpp"
        exit 1
    fi
    include_list="$include_list cppadcg"
fi
if [ "$yes_fadbad" == 'yes' ]
then
    if [ ! -e "$prefix/include/FADBAD++/badiff.h" ]
    then
        echo "Cannot find $prefix/include/FADBAD++/badiff.h"
        exit 1
    fi
    prefix_list="$prefix_list fadbad"
fi
if [ "$yes_adolc" == 'yes' ]
then
    if [ ! -d "$prefix/include/adolc" ]
    then
        echo "Cannot file $prefix/include/adolc"
        exit 1
    fi
    include_list="$include_list adolc"
fi
if [ "$yes_colpack" == 'yes' ]
then
    if [ ! -e "$prefix/include/ColPack" ]
    then
        echo "Cannot find $prefix/include/ColPack"
        exit 1
    fi
    prefix_list="$prefix_list colpack"
fi
if [ "$yes_eigen" == 'yes' ]
then
    if [ ! -e "$prefix/include/Eigen" ]
    then
        echo "Cannot find $prefix/include/Eigen"
        exit 1
    fi
    include_list="$include_list eigen"
fi
if [ "$yes_ipopt" == 'yes' ]
then
    if [ ! -e "$prefix/include/coin-or/IpNLP.hpp" ]
    then
        echo "Cannot find $prefix/include/coin-or/IpoptConfig.hpp"
        exit 1
    fi
    include_list="$include_list ipopt"
fi
if [ "$yes_sacado" == 'yes' ]
then
    if [ ! -e "$prefix/include/Sacado_config.h" ]
    then
        echo "Cannot find $prefix/include/Sacado_config.h"
        exit
    fi
    prefix_list="$prefix_list sacado"
fi
for package in $include_list
do
    cmake_args="$cmake_args -D include_${package}=true"
done
for package in $prefix_list
do
    cmake_args="$cmake_args  -D ${package}_prefix=$prefix"
done
#
# cppad_cxx_flags
cppad_cxx_flags="-Wall -pedantic-errors -std=$standard -Wshadow"
cppad_cxx_flags="$cppad_cxx_flags -Wfloat-conversion -Wconversion"
if [ "$debug_which" == 'debug_odd' ] || [ "$debug_which" == 'debug_even' ]
then
    cppad_cxx_flags="$cppad_cxx_flags -D CPPAD_DEBUG_AND_RELEASE"
fi
if [ "$callgrind" == 'yes' ]
then
    if [ "$debug_which" != 'debug_none' ]
    then
        echo 'run_cmake.sh: --callgrind requires --debug_none'
        exit 1
    fi
    cppad_cxx_flags="$cppad_cxx_flags -g"
fi
cmake_args="$cmake_args -D cppad_cxx_flags='$cppad_cxx_flags'"
#
# clang
if [ "$clang" == 'yes' ]
then
    cmake_args="$cmake_args -D CMAKE_C_COMPILER=clang"
    cmake_args="$cmake_args -D CMAKE_CXX_COMPILER=clang++"
fi
#
# profile
if [ "$profile_speed" == 'yes' ]
then
    cmake_args="$cmake_args -D cppad_profile_flag=-pg"
fi
#
# simple options
cmake_args="$cmake_args -D cppad_testvector=$testvector"
cmake_args="$cmake_args -D cppad_debug_which=$debug_which"
cmake_args="$cmake_args -D cppad_max_num_threads=48"
if [ "$addr_t_size_t" == 'yes' ]
then
    cmake_args="$cmake_args -D cppad_tape_id_type='size_t'"
    cmake_args="$cmake_args -D cppad_tape_addr_type=size_t"
else
    cmake_args="$cmake_args -D cppad_tape_id_type='int32_t'"
    cmake_args="$cmake_args -D cppad_tape_addr_type=int32_t"
fi
#
echo_eval cmake $cmake_args ..
#
# ----------------------------------------------------------------------------
echo "$0: OK"
exit 0
