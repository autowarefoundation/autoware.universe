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
# $begin get_sacado.sh$$ $newlinech #$$
# $spell
#   trilinos
#   gz
#   Sacado
#   CppAD
# $$
#
# $section Download and Install Sacado in Build Directory$$
#
# $head Syntax$$
# $code bin/get_sacado.sh$$
#
# $head Purpose$$
# If you are using Unix, this command will download and install
# $cref/Sacado/sacado_prefix/Sacado Home Page/$$
# in the CppAD $code build$$ directory.
#
# $head Distribution Directory$$
# This command must be executed in the
# $cref/distribution directory/download/Distribution Directory/$$.
#
# $head Source Directory$$
# The Sacado source code is downloaded into the sub-directory
# $code external/trilinos.git$$ below the distribution directory.
#
# $head Prefix$$
# The $cref/prefix/get_optional.sh/prefix/$$
# in the file $code bin/get_optional.sh$$ is used for this install.
#
# $head Version$$
# This will install the version of Sacado corresponding to the following
# version of Trilinos:
# $srccode%sh%
version='13-0-1'
# %$$
#
# $head Configuration$$
# If the file
# $codei%
#   external/sacado-%version%.configured
# %$$
# exists, the configuration will be skipped.
# Delete this file if you want to re-run the configuration.
#
#
# $end
# -----------------------------------------------------------------------------
package='sacado'
if [ $0 != "bin/get_$package.sh" ]
then
    echo "bin/get_$package.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
web_page='https://github.com/trilinos/Trilinos.git'
cppad_dir=`pwd`
# -----------------------------------------------------------------------------
# n_job
if which nproc > /dev/null
then
    n_job=$(nproc)
else
    n_job=$(sysctl -n hw.ncpu)
fi
# ----------------------------------------------------------------------------
# prefix
eval `grep '^prefix=' bin/get_optional.sh`
if [[ "$prefix" =~ ^[^/] ]]
then
    prefix="$cppad_dir/$prefix"
fi
echo "prefix=$prefix"
# -----------------------------------------------------------------------------
configured_flag="external/$package-${version}.configured"
echo "Executing get_$package.sh"
if [ -e "$configured_flag" ]
then
    echo "Skipping configuration because $configured_flag exits"
    echo_eval cd external/trilinos.git/build
    echo_eval make -j $n_job install
    echo "get_$package.sh: OK"
    exit 0
fi
# -----------------------------------------------------------------------------
# libdir
if [ -e /usr/lib64 ]
then
    libdir='lib64'
else
    libdir='lib'
fi
# -----------------------------------------------------------------------------
# change into external directory
if [ ! -d external ]
then
    echo_eval mkdir external
fi
echo_eval cd external
# -----------------------------------------------------------------------------
# create the trilions source directory and change into it
if [ ! -e trilinos.git ]
then
    echo_eval git clone $web_page trilinos.git
fi
echo_eval cd trilinos.git
echo_eval git checkout master
echo_eval git pull
echo_eval git checkout trilinos-release-$version
# -----------------------------------------------------------------------------
# change into build sub-directory
if [ ! -e build ]
then
    echo_eval mkdir build
fi
echo_eval cd build
# -----------------------------------------------------------------------------
echo_eval cmake \
    -D CMAKE_BUILD_TYPE:STRING=RELEASE \
    -D Trilinos_ENABLE_Sacado:BOOL=ON \
    -D Sacado_ENABLE_TESTS:BOOL=OFF \
    -D CMAKE_INSTALL_PREFIX:PATH=$prefix \
    -D Trilinos_INSTALL_LIB_DIR=$prefix/$libdir \
    ..
echo_eval make -j $n_job install
# -----------------------------------------------------------------------------
echo_eval touch $cppad_dir/$configured_flag
echo "get_$package.sh: OK"
