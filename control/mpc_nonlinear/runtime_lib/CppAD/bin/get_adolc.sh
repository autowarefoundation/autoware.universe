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
# $begin get_adolc.sh$$ $newlinech #$$
# $dollar @$$
# $spell
#   tgz
#   Adolc
#   gz
#   CppAD
#   Nov
# $$
#
# $section Download and Install Adolc in Build Directory$$
#
# $head Syntax$$
# $code bin/get_adolc.sh$$
#
# $head Purpose$$
# If you are using Unix, this command will download and install
# $cref/Adolc/adolc/Adolc Home Page/$$
# in the CppAD $code build$$ directory.
#
# $head Requirements$$
# You must first use $cref get_colpack.sh$$ to download and install
# the ColPack coloring algorithms (used for sparse matrix derivatives).
#
# $head Distribution Directory$$
# This command must be executed in the
# $cref/distribution directory/download/Distribution Directory/$$.
#
# $head Source Directory$$
# The Adolc source code is downloaded into the sub-directory
# $code external/adolc.git$$ below the distribution directory.
#
# $head Prefix$$
# The $cref/prefix/get_optional.sh/prefix/$$
# in the file $code bin/get_optional.sh$$ is used for this install.
#
# $head Version$$
# This will install the following version of Adolc
# $srccode%sh%
version='e1fe476'
# %$$
# This corresponds to the git master on Nov 13, 2020.
#
# $head Configuration$$
# If the file
# $codei%
#   external/adolc-%version%.configured
# %$$
# exists, the configuration will be skipped.
# Delete this file if you want to re-run the configuration.
#
# $end
# -----------------------------------------------------------------------------
package='adolc'
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
web_page='https://github.com/coin-or/ADOL-C.git'
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
    echo_eval cd external/$package.git/build
    echo_eval make -j $n_job install
    echo "get_$package.sh: OK"
    exit 0
fi
# --------------------------------------------------------------------------
if [ -e /usr/lib64 ]
then
    libdir='lib64'
else
    libdir='lib'
fi
# -----------------------------------------------------------------------------
if [ ! -d external ]
then
    echo_eval mkdir external
fi
echo_eval cd external
# -----------------------------------------------------------------------------
if [ ! -e "$package.git" ]
then
    echo_eval git clone $web_page $package.git
fi
echo_eval cd $package.git
echo_eval git reset --hard
echo_eval git checkout --quiet $version
# -----------------------------------------------------------------------------
system=`uname | tr [A-Z] [a-z] | sed -e 's|\([a-z][a-z]*\).*|\1|'`
# -----------------------------------------------------------------------------
if which autoconf >& /dev/null
then
    echo_eval autoreconf --install --force
fi
# -----------------------------------------------------------------------------
if [ ! -e build ]
then
    echo_eval mkdir build
fi
echo_eval cd build
# -----------------------------------------------------------------------------
flags="--prefix=$prefix --with-colpack=$prefix --libdir=$prefix/$libdir"
flags="$flags --enable-static --enable-shared --enable-atrig-erf"
#
echo_eval ../configure $flags
echo_eval make -j $n_job install
# -----------------------------------------------------------------------------
echo_eval touch $cppad_dir/$configured_flag
echo "get_$package: OK"
