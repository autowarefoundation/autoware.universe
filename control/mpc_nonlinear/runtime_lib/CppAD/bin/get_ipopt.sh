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
# $begin get_ipopt.sh$$ $newlinech #$$
# $spell
#   tgz
#   Ipopt
#   CppAD
#   Blas
#   Lapack
# $$
#
# $section Download and Install Ipopt in Build Directory$$
#
# $head Syntax$$
# $code bin/get_ipopt.sh$$
#
# $head Purpose$$
# If you are using Unix, this command will download and install
# $href%http://www.coin-or.org/projects/Ipopt.xml%Ipopt%$$ in the
# CppAD $code build$$ directory.
#
# $head Requirements$$
# This It is assumed that a copy of the Blas and Lapack is installed on
# the system.
#
# $head Distribution Directory$$
# This command must be executed in the
# $cref/distribution directory/download/Distribution Directory/$$.
#
# $head Source Directory$$
# The Ipopt source code is downloaded and compiled in the sub-directory
# $codei%external/Ipopt-%version%$$ below the distribution directory.
#
# $head Prefix$$
# The $cref/prefix/get_optional.sh/prefix/$$
# in the file $code bin/get_optional.sh$$ is used for this install.
#
# $head Version$$
# This will install the following version of Ipopt
# $srccode%sh%
version='3.13.2'
# %$$
#
# $head Configuration$$
# If the file
# $codei%
#   external/ipopt-%version%.configured
# %$$
# exists, the configuration will be skipped.
# Delete this file if you want to re-run the configuration.
#
# $end
# -----------------------------------------------------------------------------
package='ipopt'
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
coinbrew='https://raw.githubusercontent.com/coin-or/coinbrew/master/coinbrew'
cppad_dir=`pwd`
# -----------------------------------------------------------------------------
# n_proc
if which nproc >& /dev/null
then
    n_job=$(nproc)
else
    n_job=$(sysctl -n hw.ncpu)
fi
# -----------------------------------------------------------------------------
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
    echo_eval cd external
    ./coinbrew -j $n_job install Ipopt --no-prompt
    echo "get_$package.sh: OK"
    exit 0
fi
# -----------------------------------------------------------------------------
if [ ! -d external ]
then
    echo_eval mkdir external
fi
echo_eval cd external
# -----------------------------------------------------------------------------
if [ ! -e coinbrew ]
then
    echo_eval wget $coinbrew
    echo_eval chmod +x coinbrew
fi
if [ ! -e Ipoot ]
then
    ./coinbrew fetch Ipopt@$version --no-prompt
fi
# -----------------------------------------------------------------------------
# klugde necessary until coin or mumps fixes this problem
cat << EOF > junk.f
      program junk
      print*, "Hello World"
      end
EOF
if gfortran -c -fallow-argument-mismatch junk.f >& /dev/null
then
    echo 'Adding -fallow-argument-mismatch to Mumps fortran compiler flags'
    ADD_FCFLAGS='ADD_FCFLAGS=-fallow-argument-mismatch'
else
    ADD_FCFLAGS=''
fi
# -----------------------------------------------------------------------------
echo_eval ./coinbrew -j $n_job build Ipopt@$version \
    --prefix=$prefix --test --no-prompt --verbosity=3 $ADD_FCFLAGS
echo_eval ./coinbrew -j $n_job install Ipopt@$version \
    --no-prompt
# -----------------------------------------------------------------------------
echo_eval touch $cppad_dir/$configured_flag
echo "get_$package.sh: OK"
