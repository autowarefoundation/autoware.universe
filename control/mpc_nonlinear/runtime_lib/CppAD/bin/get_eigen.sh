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
# $begin get_eigen.sh$$ $newlinech #$$
# $spell
#   gz
#   Eigen
#   CppAD
# $$
#
# $section Download and Install Eigen in Build Directory$$
#
# $head Syntax$$
# $code bin/get_eigen.sh$$
#
# $head Purpose$$
# If you are using Unix, this command will download and install
# $cref/Eigen/eigen/Eigen Home Page/$$
# in the CppAD $code build$$ directory.
#
# $head Distribution Directory$$
# This command must be executed in the
# $cref/distribution directory/download/Distribution Directory/$$.
#
# $head Source Directory$$
# The Eigen source code is downloaded into the sub-directory
# $code external/eigen.git$$ below the distribution directory.
#
# $head Prefix$$
# The $cref/prefix/get_optional.sh/prefix/$$
# in the file $code bin/get_optional.sh$$ is used for this install.
#
# $head Version$$
# This will install the following version of Eigen
# $srccode%sh%
version='3.3.7'
# %$$
#
# $head Configuration$$
# If the file
# $codei%
#   external/eigen-%version%.configured
# %$$
# exists, the configuration will be skipped.
# Delete this file if you want to re-run the configuration.
#
# $end
# -----------------------------------------------------------------------------
package='eigen'
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
web_page='https://gitlab.com/libeigen/$package.git'
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
    if [ -e $prefix/include/Eigen ]
    then
        echo_eval rm $prefix/include/Eigen
    fi
    echo_eval ln -s $prefix/include/eigen3/Eigen $prefix/include/Eigen
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
if [ ! -e $package.git ]
then
    echo_eval git clone $web_page $package.git
fi
# -----------------------------------------------------------------------------
echo_eval cd $package.git
echo_eval git checkout master
echo_eval git checkout --quiet $version
if [ ! -e build ]
then
    echo_eval mkdir build
fi
echo_eval cd build
echo_eval cmake .. -DCMAKE_INSTALL_PREFIX=$prefix
echo_eval make -j $n_job install
if [ -e $prefix/include/Eigen ]
then
    echo_eval rm $prefix/include/Eigen
fi
echo_eval ln -s $prefix/include/eigen3/Eigen $prefix/include/Eigen
# -----------------------------------------------------------------------------
echo_eval touch $cppad_dir/$configured_flag
echo "get_$package.sh: OK"
