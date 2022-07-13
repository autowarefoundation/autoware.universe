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
# $begin get_fadbad.sh$$ $newlinech #$$
# $spell
#   Fadbad
#   CppAD
# $$
#
# $section Download and Install Fadbad in Build Directory$$
#
# $head Syntax$$
# $code bin/get_fadbad.sh$$
#
# $head Purpose$$
# If you are using Unix, this command will download and install
# $cref/Fadbad/fadbad_prefix/Fadbad Home Page/$$
# in the CppAD $code build$$ directory.
#
# $head Distribution Directory$$
# This command must be executed in the
# $cref/distribution directory/download/Distribution Directory/$$.
#
# $head Source Directory$$
# The Fadbad source code is downloaded into the sub-directory
# $code external/FADBAD++$$ below the distribution directory.
#
# $head Prefix$$
# The $cref/prefix/get_optional.sh/prefix/$$
# in the file $code bin/get_optional.sh$$ is used for this install.
#
# $head Version$$
# This will install the following version of Fadbad
# $srccode%sh%
version='2.1'
# %$$
#
# $end
# -----------------------------------------------------------------------------
package='fadbad'
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
web_page='http://www.fadbad.com/download'
cppad_dir=`pwd`
# -----------------------------------------------------------------------------
# prefix
eval `grep '^prefix=' bin/get_optional.sh`
if [[ "$prefix" =~ ^[^/] ]]
then
    prefix="$cppad_dir/$prefix"
fi
echo "prefix=$prefix"
# -----------------------------------------------------------------------------
if [ ! -d external ]
then
    echo_eval mkdir external
fi
echo_eval cd external
# -----------------------------------------------------------------------------
if [ ! -e "FADBAD++-$version.tar.gz" ]
then
    echo_eval wget --no-check-certificate $web_page/FADBAD++-$version.tar.gz
fi
if [ -e "FADBAD++" ]
then
    echo_eval rm -r FADBAD++
fi
echo_eval tar -xzf FADBAD++-$version.tar.gz
if [ ! -e "$prefix/include" ]
then
    echo_eval mkdir -p "$prefix/include"
fi
if [ -e "$prefix/include/FADBAD++" ]
then
    echo_eval rm -r "$prefix/include/FADBAD++"
fi
echo_eval cp -r FADBAD++ "$prefix/include/FADBAD++"
# -----------------------------------------------------------------------------
echo "get_$package.sh: OK"
