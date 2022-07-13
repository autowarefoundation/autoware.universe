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
# $OMhelpKeyCharacter=@
# @begin get_optional.sh@@ @newlinech #@@
# @spell
#   ls
#   CppAD
# @@
#
# @section Download and Install The CppAD Optional Packages@@
#
# @head Syntax@@
# @code bin/get_optional.sh@@
#
# @head Purpose@@
# If you are using Unix, this command will download and install
# all of the optional packages that can be used with CppAD.
#
# @head Distribution Directory@@
# This command must be executed in the
# @cref/distribution directory/download/Distribution Directory/@@.
#
# @head prefix@@
# This is the prefix for installing the optional packages.
# It can be changed by editing its setting of @icode prefix@@ below
# in the file @code bin/get_optional.sh@@.
# Note that there can only be one setting that is not commented out with
# a @code #@@ at the start of its line.
#
# @subhead Absolute Path@@
# If the first character in the prefix is a @code /@@,
# it is an absolute path; e.g., the following setting:
# @srccode%sh%
# prefix="$HOME/prefix/cppad"
# %@@
#
# @subhead Relative Path@@
# If the first character in the prefix is @bold not@@ a @code /@@,
# it is a path relative to the distribution directory;
# e.g., the following setting:
# @srccode%sh%
prefix="build/prefix"
# %@@
#
# @subhead Configuration@@
# If you do an install and then change the @icode prefix@@,
# you should delete all the files listed by the following command:
# @codei%
#   ls external/*.configured
# %@@
#
# @head get_optional.log@@
# This file contains the standard out output for each of the optional scripts
# in the order that they are executed.
#
# @head get_optional.err@@
# This file contains the standard error output for each of the optional scripts
# in the order that they are executed.
#
# @childtable%
#   bin/get_adolc.sh%
#   bin/get_cppadcg.sh%
#   bin/get_colpack.sh%
#   bin/get_eigen.sh%
#   bin/get_fadbad.sh%
#   bin/get_ipopt.sh%
#   bin/get_sacado.sh
# %@@
#
# @end
# -----------------------------------------------------------------------------
if [ $0 != "bin/get_optional.sh" ]
then
    echo "bin/get_optional.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
if [ -e 'get_optional.log' ]
then
    echo_eval rm get_optional.log
fi
if [ -e 'get_optional.err' ]
then
    echo_eval rm get_optional.err
fi
# -----------------------------------------------------------------------------
list='colpack adolc eigen fadbad ipopt sacado cppadcg'
for package in $list
do
    echo "bin/get_${package}.sh 1>> get_optional.log 2>> get_optional.err"
    if bin/get_${package}.sh 1>> get_optional.log 2>> get_optional.err
    then
        echo "bin/get_${package}.sh: OK"
    else
        echo "bin/get_${package}.sh: Error; try following:"
        echo '  tail ./get_optional.err'
        exit 1
    fi
done
# -----------------------------------------------------------------------------
echo "get_optional: OK"
