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
# $begin get_cppadcg.sh$$ $newlinech #$$
# $spell
#   gz
#   CppAD
#   cppadcg
#   Eigen
#   doxygen
#   html
# $$
#
# $section Download and Install CppADCodeGen in Build Directory$$
#
# $head Syntax$$
# $code bin/get_cppadcg.sh$$
#
# $head Purpose$$
# If you are using Unix, this command will download and install
# $href%https://github.com/joaoleal/CppADCodeGen%cppadcg%$$
# in the CppAD $code build$$ directory.
#
# $head Requirements$$
# You must first use $cref get_eigen.sh$$ to download and install Eigen.
#
# $head Distribution Directory$$
# This command must be executed in the
# $cref/distribution directory/download/Distribution Directory/$$.
#
# $head Source Directory$$
# The Cppadcg source code is downloaded into the sub-directory
# $code external/cppadcg.git$$ below the distribution directory.
#
# $head Prefix$$
# The $cref/prefix/get_optional.sh/prefix/$$
# in the file $code bin/get_optional.sh$$ is used for this install.
#
# $head Git Hash$$
# This will install the commit of Cppadcg with the following git hash
# $srccode%sh%
git_hash='b5307ad'
# %$$
# The date corresponding to this commit was 20201009.
#
# $head Configuration$$
# If the file
# $codei%
#   external/cppadcg-%git_hash%.configured
# %$$
# exists, the configuration will be skipped.
# Delete this file if you want to re-run the configuration.
#
# $head Documentation$$
# If you change the setting for $code CREATE_DOXYGEN_DOC$$ to $code ON$$,
# the doxygen documentation for CppADCodeGen will be installed in the directory
# $codei%
#   %prefix%/share/doc/cppadcg/html
# %$$
# where $icode prefix$$ has the value specified in the
# $cref/get_optional.sh/get_optional.sh/prefix/$$ file.
#
# $end
# -----------------------------------------------------------------------------
package='cppadcg'
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
web_page='https://github.com/joaoleal/CppADCodeGen.git'
cppad_repo=$(pwd)
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
    prefix="$cppad_repo/$prefix"
fi
echo "prefix=$prefix"
# -----------------------------------------------------------------------------
configured_flag="external/$package-${git_hash}.configured"
echo "Executing get_$package.sh"
if [ -e "$configured_flag" ]
then
    echo "Skipping configuration because $configured_flag exits"
    echo_eval cd external/$package.git/build
    echo_eval make -j $n_job install
    echo "get_$package.sh: OK"
    exit 0
fi
# -----------------------------------------------------------------------------
# Create build/cppad_lib/libcppad_lib.* to aid in cppadcg install
if [ ! -d build ]
then
    echo_eval mkdir build
fi
echo_eval cd build
if [ -e CMakeCache.txt ]
then
    echo_eval rm CMakeCache.txt
fi
echo_eval cmake ..
echo_eval make
echo_eval cd ..
# -----------------------------------------------------------------------------
# Change into external
if [ ! -d external ]
then
    echo_eval mkdir external
fi
echo_eval cd external
# -----------------------------------------------------------------------------
# cppadcg.git
if [ ! -e $package.git ]
then
    echo_eval git clone $web_page $package.git
fi
echo_eval cd $package.git
# -----------------------------------------------------------------------------
# Modify FindCppAD.cmake so can use git repository
# version of CppAD (not yet installed).
cat << EOF > get_cppadcg.sed
s|IF *( *DEFINED *CPPAD_HOME *)|IF (DEFINED CPPAD_GIT_REPO)\\
    # This setting is used for testing before installing CppAD.\\
    # CPPAD_GIT_REPO is the a CppAD git repository. It is assumed that\\
    # cmake and make have been executed in CPPAD_GIT_REPO/build.\\
    SET(CPPAD_INCLUDE_DIR "\${CPPAD_GIT_REPO}/include" )\\
    SET(CPPAD_LIBRARIES\\
        "\${CPPAD_GIT_REPO}/build/cppad_lib"\\
    )\\
    INCLUDE_DIRECTORIES(\\
        "\${CPPAD_INCLUDE_DIR}"\\
    )\\
    #\\
    IF( NOT EXISTS "\${CPPAD_INCLUDE_DIR}/cppad/cppad.hpp" )\\
        MESSAGE(FATAL_ERROR\\
            "Cannot find CPPAD_GIT_REPO/include/cppad/cppad.hpp"\\
        )\\
    ENDIF()\\
    IF( NOT EXISTS "\${CPPAD_INCLUDE_DIR}/cppad/configure.hpp" )\\
        MESSAGE(FATAL_ERROR\\
            "Cannot find CPPAD_GIT_REPO/include/cppad/configure.hpp"\\
        )\\
    ENDIF()\\
    #\\
    FIND_LIBRARY( CPPAD_LIB_PATH\\
        cppad_lib\\
        PATHS \${CPPAD_LIBRARIES}\\
        NO_DEFAULT_PATH\\
    )\\
    IF( NOT CPPAD_LIB_PATH  )\\
        MESSAGE(FATAL_ERROR\\
            "Cannot find \${library} library below CPPAD_GIT_REPO="\\
            "{CPPAD_GIT_REPO}"\\
        )\\
    ENDIF()\\
    #\\
    SET(CPPAD_FOUND TRUE)\\
\\
ELSEIF (DEFINED CPPAD_HOME)|
EOF
echo_eval git checkout  cmake/FindCppAD.cmake
echo_eval sed -i cmake/FindCppAD.cmake -f get_cppadcg.sed
echo_eval rm get_cppadcg.sed
# -----------------------------------------------------------------------------
#  make install
echo_eval git checkout --quiet $git_hash
if [ ! -e build ]
then
    echo_eval mkdir build
fi
echo_eval cd build
echo_eval cmake \
    -D CPPAD_GIT_REPO="$cppad_repo" \
    -D CMAKE_INSTALL_PREFIX=$prefix \
    -D EIGNE_INCLUDE_DIR=$prefix/include \
    -D GOOGLETEST_GIT=ON \
    -D CREATE_DOXYGEN_DOC=OFF \
    ..
echo_eval make -j $n_job install
# -----------------------------------------------------------------------------
echo_eval touch $cppad_repo/$configured_flag
echo "get_$package.sh: OK"
