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
if [ "$0" != 'bin/speed_package.sh' ]
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
#! /bin/bash -e
git reset --hard
package='cppadcg'
include_file='cppad/cg/cg.hpp'
home_page='https://github.com/joaoleal/CppADCodeGen.git'
# --------------------------------------------------------
Package=`echo $package | sed -e 's|^.|\U&|'`
PACKAGE=`echo $package | sed -e 's|^.*|\U&|'`
# --------------------------------------------------------
# speed/package directory
cat << EOF > speed_package.$$
s|XPACKAGE|$PACKAGE|g
s|Xpackage|$Package|g
s|xpackage|$package|g
EOF
if [ -e speed/$package ]
then
    echo_eval rm -r speed/$package
fi
cp -r speed/xpackage speed/$package
git add speed/$package
list=`ls speed/$package`
for file in $list
do
    echo_eval sed -i speed/$package/$file -f speed_package.$$
done
# --------------------------------------------------------
# omh/speed_package.omh
cat << EOF > speed_package.$$
/\$head fadbad_prefix/! b one
: loop_1
N
/\\n *\$/! b loop_1
s|.*|\\n|
b end
#
: one
s|Fadbad|$Package|g
s|fadbad|$package|g
: end
EOF
file="speed/$package/speed_$package.omh"
cp speed/fadbad/speed_fadbad.omh $file
git add $file
echo_eval sed -i $file -f speed_package.$$
# --------------------------------------------------------
# speed/speed.omh
cat << EOF > speed_package.$$
s|\$cref/Fadbad/fadbad_prefix/Fadbad Home Page/\$\$,|&\\
\$cref/$Package/${package}_prefix/$Package Home Page/\$\$,|
s|^\\( *\\)speed/fadbad/speed_fadbad.omh%|&\\
\\1speed/$package/speed_$package.omh%|
s|\$spell|&\\
    $Package|
EOF
file='speed/speed.omh'
echo_eval sed -i $file -f speed_package.$$
# --------------------------------------------------------
# speed/main.cpp
cat << EOF > speed_package.$$
s|# ifdef CPPAD_XPACKAGE_SPEED|# ifdef CPPAD_${PACKAGE}_SPEED\\
# define AD_PACKAGE "$package"\\
# endif\\
&|
EOF
file='speed/main.cpp'
echo_eval sed -i $file -f speed_package.$$
# --------------------------------------------------------
# speed/CMakeLists.txt
cat << EOF > speed_package.$$
/ENDIF( *cppad_has_fadbad *)/! b end
s|\$|\\
IF( cppad_has_$package )\\
    ADD_SUBDIRECTORY($package)\\
ENDIF( cppad_has_$package )|
: end
EOF
file='speed/CMakeLists.txt'
echo_eval sed -i $file -f speed_package.$$
# --------------------------------------------------------
# CMakeLists.txt
cat << EOF > speed_package.$$
/optional_package(fadbad/! b end
s|\$|\\
#\\
# ${package}_prefix\\
optional_package($package \${system_include} "$package install prefix")|
: end
EOF
file='CMakeLists.txt'
echo_eval sed -i $file -f speed_package.$$
# --------------------------------------------------------
# omh/install/package_prefix
short_name=`echo $include_file | sed -e 's|.*/||'`
cat << EOF > speed_package.$$
s|http://www.fadbad.com|$home_page|
s|%fadbad_prefix%/%dir%/FADBAD++/badiff.h|%${package}_prefix%/%dir%/$include_file|
s|badiff[.]h|$short_name|
s|Fadbad|$Package|g
s|fadbad|$package|g
s|\$spell|&\\
    $include_file|
EOF
file="omh/install/${package}_prefix.omh"
cp omh/install/fadbad_prefix.omh $file
git add $file
echo_eval sed -i $file -f speed_package.$$
# --------------------------------------------------------
# omh/install/cmake.omh
cat << EOF > speed_package.$$
/^    -D fadbad_prefix=%fadbad_prefix%/! b one
s|\$|\\
    -D ${package}_prefix=%${package}_prefix% \\\\|
b end
#
:one
s|^\$rref fadbad_prefix\$\\\$|&\\
\$rref ${package}_prefix\$\$|
s|^    omh/install/fadbad_prefix.omh%|&\\
    omh/install/${package}_prefix.omh%|
s|\$spell|&\\
    $package|
#
: end
EOF
file='omh/install/cmake.omh'
echo_eval sed -i $file -f speed_package.$$
# --------------------------------------------------------
# bin/run_cmake.sh
cat << EOF > speed_package.$$
s|^yes_fadbad='yes'|&\\
yes_$package='yes'|
s|^\\( *\\)\\[--no_fadbad\\].*|&\\
\\1[--no_$package] \\\\\\\\|
s|^\\( *\\)--no_fadbad)|\\1--no_$package)\\
\\1yes_$package='no'\\
\\1;;\\
\\
&|
s|^package_list=''|&\\
if [ "\$yes_$package" == 'yes' ]\\
then\\
    if [ ! -e "\$prefix/include/$include_file" ]\\
    then\\
        echo "Cannot find \$prefix/include/$include_file"\\
        exit 1\\
    fi\\
    package_list="\$package_list $package"\\
fi|
EOF
file='bin/run_cmake.sh'
echo_eval sed -i $file -f speed_package.$$
# --------------------------------------------------------
echo 'speed_package.sh: OK'
exit 0
