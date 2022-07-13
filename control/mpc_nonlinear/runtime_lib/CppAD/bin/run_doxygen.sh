#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ $0 != 'bin/run_doxygen.sh' ]
then
    echo 'bin/run_doxygen.sh: must be executed from its parent directory'
    exit 1
fi
echo_eval() {
     echo $*
     eval $*
}
# -----------------------------------------------------------------------------
# run doxygen
version=`version.sh get`
error_file='doxygen.err'
output_directory='doxydoc'
for name in doxyfile $error_file $output_directory
do
    if [ -e $name ]
    then
        echo_eval rm -r $name
    fi
done
echo_eval mkdir doxydoc
echo_eval bin/doxyfile.sh $version $error_file $output_directory
#
echo 'doxygen doxyfile > doxygen.log'
doxygen doxyfile       > doxygen.log
# -----------------------------------------------------------------------------
# check for warnings and errors
#
doxygen_version=`doxygen --version  |
    sed -e 's|\.|*100+|' -e 's|\.|*10+|' -e 's|\..*||'`
let doxygen_version=$doxygen_version
if (( $doxygen_version <= 155 ))
then
    doxygen_version=`doxygen --version`
    echo "doxygen version $doxygen_version is <= 1.5.6"
    echo "Hence it is to old to check for warnings or errors."
    exit 0
fi
if (( $doxygen_version == 163 ))
then
    doxygen_version=`doxygen --version`
    echo "doxygen version $doxygen_version is == 1.6.3"
    echo "Hence it has a problem with warnings about missing # defines;"
    echo "see http://comments.gmane.org/gmane.text.doxygen.general/8594"
    exit 0
fi
list=`head doxygen.err`
if [ "$list" == "" ]
then
    echo 'run_doxygen.sh OK'
    exit 0
fi
echo 'bin/run_doxygen.sh: Doxygen errors or warnings; see doxygen.err'
echo 'run_doxygen.sh: Error'
exit 1
