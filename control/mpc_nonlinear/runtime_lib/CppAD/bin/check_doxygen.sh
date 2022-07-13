#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ ! -e "bin/check_doxygen.sh" ]
then
    echo "bin/check_doxygen.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
if ! bin/run_doxygen.sh
then
    echo 'check_doxygen.sh: Error'
    exit 1
fi
echo 'check_doxygen.sh: OK'
exit 0
