#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
./print_for | tee test.log
sed -e '/^Test passes/,\$$d' < test.log > test.1
sed -e '1,/^Test passes/d'   < test.log > test.2
if ! diff test.1 test.2 ; then exit 1 ; fi
