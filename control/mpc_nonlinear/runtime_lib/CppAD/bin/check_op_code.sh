#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ ! -e "bin/check_op_code.sh" ]
then
    echo "bin/check_op_code.sh: must be executed from its parent directory"
    exit 1
fi
echo "bin/check_op_code.sh: checking that op codes are in alphabetical order:"
file='include/cppad/local/op_code_var.hpp'
# ---------------------------------------------------------------------------
# check enum list of codes are in alphabetical order
sed -n -e '/^enum/,/^    NumberOp  /p' $file | sed \
    -e '/^enum/d' \
    -e '/^    NumberOp  /d' \
    -e 's/^[ ]*//' \
    -e 's/Op[, ].*//' \
    -e '/^\/\//d' > op_code.1.$$
#
sort --ignore-case op_code.1.$$ > op_code.2.$$
if ! diff op_code.1.$$ op_code.2.$$
then
    echo "check_op_code.sh: enum list is not in alphabetical order"
    rm op_code.*.$$
    exit 1
fi
# -----------------------------------------------------------------------------
# check NumArgTable
sed -n -e '/NumArgTable\[\]/,/^[ ]*};/p' $file | \
    sed \
        -e '/NumArgTable\[\]/d' \
        -e '/NumberOp.*not used/d' \
        -e '/^[ ]*};/d' \
        -e 's|^[ ]*[0-9],* *// *||' \
        -e 's|Op.*||' \
        > op_code.3.$$
#
if ! diff op_code.1.$$ op_code.3.$$
then
    echo "check_op_code.sh: NumArgTable list is not in alphabetical order"
    rm op_code.*.$$
    exit 1
fi
# -----------------------------------------------------------------------------
# check NumResTable (last line of NumResTable is not used)
sed -n -e '/NumResTable\[\]/,/^[ ]*};/p' $file | \
    sed \
        -e '/NumResTable\[\]/d' \
        -e '/^[ ]*};/d' \
        -e '/NumberOp.*not used/d' \
        -e 's|^[ ]*[0-9],* *// *||' \
        -e 's|Op.*||' \
        > op_code.4.$$
#
if ! diff op_code.1.$$ op_code.4.$$
then
    echo "check_op_code.sh: NumResTable list is not in alphabetical order"
    echo "(or missing last line)"
    rm op_code.*.$$
    exit 1
fi
# -----------------------------------------------------------------------------
# check OpNameTable
sed -n -e '/const char \*OpNameTable\[\]/,/^[ ]*};/p' $file | \
    sed \
        -e '/OpNameTable\[\]/d' \
        -e '/"Number".*not used/d' \
        -e '/^[ ]*};/d' \
        -e 's|^[ ]*"||' \
        -e 's|".*||' \
        > op_code.5.$$
#
if ! diff op_code.1.$$ op_code.5.$$
then
    echo "check_op_code.sh: OpName list is not in alphabetical order"
    rm op_code.*.$$
    exit 1
fi
# -----------------------------------------------------------------------------
# clean up
rm op_code.*.$$
# ----------------------------------------------------------------------------
echo "$0: OK"
exit 0
