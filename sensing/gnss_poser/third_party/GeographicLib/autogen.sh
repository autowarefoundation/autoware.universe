#!/bin/sh
#
# A simple script to add/reset autotools support
# This requires that autoconf and autoconf-archive are installed

run ()
{
    echo "Running: $*"
    eval $*

    if test $? != 0 ; then
        echo "Error: while running '$*'"
        exit 1
    fi
}

mv m4/pkg.m4 ./
rm -rf autom4te.cache m4
mkdir m4
mv pkg.m4 m4/
run aclocal
run autoheader
LIBTOOLIZE=libtoolize
type $LIBTOOLIZE > /dev/null 2>&1 || LIBTOOLIZE=g$LIBTOOLIZE
run $LIBTOOLIZE --force --copy
run automake --add-missing --copy --force-missing --foreign
run autoconf
run autoreconf

rm -rf autom4te.cache configure~
