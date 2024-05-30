#! /bin/sh -ex

# This is the script to build the utilities for the cgi-bin scripts.  Note:

# 0: To access the machine for running this command, use
#   ssh -t karney,geographiclib@shell.sourceforge.net create
# Once logged in the following paths are available
# git: /home/git/p/geographiclib/code.git
WEB=/home/project-web/geographiclib
FRS=/home/frs/project/geographiclib

# 1: The utilities used long double (GEOGRAPHICLIB_PRECISION=3)

# 2: GeoidEval needs access to geoid data which is installed in
# $WEB/geoids.  It find this with
#   export GEOGRAPHICLIB_DATA=..

# 3: Static libraries are used so the utilities are self contained
# executables.

VERSION=2.0
FULLVERSION=$VERSION-alpha
rm -rf /tmp/GeographicLib-$VERSION /tmp/geog-$VERSION
tar xfpzC $FRS/distrib-C++/GeographicLib-$FULLVERSION.tar.gz /tmp
cd /tmp/GeographicLib-$VERSION
# N.B. $HOME/cmake/bin is in PATH for cmake
cmake \
    -D CMAKE_INSTALL_PREFIX=/tmp/geog-$VERSION \
    -D BUILD_SHARED_LIBS=OFF \
    -D GEOGRAPHICLIB_PRECISION=3 \
    -D EXAMPLEDIR= -B BUILD -S .
cd BUILD
make
make install
mkdir -p $WEB/bin-$VERSION
cd /tmp/geog-$VERSION/bin
install CartConvert ConicProj GeodesicProj GeoConvert GeodSolve GeoidEval Gravity MagneticField Planimeter RhumbSolve TransverseMercatorProj $WEB/bin-$VERSION/
