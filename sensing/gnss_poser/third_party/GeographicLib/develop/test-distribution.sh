#! /bin/sh
#
# tar.gz and zip distrib files copied to $DEVELSOURCE
# html documentation rsync'ed to $WEBDIST/htdocs/C++/$VERSION/
#
# Windows version ready to build in
# $WINDOWSBUILD/GeographicLib-$VERSION/BUILD-vc10{,-x64}
# after ./build installer is copied to
# $DEVELSOURCE/GeographicLib-$VERSION-win{32,64}.exe
#
# Built version ready to install in /usr/local in
# relc/GeographicLib-$VERSION/BUILD-system
#
# gita - check out from git, create package with cmake
# gitb - check out from git, check various builds in the non-release tree
# gitr - new release branch
# relb - release package, build with autoconf
# relc - release package, build with cmake
# relx - cmake release package inventory
# rely - autoconf release package inventory
# instb - installed files, autoconf
# instc - installed files, cmake

set -e
umask 0022

# The following files contain version information:
#   pom.xml
#   CMakeLists.txt (PROJECT_VERSION_* LIBVERSION_*)
#   NEWS
#   configure.ac (AC_INIT, GEOGRAPHICLIB_VERSION_* LT_*)
#   develop/test-distribution.sh
#   doc/GeographicLib.dox.in (3 places)

# maxima
#   maxima/geodesic.mac

START=`date +%s`
DATE=`date +%F`
VERSION=2.3
SUFFIX=
DISTVERSION=$VERSION$SUFFIX
BRANCH=main
TEMP=/home/scratch/geographiclib-dist
if test `hostname` = petrel; then
    DEVELSOURCE=$HOME/geographiclib
    WINDEVELSOURCE=//datalake-pr-smb/vt-open/ckarney/geographiclib
    WINDOWSBUILD=/var/tmp
else
    DEVELSOURCE=/u/geographiclib
    WINDEVELSOURCE=//datalake-pr-smb/vt-open/ckarney/geographiclib
    WINDOWSBUILD=/u/temp
fi
WINDOWSBUILDWIN=//datalake-pr-smb/vt-open/ckarney/temp
GITSOURCE=file://$DEVELSOURCE
WEBDIST=/home/ckarney/web/geographiclib-web
mkdir -p $WEBDIST/htdocs/C++
NUMCPUS=4
HAVEINTEL=

test -d $TEMP || mkdir $TEMP
rm -rf $TEMP/*
mkdir $TEMP/gita # Package creation via cmake
mkdir $TEMP/gitb # Non release testing
mkdir $TEMP/gitr # For release branch
(cd $TEMP/gitr; git clone -b $BRANCH $GITSOURCE)
(cd $TEMP/gita; git clone -b $BRANCH file://$TEMP/gitr/geographiclib)
(cd $TEMP/gitb; git clone -b $BRANCH file://$TEMP/gitr/geographiclib)

echo ==============================================================
echo Make a source package in $TEMP/gita/geographiclib/BUILD

cd $TEMP/gita/geographiclib
cmake -S . -B BUILD
(cd BUILD && make dist)
# cp $TEMP/gita/geographiclib/BUILD/distrib/GeographicLib-$DISTVERSION.{zip,tar.gz} $DEVELSOURCE/data-distrib/distrib-C++/

echo ==============================================================
echo Unpack source package in $TEMP/rel bcx

mkdir $TEMP/rel{b,c,x}
tar xfpzC BUILD/distrib/GeographicLib-$DISTVERSION.tar.gz $TEMP/relb # Version for autoconf
tar xfpzC BUILD/distrib/GeographicLib-$DISTVERSION.tar.gz $TEMP/relc # Version for cmake+mvn
tar xfpzC BUILD/distrib/GeographicLib-$DISTVERSION.tar.gz $TEMP/relx # for listing

echo ==============================================================
echo Unpack devel cmake distribution in $TEMP/relx and list in $TEMP/files.x
(
    cd $TEMP/relx
    find . -type f | sort -u > ../files.x
)

echo ==============================================================
echo Make a release for Windows testing in $WINDOWSBUILD/GeographicLib-$VERSION
rm -rf $WINDOWSBUILD/GeographicLib-$VERSION

unzip -qq -d $WINDOWSBUILD BUILD/distrib/GeographicLib-$DISTVERSION.zip

cat > $WINDOWSBUILD/GeographicLib-$VERSION/mvn-build <<'EOF'
#! /bin/sh -exv
unset GEOGRAPHICLIB_DATA
# for v in 2019 2017 2015 2013 2012 2010; do
for v in 2019 2017 2015; do
  for a in 64 32; do
    echo ========== maven $v-$a ==========
    rm -rf c:/scratch/geog-mvn-$v-$a
    mvn -Dcmake.compiler=vc$v -Dcmake.arch=$a \
      -Dcmake.project.bin.directory=c:/scratch/geog-mvn-$v-$a install
  done
done
EOF
chmod +x $WINDOWSBUILD/GeographicLib-$VERSION/mvn-build
cp pom.xml $WINDOWSBUILD/GeographicLib-$VERSION/

# for ver in 10 11 12 14 15 16; do
for ver in 14 15 16 17; do
    for arch in win32 x64; do
        pkg=vc$ver-$arch
        gen="Visual Studio $ver"
        installer=
        # N.B. update CPACK_NSIS_INSTALL_ROOT in CMakeLists.txt and
        # update documentation examples if VS version for binary
        # installer changes.
        test "$ver" = 14 && installer=y
        (
            echo "#! /bin/sh -exv"
            echo echo ========== cmake $pkg ==========
            echo b=c:/scratch/geog-$pkg
            echo rm -rf \$b //datalake-pr-smb/vt-open/ckarney/pkg-$pkg/GeographicLib-$VERSION/\*
            echo 'unset GEOGRAPHICLIB_DATA'
            echo cmake -G \"$gen\" -A $arch -D BUILD_BOTH_LIBS=ON -D CMAKE_INSTALL_PREFIX=//datalake-pr-smb/vt-open/ckarney/pkg-$pkg/GeographicLib-$VERSION -D PACKAGE_DEBUG_LIBS=ON -D CONVERT_WARNINGS_TO_ERRORS=ON -D EXAMPLEDIR= -S . -B \$b
            echo cmake --build \$b --config Debug   --target ALL_BUILD
            echo cmake --build \$b --config Debug   --target RUN_TESTS
            echo cmake --build \$b --config Debug   --target INSTALL
            echo cmake --build \$b --config Release --target ALL_BUILD
            echo cmake --build \$b --config Release --target exampleprograms
            echo cmake --build \$b --config Release --target experimental
            echo cmake --build \$b --config Release --target RUN_TESTS
            echo cmake --build \$b --config Release --target INSTALL
            echo cmake --build \$b --config Release --target PACKAGE
            test "$installer" &&
                echo cp \$b/GeographicLib-$DISTVERSION-*.exe $WINDEVELSOURCE/ ||
                    true
        ) > $WINDOWSBUILD/GeographicLib-$VERSION/build-$pkg
        chmod +x $WINDOWSBUILD/GeographicLib-$VERSION/build-$pkg
    done
done
cat > $WINDOWSBUILD/GeographicLib-$VERSION/test-all <<'EOF'
#! /bin/sh
(
    for d in build-*; do
        ./$d
    done
    ./mvn-build
) >& build.log
EOF
chmod +x $WINDOWSBUILD/GeographicLib-$VERSION/test-all

echo ==============================================================
echo Set up release branch in $TEMP/gitr/geographiclib

cd $TEMP/gitr/geographiclib
git checkout release
git config user.email karney@alum.mit.edu
git ls-files | sort > ../files.old
(
    cd $TEMP/gitb/geographiclib
    git ls-files | sort
) > ../files.current
cut -f3- -d/ $TEMP/files.x | sort > ../files.new
(
    cd ..
    comm -23 files.old files.new | grep -v gitattributes > files.delete || true
    comm -23 files.new files.current > files.relonly
    comm -12 files.new files.current > files.main
    comm -13 files.delete files.new > files.add
    comm -13 files.old files.new > files.add
    comm -12 files.old files.new > files.common
)

(
    S=$TEMP/relx/GeographicLib-$VERSION
    C=$TEMP/gitb/geographiclib
    test -s ../files.delete && xargs git rm -f < ../files.delete
    cat ../files.main ../files.relonly |
        sed -e 's%[^/][^/]*$%%' -e 's%/$%%' | sort -u | grep -v '^$' |
        while read d; do
            test -d $d || mkdir -p $d
        done
    while read f; do
        if cmp $S/$f $f >& /dev/null; then :
        else
            cp -p $S/$f $f
            git add $f
        fi
    done < ../files.relonly
    while read f; do
        test -d `dirname $f` || mkdir `dirname $f`
        git checkout $BRANCH $f
        if cmp $S/$f $f >& /dev/null; then :
        else
            cp -p $S/$f $f
            git add $f
        fi
    done < ../files.main
    git checkout $BRANCH .gitattributes
    for i in 1 2 3 4 5; do
        find -type d -empty | xargs -r rmdir
    done
)
rm -rf GeographicLib-$VERSION
rm -f java/.gitignore
for ((i=0; i<7; ++i)); do
    find * -type d -empty | xargs -r rmdir
done

echo ==============================================================
echo CMake build in $TEMP/relc/GeographicLib-$VERSION/BUILD install to $TEMP/instc
cd $TEMP/relc/GeographicLib-$VERSION
cmake -D BUILD_BOTH_LIBS=ON -D BUILD_DOCUMENTATION=ON -D USE_BOOST_FOR_EXAMPLES=ON -D CONVERT_WARNINGS_TO_ERRORS=ON -D CMAKE_INSTALL_PREFIX=$TEMP/instc -S . -B BUILD
(
    cd BUILD
    make package_source
    make -j$NUMCPUS all
    make test
    make exampleprograms
    make -j$NUMCPUS experimental
    make install
    # rsync -a --delete doc/html/ $WEBDIST/htdocs/C++/$VERSION/
)

echo ==============================================================
echo List installed files fron cmake build in $TEMP/instc to $TEMP/files.c
(
    cd $TEMP/instc
    find . -type f | sort -u > ../files.c
)

echo ==============================================================
echo Make distribution from release tree with cmake

cd $TEMP/relc/GeographicLib-$VERSION
cmake -S . -B BUILD-dist
(cd BUILD-dist && make package_source)

echo ==============================================================
echo Unpack release cmake distribution in $TEMP/relz and list in $TEMP/files.z
mkdir $TEMP/relz
tar xfpzC BUILD-dist/GeographicLib-$VERSION.tar.gz $TEMP/relz

(
    cd $TEMP/relz
    find . -type f | sort -u > ../files.z
)

echo ==============================================================
echo CMake build in $TEMP/relc/GeographicLib-$VERSION/BUILD-system install to /usr/local

cmake -D BUILD_BOTH_LIBS=ON -D CONVERT_WARNINGS_TO_ERRORS=ON -S . -B BUILD-system
(cd BUILD-system && make -j$NUMCPUS all && make test)

if test "$HAVEINTEL"; then
    echo ==============================================================
    echo CMake build for intel in $TEMP/relc/GeographicLib-$VERSION/BUILD-intel
    env FC=ifort CC=icc CXX=icpc cmake -D BUILD_BOTH_LIBS=ON -D CONVERT_WARNINGS_TO_ERRORS=ON -S . -B BUILD-intel
    (
        cd BUILD-intel
        make -j$NUMCPUS all
        make test
        make exampleprograms
        make -j$NUMCPUS experimental
    )
fi

echo ==============================================================
echo Check build with configure in $TEMP/relb/GeographicLib-$VERSION/BUILD-config to $TEMP/instb

cd $TEMP/relb/GeographicLib-$VERSION
mkdir BUILD-config
cd BUILD-config
../configure --prefix=$TEMP/instb
make -j$NUMCPUS
make install
cd ..

if test "$HAVEINTEL"; then
echo ==============================================================
echo Check build with configure + intell in $TEMP/relb/GeographicLib-$VERSION/BUILD-config-intel

    mkdir BUILD-config-intel
    cd BUILD-config-intel
    env FC=ifort CC=icc CXX=icpc ../configure
    make -j$NUMCPUS
    cd ..
fi

echo ==============================================================
echo Make source dist with autoconf $TEMP/relb/GeographicLib-$VERSION/BUILD-dist

mkdir BUILD-dist
cd BUILD-dist
../configure
make dist-gzip

echo ==============================================================
echo Unpack release autoconf distribution in $TEMP/rely and list in $TEMP/files.y

mkdir $TEMP/rely
tar xfpzC geographiclib-$VERSION.tar.gz $TEMP/rely
mv $TEMP/rely/{geographiclib,GeographicLib}-$VERSION
cd $TEMP/rely
find . -type f | sort -u > ../files.y

echo ==============================================================
echo List installed files fron autoconf build in $TEMP/instb to $TEMP/files.b

mv $TEMP/instb/share/doc/{geographiclib,GeographicLib}
cd $TEMP/instb
find . -type f | sort -u > ../files.b

echo ==============================================================
echo CMake build of devel tree in $TEMP/gitb/geographiclib/BUILD

cd $TEMP/gitb/geographiclib
cmake -D BUILD_BOTH_LIBS=ON -D BUILD_DOCUMENTATION=ON -D USE_BOOST_FOR_EXAMPLES=ON -D CONVERT_WARNINGS_TO_ERRORS=ON -S . -B BUILD
(cd BUILD && make -j$NUMCPUS && make -j$NUMCPUS develprograms)

cp $DEVELSOURCE/include/mpreal.h include/
# Skip 4 for now because of various boost bugs
for p in 1 3 5; do
    echo ==============================================================
    echo CMake build of devel tree at precision $p in $TEMP/gitb/geographiclib/BUILD-$p
    mkdir BUILD-$p
    cmake -D USE_BOOST_FOR_EXAMPLES=ON -D GEOGRAPHICLIB_PRECISION=$p -S . -B BUILD-$p
    (
        cd BUILD-$p
        make -j$NUMCPUS all
        if test $p -ne 1; then
            make test
        fi
        make -j$NUMCPUS develprograms
        make exampleprograms
        make -j$NUMCPUS experimental
    )
done

echo ==============================================================
echo Compile and run little test program
cd $TEMP
cat > testprogram.cpp <<EOF
#include <iostream>
#include <iomanip>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/LambertConformalConic.hpp>

int main() {
  using namespace GeographicLib;
  double
    // These are the constants for Pennsylvania South, EPSG:3364
    // https://www.spatialreference.org/ref/epsg/3364/
    a = Constants::WGS84_a(),   // major radius
    f = 1/298.257222101,        // inverse flattening (GRS80)
    lat1 = DMS::Decode(40,58),  // standard parallel 1
    lat2 = DMS::Decode(39,56),  // standard parallel 2
    k1 = 1,                     // scale on std parallels
    lat0 =  DMS::Decode(39,20), // latitude of origin
    lon0 = -DMS::Decode(77,45), // longitude of origin
    fe = 600000,                // false easting
    fn = 0;                     // false northing
  LambertConformalConic PASouth(a, f, lat1, lat2, k1);
  double x0, y0;
  PASouth.Forward(lon0, lat0, lon0, x0, y0); // Transform origin point
  x0 -= fe; y0 -= fn;           // Combine result with false origin

  double lat = 39.95, lon = -75.17;    // Philadelphia
  double x, y;
  PASouth.Forward(lon0, lat, lon, x, y);
  x -= x0; y -= y0;             // Philadelphia in PA South coordinates

  std::cout << std::fixed << std::setprecision(3)
            << x << " " << y << "\n";
  return 0;
}
EOF
for i in  b c; do
    cp testprogram.cpp testprogram$i.cpp
    g++ -c -g -O3 -I$TEMP/inst$i/include testprogram$i.cpp
    g++ -g -o testprogram$i testprogram$i.o -Wl,-rpath=$TEMP/inst$i/lib \
        -L$TEMP/inst$i/lib -lGeographicLib
    ./testprogram$i
done

echo ==============================================================
echo Verify library versions of cmake and autoconf builds are the same and other checks

libversion=`find $TEMP/instc/lib -type f \
-name 'libGeographicLib.so.*.*' -printf "%f" |
sed 's/libGeographicLib\.so\.//'`
test -f $TEMP/instb/lib/libGeographicLib.so.$libversion ||
echo autoconf/cmake library so mismatch

CONFIG_FILE=$TEMP/gitr/geographiclib/configure
CONFIG_MAJOR=`grep ^GEOGRAPHICLIB_VERSION_MAJOR= $CONFIG_FILE | cut -f2 -d=`
CONFIG_MINOR=`grep ^GEOGRAPHICLIB_VERSION_MINOR= $CONFIG_FILE | cut -f2 -d=`
CONFIG_PATCH=`grep ^GEOGRAPHICLIB_VERSION_PATCH= $CONFIG_FILE | cut -f2 -d=`
CONFIG_VERSIONA=`grep ^PACKAGE_VERSION= $CONFIG_FILE | cut -f2 -d= |
cut -f2 -d\'`
CONFIG_VERSION=$CONFIG_MAJOR.$CONFIG_MINOR
test "$CONFIG_PATCH" = 0 || CONFIG_VERSION=$CONFIG_VERSION.$CONFIG_PATCH
test "$CONFIG_VERSION"  = "$VERSION" || echo autoconf version number mismatch
test "$CONFIG_VERSIONA" = "$VERSION" || echo autoconf version string mismatch

cd $TEMP/relx/GeographicLib-$VERSION
(
    echo Files with trailing spaces:
    find . -type f | grep -E -v 'config\.guess|Makefile\.in|\.m4|\.png|\.gif' |
        while read f; do
            tr -d '\r' < $f | grep ' $' > /dev/null && echo $f || true
        done
    echo
    echo Files with tabs:
    find . -type f |
        grep -E -v '[Mm]akefile|\.html|\.m4|\.png|\.gif' |
        grep -E -v \
             '\.sh|depcomp|install-sh|/config\.|configure$|compile|missing' |
        xargs grep -l  '	' || true
    echo
    echo Files with multiple newlines:
    find . -type f |
        grep -E -v \
           '/Makefile\.in|\.1\.html|\.png|\.gif|/ltmain|/config|\.m4' |
        while read f; do
            tr 'X\n' 'xX' < $f | grep XXX > /dev/null && echo $f || true
        done
    echo
    echo Files with no newline at end:
    find . -type f |
        grep -E -v '\.png|\.gif' |
        while read f; do
            n=`tail -1 $f | wc -l`; test $n -eq 0 && echo $f || true
        done
    echo
    echo Files with extra newlines at end:
    find . -type f |
        grep -E -v '/configure|/ltmain.sh|\.png|\.gif|\.1\.html' |
        while read f; do
            n=`tail -1 $f | wc -w`; test $n -eq 0 && echo $f || true
        done
    echo
) > $TEMP/badfiles.txt
cat $TEMP/badfiles.txt
cat > $TEMP/tasks.txt <<EOF
# install built version
sudo make -C $TEMP/relc/GeographicLib-$VERSION/BUILD-system install

# commit and tag release branch
cd $TEMP/gitr/geographiclib
# Check .gitignore files!
git add -A
git commit -m "Version $VERSION ($DATE)"
git tag -m "Version $VERSION ($DATE)" r$VERSION
git push
git push --tags

# tag main branch
cd $DEVELSOURCE
git tag -m "Version $VERSION ($DATE)" v$VERSION
git push --all
git push --tags

# Also to do
# post release notices
# set default download files
# update home brew
#   dir = /usr/local/Homebrew/Library/Taps/homebrew/homebrew-core
#   branch = geographiclib/$VERSION
#   file = Formula/geographiclib.rb
#   brew install --build-from-source geographiclib
#   commit message = geographiclib $VERSION
# update vcpkg git@github.com:microsoft/vcpkg
#   dir = ports/geographiclib
#   ./vcpkg install 'geographiclib[tools]'
#   binaries in installed/x64-linux/tools/geographiclib
#   libs in installed/x64-linux/{include,lib,debug/lib}
#   ./vcpkg x-add-version geographiclib
#   commit message = [geographiclib] Update to version $VERSION
# update conda-forge
#   url = git@github.com:conda-forge/geographiclib-cpp-feedstock
#   conda build recipe
# upload matlab packages
# update binaries for cgi applications
# trigger build on build-open
EOF
echo cat $TEMP/tasks.txt
cat $TEMP/tasks.txt
END=`date +%s`
echo Elapsed time $((END-START)) secs
