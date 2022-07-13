#! /bin/bash -e
# Copyright: None
# --------------------------------------------------------------------------
# tarball='source-highlight-3.1.5.tar.gz'
tarball='source-highlight-3.1.8.tar.gz'
web_page='ftp://ftp.gnu.org/gnu/src-highlite/'
start_dir=`pwd`
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
name='highlight'
if [ "$0" != "bin/get_$name.sh" ]
then
    echo "get_$name.sh should be in the ./bin directory and executed using"
    echo "bin/get_$name.sh"
    exit 1
fi
# -----------------------------------------------------------------------------
# n_proc
if which nproc >& /dev/null
then
    n_job=$(nproc)
else
    n_job=$(sysctl -n hw.ncpu)
fi
# -----------------------------------------------------------------------------
if [ ! -e build ]
then
    mkdir build
fi
cd build
# -----------------------------------------------------------------------------
dir=`echo $tarball | sed -e 's|\.tar||' -e 's|\.gz||'`
if [ ! -e $dir ]
then
    echo_eval wget "$web_page/$tarball"
    echo_eval tar -xzf $tarball
    rm $tarball
fi
# -----------------------------------------------------------------------------
echo_eval cd $dir
if [ ! -e build ]
then
    mkdir build
fi
echo_eval cd build
#
echo_eval ../configure --prefix="$start_dir/build/prefix"
echo_eval make -j $n_job install
# -----------------------------------------------------------------------------
echo "get_$name.sh: OK"
exit 1
