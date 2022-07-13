#! /bin/bash -e
# Copyright: None
start_dir=`pwd`
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
if ! which omhelp >& /dev/null
then
    if [ -e build/prefix/bin/omhelp ]
    then
        PATH="$PATH:$start_dir/build/prefix/bin"
    fi
fi
if ! which omhelp 
then
    echo 'run_omhel.sh: cannot find omhelp'
    echo 'perhaps you need to execute the following commands:'
    echo '  bin/get_highlight.sh'
    echo '  bin/get_omhelp.sh'
    exit 1
fi
# -----------------------------------------------------------------------------
# command line arguments
cat << EOF > run_omhelp.$$
usage: run_omhelp.sh [-clean] [-printable] [-xml] root
where ./root.omh is the top level (root) omhelp file.
The output is written in the ./root directory.
EOF
clean='no'
printable=''
xml=''
root=''
while [ "$1" != '' ]
do
    case $1 in
    
        -clean)
        clean='yes'
        shift
        ;;
    
        -printable)
        printable='-printable'
        shift
        ;;
    
        -xml)
        xml='-xml'
        shift
        ;;
    
        *)
        root="$1"
        shift
        if [ ! -f "$root".omh ] || [ "$1" != '' ]
        then
            cat run_omhelp.$$
            rm run_omhelp.$$
            exit 1
        fi
        ;;
    
    esac
done
#
if [ "$root" == '' ]
then
    cat run_omhelp.$$
    rm run_omhelp.$$
    exit 1
fi
rm run_omhelp.$$
# -----------------------------------------------------------------------------
for file in bin/devel.sh $root.omh
do
    if [ ! -e "$file" ]
    then
        echo "run_omhelp.sh: cannot find the file: $file"
        exit 1
    fi
done
# -----------------------------------------------------------------------------
# get image_link
if ! grep "^image_link='[^']*'$" bin/devel.sh > /dev/null
then
    echo 'run_omhelp.sh cannot find the following pattern in bin/devel.sh' 
    echo "^image_link='[^']*'$"
    exit 1
fi
#
image_link=`grep '^image_link=' bin/devel.sh | sed \
    -e "s|^image_link='||" -e "s|'$||"`
echo "image_link ='$image_link'"
if [ "$image_link" != '' ]
then
    image_link="-image_link $image_link"
fi
# -----------------------------------------------------------------------------
if [ "$clean" == 'yes' ]
then
    if [ -d $root ]
    then
        echo_eval rm -r $root
    fi
fi
# -----------------------------------------------------------------------------
if [ ! -d "$root" ]
then
    echo_eval mkdir $root
fi
echo_eval cd $root
if omhelp \
    "../$root.omh" \
    $printable \
    $xml \
    $image_link \
    -noframe \
    -debug \
    > ../omhelp.$root.log
then
    omhelp_error='no'
else
    omhelp_error='yes'
fi
cd ..
# -----------------------------------------------------------------------------
if [ "$omhelp_error" == 'yes' ]
then
    cat omhelp.$root.log
    echo "OMhelp could not build $root."
    echo "See the complete error message in omhelp.$root.log."
    grep "^OMhelp Error:" omhelp.$root.log
    exit 1
fi
if grep "^OMhelp Warning:" omhelp.$root.log
then
    echo "See the complete warning messages in omhelp.$root.log."
    exit 1
fi
# -----------------------------------------------------------------------------
echo 'run_omhelp.sh OK'
exit 0
