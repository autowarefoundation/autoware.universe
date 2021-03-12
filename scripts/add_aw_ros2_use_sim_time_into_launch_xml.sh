#!/bin/bash

if [ -n "$1" ]; then
	find_dir=$1
else
	echo "Usage: $0 <dir ('install' dir is recommended)>"
	exit 0
fi

patch_script=$(dirname $(realpath $0))/$(basename $0 .sh).pl

for xml in $(find $find_dir -type f | grep launch.xml$); do
	echo "mv $xml ${xml}.org_tmp"
	eval "mv $xml ${xml}.org_tmp"
	echo "$patch_script ${xml}.org_tmp > $xml"
	eval "$patch_script ${xml}.org_tmp > $xml"
	echo "rm ${xml}.org_tmp"
	eval "rm ${xml}.org_tmp"
	echo ""
done

