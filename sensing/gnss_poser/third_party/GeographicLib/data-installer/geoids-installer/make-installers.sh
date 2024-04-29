#! /bin/sh
# Run Inno Setup Compiler to create installers for geoid datasets

INNO="c:/Program Files/Inno Setup 5/ISCC.exe"
test -f "$INNO" || INNO="c:/Program Files (x86)/Inno Setup 5/ISCC.exe"

GEOIDDIR=/usr/local/share/GeographicLib
test -d "$GEOIDDIR"/installers || mkdir -p "$GEOIDDIR"/installers
GEOIDDIR=`cygpath -w $GEOIDDIR`
(
cat <<EOF
egm84-30    EGM84 30'
egm84-15    EGM84 15'
egm96-15    EGM96 15'
egm96-5     EGM96 5'
egm2008-5   EGM2008 5'
egm2008-2_5 EGM2008 2.5'
egm2008-1   EGM2008 1'
EOF
) | while read prefix name; do
    "$INNO" geoid-installers.iss \
        /dGEOIDDIR="$GEOIDDIR" \
        /dPREFIX="$prefix" \
        /dNAME="$name" > $prefix.log 2>&1
done
exit 0
