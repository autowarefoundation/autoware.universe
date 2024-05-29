# Look up a key in a QUERY_STRING and return raw value
lookuprawkey () {
    QUERY="$1"
    KEY="$2"
    echo "$QUERY" | tr '&' '\n' | grep "^$KEY=" | tail -1 | cut -f2- -d=
}

# Decode raw value translating + to space, changing CR-LF to LF,
# interpreting %XX, converting "," and TAB to spaces, and squeezing
# and trimming spaces.
decodevalue () {
    echo "$1" | tr -s '+' ' ' | sed \
        -e 's/\\/%5C/g' \
        -e 's/%0[dD]%0[aA]/%0A/g' \
        -e 's/%\([0-9a-fA-F][0-9a-fA-F]\)/\\x\1/g' -e s/%/%%/g |
    xargs -d '\n' printf | tr -s ',\t' ' ' | sed -e 's/^ //' -e 's/ $//'
}

# Apply conversions for the various degree, minute, and second
# symbols, following conversions in DMS.cpp.  Add left/right guillemot
# symbols (used to quote examples) to the list of removed symbols.

# Translate UTF-8 and &#nnn; sequences first, then single character
# replacements, then ' ' -> ".  This is the same as the order in
# DMS.cpp except for the addition of the &#nnn; sequences and the
# removal of left/right guillemots.
translate () {
    echo "$1" |
        sed \
            -e 's/%C2%B0/d/g'      -e 's/%26%23176%3B/d/g'     \
            -e 's/%C2%BA/d/g'      -e 's/%26%23186%3B/d/g'     \
            -e 's/%E2%81%B0/d/g'   -e 's/%26%238304%3B/d/g'    \
            -e 's/%CB%9A/d/g'      -e 's/%26%23730%3B/d/g'     \
            -e 's/%E2%88%98/d/g'   -e 's/%26%238728%3B/d/g'    \
                                                               \
            -e 's/%E2%80%B2/%27/g' -e 's/%26%238242%3B/%27/g'  \
            -e 's/%E2%80%B5/%27/g' -e 's/%26%238245%3B/%27/g'  \
            -e 's/%C2%B4/%27/g'    -e 's/%26%23180%3B/%27/g'   \
            -e 's/%E2%80%98/%27/g' -e 's/%26%238216%3B/%27/g'  \
            -e 's/%E2%80%99/%27/g' -e 's/%26%238217%3B/%27/g'  \
            -e 's/%E2%80%9B/%27/g' -e 's/%26%238219%3B/%27/g'  \
            -e 's/%CA%B9/%27/g'    -e 's/%26%23697%3B/%27/g'   \
            -e 's/%CB%8A/%27/g'    -e 's/%26%23714%3B/%27/g'   \
            -e 's/%CB%8B/%27/g'    -e 's/%26%23715%3B/%27/g'   \
                                                               \
            -e 's/%E2%80%B3/%22/g' -e 's/%26%238243%3B/%22/g'  \
            -e 's/%E2%80%B6/%22/g' -e 's/%26%238246%3B/%22/g'  \
            -e 's/%CB%9D/%22/g'    -e 's/%26%23733%3B/%22/g'   \
            -e 's/%E2%80%9C/%22/g' -e 's/%26%238220%3B/%22/g'  \
            -e 's/%E2%80%9D/%22/g' -e 's/%26%238221%3B/%22/g'  \
            -e 's/%E2%80%9F/%22/g' -e 's/%26%238223%3B/%22/g'  \
            -e 's/%CA%BA/%22/g'    -e 's/%26%23698%3B/%22/g'   \
                                                               \
            -e 's/%E2%9E%95/%2B/g' -e 's/%26%2310133%3B/%2B/g' \
            -e 's/%E2%81%A4/%2B/g' -e 's/%26%238292%3B/%2B/g'  \
                                                               \
            -e 's/%E2%80%90/-/g'   -e 's/%26%238208%3B/-/g'    \
            -e 's/%E2%80%91/-/g'   -e 's/%26%238209%3B/-/g'    \
            -e 's/%E2%80%93/-/g'   -e 's/%26%238211%3B/-/g'    \
            -e 's/%E2%80%94/-/g'   -e 's/%26%238212%3B/-/g'    \
            -e 's/%E2%88%92/-/g'   -e 's/%26%238722%3B/-/g'    \
            -e 's/%E2%9E%96/-/g'   -e 's/%26%2310134%3B/-/g'   \
                                                               \
            -e 's/%C2%A0//g'       -e 's/%26%23160%3B//g'      \
            -e 's/%E2%80%87//g'    -e 's/%26%238199%3B//g'     \
            -e 's/%E2%80%89//g'    -e 's/%26%238201%3B//g'     \
            -e 's/%E2%80%8A//g'    -e 's/%26%238202%3B//g'     \
            -e 's/%E2%80%8B//g'    -e 's/%26%238203%3B//g'     \
            -e 's/%E2%80%AF//g'    -e 's/%26%238239%3B//g'     \
            -e 's/%C2%AB//g'       -e 's/%26%23171%3B//g'      \
            -e 's/%C2%BB//g'       -e 's/%26%23187%3B//g'      \
            -e 's/%E2%81%A3//g'    -e 's/%26%238291%3B//g'     \
                             \
            -e 's/%B0/d/g'   \
            -e 's/%BA/d/g'   \
            -e 's/%2A/d/g'     -e 's/\*/d/g'  \
            -e 's/%60/%27/g'   -e 's/`/%27/g' \
            -e 's/%B4/%27/g' \
            -e 's/%91/%27/g' \
            -e 's/%92/%27/g' \
            -e 's/%93/%22/g' \
            -e 's/%94/%22/g' \
            -e 's/%96/-/g'   \
            -e 's/%97/-/g'   \
            -e 's/%A0//g'    \
            -e 's/%AB//g'    \
            -e 's/%BB//g'    \
                             \
            -e 's/%27%27/%22/g'
}

# Look up and decode a key
lookupkey () {
    decodevalue `lookuprawkey "$1" "$2"`
}

# Look up, translate, and decode a key.  If result has unprintable
# characters, log the raw value.
lookupcheckkey () {
    RAWVAL=`lookuprawkey "$1" "$2"`
    VALUE=`translate "$RAWVAL"`
    VALUE=`decodevalue "$VALUE"`
    test `echo "$VALUE" | tr -d '[ -~\n\t]' | wc -c` -ne 0 &&
    echo `date +"%F %T"` Unprintable "$RAWVAL" >> ../persistent/utilities.log
    echo "$VALUE"
}

# Look up ellipsoid parameter leaving only allowed characters (--/ -> -, ., /)
lookupellipsoid () {
    VALUE=`lookuprawkey "$1" "$2"`
    VALUE=`echo "$VALUE" | sed -e 's/%26%238722%3B/-/g'`
    VALUE=`decodevalue "$VALUE"`
    VALUE=`echo "$VALUE" | tr -cd '[0-9--/Ee]'`
    echo "$VALUE"
}

# Encode a string for inclusion into HTML.
encodevalue () {
    echo "$1" | sed -e 's/&/\&amp;/g' -e 's/"/\&quot;/g' \
        -e 's/>/\&gt;/g' -e 's/</\&lt;/g' -e "s/'/\&#39;/g" -e 's/`/\&#96;/g'
}

# Encode and convert d to &deg;
convertdeg () {
    encodevalue "$1" | sed -e 's/d/\&deg;/g'
}

# Generate GeoHack URL.  $1 $2 are real position; $3 $4 is displayed
# postion; $5 is link color
geohack () {
    echo "<a href=\"http://tools.wmflabs.org/geohack/geohack.php?params=$1;$2\" style=\"color:$5\">$(convertdeg "$3 $4")</a>"
}
