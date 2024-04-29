#! /bin/sh
#
# GeoConvert.cgi
# cgi script for geographic coordinate conversion
#
# Copyright (c) Charles Karney (2011-2022) <karney@alum.mit.edu> and
# licensed under the MIT/X11 License.  For more information, see
# https://geographiclib.sourceforge.io/

. ./utils.sh
OPTION=`lookupkey "$QUERY_STRING" option`
if test "$OPTION" = Reset; then
    INPUT=
else
    INPUT=`lookupcheckkey "$QUERY_STRING" input`
    ZONE=`lookupkey "$QUERY_STRING" zone`
    PREC=`lookupkey "$QUERY_STRING" prec`
fi
test "$ZONE" || ZONE=-3
test "$PREC" || PREC=0
INPUTENC=`encodevalue "$INPUT"`
EXECDIR=../bin
COMMAND="GeoConvert"
VERSION=`$EXECDIR/$COMMAND --version | cut -f4 -d" "`
F='<font color="Blue">'
G='</font>'
test $PREC = 0 || COMMAND="$COMMAND -p $PREC"
LOCG=
LOCD=
LOCU=
LOCM=
GEOH=
set -o pipefail
if test "$INPUT"; then
    LOCG=`echo $INPUT | $EXECDIR/$COMMAND | head -1`
    if test $? -eq 0; then
        GEOH=`echo $INPUT | $EXECDIR/$COMMAND -p 1 | head -1`
        LOCG=`geohack $GEOH $LOCG Blue`
        LOCD=\(`echo $INPUT | $EXECDIR/$COMMAND -d | head -1`\)
        case $ZONE in
            -3 ) ;;
            -2 ) COMMAND="$COMMAND -t";;
            -1 ) COMMAND="$COMMAND -s";;
            * ) COMMAND="$COMMAND -z $ZONE"
        esac
        LOCU=`echo $INPUT | $EXECDIR/$COMMAND -u | head -1`
        LOCM=`echo $INPUT | $EXECDIR/$COMMAND -m | head -1`
    fi
    # echo `date +"%F %T"` "$COMMAND: $INPUT" >> ../persistent/utilities.log
fi

echo Content-type: text/html
echo
cat <<EOF
<html>
  <head>
    <title>
      Online geographic coordinate converter
    </title>
    <meta name="description"
          content="Online geographic coordinate converter" />
    <meta name="author" content="Charles F. F. Karney" />
    <meta name="keywords"
          content="geographic coordinate conversions,
                   geographic coordinates,
                   latitude and longitude,
                   degrees minutes seconds,
                   universal transverse Mercator, UTM,
                   easting northing and zone,
                   universal polar sterographic, UPS,
                   military grid reference system, MGRS,
                   online calculator,
                   WGS84 ellipsoid,
                   GeographicLib" />
  </head>
  <body>
    <h3>
      Online geographic coordinate conversions using the
      <a href="https://geographiclib.sourceforge.io/C++/doc/GeoConvert.1.html">
        GeoConvert</a> utility
    </h3>
    <form action="/cgi-bin/GeoConvert" method="get">
      <p>
        Location
        (ex. &laquo;<tt>33.33 44.4</tt>&raquo;,
        &laquo;<tt>33d19'47"N 44d23.9'E</tt>&raquo;,
        &laquo;<tt>38SMB4488</tt>&raquo;,
        &laquo;<tt>38n 444000 3688000</tt>&raquo;):<br>
        &nbsp;&nbsp;&nbsp;
        <input type=text name="input" size=40 value="$INPUTENC">
      </p>
      <table>
        <tr>
          <td>
            &nbsp;&nbsp;&nbsp;
            Output zone:<br>
            &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
            <select name="zone" size=1>
EOF
(
    cat <<EOF
-3 Match input or standard
-1 Standard UPS/UTM zone
0 UPS
-2 Standard UTM zone
EOF
) | while read z name; do
    SELECTED=
    test "$z" = "$ZONE" && SELECTED=SELECTED
    echo "<option $SELECTED value=\"$z\">$name</option>"
done
for ((z=1; z<=60; ++z)); do
    SELECTED=
    test "$z" = "$ZONE" && SELECTED=SELECTED
    name="UTM zone $z"
    echo "<option $SELECTED value=\"$z\">$name</option>"
done
cat <<EOF
            </select>
          </td>
          <td>
            &nbsp;&nbsp;&nbsp;
            Output precision:<br>
            &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
            <select name="prec" size=1>
EOF
(
    cat <<EOF
-5 100km 1d
-4 10km 0.1d
-3 1km 0.01d 1'
-2 100m 0.001d 0.1'
-1 10m 0.0001d 1"
0 1m 0.00001d 0.1"
1 100mm 0.01"
2 10mm 0.001"
3 1mm 0.0001"
4 100um 0.00001"
5 10um 0.000001"
6 1um 0.0000001"
7 100nm 0.00000001"
8 10nm 0.000000001"
9 1nm 0.0000000001"
10 0.00000000001"
EOF
) | while read p desc; do
    SELECTED=
    test "$p" = "$PREC" && SELECTED=SELECTED
    echo "<option $SELECTED value=\"$p\">$desc</option>"
done
cat <<EOF
            </select>
          </td>
        </tr>
      </table>
      <p>
        Select action:<br>
        &nbsp;&nbsp;&nbsp;
        <input type="submit" name="option" value="Submit">
        <input type="submit" name="option" value="Reset">
      </p>
      <p>
        Results:
<font size="4"><pre>
    input   = `encodevalue "$INPUT"`
    lat lon = $F$LOCG `convertdeg "$LOCD"`$G
    <a href="https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system">UTM</a>/<a href="https://en.wikipedia.org/wiki/Universal_Polar_Stereographic">UPS</a> = $F`encodevalue "$LOCU"`$G
    <a href="https://en.wikipedia.org/wiki/Military_grid_reference_system">MGRS</a>    = $F`encodevalue "$LOCM"`$G</pre></font>
      </p>
    </form>
    <hr>
    <p>
      <a href="https://geographiclib.sourceforge.io/C++/doc/GeoConvert.1.html">
        GeoConvert (version $VERSION)</a>
      converts between geographic (latitude and longitude) coordinates,
      <a href="https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system">
        universal transverse Mercator (UTM)</a> or
      <a href="https://en.wikipedia.org/wiki/Universal_Polar_Stereographic">
        universal polar stereographic (UPS)</a> coordinates, and the
      <a href="https://en.wikipedia.org/wiki/Military_grid_reference_system">
        military grid reference system (MGRS)</a>.
      Examples of legal geographic locations are (these all refer to roughly
      the same place, Cape Morris Jesup on the northern tip of Greenland):
      <pre>
    Latitude and longitude:       MGRS:
        83.627 -32.664                24XWT783908
        W32d40 N83d37.6               YUB17770380
        83&deg;37'39"N 32&deg;39'52"W     UTM:
        83:37:39 32:39:52W            25n 504158 9286521
        32:39.9W 83:37.6N             430000 9290000 26n
        32:40W+0:0:6 83:37.6      UPS:
        32:40W+0:0:6 83:38-0:0.4      n 1617772 1403805</pre>
      <b>Notes:</b>
      <ul>
        <li>
          The letter in following the zone number in the UTM position is
          a hemisphere designator (n or s) and <em>not</em> the MGRS
          latitude band letter.  "north" or "south" can also be used,
          e.g., 25north.
        <li>
          MGRS coordinates are taken to refer to <em>grid squares</em>
          (<em>not</em> to the intersections of grid lines).  Thus in UTM
          zone 38n, the square area with easting in [444 km, 445 km) and
          northing in [3688 km, 3689 km) corresponds to the MGRS square
          38SMB4488 (at 1 km precision).
          <ul>
            <li>
              When an MGRS coordinate is read, a representative point
              within the grid square, namely, the <em>center</em>, is
              returned.
            <li>
              The MGRS easting and northing are obtained
              by <em>truncation</em> to the requested precision
              (<em>not</em> rounding).
          </ul>
        <li>
          Usually <em>Output zone</em> should be <em>Match input or
          standard</em>.  If the latitude and longitude are given, the
          standard UPS and UTM zone rules are applied; otherwise the
          UPS/UTM selection and the UTM zone match the input.  The other
          choices let you override this selection.
      </ul>
    </p>
    <p>
      <a href="https://geographiclib.sourceforge.io/C++/doc/GeoConvert.1.html">
        GeoConvert</a>,
      which is a simple wrapper of the
      <a href="https://geographiclib.sourceforge.io/C++/doc/classGeographicLib_1_1GeoCoords.html">
        GeographicLib::GeoCoords</a> class,
      is one of the utilities provided
      with <a href="https://geographiclib.sourceforge.io/">
        GeographicLib</a>.
      This web interface illustrates a subset of its capabilities.  If
      you wish to use GeoConvert directly,
      <a href="https://sourceforge.net/projects/geographiclib/files/distrib-C++">
        download</a>
      and compile GeographicLib.
    </p>
    <hr>
    <address>Charles Karney
      <a href="mailto:karney@alum.mit.edu">&lt;karney@alum.mit.edu&gt;</a>
      (2022-04-10)</address>
    <a href="https://geographiclib.sourceforge.io">
      GeographicLib home
    </a>
  </body>
</html>
EOF
