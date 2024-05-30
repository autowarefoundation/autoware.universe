#! /bin/sh
#
# GeodSolve.cgi
# cgi script for geodesic calculations
#
# Copyright (c) Charles Karney (2011-2022) <karney@alum.mit.edu> and
# licensed under the MIT/X11 License.  For more information, see
# https://geographiclib.sourceforge.io/

. ./utils.sh
DEFAULTRADIUS=6378137
DEFAULTFLATTENING=1/298.257223563
OPTION=`lookupkey "$QUERY_STRING" option`
if test "$OPTION" = Reset; then
    INPUT=
    RADIUS=
    FLATTENING=
else
    INPUT=`lookupcheckkey "$QUERY_STRING" input`
    RADIUS=`lookupellipsoid "$QUERY_STRING" radius`
    FLATTENING=`lookupellipsoid "$QUERY_STRING" flattening`
    FORMAT=`lookupkey "$QUERY_STRING" format`
    AZF2=`lookupkey "$QUERY_STRING" azi2`
    UNROLL=`lookupkey "$QUERY_STRING" unroll`
    PREC=`lookupkey "$QUERY_STRING" prec`
    TYPE=`lookupkey "$QUERY_STRING" type`
fi
test "$RADIUS" || RADIUS=$DEFAULTRADIUS
test "$FLATTENING" || FLATTENING=$DEFAULTFLATTENING
test "$FORMAT" || FORMAT=g
test "$AZF2" || AZF2=f
test "$UNROLL" || UNROLL=r
test "$PREC" || PREC=3
test "$TYPE" || TYPE=I
AZIX="fazi2"
test "$AZF2" = b && AZIX="bazi2"
TAG=
if test "$RADIUS" = "$DEFAULTRADIUS" -a \
  "$FLATTENING" = "$DEFAULTFLATTENING"; then
  TAG=" (WGS84)"
fi

INPUTENC=`encodevalue "$INPUT"`
EXECDIR=../bin
COMMAND="GeodSolve -E -f"
VERSION=`$EXECDIR/$COMMAND --version | cut -f4 -d" "`
F='<font color="blue">'
G='</font>'
S='     '
test $TYPE = I && COMMAND="$COMMAND -i"
COMMANDX="$COMMAND -p 1"
test $FORMAT = d && COMMAND="$COMMAND -$FORMAT"
test $AZF2   = b && COMMAND="$COMMAND -$AZF2"
test $UNROLL = u && COMMAND="$COMMAND -$UNROLL"
test $PREC   = 3 || COMMAND="$COMMAND -p $PREC"
STATUS=
POSITION1=
POSITION2=
DIST12=
a12=
m12=
M1221=
S12=
set -o pipefail
if test "$INPUT"; then
    OUTPUT=`echo $INPUT | $EXECDIR/$COMMAND -e "$RADIUS" "$FLATTENING" 2>&1 |
            head -1`
    if test $? -eq 0; then
        STATUS=OK
        OUTPUTG=`echo $INPUT | $EXECDIR/$COMMANDX -e "$RADIUS" "$FLATTENING" |
                 head -1`
        POS1="`echo $OUTPUT | cut -f1-2 -d' '`"
        POS2="`echo $OUTPUT | cut -f4-5 -d' '`"
        POSG1="`echo $OUTPUTG | cut -f1-2 -d' '`"
        POSG2="`echo $OUTPUTG | cut -f4-5 -d' '`"
        AZI1="`echo $OUTPUT | cut -f3 -d' '`"
        AZI2="`echo $OUTPUT | cut -f6 -d' '`"
        DIST12="`echo $OUTPUT | cut -f7 -d' '`"
        a12="`echo $OUTPUT | cut -f8 -d' '`"
        m12="`echo $OUTPUT | cut -f9 -d' '`"
        M1221="`echo $OUTPUT | cut -f10-11 -d' '`"
        S12="`echo $OUTPUT | cut -f12 -d' '`"
        if test "$TYPE" = D; then
            POSITION1=$(geohack $POSG1 $POS1 Black)\ $(convertdeg "$AZI1")
            POSITION2=$F$(geohack $POSG2 $POS2 Blue)\ $(convertdeg "$AZI2")$G
            DIST12=$(encodevalue "$DIST12")
        else
            POSITION1=$(geohack $POSG1 $POS1 Black)\ $F$(convertdeg "$AZI1")$G
            POSITION2=$(geohack $POSG2 $POS2 Black)\ $F$(convertdeg "$AZI2")$G
            DIST12=$F$(encodevalue "$DIST12")$G
        fi
    else
        STATUS="$OUTPUT"
    fi
    # echo `date +"%F %T"` "$COMMAND: $INPUT" >> ../persistent/utilities.log
fi

echo Content-type: text/html
echo
cat <<EOF
<html>
  <head>
    <title>
      Online geodesic calculator
    </title>
    <meta name="description" content="Online geodesic calculator" />
    <meta name="author" content="Charles F. F. Karney" />
    <meta name="keywords"
          content="geodesics,
                   geodesic distance,
                   geographic distance,
                   shortest path,
                   direct geodesic problem,
                   inverse geodesic problem,
                   distance and azimuth,
                   distance and heading,
                   range and bearing,
                   spheroidal triangle,
                   latitude and longitude,
                   online calculator,
                   WGS84 ellipsoid,
                   GeographicLib" />
  </head>
  <body>
    <h3>
      Online geodesic calculations using the
      <a href="https://geographiclib.sourceforge.io/C++/doc/GeodSolve.1.html">
         GeodSolve</a> utility
    </h3>
    <form action="/cgi-bin/GeodSolve" method="get">
      <p>
        Geodesic calculation:
        <table>
          <tr>
            <td valign='baseline'>
              &nbsp;&nbsp;&nbsp;
              <label for='I'>
                <input type="radio" name="type" value="I" id='I'
                       `test "$TYPE" = I && echo CHECKED`>
                &nbsp;Inverse:&nbsp;
              </label>
            </td>
            <td valign='baseline'>
              <em>lat1 lon1 lat2 lon2</em>
            </td>
            <td valign='baseline'>
              &rarr; <em>azi1 azi2 s12</em>
            </td>
          </tr>
          <tr>
            <td valign='baseline'>
              &nbsp;&nbsp;&nbsp;
              <label for='D'>
                <input type="radio" name="type" value="D" id='D'
                       `test "$TYPE" = D && echo CHECKED`>
                &nbsp;Direct:&nbsp;
              </label>
            </td>
            <td valign='baseline'>
              <em>lat1 lon1 azi1 s12</em>
            </td>
            <td valign='baseline'>
              &rarr; <em>lat2 lon2 azi2</em>
            </td>
          </tr>
        </table>
      </p>
      <p>
        Input (ex. &laquo;<tt>40.6 -73.8 49&deg;01'N 2&deg;33'E</tt>&raquo;
        [inverse],
        &laquo;<tt>40d38'23"N 073d46'44"W 53d30' 5850e3</tt>&raquo;
        [direct]):
        <br>
        &nbsp;&nbsp;&nbsp;
        <input type=text name="input" size=72 value="$INPUTENC">
      </p>
      <p>
        <table>
          <tr>
            <td>
              Output format:
            </td>
EOF
while read c desc; do
    CHECKED=
    test "$c" = "$FORMAT" && CHECKED=CHECKED
    echo "<td>&nbsp;<label for='$c'>"
    echo "<input type='radio' name='format' value='$c' id='$c' $CHECKED>"
    echo "$desc</label></td>"
done <<EOF
g decimal degrees
d degrees minutes seconds
EOF
cat <<EOF
          </tr>
          <tr>
            <td>
              Heading at point 2:
            </td>
EOF
while read c desc; do
    CHECKED=
    test "$c" = "$AZF2" && CHECKED=CHECKED
    echo "<td>&nbsp;<label for='$c'>"
    echo "<input type='radio' name='azi2' value='$c' id='$c' $CHECKED>"
    echo "$desc</label></td>"
done <<EOF
f forward azimuth
b back azimuth
EOF
cat <<EOF
          </tr>
          <tr>
            <td>
              Longitude:
            </td>
EOF
while read c desc; do
    CHECKED=
    test "$c" = "$UNROLL" && CHECKED=CHECKED
    echo "<td>&nbsp;<label for='$c'>"
    echo "<input type='radio' name='unroll' value='$c' id='$c' $CHECKED>"
    echo "$desc</label></td>"
done <<EOF
r reduce to [&minus;180&deg;,180&deg;]
u unroll
EOF
cat <<EOF
          </tr>
          <tr>
            <td>
              Output precision:
            </td>
            <td colspan="2">&nbsp;
              <select name="prec" size=1>
EOF
while read p desc; do
    SELECTED=
    test "$p" = "$PREC" && SELECTED=SELECTED
    echo "<option $SELECTED value='$p'> $desc</option>"
done <<EOF
0 1m 0.00001d 0.1"
1 100mm 0.01"
2 10mm 0.001"
3 1mm 0.0001"
4 100&mu;m 0.00001"
5 10&mu;m 0.000001"
6 1&mu;m 0.0000001"
7 100nm 0.00000001"
8 10nm 0.000000001"
9 1nm 0.0000000001"
EOF
cat <<EOF
              </select>
            </td>
          </tr>
          <tr>
            <td>Equatorial radius:</td>
            <td>
              <input type=text name="radius" size=20 value="$RADIUS">
            </td>
            <td>meters</td>
          </tr>
          <tr>
            <td>Flattening:</td>
            <td>
              <input type=text name="flattening" size=20 value="$FLATTENING">
            </td>
          </tr>
        </table>
      </p>
      <p>
        Select action:<br>
        &nbsp;&nbsp;&nbsp;
        <input type="submit" name="option" value="Submit">
        <input type="submit" name="option" value="Reset">
      </p>
      <p>
        Geodesic (input in black, output in ${F}blue${G}):<br>
        <font size="4"><pre>
    ellipsoid (a f)$S= `encodevalue "$RADIUS"` `encodevalue "$FLATTENING"`$TAG
    status         $S= `encodevalue "$STATUS"`

    lat1 lon1 fazi1 (&deg;) = $POSITION1
    lat2 lon2 $AZIX (&deg;) = $POSITION2
    s12 (m)             = $DIST12

    a12 (&deg;)             = $F$a12$G
    m12 (m)             = $F$m12$G
    M12 M21             = $F$M1221$G
    S12 (m^2)           = $F$S12$G</pre></font>
      </p>
    </form>
    <hr>
    <p>
      <a href="https://geographiclib.sourceforge.io/C++/doc/GeodSolve.1.html">
        GeodSolve (version $VERSION)</a>
      performs geodesic calculations for an arbitrary ellipsoid of
      revolution.  The shortest path between two points on the ellipsoid
      at (<em>lat1</em>, <em>lon1</em>) and (<em>lat2</em>,
      <em>lon2</em>) is called the <em>geodesic</em>; its length
      is <em>s12</em> and the geodesic from point 1 to point 2 has
      azimuths <em>azi1</em> and <em>azi2</em> at the two end points.
      There are two standard geodesic problems:
      <ul>
        <li> Direct: &nbsp; given [<em>lat1 lon1 azi1 s12</em>], find
          [<em>lat2 lon2 azi2</em>];
        <li> Inverse: given [<em>lat1 lon1 lat2 lon2</em>], find
          [<em>azi1 azi2 s12</em>].
      </ul>
      Latitudes and longitudes can be given in various formats, for
      example (these all refer to the position of Timbuktu):
      <pre>
        16.776 -3.009
        16d47' -3d1'
        W3&deg;0'34" N16&deg;46'33"
        3:0:34W 16:46:33N</pre>
      Azimuths are given in degrees clockwise from north.  The
      distance <em>s12</em> is in meters.
    </p>
    <p>
      The additional quantities computed are:
      <ul>
        <li> <em>a12</em>, the arc length on the auxiliary sphere (&deg;),
        <li> <em>m12</em>, the reduced length (m),
        <li> <em>M12</em> and <em>M21</em>, the geodesic scales,
        <li> <em>S12</em>, the area between the geodesic
          and the equator (m<sup>2</sup>).
      </ul>
    </p>
    <p>
      The ellipsoid is specified by its equatorial radius, <em>a</em>,
      and its flattening,
      <em>f</em>&nbsp;=
      (<em>a</em>&nbsp;&minus;&nbsp;<em>b</em>)/<em>a</em>,
      where <em>b</em> is the polar semi-axis.  The default values for
      these parameters correspond to the WGS84 ellipsoid.  The method is
      accurate for &minus;99&nbsp;&le; <em>f</em>&nbsp;&le; 0.99
      (corresponding to 0.01&nbsp;&le; <em>b</em>/<em>a</em>&nbsp;&le;
      100).  Note that <em>f</em> is negative for a prolate ellipsoid
      (<em>b</em>&nbsp;&gt; <em>a</em>) and that it can be entered as a
      fraction, e.g., 1/297.
    </p>
    <p>
      GeodSolve is accurate to about 15&nbsp;nanometers (for the WGS84
      ellipsoid) and gives solutions for the inverse problem for any
      pair of points.
    </p>
    <p>
      <a href="https://geographiclib.sourceforge.io/C++/doc/GeodSolve.1.html">
        GeodSolve</a>,
      which is a simple wrapper of the
      <a href="https://geographiclib.sourceforge.io/C++/doc/classGeographicLib_1_1Geodesic.html">
        GeographicLib::Geodesic</a> class,
      is one of the utilities provided
      with <a href="https://geographiclib.sourceforge.io/">
        GeographicLib</a>.
      Geodesics can also be computed using JavaScript; see the
      <a href="../scripts/geod-calc.html">JavaScript geodesic
        calculator</a> and
      <a href="../scripts/geod-google.html">geodesics on Google
        maps</a>.  If you wish to use GeodSolve directly,
      <a href="https://sourceforge.net/projects/geographiclib/files/distrib-C++">
        download</a>
      and compile GeographicLib.  The algorithms are described
      in C. F. F. Karney,
      <a href="https://doi.org/10.1007/s00190-012-0578-z"><i>Algorithms for
      geodesics</i></a>,
      J. Geodesy <b>87</b>, 43&ndash;55 (2013); DOI:
      <a href="https://doi.org/10.1007/s00190-012-0578-z">
        10.1007/s00190-012-0578-z</a>;
      addenda:
      <a href="https://geographiclib.sourceforge.io/geod-addenda.html">
        geod-addenda.html</a>.  See also the Wikipedia page,
      <a href="https://en.wikipedia.org/wiki/Geodesics_on_an_ellipsoid">
        Geodesics on an ellipsoid</a>.
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
