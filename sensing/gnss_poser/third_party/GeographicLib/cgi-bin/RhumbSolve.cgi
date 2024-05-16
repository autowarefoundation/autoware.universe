#! /bin/sh
#
# RhumbSolve.cgi
# cgi script for rhumb line calculations
#
# Copyright (c) Charles Karney (2014-2023) <karney@alum.mit.edu> and
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
    PREC=`lookupkey "$QUERY_STRING" prec`
    TYPE=`lookupkey "$QUERY_STRING" type`
fi
test "$RADIUS" || RADIUS=$DEFAULTRADIUS
test "$FLATTENING" || FLATTENING=$DEFAULTFLATTENING
test "$FORMAT" || FORMAT=g
test "$PREC" || PREC=3
test "$TYPE" || TYPE=I
TAG=
if test "$RADIUS" = "$DEFAULTRADIUS" -a \
  "$FLATTENING" = "$DEFAULTFLATTENING"; then
  TAG=" (WGS84)"
fi

INPUTENC=`encodevalue "$INPUT"`
EXECDIR=../bin
COMMAND="RhumbSolve -E"
VERSION=`$EXECDIR/$COMMAND --version | cut -f4 -d" "`
F='<font color="blue">'
G='</font>'
test $TYPE = D || COMMAND="$COMMAND -i"
COMMANDX="$COMMAND -p 1"
test $FORMAT = g || COMMAND="$COMMAND -$FORMAT"
test $PREC = 3 || COMMAND="$COMMAND -p $PREC"
GCOMMAND=`echo "$COMMAND" | sed "s/RhumbSolve/GeodSolve -f/"`
GCOMMANDX=`echo "$COMMANDX" | sed "s/RhumbSolve/GeodSolve -f/"`
STATUS=
POSITION1=
POSITION2=
DIST12=
S12=
set -o pipefail
if test "$INPUT"; then
    OUTPUT=`echo $INPUT | $EXECDIR/$COMMAND -e "$RADIUS" "$FLATTENING" 2>&1 |
            head -1`
    if test $? -eq 0; then
        STATUS=OK
        GOUTPUT=`echo $INPUT | $EXECDIR/$GCOMMAND -e "$RADIUS" "$FLATTENING" |
            head -1`
        OUTPUTF=`echo $INPUT | $EXECDIR/$COMMANDX -e "$RADIUS" "$FLATTENING" |
            head -1`
        GOUTPUTF=`echo $INPUT |
            $EXECDIR/$GCOMMANDX -e "$RADIUS" "$FLATTENING" |
            head -1`
        if test "$TYPE" = D; then
            POS1="`echo $GOUTPUT | cut -f1-2 -d' '`"
            POSG1="`echo $GOUTPUTF | cut -f1-2 -d' '`"
            AZI12="`echo $GOUTPUT | cut -f3 -d' '`"
            DIST12="`echo $GOUTPUT | cut -f7 -d' '`"
            POS2="`echo $OUTPUT | cut -f1-2 -d' '`"
            POSG2="`echo $OUTPUTF | cut -f1-2 -d' '`"
            S12="`echo $OUTPUT | cut -f3 -d' '`"
            POSITION1=$(geohack $POSG1 $POS1 Black)
            POSITION2=$F$(geohack $POSG2 $POS2 Blue)$G
            echo $POS2 | grep nan > /dev/null &&
            POSITION2=$F$(convertdeg "$POS2")$G
            AZIMUTH=$(convertdeg "$AZI12")
            DIST12=$(encodevalue "$DIST12")
        else
            POS1="`echo $GOUTPUT | cut -f1-2 -d' '`"
            POSG1="`echo $GOUTPUTF | cut -f1-2 -d' '`"
            POS2="`echo $GOUTPUT | cut -f4-5 -d' '`"
            POSG2="`echo $GOUTPUTF | cut -f4-5 -d' '`"
            AZI12="`echo $OUTPUT | cut -f1 -d' '`"
            DIST12="`echo $OUTPUT | cut -f2 -d' '`"
            S12="`echo $OUTPUT | cut -f3 -d' '`"
            POSITION1=$(geohack $POSG1 $POS1 Black)
            POSITION2=$(geohack $POSG2 $POS2 Black)
            AZIMUTH=$F$(convertdeg "$AZI12")$G
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
      Online rhumb line calculator
    </title>
    <meta name="description" content="Online rhumb line calculator" />
    <meta name="author" content="Charles F. F. Karney" />
    <meta name="keywords"
          content="rhumb line, loxodrome,
                   rhumb line distance,
                   geographic distance,
                   direct rhumb line problem,
                   inverse rhumb line problem,
                   distance and azimuth,
                   distance and heading,
                   range and bearing,
                   latitude and longitude,
                   online calculator,
                   WGS84 ellipsoid,
                   GeographicLib" />
  </head>
  <body>
    <h3>
      Online rhumb line calculations using the
      <a href="https://geographiclib.sourceforge.io/C++/doc/RhumbSolve.1.html">
         RhumbSolve</a> utility
    </h3>
    <form action="/cgi-bin/RhumbSolve" method="get">
      <p>
        Rhumb Line calculation:
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
              &rarr; <em>azi12 s12</em>
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
              <em>lat1 lon1 azi12 s12</em>
            </td>
            <td valign='baseline'>
              &rarr; <em>lat2 lon2</em>
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
    echo "$desc</label>"
    echo "</td>"
done <<EOF
g Decimal degrees
d Degrees minutes seconds
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
        Rhumb Line (input in black, output in ${F}blue${G}):<br>
        <font size="4"><pre>
    ellipsoid (a f) = `encodevalue "$RADIUS"` `encodevalue "$FLATTENING"`$TAG
    status          = `encodevalue "$STATUS"`

    lat1 lon1 (&deg;)   = $POSITION1
    lat2 lon2 (&deg;)   = $POSITION2
    azi12 (&deg;)       = $AZIMUTH
    s12 (m)         = $DIST12

    S12 (m^2)       = $F$S12$G</pre></font>
      </p>
    </form>
    <hr>
    <p>
      <a href="https://geographiclib.sourceforge.io/C++/doc/RhumbSolve.1.html">
        RhumbSolve (version $VERSION)</a>
      performs rhumb line calculations for an arbitrary ellipsoid of
      revolution.  The path with a constant heading between two points
      on the ellipsoid at (<em>lat1</em>, <em>lon1</em>) and
      (<em>lat2</em>, <em>lon2</em>) is called the <em>rhumb line</em>
      (or <em>loxodrome</em>); its length is <em>s12</em> and the rhumb
      line has a forward azimuth <em>azi12</em> along its
      length.  <b>NOTE:</b> the rhumb line is <em>not</em> the shortest
      path between two points; that is the geodesic and it is calculated
      by
      <a href="https://geographiclib.sourceforge.io/cgi-bin/GeodSolve">
        GeodSolve</a>.
    </p>
    <p>
      There are two standard rhumb line problems:
      <ul>
        <li> Direct: &nbsp; given [<em>lat1 lon1 azi12 s12</em>], find
          [<em>lat2 lon2</em>];
        <li> Inverse: given [<em>lat1 lon1 lat2 lon2</em>], find
          [<em>azi12 s12</em>].
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
      The additional quantity computed is:
      <ul>
        <li> <em>S12</em>, the area between the rhumb line
          and the equator (m<sup>2</sup>).
      </ul>
    </p>
    <p>
      The ellipsoid is specified by its equatorial radius, <em>a</em>,
      and its flattening, <em>f</em>&nbsp;=
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
      RhumbSolve is accurate to about 15&nbsp;nanometers (for the WGS84
      ellipsoid) and gives solutions for the inverse problem for any
      pair of points.  The longitude becomes indeterminate when a rhumb
      line passes through a pole, and this tool reports NaNs (not a
      number) for <em>lon2</em> and <em>S12</em> in this case.
    </p>
    <p>
      <a href="https://geographiclib.sourceforge.io/C++/doc/RhumbSolve.1.html">
        RhumbSolve</a>,
      which is a simple wrapper of the
      <a href="https://geographiclib.sourceforge.io/C++/doc/classGeographicLib_1_1Rhumb.html">
        GeographicLib::Rhumb</a> class, is one of the utilities provided
      with <a href="https://geographiclib.sourceforge.io/">
      GeographicLib</a>.  This methods are described in
      C. F. F. Karney,
      <a href="https://arxiv.org/abs/2303.03219">The area of rhumb
        polygons</a>,
      Technical Report, SRI International, March 2023; URL
      <a href="https://arxiv.org/abs/2303.03219">arxiv:2303.03219</a>.
    </p>
    <hr>
    <address>Charles Karney
      <a href="mailto:karney@alum.mit.edu">&lt;karney@alum.mit.edu&gt;</a>
      (2022-04-10)</address>
    <a href="https://geographiclib.sourceforge.io">
      GeographicLib home page
    </a>
  </body>
</html>
EOF
