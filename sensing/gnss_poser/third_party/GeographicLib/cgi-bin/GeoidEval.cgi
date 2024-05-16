#! /bin/sh
#
# GeoidEval.cgi
# cgi script for geoid height evaluations
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
fi
INPUTENC=`encodevalue "$INPUT"`
EXECDIR=../bin
COMMAND="GeoidEval"
VERSION=`$EXECDIR/$COMMAND --version | cut -f4 -d" "`
export GEOGRAPHICLIB_DATA=..
F='<font color="blue">'
G='</font>'
POSITION1=
POSITION2=
HEIGHT96=
HEIGHT84=
HEIGHT2008=
set -o pipefail
if test "$INPUT"; then
    HEIGHT96=`echo $INPUT |
    $EXECDIR/$COMMAND -n egm96-5 | head -1`
    if test $? -eq 0; then
        POSITION1=`echo $INPUT | $EXECDIR/GeoConvert | head -1`
        POSITION1=`geohack $POSITION1 $POSITION1 Black`
        POSITION2=\(`echo $INPUT | $EXECDIR/GeoConvert -d -p -1 | head -1`\)
        HEIGHT2008=`echo $INPUT |
        $EXECDIR/$COMMAND -n egm2008-1 | head -1`
        HEIGHT84=`echo $INPUT |
        $EXECDIR/$COMMAND -n egm84-15 | head -1`
    else
        POSITION1=`encodevalue "$HEIGHT96"`
        HEIGHT96=
    fi
    # echo `date +"%F %T"` "$COMMAND: $INPUT" >> ../persistent/utilities.log
fi

echo Content-type: text/html
echo
cat <<EOF
<html>
  <head>
    <title>
      Online geoid calculator
    </title>
    <meta name="description" content="Online geoid calculator" />
    <meta name="author" content="Charles F. F. Karney" />
    <meta name="keywords"
          content="geoid height,
                   orthometric height,
                   earth gravity model,
                   EGM84, EGM96, EGM2008,
                   mean sea level, MSL,
                   height above ellipsoid, HAE,
                   vertical datum,
                   latitude and longitude,
                   online calculator,
                   WGS84 ellipsoid,
                   GeographicLib" />
  </head>
  <body>
    <h3>
      Online geoid calculations using the
      <a href="https://geographiclib.sourceforge.io/C++/doc/GeoidEval.1.html">
        GeoidEval</a> utility
    </h3>
    <form action="/cgi-bin/GeoidEval" method="get">
      <p>
        Position
        (ex. &laquo;<tt>16.78 -3.01</tt>&raquo;,
        &laquo;<tt>16d46'33"N 3d0.6'W</tt>&raquo;):<br>
        &nbsp;&nbsp;&nbsp;
        <input type=text name="input" size=30 value="$INPUTENC">
      </p>
      <p>
        Select action:<br>
        &nbsp;&nbsp;&nbsp;
        <input type="submit" name="option" value="Submit">
        <input type="submit" name="option" value="Reset">
      </p>
      <p>
        Geoid height:
<font size="4"><pre>
    lat lon = $POSITION1 `convertdeg "$POSITION2"`
    geoid heights (m)
        <a href="https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84#tab_egm2008">EGM2008</a> = $F`encodevalue "$HEIGHT2008"`$G
        <a href="https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84#tab_egm96">EGM96</a>   = $F`encodevalue "$HEIGHT96"`$G
        <a href="https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84#tab_egm84">EGM84</a>   = $F`encodevalue "$HEIGHT84"`$G</pre></font>
      </p>
    </form>
    <hr>
    <p>
      <a href="https://geographiclib.sourceforge.io/C++/doc/GeoidEval.1.html">
        GeoidEval (version $VERSION)</a>
      computes the height of the geoid above the WGS84 ellipsoid
      using interpolation in a grid of values for the earth
      gravity models,
      <a href="https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84#tab_egm84">
        EGM84</a>, or
      <a href="https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84#tab_egm96">
        EGM96</a>,
      <a href="https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84#tab_egm2008">
            EGM2008</a>.
      The RMS error in the interpolated height is about 1&nbsp;mm.
      Give the position in terms of latitude and longitude, for example
      (these all refer to the position of Timbuktu):
      <pre>
        16.776 -3.009
        16d47' -3d1'
        W3&deg;0'34" N16&deg;46'33"
        3:0:34W 16:46:33N</pre>
      The coordinates can also be given in UTM, UPS, or MGRS coordinates (see
      the documentation on the
      <a href="https://geographiclib.sourceforge.io/C++/doc/GeoConvert.1.html">
        GeoConvert</a> utility).
    </p>
    <p>
      The height of the geoid above the ellipsoid, <i>N</i>, is
      sometimes called the geoid undulation.  It can be used to convert
      a height above the ellipsoid, <i>h</i>, to the corresponding
      height above the geoid (the orthometric height, roughly the height
      above mean sea level), <i>H</i>, using the relations
      <blockquote>
        <i>h</i> = <i>N</i> + <i>H</i>;
        &nbsp;&nbsp;<i>H</i> = &minus;<i>N</i> + <i>h</i>.
      </blockquote>
    </p>
    <p>
      <a href="https://geographiclib.sourceforge.io/C++/doc/GeoidEval.1.html">
        GeoidEval</a>,
      which is a simple wrapper of the
      <a href="https://geographiclib.sourceforge.io/C++/doc/classGeographicLib_1_1Geoid.html">
        GeographicLib::Geoid</a> class,
      is one of the utilities provided
      with <a href="https://geographiclib.sourceforge.io/">
        GeographicLib</a>.
      This web interface illustrates a subset of its capabilities.  If
      you wish to use GeoidEval directly,
      <a href="https://sourceforge.net/projects/geographiclib/files/distrib-C++">
        download</a>
      and compile GeographicLib.  A description of the methods is given
      <a href="https://geographiclib.sourceforge.io/C++/doc/geoid.html">
        here</a>.
    </p>
    <p>
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
