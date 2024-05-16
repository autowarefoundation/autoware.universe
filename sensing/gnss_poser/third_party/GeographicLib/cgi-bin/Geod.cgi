#! /bin/sh
#
# Geod.cgi (redirect to GeodSolve.cgi)
#
# Copyright (c) Charles Karney (2013) <karney@alum.mit.edu> and licensed
# under the MIT/X11 License.  For more information, see
# https://geographiclib.sourceforge.io/

cat <<EOF
Content-type: text/html

<html>
  <head>
    <title>Online geodesic calculator</title>
    <meta HTTP-EQUIV="Refresh"
          CONTENT="5; URL=https://geographiclib.sourceforge.io/cgi-bin/GeodSolve">
  </head>
  <body topmargin=10 leftmargin=10>
    <h3>
      <blockquote>
        <em>
          The Geod calculator has been renamed GeodSolve which is available at
          <center>
            <a href="https://geographiclib.sourceforge.io/cgi-bin/GeodSolve">
              https://geographiclib.sourceforge.io/cgi-bin/GeodSolve</a>.
          </center>
          <br>
          You will be redirected there.  Click on the link to go there
          directly.
        </em>
      </blockquote>
    </h3>
  </body>
</html>
EOF
