#! /bin/sh
(
    cat <<EOF
<deployment-project plugin="plugin.toolbox" plugin-version="1.0">
  <configuration build-checksum="" file="$ROOT/geographiclib.prj" location="$ROOT" name="geographiclib" target="target.toolbox" target-name="Package Toolbox">
    <param.appname>geographiclib</param.appname>
    <param.authnamewatermark>Charles Karney</param.authnamewatermark>
    <param.email>karney@alum.mit.edu</param.email>
    <param.company />
    <param.summary>MATLAB implementations of a subset of the C++ library, GeographicLib</param.summary>
    <param.description>GeographicLib toolbox
Version $VERSION $DATE

EOF
    cat $ROOT/geographiclib-blurb.txt
    cat <<EOF
</param.description>
    <param.screenshot>\${PROJECT_ROOT}/geodesic.png</param.screenshot>
    <param.version>$VERSION</param.version>
    <param.output>\${PROJECT_ROOT}/geographiclib.mltbx</param.output>
    <param.products.name>
      <item>MATLAB</item>
    </param.products.name>
    <param.products.id>
      <item>1</item>
    </param.products.id>
    <param.products.version>
      <item>7.9</item>
    </param.products.version>
    <param.platforms />
    <param.guid />
    <param.exclude.filters />
    <param.examples>&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;examples/&gt;</param.examples>
    <param.demosxml />
    <param.apps />
    <param.docs />
    <unset>
      <param.company />
      <param.output />
      <param.platforms />
      <param.exclude.filters />
      <param.demosxml />
      <param.apps />
      <param.docs />
    </unset>
    <fileset.rootdir>
      <file>\${PROJECT_ROOT}/geographiclib</file>
    </fileset.rootdir>
    <fileset.rootfiles>
      <file>${PROJECT_ROOT}/geographiclib</file>
    </fileset.rootfiles>
    <fileset.depfun.included />
    <fileset.depfun.excluded />
    <fileset.package />
    <build-deliverables>
      <file location="$ROOT" name="geographiclib.mltbx" optional="false">$ROOT/geographiclib.mltbx</file>
    </build-deliverables>
    <workflow />
    <matlab />
    <platform>
      <unix>true</unix>
      <mac>false</mac>
      <windows>false</windows>
      <win2k>false</win2k>
      <winxp>false</winxp>
      <vista>false</vista>
      <linux>true</linux>
      <solaris>false</solaris>
      <osver />
      <os32>false</os32>
      <os64>true</os64>
      <arch>glnxa64</arch>
      <matlab>true</matlab>
    </platform>
  </configuration>
</deployment-project>
EOF
) > $ROOT/geographiclib.prj

exit
