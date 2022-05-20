#!/usr/bin/env python3

# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
from collections import OrderedDict
import os
import sys

from catkin_pkg.package import evaluate_condition
from catkin_pkg.package import parse_package_string


def main(argv=sys.argv[1:]):
    """
    Extract the information from package.xml and make them accessible to CMake.

    Parse the given package.xml file and
    print CMake code defining several variables containing the content.
    """
    parser = argparse.ArgumentParser(
        description='Parse package.xml file and print CMake code defining '
                    'several variables',
    )
    parser.add_argument(
        'package_xml',
        type=argparse.FileType('r', encoding='utf-8'),
        help='The path to a package.xml file',
    )
    parser.add_argument(
        'outfile',
        nargs='?',
        help='The filename where the output should be written to',
    )
    args = parser.parse_args(argv)

    try:
        package = parse_package_string(
            args.package_xml.read(), filename=args.package_xml.name)
    except Exception as e:
        print("Error parsing '%s':" % args.package_xml.name, file=sys.stderr)
        raise e
    finally:
        args.package_xml.close()

    lines = generate_cmake_code(package)
    if args.outfile:
        with open(args.outfile, 'w', encoding='utf-8') as f:
            for line in lines:
                f.write('%s\n' % line)
    else:
        for line in lines:
            print(line)


def get_dependency_values(key, depends):
    dependencies = []

    # Filter the dependencies, checking for any condition attributes
    dependencies.append((key, ' '.join([
        '"%s"' % str(d) for d in depends
        if d.condition is None or d.evaluate_condition(os.environ)
    ])))

    for d in depends:
        comparisons = [
            'version_lt',
            'version_lte',
            'version_eq',
            'version_gte',
            'version_gt']
        for comp in comparisons:
            value = getattr(d, comp, None)
            if value is not None:
                dependencies.append(('%s_%s_%s' % (key, str(d), comp.upper()),
                                     '"%s"' % value))
    return dependencies


def generate_cmake_code(package):
    """
    Return a list of CMake set() commands containing the manifest information.

    :param package: catkin_pkg.package.Package
    :returns: list of str
    """
    variables = []
    variables.append(('VERSION', '"%s"' % package.version))

    variables.append((
        'MAINTAINER',
        '"%s"' % (', '.join([str(m) for m in package.maintainers]))))

    variables.extend(get_dependency_values('BUILD_DEPENDS',
                                           package.build_depends))
    variables.extend(get_dependency_values('BUILDTOOL_DEPENDS',
                                           package.buildtool_depends))
    variables.extend(get_dependency_values('BUILD_EXPORT_DEPENDS',
                                           package.build_export_depends))
    variables.extend(get_dependency_values('BUILDTOOL_EXPORT_DEPENDS',
                                           package.buildtool_export_depends))
    variables.extend(get_dependency_values('EXEC_DEPENDS',
                                           package.exec_depends))
    variables.extend(get_dependency_values('TEST_DEPENDS',
                                           package.test_depends))
    variables.extend(get_dependency_values('GROUP_DEPENDS',
                                           package.group_depends))
    variables.extend(get_dependency_values('MEMBER_OF_GROUPS',
                                           package.member_of_groups))

    deprecated = [e.content for e in package.exports
                  if e.tagname == 'deprecated']
    variables.append(('DEPRECATED',
                      '"%s"' % ((deprecated[0] if deprecated[0] else 'TRUE')
                                if deprecated
                                else '')))

    lines = []
    lines.append('set(_AMENT_PACKAGE_NAME "%s")' % package.name)
    for (k, v) in variables:
        lines.append('set(%s_%s %s)' % (package.name, k, v))

    lines.append('set(%s_EXPORT_TAGS)' % package.name)
    replaces = OrderedDict()
    replaces['${prefix}/'] = ''
    replaces['\\'] = '\\\\'  # escape backslashes
    replaces['"'] = '\\"'  # prevent double quotes to end the CMake string
    replaces[';'] = '\\;'  # prevent semicolons to be interpreted as list separators
    for export in package.exports:
        export = str(export)
        for k, v in replaces.items():
            export = export.replace(k, v)
        lines.append('list(APPEND %s_EXPORT_TAGS "%s")' % (package.name, export))

    return lines


if __name__ == '__main__':
    main()
