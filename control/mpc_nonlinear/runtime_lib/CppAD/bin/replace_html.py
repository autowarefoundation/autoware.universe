#! /usr/bin/env python2
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
import sys
import os
import re
# -----------------------------------------------------------------------------
if sys.argv[0] != 'bin/replace_html.py' :
    msg = 'bin/replace_html.py: must be executed from its parent directory'
    sys.exit(msg)
#
usage = '''\nusage: replace_html.py define_file replace_file new_file where

define_file:  contains the define commands
replace_file: contains the replace commands (many be same as define_file)
new_file:     is a copy of replace file with the replacements.

The definitions are specified by:
    <!-- define name -->source<!-- end name -->
where name is any unique name, with no spaces ' ', for the replacement text
and source is the replacement text.

The replacement positions are specified by:
    <!-- replace name -->desination<!-- end name -->
where name refers to a defined replacement text and destination
is the text that is replaced.
'''
narg = len(sys.argv)
if narg != 4 :
    msg = '\nExpected 3 but found ' + str(narg-1) + ' command line arguments.'
    sys.exit(usage + msg)
define_file  = sys.argv[1]
replace_file = sys.argv[2]
new_file     = sys.argv[3]
# -----------------------------------------------------------------------------
if not os.path.exists(define_file) :
    msg = 'bin/replace_html.py: cannot find define_file = ' + define_file
    sys.exit(msg)
if not os.path.exists(replace_file) :
    msg = 'bin/replace_html.py: cannot find replace_file = ' + replace_file
    sys.exit(msg)
if os.path.exists(new_file) :
    msg = 'bin/replace_html.py: cannot overwrite new_file ' + new_file
    sys.exit(msg)
f_in        = open(define_file, 'rb')
define_data = f_in.read()
f_in.close()
f_in         = open(replace_file, 'rb')
replace_data = f_in.read()
f_in.close()
# -----------------------------------------------------------------------------
# create define: a dictionary with replacement text definitions
define    = {}
p_define  = re.compile('<!-- define ([^ ]*) -->')
p_end     = re.compile('<!-- end ([^ ]*) -->')
start     = 0
while start < len(define_data) :
    rest         = define_data[start : ]
    next_define  = p_define.search(rest)
    if next_define == None :
        start = len(define_data)
    else :
        name         = next_define.group(1)
        if name in define :
            msg  = 'bin/replace_html.py: file = ' + define_file
            msg += '\ncontains two defintions for name = ' + name
            sys.exit(msg)
        rest         = rest[ next_define.end() : ]
        #
        next_end     = p_end.search(rest)
        source       = rest [ 0 : next_end.start() ]
        define[name] = source
        start       += next_define.end() + next_end.end()
        if name != next_end.group(1) :
            msg  = 'bin/replace_html.py: file = ' + define_file
            msg += '\ndefine name = ' + name
            msg += ', end name = ' + next_end.group(1)
            sys.exit(msg)
# -----------------------------------------------------------------------------
# create new_data: a string with the replacements made
new_data  = ''
p_replace = re.compile('<!-- replace ([^ ]*) -->')
start     = 0
while start < len(replace_data) :
    rest          = replace_data[start : ]
    next_replace  = p_replace.search(rest)
    if next_replace == None :
        new_data += rest
        start     = len(replace_data)
    else :
        name      = next_replace.group(1)
        if name not in define :
            msg  = 'bin/replace_html.py: file = ' + define_file
            msg += '\ncontains no defintions for name = ' + name
            sys.exit(msg)
        new_data    += rest[0 : next_replace.end() ]
        new_data    += define[name]
        #
        rest         = rest[ next_replace.end() : ]
        next_end     = p_end.search(rest)
        new_data    += rest[ next_end.start() : next_end.end() ]
        start       += next_replace.end() + next_end.end()
        if name != next_end.group(1) :
            msg  = 'bin/replace_html.py: file = ' + replace_file
            msg += '\nreplace name = ' + name
            msg += ', end name = ' + next_end.group(1)
            sys.exit(msg)
# -----------------------------------------------------------------------------
f_out    = open(new_file, 'wb')
f_out.write(new_data)
f_out.close()
# -----------------------------------------------------------------------------
sys.exit(0)
