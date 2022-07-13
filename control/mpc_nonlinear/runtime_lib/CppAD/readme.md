# Title
CppAD: A Package for Differentiation of C++ Algorithms

# Links

- [Documentation](https://coin-or.github.io/CppAD/doc)

- [News](https://coin-or.github.io/CppAD/doc/whats_new.htm)

- [Install](https://coin-or.github.io/CppAD/doc/install.htm)

- [Directories](https://coin-or.github.io/CppAD/doc/directory.htm)

- [Downloads Before 2019](https://www.coin-or.org/download/source/CppAD/)


# License
<pre>
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
</pre>


# Autotools
The preferred method to test and install CppAD uses [CMake](https://cmake.org).
The deprecated Autotools procedure can be used for this purpose,
but it will eventually be removed.
For any sub-directory *dir*,
files of the form *dir*/`makefile.am` and *dir*/`makefile.in`
are used to support the Autotools test and install procedure.
In addition,
the following files, in this directory, are also for this purpose:
`compile`,
`config.guess`,
`config.sub`,
`configure.ac`,
`depcomp`,
`install-sh`,
`missing`.


# Copyright
See the file `authors` in this directory.
