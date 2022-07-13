# CppADCodeGen

CppADCodeGen performs **hybrid Automatic Differentiation** (AD), that is, uses 
operator-overloading and produces source-code. Such source-code can be 
statically compiled at runtime using an existing compiler and linked dynamically 
or, alternatively, go through a JIT compilation using Clang/LLVM.

The evaluation of differential information can be orders of magnitude faster
to compute using a compiled model than using a regular operator overloading
strategy.

In addition to C source generation, CppADCodeGen can also produce
 [Latex](http://www.latex-project.org/),
 html+[MathML](http://www.w3.org/Math/), and
 [dot](https://en.wikipedia.org/wiki/DOT_%28graph_description_language%29)
 source-code files for your algorithm.
Latex sources can be used to create PDF files for documentation purposes,
html+MathML can be used to display your algorithm in a web browser, and
dot files can be used to create images with a graph of your model
(see [graphviz](http://graphviz.org/)).

CppADCodeGen can also be used to perform differentiation index reduction of
Differential Algebraic Equations (DAE) through Pantelides, Soares-Secchi, and Dummy
Derivatives methods.

CppADCodeGen is built on top of the [CppAD](http://www.coin-or.org/CppAD)
library, which is a header only C++ AD library using operator overloading.

## License ##

CppADCodeGen is available with both the **EPL** and **GPL** licenses
(suitable for both open-source and closed-source commercial projects).
See epl-v10.txt and gpl3.txt for a copy of the licenses.

## Requirements ##

CppADCodeGen is a C++11 header only library, therefore there aren't many dependencies:

 - **CppAD** (2020),
 - A **C++11** compiler (such as GCC and Clang),
 - Clang/LLVM (only required for JIT compilation), and
 - Eigen 3 (required when DAE differentiation index reduction is used).

Runtime compilation and dynamic linking:
 - Linux (it might be very easy to support other OSes but it is not implemented yet)

## Installing ##

### General installation ###

Get the sources from GitHub:
```
    git clone https://github.com/joaoleal/CppADCodeGen.git CppADCodeGen
```
Create a new folder to build the project:
```
    mkdir cppadcg-build
```
Build the project (no compilation of C/C++ occurs, just generation of header files):
```
    cd cppadcg-build
    cmake ../CppADCodeGen
```
Either install the project in your system:
```
    make install
```
or to some other folder:
```
    make DESTDIR=someotherfolder install
```

### Debian/Ubuntu ###

A debian installation package can be created at the root of the project.
Typically you can create the installer by just typing:
```
    dpkg-buildpackage
```
It will create a debian package outside the project's folder.

## Using CppADCodeGen ##

See the [wiki](https://github.com/joaoleal/CppADCodeGen/wiki).

The folder example includes some simple use cases.

## Testing ##

Get the sources from GitHub:
```
    git clone https://github.com/joaoleal/CppADCodeGen.git CppADCodeGen
```
Create a new folder for the tests:
```
    cd make-build-debug
    cmake ../CppADCodeGen
```
Testing requires [google-test](https://github.com/google/googletest) (version 1.8.1).
Either install it in your system or configure CppADCodeGen to download the sources from GitHub by replacing the previous line with:
```
    cmake -DGOOGLETEST_GIT=ON ../CppADCodeGen 
```

Then compile the tests:
```
    make build_tests
```

Run the complete set of tests:
```
    make test
```
If [valgrind](https://valgrind.org/) is available in your system, CppADCodeGen will also perform memory checks which can
lead to a very lengthy test execution.
It is possible to disable memory validations by turning off the CMake option `USE_VALGRIND`.
For instance, by calling the following command before running the tests:
 ```
     cmake -DUSE_VALGRIND=OFF ../CppADCodeGen 
 ```
---

## Repository Content

|Directories |  Description                                                    |
|------------|-----------------------------------------------------------------|
|bin         | Helper shell and sed scripts used for CppAD development.        |
|bug         | Directory containing demonstration of known bugs (may be empty).|
|debian      | Debian package creation files (Linux).                          |
|doc         | Holds files for generation of developer documentation.          |
|example     | CppADCodegen example files are here.                            |
|include     | The CppADCodeGen header files.                                  |
|pkgconfig   | pkg-config support files.                                       |
|python      | Pretty printers for GDB (debugging).                            |
|speed       | Contains some benchmarks for CppADCodeGen.                      |
|test        | Contains tests for CppADCodeGen.                                |


| Files         |  Description                                                 |
|---------------|--------------------------------------------------------------|
|AUTHORS        | Statement of authorship and copyright.                       |
|CMakeLists.txt | CppADCodeGen CMake input file.                               |
|COPYING        | Statement of user license to use software.                   |
|epl-v10.txt    | A copy of the Eclipse Public License version 1.              |
|gpl3.txt       | A copy of the GNU General Public License version 3.          |
|INSTALL        | Points to this file.                                         |
