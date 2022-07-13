#! /bin/bash -e
# $Id:$
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
#! /bin/bash -e
web_page='https://github.com/joaoleal/CppADCodeGen.git'
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
cat << EOF
This is not a bug but rather a demonstration of using conditional expressions
with CppADCodeGen
    https://github.com/joaoleal/CppADCodeGen/
EOF
#
if [ ! -e build ]
then
    echo_eval mkdir -p build
fi
echo_eval cd build
cat << EOF > cppad_cg.cpp
# include <iosfwd>
# include <cppad/cg/cppadcg.hpp>
int main(void)
{   typedef CppAD::cg::CG<double> cg_double;
    typedef CppAD::AD<cg_double>  acg_double;

    // declare independent variables for f(x)
    size_t nx = 2;
    CppAD::vector<acg_double> ax(nx);
    ax[0] = 2.0;
    ax[1] = 3.0;
    CppAD::Independent(ax);

    // create dependent variables and values for f(x)
    // f(x) = x[0] / x[1] if (x[0] > 0 and x[1] >= x[0]) else 1.0
    size_t nz = 1;
    CppAD::vector<acg_double> az(nz);
    acg_double acg_zero   = acg_double(0.0);
    acg_double acg_one    =   acg_double(1.0);
    acg_double acg_temp_1 = CondExpGt(ax[1], ax[0], ax[0] / ax[1], acg_one);
    acg_double acg_temp_2 = CondExpGt(ax[0], acg_zero, acg_temp_1, acg_one);
    az[0] = acg_temp_2;

    // create AD function mapping independent to dependent variables
    CppAD::ADFun<cg_double> F(ax, az);

    // create the source code generator for function g(x)
    CppAD::cg::CodeHandler<double> G;

    // declare the independent variables for g(x)
    CppAD::vector<cg_double>   cg_x(nx);
    G.makeVariables(cg_x);

    // Compute the dependent variables and values for g(x)
    size_t ny = nz * nx;
    CppAD::vector<cg_double> cg_y(ny);
    cg_y = F.Jacobian(cg_x);

    // Mapping from variables in this program to variables in source_code
    // independent variable = x
    // dependent variable   = y
    // temporary variable   = v
    CppAD::cg::LanguageC<double> langC("double");
    CppAD::cg::LangCDefaultVariableNameGenerator<double> nameGen;

    // generate the source code
    std::ostringstream source_code;
    G.generateCode(source_code, langC, cg_y, nameGen);

    // string corresponding to source code
    std::string source_str = source_code.str();

    // C souce code corresponding to y = g(x)
    std::cout << source_str;

    return 0;
}
EOF
#
# Compile and run cppad_cg.cpp
echo_eval g++ \
    -g \
    -std=c++11 \
    -I$HOME/prefix/cppad_cg/include \
    -I../.. \
    cppad_cg.cpp -o cppad_cg
#
# Determine the maximum v index
v_max_index=`./cppad_cg | sed \
    -e '/^ *v\[[0-9]*\]/! d' \
    -e 's|^ *v\[\([0-9]*\)\].*|\1|'  | sort | tail -1`
#
#
# Wrap y = g(x) in C++ function and test it
cat << EOF > tst_cppad_cg.cpp
# include <cppad/cppad.hpp>
namespace {
    using CppAD::zdouble;
    typedef CppAD::vector<zdouble> zvector;
    //
    // f(x) = x[0] / x[1] if (x[0] > 0 and x[1] >= x[0]) else 1.0
    // g(x) = d/dx f(x)
    void g(const zvector& x, zvector& y)
    {   zvector v($v_max_index + 1);
EOF
./cppad_cg | sed -e 's|^ *|\t\t|' >> tst_cppad_cg.cpp
cat << EOF >> tst_cppad_cg.cpp
        return;
    }
}
int main(void)
{   // initialize flag
    bool   ok   = true;
    // numerical precision for tests
    zdouble eps = 10. * std::numeric_limits<double>::epsilon();
    // number of components in vectors
    size_t nx    = 2;
    size_t nz    = 1;
    size_t ny    = nz * nx;
    zdouble zero = 0.0;
    //
    // compute y = g(x) case where x[0] == 0.0
    zvector x(nx), y(ny);
    x[0] = 0.0;
    x[1] = 0.0;
    g(x, y);
    //
    // check results
    ok  &= CppAD::NearEqual(y[0], zero, eps, eps);
    ok  &= CppAD::NearEqual(y[1], zero, eps, eps);
    //
    // compute y = g(x) case where g(x) = x[0] / x[1]
    x[0] = 2.0;
    x[1] = 3.0;
    g(x, y);
    //
    // check results
    ok  &= CppAD::NearEqual(y[0], 1.0/x[1], eps, eps);
    ok  &= CppAD::NearEqual(y[1], -x[0]/(x[1]*x[1]), eps, eps);
    //
    if( ! ok )
        return 1;
    return 0;
}
EOF
#
# Compile test
echo_eval g++ \
    -g \
    -std=c++11 \
    -I../.. \
    tst_cppad_cg.cpp -o tst_cppad_cg
#
if ! ./tst_cppad_cg
then
    file="$HOME/install/cppad_cg/build/tst_cppad_cg.cpp"
    echo "install_cppad_cg.sh: Error"
    exit 1
fi
#
echo 'cppad_cg.sh: OK'
exit 0
