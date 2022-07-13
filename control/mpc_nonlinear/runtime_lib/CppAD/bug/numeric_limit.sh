#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-15 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ ! -e ../include/cppad/configure.hpp ]
then
    echo 'numeric_limits.sh: must first run bin/run_cmake.sh'
    echo 'from parent directory.'
    exit 1
fi
cat << EOF
This is not a bug, but rather a test of specialization of numeric_limits
for AD types.
EOF
cat << EOF > bug.$$
# include <cppad/cppad.hpp>

/*!
\\def CPPAD_STD_NUMERIC_LIMITS(Other, Base)
This macro defines the specialization std::numeric_limits<Base>
to have the same values and functions as the existing specialization
std::numeric_limits<Other>.
*/
# define CPPAD_STD_NUMERIC_LIMITS(Other, Base) \\
namespace std {\\
    template <> class numeric_limits<Base>\\
    {\\
    public:\\
    static const bool is_specialized =\\
        numeric_limits<Other>::is_specialized;\\
    static const bool is_signed =\\
        numeric_limits<Other>::is_signed;\\
    static const bool is_integer =\\
        numeric_limits<Other>::is_integer;\\
    static const bool is_exact =\\
        numeric_limits<Other>::is_exact;\\
    static const bool has_infinity =\\
        numeric_limits<Other>::has_infinity;\\
    static const bool has_quiet_NaN =\\
        numeric_limits<Other>::has_quiet_NaN;\\
    static const bool has_signaling_NaN =\\
        numeric_limits<Other>::has_signaling_NaN;\\
    static const bool has_denorm_loss =\\
        numeric_limits<Other>::has_denorm_loss;\\
    static const bool is_iec559 =\\
        numeric_limits<Other>::is_iec559;\\
    static const bool is_bounded =\\
        numeric_limits<Other>::is_bounded;\\
    static const bool is_modulo =\\
        numeric_limits<Other>::is_modulo;\\
    static const bool traps =\\
        numeric_limits<Other>::traps;\\
    static const bool tinyness_before =\\
        numeric_limits<Other>::tinyness_before;\\
    static const int digits =\\
        numeric_limits<Other>::digits;\\
    static const int digits10 =\\
        numeric_limits<Other>::digits10;\\
    static const int radix =\\
        numeric_limits<Other>::radix;\\
    static const int min_exponent =\\
        numeric_limits<Other>::min_exponent;\\
    static const int min_exponent10 =\\
        numeric_limits<Other>::min_exponent10;\\
    static const int max_exponent =\\
        numeric_limits<Other>::max_exponent;\\
    static const int max_exponent10 =\\
        numeric_limits<Other>::max_exponent10;\\
    static const Base min(void)\\
    {   return static_cast<Base>( numeric_limits<Other>::min() ); }\\
    static const Base max(void)\\
    {   return static_cast<Base>( numeric_limits<Other>::max() ); }\\
    static const Base epsilon(void)\\
    {   return static_cast<Base>( numeric_limits<Other>::epsilon() ); }\\
    static const Base round_error(void)\\
    {   return static_cast<Base>( numeric_limits<Other>::round_error() ); }\\
    static const Base infinity(void)\\
    {   return static_cast<Base>( numeric_limits<Other>::infinity() ); }\\
    static const Base quiet_NaN(void)\\
    {   return static_cast<Base>( numeric_limits<Other>::quiet_NaN() ); }\\
    static const Base signaling_NaN(void)\\
    {   return static_cast<Base>( numeric_limits<Other>::signaling_NaN() ); }\\
    static const Base denorm_min(void)\\
    {   return static_cast<Base>( numeric_limits<Other>::denorm_min() ); }\\
    static const float_denorm_style has_denorm =\\
        numeric_limits<Other>::has_denorm;\\
    static const float_round_style round_style =\\
        numeric_limits<Other>::round_style;\\
    };\\
}
CPPAD_STD_NUMERIC_LIMITS(double, CppAD::AD<double>)

# define PRINT_VAL(name) \\
std::cout << #name << " = " \\
<< std::numeric_limits< CppAD::AD<double> >::name << std::endl;

# define PRINT_FUN(name) \\
std::cout << #name << " = " \\
<< std::numeric_limits< CppAD::AD<double> >::name() << std::endl;

int main(void)
{   bool ok = true;
    //
    PRINT_VAL(is_specialized)
    PRINT_VAL(is_signed)
    PRINT_VAL(is_integer)
    PRINT_VAL(is_exact)
    PRINT_VAL(has_infinity)
    PRINT_VAL(has_quiet_NaN)
    PRINT_VAL(has_signaling_NaN)
    PRINT_VAL(has_denorm_loss)
    PRINT_VAL(is_iec559)
    PRINT_VAL(is_bounded)
    PRINT_VAL(is_modulo)
    PRINT_VAL(traps)
    PRINT_VAL(tinyness_before)
    // int
    PRINT_VAL(digits)
    PRINT_VAL(digits10)
    PRINT_VAL(radix)
    PRINT_VAL(min_exponent)
    PRINT_VAL(min_exponent10)
    PRINT_VAL(max_exponent)
    PRINT_VAL(max_exponent10)
    // function
    PRINT_FUN(min)
    PRINT_FUN(max)
    PRINT_FUN(epsilon)
    PRINT_FUN(round_error)
    PRINT_FUN(infinity)
    PRINT_FUN(quiet_NaN)
    PRINT_FUN(signaling_NaN)
    PRINT_FUN(denorm_min)
    // other
    PRINT_VAL(has_denorm)
    PRINT_VAL(round_style)
    // C++11 only
    // PRINT_VAL(max_digits10)
    // PRINT_FUN(lowest)
    //
    if( ok )
        return 0;
    return 1;
}
EOF
# -----------------------------------------------------------------------------
if [ ! -e build ]
then
    mkdir build
fi
cd build
echo "$0"
name=`echo $0 | sed -e 's|.*/||' -e 's|\..*||'`
mv ../bug.$$ $name.cpp
echo "g++ -I../.. --std=c++11 -g $name.cpp -o $name"
g++ -I../.. --std=c++11 -g $name.cpp -o $name
#
echo "./$name"
if ! ./$name
then
    echo
    echo "$name.sh: Error"
    exit 1
fi
echo
echo "$name.sh: OK"
exit 0
