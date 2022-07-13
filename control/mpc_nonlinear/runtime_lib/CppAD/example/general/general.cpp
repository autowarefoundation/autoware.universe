/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin general.cpp$$
$spell
    Cpp
$$

$section CppAD Examples and Tests$$

$head Running Tests$$
To build this program and run its correctness tests see $cref cmake_check$$.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

// CPPAD_HAS_* defines
# include <cppad/configure.hpp>

// system include files used for I/O
# include <iostream>

// C style asserts
# include <cassert>

// standard string
# include <string>

// memory utility
# include <cppad/utility/thread_alloc.hpp>

// test runner
# include <cppad/utility/test_boolofvoid.hpp>

// BEGIN_SORT_THIS_LINE_PLUS_1
extern bool Add(void);
extern bool AddEq(void);
extern bool BenderQuad(void);
extern bool BoolFun(void);
extern bool Compare(void);
extern bool CondExp(void);
extern bool Cos(void);
extern bool Cosh(void);
extern bool Div(void);
extern bool DivEq(void);
extern bool EqualOpSeq(void);
extern bool ForOne(void);
extern bool ForTwo(void);
extern bool Forward(void);
extern bool FunCheck(void);
extern bool HesLagrangian(void);
extern bool HesLuDet(void);
extern bool HesMinorDet(void);
extern bool HesTimesDir(void);
extern bool Hessian(void);
extern bool Independent(void);
extern bool Integer(void);
extern bool Interface2C(void);
extern bool JacLuDet(void);
extern bool JacMinorDet(void);
extern bool Jacobian(void);
extern bool LuRatio(void);
extern bool Mul(void);
extern bool MulEq(void);
extern bool NearEqualExt(void);
extern bool NumericType(void);
extern bool OdeStiff(void);
extern bool RevOne(void);
extern bool RevTwo(void);
extern bool Sin(void);
extern bool Sinh(void);
extern bool Sqrt(void);
extern bool StackMachine(void);
extern bool Sub(void);
extern bool SubEq(void);
extern bool Tan(void);
extern bool Tanh(void);
extern bool TapeIndex(void);
extern bool UnaryMinus(void);
extern bool UnaryPlus(void);
extern bool Value(void);
extern bool Var2Par(void);
extern bool abort_recording(void);
extern bool acos(void);
extern bool acosh(void);
extern bool ad_assign(void);
extern bool ad_ctor(void);
extern bool ad_fun(void);
extern bool ad_in_c(void);
extern bool ad_input(void);
extern bool ad_output(void);
extern bool asin(void);
extern bool asinh(void);
extern bool atan(void);
extern bool atan2(void);
extern bool atanh(void);
extern bool azmul(void);
extern bool base2ad(void);
extern bool base_require(void);
extern bool capacity_order(void);
extern bool change_param(void);
extern bool check_for_nan(void);
extern bool compare_change(void);
extern bool complex_poly(void);
extern bool con_dyn_var(void);
extern bool eigen_array(void);
extern bool eigen_det(void);
extern bool erf(void);
extern bool erfc(void);
extern bool exp(void);
extern bool expm1(void);
extern bool fabs(void);
extern bool forward_dir(void);
extern bool forward_order(void);
extern bool fun_assign(void);
extern bool fun_property(void);
extern bool function_name(void);
extern bool interp_onetape(void);
extern bool interp_retape(void);
extern bool log(void);
extern bool log10(void);
extern bool log1p(void);
extern bool lu_vec_ad_ok(void);
extern bool mul_level(void);
extern bool mul_level_adolc(void);
extern bool mul_level_adolc_ode(void);
extern bool mul_level_ode(void);
extern bool new_dynamic(void);
extern bool num_limits(void);
extern bool number_skip(void);
extern bool opt_val_hes(void);
extern bool pow(void);
extern bool pow_nan(void);
extern bool print_for(void);
extern bool rev_checkpoint(void);
extern bool reverse_one(void);
extern bool reverse_three(void);
extern bool reverse_two(void);
extern bool sign(void);
extern bool taylor_ode(void);
extern bool vec_ad(void);
// END_SORT_THIS_LINE_MINUS_1

// main program that runs all the tests
int main(void)
{   std::string group = "example/general";
    size_t      width = 20;
    CppAD::test_boolofvoid Run(group, width);

    // This line is used by test_one.sh

    // BEGIN_SORT_THIS_LINE_PLUS_1
    Run( Add,               "Add"              );
    Run( AddEq,             "AddEq"            );
    Run( BenderQuad,        "BenderQuad"       );
    Run( BoolFun,           "BoolFun"          );
    Run( Compare,           "Compare"          );
    Run( CondExp,           "CondExp"          );
    Run( Cos,               "Cos"              );
    Run( Cosh,              "Cosh"             );
    Run( Div,               "Div"              );
    Run( DivEq,             "DivEq"            );
    Run( EqualOpSeq,        "EqualOpSeq"       );
    Run( ForOne,            "ForOne"           );
    Run( ForTwo,            "ForTwo"           );
    Run( Forward,           "Forward"          );
    Run( FunCheck,          "FunCheck"         );
    Run( HesLagrangian,     "HesLagrangian"    );
    Run( HesLuDet,          "HesLuDet"         );
    Run( HesMinorDet,       "HesMinorDet"      );
    Run( HesTimesDir,       "HesTimesDir"      );
    Run( Hessian,           "Hessian"          );
    Run( Independent,       "Independent"      );
    Run( Integer,           "Integer"          );
    Run( Interface2C,       "Interface2C"      );
    Run( JacLuDet,          "JacLuDet"         );
    Run( JacMinorDet,       "JacMinorDet"      );
    Run( Jacobian,          "Jacobian"         );
    Run( LuRatio,           "LuRatio"          );
    Run( Mul,               "Mul"              );
    Run( MulEq,             "MulEq"            );
    Run( NearEqualExt,      "NearEqualExt"     );
    Run( NumericType,       "NumericType"      );
    Run( OdeStiff,          "OdeStiff"         );
    Run( RevOne,            "RevOne"           );
    Run( RevTwo,            "RevTwo"           );
    Run( Sin,               "Sin"              );
    Run( Sinh,              "Sinh"             );
    Run( Sqrt,              "Sqrt"             );
    Run( StackMachine,      "StackMachine"     );
    Run( Sub,               "Sub"              );
    Run( SubEq,             "SubEq"            );
    Run( Tan,               "Tan"              );
    Run( Tanh,              "Tanh"             );
    Run( TapeIndex,         "TapeIndex"        );
    Run( UnaryMinus,        "UnaryMinus"       );
    Run( UnaryPlus,         "UnaryPlus"        );
    Run( Value,             "Value"            );
    Run( Var2Par,           "Var2Par"          );
    Run( abort_recording,   "abort_recording"  );
    Run( acos,              "acos"             );
    Run( acosh,             "acosh"            );
    Run( ad_assign,         "ad_assign"        );
    Run( ad_ctor,           "ad_ctor"          );
    Run( ad_fun,            "ad_fun"           );
    Run( ad_in_c,           "ad_in_c"          );
    Run( ad_input,          "ad_input"         );
    Run( ad_output,         "ad_output"        );
    Run( asin,              "asin"             );
    Run( asinh,             "asinh"            );
    Run( atan,              "atan"             );
    Run( atan2,             "atan2"            );
    Run( atanh,             "atanh"            );
    Run( azmul,             "azmul"            );
    Run( base2ad,           "base2ad"          );
    Run( base_require,      "base_require"     );
    Run( capacity_order,    "capacity_order"   );
    Run( change_param,      "change_param"     );
    Run( compare_change,    "compare_change"   );
    Run( complex_poly,      "complex_poly"     );
    Run( con_dyn_var,       "con_dyn_var"      );
    Run( erf,               "erf"              );
    Run( erfc,              "erfc"             );
    Run( exp,               "exp"              );
    Run( expm1,             "expm1"            );
    Run( fabs,              "fabs"             );
    Run( forward_dir,       "forward_dir"      );
    Run( forward_order,     "forward_order"    );
    Run( fun_assign,        "fun_assign"       );
    Run( fun_property,      "fun_property"     );
    Run( function_name,     "function_name"    );
    Run( interp_onetape,    "interp_onetape"   );
    Run( interp_retape,     "interp_retape"    );
    Run( log,               "log"              );
    Run( log10,             "log10"            );
    Run( log1p,             "log1p"            );
    Run( lu_vec_ad_ok,      "lu_vec_ad_ok"     );
    Run( mul_level,         "mul_level"        );
    Run( mul_level_ode,     "mul_level_ode"    );
    Run( new_dynamic,       "new_dynamic"      );
    Run( num_limits,        "num_limits"       );
    Run( number_skip,       "number_skip"      );
    Run( opt_val_hes,       "opt_val_hes"      );
    Run( pow,               "pow"              );
    Run( pow_nan,           "pow_nan"          );
    Run( rev_checkpoint,    "rev_checkpoint"   );
    Run( reverse_one,       "reverse_one"      );
    Run( reverse_three,     "reverse_three"    );
    Run( reverse_two,       "reverse_two"      );
    Run( sign,              "sign"             );
    Run( taylor_ode,        "ode_taylor"       );
    Run( vec_ad,            "vec_ad"           );
    // END_SORT_THIS_LINE_MINUS_1
# ifndef CPPAD_DEBUG_AND_RELEASE
    Run( check_for_nan,     "check_for_nan"    );
# endif
# if CPPAD_HAS_ADOLC
    Run( mul_level_adolc,      "mul_level_adolc"     );
    Run( mul_level_adolc_ode,  "mul_level_adolc_ode" );
# endif
# if CPPAD_HAS_EIGEN
    Run( eigen_array,       "eigen_array"      );
    Run( eigen_det,         "eigen_det"        );
# endif
    //
    // check for memory leak
    bool memory_ok = CppAD::thread_alloc::free_all();
    // print summary at end
    bool ok = Run.summary(memory_ok);
    //
    return static_cast<int>( ! ok );
}
// END C++
