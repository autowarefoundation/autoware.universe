/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// CPPAD_HAS_* defines
# include <cppad/configure.hpp>

// system include files used for I/O
# include <iostream>

// memory leak checker
# include <cppad/utility/thread_alloc.hpp>

// test runner
# include <cppad/utility/test_boolofvoid.hpp>

// BEGIN_SORT_THIS_LINE_PLUS_1
extern bool abs_normal(void);
extern bool acosh(void);
extern bool acos(void);
extern bool AddEq(void);
extern bool Add(void);
extern bool AddZero(void);
extern bool adfun(void);
extern bool alloc_openmp(void);
extern bool asinh(void);
extern bool asin(void);
extern bool assign(void);
extern bool atan2(void);
extern bool atanh(void);
extern bool atan(void);
extern bool atomic_three(void);
extern bool azmul(void);
extern bool base_adolc(void);
extern bool base_alloc_test(void);
extern bool bool_sparsity(void);
extern bool check_simple_vector(void);
extern bool chkpoint_one(void);
extern bool chkpoint_two(void);
extern bool compare_change(void);
extern bool Compare(void);
extern bool CondExpAD(void);
extern bool cond_exp_rev(void);
extern bool CondExp(void);
extern bool copy(void);
extern bool Cosh(void);
extern bool Cos(void);
extern bool cppad_eigen(void);
extern bool cppad_vector(void);
extern bool dbl_epsilon(void);
extern bool dependency(void);
extern bool DivEq(void);
extern bool Div(void);
extern bool DivZeroOne(void);
extern bool eigen_mat_inv(void);
extern bool erf(void);
extern bool expm1(void);
extern bool Exp(void);
extern bool fabs(void);
extern bool ForHess(void);
extern bool for_sparse_hes(void);
extern bool for_sparse_jac(void);
extern bool forward_dir(void);
extern bool forward_order(void);
extern bool Forward(void);
extern bool FromBase(void);
extern bool FunCheck(void);
extern bool cpp_graph(void);
extern bool hes_sparsity(void);
extern bool ipopt_solve(void);
extern bool jacobian(void);
extern bool json_graph(void);
extern bool log10(void);
extern bool log1p(void);
extern bool log(void);
extern bool mul_cond_rev(void);
extern bool mul_cskip(void);
extern bool MulEq(void);
extern bool mul_level(void);
extern bool Mul(void);
extern bool mul_zdouble(void);
extern bool MulZeroOne(void);
extern bool NearEqualExt(void);
extern bool Neg(void);
extern bool new_dynamic(void);
extern bool num_limits(void);
extern bool ode_err_control(void);
extern bool optimize(void);
extern bool parameter(void);
extern bool Poly(void);
extern bool PowInt(void);
extern bool Pow(void);
extern bool print_for(void);
extern bool reverse(void);
extern bool rev_sparse_jac(void);
extern bool RevTwo(void);
extern bool RombergOne(void);
extern bool Rosen34(void);
extern bool Runge45(void);
extern bool SimpleVector(void);
extern bool SinCos(void);
extern bool Sinh(void);
extern bool Sin(void);
extern bool sparse_hessian(void);
extern bool sparse_jacobian(void);
extern bool sparse_jac_work(void);
extern bool sparse_sub_hes(void);
extern bool sparse_vec_ad(void);
extern bool Sqrt(void);
extern bool std_math(void);
extern bool SubEq(void);
extern bool subgraph_1(void);
extern bool subgraph_2(void);
extern bool subgraph_hes2jac(void);
extern bool Sub(void);
extern bool SubZero(void);
extern bool tan(void);
extern bool test_vector(void);
extern bool to_string(void);
extern bool Value(void);
extern bool VecADPar(void);
extern bool VecAD(void);
extern bool VecUnary(void);
// END_SORT_THIS_LINE_MINUS_1

// tests in local subdirectory
extern bool is_pod(void);
extern bool json_lexer(void);
extern bool json_parser(void);
extern bool vector_set(void);

// main program that runs all the tests
int main(void)
{   std::string group = "test_more/general";
    size_t      width = 20;
    CppAD::test_boolofvoid Run(group, width);

    // This line is used by test_one.sh

    // BEGIN_SORT_THIS_LINE_PLUS_1
    Run( acos,            "acos"           );
    Run( abs_normal,      "abs_normal"     );
    Run( acosh,           "acosh"          );
    Run( Add,             "Add"            );
    Run( AddEq,           "AddEq"          );
    Run( AddZero,         "AddZero"        );
    Run( adfun,           "adfun"          );
    Run( asin,            "asin"           );
    Run( asinh,           "asinh"          );
    Run( assign,          "assign"         );
    Run( atan2,           "atan2"          );
    Run( atan,            "atan"           );
    Run( atanh,           "atanh"          );
    Run( atomic_three,    "atomic_three"   );
    Run( azmul,           "azmul"          );
    Run( bool_sparsity,   "bool_sparsity"  );
    Run( check_simple_vector, "check_simple_vector" );
    Run( chkpoint_one,    "chkpoint_one"   );
    Run( chkpoint_two,    "chkpoint_two"   );
    Run( compare_change,  "compare_change" );
    Run( Compare,         "Compare"        );
    Run( CondExpAD,       "CondExpAD"      );
    Run( CondExp,         "CondExp"        );
    Run( cond_exp_rev,    "cond_exp_rev"   );
    Run( copy,            "copy"           );
    Run( Cos,             "Cos"            );
    Run( Cosh,            "Cosh"           );
    Run( cppad_vector,    "cppad_vector"   );
    Run( dbl_epsilon,     "dbl_epsilon"    );
    Run( dependency,      "dependency"     );
    Run( Div,             "Div"            );
    Run( DivEq,           "DivEq"          );
    Run( DivZeroOne,      "DivZeroOne"     );
    Run( erf,             "erf"            );
    Run( Exp,             "Exp"            );
    Run( expm1,           "expm1"          );
    Run( fabs,            "fabs"           );
    Run( ForHess,         "ForHess"        );
    Run( for_sparse_hes,  "for_sparse_hes" );
    Run( for_sparse_jac,  "for_sparse_jac" );
    Run( forward_dir,     "forward_dir"    );
    Run( Forward,         "Forward"        );
    Run( forward_order,   "forward_order"  );
    Run( FromBase,        "FromBase"       );
    Run( FunCheck,        "FunCheck"       );
    Run( cpp_graph,       "cpp_graph"      );
    Run( hes_sparsity,    "hes_sparsity"   );
    Run( jacobian,        "jacobian"       );
    Run( json_graph,      "json_graph"     );
    Run( log10,           "log10"          );
    Run( log1p,           "log1p"          );
    Run( log,             "log"            );
    Run( mul_cond_rev,    "mul_cond_rev"   );
    Run( mul_cskip,       "Mul_cskip"      );
    Run( MulEq,           "MulEq"          );
    Run( mul_level,       "mul_level"      );
    Run( Mul,             "Mul"            );
    Run( mul_zdouble,     "mul_zdouble"    );
    Run( MulZeroOne,      "MulZeroOne"     );
    Run( NearEqualExt,    "NearEqualExt"   );
    Run( Neg,             "Neg"            );
    Run( new_dynamic,     "new_dynamic"    );
    Run( num_limits,      "num_limits"     );
    Run( ode_err_control, "ode_err_control");
    Run( optimize,        "optimize"       );
    Run( parameter,       "parameter"      );
    Run( Poly,            "Poly"           );
    Run( PowInt,          "PowInt"         );
    Run( Pow,             "Pow"            );
    Run( print_for,       "print_for"      );
    Run( reverse,         "reverse"        );
    Run( rev_sparse_jac,  "rev_sparse_jac" );
    Run( RevTwo,          "RevTwo"         );
    Run( RombergOne,      "RombergOne"     );
    Run( Rosen34,         "Rosen34"        );
    Run( Runge45,         "Runge45"        );
    Run( SimpleVector,    "SimpleVector"   );
    Run( SinCos,          "SinCos"         );
    Run( Sinh,            "Sinh"           );
    Run( Sin,             "Sin"            );
    Run( sparse_hessian,  "sparse_hessian" );
    Run( sparse_jacobian, "sparse_jacobian");
    Run( sparse_jac_work, "sparse_jac_work");
    Run( sparse_sub_hes,  "sparse_sub_hes" );
    Run( sparse_vec_ad,   "sparse_vec_ad"  );
    Run( Sqrt,            "Sqrt"           );
    Run( std_math,        "std_math"       );
    Run( SubEq,           "SubEq"          );
    Run( subgraph_1,      "subgraph_1"     );
    Run( subgraph_2,      "subgraph_2"     );
    Run( subgraph_hes2jac, "subgraph_hes2jac" );
    Run( Sub,             "Sub"            );
    Run( SubZero,         "SubZero"        );
    Run( tan,             "tan"            );
    Run( to_string,       "to_string"      );
    Run( Value,           "Value"          );
    Run( VecADPar,        "VecADPar"       );
    Run( VecAD,           "VecAD"          );
    Run( VecUnary,        "VecUnary"       );
    // END_SORT_THIS_LINE_MINUS_1
#if CPPAD_HAS_ADOLC
    Run( base_adolc,      "base_adolc"     );
# endif
#if CPPAD_HAS_IPOPT
    Run( ipopt_solve,     "ipopt_solve"    );
# endif
# ifdef CPPAD_OPENMP_TEST
    Run( alloc_openmp,    "alloc_openmp"   );
# endif
# if CPPAD_HAS_EIGEN
    Run( cppad_eigen,     "cppad_eigen"    );
    Run( eigen_mat_inv,   "eigen_mat_inv"  );
# endif
# if ! CPPAD_EIGENVECTOR
    Run( test_vector,     "test_vector"    );
# endif
    // local sub-directory
    Run( is_pod,         "is_pod"          );
    Run( json_lexer,     "json_lexer"      );
    Run( json_parser,    "json_parser"     );
    Run( vector_set,      "vector_set"     );
    //
    // check for memory leak
    bool memory_ok = CppAD::thread_alloc::free_all();
    //
    // Run base_alloc after memory leak check because base_alloc.hpp uses
    // thread_alloc to allocate memory for static copies of nan.
    Run( base_alloc_test,  "base_alloc"    );
    //
    // print summary at end
    bool ok = Run.summary(memory_ok);
    //
    return static_cast<int>( ! ok );
}
// END PROGRAM
