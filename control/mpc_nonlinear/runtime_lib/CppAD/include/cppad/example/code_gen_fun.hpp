# ifndef CPPAD_EXAMPLE_CODE_GEN_FUN_HPP
# define CPPAD_EXAMPLE_CODE_GEN_FUN_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

  CppAD is distributed under the terms of the
               Eclipse Public License Version 2.0.

  This Source Code may also be made available under the following
  Secondary License when the conditions for such availability set forth
  in the Eclipse Public License, Version 2.0 are satisfied:
        GNU General Public License, Version 2.0 or later.
-------------------------------------------------------------------------- */
// BEGIN C++
# include <cppad/cg/cppadcg.hpp>

// See https://docs.microsoft.com/en-us/cpp/cpp/
//      using-dllimport-and-dllexport-in-cpp-classes?view=msvc-160
// Also see define.hpp where CPPAD_LIB_EXPORTS is also defined and
// undef.hpp where it gets undefined.
# ifdef  _MSC_VER
# ifdef  cppad_lib_EXPORTS
# define CPPAD_LIB_EXPORT __declspec(dllexport)
# else
# define CPPAD_LIB_EXPORT __declspec(dllimport)
# endif  // cppad_lib_EXPORTS
# else   // _MSC_VER
# define CPPAD_LIB_EXPORT
# endif

class CPPAD_LIB_EXPORT code_gen_fun {
public:
    // type of evaluation for Jacobians (possibly Hessians in the future)
    enum evaluation_enum { none_enum, dense_enum, sparse_enum };
private:
    // dynamic_lib_
    std::unique_ptr< CppAD::cg::DynamicLib<double> > dynamic_lib_;
    //
    // model_ (contains a reference to dynamic_lib_)
    std::unique_ptr< CppAD::cg::GenericModel<double> > model_;
    //
public:
    // -----------------------------------------------------------------------
    // constructors
    // -----------------------------------------------------------------------
    // fun_name()
    code_gen_fun(void);
    //
    // fun_name( file_name )
    code_gen_fun(const std::string& file_name);
    //
    // fun_name(file_name, cg_fun, eval_jac)
    code_gen_fun(
        const std::string&                     file_name             ,
        CppAD::ADFun< CppAD::cg::CG<double> >& cg_fun                ,
        evaluation_enum                        eval_jac = none_enum
    );
    // -----------------------------------------------------------------------
    // operations
    // -----------------------------------------------------------------------
    // swap(other_fun)
    void swap(code_gen_fun& other_fun);
    //
    // y = fun_name(x)
    CppAD::vector<double>  operator()(const CppAD::vector<double> & x);
    //
    // J = fun_name.jacobian(x)
    CppAD::vector<double>  jacobian(const CppAD::vector<double> & x);
    //
    // Jrcv = fun_name.sparse_jacobian(x)
    CppAD::sparse_rcv< CppAD::vector<size_t>, CppAD::vector<double> >
    sparse_jacobian(const CppAD::vector<double>& x);
};
// END C++

# endif
