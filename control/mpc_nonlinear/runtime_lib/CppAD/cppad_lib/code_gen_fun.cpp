/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

  CppAD is distributed under the terms of the
               Eclipse Public License Version 2.0.

  This Source Code may also be made available under the following
  Secondary License when the conditions for such availability set forth
  in the Eclipse Public License, Version 2.0 are satisfied:

-------------------------------------------------------------------------- */
/*
$begin code_gen_fun$$
$spell
    CppAD
    cppad
    hpp
    cppadcg
    cg
    eval_jac
    jacobian
    enum
    Jrcv
$$

$section Generate Source Code and Compile an AD Function$$

$head Syntax$$
$codei%# include <cppad/example/code_gen_fun.hpp>
%$$

$subhead Constructors$$
$codei%code_gen_fun %fun_name%()
%$$
$codei%code_gen_fun %fun_name%(%file_name%)
%$$
$codei%code_gen_fun %fun_name%(%file_name%, %cg_fun%)
%$$
$codei%code_gen_fun %fun_name%(%file_name%, %cg_fun%, %eval_jac%)
%$$

$subhead swap$$
$icode%fun_name%.swap(%other_fun%)%$$

$subhead function$$
$icode%y% = %fun_name%(%x%)%$$

$subhead jacobian$$
$icode%J% = %fun_name%.jacobian(%x%)%$$

$subhead sparse_jacobian$$
$icode%Jrcv% = %fun_name%.sparse_jacobian(%x%)%$$


$head Prototype$$

$subhead Constructors$$
$srcthisfile%
    0%// BEGIN_CTOR_VOID%// END_CTOR_VOID%1
%$$
$srcthisfile%
    0%// BEGIN_CTOR_FILE_NAME%// END_CTOR_FILE_NAME%1
%$$
$srcthisfile%
    0%// BEGIN_CTOR_CG_FUN%// END_CTOR_CG_FUN%1
%$$

$subhead Operations$$
$srcthisfile%
    0%// BEGIN_SWAP_OTHER_FUN%// END_SWAP_OTHER_FUN%1
%$$
$srcthisfile%
    0%// BEGIN_FUN_NAME_X%// END_FUN_NAME_X%1
%$$
$srcthisfile%
    0%// BEGIN_JACOBIAN%// END_JACOBIAN%1
%$$
$srcthisfile%
    0%// BEGIN_SPARSE_JACOBIAN%// END_SPARSE_JACOBIAN%1
%$$
$pre
$$

$head CppAD::cg::CG<double>$$
This is the CppAD $icode Base$$ type for the function
$icode cg_fun$$.
It is defined by
$href%https://github.com/joaoleal/CppADCodeGen%CppADCodeGen%$$.
and used to convert the $icode cg_fun$$ function object to source code,
compile the source code, and then link the corresponding function evaluation
$codei%
    %y% = cg_fun.Forward(0, %x%)
%$$

$head Speed$$
The conversion to source and linking is expected to take a significant
amount of time and the evaluation of the function is expected to be
much faster; see the following speed tests:
$table
$rref cppadcg_det_minor.cpp$$
$rref cppadcg_sparse_jacobian.cpp$$
$tend


$head fun_name$$
This is the name of the $code code_gen_fun$$ object.

$head other_fun$$
This is the name of another $code code_gen_fun$$ object.

$head file_name$$
This is the absolute or relative path for the
file that contains the dynamic library.
It does not include the files extension at the end that is used
for dynamic libraries on this system.
If $icode cg_fun$$ is not present in the constructor,
it must have been present in a previous constructor with the same
$icode file_name$$.

$head cg_fun$$
This is a CppAD function object that corresponds to a function
$latex f : \B{R}^n \rightarrow \B{R}^m$$.
If this arguments is present in the constructor,
a new dynamic library is created.

$head eval_jac$$
If this argument is present in the constructor,
it determines which type of Jacobian $latex f'(x)$$ will be enabled.
The possible choices for $icode eval_jac$$ are:
$table
$icode eval_jac$$                 $pre  $$ $cnext Available Jacobian
$rnext
$code code_gen_fun::none_enum$$   $pre  $$ $cnext none
$rnext
$code code_gen_fun::dense_enum$$  $pre  $$ $cnext $icode%fun_name%.jacobian%$$
$tend
The default value for $icode eval_jac$$ is none.

$head swap$$
This exchanges the library in $icode fun_name$$ with the library in
$icode other_fun$$.

$head x$$
is a vector of size $icode n$$ specifying the argument value
at which the function will be evaluated.

$head y$$
This return value has size $icode m$$ and is the value of $latex f(x)$$.

$head jacobian$$

$subhead J$$
This return value has size $icode%m% * %n%$$ and is the value of
the Jacobian $latex f'(x)$$ where
$latex \[
    J[ i \cdot n + j ] =  ( \partial f_i / \partial x_j )  (x)
\] $$

$subhead Speed$$
The speed test $cref cppadcg_det_minor.cpp$$ has the option to pass
the determinant function, or the Jacobian of the determinant function,
to CppADCodeGen (for the same eventual calculation); see
$cref/PASS_JACOBIAN_TO_CODE_GEN
    /cppadcg_det_minor.cpp
    /PASS_JACOBIAN_TO_CODE_GEN
/$$.
This test indicates that both methods have similar setup
and derivative calculation times.

$head sparse_jacobian$$

$head Jrcv$$
This return value is a $cref sparse_rcv$$ sparse matrix representation
of the Jacobian.

$subhead Speed$$
The speed test $cref cppadcg_sparse_jacobian.cpp$$ has the option to pass
a function ($cref sparse_jac_fun$$) or it's Jacobian to CppADCodeGen
(for the same eventual calculation); see
$cref/PASS_SPARSE_JACOBIAN_TO_CODE_GEN
    /cppadcg_sparse_jacobian.cpp
    /PASS_SPARSE_JACOBIAN_TO_CODE_GEN
/$$.
THis test indicates that both methods have similar setup
and derivative calculation times.

$children%
    example/code_gen_fun/function.cpp%
    example/code_gen_fun/file.cpp%
    example/code_gen_fun/jacobian.cpp%
    example/code_gen_fun/jac_as_fun.cpp%
    example/code_gen_fun/sparse_jacobian.cpp%
    example/code_gen_fun/sparse_jac_as_fun.cpp
%$$
$head Examples$$
$table
$rref code_gen_fun_function.cpp$$
$rref code_gen_fun_file.cpp$$
$rref code_gen_fun_jacobian.cpp$$
$rref code_gen_fun_jac_as_fun.cpp$$
$rref code_gen_fun_sparse_jacobian.cpp$$
$rref code_gen_fun_sparse_jac_as_fun.cpp$$
$tend

$head Implementation$$
see $cref code_gen_fun.hpp$$ and $cref code_gen_fun.cpp$$

$end
-----------------------------------------------------------------------------
$begin code_gen_fun.hpp$$

$section code_gen_fun Class Include File$$

$head See Also$$
$cref code_gen_fun$$, $cref code_gen_fun.cpp$$

$head Source$$
$srcfile%include/cppad/example/code_gen_fun.hpp%0%// BEGIN C++%// END C++%$$

$end
-----------------------------------------------------------------------------
$begin code_gen_fun.cpp$$

$section code_gen_fun Class Member  Implementation$$

$head See Also$$
$cref code_gen_fun$$, $cref code_gen_fun.hpp$$

$head Source$$
$srcthisfile%0%// BEGIN C++%// END C++%2%$$

$end
*/
// BEGIN C++
# include <cppad/example/code_gen_fun.hpp>

// ---------------------------------------------------------------------------
// code_gen_fun fun_name(file_name, cg_name, eval_jac)
// ---------------------------------------------------------------------------
// BEGIN_CTOR_CG_FUN
code_gen_fun::code_gen_fun(
    const std::string&                     file_name  ,
    CppAD::ADFun< CppAD::cg::CG<double> >& cg_fun     ,
    evaluation_enum                        eval_jac   )
// END_CTOR_CG_FUN
{   // Generate source code
    CppAD::cg::ModelCSourceGen<double> cgen(cg_fun, "model");
    switch(eval_jac)
    {   case none_enum:
        break;

        case dense_enum:
        cgen.setCreateJacobian(true);
        break;

        case sparse_enum:
        cgen.setCreateSparseJacobian(true);
        break;
    }
    CppAD::cg::ModelLibraryCSourceGen<double> libcgen(cgen);

    // Compile source, create the library file, and load the library
    CppAD::cg::DynamicModelLibraryProcessor<double> proc(libcgen, file_name);
    CppAD::cg::ClangCompiler<double> compiler;
    bool loadLib = true;
    dynamic_lib_ = proc.createDynamicLibrary(compiler, loadLib);
    //
    // create the model object
    model_        = dynamic_lib_->model("model");
}
// ---------------------------------------------------------------------------
// code_gen_fun fun_name(file_name)
// ---------------------------------------------------------------------------
// BEGIN_CTOR_FILE_NAME
code_gen_fun::code_gen_fun(const std::string&  file_name )
// END_CTOR_FILE_NAME
{   // file name plus extension used for dynamic libraries on this system
    std::string file_name_ext = file_name +
        CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;

    // load the library
    CppAD::cg::DynamicLib<double>* ptr =
        new CppAD::cg::LinuxDynamicLib<double>(file_name_ext);
    dynamic_lib_  = std::unique_ptr< CppAD::cg::DynamicLib<double> >(ptr);
    //
    // create the model object
    model_        = dynamic_lib_->model("model");
}
// ---------------------------------------------------------------------------
// code_gen_fun fun_name
// ---------------------------------------------------------------------------
// BEGIN_CTOR_VOID
code_gen_fun::code_gen_fun(void)
// END_CTOR_VOID
{ }
// --------------------------------------------------------------------------
// fun_name.swap(other_fun)
// --------------------------------------------------------------------------
// BEGIN_SWAP_OTHER_FUN
void code_gen_fun::swap(code_gen_fun& other_fun)
// END_SWAP_OTHER_FUN
{   std::swap(dynamic_lib_, other_fun.dynamic_lib_);
    std::swap(model_, other_fun.model_ );
}
// --------------------------------------------------------------------------
// y = fun_name(x)
// --------------------------------------------------------------------------
// BEGIN_FUN_NAME_X
CppAD::vector<double>
code_gen_fun::operator()(const CppAD::vector<double>& x)
// END_FUN_NAME_X
{   return model_->ForwardZero(x);
}
// --------------------------------------------------------------------------
// J = fun_name.jacobian(x)
// --------------------------------------------------------------------------
// BEGIN_JACOBIAN
CppAD::vector<double>
code_gen_fun::jacobian(const CppAD::vector<double>& x)
// END_JACOBIAN
{   CPPAD_ASSERT_KNOWN( model_->isJacobianAvailable() ,
        "code_gen_fun: dense jacobian not enables during constructor"
    );
    return model_-> Jacobian(x);
}
// --------------------------------------------------------------------------
// Jrcv = fun_name.sparse_jacobian(x)
// --------------------------------------------------------------------------
// BEGIN_SPARSE_JACOBIAN
CppAD::sparse_rcv< CppAD::vector<size_t>, CppAD::vector<double> >
code_gen_fun::sparse_jacobian(const CppAD::vector<double>& x)
// END_SPARSE_JACOBIAN
{   CPPAD_ASSERT_KNOWN( model_->isSparseJacobianAvailable() ,
        "code_gen_fun: sparse jacobian not enabled during constructor"
    );
    // x_std
    size_t n = model_->Domain();
    std::vector<double> x_std(n);
    for(size_t j = 0; j < n; ++j)
        x_std[j] = x[j];
    //
    // 2DO: Prepahs CppAD should have a sparse_rcv constructor (jac, row, col)
    // that uses swap to swap the vectors
    //
    // jac, row, col
    std::vector<double> jac;
    std::vector<size_t> row, col;
    model_-> SparseJacobian(x_std, jac, row, col);
    //
    // sparse_rc
    size_t nr  = model_->Range();
    size_t nc  = model_->Domain();
    size_t nnz = row.size();
    CppAD::sparse_rc< CppAD::vector<size_t> > pattern(nr, nc, nnz);
    for(size_t k = 0; k < nnz; ++k)
        pattern.set(k, row[k], col[k]);
    // sparse_rcv
    CppAD::sparse_rcv< CppAD::vector<size_t>, CppAD::vector<double> >
    Jrcv(pattern);
    for(size_t k = 0; k < nnz; ++k)
        Jrcv.set(k, jac[k]);
    //
    return Jrcv;
}
// END C++
