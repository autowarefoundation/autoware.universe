#ifndef CPPAD_CG_MATH_INCLUDED
#define CPPAD_CG_MATH_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

namespace CppAD {

#define CPPAD_CG_CREATE_OPERATION(OpName, OpCode)                              \
    template<class Base>                                                       \
    inline cg::CG<Base> OpName(const cg::CG<Base>& var) {                      \
        using namespace CppAD::cg;                                             \
        if (var.isParameter()) {                                               \
            return CG<Base> (OpName(var.getValue()));                          \
        } else {                                                               \
            CodeHandler<Base>& h = *var.getOperationNode()->getCodeHandler();  \
            CG<Base> result(*h.makeNode(CGOpCode::OpCode, var.argument()));    \
            if(var.isValueDefined())                                           \
                result.setValue(OpName(var.getValue()));                       \
            return result;                                                     \
        }                                                                      \
    }

CPPAD_CG_CREATE_OPERATION(abs, Abs)
CPPAD_CG_CREATE_OPERATION(fabs, Abs)
CPPAD_CG_CREATE_OPERATION(acos, Acos)
CPPAD_CG_CREATE_OPERATION(asin, Asin)
CPPAD_CG_CREATE_OPERATION(atan, Atan)
CPPAD_CG_CREATE_OPERATION(cos, Cos)
CPPAD_CG_CREATE_OPERATION(cosh, Cosh)
CPPAD_CG_CREATE_OPERATION(exp, Exp)
CPPAD_CG_CREATE_OPERATION(log, Log)
CPPAD_CG_CREATE_OPERATION(sin, Sin)
CPPAD_CG_CREATE_OPERATION(sinh, Sinh)
CPPAD_CG_CREATE_OPERATION(sqrt, Sqrt)
CPPAD_CG_CREATE_OPERATION(tan, Tan)
CPPAD_CG_CREATE_OPERATION(tanh, Tanh)

#if CPPAD_USE_CPLUSPLUS_2011
// c++11 functions
CPPAD_CG_CREATE_OPERATION(erf, Erf)
CPPAD_CG_CREATE_OPERATION(erfc, Erfc)
CPPAD_CG_CREATE_OPERATION(asinh, Asinh)
CPPAD_CG_CREATE_OPERATION(acosh, Acosh)
CPPAD_CG_CREATE_OPERATION(atanh, Atanh)
CPPAD_CG_CREATE_OPERATION(expm1, Expm1)
CPPAD_CG_CREATE_OPERATION(log1p, Log1p)
#endif

template <class Base>
inline cg::CG<Base> log10(const cg::CG<Base> &x) {
    return CppAD::log(x) / CppAD::log(Base(10));
}

namespace cg  {
  
  using CppAD::abs;
  using CppAD::fabs;
  using CppAD::acos;
  using CppAD::asin;
  using CppAD::atan;
  using CppAD::cos;
  using CppAD::cosh;
  using CppAD::exp;
  using CppAD::log;
  using CppAD::sin;
  using CppAD::sinh;
  using CppAD::asin;
  using CppAD::sqrt;
  using CppAD::tan;
  using CppAD::tanh;
  
#if CPPAD_USE_CPLUSPLUS_2011
  using CppAD::erf;
  using CppAD::erfc;
  using CppAD::asinh;
  using CppAD::acosh;
  using CppAD::atanh;
  using CppAD::expm1;
  using CppAD::log1p;
#endif
  
  using CppAD::log10;

} // END cg namespace
} // END CppAD namespace

#endif
