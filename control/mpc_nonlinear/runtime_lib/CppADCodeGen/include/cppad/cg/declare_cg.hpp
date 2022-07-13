#ifndef CPPAD_CG_DECLARE_CG_INCLUDED
#define CPPAD_CG_DECLARE_CG_INCLUDED
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

// forward declarations
namespace CppAD {

template<class Base>
class vector;

template<class Base>
class AD;

template<class Base, class RecBase>
class ADFun;

namespace cg {

/***************************************************************************
 * Atomics
 **************************************************************************/
template<class Base>
class BaseAbstractAtomicFun;

template<class Base>
class CGAbstractAtomicFun;

template<class Base>
class CGAtomicFun;

template<class Base>
class AtomicUseInfo;

/***************************************************************************
 * Core
 **************************************************************************/
template<class Base>
class CodeHandler;

template<class Base>
class CodeHandlerVectorSync;

template<class Base, class T>
class CodeHandlerVector;

template<class Base>
class CG;

template<class Base>
struct OperationPathNode;

template<class Base>
class PathNodeEdges;

template<class Base>
class BidirGraph;

template<class Base>
class ScopePathElement;

/***************************************************************************
 * Nodes
 **************************************************************************/
template<class Base>
class OperationNode;

template<class Base>
class IndexOperationNode;

template<class Base>
class IndexAssignOperationNode;

template<class Base>
class LoopStartOperationNode;

template<class Base>
class LoopEndOperationNode;

/***************************************************************************
 * Loops
 **************************************************************************/
template<class Base>
class EquationPattern;

template<class Base>
class DependentPatternMatcher;

template<class Base>
class Loop;

template<class Base>
class LoopFreeModel;

template<class Base>
class LoopModel;

template<class Base>
class IndexedDependentLoopInfo;

class IndexPattern;
class LinearIndexPattern;
class Plane2DIndexPattern;
class RandomIndexPattern;
class SectionedIndexPattern;

/***************************************************************************
 * Languages
 **************************************************************************/
template<class Base>
class LangStreamOperation;

template<class Base>
class LanguageC;

template<class Base>
class VariableNameGenerator;

template<class Base>
class LangCDefaultVariableNameGenerator;

template<class Base>
class LangCCustomVariableNameGenerator;

/***************************************************************************
 * Models
 **************************************************************************/
template<class Base>
class GenericModel;

template<class Base>
class ModelLibraryProcessor;

template<class Base>
class FunctorGenericModel;

/***************************************************************************
 * Dynamic model compilation
 **************************************************************************/

template<class Base>
class CCompiler;

template<class Base>
class DynamicLib;

template<class Base>
class ModelCSourceGen;

template<class Base>
class ModelLibraryCSourceGen;

#if CPPAD_CG_SYSTEM_LINUX
template<class Base>
class LinuxDynamicLibModel;

template<class Base>
class LinuxDynamicLib;
#endif

/***************************************************************************
 * Index reduction classes
 **************************************************************************/
template<class Base>
class Enode;

template<class Base>
class Vnode;

template<class ScalarIn, class ScalarOut, class ActiveOut>
class Evaluator;

/***************************************************************************
 *  Utilities
 **************************************************************************/

template<class Base>
class SmartVectorPointer;

template<class Base>
class SmartListPointer;

template<class Key, class Value>
class SmartMapValuePointer;

template<class Type>
class ArrayView;

template<class Base>
inline void print(const Base& v);

template<class Key, class Value>
inline void print(const std::map<Key, Value>& m);

template<class Base>
inline void print(const std::set<Base>& s);

template<class Base>
inline void print(const std::set<Base*>& s);

template<class Base>
inline void print(const std::vector<Base>& v);

/**
 * arithmetic
 */
template<class Base>
CG<Base> operator+(const CG<Base>& left, const CG<Base>& right);

template<class Base>
CG<Base> operator-(const CG<Base>& left, const CG<Base>& right);

template<class Base>
CG<Base> operator*(const CG<Base>& left, const CG<Base>& right);

template<class Base>
CG<Base> operator/(const CG<Base>& left, const CG<Base>& right);

/**
 * comparisons
 */
template<class Base>
bool operator==(const CG<Base>& left, const CG<Base>& right);

template<class Base>
bool operator!=(const CG<Base>& left, const CG<Base>& right);

template<class Base>
bool operator<(const CG<Base>& left, const CG<Base>& right);

template<class Base>
bool operator<=(const CG<Base>& left, const CG<Base>& right);

template<class Base>
bool operator>(const CG<Base>& left, const CG<Base>& right);

template<class Base>
bool operator>=(const CG<Base>& left, const CG<Base>& right);

template<class Base>
bool operator!=(const CG<Base>& left, double right);

template<class Base>
bool operator==(const CG<Base>& left, const Base& right);

template<class Base>
bool operator==(const Base& left, const CG<Base>& right);

template<class Base>
bool operator!=(const CG<Base>& left, Base right);

template<class Base>
bool operator!=(const Base& left, const CG<Base>& right);

/***************************************************************************
 * Index reduction functions
 **************************************************************************/

template<class Base>
inline std::ostream& operator<<(std::ostream& os, const Enode<Base>& i);

template<class Base>
inline std::ostream& operator<<(std::ostream& os, const Vnode<Base>& j);

/***************************************************************************
 * Enums
 **************************************************************************/

/**
 * Verbosity level for print-outs
 */
enum class Verbosity {
    None, Low, High
};

/**
 * Automatic Differentiation modes used to determine the Jacobian
 */
enum class JacobianADMode {
    Forward, Reverse, Automatic
};

/**
 * Index pattern types
 */
enum class IndexPatternType {
    Linear, // y = (x / dx) * dy + b
    Sectioned, // several index patterns
    Random1D,
    Random2D,
    Plane2D // y = f(x) + f(z)
};

} // END cg namespace

/***************************************************************************
 *
 **************************************************************************/
// order determining functions, see ordered.hpp
template<class Base>
bool GreaterThanZero(const cg::CG<Base>& x);

template<class Base>
bool GreaterThanOrZero(const cg::CG<Base>& x);

template<class Base>
bool LessThanZero(const cg::CG<Base>& x);

template<class Base>
bool LessThanOrZero(const cg::CG<Base>& x);

template<class Base>
bool abs_geq(const cg::CG<Base>& x, const cg::CG<Base>& y);

// The identical property functions, see identical.hpp
/**
 * @throws CppAD::cg::CGException
 */
template<class Base>
inline bool IdenticalPar(const cg::CG<Base>& x);

/**
 * @throws CppAD::cg::CGException
 */
template<class Base>
bool IdenticalZero(const cg::CG<Base>& x);

/**
 * @throws CppAD::cg::CGException
 */
template<class Base>
bool IdenticalOne(const cg::CG<Base>& x);

template<class Base>
bool IdenticalEqualPar(const cg::CG<Base>& x, const cg::CG<Base>& y);

// EqualOpSeq function
template<class Base>
bool EqualOpSeq(const cg::CG<Base>& u, const cg::CG<Base>& v);

// NearEqual function
template<class Base>
bool NearEqual(const cg::CG<Base>& x, const cg::CG<Base>& y, const Base& r, const Base& a);

template<class Base>
bool NearEqual(const Base& x, const cg::CG<Base>& y, const Base& r, const Base& a);

template<class Base>
bool NearEqual(const cg::CG<Base>& x, const Base& y, const Base& r, const Base& a);

template <class Base>
inline bool isnan(const cg::CG<Base>& s);

template <class Base>
int Integer(const cg::CG<Base>& x);

template<class Base>
cg::CG<Base> CondExp(cg::CGOpCode op,
                     const cg::CG<Base>& left, const cg::CG<Base>& right,
                     const cg::CG<Base>& trueCase, const cg::CG<Base>& falseCase,
                     bool (*compare)(const Base&, const Base&));

/**
 * Math functions
 */
template<class Base>
inline cg::CG<Base> sign(const cg::CG<Base>& x);

// power function
template<class Base>
inline cg::CG<Base> pow(const cg::CG<Base>& x, const cg::CG<Base>& y);
template <class Base>
inline cg::CG<Base> pow(const Base& x, const cg::CG<Base>& y);
template <class Base>
inline cg::CG<Base> pow(const cg::CG<Base>& x, const Base& y);

// absolute value
template<class Base>
inline cg::CG<Base> abs(const cg::CG<Base>& x);

template<class Base>
inline cg::CG<Base> fabs(const cg::CG<Base>& x);

// inverse cosine
template<class Base>
inline cg::CG<Base> acos(const cg::CG<Base>& x);

// inverse sine
template<class Base>
inline cg::CG<Base> asin(const cg::CG<Base>& x);

// inverse tangent
template<class Base>
inline cg::CG<Base> atan(const cg::CG<Base>& x);

// cosine
template<class Base>
inline cg::CG<Base> cos(const cg::CG<Base>& x);

// hyperbolic cosine
template<class Base>
inline cg::CG<Base> cosh(const cg::CG<Base>& x);

// exponential
template<class Base>
inline cg::CG<Base> exp(const cg::CG<Base>& x);

// natural logarithm
template<class Base>
inline cg::CG<Base> log(const cg::CG<Base>& x);

// sine
template<class Base>
inline cg::CG<Base> sin(const cg::CG<Base>& x);

// hyperbolic sine
template<class Base>
inline cg::CG<Base> sinh(const cg::CG<Base>& x);

// square root
template<class Base>
inline cg::CG<Base> sqrt(const cg::CG<Base>& x);

// tangent
template<class Base>
inline cg::CG<Base> tan(const cg::CG<Base>& x);

// hyperbolic tangent
template<class Base>
inline cg::CG<Base> tanh(const cg::CG<Base>& x);

#if CPPAD_USE_CPLUSPLUS_2011
/**
 * c++11 functions
 */
// error function
template<class Base>
inline cg::CG<Base> erf(const cg::CG<Base>& x);

// complementary error function
template<class Base>
inline cg::CG<Base> erfc(const cg::CG<Base>& var);

// inverse hyperbolic sin
template<class Base>
inline cg::CG<Base> asinh(const cg::CG<Base>& x);

// inverse hyperbolic cosine
template<class Base>
inline cg::CG<Base> acosh(const cg::CG<Base>& x);

// inverse hyperbolic tangent
template<class Base>
inline cg::CG<Base> atanh(const cg::CG<Base>& x);

// exponential of x minus one
template<class Base>
inline cg::CG<Base> expm1(const cg::CG<Base>& x);

// logarithm of one plus x
template<class Base>
inline cg::CG<Base> log1p(const cg::CG<Base>& x);
#endif

} // END CppAD namespace

/**
 * loops namespace
 */
#include <cppad/cg/declare_cg_loops.hpp>

#endif

