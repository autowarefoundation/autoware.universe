# ifndef CPPAD_EXAMPLE_CPPAD_EIGEN_HPP
# define CPPAD_EXAMPLE_CPPAD_EIGEN_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-22 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin cppad_eigen.hpp$$
$spell
    impl
    typename
    Real Real
    inline
    neg
    eps
    atan
    Num
    acos
    asin
    CppAD
    std::numeric
    enum
    Mul
    Eigen
    cppad.hpp
    namespace
    struct
    typedef
    const
    imag
    sqrt
    exp
    cos
    plugin
$$
$section Enable Use of Eigen Linear Algebra Package with CppAD$$

$head Syntax$$
$codei%# include <cppad/example/cppad_eigen.hpp>
%$$
$children%
    include/cppad/example/eigen_plugin.hpp%
    example/general/eigen_array.cpp%
    example/general/eigen_det.cpp
%$$

$head Purpose$$
Enables the use of the $cref eigen$$
linear algebra package with the type $icode%AD<%Base%>%$$; see
$href%
    https://eigen.tuxfamily.org/dox/TopicCustomizing_CustomScalar.html%
    custom scalar types
%$$.

$head Example$$
The files $cref eigen_array.cpp$$ and $cref eigen_det.cpp$$
contain an example and test of this include file.
They return true if they succeed and false otherwise.

$head CppAD Declarations$$
First declare some items that are defined by cppad.hpp:
$srccode%cpp% */
namespace CppAD {
    // AD<Base>
    template <class Base> class AD;
    // numeric_limits<Float>
    template <class Float>  class numeric_limits;
}
/* %$$

$head std Declarations$$
Next declare some template specializations in std namespace:
$srccode%cpp% */
namespace std {
    template <class Base> bool isinf(const CppAD::AD<Base> &x);
    template <class Base> bool isfinite(const CppAD::AD<Base> &x);
    template <class Base> bool isnan(const CppAD::AD<Base> &x);
}
/* %$$

$head Include Eigen/Core$$
Next define the eigen plugin and then include Eigen/Core:
$srccode%cpp% */

# define EIGEN_MATRIXBASE_PLUGIN <cppad/example/eigen_plugin.hpp>
# include <Eigen/Core>
/* %$$

$head Eigen NumTraits$$
Eigen needs the following definitions, in the Eigen namespace,
to work properly with $codei%AD<%Base%>%$$ scalars:
$srccode%cpp% */
namespace Eigen {
    template <class Base> struct NumTraits< CppAD::AD<Base> >
    {   // type that corresponds to the real part of an AD<Base> value
        typedef CppAD::AD<Base>   Real;
        // type for AD<Base> operations that result in non-integer values
        typedef CppAD::AD<Base>   NonInteger;
        //  type to use for numeric literals such as "2" or "0.5".
        typedef CppAD::AD<Base>   Literal;
        // type for nested value inside an AD<Base> expression tree
        typedef CppAD::AD<Base>   Nested;

        enum {
            // does not support complex Base types
            IsComplex             = 0 ,
            // does not support integer Base types
            IsInteger             = 0 ,
            // only support signed Base types
            IsSigned              = 1 ,
            // must initialize an AD<Base> object
            RequireInitialization = 1 ,
            // computational cost of the corresponding operations
            ReadCost              = 1 ,
            AddCost               = 2 ,
            MulCost               = 2
        };

        // machine epsilon with type of real part of x
        // (use assumption that Base is not complex)
        static CppAD::AD<Base> epsilon(void)
        {   return CppAD::numeric_limits< CppAD::AD<Base> >::epsilon(); }

        // relaxed version of machine epsilon for comparison of different
        // operations that should result in the same value
        static CppAD::AD<Base> dummy_precision(void)
        {   return 100. *
                CppAD::numeric_limits< CppAD::AD<Base> >::epsilon();
        }

        // minimum normalized positive value
        static CppAD::AD<Base> lowest(void)
        {   return CppAD::numeric_limits< CppAD::AD<Base> >::min(); }

        // maximum finite value
        static CppAD::AD<Base> highest(void)
        {   return CppAD::numeric_limits< CppAD::AD<Base> >::max(); }

        // number of decimal digits that can be represented without change.
        static int digits10(void)
        {   return CppAD::numeric_limits< CppAD::AD<Base> >::digits10; }

        // not a number
        static CppAD::AD<Base> quiet_NaN(void)
        {   return CppAD::numeric_limits< CppAD::AD<Base> >::quiet_NaN(); }

        // positive infinite value
        static CppAD::AD<Base> infinity(void)
        {   return CppAD::numeric_limits< CppAD::AD<Base> >::infinity(); }
    };
}
/* %$$

$head CppAD Namespace$$
Eigen needs the following definitions, in the CppAD namespace,
to work properly with $codei%AD<%Base%>%$$ scalars:
$srccode%cpp% */
namespace CppAD {
        // functions that return references
        template <class Base> const AD<Base>& conj(const AD<Base>& x)
        {   return x; }
        template <class Base> const AD<Base>& real(const AD<Base>& x)
        {   return x; }

        // functions that return values (note abs is defined by cppad.hpp)
        template <class Base> AD<Base> imag(const AD<Base>& x)
        {   return CppAD::AD<Base>(0.); }
        template <class Base> AD<Base> abs2(const AD<Base>& x)
        {   return x * x; }
}
/* %$$

$head eigen_vector$$
The class $code CppAD::eigen_vector$$ is a wrapper for Eigen column vectors
so that they are $cref/simple vectors/SimpleVector/$$.
To be specific, it converts $code Eigen::Index$$ arguments and
return values to $code size_t$$.
$srccode%cpp% */
namespace CppAD {
    template <class Scalar>
    class eigen_vector : public Eigen::Matrix<Scalar, Eigen::Dynamic, 1> {
    private:
        // base_class
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> base_class;
    public:
        // constructor
        eigen_vector(size_t n) : base_class( Eigen::Index(n) )
        { }
        eigen_vector(void) : base_class()
        { }
        // operator[]
        Scalar& operator[](size_t i)
        {   return base_class::operator[]( Eigen::Index(i) ); }
        const Scalar& operator[](size_t i) const
        {   return base_class::operator[]( Eigen::Index(i) ); }
        // size
        size_t size(void) const
        {   return size_t( base_class::size() ); }
        // resize
        void resize(size_t n)
        {   base_class::resize( Eigen::Index(n) ); }
    };
}
/* %$$

$head Include cppad.hpp$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
/* %$$

$head std Definitions$$
The definitions below use cppad.hpp.
Note that $cref Value$$ function can only be used with a
$cref/constant parameter/glossary/Parameter/Constant/$$ argument.
$srccode%cpp% */
namespace std {
    template <class Base> bool isinf(const CppAD::AD<Base> &x)
    {   return isinf(CppAD::Value( CppAD::Var2Par(x) ) ); }

    template <class Base> bool isfinite(const CppAD::AD<Base> &x)
    {   return isfinite(CppAD::Value( CppAD::Var2Par(x) ) ); }

    template <class Base> bool isnan(const CppAD::AD<Base> &x)
    {   return isnan(CppAD::Value( CppAD::Var2Par(x) ) ); }
}
/* %$$
$end
*/
# endif
