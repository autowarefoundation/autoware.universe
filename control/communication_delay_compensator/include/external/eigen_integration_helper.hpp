//
// Created by ali on 24/05/22.
//

#ifndef COMMUNICATION_DELAY_COMPENSATOR__EIGEN_INTEGRATION_HELPER_HPP
#define COMMUNICATION_DELAY_COMPENSATOR__EIGEN_INTEGRATION_HELPER_HPP


#include <eigen3/Eigen/Dense>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/version.hpp>

/**
 * @brief Necessary routines for Eigen matrices to make them work with vector_space_algebra.
*/

#if (EIGEN_VERSION_AT_LEAST(3, 3, 0) && BOOST_VERSION < 107100)
namespace Eigen
{
namespace internal
{
template <typename Scalar>
struct scalar_add_op
{
  EIGEN_DEVICE_FUNC inline scalar_add_op(const scalar_add_op & other) : m_other(other.m_other) {}
  EIGEN_DEVICE_FUNC inline scalar_add_op(const Scalar & other) : m_other(other) {}
  EIGEN_DEVICE_FUNC inline Scalar operator()(const Scalar & a) const { return a + m_other; }
  template <typename Packet>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Packet packetOp(const Packet & a) const
  {
	return internal::padd(a, pset1<Packet>(m_other));
  }
  const Scalar m_other;
};

template <typename Scalar>
struct functor_traits<scalar_add_op<Scalar>>
{
  enum { Cost = NumTraits<Scalar>::AddCost, PacketAccess = packet_traits<Scalar>::HasAdd };
};

}  // namespace internal

template <typename D>
inline const typename Eigen::CwiseUnaryOp<
  typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>, const D>
operator+(
  const typename Eigen::MatrixBase<D> & m, const typename Eigen::internal::traits<D>::Scalar & s)
{
  return Eigen::CwiseUnaryOp<
	typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>, const D>(
	m.derived(), Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>(s));
}

template <typename D>
inline const typename Eigen::CwiseUnaryOp<
  typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>, const D>
operator+(
  const typename Eigen::internal::traits<D>::Scalar & s, const typename Eigen::MatrixBase<D> & m)
{
  return Eigen::CwiseUnaryOp<
	typename Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>, const D>(
	m.derived(), Eigen::internal::scalar_add_op<typename Eigen::internal::traits<D>::Scalar>(s));
}

template <typename D1, typename D2>
inline const typename Eigen::CwiseBinaryOp<
  typename Eigen::internal::scalar_quotient_op<typename Eigen::internal::traits<D1>::Scalar>,
  const D1, const D2>
operator/(const Eigen::MatrixBase<D1> & x1, const Eigen::MatrixBase<D2> & x2)
{
  return x1.cwiseQuotient(x2);
}

template <typename D>
inline const typename Eigen::CwiseUnaryOp<
  typename Eigen::internal::scalar_abs_op<typename Eigen::internal::traits<D>::Scalar>, const D>
abs(const Eigen::MatrixBase<D> & m)
{
  return m.cwiseAbs();
}

}  // namespace Eigen

namespace boost
{
namespace numeric
{
namespace odeint
{
template <int S1, int S2, int O, int M1, int M2>
struct vector_space_norm_inf<Eigen::Matrix<double, S1, S2, O, M1, M2>>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef double result_type;
  result_type operator()(const Eigen::Matrix<double, S1, S2, O, M1, M2> & m) const
  {
	return m.template lpNorm<Eigen::Infinity>();
  }
};

}  // namespace odeint
}  // namespace numeric
}  // namespace boost

#endif
#endif //COMMUNICATION_DELAY_COMPENSATOR__EIGEN_INTEGRATION_HELPER_HPP
