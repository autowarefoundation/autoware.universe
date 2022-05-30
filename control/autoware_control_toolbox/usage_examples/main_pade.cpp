//
// Created by ali on 21/2/22.
//

#include <fmt/core.h>
#include <cassert>

#include "autoware_control_toolbox.hpp"


int main()
{

	double Td    = 0.1; // time delay in seconds.
	size_t order = 3; // order of the Pade approximation.

	auto tf_delay = ns_control_toolbox::pade(Td, order);
	tf_delay.print();

	auto ss_sys = ns_control_toolbox::tf2ss(tf_delay);
	ss_sys.print();

	// Test discretization and compare with Matlab.
	double Ts = 0.1;
	ss_sys.discretisize(Ts);
	ss_sys.print_discrete_system();

	//
	ns_utils::print("Discretization with a given Ts when constructing");
	Ts = 0.15;
	auto sys_ss2 = ns_control_toolbox::tf2ss(tf_delay, Ts);
	sys_ss2.print_discrete_system();

	// Test balance.
//	auto const nx = sys_ss2.A_.rows();
//	Eigen::MatrixXd system_mat(Eigen::MatrixXd::Zero(nx + 1, nx + 1)); // Make system matrix [A, B; C, D]
//	system_mat.topLeftCorner(nx, nx)   = sys_ss2.A_;
//	system_mat.topRightCorner(nx, 1)   = sys_ss2.B_;
//	system_mat.bottomLeftCorner(1, nx) = sys_ss2.C_;
//	system_mat.bottomRightCorner(1, 1) = sys_ss2.Dd_;
//
//	ns_eigen_utils::printEigenMat(system_mat);


	return 0;
}
