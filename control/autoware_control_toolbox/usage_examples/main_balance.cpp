//
// Created by ali on 30/05/22.
//
#include "autoware_control_toolbox.hpp"
#include <vector>

int main()
{

	double Td = 0.11; // time delay in seconds.
	size_t order = 3; // order of the Pade approximation.

	auto tf_delay = ns_control_toolbox::pade(Td, order);
	tf_delay.print();

	// Define Ts
	double Ts{ 0.1 };
	auto ss_sys = ns_control_toolbox::tf2ss(tf_delay);
	ss_sys.print();

	// Test discretization and compare with Matlab.

	ss_sys.discretisize(Ts);
	ss_sys.print_discrete_system();

//	ns_control_toolbox::balance(ss_sys.A_);
//	ns_utils::print("Balanced matrix A_");
//	ns_eigen_utils::printEigenMat(ss_sys.A_);

	// Concat
	auto AB = ns_eigen_utils::hstack<double>(ss_sys.A(), ss_sys.B());
	auto CD = ns_eigen_utils::hstack<double>(ss_sys.C(), ss_sys.D());
	auto ss_system = ns_eigen_utils::vstack<double>(AB, CD);

	ns_utils::print("Matrix to be balanced ");
	ns_eigen_utils::printEigenMat(ss_system);

	ns_utils::print("Balanced matrix for the system ");
	ns_control_toolbox::balance(ss_system);
	ns_eigen_utils::printEigenMat(ss_system);

	// PERMUTATION TEST



	return 0;
}