//
// Created by ali on 17/2/22.
//

#include <fmt/core.h>
#include <cassert>
#include "utils_act/act_utils.hpp"
#include "autoware_control_toolbox.hpp"
#include "utils_act/state_space.hpp"

int main()
{


	std::vector<double> num{ 1. };
	std::vector<double> den{ 5.039e-07, 0.00019, 0.02387, 1 };


//	std::vector<double> num{ 1 };
//	std::vector<double> den{ 3 };

	// With a num, den
	ns_control_toolbox::tf sys_tf(num, den);

	// Print sys
	sys_tf.print();

	//  Default constructor example
	// ns_control_toolbox::tf2ss sys_ss; args : sys, Ts with Ts is defaulted = 0.1
	ns_control_toolbox::tf2ss sys_ss1(sys_tf);

	// Constructor with num and den
	// args : sys, Ts with Ts is defaulted = 0.1
	ns_control_toolbox::tf2ss sys_ss2(num, den);
	sys_ss2.print();

	ns_utils::print("SS.A \n");
	ns_eigen_utils::printEigenMat(sys_ss2.A());

	ns_utils::print("SS.Ad \n");
	ns_eigen_utils::printEigenMat(sys_ss2.Ad());

	// DISCRETIZATION : Note sstf2 automatically discretisize
	double Ts{ 0.05 };
	// sys_ss2.discretisize(Ts);
	sys_ss2.print_discrete_system();

	// Test defaulted.
	ns_utils::print("Discretization with a given Ts when constructing");
	auto sys_ss3 = ns_control_toolbox::tf2ss(sys_tf, Ts);
	sys_ss3.print_discrete_system();


	// Using ss
//	auto ss1 = ns_control_toolbox::ss<3>{ sys_tf,
//	                                      Ts };
//
//	ns_utils::print("Templated state-space class");
//	ns_utils::print("----------------------------");
//	ss1.print_discrete_system();
//
//
//	// Get state space matrices from ss.
//	int const nx{ 3 };
//
//	ns_control_toolbox::mat_type_t<nx, nx> A;
//	ns_control_toolbox::mat_type_t<nx, 1> B;
//	ns_control_toolbox::mat_type_t<1, nx> C;
//	ns_control_toolbox::mat_type_t<1, 1> D;
//
////	ss1.getSSc(A, B, C, D);
//	ss1.getSSd(A, B, C, D);
//	ns_utils::print("Getting A, B, C, D");
//	ns_eigen_utils::printEigenMat(A, "A");
//	ns_eigen_utils::printEigenMat(A);
//	ns_eigen_utils::printEigenMat(B, "B");
//	ns_eigen_utils::printEigenMat(C, "C");
//	ns_eigen_utils::printEigenMat(D, "D");



//
//	auto ss0 = ns_control_toolbox::ss<3>{{ 1, 0, 0 },
//	                                     { 2, 2, 4 },
//	                                     Ts };
//
//	ns_utils::print("Templated state-space class");
//	ns_utils::print("----------------------------");
//	ss0.print_discrete_system();

	return 0;
}