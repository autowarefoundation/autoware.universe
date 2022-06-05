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

	// Concat
	auto AB = ns_eigen_utils::hstack<double>(ss_sys.A(), ss_sys.B());
	auto CD = ns_eigen_utils::hstack<double>(ss_sys.C(), ss_sys.D());
	auto ss_system = ns_eigen_utils::vstack<double>(AB, CD);

	ns_utils::print("Matrix to be balanced ");
	ns_eigen_utils::printEigenMat(ss_system);

	ns_utils::print("Balanced matrix for the system ");
	ns_control_toolbox::balance(ss_system);
	ns_eigen_utils::printEigenMat(ss_system);

	// COMPARE with MATLAB RESULTS.
	double tau_steer{ 0.3 };
	double wheelbase{ 2.9 }; // L in vehicle model.

	ns_control_toolbox::tf_factor m_den1{{ wheelbase, 0, 0 }}; // L*s^2
	ns_control_toolbox::tf_factor m_den2{{ tau_steer, 1 }}; // (tau*s + 1)
	auto den_tf_factor = m_den1 * m_den2;

	ns_control_toolbox::tf Gey({ 1. }, den_tf_factor(), 4., 1.); // num, den, num constant, den constant
	Gey.print();

	// Qfilter
	double dt = 1. / 40;
	double frq = 20.;
	double wc = 2 * M_PI * frq;
	double tau_q = 1 / wc;

	ns_control_toolbox::tf_factor den_q{{ tau_q, 1 }};
	den_q.power(3);

	auto Qtf = ns_control_toolbox::tf({ 1 }, { den_q() });
	Qtf.print();

	Gey.inv();
	Gey.print();

	auto QGinv = Qtf * Gey;
	QGinv.print();

	auto QGinvss = ns_control_toolbox::tf2ss(QGinv, dt);

	ns_utils::print("Balanced System Matrices");
	QGinvss.print();
	// PERMUTATION TEST




	return 0;
}