/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "utils_act/transfer_functions.hpp"


size_t ns_control_toolbox::tf_base::getPolynomialStringAndSize(std::vector<double> const& num_or_den,
                                                               std::ostringstream& string_stream)
	{
		
		auto Nn        = num_or_den.size();
		int  precision = 4;
		
		for (size_t k = 0; k < Nn; ++k)
			{
				
				auto&& power_of_term = Nn - 1 - k;
				auto&& coeff_abs     = std::abs(num_or_den.at(k));
				
				if (std::fabs(num_or_den.at(k)) > EPS)
					{
						// to put "+" or "-" char in front of "s"
						auto sign_str = num_or_den.at(k) > 0 ? " + " : " - ";
						
						// k is not the last idx.
						if (k != Nn - 1)
							{
								// if k==0, do not use +s for the first term of the polynomial.
								if (k == 0 && num_or_den.at(k) > 0)
									{
										sign_str = "";
									}
								
								// If the coefficient is 1 do not print 1s --> instead print s
								if (std::fabs(std::fabs(num_or_den.at(k)) - 1) > EPS)
									{
										string_stream << sign_str << std::setprecision(precision) << coeff_abs;
									}
								else
									{
										string_stream << sign_str;
									}
								
								// if power is 1 do not print s^1 --> instead s
								if (power_of_term != 1)
									{
										string_stream << " s^" << power_of_term;
									}
								else
									{
										string_stream << " s";
									}
								
							}
						else
							{               // if k is the last index
								if (Nn > 1)
									{
										string_stream << sign_str << std::setprecision(precision) << coeff_abs;
									}
								else
									{
										string_stream << num_or_den.at(k);
									}
								
							}
					}
			}
		return string_stream.str().length();
	}

void ns_control_toolbox::tf_base::print() const
	{
		
		std::ostringstream num_string;
		std::ostringstream den_string;
		
		auto num_string_size = getPolynomialStringAndSize(num_, num_string);
		auto den_string_size = getPolynomialStringAndSize(den_, den_string);
		
		if (num_string_size > den_string_size)
			{
				size_t num_of_tab = (num_string_size - den_string_size) / 4;
				ns_utils::print(num_string.str());
				ns_utils::print(std::string(std::max(num_string_size, den_string_size), '-'));
				ns_utils::print(std::string(num_of_tab, ' '), den_string.str());
				
			}
		else if (num_string_size < den_string_size)
			{
				size_t num_of_tab = (den_string_size - num_string_size) / 4;
				ns_utils::print(std::string(num_of_tab, ' '), num_string.str());
				ns_utils::print(std::string(std::max(num_string_size, den_string_size), '-'));
				ns_utils::print(den_string.str());
				
			}
		else
			{
				
				ns_utils::print(num_string.str());
				ns_utils::print(std::string(std::max(num_string_size, den_string_size), '-'));
				ns_utils::print(den_string.str());
			}
		
		ns_utils::print("\nContinuous-time transfer function.\n");
		
	}

void ns_control_toolbox::tf_base::inv()
	{
		std::vector<double> temp_den{ std::move(den_) };
		den_ = std::move(num_);
		num_ = std::move(temp_den);
		
	}

ns_control_toolbox::tf_base::tf_base(const ns_control_toolbox::tf_factor& num,
                                     const ns_control_toolbox::tf_factor& den)
	{
		num_ = num();
		den_ = den();
		
	}

void ns_control_toolbox::tf_base::update_num(std::vector<double> const& num)
	{
		
		num_ = num;
	}

void ns_control_toolbox::tf_base::update_num(tf_factor const& num)
	{
		
		num_ = num();
	}

void ns_control_toolbox::tf_base::update_den(const std::vector<double>& den)
	{
		den_ = den;
		
	}

void ns_control_toolbox::tf_base::update_den(const ns_control_toolbox::tf_factor& den)
	{
		den_ = den();
	}


ns_control_toolbox::tf ns_control_toolbox::padecoeff(const double& Td, const size_t& order)
	{
		if (std::fabs(Td) <= EPS)
			{
				return {};             // constant tf.
			}
		
		auto&& n = order + 1;
		std::vector<double> num(n, 0.0);
		std::vector<double> den(n, 0.0);
		
		num.back() = 1.;
		den.back() = 1.;
		
		// Double conversion of order
		double order_double_t{ static_cast<double>(order) };
		
		for (size_t k = 1; k < n; ++k)
			{
				double k_double_t{ static_cast<double>(k) };
				double fact = Td * (order_double_t - k_double_t + 1) / (2. * order_double_t - k_double_t + 1) /
				              k_double_t;
				num[n - 1 - k] = -fact * num[n - k];
				den[n - 1 - k] = fact * den[n - k];
			}
		
		auto const den0 = den[0];
		std::transform(num.begin(), num.end(), num.begin(),
		               [&den0](auto& x)
			               { return x / den0; });
		
		std::transform(den.begin(), den.end(), den.begin(),
		               [&den0](auto& x)
			               { return x / den0; });
		
		return { num, den };
	}

ns_control_toolbox::tf ns_control_toolbox::pade(const double& Td, const size_t& order)
	{
		return padecoeff(Td, order);
		
	}