//
// Created by ali on 24/05/22.
//

#ifndef COMMUNICATION_DELAY_COMPENSATOR__INTEGRATE_EIGEN_STATES_HPP
#define COMMUNICATION_DELAY_COMPENSATOR__INTEGRATE_EIGEN_STATES_HPP


#include <memory>
#include "external/eigen_integration_helper.hpp"
#include "utils_act/act_utils.hpp"

class ODE_zoh_common
	{
public:
	template<class T>
	explicit ODE_zoh_common(T&& ode_object)
	{
		*this = ode_object;
	}

	template<class T>
	ODE_zoh_common& operator=(T const& ode_object)
	{
		this->ode_ = std::make_shared<Model < T>>
		(ode_object);

		return *this;
	}


	void operator()() const
	{
		(*ode_)();

	}

	void print() const
	{
		ode_->print();
	}


private:
	// Concept base class
	class Concept
		{
	public:
		virtual ~Concept() = default;

		virtual void operator()() const = 0;

		virtual void print() const = 0;
		};

	// Model that stores external class
	template<class T>
	class Model : public Concept
		{
	public:
		explicit Model(T object)
		{
			this->object_.reset(new T(object));
		};

		void operator()() const override
		{
			(*object_)();
			// object_->print();
		}

		void print() const override
		{
			object_->print();
		}

	private:
		std::unique_ptr<T> object_;

		};

	// Common interface data type.
	std::shared_ptr<Concept> ode_{};

	};


// Ode interfaces.

class dummyOde
	{

public:
	explicit dummyOde(int num) : num_{ num }
	{
	}

	void operator()() const
	{

		ns_utils::print("This class number is : ", num_);
	}

	void print() const
	{

		ns_utils::print("This class number is : ", num_);
	}


private:
	int num_;
	};

// Simulation method.
void simulate_model();

#endif //COMMUNICATION_DELAY_COMPENSATOR__INTEGRATE_EIGEN_STATES_HPP
