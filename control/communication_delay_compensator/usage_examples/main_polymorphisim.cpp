//
// Created by ali on 20/05/22.
//

// See http://sean-parent.stlab.cc/presentations/2016-10-10-runtime-polymorphism/2016-10-10-runtime-polymorphism.pdf
// or an earlier version, "Inheritance Is The Base Class of Evil"

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "vehicle_models/vehicle_kinematic_error_model.hpp"

// Print anything iostreams knows how to:
template<typename T>
void draw(const T& x, std::ostream& out, size_t indent)
{
	out << std::string(indent, ' ') << x << std::endl;
}

// An object holds any type T which has a corresponding function
// `void draw(const T&, ostream& out, size_t indent)`.
// It uses polymorphism to dispatch the correct draw function for each instance
// at runtime, but that polymorphism is an implementation detail,
// **not** a mandatory part of the client code/interface.
class Object
{
public:
		template<typename T>
		Object(T x) : self(std::make_shared<Model < T>>

		(
		std::move(x)
		)) {}

		// Make this a freestanding friend function (instead of a member)
		// so that it shares the _exact_ same interface as other draw functions.
		// We can now embed Objects in Objects.
		friend void draw(const Object& x, std::ostream& out, size_t indent)
		{
			x.self->_draw(out, indent);
		}

private:
		// Here we define our interface.
		struct Drawable
		{
				virtual ~Drawable() = default;  // virtual dtor needed for all interfaces
				virtual void _draw(std::ostream&, size_t) const = 0;
		};

		// Here we define the model for a particular type T.
		template<typename T>
		struct Model final : Drawable
		{
				explicit Model(T x) : data(std::move(x))
				{
				}

				void _draw(std::ostream& out, size_t indent) const override
				{
					// This calls the correct draw() function for whatever type it happens to hold
					draw(data, out, indent);
				}

				T data;
		};

		// Making this a shared_ptr to const data has a few really nice properties:
		// 1. We get automatic copy-on-write (CoW) semantics.
		//    Making a copy of ourselves just bumps the reference count.
		//
		// 2. The default special functions (move, assignment, etc.) work great.
		//    If this was a unique_ptr instead, we'd have to implement our own
		//    copy operators for Object, and have a virtual copy() function for
		//    Drawable.
		std::shared_ptr<const Drawable> self;
};



// Let's make a "document" a vector of Objects, and say how to print it.
using Document = std::vector<Object>;

// Note how composable this design is. We can use the exact same interface
// for things like std::vector. Had this design demanded inheritance,
// we would have to make our own wrapper classes around vector, etc.
// (Mantis does this unfortunately often.)
void draw(const Document& x, std::ostream& out, size_t indent)
{
	const auto indentString = std::string(indent, ' ');

	out << indentString << "{" << std::endl;

	for (const auto& e : x) draw(e, out, indent + 4);

	out << indentString << "}" << std::endl;
}

// We can insert arbitrary things too, so long as they have a draw()
class Foo
{
public:
		Foo() = default;

		explicit Foo(int x_)
			: x(x_)
		{
		}

		int x = 42;
};

void draw(const Foo& f, std::ostream& out, size_t indent)
{
	out << std::string(indent, ' ') << "I am_ a foo with value " << f.x << std::endl;
}

// Actual use:
int main()
{
	Document d;

	d.push_back(42);
	d.push_back("O hi!");
	d.push_back(Foo(29));
	draw(d, std::cout, 0);

	std::cout << "-------------" << std::endl;

	d.push_back(d); // CoW!
	d.push_back("Ta da!");
	draw(d, std::cout, 0);

	return 0;
}