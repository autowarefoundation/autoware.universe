// Copyright 2022 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "integration/integrate_eigen_states.hpp"

int main()
{

	auto dummyode_1 = dummyOde(1);
	auto dummyode_2 = dummyOde(2);

	// Dummy Type Erasure for operator () interface.
	ODE_zoh_common interface{ dummyode_1 };
	interface.print();
	interface();

	// Now change.
	interface = dummyode_2;
	interface.print();
	interface();

	// interface();

	return 0;
}
