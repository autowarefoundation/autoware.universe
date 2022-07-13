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

#ifndef DELAY_COMPENSATOR_INCLUDE_NODE_DENIFITIONS_NODE_DEFINITIONS_HPP
#define DELAY_COMPENSATOR_INCLUDE_NODE_DENIFITIONS_NODE_DEFINITIONS_HPP

#include "common/types.hpp"

// Delay Compensation Libraries.
#include "autoware_control_toolbox.hpp"

namespace observers {
    using autoware::common::types::bool8_t;
    using autoware::common::types::float32_t;
    using autoware::common::types::float64_t;
    using tf_t = ns_control_toolbox::tf;
    using ss_t = ns_control_toolbox::tf2ss;


}  // namespace observers

#endif  // DELAY_COMPENSATOR_INCLUDE_NODE_DENIFITIONS_NODE_DEFINITIONS_HPP
