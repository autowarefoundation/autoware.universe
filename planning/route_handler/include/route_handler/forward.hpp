// Copyright 2022 Macnica, Inc.
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

#ifndef ROUTE_HANDLER__FORWARD_HPP_
#define ROUTE_HANDLER__FORWARD_HPP_

#include <memory>
#include <vector>

namespace route_handler
{

// forward declaration
class LaneletPath;
class LaneletPoint;
class LaneletRoute;
class LaneletRouteBuilder;
class LaneletSection;

using LaneletRoutePtr = std::shared_ptr<LaneletRoute>;
using LaneletSections = std::vector<LaneletSection>;

//! @brief Explicit point removal strategy to handle path overlapping issues
//! - DO_NOTHING: keep overlapped section as is (if path is broken it will need to be fixed later)
//! - DISCARD: overlapped section is removed from the path
//! - KEEP_START: overlapped section is kept at the start of the path only
//! - KEEP_END: overlapped section is kept at the end of the path only
//! - SPLIT: overlapped section is split and shared evenly among the to the start and end section
enum class OverlapRemovalStrategy { DO_NOTHING, DISCARD, KEEP_START, KEEP_END, SPLIT };

}  // namespace route_handler

#endif  // ROUTE_HANDLER__FORWARD_HPP_
