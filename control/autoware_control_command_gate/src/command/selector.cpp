// Copyright 2025 The Autoware Contributors
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

#include "selector.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::control_command_gate
{

CommandSelector::CommandSelector(SourceChangeCallback on_change_source)
{
  on_change_source_ = std::move(on_change_source);
}

void CommandSelector::add_source(std::unique_ptr<CommandSource> && source)
{
  source->set_output(nullptr);
  sources_.emplace(source->name(), std::move(source));
}

void CommandSelector::set_output(std::unique_ptr<CommandOutput> && output)
{
  output_ = std::move(output);
}

bool CommandSelector::select(const std::string & name)
{
  const auto iter = sources_.find(name);
  if (iter == sources_.end()) {
    return false;
  }
  for (auto & [key, source] : sources_) {
    if (key == name) {
      source->set_output(output_.get());
      source->resend_last_command();
    } else {
      source->set_output(nullptr);
    }
  }
  on_change_source_(name);
  return true;
}

}  // namespace autoware::control_command_gate
