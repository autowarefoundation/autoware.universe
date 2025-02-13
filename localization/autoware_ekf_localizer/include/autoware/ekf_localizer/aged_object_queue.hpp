// Copyright 2023 Autoware Foundation
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

#ifndef AUTOWARE__EKF_LOCALIZER__AGED_OBJECT_QUEUE_HPP_
#define AUTOWARE__EKF_LOCALIZER__AGED_OBJECT_QUEUE_HPP_

#include <cstddef>
#include <queue>

namespace autoware::ekf_localizer
{

template <typename Object>
class AgedObjectQueue
{
public:
  explicit AgedObjectQueue(const size_t max_age) : max_age_(max_age) {}

  [[nodiscard]] bool empty() const { return this->size() == 0; }

  [[nodiscard]] size_t size() const { return objects_.size(); }

  Object back() const { return objects_.back(); }

  void push(const Object & object)
  {
    objects_.push(object);
    ages_.push(0);
  }

  Object pop_increment_age()
  {
    const Object object = objects_.front();
    const size_t age = ages_.front() + 1;
    objects_.pop();
    ages_.pop();

    if (age < max_age_) {
      objects_.push(object);
      ages_.push(age);
    }

    return object;
  }

  void clear()
  {
    objects_ = std::queue<Object>();
    ages_ = std::queue<size_t>();
  }

private:
  const size_t max_age_;
  std::queue<Object> objects_;
  std::queue<size_t> ages_;
};

}  // namespace autoware::ekf_localizer

#endif  // AUTOWARE__EKF_LOCALIZER__AGED_OBJECT_QUEUE_HPP_
