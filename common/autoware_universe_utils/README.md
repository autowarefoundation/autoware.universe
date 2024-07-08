# autoware_universe_utils

## Purpose

This package contains many common functions used by other packages, so please refer to them as needed.

## For developers

`autoware_universe_utils.hpp` header file was removed because the source files that directly/indirectly include this file took a long time for preprocessing.

### TimeKeeper and ScopedTimeTrack

#### `autoware::universe_utils::TimeKeeper`

##### Constructor

```cpp
template <typename... Reporters>
explicit TimeKeeper(Reporters... reporters);
```

- Initializes the `TimeKeeper` with a list of reporters.

##### Methods

- `void add_reporter(std::ostream * os);`

  - Adds a reporter to output processing times to an `ostream`.
  - `os`: Pointer to the `ostream` object.

- `void add_reporter(rclcpp::Publisher<ProcessingTimeDetail>::SharedPtr publisher);`

  - Adds a reporter to publish processing times to an `rclcpp` publisher.
  - `publisher`: Shared pointer to the `rclcpp` publisher.

- `void add_reporter(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher);`

  - Adds a reporter to publish processing times to an `rclcpp` publisher with `std_msgs::msg::String`.
  - `publisher`: Shared pointer to the `rclcpp` publisher.

- `void start_track(const std::string & func_name);`

  - Starts tracking the processing time of a function.
  - `func_name`: Name of the function to be tracked.

- `void end_track(const std::string & func_name);`
  - Ends tracking the processing time of a function.
  - `func_name`: Name of the function to end tracking.

#### Example

```cpp
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "autoware_universe_utils/time_keeper.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("time_keeper_example");

  auto time_keeper = std::make_shared<autoware::universe_utils::TimeKeeper>();
  time_keeper->add_reporter(&std::cout);

  auto publisher = node->create_publisher<autoware::universe_utils::ProcessingTimeDetail>("time_topic", 10);
  time_keeper->add_reporter(publisher);

  time_keeper->start_track("example_function");
  // Do some processing here
  time_keeper->end_track("example_function");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

#### `autoware::universe_utils::ScopedTimeTrack`

##### Description

Class for automatically tracking the processing time of a function within a scope.

##### Constructor

```cpp
ScopedTimeTrack(const std::string & func_name, TimeKeeper & time_keeper);
```

- `func_name`: Name of the function to be tracked.
- `time_keeper`: Reference to the `TimeKeeper` object.

##### Destructor

```cpp
~ScopedTimeTrack();
```

- Destroys the `ScopedTimeTrack` object, ending the tracking of the function.
