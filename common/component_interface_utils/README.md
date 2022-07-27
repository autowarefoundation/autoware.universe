# component_interface_utils

## Features

This is a utility package that provides the following features:

- Instantiation of wrapper class
- Logging for service and client
- Service exception for response
- Macros
- Relays for topic and service

## Design

This package provides the wrappers for the interface classes of rclcpp.
The wrappers limit the usage of the original class to enforce the processing recommended by the component interface.
Do not inherit the class of rclcpp, and forward or wrap the member function that is allowed to be used.

## Instantiation of wrapper class

The wrapper class requires interface information in this format.

```cpp
struct SampleService
{
  using Service = sample_msgs::srv::ServiceType;
  static constexpr char name[] = "/sample/service";
};

struct SampleMessage
{
  using Message = sample_msgs::msg::MessageType;
  static constexpr char name[] = "/sample/message";
  static constexpr size_t depth = 3;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};
```

Create a wrapper using the above definition as follows.

```cpp
// header file
component_interface_utils::Service<SampleService>::SharedPtr srv_;
component_interface_utils::Client<SampleService>::SharedPtr cli_;
component_interface_utils::Publisher<SampleMessage>::SharedPtr pub_;
component_interface_utils::Subscription<SampleMessage>::SharedPtr sub_;

// source file
const auto node = component_interface_utils::NodeAdaptor(this);
node.init_srv(srv_, callback);
node.init_cli(srv_);
node.init_pub(srv_);
node.init_sub(srv_, callback);
```

## Logging for service and client

If the wrapper class is used, logging is automatically enabled. The log level is `RCLCPP_INFO`.

## Service exception for response

If the wrapper class is used, throwing `ServiceException` will automatically catch it and set it to response status.
This is useful for abending service processing in a function called from the service callback.

```cpp
void ServiceCallback(Request req, Response res)
{
   Function();
   req->status.success = true;
}

void Function()
{
   throw ServiceException(ERROR_CODE, "message");
}
```

If the wrapper class is not used, manually catch the `ServiceException` and set the response status as follows.

```cpp
void ServiceCallback(Request req, Response res)
{
   try {
      Function();
      req->status.success = true;
   } catch (const ServiceException & error) {
      res->status = error.status();
   }
}
```

## Macros

When creating callbacks for services and messages, developers have to write long arguments.
This contains a lot of redundant information that is common to all callbacks.

```cpp
void ServiceCallback(
   const some_package_name::srv::ServiceName::Request::SharedPtr req,
   const some_package_name::srv::ServiceName::Response::SharedPtr res);
```

Developers only needs what type the callback argument is for.
This macro complements the intersection from the type name.

```cpp
void ServiceCallback(ROS_SERVICE_ARG(some_package_name::srv::ServiceName));
```

## Relays for topic and service

There are utilities for relaying services and messages of the same type.

```cpp
const auto node = component_interface_utils::NodeAdaptor(this);
service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
node.relay_message(pub_, );
node.relay_service(cli_, srv_, service_callback_group_);
```
