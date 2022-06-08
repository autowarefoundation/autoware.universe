# autoware_ad_api_msgs

## ResponseStatus

This message is a response status commonly used in the service type API. For details, see [the response status][docs-response-status].
Each API defines its own error code and assigns a level to them. For example, define 12345 as the code to indicate that the parameter is invalid.
The level of this code is an error if the process cannot be continued, or a warning if the process can be continued by using the default value or something.
This makes it possible to know the success or failure regardless of the API by checking the level.

The code for frequent errors is predefined as shown below and numbers less than 10000 are reserved for these.
Use numbers greater than or equal to 10000 for the code defined by each API.

| Name                     | Code | Level | Description                                              |
| ------------------------ | ---- | ----- | -------------------------------------------------------- |
| INTERNAL_SERVICE_UNREADY | 101  | ERROR | The service on which this API depends was not available. |
| INTERNAL_SERVICE_TIMEOUT | 102  | ERROR | The service on which this API depends has timed out.     |

## InterfaceVersion

This message is for the interface version of the set of AD APIs. For details, see [the interface feature][docs-interface].

<!-- link -->

[docs-response-status]: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/#response-status
[docs-interface]: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/interface/
