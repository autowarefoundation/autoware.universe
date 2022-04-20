# autoware_ad_api_msgs

## InterfaceVersion

This service provides an interface version based on [Semantic Versioning](SemVer).
Unlike others, this interface must be backwards and forwards compatible across major versions.

## ResponseStatus

This status is commonly used in software to unify error handling. The field `summary` is the overall result and processing is based on it.
The field `details` is used to pass the result of the interface used internally. This is for analysis and is mainly used by developers.

## ResponseStatusDetail

This is the content of ResponseStatus. The `code` is the information for the program to process the result, and the rest are the text for the user.
The `message` is an error summary and the application first displays this to the user.
The `description` is an error detail such as solution tips and URL to the documentation. It is displayed when requested by the user.
The `component`indicates where the error occurred. which is mainly used when contacting the developer.

<!-- link -->

[semver]: https://semver.org/
