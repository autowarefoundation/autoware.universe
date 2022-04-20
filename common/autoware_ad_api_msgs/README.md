# autoware_ad_api_msgs

## InterfaceVersion

The interface should be stable, but it also needs to be improved on demand. This requires version control.
The application needs to check compatibility with the interface version and to switch the processing depending on that.
For that, this service provides an interface version based on [Semantic Versioning][semver].
Unlike others, this interface should be especially stable because it needs to be backwards and forwards compatible across major versions.

## ResponseStatus

This status is commonly used in software to unify error handling. The field `summary` is the overall result and processing is based on it.
The field `details` is used to pass the result of the interface used internally. This is for analysis and is mainly used by developers.

Below is a sample response status. The user knows that an parameter error has occurred in `sample/module2`, so contacting the developer with this information will facilitate the analysis.

```yaml
summary:
  code: ERROR
  component: sample
  message: ...
  description: ...
details:
  - code: SUCCESS
    component: sample/module1
    message: ...
    description: ...
  - code: ERROR
    component: sample/module2
    message: unknown parameter
    description: ...
```

## ResponseStatusDetail

This is the content of ResponseStatus. The `code` is the information for the program to process the result, and the rest are the text for the user.
The `message` is an error summary and the application first displays this to the user.
The `description` is an error detail such as solution tips and URL to the documentation. It is displayed when requested by the user.
The `component`indicates where the error occurred. which is mainly used when contacting the developer.

<!-- link -->

[semver]: https://semver.org/
