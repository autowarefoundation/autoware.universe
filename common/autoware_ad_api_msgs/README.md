# autoware_ad_api_msgs

## InterfaceVersion

The interface should be stable, but it also needs to be improved on demand. This requires version control.
The application needs to check compatibility with the interface version and to switch the processing depending on that.
For that, this service provides an interface version based on [Semantic Versioning][semver].
Unlike others, this interface should be especially stable because it needs to be backwards and forwards compatible across major versions.

<!-- link -->

[semver]: https://semver.org/
