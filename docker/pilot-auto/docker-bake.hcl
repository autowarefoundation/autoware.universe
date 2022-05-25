group "default" {
  targets = ["devel", "prebuilt"]
}

// For docker/metadata-action
target "docker-metadata-action-devel" {}
target "docker-metadata-action-prebuilt" {}

target "devel" {
  inherits = ["docker-metadata-action-devel"]
  dockerfile = "docker/pilot-auto/Dockerfile"
  target = "devel"
}

target "prebuilt" {
  inherits = ["docker-metadata-action-prebuilt"]
  dockerfile = "docker/pilot-auto/Dockerfile"
  target = "prebuilt"
}
