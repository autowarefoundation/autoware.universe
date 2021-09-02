# TierIV Unit Test Guideline

## How to run unit tests

You may modify the code before running unit tests. This is how to rebuild the package you modified.

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to <package name>
```

`package name` must be the name of a package that contains package.xml

Use the commands below to run unit tests. `--packages-skip-test-passed` can be used to ignore test cases that already passed.

```sh
source install/setup.bash
colcon test --event-handlers console_cohesion+ --packages-skip-test-passed
```

If you would like to run unit tests in a specific directory, you can use the `--base-paths` option. For example, if you would like to run unit tests in autoware.iv, you can run

```sh
colcon test --event-handlers console_cohesion+ --base-paths src/autoware/autoware.iv --packages-skip-test-passed
```

## How to write unit tests

See [confluence page](https://tier4.atlassian.net/wiki/spaces/AIP/pages/1400045921/T4) for the details.
