# colcon-test

This action runs `colcon test` with labels specified in a regex format.
Note that you need to build target packages before running this action.

## Usage

```yaml
jobs:
  build:
    runs-on: ubuntu-latest
    container: ros:galactic
    steps:
      - name: Check out repository
        uses: actions/checkout@v3
        with:
          fetch-depth: 0

      - name: Get modified packages
        id: get-modified-packages
        uses: autowarefoundation/autoware-github-actions/get-modified-packages@v1

      - name: Build
        if: ${{ steps.get-modified-packages.outputs.modified-packages != '' }}
        uses: autowarefoundation/autoware-github-actions/colcon-build@v1
        with:
          rosdistro: galactic
          target-packages: ${{ steps.get-modified-packages.outputs.modified-packages }}
          build-depends-repos: build_depends.repos

  test:
    needs: build
    runs-on: ubuntu-latest
    container: ros:galactic
    strategy:
      matrix:
        include:
          - test-label: gtest
            codecov-flags: gtest
          - test-label: launch_test
            codecov-flags: launch_test
    steps:
      - name: Check out repository
        uses: actions/checkout@v3
        with:
          fetch-depth: 0

      - name: Get modified packages
        id: get-modified-packages
        uses: autowarefoundation/autoware-github-actions/get-modified-packages@v1

      - name: Test
        id: test
        if: ${{ steps.get-modified-packages.outputs.modified-packages != '' }}
        uses: autowarefoundation/autoware-github-actions/colcon-test@v1
        with:
          rosdistro: galactic
          target-packages: ${{ steps.get-modified-packages.outputs.modified-packages }}
          build-depends-repos: build_depends.repos
          label-regex: ${{ matrix.test-label }}

      - name: Upload coverage to Codecov
        if: ${{ steps.test.outputs.coverage-report-files != '' }}
        uses: codecov/codecov-action@v2
        with:
          files: ${{ steps.test.outputs.coverage-report-files }}
          fail_ci_if_error: false
          verbose: true
          flags: ${{ matrix.codecov-flags }}
```

## Inputs

| Name                | Required | Description                                         |
| ------------------- | -------- | --------------------------------------------------- |
| rosdistro           | true     | ROS distro.                                         |
| target-packages     | true     | The target packages to test.                        |
| build-depends-repos | false    | The `.repos` file that includes build dependencies. |
| label-regex         | false    | The regex pattern of test labels to be run.         |
| token               | false    | The token for build dependencies.                   |

## Outputs

| Name                  | Description                      |
| --------------------- | -------------------------------- |
| coverage-report-files | Generated coverage report files. |
