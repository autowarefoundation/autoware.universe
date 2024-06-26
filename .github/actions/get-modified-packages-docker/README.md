# get-modified-packages

This action get the list of ROS packages modified in the pull request.

## Usage

### Basic

```yaml
jobs:
  get-modified-packages:
    runs-on: ubuntu-latest
    steps:
      - name: Check out repository
        uses: actions/checkout@v4
        with:
          fetch-depth: 0

      - name: Get modified packages
        id: get-modified-packages
        uses: autowarefoundation/autoware-github-actions/get-modified-packages@v1
```

### Fetch only the PR branch and all PR commits

```yaml
jobs:
  get-modified-packages:
    runs-on: ubuntu-latest
    steps:
      - name: Set PR fetch depth
        run: echo "PR_FETCH_DEPTH=$(( ${{ github.event.pull_request.commits }} + 1 ))" >> "${GITHUB_ENV}"

      - name: Checkout PR branch and all PR commits
        uses: actions/checkout@v4
        with:
          ref: ${{ github.event.pull_request.head.sha }}
          fetch-depth: ${{ env.PR_FETCH_DEPTH }}

      - name: Get modified packages
        id: get-modified-packages
        uses: autowarefoundation/autoware-github-actions/get-modified-packages@v1
```

## Inputs

| Name        | Required | Description                                      |
| ----------- | -------- | ------------------------------------------------ |
| base-branch | false    | The base branch to search for modified packages. |

## Outputs

| Name              | Description                                            |
| ----------------- | ------------------------------------------------------ |
| modified-packages | The list of ROS packages modified in the pull request. |
