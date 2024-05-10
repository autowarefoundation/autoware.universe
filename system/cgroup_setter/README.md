# cgroup_setter

## Purpose

This package set a PID to a custom cgroup.
The PID is found by `pgrep -f`.

## Inputs / Outputs

### Outputs

| Name           | Type                                     | Description         |
| -------------- | ---------------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs::msgs::DiagnosticArray` | Diagnostics outputs |

## Parameters

### Node Parameters

| Name                        | Type   | Default Value                                       | Explanation                             | Reconfigurable |
| --------------------------- | ------ | --------------------------------------------------- | --------------------------------------- | -------------- |
| `cgroup_setting_config_path`| string | `$(find-pkg-share cgroup_setter)/config/cgroup.yaml`| yaml file path                          | |

### YAML format for cgroup_setter

format
```yaml
base_path: "/sys/fs/cgroup"
settings:
  - directory: "xxx/xxx"
    search_word: 
      - "xxxxx"
      - "xxxxx"
  - directory: "xxx/xxx"
    search_word: 
      - "xxxxx"
```
The following is an example of joining the PID from running `pgrep -f`
with the keyword `__node:=system_monitor_container` to a cgroup named `/sys/fs/cgroup/autoware/system_monitor`.

example
```yaml
base_path: "/sys/fs/cgroup"
settings:
  - directory: "autoware/system_monitor"
    search_word: 
      - "__node:=system_monitor_container"
```
#### Rules

- The value of settings must be a sequence.

example
```yaml
# NG
base_path: "/sys/fs/cgroup"
settings:
  directory: "autoware/system_monitor" # - directory
  search_word: 
  - "__node:=system_monitor_container"
```
- The value of search_word must be a sequence.

example
```yaml
# NG
base_path: "/sys/fs/cgroup"
settings:
  - directory: "autoware/system_monitor"
    search_word: "__node:=system_monitor_container" # ["__node:=system_monitor_container"] or - "__node:=system_monitor_container"
```


## Assumptions / Known limits

TBD.

## Usage

### launch

```sh
ros2 launch cgroup_setter cgroup_setter.launch.xml
```

