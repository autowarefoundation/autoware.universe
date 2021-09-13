# fault_injection

This package is used to convert pseudo system faults from PSim to Diagnostics and notify Autoware.
The component diagram is as follows:

![img/component.png](img/component.svg)

## Test

```bash
source install/setup.bash
cd fault_injection
launch_test test/test_fault_injection_node.test.py
```
