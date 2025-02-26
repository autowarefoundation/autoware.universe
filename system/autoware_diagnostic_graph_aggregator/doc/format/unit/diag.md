# Diag

The `diag` object is a unit that refers to a specific status within the source diagnostics.

## Format

| Name      | Type     | Required | Description                            |
| --------- | -------- | -------- | -------------------------------------- |
| `type`    | `string` | yes      | Specify `diag` when using this object. |
| `node`    | `string` | yes      | The name of the diagnostic status.     |
| `name`    | `string` | yes      | The name of the diagnostic status.     |
| `timeout` | `float`  | yes      | The name of the diagnostic status.     |

In normal use cases, `node` parameter should be the `Hardware ID` of a node and the `name` should be one of the diagnosis attributes of that node.
