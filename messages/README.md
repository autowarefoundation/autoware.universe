This is a Tier4 forked version of [autoware_auto_msgs](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs)

# autoware_auto_msgs

Interfaces between core Autoware.Auto components.

## Conventions

### Comments

Add a comment to describe the meaning of the field.

### Default Value

Prefer a meaningful default value. Otherwise, the field is uninitialized.

### Optional parameters

There is nothing like `std::optional` in IDL, unfortunately. To accomodate the common use case of a
fixed message where some variables are not always filled, add an additional boolean variable with a
prefix `has_` and a default value of `FALSE`.

### Units

If a quantity described by a field has an associated unit of measurement, the following rules apply to determine the field name:

1. If the unit is as base or [derived SI unit](https://en.wikipedia.org/wiki/International_System_of_Units#Derived_units), do not add a suffix and assume the default from [REP-103](https://www.ros.org/reps/rep-0103.html).
1. If the unit is a multiple of a base or derived SI unit, apply a suffix according to the table below.
1. If the unit is composed of non-SI units, apply a suffix according to the table below.

Only deviate from the SI units when absolutely necessary and with justification.

| Quantity        | Unit                        | Suffix  | Notes                                                   |
|-----------------|-----------------------------|---------|---------------------------------------------------------|
| distance        | meters                      | None    |                                                         |
|                 | micrometers                 | `_um`   |                                                         |
|                 | millimeters                 | `_mm`   |                                                         |
|                 | kilometers                  | `_km`   |                                                         |
| speed, velocity | meters / second             | None    | Use speed for scalar and velocity for vector quantities |
|                 | kilometers / hour           | `_kmph` |                                                         |
| acceleration    | meters / second<sup>2</sup> | None    |                                                         |
| radial velocity | radians / second            | None    |                                                         |
| time            | second                      | None    |                                                         |
|                 | microsecond                 | `_us`   |                                                         |
|                 | nanosecond                  | `_ns`   |                                                         |

#### Examples

The first alternative is recommended, the second discouraged:

1. `float elapsed_time` vs `float_elapsed_time_s`
1. `float distance_travelled` vs `float distance_travelled_m`
1. `int32 time_step_ms` vs `int32 time_step`
1. `float speed_mps` vs `float speed`

### Minimal Example

```idl
struct Foo {
  @verbatim (language="comment", text=
    " A multiline" "\n"
    " comment")
  @default (value=0.0)
  float bar_speed;

  @verbatim (language="comment", text=
    " Another multiline" "\n"
    " comment")
  @default (value=FALSE)
  boolean has_bar_speed;

  @verbatim (language="comment", text=
    " Describe the time stamp")
  @default (value=0)
  int32 timestamp_ns;
};
```
