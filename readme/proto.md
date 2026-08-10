# Proto

---

## Struct

* struct
* self-reference
* `oneof`

```proto
syntax = "proto3";
package google.protobuf;

enum NullValue {
  NULL_VALUE = 0;
}

message Struct {
  map<string, Value> fields = 1;
}

message Value {
  oneof kind {
    NullValue null_value = 1;
    double number_value = 2;
    string string_value = 3;
    bool bool_value = 4;
    Struct struct_value = 5;
    ListValue list_value = 6;
  }
}

// wrap `Value[]` in message to allow us to include it in `oneof`
message ListValue {
  repeated Value values = 1;
}
```

---

## Message self-reference

```proto
syntax = "proto3";

package intrinsic_proto.motion_planning;

import "google/protobuf/struct.proto";

message TimeInterval {
  string name = 1;

  // Arbitrary data to be logged with the interval.
  google.protobuf.Struct data = 2;

  // Note, we do not use a oneof field here because you can't have a repeated
  // field inside a oneof. Code should ignore `seconds` if one or more
  // interval is present.
  double seconds = 3;
  repeated TimeInterval intervals = 4;
}
```

---

## Reserve

```proto
message MotionPlannerFlags {
  reserved 1 to 4;
  reserved "enable_jerk_limited_topp", 
           "enable_topp_structured_analytical_integrals", 
           "enable_adaptive_sampling_and_topp_integrals", 
           "enable_conversion_to_knot_via_single_approximation";

  // Actual fields would go here...
}
```

---

## Optional

```proto
message MotionSpecification {
  repeated MotionSegment motion_segments = 1;
  optional BlendingParameters curve_parameters = 3;
  optional HighLevelBlendingParameters default_blending_parameters = 4;
}
```

### 1. It Generates `has_...()` Methods

```cpp
MotionSpecification message;

// Check if the sender actually provided these parameters
if (message.has_curve_parameters()) {
    // Do something specific because curve parameters were explicitly sent
} else {
    // Fall back to a default logic or error out
}

```

### 2. It Generates `clear_...()` Methods

```cpp
// If you want to strip the default blending parameters from the message:
message.clear_default_blending_parameters();
assert(!message.has_default_blending_parameters()); // Now evaluates to true
```

### 3. Smart Memory Management (Pointers Under the Hood)

Because `BlendingParameters` and `HighLevelBlendingParameters` are sub-messages, the `MotionSpecification` C++ class will manage them using heap-allocated pointers under the hood to save memory.

* If you don't touch or set `curve_parameters`, its internal pointer is `nullptr`.
* The moment you call `message.curve_parameters()`, if it wasn't set, it will return a reference to a default, static instance (so it won't crash).
* If you want to safely mutate it, you use `mutable_curve_parameters()`, which allocates memory on the heap and automatically flips the `has_curve_parameters()` flag to `true`.

```cpp
// Allocates memory for HighLevelBlendingParameters and marks it as "present"
auto* blending = message.mutable_default_blending_parameters();
blending->set_blending_radius_in_meters(0.005); 
assert(message.has_default_blending_parameters()); // True
```

### Summary Table for C++ Usage

| Action | Code Example | Impact on `has_...()` |
| --- | --- | --- |
| **Check Presence** | `if (msg.has_curve_parameters())` | Returns `true` if set, `false` if not. |
| **Read Value** | `auto params = msg.curve_parameters();` | Safe to call anytime; returns default instance if unset. |
| **Modify Value** | `msg.mutable_curve_parameters()->set_...();` | Allocates memory; switches `has_...()` to `true`. |
| **Clear Field** | `msg.clear_curve_parameters();` | Deallocates/unsets field; switches `has_...()` to `false`. |

---
