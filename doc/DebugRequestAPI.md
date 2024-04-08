# Debug Request API

Input for this driver can also be triggered from an external application, via the use of the OpenVR debug interface.

This interface implements a JSON request-response mechanism allowing for the setting of all inputs and settings that
this driver supports.

## Request Payload Structure

The response payload is a JSON object with fields as follows, and is also described as
a [JSON Schema](../dev-resources/json/debug_response_schema.json) included with this project.

### `inputs`

This object contains the input data for the system. It has the following properties:

- `time_offset`: A floating-point number representing the time offset in seconds from now.

- `paths`: An object containing a unique input path and its corresponding value:
    - Example: `{"/input/index_pinch/value": 0.5}` sets `index_pinch` to halfway.
    - Example: `{"/input/thumbstick/x": 0.1}` sets the `thumbstick` X component to `0.1`

### `settings`

This object defines the settings and trackers to be used in the system. These settings correspond to the settings
available in the configuration (sometimes with a slightly altered form to allow entering vectors more eloquently).

- `tracking_mode`: A string specifying the tracking mode. For example, `"hmd"`.

- `hmd_tracker_offset` and `desktop_tracker_offset`: An array of three floating-point numbers that signify offsets for
  the
  Hand-Mounted Display (HMD) tracker or desktop tracker (in meters). For example: `[1.0, 2.0, 3.0]`.

- `enable_elbow_trackers`: A boolean indicating whether elbow trackers are enabled. `true` indicates enabled and `false`
  indicates disabled.

- `external_input_only`: A boolean indicating whether only external inputs should be used. When `true`, only external
  inputs are used (other than for finger curl and skeleton bones).

- `extended_hand_profile`:  A boolean indicating whether the extended hand-profile should be enabled.

### Example

```json
{
  "inputs": {
    "time_offset": -0.123,
    "paths": {
      "/input/index_pinch/value": 0.5,
      "/input/grip/value": 0.5,
      "/input/thumbstick/x": 0.1,
      "/input/thumbstick/y": 0.2
    }
  },
  "settings": {
    "tracking_mode": "hmd",
    "hmd_tracker_offset": [1.0, 2.0, 3.0],
    "desktop_tracker_offset": [1.0, 2.0, 3.0],
    "enable_elbow_trackers": true,
    "external_input_only": false
  }
}
```

## Response Payload Structure

The response payload structure also a JSON object as follows, and is also described as
a [JSON Schema](../dev-resources/json/debug_response_schema.json).

### `result`

A string that indicates the result of the performed action.

- `"success"` if the operation was successful.

- `"error"` if the request failed.

### `errors`

An array of string messages, each representing an error that occurred during the operation. If no errors occurred, this
array will be either empty or omitted.

### `warnings`

An array of string messages, each representing a warning that was identified during the operation. If there were no
warnings, this array will be either empty or omitted.

### Example

```json
{
  "result": "success",
  "errors": [
    "Sample error message"
  ],
  "warnings": [
    "Sample warning message"
  ]
}
```