# Configuration

The driver comes with some configurable options and predefined defaults which the user can change if desired.
To overwrite the defaults navigate to `${SteamInstallFolder}/config` and edit the `steamvr.vrsettings` file
whilst SteamVR **isn't** open.

When adding the following key value pair configurations, ensure that they are only added under the
`driver_ultraleap` section. Any values that aren't defined in `steamvr.vrsettings` will fallback to defaults,
so you should only redefine the ones you actively want changed.

## Valid Configuration Keys

| Key                     | Description                                        | Type/Values      | Default |
|-------------------------|----------------------------------------------------|------------------|---------|
| `tracker_mode`          | Sets the requested tracking mode                   | `hmd`, `desktop` | `hmd`   |
| `hmd_offset_x`          | X-axis tracker offset for HMD mode                 | _meters_         | `0.0`   |
| `hmd_offset_y`          | Y-axis tracker offset for HMD mode                 | _meters_         | `0.0`   |
| `hmd_offset_z`          | Z-axis tracker offset for HMD mode                 | _meters_         | `-0.08` |
| `desktop_offset_x`      | X-axis tracker offset for desktop mode             | _meters_         | `0.0`   |
| `desktop_offset_y`      | Y-axis tracker offset for desktop mode             | _meters_         | `-0.2`  |
| `desktop_offset_z`      | Z-axis tracker offset for desktop mode             | _meters_         | `-0.35` |
| `enable_elbow_trackers` | Enable elbow trackers                              | `true`/`false`   | `true`  |
| `external_input_only`   | Disable driver input (for external input)          | `true`/`false`   | `false` |
| `extended_hand_profile` | Extended hand-profile support (for external input) | `true`/`false`   | `false` |

> **NOTE:** HMD offsets follow
> the [OpenXR View space convention](https://openxr-tutorial.com/windows/opengl/_images/ViewSpace.png).

## Example

A simplified example of the `steamvr.vrsettings` would look like the following:

```json
{
  "DesktopUI": {
    "controllerbinding_desktop": "151,39,1920,1073,0"
  },
  "GpuSpeed": {
    "gpuSpeedRenderTargetScale": 1.5,
    "gpuSpeedVendor": "NVIDIA GeForce RTX 3080 Ti Laptop GPU"
  },
  "LastKnown": {
    "ActualHMDDriver": "holographic",
    "HMDManufacturer": "WindowsMR",
    "HMDModel": "Perception Simulation Headset0"
  },
  "driver_ultraleap": {
    "blocked_by_safe_mode": false,
    "orientation": "Desktop"
  },
  "steamvr": {
    "showAdvancedSettings": true
  }
}
```

> **NOTE**: This settings file _must_ be valid JSON. Errors like additional trailing commas will cause issues
> when loading settings, resulting in default values being used instead. Ensure you check your JSON with a suitable
> [JSON validator](https://jsonlint.com/) before saving.
