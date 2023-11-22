# Ultraleap SteamVR Driver

This is the official Ultraleap driver for SteamVR, supporting using your hands as a first class input system.

[![Build](https://github.com/rblenkinsopp/driver_ultraleap/actions/workflows/build.yml/badge.svg)](https://github.com/rblenkinsopp/driver_ultraleap/actions/workflows/build.yml)

## Supported devices

This driver supports the follow Ultraleap/Leap Motion devices

| Device Name              | Icon                                                                      |
|:-------------------------|---------------------------------------------------------------------------|
| Leap Motion Controller   | ![Leap Motion Controller icon](resources/icons/lmc_status_ready.png)      |
| Leap Motion Controller 2 | ![Leap Motion Controller 2 icon](resources/icons/lmc2_status_ready.png)   |
| Ultraleap 3di            | ![Leap Motion Controller 2 icon](resources/icons/3di_status_ready.png)    |
| Ultraleap StereoIR 170   | ![Leap Motion Controller 2 icon](resources/icons/sir170_status_ready.png) |

## Installation
- Install the latest version of the [Ultraleap Tracking Service](https://developer.leapmotion.com/tracking-software-download).
- Download the release of the [Ultraleap SteamVR Driver](https://github.com/rblenkinsopp/driver_ultraleap/releases).
- Extract the build to a well known path you can reference in a later step.
- Navigate to `${SteamInstallFolder}/steamapps/common/SteamVR/bin/win64` and open a console in this folder.
- In the console type the command `vrpathreg.exe adddriver "{PathToExtractedBuild}"`.
- Now when you type `vrpathreg.exe show` you should see `ultraleap` under `External Drivers:`

## Configuration
The driver comes with some configurable options and predefined defaults which the user can change if desired.
To overwrite the defaults navigate to `${SteamInstallFolder}/config` and edit the `steamvr.vrsettings` file
whilst SteamVR **isn't** open.

When adding the following key value pair configurations, ensure that they are only added under the
`driver_ultraleap` section. Any values that aren't defined in `steamvr.vrsettings` will fallback to defaults
so you should only redefine the ones you actively want changed.

#### Key-Value pairs:
- `tracker_mode`: Forces the service to be set to the requested tracking mode. Values: `hmd`, `desktop`. Default: `hmd`.
- `hmd_offset_x`: tracker offset in the x-axis when mounted to an HMD in meters. Values: `{float}`. Default: `0.0`.
- `hmd_offset_y`: tracker offset in the y-axis when mounted to an HMD in meters. Values: `{float}`. Default: `0.0`.
- `hmd_offset_z`: tracker offset in the x-axis when mounted to an HMD in meters. Values: `{float}`. Default: `-0.08`.
- `desktop_offset_x`: tracker offset in the x-axis when on the tabletop in front of you in meters. Values: `{float}`. Default: `0.0`.
- `desktop_offset_y`: tracker offset in the y-axis when on the tabletop in front of you in meters. Values: `{float}`. Default: `-0.2`.
- `desktop_offset_z`: tracker offset in the x-axis when on the tabletop in front of you in meters. Values: `{float}`. Default: `-0.35`.

**TODO** - Make gnomon diagrams for both HMD and Desktop co-ordinate directions.

A simplified example of the `steamvr.vrsettings` would look like the following:

```
{
   "DesktopUI" : {
      "controllerbinding_desktop" : "151,39,1920,1073,0",
   },
   "GpuSpeed" : {
      "gpuSpeedRenderTargetScale" : 1.5,
      "gpuSpeedVendor" : "NVIDIA GeForce RTX 3080 Ti Laptop GPU"
   },
   "LastKnown" : {
      "ActualHMDDriver" : "holographic",
      "HMDManufacturer" : "WindowsMR",
      "HMDModel" : "Perception Simulation Headset0"
   },
   "driver_ultraleap" : {
      "blocked_by_safe_mode" : false,
      "orientation" : "Desktop"
   },
   "steamvr" : {
      "showAdvancedSettings" : true
   }
}
```

> **NOTE**: Trailing commas will cause the parsing of the settings file to break and automatically fall back
> to defaults. Ensure the last element in each section doesn't have a trailing comma!

## Uninstallation
To remove the driver:
- Navigate to `${SteamInstallFolder}/steamapps/common/SteamVR/bin/win64` and open a console in this folder.
- In the console type the command `vrpathreg.exe removedriverswithname ultraleap`.
- The driver will now be removed from your SteamVR install.