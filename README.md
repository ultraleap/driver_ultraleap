<p align="center">
  <a href="https://ultraleap.com#gh-light-mode-only"><img width="250px" alt="Ultraleap" src="doc/UltraleapLogoColor.png" /></a>
  <a href="https://ultraleap.com#gh-dark-mode-only"><img width="250px" alt="Ultraleap" src="doc/UltraleapLogoDarkMode.png" /></a>
  <br />
  <img height="32px" width="32px" alt="LMC" src="ultraleap/resources/icons/lmc_status_ready@2x.png" />
  <img height="32px" width="32px" alt="SIR170" src="ultraleap/resources/icons/sir170_status_ready@2x.png" />
  <img height="32px" width="32px" alt="3di" src="ultraleap/resources/icons/3di_status_ready@2x.png" />
  <img height="32px" width="32px" alt="LMC2" src="ultraleap/resources/icons/lmc2_status_ready@2x.png" />
</p>

<span align="center">

# SteamVR Driver

</span>

This is the official Ultraleap driver for SteamVR, supporting using your hands as a first class input system.

## Supported devices

This driver supports the follow Ultraleap/Leap Motion devices

* Leap Motion Controller
* Leap Motion Controller 2
* Ultraleap 3di
* Ultraleap StereoIR 170

## Installation

- Install the latest version of
  the [Ultraleap Tracking Service](https://developer.leapmotion.com/tracking-software-download).
- Download the release of the [Ultraleap SteamVR Driver](https://github.com/rblenkinsopp/driver_ultraleap/releases).
- Extract the build to a well known path you can reference in a later step.
- Navigate to `${SteamInstallFolder}/steamapps/common/SteamVR/bin/win64` and open a console in this folder.
- In the console type the command `vrpathreg.exe adddriver "{PathToExtractedBuild}"`.
- Now when you type `vrpathreg.exe show` you should see `ultraleap` under `External Drivers:`

## Configuration

The SteamVR driver supports a number of configuration options which are details in
the [configuration guide](doc/Configuration.md).

## Uninstallation

To remove the driver:

- Navigate to `${SteamInstallFolder}/steamapps/common/SteamVR/bin/win64` and open a console in this folder.
- In the console type the command `vrpathreg.exe removedriverswithname ultraleap`.
- The driver will now be removed from your SteamVR install.

## Advanced Topics

This driver allows supports [external input triggering via the OpenVR Debug Request API](doc/DebugRequestAPI.md). This
allows other applications or support programs to drive the inputs through this driver, allowing for novel interactions
and overlays.