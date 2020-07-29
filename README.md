# RoboRIO SDK Readme

## Overview

This respository is for the CANBridge software that is run on non-roboRIO platforms. 

## Build Requirements

1. Git
2. Visual Studio Code or some other equivalent IDE
3. Gradle Build Tool 
4. Java JDK/JRE

All of these, excluding Git, can be installed and configured with the [WPILib Installer](https://github.com/wpilibsuite/allwpilib/releases), which will include specific versions for building FRC robot code. Gradle needs a C++ compiler, which is included with VS Code. 

## Building and Publishing

Build and publish steps are done using the Gradle wrapper, `gradlew`. The Gradle wrapper is located in the root of the project, so Gradle commands must be run from there. 

1. Clone this repository and open in VS Code
   - When VS Code first opens, select `Add workspace folder...` underneath `Start` on the Welcome Screen
2. Open the VS Code terminal
   -  `View -> Terminal` or ``Ctrl+` ``
3. Run `./gradlew build` from root
4. Run `./gradlew publish` from root

The output folders will be generated under `~\releases\maven\`.

## Linux

This branch is a work in progress. The latest firmware version will work with Linux if the socketCAN and gs_usb drivers are enabled. The following udev rule will work to enable this:

/etc/udev/rules.d/

```
ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="a30e", RUN+="/sbin/modprobe gs_usb" RUN+="/bin/sh -c 'echo 0483 a30e > /sys/bus/usb/drivers/gs_usb/new_id'"
```

## Changelog

The SDK Changelog can be viewed with [Changelog.md](Changelog.md).

