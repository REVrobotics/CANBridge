# RoboRIO SDK Readme

## Overview

This repository is for the CANBridge software that is run on non-roboRIO platforms. 

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
3. Run `./gradlew build -PreleaseMode` from root
4. Run `./gradlew publish -PreleaseMode` from root

The output is placed at `~\releases\maven\release\com\revrobotics\usb\CANBridge-cpp\<version>\`.

When publishing a new version, create a GitHub release and attach the following artifacts to it:

| Filename               | Location                                                                              |
|------------------------|---------------------------------------------------------------------------------------|
| `CANBridge-static.lib` | `CANBridge-cpp-<version>-windowsx86-64static.zip/windows/x86-64/static/CANBridge.lib` |
| `CANBridge.lib`        | `CANBridge-cpp-<version>-windowsx86-64.zip/windows/x86-64/shared/CANBridge.lib`       |
| `CANBridge.dll`        | `CANBridge-cpp-<version>-windowsx86-64.zip/windows/x86-64/shared/CANBridge.dll`       |


## Changelog

The SDK Changelog can be viewed with [Changelog.md](Changelog.md).

