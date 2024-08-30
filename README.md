# CANBridge

## Overview

This repository is for the CANBridge software that is run on non-roboRIO platforms.

## Behavior

When sending a frame with a given interval, the behavior when setting a new interval is as follows:

The first time a frame is scheduled with an interval, it will be sent at the next available time. The next instance of the same frame id will be sent after that interval, even if the interval has changed.

See the following pseudo-example:

```py
sendMessage(frame, 5000)
delay(1000)

updateFrameData(frame)
sendMessage(frame, 1000)
delay(500)

updateFrameData(frame)
sendMessage(frame, 2000)
```

In this case, the first frame will be scheduled immediately, the second will be scheduled 5 seconds later, and after that, subsequent frames will be scheduled every 2 seconds. Note
that any change to the data in the second call will not be sent, meaning the second call is essentially a no-op if a new call with different data is sent before the previous
interval is up. Sending a frame with an interval of `-1` will cancel the repeat, and not send the frame. Sending with an interval of `0` will schedule the new frame once, then stop repeating.

## Build Requirements

1. Git
2. Gradle Build Tool
3. Java JDK 11 or newer (we recommend [Azul](https://www.azul.com/downloads/#zulu) or [Eclipse Temurin](https://adoptium.net/temurin/))
4. A modern C++ compiler with C++20 support

> [!NOTE]
>
> All of these, excluding Git, can be installed and configured with the [WPILib Installer](https://github.com/wpilibsuite/allwpilib/releases), which will include specific versions for building FRC robot code. Gradle needs a C++ compiler, which is included with WPILib's development environment.

## Building and Publishing

Building is done using the Gradle wrapper, `gradlew`. The Gradle wrapper is located in the root of the project, so Gradle commands must be run from there.

1. Clone repository
2. Open a terminal inside the newly cloned repository
3. `./gradlew build`

The output is placed at `~\releases\maven\release\com\revrobotics\usb\CANBridge-cpp\<version>\`.

### Publishing a new version (for repository owners)

> [!NOTE]
>
> Before publishing a new version, run `./gradlew build` locally to run the tests. GitHub Actions ***cannot*** run the tests because they depend on having a USB CAN device connected (e.g. SPARK MAX motor controller).

1. Bump the version number in `publish.gradle` and `CANBridge.json`
2. Commit the version bump
3. Create a new tag named `vX.X.X` at that commit
4. Push the tag to GitHub
5. Wait for the draft release to be created (this is automatically done via GitHub Actions)
6. Add release notes to the draft release
7. Publish the draft release

## Linux

> [!NOTE]
>
> This branch is a work in progress, testing does show that it does work, however this still might be some issues. If you do run across those issues, please create an issue so we can diagnose.

The latest firmware version will work with Linux if the `SocketCAN` and `gs_usb` drivers are enabled. The following udev rule will work to enable this:

Create a new file named and located at: `/etc/udev/rules.d/99-canbridge.rules`

```text
ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="a30e", RUN+="/sbin/modprobe gs_usb" RUN+="/bin/sh -c 'echo 0483 a30e > /sys/bus/usb/drivers/gs_usb/new_id'"
```

## Changelog

The SDK Changelog can be viewed with [Changelog.md](Changelog.md).
