#! /bin/bash

BUILD_YEAR=2020
BUILD_REPO_DIR=./build/repos
RELEASE_DIR=$BUILD_REPO_DIR/releases/com/revrobotics/usb
MOVE_DIR=C:/Users/Public/wpilib/$BUILD_YEAR/maven/com/revrobotics/usb
VENDOR_DIR=C:/Users/Public/wpilib/$BUILD_YEAR/vendordeps

echo "*** Moving maven directories ***"
cp -r $RELEASE_DIR/CANBridge-cpp $MOVE_DIR
cp -r $RELEASE_DIR/CANBridge-java $MOVE_DIR

echo "*** Moving vendor deps ***"
cp -r ./vendordeps/* $VENDOR_DIR
