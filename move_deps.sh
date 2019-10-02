#! /bin/bash

RELEASE_DIR=~/releases/maven/release/com/revrobotics/usb
MOVE_DIR=C:/Users/Public/frc2019/maven/com/revrobotics/usb
VENDOR_DIR=C:/Users/Public/frc2019/vendordeps

echo "*** Moving maven directories ***"
cp -r $RELEASE_DIR/CANBridge-cpp $MOVE_DIR
cp -r $RELEASE_DIR/CANBridge-java $MOVE_DIR

echo "*** Moving vendor deps ***"
cp -r ./vendordeps/* $VENDOR_DIR
