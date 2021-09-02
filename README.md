Overview
========

The HID Over GATT Profile (HOGP), also known as HID over BLE, is a BLE service that implements the USB HID protol through BLE. This allows for the development of HID devices that use BLE as its standard form of communication instead of USB. This example project implements HOGP and demos the keyboard, consumer report, and mouse functionality. 

Usage
================

Import the HID project into CCS, build and load the code to the LaunchPad. Then, connect it via Bluetooth to another PC or a smartphone device. Once the HID is connected, pressing the left button will press the left arrow on the computer and lower the volume (if allowed), pressing the right button will press the right arrow on the computer and raise the volume (if allowed), and pressing both buttons will begin the mouse demo. The mouse demo can be terminated by pressing both buttons once more. Once the mouse demo is activated, the cursor will move in a rectangle shape until the demo is terminated.

Recent Changes
================

The following are recent fixes/additions that have been added to the HID project

* HIDDEVICE_TASK_STACK_SIZE has been increased to 1024.
* HIDEMUKBD_TASK_STACK_SIZE has been increased to 8192.
* Added additional logic to eliminate memory leak that presents when rapid reconnects occur.
* Fixed throughput drop issue that presented in noisy HID BLE environment.
* Fixed connection interval not being able to reach 7.5ms.
* Modified report map for HID mouse and added missing information.
* Reworked notification payload to use signed integers to better match the modified report map.
* Modified mouse movement logic to better suit signed integer payload.
* Ported to the CC13X2/26X2 5.10 SDK
