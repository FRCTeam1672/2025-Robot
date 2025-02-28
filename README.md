# Team 1672 REEFSCAPE

The currently work in progress codebase which we used for the 2025 FIRST Robotics Competition Game, REEFSCAPE, based on the YAGSL-Example project.

Electrical Robot Specifications: 
* SDS MK4i Modules 
* SPARK MAX
* Pigeon 2 for Field Oriented Drive
* CANCoders

## Branches
| Main | 
|------|
| The main code currently used for testing on our robot.

## PhotonVision
Our robot uses a [Beelink Mini PC](https://www.bee-link.com/catalog/product/buy?id=303) running PhotonVision to be able to detect AprilTags. 
3 [Arducam OV9281](https://www.arducam.com/product/arducam-100fps-global-shutter-usb-camera-board-1mp-720p-ov9281-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-for-computer-laptop-android-device-and-raspberry-pi/)  cameras for real-time apriltag detection. Cameras are orienting fowards and backwards.

## Automated Scoring App && Operator Controller
The robot gets information for where to autoscore based on values published to NT4 by the [Reefscape Scoring App.](https://github.com/FRCTeam1672/Reefscape-Scoring-App) Please visit the repo for more information on the codebase. 

The operator controller is only used as a backup controls in the event the scoring app does not work, or we lose complete field odometry. The operator controller also controls the climb subsystem. 
