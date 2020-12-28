# Robot-Arm

WIP project for controlling a 5 DOF robotic arm with computer vision.

![arm](videos/arm.gif)

The arm consists of 6 servo motors and a camera, which are controlled with a raspberry pi zero W and a windows laptop as follows:

1. The raspberry pi communicates with the camera via SPI, sending instructions to take photos and receiving compressed jpeg files from the camera in a loop
2. The raspberry pi sends the jpeg file to a windows computer for image processing
3. The windows machine performs image segmentation, object recognition, and a homography transformation to estimate the 3D position of the target object (distance vector in milimeters from a known point on the checkerboard) [details](/Windows)
4. The raspberry pi performs inverse kinematics to solve for the required servo angles to reach the target position. You can find the source code for the inverse kinematics algorithm in [RaspberryPi/src/XNlib/xn_ik.hpp](/RaspberryPi/src/XNlib/xn_ik.hpp)
5. The raspberry pi instructs the arm to move to the target position, close the claw, and return to its neutral position.

This project uses the following libraries:
* OpenCV - Image processing, general computer vision algorithms
* Darknet - Real-time object recognition algorithm
* Pigpio - Raspberry pi gpio and PWM helper library