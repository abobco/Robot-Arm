# Robot-Arm

WIP project for controlling a 5 DOF robotic arm with computer vision.

![arm](videos/arm.gif)

The arm consists of 6 servo motors and a camera, which are controlled with a raspberry pi zero W and a windows laptop. The raspberry pi functions as a microcontroller, sending and receiving signals to the motors and camera. Because the raspberry pi would take a very long time to perform object detection on these images, they are instead sent to a more powerful computer(my laptop) via a TCP socket. This computer evaluates a convolutional neural network to classify objects in this photo, and if the requested object label is found (such as "bottle"), the predicted position of this object is sent back to the microcontroller to be picked up by the arm.

The basic process:

1. The microcontroller communicates with the camera via Serial Peripheral Interface, sending instructions to take photos and receiving photos as compressed jpeg files 
2. The microcontroller sends each jpeg file to a windows computer via a TCP socket for image processing
3. The windows machine performs image segmentation, object classification, and a homography transformation to estimate the 3D position of the target object (a distance vector in milimeters relative to the anchor point of the arm) [more details](/Windows)
4. The windows machine sends the position of the target object to the microcontroller via a TCP socket
5. The microcontroller evaluates an inverse kinematics solver to get the required joint angles to reach the target position. You can find the source code for the inverse kinematics algorithm in [RaspberryPi/src/XNlib/xn_ik.hpp](/RaspberryPi/src/XNlib/xn_ik.hpp)
6. The microcontroller instructs the servo motors to move to the required joint angles, close the gripper, and return to their starting angles.


This project uses the following libraries:
* OpenCV - Image processing, computer vision algorithms
* Darknet - "You Only Look Once" object recognition convolutional neural network
* Pigpio - Raspberry pi gpio library