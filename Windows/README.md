# Image processing steps:

1. Receive a photo from the raspberry pi via a local area network TCP socket for processing
2. Find a known, flat pattern that lies on the same plane as the target object(a checkerboard on the floor)
3. Using the distortion of features found on the checkerboard pattern, solve for the homography matrix between the camera's viewing plane and the floor. This will allow us to find the distance vector (in milimeters) from a stationary point on the checkerboard to an object that rests on the floor.
4. Get a labeled bounding box for all recognizable objects in the image, in 2d image coordinates
5. If the target object is found, transform a point on the bottom-center of its bounding box 3d space with the previously found homography matrix. I assume that the input point lies on the same plane as the checkerboard pattern, so the unknown "height" of the output point is set to 0.
6. The 3d coordinates of the object (in milimeters relative to a point on the checkerboard) are sent to the raspberry pi