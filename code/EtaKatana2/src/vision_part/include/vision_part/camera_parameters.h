#ifndef CAMERA_PARAMETERS_H
#define CAMERA_PARAMETERS_H
    /// For choosing the resolution HD set HD on value true, for SD set on false.
    #define HD true
    #if HD==true
        /// scale between camera pixels and meters in distance one meter from camera
        #define TMP_SCALE  9.0716/10000;
        #define FOCAL_LENGTH 483.8
        /// Half of the x size of the current picture resolution.
        #define X_MOVE 960
        /// Half of the x size of the current picture resolution.
        #define Y_MOVE 540
        /// Number of pixels on one cm with current picture resolution.
        #define PIXELS_TO_CM 11
        /// move between RGB and depth map in cm in axes X
        #define DEPTH_COLOUR_MOVE_X 4
        /// move between RGB and depth map in cm in axes Y
        #define DEPTH_COLOUR_MOVE_Y 1
    #else
        #define FOCAL_LENGTH 158.8
        #define TMP_SCALE  0.0028;
        #define X_MOVE 256
        #define Y_MOVE 212
        #define PIXELS_TO_CM 3.57
        #define DEPTH_COLOUR_MOVE_X 0
        #define DEPTH_COLOUR_MOVE_Y 0
    #endif

#endif // CAMERA_PARAMETERS_H
