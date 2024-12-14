
#ifndef STEWART_H
#define STEWART_H

#include <stdio.h>
#include <math.h>

/*      Definitions     */

#define R 300       // Base radius
#define r 100       // platform radius
#define h 500      // minimum height difference
#define beta 10     // beta angle (deg)
#define PI 3.14159265358979323846




typedef struct stewart {

    float c_target[6];      // cylinder target position
    float c_actual[6];      // cylinder actual position 
    float a1[3];
    float a2[3];
    float a3[3];
    float a4[3];
    float a5[3];
    float a6[3];
    float b1[3];
    float b2[3];
    float b3[3];
    float b4[3];
    float b5[3];
    float b6[3];
    float tp_target_pos[6];    // postion and orientation of tool point [x,y,z,theta x, theta y, theta z]
    float xyz_limit;
    float tilt_limit;
    float rot_limit;

} stewart;

// function declarations 
float deg_to_rad(float deg);

void platform_init(stewart* stewart);

void rot_x(float theta_x, float vector[3]);

void rot_y(float theta_y, float vector[3]);

void rot_z(float theta_z, float vector[3]);

void rotate_platform(stewart* stewart, float vector[3]);

void run_platform(stewart* stewart);



#endif
