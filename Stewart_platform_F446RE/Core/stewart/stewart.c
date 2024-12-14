/*
Ibrahim Dali Abo
file containing functions needed to drive a stewart manipulator

*/

#include "stewart.h"




void platform_init(stewart* stewart) {

    // base plate joints positions 
    stewart->b1[0]=R*cos(deg_to_rad(60-beta/2)),  stewart->b1[1]=R*sin(deg_to_rad(60-beta/2)),  stewart->b1[2]=h;
    stewart->b2[0]=R*cos(deg_to_rad(60+beta/2)),  stewart->b2[1]=R*sin(deg_to_rad(60+beta/2)),  stewart->b2[2]=h;
    stewart->b3[0]=R*cos(deg_to_rad(180-beta/2)), stewart->b3[1]=R*sin(deg_to_rad(180-beta/2)), stewart->b3[2]=h;
    stewart->b4[0]=R*cos(deg_to_rad(180+beta/2)), stewart->b4[1]=R*sin(deg_to_rad(180+beta/2)), stewart->b4[2]=h;
    stewart->b5[0]=R*cos(deg_to_rad(300-beta/2)), stewart->b5[1]=R*sin(deg_to_rad(300-beta/2)), stewart->b5[2]=h;
    stewart->b6[0]=R*cos(deg_to_rad(300+beta/2)), stewart->b6[1]=R*sin(deg_to_rad(300+beta/2)), stewart->b6[2]=h;

    stewart->tp_target_pos[0] = 0;
    stewart->tp_target_pos[1] = 0;
    stewart->tp_target_pos[2] = 0;
    stewart->tp_target_pos[3] = 0;
    stewart->tp_target_pos[4] = 0;
    stewart->tp_target_pos[5] = 0;

}


/*convert degrees to radians */
float deg_to_rad(float deg) {
    float rad = deg * PI / 180;
    return rad;
}


void rot_x(float theta_x, float vector[3]) {

    float vector_copy[3]={vector[0], vector[1], vector[2]};

    float rot_x[3][3] = {
        {1, 0, 0},
        {0, cos(deg_to_rad(theta_x)), -sin(deg_to_rad(theta_x))},
        {0, sin(deg_to_rad(theta_x)), cos(deg_to_rad(theta_x))}
    };

    vector[0]=0, vector[1]=0, vector[2]=0;

    for (int row=0;row<3;row++) {
        
        for (int column=0;column<3;column++) {
            vector[row] += rot_x[row][column] * vector_copy[column];  
            
        }
    }
}



void rot_y(float theta_y, float vector[3]) {

    float vector_copy[3]={vector[0], vector[1], vector[2]};

    float rot_y[3][3] = {
        {cos(deg_to_rad(theta_y)), 0, sin(deg_to_rad(theta_y))},
        {0, 1, 0},
        {-sin(deg_to_rad(theta_y)), 0, cos(deg_to_rad(theta_y))}
    };

    vector[0]=0, vector[1]=0, vector[2]=0;

    for (int row=0;row<3;row++) {
        
        for (int column=0;column<3;column++) {
            vector[row] += rot_y[row][column] * vector_copy[column];  
            
        }
    }
}


void rot_z(float theta_z, float vector[3]) {

    float vector_copy[3]={vector[0], vector[1], vector[2]};

    float rot_z[3][3] = {
        {cos(deg_to_rad(theta_z)), -sin(deg_to_rad(theta_z)), 0},
        {sin(deg_to_rad(theta_z)), cos(deg_to_rad(theta_z)), 0},
        {0, 0, 1},
    };

    vector[0]=0, vector[1]=0, vector[2]=0;

    for (int row=0;row<3;row++) {
        
        for (int column=0;column<3;column++) {
            vector[row] += rot_z[row][column] * vector_copy[column];  
            
        }
    }
}


void rotate_platform(stewart* stewart, float vector[3]) {

    rot_z(stewart->tp_target_pos[5], vector);
    rot_y(stewart->tp_target_pos[4], vector);
    rot_x(stewart->tp_target_pos[3], vector);

    vector[0] = vector[0] + stewart->tp_target_pos[0];
    vector[1] = vector[1] + stewart->tp_target_pos[1];
    vector[2] = vector[2] + stewart->tp_target_pos[2];

}





void run_platform(stewart* stewart)  {

    stewart->a1[0]=r*cos(deg_to_rad(0+beta/2)),   stewart->a1[1]=r*sin(deg_to_rad(0+beta/2)),    stewart->a1[2] = 0;
    stewart->a2[0]=r*cos(deg_to_rad(120-beta/2)), stewart->a2[1]=r*sin(deg_to_rad(120-beta/2)),  stewart->a2[2] = 0;
    stewart->a3[0]=r*cos(deg_to_rad(120+beta/2)), stewart->a3[1]=r*sin(deg_to_rad(120+beta/2)),  stewart->a3[2] = 0;
    stewart->a4[0]=r*cos(deg_to_rad(240-beta/2)), stewart->a4[1]=r*sin(deg_to_rad(240-beta/2)),  stewart->a4[2] = 0;
    stewart->a5[0]=r*cos(deg_to_rad(240+beta/2)), stewart->a5[1]=r*sin(deg_to_rad(240+beta/2)),  stewart->a5[2] = 0;
    stewart->a6[0]=r*cos(deg_to_rad(360-beta/2)), stewart->a6[1]=r*sin(deg_to_rad(360-beta/2)),  stewart->a6[2] = 0;


    rotate_platform(stewart, stewart->a1);
    rotate_platform(stewart, stewart->a2);
    rotate_platform(stewart, stewart->a3);
    rotate_platform(stewart, stewart->a4);
    rotate_platform(stewart, stewart->a5);
    rotate_platform(stewart, stewart->a6);


    // calculate the length of each arm 
    stewart->c_target[0]=sqrt(pow(stewart->a1[0]-stewart->b1[0],2)+pow(stewart->a1[1]-stewart->b1[1],2) + pow(stewart->a1[2]-stewart->b1[2],2));
    stewart->c_target[1]=sqrt(pow(stewart->a2[0]-stewart->b2[0],2)+pow(stewart->a2[1]-stewart->b2[1],2) + pow(stewart->a2[2]-stewart->b2[2],2));
    stewart->c_target[2]=sqrt(pow(stewart->a3[0]-stewart->b3[0],2)+pow(stewart->a3[1]-stewart->b3[1],2) + pow(stewart->a3[2]-stewart->b3[2],2));
    stewart->c_target[3]=sqrt(pow(stewart->a4[0]-stewart->b4[0],2)+pow(stewart->a4[1]-stewart->b4[1],2) + pow(stewart->a4[2]-stewart->b4[2],2));
    stewart->c_target[4]=sqrt(pow(stewart->a5[0]-stewart->b5[0],2)+pow(stewart->a5[1]-stewart->b5[1],2) + pow(stewart->a5[2]-stewart->b5[2],2));
    stewart->c_target[5]=sqrt(pow(stewart->a6[0]-stewart->b6[0],2)+pow(stewart->a6[1]-stewart->b6[1],2) + pow(stewart->a6[2]-stewart->b6[2],2));

}

