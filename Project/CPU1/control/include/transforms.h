/*
 * this file is Structure Definition and Data transforms
 *
 *
 *
 */

#ifndef CONTROL_INCLUDE_transforms_H_
#define CONTROL_INCLUDE_transforms_H_

/***************Structure Definition*******************/

typedef struct{
    float a;
    float b;
    float c;
    float alpha;
    float beta;
    float zero;
    float d;
    float q;
    float z;
}ABC_AB0_DQ0_Struct;

typedef struct{
    float sin_theta;
    float cos_theta;
    float theta;
}THETA_SIN_COS_Struct;

/******************************************************/

/*****************Data Transforms**********************/

inline void ABC_AB0_DQ0_Struct_Init(ABC_AB0_DQ0_Struct& v){
    v.a = 0;
    v.b = 0;
    v.c = 0;
    v.alpha = 0;
    v.beta = 0;
    v.zero = 0;
    v.d = 0;
    v.q = 0;
    v.z = 0;
}

inline void THETA_SIN_COS_Struct_Init(THETA_SIN_COS_Struct& v){
    v.sin_theta = 0;
    v.cos_theta = 1;
    v.theta = 0;
}

/******************************************************/

#endif
