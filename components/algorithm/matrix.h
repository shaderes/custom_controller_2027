#ifndef __MATRIX_H
#define __MATRIX_H

void matrix_mul44(float mA[16], float mB[16], float mC[16]);
void matrix_mul33(float mA[9], float mB[9], float mC[9]);
void vec_add(float a[3], float b[3], float c[3]);
void vec_sub(float a[3], float b[3], float c[3]);
void vec_cross(float a[3], float b[3], float c[3]);
float vec_dot(const float a[3], const float b[3]); 
void vec_mul31(float a[3], float b, float c[3]);
void transform_point(float T[16], float P_in[3], float P_out[3]);

#endif 
