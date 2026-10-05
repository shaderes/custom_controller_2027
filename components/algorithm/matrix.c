#include "stm32f4xx_hal.h"
#include "matrix.h"

void matrix_mul44(float mA[16], float mB[16], float mC[16])
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            mC[i * 4 + j] =
                mA[i * 4 + 0] * mB[0 * 4 + j] +
                mA[i * 4 + 1] * mB[1 * 4 + j] +
                mA[i * 4 + 2] * mB[2 * 4 + j] +
                mA[i * 4 + 3] * mB[3 * 4 + j];
        }
    }
}

void matrix_mul33(float mA[9], float mB[9], float mC[9])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mC[i * 3 + j] =
                mA[i * 3 + 0] * mB[0 * 3 + j] +
                mA[i * 3 + 1] * mB[1 * 3 + j] +
                mA[i * 3 + 2] * mB[2 * 3 + j];
        }
    }
}

void vec_add(float a[3], float b[3], float c[3]) 
{
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
}

void vec_sub(float a[3], float b[3], float c[3]) 
{
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
}

void vec_cross(float a[3], float b[3], float c[3]) 
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

float vec_dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vec_mul31(float a[3], float b, float c[3]) 
{
    c[0] = a[0] * b;
    c[1] = a[1] * b;
    c[2] = a[2] * b;
}

/* 用 4x4 齐次变换矩阵（行主序，平移在 T[3]/T[7]/T[11]）变换一个点：P_out = R*P_in + t
   原实现是 P_out[0] = P_in[0]*T[0] + P_in[0]*T[4] + P_in[0]*T[8] + T[3] 这种写法，
   每行的三个乘积项都只用了 P_in[0]/P_in[1]/P_in[2] 里的同一个分量、且行列出错，
   只要质心不在原点就算错（link_center 全 0 时看不出来）。 */
void transform_point(float T[16], float P_in[3], float P_out[3])
{
    P_out[0] = T[0] * P_in[0] + T[1] * P_in[1] + T[2]  * P_in[2] + T[3];
    P_out[1] = T[4] * P_in[0] + T[5] * P_in[1] + T[6]  * P_in[2] + T[7];
    P_out[2] = T[8] * P_in[0] + T[9] * P_in[1] + T[10] * P_in[2] + T[11];
}


