/* 
    数学数值定义 & 基础运算 & 矩阵运算 函数定义 库文件
*/

#ifndef COMMOM_H
#define COMMOM_H

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

namespace sift{

/**********************************************
********           数值定义           ***********
***********************************************/
#define PI 3.141592653589793f
#define _2PI 6.283185307179586f
#define PI_4 0.785398163397448f
#define PI_3_4 2.356194490192345f
#define SQRT2 1.414213562373095f

/**********************************************
********           基础运算           ***********
***********************************************/
// 最大最小值
#define MAX(a, b) (a >= b ? a : b)
#define MIN(a, b) (a <= b ? a : b)

// 计算log2
inline float my_log2(float n)
{
    return (float)((log(n)) / 0.69314718055994530941723212145818);
}

// 快速Atan2()
#define EPSILON_F 1.19209290E-07F
inline float fast_atan2_f(float y, float x)
{
    float angle, r;
    float const c3 = 0.1821F;
    float const c1 = 0.9675F;
    float abs_y = fabsf(y) + EPSILON_F;

    if (x >= 0) {
        r = (x - abs_y) / (x + abs_y);
        angle = PI_4;
    }
    else {
        r = (x + abs_y) / (abs_y - x);
        angle = PI_3_4;
    }
    angle += (c3 * r * r - c1) * r;

    return (y < 0) ? _2PI - angle : angle;
}

// 快速Sqrt()
inline float fast_resqrt_f(float x)
{
    // 32-bit version
    union {
        float x;
        int i;
    } u;

    float xhalf = (float)0.5 * x;

    // convert floating point value in RAW integer
    u.x = x;

    // gives initial guess y0
    u.i = 0x5f3759df - (u.i >> 1);

    // two Newton steps
    u.x = u.x * ((float)1.5 - xhalf * u.x * u.x);
    u.x = u.x * ((float)1.5 - xhalf * u.x * u.x);
    return u.x;
}

inline float fast_sqrt_f(float x)
{
    return (x < 1e-8) ? 0 : x * fast_resqrt_f(x);
}


/**********************************************
********           矩阵运算           ***********
***********************************************/
//矩阵乘向量
#define MAT_DOT_VEC_3X3(p, m, v)                                               \
                                                                               \
    {                                                                          \
        p[0] = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];               \
        p[1] = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];               \
        p[2] = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];               \
    }

//计算Det
#define DETERMINANT_3X3(d, m)                                                  \
                                                                               \
    {                                                                          \
        d = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]);                 \
        d -= m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]);                \
        d += m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);                \
    }


//compute adjoint of matrix and scale, Computes adjoint of matrix m, scales it by s, returning a
#define SCALE_ADJOINT_3X3(a, s, m)                                             \
                                                                               \
    {                                                                          \
        a[0][0] = (s) * (m[1][1] * m[2][2] - m[1][2] * m[2][1]);               \
        a[1][0] = (s) * (m[1][2] * m[2][0] - m[1][0] * m[2][2]);               \
        a[2][0] = (s) * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);               \
                                                                               \
        a[0][1] = (s) * (m[0][2] * m[2][1] - m[0][1] * m[2][2]);               \
        a[1][1] = (s) * (m[0][0] * m[2][2] - m[0][2] * m[2][0]);               \
        a[2][1] = (s) * (m[0][1] * m[2][0] - m[0][0] * m[2][1]);               \
                                                                               \
        a[0][2] = (s) * (m[0][1] * m[1][2] - m[0][2] * m[1][1]);               \
        a[1][2] = (s) * (m[0][2] * m[1][0] - m[0][0] * m[1][2]);               \
        a[2][2] = (s) * (m[0][0] * m[1][1] - m[0][1] * m[1][0]);               \
    }

} // end namespace sift
#endif