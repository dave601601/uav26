/*
 * linalg.h
 *
 * Ported verbatim from the UAV26 STM32 firmware (Author: Segang).
 * No HAL dependency.
 */

#ifndef FC_CORE_LINALG_H_
#define FC_CORE_LINALG_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PI                  3.14159265358979323846f
#define MS2KMH              3.6
#define KMH2MS              (1.0 / 3.6)

#define ms                  (1.0 / 1000.0)

#define D2R                 (PI / 180.0)
#define R2D                 (180.0 / PI)

#define g0  9.80665f

float max(float a, float b);
float min(float a, float b);

typedef struct {
    float T1;
    float T2;
    float T3;
    float T4;
} thrvec;

typedef struct {
    uint16_t T1;
    uint16_t T2;
    uint16_t T3;
    uint16_t T4;
} thrvec16;

typedef struct{
    float x;
    float y;
} vec2d;

typedef struct {
    float x;
    float y;
    float z;
} vec3d;

typedef struct {
    vec2d row1;
    vec2d row2;
} mat2;

typedef struct {
    vec3d row1;
    vec3d row2;
    vec3d row3;
} mat;

typedef struct{
    float re;
    vec3d im;
} quaternion;

typedef struct{
    vec3d _LMN;
    float _thrust;
} controlpack;

typedef struct{
    uint16_t row;
    uint16_t col;
} mat_size_t;

typedef struct{
    uint32_t nnz;
    uint32_t ncol;
    uint32_t nrow;
    uint32_t *row_idx;
    uint32_t *col_ptr;
    float *values;
} CCS;

typedef struct {
    uint32_t nrow;
    uint32_t ncol;
    uint32_t nnz;
    uint32_t capacity;

    uint32_t *row;
    uint32_t *col;
    float    *val;
} COO;

typedef struct {
    uint32_t *data;
    uint32_t  len;
    uint32_t  cap;
} u32vec;

float clampfloat(float value, float minv, float maxv);
double clampdouble(double value, double minv, double maxv);
uint16_t clampuint16t(uint16_t value, uint16_t minv, uint16_t maxv);
vec3d clampvec3d(vec3d value, float minv, float maxv);

float saturation(float value);
float sign(float val);

vec2d vec2(float x, float y);
vec2d addv2(vec2d a, vec2d b);
vec2d subv2(vec2d a, vec2d b);
vec2d mulf2(float a, vec2d b);
vec2d mulv2(vec2d a, vec2d b);

vec3d vec(float x, float y, float z);
vec3d addv(vec3d a, vec3d b);
vec3d subv(vec3d a, vec3d b);
vec3d mulf(float a, vec3d b);
vec3d mulv(vec3d a, vec3d b);

float mag2(vec2d a);
float mag(vec3d a);

float dot2(vec2d a, vec2d b);
float cross2(vec2d a, vec2d b);
vec2d hat2(vec2d a);
vec2d absv2(vec2d a);

float dot(vec3d a, vec3d b);
vec3d cross(vec3d a, vec3d b);
vec3d hat(vec3d a);
vec3d absv(vec3d a);

vec3d clampvec(vec3d data, vec3d minv, vec3d maxv);
vec3d deadbandvec(vec3d value, vec3d bound);

vec2d mulmv2(mat2 a, vec2d b);
vec3d mulmv(mat a, vec3d b);

vec3d rotz90(vec3d v);
vec3d rotz180(vec3d v);
vec3d rotz270(vec3d v);

mat2 mat2init(vec2d row1, vec2d row2);
mat2 addm2(mat2 a, mat2 b);
mat2 subm2(mat2 a, mat2 b);
mat2 mulmf2(float a, mat2 b);
mat2 mulm2(mat2 a, mat2 b);

mat matinit(vec3d row1, vec3d row2, vec3d row3);
mat addm(mat a, mat b);
mat subm(mat a, mat b);
mat mulmf(float a, mat b);
mat mulm(mat a, mat b);

mat tran(mat a);

mat2 DCMB2I2(float theta);
mat2 DCMI2B2(float theta);

mat DCMI2B3(vec3d RPY);
mat DCMB2I3(vec3d RPY);
mat DCMI2L(vec3d RPY);
mat DCML2I(vec3d RPY);

mat DCM1(float roll);
mat DCM2(float pitch);
mat DCM3(float yaw);

float element(mat A, uint8_t i, uint8_t j);

quaternion quarinit(float re, vec3d im);
quaternion addq(quaternion a, quaternion b);
quaternion subq(quaternion a, quaternion b);
quaternion mulfq(float a, quaternion b);
quaternion mulq(quaternion a, quaternion b);
quaternion conjq(quaternion a);
quaternion exp_halfq(vec3d v);
vec3d vecrot(vec3d a, quaternion b);
quaternion normalize(quaternion q);
vec3d quat_to_euler(quaternion q);

float norm(quaternion a);

controlpack cpinit(vec3d _LMN, float _thrust);

float randd(float mag);
float computealpha(float dt, float spooltime);
thrvec smoothfollow(thrvec curr, thrvec targ, float alpha);

vec3d GetAngle(vec3d vec3);
vec3d GetAngle2Vec(vec3d ang2);
mat GetAng2DCM(vec3d ang2);

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_LINALG_H_ */
