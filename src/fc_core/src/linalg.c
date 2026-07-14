/*
 * linalg.c (verbatim port from UAV26 STM32 firmware; Author: Segang).
 */

#include "fc_core/linalg.h"

float max(float a, float b){
    if (a > b) return a;
    else return b;
}

float min(float a, float b){
    if (a < b) return a;
    else return b;
}

float normalize_angle(float angle) {
    float result = fmodf(angle + PI, 2 * PI);
    if (result < 0) {
        result += 2 * PI;
    }
    return result - PI;
}

float clampfloat(float value, float minv, float maxv){
    return fminf(fmaxf(value, minv), maxv);
}

double clampdouble(double value, double minv, double maxv){
    return fmin(fmax(value, minv), maxv);
}

uint16_t clampuint16t(uint16_t value, uint16_t minv, uint16_t maxv){
    return min(max(value, minv), maxv);
}

vec3d clampvec3d(vec3d value, float minv, float maxv){
    vec3d res;
    res.x = clampdouble(value.x, minv, maxv);
    res.y = clampdouble(value.y, minv, maxv);
    res.z = clampdouble(value.z, minv, maxv);
    return res;
}

float saturation(float value){
    return tanh(value);
}

float sign(float val){
    if(val > 0.0) return 1.0;
    else if(val < 0.0) return - 1.0;
    else return 0;
}

controlpack cpinit(vec3d _LMN, float _thrust){
    controlpack res;
    res._LMN = _LMN;
    res._thrust = _thrust;
    return res;
}

vec2d vec2(float x, float y) {
    vec2d c;
    c.x = x;
    c.y = y;
    return c;
}

vec2d addv2(vec2d a, vec2d b) {
    vec2d c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    return c;
}

vec2d subv2(vec2d a, vec2d b) {
    vec2d c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    return c;
}

vec2d mulf2(float a, vec2d b) {
    vec2d c;
    c.x = a * b.x;
    c.y = a * b.y;
    return c;
}

vec2d mulv2(vec2d a, vec2d b) {
    vec2d c;
    c.x = a.x * b.x;
    c.y = a.y * b.y;
    return c;
}

vec3d vec(float x, float y, float z){
    vec3d a;
    a.x = x;
    a.y = y;
    a.z = z;
    return a;
}

vec3d addv(vec3d a, vec3d b){
    vec3d c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
}

vec3d subv(vec3d a, vec3d b){
    vec3d c;
    c.x = a.x - b.x;
    c.y = a.y - b.y;
    c.z = a.z - b.z;
    return c;
}

vec3d mulf(float a, vec3d b){
    vec3d c;
    c.x = a * b.x;
    c.y = a * b.y;
    c.z = a * b.z;
    return c;
}

vec3d mulv(vec3d a, vec3d b){
    vec3d c;
    c.x = a.x * b.x;
    c.y = a.y * b.y;
    c.z = a.z * b.z;
    return c;
}

float mag2(vec2d a){
    return sqrtf(( a.x * a.x ) + ( a.y * a.y ));
}

float mag(vec3d a){
    return sqrtf(( a.x * a.x ) + ( a.y * a.y ) + ( a.z * a.z ));
}

float dot2(vec2d a, vec2d b){
    return ( a.x * b.x ) + ( a.y * b.y );
}

float cross2(vec2d a, vec2d b) {
    return a.x * b.y - a.y * b.x;
}

vec2d hat2(vec2d a) {
    float magnitude = mag2(a);
    if (magnitude < 1e-6) {
        return (vec2d){0.0f, 0.0f};
    }
    float mul = 1.0f / magnitude;
    return mulf2(mul, a);
}

vec2d absv2(vec2d a){
    vec2d c;
    c.x = fabs(a.x);
    c.y = fabs(a.y);
    return c;
}

float dot(vec3d a, vec3d b){
    return ( a.x * b.x ) + ( a.y * b.y ) + ( a.z * b.z );
}

vec3d cross(vec3d a, vec3d b) {
    vec3d c;
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

vec3d hat(vec3d a) {
    float magnitude = mag(a);
    if (magnitude < 1e-6) {
        return (vec3d){0.0f, 0.0f, 0.0f};
    }
    float mul = 1.0f / magnitude;
    return mulf(mul, a);
}

vec3d absv(vec3d a){
    vec3d c;
    c.x = fabs(a.x);
    c.y = fabs(a.y);
    c.z = fabs(a.z);
    return c;
}

vec3d clampvec(vec3d data, vec3d minv, vec3d maxv){
    vec3d c;
    c.x = clampdouble(data.x, minv.x, maxv.x);
    c.y = clampdouble(data.y, minv.y, maxv.y);
    c.z = clampdouble(data.z, minv.z, maxv.z);
    return c;
}

vec3d deadbandvec(vec3d value, vec3d bound){
    vec3d c;
    c.x = value.x * (fabs(value.x) >= bound.x);
    c.y = value.y * (fabs(value.y) >= bound.y);
    c.z = value.z * (fabs(value.z) >= bound.z);
    return c;
}

vec3d rotz90(vec3d v){
    return vec(-v.y,  v.x, v.z);
}

vec3d rotz180(vec3d acc){
    return vec(-acc.x, -acc.y, acc.z);
}

vec3d rotz270(vec3d v){
    return vec( v.y, -v.x, v.z);
}

mat2 mat2init(vec2d row1, vec2d row2){
    mat2 c;
    c.row1 = row1;
    c.row2 = row2;
    return c;
}

mat2 addm2(mat2 a, mat2 b){
    mat2 c;
    c.row1 = addv2(a.row1, b.row1);
    c.row2 = addv2(a.row2, b.row2);
    return c;
}

mat2 subm2(mat2 a, mat2 b){
    mat2 c;
    c.row1 = subv2(a.row1, b.row1);
    c.row2 = subv2(a.row2, b.row2);
    return c;
}

mat2 mulmf2(float a, mat2 b){
    mat2 c;
    c.row1 = mulf2(a, b.row1);
    c.row2 = mulf2(a, b.row2);
    return c;
}

mat2 mulm2(mat2 a, mat2 b) {
    mat2 c;
    c.row1.x = a.row1.x * b.row1.x + a.row1.y * b.row2.x;
    c.row1.y = a.row1.x * b.row1.y + a.row1.y * b.row2.y;
    c.row2.x = a.row2.x * b.row1.x + a.row2.y * b.row2.x;
    c.row2.y = a.row2.x * b.row1.y + a.row2.y * b.row2.y;
    return c;
}

mat matinit(vec3d row1, vec3d row2, vec3d row3){
    mat c;
    c.row1 = row1;
    c.row2 = row2;
    c.row3 = row3;
    return c;
}

mat addm(mat a, mat b){
    mat c;
    c.row1 = addv(a.row1, b.row1);
    c.row2 = addv(a.row2, b.row2);
    c.row3 = addv(a.row3, b.row3);
    return c;
}

mat subm(mat a, mat b){
    mat c;
    c.row1 = subv(a.row1, b.row1);
    c.row2 = subv(a.row2, b.row2);
    c.row3 = subv(a.row3, b.row3);
    return c;
}

mat mulmf(float a, mat b){
    mat c;
    c.row1 = mulf(a, b.row1);
    c.row2 = mulf(a, b.row2);
    c.row3 = mulf(a, b.row3);
    return c;
}

mat mulm(mat a, mat b){
    mat c;

    c.row1.x = a.row1.x * b.row1.x + a.row1.y * b.row2.x + a.row1.z * b.row3.x;
    c.row1.y = a.row1.x * b.row1.y + a.row1.y * b.row2.y + a.row1.z * b.row3.y;
    c.row1.z = a.row1.x * b.row1.z + a.row1.y * b.row2.z + a.row1.z * b.row3.z;

    c.row2.x = a.row2.x * b.row1.x + a.row2.y * b.row2.x + a.row2.z * b.row3.x;
    c.row2.y = a.row2.x * b.row1.y + a.row2.y * b.row2.y + a.row2.z * b.row3.y;
    c.row2.z = a.row2.x * b.row1.z + a.row2.y * b.row2.z + a.row2.z * b.row3.z;

    c.row3.x = a.row3.x * b.row1.x + a.row3.y * b.row2.x + a.row3.z * b.row3.x;
    c.row3.y = a.row3.x * b.row1.y + a.row3.y * b.row2.y + a.row3.z * b.row3.y;
    c.row3.z = a.row3.x * b.row1.z + a.row3.y * b.row2.z + a.row3.z * b.row3.z;

    return c;
}

mat tran(mat a) {
    mat at;

    at.row1.x = a.row1.x;
    at.row1.y = a.row2.x;
    at.row1.z = a.row3.x;

    at.row2.x = a.row1.y;
    at.row2.y = a.row2.y;
    at.row2.z = a.row3.y;

    at.row3.x = a.row1.z;
    at.row3.y = a.row2.z;
    at.row3.z = a.row3.z;

    return at;
}

vec2d mulmv2(mat2 a, vec2d b){
    vec2d c;
    c.x = a.row1.x * b.x + a.row1.y * b.y;
    c.y = a.row2.x * b.x + a.row2.y * b.y;
    return c;
}

vec3d mulmv(mat a, vec3d b) {
    vec3d c;

    c.x = a.row1.x * b.x + a.row1.y * b.y + a.row1.z * b.z;
    c.y = a.row2.x * b.x + a.row2.y * b.y + a.row2.z * b.z;
    c.z = a.row3.x * b.x + a.row3.y * b.y + a.row3.z * b.z;

    return c;
}

mat2 DCMB2I2(float theta) {
    mat2 res;
    float c = cos(theta), s = sin(theta);

    res.row1.x =  c;  res.row1.y = -s;
    res.row2.x =  s;  res.row2.y =  c;

    return res;
}

mat2 DCMI2B2(float theta) {
    mat2 res;
    float c = cos(theta), s = sin(theta);

    res.row1.x =  c;  res.row1.y =  s;
    res.row2.x = -s;  res.row2.y =  c;

    return res;
}

mat DCMI2B3(vec3d RPY) {
    float cr = cos(RPY.x), sr = sin(RPY.x);
    float cp = cos(RPY.y), sp = sin(RPY.y);
    float cy = cos(RPY.z), sy = sin(RPY.z);
    mat res;
    res.row1 = vec(cp * cy,                     cp * sy,                    -sp);
    res.row2 = vec(sr * sp * cy - cr * sy,      sr * sp * sy + cr * cy,     sr * cp);
    res.row3 = vec(cr * sp * cy + sr * sy,      cr * sp * sy - sr * cy,     cr * cp);
    return res;
}

mat DCMB2I3(vec3d RPY) {
    float cr = cos(RPY.x), sr = sin(RPY.x);
    float cp = cos(RPY.y), sp = sin(RPY.y);
    float cy = cos(RPY.z), sy = sin(RPY.z);

    mat res;
    res.row1 = vec(cp * cy,         sr * sp * cy - cr * sy,         cr * sp * cy + sr * sy);
    res.row2 = vec(cp * sy,         sr * sp * sy + cr * cy,         cr * sp * sy - sr * cy);
    res.row3 = vec(-sp,             sr * cp,                        cr * cp);

    return res;
}

mat DCMI2L(vec3d RPY) {
    float cp = cos(RPY.y), sp = sin(RPY.y);
    float ct = cos(RPY.z), st = sin(RPY.z);
    mat res;
    res.row1 = vec(cp * ct,     sp * ct,    -st);
    res.row2 = vec(-sp,         cp,          0);
    res.row3 = vec(cp * st,     sp * st,    ct);
    return res;
}

mat DCML2I(vec3d RPY) {
    mat res;
    float cp = cos(RPY.y), sp = sin(RPY.y);
    float ct = cos(RPY.z), st = sin(RPY.z);
    res.row1 = vec(cp * ct,     -sp,        cp * st);
    res.row2 = vec(sp * ct,     cp,         sp * st);
    res.row3 = vec(-st,         0,          ct);
    return res;
}

mat DCM1(float roll) {
    float c = cos(roll), s = sin(roll);
    mat R;
    R.row1 = vec(1, 0,  0);
    R.row2 = vec(0, c,  s);
    R.row3 = vec(0, -s, c);
    return R;
}

mat DCM2(float pitch) {
    float c = cos(pitch), s = sin(pitch);
    mat R;
    R.row1 = vec( c, 0, -s);
    R.row2 = vec( 0, 1,  0);
    R.row3 = vec( s, 0,  c);
    return R;
}

mat DCM3(float yaw) {
    float c = cos(yaw), s = sin(yaw);
    mat R;
    R.row1 = vec( c,  s, 0);
    R.row2 = vec(-s,  c, 0);
    R.row3 = vec( 0,  0, 1);
    return R;
}

float element(mat A, uint8_t i, uint8_t j) {
    if (i >= 3 || j >= 3) return 0.0;

    vec3d row;
    switch (i) {
        case 0: row = A.row1; break;
        case 1: row = A.row2; break;
        case 2: row = A.row3; break;
        default: return 0.0;
    }

    switch (j) {
        case 0: return row.x;
        case 1: return row.y;
        case 2: return row.z;
        default: return 0.0;
    }
}

quaternion quarinit(float re, vec3d im){
    quaternion res;
    res.re = re;
    res.im = im;
    return res;
}

quaternion addq(quaternion a, quaternion b){
    return quarinit(a.re + b.re, addv(a.im, b.im));
}

quaternion subq(quaternion a, quaternion b){
    return quarinit(a.re - b.re, subv(a.im, b.im));
}

quaternion mulfq(float a, quaternion b){
    return quarinit(a * b.re, mulf(a, b.im));
}

quaternion mulq(quaternion a, quaternion b){
    return quarinit(
        a.re * b.re - (a.im.x * b.im.x + a.im.y * b.im.y + a.im.z * b.im.z),
        vec(
            a.re * b.im.x + b.re * a.im.x + (a.im.y * b.im.z - a.im.z * b.im.y),
            a.re * b.im.y + b.re * a.im.y + (a.im.z * b.im.x - a.im.x * b.im.z),
            a.re * b.im.z + b.re * a.im.z + (a.im.x * b.im.y - a.im.y * b.im.x)
        )
    );
}

quaternion conjq(quaternion a){
    return quarinit(a.re, mulf(-1.0, a.im));
}

quaternion exp_halfq(vec3d v){
    quaternion res;
    res.re = cosf(mag(v) / 2.0f);
    res.im = mulf(sinf(mag(v) / 2.0f), hat(v));
    return res;
}

vec3d vecrot(vec3d a, quaternion b) {
    vec3d res;

    res.x   = a.x * (b.im.x * b.im.x + b.re * b.re - b.im.y * b.im.y - b.im.z * b.im.z)
            + a.y * (2 * b.im.x * b.im.y - 2 * b.re * b.im.z)
            + a.z * (2 * b.im.x * b.im.z + 2 * b.re * b.im.y);

    res.y = a.x * (2 * b.re * b.im.z + 2 * b.im.x * b.im.y)
             + a.y * (b.re * b.re - b.im.x * b.im.x + b.im.y * b.im.y - b.im.z * b.im.z)
             + a.z * (-2 * b.re * b.im.x + 2 * b.im.y * b.im.z);

    res.z = a.x * (-2 * b.re * b.im.y + 2 * b.im.x * b.im.z)
             + a.y * (2 * b.re * b.im.x + 2 * b.im.y * b.im.z)
             + a.z * (b.re * b.re - b.im.x * b.im.x - b.im.y * b.im.y + b.im.z * b.im.z);

    return res;
}

quaternion normalize(quaternion q) {
    double n = sqrt(q.re*q.re + q.im.x*q.im.x + q.im.y*q.im.y + q.im.z*q.im.z);
    if (n > 0.0) { q.re /= n; q.im.x /= n; q.im.y /= n; q.im.z /= n; }
    return q;
}

float norm(quaternion a){
    return sqrt(a.re * a.re + a.im.x * a.im.x + a.im.y * a.im.y + a.im.z * a.im.z);
}

/* KNOWN BUG (firmware parity): the returned pitch is sign-flipped; the
 * sim node compensates. Fix only with a paired firmware re-test. */
vec3d quat_to_euler(quaternion q) {
    vec3d eul;

    float qw = q.re;
    float qx = q.im.x;
    float qy = q.im.y;
    float qz = q.im.z;

    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx*qx + qy*qy);
    eul.x = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f)
       eul.y = -copysignf(PI / 2.0f, sinp);
    else
       eul.y = -asinf(sinp);

    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy*qy + qz*qz);
    eul.z = atan2f(siny_cosp, cosy_cosp);

    return eul;
}

float randd(float mag) {
    return ((float)rand() / RAND_MAX) * 2.0 * mag - mag;
}

float computealpha(float dt, float spooltime){
    float k = 3.0 / spooltime;
    return 1.0 - exp(-k * dt);
}

thrvec smoothfollow(thrvec curr, thrvec targ, float alpha){
    thrvec res;
    res.T1 = curr.T1 + (targ.T1 - curr.T1) * alpha;
    res.T2 = curr.T2 + (targ.T2 - curr.T2) * alpha;
    res.T3 = curr.T3 + (targ.T3 - curr.T3) * alpha;
    res.T4 = curr.T4 + (targ.T4 - curr.T4) * alpha;
    return res;
}

vec3d GetAngle(vec3d vec3){
    vec3d res;
    res.x = atan2(vec3.y, vec3.x);
    res.y = atan2(-vec3.z, sqrt(vec3.x * vec3.x + vec3.y * vec3.y));
    res.z = 0.0;
    return res;
}

/* KNOWN BUG (firmware parity): every statement assigns res.x, so res.y
 * and res.z are uninitialized. No callers; must be fixed before use. */
vec3d GetAngle2Vec(vec3d ang2){
    vec3d res;
    res.x =  cos(ang2.y) * cos(ang2.x);
    res.x =  cos(ang2.y) * sin(ang2.x);
    res.x = -sin(ang2.y);
    return res;
}

mat GetAng2DCM(vec3d ang2){
    mat res;
    float psi = ang2.x; float cp = cos(psi); float sp = sin(psi);
    float the = ang2.y; float ct = cos(the); float st = sin(the);
    res.row1 = vec(ct * cp, ct * sp,    -st );
    res.row2 = vec(-sp,     cp,         0   );
    res.row3 = vec(st * cp, st * sp,    ct  );
    return res;
}
