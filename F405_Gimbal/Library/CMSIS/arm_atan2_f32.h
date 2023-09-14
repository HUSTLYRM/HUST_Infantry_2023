//根据官网所给库修改得

#ifndef _ARM_ATAN2_F32_
#define _ARM_ATAN2_F32_

#include <string.h>
#include <stdint.h>

#ifndef TRUE
#define TRUE 1
#endif // !
#ifndef FALSE
#define FALSE 0
#endif // !
#ifndef PI
#define PI 3.14159265358979f
#endif

float arm_atan2_f32(float y, float x);
#endif // !ARM_ATAN2_F32
