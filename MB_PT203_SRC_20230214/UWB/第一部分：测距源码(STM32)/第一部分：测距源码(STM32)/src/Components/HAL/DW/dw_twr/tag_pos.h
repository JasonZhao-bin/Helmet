#ifndef __TAG_POS_H
#define __TAG_POS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "OSAL_Comdef.h"
#include <stdbool.h>

/*******************一维所需结构体********************/
struct cross_point_t {		// 点
    double x, y;
};


struct circle_t {			// 圆
    struct cross_point_t center;
    double r;
};

typedef enum{
    CIRCLE_CROSS_NONE = 0,
    CIRCLE_CROSS_ONE,
    CIRCLE_CROSS_TWO,
    CIRCLE_CROSS_INVALID = 0xff
}ret_circle_t;
/*********************************************/



typedef struct
{
    float x;
    float y;
    float z;
}vec3d_t;

extern bool Get_trilateration_1Dimen(double x1, double y1, double d1,
                            double x2, double y2, double d2,
                            vec3d_t *report);

extern bool Get_trilateration_2Dimen(double x1, double y1, double d1,
                                    double x2, double y2, double d2,
                                    double x3, double y3, double d3,
                                    vec3d_t *report);

extern bool Get_trilateration_3Dimen(double x1, double y1, double z1, double d1,
                              double x2, double y2, double z2, double d2,
                              double x3, double y3, double z3, double d3,
                              double x4, double y4, double z4, double d4,
                              vec3d_t *report);
	

	
	
	
	
	
#ifdef __cplusplus
}
#endif
#endif//__TAG_POS_H
