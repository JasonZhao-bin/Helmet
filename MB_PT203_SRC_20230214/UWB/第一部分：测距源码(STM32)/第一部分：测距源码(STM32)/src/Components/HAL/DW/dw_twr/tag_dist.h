#ifndef __TAG_DIST_F_H
#define __TAG_DIST_F_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "OSAL_Comdef.h"
#include "instance.h"

#define MAX_NUM_TAGS MAX_TAG_LIST_SIZE
#define MAX_NUM_ANCS MAX_ANCHOR_LIST_SIZE
extern void processTagRangeReports_KalmanFilter(int tid, int *range, int *filter_range,int mask, float Q, float R);


#ifdef __cplusplus
}
#endif
#endif//__TAG_DIST_F_H
