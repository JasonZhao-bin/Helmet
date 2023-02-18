#include "tag_dist.h"


typedef struct
{
    float x_last[MAX_NUM_TAGS][MAX_NUM_ANCS];
    float p_last[MAX_NUM_TAGS][MAX_NUM_ANCS];
    float Q;
    float R;
    float kg[MAX_NUM_TAGS][MAX_NUM_ANCS];
    float x_mid[MAX_NUM_TAGS][MAX_NUM_ANCS];
    float x_now[MAX_NUM_TAGS][MAX_NUM_ANCS];
    float p_mid[MAX_NUM_TAGS][MAX_NUM_ANCS];
    float p_now[MAX_NUM_TAGS][MAX_NUM_ANCS];
}kalman_t;


kalman_t kalman;

void processTagRangeReports_KalmanFilter(int tid, int *range, int *filter_range,int mask, float Q, float R)
{
    kalman.Q = Q;
    kalman.R = R;
    for(int k=0; k < MAX_NUM_ANCS; k++)
    {
        if((0x1 << k) & mask) //we have a valid range
        {
            //kalman filter
            kalman.x_mid[tid][k]  = kalman.x_last[tid][k];
            kalman.p_mid[tid][k]  = kalman.p_last[tid][k] + kalman.Q;
            kalman.kg[tid][k]	   = kalman.p_mid[tid][k] / (kalman.p_mid[tid][k] + kalman.R);
            //z_measure = z_real + frand()*0.3;//
            kalman.x_now[tid][k]  = kalman.x_mid[tid][k] + kalman.kg[tid][k]*(range[k] - kalman.x_mid[tid][k]);
            kalman.p_now[tid][k]  = (1 - kalman.kg[tid][k])*kalman.p_mid[tid][k];
            kalman.p_last[tid][k] = kalman.p_now[tid][k];
            kalman.x_last[tid][k] = kalman.x_now[tid][k];
            filter_range[k] = kalman.x_now[tid][k];
        }
        else
        {
        }
    }
}




