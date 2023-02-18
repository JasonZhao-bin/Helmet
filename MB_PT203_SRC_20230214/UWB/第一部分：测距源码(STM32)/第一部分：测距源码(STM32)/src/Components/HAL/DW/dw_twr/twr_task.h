#ifndef __TWR_TASK_H
#define __TWR_TASK_H

#ifdef __cplusplus
extern "C"
{
#endif




#if 1
	#define SWS1_SHF_MODE 0x0002	//short frame mode (6.81M)
	#define SWS1_CH5_MODE 0x0004	//channel 5 mode
	#define SWS1_ANC_MODE 0x0008	//anchor mode
#else
	#define SWS1_SHF_MODE 0x02	//short frame mode (6.81M)
	#define SWS1_CH5_MODE 0x04	//channel 5 mode
	#define SWS1_ANC_MODE 0x08  //anchor mode
	#define SWS1_A1A_MODE 0x10  //anchor/tag address A1
	#define SWS1_A2A_MODE 0x20  //anchor/tag address A2
	#define SWS1_A3A_MODE 0x40  //anchor/tag address A3
#endif

static void tag_distance_cpy(void);

static void tag_position_cal(void);

static void tag_output(void);
	
static int twr_init(void);

static void twr_run(void);

extern void twr_task(void);

extern void instance_update_coord(void);


#ifdef __cplusplus
}
#endif
#endif
