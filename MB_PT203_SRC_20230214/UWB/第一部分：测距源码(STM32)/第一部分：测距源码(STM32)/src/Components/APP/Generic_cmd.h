#ifndef __GENERIC_AT_H
#define __GENERIC_AT_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * 																				INCLUDES
 **************************************************************************************************/	
#include "OSAL_Comdef.h"
	
#include <stdbool.h>	
/**************************************************************************************************
 * 																				CONSTANTS
 **************************************************************************************************/		

#define SOP_STATE       0x00
#define LEN_STATE       0x01
#define DATA_STATE      0x02
#define FCS_STATE       0x03
#define END_STATE       0x04

/***************************************************************************************************
 * 																				TYPEDEF
 ***************************************************************************************************/
#pragma pack(push,1)


typedef struct
{
    uint32_t    timer;
    uint16_t     tagid;
    uint16_t     ancid;
    uint8_t     seq;
    uint8_t     mask;
    int         rawrange[8];
    bool        kalman_enable;
    int         kalmanrange[8];
    bool        pos_enable;
    uint8_t     dimen;
    uint8_t     ancmask;	
    bool        Coord_valid;
    struct{
        float x;
        float y;
        float z;
    }pos_loc;
}General_t;//General_variable;


typedef struct
{
    uint8_t header[6];		//"CmdM:5"
    uint8_t length;
    General_t gerneal_para;
    uint8_t check;
    uint8_t footer[2];		//"\r\n"
}Msg_Hex_Output_t;  

#pragma pack(pop)


/***************************************************************************************************
 * 																				GLOBAL VARIABLES
 ***************************************************************************************************/
	
/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/	
extern int App_Module_CMD_Queue_Init(void);

extern int App_Module_format_conver_uint8(General_t msg, uint8_t *buf);

extern int App_Module_Process_USB_CMD(void);

extern int App_Module_Process_USART_CMD(void);

extern void port_tx_msg(uint8_t *buf, uint16_t len);
#ifdef __cplusplus
}
#endif
#endif//__GENERIC_AT_H

