/*
 * @file     cmd_fn.c
 * @brief    collection of executables functions from defined known_commands[]
 *
 * @author   Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */


#include <string.h>

#include "Generic.h"
#include "cmd.h"
#include "cmd_fn.h"
#include "instance.h"

#include "hal_drivers.h"
//-----------------------------------------------------------------------------
const char CMD_FN_RET_OK[] = "ok\r\n";
const char CMD_FN_RET_ERR[] = "error\r\n";

#if !defined(DYNAMIC_STR_DEBUG_ME)
char staticstr[MAX_STR_SIZE];
#endif


/*
 * ����:����
 *
 * */
REG_FN(f_help_app)
{
    int        indx = 0;
    const char * ret = NULL;
    char *str = (char *)CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        while (known_commands[indx].name != NULL)
        {
            sprintf(str,"%s \r\n", known_commands[indx].name);

            port_tx_msg((uint8_t*)str, strlen(str));

            indx++;
        }

        CMD_FREE(str);
        ret = CMD_FN_RET_OK;
    }

    return (ret);
}

//-----------------------------------------------------------------------------
/*
 * ����:��ȡ�汾
 *
 * */
REG_FN(f_getver)
{
    char str[64];
	memset(str, 0, sizeof(str));

	sprintf(str,"getver software:%s,hardware:%s\r\n",uwb_software_ver,uwb_hardware_ver);    // reserve space for length of JS object
    port_tx_msg(str, strlen(str));	
    return (CMD_FN_RET_OK);		
}

/*
 * ����:��λ
 *
 * */
REG_FN(f_restart)
{
	NVIC_SystemReset();
	return (CMD_FN_RET_OK);	
}

/*
 * ����:�ָ�����ģʽ
 *
 * */
REG_FN(f_restore)
{
	App_Module_sys_para_Init();			
	App_Module_Sys_Write_NVM();	
	HalDelay_nMs(100);	
	NVIC_SystemReset(); 	
	return (CMD_FN_RET_OK);		
}

/*
 * ����:��������
 *
 * */
REG_FN(f_save)
{
	App_Module_Sys_Write_NVM();	
	HalDelay_nMs(100);	
	NVIC_SystemReset();	
    return (CMD_FN_RET_OK);
}


/*
 * ����:����������Ϣ
 *
 * */
REG_FN(f_setcfg)
{
	char ret[16];
    char str[256];
	memset(str, 0, sizeof(str));	
	
	uint8_t role = 0, ch = 0, rate = 0, id = 0; 
	int n = 0;
	char tmp[10];

	n = sscanf(text, "%8s %d %d %d %d",tmp, &id, &role, &ch, &rate);
	
	if((role == 0 || role == 1) &&
		(ch == 0  || ch == 1)   &&
		(rate == 0|| rate == 1) &&
		(id <= MAX_TAG_LIST_SIZE) &&
		(n == 5))
	{
		//ID
		sys_para.device_switch &= ~(0x000000FE);
		sys_para.device_switch |= (id)<<1;	
		//��ɫ
		sys_para.device_switch &= ~(0x00000100);		
		sys_para.device_switch |= (role)<<8;	
		//�ŵ�
		sys_para.device_switch &= ~(0x00000200);		
		sys_para.device_switch |= (ch)<<9;
		//����
		sys_para.device_switch &= ~(0x00000400);		
		sys_para.device_switch |= (rate)<<10;		
		
		//���ñ�־λ
		sys_para.device_switch |= 0x1 << 11;
		
		memcpy(ret, CMD_FN_RET_OK, sizeof(CMD_FN_RET_OK));
	}
	else
	{
		memcpy(ret, CMD_FN_RET_ERR, sizeof(CMD_FN_RET_ERR));
	}	
	
	sprintf(str,"getcfg ID:%d, Role:%d, CH:%d, Rate:%d\r\n",
		(sys_para.device_switch&0x000000FE) >> 1,
		(sys_para.device_switch&0x00000100) >> 8,
		(sys_para.device_switch&0x00000200) >> 9,	
		(sys_para.device_switch&0x00000400) >> 10);			
	port_tx_msg(str, strlen(str));
	
	return (ret);//(CMD_FN_RET_OK);
}

/*
 * ����:��ȡ������Ϣ
 *
 * */
REG_FN(f_getcfg)
{
    char str[256];
	memset(str, 0, sizeof(str));

	if(sys_para.device_switch == 0)
	{
		sprintf(str,"getcfg ID:255, Role:255, CH:255, Rate:255\r\n");		
	}
	else
	{
		sprintf(str,"getcfg ID:%d, Role:%d, CH:%d, Rate:%d\r\n",
				(sys_para.device_switch&0x000000FE) >> 1,
				(sys_para.device_switch&0x00000100) >> 8,
				(sys_para.device_switch&0x00000200) >> 9,	
				(sys_para.device_switch&0x00000400) >> 10);		
	}
    port_tx_msg(str, strlen(str));	
	return (CMD_FN_RET_OK);
}



/*
 * ����:���û�վ����
 *
 * */
REG_FN(f_setCoord)
{
	uint8_t str[256];
	memset(str, 0, sizeof(str));	
	
	char tmp[10];
	uint8_t addr = 0, n = 0;
	bool An_enable;
	double x,y,z; 
	
	n = sscanf(text, "%8s %d %d %lf %lf %lf",tmp, &addr, &An_enable, &x, &y, &z);
	
	if(n != 6 || addr > 7)
	{
		return CMD_FN_RET_ERR;
	}
	
	sys_para.loc.Anc_coord[addr].enable = An_enable;
	sys_para.loc.Anc_coord[addr].x = x;
	sys_para.loc.Anc_coord[addr].y = y;
	sys_para.loc.Anc_coord[addr].z = z;

	sprintf(str ,"getcoord A%d enable:%d x:%.2lf y:%.2lf z:%.2lf\r\n",addr, sys_para.loc.Anc_coord[addr].enable,sys_para.loc.Anc_coord[addr].x, sys_para.loc.Anc_coord[addr].y, sys_para.loc.Anc_coord[addr].z);
	
	instance_update_coord();
	port_tx_msg(str, strlen(str));
	
	return (CMD_FN_RET_OK);		
}


/*
 * ����:��ȡ��վ����
 *
 * */
REG_FN(f_getCoord)
{
	uint8_t str[512];
	memset(str, 0, sizeof(str));
	
	uint8_t addr = 0, n = 0;
	char tmp[10];
	
	n = sscanf(text, "%8s %d",tmp, &addr);
		
	if(n != 2 || addr > 7)
	{
		return CMD_FN_RET_ERR;
	}
	
	sprintf(&str[strlen(str)],"getcoord A%d enable:%d x:%.2lf y:%.2lf z:%.2lf\r\n",addr, sys_para.loc.Anc_coord[addr].enable, sys_para.loc.Anc_coord[addr].x, sys_para.loc.Anc_coord[addr].y, sys_para.loc.Anc_coord[addr].z);

	port_tx_msg(str, strlen(str));
	
	return (CMD_FN_RET_OK);		
}


/*
 * ����:���û�������
 *
 * */
REG_FN(f_setdev)
{
	uint8_t str[256];
	memset(str, 0, sizeof(str));	
	
    //(���ñ�ǩ����,�����ӳ�,�˲�����,�˲�ϵ��Q,�˲�ϵ��R,У��ϵ��a,У��ϵ��b,��λ����,��λά��)
	uint8_t cap = 0, kalman_enable = 0 , loc_enable = 0, loc_dimen = 0;
	float kalman_Q, kalman_R, para_a, para_b;
	int antdelay = 0;
	
	int n = 0;
	char tmp[10];

	n = sscanf(text, "%8s %d %d %d %f %f %f %f %d %d",tmp, &cap, &antdelay, &kalman_enable, &kalman_Q, &kalman_R, &para_a, &para_b, &loc_enable, &loc_dimen);
	
	if((n == 10))
	{
		sys_para.dist.AntennaDelay = antdelay;
		sys_para.dist.is_kalman_filter = kalman_enable;
		sys_para.dist.kalman_Q = kalman_Q;
		sys_para.dist.kalman_R = kalman_R;
		sys_para.dist.para_a = para_a;
		sys_para.dist.para_b = para_b;
		sys_para.loc.is_loc = loc_enable;
		sys_para.loc.loc_dimen = loc_dimen;
        sys_para.device_max_tag_list_size = cap;
		
		instance_config_antennadelays(sys_para.dist.AntennaDelay,sys_para.dist.AntennaDelay);
		
		
		sprintf(str, "getdev cap:%d anndelay:%d, kalman_enable:%d, kalman_Q:%.3f, kalman_R:%.3f, para_a:%.4f, para_b:%.2f, pos_enable:%d, pos_dimen:%d\r\n",
			sys_para.device_max_tag_list_size, sys_para.dist.AntennaDelay, sys_para.dist.is_kalman_filter, 
			sys_para.dist.kalman_Q, sys_para.dist.kalman_R, 
			sys_para.dist.para_a, sys_para.dist.para_b,
			sys_para.loc.is_loc, sys_para.loc.loc_dimen);

		port_tx_msg(str, strlen(str));

	}		

	return (CMD_FN_RET_OK);
}

/*
 * ����:��ȡ��������
 *
 * */
REG_FN(f_getdev)
{
    //(��ȡ��ǩ����,�����ӳ�,�˲�����,�˲�ϵ��Q,�˲�ϵ��R,У��ϵ��a,У��ϵ��b,��λ����,��λά��)
    char str[256];
	memset(str, 0, sizeof(str));

	sprintf(str, "getdev cap:%d anndelay:%d, kalman_enable:%d, kalman_Q:%.3f, kalman_R:%.3f, para_a:%.4f, para_b:%.2f, pos_enable:%d, pos_dimen:%d\r\n",
			sys_para.device_max_tag_list_size, sys_para.dist.AntennaDelay, sys_para.dist.is_kalman_filter, 
			sys_para.dist.kalman_Q, sys_para.dist.kalman_R, 
			sys_para.dist.para_a, sys_para.dist.para_b,
			sys_para.loc.is_loc, sys_para.loc.loc_dimen);

    port_tx_msg(str, strlen(str));		
	
    return (CMD_FN_RET_OK);
}


REG_FN(f_sendcmd)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	uint16_t addr = 0;  //��ַ 0xffff�㲥 
    uint8_t type = 0;   //���� 0:ʮ������ 1:�ַ�����ʽ
    uint8_t transport_cmd_tmp[64];
    uint8_t transport_cmd[32];
	memset(transport_cmd, 0, sizeof(transport_cmd));
	memset(transport_cmd_tmp, 0, sizeof(transport_cmd_tmp));
    
	int n = 0;
	char tmp[10];

	n = sscanf(text, "%8s %04X %d %s",tmp, &addr, &type, transport_cmd_tmp);
	
	if((n == 4) && 
       (type == 0 || type == 1)
       )
	{	
//	    //ʮ������
//	    if(type == 0)
//        {   
//            osal_Str2Byte(transport_cmd_tmp, transport_cmd, strlen(transport_cmd_tmp));
//            inst->trasport_cmd.send_cmdlen = strlen(transport_cmd_tmp)/2;			
//        }
//        //ascii��
//        else			
        {
            memcpy(transport_cmd, transport_cmd_tmp, strlen(transport_cmd_tmp));        
            inst->trasport_cmd.send_cmdlen = strlen(transport_cmd_tmp);			
            
//            _dbg_printf("ascii:%s",transport_cmd);
        }


        memcpy(inst->trasport_cmd.send_cmd, transport_cmd, inst->trasport_cmd.send_cmdlen);
        inst->trasport_cmd.send_enable = true;
        inst->trasport_cmd.send_addr = addr;
	}
    else
    {
        return (CMD_FN_RET_ERR);
    }

    return (CMD_FN_RET_OK);    
}



/*
 * ����:���Թ�������
 *
 * */
REG_FN(f_test)
{
//    _dbg_printf("sizeof(General_t):%d\n", sizeof(General_t));
	return (CMD_FN_RET_OK);
}


//-----------------------------------------------------------------------------
/* list of known commands:
 * NAME, allowed_MODE,     REG_FN(fn_name)
 * */
const command_t known_commands []= {

    {"HELP",		f_help_app},		//����
    {"SAVE",		f_save},			//��������
	{"GETVER",		f_getver},			//��ȡ�汾��		
	{"RESTART",		f_restart},			//����
	{"RESTORE",		f_restore},			//�ָ�����ģʽ
	{"SETCFG",		f_setcfg},			//����������Ϣ
	{"GETCFG",		f_getcfg},			//��ȡ������Ϣ
    {"SETCOORD",    f_setCoord},        //���û�վ����
    {"GETCOORD",    f_getCoord},        //��ȡ��վ����
    {"SETDEV",  	f_setdev},      	//���û�������(���ñ�ǩ����,�����ӳ�,�˲�����,�˲�ϵ��Q,�˲�ϵ��R,��λ����,��λά��)
    {"GETDEV",  	f_getdev},      	//���û�������(���ñ�ǩ����,�����ӳ�,�˲�����,�˲�ϵ��Q,�˲�ϵ��R,��λ����,��λά��)
    {"SENDCMD",     f_sendcmd},         //�����Զ�������
    
    {"TEST",        f_test},            //����
    {NULL,         NULL}
};
