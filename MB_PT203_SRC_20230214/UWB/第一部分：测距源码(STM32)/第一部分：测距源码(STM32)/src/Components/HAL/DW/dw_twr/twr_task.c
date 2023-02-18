/**********************************************************************************
 * @���ߣ�	�������
 * @�汾��	Twr(v3.0.0)
 * @���ڣ�	2021.10.11
 * @˵����	����� TWR����(stm32)
**********************************************************************************/
#include "twr_task.h"
#include "tag_dist.h"
#include "tag_pos.h"
#include "instance.h"
#include "Generic.h"
#include "uwb.h"
#include "OSAL.h"

#include "hal_usart.h"




typedef struct
{

    int mask;           //����ֵ��Чλ
    int rnum;           //UWB�����ź����кż���
    int lnum;           //����ֵ
    int anc_addr;       //��վ��ַ
    int tag_addr;       //��ǩ��ַ
    int newRangeTime;   //ʱ��(stm32)
    int trxa;           //�����ӳٲ���
    
    int dist[MAX_ANCHOR_LIST_SIZE];         //����ֵ ��λmm
    int filter_dist[MAX_ANCHOR_LIST_SIZE];  //�˲�����ֵ ��λmm

    struct{
        bool res;           //��λ���
        float x;            //��ǩ����õ�x����ֵ
        float y;            //��ǩ����õ�y����ֵ
        float z;            //��ǩ����õ�z����ֵ
    }tag_coord;

    dwt_rxdiag_t rx_diag;   //uwb�źŲ���

    uint8_t resv_trasport[16];//����͸��ָ��
}instance_otpt_t;


typedef struct {
	bool valid;         //�Ƿ���Ч
	float x;            //��վx����ֵ
	float y;            //��վy����ֵ
	float z;            //��վz����ֵ
}anc_coord_t;

static uint8_t anc_coord_mask;		//��վ��Чλ
static anc_coord_t anc_coord[MAX_ANCHOR_LIST_SIZE];

static instance_otpt_t instance_otpt;
int instance_mode = ANCHOR;
int instance_anchaddr = 0;
int dr_mode = 0;
int chan, tagaddr, ancaddr;

//Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
instanceConfig_t chConfig[4] ={
    //ģʽ1�� �ŵ�:2 �����ظ���:16M ��������:110k
    {
        .channelNumber = 2,             // �ŵ�2 channel
        .preambleCode = 4,              // ǰ���� preambleCode
        .pulseRepFreq = DWT_PRF_16M,    // �����ظ�Ƶ�� prf
        .dataRate = DWT_BR_110K,        // �������� datarate
        .preambleLen = DWT_PLEN_1024,   // preambleLength
        .pacSize = DWT_PAC32,           // pacSize
        .nsSFD = 1,                     // 0:�Ǳ�׼SFD, 1:��׼SFD 
        .sfdTO = (1025 + 64 - 32)       // SFD ��ʱ SFD timeout
    },
    //ģʽ2�� �ŵ�:2 �����ظ���:16M ��������:6.8M
    {
		.channelNumber = 2,            // �ŵ�2 channel
		.preambleCode = 4,             // ǰ���� preambleCode
		.pulseRepFreq = DWT_PRF_16M,   // �����ظ�Ƶ�� prf
		.dataRate = DWT_BR_6M8,        // �������� datarate
		.preambleLen = DWT_PLEN_128,   // preambleLength
		.pacSize = DWT_PAC8,           // pacSize
		.nsSFD = 0,                    // 0:�Ǳ�׼SFD, 1:��׼SFD
		.sfdTO = (129 + 8 - 8)         // SFD ��ʱ SFD timeout
    },
    //ģʽ3�� �ŵ�:5 �����ظ���:16M ��������:110K
    {
		.channelNumber = 5,             // �ŵ�5 channel
		.preambleCode = 3,              // ǰ���� preambleCode
		.pulseRepFreq = DWT_PRF_16M,    // �����ظ�Ƶ�� prf
		.dataRate = DWT_BR_110K,        // �������� datarate
		.preambleLen = DWT_PLEN_1024,   // preambleLength
		.pacSize = DWT_PAC32,           // pacSize
		.nsSFD = 1,                     // 0:�Ǳ�׼SFD, 1:��׼SFD
		.sfdTO = (1025 + 64 - 32)       // SFD ��ʱ SFD timeout
    },
    //ģʽ4�� �ŵ�:5 �����ظ���:16M ��������:6.8M
    {
		.channelNumber = 5,            // �ŵ�5 channel
		.preambleCode = 3,             // ǰ���� preambleCode
		.pulseRepFreq = DWT_PRF_16M,   // �����ظ�Ƶ�� prf
		.dataRate = DWT_BR_6M8,        // �������� datarate
		.preambleLen = DWT_PLEN_128,   // preambleLength
		.pacSize = DWT_PAC8,           // pacSize
		.nsSFD = 0,                    // 0:�Ǳ�׼SFD, 1:��׼SFD
		.sfdTO = (129 + 8 - 8)         // SFD ��ʱ SFD timeout
    }
};

//�ۺͳ���֡4��ģʽ Slot and Superframe Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
sfConfig_t sfConfig[4] ={
    //ģʽ1:
	{
		.slotDuration_ms = (40),        //����Slot����40ms
		.numSlots = (10),               //�۸���
		.sfPeriod_ms = (10*40),         //����֡����
		.tagPeriod_ms = (10*40),        //Poll->Poll����ʱ��
		.pollTxToFinalTxDly_us = (40000)//Poll->Final��ʱʱ��
	},
    //ģʽ2:
    {
		.slotDuration_ms = (10),        //����Slot����10ms
		.numSlots = (10),               //�۸���
		.sfPeriod_ms = (10*10),         //����֡����
		.tagPeriod_ms = (10*10),        //Poll->Poll����ʱ��
		.pollTxToFinalTxDly_us = (3800) //Poll->Final��ʱʱ��
    },
    //ģʽ3:
    {
		.slotDuration_ms = (40),        //����Slot����40ms
		.numSlots = (10),               //�۸���
		.sfPeriod_ms = (10*40),         //����֡����
		.tagPeriod_ms = (10*40),        //Poll->Poll����ʱ��
		.pollTxToFinalTxDly_us = (40000)//Poll->Final��ʱʱ��

    },
    //ģʽ4:
    {
		.slotDuration_ms = (10),        //����Slot����10ms
		.numSlots = (10),               //�۸���
		.sfPeriod_ms = (10*10),         //����֡����
		.tagPeriod_ms = (10*10),        //Poll->Poll����ʱ��
		.pollTxToFinalTxDly_us = (3800) //Poll->Final��ʱʱ��
    }
};


/*
 *  ����16bit�̵�ַ  
 */
void addressconfigure(uint16 s1switch, uint8 mode)
{
    uint16 instAddress ;

    instance_anchaddr = (sys_para.device_switch & 0x0fe) >> 1;

    if(mode == ANCHOR)
    {
    	if(instance_anchaddr > (MAX_ANCHOR_LIST_SIZE - 1)/*3*/)
		{
			instAddress = GATEWAY_ANCHOR_ADDR | MAX_ANCHOR_LIST_SIZE/*0x4*/ ; //listener
		}
		else
		{
			instAddress = GATEWAY_ANCHOR_ADDR | instance_anchaddr;
		}
	}
    else
    {
    	instAddress = instance_anchaddr;
    }

    instance_set_16bit_address(instAddress);
}


/*
 *  ��ȡ����ģʽ����         returns the use case / operational mode
 */
int decarangingmode(uint16 s1switch)
{
    int mode = 0;

    if(s1switch & SWS1_SHF_MODE)
    {
        mode = 1;
    }

    if(s1switch & SWS1_CH5_MODE)
    {
        mode = mode + 2;
    }

    return mode;
}


/*
 *  �޸�ʱ�����Ϣ
 */
void instance_update_sfConfig(uint16_t max_tag_list_size)
{
    for(int i = 0; i < 4; i++)
    {
        sfConfig[i].numSlots = max_tag_list_size;
        sfConfig[i].sfPeriod_ms = sfConfig[i].slotDuration_ms * sfConfig[i].numSlots;
        sfConfig[i].tagPeriod_ms = sfConfig[i].sfPeriod_ms;
    }    
}

/*
 *  instance_otp��վ��Ϣ����
 */
void instance_update_coord(void)
{
	anc_coord_mask = 0x00;

	for(int addr = 0; addr < MAX_ANCHOR_LIST_SIZE; addr++)
	{
		anc_coord_mask |= (sys_para.loc.Anc_coord[addr].enable << addr);

		anc_coord[addr].valid = sys_para.loc.Anc_coord[addr].enable;
		anc_coord[addr].x = sys_para.loc.Anc_coord[addr].x;
		anc_coord[addr].y = sys_para.loc.Anc_coord[addr].y;
		anc_coord[addr].z = sys_para.loc.Anc_coord[addr].z;	
	}
}

/*
 * ��һ����SPI��֤        
 * �ڶ�����DW1000��ʼ�� | instance_init(instance_mode)
 * �������������ŵ���Ϣ | instance_config(&chConfig[dr_mode], &sfConfig[dr_mode])
 */
uint32 inittestapplication(uint16_t s1switch)
{
    uint32 devID;
    int result;

    //����SPI����ֵ����3MHz
    port_set_dw1000_slowrate();//Ӳ�����

    //��ȡ�豸Dev IDֵ(SPI ������)
    devID = instance_readdeviceid();
    if(DWT_DEVICE_ID != devID)
    {
        //����dw1000    
    	port_wakeup_dw1000();

        //�ٴζ�ȡ�豸Dev IDֵ(SPI ������)
        devID = instance_readdeviceid() ;
        if(DWT_DEVICE_ID != devID)
            return(-1) ;

        //��λdw1000        
        dwt_softreset();
    }

    //Ӳ����λdw1000
    reset_DW1000();

    //�ж�����Ϊ��վ/��ǩ
    if((s1switch & SWS1_ANC_MODE) == 0)
        instance_mode = TAG;
    else
        instance_mode = ANCHOR;

    //�����豸��ɫ,��ʼ��dw1000
    result = instance_init(instance_mode) ; //��ǩ��ʼ��
    if (0 > result) return(-1) ; 

    //���SPI����ֵ20MHz
    port_set_dw1000_fastrate(); 
    //��ȡ�豸Dev IDֵ(SPI ������)    
    devID = instance_readdeviceid();

    if (DWT_DEVICE_ID != devID)   
    {
        return(-1) ;
    }

    //����16bit��ַ
    addressconfigure(s1switch, instance_mode) ; //��ȷ�������ַ���ã�

#ifdef HW_8Anc
    if((instance_mode == ANCHOR) && (instance_anchaddr > (MAX_ANCHOR_LIST_SIZE - 1)))
#else
    if((instance_mode == ANCHOR) && (instance_anchaddr > 0x3))
#endif
    {
        _dbg_printf("�Ƿ���ַ");
        return (-1);
        //�Ƿ�����
    }
    else
    {

		dr_mode = decarangingmode(s1switch);

		chan = chConfig[dr_mode].channelNumber ;
		//�����û����ã��޸�ʱ�������
		instance_update_sfConfig(sys_para.device_max_tag_list_size);		
        //�������ã�ѡ��sfconfig(ʱ�������)��chConfig(�ŵ�����)
		instance_config(&chConfig[dr_mode], &sfConfig[dr_mode]) ;
    }
    return devID;
}

/*
 *  ��ǩ���븳ֵ
 */
void tag_distance_cpy(void)
{    
    instance_otpt.anc_addr = instance_newrangeancadd() & 0xf;
    instance_otpt.tag_addr = instance_newrangetagadd() & 0xf;
	instance_otpt.newRangeTime = instance_newrangetim() & 0xffffffff;
    instance_otpt.lnum = instance_get_lcount() & 0xFFFF;
    if(instance_mode == TAG)
        instance_otpt.rnum = instance_get_rnum();
    else
        instance_otpt.rnum = instance_get_rnuma(instance_otpt.tag_addr);
	instance_otpt.trxa =  instance_get_txantdly();
	instance_otpt.mask = instance_validranges();

    for(int i = 0; i < MAX_ANCHOR_LIST_SIZE; i++)
    {
        instance_otpt.dist[i] = instance_get_idist_mm(i) * sys_para.dist.para_a + sys_para.dist.para_b;
    }

    instance_otpt.rx_diag = instance_newrangerx_diag();

    instance_newtrasport_cmd(instance_otpt.resv_trasport);

	instance_cleardisttableall();
}

/*
 *  ��ǩ�����˲�  
 */
void tag_distance_fil(void)
{
    if(sys_para.dist.is_kalman_filter)
    {
        processTagRangeReports_KalmanFilter(instance_otpt.tag_addr, 
            instance_otpt.dist, 
            instance_otpt.filter_dist,
            instance_otpt.mask,
            sys_para.dist.kalman_Q,
            sys_para.dist.kalman_R);
	}
}

/*
 *  ��ǩ�������  
 */
void tag_position_cal(void)
{
    if(sys_para.loc.is_loc)
    {
        int ranges[MAX_NUM_ANCS];
        int loc_num = 0;	
        int mask = instance_otpt.mask;
        vec3d_t tmp_report, tmp_report_sum;
        memset(&tmp_report, 0, sizeof(vec3d_t));
        memset(&tmp_report_sum, 0, sizeof(vec3d_t));

        if(sys_para.dist.is_kalman_filter)
        {
            memcpy(ranges, instance_otpt.filter_dist, MAX_NUM_ANCS*sizeof(int));
		}
        else
        {
            memcpy(ranges, instance_otpt.dist, MAX_NUM_ANCS*sizeof(int)); 
		}
        

        switch(sys_para.loc.loc_dimen)
        {
			//��ά��λ
            case Sys_Dimension_Two:
                {
    				for(int i = 0; i < (MAX_NUM_ANCS - 2); i++)
    				{
    					if(((mask >> i) & 0x01) != 0x01)
    						continue;
    					
    					for(int j = (i + 1); j < (MAX_NUM_ANCS - 1); j++)
    					{
    						if(((mask >> j) & 0x01) != 0x01)
    							continue;						
    						
    						for(int k = (j + 1); k < MAX_NUM_ANCS; k++)
    						{			
    							if(((mask >> k) & 0x01) != 0x01 )
    								continue;

                                if(anc_coord[i].valid == false || anc_coord[j].valid == false || anc_coord[k].valid == false)
                                    continue;
                                
    							if(Get_trilateration_2Dimen( anc_coord[i].x, anc_coord[i].y, ranges[i]/1000.0, 
    														 anc_coord[j].x, anc_coord[j].y, ranges[j]/1000.0, 
    														 anc_coord[k].x, anc_coord[k].y, ranges[k]/1000.0, 
    														  &tmp_report) == true)
    							{
    								tmp_report_sum.x += tmp_report.x;
    								tmp_report_sum.y += tmp_report.y;
    								tmp_report_sum.z += tmp_report.z;								
    								loc_num++;
//    								_dbg_printf("��ά��λ�ɹ� i:%d, j:%d, k:%d\n", i, j, k);
    							}
    							else
    							{
//    								_dbg_printf("��ά��λʧ�� i:%d, j:%d, k:%d\n", i, j, k);
    							}
    						}
    					}
    				}
    				
    				if(loc_num >0)
    				{
    					instance_otpt.tag_coord.x = tmp_report_sum.x/loc_num;
    					instance_otpt.tag_coord.y = tmp_report_sum.y/loc_num;
    					instance_otpt.tag_coord.z = tmp_report_sum.z/loc_num;					
    					instance_otpt.tag_coord.res = true;
    				}					
                }
                break;
				
			//��ά��λ	
            case Sys_Dimension_Three:
                {
    				for(int i = 0; i < (MAX_NUM_ANCS - 3); i++)
    				{
    					if(((mask >> i) & 0x01) != 0x01)
    						continue;
    					
    					for(int j = (i + 1); j < (MAX_NUM_ANCS - 2); j++)
    					{
    						if(((mask >> j) & 0x01) != 0x01)
    							continue;						
    						
    						for(int k = (j + 1); k < (MAX_NUM_ANCS - 1); k++)
    						{			
    							if(((mask >> k) & 0x01) != 0x01)
    								continue;

    							for(int l = (k + 1); l < MAX_NUM_ANCS; l++)
    							{
    								if(((mask >> l) & 0x01) != 0x01)
    									continue;								

                                    if(anc_coord[i].valid != true || anc_coord[j].valid != true || anc_coord[k].valid != true || anc_coord[l].valid != true)
                                        continue;
                                    
    								if(Get_trilateration_3Dimen( anc_coord[i].x, anc_coord[i].y, anc_coord[i].z, ranges[i]/1000.0, 
    															 anc_coord[j].x, anc_coord[j].y, anc_coord[j].z, ranges[j]/1000.0, 
    															 anc_coord[k].x, anc_coord[k].y, anc_coord[k].z, ranges[k]/1000.0, 
    															 anc_coord[l].x, anc_coord[l].y, anc_coord[l].z, ranges[l]/1000.0, 
    															 &tmp_report) == true)
    								{
    									tmp_report_sum.x += tmp_report.x;
    									tmp_report_sum.y += tmp_report.y;
    									tmp_report_sum.z += tmp_report.z;								
    									loc_num++;
//    									_dbg_printf("��ά��λ�ɹ� i:%d, j:%d, k:%d l:%d\n", i, j, k, l);
    								}
    								else
    								{
//    									_dbg_printf("��ά��λʧ�� i:%d, j:%d, k:%d l:%d\n", i, j, k, l);
    								}
    							}
    						}
    					}
    				}				
    								
    				if(loc_num >0)
    				{
    					instance_otpt.tag_coord.x = tmp_report_sum.x/loc_num;
    					instance_otpt.tag_coord.y = tmp_report_sum.y/loc_num;
    					instance_otpt.tag_coord.z = tmp_report_sum.z/loc_num;					
    					instance_otpt.tag_coord.res = true;
    				}					
                }
                break;
				
            case Sys_Dimension_Invalid:     //��Ч��λ
            default:
                break;
		}
    }
}


void tag_output_test(void)
{
}

/*
 *  �������  
 */
void tag_output(void)
{
    uint8_t buf[512];
    memset(buf, 0, sizeof(buf));
    
    switch(sys_para.cmd_mode)
    {
        //�ַ�����ʽ���ԭʼ����
        case Sys_Commond_Mode_Char_RawDist:
            {
    sprintf((char*)buf, "CmdM:%d,T:%d,TagID:%04X,AncID:%04X,Seq:%d,Mask:%02X,\
[Raw:%08X %08X %08X %08X %08X %08X %08X %08X],\
[DTU:%s]\r\n",    
				Sys_Commond_Mode_Char_RawDist,
                instance_otpt.newRangeTime, instance_otpt.tag_addr, instance_otpt.anc_addr, instance_otpt.rnum, instance_otpt.mask, 
                instance_otpt.dist[0], instance_otpt.dist[1],instance_otpt.dist[2], instance_otpt.dist[3],
                instance_otpt.dist[4], instance_otpt.dist[5],instance_otpt.dist[6], instance_otpt.dist[7],
				instance_otpt.resv_trasport
				);   

                port_tx_msg(buf, strlen(buf));
            }
            break;

        //�ַ�����ʽ����������˲�����		
		case Sys_Commond_Mode_Char_KalmanDist:
			{
    sprintf((char*)buf, "CmdM:%d,T:%d,TagID:%04X,AncID:%04X,Seq:%d,Mask:%02X,\
[Filter: %d %08X %08X %08X %08X %08X %08X %08X %08X],\
[DTU:%s]\r\n",

				Sys_Commond_Mode_Char_KalmanDist,
                instance_otpt.newRangeTime, instance_otpt.tag_addr, instance_otpt.anc_addr, instance_otpt.rnum, instance_otpt.mask, 
	            sys_para.dist.is_kalman_filter,
                instance_otpt.filter_dist[0], instance_otpt.filter_dist[1],instance_otpt.filter_dist[2], instance_otpt.filter_dist[3],
                instance_otpt.filter_dist[4], instance_otpt.filter_dist[5],instance_otpt.filter_dist[6], instance_otpt.filter_dist[7],             
                instance_otpt.resv_trasport);   
                
                port_tx_msg(buf, strlen(buf));

			}
			break;

        //�ַ�����ʽ�����λ����		
		case Sys_Commond_Mode_Char_Pos:
			{
    sprintf((char*)buf, "CmdM:%d,T:%d,TagID:%04X,AncID:%04X,Seq:%d,Mask:%02X,\
[Pos_:%d %d %02X %d %d %d %d],\
[DTU:%s]\r\n", 
				Sys_Commond_Mode_Char_Pos,
                instance_otpt.newRangeTime, instance_otpt.tag_addr, instance_otpt.anc_addr, instance_otpt.rnum, instance_otpt.mask, 
				sys_para.loc.is_loc,sys_para.loc.loc_dimen, anc_coord_mask,
	
				instance_otpt.tag_coord.res,
				(int)(instance_otpt.tag_coord.x*100), (int)(instance_otpt.tag_coord.y*100), (int)(instance_otpt.tag_coord.z*100),
                instance_otpt.resv_trasport);    
			}
			break;

        //�ַ�����ʽ����ź�ǿ��     
        case Sys_Commond_Mode_Char_Sign:
            {
    sprintf((char*)buf, "CmdM:%d,T:%d,TagID:%04X,AncID:%04X,Seq:%d,Mask:%02X,\
[Sign:%d %d %d %d %d %d %d %d],\
[DTU:%s]\r\n", 
				Sys_Commond_Mode_Char_Sign,
                instance_otpt.newRangeTime, instance_otpt.tag_addr, instance_otpt.anc_addr, instance_otpt.rnum, instance_otpt.mask, 
                instance_otpt.rx_diag.maxNoise, instance_otpt.rx_diag.firstPathAmp1, 
                instance_otpt.rx_diag.stdNoise, instance_otpt.rx_diag.firstPathAmp2,
                instance_otpt.rx_diag.firstPathAmp3, instance_otpt.rx_diag.maxGrowthCIR,
                instance_otpt.rx_diag.rxPreamCount, instance_otpt.rx_diag.firstPath,
				instance_otpt.resv_trasport);   

                port_tx_msg(buf, strlen(buf));
            }
            break;
			
        //ʮ���������
        case Sys_Commond_Mode_Hex:
            {
            	int len = 0;
                General_t para;
                para.timer = instance_otpt.newRangeTime;
                para.tagid = instance_otpt.tag_addr;
                para.ancid = instance_otpt.anc_addr;
                para.seq = instance_otpt.rnum;
                para.mask = instance_otpt.mask;
                for(int i = 0; i < MAX_ANCHOR_LIST_SIZE; i++)
                {
                    para.rawrange[i] = instance_otpt.dist[i];
                }
                para.kalman_enable = sys_para.dist.is_kalman_filter;
                for(int i = 0; i < MAX_ANCHOR_LIST_SIZE; i++)
                {
                    para.kalmanrange[i] = instance_otpt.filter_dist[i];
                }     
                para.pos_enable = sys_para.loc.is_loc;
                para.dimen = sys_para.loc.loc_dimen;
            	para.ancmask = anc_coord_mask;
                para.Coord_valid = instance_otpt.tag_coord.res;
                para.pos_loc.x = instance_otpt.tag_coord.x;
                para.pos_loc.y = instance_otpt.tag_coord.y;
                para.pos_loc.z = instance_otpt.tag_coord.z;		

				
            	len = App_Module_format_conver_uint8(para, buf);

            	port_tx_msg(buf , len);            
            }
            break;		
		case Sys_Commond_Mode_Char_Test:
			{
				sprintf((char*)&buf, "mc %02x %08x %08x %08x %08x %08x %08x %08x %08x %04x %02x %08x %c%d:%d\r\n",
											instance_otpt.mask, instance_otpt.dist[0], instance_otpt.dist[1],
											instance_otpt.dist[2], instance_otpt.dist[3],
											instance_otpt.dist[4], instance_otpt.dist[5],
											instance_otpt.dist[6], instance_otpt.dist[7],
											instance_otpt.lnum, instance_otpt.rnum, instance_otpt.newRangeTime,
											(instance_mode == TAG)?'t':'a', instance_otpt.tag_addr, instance_otpt.anc_addr);				

                port_tx_msg(buf, strlen(buf));
			}
			break;
		
        default:
            break;
	}

    memset(&instance_otpt, 0, sizeof(instance_otpt));
	
}

/*
 * twr��ʼ��
 */
int twr_init(void)
{
    uint16_t s1switch = 0;
    //����DW1000 IRQ�ж�(���ý׶�,����)
    port_DisableEXT_IRQ(); 	   //W600Ӳ��Ҳ��ر�
		s1switch = osal_Transfer_Byte((uint16)(sys_para.device_switch & 0xFFFF));//��ȥ��
    //����Ƿ�����DW1000    
		while(s1switch == 0x0000)   
		{
			Sleep(500);	
			App_Module_Sys_Work_Mode_Event(Sys_Operate_Mode_CFG_ING);//��ȥ��
			App_Module_Sys_IO_Led_Mode_Set(Sys_Operate_Mode_CFG_ING);//��ȥ��
		} 	

    //��ʼ��DW1000
		while(inittestapplication(s1switch) == (uint32)-1)//��Ҫ����Ҫ
		{
			_dbg_printf("��ʼ��ʧ��\n");
        reset_DW1000();//Ӳ����λdw1000        
			Sleep(500);
		}
    //ʹ��DW1000 IRQ�ж�    
    port_EnableEXT_IRQ(); //W600Ӳ��Ҳ���      
	
	App_Module_UWB_Mode_Display((uint16_t)sys_para.device_switch);//��ȥ��
	
#ifdef HAL_IWDG
	HalIWDG_Init(7,625);//��ȥ��
#endif
}

/*
 * twr����
 */
void twr_run(void)
{
	//��վ���긳ֵ
	//instance_update_coord();
    int rx = 0;	
    while(1)
    {
    	//instance_data_t* inst = instance_get_local_structure_ptr(0);

    	//int monitor_local = inst->monitor ;
			//	int txdiff = (portGetTickCnt() - inst->timeofTx);

     //   instance_mode = instance_get_role();

      if(instance_mode == TAG)
    	{
        	//��ǩ���г���
    		tag_run();	
    	}
			
			/*
    	else
    	{
            //��վ���г���	
    		anch_run();	
    	}
			*/
        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors
        //if tag handle as a timeout
			//��ɾ��
			/*
        if((monitor_local == 1) && ( txdiff > inst->slotDuration_ms))//������ʲô��
        {
        	int an = 0;
        	uint32 tdly ;
        	uint32 reg1, reg2;

        	reg1 = dwt_read32bitoffsetreg(0x0f, 0x1);
        	reg2 = dwt_read32bitoffsetreg(0x019, 0x1);
        	tdly = dwt_read32bitoffsetreg(0x0a, 0x1);
//        	an = sprintf((char*)&usbVCOMout[0], "T%08x %08x time %08x %08x", (unsigned int) reg2, (unsigned int) reg1,
//        			(unsigned int) dwt_read32bitoffsetreg(0x06, 0x1), (unsigned int) tdly);

					
            inst->wait4ack = 0;

        	if(instance_mode == TAG)
					{
        		tag_process_rx_timeout(inst);
					}
					else //if(instance_mode == ANCHOR)
					{
						dwt_forcetrxoff();	//this will clear all events
						//dwt_rxreset();
						//enable the RX
						inst->testAppState = TA_RXE_WAIT ;
					}
						inst->monitor = 0;
        }

       // rx = instance_newrange();//����ɾ��
        //��⵽���µľ������ݲ���,���ӡ
       // if(rx == TOF_REPORT_T2A)
      //  {
						//ע1:tag_position_cal()������,�����ǩ����ʱ�����������ʹ��������λ����������MCU�����������
						//ע2��tag_output()������,sprintfЧ�ʺܵͣ��û��Զ����ʽʱ������ʹ��HEX��ʽ
			
            //��ǩ���븳ֵ
           // tag_distance_cpy();//����ɾ��

            //��ǩ�����˲�
           // tag_distance_fil();//����ɾ��

            //��ǩ�������
					//	tag_position_cal();//����ɾ��
			
            //���������ӡ
					//	tag_output();//����ɾ��
       // }

		//App_Module_Sys_Work_Mode_Event(Sys_Operate_Mode_WORK_DONE);//����ɾ��
		//App_Module_Sys_IO_Led_Mode_Set(Sys_Operate_Mode_WORK_DONE);	//����ɾ��
		
		//HalIWDG_Feed();	//����ɾ��
		*/		
    }
}

void twr_task(void)
{
    //twr��ʼ�� 
	twr_init();

    //twr����
	twr_run();
}
