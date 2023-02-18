/**********************************************************************************
 * @作者：	久凌电子
 * @版本：	Twr(v3.0.0)
 * @日期：	2021.10.11
 * @说明：	超宽带 TWR方案(stm32)
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

    int mask;           //距离值有效位
    int rnum;           //UWB无线信号序列号计数
    int lnum;           //计数值
    int anc_addr;       //基站地址
    int tag_addr;       //标签地址
    int newRangeTime;   //时间(stm32)
    int trxa;           //天线延迟参数
    
    int dist[MAX_ANCHOR_LIST_SIZE];         //距离值 单位mm
    int filter_dist[MAX_ANCHOR_LIST_SIZE];  //滤波距离值 单位mm

    struct{
        bool res;           //定位结果
        float x;            //标签计算得到x轴数值
        float y;            //标签计算得到y轴数值
        float z;            //标签计算得到z轴数值
    }tag_coord;

    dwt_rxdiag_t rx_diag;   //uwb信号参数

    uint8_t resv_trasport[16];//接受透传指令
}instance_otpt_t;


typedef struct {
	bool valid;         //是否有效
	float x;            //基站x轴数值
	float y;            //基站y轴数值
	float z;            //基站z轴数值
}anc_coord_t;

static uint8_t anc_coord_mask;		//基站有效位
static anc_coord_t anc_coord[MAX_ANCHOR_LIST_SIZE];

static instance_otpt_t instance_otpt;
int instance_mode = ANCHOR;
int instance_anchaddr = 0;
int dr_mode = 0;
int chan, tagaddr, ancaddr;

//Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
instanceConfig_t chConfig[4] ={
    //模式1： 信道:2 脉冲重复率:16M 空中速率:110k
    {
        .channelNumber = 2,             // 信道2 channel
        .preambleCode = 4,              // 前导码 preambleCode
        .pulseRepFreq = DWT_PRF_16M,    // 脉冲重复频率 prf
        .dataRate = DWT_BR_110K,        // 数据速率 datarate
        .preambleLen = DWT_PLEN_1024,   // preambleLength
        .pacSize = DWT_PAC32,           // pacSize
        .nsSFD = 1,                     // 0:非标准SFD, 1:标准SFD 
        .sfdTO = (1025 + 64 - 32)       // SFD 超时 SFD timeout
    },
    //模式2： 信道:2 脉冲重复率:16M 空中速率:6.8M
    {
		.channelNumber = 2,            // 信道2 channel
		.preambleCode = 4,             // 前导码 preambleCode
		.pulseRepFreq = DWT_PRF_16M,   // 脉冲重复频率 prf
		.dataRate = DWT_BR_6M8,        // 数据速率 datarate
		.preambleLen = DWT_PLEN_128,   // preambleLength
		.pacSize = DWT_PAC8,           // pacSize
		.nsSFD = 0,                    // 0:非标准SFD, 1:标准SFD
		.sfdTO = (129 + 8 - 8)         // SFD 超时 SFD timeout
    },
    //模式3： 信道:5 脉冲重复率:16M 空中速率:110K
    {
		.channelNumber = 5,             // 信道5 channel
		.preambleCode = 3,              // 前导码 preambleCode
		.pulseRepFreq = DWT_PRF_16M,    // 脉冲重复频率 prf
		.dataRate = DWT_BR_110K,        // 数据速率 datarate
		.preambleLen = DWT_PLEN_1024,   // preambleLength
		.pacSize = DWT_PAC32,           // pacSize
		.nsSFD = 1,                     // 0:非标准SFD, 1:标准SFD
		.sfdTO = (1025 + 64 - 32)       // SFD 超时 SFD timeout
    },
    //模式4： 信道:5 脉冲重复率:16M 空中速率:6.8M
    {
		.channelNumber = 5,            // 信道5 channel
		.preambleCode = 3,             // 前导码 preambleCode
		.pulseRepFreq = DWT_PRF_16M,   // 脉冲重复频率 prf
		.dataRate = DWT_BR_6M8,        // 数据速率 datarate
		.preambleLen = DWT_PLEN_128,   // preambleLength
		.pacSize = DWT_PAC8,           // pacSize
		.nsSFD = 0,                    // 0:非标准SFD, 1:标准SFD
		.sfdTO = (129 + 8 - 8)         // SFD 超时 SFD timeout
    }
};

//槽和超级帧4种模式 Slot and Superframe Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
sfConfig_t sfConfig[4] ={
    //模式1:
	{
		.slotDuration_ms = (40),        //单个Slot周期40ms
		.numSlots = (10),               //槽个数
		.sfPeriod_ms = (10*40),         //超级帧周期
		.tagPeriod_ms = (10*40),        //Poll->Poll休眠时间
		.pollTxToFinalTxDly_us = (40000)//Poll->Final延时时间
	},
    //模式2:
    {
		.slotDuration_ms = (10),        //单个Slot周期10ms
		.numSlots = (10),               //槽个数
		.sfPeriod_ms = (10*10),         //超级帧周期
		.tagPeriod_ms = (10*10),        //Poll->Poll休眠时间
		.pollTxToFinalTxDly_us = (3800) //Poll->Final延时时间
    },
    //模式3:
    {
		.slotDuration_ms = (40),        //单个Slot周期40ms
		.numSlots = (10),               //槽个数
		.sfPeriod_ms = (10*40),         //超级帧周期
		.tagPeriod_ms = (10*40),        //Poll->Poll休眠时间
		.pollTxToFinalTxDly_us = (40000)//Poll->Final延时时间

    },
    //模式4:
    {
		.slotDuration_ms = (10),        //单个Slot周期10ms
		.numSlots = (10),               //槽个数
		.sfPeriod_ms = (10*10),         //超级帧周期
		.tagPeriod_ms = (10*10),        //Poll->Poll休眠时间
		.pollTxToFinalTxDly_us = (3800) //Poll->Final延时时间
    }
};


/*
 *  设置16bit短地址  
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
 *  获取配置模式索引         returns the use case / operational mode
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
 *  修改时间槽信息
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
 *  instance_otp基站信息更新
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
 * 第一步：SPI验证        
 * 第二步：DW1000初始化 | instance_init(instance_mode)
 * 第三步：配置信道信息 | instance_config(&chConfig[dr_mode], &sfConfig[dr_mode])
 */
uint32 inittestapplication(uint16_t s1switch)
{
    uint32 devID;
    int result;

    //降低SPI速率值低于3MHz
    port_set_dw1000_slowrate();//硬件相关

    //读取设备Dev ID值(SPI 低速率)
    devID = instance_readdeviceid();
    if(DWT_DEVICE_ID != devID)
    {
        //唤醒dw1000    
    	port_wakeup_dw1000();

        //再次读取设备Dev ID值(SPI 低速率)
        devID = instance_readdeviceid() ;
        if(DWT_DEVICE_ID != devID)
            return(-1) ;

        //软复位dw1000        
        dwt_softreset();
    }

    //硬件复位dw1000
    reset_DW1000();

    //判断配置为基站/标签
    if((s1switch & SWS1_ANC_MODE) == 0)
        instance_mode = TAG;
    else
        instance_mode = ANCHOR;

    //根据设备角色,初始化dw1000
    result = instance_init(instance_mode) ; //标签初始化
    if (0 > result) return(-1) ; 

    //提高SPI速率值20MHz
    port_set_dw1000_fastrate(); 
    //读取设备Dev ID值(SPI 高速率)    
    devID = instance_readdeviceid();

    if (DWT_DEVICE_ID != devID)   
    {
        return(-1) ;
    }

    //设置16bit地址
    addressconfigure(s1switch, instance_mode) ; //待确认这个地址作用？

#ifdef HW_8Anc
    if((instance_mode == ANCHOR) && (instance_anchaddr > (MAX_ANCHOR_LIST_SIZE - 1)))
#else
    if((instance_mode == ANCHOR) && (instance_anchaddr > 0x3))
#endif
    {
        _dbg_printf("非法地址");
        return (-1);
        //非法配置
    }
    else
    {

		dr_mode = decarangingmode(s1switch);

		chan = chConfig[dr_mode].channelNumber ;
		//根据用户配置，修改时间槽数量
		instance_update_sfConfig(sys_para.device_max_tag_list_size);		
        //根据配置，选择sfconfig(时间槽配置)和chConfig(信道配置)
		instance_config(&chConfig[dr_mode], &sfConfig[dr_mode]) ;
    }
    return devID;
}

/*
 *  标签距离赋值
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
 *  标签距离滤波  
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
 *  标签坐标计算  
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
			//二维定位
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
//    								_dbg_printf("二维定位成功 i:%d, j:%d, k:%d\n", i, j, k);
    							}
    							else
    							{
//    								_dbg_printf("二维定位失败 i:%d, j:%d, k:%d\n", i, j, k);
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
				
			//三维定位	
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
//    									_dbg_printf("三维定位成功 i:%d, j:%d, k:%d l:%d\n", i, j, k, l);
    								}
    								else
    								{
//    									_dbg_printf("三维定位失败 i:%d, j:%d, k:%d l:%d\n", i, j, k, l);
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
				
            case Sys_Dimension_Invalid:     //无效定位
            default:
                break;
		}
    }
}


void tag_output_test(void)
{
}

/*
 *  数据输出  
 */
void tag_output(void)
{
    uint8_t buf[512];
    memset(buf, 0, sizeof(buf));
    
    switch(sys_para.cmd_mode)
    {
        //字符串形式输出原始距离
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

        //字符串形式输出卡尔曼滤波距离		
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

        //字符串形式输出定位数据		
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

        //字符串形式输出信号强度     
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
			
        //十六进制输出
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
 * twr初始化
 */
int twr_init(void)
{
    uint16_t s1switch = 0;
    //禁用DW1000 IRQ中断(配置阶段,禁用)
    port_DisableEXT_IRQ(); 	   //W600硬件也需关闭
		s1switch = osal_Transfer_Byte((uint16)(sys_para.device_switch & 0xFFFF));//可去除
    //检查是否配置DW1000    
		while(s1switch == 0x0000)   
		{
			Sleep(500);	
			App_Module_Sys_Work_Mode_Event(Sys_Operate_Mode_CFG_ING);//可去除
			App_Module_Sys_IO_Led_Mode_Set(Sys_Operate_Mode_CFG_ING);//可去除
		} 	

    //初始化DW1000
		while(inittestapplication(s1switch) == (uint32)-1)//需要，重要
		{
			_dbg_printf("初始化失败\n");
        reset_DW1000();//硬件复位dw1000        
			Sleep(500);
		}
    //使能DW1000 IRQ中断    
    port_EnableEXT_IRQ(); //W600硬件也需打开      
	
	App_Module_UWB_Mode_Display((uint16_t)sys_para.device_switch);//可去除
	
#ifdef HAL_IWDG
	HalIWDG_Init(7,625);//可去除
#endif
}

/*
 * twr工作
 */
void twr_run(void)
{
	//基站坐标赋值
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
        	//标签运行程序
    		tag_run();	
    	}
			
			/*
    	else
    	{
            //基站运行程序	
    		anch_run();	
    	}
			*/
        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors
        //if tag handle as a timeout
			//先删除
			/*
        if((monitor_local == 1) && ( txdiff > inst->slotDuration_ms))//作用是什么？
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

       // rx = instance_newrange();//可以删除
        //检测到有新的距离数据产生,则打印
       // if(rx == TOF_REPORT_T2A)
      //  {
						//注1:tag_position_cal()函数中,计算标签消耗时间过长，建议使用其他上位机或者其他MCU来做坐标解算
						//注2：tag_output()函数中,sprintf效率很低，用户自定义格式时，建议使用HEX格式
			
            //标签距离赋值
           // tag_distance_cpy();//可以删除

            //标签距离滤波
           // tag_distance_fil();//可以删除

            //标签坐标计算
					//	tag_position_cal();//可以删除
			
            //数据输出打印
					//	tag_output();//可以删除
       // }

		//App_Module_Sys_Work_Mode_Event(Sys_Operate_Mode_WORK_DONE);//可以删除
		//App_Module_Sys_IO_Led_Mode_Set(Sys_Operate_Mode_WORK_DONE);	//可以删除
		
		//HalIWDG_Feed();	//可以删除
		*/		
    }
}

void twr_task(void)
{
    //twr初始化 
	twr_init();

    //twr工作
	twr_run();
}
