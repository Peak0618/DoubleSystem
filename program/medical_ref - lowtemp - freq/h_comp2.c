#ifndef _COMP2_C_
#define _COMP2_C_
//------------------------------------------------------------------------------
//包含文件声明
#include "r_cg_macrodriver.h"

#include "h_type_define.h"
#include "h_ad.h"
#include "h_key_operation.h"
#include "h_rom_para2.h"
#include "h_comp2.h"
#include "h_volt_ctrl.h"
#include "h_protect.h"
#include "h_main.h"
#include "m_peripheral.h"

//------------------------------------------------------------------------------
//函数声明
void comp2_power_on_delaytime(void); 
void comp2_run_stop_delaytime(void); 
void db_comp_run_stop_ask_for_delaytime();
void db_first_system_run_stop_deal(void);    
void db_secend_system_run_stop_deal(void);   
void comp2_run_delaytime(void);    
void comp2_stop_delaytime(void); 
void comp2_TA_fault_run_delaytime(void);      
void comp2_TA_fault_stop_delaytime(void);     
void comp2_continue_run_delaytime(void);      
void comp2_force_stop_delaytime(void);        
void comp2_run_space_delaytime(void);    
void comp2_start_delaytime(void);
void comp2_update_delaytime(void);
void comp21_start_delaytime(void);
void comp21_update_delaytime(void);
void comp2_power_down_delaytime(void);
void freq_ctrl_deal(void);
void init_freq_ctrl(void);

void db_init_debug_parm_value(void);
void db_single_to_double_mode_switch(void);
void db_comp_fault_judge_delaytime(void);
void db_comp_system_fault_judge(void);
void db_comp_RL_out(void);
void db_fan_stop_delaytime(void);
void db_thermocouple_reverse_judge(void);


//flag
flag_type flg_comp2,flg_freq,db_flag;

//----------------------------------------------------------
//压缩机控制相关变量
int16_t   gss_comp2_power_on_delaytimer;     //上电延时定时器

int16_t   gss_comp2_run_T_delaytimer;        //高温压机达到开机温度持续时间
int16_t   gss_comp2_stop_T_delaytimer;       //            关机

int16_t   gss_comp2_run_delaytimer;          //高温压缩机开机延时定时器
int16_t   gss_comp2_stop_delaytimer;         //高温压缩机关机延时定时器
//----------------------------------------------------------
int16_t   gss_comp2_TA_fault_run_delaytimer;      //主传感器故障高温压缩机开机延时定时器
int16_t   gss_comp2_TA_fault_stop_delaytimer;     //主传感器故障高温压缩机关机延时定时器

int16_t   gss_comp2_continue_run_delaytimer;      //高温压缩机连续运行计时器
int16_t   gss_comp2_force_stop_delaytimer;        //高温压缩机强制停止计时器

int16_t   gss_comp2_run_space_delaytimer;         //高温压缩机运行间隔延时定时器
int8_t    gss_power_down_delaytime;               //断电后重新上电高温压机延时
uint8_t   guc_comp2_power_on_again;               //断电后重新上电高压机以4500启动


//频率控制相关变量
int16_t   gss_para[27];

int16_t   gss_comp2_freq;     //高温压机频率
int16_t   gss_comp21_freq;    //低温压机频率   peak 即转速

int16_t   gss_comp2_delt_freq;
int16_t   gss_comp21_delt_freq;

int16_t   gss_comp2_start_delaytimer;
int16_t   gss_comp2_update_delaytimer;

int16_t   gss_comp21_start_delaytimer;
int16_t   gss_comp21_update_delaytimer;

uint16_t  gus_comp2_freq_cnt;
uint16_t  gus_comp21_freq_cnt;

//---------------------------------------------------------------------------
//macro

#define A_SYSTEM  1  //为了初始上电时是A先开
#define B_SYSTEM  0

//variate
int16_t gss_db_fault_mode_T_delaytime;         //任意系统有故障时fault模式请求开机时间
int16_t gss_db_double_mode_T_delaytime;        //double运行模式请求延时
int16_t gss_db_single_mode_T_delaytime;        //single运行模式请求延时
int16_t gss_db_first_comp_stop_T_delaytime ;   //A压机停机请求延时
int16_t gss_single_to_double_switch_delaytime; //single运行模式切换double延时
int16_t gss_db_comp_fan_stop_delaytime;        //压机停机延时xmin后风机停
int16_t gss_db_A_system_runing_delaytime;      //A压机开xmin后判故障
int16_t gss_db_B_system_runing_delaytime;      //B压机开xmin后判故障
int16_t gss_db_A_system_fault_delaytime;       //A系统故障判定延时
int16_t gss_db_B_system_fault_delaytime;       //B.....
int16_t gss_B_Tcouple_memory;                  //首次上电压机运行时记录B系统对应热电偶温度


uint8_t guc_db_comp_mode;                      //压机运行模式double or single
uint8_t guc_db_system_state;                   //运行时压机是已正常状态还是故障状态



//array
int16_t gss_db_debug_parm_value[DB_SUM_P];    //调试的参数值P00~P13

//macro
#define SINGLE_MODE        1       //单系统模式
#define DOUBLE_MODE        2       //双系统模式

#define SYSTEM_NORMAL      0       //系统正常
#define SYSTEM_A_FAULT     1       //A系统故障
#define SYSTEM_B_FAULT     2       //B系统故障
#define SYSTEM_AB_FAULT    3       //AB都故障


/***************************************************************************************************************************************
函数功能：     上电延时程序,初始上电延时 1min（可调） 后，再对高、低压机进行控制


函数位置：  1s-------------------------ok   
***************************************************************************************************************************************/
void comp2_power_on_delaytime(void)
{
    if (bflg_comp2_power_on_delaytime == 1) //上电初始化为1
    {
        gss_comp2_power_on_delaytimer++;
        if (gss_comp2_power_on_delaytimer >= gss_factory_parm_value[DISP_FACTORY_Cd]*60) //min *60 = s                       
        {
            gss_comp2_power_on_delaytimer = 0;
            bflg_comp2_power_on_delaytime = 0;
        }
    }
}

/****************************************************************************************************************************************
函数功能:     根据要求的温度开、停机点，判断高温压机是否有开、停机的请求
       
函数位置: 1s定时器中-----------------------------------------------不用了
*****************************************************************************************************************************************/
void comp2_run_stop_delaytime(void)
{
    if((bflg_alarm_power_off == 1)||(bflg_alarm_power_fault == 1))      //断强电、断12v           
    {
        gss_comp2_run_T_delaytimer = 0;
        gss_comp2_stop_T_delaytimer = 0;

        bflg_db_first_comp_askfor_run = 0;
        bflg_db_first_comp_askfor_stop = 0;
    }
    else
    {
        if (ram_para[num_comp2_mode] == 1)  //如果是高性能运行(液晶、LED都默认高性能)     
        {
            if (bflg_db_first_comp_runing == 0)    //此时关机
            {
                if (gss_TA >= gss_set_parm_value[DISP_SET_TS] + gss_para[COMP_START_DELT_TEMP])   // 开机点
    	        {
    	            gss_comp2_stop_T_delaytimer = 0;  
    	            
    	            if (bflg_db_first_comp_askfor_run == 0)
    	            {
    	                gss_comp2_run_T_delaytimer++;
    	                if (gss_comp2_run_T_delaytimer >= ram_para2[num_comp2_run_T_delaytime])  //30s
    	                {
    	                    gss_comp2_run_T_delaytimer = 0;
    	                    bflg_db_first_comp_askfor_run = 1;
    	                }
    	            }
    	        }
    	        else                                                                      //否则 没有动作
    	        {
    	        	  gss_comp2_run_T_delaytimer = 0;
    	        	  gss_comp2_stop_T_delaytimer = 0;
    	        	  bflg_db_first_comp_askfor_run = 0;
    	        }
            }
            else   //此时开机
            {
                if (gss_TA <= gss_set_parm_value[DISP_SET_TS] + gss_para[COMP_STOP_DELT_TEMP])  //停机点
    	        {
    	            gss_comp2_run_T_delaytimer = 0;
    	            
    	            if (bflg_db_first_comp_askfor_stop == 0)
    	            {
    	                gss_comp2_stop_T_delaytimer++;
    	                if (gss_comp2_stop_T_delaytimer >= ram_para2[num_comp2_stop_T_delaytime]) //30s
    	                {
    	                    gss_comp2_stop_T_delaytimer = 0;
    	                    bflg_db_first_comp_askfor_stop = 1;
    	                }
    	            }
    	        }
    	        else
    	        {
    	        	gss_comp2_run_T_delaytimer = 0;
    	            gss_comp2_stop_T_delaytimer = 0;
    	            bflg_db_first_comp_askfor_stop = 0;
    	        }
            }
        }
        else  //节能运行
        {
            if (bflg_db_first_comp_runing == 0)
            {
    		    if (gss_TA >= gss_set_parm_value[DISP_SET_TS] + ram_para2[num_comp2_mode_T] + ram_para2[num_comp2_run_Tdelt])
    		    //if (gss_TA >= gss_set_parm_value[DISP_SET_TS] + gss_para[COMP_START_DELT_TEMP])  // 10 开机
                {
    		            gss_comp2_stop_T_delaytimer = 0;
    		            
    		            if (bflg_db_first_comp_askfor_run == 0)
    		            {
    		                gss_comp2_run_T_delaytimer++;
    		                if (gss_comp2_run_T_delaytimer >= ram_para2[num_comp2_run_T_delaytime])
    		                {
    		                    gss_comp2_run_T_delaytimer = 0;
    		                    bflg_db_first_comp_askfor_run = 1;
    		                }
    		            }
    		        }
    		        else
    		        {
    		        	  gss_comp2_run_T_delaytimer = 0;
    		        	  gss_comp2_stop_T_delaytimer = 0;
    		        	  bflg_db_first_comp_askfor_run = 0;
    		        }
            }
            else
            {
                    if (gss_TA <= gss_set_parm_value[DISP_SET_TS] + ram_para2[num_comp2_mode_T] + ram_para2[num_comp2_stop_Tdelt])
                   //if (gss_TA <= gss_set_parm_value[DISP_SET_TS] + gss_para[COMP_STOP_DELT_TEMP])  // 10 停机
    		        {
    		            gss_comp2_run_T_delaytimer = 0;
    		            
    		            if (bflg_db_first_comp_askfor_stop == 0)
    		            {
    		                gss_comp2_stop_T_delaytimer++;
    		                if (gss_comp2_stop_T_delaytimer >= ram_para2[num_comp2_stop_T_delaytime])
    		                {
    		                    gss_comp2_stop_T_delaytimer = 0;
    		                    bflg_db_first_comp_askfor_stop = 1;
    		                }
    		            }
    		        }
    		        else
    		        {
    		            gss_comp2_run_T_delaytimer = 0;
    		            gss_comp2_stop_T_delaytimer = 0;
    		            bflg_db_first_comp_askfor_stop = 0;
    		        }
            }
        }
        //------------------------------------------------------
        if (bflg_comp21_door_freq == 1)
        {
            bflg_db_first_comp_askfor_run = 1;
            
            bflg_comp2_stop_delaytime = 0;
            gss_comp2_stop_delaytimer = 0;
            
            bflg_comp2_force_stop_delaytime = 0;
            gss_comp2_force_stop_delaytimer = 0;
        }
        //------------------------------------------------------
        if (bflg_comp21_door_freq_tmp == 1)
        {
            if ((gss_TA - gss_set_parm_value[DISP_SET_TS]) >= gss_para[COMP_DOOR_TDELT])     //5.0℃
            {
                bflg_comp2_large_freq = 1;
                bflg_comp21_door_freq_tmp = 0;
            }
        }
    }
}



/***********************************************************************************************************************************************
函数功能：1、判断第一台压机的启停请求； 2、判定整体运行模式double or single
                            
                              
函数位置：1s--------------------------------ok
************************************************************************************************************************************************/
void db_comp_run_stop_ask_for_delaytime(void)
{
    if((bflg_alarm_power_off == 1)||(bflg_alarm_power_fault == 1))      //断强电、断12v           
    {
        gss_db_double_mode_T_delaytime = 0;
        gss_db_single_mode_T_delaytime = 0;
        gss_db_first_comp_stop_T_delaytime = 0;
    
        bflg_db_first_comp_askfor_run = 0;
        bflg_db_first_comp_askfor_stop = 0;
    }
    else
    {
        if(bflg_db_first_comp_runing == 0) //若此刻关机
        {                                        
            gss_db_first_comp_stop_T_delaytime = 0;

            if(gss_TA - gss_set_parm_value[DISP_SET_TS] >= gss_db_debug_parm_value[DB_P09])  //启停点
            {
                if((bflg_db_A_system_fault == 1)||(bflg_db_B_system_fault == 1))//任意系统有故障时(非初次上电)
                {
                    gss_db_single_mode_T_delaytime = 0;
                    gss_db_double_mode_T_delaytime = 0;
                    
                    gss_db_fault_mode_T_delaytime++;
                    if(gss_db_fault_mode_T_delaytime >= 30) //30s 自+
                    {
                        gss_db_fault_mode_T_delaytime = 0;

                        if((bflg_db_A_system_fault == 1)&&(bflg_db_B_system_fault == 1))
                        {
                            guc_db_system_state = SYSTEM_AB_FAULT;
                            guc_db_comp_mode = DOUBLE_MODE;
                        }
                        else if((bflg_db_A_system_fault == 1)&&(bflg_db_B_system_fault == 0))
                        {
                            guc_db_system_state = SYSTEM_A_FAULT;
                            guc_db_comp_mode = SINGLE_MODE;
                        }
                        else if((bflg_db_A_system_fault == 0)&&(bflg_db_B_system_fault == 1))
                        {
                            guc_db_system_state = SYSTEM_B_FAULT;
                            guc_db_comp_mode = SINGLE_MODE;
                        }
                        bflg_db_first_comp_askfor_run = 1;
                    }
                }
                else
                {
                    if(gss_TA - gss_set_parm_value[DISP_SET_TS] > gss_db_debug_parm_value[DB_P00])
                    {
                        gss_db_single_mode_T_delaytime = 0;
                        gss_db_fault_mode_T_delaytime = 0;
                        
                        gss_db_double_mode_T_delaytime++;
                        if(gss_db_double_mode_T_delaytime >= 30)   //30s 自+
                        {
                            gss_db_double_mode_T_delaytime = 0;
                            
                            guc_db_system_state = SYSTEM_NORMAL;
                            guc_db_comp_mode = DOUBLE_MODE;
                
                            bflg_db_first_comp_askfor_run = 1;
                        }
                    }
                    else if((gss_TA - gss_set_parm_value[DISP_SET_TS] >= gss_db_debug_parm_value[DB_P01])&&
                            (gss_TA - gss_set_parm_value[DISP_SET_TS] <= gss_db_debug_parm_value[DB_P02]) )
                    {
                        gss_db_double_mode_T_delaytime = 0;
                        gss_db_fault_mode_T_delaytime = 0;
                        
                        gss_db_single_mode_T_delaytime++;
                        if(gss_db_single_mode_T_delaytime >= 30)   //30s
                        {
                            gss_db_single_mode_T_delaytime = 0;
                            
                            guc_db_system_state = SYSTEM_NORMAL;                            
                            guc_db_comp_mode = SINGLE_MODE;
                
                            bflg_db_first_comp_askfor_run = 1;
                        }
                    }
                    //---------------------------------------------上电初次运行时记忆B热电偶温度
                    if(bflg_db_first_comp_askfor_run == 1)
                    {
                        if(bflg_power_on_Tcouple_judge == 1)
                        {
                            gss_B_Tcouple_memory = gss_adg_Temp[7];  //记忆刚开机时的B热电偶温度
                        }
                    }
                }
            }
            else
            {
                gss_db_double_mode_T_delaytime = 0;
                gss_db_single_mode_T_delaytime = 0;
                gss_db_fault_mode_T_delaytime = 0;
                
                bflg_db_first_comp_askfor_run = 0;
            }
        }
        else    //此刻开机
        {
            gss_db_double_mode_T_delaytime = 0;
            gss_db_single_mode_T_delaytime = 0;
            
            if(gss_TA - gss_set_parm_value[DISP_SET_TS] <= gss_db_debug_parm_value[DB_P10])
            {
                gss_db_double_mode_T_delaytime = 0;
                gss_db_single_mode_T_delaytime = 0;
                
                if(bflg_db_first_comp_askfor_stop == 0)
                {
                    gss_db_first_comp_stop_T_delaytime++;
                    if(gss_db_first_comp_stop_T_delaytime >= 30)   //30s
                    {
                        gss_db_first_comp_stop_T_delaytime = 0;
                        bflg_db_first_comp_askfor_stop = 1;
                    }
                }
            }
            else
            {
                gss_db_first_comp_stop_T_delaytime = 0;
                bflg_db_first_comp_askfor_stop = 0;
            }
            
            //系统模式切换 和 单/双压机运行切换
            db_single_to_double_mode_switch();
        }
    }
}

/********************************************************************************************************************************
函数功能: 正常情况下第一台压机启动的条件：    
          1、有第一台压机开停机请求、
          2、最小开停机延时是否结束、
          3、强制停机是否结束，

函数位置:主循环中-------------------------------------------------------ok          
********************************************************************************************************************************/
void db_first_system_run_stop_deal(void)    
{
    if((bflg_alarm_power_off == 1)||(bflg_alarm_power_fault == 1))   //断强电、12v
    {
        bflg_db_first_comp_runing = 0;                  //停机状态

        bflg_comp2_run_space_delaytime = 0;
        gss_comp2_run_space_delaytimer = 0;

        bflg_comp2_run_delaytime = 0;            //置开机延时标志
        gss_comp2_run_delaytimer = 0;            //清开机延时计时器

        bflg_comp2_continue_run_delaytime = 0;
        gss_comp2_continue_run_delaytimer = 0;

        bflg_comp2_first_run = 0;                //清首次运行标志

        bflg_comp2_force_stop_delaytime = 0;     //置强制停止延时标志  //功能书要求强制停止
        gss_comp2_force_stop_delaytimer = 0;     //清强制停止延时计时器

        bflg_comp2_stop_delaytime = 0;           //置关机延时标志
        gss_comp2_stop_delaytimer = 0;           //清关机延时计时器

        //--------------主故障时
        bflg_comp2_TA_fault_run_delaytime = 0;   //置主传感器故障开机延时标志        // peak 开30分钟--功能书 备注3
        gss_comp2_TA_fault_run_delaytimer = 0;   //清主传感器故障开机延时计时器

        bflg_comp2_TA_fault_stop_delaytime = 0;  //置主传感器故障关机延时标志
        gss_comp2_TA_fault_stop_delaytimer = 0;
    }
    else
    {
        if(bflg_power_down_delaytime == 0)      //如果未断电或是断电后延时结束
        {
            if (bflg_TA_fault == 1)             //如果主传感器故障   
            {
                if (bflg_db_first_comp_runing == 0)    //首先启动的关机
                {
                    if (bflg_comp2_TA_fault_stop_delaytime == 0)    //如果主传感器故障关机延时时间结束   //peak 关10分钟结束--功能书 备注3
                    {
                        bflg_db_first_comp_runing = 1;    //开机
                        bflg_first_comp_open = ~bflg_first_comp_open;
                        
                        bflg_comp2_TA_fault_run_delaytime = 1;      //置主传感器故障开机延时标志        
                        gss_comp2_TA_fault_run_delaytimer = 0;      //清主传感器故障开机延时计时器
                        
                        bflg_comp2_run_space_delaytime = 1;        //高温压缩机运行间隔延时标志  //备注1
                        gss_comp2_run_space_delaytimer = 0;
                    }
                }
                else   //如果开机
                {
                    if (bflg_comp2_TA_fault_run_delaytime == 0)     //如果主传感器故障开机延时时间结束
                    {
                        if (bflg_comp2_volt_low == 0)     //如果没有电压过低
                        {
                            bflg_db_first_comp_runing = 0;       //关机
                            
                            bflg_comp2_first_run = 0;     //清首次运行标志
                            
                            bflg_comp2_TA_fault_stop_delaytime = 1; //置主传感器故障关机延时标志
                            gss_comp2_TA_fault_stop_delaytimer = 0; //清主传感器故障关机延时计时器

                            bflg_db_comp_fan_stop_delaytime = 1;
                            gss_db_comp_fan_stop_delaytime = 0;
                        }
                    }
                }
            }
            else       //如果主传感器正常
            {
                if (bflg_db_first_comp_runing == 0)     //关
                {
                    if (bflg_db_first_comp_askfor_run == 1)     
                    {
                        if ((bflg_comp2_stop_delaytime == 0) && (bflg_comp2_force_stop_delaytime == 0))//停机延时时间结束
                        {
                            bflg_db_first_comp_askfor_run = 0;             //清请求开机标志
                            bflg_db_first_comp_runing = 1;                 //开机
                            bflg_first_comp_open = ~bflg_first_comp_open;  //交替开启
                            
                            bflg_comp2_run_space_delaytime = 1;
                            gss_comp2_run_space_delaytimer = 0;
                            
                            bflg_comp2_run_delaytime = 1; //置开机延时标志
                            gss_comp2_run_delaytimer = 0; //清开机延时计时器
                            
                            bflg_comp2_continue_run_delaytime = 1;
                            gss_comp2_continue_run_delaytimer = 0;
                        }
                    }
                }
                else    //如果开机
                {
                    if (bflg_comp2_continue_run_delaytime == 0)     //如果高温压缩机连续运行时间结束
                    {
                        if (bflg_comp2_volt_low == 0)     //如果没有电压过低
                        {
                            bflg_db_first_comp_runing = 0;  //关机
                            
                            bflg_comp2_first_run = 0;     //清首次运行标志
                            
                            bflg_comp2_force_stop_delaytime = 1;    //置强制停止延时标志  //功能书要求强制停止
                            gss_comp2_force_stop_delaytimer = 0;    //清强制停止延时计时器

                            bflg_db_comp_fan_stop_delaytime = 1;
                            gss_db_comp_fan_stop_delaytime = 0;
                        }
                    }
                    else     //如果压机连续运行时间未结束
                    {
                        if (bflg_db_first_comp_askfor_stop == 1)  
                        {
                            if (bflg_comp2_run_delaytime == 0) //开机延时时间结束
                            {
                                if (bflg_comp2_volt_low == 0)            //如果没有电压过低
                                {
                                    bflg_db_first_comp_askfor_stop = 0;     //清请求关机标志
                                    bflg_db_first_comp_runing = 0;         //关机
                                    
                                    bflg_comp2_first_run = 0;     //清首次运行标志
                            
                                    bflg_comp2_stop_delaytime = 1;  //置关机延时标志
                                    gss_comp2_stop_delaytimer = 0;  //清关机延时计时器   
                                    
                                    bflg_db_comp_fan_stop_delaytime = 1;
                                    gss_db_comp_fan_stop_delaytime = 0;                                    
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
/*******************************************************************************************************************************
函数功能:第二台压机启动条件：
         1、first启动1min后；
         2、运行模式是double模式；

函数位置：主循环---------------------------------ok
********************************************************************************************************************************/
void db_secend_system_run_stop_deal(void) 
{
    if (bflg_db_first_comp_runing == 0)    
    {
        bflg_db_secend_comp_runing = 0;   
    }
    else  
    {
        if (bflg_comp2_run_space_delaytime == 0)   //延时1min到
        {
            if(bflg_TA_fault == 1)                
            {
                bflg_db_secend_comp_runing = 1;
            }
            else
            {
                if(guc_db_comp_mode == DOUBLE_MODE)
                {
                    bflg_db_secend_comp_runing = 1;                 
                }
                else if((guc_db_comp_mode == SINGLE_MODE))
                {
                    bflg_db_secend_comp_runing = 0;
                }
            }
        }
    }
}
/********************************************************************************************************************************
函数功能:最短开机时间，高温压机至少开机5min才能关机,

函数位置：1s定时器--------------------------------------ok  
********************************************************************************************************************************/
void comp2_run_delaytime(void)      
{
    if (bflg_comp2_run_delaytime == 1)
    {
        gss_comp2_run_delaytimer++;
        //if (gss_comp2_run_delaytimer >= gss_para[COMP_RUN_DELAYTIME])   //300s = 5min
        if (gss_comp2_run_delaytimer >= gss_db_debug_parm_value[DB_P11]*60)   //min*60 = s
        {
            gss_comp2_run_delaytimer = 0;
            bflg_comp2_run_delaytime = 0;
        }
    }
}
/********************************************************************************************************************************
函数功能: 最短停机时间，至少关机5min才能重新开机

函数位置：1s定时器-------------------------------ok
*********************************************************************************************************************************/
void comp2_stop_delaytime(void)     
{
    if (bflg_comp2_stop_delaytime == 1)
    {
        gss_comp2_stop_delaytimer++;
        //if (gss_comp2_stop_delaytimer >= gss_para[COMP_STOP_DELAYTIME])   //300s=5min
        if (gss_comp2_stop_delaytimer >= gss_db_debug_parm_value[DB_P12]*60)   //min*60 = s
        {
            gss_comp2_stop_delaytimer = 0;
            bflg_comp2_stop_delaytime = 0;
        }
    }
}
/***********************************************************************************************************************************
函数功能:主传感器故障时，高温压缩机开机延时，开30min后关 

函数位置：1min--------------------------------ok
************************************************************************************************************************************/
void comp2_TA_fault_run_delaytime(void)
{
    if (bflg_comp2_TA_fault_run_delaytime == 1)
    {
        gss_comp2_TA_fault_run_delaytimer++;
        if (gss_comp2_TA_fault_run_delaytimer >= ram_para2[num_comp2_TA_fault_run_delaytime])  //30
        {
            gss_comp2_TA_fault_run_delaytimer = 0;
            bflg_comp2_TA_fault_run_delaytime = 0;
        }
    }
}
/***************************************************************************************************************************
函数功能:主传感器故障，高温压缩机关机延时，关机10min后才能开

函数位置：1s定时器---------------------------------------ok
****************************************************************************************************************************/
void comp2_TA_fault_stop_delaytime(void)
{
    if (bflg_comp2_TA_fault_stop_delaytime == 1)
    {
        gss_comp2_TA_fault_stop_delaytimer++;
        if (gss_comp2_TA_fault_stop_delaytimer >= ram_para2[num_comp2_TA_fault_stop_delaytime])  //600s = 10min
        {
            gss_comp2_TA_fault_stop_delaytimer = 0;
            bflg_comp2_TA_fault_stop_delaytime = 0;
        }
    }
}

/*****************************************************************************************************************************
函数功能:高压机首次上电连续16小时、其他时间连续运行5小时后强制停机 

函数位置：1min---------------------------------ok
******************************************************************************************************************************/
void comp2_continue_run_delaytime(void)
{
    if (bflg_comp2_continue_run_delaytime == 1)
    {
        gss_comp2_continue_run_delaytimer++;
        if (bflg_comp2_first_run == 1)  //如果是首次运行
        {
            //if (gss_comp2_continue_run_delaytimer >= ram_para2[num_comp2_continue_first_run_delaytime])  //960min = 16h
            if (gss_comp2_continue_run_delaytimer >= gss_db_debug_parm_value[DB_P05]*60)  //h*60 = min
            {
                gss_comp2_continue_run_delaytimer = 0;
                bflg_comp2_continue_run_delaytime = 0;
            }
        }
        else   //如果是正常运行
        {
            //if (gss_comp2_continue_run_delaytimer >= ram_para2[num_comp2_continue_run_delaytime])  //300min= 5h
            if (gss_comp2_continue_run_delaytimer >= gss_db_debug_parm_value[DB_P04]*60)  //h*60 = min
            {
                gss_comp2_continue_run_delaytimer = 0;
                bflg_comp2_continue_run_delaytime = 0;
            }
        }
    }
}

/****************************************************************************************************************************
函数功能: 高压机强制停机延时，强制停机后，若要开机需延时10min再开机

函数位置：1s定时器-----------------------------------ok
*****************************************************************************************************************************/
void comp2_force_stop_delaytime(void)
{
    if (bflg_comp2_force_stop_delaytime == 1)
    {
        gss_comp2_force_stop_delaytimer++;
        //if (gss_comp2_force_stop_delaytimer >= ram_para2[num_comp2_force_stop_delaytime]) //600s = 10min
        if (gss_comp2_force_stop_delaytimer >=  gss_db_debug_parm_value[DB_P06]*60) //min*60 = s
        {
            gss_comp2_force_stop_delaytimer = 0;
            bflg_comp2_force_stop_delaytime = 0;
        }
    }
}
/***************************************************************************************************************************
函数功能:高温压机开机延时后，才能开低温压机
         

函数位置：1s定时器--------------------------------ok
****************************************************************************************************************************/
void comp2_run_space_delaytime(void)     
{
    if (bflg_comp2_run_space_delaytime == 1)
    {
        gss_comp2_run_space_delaytimer++;
        if(gss_comp2_run_space_delaytimer >= gss_db_debug_parm_value[DB_P03]*60) //min*60 = s
        {
            gss_comp2_run_space_delaytimer = 0;
            bflg_comp2_run_space_delaytime = 0;
        }
       /* gss_comp2_run_space_delaytimer++;
        if (bflg_comp2_first_run == 1)  //如果是首次运行
        {
            if (gss_comp2_run_space_delaytimer >= ram_para2[num_comp2_first_run_space_delaytime])   //600s = 10min
            {
                gss_comp2_run_space_delaytimer = 0;
                bflg_comp2_run_space_delaytime = 0;
            }
        }
        else
        {
            if (gss_comp2_run_space_delaytimer >= ram_para2[num_comp2_run_space_delaytime])   //60s=1min
            {
                gss_comp2_run_space_delaytimer = 0;
                bflg_comp2_run_space_delaytime = 0;
            }
        }*/
    }
}

/*********************************************************************************************************************************************************
函数功能：正常开机时，以2000转运行1min

函数位置：1s定时器--------------------------ok
*********************************************************************************************************************************************************/
void comp2_start_delaytime(void)
{
    if (bflg_comp2_start_delaytime == 1)
    {
        gss_comp2_start_delaytimer++;
        if (gss_comp2_start_delaytimer >= gss_para[COMP2_START_TIME])
        {
            gss_comp2_start_delaytimer = 0;
            bflg_comp2_start_delaytime = 0;
        }
    }
}
/*********************************************************************************************************************************************************
函数功能：压机的转速每间隔80s进行一次更新

函数位置：1s定时器--------------------------------------ok
*********************************************************************************************************************************************************/
void comp2_update_delaytime(void)
{
    if (bflg_comp2_update_delaytime == 1)
    {
        gss_comp2_update_delaytimer++;
        if (gss_comp2_update_delaytimer >= gss_para[COMP2_UPDATE_TIME])
        {
            gss_comp2_update_delaytimer = 0;
            bflg_comp2_update_delaytime = 0;
        }
    }
}
/*********************************************************************************************************************************************************
函数功能：低温压机正常启动时以2000转运行1min 或断电后再启动以4500运行1min

函数位置：1s定时器------------------------------------ok
*********************************************************************************************************************************************************/
void comp21_start_delaytime(void)
{
    if (bflg_comp21_start_delaytime == 1)
    {
        gss_comp21_start_delaytimer++;
        if (gss_comp21_start_delaytimer >= gss_para[COMP21_START_TIME])
        {
            gss_comp21_start_delaytimer = 0;
            bflg_comp21_start_delaytime = 0;
        }
    }
}
/*********************************************************************************************************************************************************
函数功能：低温压机转速每80s更新一次

函数位置：1s--------------------------------------ok
*********************************************************************************************************************************************************/
void comp21_update_delaytime(void)
{
    if (bflg_comp21_update_delaytime == 1)
    {
        gss_comp21_update_delaytimer++;
        if (gss_comp21_update_delaytimer >= gss_para[COMP21_UPDATE_TIME])
        {
            gss_comp21_update_delaytimer = 0;
            bflg_comp21_update_delaytime = 0;
        }
    }
}

/*********************************************************************************************************************************
函数功能: 断电后重新上电，达到开机条件，高温压机需要延时1min后再启动

函数位置: 1s定时器中------------------------------------ok
*********************************************************************************************************************************/
void comp2_power_down_delaytime(void)
{
    if(bflg_power_down_delaytime  == 1)
    {
        gss_power_down_delaytime++;
        if(gss_power_down_delaytime >= 60)
        {
            gss_power_down_delaytime = 0;
            bflg_power_down_delaytime = 0;
        }
    }
}
/***********************************************************************************************************************
函数功能：高、低压机的频率控制

函数位置；1s定时器中------------------------------------------------ok
***********************************************************************************************************************/
void freq_ctrl_deal(void)
{
    int32_t lss_tmp;
    //----------------------------------
    if (bflg_db_first_comp_runing == 0)   //未开机
    {
        if (gss_comp2_freq != 0)
        {
            gss_comp2_freq = 0;
        }
    }
    else                           //开机
    {
        if((bflg_comp2_first_power_on == 1)||(guc_comp2_power_on_again == 2)) //初次上电/断电再上电
        {
            bflg_comp2_first_power_on = 0;
            guc_comp2_power_on_again = 0;
            gss_comp2_freq = gss_para[COMP2_FREQ_MAX];   //4500

            bflg_comp2_update_delaytime = 1;             //80s后在更新
            gss_comp2_update_delaytimer = 0;
        }
        else
        {   
            if (gss_comp2_freq == 0)
            {
                gss_comp2_freq = gss_para[COMP2_START_FREQ];
                
                bflg_comp2_start_delaytime = 1;
                gss_comp2_start_delaytimer = 0;
            }
            else
            {
                if (bflg_comp2_start_delaytime == 0)
                {
                    if (bflg_comp2_update_delaytime == 0)
                    {
                        bflg_comp2_update_delaytime = 1;
                        gss_comp2_update_delaytimer = 0;
                        
                        if (bflg_db_secend_comp_runing == 0)
                        {
                            lss_tmp = gss_TE;
                            lss_tmp -= gss_para[COMP2_SET_TEMP1];
                            lss_tmp *= gss_para[COMP2_FREQ_DELT_K1];
                        }
                        else
                        {
                            lss_tmp = gss_TE;
                            lss_tmp -= gss_para[COMP2_SET_TEMP2];
                            lss_tmp *= gss_para[COMP2_FREQ_DELT_K2];
                        }
                        
                        if(lss_tmp < gss_para[COMP2_FREQ_DELT_MIN])
                        {
                            lss_tmp = gss_para[COMP2_FREQ_DELT_MIN];
                        }
                        
                        if (lss_tmp > gss_para[COMP2_FREQ_DELT_MAX])
                        {
                            lss_tmp = gss_para[COMP2_FREQ_DELT_MAX];
                        }
                        
                        gss_comp2_delt_freq = (int16_t) (lss_tmp);
                        gss_comp2_freq += gss_comp2_delt_freq;
                        
                        if (gss_comp2_freq >= gss_para[COMP2_FREQ_MAX])
                        {
                            gss_comp2_freq = gss_para[COMP2_FREQ_MAX];
                        }
                        if (gss_comp2_freq <= gss_para[COMP2_FREQ_MIN])
                        {
                            gss_comp2_freq = gss_para[COMP2_FREQ_MIN];
                        }
                        
                    }
                }
            }
        }
    }
    //----------------------------------
    //低温压机
    if (bflg_db_secend_comp_runing == 0)
    {
        if (gss_comp21_freq != 0)
        {
            gss_comp21_freq = 0;
        }
    }
    else
    {
        if(guc_power_down_lfreq_cnt == 2)
        {
            guc_power_down_lfreq_cnt = 0;
            gss_comp21_freq = 4500;          //断电又重新上电后低压机第一次启动以4500转

            bflg_comp21_start_delaytime = 1;
            gss_comp21_start_delaytimer = 0;
        }
        else
        {
            if (gss_comp21_freq == 0)
            {
                gss_comp21_freq = gss_para[COMP21_START_FREQ];
                
                bflg_comp21_start_delaytime = 1;
                gss_comp21_start_delaytimer = 0;
            }
            else
            {
                if (bflg_comp21_start_delaytime == 0)
                {
                    if (bflg_comp21_update_delaytime == 0)
                    {
                        bflg_comp21_update_delaytime = 1;
                        gss_comp21_update_delaytimer = 0;
                        
                        lss_tmp = gss_TA;
                        lss_tmp -= gss_set_parm_value[DISP_SET_TS];
                        lss_tmp += (gss_para[COMP21_FREQ_DELT_TEMP] * 10);
                        lss_tmp *= gss_para[COMP21_FREQ_DELT_K];
                        lss_tmp /= 10;
                        
                        if(lss_tmp < gss_para[COMP21_FREQ_DELT_MIN])
                        {
                            lss_tmp = gss_para[COMP21_FREQ_DELT_MIN];
                        }
                        
                        if (lss_tmp > gss_para[COMP21_FREQ_DELT_MAX])
                        {
                            lss_tmp = gss_para[COMP21_FREQ_DELT_MAX];
                        }
                        
                        gss_comp21_delt_freq = (int16_t) (lss_tmp);
                        gss_comp21_freq += gss_comp21_delt_freq;
                        
                        if (gss_comp21_freq >= gss_para[COMP21_FREQ_MAX])
                        {
                            gss_comp21_freq = gss_para[COMP21_FREQ_MAX];
                        }
                        if (gss_comp21_freq <= gss_para[COMP21_FREQ_MIN])
                        {
                            gss_comp21_freq = gss_para[COMP21_FREQ_MIN];
                        }
                        //--------------------------------------
                        if (gss_comp21_delt_freq < 0)
                        {
                        	  bflg_comp21_door_freq_tmp = 0;
                        }
                    }
                    //------------------------------------------
                    if (bflg_comp2_large_freq == 1)                       //开门1s 且>5°
                    {
                        gss_comp21_freq = gss_para[COMP21_FREQ_MAX];      //4500
                        
                        bflg_comp2_large_freq = 0;                        //bflg_comp2_large_freq 按习惯应改为--->bflg_comp21_large_freq，因为只调低温压机
                    }
                    else if (bflg_comp21_door_freq == 1)                   //开门1s
                    {
                        if (gss_comp21_freq < gss_para[COMP_DOOR_FREQ])
                        {
                        	  gss_comp21_freq = gss_para[COMP_DOOR_FREQ];  //3500
                        }
                        
                        bflg_comp21_door_freq = 0;
                    }
                }
            }
        }
    }
    //----------------------------------
    if (gss_comp2_freq == 0)
    {
    	  gus_comp2_freq_cnt = (60000 - 1);
    }
    else
    {
    	  gus_comp2_freq_cnt = (uint16_t) (60000000 / gss_comp2_freq);
    }
    //----------------------------------
    if (gss_comp21_freq == 0)
    {
    	  gus_comp21_freq_cnt = (60000 - 1);
    }
    else
    {
    	  gus_comp21_freq_cnt = (uint16_t) (60000000 / gss_comp21_freq);
    }
}
/*******************************************************************************************************************************
函数功能: 高、低压机的参数

函数位置：初始化--------------------------------不使用了
********************************************************************************************************************************/
void init_freq_ctrl(void)
{
    gss_para[COMP2_START_FREQ] = 2000;        // 0
    gss_para[COMP2_START_TIME] = 60;          // 1
    gss_para[COMP2_UPDATE_TIME] = 80;         // 2
    gss_para[COMP2_SET_TEMP1] = -27;          // 3
    gss_para[COMP2_FREQ_DELT_K1] = 50;        // 4
    gss_para[COMP2_FREQ_DELT_MIN] = -20;      // 5
    gss_para[COMP2_FREQ_DELT_MAX] = 20;       // 6
    gss_para[COMP2_FREQ_MIN] = 2000;          // 7
    gss_para[COMP2_FREQ_MAX] = 4500;          // 8
    gss_para[COMP2_SET_TEMP2] = -25;          // 9
    gss_para[COMP2_FREQ_DELT_K2] = 50;        // 10
    gss_para[COMP21_START_FREQ] = 2000;       // 11
    gss_para[COMP21_START_TIME] = 60;         // 12
    gss_para[COMP21_UPDATE_TIME] = 80;        // 13
    gss_para[COMP21_FREQ_DELT_K] = 50;        // 14
    gss_para[COMP21_FREQ_DELT_MIN] = -20;     // 15
    gss_para[COMP21_FREQ_DELT_MAX] = 20;      // 16
    gss_para[COMP21_FREQ_MIN] = 2000;         // 17
    gss_para[COMP21_FREQ_MAX] = 4500;         // 18
    gss_para[COMP21_FREQ_DELT_TEMP] = 0;      // 19
    //gss_para[COMP_START_DELT_TEMP] = 15;    // 20    
    //gss_para[COMP_STOP_DELT_TEMP] = 0;      // 21    
    gss_para[COMP_START_DELT_TEMP] = ram_para[num_debug_P20]; // P20 开机点调整现可保存  默认15
    gss_para[COMP_STOP_DELT_TEMP]  = ram_para[num_debug_P21]; // P21 停机点调整               默认   0
    gss_para[PT100_DELT_TEMP] = 0;            // 22
    gss_para[COMP_STOP_DELAYTIME] = 300;      // 23 关机5min后才能开机
    gss_para[COMP_DOOR_TDELT] = 50;           // 24
    gss_para[COMP_DOOR_FREQ] = 3500;          // 25
    gss_para[COMP_RUN_DELAYTIME] = 300;       // 26 开机5min后才能关机
}

//db
/***********************************************************************************************************************************************
函数功能：调试参数的初始化(升级从E2中读取)

函数位置：系统(1)--------------------------------ok
************************************************************************************************************************************************/
void db_init_debug_parm_value(void)
{
    gss_db_debug_parm_value[DB_P00] = ram_para[num_db_debug_P00];   //3℃     magnify 10
    gss_db_debug_parm_value[DB_P01] = ram_para[num_db_debug_P01];   //0.2℃   magnify 10
    gss_db_debug_parm_value[DB_P02] = ram_para[num_db_debug_P02];   //3℃     magnify 10  
    gss_db_debug_parm_value[DB_P03] = ram_para[num_db_debug_P03];   //1min
    gss_db_debug_parm_value[DB_P04] = ram_para[num_db_debug_P04];   //5h
    gss_db_debug_parm_value[DB_P05] = ram_para[num_db_debug_P05];   //16h
    gss_db_debug_parm_value[DB_P06] = ram_para[num_db_debug_P06];   //10min
    gss_db_debug_parm_value[DB_P07] = ram_para[num_db_debug_P07];   //5min
    gss_db_debug_parm_value[DB_P08] = ram_para[num_db_debug_P08];   //-40℃
    gss_db_debug_parm_value[DB_P09] = ram_para[num_db_debug_P09];   //0.2℃    magnify 10 
    gss_db_debug_parm_value[DB_P10] = ram_para[num_db_debug_P10];   //-0.4℃   magnify 10
    gss_db_debug_parm_value[DB_P11] = ram_para[num_db_debug_P11];   //5min    
    gss_db_debug_parm_value[DB_P12] = ram_para[num_db_debug_P12];   //5min
    gss_db_debug_parm_value[DB_P13] = ram_para[num_db_debug_P13];   //1min 
}

/***********************************************************************************************************************************************
函数功能: 只有在压机运行时进行切换：

          1、系统正常时：single模式切换到double模式
          
          2、系统在正常和不正常状态之间的切换：

         
         guc_db_comp_mode   : 1、single(一个压机运行模式)
                              2、double(两个压机运行模式)
                              
         guc_db_system_state: 1、正常状态 (single or double) 
                              2、A故障       (single)  
                              3、B故障       (single) 
                              3、AB都故障 (double)
                              
         根据上面判断的状态进行实际输出           ：1、正常运行         (single or double) 
                                         2、A故障         (single)
                                         3、B故障         (single)
                                         4、AB都故障       (double)

函数位置：1s--db_comp_run_stop_ask_for_delaytime()--------------------------------ok
************************************************************************************************************************************************/
void db_single_to_double_mode_switch(void)
{
    if(guc_db_system_state == SYSTEM_NORMAL)  //无故障时
    {
        if(guc_db_comp_mode == SINGLE_MODE)
        {
            if(gss_TA - gss_set_parm_value[DISP_SET_TS] > gss_db_debug_parm_value[DB_P00])
            {
                gss_single_to_double_switch_delaytime++;
                if(gss_single_to_double_switch_delaytime >= 30)  //30s 自+
                {
                    gss_single_to_double_switch_delaytime = 0;
                    
                    guc_db_system_state = SYSTEM_NORMAL;
                    guc_db_comp_mode = DOUBLE_MODE;              //切换到double
                }
             }
             else
             {
                gss_single_to_double_switch_delaytime = 0;
             }
        }
        else
        {
            gss_single_to_double_switch_delaytime = 0;
        }
    }
    else
    {
        gss_single_to_double_switch_delaytime = 0;
    }

    //---------------------------------------------------------------
    //运行中模式发生转换
    if((bflg_db_A_system_fault == 1)&&(bflg_db_B_system_fault == 1))  //AB都故障
    {
        if(guc_db_system_state != SYSTEM_AB_FAULT)
        {
            guc_db_system_state = SYSTEM_AB_FAULT;
            guc_db_comp_mode = DOUBLE_MODE;
            
            bflg_comp2_continue_run_delaytime = 1;   //时间重新更新
            gss_comp2_continue_run_delaytimer = 0;
        }
    }
    else if((bflg_db_A_system_fault == 1)&&(bflg_db_B_system_fault == 0))//只A故障
    {
        if(guc_db_system_state != SYSTEM_A_FAULT)
        {
            guc_db_system_state = SYSTEM_A_FAULT;
            guc_db_comp_mode = SINGLE_MODE;
            bflg_first_comp_open = B_SYSTEM;      //单独运行B压机，防止切换到全/无故障时跳变
            
            bflg_comp2_continue_run_delaytime = 1;   
            gss_comp2_continue_run_delaytimer = 0;
        }
    }
    else if((bflg_db_A_system_fault == 0)&&(bflg_db_B_system_fault == 1)) //只B故障
    {
        if(guc_db_system_state != SYSTEM_B_FAULT)
        {
            guc_db_system_state = SYSTEM_B_FAULT;
            guc_db_comp_mode = SINGLE_MODE;
            bflg_first_comp_open = A_SYSTEM;
            
            bflg_comp2_continue_run_delaytime = 1;   
            gss_comp2_continue_run_delaytimer = 0;
        }
    }    
    else if((bflg_db_A_system_fault == 0)&&(bflg_db_B_system_fault == 0)) //无故障
    {
        if(guc_db_system_state != SYSTEM_NORMAL)
        {
            guc_db_system_state = SYSTEM_NORMAL;     //从有故障切到无故障的单模式，因为单可转为双；
            guc_db_comp_mode = SINGLE_MODE;
            
            bflg_comp2_continue_run_delaytime = 1;   
            gss_comp2_continue_run_delaytimer = 0;
        }
    }    
}
/***********************************************************************************************************************************************
函数功能: AB系统运行xmin后判断是否故障延时

函数位置：1min-------------------------------未调
************************************************************************************************************************************************/
void db_comp_fault_judge_delaytime(void)
{
    if(RL_A_COMP_OUT_PIN == 1)      //A系统故障：每次压机开启都是5min后判断
    {
        if(bflg_db_A_system_runing_delaytime == 0)
        {
            gss_db_A_system_runing_delaytime++;
            if(gss_db_A_system_runing_delaytime >= gss_db_debug_parm_value[DB_P07])  //min
            {
                bflg_db_A_system_runing_delaytime = 1;
                gss_db_A_system_runing_delaytime = 0;
            }
        }
    }
    else
    {
        bflg_db_A_system_runing_delaytime = 0;
        gss_db_A_system_runing_delaytime = 0;
    }
    //----------------------------------------------------------------------------
    if(RL_B_COMP_OUT_PIN == 1)      //B系统
    {
        if(bflg_db_B_system_runing_delaytime == 0)
        {
            gss_db_B_system_runing_delaytime++;
            if(gss_db_B_system_runing_delaytime >= gss_db_debug_parm_value[DB_P07])  //min
            {
                bflg_db_B_system_runing_delaytime = 1;
                gss_db_B_system_runing_delaytime = 0;
            }
        }
    }
    else
    {
        bflg_db_B_system_runing_delaytime = 0;
        gss_db_B_system_runing_delaytime = 0;
    }
}
/***********************************************************************************************************************************************
函数功能:     系统故障判定

函数位置：1s-------------------------------------------OK
************************************************************************************************************************************************/
void db_comp_system_fault_judge(void)
{
    if(bflg_db_A_system_fault == 0)      //A系统无故障
    {
        if(bflg_db_A_system_runing_delaytime == 1)
        {
            if(gss_adg_Temp[6] > gss_db_debug_parm_value[DB_P08])  //gss_adg_Temp[6] = 7号热电偶
            {
                gss_db_A_system_fault_delaytime++;
                if(gss_db_A_system_fault_delaytime >= 30) //30s
                {
                    gss_db_A_system_fault_delaytime = 0;
                    bflg_db_A_system_fault = 1;           //A系统故障
                }
            }
            else
            {
                gss_db_A_system_fault_delaytime = 0;
            } 
        }
        else
        {
            gss_db_A_system_fault_delaytime = 0;
        }
    }
    else                                //A系统故障
    {
        if(gss_adg_Temp[6] <= gss_db_debug_parm_value[DB_P08]) 
        {
            gss_db_A_system_fault_delaytime++;
            if(gss_db_A_system_fault_delaytime >= 30) //30s
            {
                gss_db_A_system_fault_delaytime = 0;
                bflg_db_A_system_fault = 0;       //A系统故障解除
            }
        }
        else
        {
            gss_db_A_system_fault_delaytime = 0;
        }
    }
    //------------------------------------------------------------------------------------------
    //B
    if(bflg_db_B_system_fault == 0)    //B系统无故障
    {
        if(bflg_db_B_system_runing_delaytime == 1)
        {
            if(gss_adg_Temp[7] > gss_db_debug_parm_value[DB_P08])  ////gss_adg_Temp[7] = 8号
            {
                gss_db_B_system_fault_delaytime++;
                if(gss_db_B_system_fault_delaytime >= 30) //30s
                {
                    gss_db_B_system_fault_delaytime = 0;
                    bflg_db_B_system_fault = 1;           //B系统故障
                }
            }
            else
            {
                gss_db_B_system_fault_delaytime = 0;
            } 
        }
        else
        {
            gss_db_B_system_fault_delaytime = 0;
        }
    }
    else                      //B系统故障
    {
        if(gss_adg_Temp[7] <= gss_db_debug_parm_value[DB_P08]) 
        {
            gss_db_B_system_fault_delaytime++;
            if(gss_db_B_system_fault_delaytime >= 30) //30s
            {
                gss_db_B_system_fault_delaytime = 0;
                bflg_db_B_system_fault = 0;           //B系统故障解除
            }
        }
        else
        {
            gss_db_B_system_fault_delaytime = 0;
        }
    }
}


/***********************************************************************************************************************************************
函数功能: 压机的输出和指示灯

函数位置:主循环--------------------------------ok
************************************************************************************************************************************************/
void db_comp_RL_out(void)
{
    if((bflg_factory_AT_test == 0)&&(bflg_self_test == 0))               // 没有自检和测试保证led正常
    {
        if(bflg_Tcouple_reverse == 0)                       //AB的热电偶未插反
        {
            if(guc_db_system_state == SYSTEM_NORMAL)        //AB都正常
            {
                if(bflg_db_first_comp_runing == 1) //首先开的压机
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_A_COMP_OUT_PIN = 1;  //A压机开
                        LED8 |= 0x20;           //A指示灯开                
                    }
                    else
                    {
                        RL_B_COMP_OUT_PIN = 1;  //B压机开
                        LED8 |= 0x40;           //B指示灯开
                    }
                }
                else
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_A_COMP_OUT_PIN = 0;  //A压机关
                        LED8 &= ~0x20;          //A指示灯关
                    }
                    else
                    {
                        RL_B_COMP_OUT_PIN = 0;  //B压机关
                        LED8 &= ~0x40;          //B指示灯关
                    }
                }
                
                //-----------------------------------------------------
                if(bflg_db_secend_comp_runing == 1) //后开的压机
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_B_COMP_OUT_PIN = 1;  //B压机开
                        LED8 |= 0x40;           
                    }
                    else
                    {
                        RL_A_COMP_OUT_PIN = 1;  //A压机开
                        LED8 |= 0x20;           
                    }
                }
                else
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_B_COMP_OUT_PIN = 0;  //B压机关
                        LED8 &= ~0x40;          
                    }
                    else
                    {
                        RL_A_COMP_OUT_PIN = 0;  //A压机关
                        LED8 &= ~0x20; 
                    }
                }
            }
            else if(guc_db_system_state == SYSTEM_A_FAULT)  //A故障B正常
            {
                if(gss_factory_parm_value[DISP_FACTORY_Pb] == 0)  //未屏蔽故障闪烁
                {
                    //A灯闪
                    if (bflg_led_loop_set == 1)
                    {
                        LED8 |= 0x20;             
                    }
                    else
                    {
                        LED8 &= ~0x20;
                    }
                }
                else
                {
                    LED8 &= ~0x20;
                }
                //-------------------------------------------------------
                if(bflg_db_first_comp_runing == 1) //首先开的压机
                {
                    RL_A_COMP_OUT_PIN = 0;  
                    RL_B_COMP_OUT_PIN = 1;  
                    LED8 |= 0x40;           //B指示灯开
                }
                else
                {
                    RL_A_COMP_OUT_PIN = 0;  
                    RL_B_COMP_OUT_PIN = 0;  
                    LED8 &= ~0x40;          //B指示灯关
                }
            }
            else if(guc_db_system_state == SYSTEM_B_FAULT)  //A正常B故障
            {
                if(gss_factory_parm_value[DISP_FACTORY_Pb] == 0)  //未屏蔽故障闪烁
                {
                    //B灯闪
                    if (bflg_led_loop_set == 1)
                    {
                        LED8 |= 0x40;               
                    }
                    else
                    {
                         LED8 &= ~0x40;
                    }
                }
                else
                {
                    LED8 &= ~0x40;
                }
                //-------------------------------------------------------
                if(bflg_db_first_comp_runing == 1) //首先开的压机
                {
                    RL_A_COMP_OUT_PIN = 1;  
                    RL_B_COMP_OUT_PIN = 0;  
                    LED8 |= 0x20;           //A指示灯开
                }
                else
                {
                    RL_A_COMP_OUT_PIN = 0;  
                    RL_B_COMP_OUT_PIN = 0;  
                    LED8 &= ~0x20;          //A指示灯关            
                }
            }  
            else if(guc_db_system_state == SYSTEM_AB_FAULT) //AB都故障
            {
                if(gss_factory_parm_value[DISP_FACTORY_Pb] == 0)  //未屏蔽故障闪烁
                {
                    //AB都闪
                    if (bflg_led_loop_set == 1)
                    {
                        LED8 |= 0x20; 
                        LED8 |= 0x40;               
                    }
                    else
                    {
                        LED8 &= ~0x20;        
                        LED8 &= ~0x40;
                    }
                }
                else
                {
                    if(RL_A_COMP_OUT_PIN == 1)
                    {
                        LED8 |= 0x20;
                    }
                    else
                    {
                        LED8 &= ~0x20; 
                    }
                    if(RL_B_COMP_OUT_PIN == 1)
                    {
                        LED8 |= 0x40;
                    }
                    else
                    {
                        LED8 &= ~0x40;
                    }
                }
                //-------------------------------------------------------
                if(bflg_db_first_comp_runing == 1) //首先开的压机
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_A_COMP_OUT_PIN = 1;  //A压机开
                    }
                    else
                    {
                        RL_B_COMP_OUT_PIN = 1;  //B压机开
                    }
                }
                else
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_A_COMP_OUT_PIN = 0;  //A压机关
                    }
                    else
                    {
                        RL_B_COMP_OUT_PIN = 0;  //B压机关
                    }
                }
                
                //-----------------------------------------------------
                if(bflg_db_secend_comp_runing == 1) //后开的压机
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_B_COMP_OUT_PIN = 1;  //B压机开
                    }
                    else
                    {
                        RL_A_COMP_OUT_PIN = 1;  //A压机开
                    }
                }
                else
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_B_COMP_OUT_PIN = 0;  //B压机关
                    }
                    else
                    {
                        RL_A_COMP_OUT_PIN = 0;  //A压机关
                    }
                }
            }
        }
        else
        {
            if (bflg_led_loop_set == 1)
            {
                LED8 |= 0x20; 
                LED8 |= 0x40;               
            }
            else
            {
                LED8 &= ~0x20;        
                LED8 &= ~0x40;
            }
            //----------------------------
            RL_A_COMP_OUT_PIN = 0;
            RL_B_COMP_OUT_PIN = 0;
        }
    }
}

/***********************************************************************************************************************************************
函数功能: 压机停机xmin后，风机停机

函数位置: 1s--------------------------------ok
************************************************************************************************************************************************/
void db_fan_stop_delaytime(void)
{
    if(bflg_db_comp_fan_stop_delaytime == 1)
    {
        gss_db_comp_fan_stop_delaytime++;
        if(gss_db_comp_fan_stop_delaytime >= gss_db_debug_parm_value[DB_P13]*60) //min*60 = s
        {
            gss_db_comp_fan_stop_delaytime = 0;
            bflg_db_comp_fan_stop_delaytime = 0;
        }
    }
}

/***********************************************************************************************************************************************
函数功能: 首次上强电（初次上电）时，判断7、8号两个热电偶是否插反的判断

          条件：
          1、因为初次上电时设定首先运行A系统，所以判B系统温度是否下降；
          2、B系统对应8号热电偶--gss_adg_Temp[7]
          3、刚开始运行时，需记忆当时B的温度；
          
          判断原因：
          1、A运行1min后，A温度未降低，B系统温度降低>10℃；

          解除热电偶插反：
          1、需要断电后重新上电；
          
函数位置: 主--------------------------------ok
************************************************************************************************************************************************/
void db_thermocouple_reverse_judge(void)
{
    //debug
    //bflg_Tcouple_reverse = 1;
    
    if(bflg_power_on_Tcouple_judge == 1)             //首次上电电偶插反是否判断过的标志
    {
        if(bflg_db_first_comp_runing == 1)           //压机运行
        {
            if(bflg_comp2_run_space_delaytime == 0)  //1min
            {
                bflg_power_on_Tcouple_judge = 0;     //需初始化设置为1-ok
                
                if(gss_adg_Temp[7] - gss_B_Tcouple_memory >= 10 ) //开始时需记忆-ok
                {
                    bflg_Tcouple_reverse = 1;        //两个热电偶插反
                }
            }
        }
    }
}




#endif
/***************************************END OF THE FILE*********************************************/
