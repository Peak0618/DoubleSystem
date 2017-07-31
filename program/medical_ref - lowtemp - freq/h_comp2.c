#ifndef _COMP2_C_
#define _COMP2_C_
//------------------------------------------------------------------------------
//�����ļ�����
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
//��������
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
//ѹ����������ر���
int16_t   gss_comp2_power_on_delaytimer;     //�ϵ���ʱ��ʱ��

int16_t   gss_comp2_run_T_delaytimer;        //����ѹ���ﵽ�����¶ȳ���ʱ��
int16_t   gss_comp2_stop_T_delaytimer;       //            �ػ�

int16_t   gss_comp2_run_delaytimer;          //����ѹ����������ʱ��ʱ��
int16_t   gss_comp2_stop_delaytimer;         //����ѹ�����ػ���ʱ��ʱ��
//----------------------------------------------------------
int16_t   gss_comp2_TA_fault_run_delaytimer;      //�����������ϸ���ѹ����������ʱ��ʱ��
int16_t   gss_comp2_TA_fault_stop_delaytimer;     //�����������ϸ���ѹ�����ػ���ʱ��ʱ��

int16_t   gss_comp2_continue_run_delaytimer;      //����ѹ�����������м�ʱ��
int16_t   gss_comp2_force_stop_delaytimer;        //����ѹ����ǿ��ֹͣ��ʱ��

int16_t   gss_comp2_run_space_delaytimer;         //����ѹ�������м����ʱ��ʱ��
int8_t    gss_power_down_delaytime;               //�ϵ�������ϵ����ѹ����ʱ
uint8_t   guc_comp2_power_on_again;               //�ϵ�������ϵ��ѹ����4500����


//Ƶ�ʿ�����ر���
int16_t   gss_para[27];

int16_t   gss_comp2_freq;     //����ѹ��Ƶ��
int16_t   gss_comp21_freq;    //����ѹ��Ƶ��   peak ��ת��

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

#define A_SYSTEM  1  //Ϊ�˳�ʼ�ϵ�ʱ��A�ȿ�
#define B_SYSTEM  0

//variate
int16_t gss_db_fault_mode_T_delaytime;         //����ϵͳ�й���ʱfaultģʽ���󿪻�ʱ��
int16_t gss_db_double_mode_T_delaytime;        //double����ģʽ������ʱ
int16_t gss_db_single_mode_T_delaytime;        //single����ģʽ������ʱ
int16_t gss_db_first_comp_stop_T_delaytime ;   //Aѹ��ͣ��������ʱ
int16_t gss_single_to_double_switch_delaytime; //single����ģʽ�л�double��ʱ
int16_t gss_db_comp_fan_stop_delaytime;        //ѹ��ͣ����ʱxmin����ͣ
int16_t gss_db_A_system_runing_delaytime;      //Aѹ����xmin���й���
int16_t gss_db_B_system_runing_delaytime;      //Bѹ����xmin���й���
int16_t gss_db_A_system_fault_delaytime;       //Aϵͳ�����ж���ʱ
int16_t gss_db_B_system_fault_delaytime;       //B.....
int16_t gss_B_Tcouple_memory;                  //�״��ϵ�ѹ������ʱ��¼Bϵͳ��Ӧ�ȵ�ż�¶�


uint8_t guc_db_comp_mode;                      //ѹ������ģʽdouble or single
uint8_t guc_db_system_state;                   //����ʱѹ����������״̬���ǹ���״̬



//array
int16_t gss_db_debug_parm_value[DB_SUM_P];    //���ԵĲ���ֵP00~P13

//macro
#define SINGLE_MODE        1       //��ϵͳģʽ
#define DOUBLE_MODE        2       //˫ϵͳģʽ

#define SYSTEM_NORMAL      0       //ϵͳ����
#define SYSTEM_A_FAULT     1       //Aϵͳ����
#define SYSTEM_B_FAULT     2       //Bϵͳ����
#define SYSTEM_AB_FAULT    3       //AB������


/***************************************************************************************************************************************
�������ܣ�     �ϵ���ʱ����,��ʼ�ϵ���ʱ 1min���ɵ��� ���ٶԸߡ���ѹ�����п���


����λ�ã�  1s-------------------------ok   
***************************************************************************************************************************************/
void comp2_power_on_delaytime(void)
{
    if (bflg_comp2_power_on_delaytime == 1) //�ϵ��ʼ��Ϊ1
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
��������:     ����Ҫ����¶ȿ���ͣ���㣬�жϸ���ѹ���Ƿ��п���ͣ��������
       
����λ��: 1s��ʱ����-----------------------------------------------������
*****************************************************************************************************************************************/
void comp2_run_stop_delaytime(void)
{
    if((bflg_alarm_power_off == 1)||(bflg_alarm_power_fault == 1))      //��ǿ�硢��12v           
    {
        gss_comp2_run_T_delaytimer = 0;
        gss_comp2_stop_T_delaytimer = 0;

        bflg_db_first_comp_askfor_run = 0;
        bflg_db_first_comp_askfor_stop = 0;
    }
    else
    {
        if (ram_para[num_comp2_mode] == 1)  //����Ǹ���������(Һ����LED��Ĭ�ϸ�����)     
        {
            if (bflg_db_first_comp_runing == 0)    //��ʱ�ػ�
            {
                if (gss_TA >= gss_set_parm_value[DISP_SET_TS] + gss_para[COMP_START_DELT_TEMP])   // ������
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
    	        else                                                                      //���� û�ж���
    	        {
    	        	  gss_comp2_run_T_delaytimer = 0;
    	        	  gss_comp2_stop_T_delaytimer = 0;
    	        	  bflg_db_first_comp_askfor_run = 0;
    	        }
            }
            else   //��ʱ����
            {
                if (gss_TA <= gss_set_parm_value[DISP_SET_TS] + gss_para[COMP_STOP_DELT_TEMP])  //ͣ����
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
        else  //��������
        {
            if (bflg_db_first_comp_runing == 0)
            {
    		    if (gss_TA >= gss_set_parm_value[DISP_SET_TS] + ram_para2[num_comp2_mode_T] + ram_para2[num_comp2_run_Tdelt])
    		    //if (gss_TA >= gss_set_parm_value[DISP_SET_TS] + gss_para[COMP_START_DELT_TEMP])  // 10 ����
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
                   //if (gss_TA <= gss_set_parm_value[DISP_SET_TS] + gss_para[COMP_STOP_DELT_TEMP])  // 10 ͣ��
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
            if ((gss_TA - gss_set_parm_value[DISP_SET_TS]) >= gss_para[COMP_DOOR_TDELT])     //5.0��
            {
                bflg_comp2_large_freq = 1;
                bflg_comp21_door_freq_tmp = 0;
            }
        }
    }
}



/***********************************************************************************************************************************************
�������ܣ�1���жϵ�һ̨ѹ������ͣ���� 2���ж���������ģʽdouble or single
                            
                              
����λ�ã�1s--------------------------------ok
************************************************************************************************************************************************/
void db_comp_run_stop_ask_for_delaytime(void)
{
    if((bflg_alarm_power_off == 1)||(bflg_alarm_power_fault == 1))      //��ǿ�硢��12v           
    {
        gss_db_double_mode_T_delaytime = 0;
        gss_db_single_mode_T_delaytime = 0;
        gss_db_first_comp_stop_T_delaytime = 0;
    
        bflg_db_first_comp_askfor_run = 0;
        bflg_db_first_comp_askfor_stop = 0;
    }
    else
    {
        if(bflg_db_first_comp_runing == 0) //���˿̹ػ�
        {                                        
            gss_db_first_comp_stop_T_delaytime = 0;

            if(gss_TA - gss_set_parm_value[DISP_SET_TS] >= gss_db_debug_parm_value[DB_P09])  //��ͣ��
            {
                if((bflg_db_A_system_fault == 1)||(bflg_db_B_system_fault == 1))//����ϵͳ�й���ʱ(�ǳ����ϵ�)
                {
                    gss_db_single_mode_T_delaytime = 0;
                    gss_db_double_mode_T_delaytime = 0;
                    
                    gss_db_fault_mode_T_delaytime++;
                    if(gss_db_fault_mode_T_delaytime >= 30) //30s ��+
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
                        if(gss_db_double_mode_T_delaytime >= 30)   //30s ��+
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
                    //---------------------------------------------�ϵ��������ʱ����B�ȵ�ż�¶�
                    if(bflg_db_first_comp_askfor_run == 1)
                    {
                        if(bflg_power_on_Tcouple_judge == 1)
                        {
                            gss_B_Tcouple_memory = gss_adg_Temp[7];  //����տ���ʱ��B�ȵ�ż�¶�
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
        else    //�˿̿���
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
            
            //ϵͳģʽ�л� �� ��/˫ѹ�������л�
            db_single_to_double_mode_switch();
        }
    }
}

/********************************************************************************************************************************
��������: ��������µ�һ̨ѹ��������������    
          1���е�һ̨ѹ����ͣ������
          2����С��ͣ����ʱ�Ƿ������
          3��ǿ��ͣ���Ƿ������

����λ��:��ѭ����-------------------------------------------------------ok          
********************************************************************************************************************************/
void db_first_system_run_stop_deal(void)    
{
    if((bflg_alarm_power_off == 1)||(bflg_alarm_power_fault == 1))   //��ǿ�硢12v
    {
        bflg_db_first_comp_runing = 0;                  //ͣ��״̬

        bflg_comp2_run_space_delaytime = 0;
        gss_comp2_run_space_delaytimer = 0;

        bflg_comp2_run_delaytime = 0;            //�ÿ�����ʱ��־
        gss_comp2_run_delaytimer = 0;            //�忪����ʱ��ʱ��

        bflg_comp2_continue_run_delaytime = 0;
        gss_comp2_continue_run_delaytimer = 0;

        bflg_comp2_first_run = 0;                //���״����б�־

        bflg_comp2_force_stop_delaytime = 0;     //��ǿ��ֹͣ��ʱ��־  //������Ҫ��ǿ��ֹͣ
        gss_comp2_force_stop_delaytimer = 0;     //��ǿ��ֹͣ��ʱ��ʱ��

        bflg_comp2_stop_delaytime = 0;           //�ùػ���ʱ��־
        gss_comp2_stop_delaytimer = 0;           //��ػ���ʱ��ʱ��

        //--------------������ʱ
        bflg_comp2_TA_fault_run_delaytime = 0;   //�������������Ͽ�����ʱ��־        // peak ��30����--������ ��ע3
        gss_comp2_TA_fault_run_delaytimer = 0;   //�������������Ͽ�����ʱ��ʱ��

        bflg_comp2_TA_fault_stop_delaytime = 0;  //�������������Ϲػ���ʱ��־
        gss_comp2_TA_fault_stop_delaytimer = 0;
    }
    else
    {
        if(bflg_power_down_delaytime == 0)      //���δ�ϵ���Ƕϵ����ʱ����
        {
            if (bflg_TA_fault == 1)             //���������������   
            {
                if (bflg_db_first_comp_runing == 0)    //���������Ĺػ�
                {
                    if (bflg_comp2_TA_fault_stop_delaytime == 0)    //��������������Ϲػ���ʱʱ�����   //peak ��10���ӽ���--������ ��ע3
                    {
                        bflg_db_first_comp_runing = 1;    //����
                        bflg_first_comp_open = ~bflg_first_comp_open;
                        
                        bflg_comp2_TA_fault_run_delaytime = 1;      //�������������Ͽ�����ʱ��־        
                        gss_comp2_TA_fault_run_delaytimer = 0;      //�������������Ͽ�����ʱ��ʱ��
                        
                        bflg_comp2_run_space_delaytime = 1;        //����ѹ�������м����ʱ��־  //��ע1
                        gss_comp2_run_space_delaytimer = 0;
                    }
                }
                else   //�������
                {
                    if (bflg_comp2_TA_fault_run_delaytime == 0)     //��������������Ͽ�����ʱʱ�����
                    {
                        if (bflg_comp2_volt_low == 0)     //���û�е�ѹ����
                        {
                            bflg_db_first_comp_runing = 0;       //�ػ�
                            
                            bflg_comp2_first_run = 0;     //���״����б�־
                            
                            bflg_comp2_TA_fault_stop_delaytime = 1; //�������������Ϲػ���ʱ��־
                            gss_comp2_TA_fault_stop_delaytimer = 0; //�������������Ϲػ���ʱ��ʱ��

                            bflg_db_comp_fan_stop_delaytime = 1;
                            gss_db_comp_fan_stop_delaytime = 0;
                        }
                    }
                }
            }
            else       //���������������
            {
                if (bflg_db_first_comp_runing == 0)     //��
                {
                    if (bflg_db_first_comp_askfor_run == 1)     
                    {
                        if ((bflg_comp2_stop_delaytime == 0) && (bflg_comp2_force_stop_delaytime == 0))//ͣ����ʱʱ�����
                        {
                            bflg_db_first_comp_askfor_run = 0;             //�����󿪻���־
                            bflg_db_first_comp_runing = 1;                 //����
                            bflg_first_comp_open = ~bflg_first_comp_open;  //���濪��
                            
                            bflg_comp2_run_space_delaytime = 1;
                            gss_comp2_run_space_delaytimer = 0;
                            
                            bflg_comp2_run_delaytime = 1; //�ÿ�����ʱ��־
                            gss_comp2_run_delaytimer = 0; //�忪����ʱ��ʱ��
                            
                            bflg_comp2_continue_run_delaytime = 1;
                            gss_comp2_continue_run_delaytimer = 0;
                        }
                    }
                }
                else    //�������
                {
                    if (bflg_comp2_continue_run_delaytime == 0)     //�������ѹ������������ʱ�����
                    {
                        if (bflg_comp2_volt_low == 0)     //���û�е�ѹ����
                        {
                            bflg_db_first_comp_runing = 0;  //�ػ�
                            
                            bflg_comp2_first_run = 0;     //���״����б�־
                            
                            bflg_comp2_force_stop_delaytime = 1;    //��ǿ��ֹͣ��ʱ��־  //������Ҫ��ǿ��ֹͣ
                            gss_comp2_force_stop_delaytimer = 0;    //��ǿ��ֹͣ��ʱ��ʱ��

                            bflg_db_comp_fan_stop_delaytime = 1;
                            gss_db_comp_fan_stop_delaytime = 0;
                        }
                    }
                    else     //���ѹ����������ʱ��δ����
                    {
                        if (bflg_db_first_comp_askfor_stop == 1)  
                        {
                            if (bflg_comp2_run_delaytime == 0) //������ʱʱ�����
                            {
                                if (bflg_comp2_volt_low == 0)            //���û�е�ѹ����
                                {
                                    bflg_db_first_comp_askfor_stop = 0;     //������ػ���־
                                    bflg_db_first_comp_runing = 0;         //�ػ�
                                    
                                    bflg_comp2_first_run = 0;     //���״����б�־
                            
                                    bflg_comp2_stop_delaytime = 1;  //�ùػ���ʱ��־
                                    gss_comp2_stop_delaytimer = 0;  //��ػ���ʱ��ʱ��   
                                    
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
��������:�ڶ�̨ѹ������������
         1��first����1min��
         2������ģʽ��doubleģʽ��

����λ�ã���ѭ��---------------------------------ok
********************************************************************************************************************************/
void db_secend_system_run_stop_deal(void) 
{
    if (bflg_db_first_comp_runing == 0)    
    {
        bflg_db_secend_comp_runing = 0;   
    }
    else  
    {
        if (bflg_comp2_run_space_delaytime == 0)   //��ʱ1min��
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
��������:��̿���ʱ�䣬����ѹ�����ٿ���5min���ܹػ�,

����λ�ã�1s��ʱ��--------------------------------------ok  
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
��������: ���ͣ��ʱ�䣬���ٹػ�5min�������¿���

����λ�ã�1s��ʱ��-------------------------------ok
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
��������:������������ʱ������ѹ����������ʱ����30min��� 

����λ�ã�1min--------------------------------ok
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
��������:�����������ϣ�����ѹ�����ػ���ʱ���ػ�10min����ܿ�

����λ�ã�1s��ʱ��---------------------------------------ok
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
��������:��ѹ���״��ϵ�����16Сʱ������ʱ����������5Сʱ��ǿ��ͣ�� 

����λ�ã�1min---------------------------------ok
******************************************************************************************************************************/
void comp2_continue_run_delaytime(void)
{
    if (bflg_comp2_continue_run_delaytime == 1)
    {
        gss_comp2_continue_run_delaytimer++;
        if (bflg_comp2_first_run == 1)  //������״�����
        {
            //if (gss_comp2_continue_run_delaytimer >= ram_para2[num_comp2_continue_first_run_delaytime])  //960min = 16h
            if (gss_comp2_continue_run_delaytimer >= gss_db_debug_parm_value[DB_P05]*60)  //h*60 = min
            {
                gss_comp2_continue_run_delaytimer = 0;
                bflg_comp2_continue_run_delaytime = 0;
            }
        }
        else   //�������������
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
��������: ��ѹ��ǿ��ͣ����ʱ��ǿ��ͣ������Ҫ��������ʱ10min�ٿ���

����λ�ã�1s��ʱ��-----------------------------------ok
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
��������:����ѹ��������ʱ�󣬲��ܿ�����ѹ��
         

����λ�ã�1s��ʱ��--------------------------------ok
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
        if (bflg_comp2_first_run == 1)  //������״�����
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
�������ܣ���������ʱ����2000ת����1min

����λ�ã�1s��ʱ��--------------------------ok
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
�������ܣ�ѹ����ת��ÿ���80s����һ�θ���

����λ�ã�1s��ʱ��--------------------------------------ok
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
�������ܣ�����ѹ����������ʱ��2000ת����1min ��ϵ����������4500����1min

����λ�ã�1s��ʱ��------------------------------------ok
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
�������ܣ�����ѹ��ת��ÿ80s����һ��

����λ�ã�1s--------------------------------------ok
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
��������: �ϵ�������ϵ磬�ﵽ��������������ѹ����Ҫ��ʱ1min��������

����λ��: 1s��ʱ����------------------------------------ok
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
�������ܣ��ߡ���ѹ����Ƶ�ʿ���

����λ�ã�1s��ʱ����------------------------------------------------ok
***********************************************************************************************************************/
void freq_ctrl_deal(void)
{
    int32_t lss_tmp;
    //----------------------------------
    if (bflg_db_first_comp_runing == 0)   //δ����
    {
        if (gss_comp2_freq != 0)
        {
            gss_comp2_freq = 0;
        }
    }
    else                           //����
    {
        if((bflg_comp2_first_power_on == 1)||(guc_comp2_power_on_again == 2)) //�����ϵ�/�ϵ����ϵ�
        {
            bflg_comp2_first_power_on = 0;
            guc_comp2_power_on_again = 0;
            gss_comp2_freq = gss_para[COMP2_FREQ_MAX];   //4500

            bflg_comp2_update_delaytime = 1;             //80s���ڸ���
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
    //����ѹ��
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
            gss_comp21_freq = 4500;          //�ϵ��������ϵ���ѹ����һ��������4500ת

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
                    if (bflg_comp2_large_freq == 1)                       //����1s ��>5��
                    {
                        gss_comp21_freq = gss_para[COMP21_FREQ_MAX];      //4500
                        
                        bflg_comp2_large_freq = 0;                        //bflg_comp2_large_freq ��ϰ��Ӧ��Ϊ--->bflg_comp21_large_freq����Ϊֻ������ѹ��
                    }
                    else if (bflg_comp21_door_freq == 1)                   //����1s
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
��������: �ߡ���ѹ���Ĳ���

����λ�ã���ʼ��--------------------------------��ʹ����
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
    gss_para[COMP_START_DELT_TEMP] = ram_para[num_debug_P20]; // P20 ����������ֿɱ���  Ĭ��15
    gss_para[COMP_STOP_DELT_TEMP]  = ram_para[num_debug_P21]; // P21 ͣ�������               Ĭ��   0
    gss_para[PT100_DELT_TEMP] = 0;            // 22
    gss_para[COMP_STOP_DELAYTIME] = 300;      // 23 �ػ�5min����ܿ���
    gss_para[COMP_DOOR_TDELT] = 50;           // 24
    gss_para[COMP_DOOR_FREQ] = 3500;          // 25
    gss_para[COMP_RUN_DELAYTIME] = 300;       // 26 ����5min����ܹػ�
}

//db
/***********************************************************************************************************************************************
�������ܣ����Բ����ĳ�ʼ��(������E2�ж�ȡ)

����λ�ã�ϵͳ(1)--------------------------------ok
************************************************************************************************************************************************/
void db_init_debug_parm_value(void)
{
    gss_db_debug_parm_value[DB_P00] = ram_para[num_db_debug_P00];   //3��     magnify 10
    gss_db_debug_parm_value[DB_P01] = ram_para[num_db_debug_P01];   //0.2��   magnify 10
    gss_db_debug_parm_value[DB_P02] = ram_para[num_db_debug_P02];   //3��     magnify 10  
    gss_db_debug_parm_value[DB_P03] = ram_para[num_db_debug_P03];   //1min
    gss_db_debug_parm_value[DB_P04] = ram_para[num_db_debug_P04];   //5h
    gss_db_debug_parm_value[DB_P05] = ram_para[num_db_debug_P05];   //16h
    gss_db_debug_parm_value[DB_P06] = ram_para[num_db_debug_P06];   //10min
    gss_db_debug_parm_value[DB_P07] = ram_para[num_db_debug_P07];   //5min
    gss_db_debug_parm_value[DB_P08] = ram_para[num_db_debug_P08];   //-40��
    gss_db_debug_parm_value[DB_P09] = ram_para[num_db_debug_P09];   //0.2��    magnify 10 
    gss_db_debug_parm_value[DB_P10] = ram_para[num_db_debug_P10];   //-0.4��   magnify 10
    gss_db_debug_parm_value[DB_P11] = ram_para[num_db_debug_P11];   //5min    
    gss_db_debug_parm_value[DB_P12] = ram_para[num_db_debug_P12];   //5min
    gss_db_debug_parm_value[DB_P13] = ram_para[num_db_debug_P13];   //1min 
}

/***********************************************************************************************************************************************
��������: ֻ����ѹ������ʱ�����л���

          1��ϵͳ����ʱ��singleģʽ�л���doubleģʽ
          
          2��ϵͳ�������Ͳ�����״̬֮����л���

         
         guc_db_comp_mode   : 1��single(һ��ѹ������ģʽ)
                              2��double(����ѹ������ģʽ)
                              
         guc_db_system_state: 1������״̬ (single or double) 
                              2��A����       (single)  
                              3��B����       (single) 
                              3��AB������ (double)
                              
         ���������жϵ�״̬����ʵ�����           ��1����������         (single or double) 
                                         2��A����         (single)
                                         3��B����         (single)
                                         4��AB������       (double)

����λ�ã�1s--db_comp_run_stop_ask_for_delaytime()--------------------------------ok
************************************************************************************************************************************************/
void db_single_to_double_mode_switch(void)
{
    if(guc_db_system_state == SYSTEM_NORMAL)  //�޹���ʱ
    {
        if(guc_db_comp_mode == SINGLE_MODE)
        {
            if(gss_TA - gss_set_parm_value[DISP_SET_TS] > gss_db_debug_parm_value[DB_P00])
            {
                gss_single_to_double_switch_delaytime++;
                if(gss_single_to_double_switch_delaytime >= 30)  //30s ��+
                {
                    gss_single_to_double_switch_delaytime = 0;
                    
                    guc_db_system_state = SYSTEM_NORMAL;
                    guc_db_comp_mode = DOUBLE_MODE;              //�л���double
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
    //������ģʽ����ת��
    if((bflg_db_A_system_fault == 1)&&(bflg_db_B_system_fault == 1))  //AB������
    {
        if(guc_db_system_state != SYSTEM_AB_FAULT)
        {
            guc_db_system_state = SYSTEM_AB_FAULT;
            guc_db_comp_mode = DOUBLE_MODE;
            
            bflg_comp2_continue_run_delaytime = 1;   //ʱ�����¸���
            gss_comp2_continue_run_delaytimer = 0;
        }
    }
    else if((bflg_db_A_system_fault == 1)&&(bflg_db_B_system_fault == 0))//ֻA����
    {
        if(guc_db_system_state != SYSTEM_A_FAULT)
        {
            guc_db_system_state = SYSTEM_A_FAULT;
            guc_db_comp_mode = SINGLE_MODE;
            bflg_first_comp_open = B_SYSTEM;      //��������Bѹ������ֹ�л���ȫ/�޹���ʱ����
            
            bflg_comp2_continue_run_delaytime = 1;   
            gss_comp2_continue_run_delaytimer = 0;
        }
    }
    else if((bflg_db_A_system_fault == 0)&&(bflg_db_B_system_fault == 1)) //ֻB����
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
    else if((bflg_db_A_system_fault == 0)&&(bflg_db_B_system_fault == 0)) //�޹���
    {
        if(guc_db_system_state != SYSTEM_NORMAL)
        {
            guc_db_system_state = SYSTEM_NORMAL;     //���й����е��޹��ϵĵ�ģʽ����Ϊ����תΪ˫��
            guc_db_comp_mode = SINGLE_MODE;
            
            bflg_comp2_continue_run_delaytime = 1;   
            gss_comp2_continue_run_delaytimer = 0;
        }
    }    
}
/***********************************************************************************************************************************************
��������: ABϵͳ����xmin���ж��Ƿ������ʱ

����λ�ã�1min-------------------------------δ��
************************************************************************************************************************************************/
void db_comp_fault_judge_delaytime(void)
{
    if(RL_A_COMP_OUT_PIN == 1)      //Aϵͳ���ϣ�ÿ��ѹ����������5min���ж�
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
    if(RL_B_COMP_OUT_PIN == 1)      //Bϵͳ
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
��������:     ϵͳ�����ж�

����λ�ã�1s-------------------------------------------OK
************************************************************************************************************************************************/
void db_comp_system_fault_judge(void)
{
    if(bflg_db_A_system_fault == 0)      //Aϵͳ�޹���
    {
        if(bflg_db_A_system_runing_delaytime == 1)
        {
            if(gss_adg_Temp[6] > gss_db_debug_parm_value[DB_P08])  //gss_adg_Temp[6] = 7���ȵ�ż
            {
                gss_db_A_system_fault_delaytime++;
                if(gss_db_A_system_fault_delaytime >= 30) //30s
                {
                    gss_db_A_system_fault_delaytime = 0;
                    bflg_db_A_system_fault = 1;           //Aϵͳ����
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
    else                                //Aϵͳ����
    {
        if(gss_adg_Temp[6] <= gss_db_debug_parm_value[DB_P08]) 
        {
            gss_db_A_system_fault_delaytime++;
            if(gss_db_A_system_fault_delaytime >= 30) //30s
            {
                gss_db_A_system_fault_delaytime = 0;
                bflg_db_A_system_fault = 0;       //Aϵͳ���Ͻ��
            }
        }
        else
        {
            gss_db_A_system_fault_delaytime = 0;
        }
    }
    //------------------------------------------------------------------------------------------
    //B
    if(bflg_db_B_system_fault == 0)    //Bϵͳ�޹���
    {
        if(bflg_db_B_system_runing_delaytime == 1)
        {
            if(gss_adg_Temp[7] > gss_db_debug_parm_value[DB_P08])  ////gss_adg_Temp[7] = 8��
            {
                gss_db_B_system_fault_delaytime++;
                if(gss_db_B_system_fault_delaytime >= 30) //30s
                {
                    gss_db_B_system_fault_delaytime = 0;
                    bflg_db_B_system_fault = 1;           //Bϵͳ����
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
    else                      //Bϵͳ����
    {
        if(gss_adg_Temp[7] <= gss_db_debug_parm_value[DB_P08]) 
        {
            gss_db_B_system_fault_delaytime++;
            if(gss_db_B_system_fault_delaytime >= 30) //30s
            {
                gss_db_B_system_fault_delaytime = 0;
                bflg_db_B_system_fault = 0;           //Bϵͳ���Ͻ��
            }
        }
        else
        {
            gss_db_B_system_fault_delaytime = 0;
        }
    }
}


/***********************************************************************************************************************************************
��������: ѹ���������ָʾ��

����λ��:��ѭ��--------------------------------ok
************************************************************************************************************************************************/
void db_comp_RL_out(void)
{
    if((bflg_factory_AT_test == 0)&&(bflg_self_test == 0))               // û���Լ�Ͳ��Ա�֤led����
    {
        if(bflg_Tcouple_reverse == 0)                       //AB���ȵ�żδ�巴
        {
            if(guc_db_system_state == SYSTEM_NORMAL)        //AB������
            {
                if(bflg_db_first_comp_runing == 1) //���ȿ���ѹ��
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_A_COMP_OUT_PIN = 1;  //Aѹ����
                        LED8 |= 0x20;           //Aָʾ�ƿ�                
                    }
                    else
                    {
                        RL_B_COMP_OUT_PIN = 1;  //Bѹ����
                        LED8 |= 0x40;           //Bָʾ�ƿ�
                    }
                }
                else
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_A_COMP_OUT_PIN = 0;  //Aѹ����
                        LED8 &= ~0x20;          //Aָʾ�ƹ�
                    }
                    else
                    {
                        RL_B_COMP_OUT_PIN = 0;  //Bѹ����
                        LED8 &= ~0x40;          //Bָʾ�ƹ�
                    }
                }
                
                //-----------------------------------------------------
                if(bflg_db_secend_comp_runing == 1) //�󿪵�ѹ��
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_B_COMP_OUT_PIN = 1;  //Bѹ����
                        LED8 |= 0x40;           
                    }
                    else
                    {
                        RL_A_COMP_OUT_PIN = 1;  //Aѹ����
                        LED8 |= 0x20;           
                    }
                }
                else
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_B_COMP_OUT_PIN = 0;  //Bѹ����
                        LED8 &= ~0x40;          
                    }
                    else
                    {
                        RL_A_COMP_OUT_PIN = 0;  //Aѹ����
                        LED8 &= ~0x20; 
                    }
                }
            }
            else if(guc_db_system_state == SYSTEM_A_FAULT)  //A����B����
            {
                if(gss_factory_parm_value[DISP_FACTORY_Pb] == 0)  //δ���ι�����˸
                {
                    //A����
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
                if(bflg_db_first_comp_runing == 1) //���ȿ���ѹ��
                {
                    RL_A_COMP_OUT_PIN = 0;  
                    RL_B_COMP_OUT_PIN = 1;  
                    LED8 |= 0x40;           //Bָʾ�ƿ�
                }
                else
                {
                    RL_A_COMP_OUT_PIN = 0;  
                    RL_B_COMP_OUT_PIN = 0;  
                    LED8 &= ~0x40;          //Bָʾ�ƹ�
                }
            }
            else if(guc_db_system_state == SYSTEM_B_FAULT)  //A����B����
            {
                if(gss_factory_parm_value[DISP_FACTORY_Pb] == 0)  //δ���ι�����˸
                {
                    //B����
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
                if(bflg_db_first_comp_runing == 1) //���ȿ���ѹ��
                {
                    RL_A_COMP_OUT_PIN = 1;  
                    RL_B_COMP_OUT_PIN = 0;  
                    LED8 |= 0x20;           //Aָʾ�ƿ�
                }
                else
                {
                    RL_A_COMP_OUT_PIN = 0;  
                    RL_B_COMP_OUT_PIN = 0;  
                    LED8 &= ~0x20;          //Aָʾ�ƹ�            
                }
            }  
            else if(guc_db_system_state == SYSTEM_AB_FAULT) //AB������
            {
                if(gss_factory_parm_value[DISP_FACTORY_Pb] == 0)  //δ���ι�����˸
                {
                    //AB����
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
                if(bflg_db_first_comp_runing == 1) //���ȿ���ѹ��
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_A_COMP_OUT_PIN = 1;  //Aѹ����
                    }
                    else
                    {
                        RL_B_COMP_OUT_PIN = 1;  //Bѹ����
                    }
                }
                else
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_A_COMP_OUT_PIN = 0;  //Aѹ����
                    }
                    else
                    {
                        RL_B_COMP_OUT_PIN = 0;  //Bѹ����
                    }
                }
                
                //-----------------------------------------------------
                if(bflg_db_secend_comp_runing == 1) //�󿪵�ѹ��
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_B_COMP_OUT_PIN = 1;  //Bѹ����
                    }
                    else
                    {
                        RL_A_COMP_OUT_PIN = 1;  //Aѹ����
                    }
                }
                else
                {
                    if(bflg_first_comp_open == A_SYSTEM)
                    {
                        RL_B_COMP_OUT_PIN = 0;  //Bѹ����
                    }
                    else
                    {
                        RL_A_COMP_OUT_PIN = 0;  //Aѹ����
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
��������: ѹ��ͣ��xmin�󣬷��ͣ��

����λ��: 1s--------------------------------ok
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
��������: �״���ǿ�磨�����ϵ磩ʱ���ж�7��8�������ȵ�ż�Ƿ�巴���ж�

          ������
          1����Ϊ�����ϵ�ʱ�趨��������Aϵͳ��������Bϵͳ�¶��Ƿ��½���
          2��Bϵͳ��Ӧ8���ȵ�ż--gss_adg_Temp[7]
          3���տ�ʼ����ʱ������䵱ʱB���¶ȣ�
          
          �ж�ԭ��
          1��A����1min��A�¶�δ���ͣ�Bϵͳ�¶Ƚ���>10�棻

          ����ȵ�ż�巴��
          1����Ҫ�ϵ�������ϵ磻
          
����λ��: ��--------------------------------ok
************************************************************************************************************************************************/
void db_thermocouple_reverse_judge(void)
{
    //debug
    //bflg_Tcouple_reverse = 1;
    
    if(bflg_power_on_Tcouple_judge == 1)             //�״��ϵ��ż�巴�Ƿ��жϹ��ı�־
    {
        if(bflg_db_first_comp_runing == 1)           //ѹ������
        {
            if(bflg_comp2_run_space_delaytime == 0)  //1min
            {
                bflg_power_on_Tcouple_judge = 0;     //���ʼ������Ϊ1-ok
                
                if(gss_adg_Temp[7] - gss_B_Tcouple_memory >= 10 ) //��ʼʱ�����-ok
                {
                    bflg_Tcouple_reverse = 1;        //�����ȵ�ż�巴
                }
            }
        }
    }
}




#endif
/***************************************END OF THE FILE*********************************************/
