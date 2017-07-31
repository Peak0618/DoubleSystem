#ifndef _COMP2_H_
#define _COMP2_H_
//------------------------------------------------------------------------------
//�ⲿ��������
extern void comp2_power_on_delaytime(void);   
extern void comp2_run_stop_delaytime(void);   
extern void  db_comp_run_stop_ask_for_delaytime();
extern void db_first_system_run_stop_deal(void);      
extern void comp2_run_delaytime(void);   
extern void comp2_stop_delaytime(void); 
extern void comp2_TA_fault_run_delaytime(void);   
extern void comp2_TA_fault_stop_delaytime(void);   
extern void comp2_continue_run_delaytime(void);    
extern void comp2_force_stop_delaytime(void); 
extern void comp2_run_space_delaytime(void);  
extern void db_secend_system_run_stop_deal(void); 
//------------------------------------------------------------------------------
extern void comp2_start_delaytime(void);
extern void comp2_update_delaytime(void);
extern void comp21_start_delaytime(void);
extern void comp21_update_delaytime(void);
extern void comp2_power_down_delaytime(void);
extern void freq_ctrl_deal(void);
extern void init_freq_ctrl(void);

extern void db_init_debug_parm_value(void);
extern void db_comp_fault_judge_delaytime(void);
extern void db_comp_system_fault_judge(void);
extern void db_comp_RL_out(void);
extern void db_fan_stop_delaytime(void);
extern void db_thermocouple_reverse_judge(void);

//------------------------------------------------------------------------------
//�ⲿ��������
extern flag_type flg_comp2;
        #define   bflg_comp2_first_run                flg_comp2.bits.bit0 //�״����б�־
        #define   bflg_comp2_power_on_delaytime       flg_comp2.bits.bit1 //�ϵ���ʱ��־
        #define   bflg_db_first_comp_askfor_run       flg_comp2.bits.bit2 //����ѹ�������󿪻���־    // peak�ﵽ������Ŀ�������  //peak�н�comp2_run_stop_delaytime()�����еı�־���еģ�
        #define   bflg_db_first_comp_askfor_stop      flg_comp2.bits.bit3 //����ѹ��������ػ���־    //peak�ﵽ������Ĺػ�����
        #define   bflg_db_first_comp_runing           flg_comp2.bits.bit4 //����ѹ�������б�־  peak 0 �ػ� 1 ����
        #define   bflg_comp2_run_delaytime            flg_comp2.bits.bit5 //����ѹ����������ʱ��־   //���� 10���� ����ͣ��
        #define   bflg_comp2_stop_delaytime           flg_comp2.bits.bit6 //����ѹ�����ػ���ʱ��־   //�ػ�8���Ӳ����ٿ���
        #define   bflg_comp2_TA_fault_run_delaytime   flg_comp2.bits.bit7  //�����������ϸ���ѹ����������ʱ��־    //peak ���������й��Ͽ�30���Ӻ�ػ�
        #define   bflg_comp2_TA_fault_stop_delaytime  flg_comp2.bits.bit8  //�����������ϸ���ѹ�����ػ���ʱ��־    //peak ���������й��Ϲ�10���Ӻ󿪻�
        #define   bflg_comp2_continue_run_delaytime   flg_comp2.bits.bit9  //����ѹ�����������б�־               //peak ������Ҫ����������һ��ʱ���Ҫǿ�ƹػ� ��ʼ�ϵ��֮������ʱ�䲻ͬ
        #define   bflg_comp2_force_stop_delaytime     flg_comp2.bits.bit10 //����ѹ����ǿ��ֹͣ��־               //peak Ҳ��ͣ��һ��ʱ���Ҫ������Ҫ�����Ĳ��ܿ�
        #define   bflg_comp2_run_space_delaytime      flg_comp2.bits.bit11 //����ѹ�������м����ʱ��־   //peak ��ע1 ������ ����ѹ����������ʱ1��10(��ʼ�ϵ�)���ٿ�����ѹ���� 
        #define   bflg_db_secend_comp_runing          flg_comp2.bits.bit12 //����ѹ�������б�־
extern flag_type flg_freq;
        #define   bflg_comp2_start_delaytime    flg_freq.bits.bit0
        #define   bflg_comp2_update_delaytime   flg_freq.bits.bit1
        #define   bflg_comp21_start_delaytime   flg_freq.bits.bit2
        #define   bflg_comp21_update_delaytime  flg_freq.bits.bit3
        #define   bflg_comp21_door_freq         flg_freq.bits.bit4   //peak ��⵽����1s  ����ѹ����ʱ ����0,�¼Ӷϵ���0
        #define   bflg_comp21_door_freq_tmp     flg_freq.bits.bit5   //peak ����1s����PT100�¶ȴ����趨�¶�5��ʱ ��0��
        #define   bflg_comp2_large_freq         flg_freq.bits.bit6
        #define   bflg_power_down_delaytime     flg_freq.bits.bit7   //�ϵ�������ϵ�ʱ����ѹ����ʱ
        #define   bflg_comp2_first_power_on     flg_freq.bits.bit8   //�����ϵ�ʱ����ѹ����4500������־

extern flag_type db_flag;
        #define bflg_db_A_system_fault                   db_flag.bits.bit0    //Aϵͳ����
        #define bflg_db_B_system_fault                   db_flag.bits.bit1    //Bϵͳ����
        #define bflg_first_comp_open                     db_flag.bits.bit2    //ѹ�����濪
        #define bflg_db_comp_fan_stop_delaytime          db_flag.bits.bit3    //ѹ��ͣ��xmin����ͣ��
        #define bflg_db_A_system_runing_delaytime        db_flag.bits.bit4    //Aϵͳ����xmin�����Ƿ��й���
        #define bflg_db_B_system_runing_delaytime        db_flag.bits.bit5    //Bϵͳ����xmin�����Ƿ��й���
        #define bflg_power_on_Tcouple_judge              db_flag.bits.bit6    //AB���ȵ�ż�巴�Ƿ��жϹ�
//------------------------------------------------------------------------------
extern int16_t   gss_para[27];
#define   COMP2_START_FREQ         0  //2000
#define   COMP2_START_TIME         1  //60
#define   COMP2_UPDATE_TIME        2  //80
#define   COMP2_SET_TEMP1          3  //-27
#define   COMP2_FREQ_DELT_K1       4  //50
#define   COMP2_FREQ_DELT_MIN      5  //-20
#define   COMP2_FREQ_DELT_MAX      6  //20
#define   COMP2_FREQ_MIN           7  //2000
#define   COMP2_FREQ_MAX           8  //4500
#define   COMP2_SET_TEMP2          9  //-25
#define   COMP2_FREQ_DELT_K2       10 //50
#define   COMP21_START_FREQ        11 //2000
#define   COMP21_START_TIME        12 //60
#define   COMP21_UPDATE_TIME       13 //80
#define   COMP21_FREQ_DELT_K       14 //50
#define   COMP21_FREQ_DELT_MIN     15 //-20
#define   COMP21_FREQ_DELT_MAX     16 //20
#define   COMP21_FREQ_MIN          17 //2000
#define   COMP21_FREQ_MAX          18 //4500
#define   COMP21_FREQ_DELT_TEMP    19 //0
#define   COMP_START_DELT_TEMP     20 //������+��ֵ
#define   COMP_STOP_DELT_TEMP      21 //ͣ����+��ֵ
#define   PT100_DELT_TEMP          22 //0
#define   COMP_STOP_DELAYTIME      23 //300
#define   COMP_DOOR_TDELT          24 //50
#define   COMP_DOOR_FREQ           25 //3500
#define   COMP_RUN_DELAYTIME       26 //300

//db
#define DB_P00       0    //��˫ϵͳ�ġ�T
#define DB_P01       1    //����ϵͳ�ġ�T����
#define DB_P02       2    //����ϵͳ�ġ�T����
#define DB_P03       3    //�׿�ѹ����ʱxmin����һ����
#define DB_P04       4    //��������xh��ǿͣ��
#define DB_P05       5    //�״ο�����������xh��ǿͣ��
#define DB_P06       6    //ǿ��ͣ��xmin
#define DB_P07       7    //����xmin���ж�ϵͳ����
#define DB_P08       8    //�¶ȸ���x���ж�Ϊ����
#define DB_P09       9    //��������ڲ���
#define DB_P10       10   //ͣ������ڲ���
#define DB_P11       11   //��̿���ʱ��
#define DB_P12       12   //���ͣ��ʱ��
#define DB_P13       13   //ѹ���غ���ʱxmin���ͣ��
#define DB_SUM_P     14   
extern int16_t gss_db_debug_parm_value[DB_SUM_P];    //���ԵĲ���ֵP00~P13


extern int16_t   gss_comp2_freq;   //����ѹ��Ƶ��
extern int16_t   gss_comp21_freq;  //����ѹ��Ƶ��
extern int16_t   gss_comp2_power_on_delaytimer;   //�ϵ���ʱ��ʱ��

extern uint16_t  gus_comp2_freq_cnt;
extern uint16_t  gus_comp21_freq_cnt;
extern int8_t    gss_power_down_delaytime;               
extern uint8_t   guc_comp2_power_on_again;



//------------------------------------------------------------------------------
#endif
