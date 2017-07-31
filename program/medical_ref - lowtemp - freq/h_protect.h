#ifndef _PROTECT_H_
#define _PROTECT_H_
//------------------------------------------------------------------------------
//�ⲿ��������

extern void prot2_TA_first_delaytime(void);  //���������״θߵ��±�����ʱ������1min��ʱ�����е���

extern void prot2_TA_high_delaytime(void);   //�����������±�����ʱ������1s��ʱ�����е���

extern void prot2_TA_low_delaytime(void);    //�����������±�����ʱ������1s��ʱ�����е���

extern void prot2_power_off_deal(void);      //�ϵ籣��������������ѭ�������е���

extern void port2_power_fault_deal(void);    //��Դ����ϴ�����������ѭ�������е���

extern void prot2_bat_discnnt_delaytime(void);    //���δ���ӱ�����ʱ������100ms��ʱ�����е���

extern void prot2_bat_space_delaytime(void);      //���δ���Ӽ������ʱ������100ms��ʱ�����е���

extern void prot2_bat_low_deal(void);   //��ص����ͱ���������������ѭ�������е���

extern void prot2_bat_low_delaytime(void);   //��ص������ӳٳ�����1min��ʱ�����е���

extern void port2_door_delaytime(void);      //�ſ��ر�����ʱ������1s��ʱ�����е���

extern void port2_T_fault_deal(void);        //���������ϴ�����������ѭ�������е���

extern void prot2_THW_high_delaytime(void);  //���´��������߱�����ʱ������1s��ʱ�����е���

extern void prot2_TC_first_delaytime(void);  //���������״α�����ʱ������1min��ʱ�����е���

extern void prot2_cond_dirty_delaytime(void);//�������ౣ����ʱ������1s��ʱ�����е���

extern void prot2_volt_deal(void);      //��ѹ���걨������������������ѭ�������е���

extern void alarm_lamp_deal(void);      //�����ƴ�����������ѭ�������е���

extern void alarm_buzz_deal(void);      //����������������������ѭ�������е���

extern void alarm_buzz_off_delaytime(void);       //����������ȡ����ʱ������1min��ʱ�����е���

extern void bat_T_hight_protect(void);               //����¶ȸ��жϳ���

extern void alarm_power_off(void);

//------------------------------------------------------------------------------
//�ⲿ��־����
extern flag_type flg_alarm;                  // ��Щ�����ǵε���
          #define   bflg_alarm_ALH           flg_alarm.bits.bit0    //���±�����־          
          #define   bflg_alarm_ALL           flg_alarm.bits.bit1    //���±�����־         
          #define   bflg_alarm_bat_discnnt   flg_alarm.bits.bit2    //���δ���ӱ�����־   
          #define   bflg_alarm_T_fault	     flg_alarm.bits.bit3    //���������ϱ�����־: �������¡��������Ƚ�����4�ִ���������һ���й���������й���
          #define   bflg_alarm_cond_dirty    flg_alarm.bits.bit4    //�������౨����־	 
          #define   bflg_alarm_open_door	 flg_alarm.bits.bit5    //���ű�����־		 
          #define   bflg_THW_fault           flg_alarm.bits.bit6    //���´��������ϱ�־
          #define   bflg_TC_fault            flg_alarm.bits.bit7    //�������¶ȴ��������ϱ�־
          #define   bflg_TC_2_fault          flg_alarm.bits.bit8    //�Ƚ�������������---->˫ϵͳ��Ϊ����
          #define   bflg_TA_fault            flg_alarm.bits.bit9    //�����������ϱ�־  
          #define   bflg_Tcouple_reverse     flg_alarm.bits.bit10   //�������ȵ�ż��巴
          //��������������λ�ˣ���Ϊʹ���� flg_alarm.uword
          //--xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

//------------------------------------------------------------------------------
extern flag_type flg_alarm1;     //�����3��������ָʾ��    ��Ϊû�еε�������û�к�����ĺϲ���һ�飬��Ϊ�����õ���.uword
         #define   bflg_alarm_bat_fed           flg_alarm1.bits.bit0   //��ؽӷ�������־    ����
         #define   bflg_alarm_bat_low           flg_alarm1.bits.bit1   //��ص����ͱ�����־  ����
         #define   bflg_alarm_THW_high          flg_alarm1.bits.bit2   //���¹��߱�����־    ����
         #define   bflg_alarm_power_off         flg_alarm1.bits.bit3   //�ϵ籨����־--���������ر𣬵���������     
         //��������������λ�ˣ���Ϊʹ���� flg_alarm1.uword
         //--xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

         
//------------------------------------------------------------------------------
extern flag_type flg_alarm2;  //��Ϊȥ�������������������ǳ�����ȷ�ж��ˣ����Ե���������

        #define   bflg_alarm_power_fault   flg_alarm2.bits.bit0       //��Դ����ϱ�����־   
        #define   bflg_alarm_volt          flg_alarm2.bits.bit1       //��ѹ���걨����־  
        #define   bflg_alarm_bat1_T_hight  flg_alarm2.bits.bit2       //���1�¶ȹ���       
        #define   bflg_alarm_bat2_T_hight  flg_alarm2.bits.bit3       //���2�¶ȹ���


//------------------------------------------------------------------------------
extern flag_type flg_prot2;
          
          #define   bflg_alarm_lamp          flg_prot2.bits.bit0 //�����Ʊ�־
          #define   bflg_alarm_buzz          flg_prot2.bits.bit1 //������������־
          #define   bflg_alarm_buzz_output   flg_prot2.bits.bit2 //�����������־
          #define   bflg_alarm_output        flg_prot2.bits.bit3 //Զ�̱��������־
          
          #define   bflg_prot2_door1_pin     flg_prot2.bits.bit4 //�ſ���1����״̬��־
          #define   bflg_prot2_door2_pin     flg_prot2.bits.bit5 //�ſ���2����״̬��־
          #define   bflg_prot2_bat_fed_pin   flg_prot2.bits.bit6 //﮵�ط��巴��־
          
          #define   bflg_prot2_TA_first_delaytime flg_prot2.bits.bit7 //���������״α�����ʱ��־
          #define   bflg_prot2_TC_first_delaytime flg_prot2.bits.bit8 //���������״α�����ʱ��־
          
          #define   bflg_alarm_buzz_off_delaytime flg_prot2.bits.bit9 //����������ȡ����ʱ��־  =1��ʾ�Ѱ���"����ȡ��"����30min�ڲ���
          
          //#define   bflg_prot2_THW_high_disp      flg_prot2.bits.bit10//���¹����¶���ʾ��־
          
          #define   bflg_prot2_bat_low_delaytime         flg_prot2.bits.bit11   //��ص�������ʱ��־    //peak ��ʼ�ϵ�Ե�س��24Сʱ��
          #define   bflg_prot2_bat_space_delaytime       flg_prot2.bits.bit12   //���δ���Ӽ������ʱ��־
          #define   bflg_prot2_bat_space_reach           flg_prot2.bits.bit13   //���δ���Ӽ������ʱ����־

		  /****peak add*********/
          #define   bflg_alarm_bat_low_led               flg_prot2.bits.bit14  // ��⵽��ص�����ʱ��ָʾ��ֻ����˸����������
          #define   bflg_power_off_alarm                 flg_prot2.bits.bit15  //�ϵ�ʱ���������������ʱ 



//------------------------------------------------------------------------------
#define   THW_FAULT_CODE      1
#define   TF_FAULT_CODE       2
#define   TC_FAULT_CODE       3
#define   TD_FAULT_CODE       4
#define   TE_FAULT_CODE       5
#define   TA_FAULT_CODE       6
//------------------------------------------------------------------------------
extern int16_t   gss_prot2_TA_first_delaytimer;   //���������״α�����ʱ��ʱ��
extern int16_t   gss_prot2_TC_first_delaytimer;   //���������״α�����ʱ��ʱ��

extern int16_t   gss_prot2_bat_space_delaytimer;  //���δ���Ӽ������ʱ��ʱ��

extern int16_t   gss_prot2_bat_low_delaytimer;    //��ص�������ʱ��ʱ��

extern int16_t   gss_alarm_buzz_off_delaytimer;   //����������ȡ����ʱ��ʱ��

extern uint16_t  gus_trip_code1;   //���ϴ���1
extern uint16_t  gus_trip_code2;   //���ϴ���2

extern uint8_t   guc_power_down_lfreq_cnt;  //�ϵ���ϵ��ѹ����4500����
extern uint8_t   guc_power_down_hdelay_cnt; //�ϵ���ϵ��ѹ����ʱ1min����


//------------------------------------------------------------------------------
#endif