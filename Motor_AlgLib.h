/* please use GB2312 to open the file */
#ifndef __MOTOR_ALGLIB_H_
#define __MOTOR_ALGLIB_H_



#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>



/* PID struct */
typedef struct {
  /* pid param*/
  volatile float kp;
  volatile float ki;
  volatile float kd;
  /* Integral max */
  float INTEGRAL_MAX;
  /* out max */
  float OUTPUT_MAX;
  /* stop/start pid���� */
  volatile uint8_t IS_STOP;
  /* input/output PN */
  volatile uint8_t IS_ION;
  /* Sample time */
  volatile float SampleTime;
  /* private */
  volatile float Integral_sum;
  volatile float last_error;
  volatile float error;
  /* ----------Inc private PID-------------*/
  volatile float I_last_error;
  volatile float I_error;
  volatile float I_llast_error;
}PID_Typedef;

/**
 * @brief PID�����ʼ��
 * @param hpid  pid���
 * @param kp ki kd
 * @param tim ������� Ision IO������� I_max �������ֵ O_max ������ֵ 
 * @return none 
*/
void PID_Init(PID_Typedef * hpid,float kp,float ki,float kd,
              float tim,uint8_t Ision,float I_max,float O_max);
/**
 * @brief ����λ��ʽPID 
 * @param hpid  pid���
 * @param actual �������ֵ
 * @param target ����Ŀ��ֵ
 * @return  PID�����+-��
*/
float PidRunPos(PID_Typedef * hpid,float actual,float target);
/**
 * @brief ��������ʽPID 
 * @param hpid  pid���
 * @param actual �������ֵ
 * @param target ����Ŀ��ֵ
 * @return  PID�����+-��
*/
float PidRunInc(PID_Typedef *hpid,float actual,float target);
/**
 * @brief ��ǰ������λ��ʽPID 
 * @param hpid  pid���
 * @param actual �������ֵ
 * @param target ����Ŀ��ֵ
 * @return  PID�����+-��
*/
float PidRunPos_F(PID_Typedef * hpid,float actual,float target);
/* Set PID Param API */
void SetPidIsIOn(PID_Typedef * hpid,uint8_t en);
void SetPidIsStop(PID_Typedef * hpid,uint8_t en);
void SetPidLimitVal(PID_Typedef * hpid,float integral_max,float output_max);
void SetPidSampleTim(PID_Typedef * hpid,float tim);
void SetPidParam(PID_Typedef * hpid,float kp,float ki,float kd);
/* Common Filter Lib */
/**
 * @brief ��ͨ�˲�
 * @param now ����ʱ��ms
 * @param Tf ʱ�䳣��(>0)s
 * @param input ��������
 * @return  �˲����
*/
float Filter_Low(uint32_t now,float Tf,float input);
/**
 * @brief ��ֵ�˲�
 * @param CallBack ���������ݻ�ȡ�ص�
 * @param n ��������
 * @return ret ��ֵ���
*/
float Filter_Mean(void(*CallBack)(float*),int n);
/**
 * @brief �޷��˲�
 * @param Max �������ֵ����ֵ
 * @param input ��������
 * @return  �˲����
*/
float Filter_LimitAmp(float Max,float input);


#ifdef __cplusplus
}
#endif




#endif


