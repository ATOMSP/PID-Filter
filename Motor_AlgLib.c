/* please use GB2312 to open the file */
#include "Motor_AlgLib.h"
#include <string.h>




/**
 * @brief ����λ��ʽPID 
 * @param hpid  pid���
 * @param actual �������ֵ
 * @param target ����Ŀ��ֵ
 * @return  PID�����+-��
*/
float PidRunPos(PID_Typedef * hpid,float actual,float target)
{
  /* stop pos PID */
  if(hpid->IS_STOP){ 
    return 0.0f;
  };
  /* �����������ֺ� */
  hpid->error = target - actual;
  hpid->Integral_sum += hpid->ki * ((hpid->error + hpid->last_error) * 0.5f);
  /* �����޷� */
  hpid->Integral_sum = (hpid->Integral_sum > hpid->INTEGRAL_MAX) ? 
                       hpid->INTEGRAL_MAX : 
                       ((hpid->Integral_sum < -(hpid->INTEGRAL_MAX)) ? -(hpid->INTEGRAL_MAX) : hpid->Integral_sum);
  /* PID�������ֵ */
  float output;
  output = hpid->kp * hpid->error + hpid->Integral_sum + hpid->kd * (hpid->error - hpid->last_error);
  /* ����޷� */
  output = (output > hpid->OUTPUT_MAX) ? 
                 hpid->OUTPUT_MAX : 
                 ((output < -(hpid->OUTPUT_MAX)) ? -(hpid->OUTPUT_MAX) : output);
  /* ������һ�ε���� */
  hpid->last_error = hpid->error;
  return output;
}


/**
 * @brief ��������ʽPID 
 * @param hpid  pid���
 * @param actual �������ֵ
 * @param target ����Ŀ��ֵ
 * @return  PID�����+-��
*/
float PidRunInc(PID_Typedef *hpid,float actual,float target)
{
  volatile static float out = 0.0f;
  /* ��ͣPID */
  if(hpid->IS_STOP) return;
  hpid->I_error = target - actual;
  out += (hpid->kp * (hpid->I_error - hpid->I_last_error)
        + hpid->ki * hpid->I_error
        + hpid->kd * (hpid->I_error - (2.0f * hpid->I_last_error) + hpid->I_llast_error));
  /* ����޷� */
  out = (out > hpid->OUTPUT_MAX) ? 
                 hpid->OUTPUT_MAX : 
                 ((out < -(hpid->OUTPUT_MAX)) ? -(hpid->OUTPUT_MAX) : out);
  hpid->I_llast_error = hpid->I_last_error;
  hpid->I_last_error = hpid->I_error;
  return out;
}


/**
 * @brief ǰ������
 * @param input �ź�����
 * @param T �������ms
 * @return ǰ�����
*/
float ForwardCTL(float input,float T)
{
  volatile static float last_in;
  float out = (input - last_in) / (T * 0.001f) + input;
  last_in = input;
  return out;
}

/**
 * @brief ��ǰ������λ��ʽPID 
 * @param hpid  pid���
 * @param actual �������ֵ
 * @param target ����Ŀ��ֵ
 * @return  PID�����+-��
*/
float PidRunPos_F(PID_Typedef * hpid,float actual,float target)
{
  float out = ForwardCTL(actual,hpid->SampleTime);
  out += PidRunPos(hpid,actual,target);
  return out;
}


/**
 * @brief ����PID����
 * @param hpid  pid���
 * @param kp ki kd (> 0)
 * @param none
*/
void SetPidParam(PID_Typedef * hpid,float kp,float ki,float kd)
{
  if(!hpid->SampleTime) return;
  hpid->kp = kp;
  hpid->ki = ki * (hpid->SampleTime * 0.001f);
  hpid->kd = kd / (hpid->SampleTime * 0.001f);
  /* �������������ϵ */
  if(hpid->IS_ION){
    hpid->kp = (0.0f - hpid->kp);
    hpid->ki = (0.0f - hpid->ki);
    hpid->kd = (0.0f - hpid->kd);
  }
}

/**
 * @brief ����PID�������
 * @param hpid  pid���
 * @param tim (+ ms)
 * @param none
*/
void SetPidSampleTim(PID_Typedef * hpid,float tim)
{
  if(tim <= 0) return;
  if(!hpid->SampleTime) return;
  float ratio  = tim / hpid->SampleTime;
  /* ����ki��kd */
  hpid->ki *= ratio;
  hpid->kd /= ratio;
  hpid->SampleTime = tim;  
}


/**
 * @brief ����PID����������޷�
 * @param hpid  pid���
 * @param integral_max �������ֵ
 * @param output_max ������ֵ
 * @param none 
*/
void SetPidLimitVal(PID_Typedef * hpid,float integral_max,float output_max)
{
  if(integral_max < 0 || output_max < 0) return;
  hpid->INTEGRAL_MAX = integral_max;
  hpid->OUTPUT_MAX = output_max;
}

/**
 * @brief PID�����ʼ��
 * @param hpid  pid���
 * @param kp ki kd
 * @param tim ������� I_max �������ֵ O_max ������ֵ 
 * @param none 
*/
void PID_Init(PID_Typedef * hpid,float kp,float ki,float kd,
              float tim,uint8_t Ision,float I_max,float O_max){
  memset((void*)hpid,0,sizeof(PID_Typedef));
  hpid->IS_STOP = 0;
  hpid->IS_ION = Ision;
  hpid->SampleTime = tim;
  hpid->INTEGRAL_MAX = I_max;
  hpid->OUTPUT_MAX = O_max;
  SetPidParam(hpid,kp,ki,kd);
}

/**
 * @brief ����PID�Ƿ�ֹͣ
 * @param hpid  pid���
 * @param en (0 / 1)
 * @param none 
*/
void SetPidIsStop(PID_Typedef * hpid,uint8_t en)
{
  hpid->IS_STOP = en;
}

/**
 * @brief ����PID�Ƿ����������/�����
 * @param hpid  pid���
 * @param en (0 / 1)
 * @param none 
*/
void SetPidIsIOn(PID_Typedef * hpid,uint8_t en)
{
  hpid->IS_ION = en;
  if(en){
    SetPidParam(hpid,hpid->kp,hpid->ki,hpid->kd);
  }
}

/**
 * @brief �ٶ�λ�ô���λ��ʽPID
 * @param E_hpid I_hpid �⻷�ڻ�pid���
 * @param E_acgtual �⻷����ֵ
 * @param I_actual �ڻ�����ֵ
 * @param target �⻷Ŀ��ֵ
 * @return output �����+-��
*/
float CascadePid(PID_Typedef * E_hpid,PID_Typedef * I_hpid,float E_actual,float I_actual,float target)
{
  float out;
  out = PidRunPos(E_hpid,E_actual,target);
  return PidRunPos(I_hpid,I_actual,out);
}


/*--------------------�˲��㷨------------------------*/

/**
 * @brief ��ֵ�˲�
 * @param CallBack ���������ݻ�ȡ�ص�
 * @param n ��������
 * @param ret ��ֵ���
*/
float Filter_Mean(void(*CallBack)(float*),int n)
{
  float temp[n];
  float ret;
  for(int i = 0 ;i < n; i++){
    CallBack(temp + i);
    ret += temp[i];
  }
#ifdef ARM_MATH_CM4
  ret = 0.0f;
  arm_mean_f32(temp,n,&ret);
  return ret;
#else
  return (float)ret / n;
#endif
}

/**
 * @brief return time dif
 * @param now now time
 * @param last last time
 * @return  time dif
*/
static uint32_t getTimeDif(uint32_t now, uint32_t last){
#define TIME_CNT_MAX ((uint32_t)~((uint32_t)0))
    if ((int)((int)now - (int)last) >= 0) {
        return (now - last);
    }
    else {
        return ((TIME_CNT_MAX - last + 1) + now);
    }
#undef TIME_CNT_MAX
}

/**
 * @brief ��ͨ�˲�
 * @param now ����ʱ��ms
 * @param Tf ʱ�䳣��(>0)s
 * @param input ��������
 * @return  �˲����
*/
float Filter_Low(uint32_t now,float Tf,float input)
{
  volatile static uint32_t last;
  volatile static float last_out;
  float dt = (float)getTimeDif(now,last) * 0.001f;
  if(dt > 0.3f){
    last_out = input;
    last = now;
    return input;
  }
  float alpha = Tf / (Tf + dt);
  float out = alpha * last_out + (1.0f - alpha) * input;
  last_out = out;
  last = now;
  return out;
}


/**
 * @brief �޷��˲�
 * @param Max �������ֵ����ֵ
 * @param input ��������
 * @return  �˲����
*/
float Filter_LimitAmp(float Max,float input)
{
#define GET_ABS(num) ((num > 0) ? num : -num)
  volatile static float last;
  if(GET_ABS(input) >= Max) return last;
  else{
    last = input;
    return input;
  }
#undef GET_ABS
}

















